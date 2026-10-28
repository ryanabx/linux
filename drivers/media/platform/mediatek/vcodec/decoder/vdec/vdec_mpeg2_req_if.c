// SPDX-License-Identifier: GPL-2.0
/*
 * MPEG-2 stateless decoding for decoders without firmware (MT8695), where
 * the kernel programs the decoder's registers itself. MPEG-2 runs on the
 * core alone.
 *
 * The register layout is MT8173's (VLD, MC and misc blocks), with misc
 * after the decode blocks: the core's second register range starts at VLD,
 * its first is misc.
 */

#include <linux/iopoll.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <uapi/linux/v4l2-controls.h>

#include "../mtk_vcodec_dec.h"
#include "../mtk_vcodec_dec_hw.h"
#include "../../common/mtk_vcodec_intr.h"
#include "../vdec_drv_base.h"
#include "../vdec_drv_if.h"

/* Two reference pictures and the one being decoded */
#define MPEG2_DPB_SIZE			3

#define MPEG2_DEC_TIMEOUT_MS		100

/*
 * After the last slice: a slice ends at the next start code, and the VLD
 * stalls in the last macroblocks without one. Appended in the buffer's
 * spare room.
 */
#define MPEG2_TAIL_SIZE			256
#define MPEG2_FETCH_TIMEOUT_US		10000

/* Offsets in the decode blocks: VLD at +0, VLD_TOP at +0x800, MC at +0x1000 */
#define VLD_PARAM			0x088	/* picture coding parameters */
#define VLD_CMD				0x08c
#define  VLD_CMD_FETCH			0x100000
#define  VLD_CMD_SHIFTER_INIT		0x800000
#define  VLD_CMD_DECODE			0x4000ff
#define VLD_PIC_SIZE			0x090
#define  VLD_PIC_SIZE_MPEG2		BIT(30)
#define VLD_094				0x094
#define VLD_098				0x098
#define VLD_09C				0x09c
#define VLD_RPTR			0x0b0
#define VLD_BS_START			0x0b4	/* >> 6 */
#define VLD_BS_END			0x0b8	/* >> 6 */
#define VLD_FETCH_STATUS		0x0e8
#define  VLD_FETCH_READY		BIT(0)
#define VLD_0EC				0x0ec
#define VLD_START_CODE			0x0f0
#define VLD_QMAT_CTRL			0x104
#define  VLD_QMAT_INTRA			0x100
#define  VLD_QMAT_NON_INTRA		0x200
#define VLD_SRST			0x108
#define VLD_WPTR			0x110
#define VLD_QMAT_ADDR			0x260
#define  VLD_QMAT_INTRA_ADDR		0x200
#define  VLD_QMAT_NON_INTRA_ADDR	0x240
#define VLD_QMAT_DATA			0x264

#define VLD_TOP_PIC_SIZE		0x870

#define MC_REF1_Y			0x1000	/* >> 9 */
#define MC_REF1_C			0x1004	/* >> 8 */
#define MC_REF2_Y			0x1008
#define MC_REF2_C			0x100c
#define MC_FWD_SLOT			0x1018
#define MC_SECOND_FIELD			0x1020
#define MC_OUT_SLOT			0x1024
#define  MC_OUT_SLOT_B			4
#define MC_LAST_MB_X			0x1028
#define MC_LAST_MB_Y			0x102c
#define MC_B_Y				0x109c
#define MC_B_C				0x10a0
#define MC_WIDTH			0x1208
#define MC_HEIGHT			0x120c
#define MC_OUT_END			0x1850	/* >> 4 */
#define MC_OUT_START			0x1854	/* >> 4 */
#define MC_OUT_ENABLE			0x1858
#define MC_MB_WIDTH			0x1980

/* Misc, as in MT8173's VP8 code */
#define MISC_SYS_SEL			0x084
#define  MISC_SYS_SEL_MPEG2		2
#define MISC_IRQ_CFG			0x0a4
#define MISC_SPEC_CON			0x0c8
#define MISC_FUNC_CON			0x0cc
#define MISC_DCM_CON			0x0ec
#define MISC_SRST			0x0f4
#define MISC_178			0x178

#define SYS_08				0x008
#define SYS_18				0x018
#define SYS_00				0x000

/* In the core's clock gate block (mediatek,vdecsys) */
#define VDECSYS_DCM_CON			0x018

struct vdec_mpeg2_slice_inst {
	struct mtk_vcodec_dec_ctx *ctx;
	void __iomem *dec;
	void __iomem *misc;
	void __iomem *sys;
};

static const void *vdec_mpeg2_get_ctrl(struct mtk_vcodec_dec_ctx *ctx, u32 id)
{
	struct v4l2_ctrl *ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, id);

	if (!ctrl)
		return ERR_PTR(-EINVAL);

	return ctrl->p_cur.p;
}

/* Bring the core up for a picture, in the order MediaTek's library does */
static void vdec_mpeg2_hw_setup(struct vdec_mpeg2_slice_inst *inst)
{
	struct mtk_vcodec_dec_dev *dev = inst->ctx->dev;

	writel(0x10, inst->misc + MISC_IRQ_CFG);
	writel(0, inst->misc + MISC_IRQ_CFG);
	writel(1, inst->sys + SYS_08);
	if (dev->vdecsys_regmap)
		regmap_set_bits(dev->vdecsys_regmap, VDECSYS_DCM_CON, BIT(0));
	writel(0x101, inst->dec + VLD_SRST);
	writel(0xfe, inst->misc + MISC_SPEC_CON);
	writel(0x3f6a15fd, inst->misc + MISC_FUNC_CON);
	writel(0xfffe, inst->misc + MISC_SPEC_CON);
	writel(0x7f6a11df, inst->misc + MISC_FUNC_CON);
	writel(0xfffffffb, inst->misc + MISC_178);
	writel(0, inst->misc + MISC_SRST);
	writel(MISC_SYS_SEL_MPEG2, inst->misc + MISC_SYS_SEL);
	writel(0, inst->dec + VLD_SRST);
	writel(1, inst->sys + SYS_18);
	writel(1, inst->misc + MISC_DCM_CON);
}

/*
 * Point the VLD at the picture's slices and let it fetch. The buffer starts
 * with the first slice's start code, which the VLD reports when it's ready.
 * The window is the whole buffer, in 64-byte units; the data ends with a
 * sequence end code that we append.
 */
static int vdec_mpeg2_start_bitstream(struct vdec_mpeg2_slice_inst *inst,
				      struct mtk_vcodec_mem *bs, size_t capacity)
{
	static const u8 seq_end[] = { 0x00, 0x00, 0x01, 0xb7 };
	u32 start = bs->dma_addr, end = bs->dma_addr + bs->size;
	u32 val;
	int ret;

	if (capacity < bs->size + MPEG2_TAIL_SIZE) {
		mtk_vdec_err(inst->ctx, "no room after the slices");
		return -ENOSPC;
	}
	memset(bs->va + bs->size, 0, MPEG2_TAIL_SIZE);
	memcpy(bs->va + bs->size, seq_end, sizeof(seq_end));
	end += MPEG2_TAIL_SIZE;

	writel(start >> 6, inst->dec + VLD_BS_START);
	writel((start + capacity) >> 6, inst->dec + VLD_BS_END);
	writel(start, inst->dec + VLD_RPTR);
	writel(end, inst->dec + VLD_WPTR);
	writel(0x100, inst->dec + VLD_SRST);
	writel(0, inst->dec + VLD_SRST);
	writel(0x30fff, inst->dec + VLD_09C);
	val = readl(inst->dec + VLD_0EC);
	writel(val | 0x10000000, inst->dec + VLD_0EC);
	writel(VLD_CMD_FETCH, inst->dec + VLD_CMD);

	/* Until the fetch is ready, reading the barrel shifter hangs the bus */
	ret = readl_poll_timeout(inst->dec + VLD_FETCH_STATUS, val,
				 val & VLD_FETCH_READY, 10, MPEG2_FETCH_TIMEOUT_US);
	if (ret) {
		mtk_vdec_err(inst->ctx, "bitstream fetch timed out");
		return ret;
	}
	writel(VLD_CMD_SHIFTER_INIT, inst->dec + VLD_CMD);

	val = readl(inst->dec + VLD_START_CODE);
	if (val < 0x101 || val > 0x1af) {
		mtk_vdec_err(inst->ctx, "no slice start code: 0x%x", val);
		return -EINVAL;
	}

	return 0;
}

static u32 vdec_mpeg2_param(const struct v4l2_ctrl_mpeg2_picture *pic)
{
	u32 val;

	val = (u32)pic->picture_structure << 30 |
	      (pic->picture_coding_type & 0x7) << 21 |
	      (pic->intra_dc_precision & 0x3) << 18 |
	      (pic->f_code[0][0] & 0xf) << 12 | (pic->f_code[0][1] & 0xf) << 8 |
	      (pic->f_code[1][0] & 0xf) << 4 | (pic->f_code[1][1] & 0xf);
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_FRAME_PRED_DCT)
		val |= BIT(29);
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_CONCEALMENT_MV)
		val |= BIT(28);
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_Q_SCALE_TYPE)
		val |= BIT(27);
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_TOP_FIELD_FIRST)
		val |= BIT(26);
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_INTRA_VLC)
		val |= BIT(20);
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_ALT_SCAN)
		val |= BIT(17);

	return val;
}

/* Four coefficients a word, in the zigzag order the control has them */
static void vdec_mpeg2_load_matrix(struct vdec_mpeg2_slice_inst *inst,
				   u32 addr, const u8 *m)
{
	int i;

	for (i = 0; i < 16; i++) {
		writel(addr + 4 * i, inst->dec + VLD_QMAT_ADDR);
		writel(m[4 * i] << 24 | m[4 * i + 1] << 16 |
		       m[4 * i + 2] << 8 | m[4 * i + 3],
		       inst->dec + VLD_QMAT_DATA);
	}
}

static void vdec_mpeg2_set_size(struct vdec_mpeg2_slice_inst *inst,
				const struct v4l2_ctrl_mpeg2_sequence *seq)
{
	u32 w16 = ALIGN(seq->horizontal_size, 16);
	u32 h16 = ALIGN(seq->vertical_size, 16);

	writel(VLD_PIC_SIZE_MPEG2 | h16 << 16 | 0x1ff,
	       inst->dec + VLD_PIC_SIZE);
	writel(h16 << 16 | w16, inst->dec + VLD_TOP_PIC_SIZE);
	writel(w16 / 16, inst->dec + MC_MB_WIDTH);
	writel(w16, inst->dec + MC_WIDTH);
	writel(h16, inst->dec + MC_HEIGHT);
}

/* A picture's luma and chroma addresses, from its capture buffer */
static void vdec_mpeg2_buf_addr(struct mtk_vcodec_dec_ctx *ctx,
				struct vb2_buffer *vb, dma_addr_t *y, dma_addr_t *c)
{
	*y = vb2_dma_contig_plane_dma_addr(vb, 0);
	if (ctx->q_data[MTK_Q_DATA_DST].fmt->num_planes == 2)
		*c = vb2_dma_contig_plane_dma_addr(vb, 1);
	else
		*c = *y + ctx->picinfo.fb_sz[0];
}

/* Reference slots, and where the output goes */
static void vdec_mpeg2_set_refs(struct vdec_mpeg2_slice_inst *inst,
				const struct v4l2_ctrl_mpeg2_picture *pic,
				struct vdec_fb *fb)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	dma_addr_t out_y, out_c, fwd_y, fwd_c, bwd_y, bwd_c;
	dma_addr_t start, end;
	struct vb2_queue *vq;
	struct vb2_buffer *vb;

	out_y = fb->base_y.dma_addr;
	if (ctx->q_data[MTK_Q_DATA_DST].fmt->num_planes == 2)
		out_c = fb->base_c.dma_addr;
	else
		out_c = out_y + ctx->picinfo.fb_sz[0];

	/* A missing reference decodes from the output: garbage, no fault */
	fwd_y = out_y;
	fwd_c = out_c;
	bwd_y = out_y;
	bwd_c = out_c;
	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (pic->picture_coding_type != V4L2_MPEG2_PIC_CODING_TYPE_I) {
		vb = vb2_find_buffer(vq, pic->forward_ref_ts);
		if (vb)
			vdec_mpeg2_buf_addr(ctx, vb, &fwd_y, &fwd_c);
		else
			mtk_vdec_err(ctx, "no forward reference");
	}
	if (pic->picture_coding_type == V4L2_MPEG2_PIC_CODING_TYPE_B) {
		vb = vb2_find_buffer(vq, pic->backward_ref_ts);
		if (vb)
			vdec_mpeg2_buf_addr(ctx, vb, &bwd_y, &bwd_c);
		else
			mtk_vdec_err(ctx, "no backward reference");
	}

	switch (pic->picture_coding_type) {
	case V4L2_MPEG2_PIC_CODING_TYPE_I:
		writel(out_y >> 9, inst->dec + MC_REF1_Y);
		writel(out_c >> 8, inst->dec + MC_REF1_C);
		writel(out_y >> 9, inst->dec + MC_REF2_Y);
		writel(out_c >> 8, inst->dec + MC_REF2_C);
		writel(0, inst->dec + MC_OUT_SLOT);
		break;
	case V4L2_MPEG2_PIC_CODING_TYPE_P:
		writel(fwd_y >> 9, inst->dec + MC_REF1_Y);
		writel(fwd_c >> 8, inst->dec + MC_REF1_C);
		writel(out_y >> 9, inst->dec + MC_REF2_Y);
		writel(out_c >> 8, inst->dec + MC_REF2_C);
		writel(0, inst->dec + MC_FWD_SLOT);
		writel(1, inst->dec + MC_OUT_SLOT);
		break;
	default:
		writel(fwd_y >> 9, inst->dec + MC_REF1_Y);
		writel(fwd_c >> 8, inst->dec + MC_REF1_C);
		writel(bwd_y >> 9, inst->dec + MC_REF2_Y);
		writel(bwd_c >> 8, inst->dec + MC_REF2_C);
		writel(out_y >> 9, inst->dec + MC_B_Y);
		writel(out_c >> 8, inst->dec + MC_B_C);
		writel(0, inst->dec + MC_FWD_SLOT);
		writel(MC_OUT_SLOT_B, inst->dec + MC_OUT_SLOT);
		break;
	}

	/* The output's range, both planes */
	start = min(out_y, out_c);
	end = max(out_y + ctx->picinfo.fb_sz[0], out_c + ctx->picinfo.fb_sz[1]);
	writel(end >> 4, inst->dec + MC_OUT_END);
	writel(start >> 4, inst->dec + MC_OUT_START);
	writel(1, inst->dec + MC_OUT_ENABLE);
	writel(0, inst->dec + MC_SECOND_FIELD);
}

static int vdec_mpeg2_slice_decode(void *h_vdec, struct mtk_vcodec_mem *bs,
				   struct vdec_fb *fb, bool *res_chg)
{
	struct vdec_mpeg2_slice_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	const struct v4l2_ctrl_mpeg2_sequence *seq;
	const struct v4l2_ctrl_mpeg2_picture *pic;
	const struct v4l2_ctrl_mpeg2_quantisation *quant;
	struct mtk_video_dec_buf *src_buf_info, *dst_buf_info;
	u32 mb_x, mb_y;
	int ret;

	*res_chg = false;

	/* bs NULL means flush decoder: nothing is kept between pictures */
	if (!bs)
		return 0;

	seq = vdec_mpeg2_get_ctrl(ctx, V4L2_CID_STATELESS_MPEG2_SEQUENCE);
	pic = vdec_mpeg2_get_ctrl(ctx, V4L2_CID_STATELESS_MPEG2_PICTURE);
	quant = vdec_mpeg2_get_ctrl(ctx, V4L2_CID_STATELESS_MPEG2_QUANTISATION);
	if (IS_ERR(seq) || IS_ERR(pic) || IS_ERR(quant))
		return -EINVAL;

	if (pic->picture_structure != V4L2_MPEG2_PIC_FRAME) {
		mtk_vdec_err(ctx, "field pictures are not supported yet");
		return -EINVAL;
	}
	if (pic->picture_coding_type < V4L2_MPEG2_PIC_CODING_TYPE_I ||
	    pic->picture_coding_type > V4L2_MPEG2_PIC_CODING_TYPE_B)
		return -EINVAL;

	fb = ctx->dev->vdec_pdata->get_cap_buffer(ctx);
	if (!fb) {
		mtk_vdec_err(ctx, "fb buffer is NULL");
		return -ENOMEM;
	}

	src_buf_info = container_of(bs, struct mtk_video_dec_buf, bs_buffer);
	dst_buf_info = container_of(fb, struct mtk_video_dec_buf, frame_buffer);
	v4l2_m2m_buf_copy_metadata(&src_buf_info->m2m_buf.vb,
				   &dst_buf_info->m2m_buf.vb);

	vdec_mpeg2_hw_setup(inst);
	ret = vdec_mpeg2_start_bitstream(inst, bs,
					 vb2_plane_size(&src_buf_info->m2m_buf.vb.vb2_buf, 0));
	if (ret)
		return -EIO;

	writel(vdec_mpeg2_param(pic), inst->dec + VLD_PARAM);
	vdec_mpeg2_set_size(inst, seq);
	writel(0, inst->dec + VLD_QMAT_CTRL);
	writel(VLD_QMAT_INTRA | VLD_QMAT_NON_INTRA, inst->dec + VLD_QMAT_CTRL);
	vdec_mpeg2_load_matrix(inst, VLD_QMAT_INTRA_ADDR,
			       quant->intra_quantiser_matrix);
	vdec_mpeg2_load_matrix(inst, VLD_QMAT_NON_INTRA_ADDR,
			       quant->non_intra_quantiser_matrix);
	writel(0x1ff, inst->dec + VLD_094);
	writel(0x3ff, inst->dec + VLD_098);
	vdec_mpeg2_set_refs(inst, pic, fb);

	writel(VLD_CMD_DECODE, inst->dec + VLD_CMD);
	ret = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					   MPEG2_DEC_TIMEOUT_MS, MTK_VDEC_CORE);

	writel(0xfe, inst->misc + MISC_SPEC_CON);
	writel(0x3f6a151d, inst->misc + MISC_FUNC_CON);
	writel(1, inst->sys + SYS_00);

	/* The last macroblock decoded tells whether the whole picture was */
	mb_x = readl(inst->dec + MC_LAST_MB_X);
	mb_y = readl(inst->dec + MC_LAST_MB_Y);
	if (ret || mb_x != DIV_ROUND_UP(seq->horizontal_size, 16) - 1 ||
	    mb_y != DIV_ROUND_UP(seq->vertical_size, 16) - 1) {
		mtk_vdec_err(ctx, "picture %d: %s, last macroblock (%u, %u)",
			     ctx->decoded_frame_cnt, ret ? "timeout" : "short",
			     mb_x, mb_y);
		return -EIO;
	}

	ctx->decoded_frame_cnt++;
	return 0;
}

/* MM21: luma in 16x32 tiles, chroma in 16x16, the pitch the coded width */
static void vdec_mpeg2_slice_get_pic_info(struct vdec_mpeg2_slice_inst *inst)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;

	ctx->picinfo.buf_w = ALIGN(ctx->picinfo.pic_w, 16);
	ctx->picinfo.buf_h = ALIGN(ctx->picinfo.pic_h, 32);
	ctx->picinfo.fb_sz[0] = ctx->picinfo.buf_w * ctx->picinfo.buf_h;
	ctx->picinfo.fb_sz[1] = ctx->picinfo.fb_sz[0] / 2;
}

static int vdec_mpeg2_slice_get_param(void *h_vdec, enum vdec_get_param_type type,
				      void *out)
{
	struct vdec_mpeg2_slice_inst *inst = h_vdec;
	struct v4l2_rect *crop = out;

	switch (type) {
	case GET_PARAM_PIC_INFO:
		vdec_mpeg2_slice_get_pic_info(inst);
		break;
	case GET_PARAM_CROP_INFO:
		crop->left = 0;
		crop->top = 0;
		crop->width = inst->ctx->picinfo.pic_w;
		crop->height = inst->ctx->picinfo.pic_h;
		break;
	case GET_PARAM_DPB_SIZE:
		*((unsigned int *)out) = MPEG2_DPB_SIZE;
		break;
	default:
		mtk_vdec_err(inst->ctx, "invalid get parameter type=%d", type);
		return -EINVAL;
	}

	return 0;
}

static int vdec_mpeg2_slice_init(struct mtk_vcodec_dec_ctx *ctx)
{
	struct vdec_mpeg2_slice_inst *inst;
	struct mtk_vdec_hw_dev *core;

	core = mtk_vcodec_get_hw_dev(ctx->dev, MTK_VDEC_CORE);
	if (!core || !core->reg_base[VDEC_HW_DEC]) {
		mtk_vdec_err(ctx, "no decode registers on the core");
		return -ENODEV;
	}

	inst = kzalloc_obj(*inst);
	if (!inst)
		return -ENOMEM;

	inst->ctx = ctx;
	inst->dec = core->reg_base[VDEC_HW_DEC];
	inst->misc = core->reg_base[VDEC_HW_MISC];
	inst->sys = core->reg_base[VDEC_HW_SYS];
	ctx->drv_handle = inst;

	return 0;
}

static void vdec_mpeg2_slice_deinit(void *h_vdec)
{
	kfree(h_vdec);
}

const struct vdec_common_if vdec_mpeg2_slice_if = {
	.init		= vdec_mpeg2_slice_init,
	.decode		= vdec_mpeg2_slice_decode,
	.get_param	= vdec_mpeg2_slice_get_param,
	.deinit		= vdec_mpeg2_slice_deinit,
};
