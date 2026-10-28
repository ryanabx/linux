// SPDX-License-Identifier: GPL-2.0
/*
 * H.264 stateless decoding for decoders without firmware (MT8695), where
 * the kernel programs the decoder's registers itself.
 *
 * Each picture runs on the LAT, which parses the slice data (CABAC or
 * CAVLC) into an intermediate stream (the UBE ring), and then on the core,
 * which reconstructs the picture from it. The two are queued through
 * vdec_msg_queue as on MT8192. Both halves have the same blocks: VLD,
 * VLD_TOP, AVC_VLD, MV and PP in the second register range (MC on the core
 * only), misc in the first. The LAT's write DMA (LAT_WDMA) sits in the
 * parent's range.
 *
 * Frame-based V4L2 hands over no slice header, so the LAT's own barrel
 * shifter reads it: AVC_VLD word 143 reads an unsigned Exp-Golomb code, 144
 * a signed one, and word n (0-32) shifts the stream n bits and returns the
 * next 32. The hardware parses ref_pic_list_modification() and
 * pred_weight_table() itself when told to.
 *
 * Only progressive pictures with one slice each for now, 8-bit 4:2:0, no
 * UFO compression (the core writes plain MM21).
 */

#include <linux/iopoll.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <media/v4l2-h264.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "../mtk_vcodec_dec.h"
#include "../mtk_vcodec_dec_hw.h"
#include "../../common/mtk_vcodec_intr.h"
#include "../vdec_drv_base.h"
#include "../vdec_drv_if.h"
#include "../vdec_msg_queue.h"
#include "vdec_h264_req_common.h"

#define H264_DEC_TIMEOUT_MS		100
#define H264_FETCH_TIMEOUT_US		10000
#define H264_DPB_SIZE			16
#define H264_NUM_SLOTS			17

/*
 * After the last slice: the LAT finds a slice's end at the next start
 * code, as the library's buffers always had one (the next access unit's
 * delimiter). Appended in the buffer's spare room.
 */
#define H264_TAIL_SIZE			64

/*
 * The blocks, from each half's second register range (its VLD): on the
 * LAT AVC_VLD +0x1000, MV +0x2000, PP +0x5000; on the core MC +0x1000,
 * AVC_VLD +0x2000, MV +0x3000, PP +0x4000. VLD_TOP is VLD +0x800 on both.
 */
#define VLD_CMD				0x08c
#define  VLD_CMD_FETCH			0x100000
#define  VLD_CMD_SHIFTER_INIT		0x800000
#define VLD_RPTR			0x0b0
#define VLD_BS_START			0x0b4	/* >> 6 */
#define VLD_BS_END			0x0b8	/* >> 6 */
#define VLD_0C0				0x0c0
#define VLD_FETCH_STATUS		0x0e8
#define  VLD_FETCH_READY		BIT(0)
#define VLD_0EC				0x0ec
#define VLD_SRST			0x108
#define VLD_WPTR			0x110
#define VLD_QMAT_ADDR			0x260
#define VLD_QMAT_DATA			0x264

#define VLD_TOP_050			0x050
#define VLD_TOP_054			0x054
#define VLD_TOP_058			0x058
#define VLD_TOP_MB_SIZE			0x068	/* height and width in MBs - 1 */
#define VLD_TOP_06C			0x06c
#define VLD_TOP_PIC_SIZE		0x070	/* height << 16 | width */
#define VLD_TOP_09C			0x09c
#define VLD_TOP_230			0x230
#define VLD_TOP_234			0x234
#define VLD_TOP_ERR_MAP			0x23c
#define VLD_TOP_SLICE_HDR		0x240

#define MC_09				0x024
#define MC_24				0x090
#define MC_32				0x0c8
#define MC_WIDTH			0x208
#define MC_HEIGHT			0x20c
#define MC_OUT_ENABLE			0x220
#define MC_OUT_Y			0x224	/* >> 9 */
#define MC_OUT_C			0x228	/* >> 8 */
#define MC_MB_WIDTH_8B			0x22c
#define MC_8E				0x238
#define MC_94				0x250
#define MC_MB_W1			0x260	/* width in MBs - 1 */
#define MC_MB_H1			0x264	/* height in MBs - 1 */
#define MC_15F				0x57c
#define MC_179				0x5e4
#define MC_198				0x660
#define MC_19A				0x668
#define MC_19B				0x66c
#define MC_1DA				0x768
#define MC_2D7				0xb5c
#define MC_MB_WIDTH			0x980
/* Reference lists' luma and chroma: P, then B0, then B1, 32 each */
#define MC_REF_Y(n)			(4 * (247 + (n)))
#define MC_REF_C(n)			(4 * (728 + (n)))
#define  MC_REF_B0			32
#define  MC_REF_B1			64
#define MC_C_OFFSET			0x55c
#define MC_OUT_Y_ADDR			0x560
#define MC_BOTTOM_P			0x564
#define MC_BOTTOM_B0			0x568
#define MC_BOTTOM_B1			0x56c

#define AVC_SHIFT(n)			(4 * (n))	/* 0-32 */
#define AVC_CTRL			0x084
#define AVC_SPS				0x088
#define AVC_SHDR0			0x08c
#define AVC_PPS				0x090
#define AVC_PPS2			0x094
#define AVC_SHDR1			0x098
#define AVC_SHDR2			0x09c
#define AVC_MAX_FRAME_NUM		0x0a0
#define AVC_CUR_PIC_NUM			0x0a4
#define AVC_P_PIC_NUM(i)		(4 * (42 + (i)))
#define AVC_B0_PIC_NUM(i)		(4 * (74 + (i)))
#define AVC_B1_PIC_NUM(i)		(4 * (106 + (i)))
#define  AVC_PIC_NUM_LONG_TERM		BIT(19)
#define AVC_LIST_INIT			0x228
#define AVC_HW_REORDER			0x22c
#define AVC_HW_WEIGHTS			0x230
#define AVC_141				0x234
#define AVC_DECODE			0x238
#define AVC_UE				0x23c
#define AVC_SE				0x240
#define AVC_151				0x25c
#define AVC_155				0x26c
#define AVC_160				0x280
#define AVC_SHIFTER_SYNC		0x28c
#define AVC_164				0x290
#define AVC_178				0x2c8
#define AVC_180				0x2d0
#define AVC_181				0x2d4
#define AVC_FIND_START_CODE		0x2d8
#define AVC_186				0x2e8

#define MV_CUR				0x20c	/* the picture's MV buffer >> 4 */
#define MV_136				0x220
/* Reference list descriptors: P (2 a frame), B0, B1, B1's MV buffers */
#define MV_P_DESC(i)			(4 * (i))
#define MV_B0_DESC(i)			(4 * (32 + (i)))
#define MV_B1_DESC(i)			(4 * (64 + (i)))
#define MV_B1_MV(i)			(4 * (96 + (i)))
#define MV_POC				0x200
#define MV_POC_TOP			0x204
#define MV_POC_BOTTOM			0x208
#define  MV_POC_MASK			0x3ffff

#define PP_03				0x00c
#define PP_16				0x040
#define PP_QMAT_LOAD			0x504

/* Misc */
#define MISC_01				0x004
#define MISC_SYS_SEL			0x084
#define  MISC_SYS_SEL_H264		2
#define MISC_IRQ_CFG			0x0a4
#define MISC_SPEC_CON			0x0c8
#define MISC_FUNC_CON			0x0cc
#define MISC_DCM_CON			0x0ec
#define MISC_SRST			0x0f4
#define MISC_178			0x178

/* The parent's range */
#define SYS_00				0x000
#define SYS_08				0x008
#define SYS_TRIGGER_CORE		0x300
#define LAT_WDMA			0x1200
#define  WDMA_07			(LAT_WDMA + 0x1c)
#define  WDMA_RST			(LAT_WDMA + 0x2c)
#define  WDMA_RST_DONE			(LAT_WDMA + 0x30)
#define  WDMA_UBE_RPTR			(LAT_WDMA + 0x34)
#define  WDMA_ERR_MAP			(LAT_WDMA + 0x3c)
#define  WDMA_UBE_WPTR			(LAT_WDMA + 0x40)
#define  WDMA_UBE_END			(LAT_WDMA + 0x44)
#define  WDMA_UBE_START			(LAT_WDMA + 0x50)
#define  WDMA_UBE_OFFSET		(LAT_WDMA + 0x78)	/* after the LAT */
#define  WDMA_SLICE_START		(LAT_WDMA + 0x80)
#define  WDMA_SLICE_END			(LAT_WDMA + 0x84)

/* The clock gate blocks: vdeccore (mediatek,vdecsys), vdecsoc */
#define VDECSYS_DCM_CON			0x018
#define VDECSOC_04			0x004
#define VDECSOC_CORE_BUS		0x0f0
#define VDECSOC_LAT_BUS			0x0f8

#define H264_SLICE_P			0
#define H264_SLICE_B			1
#define H264_SLICE_I			2

/* What the LAT's parse of the slice header found, for the core too */
struct vdec_h264_lat_shdr {
	u32 first_mb;
	u32 slice_type;		/* 0-4, modulo 5 */
	u32 slice_type_raw;	/* 0-9, as coded */
	u32 frame_num;
	u32 idr_pic_id;
	u32 poc_lsb;
	s32 delta_poc_bottom;
	bool direct_spatial;
	u32 num_ref_idx[2];	/* active, minus 1 */
	u32 cabac_init_idc;
	s32 qp_delta;
	u32 disable_deblocking;
	s32 alpha_div2;
	s32 beta_div2;
};

/*
 * A reference, by its index in the V4L2 DPB. The hardware knows it by a
 * slot ("fb" in its list descriptors), 0-16, that has to stay with the
 * picture while it's a reference, as the library's frame buffer slots do.
 */
struct vdec_h264_lat_ref {
	dma_addr_t y, c, mv;
	u32 fb;
	u32 pic_num;		/* FrameNumWrap, or LongTermFrameIdx */
	s32 top_poc, bottom_poc;
	bool long_term;
};

/* What a picture's LAT run hands the core */
struct vdec_h264_lat_share {
	struct v4l2_ctrl_h264_sps sps;
	struct v4l2_ctrl_h264_pps pps;
	struct v4l2_ctrl_h264_scaling_matrix scaling;
	struct v4l2_ctrl_h264_decode_params dec;
	struct vdec_h264_lat_shdr shdr;

	/* The initial reference lists (8.2.4.2), as DPB indices */
	struct vdec_h264_lat_ref refs[V4L2_H264_NUM_DPB_ENTRIES];
	u8 p0[V4L2_H264_REF_LIST_LEN], b0[V4L2_H264_REF_LIST_LEN],
	   b1[V4L2_H264_REF_LIST_LEN];
	u8 num_refs;

	u64 trans_start, trans_end;
};

/* One half's blocks; mc is the core's only */
struct vdec_h264_lat_hw {
	void __iomem *vld, *vld_top, *avc, *mv, *pp, *mc, *misc;
	bool lat;
};

struct vdec_h264_lat_inst {
	struct mtk_vcodec_dec_ctx *ctx;
	void __iomem *sys;
	struct vdec_h264_lat_hw lat, core;

	/* One MV buffer per capture buffer, indexed like them */
	struct mtk_vcodec_mem mv_buf[H264_MAX_MV_NUM];
	bool realloc_mv_buf;

	/* The references' slots, and the capture timestamp each holds */
	bool slot_used[H264_NUM_SLOTS];
	u64 slot_ts[H264_NUM_SLOTS];
};

static void vdec_h264_lat_soc_write(struct mtk_vcodec_dec_dev *dev, u32 off, u32 val)
{
	if (dev->vdecsoc_regmap)
		regmap_write(dev->vdecsoc_regmap, off, val);
}

/*
 * The library's reset of one half (H264_HAL_ResetHW): the same on both
 * but for the vdecsoc words, which set up each half's memory access
 * (without them the LAT's fetch never finishes, nor the core's decode).
 */
static void vdec_h264_lat_reset(struct vdec_h264_lat_inst *inst,
				struct vdec_h264_lat_hw *hw)
{
	struct mtk_vcodec_dec_dev *dev = inst->ctx->dev;
	u32 val;

	writel(0x10, hw->misc + MISC_IRQ_CFG);
	writel(0, hw->misc + MISC_IRQ_CFG);
	writel(1, inst->sys + SYS_08);
	if (dev->vdecsys_regmap)
		regmap_set_bits(dev->vdecsys_regmap, VDECSYS_DCM_CON, 0x111);
	if (hw->lat) {
		vdec_h264_lat_soc_write(dev, VDECSOC_LAT_BUS, 0);
		writel(1, inst->sys + WDMA_RST);
		readl_poll_timeout(inst->sys + WDMA_RST_DONE, val, val & BIT(0),
				   1, 1000);
	} else {
		vdec_h264_lat_soc_write(dev, VDECSOC_CORE_BUS, 0x40000000);
	}
	writel(0x101, hw->vld + VLD_SRST);
	writel(0xfff7, hw->misc + MISC_SPEC_CON);
	writel(0x11e30182, hw->misc + MISC_FUNC_CON);
	writel(0xfffffe20, hw->misc + MISC_178);
	writel(0, inst->sys + SYS_00);
	writel(MISC_SYS_SEL_H264, hw->misc + MISC_SYS_SEL);
	if (!hw->lat)
		vdec_h264_lat_soc_write(dev, VDECSOC_04, 2);
	writel(0, hw->misc + MISC_SRST);
	writel(1, hw->misc + MISC_DCM_CON);
	writel(0, hw->vld + VLD_SRST);
}

/*
 * Point a half's VLD at a stream: the LAT at the picture's slices, the
 * core at the LAT's output in the UBE ring. [start, end) is the window
 * (64-byte units), rptr where to begin, wptr where the data ends.
 */
static int vdec_h264_lat_start_bitstream(struct vdec_h264_lat_inst *inst,
					 struct vdec_h264_lat_hw *hw,
					 u32 start, u32 end, u32 rptr, u32 wptr)
{
	u32 val;
	int ret;

	val = readl(hw->vld + VLD_0EC);
	writel(val | 0x10000000, hw->vld + VLD_0EC);
	if (!hw->lat) {
		val = readl(hw->vld_top + VLD_TOP_230);
		writel(val | 0x2, hw->vld_top + VLD_TOP_230);
	}
	writel(0x40000, hw->vld + VLD_0C0);
	writel(0xa880c007, hw->avc + AVC_CTRL);
	val = readl(hw->vld_top + VLD_TOP_054);
	writel(val | 0x1, hw->vld_top + VLD_TOP_054);
	if (!hw->lat)
		writel(6, hw->mc + MC_09);
	writel(start >> 6, hw->vld + VLD_BS_START);
	writel(end >> 6, hw->vld + VLD_BS_END);
	if (hw->lat) {
		writel(1, hw->avc + AVC_SHIFTER_SYNC);
		writel(0, hw->avc + AVC_SHIFTER_SYNC);
	}

	writel(0xffffffff, hw->vld + VLD_WPTR);
	writel(rptr, hw->vld + VLD_RPTR);
	writel(wptr, hw->vld + VLD_WPTR);
	writel(1, hw->avc + AVC_SHIFTER_SYNC);
	writel(0x100, hw->vld + VLD_SRST);
	writel(0, hw->vld + VLD_SRST);
	writel(VLD_CMD_FETCH, hw->vld + VLD_CMD);

	/* Until the fetch is ready, reading the barrel shifter hangs the bus */
	ret = readl_poll_timeout(hw->vld + VLD_FETCH_STATUS, val,
				 val & VLD_FETCH_READY, 10, H264_FETCH_TIMEOUT_US);
	if (ret) {
		mtk_vdec_err(inst->ctx, "%s bitstream fetch timed out",
			     hw->lat ? "lat" : "core");
		return ret;
	}
	writel(VLD_CMD_SHIFTER_INIT, hw->vld + VLD_CMD);
	writel(0, hw->avc + AVC_SHIFTER_SYNC);

	return 0;
}

static void vdec_h264_lat_set_slice_words(struct vdec_h264_lat_inst *inst,
					  struct vdec_h264_lat_hw *hw,
					  const struct vdec_h264_lat_share *sh,
					  dma_addr_t y, dma_addr_t c, dma_addr_t mv);
static void vdec_h264_lat_buf_addr(struct mtk_vcodec_dec_ctx *ctx,
				   struct vb2_buffer *vb, dma_addr_t *y, dma_addr_t *c);

/* The LAT's barrel shifter */
static u32 vdec_h264_lat_peek(struct vdec_h264_lat_inst *inst)
{
	return readl(inst->lat.avc + AVC_SHIFT(0));
}

static u32 vdec_h264_lat_bits(struct vdec_h264_lat_inst *inst, unsigned int n)
{
	u32 val = vdec_h264_lat_peek(inst);

	if (!n)
		return 0;
	readl(inst->lat.avc + AVC_SHIFT(n));
	return val >> (32 - n);
}

static u32 vdec_h264_lat_ue(struct vdec_h264_lat_inst *inst)
{
	return readl(inst->lat.avc + AVC_UE);
}

static s32 vdec_h264_lat_se(struct vdec_h264_lat_inst *inst)
{
	return (s32)readl(inst->lat.avc + AVC_SE);
}

/* Find the slice's start code in hardware; the shifter stops on it */
static int vdec_h264_lat_find_slice(struct vdec_h264_lat_inst *inst, u8 *nal_hdr)
{
	u32 val;
	int ret;

	val = vdec_h264_lat_peek(inst);
	if ((val & ~0xff) != 0x100) {
		writel(1, inst->lat.avc + AVC_FIND_START_CODE);
		ret = readl_poll_timeout(inst->lat.avc + AVC_FIND_START_CODE,
					 val, !(val & BIT(0)), 1, 10000);
		if (ret)
			return ret;
		val = vdec_h264_lat_peek(inst);
	}
	if ((val & ~0xff) != 0x100) {
		mtk_vdec_err(inst->ctx, "no start code: 0x%08x", val);
		return -EINVAL;
	}
	*nal_hdr = val & 0xff;
	return 0;
}

/*
 * The slice header up to the slice data (7.3.3), as the library reads it.
 * The hardware takes ref_pic_list_modification() and pred_weight_table()
 * itself, from where the shifter is, after the words that say what the
 * slice is (vdec_h264_lat_set_slice_words()).
 */
static int vdec_h264_lat_parse_shdr(struct vdec_h264_lat_inst *inst,
				    struct vdec_h264_lat_share *sh, u8 nal_hdr)
{
	const struct v4l2_ctrl_h264_sps *sps = &sh->sps;
	const struct v4l2_ctrl_h264_pps *pps = &sh->pps;
	struct vdec_h264_lat_shdr *h = &sh->shdr;
	u32 nal_type = nal_hdr & 0x1f, nal_ref_idc = (nal_hdr >> 5) & 0x3;
	u32 val;
	int ret;

	h->first_mb = vdec_h264_lat_ue(inst);
	h->slice_type_raw = vdec_h264_lat_ue(inst);
	h->slice_type = h->slice_type_raw % 5;
	vdec_h264_lat_ue(inst);		/* pic_parameter_set_id */
	if (sps->flags & V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE)
		vdec_h264_lat_bits(inst, 2);
	h->frame_num = vdec_h264_lat_bits(inst, sps->log2_max_frame_num_minus4 + 4);
	if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY))
		return -EOPNOTSUPP;	/* field_pic_flag: not yet */
	if (nal_type == NAL_IDR_SLICE)
		h->idr_pic_id = vdec_h264_lat_ue(inst);
	if (sps->pic_order_cnt_type == 0) {
		h->poc_lsb = vdec_h264_lat_bits(inst,
						sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
		if (pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT)
			h->delta_poc_bottom = vdec_h264_lat_se(inst);
	} else if (sps->pic_order_cnt_type == 1 &&
		   !(sps->flags & V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO)) {
		vdec_h264_lat_se(inst);
		if (pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT)
			vdec_h264_lat_se(inst);
	}
	if (pps->flags & V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT)
		vdec_h264_lat_ue(inst);
	if (h->slice_type == H264_SLICE_B)
		h->direct_spatial = vdec_h264_lat_bits(inst, 1);
	h->num_ref_idx[0] = pps->num_ref_idx_l0_default_active_minus1;
	h->num_ref_idx[1] = pps->num_ref_idx_l1_default_active_minus1;
	if (h->slice_type == H264_SLICE_P || h->slice_type == H264_SLICE_B) {
		if (vdec_h264_lat_bits(inst, 1)) {
			h->num_ref_idx[0] = vdec_h264_lat_ue(inst);
			if (h->slice_type == H264_SLICE_B)
				h->num_ref_idx[1] = vdec_h264_lat_ue(inst);
		}
	}
	mtk_vdec_debug(inst->ctx, "nal 0x%02x: first_mb %u type %u frame_num %u poc_lsb %u refs %u/%u",
		       nal_hdr, h->first_mb, h->slice_type_raw, h->frame_num,
		       h->poc_lsb, h->num_ref_idx[0], h->num_ref_idx[1]);
	vdec_h264_lat_set_slice_words(inst, &inst->lat, sh, 0, 0, 0);

	if (h->slice_type != H264_SLICE_I) {
		writel(1, inst->lat.avc + AVC_HW_REORDER);
		ret = readl_poll_timeout(inst->lat.avc + AVC_HW_REORDER, val,
					 val, 1, 10000);
		if (ret) {
			mtk_vdec_err(inst->ctx, "list modification: 0x%x", val);
			return ret;
		}
	}
	if (((pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED) &&
	     h->slice_type == H264_SLICE_P) ||
	    (pps->weighted_bipred_idc == 1 && h->slice_type == H264_SLICE_B)) {
		writel(1, inst->lat.avc + AVC_HW_WEIGHTS);
		ret = readl_poll_timeout(inst->lat.avc + AVC_HW_WEIGHTS, val,
					 val, 1, 10000);
		if (ret) {
			mtk_vdec_err(inst->ctx, "weight table: 0x%x", val);
			return ret;
		}
	}
	if (nal_ref_idc) {
		/* dec_ref_pic_marking(), which V4L2 has already applied */
		if (nal_type == NAL_IDR_SLICE) {
			vdec_h264_lat_bits(inst, 2);
		} else if (vdec_h264_lat_bits(inst, 1)) {
			u32 mmco;

			do {
				mmco = vdec_h264_lat_ue(inst);
				if (mmco == 1 || mmco == 3)
					vdec_h264_lat_ue(inst);
				if (mmco == 2 || mmco == 3 || mmco == 4 || mmco == 6)
					vdec_h264_lat_ue(inst);
			} while (mmco);
		}
	}
	if ((pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) &&
	    h->slice_type != H264_SLICE_I)
		h->cabac_init_idc = vdec_h264_lat_ue(inst);
	h->qp_delta = vdec_h264_lat_se(inst);
	if (pps->flags & V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT) {
		h->disable_deblocking = vdec_h264_lat_ue(inst);
		if (h->disable_deblocking != 1) {
			h->alpha_div2 = vdec_h264_lat_se(inst);
			h->beta_div2 = vdec_h264_lat_se(inst);
		}
	}

	return 0;
}

static u32 vdec_h264_lat_sps_word(const struct v4l2_ctrl_h264_sps *sps)
{
	return (sps->chroma_format_idc & 0x3) |
	       (sps->log2_max_frame_num_minus4 & 0xf) << 2 |
	       (sps->pic_order_cnt_type & 0x3) << 6 |
	       (sps->log2_max_pic_order_cnt_lsb_minus4 & 0xf) << 8 |
	       !!(sps->flags & V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO) << 12 |
	       (sps->max_num_ref_frames & 0x1f) << 13 |
	       !!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) << 18 |
	       !!(sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD) << 19 |
	       !!(sps->flags & V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE) << 20 |
	       BIT(21);
}

static u32 vdec_h264_lat_pps_word(const struct v4l2_ctrl_h264_pps *pps)
{
	return !!(pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) |
	       !!(pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT) << 1 |
	       !!(pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED) << 2 |
	       (pps->weighted_bipred_idc & 0x3) << 3 |
	       (pps->pic_init_qp_minus26 & 0x3f) << 5 |
	       (pps->chroma_qp_index_offset & 0x1f) << 11 |
	       !!(pps->flags & V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT) << 16 |
	       !!(pps->flags & V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED) << 17 |
	       !!(pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE) << 18 |
	       (pps->second_chroma_qp_index_offset & 0x1f) << 19;
}

/* V4L2 sets this when the SPS or the PPS has a matrix, with fall-backs applied */
static bool vdec_h264_lat_has_scaling(const struct vdec_h264_lat_share *sh)
{
	return sh->pps.flags & V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT;
}

/* The default reference counts, and which scaling lists are loaded */
static u32 vdec_h264_lat_pps2_word(const struct vdec_h264_lat_share *sh)
{
	u32 val = (sh->pps.num_ref_idx_l0_default_active_minus1 & 0x1f) |
		  (sh->pps.num_ref_idx_l1_default_active_minus1 & 0x1f) << 5;

	if (vdec_h264_lat_has_scaling(sh))
		val |= 0xff << 10 | BIT(18);
	return val;
}

/* The slice: its type, the active reference counts, direct mode */
static u32 vdec_h264_lat_shdr1(const struct vdec_h264_lat_share *sh)
{
	const struct vdec_h264_lat_shdr *h = &sh->shdr;

	return (h->slice_type_raw & 0xf) << 13 |
	       !!(sh->dec.flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC) << 17 |
	       !!(sh->dec.flags & V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD) << 18 |
	       h->direct_spatial << 19 |
	       min(h->num_ref_idx[0], 31U) << 20 |
	       (h->slice_type == H264_SLICE_B ? min(h->num_ref_idx[1], 31U) : 0) << 25;
}

/* The rest of the slice header, and what kind of picture it is */
static u32 vdec_h264_lat_shdr2(const struct vdec_h264_lat_share *sh)
{
	const struct vdec_h264_lat_shdr *h = &sh->shdr;

	return (h->qp_delta & 0x7f) |
	       (h->disable_deblocking & 0x3) << 7 |
	       (h->alpha_div2 & 0xf) << 9 |
	       (h->beta_div2 & 0xf) << 13 |
	       (sh->dec.nal_ref_idc & 0x3) << 17 |
	       !!(sh->dec.flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) << 19 |
	       (h->cabac_init_idc & 0x3) << 20;
}

static int vdec_h264_lat_find_slot(struct vdec_h264_lat_inst *inst, u64 ts)
{
	int i;

	for (i = 0; i < H264_NUM_SLOTS; i++)
		if (inst->slot_used[i] && inst->slot_ts[i] == ts)
			return i;
	return -1;
}

static int vdec_h264_lat_slot(struct vdec_h264_lat_inst *inst, u64 ts)
{
	int i = vdec_h264_lat_find_slot(inst, ts);

	return i < 0 ? 0 : i;
}

/*
 * Free the slots of pictures that left the DPB, then give new references
 * the lowest free slot
 */
static void vdec_h264_lat_update_slots(struct vdec_h264_lat_inst *inst,
				       const struct v4l2_ctrl_h264_decode_params *dec)
{
	bool keep[H264_NUM_SLOTS] = { };
	int i, j;

	for (i = 0; i < V4L2_H264_NUM_DPB_ENTRIES; i++) {
		if (!(dec->dpb[i].flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE))
			continue;
		j = vdec_h264_lat_find_slot(inst, dec->dpb[i].reference_ts);
		if (j >= 0)
			keep[j] = true;
	}
	for (j = 0; j < H264_NUM_SLOTS; j++)
		inst->slot_used[j] = keep[j];

	for (i = 0; i < V4L2_H264_NUM_DPB_ENTRIES; i++) {
		u64 ts = dec->dpb[i].reference_ts;

		if (!(dec->dpb[i].flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) ||
		    vdec_h264_lat_find_slot(inst, ts) >= 0)
			continue;
		for (j = 0; j < H264_NUM_SLOTS; j++) {
			if (!inst->slot_used[j]) {
				inst->slot_used[j] = true;
				inst->slot_ts[j] = ts;
				break;
			}
		}
	}
}

/*
 * The initial reference lists (8.2.4.2) with every reference picture, as
 * the library programs them: the hardware applies the slice's
 * ref_pic_list_modification() itself. Also each reference's buffers.
 */
static void vdec_h264_lat_build_lists(struct vdec_h264_lat_inst *inst,
				      struct vdec_h264_lat_share *sh)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vb2_queue *vq = v4l2_m2m_get_dst_vq(ctx->m2m_ctx);
	struct v4l2_h264_reflist_builder b;
	struct v4l2_h264_reference p0[V4L2_H264_REF_LIST_LEN];
	struct v4l2_h264_reference b0[V4L2_H264_REF_LIST_LEN];
	struct v4l2_h264_reference b1[V4L2_H264_REF_LIST_LEN];
	int i;

	v4l2_h264_init_reflist_builder(&b, &sh->dec, &sh->sps, sh->dec.dpb);
	v4l2_h264_build_p_ref_list(&b, p0);
	v4l2_h264_build_b_ref_lists(&b, b0, b1);
	sh->num_refs = b.num_valid;
	for (i = 0; i < b.num_valid; i++) {
		sh->p0[i] = p0[i].index;
		sh->b0[i] = b0[i].index;
		sh->b1[i] = b1[i].index;
	}

	vdec_h264_lat_update_slots(inst, &sh->dec);
	for (i = 0; i < V4L2_H264_NUM_DPB_ENTRIES; i++) {
		const struct v4l2_h264_dpb_entry *e = &sh->dec.dpb[i];
		struct vdec_h264_lat_ref *r = &sh->refs[i];
		struct vb2_buffer *vb;

		if (!(e->flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE))
			continue;
		r->fb = vdec_h264_lat_slot(inst, e->reference_ts);
		r->pic_num = b.refs[i].frame_num;
		r->long_term = b.refs[i].longterm;
		r->top_poc = b.refs[i].top_field_order_cnt;
		r->bottom_poc = b.refs[i].bottom_field_order_cnt;
		vb = vb2_find_buffer(vq, e->reference_ts);
		if (!vb || vb->index >= H264_MAX_MV_NUM) {
			mtk_vdec_err(ctx, "reference %d: no capture buffer", i);
			continue;
		}
		vdec_h264_lat_buf_addr(ctx, vb, &r->y, &r->c);
		r->mv = inst->mv_buf[vb->index].dma_addr;
	}
}

/* A reference field's list descriptor; B lists add the field's POC */
static u32 vdec_h264_lat_desc(const struct vdec_h264_lat_ref *r, bool bottom,
			      bool poc)
{
	u32 val = r->fb << 23 | bottom << 22 | r->long_term << 20;

	if (r->top_poc <= r->bottom_poc)
		val |= BIT(21);
	if (poc)
		val += (bottom ? r->bottom_poc : r->top_poc) & MV_POC_MASK;
	return val;
}

static u32 vdec_h264_lat_pic_num(const struct vdec_h264_lat_ref *r)
{
	return (r->pic_num & 0x7ffff) |
	       (r->long_term ? AVC_PIC_NUM_LONG_TERM : 0);
}

/* The reference lists; the core also gets their buffers */
static void vdec_h264_lat_set_lists(struct vdec_h264_lat_inst *inst,
				    struct vdec_h264_lat_hw *hw,
				    const struct vdec_h264_lat_share *sh,
				    dma_addr_t y, dma_addr_t c, dma_addr_t mv)
{
	const struct v4l2_ctrl_h264_decode_params *dec = &sh->dec;
	u32 val;
	int i;

	/* H264_HAL_InitPRefList */
	if (!hw->lat)
		writel(0, hw->mc + MC_BOTTOM_P);
	writel(1, hw->avc + AVC_LIST_INIT);
	readl_poll_timeout(hw->avc + AVC_LIST_INIT, val, !(val & BIT(0)), 1, 1000);
	writel(1 << (sh->sps.log2_max_frame_num_minus4 + 4),
	       hw->avc + AVC_MAX_FRAME_NUM);
	writel(dec->frame_num, hw->avc + AVC_CUR_PIC_NUM);

	/* Unused entries point at the picture itself */
	if (!hw->lat) {
		for (i = 0; i < 96; i++) {
			writel(y, hw->mc + MC_REF_Y(i));
			writel(c, hw->mc + MC_REF_C(i));
			writel(mv, hw->mv + MV_P_DESC(i));
		}
	}

	for (i = 0; i < sh->num_refs; i++) {
		const struct vdec_h264_lat_ref *r = &sh->refs[sh->p0[i]];

		writel(vdec_h264_lat_pic_num(r), hw->avc + AVC_P_PIC_NUM(i));
		writel(vdec_h264_lat_desc(r, false, false),
		       hw->mv + MV_P_DESC(2 * i));
		writel(vdec_h264_lat_desc(r, true, false),
		       hw->mv + MV_P_DESC(2 * i + 1));
		if (!hw->lat) {
			writel(r->y, hw->mc + MC_REF_Y(i));
			writel(r->c, hw->mc + MC_REF_C(i));
		}
	}

	/* H264_HAL_SetPOC */
	writel(dec->top_field_order_cnt & MV_POC_MASK, hw->mv + MV_POC_TOP);
	writel(dec->bottom_field_order_cnt & MV_POC_MASK, hw->mv + MV_POC_BOTTOM);
	writel(min(dec->top_field_order_cnt, dec->bottom_field_order_cnt) &
	       MV_POC_MASK, hw->mv + MV_POC);

	/* H264_HAL_InitBRefList, then the B lists */
	if (!hw->lat) {
		writel(0, hw->mc + MC_BOTTOM_B0);
		writel(0, hw->mc + MC_BOTTOM_B1);
		val = readl(hw->mv + MV_136);
		writel(val | 0x20, hw->mv + MV_136);	/* the reduced MV buffer */
	}
	for (i = 0; i < sh->num_refs; i++) {
		const struct vdec_h264_lat_ref *r0 = &sh->refs[sh->b0[i]];
		const struct vdec_h264_lat_ref *r1 = &sh->refs[sh->b1[i]];

		writel(vdec_h264_lat_pic_num(r0), hw->avc + AVC_B0_PIC_NUM(i));
		writel(vdec_h264_lat_pic_num(r1), hw->avc + AVC_B1_PIC_NUM(i));
		if (hw->lat)
			continue;
		writel(r0->y, hw->mc + MC_REF_Y(MC_REF_B0 + i));
		writel(r0->c, hw->mc + MC_REF_C(MC_REF_B0 + i));
		writel(vdec_h264_lat_desc(r0, false, true),
		       hw->mv + MV_B0_DESC(2 * i));
		writel(vdec_h264_lat_desc(r0, true, true),
		       hw->mv + MV_B0_DESC(2 * i + 1));
		writel(r1->y, hw->mc + MC_REF_Y(MC_REF_B1 + i));
		writel(r1->c, hw->mc + MC_REF_C(MC_REF_B1 + i));
		writel(vdec_h264_lat_desc(r1, false, true),
		       hw->mv + MV_B1_DESC(2 * i));
		writel(vdec_h264_lat_desc(r1, true, true),
		       hw->mv + MV_B1_DESC(2 * i + 1));
		writel(r1->mv >> 4, hw->mv + MV_B1_MV(2 * i));
		writel((r1->mv >> 4) + 2, hw->mv + MV_B1_MV(2 * i + 1));
	}
}

/*
 * The scaling lists, in raster order as V4L2 has them: six 4x4 lists at
 * byte 16 * list, the two 8x8 luma lists at 0x60 and 0xa0, two
 * coefficients a write.
 */
static void vdec_h264_lat_load_scaling(struct vdec_h264_lat_hw *hw,
				       const struct v4l2_ctrl_h264_scaling_matrix *sm)
{
	u32 val;
	int i, k;

	val = readl(hw->pp + PP_QMAT_LOAD);
	writel(val | 1, hw->pp + PP_QMAT_LOAD);
	for (i = 0; i < 6; i++) {
		for (k = 0; k < 16; k += 2) {
			writel(16 * i + k, hw->vld + VLD_QMAT_ADDR);
			writel(sm->scaling_list_4x4[i][k] |
			       sm->scaling_list_4x4[i][k + 1] << 8,
			       hw->vld + VLD_QMAT_DATA);
		}
	}
	for (i = 0; i < 2; i++) {
		for (k = 0; k < 64; k += 2) {
			writel((i ? 0xa0 : 0x60) + k, hw->vld + VLD_QMAT_ADDR);
			writel(sm->scaling_list_8x8[i][k] |
			       sm->scaling_list_8x8[i][k + 1] << 8,
			       hw->vld + VLD_QMAT_DATA);
		}
	}
	val = readl(hw->pp + PP_QMAT_LOAD);
	writel(val & ~1, hw->pp + PP_QMAT_LOAD);
}

/*
 * What the library writes once the slice header is read as far as the
 * reference counts (Vdec_Drv_setDecPicInfo): the lists, the POC, the
 * scaling lists, the SPS, PPS and slice words. On the core, the picture's
 * buffers go in as the default for unused list entries.
 */
static void vdec_h264_lat_set_slice_words(struct vdec_h264_lat_inst *inst,
					  struct vdec_h264_lat_hw *hw,
					  const struct vdec_h264_lat_share *sh,
					  dma_addr_t y, dma_addr_t c, dma_addr_t mv)
{
	u32 val;

	vdec_h264_lat_set_lists(inst, hw, sh, y, c, mv);
	if (vdec_h264_lat_has_scaling(sh))
		vdec_h264_lat_load_scaling(hw, &sh->scaling);
	writel(vdec_h264_lat_sps_word(&sh->sps), hw->avc + AVC_SPS);
	writel(sh->sps.pic_width_in_mbs_minus1 |
	       sh->sps.pic_height_in_map_units_minus1 << 16,
	       hw->vld_top + VLD_TOP_MB_SIZE);
	writel(vdec_h264_lat_pps_word(&sh->pps), hw->avc + AVC_PPS);
	writel(vdec_h264_lat_pps2_word(sh), hw->avc + AVC_PPS2);
	val = readl(hw->avc + AVC_SHDR0);
	writel((val & ~0x1ffff) | (sh->shdr.first_mb & 0x1fff), hw->avc + AVC_SHDR0);
	val = readl(hw->avc + AVC_SHDR1);
	writel((val & 0x1fff) | vdec_h264_lat_shdr1(sh), hw->avc + AVC_SHDR1);
}

/*
 * What both halves write last before their trigger: the picture size
 * words, then H264_HAL_SetErrConceal
 */
static void vdec_h264_lat_start_common(struct vdec_h264_lat_inst *inst,
				       struct vdec_h264_lat_hw *hw,
				       const struct vdec_h264_lat_share *sh)
{
	u32 w = sh->sps.pic_width_in_mbs_minus1 + 1;
	u32 h = sh->sps.pic_height_in_map_units_minus1 + 1;
	u32 val;

	writel(w * h, hw->vld_top + VLD_TOP_06C);
	writel(0xffffffff, hw->vld_top + VLD_TOP_050);
	writel((h * 16) << 16 | (w * 16), hw->vld_top + VLD_TOP_PIC_SIZE);

	val = readl(hw->avc + AVC_CTRL);
	writel((val | 0x0680c000) & ~0x01400000, hw->avc + AVC_CTRL);
	writel(0xbfddb8ff, hw->avc + AVC_160);
	writel(0x08001101, hw->avc + AVC_180);
	writel(0x00204008, hw->avc + AVC_181);
	val = readl(hw->pp + PP_16);
	writel(val | 0x3011, hw->pp + PP_16);
	val = readl(hw->avc + AVC_186);
	writel(val | 0x20, hw->avc + AVC_186);
	val = readl(hw->avc + AVC_SHDR0);
	writel(val & ~0xffff, hw->avc + AVC_SHDR0);
}

/* CABAC: the hardware's init, and on the LAT its alignment */
static void vdec_h264_lat_cabac_init(struct vdec_h264_lat_hw *hw,
				     const struct vdec_h264_lat_share *sh)
{
	u32 val;

	if (!(sh->pps.flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE))
		return;
	writel(1, hw->avc + AVC_141);
	if (hw->lat)
		readl_poll_timeout(hw->avc + AVC_151, val, !(val & BIT(25)), 1, 1000);
}

/* The LAT's run: the slice header, then the LAT's own decode */
static int vdec_h264_lat_run(struct vdec_h264_lat_inst *inst,
			     struct vdec_h264_lat_share *sh,
			     struct mtk_vcodec_mem *bs, size_t capacity,
			     struct vdec_lat_buf *lat_buf)
{
	static const u8 aud[] = { 0x00, 0x00, 0x00, 0x01, 0x09, 0xf0 };
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vdec_msg_queue *q = &ctx->msg_queue;
	struct vdec_h264_lat_hw *hw = &inst->lat;
	u32 start = bs->dma_addr, end = bs->dma_addr + bs->size;
	u32 ube = q->wdma_addr.dma_addr, val;
	u8 nal_hdr;
	int ret;

	if (capacity < bs->size + H264_TAIL_SIZE) {
		mtk_vdec_err(ctx, "no room after the slices");
		return -ENOSPC;
	}
	memset(bs->va + bs->size, 0, H264_TAIL_SIZE);
	memcpy(bs->va + bs->size, aud, sizeof(aud));
	end += H264_TAIL_SIZE;

	vdec_h264_lat_reset(inst, hw);
	ret = vdec_h264_lat_start_bitstream(inst, hw, start,
					    bs->dma_addr + capacity, start, end);
	if (ret)
		return ret;
	ret = vdec_h264_lat_find_slice(inst, &nal_hdr);
	if (ret) {
		mtk_vdec_err(ctx, "no slice start code");
		return ret;
	}

	/* Where the LAT writes: the UBE ring after the core's data */
	val = readl(inst->sys + WDMA_07);
	writel(val | 0x3, inst->sys + WDMA_07);
	writel(lat_buf->wdma_err_addr.dma_addr, inst->sys + WDMA_ERR_MAP);
	writel(q->wdma_wptr_addr, inst->sys + WDMA_UBE_WPTR);
	writel(q->wdma_rptr_addr, inst->sys + WDMA_UBE_RPTR);
	writel(ube + q->wdma_addr.size, inst->sys + WDMA_UBE_END);
	writel(ube, inst->sys + WDMA_UBE_START);
	writel(lat_buf->slice_bc_addr.dma_addr, inst->sys + WDMA_SLICE_START);
	writel(lat_buf->slice_bc_addr.dma_addr + lat_buf->slice_bc_addr.size,
	       inst->sys + WDMA_SLICE_END);
	val = readl(hw->vld_top + VLD_TOP_230);
	writel(val | 0x1, hw->vld_top + VLD_TOP_230);

	/* Past the start code and the NAL header */
	readl(hw->avc + AVC_SHIFT(32));
	ret = vdec_h264_lat_parse_shdr(inst, sh, nal_hdr);
	if (ret) {
		mtk_vdec_err(ctx, "slice header: %d", ret);
		return ret;
	}

	/* H264_HAL_LATDecStart */
	writel(0xffffffff, hw->vld_top + VLD_TOP_058);
	writel(0xa89ec007, hw->avc + AVC_CTRL);
	writel(0xffdfbfff, hw->avc + AVC_160);
	writel(vdec_h264_lat_shdr2(sh), hw->avc + AVC_SHDR2);
	vdec_h264_lat_cabac_init(hw, sh);
	vdec_h264_lat_start_common(inst, hw, sh);

	/* TriggerLAT */
	val = readl(hw->avc + AVC_180);
	writel(val | 0x40000000, hw->avc + AVC_180);
	val = readl(hw->vld_top + VLD_TOP_09C);
	writel((val & ~0x1fff) | 0x800, hw->vld_top + VLD_TOP_09C);
	writel(0xbfddb8ff, hw->avc + AVC_155);
	writel(0, hw->avc + AVC_164);
	val = readl(hw->misc + MISC_01);
	writel(val | 0x10, hw->misc + MISC_01);
	writel(1, hw->avc + AVC_DECODE);

	ret = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					   H264_DEC_TIMEOUT_MS, MTK_VDEC_LAT0);
	if (ret) {
		mtk_vdec_err(ctx, "lat timeout: picture %d", ctx->decoded_frame_cnt);
		return -ETIMEDOUT;
	}

	/* The LAT's output ends at its new write offset in the ring */
	sh->trans_start = q->wdma_wptr_addr;
	sh->trans_end = ube + readl(inst->sys + WDMA_UBE_OFFSET);
	vdec_msg_queue_update_ube_wptr(q, sh->trans_end);
	return 0;
}

/* The picture's luma and chroma addresses, from its capture buffer */
static void vdec_h264_lat_buf_addr(struct mtk_vcodec_dec_ctx *ctx,
				   struct vb2_buffer *vb, dma_addr_t *y, dma_addr_t *c)
{
	*y = vb2_dma_contig_plane_dma_addr(vb, 0);
	if (ctx->q_data[MTK_Q_DATA_DST].fmt->num_planes == 2)
		*c = vb2_dma_contig_plane_dma_addr(vb, 1);
	else
		*c = *y + ctx->picinfo.fb_sz[0];
}

/* The core's run: reconstruct the picture from the LAT's output */
static int vdec_h264_lat_core_run(struct vdec_h264_lat_inst *inst,
				  struct vdec_h264_lat_share *sh,
				  struct vdec_lat_buf *lat_buf, struct vb2_buffer *dst)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vdec_msg_queue *q = &ctx->msg_queue;
	struct vdec_h264_lat_hw *hw = &inst->core;
	u32 ube = q->wdma_addr.dma_addr;
	u32 w = sh->sps.pic_width_in_mbs_minus1 + 1;
	u32 h = sh->sps.pic_height_in_map_units_minus1 + 1;
	dma_addr_t y, c;
	u32 val, mb_x, mb_y;
	int ret;

	vdec_h264_lat_buf_addr(ctx, dst, &y, &c);

	vdec_h264_lat_reset(inst, hw);
	ret = vdec_h264_lat_start_bitstream(inst, hw, ube, ube + q->wdma_addr.size,
					    sh->trans_start, sh->trans_end + 0x800);
	if (ret)
		return ret;
	vdec_h264_lat_set_slice_words(inst, hw, sh, y, c,
				      inst->mv_buf[dst->index].dma_addr);

	/* H264_HAL_DecStart, without UFO */
	val = readl(hw->mv + MV_136);
	writel(val | 0x20, hw->mv + MV_136);	/* the reduced MV buffer */
	writel(3, hw->misc + MISC_01);		/* CRC on */
	writel(4, hw->mc + MC_24);
	writel(0xffffffff, hw->vld_top + VLD_TOP_058);
	writel(w * 16, hw->mc + MC_WIDTH);
	writel(h * 16, hw->mc + MC_HEIGHT);
	writel(0xa89ec007, hw->avc + AVC_CTRL);
	writel(0xbfddb8ff, hw->avc + AVC_160);
	writel(1, hw->mc + MC_OUT_ENABLE);
	writel(c - y, hw->mc + MC_C_OFFSET);
	writel(y >> 9, hw->mc + MC_OUT_Y);
	writel(c >> 8, hw->mc + MC_OUT_C);
	writel(1, hw->mc + MC_94);
	writel(3, hw->mc + MC_8E);
	writel(w - 1, hw->mc + MC_MB_W1);
	writel(h - 1, hw->mc + MC_MB_H1);
	writel(w, hw->mc + MC_MB_WIDTH_8B);
	writel(0, hw->mc + MC_15F);
	val = readl(hw->mc + MC_19A);
	writel((val & 0xff000000) | 1, hw->mc + MC_19A);
	val = readl(hw->mc + MC_19B);
	writel((val & 0xff000000) | 1, hw->mc + MC_19B);
	writel(w, hw->mc + MC_32);
	writel(1, hw->pp + PP_03);
	writel(y, hw->mc + MC_OUT_Y_ADDR);
	writel(w, hw->mc + MC_MB_WIDTH);
	val = readl(hw->avc + AVC_178);
	writel(val & ~0x20, hw->avc + AVC_178);
	writel(inst->mv_buf[dst->index].dma_addr >> 4, hw->mv + MV_CUR);
	writel(vdec_h264_lat_shdr2(sh), hw->avc + AVC_SHDR2);
	vdec_h264_lat_cabac_init(hw, sh);
	val = readl(hw->mc + MC_179);
	writel(val | 0x1000, hw->mc + MC_179);
	val = readl(hw->mc + MC_198);
	writel(val | 0x80000000, hw->mc + MC_198);
	writel(4, hw->mc + MC_1DA);
	val = readl(hw->mc + MC_1DA);
	writel(val | 0x88000000, hw->mc + MC_1DA);
	vdec_h264_lat_start_common(inst, hw, sh);
	val = readl(hw->misc + MISC_IRQ_CFG);
	writel(val | 0x10000000, hw->misc + MISC_IRQ_CFG);

	/* H264_HAL_TriggerCore */
	writel(1, inst->sys + SYS_TRIGGER_CORE);
	val = readl(hw->vld_top + VLD_TOP_234);
	writel(val | 1, hw->vld_top + VLD_TOP_234);
	writel(lat_buf->wdma_err_addr.dma_addr, hw->vld_top + VLD_TOP_ERR_MAP);
	writel(lat_buf->slice_bc_addr.dma_addr, hw->vld_top + VLD_TOP_SLICE_HDR);
	val = readl(hw->avc + AVC_164);
	writel(val | 1, hw->avc + AVC_164);
	val = readl(hw->vld_top + VLD_TOP_09C);
	writel((val & ~0x1fff) | 0x800, hw->vld_top + VLD_TOP_09C);
	val = readl(hw->avc + AVC_180);
	writel(val | 0x40000000, hw->avc + AVC_180);
	writel(0x110, hw->misc + MISC_01);
	writel(1, hw->avc + AVC_DECODE);

	ret = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					   H264_DEC_TIMEOUT_MS, MTK_VDEC_CORE);
	writel(0x10, hw->misc + MISC_IRQ_CFG);
	writel(0, hw->misc + MISC_IRQ_CFG);

	/* The last macroblock decoded tells whether the whole picture was */
	mb_x = readl(hw->mc + 0x028);
	mb_y = readl(hw->mc + 0x02c);
	if (ret || mb_x != w - 1 || mb_y != h - 1) {
		mtk_vdec_err(ctx, "picture %d: %s, last macroblock (%u, %u)",
			     ctx->decoded_frame_cnt, ret ? "timeout" : "short",
			     mb_x, mb_y);
		return -EIO;
	}
	return 0;
}

static int vdec_h264_lat_core_decode(struct vdec_lat_buf *lat_buf)
{
	struct mtk_vcodec_dec_ctx *ctx = lat_buf->ctx;
	struct vdec_h264_lat_inst *inst = ctx->drv_handle;
	struct vdec_h264_lat_share *sh = lat_buf->private_data;
	struct vb2_v4l2_buffer *dst;
	struct vdec_fb *fb;
	int err = 0;

	fb = ctx->dev->vdec_pdata->get_cap_buffer(ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);
	if (!fb || !dst) {
		mtk_vdec_err(ctx, "no capture buffer");
		err = -EBUSY;
		goto out;
	}
	v4l2_m2m_buf_copy_metadata(&lat_buf->ts_info, dst);
	err = vdec_h264_lat_core_run(inst, sh, lat_buf, &dst->vb2_buf);

out:
	vdec_msg_queue_update_ube_rptr(&ctx->msg_queue, sh->trans_end);
	ctx->dev->vdec_pdata->cap_to_disp(ctx, !!err, lat_buf->src_buf_req);
	ctx->decoded_frame_cnt++;
	return 0;
}

static int vdec_h264_lat_alloc_mv_bufs(struct vdec_h264_lat_inst *inst)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	unsigned int size = mtk_vdec_h264_get_mv_buf_size(ctx->picinfo.buf_w,
							  ctx->picinfo.buf_h);
	int i, err;

	for (i = 0; i < H264_MAX_MV_NUM; i++) {
		struct mtk_vcodec_mem *mem = &inst->mv_buf[i];

		if (mem->va)
			mtk_vcodec_mem_free(ctx, mem);
		mem->size = size;
		err = mtk_vcodec_mem_alloc(ctx, mem);
		if (err)
			return err;
	}
	return 0;
}

static int vdec_h264_lat_decode(void *h_vdec, struct mtk_vcodec_mem *bs,
				struct vdec_fb *unused, bool *res_chg)
{
	struct vdec_h264_lat_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct mtk_video_dec_buf *src_buf;
	struct vdec_h264_lat_share *sh;
	struct vdec_lat_buf *lat_buf;
	const void *p;
	int err;

	if (vdec_msg_queue_init(&ctx->msg_queue, ctx, vdec_h264_lat_core_decode,
				sizeof(*sh)))
		return -ENOMEM;

	/* bs NULL means flush the decoder */
	if (!bs) {
		vdec_msg_queue_wait_lat_buf_full(&ctx->msg_queue);
		return 0;
	}

	*res_chg = false;
	if (inst->realloc_mv_buf) {
		err = vdec_h264_lat_alloc_mv_bufs(inst);
		if (err)
			return err;
		inst->realloc_mv_buf = false;
	}

	lat_buf = vdec_msg_queue_dqbuf(&ctx->msg_queue.lat_ctx);
	if (!lat_buf)
		return -EAGAIN;
	sh = lat_buf->private_data;
	src_buf = container_of(bs, struct mtk_video_dec_buf, bs_buffer);
	lat_buf->src_buf_req = src_buf->m2m_buf.vb.vb2_buf.req_obj.req;
	v4l2_m2m_buf_copy_metadata(&src_buf->m2m_buf.vb, &lat_buf->ts_info);

	memset(sh, 0, sizeof(*sh));
	p = mtk_vdec_h264_get_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_SPS);
	memcpy(&sh->sps, p, sizeof(sh->sps));
	p = mtk_vdec_h264_get_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_PPS);
	memcpy(&sh->pps, p, sizeof(sh->pps));
	p = mtk_vdec_h264_get_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_SCALING_MATRIX);
	memcpy(&sh->scaling, p, sizeof(sh->scaling));
	p = mtk_vdec_h264_get_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_DECODE_PARAMS);
	memcpy(&sh->dec, p, sizeof(sh->dec));
	vdec_h264_lat_build_lists(inst, sh);

	err = vdec_h264_lat_run(inst, sh, bs,
				vb2_plane_size(&src_buf->m2m_buf.vb.vb2_buf, 0),
				lat_buf);
	if (err) {
		vdec_msg_queue_qbuf(&ctx->msg_queue.lat_ctx, lat_buf);
		return err;
	}
	vdec_msg_queue_qbuf(&ctx->msg_queue.core_ctx, lat_buf);
	return 0;
}

/* MM21: luma in 16x32 tiles, chroma in 16x16, the pitch the coded width */
static void vdec_h264_lat_get_pic_info(struct vdec_h264_lat_inst *inst)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;

	ctx->picinfo.buf_w = ALIGN(ctx->picinfo.pic_w, 16);
	ctx->picinfo.buf_h = ALIGN(ctx->picinfo.pic_h, 32);
	ctx->picinfo.fb_sz[0] = ctx->picinfo.buf_w * ctx->picinfo.buf_h;
	ctx->picinfo.fb_sz[1] = ctx->picinfo.fb_sz[0] / 2;
	inst->realloc_mv_buf = true;
}

static int vdec_h264_lat_get_param(void *h_vdec, enum vdec_get_param_type type,
				   void *out)
{
	struct vdec_h264_lat_inst *inst = h_vdec;
	struct v4l2_rect *crop = out;

	switch (type) {
	case GET_PARAM_PIC_INFO:
		vdec_h264_lat_get_pic_info(inst);
		break;
	case GET_PARAM_CROP_INFO:
		crop->left = 0;
		crop->top = 0;
		crop->width = inst->ctx->picinfo.pic_w;
		crop->height = inst->ctx->picinfo.pic_h;
		break;
	case GET_PARAM_DPB_SIZE:
		*((unsigned int *)out) = H264_DPB_SIZE;
		break;
	default:
		mtk_vdec_err(inst->ctx, "invalid get parameter type=%d", type);
		return -EINVAL;
	}

	return 0;
}

static void vdec_h264_lat_hw_init(struct vdec_h264_lat_hw *hw,
				  struct mtk_vdec_hw_dev *dev, bool lat)
{
	void __iomem *vld = dev->reg_base[VDEC_HW_DEC];

	hw->lat = lat;
	hw->vld = vld;
	hw->vld_top = vld + 0x800;
	hw->misc = dev->reg_base[VDEC_HW_MISC];
	if (lat) {
		hw->avc = vld + 0x1000;
		hw->mv = vld + 0x2000;
		hw->pp = vld + 0x5000;
	} else {
		hw->mc = vld + 0x1000;
		hw->avc = vld + 0x2000;
		hw->mv = vld + 0x3000;
		hw->pp = vld + 0x4000;
	}
}

static int vdec_h264_lat_init(struct mtk_vcodec_dec_ctx *ctx)
{
	struct vdec_h264_lat_inst *inst;
	struct mtk_vdec_hw_dev *lat, *core;

	lat = mtk_vcodec_get_hw_dev(ctx->dev, MTK_VDEC_LAT0);
	core = mtk_vcodec_get_hw_dev(ctx->dev, MTK_VDEC_CORE);
	if (!lat || !core || !lat->reg_base[VDEC_HW_DEC] ||
	    !core->reg_base[VDEC_HW_DEC]) {
		mtk_vdec_err(ctx, "no decode registers on the lat or the core");
		return -ENODEV;
	}

	inst = kzalloc_obj(*inst);
	if (!inst)
		return -ENOMEM;

	inst->ctx = ctx;
	inst->sys = core->reg_base[VDEC_HW_SYS];
	vdec_h264_lat_hw_init(&inst->lat, lat, true);
	vdec_h264_lat_hw_init(&inst->core, core, false);
	ctx->drv_handle = inst;

	return 0;
}

static void vdec_h264_lat_deinit(void *h_vdec)
{
	struct vdec_h264_lat_inst *inst = h_vdec;
	int i;

	vdec_msg_queue_deinit(&inst->ctx->msg_queue, inst->ctx);
	for (i = 0; i < H264_MAX_MV_NUM; i++)
		if (inst->mv_buf[i].va)
			mtk_vcodec_mem_free(inst->ctx, &inst->mv_buf[i]);
	kfree(inst);
}

const struct vdec_common_if vdec_h264_slice_lat_if = {
	.init		= vdec_h264_lat_init,
	.decode		= vdec_h264_lat_decode,
	.get_param	= vdec_h264_lat_get_param,
	.deinit		= vdec_h264_lat_deinit,
};
