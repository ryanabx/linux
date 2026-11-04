// SPDX-License-Identifier: GPL-2.0
/*
 * HEVC stateless decoding for decoders without firmware (MT8695), where
 * the kernel programs the decoder's registers itself.
 *
 * Each picture runs once on the LAT, which parses the slice data of all
 * the picture's slice segments into the UBE ring, and once on the core,
 * which reconstructs the picture from it; the two are queued through
 * vdec_msg_queue as for H.264. The HEVC_VLD blocks sit in the LAT's second
 * register range and in the core's third.
 *
 * Frame-based V4L2 hands over no slice header, and the LAT starts on the
 * slice data where its bit reader stands, so the driver walks the first
 * slice segment header through it (HEVC_VLD word 38 reads ue(v), 39 se(v),
 * word n 1-32 shifts n bits). The hardware parses pred_weight_table() and
 * ref_pic_lists_modification() itself when told to at their place, and all
 * later slice segments of the picture on its own.
 *
 * 8-bit and 10-bit 4:2:0 output as MM21 / MS21 is not done yet: 8-bit
 * only for now, no scaling lists, no long-term references checked, no UFO
 * compression.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/moduleparam.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "../mtk_vcodec_dec.h"
#include "../mtk_vcodec_dec_hw.h"
#include "../../common/mtk_vcodec_intr.h"
#include "../vdec_drv_base.h"
#include "../vdec_drv_if.h"
#include "../vdec_msg_queue.h"

#define HEVC_DEC_TIMEOUT_MS		100
#define HEVC_FETCH_TIMEOUT_US		10000
#define HEVC_DPB_SIZE			16
#define HEVC_LIST_LEN			16
#define HEVC_ERR_MAP_SIZE		0x11000
#define HEVC_SLICE_HDR2_OFFSET		0x21c00
#define HEVC_MAX_MV_NUM			32

/*
 * DEBUG: 1 = UFO on, as the vendor, into a scratch frame in the vendor's
 * layout (1920 pitch); 2 = the same scratch frame without UFO. hevc_dbg_bus
 * overrides the core bus word when nonzero.
 */
static int hevc_dbg_ufo;
module_param(hevc_dbg_ufo, int, 0644);
static uint hevc_dbg_bus;
module_param(hevc_dbg_bus, uint, 0644);
/* DEBUG: 1 = the core decodes the bitstream itself, without the LAT's output */
static int hevc_dbg_single;
module_param(hevc_dbg_single, int, 0644);
#define HEVC_DBG_BS_SIZE		SZ_1M
/* DEBUG: offset, value pairs written to the core's PP and MC before its trigger */
static uint hevc_dbg_pp[16];
static int hevc_dbg_pp_n;
module_param_array(hevc_dbg_pp, uint, &hevc_dbg_pp_n, 0644);
static uint hevc_dbg_mc[16];
static int hevc_dbg_mc_n;
module_param_array(hevc_dbg_mc, uint, &hevc_dbg_mc_n, 0644);
/*
 * DEBUG: block, offset, value triples written last before the core's trigger;
 * blocks 0 vld, 1 vld_top, 2 mc, 3 mv, 4 pp, 5 misc, 6 hevc, 7 sys
 */
static uint hevc_dbg_poke[30];
static int hevc_dbg_poke_n;
module_param_array(hevc_dbg_poke, uint, &hevc_dbg_poke_n, 0644);
#define HEVC_DBG_FB_SIZE		0x302000
#define HEVC_DBG_C_OFF			0x1fe000
#define HEVC_DBG_YLEN_OFF		0x2fd000
#define HEVC_DBG_CLEN_OFF		0x2ff080

/* VLD and VLD_TOP, as for H.264 */
#define VLD_CMD				0x08c
#define  VLD_CMD_FETCH			0x100000
#define  VLD_CMD_SHIFTER_INIT		0x800000
#define VLD_RPTR			0x0b0
#define VLD_BS_START			0x0b4	/* >> 6 */
#define VLD_BS_END			0x0b8	/* >> 6 */
#define VLD_FETCH_STATUS		0x0e8
#define  VLD_FETCH_READY		BIT(0)
#define VLD_0EC				0x0ec
#define VLD_SRST			0x108
#define VLD_WPTR			0x110

#define VLD_TOP_CTB_SIZE1		0x068	/* rows - 1 << 16 | columns - 1 */
#define VLD_TOP_PIC_SIZE		0x070	/* height << 16 | width */
#define VLD_TOP_CTB_SIZE		0x094	/* rows << 16 | columns */
#define VLD_TOP_BIT_DEPTH		0x100
#define VLD_TOP_230			0x230
#define VLD_TOP_234			0x234
#define VLD_TOP_ERR_MAP			0x23c
#define VLD_TOP_SLICE_HDR		0x240

/* HEVC_VLD */
#define HEVC_SHIFT(n)			(4 * (n))	/* 0-32 */
#define HEVC_POS			0x084	/* bits 10:8: in the byte */
#define HEVC_088			0x088
#define HEVC_DECODE			0x090
#define HEVC_SUBPARSE			0x094
#define  HEVC_SUBPARSE_PWT		0x1
#define  HEVC_SUBPARSE_RPLM		0x10
#define HEVC_UE				0x098
#define HEVC_SE				0x09c
#define HEVC_SPS0			0x0a0
#define HEVC_SPS1			0x0a4
#define HEVC_SPS2			0x0a8
#define HEVC_PPS0			0x0ac
#define HEVC_PPS1			0x0b0
#define HEVC_TILES			0x0b4
#define HEVC_PPS3			0x0b8
#define HEVC_SLICE0			0x0bc
#define HEVC_SLICE1			0x0c0
#define HEVC_SLICE2			0x0c4
#define HEVC_SLICE3			0x0c8
#define HEVC_NPTC			0x0cc
#define HEVC_0D4			0x0d4
#define HEVC_0E0			0x0e0
#define HEVC_NUT			0x0f4
#define HEVC_COL_END(i)			(0x140 + 4 * (i))	/* 16 words */
#define HEVC_ROW_END(i)			(0x180 + 4 * (i))	/* 16 words */
#define HEVC_COL_END_HI			0x1d0
#define HEVC_ROW_END_HI			0x1ec

#define MV_REF_L0(i)			(4 * (i))	/* >> 4 */
#define MV_REF_L1(i)			(4 * (16 + (i)))
#define MV_CUR				0x20c	/* >> 4 */
#define MV_210				0x210
#define MV_LOW_DELAY			0x320
#define MV_324				0x324
#define MV_DESC_L0(i)			(4 * (206 + (i)))
#define MV_DESC_L1(i)			(4 * (222 + (i)))

#define PP_16				0x040
#define PP_SQT				0x630
#define PP_TAIL				0xb80
#define PP_UFO				0xc90

#define MC_WIDTH			0x208
#define MC_HEIGHT			0x20c
#define MC_OUT_ENABLE			0x220
#define MC_OUT_Y			0x224	/* >> 9 */
#define MC_OUT_C			0x228	/* >> 8 */
#define MC_PITCH			0x22c	/* in 16 bytes */
#define MC_8E				0x238
#define MC_94				0x250
#define MC_CTB_W			0x260	/* in 16 pixels - 1 */
#define MC_CTB_H			0x264
#define MC_REF_Y_L0(i)			(4 * (279 + (i)))
#define MC_REF_Y_L1(i)			(4 * (311 + (i)))
#define MC_PITCH2			0x980
#define MC_UFO				0xa60
#define MC_UFO_Y_LEN			0xae8
#define MC_UFO_C_LEN			0xaec
#define MC_UFO_Y_OFF			0xce4
#define MC_UFO_C_OFF			0xce8
#define MC_2D7				0xb5c
#define MC_REF_C_L0(i)			(4 * (760 + (i)))
#define MC_REF_C_L1(i)			(4 * (792 + (i)))

/* Misc */
#define MISC_IRQ_CFG			0x0a4
#define MISC_SYS_SEL			0x084
#define  MISC_SYS_SEL_HEVC		2
#define MISC_SPEC_CON			0x0c8
#define MISC_FUNC_CON			0x0cc
#define MISC_DCM_CON			0x0ec
#define MISC_SRST			0x0f4
#define MISC_178			0x178

/* The parent's range */
#define SYS_08				0x008
#define SOCUFO_03			0x80c
#define SOCUFO_14			0x838
#define SOCUFO_Y_LEN			0x87c
#define SOCUFO_C_LEN			0x880
#define SOCUFO_Y			0x884
#define SOCUFO_C			0x888
#define SOCUFO_CFG			0x88c
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

#define HEVC_SLICE_B			0
#define HEVC_SLICE_P			1
#define HEVC_SLICE_I			2

#define HEVC_NUT_BLA_W_LP		16
#define HEVC_NUT_IDR_W_RADL		19
#define HEVC_NUT_IDR_N_LP		20
#define HEVC_NUT_RSV_IRAP_23		23

/* What the walk through the first slice segment header found */
struct vdec_hevc_nofw_shdr {
	u32 nut;
	u32 slice_type;
	bool sao_luma, sao_chroma;
	u32 num_ref_idx[2];	/* active, minus 1 */
	bool mvd_l1_zero, cabac_init, temporal_mvp, col_from_l0;
	u32 col_ref_idx;
	u32 max_num_merge_cand;
	s32 qp_delta, cb_qp_offset, cr_qp_offset;
	bool deblocking_disabled;
	s32 beta_div2, tc_div2;
	bool lf_across_slices;
	u32 num_lt_sps, num_lt_pics;
};

/* What a picture's LAT run hands the core */
struct vdec_hevc_nofw_share {
	struct v4l2_ctrl_hevc_sps sps;
	struct v4l2_ctrl_hevc_pps pps;
	struct v4l2_ctrl_hevc_decode_params dec;
	struct vdec_hevc_nofw_shdr shdr;
	u32 slice0, slice2_early;	/* HEVC_SLICE0, early HEVC_SLICE2 */
	dma_addr_t err_map;
	u64 trans_start, trans_end;
};

/* One half's blocks; mc and pp are the core's only */
struct vdec_hevc_nofw_hw {
	void __iomem *vld, *vld_top, *hevc, *mv, *pp, *mc, *misc;
	bool lat;
};

struct vdec_hevc_nofw_inst {
	struct mtk_vcodec_dec_ctx *ctx;
	void __iomem *sys;
	struct vdec_hevc_nofw_hw lat, core;

	/* One MV buffer per capture buffer, indexed like them */
	struct mtk_vcodec_mem mv_buf[HEVC_MAX_MV_NUM];
	bool realloc_mv_buf;
	/* Larger error maps than the message queue's, one per LAT buffer */
	struct mtk_vcodec_mem err_map[NUM_BUFFER_COUNT];
	struct mtk_vcodec_mem dbg_fb;
	/* DEBUG: the bit reader's half, and the bitstream copy for hevc_dbg_single */
	struct vdec_hevc_nofw_hw *bsr;
	struct mtk_vcodec_mem dbg_bs;
	u32 dbg_bs_size;
};

static void vdec_hevc_nofw_soc_write(struct mtk_vcodec_dec_dev *dev, u32 off, u32 val)
{
	if (dev->vdecsoc_regmap)
		regmap_write(dev->vdecsoc_regmap, off, val);
}

static void vdec_hevc_nofw_rmw(void __iomem *reg, u32 clear, u32 set)
{
	writel((readl(reg) & ~clear) | set, reg);
}

static u32 vdec_hevc_nofw_ceil_log2(u32 v)
{
	return v > 1 ? order_base_2(v) : 0;
}

/* H265_HAL_ResetHW; the decoder and soc gates it writes are the clocks' */
static void vdec_hevc_nofw_reset(struct vdec_hevc_nofw_inst *inst,
				 struct vdec_hevc_nofw_hw *hw)
{
	struct mtk_vcodec_dec_dev *dev = inst->ctx->dev;
	u32 val;

	writel(0x10, hw->misc + MISC_IRQ_CFG);
	writel(0, hw->misc + MISC_IRQ_CFG);
	writel(1, inst->sys + SYS_08);
	if (dev->vdecsys_regmap)
		regmap_set_bits(dev->vdecsys_regmap, VDECSYS_DCM_CON, 0x111);
	if (hw->lat) {
		vdec_hevc_nofw_soc_write(dev, VDECSOC_LAT_BUS, 0);
		vdec_hevc_nofw_rmw(inst->sys + WDMA_RST, 0, 1);
		readl_poll_timeout(inst->sys + WDMA_RST_DONE, val, val & BIT(0),
				   1, 1000);
	} else {
		vdec_hevc_nofw_soc_write(dev, VDECSOC_CORE_BUS, hevc_dbg_bus ?:
					 hevc_dbg_ufo == 1 ? 0x40000000 : 0x40004);
	}
	writel(0, hw->misc + MISC_SRST);
	writel(0x101, hw->vld + VLD_SRST);
	writel(1, hw->misc + MISC_DCM_CON);
	writel(0, hw->misc + MISC_SPEC_CON);
	writel(0, hw->misc + MISC_FUNC_CON);
	writel(0, hw->misc + MISC_178);
	writel(MISC_SYS_SEL_HEVC, hw->misc + MISC_SYS_SEL);
	if (!hw->lat)
		vdec_hevc_nofw_soc_write(dev, VDECSOC_04, 2);
	writel(0, hw->vld + VLD_SRST);
}

/*
 * H265_HAL_Init{Lat,Core}BarrelShift: point a half's VLD at a stream, the
 * LAT at the picture's slices, the core at the LAT's output in the UBE
 * ring. [start, end) is the window, rptr where to begin, wptr the end.
 */
static int vdec_hevc_nofw_start_bitstream(struct vdec_hevc_nofw_inst *inst,
					  struct vdec_hevc_nofw_hw *hw,
					  u32 start, u32 end, u32 rptr, u32 wptr)
{
	u32 val;
	int ret;

	writel(1, hw->hevc + HEVC_POS);
	writel(0x10000000, hw->vld + VLD_0EC);
	writel(start >> 6, hw->vld + VLD_BS_START);
	writel(end >> 6, hw->vld + VLD_BS_END);
	writel(rptr, hw->vld + VLD_RPTR);
	writel(wptr, hw->vld + VLD_WPTR);
	writel(0x100, hw->vld + VLD_SRST);
	writel(0, hw->vld + VLD_SRST);
	writel(VLD_CMD_FETCH, hw->vld + VLD_CMD);

	/* Until the fetch is ready, reading the bit reader hangs the bus */
	ret = readl_poll_timeout(hw->vld + VLD_FETCH_STATUS, val,
				 val & VLD_FETCH_READY, 10, HEVC_FETCH_TIMEOUT_US);
	if (ret) {
		mtk_vdec_err(inst->ctx, "%s bitstream fetch timed out",
			     hw->lat ? "lat" : "core");
		return ret;
	}
	writel(VLD_CMD_SHIFTER_INIT, hw->vld + VLD_CMD);
	return 0;
}

/* The LAT's bit reader */
static u32 vdec_hevc_nofw_peek(struct vdec_hevc_nofw_inst *inst)
{
	return readl(inst->bsr->hevc + HEVC_SHIFT(0));
}

static u32 vdec_hevc_nofw_bits(struct vdec_hevc_nofw_inst *inst, unsigned int n)
{
	u32 val = vdec_hevc_nofw_peek(inst);

	if (!n)
		return 0;
	readl(inst->bsr->hevc + HEVC_SHIFT(n));
	return val >> (32 - n);
}

static u32 vdec_hevc_nofw_ue(struct vdec_hevc_nofw_inst *inst)
{
	return readl(inst->bsr->hevc + HEVC_UE);
}

static s32 vdec_hevc_nofw_se(struct vdec_hevc_nofw_inst *inst)
{
	return (s32)readl(inst->bsr->hevc + HEVC_SE);
}

static void vdec_hevc_nofw_skip(struct vdec_hevc_nofw_inst *inst, u32 bits)
{
	while (bits) {
		u32 n = min(bits, 32u);

		vdec_hevc_nofw_bits(inst, n);
		bits -= n;
	}
}

/* Start one of the hardware's own parses of the slice header */
static int vdec_hevc_nofw_subparse(struct vdec_hevc_nofw_inst *inst, u32 what)
{
	u32 val;

	writel(what, inst->bsr->hevc + HEVC_SUBPARSE);
	return readl_poll_timeout(inst->bsr->hevc + HEVC_SUBPARSE, val, !val, 1, 10000);
}

/* The start code and the NAL header of the picture's first slice */
static int vdec_hevc_nofw_find_slice(struct vdec_hevc_nofw_inst *inst, u32 *nut)
{
	int i;

	for (i = 0; i < 64; i++) {
		u32 val = vdec_hevc_nofw_peek(inst);

		if ((val >> 8) == 0x000001) {
			*nut = (val >> 1) & 0x3f;
			vdec_hevc_nofw_skip(inst, 24 + 16);
			return 0;
		}
		vdec_hevc_nofw_bits(inst, 8);
	}
	return -EINVAL;
}

static bool vdec_hevc_nofw_irap(u32 nut)
{
	return nut >= HEVC_NUT_BLA_W_LP && nut <= HEVC_NUT_RSV_IRAP_23;
}

static bool vdec_hevc_nofw_idr(u32 nut)
{
	return nut == HEVC_NUT_IDR_W_RADL || nut == HEVC_NUT_IDR_N_LP;
}

static u32 vdec_hevc_nofw_ctb_log2(const struct v4l2_ctrl_hevc_sps *sps)
{
	return sps->log2_min_luma_coding_block_size_minus3 + 3 +
	       sps->log2_diff_max_min_luma_coding_block_size;
}

static u32 vdec_hevc_nofw_ctbs(u32 n, const struct v4l2_ctrl_hevc_sps *sps)
{
	u32 log2 = vdec_hevc_nofw_ctb_log2(sps);

	return (n + (1 << log2) - 1) >> log2;
}

static u32 vdec_hevc_nofw_nptc(const struct v4l2_ctrl_hevc_decode_params *dec)
{
	return dec->num_poc_st_curr_before + dec->num_poc_st_curr_after +
	       dec->num_poc_lt_curr;
}

/* HEVC_VLD 51: NumPicTotalCurr, and its length in bits */
static u32 vdec_hevc_nofw_nptc_word(const struct v4l2_ctrl_hevc_decode_params *dec)
{
	u32 n = vdec_hevc_nofw_nptc(dec);

	return (n & 0xf) | vdec_hevc_nofw_ceil_log2(n) << 4;
}

/*
 * The first slice segment header (7.3.6.1), up to the slice data, as the
 * library reads it; writing the words the hardware's own parses need at
 * their place. The RPS parts are skipped by the sizes the decode
 * parameters give, the long-term counts read for HEVC_VLD 50.
 */
static int vdec_hevc_nofw_parse_shdr(struct vdec_hevc_nofw_inst *inst,
				     struct vdec_hevc_nofw_share *sh)
{
	const struct v4l2_ctrl_hevc_sps *sps = &sh->sps;
	const struct v4l2_ctrl_hevc_pps *pps = &sh->pps;
	const struct v4l2_ctrl_hevc_decode_params *dec = &sh->dec;
	struct vdec_hevc_nofw_shdr *s = &sh->shdr;
	struct vdec_hevc_nofw_hw *hw = inst->bsr;
	u32 wc = vdec_hevc_nofw_ctbs(sps->pic_width_in_luma_samples, sps);
	u32 hc = vdec_hevc_nofw_ctbs(sps->pic_height_in_luma_samples, sps);
	u32 poc_bits = sps->log2_max_pic_order_cnt_lsb_minus4 + 4;
	u32 st_idx_bits = vdec_hevc_nofw_ceil_log2(sps->num_short_term_ref_pic_sets);
	u32 lt_idx_bits = vdec_hevc_nofw_ceil_log2(sps->num_long_term_ref_pics_sps);
	bool slice_deblocking_override = false;
	u32 i, n;
	int ret;

	if (!vdec_hevc_nofw_bits(inst, 1))
		return -EINVAL;		/* not the picture's first slice segment */
	if (vdec_hevc_nofw_irap(s->nut))
		vdec_hevc_nofw_bits(inst, 1);	/* no_output_of_prior_pics_flag */
	vdec_hevc_nofw_ue(inst);		/* slice_pic_parameter_set_id */
	vdec_hevc_nofw_skip(inst, pps->num_extra_slice_header_bits);
	s->slice_type = vdec_hevc_nofw_ue(inst);
	if (s->slice_type > HEVC_SLICE_I)
		return -EINVAL;

	sh->slice0 = (vdec_hevc_nofw_idr(s->nut) ? BIT(31) : 0) |
		     s->slice_type << 28 | vdec_hevc_nofw_ceil_log2(wc * hc) << 20;
	writel(sh->slice0, hw->hevc + HEVC_SLICE0);

	if (pps->flags & V4L2_HEVC_PPS_FLAG_OUTPUT_FLAG_PRESENT)
		vdec_hevc_nofw_bits(inst, 1);	/* pic_output_flag */
	if (sps->flags & V4L2_HEVC_SPS_FLAG_SEPARATE_COLOUR_PLANE)
		vdec_hevc_nofw_bits(inst, 2);
	if (!vdec_hevc_nofw_idr(s->nut)) {
		vdec_hevc_nofw_bits(inst, poc_bits);	/* slice_pic_order_cnt_lsb */
		if (!vdec_hevc_nofw_bits(inst, 1))	/* short_term_ref_pic_set_sps_flag */
			vdec_hevc_nofw_skip(inst, dec->short_term_ref_pic_set_size);
		else
			vdec_hevc_nofw_bits(inst, st_idx_bits);
		if (sps->flags & V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT) {
			if (sps->num_long_term_ref_pics_sps)
				s->num_lt_sps = vdec_hevc_nofw_ue(inst);
			s->num_lt_pics = vdec_hevc_nofw_ue(inst);
			for (i = 0; i < s->num_lt_sps + s->num_lt_pics; i++) {
				if (i < s->num_lt_sps) {
					vdec_hevc_nofw_bits(inst, lt_idx_bits);
				} else {
					vdec_hevc_nofw_bits(inst, poc_bits);
					vdec_hevc_nofw_bits(inst, 1);
				}
				/* delta_poc_msb_present_flag, delta_poc_msb_cycle_lt */
				if (vdec_hevc_nofw_bits(inst, 1))
					vdec_hevc_nofw_ue(inst);
			}
		}
		if (sps->flags & V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED)
			s->temporal_mvp = vdec_hevc_nofw_bits(inst, 1);
	}
	if (sps->flags & V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET) {
		s->sao_luma = vdec_hevc_nofw_bits(inst, 1);
		if (sps->chroma_format_idc)
			s->sao_chroma = vdec_hevc_nofw_bits(inst, 1);
	}

	s->num_ref_idx[0] = 0xf;
	s->num_ref_idx[1] = 0xf;
	s->col_from_l0 = true;
	if (s->slice_type != HEVC_SLICE_I) {
		bool b = s->slice_type == HEVC_SLICE_B;

		s->num_ref_idx[0] = pps->num_ref_idx_l0_default_active_minus1;
		if (b)
			s->num_ref_idx[1] = pps->num_ref_idx_l1_default_active_minus1;
		if (vdec_hevc_nofw_bits(inst, 1)) {	/* num_ref_idx_active_override_flag */
			s->num_ref_idx[0] = vdec_hevc_nofw_ue(inst);
			if (b)
				s->num_ref_idx[1] = vdec_hevc_nofw_ue(inst);
		}
		sh->slice2_early = s->num_ref_idx[0] | s->num_ref_idx[1] << 4;
		writel(sh->slice2_early, hw->hevc + HEVC_SLICE2);

		if ((pps->flags & V4L2_HEVC_PPS_FLAG_LISTS_MODIFICATION_PRESENT) &&
		    vdec_hevc_nofw_nptc(dec) > 1) {
			writel(vdec_hevc_nofw_nptc_word(dec), hw->hevc + HEVC_NPTC);
			ret = vdec_hevc_nofw_subparse(inst, HEVC_SUBPARSE_RPLM);
			if (ret)
				return ret;
		}
		if (b)
			s->mvd_l1_zero = vdec_hevc_nofw_bits(inst, 1);
		if (pps->flags & V4L2_HEVC_PPS_FLAG_CABAC_INIT_PRESENT)
			s->cabac_init = vdec_hevc_nofw_bits(inst, 1);
		if (s->temporal_mvp) {
			if (b)
				s->col_from_l0 = vdec_hevc_nofw_bits(inst, 1);
			if ((s->col_from_l0 && s->num_ref_idx[0] > 0) ||
			    (!s->col_from_l0 && s->num_ref_idx[1] > 0 && b))
				s->col_ref_idx = vdec_hevc_nofw_ue(inst);
		}
		if (((pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED) && !b) ||
		    ((pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED) && b)) {
			ret = vdec_hevc_nofw_subparse(inst, HEVC_SUBPARSE_PWT);
			if (ret)
				return ret;
		}
		s->max_num_merge_cand = 5 - vdec_hevc_nofw_ue(inst);
	}
	s->qp_delta = vdec_hevc_nofw_se(inst);
	if (pps->flags & V4L2_HEVC_PPS_FLAG_PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT) {
		s->cb_qp_offset = vdec_hevc_nofw_se(inst);
		s->cr_qp_offset = vdec_hevc_nofw_se(inst);
	}

	s->deblocking_disabled = !!(pps->flags & V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER);
	s->beta_div2 = pps->pps_beta_offset_div2;
	s->tc_div2 = pps->pps_tc_offset_div2;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED)
		slice_deblocking_override = vdec_hevc_nofw_bits(inst, 1);
	if (slice_deblocking_override) {
		s->deblocking_disabled = vdec_hevc_nofw_bits(inst, 1);
		if (!s->deblocking_disabled) {
			s->beta_div2 = vdec_hevc_nofw_se(inst);
			s->tc_div2 = vdec_hevc_nofw_se(inst);
		}
	}
	s->lf_across_slices =
		!!(pps->flags & V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED);
	if (s->lf_across_slices &&
	    (s->sao_luma || s->sao_chroma || !s->deblocking_disabled))
		s->lf_across_slices = vdec_hevc_nofw_bits(inst, 1);

	if (pps->flags & (V4L2_HEVC_PPS_FLAG_TILES_ENABLED |
			  V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED)) {
		n = vdec_hevc_nofw_ue(inst);	/* num_entry_point_offsets */
		if (n) {
			u32 len = vdec_hevc_nofw_ue(inst) + 1;

			for (i = 0; i < n; i++)
				vdec_hevc_nofw_skip(inst, len);
		}
	}
	if (pps->flags & V4L2_HEVC_PPS_FLAG_SLICE_SEGMENT_HEADER_EXTENSION_PRESENT)
		vdec_hevc_nofw_skip(inst, 8 * vdec_hevc_nofw_ue(inst));

	/* byte_alignment(): the one, then zeros to the byte's end */
	n = (readl(hw->hevc + HEVC_POS) >> 8) & 7;
	vdec_hevc_nofw_bits(inst, 8 - n);
	return 0;
}

/* H265_HAL_SetSPSHEVLD, SetPPSHEVLD, SetSHDRHEVLD, SetTilesInfo */
static void vdec_hevc_nofw_set_pic_words(struct vdec_hevc_nofw_hw *hw,
					 const struct vdec_hevc_nofw_share *sh)
{
	const struct v4l2_ctrl_hevc_sps *sps = &sh->sps;
	const struct v4l2_ctrl_hevc_pps *pps = &sh->pps;
	const struct v4l2_ctrl_hevc_decode_params *dec = &sh->dec;
	const struct vdec_hevc_nofw_shdr *s = &sh->shdr;
	u32 w = sps->pic_width_in_luma_samples, h = sps->pic_height_in_luma_samples;
	u32 wc = vdec_hevc_nofw_ctbs(w, sps), hc = vdec_hevc_nofw_ctbs(h, sps);
	u32 ctb_log2 = vdec_hevc_nofw_ctb_log2(sps);
	u32 min_cb = sps->log2_min_luma_coding_block_size_minus3 + 3;
	u32 min_tb = sps->log2_min_luma_transform_block_size_minus2 + 2;
	bool pcm = sps->flags & V4L2_HEVC_SPS_FLAG_PCM_ENABLED;
	bool hbd = sps->bit_depth_luma_minus8 || sps->bit_depth_chroma_minus8;
	u32 qp_mask = hbd ? 0x7f : 0x3f;
	u32 cols[21], rows[23], ncols, nrows, i, val, ref_nd = 0;
	u32 col_end[17] = { }, row_end[17] = { };

	/* The slice */
	val = (s->sao_luma ? BIT(5) : 0) | (s->sao_chroma ? BIT(6) : 0) |
	      (s->cabac_init ? BIT(7) : 0) |
	      ((26 + pps->init_qp_minus26 + s->qp_delta) & qp_mask) << 8 |
	      ((pps->pps_cb_qp_offset + s->cb_qp_offset) & 0x1f) << 16 |
	      ((pps->pps_cr_qp_offset + s->cr_qp_offset) & 0x1f) << 24;
	writel(val, hw->hevc + HEVC_SLICE1);
	val = (s->num_ref_idx[0] & 0xf) | (s->num_ref_idx[1] & 0xf) << 4 |
	      s->col_ref_idx << 8 | s->max_num_merge_cand << 12 |
	      (s->beta_div2 & 0x1f) << 16 | (s->tc_div2 & 0x1f) << 21 |
	      (s->temporal_mvp ? BIT(26) : 0) | (s->col_from_l0 ? BIT(27) : 0) |
	      (s->mvd_l1_zero ? BIT(28) : 0) | (s->lf_across_slices ? BIT(29) : 0) |
	      (s->deblocking_disabled ? BIT(30) : 0);
	writel(val, hw->hevc + HEVC_SLICE2);
	/*
	 * Bits 13:8 are what the library had from the last explicitly coded
	 * RPS; what the hardware does with them is unknown
	 */
	if (dec->num_delta_pocs_of_ref_rps_idx)
		ref_nd = dec->num_delta_pocs_of_ref_rps_idx;
	writel((ref_nd & 0x1f) | ((s->num_lt_sps + s->num_lt_pics) & 0x1f) << 16 |
	       (s->num_lt_sps & 0x1f) << 24, hw->hevc + HEVC_SLICE3);
	writel(vdec_hevc_nofw_nptc_word(dec), hw->hevc + HEVC_NPTC);
	writel(0xffff0015 | s->nut << 8, hw->hevc + HEVC_NUT);

	/* The SPS */
	writel((hc - 1) << 16 | (wc - 1), hw->vld_top + VLD_TOP_CTB_SIZE1);
	writel(h << 16 | w, hw->vld_top + VLD_TOP_PIC_SIZE);
	writel(hc << 16 | wc, hw->vld_top + VLD_TOP_CTB_SIZE);
	if (sps->bit_depth_luma_minus8 == sps->bit_depth_chroma_minus8 &&
	    (sps->bit_depth_luma_minus8 == 2 || sps->bit_depth_luma_minus8 == 4))
		writel(0x22, hw->vld_top + VLD_TOP_BIT_DEPTH);
	val = sps->chroma_format_idc |
	      (pcm ? BIT(2) : 0) |
	      ((pps->flags & V4L2_HEVC_PPS_FLAG_LISTS_MODIFICATION_PRESENT) ? BIT(3) : 0) |
	      ((sps->flags & V4L2_HEVC_SPS_FLAG_AMP_ENABLED) ? BIT(4) : 0) |
	      ((sps->flags & V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET) ? BIT(5) : 0) |
	      ((sps->flags & V4L2_HEVC_SPS_FLAG_PCM_LOOP_FILTER_DISABLED) ? BIT(6) : 0) |
	      ((sps->flags & V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED) ? BIT(7) : 0) |
	      (pcm ? sps->pcm_sample_bit_depth_luma_minus1 : 7) << 8 |
	      (pcm ? sps->pcm_sample_bit_depth_chroma_minus1 : 7) << 12 |
	      sps->num_long_term_ref_pics_sps << 16 |
	      ((sps->flags & V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT) ? BIT(22) : 0) |
	      sps->num_short_term_ref_pic_sets << 24;
	writel(val, hw->hevc + HEVC_SPS0);
	val = (sps->log2_max_pic_order_cnt_lsb_minus4 + 4) |
	      max(1u, vdec_hevc_nofw_ceil_log2(sps->num_short_term_ref_pic_sets)) << 8 |
	      max(1u, vdec_hevc_nofw_ceil_log2(sps->num_long_term_ref_pics_sps)) << 12 |
	      ((sps->flags & V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED) ? BIT(16) : 0);
	writel(val, hw->hevc + HEVC_SPS1);
	val = min_cb | ctb_log2 << 4 | min_tb << 8 |
	      (min_tb + sps->log2_diff_max_min_luma_transform_block_size) << 12 |
	      (pcm ? sps->log2_min_pcm_luma_coding_block_size_minus3 + 3 : 7) << 16 |
	      (pcm ? sps->log2_min_pcm_luma_coding_block_size_minus3 + 3 +
		     sps->log2_diff_max_min_pcm_luma_coding_block_size : 5) << 20 |
	      sps->max_transform_hierarchy_depth_inter << 24 |
	      sps->max_transform_hierarchy_depth_intra << 28;
	writel(val, hw->hevc + HEVC_SPS2);

	/* The PPS */
	val = ((pps->flags & V4L2_HEVC_PPS_FLAG_SIGN_DATA_HIDING_ENABLED) ? BIT(0) : 0) |
	      ((pps->flags & V4L2_HEVC_PPS_FLAG_CABAC_INIT_PRESENT) ? BIT(1) : 0) |
	      ((pps->flags & V4L2_HEVC_PPS_FLAG_CONSTRAINED_INTRA_PRED) ? BIT(2) : 0) |
	      ((pps->flags & V4L2_HEVC_PPS_FLAG_TRANSFORM_SKIP_ENABLED) ? BIT(3) : 0) |
	      ((pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED) ? BIT(4) : 0) |
	      ((pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED) ? BIT(5) : 0) |
	      ((pps->flags & V4L2_HEVC_PPS_FLAG_OUTPUT_FLAG_PRESENT) ? BIT(6) : 0) |
	      ((pps->flags & V4L2_HEVC_PPS_FLAG_TRANSQUANT_BYPASS_ENABLED) ? BIT(7) : 0) |
	      ((pps->flags & V4L2_HEVC_PPS_FLAG_DEPENDENT_SLICE_SEGMENT_ENABLED) ? BIT(8) : 0) |
	      ((pps->flags & V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED) ? BIT(9) : 0) |
	      ((pps->flags & V4L2_HEVC_PPS_FLAG_SLICE_SEGMENT_HEADER_EXTENSION_PRESENT) ?
	       BIT(11) : 0) |
	      pps->num_ref_idx_l0_default_active_minus1 << 16 |
	      pps->num_ref_idx_l1_default_active_minus1 << 20 |
	      (pps->log2_parallel_merge_level_minus2 + 2) << 24 |
	      pps->num_extra_slice_header_bits << 28;
	writel(val, hw->hevc + HEVC_PPS0);
	val = ((pps->flags & V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED) ? BIT(0) : 0) |
	      ((pps->flags & V4L2_HEVC_PPS_FLAG_PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT) ?
	       BIT(1) : 0) |
	      (ctb_log2 - pps->diff_cu_qp_delta_depth) << 4 |
	      ((26 + pps->init_qp_minus26) & qp_mask) << 8 |
	      (pps->pps_cb_qp_offset & 0x1f) << 16 |
	      (pps->pps_cr_qp_offset & 0x1f) << 24;
	writel(val, hw->hevc + HEVC_PPS1);

	ncols = 1;
	nrows = 1;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED) {
		ncols = pps->num_tile_columns_minus1 + 1;
		nrows = pps->num_tile_rows_minus1 + 1;
	}
	val = (ncols - 1) << 8 | (nrows - 1) << 16;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED) {
		val |= BIT(0);
		if (pps->flags & V4L2_HEVC_PPS_FLAG_LOOP_FILTER_ACROSS_TILES_ENABLED)
			val |= BIT(24);
	} else {
		val |= BIT(24);
	}
	writel(val, hw->hevc + HEVC_TILES);
	val = ((pps->flags & V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED) ?
	       BIT(0) : 0) |
	      ((pps->flags & V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT) ? BIT(1) : 0) |
	      ((pps->flags & V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED) ?
	       BIT(2) : 0) |
	      ((pps->flags & V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER) ? BIT(3) : 0) |
	      (pps->pps_beta_offset_div2 & 0x1f) << 4 |
	      (pps->pps_tc_offset_div2 & 0x1f) << 9;
	writel(val, hw->hevc + HEVC_PPS3);

	/* Tile boundaries (6.5.1): each column's and row's last CTB */
	for (i = 0; i < ncols; i++) {
		if (pps->flags & V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING)
			cols[i] = ((i + 1) * wc) / ncols - (i * wc) / ncols;
		else
			cols[i] = i < ncols - 1 ? pps->column_width_minus1[i] + 1 : 0;
	}
	if (!(pps->flags & V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING)) {
		cols[ncols - 1] = wc;
		for (i = 0; i < ncols - 1; i++)
			cols[ncols - 1] -= cols[i];
	}
	for (i = 0; i < nrows; i++) {
		if (pps->flags & V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING)
			rows[i] = ((i + 1) * hc) / nrows - (i * hc) / nrows;
		else
			rows[i] = i < nrows - 1 ? pps->row_height_minus1[i] + 1 : 0;
	}
	if (!(pps->flags & V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING)) {
		rows[nrows - 1] = hc;
		for (i = 0; i < nrows - 1; i++)
			rows[nrows - 1] -= rows[i];
	}
	for (i = 0, val = 0; i < ncols; i++) {
		val += cols[i];
		col_end[min((val - 1) >> 5, 16u)] |= BIT((val - 1) & 31);
	}
	for (i = 0, val = 0; i < nrows; i++) {
		val += rows[i];
		row_end[min((val - 1) >> 5, 16u)] |= BIT((val - 1) & 31);
	}
	for (i = 0; i < 16; i++)
		writel(col_end[i], hw->hevc + HEVC_COL_END(i));
	writel(col_end[16], hw->hevc + HEVC_COL_END_HI);
	for (i = 0; i < 16; i++)
		writel(row_end[i], hw->hevc + HEVC_ROW_END(i));
	writel(row_end[16], hw->hevc + HEVC_ROW_END_HI);
}

/* The LAT's run: the slice header, then all the picture's slice data */
/* DEBUG: the vendor's last value of each LAT register for POC_A picture 0 */
static const struct { u8 blk; u16 off; u32 val; } hevc_dbg_lat_regs[] = {
	{ 3, 0xf4, 0x0 },
	{ 3, 0xec, 0x1 },
	{ 3, 0xc8, 0x0 },
	{ 3, 0xcc, 0x0 },
	{ 3, 0x178, 0x0 },
	{ 3, 0x84, 0x2 },
	{ 4, 0x84, 0x1 },
	{ 0, 0xec, 0x10000000 },
	{ 0, 0xb4, 0x798000 },
	{ 0, 0xb8, 0x798112 },
	{ 0, 0xb0, 0x1e600046 },
	{ 0, 0x110, 0x1e604480 },
	{ 5, 0x3c, 0x1dfe0000 },
	{ 5, 0x1c, 0x3 },
	{ 5, 0x50, 0x22000000 },
	{ 5, 0x44, 0x22400000 },
	{ 5, 0x40, 0x22000000 },
	{ 5, 0x34, 0x22000000 },
	{ 5, 0x80, 0x1e5c0000 },
	{ 5, 0x84, 0x1e5e1c00 },
	{ 1, 0x230, 0x1 },
	{ 4, 0xbc, 0xa0500000 },
	{ 4, 0xe0, 0xfffefffb },
	{ 4, 0xd4, 0x4011d01 },
	{ 4, 0xc0, 0x1a20 },
	{ 4, 0xc4, 0x80000ff },
	{ 4, 0xc8, 0x0 },
	{ 4, 0xcc, 0x0 },
	{ 4, 0xf4, 0xffff1315 },
	{ 1, 0x68, 0x30006 },
	{ 1, 0x70, 0xf001a0 },
	{ 1, 0x94, 0x40007 },
	{ 4, 0xa0, 0x7721 },
	{ 4, 0xa4, 0x1104 },
	{ 4, 0xa8, 0x575263 },
	{ 4, 0xac, 0x2000000 },
	{ 4, 0xb0, 0x1a60 },
	{ 4, 0xb4, 0x1000000 },
	{ 4, 0xb8, 0x0 },
	{ 4, 0x140, 0x40 },
	{ 4, 0x144, 0x0 },
	{ 4, 0x148, 0x0 },
	{ 4, 0x14c, 0x0 },
	{ 4, 0x150, 0x0 },
	{ 4, 0x154, 0x0 },
	{ 4, 0x158, 0x0 },
	{ 4, 0x15c, 0x0 },
	{ 4, 0x160, 0x0 },
	{ 4, 0x164, 0x0 },
	{ 4, 0x168, 0x0 },
	{ 4, 0x16c, 0x0 },
	{ 4, 0x170, 0x0 },
	{ 4, 0x174, 0x0 },
	{ 4, 0x178, 0x0 },
	{ 4, 0x17c, 0x0 },
	{ 4, 0x1d0, 0x0 },
	{ 4, 0x180, 0x8 },
	{ 4, 0x184, 0x0 },
	{ 4, 0x188, 0x0 },
	{ 4, 0x18c, 0x0 },
	{ 4, 0x190, 0x0 },
	{ 4, 0x194, 0x0 },
	{ 4, 0x198, 0x0 },
	{ 4, 0x19c, 0x0 },
	{ 4, 0x1a0, 0x0 },
	{ 4, 0x1a4, 0x0 },
	{ 4, 0x1a8, 0x0 },
	{ 4, 0x1ac, 0x0 },
	{ 4, 0x1b0, 0x0 },
	{ 4, 0x1b4, 0x0 },
	{ 4, 0x1b8, 0x0 },
	{ 4, 0x1bc, 0x0 },
	{ 4, 0x1ec, 0x0 },
};

static void vdec_hevc_nofw_dbg_lat_dump(struct vdec_hevc_nofw_inst *inst)
{
	static const char * const name[] = { "vld", "vld_top", "mv", "misc", "hevc", "wdma" };
	struct vdec_hevc_nofw_hw *hw = &inst->lat;
	void __iomem *base[] = { hw->vld, hw->vld_top, hw->mv, hw->misc, hw->hevc,
				 inst->sys + LAT_WDMA };
	int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(hevc_dbg_lat_regs); i++) {
		u32 v = readl(base[hevc_dbg_lat_regs[i].blk] + hevc_dbg_lat_regs[i].off);

		if (v == hevc_dbg_lat_regs[i].val)
			continue;
		n++;
		dev_info(&inst->ctx->dev->plat_dev->dev, "LDUMP %s+0x%03x vendor 0x%08x ours 0x%08x\n",
			 name[hevc_dbg_lat_regs[i].blk], hevc_dbg_lat_regs[i].off,
			 hevc_dbg_lat_regs[i].val, v);
	}
	dev_info(&inst->ctx->dev->plat_dev->dev, "LDUMP %d of %zu differ\n", n,
		 ARRAY_SIZE(hevc_dbg_lat_regs));
}

/* DEBUG: how much of a buffer is nonzero */
static void vdec_hevc_nofw_dbg_extent(struct vdec_hevc_nofw_inst *inst, const char *what,
				      const void *va, size_t size)
{
	const u32 *w = va;
	size_t i, n = 0, first = size, last = 0;

	for (i = 0; i < size / 4; i++) {
		if (!w[i])
			continue;
		n++;
		first = min(first, i * 4);
		last = i * 4;
	}
	dev_info(&inst->ctx->dev->plat_dev->dev,
		 "EXTENT %s: %zu nonzero words, 0x%zx..0x%zx of 0x%zx\n",
		 what, n, first, last, size);
}

static int vdec_hevc_nofw_lat_run(struct vdec_hevc_nofw_inst *inst,
				  struct vdec_hevc_nofw_share *sh,
				  struct mtk_vcodec_mem *bs, size_t capacity,
				  struct vdec_lat_buf *lat_buf)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vdec_msg_queue *q = &ctx->msg_queue;
	struct vdec_hevc_nofw_hw *hw = &inst->lat;
	u32 start = bs->dma_addr;
	u32 end = min_t(u32, start + ALIGN(bs->size, 128) + 512, start + capacity);
	u32 ube = q->wdma_addr.dma_addr;
	int ret;

	if (hevc_dbg_single && !inst->dbg_bs.va) {
		inst->dbg_bs.size = HEVC_DBG_BS_SIZE;
		ret = mtk_vcodec_mem_alloc(ctx, &inst->dbg_bs);
		if (ret)
			return ret;
	}
	if (hevc_dbg_single && bs->size + 1024 <= HEVC_DBG_BS_SIZE) {
		memcpy(inst->dbg_bs.va, bs->va, bs->size);
		memset(inst->dbg_bs.va + bs->size, 0, 1024);
		inst->dbg_bs_size = bs->size;
	}
	vdec_hevc_nofw_reset(inst, hw);
	ret = vdec_hevc_nofw_start_bitstream(inst, hw, start, end, start, end);
	if (ret)
		return ret;

	/* H265_HAL_SetLATDecAddr: the UBE ring after the core's data */
	writel(sh->err_map, inst->sys + WDMA_ERR_MAP);
	vdec_hevc_nofw_rmw(inst->sys + WDMA_07, 0, 0x3);
	writel(ube, inst->sys + WDMA_UBE_START);
	writel(ube + q->wdma_addr.size, inst->sys + WDMA_UBE_END);
	writel(q->wdma_wptr_addr, inst->sys + WDMA_UBE_WPTR);
	writel(q->wdma_rptr_addr, inst->sys + WDMA_UBE_RPTR);
	writel(lat_buf->slice_bc_addr.dma_addr, inst->sys + WDMA_SLICE_START);
	writel(lat_buf->slice_bc_addr.dma_addr + HEVC_SLICE_HDR2_OFFSET,
	       inst->sys + WDMA_SLICE_END);

	ret = vdec_hevc_nofw_find_slice(inst, &sh->shdr.nut);
	if (ret) {
		mtk_vdec_err(ctx, "no slice start code");
		return ret;
	}
	vdec_hevc_nofw_rmw(hw->vld_top + VLD_TOP_230, 0, 0x1);
	ret = vdec_hevc_nofw_parse_shdr(inst, sh);
	if (ret) {
		mtk_vdec_err(ctx, "slice header: %d", ret);
		return ret;
	}

	writel(0xfffefffb, hw->hevc + HEVC_0E0);
	writel(0x04011d01, hw->hevc + HEVC_0D4);
	vdec_hevc_nofw_set_pic_words(hw, sh);
	if (!ctx->decoded_frame_cnt) {
		int i;

		vdec_hevc_nofw_dbg_lat_dump(inst);
		for (i = 0; i < NUM_BUFFER_COUNT; i++)
			if (inst->err_map[i].dma_addr == sh->err_map)
				vdec_hevc_nofw_dbg_extent(inst, "errmap before", inst->err_map[i].va,
							  inst->err_map[i].size);
		vdec_hevc_nofw_dbg_extent(inst, "slicehdr before", lat_buf->slice_bc_addr.va,
					  lat_buf->slice_bc_addr.size);
	}
	writel(1, hw->hevc + HEVC_DECODE);

	ret = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					   HEVC_DEC_TIMEOUT_MS, MTK_VDEC_LAT0);
	{
		u32 w9 = readl(inst->sys + LAT_WDMA + 0x24);
		u32 off0 = readl(inst->sys + WDMA_UBE_OFFSET);

		udelay(1000);
		dev_info(&ctx->dev->plat_dev->dev,
			 "DEBUG lat %d: wdma9 0x%08x then 0x%08x, offset 0x%x then 0x%x\n",
			 ctx->decoded_frame_cnt, w9, readl(inst->sys + LAT_WDMA + 0x24), off0,
			 readl(inst->sys + WDMA_UBE_OFFSET));
	}
	vdec_hevc_nofw_soc_write(ctx->dev, VDECSOC_LAT_BUS, 0);
	writel(0, hw->misc + MISC_SRST);
	if (ret) {
		mtk_vdec_err(ctx, "lat timeout: picture %d", ctx->decoded_frame_cnt);
		return -ETIMEDOUT;
	}

	/* The LAT's output ends at its new write offset in the ring */
	sh->trans_start = q->wdma_wptr_addr;
	sh->trans_end = ube + readl(inst->sys + WDMA_UBE_OFFSET);
	vdec_msg_queue_update_ube_wptr(q, sh->trans_end);
	if (ctx->decoded_frame_cnt < 2) {
		u8 *u = q->wdma_addr.va + (sh->trans_end - ube);
		int i;

		print_hex_dump(KERN_INFO, "ube-start ", DUMP_PREFIX_OFFSET, 16, 4,
			       q->wdma_addr.va + (sh->trans_start - ube), 64, false);
		print_hex_dump(KERN_INFO, "ube-end ", DUMP_PREFIX_OFFSET, 16, 4, u - 48, 96, false);
		for (i = 0; i < NUM_BUFFER_COUNT; i++)
			if (inst->err_map[i].dma_addr == sh->err_map) {
				dev_info(&ctx->dev->plat_dev->dev, "DEBUG errmap %d of lat_buf %p\n",
					 i, lat_buf);
				vdec_hevc_nofw_dbg_extent(inst, "errmap", inst->err_map[i].va,
							  inst->err_map[i].size);
			}
		vdec_hevc_nofw_dbg_extent(inst, "slicehdr", lat_buf->slice_bc_addr.va,
					  HEVC_SLICE_HDR2_OFFSET);
		vdec_hevc_nofw_dbg_extent(inst, "slicehdr2",
					  lat_buf->slice_bc_addr.va + HEVC_SLICE_HDR2_OFFSET,
					  lat_buf->slice_bc_addr.size - HEVC_SLICE_HDR2_OFFSET);
		vdec_hevc_nofw_dbg_extent(inst, "ube", q->wdma_addr.va, 0x20000);
		print_hex_dump(KERN_INFO, "slicehdr2 ", DUMP_PREFIX_OFFSET, 16, 4,
			       lat_buf->slice_bc_addr.va + HEVC_SLICE_HDR2_OFFSET, 64, false);
	}
	dev_info(&ctx->dev->plat_dev->dev,
		 "DEBUG lat %d: nut %u type %u last ctb 0x%08x ube 0x%llx..0x%llx err 0x%08x 0x%08x\n",
		 ctx->decoded_frame_cnt, sh->shdr.nut, sh->shdr.slice_type,
		 readl(hw->hevc + 0x12c), sh->trans_start, sh->trans_end,
		 readl(hw->hevc + 0x0e8), readl(hw->hevc + 0x0e4));
	return 0;
}

static void vdec_hevc_nofw_buf_addr(struct mtk_vcodec_dec_ctx *ctx,
				    struct vb2_buffer *vb, dma_addr_t *y, dma_addr_t *c)
{
	struct vdec_hevc_nofw_inst *inst = ctx->drv_handle;

	if (hevc_dbg_ufo && inst->dbg_fb.va) {
		*y = inst->dbg_fb.dma_addr;
		*c = *y + HEVC_DBG_C_OFF;
		return;
	}
	*y = vb2_dma_contig_plane_dma_addr(vb, 0);
	if (ctx->q_data[MTK_Q_DATA_DST].fmt->num_planes == 2)
		*c = vb2_dma_contig_plane_dma_addr(vb, 1);
	else
		*c = *y + ctx->picinfo.fb_sz[0];
}

/* A reference, by its DPB index: its frame buffer, or the picture itself */
struct vdec_hevc_nofw_ref {
	dma_addr_t y, c, mv;
	u32 fb;
	s32 poc;
};

static void vdec_hevc_nofw_get_ref(struct vdec_hevc_nofw_inst *inst,
				   const struct vdec_hevc_nofw_share *sh, u8 idx,
				   struct vb2_buffer *dst, struct vdec_hevc_nofw_ref *r)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vb2_buffer *vb = NULL;

	if (idx < V4L2_HEVC_DPB_ENTRIES_NUM_MAX)
		vb = vb2_find_buffer(v4l2_m2m_get_dst_vq(ctx->m2m_ctx),
				     sh->dec.dpb[idx].timestamp);
	if (vb && vb->index >= HEVC_MAX_MV_NUM)
		vb = NULL;
	if (!vb) {
		mtk_vdec_err(ctx, "picture %d: no reference %u", ctx->decoded_frame_cnt, idx);
		vb = dst;
		r->poc = sh->dec.pic_order_cnt_val;
	} else {
		r->poc = sh->dec.dpb[idx].pic_order_cnt_val;
	}
	vdec_hevc_nofw_buf_addr(ctx, vb, &r->y, &r->c);
	r->mv = inst->mv_buf[vb->index].dma_addr;
	r->fb = vb->index;
}

/*
 * The reference lists (8.3.4): 16 entries each, cycling through the
 * initial list, as the library programs them; the hardware applies
 * ref_pic_lists_modification() itself.
 */
static void vdec_hevc_nofw_set_lists(struct vdec_hevc_nofw_inst *inst,
				     const struct vdec_hevc_nofw_share *sh,
				     struct vb2_buffer *dst, dma_addr_t mv)
{
	const struct v4l2_ctrl_hevc_decode_params *dec = &sh->dec;
	const struct vdec_hevc_nofw_shdr *s = &sh->shdr;
	struct vdec_hevc_nofw_hw *hw = &inst->core;
	struct vdec_hevc_nofw_ref l[2][HEVC_LIST_LEN], conceal;
	u8 temp[2][3 * V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	u32 nptc = vdec_hevc_nofw_nptc(dec), n = 0, i, x;
	s32 cur = dec->pic_order_cnt_val, best = S32_MAX;
	bool low_delay = true;
	dma_addr_t y, c;

	vdec_hevc_nofw_buf_addr(inst->ctx, dst, &y, &c);

	/* For I pictures, and anything left unset: the nearest picture in the RPS */
	conceal.y = y;
	conceal.c = c;
	conceal.mv = mv;
	for (i = 0; i < dec->num_active_dpb_entries; i++) {
		s32 d = abs(dec->dpb[i].pic_order_cnt_val - cur);
		struct vdec_hevc_nofw_ref r;

		vdec_hevc_nofw_get_ref(inst, sh, i, dst, &r);
		if (d < best || (d == best && r.fb < conceal.fb)) {
			best = d;
			conceal = r;
		}
	}
	for (i = 0; i < HEVC_LIST_LEN; i++) {
		writel(conceal.y, hw->mc + MC_REF_Y_L1(i));
		writel(conceal.y, hw->mc + MC_REF_Y_L0(i));
		writel(conceal.c, hw->mc + MC_REF_C_L0(i));
		writel(conceal.c, hw->mc + MC_REF_C_L1(i));
		writel(conceal.mv >> 4, hw->mv + MV_REF_L0(i));
		writel(conceal.mv >> 4, hw->mv + MV_REF_L1(i));
	}
	writel(0xfffefffb, hw->hevc + HEVC_0E0);
	writel(0x04011d01, hw->hevc + HEVC_0D4);
	writel(0xf011, hw->pp + PP_16);
	writel(0, hw->mv + MV_210);

	if (s->slice_type == HEVC_SLICE_I || !nptc) {
		writel(0, hw->mv + MV_LOW_DELAY);
		writel(mv >> 4, hw->mv + MV_CUR);
		writel(1, hw->mc + MC_2D7);
		return;
	}

	for (i = 0; i < dec->num_poc_st_curr_before; i++)
		temp[0][n++] = dec->poc_st_curr_before[i];
	for (i = 0; i < dec->num_poc_st_curr_after; i++)
		temp[0][n++] = dec->poc_st_curr_after[i];
	for (i = 0; i < dec->num_poc_lt_curr; i++)
		temp[0][n++] = dec->poc_lt_curr[i];
	n = 0;
	for (i = 0; i < dec->num_poc_st_curr_after; i++)
		temp[1][n++] = dec->poc_st_curr_after[i];
	for (i = 0; i < dec->num_poc_st_curr_before; i++)
		temp[1][n++] = dec->poc_st_curr_before[i];
	for (i = 0; i < dec->num_poc_lt_curr; i++)
		temp[1][n++] = dec->poc_lt_curr[i];
	if (s->slice_type == HEVC_SLICE_P)
		memcpy(temp[1], temp[0], sizeof(temp[0]));

	for (x = 0; x < 2; x++) {
		for (i = 0; i < HEVC_LIST_LEN; i++) {
			vdec_hevc_nofw_get_ref(inst, sh, temp[x][i % nptc], dst, &l[x][i]);
			if (i <= s->num_ref_idx[x] &&
			    (x == 0 || s->slice_type == HEVC_SLICE_B) &&
			    l[x][i].poc > cur)
				low_delay = false;
		}
	}

	writel(low_delay, hw->mv + MV_LOW_DELAY);
	writel(0, hw->mv + MV_324);
	for (i = 0; i < HEVC_LIST_LEN; i++)
		writel(((l[0][i].fb + 1) & 0x1f) << 8 | ((cur - l[0][i].poc) & 0xff),
		       hw->mv + MV_DESC_L0(i));
	for (i = 0; i < HEVC_LIST_LEN; i++)
		writel(((l[1][i].fb + 1) & 0x1f) << 8 | ((cur - l[1][i].poc) & 0xff),
		       hw->mv + MV_DESC_L1(i));
	writel(mv >> 4, hw->mv + MV_CUR);
	writel(1, hw->mc + MC_2D7);
	for (i = 0; i < HEVC_LIST_LEN; i++) {
		writel(l[0][i].mv >> 4, hw->mv + MV_REF_L0(i));
		writel(l[0][i].y, hw->mc + MC_REF_Y_L0(i));
		writel(l[0][i].c, hw->mc + MC_REF_C_L0(i));
	}
	for (i = 0; i < HEVC_LIST_LEN; i++) {
		writel(l[1][i].mv >> 4, hw->mv + MV_REF_L1(i));
		writel(l[1][i].y, hw->mc + MC_REF_Y_L1(i));
		writel(l[1][i].c, hw->mc + MC_REF_C_L1(i));
	}
}

/* DEBUG: the vendor's last value of each core register for POC_A picture 0 */
static const struct { u8 blk; u16 off; u32 val; } hevc_dbg_regs[] = {
	{ 5, 0xf4, 0x0 },
	{ 5, 0xec, 0x1 },
	{ 5, 0xc8, 0x0 },
	{ 5, 0xcc, 0x0 },
	{ 5, 0x178, 0x0 },
	{ 5, 0x84, 0x2 },
	{ 6, 0x84, 0x1 },
	{ 0, 0xec, 0x10000000 },
	{ 0, 0xb4, 0x880000 },
	{ 0, 0xb8, 0x890000 },
	{ 0, 0xb0, 0x22000000 },
	{ 0, 0x110, 0x220054c0 },
	{ 6, 0xbc, 0xa0500000 },
	{ 2, 0xa60, 0x11 },
	{ 7, 0x8c, 0x11301 },
	{ 2, 0xae8, 0x226fd000 },
	{ 2, 0xaec, 0x226ff080 },
	{ 7, 0x7c, 0x226fd000 },
	{ 7, 0x80, 0x226ff080 },
	{ 7, 0x84, 0x112000 },
	{ 7, 0x88, 0x225fe0 },
	{ 2, 0xce4, 0x2fd000 },
	{ 2, 0xce8, 0x101080 },
	{ 4, 0xc90, 0x3 },
	{ 2, 0x4dc, 0x22400000 },
	{ 2, 0x45c, 0x22400000 },
	{ 2, 0xbe0, 0x225fe000 },
	{ 2, 0xc60, 0x225fe000 },
	{ 3, 0x0, 0x1e78000 },
	{ 3, 0x40, 0x1e78000 },
	{ 2, 0x4e0, 0x22400000 },
	{ 2, 0x460, 0x22400000 },
	{ 2, 0xbe4, 0x225fe000 },
	{ 2, 0xc64, 0x225fe000 },
	{ 3, 0x4, 0x1e78000 },
	{ 3, 0x44, 0x1e78000 },
	{ 2, 0x4e4, 0x22400000 },
	{ 2, 0x464, 0x22400000 },
	{ 2, 0xbe8, 0x225fe000 },
	{ 2, 0xc68, 0x225fe000 },
	{ 3, 0x8, 0x1e78000 },
	{ 3, 0x48, 0x1e78000 },
	{ 2, 0x4e8, 0x22400000 },
	{ 2, 0x468, 0x22400000 },
	{ 2, 0xbec, 0x225fe000 },
	{ 2, 0xc6c, 0x225fe000 },
	{ 3, 0xc, 0x1e78000 },
	{ 3, 0x4c, 0x1e78000 },
	{ 2, 0x4ec, 0x22400000 },
	{ 2, 0x46c, 0x22400000 },
	{ 2, 0xbf0, 0x225fe000 },
	{ 2, 0xc70, 0x225fe000 },
	{ 3, 0x10, 0x1e78000 },
	{ 3, 0x50, 0x1e78000 },
	{ 2, 0x4f0, 0x22400000 },
	{ 2, 0x470, 0x22400000 },
	{ 2, 0xbf4, 0x225fe000 },
	{ 2, 0xc74, 0x225fe000 },
	{ 3, 0x14, 0x1e78000 },
	{ 3, 0x54, 0x1e78000 },
	{ 2, 0x4f4, 0x22400000 },
	{ 2, 0x474, 0x22400000 },
	{ 2, 0xbf8, 0x225fe000 },
	{ 2, 0xc78, 0x225fe000 },
	{ 3, 0x18, 0x1e78000 },
	{ 3, 0x58, 0x1e78000 },
	{ 2, 0x4f8, 0x22400000 },
	{ 2, 0x478, 0x22400000 },
	{ 2, 0xbfc, 0x225fe000 },
	{ 2, 0xc7c, 0x225fe000 },
	{ 3, 0x1c, 0x1e78000 },
	{ 3, 0x5c, 0x1e78000 },
	{ 2, 0x4fc, 0x22400000 },
	{ 2, 0x47c, 0x22400000 },
	{ 2, 0xc00, 0x225fe000 },
	{ 2, 0xc80, 0x225fe000 },
	{ 3, 0x20, 0x1e78000 },
	{ 3, 0x60, 0x1e78000 },
	{ 2, 0x500, 0x22400000 },
	{ 2, 0x480, 0x22400000 },
	{ 2, 0xc04, 0x225fe000 },
	{ 2, 0xc84, 0x225fe000 },
	{ 3, 0x24, 0x1e78000 },
	{ 3, 0x64, 0x1e78000 },
	{ 2, 0x504, 0x22400000 },
	{ 2, 0x484, 0x22400000 },
	{ 2, 0xc08, 0x225fe000 },
	{ 2, 0xc88, 0x225fe000 },
	{ 3, 0x28, 0x1e78000 },
	{ 3, 0x68, 0x1e78000 },
	{ 2, 0x508, 0x22400000 },
	{ 2, 0x488, 0x22400000 },
	{ 2, 0xc0c, 0x225fe000 },
	{ 2, 0xc8c, 0x225fe000 },
	{ 3, 0x2c, 0x1e78000 },
	{ 3, 0x6c, 0x1e78000 },
	{ 2, 0x50c, 0x22400000 },
	{ 2, 0x48c, 0x22400000 },
	{ 2, 0xc10, 0x225fe000 },
	{ 2, 0xc90, 0x225fe000 },
	{ 3, 0x30, 0x1e78000 },
	{ 3, 0x70, 0x1e78000 },
	{ 2, 0x510, 0x22400000 },
	{ 2, 0x490, 0x22400000 },
	{ 2, 0xc14, 0x225fe000 },
	{ 2, 0xc94, 0x225fe000 },
	{ 3, 0x34, 0x1e78000 },
	{ 3, 0x74, 0x1e78000 },
	{ 2, 0x514, 0x22400000 },
	{ 2, 0x494, 0x22400000 },
	{ 2, 0xc18, 0x225fe000 },
	{ 2, 0xc98, 0x225fe000 },
	{ 3, 0x38, 0x1e78000 },
	{ 3, 0x78, 0x1e78000 },
	{ 2, 0x518, 0x22400000 },
	{ 2, 0x498, 0x22400000 },
	{ 2, 0xc1c, 0x225fe000 },
	{ 2, 0xc9c, 0x225fe000 },
	{ 3, 0x3c, 0x1e78000 },
	{ 3, 0x7c, 0x1e78000 },
	{ 6, 0xe0, 0xfffefffb },
	{ 6, 0xd4, 0x4011d01 },
	{ 4, 0x40, 0xf011 },
	{ 3, 0x210, 0x0 },
	{ 3, 0x320, 0x0 },
	{ 3, 0x20c, 0x1e78000 },
	{ 2, 0xb5c, 0x1 },
	{ 2, 0x208, 0x1a0 },
	{ 2, 0x20c, 0xf0 },
	{ 2, 0x980, 0x78 },
	{ 2, 0x220, 0x1 },
	{ 2, 0x224, 0x112000 },
	{ 2, 0x228, 0x225fe0 },
	{ 2, 0x22c, 0x78 },
	{ 2, 0x238, 0x3 },
	{ 2, 0x250, 0x1 },
	{ 2, 0x260, 0x1b },
	{ 2, 0x264, 0xf },
	{ 4, 0x630, 0x0 },
	{ 6, 0xc0, 0x1a20 },
	{ 6, 0xc4, 0x80000ff },
	{ 6, 0xc8, 0x0 },
	{ 6, 0xcc, 0x0 },
	{ 6, 0xf4, 0xffff1315 },
	{ 1, 0x68, 0x30006 },
	{ 1, 0x70, 0xf001a0 },
	{ 1, 0x94, 0x40007 },
	{ 6, 0xa0, 0x7721 },
	{ 6, 0xa4, 0x1104 },
	{ 6, 0xa8, 0x575263 },
	{ 6, 0xac, 0x2000000 },
	{ 6, 0xb0, 0x1a60 },
	{ 6, 0xb4, 0x1000000 },
	{ 6, 0xb8, 0x0 },
	{ 6, 0x140, 0x40 },
	{ 6, 0x144, 0x0 },
	{ 6, 0x148, 0x0 },
	{ 6, 0x14c, 0x0 },
	{ 6, 0x150, 0x0 },
	{ 6, 0x154, 0x0 },
	{ 6, 0x158, 0x0 },
	{ 6, 0x15c, 0x0 },
	{ 6, 0x160, 0x0 },
	{ 6, 0x164, 0x0 },
	{ 6, 0x168, 0x0 },
	{ 6, 0x16c, 0x0 },
	{ 6, 0x170, 0x0 },
	{ 6, 0x174, 0x0 },
	{ 6, 0x178, 0x0 },
	{ 6, 0x17c, 0x0 },
	{ 6, 0x1d0, 0x0 },
	{ 6, 0x180, 0x8 },
	{ 6, 0x184, 0x0 },
	{ 6, 0x188, 0x0 },
	{ 6, 0x18c, 0x0 },
	{ 6, 0x190, 0x0 },
	{ 6, 0x194, 0x0 },
	{ 6, 0x198, 0x0 },
	{ 6, 0x19c, 0x0 },
	{ 6, 0x1a0, 0x0 },
	{ 6, 0x1a4, 0x0 },
	{ 6, 0x1a8, 0x0 },
	{ 6, 0x1ac, 0x0 },
	{ 6, 0x1b0, 0x0 },
	{ 6, 0x1b4, 0x0 },
	{ 6, 0x1b8, 0x0 },
	{ 6, 0x1bc, 0x0 },
	{ 6, 0x1ec, 0x0 },
	{ 1, 0x234, 0x1 },
	{ 1, 0x23c, 0x1dfe0000 },
	{ 1, 0x240, 0x1e5c0000 },
	{ 1, 0x230, 0x2 },
	{ 7, 0xc, 0x0 },
	{ 4, 0xb80, 0x808010 },
	{ 7, 0x38, 0x2 },
};

static void vdec_hevc_nofw_dbg_dump(struct vdec_hevc_nofw_inst *inst)
{
	static const char * const name[] = { "vld", "vld_top", "mc", "mv", "pp", "misc", "hevc", "socufo" };
	struct vdec_hevc_nofw_hw *hw = &inst->core;
	void __iomem *base[] = { hw->vld, hw->vld_top, hw->mc, hw->mv, hw->pp,
				 hw->misc, hw->hevc, inst->sys + 0x800 };
	int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(hevc_dbg_regs); i++) {
		u32 v = readl(base[hevc_dbg_regs[i].blk] + hevc_dbg_regs[i].off);

		if (v == hevc_dbg_regs[i].val)
			continue;
		n++;
		dev_info(&inst->ctx->dev->plat_dev->dev, "DUMP %s+0x%03x vendor 0x%08x ours 0x%08x\n",
			 name[hevc_dbg_regs[i].blk], hevc_dbg_regs[i].off,
			 hevc_dbg_regs[i].val, v);
	}
	dev_info(&inst->ctx->dev->plat_dev->dev, "DUMP %d of %zu differ\n", n,
		 ARRAY_SIZE(hevc_dbg_regs));
}

/* DEBUG: the core ranges the vendor's H265_HAL_DumpReg reads */
#define HEVC_DBG_SNAP_WORDS	3200
static const struct { u8 blk; u16 start, end; } hevc_dbg_ranges[] = {
	{ 0, 0x084, 0x400 }, { 1, 0x000, 0x25c }, { 6, 0x084, 0x400 },
	{ 5, 0x000, 0x234 }, { 3, 0x000, 0x3fc }, { 2, 0x000, 0x1000 },
	{ 4, 0x000, 0x1000 },
};

static void vdec_hevc_nofw_dbg_snap(struct vdec_hevc_nofw_inst *inst, u32 *snap)
{
	static const char * const name[] = { "vld", "vld_top", "mc", "mv", "pp", "misc", "hevc" };
	struct vdec_hevc_nofw_hw *hw = &inst->core;
	void __iomem *base[] = { hw->vld, hw->vld_top, hw->mc, hw->mv, hw->pp,
				 hw->misc, hw->hevc };
	int i, n = 0, changed = 0;
	u32 off;

	for (i = 0; i < ARRAY_SIZE(hevc_dbg_ranges); i++) {
		for (off = hevc_dbg_ranges[i].start; off < hevc_dbg_ranges[i].end; off += 4) {
			u32 v;

			if (hevc_dbg_ranges[i].blk == 6 && (off == 0x098 || off == 0x09c))
				continue;
			if (n >= HEVC_DBG_SNAP_WORDS)
				break;
			v = readl(base[hevc_dbg_ranges[i].blk] + off);
			if (snap[HEVC_DBG_SNAP_WORDS] && v != snap[n]) {
				changed++;
				dev_info(&inst->ctx->dev->plat_dev->dev,
					 "CHG %s+0x%03x 0x%08x -> 0x%08x\n",
					 name[hevc_dbg_ranges[i].blk], off, snap[n], v);
			}
			snap[n++] = v;
		}
	}
	if (snap[HEVC_DBG_SNAP_WORDS])
		dev_info(&inst->ctx->dev->plat_dev->dev, "CHG %d of %d words changed\n",
			 changed, n);
	snap[HEVC_DBG_SNAP_WORDS] = 1;
}

/* The core's run: reconstruct the picture from the LAT's output */
static int vdec_hevc_nofw_core_run(struct vdec_hevc_nofw_inst *inst,
				   struct vdec_hevc_nofw_share *sh,
				   struct vdec_lat_buf *lat_buf, struct vb2_buffer *dst)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vdec_msg_queue *q = &ctx->msg_queue;
	struct vdec_hevc_nofw_hw *hw = &inst->core;
	const struct v4l2_ctrl_hevc_sps *sps = &sh->sps;
	u32 w = sps->pic_width_in_luma_samples, h = sps->pic_height_in_luma_samples;
	u32 ctb = 1 << vdec_hevc_nofw_ctb_log2(sps);
	u32 pitch = ctx->picinfo.buf_w;
	u32 ube = q->wdma_addr.dma_addr;
	dma_addr_t y, c, mv = inst->mv_buf[dst->index % HEVC_MAX_MV_NUM].dma_addr;
	u32 last;
	u32 *snap;
	int ret;

	if (dst->index >= HEVC_MAX_MV_NUM)
		return -EINVAL;
	if (hevc_dbg_ufo && !inst->dbg_fb.va) {
		inst->dbg_fb.size = HEVC_DBG_FB_SIZE;
		ret = mtk_vcodec_mem_alloc(ctx, &inst->dbg_fb);
		if (ret)
			return ret;
	}
	if (hevc_dbg_ufo)
		pitch = 1920;
	vdec_hevc_nofw_buf_addr(ctx, dst, &y, &c);
	vdec_hevc_nofw_reset(inst, hw);
	if (hevc_dbg_single && inst->dbg_bs.va) {
		u32 bstart = inst->dbg_bs.dma_addr;
		u32 bend = bstart + ALIGN(inst->dbg_bs_size, 128) + 512;
		struct vdec_hevc_nofw_shdr keep = sh->shdr;

		ret = vdec_hevc_nofw_start_bitstream(inst, hw, bstart, bend, bstart, bend);
		if (ret)
			return ret;
		inst->bsr = &inst->core;
		ret = vdec_hevc_nofw_find_slice(inst, &sh->shdr.nut);
		if (!ret)
			ret = vdec_hevc_nofw_parse_shdr(inst, sh);
		inst->bsr = &inst->lat;
		dev_info(&ctx->dev->plat_dev->dev,
			 "DEBUG single %d: parse %d, nut %u/%u type %u/%u slice0 0x%08x\n",
			 ctx->decoded_frame_cnt, ret, keep.nut, sh->shdr.nut,
			 keep.slice_type, sh->shdr.slice_type, sh->slice0);
		if (ret)
			return ret;
	} else {
		ret = vdec_hevc_nofw_start_bitstream(inst, hw, ube, ube + q->wdma_addr.size,
						     sh->trans_start,
						     ube + ((sh->trans_end - ube + 0x800) & ~63));
		if (ret)
			return ret;
	}

	/* H265_HAL_SetCoreSliceReg */
	writel(sh->slice0, hw->hevc + HEVC_SLICE0);
	if (sh->shdr.slice_type != HEVC_SLICE_I)
		writel(sh->slice2_early, hw->hevc + HEVC_SLICE2);
	if (hevc_dbg_ufo == 1) {
		writel(0x11, hw->mc + MC_UFO);
		vdec_hevc_nofw_rmw(inst->sys + SOCUFO_CFG, 0, 1);
		writel(y + HEVC_DBG_YLEN_OFF, hw->mc + MC_UFO_Y_LEN);
		writel(y + HEVC_DBG_CLEN_OFF, hw->mc + MC_UFO_C_LEN);
		writel(y + HEVC_DBG_YLEN_OFF, inst->sys + SOCUFO_Y_LEN);
		writel(y + HEVC_DBG_CLEN_OFF, inst->sys + SOCUFO_C_LEN);
		writel(y >> 9, inst->sys + SOCUFO_Y);
		writel(c >> 8, inst->sys + SOCUFO_C);
		writel(HEVC_DBG_YLEN_OFF, hw->mc + MC_UFO_Y_OFF);
		writel(HEVC_DBG_CLEN_OFF - HEVC_DBG_C_OFF, hw->mc + MC_UFO_C_OFF);
		writel(3, hw->pp + PP_UFO);
	} else {
		writel(0, hw->mc + MC_UFO);
		vdec_hevc_nofw_rmw(inst->sys + SOCUFO_CFG, 1, 0);
	}

	vdec_hevc_nofw_set_lists(inst, sh, dst, mv);

	/* H265_HAL_SetPicInfoReg */
	writel(w, hw->mc + MC_WIDTH);
	writel(h, hw->mc + MC_HEIGHT);
	writel(pitch / 16, hw->mc + MC_PITCH2);
	writel(1, hw->mc + MC_OUT_ENABLE);
	writel(y >> 9, hw->mc + MC_OUT_Y);
	writel(c >> 8, hw->mc + MC_OUT_C);
	writel(pitch / 16, hw->mc + MC_PITCH);
	writel(3, hw->mc + MC_8E);
	writel(1, hw->mc + MC_94);
	writel(vdec_hevc_nofw_ctbs(w, sps) * ctb / 16 - 1, hw->mc + MC_CTB_W);
	writel(vdec_hevc_nofw_ctbs(h, sps) * ctb / 16 - 1, hw->mc + MC_CTB_H);
	if (hevc_dbg_ufo == 1)
		vdec_hevc_nofw_rmw(inst->sys + SOCUFO_CFG, 0xf0, 1);
	writel(0, hw->pp + PP_SQT);

	vdec_hevc_nofw_set_pic_words(hw, sh);

	/* H265_HAL_SetCoreLatInfo */
	if (!hevc_dbg_single) {
		vdec_hevc_nofw_rmw(hw->vld_top + VLD_TOP_234, 0, 1);
		writel(sh->err_map, hw->vld_top + VLD_TOP_ERR_MAP);
		writel(lat_buf->slice_bc_addr.dma_addr, hw->vld_top + VLD_TOP_SLICE_HDR);
		vdec_hevc_nofw_rmw(hw->vld_top + VLD_TOP_230, 0, 2);
	} else {
		vdec_hevc_nofw_rmw(hw->vld_top + VLD_TOP_234, 1, 0);
		vdec_hevc_nofw_rmw(hw->vld_top + VLD_TOP_230, 3, 0);
	}
	writel(0x101, hw->hevc + HEVC_088);
	writel(0, hw->hevc + HEVC_088);
	if (hevc_dbg_ufo == 1)
		writel(0, inst->sys + SOCUFO_03);
	vdec_hevc_nofw_rmw(hw->pp + PP_TAIL, 4, 0);
	if (hevc_dbg_ufo == 1)
		vdec_hevc_nofw_rmw(inst->sys + SOCUFO_14, 0, 2);
	{
		int i;

		for (i = 0; i + 1 < hevc_dbg_pp_n; i += 2)
			writel(hevc_dbg_pp[i + 1], hw->pp + (hevc_dbg_pp[i] & 0xffc));
		for (i = 0; i + 1 < hevc_dbg_mc_n; i += 2)
			writel(hevc_dbg_mc[i + 1], hw->mc + (hevc_dbg_mc[i] & 0xffc));
	}
	{
		void __iomem *base[] = { hw->vld, hw->vld_top, hw->mc, hw->mv, hw->pp,
					 hw->misc, hw->hevc, inst->sys };
		int i;

		for (i = 0; i + 2 < hevc_dbg_poke_n; i += 3)
			if (hevc_dbg_poke[i] < ARRAY_SIZE(base))
				writel(hevc_dbg_poke[i + 2],
				       base[hevc_dbg_poke[i]] + (hevc_dbg_poke[i + 1] & 0xffc));
	}
	snap = NULL;
	if (!ctx->decoded_frame_cnt) {
		snap = kzalloc((HEVC_DBG_SNAP_WORDS + 1) * sizeof(u32), GFP_KERNEL);
		if (snap)
			vdec_hevc_nofw_dbg_snap(inst, snap);
	}
	if (!ctx->decoded_frame_cnt) {
		struct mtk_vcodec_dec_dev *dev = ctx->dev;
		u32 g[3] = { }, so[6] = { }, r[3];
		static const u32 go[] = { 0x0, 0x8, 0x18 }, soo[] = { 0x4, 0xa0, 0xa8, 0xb0, 0xf0, 0xf8 };
		int i;

		vdec_hevc_nofw_dbg_dump(inst);
		for (i = 0; i < 3 && dev->vdecsys_regmap; i++)
			regmap_read(dev->vdecsys_regmap, go[i], &g[i]);
		for (i = 0; i < 6 && dev->vdecsoc_regmap; i++)
			regmap_read(dev->vdecsoc_regmap, soo[i], &so[i]);
		r[0] = readl(hw->hevc + HEVC_ROW_END_HI);
		writel(0x5, hw->hevc + HEVC_ROW_END_HI);
		r[1] = readl(hw->hevc + HEVC_ROW_END_HI);
		writel(0, hw->hevc + HEVC_ROW_END_HI);
		r[2] = readl(hw->hevc + HEVC_ROW_END_HI);
		dev_info(&dev->plat_dev->dev,
			 "DEBUG gates: vdecsys 0x0 0x%08x 0x8 0x%08x 0x18 0x%08x; vdecsoc 0x4 0x%08x 0xa0 0x%08x 0xa8 0x%08x 0xb0 0x%08x 0xf0 0x%08x 0xf8 0x%08x; 0x1ec 0x%x 0x%x 0x%x\n",
			 g[0], g[1], g[2], so[0], so[1], so[2], so[3], so[4], so[5], r[0], r[1], r[2]);
	}
	writel(1, hw->hevc + HEVC_DECODE);

	ret = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					   HEVC_DEC_TIMEOUT_MS, MTK_VDEC_CORE);
	if (snap) {
		vdec_hevc_nofw_dbg_snap(inst, snap);
		kfree(snap);
	}
	dev_info(&ctx->dev->plat_dev->dev,
		 "DEBUG core %d: ret %d misc41 0x%08x vld61 0x%08x vld58 0x%08x 56-58 0x%08x 0x%08x 0x%08x rptr 0x%08x wptr 0x%08x\n",
		 ctx->decoded_frame_cnt, ret, readl(hw->misc + MISC_IRQ_CFG),
		 readl(hw->vld + 0xf4), readl(hw->vld + VLD_FETCH_STATUS),
		 readl(hw->hevc + 0xe0), readl(hw->hevc + 0xe4), readl(hw->hevc + 0xe8),
		 readl(hw->vld + 0xfc), readl(hw->vld + VLD_WPTR));
	writel(0x10, hw->misc + MISC_IRQ_CFG);
	writel(0, hw->misc + MISC_IRQ_CFG);
	/* The last CTB decoded tells whether the whole picture was */
	last = readl(hw->hevc + 0x12c);
	if (ret || last != ((vdec_hevc_nofw_ctbs(h, sps) - 1) << 16 |
			    (vdec_hevc_nofw_ctbs(w, sps) - 1))) {
		mtk_vdec_err(ctx, "picture %d: %s, last CTB 0x%08x",
			     ctx->decoded_frame_cnt, ret ? "timeout" : "short", last);
		return -EIO;
	}
	return 0;
}

static int vdec_hevc_nofw_core_decode(struct vdec_lat_buf *lat_buf)
{
	struct mtk_vcodec_dec_ctx *ctx = lat_buf->ctx;
	struct vdec_hevc_nofw_inst *inst = ctx->drv_handle;
	struct vdec_hevc_nofw_share *sh = lat_buf->private_data;
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
	err = vdec_hevc_nofw_core_run(inst, sh, lat_buf, &dst->vb2_buf);
out:
	vdec_msg_queue_update_ube_rptr(&ctx->msg_queue, sh->trans_end);
	ctx->dev->vdec_pdata->cap_to_disp(ctx, !!err, lat_buf->src_buf_req);
	ctx->decoded_frame_cnt++;
	return 0;
}

static int vdec_hevc_nofw_alloc_mv_bufs(struct vdec_hevc_nofw_inst *inst)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	/* The library's: a quarter of a byte per sample */
	unsigned int size = ctx->picinfo.buf_w * ctx->picinfo.buf_h / 4;
	int i, err;

	for (i = 0; i < HEVC_MAX_MV_NUM; i++) {
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

static const void *vdec_hevc_nofw_ctrl(struct mtk_vcodec_dec_ctx *ctx, u32 id)
{
	struct v4l2_ctrl *ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, id);

	return ctrl ? ctrl->p_cur.p : NULL;
}

static int vdec_hevc_nofw_decode(void *h_vdec, struct mtk_vcodec_mem *bs,
				 struct vdec_fb *unused, bool *res_chg)
{
	struct vdec_hevc_nofw_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct mtk_video_dec_buf *src_buf;
	struct vdec_hevc_nofw_share *sh;
	struct vdec_lat_buf *lat_buf;
	const void *sps, *pps, *dec;
	int err, i;

	if (vdec_msg_queue_init(&ctx->msg_queue, ctx, vdec_hevc_nofw_core_decode,
				sizeof(*sh)))
		return -ENOMEM;

	/* bs NULL means flush the decoder */
	if (!bs) {
		vdec_msg_queue_wait_lat_buf_full(&ctx->msg_queue);
		return 0;
	}
	*res_chg = false;

	sps = vdec_hevc_nofw_ctrl(ctx, V4L2_CID_STATELESS_HEVC_SPS);
	pps = vdec_hevc_nofw_ctrl(ctx, V4L2_CID_STATELESS_HEVC_PPS);
	dec = vdec_hevc_nofw_ctrl(ctx, V4L2_CID_STATELESS_HEVC_DECODE_PARAMS);
	if (!sps || !pps || !dec)
		return -EINVAL;

	if (inst->realloc_mv_buf) {
		err = vdec_hevc_nofw_alloc_mv_bufs(inst);
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
	memcpy(&sh->sps, sps, sizeof(sh->sps));
	memcpy(&sh->pps, pps, sizeof(sh->pps));
	memcpy(&sh->dec, dec, sizeof(sh->dec));
	if (sh->sps.bit_depth_luma_minus8 || sh->sps.bit_depth_chroma_minus8 ||
	    sh->sps.chroma_format_idc != 1) {
		mtk_vdec_err(ctx, "only 8-bit 4:2:0 HEVC for now");
		vdec_msg_queue_qbuf(&ctx->msg_queue.lat_ctx, lat_buf);
		return -EOPNOTSUPP;
	}
	for (i = 0; i < NUM_BUFFER_COUNT; i++)
		if (lat_buf == &ctx->msg_queue.lat_buf[i])
			sh->err_map = inst->err_map[i].dma_addr;

	err = vdec_hevc_nofw_lat_run(inst, sh, bs,
				     vb2_plane_size(&src_buf->m2m_buf.vb.vb2_buf, 0),
				     lat_buf);
	if (err) {
		vdec_msg_queue_qbuf(&ctx->msg_queue.lat_ctx, lat_buf);
		return err;
	}
	vdec_msg_queue_qbuf(&ctx->msg_queue.core_ctx, lat_buf);
	return 0;
}

/* MM21, as for H.264; whole 64-sample CTBs' worth */
static void vdec_hevc_nofw_get_pic_info(struct vdec_hevc_nofw_inst *inst)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;

	ctx->picinfo.buf_w = ALIGN(ctx->picinfo.pic_w, 64);
	ctx->picinfo.buf_h = ALIGN(ctx->picinfo.pic_h, 64);
	ctx->picinfo.fb_sz[0] = ctx->picinfo.buf_w * ctx->picinfo.buf_h;
	ctx->picinfo.fb_sz[1] = ctx->picinfo.fb_sz[0] / 2;
	inst->realloc_mv_buf = true;
}

static int vdec_hevc_nofw_get_param(void *h_vdec, enum vdec_get_param_type type,
				    void *out)
{
	struct vdec_hevc_nofw_inst *inst = h_vdec;
	struct v4l2_rect *crop = out;

	switch (type) {
	case GET_PARAM_PIC_INFO:
		vdec_hevc_nofw_get_pic_info(inst);
		break;
	case GET_PARAM_CROP_INFO:
		crop->left = 0;
		crop->top = 0;
		crop->width = inst->ctx->picinfo.pic_w;
		crop->height = inst->ctx->picinfo.pic_h;
		break;
	case GET_PARAM_DPB_SIZE:
		*((unsigned int *)out) = HEVC_DPB_SIZE;
		break;
	default:
		mtk_vdec_err(inst->ctx, "invalid get parameter type=%d", type);
		return -EINVAL;
	}
	return 0;
}

static void vdec_hevc_nofw_hw_init(struct vdec_hevc_nofw_hw *hw,
				   struct mtk_vdec_hw_dev *dev, bool lat)
{
	void __iomem *vld = dev->reg_base[VDEC_HW_DEC];

	hw->lat = lat;
	hw->vld = vld;
	hw->vld_top = vld + 0x800;
	hw->misc = dev->reg_base[VDEC_HW_MISC];
	if (lat) {
		hw->mv = vld + 0x2000;
		hw->hevc = vld + 0x4000;
	} else {
		/* HEVC_VLD sits above the core's misc, in its third range */
		hw->mc = vld + 0x1000;
		hw->mv = vld + 0x3000;
		hw->pp = vld + 0x4000;
		hw->hevc = dev->reg_base[VDEC_HW_DEC2] + 0x2000;
	}
}

static void vdec_hevc_nofw_free(struct vdec_hevc_nofw_inst *inst)
{
	int i;

	for (i = 0; i < HEVC_MAX_MV_NUM; i++)
		if (inst->mv_buf[i].va)
			mtk_vcodec_mem_free(inst->ctx, &inst->mv_buf[i]);
	for (i = 0; i < NUM_BUFFER_COUNT; i++)
		if (inst->err_map[i].va)
			mtk_vcodec_mem_free(inst->ctx, &inst->err_map[i]);
	if (inst->dbg_fb.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->dbg_fb);
	if (inst->dbg_bs.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->dbg_bs);
}

static int vdec_hevc_nofw_init(struct mtk_vcodec_dec_ctx *ctx)
{
	struct vdec_hevc_nofw_inst *inst;
	struct mtk_vdec_hw_dev *lat, *core;
	int i, err;

	lat = mtk_vcodec_get_hw_dev(ctx->dev, MTK_VDEC_LAT0);
	core = mtk_vcodec_get_hw_dev(ctx->dev, MTK_VDEC_CORE);
	if (!lat || !core || !lat->reg_base[VDEC_HW_DEC] ||
	    !core->reg_base[VDEC_HW_DEC] || !core->reg_base[VDEC_HW_DEC2]) {
		mtk_vdec_err(ctx, "no HEVC decode registers on the lat or the core");
		return -ENODEV;
	}

	inst = kzalloc_obj(*inst);
	if (!inst)
		return -ENOMEM;
	inst->ctx = ctx;
	inst->sys = core->reg_base[VDEC_HW_SYS];
	vdec_hevc_nofw_hw_init(&inst->lat, lat, true);
	vdec_hevc_nofw_hw_init(&inst->core, core, false);
	inst->bsr = &inst->lat;
	for (i = 0; i < NUM_BUFFER_COUNT; i++) {
		inst->err_map[i].size = HEVC_ERR_MAP_SIZE;
		err = mtk_vcodec_mem_alloc(ctx, &inst->err_map[i]);
		if (err) {
			vdec_hevc_nofw_free(inst);
			kfree(inst);
			return err;
		}
	}
	ctx->drv_handle = inst;
	return 0;
}

static void vdec_hevc_nofw_deinit(void *h_vdec)
{
	struct vdec_hevc_nofw_inst *inst = h_vdec;

	vdec_msg_queue_deinit(&inst->ctx->msg_queue, inst->ctx);
	vdec_hevc_nofw_free(inst);
	kfree(inst);
}

const struct vdec_common_if vdec_hevc_slice_nofw_if = {
	.init		= vdec_hevc_nofw_init,
	.decode		= vdec_hevc_nofw_decode,
	.get_param	= vdec_hevc_nofw_get_param,
	.deinit		= vdec_hevc_nofw_deinit,
};
