// SPDX-License-Identifier: GPL-2.0
/*
 * VP9 stateless decoding for decoders without firmware (MT8695), where the
 * kernel programs the decoder's registers itself.
 *
 * Each frame runs on the LAT, which decodes the tiles' symbols into an
 * intermediate stream (the UBE ring) and counts them, and then on the core,
 * which reconstructs the frame from it; the two are queued through
 * vdec_msg_queue as for H.264. Both halves have VLD, VLD_TOP and MV in their
 * second register range, and BS2 and VP9_VLD in the core's third (in the
 * LAT's second); PP and MC are the core's.
 *
 * The uncompressed header comes from userspace (the LAT's shifter only
 * skips it), the compressed header is parsed by the LAT itself: the driver
 * loads the frame context into the LAT's probability SRAM, starts its
 * boolean decoder and lets it apply the header's deltas section by
 * section. After the LAT, the symbol counts come out by DMA, and the
 * driver adapts and saves the frame context as the spec's refresh_probs()
 * says.
 *
 * 8-bit 4:2:0, no UFO compression (the core writes plain MM21).
 */

#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-vp9.h>
#include <media/videobuf2-dma-contig.h>

#include "../mtk_vcodec_dec.h"
#include "../mtk_vcodec_dec_hw.h"
#include "../../common/mtk_vcodec_intr.h"
#include "../vdec_drv_base.h"
#include "../vdec_drv_if.h"
#include "../vdec_msg_queue.h"
#include "vdec_vp9_req_common.h"

#define VP9_DEC_TIMEOUT_MS		100
#define VP9_FETCH_TIMEOUT_US		10000
#define VP9_MAX_TILES			256
/* From 9 tiles on, the LAT takes its tile records from memory */
#define VP9_REG_TILES			8
#define VP9_DPB_SIZE			9

/* The library's allocations, for frames up to 1920x1088 */
#define VP9_MV_BUF_SIZE			0x144480
#define VP9_SEG_BUF_SIZE		0x12100
#define VP9_COUNT_BUF_SIZE		0x4000
#define VP9_TILE_BUF_SIZE		0x1000

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
#define VLD_TILES			0x140
#define VLD_DQ_ADDR			0x260
#define VLD_DQ_DATA			0x264

#define VLD_TOP_SB_SIZE			0x068	/* SB64 rows - 1 << 16 | columns - 1 */
#define VLD_TOP_PIC_SIZE		0x070	/* height << 16 | width */
#define VLD_TOP_TILE_BUF		0x0b8
#define VLD_TOP_BIT_DEPTH		0x100
#define VLD_TOP_SEG_RD			0x118
#define VLD_TOP_SEG_WR			0x11c
#define VLD_TOP_124			0x124
#define VLD_TOP_230			0x230
#define VLD_TOP_234			0x234
#define VLD_TOP_ERR_MAP			0x23c

/* VP9_VLD */
#define VP9_SHIFT(n)			(4 * (n))	/* 0-32 */
#define VP9_BOOL_BIT			0x084
#define VP9_BOOL_2BITS			0x088
#define VP9_MODE			0x0a4
#define VP9_FLAGS			0x0a8
#define  VP9_FLAG_INTER			BIT(0)
#define  VP9_FLAG_INTRA_ONLY		BIT(1)
#define  VP9_FLAG_LOSSLESS		BIT(2)
#define  VP9_FLAG_PARALLEL		BIT(3)
#define  VP9_FLAG_ERROR_RES		BIT(4)
#define  VP9_FLAG_SEG			BIT(8)
#define  VP9_FLAG_SEG_MAP		BIT(9)
#define  VP9_FLAG_SEG_TEMPORAL		BIT(10)
#define  VP9_FLAG_INTERP(f)		((f) << 24)
#define  VP9_FLAG_HP			BIT(31)
#define VP9_SEG_FEATURES		0x0ac
#define VP9_REF_MODE			0x0b0
#define VP9_TX_MODE			0x0b4
#define VP9_DECODE			0x0b8
#define VP9_0BC				0x0bc
#define VP9_TILE_COUNT			0x0c4
#define VP9_SECTION			0x0c8
#define VP9_SECTION_DONE		0x0cc
#define VP9_TILE_SEL			0x100
#define VP9_TILE_DATA			0x104
#define VP9_BOOL_INIT			0x110
#define VP9_12C				0x12c
#define VP9_BITS_RESET			0x144
#define VP9_PROB_ADDR_A			0x198
#define VP9_PROB_DATA_A			0x19c
#define VP9_PROB_ADDR_B			0x1a0
#define VP9_PROB_DATA_B			0x1a4
#define VP9_CLEAR_COUNTS		0x1a8
#define VP9_DMA_SEL			0x1dc
#define  VP9_DMA_COUNTS			1
#define VP9_DMA_ADDR			0x1e0
#define VP9_DMA_START			0x1e8
#define VP9_DMA_DONE			0x1ec
#define VP9_SEG_TREE_PROBS0		0x1f0
#define VP9_SEG_TREE_PROBS1		0x1f4
#define VP9_SEG_PRED_PROBS		0x1f8

#define MV_CUR				0x20c	/* >> 4 */
#define MV_BUF				0x3c0	/* >> 4 */
#define MV_USE_PREV			0x3c4

#define PP_LF				0x0d8
#define PP_LF_REF_DELTAS		0x0dc
#define PP_LF_MODE_DELTAS		0x0e0
#define PP_SEG_LF			0x0e4
#define PP_SEG_LF_DATA0			0x0e8
#define PP_SEG_LF_DATA1			0x0ec
#define PP_PIC_SIZE			0x0f0
#define PP_DQ_LOAD			0x504
#define PP_LOSSLESS			0x9c0

#define MC_WIDTH			0x208
#define MC_HEIGHT			0x20c
#define MC_OUT_ENABLE			0x220
#define MC_OUT_Y			0x224	/* >> 9 */
#define MC_OUT_C			0x228	/* >> 8 */
#define MC_PITCH			0x22c	/* in 16 bytes */
#define MC_8E				0x238
#define MC_94				0x250
#define MC_SB_W4			0x260	/* SB64 columns * 4 - 1 */
#define MC_SB_H4			0x264	/* SB64 rows * 4 - 1 */
#define MC_UFO				0xa60
#define MC_MB_SIZE			0xaf0	/* MB rows - 1 << 16 | columns - 1 */
#define MC_REF_Y(i)			(0xd28 + 4 * (i))	/* LAST, GOLDEN, ALTREF */
#define MC_REF_C(i)			(0xd34 + 4 * (i))
#define MC_REF_STEP(i)			(0xd40 + 4 * (i))
#define MC_REF_SIZE(i)			(0xd4c + 4 * (i))
#define MC_REF_SCALE_X(i)		(0xd58 + 8 * (i))
#define MC_REF_SCALE_Y(i)		(0xd5c + 8 * (i))
#define MC_REF_PITCH(i)			(0xd70 + 4 * (i))
#define MC_REF_FILTER			0xd7c
#define MC_REF_SCALED			0xd80
#define MC_MI_SIZE			0xd88	/* MiCols << 16 | MiRows */

/* Misc */
#define MISC_01				0x004
#define MISC_SYS_SEL			0x084
#define  MISC_SYS_SEL_VP9		2
#define MISC_IRQ_CFG			0x0a4
#define MISC_SPEC_CON			0x0c8
#define MISC_FUNC_CON			0x0cc
#define MISC_DCM_CON			0x0ec
#define MISC_SRST			0x0f4
#define MISC_178			0x178

/* The parent's range */
#define LAT_WDMA			0x1200
#define  WDMA_07			(LAT_WDMA + 0x1c)
#define  WDMA_08			(LAT_WDMA + 0x20)
#define  WDMA_RST			(LAT_WDMA + 0x2c)
#define  WDMA_RST_DONE			(LAT_WDMA + 0x30)
#define  WDMA_UBE_RPTR			(LAT_WDMA + 0x34)
#define  WDMA_ERR_MAP			(LAT_WDMA + 0x3c)
#define  WDMA_UBE_WPTR			(LAT_WDMA + 0x40)
#define  WDMA_UBE_END			(LAT_WDMA + 0x44)
#define  WDMA_UBE_START			(LAT_WDMA + 0x50)
#define  WDMA_UBE_OFFSET		(LAT_WDMA + 0x78)	/* after the LAT */

/* The clock gate blocks: vdeccore (mediatek,vdecsys), vdecsoc */
#define VDECSYS_DCM_CON			0x018
#define VDECSOC_04			0x004
#define VDECSOC_CORE_BUS		0x0f0
#define VDECSOC_LAT_BUS			0x0f8

/* The compressed header's sections, as the LAT parses them */
#define VP9_SEC_TX			BIT(0)
#define VP9_SEC_COEF			BIT(1)
#define VP9_SEC_SKIP			BIT(2)
#define VP9_SEC_INTER_MODE		BIT(3)
#define VP9_SEC_INTERP			BIT(4)
#define VP9_SEC_IS_INTER		BIT(5)
#define VP9_SEC_COMP_MODE		BIT(6)
#define VP9_SEC_SINGLE_REF		BIT(7)
#define VP9_SEC_COMP_REF		BIT(8)
#define VP9_SEC_Y_MODE			BIT(9)
#define VP9_SEC_PARTITION		BIT(10)
#define VP9_SEC_MV_INT			BIT(11)
#define VP9_SEC_MV_FP			BIT(12)
#define VP9_SEC_MV_HP			BIT(13)

/* The spec's dc_qlookup and ac_qlookup, 8-bit */
static const u16 vp9_dc_q[256] = {
	4, 8, 8, 9, 10, 11, 12, 12, 13, 14, 15, 16,
	17, 18, 19, 19, 20, 21, 22, 23, 24, 25, 26, 26,
	27, 28, 29, 30, 31, 32, 32, 33, 34, 35, 36, 37,
	38, 38, 39, 40, 41, 42, 43, 43, 44, 45, 46, 47,
	48, 48, 49, 50, 51, 52, 53, 53, 54, 55, 56, 57,
	57, 58, 59, 60, 61, 62, 62, 63, 64, 65, 66, 66,
	67, 68, 69, 70, 70, 71, 72, 73, 74, 74, 75, 76,
	77, 78, 78, 79, 80, 81, 81, 82, 83, 84, 85, 85,
	87, 88, 90, 92, 93, 95, 96, 98, 99, 101, 102, 104,
	105, 107, 108, 110, 111, 113, 114, 116, 117, 118, 120, 121,
	123, 125, 127, 129, 131, 134, 136, 138, 140, 142, 144, 146,
	148, 150, 152, 154, 156, 158, 161, 164, 166, 169, 172, 174,
	177, 180, 182, 185, 187, 190, 192, 195, 199, 202, 205, 208,
	211, 214, 217, 220, 223, 226, 230, 233, 237, 240, 243, 247,
	250, 253, 257, 261, 265, 269, 272, 276, 280, 284, 288, 292,
	296, 300, 304, 309, 313, 317, 322, 326, 330, 335, 340, 344,
	349, 354, 359, 364, 369, 374, 379, 384, 389, 395, 400, 406,
	411, 417, 423, 429, 435, 441, 447, 454, 461, 467, 475, 482,
	489, 497, 505, 513, 522, 530, 539, 549, 559, 569, 579, 590,
	602, 614, 626, 640, 654, 668, 684, 700, 717, 736, 755, 775,
	796, 819, 843, 869, 896, 925, 955, 988, 1022, 1058, 1098, 1139,
	1184, 1232, 1282, 1336,
};

static const u16 vp9_ac_q[256] = {
	4, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
	19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
	31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42,
	43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54,
	55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66,
	67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78,
	79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90,
	91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102,
	104, 106, 108, 110, 112, 114, 116, 118, 120, 122, 124, 126,
	128, 130, 132, 134, 136, 138, 140, 142, 144, 146, 148, 150,
	152, 155, 158, 161, 164, 167, 170, 173, 176, 179, 182, 185,
	188, 191, 194, 197, 200, 203, 207, 211, 215, 219, 223, 227,
	231, 235, 239, 243, 247, 251, 255, 260, 265, 270, 275, 280,
	285, 290, 295, 300, 305, 311, 317, 323, 329, 335, 341, 347,
	353, 359, 366, 373, 380, 387, 394, 401, 408, 416, 424, 432,
	440, 448, 456, 465, 474, 483, 492, 501, 510, 520, 530, 540,
	550, 560, 571, 582, 593, 604, 615, 627, 639, 651, 663, 676,
	689, 702, 715, 729, 743, 757, 771, 786, 801, 816, 832, 848,
	864, 881, 898, 915, 933, 951, 969, 988, 1007, 1026, 1046, 1066,
	1087, 1108, 1129, 1151, 1173, 1196, 1219, 1243, 1267, 1292, 1317, 1343,
	1369, 1396, 1423, 1451, 1479, 1508, 1537, 1567, 1597, 1628, 1660, 1692,
	1725, 1759, 1793, 1828,
};

static_assert(sizeof(struct vdec_vp9_slice_frame_counts) <= VP9_COUNT_BUF_SIZE);

/* What a frame's LAT run hands the core */
struct vdec_vp9_nofw_share {
	struct v4l2_ctrl_vp9_frame f;
	u8 tx_mode;
	u8 sign_bias;		/* the four, bits 0-3 */
	u8 comp_fixed, comp_var[2];
	bool use_prev_mvs;
	bool seg_reset;		/* the previous segment ids are all 0 */
	u32 interp, allow_hp;	/* as VP9_FLAGS has them */
	u64 trans_start, trans_end;
};

/* One half's blocks; pp and mc are the core's only */
struct vdec_vp9_nofw_hw {
	void __iomem *vld, *vld_top, *bs2, *vp9, *mv, *pp, *mc, *misc;
	bool lat;
};

struct vdec_vp9_nofw_inst {
	struct mtk_vcodec_dec_ctx *ctx;
	void __iomem *sys;
	struct vdec_vp9_nofw_hw lat, core;

	struct mtk_vcodec_mem mv_buf[2];	/* the LAT's, the core's */
	struct mtk_vcodec_mem seg_buf[2];
	struct mtk_vcodec_mem counts;
	struct mtk_vcodec_mem tile_buf;

	struct v4l2_vp9_frame_context fctx[4];
	struct v4l2_vp9_frame_symbol_counts counts_helper;
	struct vdec_vp9_slice_counts_map counts_map;

	/* What the header's semantics carry from frame to frame */
	u32 last_w, last_h;
	bool last_show, last_key, last_valid;
	u8 comp_fixed, comp_var[2];
	u32 interp, allow_hp;

	/*
	 * By capture buffer, for the references: the frame decoded into it,
	 * and the buffer's layout then (a size change brings new buffers
	 * while the old ones are still referenced)
	 */
	struct {
		u32 w, h, pitch, coff;
	} fb[VIDEO_MAX_FRAME];
};

static void vdec_vp9_nofw_soc_write(struct mtk_vcodec_dec_dev *dev, u32 off, u32 val)
{
	if (dev->vdecsoc_regmap)
		regmap_write(dev->vdecsoc_regmap, off, val);
}

static void vdec_vp9_nofw_rmw(void __iomem *reg, u32 clear, u32 set)
{
	writel((readl(reg) & ~clear) | set, reg);
}

/*
 * The library's reset of one half (Vdec_Drv_VP9_resetHW), with the values
 * its path without UFO takes. The decoder and soc gates it also writes are
 * the clocks'.
 */
static int vdec_vp9_nofw_reset(struct vdec_vp9_nofw_inst *inst,
			       struct vdec_vp9_nofw_hw *hw)
{
	struct mtk_vcodec_dec_dev *dev = inst->ctx->dev;
	u32 val;
	int i;

	writel(0x10, hw->misc + MISC_IRQ_CFG);
	writel(0, hw->misc + MISC_IRQ_CFG);
	if (hw->lat) {
		vdec_vp9_nofw_soc_write(dev, VDECSOC_LAT_BUS, 0);
		vdec_vp9_nofw_rmw(inst->sys + WDMA_RST, 0, 1);
		readl_poll_timeout(inst->sys + WDMA_RST_DONE, val, val & BIT(0),
				   1, 1000);
		vdec_vp9_nofw_rmw(inst->sys + WDMA_08, 0, 1);
	} else {
		vdec_vp9_nofw_soc_write(dev, VDECSOC_CORE_BUS, 0x40004);
	}
	for (i = 0; i < 6; i++)
		writel(0x101, hw->vld + VLD_SRST);
	writel(0xfdff, hw->misc + MISC_SPEC_CON);
	writel(0x71e31182, hw->misc + MISC_FUNC_CON);
	writel(0xfffff041, hw->misc + MISC_178);
	writel(MISC_SYS_SEL_VP9, hw->misc + MISC_SYS_SEL);
	if (!hw->lat)
		vdec_vp9_nofw_soc_write(dev, VDECSOC_04, 2);
	writel(0, hw->misc + MISC_SRST);
	writel(1, hw->misc + MISC_DCM_CON);
	if (dev->vdecsys_regmap)
		regmap_set_bits(dev->vdecsys_regmap, VDECSYS_DCM_CON, 1);
	writel(0, hw->vld + VLD_SRST);
	writel(3, hw->misc + MISC_01);
	writel(1, hw->vp9 + VP9_MODE);
	writel(1, hw->vp9 + VP9_CLEAR_COUNTS);
	if (readl_poll_timeout(hw->vp9 + VP9_CLEAR_COUNTS, val, !(val & BIT(0)),
			       1, 1000))
		return -ETIMEDOUT;
	if (hw->lat)
		vdec_vp9_nofw_rmw(inst->sys + WDMA_08, 1, 0);
	return 0;
}

/*
 * Point a half's VLD at a stream (Vdec_Drv_VP9_init*BarrelShifter): the
 * LAT at the frame, the core at the LAT's output in the UBE ring. [start,
 * end) is the window, rptr where to begin, wptr where the data ends.
 */
static int vdec_vp9_nofw_start_bitstream(struct vdec_vp9_nofw_inst *inst,
					 struct vdec_vp9_nofw_hw *hw,
					 u32 start, u32 end, u32 rptr, u32 wptr)
{
	u32 val;
	int i, ret;

	writel(start >> 6, hw->vld + VLD_BS_START);
	writel(end >> 6, hw->vld + VLD_BS_END);
	writel(start >> 6, hw->bs2 + VLD_BS_START);
	writel(end >> 6, hw->bs2 + VLD_BS_END);
	vdec_vp9_nofw_rmw(hw->vld + VLD_0EC, 0, 0x10000000);
	vdec_vp9_nofw_rmw(hw->bs2 + VLD_0EC, 0, 0x10000000);
	writel(rptr, hw->vld + VLD_RPTR);
	writel(wptr & ~0x3f, hw->vld + VLD_WPTR);
	for (i = 0; i < 5; i++)
		writel(0x100, hw->vld + VLD_SRST);
	writel(0, hw->vld + VLD_SRST);
	writel(VLD_CMD_FETCH, hw->vld + VLD_CMD);

	/* Until the fetch is ready, reading the barrel shifter hangs the bus */
	ret = readl_poll_timeout(hw->vld + VLD_FETCH_STATUS, val,
				 val & VLD_FETCH_READY, 10, VP9_FETCH_TIMEOUT_US);
	if (ret) {
		mtk_vdec_err(inst->ctx, "%s bitstream fetch timed out",
			     hw->lat ? "lat" : "core");
		return ret;
	}
	writel(VLD_CMD_SHIFTER_INIT, hw->vld + VLD_CMD);
	writel(1, hw->vp9 + VP9_BITS_RESET);
	writel(0, hw->vp9 + VP9_BITS_RESET);
	return 0;
}

/* The LAT's barrel shifter: skip bits, 32 at most at a time */
static void vdec_vp9_nofw_skip_bits(struct vdec_vp9_nofw_inst *inst, u32 bits)
{
	while (bits) {
		u32 n = min(bits, 32u);

		readl(inst->lat.vp9 + VP9_SHIFT(0));
		readl(inst->lat.vp9 + VP9_SHIFT(n));
		bits -= n;
	}
}

static bool vdec_vp9_nofw_intra(const struct v4l2_ctrl_vp9_frame *f)
{
	return f->flags & (V4L2_VP9_FRAME_FLAG_KEY_FRAME |
			   V4L2_VP9_FRAME_FLAG_INTRA_ONLY);
}

static bool vdec_vp9_nofw_lossless(const struct v4l2_ctrl_vp9_frame *f)
{
	return !f->quant.base_q_idx && !f->quant.delta_q_y_dc &&
	       !f->quant.delta_q_uv_dc && !f->quant.delta_q_uv_ac;
}

static bool vdec_vp9_nofw_seg(const struct v4l2_ctrl_vp9_frame *f)
{
	return f->seg.flags & V4L2_VP9_SEGMENTATION_FLAG_ENABLED;
}

static u32 vdec_vp9_nofw_sb64(u32 n)
{
	return (n + 63) >> 6;
}

/* VP9_setUnCompressed: the header's words, the same on both halves */
static void vdec_vp9_nofw_set_header(struct vdec_vp9_nofw_inst *inst,
				     struct vdec_vp9_nofw_hw *hw,
				     const struct vdec_vp9_nofw_share *sh)
{
	const struct v4l2_ctrl_vp9_frame *f = &sh->f;
	const struct v4l2_vp9_segmentation *seg = &f->seg;
	u32 w = f->frame_width_minus_1 + 1, h = f->frame_height_minus_1 + 1;
	u32 val = 0;
	int s;

	writel(h << 16 | w, hw->vld_top + VLD_TOP_PIC_SIZE);
	writel((vdec_vp9_nofw_sb64(h) - 1) << 16 | (vdec_vp9_nofw_sb64(w) - 1),
	       hw->vld_top + VLD_TOP_SB_SIZE);
	writel(1, hw->vp9 + VP9_MODE);

	/* 8-bit intra-only frames go as intra, not as inter */
	if (!(f->flags & V4L2_VP9_FRAME_FLAG_KEY_FRAME) &&
	    !(f->flags & V4L2_VP9_FRAME_FLAG_INTRA_ONLY))
		val |= VP9_FLAG_INTER;
	if (f->flags & V4L2_VP9_FRAME_FLAG_INTRA_ONLY)
		val |= VP9_FLAG_INTRA_ONLY;
	if (vdec_vp9_nofw_lossless(f))
		val |= VP9_FLAG_LOSSLESS;
	if (f->flags & V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE)
		val |= VP9_FLAG_PARALLEL;
	if (f->flags & V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT)
		val |= VP9_FLAG_ERROR_RES;
	if (vdec_vp9_nofw_seg(f)) {
		val |= VP9_FLAG_SEG;
		if (seg->flags & V4L2_VP9_SEGMENTATION_FLAG_UPDATE_MAP)
			val |= VP9_FLAG_SEG_MAP;
		if (seg->flags & V4L2_VP9_SEGMENTATION_FLAG_TEMPORAL_UPDATE)
			val |= VP9_FLAG_SEG_TEMPORAL;
	}
	/* Intra frames code neither: the library keeps the last ones */
	val |= VP9_FLAG_INTERP(sh->interp);
	if (sh->allow_hp)
		val |= VP9_FLAG_HP;
	writel(val, hw->vp9 + VP9_FLAGS);

	/* Segment features the LAT needs: SKIP and REF_FRAME */
	val = 0;
	if (vdec_vp9_nofw_seg(f)) {
		for (s = 0; s < 8; s++) {
			u8 en = seg->feature_enabled[s];

			if (en & V4L2_VP9_SEGMENT_FEATURE_ENABLED(V4L2_VP9_SEG_LVL_SKIP))
				val |= BIT(s);
			val |= (seg->feature_data[s][V4L2_VP9_SEG_LVL_REF_FRAME] & 3) <<
			       (8 + 2 * s);
			if (en & V4L2_VP9_SEGMENT_FEATURE_ENABLED(V4L2_VP9_SEG_LVL_REF_FRAME))
				val |= BIT(24 + s);
		}
	}
	writel(val, hw->vp9 + VP9_SEG_FEATURES);

	if (!hw->lat) {
		const struct v4l2_vp9_loop_filter *lf = &f->lf;

		writel((lf->flags & V4L2_VP9_LOOP_FILTER_FLAG_DELTA_ENABLED) |
		       lf->sharpness << 8 | lf->level << 16, hw->pp + PP_LF);
		if (lf->flags & V4L2_VP9_LOOP_FILTER_FLAG_DELTA_ENABLED) {
			writel((u8)lf->ref_deltas[0] | (u8)lf->ref_deltas[1] << 8 |
			       (u8)lf->ref_deltas[2] << 16 |
			       (u32)(u8)lf->ref_deltas[3] << 24,
			       hw->pp + PP_LF_REF_DELTAS);
			writel((u8)lf->mode_deltas[0] | (u8)lf->mode_deltas[1] << 8,
			       hw->pp + PP_LF_MODE_DELTAS);
		}
		if (vdec_vp9_nofw_seg(f)) {
			u32 mask = 0, d0 = 0, d1 = 0;

			for (s = 0; s < 8; s++) {
				u8 d = seg->feature_data[s][V4L2_VP9_SEG_LVL_ALT_L];

				if (seg->feature_enabled[s] &
				    V4L2_VP9_SEGMENT_FEATURE_ENABLED(V4L2_VP9_SEG_LVL_ALT_L))
					mask |= BIT(s);
				if (s < 4)
					d0 |= (u32)d << (8 * s);
				else
					d1 |= (u32)d << (8 * (s - 4));
			}
			if (seg->flags & V4L2_VP9_SEGMENTATION_FLAG_ABS_OR_DELTA_UPDATE)
				mask |= BIT(8);
			writel(mask, hw->pp + PP_SEG_LF);
			writel(d0, hw->pp + PP_SEG_LF_DATA0);
			writel(d1, hw->pp + PP_SEG_LF_DATA1);
		}
		if (vdec_vp9_nofw_lossless(f))
			writel(0xe00, hw->pp + PP_LOSSLESS);
	}

	/* VP9_setSegmentation; one map, read and written in place */
	if (sh->seg_reset)
		memset(inst->seg_buf[hw->lat ? 0 : 1].va, 0, VP9_SEG_BUF_SIZE);
	if (vdec_vp9_nofw_seg(f)) {
		const u8 *tp = seg->tree_probs, *pp = seg->pred_probs;
		u32 map = inst->seg_buf[hw->lat ? 0 : 1].dma_addr;

		writel(tp[0] | tp[1] << 8 | tp[2] << 16 | (u32)tp[3] << 24,
		       hw->vp9 + VP9_SEG_TREE_PROBS0);
		writel(tp[4] | tp[5] << 8 | tp[6] << 16, hw->vp9 + VP9_SEG_TREE_PROBS1);
		writel(pp[0] | pp[1] << 8 | pp[2] << 16, hw->vp9 + VP9_SEG_PRED_PROBS);
		writel(map, hw->vld_top + VLD_TOP_SEG_RD);
		writel(map, hw->vld_top + VLD_TOP_SEG_WR);
	}
}

/* VP9_setPicInfo, both halves: reference mode, and the MV buffer */
static void vdec_vp9_nofw_set_picinfo(struct vdec_vp9_nofw_hw *hw,
				      const struct vdec_vp9_nofw_share *sh,
				      dma_addr_t mv)
{
	writel(sh->f.reference_mode | sh->comp_fixed << 8 | sh->sign_bias << 16 |
	       (sh->comp_var[0] | sh->comp_var[1] << 2) << 24, hw->vp9 + VP9_REF_MODE);
	vdec_vp9_nofw_rmw(hw->mv + MV_USE_PREV, 1, sh->use_prev_mvs);
	writel(mv >> 4, hw->mv + MV_BUF);
	writel(mv >> 4, hw->mv + MV_CUR);
}

/* libvpx's get_tile_offset(), in mode-info units */
static void vdec_vp9_nofw_tile(u32 n_mi, u32 log2, u32 k, u32 *start, u32 *end)
{
	u32 sb = (n_mi + 7) >> 3;

	*start = min(((sb * k) >> log2) << 3, n_mi);
	*end = min(((sb * (k + 1)) >> log2) << 3, n_mi);
}

static void vdec_vp9_nofw_tile_count(const struct v4l2_ctrl_vp9_frame *f,
				     u32 *cols, u32 *rows)
{
	u32 mi_rows = (f->frame_height_minus_1 + 1 + 7) >> 3;
	u32 k, s, e;

	*cols = 1 << f->tile_cols_log2;
	/* Empty tile rows are left out */
	*rows = 0;
	for (k = 0; k < (1u << f->tile_rows_log2); k++) {
		vdec_vp9_nofw_tile(mi_rows, f->tile_rows_log2, k, &s, &e);
		if (e > s)
			(*rows)++;
	}
}

/*
 * The tile words' common tail (VP9_setLatTileInfo, VP9_setCoreTileInfo);
 * bit 16 of VP9_0BC says the LAT's tile records are in registers
 */
static void vdec_vp9_nofw_tiles_end(struct vdec_vp9_nofw_hw *hw, bool in_regs)
{
	vdec_vp9_nofw_rmw(hw->vld + VLD_TILES, 0x001000ff, 0x100014);
	vdec_vp9_nofw_rmw(hw->bs2 + VLD_TILES, 0x001000ff, 0x100014);
	writel(in_regs ? 0x10111 : 0x111, hw->vp9 + VP9_0BC);
}

/*
 * VP9_setLatTileInfo: per tile its data's address and size, from the
 * 4-byte sizes before all but the last tile (after both headers).
 */
static int vdec_vp9_nofw_set_lat_tiles(struct vdec_vp9_nofw_inst *inst,
				       const struct vdec_vp9_nofw_share *sh,
				       struct mtk_vcodec_mem *bs)
{
	const struct v4l2_ctrl_vp9_frame *f = &sh->f;
	struct vdec_vp9_nofw_hw *hw = &inst->lat;
	u32 mi_cols = (f->frame_width_minus_1 + 1 + 7) >> 3;
	u32 mi_rows = (f->frame_height_minus_1 + 1 + 7) >> 3;
	u32 off = f->uncompressed_header_size + f->compressed_header_size;
	u32 cols, rows, r, c, ntiles = 1 << (f->tile_cols_log2 + f->tile_rows_log2);
	u32 *rec = inst->tile_buf.va;
	bool in_regs;
	u32 t = 0;

	vdec_vp9_nofw_tile_count(f, &cols, &rows);
	in_regs = cols * rows <= VP9_REG_TILES;
	if (cols * rows > VP9_MAX_TILES)	/* what the record buffer holds */
		return -EINVAL;
	writel((cols - 1) | (rows - 1) << 16, hw->vp9 + VP9_TILE_COUNT);
	writel(0, hw->vp9 + VP9_TILE_SEL);

	for (r = 0; r < (1u << f->tile_rows_log2); r++) {
		u32 rs, re;

		vdec_vp9_nofw_tile(mi_rows, f->tile_rows_log2, r, &rs, &re);
		for (c = 0; c < cols; c++, t++) {
			u32 cs, ce, size, a, words[4], k;

			if (t == ntiles - 1) {
				if (off > bs->size)
					return -EINVAL;
				size = bs->size - off;
			} else {
				if (off + 4 > bs->size)
					return -EINVAL;
				size = get_unaligned_be32(bs->va + off);
				off += 4;
				if (size > bs->size - off)
					return -EINVAL;
			}
			a = bs->dma_addr + off;
			off += size;
			if (re <= rs)
				continue;
			vdec_vp9_nofw_tile(mi_cols, f->tile_cols_log2, c, &cs, &ce);
			/* Bits, address, offset, width and height in SB64 - 1 */
			words[0] = size * 8 + (a & 15) * 8;
			words[1] = a & ~15;
			words[2] = (a & 15) * 8;
			words[3] = ((((ce - cs + 7) >> 3) - 1) & 0x3f) |
				   (((((re - rs + 7) >> 3) - 1) << 6) & 0x7fc0);
			if (in_regs) {
				for (k = 0; k < 4; k++)
					writel(words[k], hw->vp9 + VP9_TILE_DATA);
			} else {
				for (k = 0; k < 4; k++)
					*rec++ = cpu_to_le32(words[k]);
			}
		}
	}
	vdec_vp9_nofw_tiles_end(hw, in_regs);
	if (!in_regs)
		writel(inst->tile_buf.dma_addr, hw->vld_top + VLD_TOP_TILE_BUF);
	return 0;
}

/* VP9_setCoreTileInfo: the tile columns' widths in SB64 - 1, four to a word */
static void vdec_vp9_nofw_set_core_tiles(struct vdec_vp9_nofw_hw *hw,
					 const struct v4l2_ctrl_vp9_frame *f)
{
	u32 mi_cols = (f->frame_width_minus_1 + 1 + 7) >> 3;
	u32 cols, rows, val = 0, c, s, e;

	vdec_vp9_nofw_tile_count(f, &cols, &rows);
	writel((cols - 1) | (rows - 1) << 16, hw->vp9 + VP9_TILE_COUNT);
	writel(0x80, hw->vp9 + VP9_TILE_SEL);
	for (c = 0; c < cols; c++) {
		vdec_vp9_nofw_tile(mi_cols, f->tile_cols_log2, c, &s, &e);
		val |= ((((e - s + 7) >> 3) - 1) & 0xff) << (8 * (c % 4));
		if (c % 4 == 3) {
			writel(val, hw->vp9 + VP9_TILE_DATA);
			val = 0;
		}
	}
	if (cols % 4)
		writel(val, hw->vp9 + VP9_TILE_DATA);
	vdec_vp9_nofw_tiles_end(hw, cols * rows <= VP9_REG_TILES);
}

/* The probability SRAM's two ports: an address, then auto-incrementing data */
static void vdec_vp9_nofw_prob_words(void __iomem *data, const u8 *b, int n)
{
	int i;

	for (i = 0; i < n; i += 4)
		writel(get_unaligned_le32(b + i), data);
}

/* A row of a table, padded to its slot in the SRAM */
static void vdec_vp9_nofw_prob_row(void __iomem *data, const u8 *row,
				   int len, int slot)
{
	u8 buf[16] = { };

	memcpy(buf, row, len);
	vdec_vp9_nofw_prob_words(data, buf, slot);
}

/*
 * VP9_setProbsTable and VP9_setProbsTable2: the frame context as loaded,
 * before the compressed header's deltas, which the LAT applies itself.
 */
static void vdec_vp9_nofw_load_probs(struct vdec_vp9_nofw_hw *hw,
				     const struct v4l2_vp9_frame_context *fc)
{
	void __iomem *a_addr = hw->vp9 + VP9_PROB_ADDR_A;
	void __iomem *b_addr = hw->vp9 + VP9_PROB_ADDR_B;
	void __iomem *a = hw->vp9 + VP9_PROB_DATA_A;
	void __iomem *b = hw->vp9 + VP9_PROB_DATA_B;
	const struct v4l2_vp9_frame_mv_context *mv = &fc->mv;
	u8 buf[480], grp[16];
	int t, i, j, k, l, c, w, n;

	writel(0x10000, a_addr);
	writel(0x10000, b_addr);
	/*
	 * Coefficients per transform size: per plane and reference, band 0's
	 * 3 contexts in 20 bytes, bands 1-5's 6 in 20 each; every fifth word
	 * goes to the second port
	 */
	for (t = 0; t < 4; t++) {
		n = 0;
		memset(buf, 0, sizeof(buf));
		for (i = 0; i < 2; i++) {
			for (j = 0; j < 2; j++) {
				for (k = 0; k < 6; k++) {
					int ctxs = k ? 6 : 3;

					for (l = 0; l < ctxs; l++) {
						memcpy(buf + n, fc->coef[t][i][j][k][l], 3);
						n += 3;
					}
					n += k ? 2 : 11;
				}
			}
		}
		for (w = 0; w < 120; w++)
			writel(get_unaligned_le32(buf + 4 * w), w % 5 == 4 ? b : a);
	}

	writel(0x101d0, a_addr);
	for (i = 0; i < 10; i++)
		vdec_vp9_nofw_prob_row(a, fc->uv_mode[i], 9, 16);
	writel(0x10180, a_addr);
	for (i = 0; i < 4; i++)
		vdec_vp9_nofw_prob_row(a, fc->y_mode[i], 9, 16);
	writel(0x10190, a_addr);
	for (i = 0; i < 4; i++)
		vdec_vp9_nofw_prob_row(a, fc->interp_filter[i], 2, 16);
	writel(0x10060, b_addr);
	for (i = 0; i < 16; i++)
		vdec_vp9_nofw_prob_row(b, fc->partition[i], 3, 4);

	writel(0x1007e, b_addr);
	vdec_vp9_nofw_prob_words(b, fc->is_inter, 4);
	writel(0x101a8, a_addr);
	vdec_vp9_nofw_prob_row(a, fc->comp_mode, 5, 8);
	writel(0x101ac, a_addr);
	vdec_vp9_nofw_prob_row(a, fc->comp_ref, 5, 8);
	writel(0x101b0, a_addr);
	vdec_vp9_nofw_prob_row(a, &fc->single_ref[0][0], 10, 12);
	writel(0x10078, b_addr);
	vdec_vp9_nofw_prob_row(b, fc->tx8[0], 1, 4);
	vdec_vp9_nofw_prob_row(b, fc->tx8[1], 1, 4);
	vdec_vp9_nofw_prob_row(b, fc->tx16[0], 2, 4);
	vdec_vp9_nofw_prob_row(b, fc->tx16[1], 2, 4);
	vdec_vp9_nofw_prob_row(b, fc->tx32[0], 3, 4);
	vdec_vp9_nofw_prob_row(b, fc->tx32[1], 3, 4);
	writel(0x10077, b_addr);
	vdec_vp9_nofw_prob_row(b, fc->skip, 3, 4);
	writel(0x10070, b_addr);
	for (i = 0; i < 7; i++)
		vdec_vp9_nofw_prob_row(b, fc->inter_mode[i], 3, 4);
	writel(0x101b4, a_addr);
	vdec_vp9_nofw_prob_row(a, mv->joint, 3, 4);

	/* Three groups of 11 bytes per component, each in 16 */
	writel(0x101b8, a_addr);
	for (c = 0; c < 2; c++) {
		grp[0] = mv->sign[c];
		memcpy(grp + 1, mv->classes[c], 10);
		vdec_vp9_nofw_prob_row(a, grp, 11, 16);
	}
	for (c = 0; c < 2; c++) {
		grp[0] = mv->class0_bit[c];
		memcpy(grp + 1, mv->bits[c], 10);
		vdec_vp9_nofw_prob_row(a, grp, 11, 16);
	}
	for (c = 0; c < 2; c++) {
		memcpy(grp, mv->class0_fr[c], 6);
		memcpy(grp + 6, mv->fr[c], 3);
		grp[9] = mv->class0_hp[c];
		grp[10] = mv->hp[c];
		vdec_vp9_nofw_prob_row(a, grp, 11, 16);
	}
}

/* Run one section of the compressed header's parse */
static int vdec_vp9_nofw_section(struct vdec_vp9_nofw_hw *hw, u32 bit)
{
	u32 val;

	writel(bit, hw->vp9 + VP9_SECTION);
	writel(0, hw->vp9 + VP9_SECTION);
	return readl_poll_timeout(hw->vp9 + VP9_SECTION_DONE, val, val & bit,
				  1, 10000);
}

/*
 * VP9_parseCompressedHeader: the LAT's boolean decoder reads the header,
 * and the driver reads the two literals the sections depend on, as the
 * spec orders them.
 */
static int vdec_vp9_nofw_compressed_header(struct vdec_vp9_nofw_inst *inst,
					   struct vdec_vp9_nofw_share *sh)
{
	const struct v4l2_ctrl_vp9_frame *f = &sh->f;
	struct vdec_vp9_nofw_hw *hw = &inst->lat;
	u32 tx_mode = 0, ref_mode = 0;
	int ret = 0;

	writel(1, hw->vp9 + VP9_BITS_RESET);
	writel(0, hw->vp9 + VP9_BITS_RESET);
	writel(1, hw->vp9 + VP9_BOOL_INIT);
	readl(hw->vp9 + VP9_BOOL_INIT);
	if (!vdec_vp9_nofw_lossless(f)) {
		tx_mode = readl(hw->vp9 + VP9_BOOL_2BITS);
		if (tx_mode == 3)
			tx_mode += readl(hw->vp9 + VP9_BOOL_BIT);
	}
	if (tx_mode != sh->tx_mode) {
		mtk_vdec_err(inst->ctx, "tx_mode %u, the stream's %u",
			     sh->tx_mode, tx_mode);
		return -EINVAL;
	}
	writel(tx_mode, hw->vp9 + VP9_TX_MODE);

	if (tx_mode == V4L2_VP9_TX_MODE_SELECT)
		ret |= vdec_vp9_nofw_section(hw, VP9_SEC_TX);
	ret |= vdec_vp9_nofw_section(hw, VP9_SEC_COEF);
	ret |= vdec_vp9_nofw_section(hw, VP9_SEC_SKIP);
	if (!vdec_vp9_nofw_intra(f)) {
		u8 sb = sh->sign_bias;

		ret |= vdec_vp9_nofw_section(hw, VP9_SEC_INTER_MODE);
		if (f->interpolation_filter == V4L2_VP9_INTERP_FILTER_SWITCHABLE)
			ret |= vdec_vp9_nofw_section(hw, VP9_SEC_INTERP);
		ret |= vdec_vp9_nofw_section(hw, VP9_SEC_IS_INTER);
		/* Compound prediction is allowed when the sign biases differ */
		if (((sb >> 1) & 1) != ((sb >> 2) & 1) ||
		    ((sb >> 1) & 1) != ((sb >> 3) & 1)) {
			ref_mode = readl(hw->vp9 + VP9_BOOL_BIT);
			if (ref_mode)
				ref_mode += readl(hw->vp9 + VP9_BOOL_BIT);
		}
		if (ref_mode != f->reference_mode) {
			mtk_vdec_err(inst->ctx, "reference_mode %u, the stream's %u",
				     f->reference_mode, ref_mode);
			return -EINVAL;
		}
		if (ref_mode == V4L2_VP9_REFERENCE_MODE_SELECT)
			ret |= vdec_vp9_nofw_section(hw, VP9_SEC_COMP_MODE);
		if (ref_mode != V4L2_VP9_REFERENCE_MODE_COMPOUND_REFERENCE)
			ret |= vdec_vp9_nofw_section(hw, VP9_SEC_SINGLE_REF);
		if (ref_mode != V4L2_VP9_REFERENCE_MODE_SINGLE_REFERENCE)
			ret |= vdec_vp9_nofw_section(hw, VP9_SEC_COMP_REF);
		ret |= vdec_vp9_nofw_section(hw, VP9_SEC_Y_MODE);
		ret |= vdec_vp9_nofw_section(hw, VP9_SEC_PARTITION);
		ret |= vdec_vp9_nofw_section(hw, VP9_SEC_MV_INT);
		ret |= vdec_vp9_nofw_section(hw, VP9_SEC_MV_FP);
		if (f->flags & V4L2_VP9_FRAME_FLAG_ALLOW_HIGH_PREC_MV)
			ret |= vdec_vp9_nofw_section(hw, VP9_SEC_MV_HP);
	}
	if (ret)
		mtk_vdec_err(inst->ctx, "compressed header: a section timed out");
	return ret ? -ETIMEDOUT : 0;
}

/* The frame's state the header leaves implicit, in decode order */
static void vdec_vp9_nofw_prepare(struct vdec_vp9_nofw_inst *inst,
				  struct vdec_vp9_nofw_share *sh)
{
	const struct v4l2_ctrl_vp9_frame *f = &sh->f;
	u32 w = f->frame_width_minus_1 + 1, h = f->frame_height_minus_1 + 1;
	bool intra = vdec_vp9_nofw_intra(f);
	u8 sb = 0;

	/* setup_past_independence() clears the sign biases */
	if (!intra && !(f->flags & V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT))
		sb = (f->ref_frame_sign_bias & 7) << 1;
	sh->sign_bias = sb;

	/* setup_compound_reference_mode(); kept while there's no compound */
	if (f->reference_mode != V4L2_VP9_REFERENCE_MODE_SINGLE_REFERENCE) {
		bool last = sb & BIT(1), golden = sb & BIT(2);

		if (last == golden) {
			inst->comp_fixed = 3;
			inst->comp_var[0] = 1;
			inst->comp_var[1] = 2;
		} else if (last == !!(sb & BIT(3))) {
			inst->comp_fixed = 2;
			inst->comp_var[0] = 1;
			inst->comp_var[1] = 3;
		} else {
			inst->comp_fixed = 1;
			inst->comp_var[0] = 2;
			inst->comp_var[1] = 3;
		}
	}
	sh->comp_fixed = inst->comp_fixed;
	sh->comp_var[0] = inst->comp_var[0];
	sh->comp_var[1] = inst->comp_var[1];

	if (!intra) {
		inst->interp = f->interpolation_filter;
		inst->allow_hp = !!(f->flags & V4L2_VP9_FRAME_FLAG_ALLOW_HIGH_PREC_MV);
	}
	sh->interp = inst->interp;
	sh->allow_hp = inst->allow_hp;

	/*
	 * libvpx clears the previous segment map in setup_past_independence()
	 * and when the frame size changes (vp9_init_context_buffers())
	 */
	sh->seg_reset = intra || (f->flags & V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT) ||
			!inst->last_valid || w != inst->last_w || h != inst->last_h;

	sh->use_prev_mvs = !(f->flags & V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT) &&
			   inst->last_valid && w == inst->last_w && h == inst->last_h &&
			   !(f->flags & V4L2_VP9_FRAME_FLAG_INTRA_ONLY) &&
			   inst->last_show;
}

/*
 * The spec's refresh_probs() (8.4.2 and 6.1.2): the context the frame
 * loaded, with the compressed header's deltas for parallel or
 * error-resilient frames; otherwise the saved context adapted to the
 * frame's counts, keeping the frame's own tx and skip probabilities on
 * intra frames.
 */
static void vdec_vp9_nofw_refresh_probs(struct vdec_vp9_nofw_inst *inst,
					const struct vdec_vp9_nofw_share *sh, u8 idx,
					const struct v4l2_vp9_frame_context *loaded,
					const struct v4l2_ctrl_vp9_compressed_hdr *ch)
{
	const struct v4l2_ctrl_vp9_frame *f = &sh->f;
	struct v4l2_vp9_frame_context *probs = &inst->fctx[idx];
	struct v4l2_vp9_frame_context *cur;
	bool intra = vdec_vp9_nofw_intra(f);

	if (!(f->flags & V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX))
		return;

	cur = kmemdup(loaded, sizeof(*cur), GFP_KERNEL);
	if (!cur)
		return;
	v4l2_vp9_fw_update_probs(cur, ch, f);
	if (f->flags & (V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT |
			V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE)) {
		*probs = *cur;
		kfree(cur);
		return;
	}

	if (intra) {
		memcpy(probs->tx8, cur->tx8, sizeof(probs->tx8));
		memcpy(probs->tx16, cur->tx16, sizeof(probs->tx16));
		memcpy(probs->tx32, cur->tx32, sizeof(probs->tx32));
		memcpy(probs->skip, cur->skip, sizeof(probs->skip));
	}
	kfree(cur);

	vdec_vp9_slice_counts_map_helper(&inst->counts_map, inst->counts.va,
					 &inst->counts_helper);
	v4l2_vp9_adapt_coef_probs(probs, &inst->counts_helper,
				  !inst->last_valid || inst->last_key, intra);
	if (!intra)
		v4l2_vp9_adapt_noncoef_probs(probs, &inst->counts_helper,
					     f->reference_mode, f->interpolation_filter,
					     ch->tx_mode, f->flags);
}

/* The LAT's run: the compressed header, then the tiles' symbols */
static int vdec_vp9_nofw_lat_run(struct vdec_vp9_nofw_inst *inst,
				 struct vdec_vp9_nofw_share *sh,
				 const struct v4l2_ctrl_vp9_compressed_hdr *ch,
				 struct mtk_vcodec_mem *bs, size_t capacity,
				 struct vdec_lat_buf *lat_buf)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vdec_msg_queue *q = &ctx->msg_queue;
	struct vdec_vp9_nofw_hw *hw = &inst->lat;
	const struct v4l2_ctrl_vp9_frame *f = &sh->f;
	struct v4l2_vp9_frame_context *loaded;
	u32 start = bs->dma_addr, end = bs->dma_addr + capacity;
	u32 ube = q->wdma_addr.dma_addr, val;
	u8 idx;
	int ret;

	ret = vdec_vp9_nofw_reset(inst, hw);
	if (ret)
		return ret;
	ret = vdec_vp9_nofw_start_bitstream(inst, hw, start, end, start,
					    min(start + bs->size + 0x800, end));
	if (ret)
		return ret;
	/* The shifter starts at a 16-byte boundary; past the uncompressed header */
	vdec_vp9_nofw_skip_bits(inst, ((start & 15) + f->uncompressed_header_size) * 8);

	vdec_vp9_nofw_set_header(inst, hw, sh);

	/* setup_past_independence() and the context the frame loads */
	idx = v4l2_vp9_reset_frame_ctx(f, inst->fctx);
	loaded = kmemdup(&inst->fctx[idx], sizeof(*loaded), GFP_KERNEL);
	if (!loaded)
		return -ENOMEM;
	vdec_vp9_nofw_load_probs(hw, loaded);
	ret = vdec_vp9_nofw_compressed_header(inst, sh);
	if (ret)
		goto out;

	/* Where the LAT writes: the UBE ring after the core's data */
	vdec_vp9_nofw_rmw(inst->sys + WDMA_07, 0, 0x3);
	writel(lat_buf->wdma_err_addr.dma_addr, inst->sys + WDMA_ERR_MAP);
	writel(q->wdma_wptr_addr, inst->sys + WDMA_UBE_WPTR);
	writel(q->wdma_rptr_addr, inst->sys + WDMA_UBE_RPTR);
	writel(ube + q->wdma_addr.size, inst->sys + WDMA_UBE_END);
	writel(ube, inst->sys + WDMA_UBE_START);
	vdec_vp9_nofw_rmw(hw->vld_top + VLD_TOP_230, 0, 0x1);

	vdec_vp9_nofw_set_picinfo(hw, sh, inst->mv_buf[0].dma_addr);
	ret = vdec_vp9_nofw_set_lat_tiles(inst, sh, bs);
	if (ret) {
		mtk_vdec_err(ctx, "tiles: %d", ret);
		goto out;
	}

	writel(1, hw->vp9 + VP9_CLEAR_COUNTS);
	ret = readl_poll_timeout(hw->vp9 + VP9_CLEAR_COUNTS, val, !(val & BIT(0)),
				 1, 1000);
	if (ret)
		goto out;
	writel(1, hw->vp9 + VP9_DECODE);

	ret = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					   VP9_DEC_TIMEOUT_MS, MTK_VDEC_LAT0);
	if (ret) {
		mtk_vdec_err(ctx, "lat timeout: frame %d", ctx->decoded_frame_cnt);
		ret = -ETIMEDOUT;
		goto out;
	}

	/* The LAT's output ends at its new write offset in the ring */
	sh->trans_start = q->wdma_wptr_addr;
	sh->trans_end = ube + readl(inst->sys + WDMA_UBE_OFFSET);
	vdec_msg_queue_update_ube_wptr(q, sh->trans_end);

	/* The symbol counts, for the adaptation */
	writel(VP9_DMA_COUNTS, hw->vp9 + VP9_DMA_SEL);
	writel(inst->counts.dma_addr, hw->vp9 + VP9_DMA_ADDR);
	writel(VP9_DMA_COUNTS, hw->vp9 + VP9_DMA_START);
	ret = readl_poll_timeout(hw->vp9 + VP9_DMA_DONE, val, val & BIT(0), 1, 10000);
	if (ret) {
		mtk_vdec_err(ctx, "count table dma timed out");
		goto out;
	}
	vdec_vp9_nofw_refresh_probs(inst, sh, idx, loaded, ch);
out:
	kfree(loaded);
	return ret;
}

/* A reference's frame buffer, by its capture buffer's timestamp */
static struct vb2_buffer *vdec_vp9_nofw_ref(struct mtk_vcodec_dec_ctx *ctx, u64 ts)
{
	return vb2_find_buffer(v4l2_m2m_get_dst_vq(ctx->m2m_ctx), ts);
}

static void vdec_vp9_nofw_buf_addr(struct vb2_buffer *vb, u32 coff,
				   dma_addr_t *y, dma_addr_t *c)
{
	*y = vb2_dma_contig_plane_dma_addr(vb, 0);
	if (vb->num_planes == 2)
		*c = vb2_dma_contig_plane_dma_addr(vb, 1);
	else
		*c = *y + coff;
}

/* The capture buffer has to hold the frame at the current layout */
static int vdec_vp9_nofw_check_dst(struct mtk_vcodec_dec_ctx *ctx,
				   const struct v4l2_ctrl_vp9_frame *f,
				   struct vb2_buffer *dst)
{
	const struct vdec_pic_info *pi = &ctx->picinfo;
	u32 w = f->frame_width_minus_1 + 1, h = f->frame_height_minus_1 + 1;
	bool fits;

	fits = ALIGN(w, 64) <= pi->buf_w && ALIGN(h, 64) <= pi->buf_h;
	if (dst->num_planes == 2)
		fits = fits && vb2_plane_size(dst, 0) >= pi->fb_sz[0] &&
		       vb2_plane_size(dst, 1) >= pi->fb_sz[1];
	else
		fits = fits && vb2_plane_size(dst, 0) >= pi->fb_sz[0] + pi->fb_sz[1];
	if (!fits) {
		mtk_vdec_err(ctx, "%ux%u frame, capture buffer %u for %ux%u",
			     w, h, dst->index, pi->pic_w, pi->pic_h);
		return -EINVAL;
	}
	return 0;
}

/* VP9_setPicInfo, the core's part: references, output, dequantizer */
static void vdec_vp9_nofw_set_core_pic(struct vdec_vp9_nofw_inst *inst,
				       const struct vdec_vp9_nofw_share *sh,
				       struct vb2_buffer *dst)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vdec_vp9_nofw_hw *hw = &inst->core;
	const struct v4l2_ctrl_vp9_frame *f = &sh->f;
	u32 w = f->frame_width_minus_1 + 1, h = f->frame_height_minus_1 + 1;
	u32 pitch = ctx->picinfo.buf_w;
	u64 ts[3] = { f->last_frame_ts, f->golden_frame_ts, f->alt_frame_ts };
	u32 ref_w[3], ref_h[3], ref_p[3], xs[3], ys[3], scaled = 0, filter = 0;
	dma_addr_t y, c, ry, rc;
	int i, s, segs;

	vdec_vp9_nofw_buf_addr(dst, ctx->picinfo.fb_sz[0], &y, &c);
	writel(w, hw->mc + MC_WIDTH);
	writel(h, hw->mc + MC_HEIGHT);

	/* Intra frames point all three at the frame itself */
	for (i = 0; i < 3; i++) {
		struct vb2_buffer *vb = NULL;

		if (!vdec_vp9_nofw_intra(f))
			vb = vdec_vp9_nofw_ref(ctx, ts[i]);
		if (vb && inst->fb[vb->index].w) {
			vdec_vp9_nofw_buf_addr(vb, inst->fb[vb->index].coff, &ry, &rc);
			ref_w[i] = inst->fb[vb->index].w;
			ref_h[i] = inst->fb[vb->index].h;
			ref_p[i] = inst->fb[vb->index].pitch;
		} else {
			if (!vdec_vp9_nofw_intra(f))
				mtk_vdec_err(ctx, "frame %d: no reference %d",
					     ctx->decoded_frame_cnt, i);
			ry = y;
			rc = c;
			ref_w[i] = w;
			ref_h[i] = h;
			ref_p[i] = pitch;
		}
		writel(ry, hw->mc + MC_REF_Y(i));
		writel(rc, hw->mc + MC_REF_C(i));
	}

	if (!(f->flags & V4L2_VP9_FRAME_FLAG_KEY_FRAME)) {
		/*
		 * VP9_setupScaleFactorsForFrame: the reference's size against
		 * the frame's, in 1/16384; all ones outside 2x down to 16x up
		 */
		for (i = 0; i < 3; i++) {
			if (2 * w >= ref_w[i] && 2 * h >= ref_h[i] &&
			    w <= 16 * ref_w[i] && h <= 16 * ref_h[i]) {
				xs[i] = (ref_w[i] << 14) / w;
				ys[i] = (ref_h[i] << 14) / h;
			} else {
				xs[i] = 0xffffffff;
				ys[i] = 0xffffffff;
			}
			if (xs[i] != 0x4000 || ys[i] != 0x4000)
				scaled |= BIT(4 * (2 - i));	/* LAST in bits 11:8 */
		}
		writel(scaled, hw->mc + MC_REF_SCALED);
		for (i = 0; i < 3; i++) {
			u32 x = 7, yc = 7;

			writel(ref_w[i] << 16 | ref_h[i], hw->mc + MC_REF_SIZE(i));
			if (scaled & BIT(4 * (2 - i))) {
				writel((xs[i] >> 10) << 16 | (ys[i] >> 10),
				       hw->mc + MC_REF_STEP(i));
				writel(xs[i], hw->mc + MC_REF_SCALE_X(i));
				writel(ys[i], hw->mc + MC_REF_SCALE_Y(i));
				x = (s32)xs[i] * 1000 < 0xfa4000 ? 4 : 3;
				yc = (s32)ys[i] * 1000 < 0xfa4000 ? 4 : 3;
			}
			filter |= x << (24 - 4 * i) | yc << (8 - 4 * i);
			writel((((ref_p[i] + 15) >> 4) + 3) & ~3, hw->mc + MC_REF_PITCH(i));
		}
		writel(filter, hw->mc + MC_REF_FILTER);
	}

	writel(((w + 7) >> 3) << 16 | ((h + 7) >> 3), hw->mc + MC_MI_SIZE);
	writel(1, hw->mc + MC_OUT_ENABLE);
	writel(y >> 9, hw->mc + MC_OUT_Y);
	writel(c >> 8, hw->mc + MC_OUT_C);
	writel(ALIGN(pitch, 64) / 16, hw->mc + MC_PITCH);
	writel(3, hw->mc + MC_8E);
	writel(1, hw->mc + MC_94);
	writel(vdec_vp9_nofw_sb64(w) * 4 - 1, hw->mc + MC_SB_W4);
	writel(vdec_vp9_nofw_sb64(h) * 4 - 1, hw->mc + MC_SB_H4);
	writel(h << 16 | w, hw->pp + PP_PIC_SIZE);

	/* VP9_setSQTQTable: per segment luma and chroma, dc | ac << 16 */
	writel(1, hw->pp + PP_DQ_LOAD);
	segs = vdec_vp9_nofw_seg(f) ? 8 : 1;
	for (s = 0; s < 8; s++) {
		u32 ydc = 0, yac = 0, uvdc = 0, uvac = 0;

		if (s < segs) {
			int q = f->quant.base_q_idx;

			if (vdec_vp9_nofw_seg(f) &&
			    f->seg.feature_enabled[s] &
			    V4L2_VP9_SEGMENT_FEATURE_ENABLED(V4L2_VP9_SEG_LVL_ALT_Q)) {
				int d = f->seg.feature_data[s][V4L2_VP9_SEG_LVL_ALT_Q];

				if (f->seg.flags & V4L2_VP9_SEGMENTATION_FLAG_ABS_OR_DELTA_UPDATE)
					q = d;
				else
					q += d;
				q = clamp(q, 0, 255);
			}
			ydc = vp9_dc_q[clamp(q + f->quant.delta_q_y_dc, 0, 255)];
			yac = vp9_ac_q[q];
			uvdc = vp9_dc_q[clamp(q + f->quant.delta_q_uv_dc, 0, 255)];
			uvac = vp9_ac_q[clamp(q + f->quant.delta_q_uv_ac, 0, 255)];
		}
		writel(2 * s, hw->vld + VLD_DQ_ADDR);
		writel(ydc | yac << 16, hw->vld + VLD_DQ_DATA);
		writel(2 * s + 1, hw->vld + VLD_DQ_ADDR);
		writel(uvdc | uvac << 16, hw->vld + VLD_DQ_DATA);
	}
	writel(0, hw->pp + PP_DQ_LOAD);

	writel(((w >> 4) - 1) << 16 | ((h >> 4) - 1), hw->mc + MC_MB_SIZE);
	writel(0, hw->mc + MC_UFO);
}

/* The core's run: reconstruct the frame from the LAT's output */
/* DEBUG: a reference for the HEVC core's registers before and after a decode */
#define VP9_DBG_SNAP_WORDS	3200
static const struct { u8 blk; u16 start, end; } vp9_dbg_ranges[] = {
	{ 0, 0x084, 0x400 }, { 1, 0x000, 0x25c }, { 5, 0x000, 0x234 },
	{ 3, 0x000, 0x3fc }, { 2, 0x000, 0x1000 }, { 4, 0x000, 0x1000 },
};

static void vdec_vp9_nofw_dbg_snap(struct vdec_vp9_nofw_inst *inst, u32 *snap)
{
	static const char * const name[] = { "vld", "vld_top", "mc", "mv", "pp", "misc" };
	struct vdec_vp9_nofw_hw *hw = &inst->core;
	void __iomem *base[] = { hw->vld, hw->vld_top, hw->mc, hw->mv, hw->pp, hw->misc };
	int i, n = 0, changed = 0;
	u32 off;

	for (i = 0; i < ARRAY_SIZE(vp9_dbg_ranges); i++) {
		for (off = vp9_dbg_ranges[i].start; off < vp9_dbg_ranges[i].end; off += 4) {
			u32 v;

			if (n >= VP9_DBG_SNAP_WORDS)
				break;
			v = readl(base[vp9_dbg_ranges[i].blk] + off);
			if (snap[VP9_DBG_SNAP_WORDS] && v != snap[n]) {
				changed++;
				dev_info(&inst->ctx->dev->plat_dev->dev,
					 "VCHG %s+0x%03x 0x%08x -> 0x%08x\n",
					 name[vp9_dbg_ranges[i].blk], off, snap[n], v);
			}
			snap[n++] = v;
		}
	}
	if (snap[VP9_DBG_SNAP_WORDS])
		dev_info(&inst->ctx->dev->plat_dev->dev, "VCHG %d of %d words changed\n",
			 changed, n);
	snap[VP9_DBG_SNAP_WORDS] = 1;
}

static int vdec_vp9_nofw_core_run(struct vdec_vp9_nofw_inst *inst,
				  struct vdec_vp9_nofw_share *sh,
				  struct vdec_lat_buf *lat_buf, struct vb2_buffer *dst)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vdec_msg_queue *q = &ctx->msg_queue;
	struct vdec_vp9_nofw_hw *hw = &inst->core;
	const struct v4l2_ctrl_vp9_frame *f = &sh->f;
	u32 ube = q->wdma_addr.dma_addr;
	u32 *snap = NULL;
	int ret;

	ret = vdec_vp9_nofw_check_dst(ctx, f, dst);
	if (ret)
		return ret;
	ret = vdec_vp9_nofw_reset(inst, hw);
	if (ret)
		return ret;
	ret = vdec_vp9_nofw_start_bitstream(inst, hw, ube, ube + q->wdma_addr.size,
					    sh->trans_start, sh->trans_end + 0x800);
	if (ret)
		return ret;
	vdec_vp9_nofw_set_header(inst, hw, sh);
	vdec_vp9_nofw_set_picinfo(hw, sh, inst->mv_buf[1].dma_addr);
	vdec_vp9_nofw_set_core_pic(inst, sh, dst);
	vdec_vp9_nofw_set_core_tiles(hw, f);
	writel(sh->tx_mode, hw->vp9 + VP9_TX_MODE);
	writel(lat_buf->wdma_err_addr.dma_addr, hw->vld_top + VLD_TOP_ERR_MAP);

	/* Vdec_Drv_VP9_trigCore, without UFO */
	vdec_vp9_nofw_rmw(hw->vld_top + VLD_TOP_124, ~0x0fff0010, 0);
	vdec_vp9_nofw_rmw(hw->vld_top + VLD_TOP_234, 0, 1);
	writel(0, hw->vp9 + VP9_12C);
	if (!ctx->decoded_frame_cnt) {
		snap = kzalloc((VP9_DBG_SNAP_WORDS + 1) * sizeof(u32), GFP_KERNEL);
		if (snap)
			vdec_vp9_nofw_dbg_snap(inst, snap);
	}
	writel(1, hw->vp9 + VP9_DECODE);

	ret = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					   VP9_DEC_TIMEOUT_MS, MTK_VDEC_CORE);
	if (snap) {
		vdec_vp9_nofw_dbg_snap(inst, snap);
		kfree(snap);
	}
	writel(0x10, hw->misc + MISC_IRQ_CFG);
	writel(0, hw->misc + MISC_IRQ_CFG);
	if (ret) {
		mtk_vdec_err(ctx, "core timeout: frame %d", ctx->decoded_frame_cnt);
		return -EIO;
	}
	inst->fb[dst->index].w = f->frame_width_minus_1 + 1;
	inst->fb[dst->index].h = f->frame_height_minus_1 + 1;
	inst->fb[dst->index].pitch = ctx->picinfo.buf_w;
	inst->fb[dst->index].coff = ctx->picinfo.fb_sz[0];
	return 0;
}

static int vdec_vp9_nofw_core_decode(struct vdec_lat_buf *lat_buf)
{
	struct mtk_vcodec_dec_ctx *ctx = lat_buf->ctx;
	struct vdec_vp9_nofw_inst *inst = ctx->drv_handle;
	struct vdec_vp9_nofw_share *sh = lat_buf->private_data;
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
	err = vdec_vp9_nofw_core_run(inst, sh, lat_buf, &dst->vb2_buf);
out:
	vdec_msg_queue_update_ube_rptr(&ctx->msg_queue, sh->trans_end);
	ctx->dev->vdec_pdata->cap_to_disp(ctx, !!err, lat_buf->src_buf_req);
	ctx->decoded_frame_cnt++;
	return 0;
}

static int vdec_vp9_nofw_alloc(struct vdec_vp9_nofw_inst *inst,
			       struct mtk_vcodec_mem *mem, size_t size)
{
	mem->size = size;
	return mtk_vcodec_mem_alloc(inst->ctx, mem);
}

static void vdec_vp9_nofw_free(struct vdec_vp9_nofw_inst *inst)
{
	int i;

	for (i = 0; i < 2; i++) {
		if (inst->mv_buf[i].va)
			mtk_vcodec_mem_free(inst->ctx, &inst->mv_buf[i]);
		if (inst->seg_buf[i].va)
			mtk_vcodec_mem_free(inst->ctx, &inst->seg_buf[i]);
	}
	if (inst->counts.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->counts);
	if (inst->tile_buf.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->tile_buf);
}

static int vdec_vp9_nofw_decode(void *h_vdec, struct mtk_vcodec_mem *bs,
				struct vdec_fb *unused, bool *res_chg)
{
	struct vdec_vp9_nofw_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	const struct v4l2_ctrl_vp9_compressed_hdr *ch;
	const struct v4l2_ctrl_vp9_frame *f;
	struct mtk_video_dec_buf *src_buf;
	struct vdec_vp9_nofw_share *sh;
	struct vdec_lat_buf *lat_buf;
	struct v4l2_ctrl *ctrl;
	int err;

	if (vdec_msg_queue_init(&ctx->msg_queue, ctx, vdec_vp9_nofw_core_decode,
				sizeof(*sh)))
		return -ENOMEM;

	/* bs NULL means flush the decoder */
	if (!bs) {
		vdec_msg_queue_wait_lat_buf_full(&ctx->msg_queue);
		return 0;
	}
	*res_chg = false;

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_VP9_FRAME);
	if (!ctrl)
		return -EINVAL;
	f = ctrl->p_cur.p;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_VP9_COMPRESSED_HDR);
	if (!ctrl)
		return -EINVAL;
	ch = ctrl->p_cur.p;
	if (f->bit_depth != 8) {
		mtk_vdec_err(ctx, "%u-bit VP9 is not supported yet", f->bit_depth);
		return -EOPNOTSUPP;
	}
	/* At most four tile rows; the control doesn't check it */
	if (f->tile_rows_log2 > 2)
		return -EINVAL;

	lat_buf = vdec_msg_queue_dqbuf(&ctx->msg_queue.lat_ctx);
	if (!lat_buf)
		return -EAGAIN;
	sh = lat_buf->private_data;
	src_buf = container_of(bs, struct mtk_video_dec_buf, bs_buffer);
	lat_buf->src_buf_req = src_buf->m2m_buf.vb.vb2_buf.req_obj.req;
	v4l2_m2m_buf_copy_metadata(&src_buf->m2m_buf.vb, &lat_buf->ts_info);

	memset(sh, 0, sizeof(*sh));
	sh->f = *f;
	sh->tx_mode = ch->tx_mode;
	vdec_vp9_nofw_prepare(inst, sh);

	err = vdec_vp9_nofw_lat_run(inst, sh, ch, bs,
				    vb2_plane_size(&src_buf->m2m_buf.vb.vb2_buf, 0),
				    lat_buf);
	if (err) {
		vdec_msg_queue_qbuf(&ctx->msg_queue.lat_ctx, lat_buf);
		return err;
	}

	inst->last_w = f->frame_width_minus_1 + 1;
	inst->last_h = f->frame_height_minus_1 + 1;
	inst->last_show = !!(f->flags & V4L2_VP9_FRAME_FLAG_SHOW_FRAME);
	inst->last_key = !!(f->flags & V4L2_VP9_FRAME_FLAG_KEY_FRAME);
	inst->last_valid = true;

	vdec_msg_queue_qbuf(&ctx->msg_queue.core_ctx, lat_buf);
	return 0;
}

/*
 * MM21, as for H.264; the pitch a multiple of 64 (the core takes it so),
 * whole superblocks' worth of lines
 */
static void vdec_vp9_nofw_get_pic_info(struct vdec_vp9_nofw_inst *inst)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;

	ctx->picinfo.buf_w = ALIGN(ctx->picinfo.pic_w, 64);
	ctx->picinfo.buf_h = ALIGN(ctx->picinfo.pic_h, 64);
	ctx->picinfo.fb_sz[0] = ctx->picinfo.buf_w * ctx->picinfo.buf_h;
	ctx->picinfo.fb_sz[1] = ctx->picinfo.fb_sz[0] / 2;
}

static int vdec_vp9_nofw_get_param(void *h_vdec, enum vdec_get_param_type type,
				   void *out)
{
	struct vdec_vp9_nofw_inst *inst = h_vdec;
	struct v4l2_rect *crop = out;

	switch (type) {
	case GET_PARAM_PIC_INFO:
		vdec_vp9_nofw_get_pic_info(inst);
		break;
	case GET_PARAM_CROP_INFO:
		crop->left = 0;
		crop->top = 0;
		crop->width = inst->ctx->picinfo.pic_w;
		crop->height = inst->ctx->picinfo.pic_h;
		break;
	case GET_PARAM_DPB_SIZE:
		*((unsigned int *)out) = VP9_DPB_SIZE;
		break;
	default:
		mtk_vdec_err(inst->ctx, "invalid get parameter type=%d", type);
		return -EINVAL;
	}
	return 0;
}

static void vdec_vp9_nofw_hw_init(struct vdec_vp9_nofw_hw *hw,
				  struct mtk_vdec_hw_dev *dev, bool lat)
{
	void __iomem *vld = dev->reg_base[VDEC_HW_DEC];

	hw->lat = lat;
	hw->vld = vld;
	hw->vld_top = vld + 0x800;
	hw->misc = dev->reg_base[VDEC_HW_MISC];
	if (lat) {
		hw->mv = vld + 0x2000;
		hw->bs2 = vld + 0x3800;
		hw->vp9 = vld + 0x4400;
	} else {
		/* BS2 and VP9_VLD sit above the core's misc, in its third range */
		void __iomem *dec2 = dev->reg_base[VDEC_HW_DEC2];

		hw->mc = vld + 0x1000;
		hw->mv = vld + 0x3000;
		hw->pp = vld + 0x4000;
		hw->bs2 = dec2 + 0x800;
		hw->vp9 = dec2 + 0x2400;
	}
}

static int vdec_vp9_nofw_init(struct mtk_vcodec_dec_ctx *ctx)
{
	struct vdec_vp9_nofw_inst *inst;
	struct mtk_vdec_hw_dev *lat, *core;
	int i, err;

	lat = mtk_vcodec_get_hw_dev(ctx->dev, MTK_VDEC_LAT0);
	core = mtk_vcodec_get_hw_dev(ctx->dev, MTK_VDEC_CORE);
	if (!lat || !core || !lat->reg_base[VDEC_HW_DEC] ||
	    !core->reg_base[VDEC_HW_DEC] || !core->reg_base[VDEC_HW_DEC2]) {
		mtk_vdec_err(ctx, "no VP9 decode registers on the lat or the core");
		return -ENODEV;
	}

	inst = kzalloc_obj(*inst);
	if (!inst)
		return -ENOMEM;
	inst->ctx = ctx;
	inst->sys = core->reg_base[VDEC_HW_SYS];
	vdec_vp9_nofw_hw_init(&inst->lat, lat, true);
	vdec_vp9_nofw_hw_init(&inst->core, core, false);

	for (i = 0; i < 2; i++) {
		err = vdec_vp9_nofw_alloc(inst, &inst->mv_buf[i], VP9_MV_BUF_SIZE);
		if (!err)
			err = vdec_vp9_nofw_alloc(inst, &inst->seg_buf[i], VP9_SEG_BUF_SIZE);
		if (err)
			goto err;
	}
	err = vdec_vp9_nofw_alloc(inst, &inst->counts, VP9_COUNT_BUF_SIZE);
	if (!err)
		err = vdec_vp9_nofw_alloc(inst, &inst->tile_buf, VP9_TILE_BUF_SIZE);
	if (err)
		goto err;
	for (i = 0; i < 4; i++)
		inst->fctx[i] = v4l2_vp9_default_probs;

	ctx->drv_handle = inst;
	return 0;

err:
	vdec_vp9_nofw_free(inst);
	kfree(inst);
	return err;
}

static void vdec_vp9_nofw_deinit(void *h_vdec)
{
	struct vdec_vp9_nofw_inst *inst = h_vdec;

	vdec_msg_queue_deinit(&inst->ctx->msg_queue, inst->ctx);
	vdec_vp9_nofw_free(inst);
	kfree(inst);
}

const struct vdec_common_if vdec_vp9_slice_nofw_if = {
	.init		= vdec_vp9_nofw_init,
	.decode		= vdec_vp9_nofw_decode,
	.get_param	= vdec_vp9_nofw_get_param,
	.deinit		= vdec_vp9_nofw_deinit,
};
