// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek VDOUT: the video planes
 *
 * Each video plane reads a two-plane YCbCr picture, luma then interleaved
 * chroma, through its DMA front end (UFOD) into its VDO core, which scales
 * it vertically. Its own formatter scales it horizontally and places it on
 * the screen, and the VDOUT mixer takes it below the OSD premix. Here the
 * buffers are linear NV12 or NV16, or NV21 or NV61 through the
 * formatter's U/V swap. All four blocks are shadowed: a frame's
 * registers latch together at the plane's next vsync, which VDOUT delays
 * from its own. The register sequences follow MediaTek's MT8695 display
 * driver.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_color_mgmt.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_plane.h>
#include <drm/drm_rect.h>

#include "mtk_vdout_drv.h"

/* Formatter at 0x000, SCL at 0x100, UFOD at 0x200, VDO core at 0x400 */
#define FMT_HCOEF		0x000	/* 16 words, then a 17th at 0x04c */
#define FMT_HCOEF16		0x04c
#define FMT_DERING		0x054
#define FMT_HDWN		0x06c
#define  FMT_HDWN_EN		BIT(0)
#define  FMT_HDWN_FULL_FIFO	BIT(5)
#define  FMT_HDWN_444		BIT(11)
#define FMT_HDWN_IN		0x070
#define  FMT_HDWN_2F		BIT(29)
#define FMT_HDWN_VO		0x074
#define FMT_HDWN_VE		0x078
#define FMT_HDWN_OUT		0x07c
#define FMT_MODE		0x094
#define FMT_PXLLEN		0x09c
#define  FMT_VP_RESET_SELECT	BIT(15)
#define FMT_ACTIVE_X		0x0a0
#define  FMT_SHADOW_EN		BIT(29)
#define  FMT_TRIGGER		BIT(31)
#define FMT_ACTIVE_Y0		0x0a4
#define FMT_ACTIVE_Y1		0x0a8
#define FMT_CTRL		0x0ac
#define  FMT_VDO_EN		BIT(0)
#define  FMT_UVSW		BIT(19)	/* Cr first */
#define FMT_HSCALE		0x0b0
#define  FMT_HSON		BIT(0)
#define  FMT_HSLR		BIT(1)	/* linear, at 1:1 */
#define  FMT_PHASE_16		BIT(31)
#define FMT_C4			0x0c4
#define FMT_ADJ_T0		0x0d0
#define FMT_ADJ_T1		0x0d4
#define FMT_RATIO		0x0e4
#define FMT_DELAY		0x0e8
#define FMT_OUTPUT		0x0f0
#define  FMT_OUTPUT_444		BIT(24)

#define SCL_LINE_BUF		0x1a0	/* left/right start [6:0] */
#define  SCL_REG_OFFSET		BIT(31)

#define UFOD_Y			0x200
#define UFOD_C			0x204
#define UFOD_SIZE		0x208	/* height [31:20], half the pitch [11:0] */
#define UFOD_PITCH		0x20c	/* 16-byte units */
#define UFOD_Y_LENGTH		0x210
#define UFOD_C_LENGTH		0x214
#define UFOD_MODE		0x218
#define  UFOD_PACK_8BIT		(3 << 24)
#define  UFOD_UHD4K		BIT(27)	/* pitch over 1024 */
#define  UFOD_YUV422		BIT(29)
#define  UFOD_RASTER		(2 << 30)
#define UFOD_TH			0x21c
#define UFOD_ULTRA		0x220
#define UFOD_EN			0x240
#define  UFOD_ENABLE		BIT(0)
#define  UFOD_TRIGGER		BIT(2)

#define VDO_SIZE		0x410
#define VDO_SCALE		0x414	/* vertical [15:0], pixel skip [27:16] */
#define  VDO_SHADOW_MODE	BIT(30)
#define  VDO_TRIGGER		BIT(31)
#define VDO_CTRL		0x41c
#define  VDO_EN			BIT(0)
#define  VDO_CLR		BIT(7)
#define VDO_YSLTT		0x420
#define VDO_CSLTT		0x424
#define VDO_CSSLTT		0x42c
#define VDO_FORMAT		0x430
#define  VDO_YUV422		BIT(21)
#define VDO_YFIR		0x478
#define  VDO_YFIR_ON		BIT(0)
#define VDO_OPT			0x47c
#define  VDO_OPT8		BIT(8)
#define VDO_DW_HD		0x4e0

/* dispsys */
#define DISPSYS_RESET		0x008	/* active low */
#define DISPSYS_PRIO_MAIN	0x014
#define DISPSYS_PRIO_SUB	0x018

/* Vertical scale: 0x800 is 1:1 */
#define VDO_VSCALE_ONE		0x800
/* Horizontal scale: 0x1000 is 1:1 */
#define FMT_HSCALE_ONE		0x1000

/* Down 8x horizontally, 2x vertically (checked by hand); up any amount */
#define VDO_MIN_SCALE		1
#define VDO_MAX_SCALE		(DRM_PLANE_NO_SCALING * 8)

/* Two frames at 50 Hz, for the plane to latch its disable */
#define VDO_OFF_DELAY_MS	50

/* The horizontal scaler's 16-phase filter; MediaTek uses it at any ratio */
static const u32 mtk_vdout_vdo_hcoef[17] = {
	0x00000000, 0xf913d2a2, 0xfff205fe, 0xff03fa10,
	0xfbe70afc, 0xfe05f322, 0xf3de0efa, 0xfd08ed35,
	0xe8d711f9, 0xfc0be64a, 0xd9d313f8, 0xfb0ee060,
	0xcad014f8, 0xfa10da76, 0xb8d014f8, 0xf912d58c,
	0x00000001,
};

/* The differences between the planes */
struct mtk_vdout_vdo_data {
	enum mtk_vdout_vdo_id id;
	u32 reset;		/* dispsys reset bit */
	u32 prio_reg;		/* dispsys bus priority: ultra, or pre-ultra */
	u32 prio;
	u32 ufod_ultra;		/* the UFOD's own, the same kind */
	bool fmt_delay;		/* the formatter takes a delay (main only) */
};

struct mtk_vdout_vdo {
	struct device *dev;
	const struct mtk_vdout_vdo_data *data;
	struct mtk_vdout *vd;		/* while bound */
	struct drm_plane plane;
	void __iomem *regs;
	struct regmap *dispsys;
	struct clk *clk_vdo;
	struct clk *clk_fmt;

	/* The mode the plane is set up for, while it is powered */
	const struct mtk_vdout_vdo_timing *timing;
};

static const u32 mtk_vdout_vdo_formats[] = {
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV61,
};

/* Linear, and the video decoder's block layout (16x32 luma, 16x16 chroma tiles) */
#define VDO_MOD_BLOCK	DRM_FORMAT_MOD_MTK(MTK_FMT_MOD_TILE_16L32S)

static const u64 mtk_vdout_vdo_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	VDO_MOD_BLOCK,
	DRM_FORMAT_MOD_INVALID,
};

static inline struct mtk_vdout_vdo *to_mtk_vdout_vdo(struct drm_plane *plane)
{
	return container_of(plane, struct mtk_vdout_vdo, plane);
}

static void rmw(void __iomem *base, u32 off, u32 mask, u32 val)
{
	writel((readl(base + off) & ~mask) | (val & mask), base + off);
}

/* Formatter registers are shadowed once FMT_SHADOW_EN is set */
static void fmt_write(struct mtk_vdout_vdo *vdo, u32 off, u32 val)
{
	writel(val, vdo->regs + off);
	rmw(vdo->regs, FMT_ACTIVE_X, FMT_TRIGGER, FMT_TRIGGER);
}

/* The VDO core and the UFOD latch together */
static void vdo_trigger(struct mtk_vdout_vdo *vdo)
{
	rmw(vdo->regs, VDO_SCALE, VDO_TRIGGER, VDO_TRIGGER);
	rmw(vdo->regs, UFOD_EN, UFOD_TRIGGER, UFOD_TRIGGER);
}

/* disp_vdp_start(): the formatter for the mode, the path, out of reset */
static int mtk_vdout_vdo_on(struct mtk_vdout_vdo *vdo,
			    const struct mtk_vdout_vdo_timing *t)
{
	const struct mtk_vdout_vdo_data *data = vdo->data;
	int ret;

	ret = pm_runtime_resume_and_get(vdo->dev);
	if (ret)
		return ret;
	ret = clk_prepare_enable(vdo->clk_fmt);
	if (ret)
		goto err_put;

	/* _fmt_set_dispfmt_mode() */
	fmt_write(vdo, FMT_PXLLEN, FMT_VP_RESET_SELECT);
	fmt_write(vdo, FMT_ADJ_T0, 0);
	fmt_write(vdo, FMT_ADJ_T1, 0);
	fmt_write(vdo, FMT_C4, 0x10);
	fmt_write(vdo, FMT_RATIO, t->fmt_ratio);
	fmt_write(vdo, FMT_MODE, t->fmt_mode);
	fmt_write(vdo, FMT_CTRL, 0);
	if (data->fmt_delay)
		fmt_write(vdo, FMT_DELAY, t->fmt_delay);
	mtk_vdout_video_delay(vdo->vd, data->id, t->path_delay);
	fmt_write(vdo, FMT_OUTPUT, FMT_OUTPUT_444);
	rmw(vdo->regs, FMT_ACTIVE_X, FMT_SHADOW_EN, FMT_SHADOW_EN);

	ret = clk_prepare_enable(vdo->clk_vdo);
	if (ret)
		goto err_fmt;
	regmap_set_bits(vdo->dispsys, data->prio_reg, data->prio);

	/* No windows until the first frame */
	mtk_vdout_video_window(vdo->vd, data->id, 0, 0);
	fmt_write(vdo, FMT_ACTIVE_X, FMT_SHADOW_EN);
	fmt_write(vdo, FMT_ACTIVE_Y0, 0);
	fmt_write(vdo, FMT_ACTIVE_Y1, 0);
	regmap_set_bits(vdo->dispsys, DISPSYS_RESET, data->reset);

	/* Shadowed from the first frame's writes on */
	writel(VDO_SHADOW_MODE, vdo->regs + VDO_SCALE);

	vdo->timing = t;
	return 0;

err_fmt:
	clk_disable_unprepare(vdo->clk_fmt);
err_put:
	pm_runtime_put(vdo->dev);
	return ret;
}

/*
 * vdp_stop_disable_hw() and vdp_disable_display_path(): the window and the
 * DMA off at the next vsync, then the plane after it
 */
void mtk_vdout_vdo_off(struct mtk_vdout_vdo *vdo)
{
	const struct mtk_vdout_vdo_data *data = vdo->data;

	if (!vdo->timing)
		return;

	fmt_write(vdo, FMT_ACTIVE_X, FMT_SHADOW_EN);
	fmt_write(vdo, FMT_ACTIVE_Y0, 0);
	fmt_write(vdo, FMT_ACTIVE_Y1, 0);
	mtk_vdout_video_window(vdo->vd, data->id, 0, 0);
	mtk_vdout_video_mix(vdo->vd, data->id, false);
	writel(VDO_CLR, vdo->regs + VDO_CTRL);
	writel(0, vdo->regs + UFOD_EN);
	vdo_trigger(vdo);

	msleep(VDO_OFF_DELAY_MS);

	rmw(vdo->regs, FMT_CTRL, FMT_VDO_EN, 0);
	clk_disable_unprepare(vdo->clk_vdo);
	clk_disable_unprepare(vdo->clk_fmt);
	regmap_clear_bits(vdo->dispsys, DISPSYS_RESET, data->reset);
	pm_runtime_put(vdo->dev);
	vdo->timing = NULL;
}

/* The source, in whole pixels; its top lines are skipped by address */
struct mtk_vdout_vdo_src {
	u32 x, y, w, h;
};

static void mtk_vdout_vdo_get_src(const struct drm_plane_state *state,
				  struct mtk_vdout_vdo_src *src)
{
	src->x = state->src.x1 >> 16;
	src->y = state->src.y1 >> 16;
	src->w = drm_rect_width(&state->src) >> 16;
	src->h = drm_rect_height(&state->src) >> 16;
}

/* The bytes of plane i the UFOD reads, from the buffer's start */
static size_t mtk_vdout_vdo_extent(const struct drm_framebuffer *fb,
				   const struct mtk_vdout_vdo_src *src, int i)
{
	unsigned int vsub = i ? fb->format->vsub : 1;

	/* Whole rows of tiles: 32 luma lines, 16 chroma lines */
	if (fb->modifier == VDO_MOD_BLOCK)
		return fb->offsets[i] +
		       (size_t)ALIGN(src->h, 32) / vsub * fb->pitches[0];

	/* It reads whole pitches, and the height rounded up to 4 lines */
	return fb->offsets[i] +
	       (size_t)(src->y + ALIGN(src->h, 4)) / vsub * fb->pitches[0];
}

static int mtk_vdout_vdo_atomic_check(struct drm_plane *plane,
				      struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = new_state->fb;
	struct drm_crtc_state *crtc_state = NULL;
	struct mtk_vdout_vdo_src src;
	int ret, i;

	if (new_state->crtc)
		crtc_state = drm_atomic_get_new_crtc_state(state, new_state->crtc);

	ret = drm_atomic_helper_check_plane_state(new_state, crtc_state,
						  VDO_MIN_SCALE, VDO_MAX_SCALE,
						  true, true);
	if (ret || !new_state->visible)
		return ret;

	mtk_vdout_vdo_get_src(new_state, &src);

	/* No larger than the mode, and down 2x at most vertically */
	if (src.w > crtc_state->mode.hdisplay ||
	    src.h > crtc_state->mode.vdisplay ||
	    src.h > 2 * drm_rect_height(&new_state->dst))
		return -EINVAL;

	/* The pixel skip drops bit 0; 4:2:0 chroma starts on an even line */
	if (src.x % 2 || src.y % fb->format->vsub)
		return -EINVAL;

	/* Blocks are read from their buffer's start (the vendor's crops by start line) */
	if (fb->modifier == VDO_MOD_BLOCK && (src.x || src.y))
		return -EINVAL;

	/* One pitch for both planes, in 16-byte units */
	if (fb->pitches[0] != fb->pitches[1] || fb->pitches[0] % 16 ||
	    fb->pitches[0] > 2048)
		return -EINVAL;

	for (i = 0; i < 2; i++) {
		if (fb->offsets[i] % 16)
			return -EINVAL;
		if (mtk_vdout_vdo_extent(fb, &src, i) > fb->obj[i]->size)
			return -EINVAL;
	}

	return 0;
}

/*
 * vdp_update_fmt_setting(): the horizontal scaler, or for shrinking the
 * H-down scaler, and the window
 */
static void mtk_vdout_vdo_fmt_update(struct mtk_vdout_vdo *vdo,
				     const struct mtk_vdout_vdo_timing *t,
				     const struct mtk_vdout_vdo_src *src,
				     const struct drm_rect *dst)
{
	u32 ox = t->hstart + dst->x1, ow = drm_rect_width(dst);
	u32 oy = t->vstart + dst->y1, oh = drm_rect_height(dst);
	u32 v = oy << 16 | (oy + oh - 1);
	unsigned int i;
	bool fifo;

	for (i = 0; i < 16; i++)
		fmt_write(vdo, FMT_HCOEF + 4 * i, mtk_vdout_vdo_hcoef[i]);
	fmt_write(vdo, FMT_HCOEF16, mtk_vdout_vdo_hcoef[16]);
	fmt_write(vdo, FMT_DERING, 0x3201c01c);
	fmt_write(vdo, FMT_PXLLEN, FMT_VP_RESET_SELECT | src->w);

	if (src->w <= ow) {
		u32 f = src->w * FMT_HSCALE_ONE / ow;

		fmt_write(vdo, FMT_HSCALE, FMT_PHASE_16 | f << 16 |
			  (f == FMT_HSCALE_ONE ? FMT_HSLR : 0) | FMT_HSON);
		fmt_write(vdo, FMT_HDWN, FMT_HSCALE_ONE << 16 | FMT_HDWN_FULL_FIFO);
		fmt_write(vdo, FMT_ACTIVE_X,
			  FMT_SHADOW_EN | ox << 16 | (ox + ow - 1));
	} else {
		/* The formatter's window spans the source; H-down places it */
		fmt_write(vdo, FMT_HSCALE, FMT_PHASE_16 | FMT_HSCALE_ONE << 16 |
			  FMT_HSLR | FMT_HSON);
		fmt_write(vdo, FMT_ACTIVE_X, FMT_SHADOW_EN |
			  t->hstart << 16 | (t->hstart + src->w - 1));
	}
	fmt_write(vdo, FMT_ACTIVE_Y0, v);
	fmt_write(vdo, FMT_ACTIVE_Y1, v);

	if (src->w > ow) {
		fmt_write(vdo, FMT_HDWN_IN, FMT_HDWN_2F |
			  (t->hstart + t->hdwn[0]) << 16 |
			  (t->hstart + src->w - 1 + t->hdwn[1]));
		/* MediaTek's driver ends these a line later than the window */
		fmt_write(vdo, FMT_HDWN_VO, oy << 16 | (oy + oh));
		fmt_write(vdo, FMT_HDWN_VE, oy << 16 | (oy + oh));
		fmt_write(vdo, FMT_HDWN_OUT, (ox + t->hdwn[2]) << 16 |
			  (ox + ow - 1 + t->hdwn[3]));
		fifo = vdo->data->id == MTK_VDOUT_VDO_SUB && src->w == 1920;
		fmt_write(vdo, FMT_HDWN, (src->w * FMT_HSCALE_ONE / ow) << 16 |
			  FMT_HDWN_444 | (fifo ? FMT_HDWN_FULL_FIFO : 0) |
			  FMT_HDWN_EN);
	}

	mtk_vdout_video_window(vdo->vd, vdo->data->id,
			       ox << 16 | (ox + ow - 1), v);
}

/* vdp_hal_config(): the VDO core, the UFOD and SCL, written whole */
static void mtk_vdout_vdo_dma_update(struct mtk_vdout_vdo *vdo,
				     const struct drm_plane_state *state,
				     const struct mtk_vdout_vdo_src *src)
{
	struct drm_framebuffer *fb = state->fb;
	bool yuv422 = fb->format->vsub == 1;
	u32 pitch = fb->pitches[0];
	u32 hb = DIV_ROUND_UP(pitch, 8);
	/* 4-pixel words to the line's end; past the source is still the pitch */
	u32 dw = DIV_ROUND_UP(src->x + src->w, 4);
	u32 oh = drm_rect_height(&state->dst);
	dma_addr_t addr[2];
	int i;

	for (i = 0; i < 2; i++)
		addr[i] = drm_fb_dma_get_gem_obj(fb, i)->dma_addr +
			  fb->offsets[i] +
			  src->y / (i ? fb->format->vsub : 1) * pitch;

	writel((hb & 0xff) | (dw & 0xff) << 8 | src->h << 16 |
	       (hb >> 8 & 3) << 28 | (dw >> 10 & 1) << 30,
	       vdo->regs + VDO_SIZE);
	writel(VDO_SHADOW_MODE | (src->x & ~1) << 16 |
	       (src->h * VDO_VSCALE_ONE / oh),
	       vdo->regs + VDO_SCALE);
	writel(VDO_CLR | VDO_EN, vdo->regs + VDO_CTRL);
	/* From line 0, where MediaTek's driver repeats the first chroma line */
	writel(0, vdo->regs + VDO_YSLTT);
	writel(1, vdo->regs + VDO_CSLTT);
	writel(0x80, vdo->regs + VDO_CSSLTT);
	writel(yuv422 ? VDO_YUV422 : 0, vdo->regs + VDO_FORMAT);
	writel(VDO_YFIR_ON, vdo->regs + VDO_YFIR);
	writel(VDO_OPT8, vdo->regs + VDO_OPT);
	writel((dw & 0x1ff) | (dw >> 9 & 1) << 27, vdo->regs + VDO_DW_HD);

	writel(addr[0], vdo->regs + UFOD_Y);
	writel(addr[1], vdo->regs + UFOD_C);
	writel(ALIGN(src->h, 4) << 20 | DIV_ROUND_UP(pitch, 2),
	       vdo->regs + UFOD_SIZE);
	writel(DIV_ROUND_UP(pitch, 16), vdo->regs + UFOD_PITCH);
	writel(0, vdo->regs + UFOD_Y_LENGTH);
	writel(0, vdo->regs + UFOD_C_LENGTH);
	writel((fb->modifier == VDO_MOD_BLOCK ? 0 : UFOD_RASTER) |
	       UFOD_PACK_8BIT | (pitch > 1024 ? UFOD_UHD4K : 0) |
	       (yuv422 ? UFOD_YUV422 : 0), vdo->regs + UFOD_MODE);
	writel(5, vdo->regs + UFOD_TH);
	writel(vdo->data->ufod_ultra, vdo->regs + UFOD_ULTRA);
	writel(UFOD_ENABLE, vdo->regs + UFOD_EN);

	writel(SCL_REG_OFFSET | 0x40, vdo->regs + SCL_LINE_BUF);

	vdo_trigger(vdo);
}

static bool mtk_vdout_vdo_cr_first(const struct drm_framebuffer *fb)
{
	return fb->format->format == DRM_FORMAT_NV21 ||
	       fb->format->format == DRM_FORMAT_NV61;
}

static void mtk_vdout_vdo_atomic_update(struct drm_plane *plane,
					struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct mtk_vdout_vdo *vdo = to_mtk_vdout_vdo(plane);
	const struct mtk_vdout_vdo_timing *t = vdo->vd->cur_vdo;
	struct mtk_vdout_vdo_src src;
	int ret;

	if (!new_state->visible) {
		mtk_vdout_vdo_off(vdo);
		return;
	}

	/* Set up again for a new mode */
	if (vdo->timing != t)
		mtk_vdout_vdo_off(vdo);
	if (!vdo->timing) {
		ret = mtk_vdout_vdo_on(vdo, t);
		if (ret) {
			dev_err(vdo->dev, "failed to power on: %d\n", ret);
			return;
		}
	}

	mtk_vdout_vdo_get_src(new_state, &src);
	mtk_vdout_vdo_fmt_update(vdo, t, &src, &new_state->dst);
	mtk_vdout_vdo_dma_update(vdo, new_state, &src);

	rmw(vdo->regs, FMT_CTRL, FMT_UVSW | FMT_VDO_EN,
	    (mtk_vdout_vdo_cr_first(new_state->fb) ? FMT_UVSW : 0) | FMT_VDO_EN);
	rmw(vdo->regs, FMT_ACTIVE_X, FMT_TRIGGER, FMT_TRIGGER);
	mtk_vdout_video_mix(vdo->vd, vdo->data->id, true);
}

static void mtk_vdout_vdo_atomic_disable(struct drm_plane *plane,
					 struct drm_atomic_commit *state)
{
	mtk_vdout_vdo_off(to_mtk_vdout_vdo(plane));
}

static const struct drm_plane_helper_funcs mtk_vdout_vdo_helper_funcs = {
	.atomic_check = mtk_vdout_vdo_atomic_check,
	.atomic_update = mtk_vdout_vdo_atomic_update,
	.atomic_disable = mtk_vdout_vdo_atomic_disable,
};

static bool mtk_vdout_vdo_format_mod_supported(struct drm_plane *plane,
					       u32 format, u64 modifier)
{
	/* The decoder's 4:2:0 block layout, Cb first */
	if (modifier == VDO_MOD_BLOCK)
		return format == DRM_FORMAT_NV12;

	return modifier == DRM_FORMAT_MOD_LINEAR;
}

static const struct drm_plane_funcs mtk_vdout_vdo_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.atomic_create_state = drm_atomic_helper_plane_create_state,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
	.destroy = drm_plane_cleanup,
	.format_mod_supported = mtk_vdout_vdo_format_mod_supported,
};

/* The registers read only while the plane is powered, and kept so */
void mtk_vdout_vdo_dump(struct seq_file *s, struct mtk_vdout_vdo *vdo)
{
	unsigned int off;

	if (pm_runtime_get_if_active(vdo->dev) <= 0)
		return;
	seq_printf(s, "## VDO %s\n",
		   vdo->data->id == MTK_VDOUT_VDO_MAIN ? "main" : "sub");
	for (off = 0; off < 0x500; off += 4)
		seq_printf(s, "%04x: %08x\n", off, readl(vdo->regs + off));
	pm_runtime_put(vdo->dev);
}

static int mtk_vdout_vdo_bind(struct device *dev, struct device *master,
			      void *data)
{
	struct mtk_vdout_vdo *vdo = dev_get_drvdata(dev);
	struct mtk_vdout *vd = data;
	int ret;

	ret = drm_universal_plane_init(vd->drm, &vdo->plane, BIT(0),
				       &mtk_vdout_vdo_funcs,
				       mtk_vdout_vdo_formats,
				       ARRAY_SIZE(mtk_vdout_vdo_formats),
				       mtk_vdout_vdo_modifiers,
				       DRM_PLANE_TYPE_OVERLAY, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&vdo->plane, &mtk_vdout_vdo_helper_funcs);

	/* The mixer stacks main, sub and the OSD premix in a fixed order */
	ret = drm_plane_create_zpos_immutable_property(&vdo->plane,
						       vdo->data->id);
	if (ret)
		return ret;

	/* The mixer and RGB2HDMI take everything as limited-range BT.709 */
	ret = drm_plane_create_color_properties(&vdo->plane,
						BIT(DRM_COLOR_YCBCR_BT709),
						BIT(DRM_COLOR_YCBCR_LIMITED_RANGE),
						DRM_COLOR_YCBCR_BT709,
						DRM_COLOR_YCBCR_LIMITED_RANGE);
	if (ret)
		return ret;

	vdo->vd = vd;
	vd->vdo[vdo->data->id] = vdo;

	return 0;
}

static void mtk_vdout_vdo_unbind(struct device *dev, struct device *master,
				 void *data)
{
	struct mtk_vdout_vdo *vdo = dev_get_drvdata(dev);
	struct mtk_vdout *vd = data;

	vd->vdo[vdo->data->id] = NULL;
	vdo->vd = NULL;
}

static const struct component_ops mtk_vdout_vdo_component_ops = {
	.bind = mtk_vdout_vdo_bind,
	.unbind = mtk_vdout_vdo_unbind,
};

static int mtk_vdout_vdo_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_vdout_vdo *vdo;
	int ret;

	vdo = devm_kzalloc(dev, sizeof(*vdo), GFP_KERNEL);
	if (!vdo)
		return -ENOMEM;
	vdo->dev = dev;
	vdo->data = device_get_match_data(dev);

	vdo->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vdo->regs))
		return PTR_ERR(vdo->regs);

	vdo->dispsys = syscon_regmap_lookup_by_phandle(dev->of_node,
						       "mediatek,dispsys");
	if (IS_ERR(vdo->dispsys))
		return dev_err_probe(dev, PTR_ERR(vdo->dispsys), "no dispsys\n");

	vdo->clk_vdo = devm_clk_get(dev, "vdo");
	if (IS_ERR(vdo->clk_vdo))
		return dev_err_probe(dev, PTR_ERR(vdo->clk_vdo), "no vdo clock\n");
	vdo->clk_fmt = devm_clk_get(dev, "fmt");
	if (IS_ERR(vdo->clk_fmt))
		return dev_err_probe(dev, PTR_ERR(vdo->clk_fmt), "no fmt clock\n");

	/* Buffers come through the M4U, in the OSD planes' domain */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/* Powered while the plane shows; the larb comes up through the M4U */
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, vdo);

	return component_add(dev, &mtk_vdout_vdo_component_ops);
}

static void mtk_vdout_vdo_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &mtk_vdout_vdo_component_ops);
}

static const struct mtk_vdout_vdo_data mtk_vdout_vdo_main = {
	.id = MTK_VDOUT_VDO_MAIN,
	.reset = BIT(0),
	.prio_reg = DISPSYS_PRIO_MAIN,
	.prio = BIT(30) | BIT(31),		/* Y and C ultra */
	.ufod_ultra = 0x3f,			/* ultra, threshold 0x1f */
	.fmt_delay = true,
};

static const struct mtk_vdout_vdo_data mtk_vdout_vdo_sub = {
	.id = MTK_VDOUT_VDO_SUB,
	.reset = BIT(7),
	.prio_reg = DISPSYS_PRIO_SUB,
	.prio = BIT(28) | BIT(29),		/* Y and C pre-ultra */
	.ufod_ultra = 0xfc0,			/* pre-ultra, threshold 0x1f */
};

static const struct of_device_id mtk_vdout_vdo_of_match[] = {
	{ .compatible = "mediatek,mt8695-vdo-main", .data = &mtk_vdout_vdo_main },
	{ .compatible = "mediatek,mt8695-vdo-sub", .data = &mtk_vdout_vdo_sub },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_vdout_vdo_of_match);

struct platform_driver mtk_vdout_vdo_driver = {
	.probe = mtk_vdout_vdo_probe,
	.remove = mtk_vdout_vdo_remove,
	.driver = {
		.name = "mtk-vdout-vdo",
		.of_match_table = mtk_vdout_vdo_of_match,
	},
};
