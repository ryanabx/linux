// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek VDOUT: the OSD planes
 *
 * Each OSD plane reads a chain of region headers from memory; each header
 * describes one buffer. One region is used here, in ARGB8888, and the
 * plane's scaler takes it to a window of the screen. A plane latches its
 * registers and the header on every vsync from VDOUT. The FHD plane (OSD3)
 * is the primary plane; the UHD plane (OSD2) is mixed above it, with
 * premultiplied alpha.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_plane.h>
#include <drm/drm_rect.h>

#include "mtk_vdout_drv.h"

/* Timing at 0x000, plane at 0x100, scaler at 0x200 */
#define OSD_UPDATE		0x000
#define  OSD_UPDATE_EN		BIT(0)
#define  OSD_ALWAYS_UPDATE	BIT(1)
#define  OSD_SYNC_VDOUT		BIT(3)	/* timing from the plane's VDOUT pulse */
#define OSD_PRGS		0x008
#define OSD_SYNC		0x00c
#define OSD_UHD_HSTART		0x010	/* [9:0] */
#define  OSD_UHD_HSTART_MASK	GENMASK(9, 0)
#define OSD_START		0x014	/* FHD: hstart [25:16] */
#define OSD_VSTART		0x018
#define OSD_SIZE		0x01c
#define OSD_HTOTAL		0x064
#define OSD_ALWAYS_UPDATE_REG	0x070
#define OSD_PLA_EN		0x100
#define OSD_PLA_HEADER		0x104
#define OSD_AUTO_ULTRA		0x1bc
#define OSD_SYNC_THRES		0x1e4
#define OSD_UHD_POS		0x024	/* x [27:16], y [11:0] */
#define OSD_FHD_POS		0x028
#define OSD_SC_CFG		0x200
#define  OSD_SC_VUSC_EN		BIT(2)
#define  OSD_SC_VDSC_EN		BIT(3)
#define  OSD_SC_HUSC_EN		BIT(4)
#define  OSD_SC_HDSC_EN		BIT(5)
#define  OSD_SC_EN		BIT(7)
#define  OSD_SC_BIT26		BIT(26)	/* undocumented, set by the bootloader */
#define OSD_SC_SRC_SIZE		0x204
#define OSD_SC_DST_SIZE		0x208
#define OSD_SC_VSC_WIDTH	0x20c	/* the line width into the vertical scaler */
#define OSD_SC_HDSC		0x210	/* offset [29:16], step [13:0] */
#define OSD_SC_HUSC		0x214
#define OSD_SC_VSC_OFST		0x218	/* top [29:16], bottom [13:0] */
#define OSD_SC_VSC_STEP		0x21c

/* Scaler steps are 1.14 fixed point */
#define OSD_SC_STEP_SHIFT	14

/* Up to 8x up, 2x down: the vendor never enables the scaler's filter */
#define OSD_MIN_SCALE		(DRM_PLANE_NO_SCALING / 8)
#define OSD_MAX_SCALE		(DRM_PLANE_NO_SCALING * 2)
#define OSD_MAX_LINE		1920

#define OSD_HDR_WORDS		16
#define OSD_HDR_SIZE		(OSD_HDR_WORDS * sizeof(u32))

struct reg_val {
	u16 off;
	u32 val;
};

/*
 * The differences between the planes: register placement, and what the
 * bootloader leaves set up (it uses the FHD plane only)
 */
struct mtk_vdout_osd_data {
	enum mtk_vdout_osd_id id;
	enum drm_plane_type type;
	u32 prgs;		/* progressive, vsync edge, auto switch */
	u32 pos;		/* the window position register */
	bool auto_ultra;
	const struct reg_val *boot_regs;
	unsigned int num_boot_regs;
};

/* Base, plane and scaler registers that don't depend on the mode; 1:1 */
static const struct reg_val osd_regs[] = {
	{ 0x038, 0x80008000 }, { 0x03c, 0x80008000 },
	{ 0x070, 0x00000040 }, { 0x0a4, 0x01f186c0 },
	{ 0x10c, 0x078f3fcf }, { 0x108, 0x0038ffff },
	{ 0x200, 0x04000080 }, { 0x210, 0x00000000 }, { 0x214, 0x00000000 },
	{ 0x218, 0x20002000 }, { 0x21c, 0x00000000 }, { 0x220, 0x00000000 },
};

/* Left by the bootloader: a second scaler and the RGB-to-YCbCr matrix */
static const struct reg_val osd_fhd_boot_regs[] = {
	{ 0x280, 0x04000084 }, { 0x284, 0x07800438 }, { 0x288, 0x07800438 },
	{ 0x28c, 0x00000780 }, { 0x298, 0x20002000 }, { 0x29c, 0x00004000 },
	{ 0x2c0, 0x00004000 }, { 0x2c4, 0x00004000 }, { 0x2c8, 0x0800d000 },
	{ 0x2cc, 0x0c500764 }, { 0x2d0, 0x02440757 }, { 0x2d4, 0x0244000f },
	{ 0x2d8, 0x02001003 },
};

static const u32 mtk_vdout_osd_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

static const u64 mtk_vdout_osd_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

static inline struct mtk_vdout_osd *to_mtk_vdout_osd(struct drm_plane *plane)
{
	return container_of(plane, struct mtk_vdout_osd, plane);
}

static void rmw(void __iomem *base, u32 off, u32 mask, u32 val)
{
	writel((readl(base + off) & ~mask) | (val & mask), base + off);
}

static void write_table(void __iomem *base, const struct reg_val *t,
			unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		writel(t[i].val, base + t[i].off);
}

/* Called by the CRTC with the mode the timing generator runs */
void mtk_vdout_osd_setup(struct mtk_vdout_osd *osd,
			 const struct drm_display_mode *mode,
			 const struct mtk_vdout_osd_timing *t)
{
	u32 size = mode->hdisplay << 16 | mode->vdisplay;

	const struct mtk_vdout_osd_data *data = osd->data;

	rmw(osd->regs, OSD_UPDATE, OSD_UPDATE_EN, 0);

	/* The plane comes back with its next update, for this mode */
	writel(0, osd->regs + OSD_PLA_EN);

	write_table(osd->regs, osd_regs, ARRAY_SIZE(osd_regs));
	write_table(osd->regs, data->boot_regs, data->num_boot_regs);
	writel(data->prgs, osd->regs + OSD_PRGS);
	writel(0, osd->regs + data->pos);

	/* i4Osd_BaseSetFmt() and the 1:1 scaler for the mode */
	writel(t->sync, osd->regs + OSD_SYNC);
	if (data->id == MTK_VDOUT_OSD_UHD) {
		rmw(osd->regs, OSD_UHD_HSTART, OSD_UHD_HSTART_MASK, t->start >> 16);
	} else {
		writel(t->start, osd->regs + OSD_START);
		/* MediaTek's driver sets the threshold for this plane only */
		writel(t->sync_thres, osd->regs + OSD_SYNC_THRES);
	}
	writel(t->vstart, osd->regs + OSD_VSTART);
	writel(size, osd->regs + OSD_SIZE);
	writel(t->htotal, osd->regs + OSD_HTOTAL);
	if (data->auto_ultra)
		writel(0x100080ff, osd->regs + OSD_AUTO_ULTRA);
	writel(size, osd->regs + OSD_SC_SRC_SIZE);
	writel(size, osd->regs + OSD_SC_DST_SIZE);
	writel(mode->hdisplay, osd->regs + OSD_SC_VSC_WIDTH);

	/* Latch on every vsync, so updates need no interrupt handling */
	rmw(osd->regs, OSD_UPDATE, OSD_SYNC_VDOUT | OSD_ALWAYS_UPDATE,
	    OSD_SYNC_VDOUT | OSD_ALWAYS_UPDATE);
	writel(0x40, osd->regs + OSD_ALWAYS_UPDATE_REG);
	rmw(osd->regs, OSD_UPDATE, OSD_UPDATE_EN, OSD_UPDATE_EN);
}

void mtk_vdout_osd_off(struct mtk_vdout_osd *osd)
{
	writel(0, osd->regs + OSD_PLA_EN);
}

static void mtk_vdout_osd_write_header(u32 *hdr, struct drm_framebuffer *fb,
				       dma_addr_t addr, u32 w, u32 h, bool alpha)
{
	u32 pa = addr;

	memset(hdr, 0, OSD_HDR_SIZE);
	hdr[0] = 0xe0000000;				/* ARGB8888, last region */
	hdr[1] = 0xff000000 | ((pa >> 4) & 0x00ffffff);
	hdr[2] = 0xe4000000 | (fb->pitches[0] >> 4);	/* A, R, G, B; pitch */
	hdr[4] = 0x1000;				/* 1:1 */
	hdr[5] = 0x1000;
	hdr[6] = ((pa >> 30) & 3) << 2;
	/* Per-pixel alpha (1) or none (2) */
	hdr[7] = ((alpha ? 1 : 2) << 30) | BIT(28) | (((pa >> 28) & 3) << 23);
	hdr[9] = h << 16 | w;
	hdr[10] = h;
	hdr[11] = w;
	dma_wmb();
}

struct mtk_vdout_osd_scaler {
	u32 cfg;
	u32 vsc_width;
	u32 hdsc;
	u32 husc;
	u32 vsc_ofst;
	u32 vsc_step;
};

/*
 * OSD_SC_Scale() in MediaTek's driver: each direction scales up, down or
 * not at all. The scaler runs horizontal downscaling, then vertical
 * scaling, then horizontal upscaling.
 */
static void mtk_vdout_osd_scaler(u32 sw, u32 sh, u32 dw, u32 dh,
				 struct mtk_vdout_osd_scaler *sc)
{
	u32 step;

	sc->cfg = OSD_SC_BIT26 | OSD_SC_EN;
	sc->vsc_width = sw;
	sc->hdsc = 0;
	sc->husc = 0;
	sc->vsc_ofst = 0x20002000;		/* as booted, for no scaling */
	sc->vsc_step = 0;

	if (dw > sw) {
		sc->cfg |= OSD_SC_HUSC_EN;
		sc->husc = ((sw - 1) << OSD_SC_STEP_SHIFT) / (dw - 1);
		/* The vendor's offset for a 1280-pixel source */
		if (sw == 1280)
			sc->husc |= 0x1800 << 16;
	} else if (dw < sw) {
		step = DIV_ROUND_UP(dw << OSD_SC_STEP_SHIFT, sw);
		sc->cfg |= OSD_SC_HDSC_EN;
		sc->hdsc = step << 16 | step;
		sc->vsc_width = dw;
	}

	if (dh > sh) {
		sc->cfg |= OSD_SC_VUSC_EN;
		sc->vsc_ofst = 0;
		sc->vsc_step = ((sh - 1) << OSD_SC_STEP_SHIFT) / (dh - 1);
	} else if (dh < sh) {
		step = DIV_ROUND_UP(dh << OSD_SC_STEP_SHIFT, sh);
		sc->cfg |= OSD_SC_VDSC_EN;
		sc->vsc_ofst = step << 16 | step;
		sc->vsc_step = step;
	}
}

static int mtk_vdout_osd_atomic_check(struct drm_plane *plane,
				      struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_framebuffer *fb = new_state->fb;
	struct drm_crtc_state *crtc_state = NULL;
	u32 offset;
	int ret;

	if (new_state->crtc)
		crtc_state = drm_atomic_get_new_crtc_state(state, new_state->crtc);

	ret = drm_atomic_helper_check_plane_state(new_state, crtc_state,
						  OSD_MIN_SCALE, OSD_MAX_SCALE,
						  true, true);
	if (ret || !new_state->visible)
		return ret;

	/* The line into the vertical scaler: the narrower of source and screen */
	if (min(drm_rect_width(&new_state->src) >> 16,
		drm_rect_width(&new_state->dst)) > OSD_MAX_LINE)
		return -EINVAL;

	/* The header holds the address and the pitch in 16-byte units */
	offset = fb->offsets[0] + (new_state->src.x1 >> 16) * fb->format->cpp[0];
	if (fb->pitches[0] % 16 || offset % 16)
		return -EINVAL;

	return 0;
}

static void mtk_vdout_osd_atomic_update(struct drm_plane *plane,
					struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct mtk_vdout_osd *osd = to_mtk_vdout_osd(plane);
	struct drm_framebuffer *fb = new_state->fb;
	unsigned int idx = osd->hdr_idx ^ 1;
	u32 *hdr = osd->hdr + idx * OSD_HDR_WORDS;
	dma_addr_t hdr_dma = osd->hdr_dma + idx * OSD_HDR_SIZE;
	const struct drm_rect *dst = &new_state->dst;
	struct mtk_vdout_osd_scaler sc;
	u32 sw, sh, dw, dh;
	bool alpha;

	if (!new_state->visible) {
		mtk_vdout_osd_off(osd);
		return;
	}

	/* The header describes the source; the scaler takes it to the window */
	sw = drm_rect_width(&new_state->src) >> 16;
	sh = drm_rect_height(&new_state->src) >> 16;
	dw = drm_rect_width(dst);
	dh = drm_rect_height(dst);
	/* Per-pixel alpha, premultiplied, over the planes below, video too */
	alpha = fb->format->has_alpha &&
		new_state->pixel_blend_mode == DRM_MODE_BLEND_PREMULTI;
	mtk_vdout_osd_write_header(hdr, fb,
				   drm_fb_dma_get_gem_addr(fb, new_state, 0),
				   sw, sh, alpha);
	mtk_vdout_osd_scaler(sw, sh, dw, dh, &sc);

	/* All of it latches together at the next vsync */
	rmw(osd->regs, OSD_UPDATE, OSD_UPDATE_EN, 0);
	writel(hdr_dma >> 4, osd->regs + OSD_PLA_HEADER);
	writel(dst->x1 << 16 | dst->y1, osd->regs + osd->data->pos);
	writel(sc.cfg, osd->regs + OSD_SC_CFG);
	writel(sw << 16 | sh, osd->regs + OSD_SC_SRC_SIZE);
	writel(dw << 16 | dh, osd->regs + OSD_SC_DST_SIZE);
	writel(sc.vsc_width, osd->regs + OSD_SC_VSC_WIDTH);
	writel(sc.hdsc, osd->regs + OSD_SC_HDSC);
	writel(sc.husc, osd->regs + OSD_SC_HUSC);
	writel(sc.vsc_ofst, osd->regs + OSD_SC_VSC_OFST);
	writel(sc.vsc_step, osd->regs + OSD_SC_VSC_STEP);
	writel(1, osd->regs + OSD_PLA_EN);
	mtk_vdout_premix_alpha(osd->vd, osd->data->id, alpha);
	rmw(osd->regs, OSD_UPDATE, OSD_UPDATE_EN, OSD_UPDATE_EN);
	osd->hdr_idx = idx;
}

static void mtk_vdout_osd_atomic_disable(struct drm_plane *plane,
					 struct drm_atomic_commit *state)
{
	mtk_vdout_osd_off(to_mtk_vdout_osd(plane));
}

static const struct drm_plane_helper_funcs mtk_vdout_osd_helper_funcs = {
	.atomic_check = mtk_vdout_osd_atomic_check,
	.atomic_update = mtk_vdout_osd_atomic_update,
	.atomic_disable = mtk_vdout_osd_atomic_disable,
};

static const struct drm_plane_funcs mtk_vdout_osd_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
	.destroy = drm_plane_cleanup,
};

static int mtk_vdout_osd_bind(struct device *dev, struct device *master,
			      void *data)
{
	struct mtk_vdout_osd *osd = dev_get_drvdata(dev);
	struct mtk_vdout *vd = data;
	int ret;

	ret = drm_universal_plane_init(vd->drm, &osd->plane, BIT(0),
				       &mtk_vdout_osd_funcs,
				       mtk_vdout_osd_formats,
				       ARRAY_SIZE(mtk_vdout_osd_formats),
				       mtk_vdout_osd_modifiers,
				       osd->data->type, NULL);
	if (ret)
		return ret;
	drm_plane_helper_add(&osd->plane, &mtk_vdout_osd_helper_funcs);

	ret = drm_plane_create_blend_mode_property(&osd->plane,
						   BIT(DRM_MODE_BLEND_PIXEL_NONE) |
						   BIT(DRM_MODE_BLEND_PREMULTI));
	if (ret)
		return ret;

	/* The premix stacks the planes in a fixed order, above the video */
	ret = drm_plane_create_zpos_immutable_property(&osd->plane,
						       MTK_VDOUT_VDO_NUM +
						       osd->data->id);
	if (ret)
		return ret;

	osd->vd = vd;
	vd->osd[osd->data->id] = osd;
	return 0;
}

static void mtk_vdout_osd_unbind(struct device *dev, struct device *master,
				 void *data)
{
	struct mtk_vdout_osd *osd = dev_get_drvdata(dev);
	struct mtk_vdout *vd = data;

	vd->osd[osd->data->id] = NULL;
	osd->vd = NULL;
}

static const struct component_ops mtk_vdout_osd_component_ops = {
	.bind = mtk_vdout_osd_bind,
	.unbind = mtk_vdout_osd_unbind,
};

static void mtk_vdout_osd_put(void *dev)
{
	pm_runtime_put(dev);
}

static int mtk_vdout_osd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_vdout_osd *osd;
	struct clk_bulk_data *clks;
	int ret;

	osd = devm_kzalloc(dev, sizeof(*osd), GFP_KERNEL);
	if (!osd)
		return -ENOMEM;
	osd->dev = dev;
	osd->data = device_get_match_data(dev);

	osd->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(osd->regs))
		return PTR_ERR(osd->regs);

	/*
	 * The OSD planes are the display's memory masters: buffers come
	 * through the M4U. On the FHD plane, memory-region only names the
	 * bootloader's splash, kept mapped 1:1 until the first modeset
	 * replaces it.
	 */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	osd->hdr = dmam_alloc_coherent(dev, 2 * OSD_HDR_SIZE, &osd->hdr_dma,
				       GFP_KERNEL);
	if (!osd->hdr)
		return -ENOMEM;

	/* SMI common and the larb come up through the M4U's device links */
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;
	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, mtk_vdout_osd_put, dev);
	if (ret)
		return ret;

	/* Held while the driver is bound, like the rest of the output */
	ret = devm_clk_bulk_get_all_enabled(dev, &clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to enable clocks\n");

	platform_set_drvdata(pdev, osd);

	return component_add(dev, &mtk_vdout_osd_component_ops);
}

static void mtk_vdout_osd_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &mtk_vdout_osd_component_ops);
}

static const struct mtk_vdout_osd_data mtk_vdout_osd_fhd = {
	.id = MTK_VDOUT_OSD_FHD,
	.type = DRM_PLANE_TYPE_PRIMARY,
	.prgs = 0x422,
	.pos = OSD_FHD_POS,
	.boot_regs = osd_fhd_boot_regs,
	.num_boot_regs = ARRAY_SIZE(osd_fhd_boot_regs),
};

static const struct mtk_vdout_osd_data mtk_vdout_osd_uhd = {
	.id = MTK_VDOUT_OSD_UHD,
	.type = DRM_PLANE_TYPE_OVERLAY,
	.prgs = 0x412,
	.pos = OSD_UHD_POS,
	.auto_ultra = true,
};

static const struct of_device_id mtk_vdout_osd_of_match[] = {
	{ .compatible = "mediatek,mt8695-osd-fhd", .data = &mtk_vdout_osd_fhd },
	{ .compatible = "mediatek,mt8695-osd-uhd", .data = &mtk_vdout_osd_uhd },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_vdout_osd_of_match);

struct platform_driver mtk_vdout_osd_driver = {
	.probe = mtk_vdout_osd_probe,
	.remove = mtk_vdout_osd_remove,
	.driver = {
		.name = "mtk-vdout-osd",
		.of_match_table = mtk_vdout_osd_of_match,
	},
};
