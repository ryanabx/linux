/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek VDOUT display driver
 */

#ifndef __MTK_VDOUT_DRV_H__
#define __MTK_VDOUT_DRV_H__

#include <linux/clk.h>
#include <linux/types.h>

#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_encoder.h>
#include <drm/drm_plane.h>

struct drm_bridge;
struct platform_driver;
struct regmap;
struct reset_control;
struct mtk_vdout;
struct mtk_vdout_mode;
struct mtk_vdout_osd_data;

/* The OSD planes, bottom to top: they feed the premix's mix1 and mix2 */
enum mtk_vdout_osd_id {
	MTK_VDOUT_OSD_FHD,
	MTK_VDOUT_OSD_UHD,
	MTK_VDOUT_OSD_NUM,
};

/* OSD timing for one mode, relative to the VDOUT OSD-FHD sync pulse */
struct mtk_vdout_osd_timing {
	u32 sync;		/* vertical total, sync widths */
	u32 htotal;
	u32 start;
	u32 vstart;
	u32 sync_thres;
};

/* An OSD plane, a component: FHD (OSD3) the primary, UHD (OSD2) above it */
struct mtk_vdout_osd {
	struct device *dev;
	const struct mtk_vdout_osd_data *data;
	struct mtk_vdout *vd;		/* while bound */
	struct drm_plane plane;
	void __iomem *regs;

	/* Two region headers, so an update never edits the live one */
	u32 *hdr;
	dma_addr_t hdr_dma;
	unsigned int hdr_idx;
};

/* VDOUT: the component master and the CRTC */
struct mtk_vdout {
	struct device *dev;
	struct drm_device *drm;		/* while bound */
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct mtk_vdout_osd *osd[MTK_VDOUT_OSD_NUM];	/* while bound */
	struct drm_bridge *bridge;

	/* FMT, sys1, RGB2HDMI, OSD premix and sys2 */
	void __iomem *regs;
	/* The OSD path's HDR block, on the premix's output */
	void __iomem *hdr_regs;
	struct regmap *mmsys;

	struct clk_bulk_data *clks;
	int num_clks;
	struct clk *tvdpll;
	struct reset_control *rst_rgb2hdmi;
	int irq;

	/* The mode the FMT runs, kept across a disable */
	const struct mtk_vdout_mode *cur;
};

/* The DRM device's parent is the VDOUT device */
static inline struct mtk_vdout *to_mtk_vdout(struct drm_device *drm)
{
	return dev_get_drvdata(drm->dev);
}

int mtk_vdout_crtc_init(struct mtk_vdout *vd, struct drm_plane *primary);

void mtk_vdout_osd_setup(struct mtk_vdout_osd *osd,
			 const struct drm_display_mode *mode,
			 const struct mtk_vdout_osd_timing *t);
void mtk_vdout_osd_off(struct mtk_vdout_osd *osd);
void mtk_vdout_premix_alpha(struct mtk_vdout *vd, enum mtk_vdout_osd_id id,
			    bool premultiplied);

extern struct platform_driver mtk_vdout_osd_driver;

#endif /* __MTK_VDOUT_DRV_H__ */
