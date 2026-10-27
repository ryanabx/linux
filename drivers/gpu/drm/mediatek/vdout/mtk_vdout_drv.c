// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek VDOUT display driver
 *
 * The display output of MediaTek's TV SoCs, starting with the MT8695: OSD
 * and video planes mixed in front of a format timing generator (FMT) that
 * makes the CEA timings, and RGB2HDMI feeding the HDMI TX. None of
 * mediatek-drm's DDP blocks are there, so this is a driver of its own. So
 * far it drives the two OSD planes, FHD (the primary) and UHD (an overlay),
 * and the two video planes below them, main and sub, in the CEA modes from
 * 480p to 1080p60, and the TX is mtk_hdmi_v2, a bridge found through the
 * OF graph.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/seq_file.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_debugfs.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "mtk_vdout_drv.h"

static void mtk_vdout_put(void *dev)
{
	pm_runtime_put(dev);
}

/*
 * Clocks and power stay on while the driver is bound: the TX's HDCP and
 * mute controls are secure-only, and a reset or a power cycle of the MM
 * domain would leave it muted.
 */
static int mtk_vdout_power_on(struct mtk_vdout *vd)
{
	struct clk_bulk_data *clks;
	int ret;

	ret = devm_pm_runtime_enable(vd->dev);
	if (ret)
		return ret;
	ret = pm_runtime_resume_and_get(vd->dev);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(vd->dev, mtk_vdout_put, vd->dev);
	if (ret)
		return ret;

	ret = devm_clk_bulk_get_all_enabled(vd->dev, &clks);
	if (ret < 0)
		return dev_err_probe(vd->dev, ret, "failed to enable clocks\n");

	return reset_control_deassert(vd->rst_rgb2hdmi);
}

static const struct drm_mode_config_funcs mtk_vdout_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

/* The CRTC turns the plane off itself when it is disabled */
static const struct drm_mode_config_helper_funcs mtk_vdout_mode_config_helpers = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

DEFINE_DRM_GEM_DMA_FOPS(mtk_vdout_fops);

static const struct drm_driver mtk_vdout_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.name = "mtk-vdout",
	.desc = "MediaTek VDOUT display",
	.major = 1,
	.minor = 0,
	.fops = &mtk_vdout_fops,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
};

static void mtk_vdout_dump_block(struct seq_file *s, const char *name,
				 void __iomem *base, unsigned int len)
{
	unsigned int off;

	seq_printf(s, "## %s\n", name);
	for (off = 0; off < len; off += 4)
		seq_printf(s, "%04x: %08x\n", off, readl(base + off));
}

static int mtk_vdout_regs_show(struct seq_file *s, void *data)
{
	struct drm_debugfs_entry *entry = s->private;
	struct mtk_vdout *vd = to_mtk_vdout(entry->dev);
	unsigned int i;

	mtk_vdout_dump_block(s, "VDOUT", vd->regs, 0x1000);
	mtk_vdout_dump_block(s, "OSD HDR", vd->hdr_regs, 0x300);
	if (vd->osd[MTK_VDOUT_OSD_FHD])
		mtk_vdout_dump_block(s, "OSD3", vd->osd[MTK_VDOUT_OSD_FHD]->regs, 0x300);
	if (vd->osd[MTK_VDOUT_OSD_UHD])
		mtk_vdout_dump_block(s, "OSD2", vd->osd[MTK_VDOUT_OSD_UHD]->regs, 0x300);
	for (i = 0; i < MTK_VDOUT_VDO_NUM; i++)
		if (vd->vdo[i])
			mtk_vdout_vdo_dump(s, vd->vdo[i]);
	return 0;
}

static int mtk_vdout_kms_init(struct mtk_vdout *vd)
{
	struct drm_device *drm = vd->drm;
	struct drm_connector *connector;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	/* The OSD scaler takes smaller buffers to the screen */
	drm->mode_config.max_width = 1920;
	drm->mode_config.max_height = 1080;
	drm->mode_config.funcs = &mtk_vdout_mode_config_funcs;
	drm->mode_config.helper_private = &mtk_vdout_mode_config_helpers;

	ret = component_bind_all(vd->dev, vd);
	if (ret)
		return ret;
	if (!vd->osd[MTK_VDOUT_OSD_FHD]) {
		component_unbind_all(vd->dev, vd);
		return -ENODEV;
	}

	/* Buffers are for the OSD, which reads them through the M4U */
	drm_dev_set_dma_dev(drm, vd->osd[MTK_VDOUT_OSD_FHD]->dev);

	ret = mtk_vdout_crtc_init(vd, &vd->osd[MTK_VDOUT_OSD_FHD]->plane);
	if (ret)
		goto err_unbind;

	ret = drm_vblank_init(drm, 1);
	if (ret)
		goto err_unbind;

	ret = drmm_encoder_init(drm, &vd->encoder, NULL, DRM_MODE_ENCODER_TMDS,
				NULL);
	if (ret)
		goto err_unbind;
	vd->encoder.possible_crtcs = drm_crtc_mask(&vd->crtc);

	ret = drm_bridge_attach(&vd->encoder, vd->bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret) {
		dev_err(vd->dev, "failed to attach the bridge: %d\n", ret);
		goto err_unbind;
	}

	connector = drm_bridge_connector_init(drm, &vd->encoder);
	if (IS_ERR(connector)) {
		ret = PTR_ERR(connector);
		goto err_unbind;
	}
	ret = drm_connector_attach_encoder(connector, &vd->encoder);
	if (ret)
		goto err_unbind;

	drmm_kms_helper_poll_init(drm);
	drm_mode_config_reset(drm);

	return 0;

err_unbind:
	component_unbind_all(vd->dev, vd);
	return ret;
}

static void mtk_vdout_drm_put(void *drm)
{
	drm_dev_put(drm);
}

static int mtk_vdout_bind(struct device *dev)
{
	struct mtk_vdout *vd = dev_get_drvdata(dev);
	struct drm_device *drm;
	int ret;

	drm = drm_dev_alloc(&mtk_vdout_drm_driver, dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);
	ret = devm_add_action_or_reset(dev, mtk_vdout_drm_put, drm);
	if (ret)
		return ret;
	vd->drm = drm;

	ret = mtk_vdout_kms_init(vd);
	if (ret)
		goto err;

	drm_debugfs_add_file(drm, "regs", mtk_vdout_regs_show, NULL);

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto err_unbind;

	drm_client_setup(drm, NULL);
	return 0;

err_unbind:
	component_unbind_all(dev, vd);
err:
	vd->drm = NULL;
	return ret;
}

static void mtk_vdout_unbind(struct device *dev)
{
	struct mtk_vdout *vd = dev_get_drvdata(dev);

	drm_dev_unregister(vd->drm);
	drm_atomic_helper_shutdown(vd->drm);
	component_unbind_all(dev, vd);
	vd->drm = NULL;
}

static const struct component_master_ops mtk_vdout_master_ops = {
	.bind = mtk_vdout_bind,
	.unbind = mtk_vdout_unbind,
};

/*
 * The planes come in through the OF graph: the OSD planes on ports 0 (FHD)
 * and 1 (UHD), the video planes on 3 (main) and 4 (sub); the TX is on
 * port 2. A plane whose node is disabled is left out.
 */
static int mtk_vdout_add_components(struct device *dev,
				    struct component_match **match)
{
	struct device_node *ep, *remote;

	for_each_endpoint_of_node(dev->of_node, ep) {
		struct of_endpoint id;

		if (of_graph_parse_endpoint(ep, &id) || id.port == 2)
			continue;
		remote = of_graph_get_remote_port_parent(ep);
		if (!remote)
			continue;
		if (of_device_is_available(remote))
			drm_of_component_match_add(dev, match,
						   component_compare_of, remote);
		of_node_put(remote);
	}

	return *match ? 0 : -ENODEV;
}

static int mtk_vdout_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct component_match *match = NULL;
	struct mtk_vdout *vd;
	int ret;

	vd = devm_kzalloc(dev, sizeof(*vd), GFP_KERNEL);
	if (!vd)
		return -ENOMEM;
	vd->dev = dev;
	platform_set_drvdata(pdev, vd);

	vd->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vd->regs))
		return PTR_ERR(vd->regs);
	vd->hdr_regs = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(vd->hdr_regs))
		return PTR_ERR(vd->hdr_regs);

	vd->irq = platform_get_irq(pdev, 0);
	if (vd->irq < 0)
		return vd->irq;

	vd->mmsys = syscon_regmap_lookup_by_phandle(dev->of_node, "mediatek,mmsys");
	if (IS_ERR(vd->mmsys))
		return dev_err_probe(dev, PTR_ERR(vd->mmsys), "no mmsys\n");

	vd->tvdpll = devm_clk_get(dev, "tvdpll");
	if (IS_ERR(vd->tvdpll))
		return dev_err_probe(dev, PTR_ERR(vd->tvdpll), "no tvdpll\n");

	vd->rst_rgb2hdmi = devm_reset_control_get_exclusive(dev, "rgb2hdmi");
	if (IS_ERR(vd->rst_rgb2hdmi))
		return dev_err_probe(dev, PTR_ERR(vd->rst_rgb2hdmi),
				     "no rgb2hdmi reset\n");

	vd->bridge = devm_drm_of_get_bridge(dev, dev->of_node, 2, 0);
	if (IS_ERR(vd->bridge))
		return dev_err_probe(dev, PTR_ERR(vd->bridge), "no HDMI bridge\n");

	ret = mtk_vdout_add_components(dev, &match);
	if (ret)
		return dev_err_probe(dev, ret, "no planes in the OF graph\n");

	ret = mtk_vdout_power_on(vd);
	if (ret)
		return dev_err_probe(dev, ret, "power on failed\n");

	return component_master_add_with_match(dev, &mtk_vdout_master_ops, match);
}

static void mtk_vdout_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &mtk_vdout_master_ops);
}

static void mtk_vdout_shutdown(struct platform_device *pdev)
{
	struct mtk_vdout *vd = platform_get_drvdata(pdev);

	if (vd->drm)
		drm_atomic_helper_shutdown(vd->drm);
}

static const struct of_device_id mtk_vdout_of_match[] = {
	{ .compatible = "mediatek,mt8695-vdout" },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_vdout_of_match);

static struct platform_driver mtk_vdout_driver = {
	.probe = mtk_vdout_probe,
	.remove = mtk_vdout_remove,
	.shutdown = mtk_vdout_shutdown,
	.driver = {
		.name = "mtk-vdout",
		.of_match_table = mtk_vdout_of_match,
	},
};

static struct platform_driver * const mtk_vdout_drivers[] = {
	&mtk_vdout_osd_driver,
	&mtk_vdout_vdo_driver,
	&mtk_vdout_driver,
};

static int __init mtk_vdout_init(void)
{
	return platform_register_drivers(mtk_vdout_drivers,
					 ARRAY_SIZE(mtk_vdout_drivers));
}
module_init(mtk_vdout_init);

static void __exit mtk_vdout_exit(void)
{
	platform_unregister_drivers(mtk_vdout_drivers,
				    ARRAY_SIZE(mtk_vdout_drivers));
}
module_exit(mtk_vdout_exit);

MODULE_DESCRIPTION("MediaTek VDOUT display driver");
MODULE_LICENSE("GPL");
