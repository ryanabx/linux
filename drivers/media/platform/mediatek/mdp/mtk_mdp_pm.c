// SPDX-License-Identifier: GPL-2.0-only
/* Keep the DMA component suppliers powered for the master runtime lifetime. */
#include <linux/component.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

struct mtk_mdp_pm_component {
	struct device_link *link;
};

static int mtk_mdp_pm_bind(struct device *dev, struct device *master, void *data)
{
	struct mtk_mdp_pm_component *comp = dev_get_drvdata(dev);

	/* The component already has its IOMMU/LARB runtime-PM supplier link. */
	comp->link = device_link_add(master, dev,
				     DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
	return comp->link ? 0 : -ENOMEM;
}

static void mtk_mdp_pm_unbind(struct device *dev, struct device *master,
			      void *data)
{
	struct mtk_mdp_pm_component *comp = dev_get_drvdata(dev);

	device_link_del(comp->link);
	comp->link = NULL;
}

static const struct component_ops mtk_mdp_pm_component_ops = {
	.bind = mtk_mdp_pm_bind,
	.unbind = mtk_mdp_pm_unbind,
};

static int mtk_mdp_pm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_mdp_pm_component *comp;
	int ret;

	/* RDMA0 is the master; it must not become its own supplier. */
	if (of_property_present(dev->of_node, "mediatek,vpu"))
		return -ENODEV;
	comp = devm_kzalloc(dev, sizeof(*comp), GFP_KERNEL);
	if (!comp)
		return -ENOMEM;
	platform_set_drvdata(pdev, comp);
	pm_runtime_enable(dev);
	ret = component_add(dev, &mtk_mdp_pm_component_ops);
	if (ret)
		pm_runtime_disable(dev);
	return ret;
}

static void mtk_mdp_pm_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &mtk_mdp_pm_component_ops);
	pm_runtime_disable(&pdev->dev);
}

static const struct of_device_id mtk_mdp_pm_of_match[] = {
	{ .compatible = "mediatek,mt8173-mdp-rdma" },
	{ .compatible = "mediatek,mt8173-mdp-rsz" },
	{ .compatible = "mediatek,mt8173-mdp-wdma" },
	{ .compatible = "mediatek,mt8173-mdp-wrot" },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_mdp_pm_of_match);

struct platform_driver mtk_mdp_pm_component_driver = {
	.probe = mtk_mdp_pm_probe,
	.remove = mtk_mdp_pm_remove,
	.driver = {
		.name = "mtk-mdp-pm-component",
		.of_match_table = mtk_mdp_pm_of_match,
	},
};
