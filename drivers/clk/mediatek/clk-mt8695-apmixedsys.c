// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017 MediaTek Inc.
 * Author: Chen Zhong <chen.zhong@mediatek.com>
 *
 * Clock tables from the MT8695 vendor kernel; driver structure follows
 * clk-mt2712{,-apmixedsys}.c.
 */

#include <linux/clk-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-pll.h"

#include <dt-bindings/clock/mediatek,mt8695-clk.h>

#define MT8695_PLL_FMAX		(3800UL * MHZ)

/*
 * _pcw_chg_reg is CON0 (_reg) for the PLLs whose PCW change bit sits there
 * rather than in the PCW register; 0 keeps the default (CON1).
 */
#define PLL_FMIN(_id, _name, _reg, _pwr_reg, _en_mask, _flags,		\
		 _rst_bar_bit, _pcwbits, _pcwibits, _pd_reg, _pd_shift,	\
		 _tuner_reg, _tuner_en_reg, _tuner_en_bit, _pcw_reg,	\
		 _pcw_shift, _pcw_chg_reg, _div_table, _fmin) {		\
		.id = _id,						\
		.name = _name,						\
		.reg = _reg,						\
		.pwr_reg = _pwr_reg,					\
		.en_mask = _en_mask,					\
		.flags = _flags,					\
		.rst_bar_mask = BIT(_rst_bar_bit),			\
		.fmax = MT8695_PLL_FMAX,				\
		.pcwbits = _pcwbits,					\
		.pcwibits = _pcwibits,					\
		.pd_reg = _pd_reg,					\
		.pd_shift = _pd_shift,					\
		.tuner_reg = _tuner_reg,				\
		.tuner_en_reg = _tuner_en_reg,				\
		.tuner_en_bit = _tuner_en_bit,				\
		.pcw_reg = _pcw_reg,					\
		.pcw_shift = _pcw_shift,				\
		.pcw_chg_reg = _pcw_chg_reg,				\
		.div_table = _div_table,				\
		.fmin = _fmin,						\
	}

#define PLL(_id, _name, _reg, _pwr_reg, _en_mask, _flags, _rst_bar_bit,	\
	    _pcwbits, _pcwibits, _pd_reg, _pd_shift, _tuner_reg,		\
	    _tuner_en_reg, _tuner_en_bit, _pcw_reg, _pcw_shift,		\
	    _pcw_chg_reg, _div_table)					\
	PLL_FMIN(_id, _name, _reg, _pwr_reg, _en_mask, _flags,		\
		 _rst_bar_bit, _pcwbits, _pcwibits, _pd_reg, _pd_shift,	\
		 _tuner_reg, _tuner_en_reg, _tuner_en_bit, _pcw_reg,	\
		 _pcw_shift, _pcw_chg_reg, _div_table, 0)

/*
 * The audio PLLs' VCO needs at least 2 GHz: at 196.608 MHz with the
 * default 1 GHz minimum (a 1.57 GHz VCO, post divider 8), APLL1 ran at
 * twice the rate it reported. The preloader leaves a 3.15 GHz VCO and
 * post divider 16, and so does this minimum.
 */
#define MT8695_APLL_FMIN	(2000UL * MHZ)

static const struct mtk_pll_div_table armpll_div_table[] = {
	{ .div = 0, .freq = MT8695_PLL_FMAX },
	{ .div = 1, .freq = 1800500000 },
	{ .div = 2, .freq = 750000000 },
	{ .div = 3, .freq = 325000000 },
	{ .div = 4, .freq = 157625000 },
	{ } /* sentinel */
};

static const struct mtk_pll_div_table common_div_table[] = {
	{ .div = 0, .freq = MT8695_PLL_FMAX },
	{ .div = 1, .freq = 1601000000 },
	{ .div = 2, .freq = 949250000 },
	{ .div = 3, .freq = 250250000 },
	{ .div = 4, .freq = 125125000 },
	{ } /* sentinel */
};

static const struct mtk_pll_data plls[] = {
	PLL(CLK_APMIXED_ARMPLL, "armpll", 0x0110, 0x011C, 0x00000101,
	    HAVE_RST_BAR | PLL_AO, 25, 22, 8, 0x0114, 24, 0, 0, 0, 0x0114, 0,
	    0, armpll_div_table),
	PLL(CLK_APMIXED_MAINPLL, "mainpll", 0x0120, 0x012C, 0x00000101, HAVE_RST_BAR, 20, 32, 8,
	    0x0120, 4, 0, 0, 0, 0x0124, 0,
	    0x0120, NULL),
	PLL(CLK_APMIXED_UNIV2PLL, "univ2pll", 0x0130, 0x013C, 0x1b000101, HAVE_RST_BAR, 20, 32, 8,
	    0x0130, 4, 0, 0, 0, 0x0134, 0,
	    0x0130, NULL),
	PLL(CLK_APMIXED_MMPLL, "mmpll", 0x0140, 0x014C, 0x00000101, 0, 0, 22, 8,
	    0x0144, 24, 0, 0, 0, 0x0144, 0,
	    0, common_div_table),
	PLL(CLK_APMIXED_MSDCPLL, "msdcpll", 0x0150, 0x015C, 0x00000101, 0, 0, 32, 8,
	    0x0150, 4, 0, 0, 0, 0x0154, 0,
	    0x0150, common_div_table),
	PLL(CLK_APMIXED_TVDPLL, "tvdpll", 0x0170, 0x017C, 0x00000101, 0, 0, 32, 8,
	    0x0170, 4, 0, 0, 0, 0x0174, 0,
	    0x0170, common_div_table),
	PLL(CLK_APMIXED_ETHERPLL, "etherpll", 0x0180, 0x018C, 0x00000101, 0, 0, 32, 8,
	    0x0180, 4, 0, 0, 0, 0x0184, 0,
	    0x0180, common_div_table),
	PLL(CLK_APMIXED_VDECPLL, "vdecpll", 0x0190, 0x019C, 0x00000101, 0, 0, 32, 8,
	    0x0190, 4, 0, 0, 0, 0x0194, 0,
	    0x0190, common_div_table),
	PLL(CLK_APMIXED_OSDPLL, "osdpll", 0x0200, 0x020C, 0x00000101, 0, 0, 32, 8,
	    0x0200, 4, 0, 0, 0, 0x0204, 0,
	    0x0200, common_div_table),
	PLL_FMIN(CLK_APMIXED_APLL1, "apll1", 0x0210, 0x0220, 0x00000101, 0, 0, 32, 8,
		 0x0210, 4, 0x0218, 0x0014, 0, 0x0214, 0,
		 0x0210, NULL, MT8695_APLL_FMIN),
	PLL_FMIN(CLK_APMIXED_APLL2, "apll2", 0x0224, 0x0234, 0x00000101, 0, 0, 32, 8,
		 0x0224, 4, 0x022C, 0x0014, 1, 0x0228, 0,
		 0x0224, NULL, MT8695_APLL_FMIN),
};

static const char * const hdmitx20_cksel_parents[] = {
	"tvdpll_d8",
	"osdpll_d12"
};

static const struct mtk_composite apmixed_muxes[] = {
	/* HDMITX20_CKSEL */
	MUX(CLK_APMIXED_HDMITX20_CKSEL, "hdmitx20_cksel", hdmitx20_cksel_parents, 0x48, 16, 1),
};

static DEFINE_SPINLOCK(mt8695_apmixed_lock);

static int clk_mt8695_apmixed_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct clk_hw_onecell_data *clk_data;
	void __iomem *base;
	int r;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	clk_data = mtk_devm_alloc_clk_data(&pdev->dev,
					   ARRAY_SIZE(plls) + ARRAY_SIZE(apmixed_muxes));
	if (!clk_data)
		return -ENOMEM;

	r = mtk_clk_register_plls(&pdev->dev, plls, ARRAY_SIZE(plls), clk_data);
	if (r)
		return r;

	r = mtk_clk_register_composites(&pdev->dev, apmixed_muxes, ARRAY_SIZE(apmixed_muxes),
					base, &mt8695_apmixed_lock, clk_data);
	if (r)
		goto unregister_plls;

	r = of_clk_add_hw_provider(node, of_clk_hw_onecell_get, clk_data);
	if (r)
		goto unregister_muxes;

	platform_set_drvdata(pdev, clk_data);
	return 0;

unregister_muxes:
	mtk_clk_unregister_composites(apmixed_muxes, ARRAY_SIZE(apmixed_muxes), clk_data);
unregister_plls:
	mtk_clk_unregister_plls(plls, ARRAY_SIZE(plls), clk_data);
	return r;
}

static void clk_mt8695_apmixed_remove(struct platform_device *pdev)
{
	struct clk_hw_onecell_data *clk_data = platform_get_drvdata(pdev);

	of_clk_del_provider(pdev->dev.of_node);
	mtk_clk_unregister_composites(apmixed_muxes, ARRAY_SIZE(apmixed_muxes), clk_data);
	mtk_clk_unregister_plls(plls, ARRAY_SIZE(plls), clk_data);
}

static const struct of_device_id of_match_clk_mt8695_apmixed[] = {
	{ .compatible = "mediatek,mt8695-apmixedsys" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt8695_apmixed);

static struct platform_driver clk_mt8695_apmixed_drv = {
	.probe = clk_mt8695_apmixed_probe,
	.remove = clk_mt8695_apmixed_remove,
	.driver = {
		.name = "clk-mt8695-apmixed",
		.of_match_table = of_match_clk_mt8695_apmixed,
	},
};
module_platform_driver(clk_mt8695_apmixed_drv);

MODULE_DESCRIPTION("MediaTek MT8695 apmixedsys clocks driver");
MODULE_LICENSE("GPL");
