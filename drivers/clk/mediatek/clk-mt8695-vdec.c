// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017 MediaTek Inc.
 * Author: Chen Zhong <chen.zhong@mediatek.com>
 */

#include <linux/clk-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-gate.h"

#include <dt-bindings/clock/mediatek,mt8695-clk.h>

/* In both blocks a set bit means the clock runs */
static const struct mtk_gate_regs vdcore0_cg_regs = {
	.set_ofs = 0x0,
	.clr_ofs = 0x4,
	.sta_ofs = 0x0,
};

static const struct mtk_gate_regs vdcore1_cg_regs = {
	.set_ofs = 0x8,
	.clr_ofs = 0xc,
	.sta_ofs = 0x8,
};

static const struct mtk_gate_regs vdsoc0_cg_regs = {
	.set_ofs = 0xa0,
	.clr_ofs = 0xa4,
	.sta_ofs = 0xa0,
};

static const struct mtk_gate_regs vdsoc1_cg_regs = {
	.set_ofs = 0xb0,
	.clr_ofs = 0xb4,
	.sta_ofs = 0xb0,
};

#define GATE_VDEC(_id, _name, _parent, _regs, _shift)	\
	GATE_MTK(_id, _name, _parent, _regs, _shift, &mtk_clk_gate_ops_setclr_inv)

static const struct mtk_gate vdcore_clks[] = {
	GATE_VDEC(CLK_VDCORE_LARB1, "vdcore_larb1", "vdec_sel", &vdcore1_cg_regs, 0),
	GATE_VDEC(CLK_VDCORE_VDEC, "vdcore_vdec", "vdec_sel", &vdcore0_cg_regs, 0),
};

static const struct mtk_gate vdsoc_clks[] = {
	GATE_VDEC(CLK_VDSOC_LARB7, "vdsoc_larb7", "vdec_sel", &vdsoc1_cg_regs, 0),
	GATE_VDEC(CLK_VDSOC_SOC, "vdsoc_soc", "vdec_sel", &vdsoc0_cg_regs, 0),
};

static const struct mtk_clk_desc vdcore_desc = {
	.clks = vdcore_clks,
	.num_clks = ARRAY_SIZE(vdcore_clks),
};

static const struct mtk_clk_desc vdsoc_desc = {
	.clks = vdsoc_clks,
	.num_clks = ARRAY_SIZE(vdsoc_clks),
};

static const struct of_device_id of_match_clk_mt8695_vdec[] = {
	{ .compatible = "mediatek,mt8695-vdeccore", .data = &vdcore_desc },
	{ .compatible = "mediatek,mt8695-vdecsoc", .data = &vdsoc_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt8695_vdec);

static struct platform_driver clk_mt8695_vdec_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt8695-vdec",
		.of_match_table = of_match_clk_mt8695_vdec,
	},
};
module_platform_driver(clk_mt8695_vdec_drv);

MODULE_DESCRIPTION("MediaTek MT8695 video decoder clocks driver");
MODULE_LICENSE("GPL");
