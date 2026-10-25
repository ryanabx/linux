// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Copyright (c) 2020 BayLibre, SAS
 * Author: James Liao <jamesjj.liao@mediatek.com>
 *         Fabien Parent <fparent@baylibre.com>
 *
 * Gate bits from the MT8695 vendor kernel's GPU glue (mtk_mfg.c); driver
 * structure follows clk-mt8167-mfgcfg.c.
 */

#include <linux/clk-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-gate.h"

#include <dt-bindings/clock/mediatek,mt8695-clk.h>

static const struct mtk_gate_regs mfg_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x8,
	.sta_ofs = 0x0,
};

#define GATE_MFG(_id, _name, _parent, _shift)	\
	GATE_MTK(_id, _name, _parent, &mfg_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

static const struct mtk_gate mfg_clks[] = {
	GATE_MFG(CLK_MFG_BAXI, "mfg_baxi", "axi_mfg_sel", 0),
	GATE_MFG(CLK_MFG_BMEM, "mfg_bmem", "mem_mfg_sel", 1),
	GATE_MFG(CLK_MFG_BG3D, "mfg_bg3d", "mfg_sel", 2),
	GATE_MFG(CLK_MFG_B26M, "mfg_b26m", "clk26m", 3),
};

/* The registers are in the GPU's 2D power domain */
static const struct mtk_clk_desc mfg_desc = {
	.clks = mfg_clks,
	.num_clks = ARRAY_SIZE(mfg_clks),
	.need_runtime_pm = true,
};

static const struct of_device_id of_match_clk_mt8695_mfgcfg[] = {
	{ .compatible = "mediatek,mt8695-mfgcfg", .data = &mfg_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt8695_mfgcfg);

static struct platform_driver clk_mt8695_mfgcfg_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt8695-mfgcfg",
		.of_match_table = of_match_clk_mt8695_mfgcfg,
	},
};
module_platform_driver(clk_mt8695_mfgcfg_drv);

MODULE_DESCRIPTION("MediaTek MT8695 GPU mfg clocks driver");
MODULE_LICENSE("GPL");
