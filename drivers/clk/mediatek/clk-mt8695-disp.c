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

/* A single register, with a set bit meaning the clock runs */
static const struct mtk_gate_regs disp_cg_regs = {
	.set_ofs = 0xc,
	.clr_ofs = 0xc,
	.sta_ofs = 0xc,
};

#define GATE_DISP(_id, _name, _parent, _shift)	\
	GATE_MTK(_id, _name, _parent, &disp_cg_regs, _shift, &mtk_clk_gate_ops_no_setclr_inv)

static const struct mtk_gate disp_clks[] = {
	GATE_DISP(CLK_DISP_VDO3, "disp_vdo3", "vdo3_sel", 0),
	GATE_DISP(CLK_DISP_FMT3, "disp_fmt3", "mm_sel", 1),
	GATE_DISP(CLK_DISP_MV_HDR2SDR, "disp_mv_hdr2sdr", "mm_sel", 2),
	GATE_DISP(CLK_DISP_MV_BT2020, "disp_mv_bt2020", "mm_sel", 3),
	GATE_DISP(CLK_DISP_SMI_LARB5, "disp_smi_larb5", "mm_sel", 4),
	GATE_DISP(CLK_DISP_SMI_LARB6, "disp_smi_larb6", "mm_sel", 5),
	GATE_DISP(CLK_DISP_DRAMC_LARB8, "disp_dc_larb8", "mm_sel", 6),
	GATE_DISP(CLK_DISP_VDO4, "disp_vdo4", "vdo4_sel", 7),
	GATE_DISP(CLK_DISP_FMT4, "disp_fmt4", "mm_sel", 8),
	GATE_DISP(CLK_DISP_SV_HDR2SDR, "disp_sv_hdr2sdr", "mm_sel", 9),
	GATE_DISP(CLK_DISP_SV_BT2020, "disp_sv_bt2020", "mm_sel", 10),
	GATE_DISP(CLK_DISP_IRT_DMA, "disp_irt_dma", "mm_sel", 11),
	GATE_DISP(CLK_DISP_RSZ0, "disp_rsz0", "rsz_sel", 12),
	GATE_DISP(CLK_DISP_VDO_DI, "disp_vdo_di", "di_sel", 14),
	GATE_DISP(CLK_DISP_FMT_DI, "disp_fmt_di", "di_sel", 15),
	GATE_DISP(CLK_DISP_NR, "disp_nr", "nr_sel", 16),
	GATE_DISP(CLK_DISP_WR_CHANNEL, "disp_wr_channel", "mm_sel", 17),
};

/* The registers are in the main display power domain */
static const struct mtk_clk_desc disp_desc = {
	.clks = disp_clks,
	.num_clks = ARRAY_SIZE(disp_clks),
	.need_runtime_pm = true,
};

static const struct of_device_id of_match_clk_mt8695_disp[] = {
	{ .compatible = "mediatek,mt8695-dispsys", .data = &disp_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt8695_disp);

static struct platform_driver clk_mt8695_disp_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt8695-disp",
		.of_match_table = of_match_clk_mt8695_disp,
	},
};
module_platform_driver(clk_mt8695_disp_drv);

MODULE_DESCRIPTION("MediaTek MT8695 dispsys clocks driver");
MODULE_LICENSE("GPL");
