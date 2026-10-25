// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018 MediaTek Inc.
 * Author: Ryder Lee <ryder.lee@mediatek.com>
 *
 * Gate bits from the MT8695 vendor kernel's AFE driver; driver structure
 * follows clk-mt2701-aud.c.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-gate.h"

#include <dt-bindings/clock/mediatek,mt8695-clk.h>

/* Single registers of power-down bits: a set bit gates the clock */
static const struct mtk_gate_regs aud0_cg_regs = {
	.set_ofs = 0x0,
	.clr_ofs = 0x0,
	.sta_ofs = 0x0,
};

static const struct mtk_gate_regs aud1_cg_regs = {
	.set_ofs = 0x10,
	.clr_ofs = 0x10,
	.sta_ofs = 0x10,
};

static const struct mtk_gate_regs aud2_cg_regs = {
	.set_ofs = 0x14,
	.clr_ofs = 0x14,
	.sta_ofs = 0x14,
};

#define GATE_AUD0(_id, _name, _parent, _shift)	\
	GATE_MTK(_id, _name, _parent, &aud0_cg_regs, _shift, &mtk_clk_gate_ops_no_setclr)

#define GATE_AUD1(_id, _name, _parent, _shift)	\
	GATE_MTK(_id, _name, _parent, &aud1_cg_regs, _shift, &mtk_clk_gate_ops_no_setclr)

#define GATE_AUD2(_id, _name, _parent, _shift)	\
	GATE_MTK(_id, _name, _parent, &aud2_cg_regs, _shift, &mtk_clk_gate_ops_no_setclr)

static const struct mtk_gate aud_clks[] = {
	/* AUDIO_TOP_CON0 */
	GATE_AUD0(CLK_AUD_AFE, "aud_afe", "aud_intbus_sel", 2),
	GATE_AUD0(CLK_AUD_LRCK_CNT, "aud_lrck_cnt", "aud_intbus_sel", 4),
	GATE_AUD0(CLK_AUD_HDMI, "aud_hdmi", "apll_sel", 20),
	GATE_AUD0(CLK_AUD_SPDF, "aud_spdf", "apll_sel", 21),
	GATE_AUD0(CLK_AUD_APLL, "aud_apll", "apll_sel", 23),
	GATE_AUD0(CLK_AUD_TML, "aud_tml", "aud_intbus_sel", 27),
	/* AUDIO_TOP_CON4 */
	GATE_AUD1(CLK_AUD_I2SIN1, "aud_i2sin1", "a1sys_hp_sel", 0),
	GATE_AUD1(CLK_AUD_I2SIN2, "aud_i2sin2", "a1sys_hp_sel", 1),
	GATE_AUD1(CLK_AUD_I2SIN3, "aud_i2sin3", "a1sys_hp_sel", 2),
	GATE_AUD1(CLK_AUD_I2SIN4, "aud_i2sin4", "a1sys_hp_sel", 3),
	GATE_AUD1(CLK_AUD_I2SO1, "aud_i2so1", "a1sys_hp_sel", 6),
	GATE_AUD1(CLK_AUD_I2SO2, "aud_i2so2", "a1sys_hp_sel", 7),
	GATE_AUD1(CLK_AUD_I2SO3, "aud_i2so3", "a1sys_hp_sel", 8),
	GATE_AUD1(CLK_AUD_I2SO4, "aud_i2so4", "a1sys_hp_sel", 9),
	GATE_AUD1(CLK_AUD_I2SO5, "aud_i2so5", "a1sys_hp_sel", 10),
	GATE_AUD1(CLK_AUD_I2SO6, "aud_i2so6", "a1sys_hp_sel", 11),
	GATE_AUD1(CLK_AUD_ASRCI1, "aud_asrci1", "asm_h_sel", 12),
	GATE_AUD1(CLK_AUD_ASRCI2, "aud_asrci2", "asm_h_sel", 13),
	GATE_AUD1(CLK_AUD_MULTI_IN, "aud_multi_in", "a1sys_hp_sel", 19),
	GATE_AUD1(CLK_AUD_INTDIR, "aud_intdir", "intdir_sel", 20),
	GATE_AUD1(CLK_AUD_A1SYS, "aud_a1sys", "a1sys_hp_sel", 21),
	GATE_AUD1(CLK_AUD_A2SYS, "aud_a2sys", "a2sys_hp_sel", 22),
	GATE_AUD1(CLK_AUD_AFE_CONN, "aud_afe_conn", "a1sys_hp_sel", 23),
	GATE_AUD1(CLK_AUD_PCMIF, "aud_pcmif", "a1sys_hp_sel", 24),
	GATE_AUD1(CLK_AUD_ASRCI3, "aud_asrci3", "asm_h_sel", 26),
	GATE_AUD1(CLK_AUD_ASRCI4, "aud_asrci4", "asm_h_sel", 27),
	/* AUDIO_TOP_CON5 */
	GATE_AUD2(CLK_AUD_MEMIF_UL1, "aud_memif_ul1", "a1sys_hp_sel", 0),
	GATE_AUD2(CLK_AUD_MEMIF_UL2, "aud_memif_ul2", "a1sys_hp_sel", 1),
	GATE_AUD2(CLK_AUD_MEMIF_UL3, "aud_memif_ul3", "a1sys_hp_sel", 2),
	GATE_AUD2(CLK_AUD_MEMIF_UL4, "aud_memif_ul4", "a1sys_hp_sel", 3),
	GATE_AUD2(CLK_AUD_MEMIF_UL5, "aud_memif_ul5", "a1sys_hp_sel", 4),
	GATE_AUD2(CLK_AUD_MEMIF_DL1, "aud_memif_dl1", "a1sys_hp_sel", 6),
	GATE_AUD2(CLK_AUD_MEMIF_DL2, "aud_memif_dl2", "a1sys_hp_sel", 7),
	GATE_AUD2(CLK_AUD_MEMIF_DL6, "aud_memif_dl6", "a1sys_hp_sel", 11),
	GATE_AUD2(CLK_AUD_MEMIF_DLMCH, "aud_memif_dlmch", "a1sys_hp_sel", 12),
};

static const struct mtk_clk_desc aud_desc = {
	.clks = aud_clks,
	.num_clks = ARRAY_SIZE(aud_clks),
};

static const struct of_device_id of_match_clk_mt8695_aud[] = {
	{ .compatible = "mediatek,mt8695-audsys", .data = &aud_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt8695_aud);

/*
 * The registers sit behind the audio bus clock: hold it, or the gate
 * accesses (from clk_disable_unused() at the latest) stall the bus. The
 * AFE is a child node sharing the registers through the syscon.
 */
static int clk_mt8695_aud_probe(struct platform_device *pdev)
{
	struct clk *bus_clk;
	int ret;

	bus_clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(bus_clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(bus_clk),
				     "failed to enable the bus clock\n");

	ret = mtk_clk_simple_probe(pdev);
	if (ret)
		return ret;

	ret = devm_of_platform_populate(&pdev->dev);
	if (ret)
		mtk_clk_simple_remove(pdev);

	return ret;
}

static void clk_mt8695_aud_remove(struct platform_device *pdev)
{
	of_platform_depopulate(&pdev->dev);
	mtk_clk_simple_remove(pdev);
}

static struct platform_driver clk_mt8695_aud_drv = {
	.probe = clk_mt8695_aud_probe,
	.remove = clk_mt8695_aud_remove,
	.driver = {
		.name = "clk-mt8695-aud",
		.of_match_table = of_match_clk_mt8695_aud,
	},
};
module_platform_driver(clk_mt8695_aud_drv);

MODULE_DESCRIPTION("MediaTek MT8695 audsys clocks driver");
MODULE_LICENSE("GPL");
