// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT8695 HDMI 2.0 PHY
 *
 * The register layout and sequences follow MediaTek's MT8695 HDMI driver.
 * Only TMDS rates below 1.65 Gbps (up to 1080p60 with 8-bit colour) are
 * supported so far; the higher bands need further drive and impedance
 * settings, some of them from efuses.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/math.h>
#include <linux/nvmem-consumer.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "phy-mtk-hdmi.h"
#include "phy-mtk-io.h"

#define HDMI20_CFG_0		0x00
#define  RG_SER_EN		GENMASK(31, 28)
#define  RG_PRD_EN		GENMASK(27, 24)
#define  RG_DRV_EN		GENMASK(23, 20)
#define  RG_DRV_IMP_EN		GENMASK(19, 16)
#define HDMI20_CFG_1		0x04
#define  RG_PRD_IBIAS_CLK	GENMASK(28, 24)
#define  RG_PRD_IBIAS_D2	GENMASK(20, 16)
#define  RG_PRD_IBIAS_D1	GENMASK(12, 8)
#define  RG_PRD_IBIAS_D0	GENMASK(4, 0)
#define HDMI20_CFG_2		0x08
#define  RG_DRV_IBIAS_CLK	GENMASK(29, 24)
#define  RG_DRV_IBIAS_D2	GENMASK(21, 16)
#define  RG_DRV_IBIAS_D1	GENMASK(13, 8)
#define  RG_DRV_IBIAS_D0	GENMASK(5, 0)
#define HDMI20_CFG_3		0x0c
#define HDMI20_CFG_4		0x10
#define  RG_SLDO_EN		GENMASK(7, 4)
#define  RG_SLDOLPF_EN		BIT(3)
#define HDMI20_CFG_5		0x14
#define  RG_CKLDO_EN		BIT(31)
#define  RG_CKLDOLPF_EN		BIT(30)
#define  RG_PRED_IMP		GENMASK(7, 0)
#define HDMI20_CFG_6		0x18
#define HDMI20_CFG_7		0x1c
#define  RG_CFG7_ANA		GENMASK(15, 3)
#define  RG_RESERVE		GENMASK(2, 0)
#define HDMI20_CFG_8		0x20
#define  RG_BIAS_LPF_EN		BIT(28)
#define  RG_BIAS_EN		BIT(27)
#define  RG_TX_POSDIV_EN	BIT(26)
#define  RG_TX_POSDIV		GENMASK(25, 24)
#define  RG_INTR_EN		BIT(21)
#define  RG_INTR_CAL		GENMASK(20, 16)
#define HDMI20_PLL_CFG_0	0x40
#define  RG_PLL_BG_EN		BIT(31)
#define  RG_PLL_EN		BIT(30)
#define  RG_PLL_FBKSEL		BIT(29)
#define  RG_PLL_PREDIV		GENMASK(28, 27)
#define  RG_PLL_POSDIV		BIT(25)
#define  RG_PLL_FBKDIV		GENMASK(23, 13)
#define  RG_PLL_BAND		GENMASK(7, 1)
#define HDMI20_PLL_CFG_1	0x44
#define  RG_PLL_BC		GENMASK(31, 27)
#define  RG_PLL_BP2		GENMASK(26, 25)
#define  RG_PLL_BP		GENMASK(23, 19)
#define  RG_PLL_BR		GENMASK(15, 12)
#define  RG_PLL_IC		GENMASK(11, 8)
#define  RG_PLL_IR		GENMASK(7, 4)
#define HDMI20_PLL_CFG_2	0x48
#define HDMI20_PLL_CFG_3	0x4c
#define HDMI20_CLK_CFG		0x70
#define  REG_TXC_DIV		GENMASK(31, 30)
#define HDMI_ANA_CTL		0x7c
#define  REG_ANA_HDMI20_FIFO_EN	BIT(16)

/* The rows of MediaTek's PLL tables for 8-bit colour */
struct mtk_hdmi_pll_row {
	unsigned long max_rate;
	u8 tx_posdiv;
	u8 br;
	u8 ir;
};

static const struct mtk_hdmi_pll_row mtk_hdmi_pll_rows[] = {
	{  27027000, 3, 7, 3 },
	{  74250000, 2, 6, 3 },
	{ 148500000, 1, 6, 4 },
};

/* The internal resistor calibration, bits 13:9 of the efuse cell's first word */
static u32 mtk_hdmi_phy_intr_cal(struct mtk_hdmi_phy *hdmi_phy)
{
	struct nvmem_cell *cell;
	size_t len;
	u32 cal = 0;
	u8 *buf;

	cell = nvmem_cell_get(hdmi_phy->dev, "intr-cal");
	if (IS_ERR(cell))
		return 0;
	buf = nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);
	if (IS_ERR(buf))
		return 0;
	if (len >= 4)
		cal = (get_unaligned_le32(buf) >> 9) & 0x1f;
	kfree(buf);
	return cal;
}

static const struct mtk_hdmi_pll_row *mtk_hdmi_pll_find(unsigned long rate)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_hdmi_pll_rows); i++)
		if (rate <= mtk_hdmi_pll_rows[i].max_rate)
			return &mtk_hdmi_pll_rows[i];
	return NULL;
}

/* vTmdsPresetOn(): the PLL, then the serializer; the drivers come later */
static int mtk_hdmi_pll_prepare(struct clk_hw *hw)
{
	struct mtk_hdmi_phy *hdmi_phy = to_mtk_hdmi_phy(hw);
	void __iomem *base = hdmi_phy->regs;

	mtk_phy_set_bits(base + HDMI20_CFG_7, RG_RESERVE);
	mtk_phy_set_bits(base + HDMI20_PLL_CFG_0, RG_PLL_BG_EN);
	mtk_phy_set_bits(base + HDMI20_PLL_CFG_0, RG_PLL_EN);
	usleep_range(100, 150);
	mtk_phy_set_bits(base + HDMI20_CFG_8, RG_BIAS_EN);
	mtk_phy_set_bits(base + HDMI20_CFG_5, RG_CKLDO_EN);
	mtk_phy_set_bits(base + HDMI20_CFG_4, RG_SLDO_EN);
	mtk_phy_set_bits(base + HDMI20_CFG_8, RG_TX_POSDIV_EN);
	mtk_phy_set_bits(base + HDMI20_CFG_0, RG_SER_EN);
	mtk_phy_set_bits(base + HDMI20_CFG_0, RG_PRD_EN);
	mtk_phy_update_bits(base + HDMI20_CFG_8, RG_BIAS_LPF_EN | RG_INTR_EN,
			    RG_INTR_EN);
	mtk_phy_clear_bits(base + HDMI20_CFG_5, RG_CKLDOLPF_EN);
	mtk_phy_clear_bits(base + HDMI20_CFG_4, RG_SLDOLPF_EN);
	mtk_phy_set_bits(base + HDMI_ANA_CTL, REG_ANA_HDMI20_FIFO_EN);
	usleep_range(100, 150);

	return 0;
}

/* vTxSignalOnOff(OFF) after the drivers */
static void mtk_hdmi_pll_unprepare(struct clk_hw *hw)
{
	struct mtk_hdmi_phy *hdmi_phy = to_mtk_hdmi_phy(hw);
	void __iomem *base = hdmi_phy->regs;

	mtk_phy_clear_bits(base + HDMI_ANA_CTL, REG_ANA_HDMI20_FIFO_EN);
	mtk_phy_clear_bits(base + HDMI20_CFG_0, RG_PRD_EN);
	mtk_phy_clear_bits(base + HDMI20_CFG_0, RG_SER_EN);
	mtk_phy_clear_bits(base + HDMI20_CFG_8, RG_TX_POSDIV_EN | RG_BIAS_EN);
	mtk_phy_clear_bits(base + HDMI20_CFG_4, RG_SLDO_EN);
	mtk_phy_clear_bits(base + HDMI20_CFG_5, RG_CKLDO_EN);
	mtk_phy_clear_bits(base + HDMI20_PLL_CFG_0, RG_PLL_EN);
	mtk_phy_clear_bits(base + HDMI20_PLL_CFG_0, RG_PLL_BG_EN);
	mtk_phy_clear_bits(base + HDMI20_CFG_7, RG_RESERVE);
}

static int mtk_hdmi_pll_determine_rate(struct clk_hw *hw,
				       struct clk_rate_request *req)
{
	if (!mtk_hdmi_pll_find(req->rate))
		return -EINVAL;
	return 0;
}

static int mtk_hdmi_pll_set_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long parent_rate)
{
	struct mtk_hdmi_phy *hdmi_phy = to_mtk_hdmi_phy(hw);
	const struct mtk_hdmi_pll_row *row = mtk_hdmi_pll_find(rate);
	void __iomem *base = hdmi_phy->regs;
	u32 ibias = hdmi_phy->ibias;
	u32 cal;

	if (!row)
		return -EINVAL;

	dev_dbg(hdmi_phy->dev, "%s: %lu Hz, parent: %lu Hz\n", __func__,
		rate, parent_rate);

	/* hdmitx_confighdmisetting() */
	mtk_phy_set_bits(base + HDMI_ANA_CTL, REG_ANA_HDMI20_FIFO_EN);
	mtk_phy_clear_bits(base + HDMI20_CLK_CFG, REG_TXC_DIV);
	writel(0x0800009c, base + HDMI20_PLL_CFG_2);
	writel(0x00480000, base + HDMI20_PLL_CFG_3);
	mtk_phy_update_field(base + HDMI20_CFG_7, RG_CFG7_ANA, 0x1ff4);

	/* vSetHDMITxPLL(): the TMDS clock from its reference */
	mtk_phy_update_field(base + HDMI20_PLL_CFG_0, RG_PLL_BAND, 0x37);
	mtk_phy_update_bits(base + HDMI20_PLL_CFG_1,
			    RG_PLL_BC | RG_PLL_BP | RG_PLL_BP2 | RG_PLL_BR |
			    RG_PLL_IC | RG_PLL_IR,
			    FIELD_PREP(RG_PLL_BC, 0x3) |
			    FIELD_PREP(RG_PLL_BP, 0xf) |
			    FIELD_PREP(RG_PLL_BP2, 0x3) |
			    FIELD_PREP(RG_PLL_BR, row->br) |
			    FIELD_PREP(RG_PLL_IC, 0x3) |
			    FIELD_PREP(RG_PLL_IR, row->ir));
	mtk_phy_update_field(base + HDMI20_PLL_CFG_0, RG_PLL_FBKDIV, 80);
	mtk_phy_set_bits(base + HDMI20_PLL_CFG_0, RG_PLL_FBKSEL);
	mtk_phy_update_field(base + HDMI20_CFG_8, RG_TX_POSDIV, row->tx_posdiv);
	cal = mtk_hdmi_phy_intr_cal(hdmi_phy);
	if (cal)
		mtk_phy_update_field(base + HDMI20_CFG_8, RG_INTR_CAL, cal);
	mtk_phy_update_field(base + HDMI20_PLL_CFG_0, RG_PLL_PREDIV, 1);
	mtk_phy_set_bits(base + HDMI20_PLL_CFG_0, RG_PLL_POSDIV);

	/* Below 1.65 Gbps: no pre-emphasis */
	mtk_phy_clear_bits(base + HDMI20_CFG_5, RG_PRED_IMP);
	mtk_phy_update_bits(base + HDMI20_CFG_1,
			    RG_PRD_IBIAS_CLK | RG_PRD_IBIAS_D2 |
			    RG_PRD_IBIAS_D1 | RG_PRD_IBIAS_D0,
			    FIELD_PREP(RG_PRD_IBIAS_CLK, 0x3) |
			    FIELD_PREP(RG_PRD_IBIAS_D2, 0x3) |
			    FIELD_PREP(RG_PRD_IBIAS_D1, 0x3) |
			    FIELD_PREP(RG_PRD_IBIAS_D0, 0x3));
	mtk_phy_clear_bits(base + HDMI20_CFG_0, RG_DRV_IMP_EN);
	mtk_phy_update_bits(base + HDMI20_CFG_2,
			    RG_DRV_IBIAS_CLK | RG_DRV_IBIAS_D2 |
			    RG_DRV_IBIAS_D1 | RG_DRV_IBIAS_D0,
			    FIELD_PREP(RG_DRV_IBIAS_CLK, ibias) |
			    FIELD_PREP(RG_DRV_IBIAS_D2, ibias) |
			    FIELD_PREP(RG_DRV_IBIAS_D1, ibias) |
			    FIELD_PREP(RG_DRV_IBIAS_D0, ibias));
	writel(0, base + HDMI20_CFG_3);
	writel(0, base + HDMI20_CFG_6);

	hdmi_phy->pll_rate = rate;
	return 0;
}

static unsigned long mtk_hdmi_pll_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct mtk_hdmi_phy *hdmi_phy = to_mtk_hdmi_phy(hw);

	return hdmi_phy->pll_rate;
}

static const struct clk_ops mtk_hdmi_phy_pll_ops = {
	.prepare = mtk_hdmi_pll_prepare,
	.unprepare = mtk_hdmi_pll_unprepare,
	.determine_rate = mtk_hdmi_pll_determine_rate,
	.set_rate = mtk_hdmi_pll_set_rate,
	.recalc_rate = mtk_hdmi_pll_recalc_rate,
};

static void mtk_hdmi_phy_enable_tmds(struct mtk_hdmi_phy *hdmi_phy)
{
	mtk_phy_set_bits(hdmi_phy->regs + HDMI20_CFG_0, RG_DRV_EN);
}

static void mtk_hdmi_phy_disable_tmds(struct mtk_hdmi_phy *hdmi_phy)
{
	mtk_phy_clear_bits(hdmi_phy->regs + HDMI20_CFG_0, RG_DRV_EN);
	usleep_range(100, 150);
}

/*
 * The PLL runs its reference times 40 and divides by 10 << TX_POSDIV for
 * the TMDS clock, so each row needs its own reference: TVDPLL / 8 for the
 * HD rates, OSDPLL / 12 (54 MHz) for 27 MHz, as MediaTek's driver selects
 * with HDMITX20_CKSEL. Pick the parent of that mux that runs closest to
 * the rate needed; it can't be left to clk_set_rate(), which could retune
 * TVDPLL, the pixel clock, instead.
 */
static int mtk_hdmi_pll_select_ref(struct mtk_hdmi_phy *hdmi_phy,
				   unsigned long rate)
{
	const struct mtk_hdmi_pll_row *row = mtk_hdmi_pll_find(rate);
	struct clk_hw *mux = clk_hw_get_parent(&hdmi_phy->pll_hw);
	struct clk_hw *best = NULL, *p;
	unsigned long ref, diff, best_diff = ULONG_MAX;
	unsigned int i;

	if (!row || !mux)
		return -EINVAL;

	ref = (rate << row->tx_posdiv) / 4;
	for (i = 0; i < clk_hw_get_num_parents(mux); i++) {
		p = clk_hw_get_parent_by_index(mux, i);
		if (!p)
			continue;
		diff = abs_diff(clk_hw_get_rate(p), ref);
		if (diff < best_diff) {
			best = p;
			best_diff = diff;
		}
	}

	/* Within 0.1%: the /1.001 rates come from TVDPLL itself */
	if (!best || best_diff > ref / 1000) {
		dev_err(hdmi_phy->dev, "no reference for %lu Hz\n", rate);
		return -EINVAL;
	}

	return clk_hw_set_parent(mux, best);
}

/* The PLL makes the TMDS clock; its rate can only change while it is off */
static int mtk_hdmi_phy_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct mtk_hdmi_phy *hdmi_phy = phy_get_drvdata(phy);
	int ret;

	ret = mtk_hdmi_pll_select_ref(hdmi_phy, opts->dp.link_rate);
	if (ret)
		return ret;

	return clk_set_rate(hdmi_phy->pll, opts->dp.link_rate);
}

struct mtk_hdmi_phy_conf mtk_hdmi_phy_8695_conf = {
	.flags = CLK_SET_RATE_GATE,
	.hdmi_phy_clk_ops = &mtk_hdmi_phy_pll_ops,
	.hdmi_phy_enable_tmds = mtk_hdmi_phy_enable_tmds,
	.hdmi_phy_disable_tmds = mtk_hdmi_phy_disable_tmds,
	.hdmi_phy_configure = mtk_hdmi_phy_configure,
};
