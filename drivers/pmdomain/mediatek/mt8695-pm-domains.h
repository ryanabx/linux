/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Domain table from the MT8695 vendor kernel (mtk-scpsys.c,
 * scp_domain_data_mt8695), and the GPU's from its MFG driver (mtk_mfg.c),
 * which switches them itself.
 */

#ifndef __SOC_MEDIATEK_MT8695_PM_DOMAINS_H
#define __SOC_MEDIATEK_MT8695_PM_DOMAINS_H

#include "mtk-pm-domains.h"
#include <dt-bindings/power/mediatek,mt8695-power.h>

/*
 * MT8695 power domain support
 */
static enum scpsys_bus_prot_block scpsys_bus_prot_blocks_mt8695[] = {
	BUS_PROT_BLOCK_INFRA
};

static const struct scpsys_domain_data scpsys_domain_data_mt8695[] = {
	/*
	 * The HDMI TX in MM comes back from a power cycle with its HDCP
	 * control at mute overrides that only the secure world can clear,
	 * so the output would stay muted for good.
	 */
	[MT8695_POWER_DOMAIN_MM] = {
		.name = "mm",
		.sta_mask = BIT(11),
		.ctl_offs = 0x026c,
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_ACTIVE_WAKEUP | MTK_SCPD_ALWAYS_ON,
	},
	[MT8695_POWER_DOMAIN_VDEC] = {
		.name = "vdec",
		.sta_mask = BIT(7),
		.ctl_offs = 0x0258,
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT8695_POWER_DOMAIN_DIS_M] = {
		.name = "dis_m",
		.sta_mask = BIT(8),
		.ctl_offs = 0x025c,
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT8695_POWER_DOMAIN_DIS_S] = {
		.name = "dis_s",
		.sta_mask = BIT(9),
		.ctl_offs = 0x0260,
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT8695_POWER_DOMAIN_DIS_C] = {
		.name = "dis_c",
		.sta_mask = BIT(10),
		.ctl_offs = 0x0264,
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT8695_POWER_DOMAIN_DOLBY] = {
		.name = "dolby",
		.sta_mask = BIT(12),
		.ctl_offs = 0x0270,
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT8695_POWER_DOMAIN_VENC] = {
		.name = "venc",
		.sta_mask = BIT(13),
		.ctl_offs = 0x0274,
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(14, 12),
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	/*
	 * The GPU: ASYNC, then 2D, which carries the GPU's bus protection
	 * (MT8173 has the same bits on its 3D domain), then 3D. The vendor
	 * kernel doesn't wait for an SRAM acknowledge on ASYNC.
	 */
	[MT8695_POWER_DOMAIN_MFG_ASYNC] = {
		.name = "mfg_async",
		.sta_mask = BIT(18),
		.ctl_offs = 0x0290,
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = 0,
	},
	[MT8695_POWER_DOMAIN_MFG_2D] = {
		.name = "mfg_2d",
		.sta_mask = BIT(19),
		.ctl_offs = 0x0294,
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
		.bp_cfg = {
			BUS_PROT_INFRA_UPDATE_TOPAXI(MT8173_TOP_AXI_PROT_EN_MFG_S |
						     MT8173_TOP_AXI_PROT_EN_MFG_M0 |
						     MT8173_TOP_AXI_PROT_EN_MFG_M1 |
						     MT8173_TOP_AXI_PROT_EN_MFG_SNOOP_OUT),
		},
	},
	[MT8695_POWER_DOMAIN_MFG] = {
		.name = "mfg",
		.sta_mask = BIT(20),
		.ctl_offs = 0x0298,
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND,
		.sram_pdn_bits = GENMASK(13, 8),
		.sram_pdn_ack_bits = GENMASK(19, 16),
	},
	[MT8695_POWER_DOMAIN_ETH] = {
		.name = "eth",
		.sta_mask = BIT(22),
		.ctl_offs = 0x02bc,
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
	[MT8695_POWER_DOMAIN_USB] = {
		.name = "usb",
		.sta_mask = BIT(23),
		.ctl_offs = 0x02b4,
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
	},
};

static const struct scpsys_soc_data mt8695_scpsys_data = {
	.domains_data = scpsys_domain_data_mt8695,
	.num_domains = ARRAY_SIZE(scpsys_domain_data_mt8695),
	.bus_prot_blocks = scpsys_bus_prot_blocks_mt8695,
	.num_bus_prot_blocks = ARRAY_SIZE(scpsys_bus_prot_blocks_mt8695),
};

#endif /* __SOC_MEDIATEK_MT8695_PM_DOMAINS_H */
