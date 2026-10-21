/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Domain table from the MT8695 vendor kernel (mtk-scpsys.c,
 * scp_domain_data_mt8695). None of the domains has bus protection.
 */

#ifndef __SOC_MEDIATEK_MT8695_PM_DOMAINS_H
#define __SOC_MEDIATEK_MT8695_PM_DOMAINS_H

#include "mtk-pm-domains.h"
#include <dt-bindings/power/mediatek,mt8695-power.h>

/*
 * MT8695 power domain support
 */

static const struct scpsys_domain_data scpsys_domain_data_mt8695[] = {
	[MT8695_POWER_DOMAIN_MM] = {
		.name = "mm",
		.sta_mask = BIT(11),
		.ctl_offs = 0x026c,
		.pwr_sta_offs = SPM_PWR_STATUS,
		.pwr_sta2nd_offs = SPM_PWR_STATUS_2ND,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
		.caps = MTK_SCPD_ACTIVE_WAKEUP,
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
};

#endif /* __SOC_MEDIATEK_MT8695_PM_DOMAINS_H */
