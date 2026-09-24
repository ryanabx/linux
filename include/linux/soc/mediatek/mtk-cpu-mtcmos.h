/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek CPU MTCMOS power sequences, for firmware that does not run them.
 */
#ifndef __SOC_MEDIATEK_MTK_CPU_MTCMOS_H
#define __SOC_MEDIATEK_MTK_CPU_MTCMOS_H

int mtk_cpu_mtcmos_init(void);
int mtk_cpu_mtcmos_power_on(unsigned int cluster, unsigned int core);
int mtk_cpu_mtcmos_power_off(unsigned int cluster, unsigned int core);

#endif /* __SOC_MEDIATEK_MTK_CPU_MTCMOS_H */
