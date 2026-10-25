/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek MT8695 audio front end registers, the HDMI output path only.
 * The layout is MT2701's (mt2701-reg.h); values from the vendor kernel.
 */

#ifndef _MT8695_REG_H_
#define _MT8695_REG_H_

#include <linux/bits.h>

#define AUDIO_TOP_CON0		0x0000
#define AUDIO_TOP_CON3		0x000c
#define AFE_HDMI_OUT_CON0	0x0370
#define AFE_HDMI_OUT_BASE	0x0374
#define AFE_HDMI_OUT_CUR	0x0378
#define AFE_HDMI_OUT_END	0x037c
#define AFE_HDMI_CONN0		0x0390
#define AFE_8CH_I2S_OUT_CON	0x0394
#define AFE_IRQ_CON		0x03a0
#define AFE_IRQ_STATUS		0x03a4
#define AFE_IRQ_CLR		0x03a8
#define AFE_IRQ_CNT5		0x03bc
#define ASYS_TOP_CON		0x0600
#define AFE_DAC_CON0		0x1200
#define AFE_MEMIF_PBUF_SIZE	0x1238
#define AFE_MAX_REGISTER	0x1fff

/* AUDIO_TOP_CON0: the 8-channel I2S clock, APLL1 (48 kHz family) or APLL2 */
#define AUDIO_TOP_CON0_8CH_I2S_APLL2	BIT(15)

/* AUDIO_TOP_CON3: HDMI clock = APLL / (div + 1), 128 fs */
#define AUDIO_TOP_CON3_HDMI_BCK_DIV	GENMASK(13, 8)

/* AFE_HDMI_OUT_CON0 */
#define AFE_HDMI_OUT_CON0_ON		BIT(0)
#define AFE_HDMI_OUT_CON0_32BIT		BIT(1)
#define AFE_HDMI_OUT_CON0_CH_NUM	GENMASK(7, 4)

/* AFE_HDMI_CONN0: output O20 + n takes input I20 + field n, 3 bits each */
#define AFE_HDMI_CONN0_IDENTITY		0x00fac688

/* AFE_8CH_I2S_OUT_CON */
#define AFE_8CH_I2S_OUT_CON_EN		BIT(0)
#define AFE_8CH_I2S_OUT_CON_BCK_INV	BIT(1)
#define AFE_8CH_I2S_OUT_CON_LRCK_INV	BIT(2)
#define AFE_8CH_I2S_OUT_CON_I2S_DELAY	BIT(3)
#define AFE_8CH_I2S_OUT_CON_WLEN	GENMASK(5, 4)
#define AFE_8CH_I2S_OUT_CON_WLEN_32BIT	3

/* AFE_IRQ_CON, AFE_IRQ_STATUS and AFE_IRQ_CLR: IRQ5 serves the HDMI memif */
#define AFE_IRQ_CON_IRQ5_ON_SHIFT	12
#define AFE_IRQ_HDMI_SHIFT		4
#define AFE_IRQ_CLR_ALL			(GENMASK(6, 0) | BIT(10))

/* ASYS_TOP_CON */
#define ASYS_TOP_CON_TIMING_ON		GENMASK(1, 0)

/* AFE_DAC_CON0 */
#define AFE_DAC_CON0_AFE_ON		BIT(0)

/* AFE_MEMIF_PBUF_SIZE: the HDMI memif's prefetch buffer */
#define AFE_MEMIF_PBUF_SIZE_HDMI	GENMASK(19, 18)

#endif
