// SPDX-License-Identifier: GPL-2.0-only
//
// MT6391 (MT6397 PMIC) audio codec driver
//
// The power sequences are MediaTek's, from their MT6397 codec driver.
//
// Copyright (c) 2026 Ryan Brue <ryanbrue.dev@gmail.com>

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mfd/mt6397/core.h>
#include <linux/mfd/mt6397/registers.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "mt6391.h"

#define MT6397_CID_E1			0x1097

/* HP trim fuses, EFUSE_DOUT_192_207 */
#define MT6397_EFUSE_HPL_TRIM		GENMASK(6, 3)
#define MT6397_EFUSE_HPR_TRIM		GENMASK(10, 7)
#define MT6397_EFUSE_HP_TRIM_EN		BIT(11)
#define MT6397_EFUSE_HPL_FINE_TRIM	GENMASK(13, 12)
#define MT6397_EFUSE_HPR_FINE_TRIM	GENMASK(15, 14)

/* Speaker offset trim readout through TEST_CON0/TEST_OUT_L */
#define MT6397_TEST_SPKL_TRIM		0x0805
#define MT6397_TEST_SPKL_TRIM_POL	0x0806
#define MT6397_TEST_SPKR_TRIM		0x080e
#define MT6397_TEST_SPKR_TRIM_POL	0x080f

/*
 * Headphone PGA: register 0 is +8 dB down to 12 at -4 dB in 1 dB steps,
 * 13 and 14 are reserved, and 15 is -40 dB.
 */
#define MT6391_HP_GAIN_LOW		12
#define MT6391_HP_GAIN_MUTE		15
#define MT6391_HP_GAIN_MAX		13

/*
 * The platform starts the stream as soon as the amplifiers are up, so let
 * them settle first: in the headphones, a stream that starts within 20 ms
 * of power-up pops, and one that starts after 50 ms does not.
 */
#define MT6391_AMP_SETTLE_MS		50

/*
 * ACCDET compares the microphone contact of the jack against two
 * thresholds, A and B: AB = 11 is an open contact (nothing plugged in),
 * 01 a headset microphone, 00 a contact grounded by a three-pole plug or
 * by a pressed headset button. Its times are in ticks of a 32 kHz clock.
 *
 * The microphone bias stays on (PWM threshold equal to the width): pulsed,
 * an open contact does not charge past threshold A and reads as 01.
 */
#define MT6391_ACCDET_AB_GROUND		0
#define MT6391_ACCDET_AB_HEADSET	1
#define MT6391_ACCDET_AB_OPEN		3
#define MT6391_ACCDET_RSV_2V8		0x5090	/* MediaTek's 2.8 V mode */
#define MT6391_ACCDET_PWM_WIDTH		0x500
#define MT6391_ACCDET_RISE_DELAY	0x1f0
#define MT6391_ACCDET_DEBOUNCE		0x800
#define MT6391_ACCDET_DEBOUNCE_OPEN	0x20
/* An insertion slides the contacts through every state before settling */
#define MT6391_ACCDET_SETTLE_MS		500

enum {
	SUPPLY_SEQ_AUD_CK,
	SUPPLY_SEQ_ZCD,
	SUPPLY_SEQ_DL_SRC,
	SUPPLY_SEQ_SPK_CK,
};

struct mt6391_priv {
	struct device *dev;
	struct regmap *regmap;
	unsigned int dl_rate_mode;
	bool dl_src_on;
	int dl_users;
	unsigned int spk_gain[2];
	unsigned int hp_gain[2];
	bool hp_on;
	unsigned int spk_trim[2];
	unsigned int hp_trim;
	struct snd_soc_jack *jack;
	struct delayed_work jack_work;
	int jack_type;
	int accdet_irq;
};

static int mt6391_dl_rate_mode(unsigned int rate)
{
	switch (rate) {
	case 8000:
		return 0;
	case 11025:
		return 1;
	case 12000:
		return 2;
	case 16000:
		return 3;
	case 22050:
		return 4;
	case 24000:
		return 5;
	case 32000:
		return 6;
	case 44100:
		return 7;
	case 48000:
		return 8;
	default:
		return -EINVAL;
	}
}

static void mt6391_set_dl_rate(struct mt6391_priv *priv)
{
	regmap_update_bits(priv->regmap, MT6397_AFE_PMIC_NEWIF_CFG0,
			   MT6397_NEWIF_DL_RATE,
			   FIELD_PREP(MT6397_NEWIF_DL_RATE, priv->dl_rate_mode));
	regmap_update_bits(priv->regmap, MT6397_AFE_DL_SRC2_CON0_H,
			   MT6397_DL_2_INPUT_MODE,
			   FIELD_PREP(MT6397_DL_2_INPUT_MODE, priv->dl_rate_mode));
}

/* Reference generators, NV regulator, LDOs and the negative charge pump */
static void mt6391_dl_analog_on(struct mt6391_priv *priv)
{
	regmap_write(priv->regmap, MT6397_AUDLDO_CFG0,
		     MT6397_AUDLDO_DEFAULT | MT6397_RG_VA28REFGEN_EN |
		     MT6397_RG_VA33REFGEN_EN);
	regmap_write(priv->regmap, MT6397_AUDNVREGGLB_CFG0,
		     MT6397_DA_NVREG_EN | MT6397_RG_ACC_DCC_SEL_AUDGLB);
	regmap_set_bits(priv->regmap, MT6397_AUD_NCP0,
			MT6397_RG_NCP_REMOTE_SENSE | MT6397_DA_HCLDO_EN |
			MT6397_DA_LCLDO_EN);
	regmap_write(priv->regmap, MT6397_NCP_CLKDIV_CON0,
		     MT6397_RG_DIVCKS_ON | FIELD_PREP(MT6397_RG_DIVCKS_PRG, 0x2b));
	regmap_write(priv->regmap, MT6397_NCP_CLKDIV_CON1, 0);
	usleep_range(200, 250);
}

static void mt6391_dl_analog_off(struct mt6391_priv *priv)
{
	regmap_write(priv->regmap, MT6397_AUDNVREGGLB_CFG0,
		     MT6397_RG_ACC_DCC_SEL_AUDGLB | MT6397_RG_AUDGLB_PWRDN);
	regmap_write(priv->regmap, MT6397_NCP_CLKDIV_CON1, MT6397_RG_PWD_NCP);
	regmap_clear_bits(priv->regmap, MT6397_AUD_NCP0,
			  MT6397_DA_HCLDO_EN | MT6397_DA_LCLDO_EN);
	regmap_write(priv->regmap, MT6397_AUDLDO_CFG0, MT6397_AUDLDO_DEFAULT);
}

static void mt6391_dac_on(struct mt6391_priv *priv)
{
	regmap_set_bits(priv->regmap, MT6397_AUDCLKGEN_CFG0,
			MT6397_RG_RSTB_DECODER);
	regmap_write(priv->regmap, MT6397_AUDDAC_CON0,
		     MT6397_RG_AUD_DAC_PWL_UP | MT6397_RG_AUD_DAC_PWR_UP |
		     MT6397_RG_AUDDACRPWRUP | MT6397_RG_AUDDACLPWRUP);
	usleep_range(100, 150);
}

static void mt6391_hp_write_gain(struct mt6391_priv *priv, unsigned int l,
				 unsigned int r)
{
	regmap_update_bits(priv->regmap, MT6397_ZCD_CON2,
			   MT6397_RG_AUDHPLGAIN | MT6397_RG_AUDHPRGAIN,
			   FIELD_PREP(MT6397_RG_AUDHPLGAIN, l) |
			   FIELD_PREP(MT6397_RG_AUDHPRGAIN, r));
}

static unsigned int mt6391_hp_gain_step(unsigned int from, unsigned int to)
{
	if (from == to)
		return to;
	if (from == MT6391_HP_GAIN_MUTE)
		return MT6391_HP_GAIN_LOW;
	if (from == MT6391_HP_GAIN_LOW && to == MT6391_HP_GAIN_MUTE)
		return MT6391_HP_GAIN_MUTE;

	return from > to ? from - 1 : from + 1;
}

/*
 * Change the headphone gain one step at a time rather than in one write, so
 * that a large change does not pop. There are no steps between -40 dB and
 * -4 dB, so the amplifier crosses that gap only while its output is not
 * connected (mt6391_hp_start_gain()).
 */
static void mt6391_hp_ramp_gain(struct mt6391_priv *priv,
				const unsigned int *from, const unsigned int *to)
{
	unsigned int l = from[0], r = from[1];

	while (l != to[0] || r != to[1]) {
		l = mt6391_hp_gain_step(l, to[0]);
		r = mt6391_hp_gain_step(r, to[1]);
		mt6391_hp_write_gain(priv, l, r);
		usleep_range(1000, 1500);
	}
}

/* The gain to connect the output at: -4 dB, or -40 dB if that is the gain */
static unsigned int mt6391_hp_start_gain(unsigned int gain)
{
	return gain == MT6391_HP_GAIN_MUTE ? gain : MT6391_HP_GAIN_LOW;
}

/* Zero-crossing detection on the IV buffer, and the IV buffer itself */
static void mt6391_iv_buffer_on(struct mt6391_priv *priv)
{
	regmap_update_bits(priv->regmap, MT6397_ZCD_CON0, MT6397_RG_AUDZCDMUXSEL,
			   FIELD_PREP(MT6397_RG_AUDZCDMUXSEL, MT6397_ZCD_MUX_IV));
	regmap_set_bits(priv->regmap, MT6397_ZCD_CON0, MT6397_RG_AUDZCDENABLE);
	regmap_write(priv->regmap, MT6397_IBIASDIST_CFG0, MT6397_AUDBIASADJ);
	regmap_write(priv->regmap, MT6397_ZCD_CON4,
		     FIELD_PREP(MT6397_RG_AUDIVLGAIN, 5) |
		     FIELD_PREP(MT6397_RG_AUDIVRGAIN, 5));
	regmap_write(priv->regmap, MT6397_AUD_IV_CFG0,
		     MT6397_RG_AUDIVLPWRUP | MT6397_RG_AUDIVRPWRUP |
		     FIELD_PREP(MT6397_RG_AUDIVLMUXSEL, MT6397_IV_MUX_AUDIO) |
		     FIELD_PREP(MT6397_RG_AUDIVRMUXSEL, MT6397_IV_MUX_AUDIO));
	usleep_range(100, 150);
}

static void mt6391_spk_amp_on(struct mt6391_priv *priv, bool first)
{
	struct regmap *map = priv->regmap;
	unsigned int spk_ctrl = FIELD_PREP(MT6397_RG_SPKAB_OBIAS, 1) |
				FIELD_PREP(MT6397_RG_SPK_SLEW, 1);

	regmap_set_bits(map, MT6397_AFUNC_AUD_CON2, MT6397_CCI_AUD_DAC_ANA_MUTE);
	regmap_update_bits(map, MT6397_SPK_CON1, MT6397_SPK_TRIM,
			   priv->spk_trim[0]);
	regmap_update_bits(map, MT6397_SPK_CON4, MT6397_SPK_TRIM,
			   priv->spk_trim[1]);
	if (first)
		mt6391_dl_analog_on(priv);
	mt6391_iv_buffer_on(priv);
	if (first)
		mt6391_dac_on(priv);
	regmap_clear_bits(map, MT6397_AFUNC_AUD_CON2,
			  MT6397_CCI_AUD_DAC_ANA_MUTE);

	/* Enable both channels in Class-D at 0 dB, then apply the gain */
	regmap_update_bits(map, MT6397_SPK_CON9, MT6397_RG_SPKPGA_GAINL,
			   FIELD_PREP(MT6397_RG_SPKPGA_GAINL, 1));
	regmap_update_bits(map, MT6397_SPK_CON5, MT6397_RG_SPKPGA_GAINR,
			   FIELD_PREP(MT6397_RG_SPKPGA_GAINR, 1));
	regmap_write(map, MT6397_SPK_CON0,
		     FIELD_PREP(MT6397_RG_SPK_GAIN, 3) | MT6397_SPK_EN);
	regmap_write(map, MT6397_SPK_CON3,
		     FIELD_PREP(MT6397_RG_SPK_GAIN, 3) | MT6397_SPK_EN);
	regmap_write(map, MT6397_SPK_CON2, spk_ctrl);
	regmap_update_bits(map, MT6397_SPK_CON5, MT6397_SPK_CTRL, spk_ctrl);
	regmap_update_bits(map, MT6397_SPK_CON9, MT6397_RG_SPKPGA_GAINL,
			   FIELD_PREP(MT6397_RG_SPKPGA_GAINL, priv->spk_gain[0]));
	regmap_update_bits(map, MT6397_SPK_CON5, MT6397_RG_SPKPGA_GAINR,
			   FIELD_PREP(MT6397_RG_SPKPGA_GAINR, priv->spk_gain[1]));

	regmap_write(map, MT6397_SPK_CON11,
		     MT6397_SPK_OUTSTG_EN_L_SW | MT6397_SPK_OUTSTG_EN_R_SW |
		     MT6397_SPK_EN_L_SW | MT6397_SPK_EN_R_SW);
	usleep_range(4000, 5000);

	msleep(MT6391_AMP_SETTLE_MS);
}

static void mt6391_spk_amp_off(struct mt6391_priv *priv, bool last)
{
	struct regmap *map = priv->regmap;

	regmap_set_bits(map, MT6397_AFUNC_AUD_CON2, MT6397_CCI_AUD_DAC_ANA_MUTE);
	regmap_write(map, MT6397_SPK_CON0, 0);
	regmap_write(map, MT6397_SPK_CON3, 0);
	regmap_write(map, MT6397_SPK_CON11, 0);
	if (last) {
		regmap_clear_bits(map, MT6397_AUDCLKGEN_CFG0,
				  MT6397_RG_RSTB_DECODER);
		regmap_write(map, MT6397_AUDDAC_CON0, 0);
	}
	regmap_write(map, MT6397_AUD_IV_CFG0, 0);
	if (last) {
		regmap_write(map, MT6397_IBIASDIST_CFG0,
			     MT6397_AUDBIASADJ | MT6397_RG_AUDIBIASPWRDN);
		mt6391_dl_analog_off(priv);
	}
	regmap_clear_bits(map, MT6397_AFUNC_AUD_CON2,
			  MT6397_CCI_AUD_DAC_ANA_MUTE);
}

static void mt6391_hp_amp_on(struct mt6391_priv *priv, bool first)
{
	unsigned int start[2] = {
		mt6391_hp_start_gain(priv->hp_gain[0]),
		mt6391_hp_start_gain(priv->hp_gain[1]),
	};
	struct regmap *map = priv->regmap;
	unsigned int hs_mute = FIELD_PREP(MT6397_RG_AUDHSMUXINPUTSEL,
					  MT6397_HS_MUX_MUTE);

	regmap_set_bits(map, MT6397_AFUNC_AUD_CON2, MT6397_CCI_AUD_DAC_ANA_MUTE);
	regmap_update_bits(map, MT6397_AUDBUF_CFG3, MT6397_HP_TRIM,
			   priv->hp_trim);
	if (first)
		mt6391_dl_analog_on(priv);

	regmap_update_bits(map, MT6397_ZCD_CON0, MT6397_RG_AUDZCDMUXSEL,
			   FIELD_PREP(MT6397_RG_AUDZCDMUXSEL, MT6397_ZCD_MUX_HP));
	regmap_set_bits(map, MT6397_ZCD_CON0, MT6397_RG_AUDZCDENABLE);
	regmap_write(map, MT6397_AUDACCDEPOP_CFG0,
		     FIELD_PREP(MT6397_RG_DEPOP_ISEL, 3));
	regmap_write(map, MT6397_AUDBUF_CFG0, hs_mute);
	regmap_write(map, MT6397_IBIASDIST_CFG0, MT6397_AUDBIASADJ);
	/* Power up at the start gain, with the handset PGA at its minimum */
	mt6391_hp_write_gain(priv, start[0], start[1]);
	regmap_update_bits(map, MT6397_ZCD_CON3, MT6397_RG_AUDHSGAIN, 0xf);

	/* Power up with the outputs shorted to VCM, then release them */
	regmap_write(map, MT6397_AUDBUF_CFG1,
		     MT6397_RG_HPOUT_SHORTVCM | MT6397_RG_HPOUTPUTSTBENH);
	regmap_write(map, MT6397_AUDBUF_CFG2,
		     MT6397_RG_HSOUT_SHORTVCM | MT6397_RG_HSOUTPUTSTBENH);
	regmap_write(map, MT6397_AUDBUF_CFG0, hs_mute | MT6397_RG_AUDHSPWRUP);
	regmap_write(map, MT6397_AUDBUF_CFG1,
		     MT6397_RG_HPOUT_SHORTVCM | MT6397_RG_HPOUTPUTSTBENH |
		     MT6397_RG_PRECHARGEBUF_EN);
	usleep_range(200, 250);
	regmap_write(map, MT6397_AUDBUF_CFG0,
		     hs_mute | MT6397_RG_AUDHSPWRUP | MT6397_RG_AUDHPLPWRUP |
		     MT6397_RG_AUDHPRPWRUP);
	regmap_write(map, MT6397_AUDBUF_CFG1, MT6397_RG_HPOUTPUTSTBENH);
	usleep_range(100, 150);
	regmap_write(map, MT6397_AUDBUF_CFG2,
		     FIELD_PREP(MT6397_RG_HPOUTSTB_RSEL, 2) |
		     MT6397_RG_HSOUTPUTSTBENH);
	usleep_range(100, 150);
	if (first)
		mt6391_dac_on(priv);

	/* Headphone buffers on, handset buffer off, both fed by the DAC */
	regmap_update_bits(map, MT6397_AUDBUF_CFG0,
			   MT6397_RG_AUDHPRPWRUP | MT6397_RG_AUDHPLPWRUP |
			   MT6397_RG_AUDHSPWRUP,
			   MT6397_RG_AUDHPRPWRUP | MT6397_RG_AUDHPLPWRUP);
	regmap_update_bits(map, MT6397_AUDBUF_CFG0, MT6397_RG_AUDHPRMUXINPUTSEL,
			   FIELD_PREP(MT6397_RG_AUDHPRMUXINPUTSEL,
				      MT6397_HP_MUX_AUDIO));
	regmap_update_bits(map, MT6397_AUDBUF_CFG0, MT6397_RG_AUDHPLMUXINPUTSEL,
			   FIELD_PREP(MT6397_RG_AUDHPLMUXINPUTSEL,
				      MT6397_HP_MUX_AUDIO));
	regmap_clear_bits(map, MT6397_AFUNC_AUD_CON2,
			  MT6397_CCI_AUD_DAC_ANA_MUTE);

	/* Only now that the output is connected, raise the gain */
	mt6391_hp_ramp_gain(priv, start, priv->hp_gain);
	priv->hp_on = true;

	msleep(MT6391_AMP_SETTLE_MS);
}

static void mt6391_hp_amp_off(struct mt6391_priv *priv, bool last)
{
	unsigned int start[2] = {
		mt6391_hp_start_gain(priv->hp_gain[0]),
		mt6391_hp_start_gain(priv->hp_gain[1]),
	};
	struct regmap *map = priv->regmap;

	priv->hp_on = false;
	mt6391_hp_ramp_gain(priv, priv->hp_gain, start);

	regmap_set_bits(map, MT6397_AFUNC_AUD_CON2, MT6397_CCI_AUD_DAC_ANA_MUTE);
	regmap_clear_bits(map, MT6397_AUDBUF_CFG0,
			  MT6397_RG_AUDHPRMUXINPUTSEL |
			  MT6397_RG_AUDHPLMUXINPUTSEL |
			  MT6397_RG_AUDHPRPWRUP | MT6397_RG_AUDHPLPWRUP |
			  MT6397_RG_AUDHSPWRUP);
	mt6391_hp_write_gain(priv, MT6391_HP_GAIN_MUTE, MT6391_HP_GAIN_MUTE);
	if (last) {
		regmap_write(map, MT6397_IBIASDIST_CFG0,
			     MT6397_AUDBIASADJ | MT6397_RG_AUDIBIASPWRDN);
		regmap_write(map, MT6397_AUDDAC_CON0, 0);
		regmap_clear_bits(map, MT6397_AUDCLKGEN_CFG0,
				  MT6397_RG_RSTB_DECODER);
		mt6391_dl_analog_off(priv);
	}
	regmap_clear_bits(map, MT6397_AFUNC_AUD_CON2,
			  MT6397_CCI_AUD_DAC_ANA_MUTE);
}

static int mt6391_aud_ck_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6391_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		regmap_set_bits(priv->regmap, MT6397_TOP_CKCON1,
				MT6397_RG_CLKSQ_EN);
		regmap_write(priv->regmap, MT6397_TOP_CKPDN_CLR,
			     MT6397_RG_AUD_26M_PDN | MT6397_RG_AUD_13M_PDN);
		break;
	case SND_SOC_DAPM_POST_PMD:
		regmap_clear_bits(priv->regmap, MT6397_TOP_CKCON1,
				  MT6397_RG_CLKSQ_EN);
		regmap_write(priv->regmap, MT6397_TOP_CKPDN_SET,
			     MT6397_RG_AUD_26M_PDN | MT6397_RG_AUD_13M_PDN);
		break;
	}

	return 0;
}

/*
 * Each amplifier points the one zero-crossing detector at its own output as
 * it powers up, so with both on it serves the last one. It is only switched
 * off once the downlink is down, to avoid a pop.
 */
static int mt6391_zcd_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6391_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	if (event == SND_SOC_DAPM_POST_PMD)
		regmap_clear_bits(priv->regmap, MT6397_ZCD_CON0,
				  MT6397_RG_AUDZCDMUXSEL |
				  MT6397_RG_AUDZCDENABLE);

	return 0;
}

static int mt6391_dl_src_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6391_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	struct regmap *map = priv->regmap;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		regmap_update_bits(map, MT6397_AFE_PMIC_NEWIF_CFG0,
				   MT6397_NEWIF_DL_RATE,
				   FIELD_PREP(MT6397_NEWIF_DL_RATE,
					      priv->dl_rate_mode));
		/*
		 * SDM FIFO clock, scrambler, SDM, then the FIFO, and the
		 * downlink source: MediaTek's values, whose fields are not
		 * documented.
		 */
		regmap_write(map, MT6397_AFUNC_AUD_CON2, 0x0006);
		regmap_write(map, MT6397_AFUNC_AUD_CON0, 0xc3a1);
		regmap_write(map, MT6397_AFUNC_AUD_CON2, 0x0003);
		regmap_write(map, MT6397_AFUNC_AUD_CON2, 0x000b);
		regmap_write(map, MT6397_AFE_DL_SDM_CON1, 0x001e);
		regmap_write(map, MT6397_AFE_DL_SRC2_CON0_H,
			     FIELD_PREP(MT6397_DL_2_INPUT_MODE,
					priv->dl_rate_mode) | 0x0300);
		regmap_write(map, MT6397_AFE_UL_DL_CON0, 0x007f);
		regmap_write(map, MT6397_AFE_DL_SRC2_CON0_L,
			     0x1800 | MT6397_DL_2_SRC_ON);
		priv->dl_src_on = true;
		break;
	case SND_SOC_DAPM_POST_PMD:
		priv->dl_src_on = false;
		regmap_write(map, MT6397_AFE_DL_SRC2_CON0_L, 0x1800);
		regmap_write(map, MT6397_AFE_UL_DL_CON0, 0);
		break;
	}

	return 0;
}

static int mt6391_spk_ck_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6391_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	unsigned int mask = MT6397_RG_SPK_CK_PDN | MT6397_RG_SPK_PWM_DIV_PDN |
			    MT6397_RG_SPK_DIV_PDN;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		regmap_write(priv->regmap, MT6397_TOP_CKPDN_CLR, mask);
		break;
	case SND_SOC_DAPM_POST_PMD:
		regmap_write(priv->regmap, MT6397_TOP_CKPDN_SET, mask);
		break;
	}

	return 0;
}

static int mt6391_spk_amp_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6391_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		mt6391_spk_amp_on(priv, priv->dl_users++ == 0);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		mt6391_spk_amp_off(priv, --priv->dl_users == 0);
		break;
	}

	return 0;
}

static int mt6391_hp_amp_event(struct snd_soc_dapm_widget *w,
			       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_to_component(w->dapm);
	struct mt6391_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		mt6391_hp_amp_on(priv, priv->dl_users++ == 0);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		mt6391_hp_amp_off(priv, --priv->dl_users == 0);
		break;
	}

	return 0;
}

static int mt6391_spk_volume_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct mt6391_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	ucontrol->value.integer.value[0] = priv->spk_gain[0];
	ucontrol->value.integer.value[1] = priv->spk_gain[1];

	return 0;
}

static int mt6391_spk_volume_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct mt6391_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	struct snd_soc_dapm_context *dapm = snd_soc_component_to_dapm(cmpnt);
	long l = ucontrol->value.integer.value[0];
	long r = ucontrol->value.integer.value[1];
	int ret = 0;

	if (l < 0 || l > 15 || r < 0 || r > 15)
		return -EINVAL;

	/* The amplifier's events read the gain too */
	snd_soc_dapm_mutex_lock(dapm);
	if (l != priv->spk_gain[0] || r != priv->spk_gain[1]) {
		priv->spk_gain[0] = l;
		priv->spk_gain[1] = r;
		regmap_update_bits(priv->regmap, MT6397_SPK_CON9,
				   MT6397_RG_SPKPGA_GAINL,
				   FIELD_PREP(MT6397_RG_SPKPGA_GAINL, l));
		regmap_update_bits(priv->regmap, MT6397_SPK_CON5,
				   MT6397_RG_SPKPGA_GAINR,
				   FIELD_PREP(MT6397_RG_SPKPGA_GAINR, r));
		ret = 1;
	}
	snd_soc_dapm_mutex_unlock(dapm);

	return ret;
}

static long mt6391_hp_gain_to_ctl(unsigned int reg)
{
	if (reg > MT6391_HP_GAIN_LOW)
		return 0;

	return MT6391_HP_GAIN_MAX - reg;
}

static int mt6391_hp_volume_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct mt6391_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	ucontrol->value.integer.value[0] = mt6391_hp_gain_to_ctl(priv->hp_gain[0]);
	ucontrol->value.integer.value[1] = mt6391_hp_gain_to_ctl(priv->hp_gain[1]);

	return 0;
}

static int mt6391_hp_volume_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct mt6391_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	struct snd_soc_dapm_context *dapm = snd_soc_component_to_dapm(cmpnt);
	unsigned int gain[2];
	int i, ret = 0;

	for (i = 0; i < 2; i++) {
		long val = ucontrol->value.integer.value[i];

		if (val < 0 || val > MT6391_HP_GAIN_MAX)
			return -EINVAL;
		gain[i] = val ? MT6391_HP_GAIN_MAX - val : MT6391_HP_GAIN_MUTE;
	}

	/* Serialise with the amplifier's events, which ramp the gain too */
	snd_soc_dapm_mutex_lock(dapm);
	if (gain[0] != priv->hp_gain[0] || gain[1] != priv->hp_gain[1]) {
		if (priv->hp_on)
			mt6391_hp_ramp_gain(priv, priv->hp_gain, gain);
		priv->hp_gain[0] = gain[0];
		priv->hp_gain[1] = gain[1];
		ret = 1;
	}
	snd_soc_dapm_mutex_unlock(dapm);

	return ret;
}

/* Register 0 mutes the speaker, 1 is 0 dB, and 2-15 are +4 dB to +17 dB */
static const DECLARE_TLV_DB_RANGE(mt6391_spk_tlv,
	0, 1, TLV_DB_SCALE_ITEM(-400, 400, 1),
	2, 15, TLV_DB_SCALE_ITEM(400, 100, 0)
);

static const DECLARE_TLV_DB_RANGE(mt6391_hp_tlv,
	0, 0, TLV_DB_SCALE_ITEM(-4000, 0, 0),
	1, 13, TLV_DB_SCALE_ITEM(-400, 100, 0)
);

static const struct snd_kcontrol_new mt6391_snd_controls[] = {
	SOC_DOUBLE_EXT_TLV("Speaker Volume", SND_SOC_NOPM, 0, 1, 15, 0,
			   mt6391_spk_volume_get, mt6391_spk_volume_put,
			   mt6391_spk_tlv),
	SOC_DOUBLE_EXT_TLV("Headphone Volume", SND_SOC_NOPM, 0, 1,
			   MT6391_HP_GAIN_MAX, 0,
			   mt6391_hp_volume_get, mt6391_hp_volume_put,
			   mt6391_hp_tlv),
};

static const struct snd_soc_dapm_widget mt6391_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY_S("AUD_CK", SUPPLY_SEQ_AUD_CK, SND_SOC_NOPM, 0, 0,
			      mt6391_aud_ck_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY_S("ZCD", SUPPLY_SEQ_ZCD, SND_SOC_NOPM, 0, 0,
			      mt6391_zcd_event, SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY_S("DL_SRC", SUPPLY_SEQ_DL_SRC, SND_SOC_NOPM, 0, 0,
			      mt6391_dl_src_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY_S("SPK_CK", SUPPLY_SEQ_SPK_CK, SND_SOC_NOPM, 0, 0,
			      mt6391_spk_ck_event,
			      SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN("AIF RX", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_OUT_DRV_E("Speaker Amp", SND_SOC_NOPM, 0, 0, NULL, 0,
			       mt6391_spk_amp_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUT_DRV_E("Headphone Amp", SND_SOC_NOPM, 0, 0, NULL, 0,
			       mt6391_hp_amp_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_OUTPUT("SPKL"),
	SND_SOC_DAPM_OUTPUT("SPKR"),
	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),
};

static const struct snd_soc_dapm_route mt6391_dapm_routes[] = {
	{ "DAC", NULL, "AIF RX" },
	{ "DAC", NULL, "AUD_CK" },
	{ "DAC", NULL, "DL_SRC" },

	{ "Speaker Amp", NULL, "DAC" },
	{ "Speaker Amp", NULL, "ZCD" },
	{ "Speaker Amp", NULL, "SPK_CK" },
	{ "SPKL", NULL, "Speaker Amp" },
	{ "SPKR", NULL, "Speaker Amp" },

	{ "Headphone Amp", NULL, "DAC" },
	{ "Headphone Amp", NULL, "ZCD" },
	{ "HPL", NULL, "Headphone Amp" },
	{ "HPR", NULL, "Headphone Amp" },
};

static int mt6391_dai_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_component *cmpnt = dai->component;
	struct mt6391_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	struct snd_soc_dapm_context *dapm = snd_soc_component_to_dapm(cmpnt);
	int mode = mt6391_dl_rate_mode(params_rate(params));

	if (mode < 0)
		return mode;

	/*
	 * A downlink still powered from the previous stream would otherwise
	 * keep its rate, and the SRC is silent at the wrong one.
	 */
	snd_soc_dapm_mutex_lock(dapm);
	priv->dl_rate_mode = mode;
	if (priv->dl_src_on)
		mt6391_set_dl_rate(priv);
	snd_soc_dapm_mutex_unlock(dapm);

	return 0;
}

static const struct snd_soc_dai_ops mt6391_dai_ops = {
	.hw_params = mt6391_dai_hw_params,
};

static struct snd_soc_dai_driver mt6391_dai_driver[] = {
	{
		.name = "mt6397-codec-aif1",
		.playback = {
			.stream_name = "AIF1 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &mt6391_dai_ops,
	},
};

static void mt6391_read_hp_trim(struct mt6391_priv *priv)
{
	unsigned int fuse, l = 2, r = 2, l_fine = 0, r_fine = 0;

	regmap_read(priv->regmap, MT6397_EFUSE_DOUT_192_207, &fuse);
	if (fuse & MT6397_EFUSE_HP_TRIM_EN) {
		l = FIELD_GET(MT6397_EFUSE_HPL_TRIM, fuse);
		r = FIELD_GET(MT6397_EFUSE_HPR_TRIM, fuse);
		l_fine = FIELD_GET(MT6397_EFUSE_HPL_FINE_TRIM, fuse);
		r_fine = FIELD_GET(MT6397_EFUSE_HPR_FINE_TRIM, fuse);
	}

	priv->hp_trim = MT6397_RG_AUDHPTRIM_EN |
			FIELD_PREP(MT6397_RG_AUDHPLTRIM, l) |
			FIELD_PREP(MT6397_RG_AUDHPRTRIM, r) |
			FIELD_PREP(MT6397_RG_AUDHPLFINETRIM, l_fine) |
			FIELD_PREP(MT6397_RG_AUDHPRFINETRIM, r_fine);
}

static unsigned int mt6391_read_spk_trim(struct mt6391_priv *priv,
					 unsigned int sel, unsigned int sel_pol)
{
	unsigned int val, trim;

	regmap_write(priv->regmap, MT6397_TEST_CON0, sel);
	regmap_read(priv->regmap, MT6397_TEST_OUT_L, &val);
	trim = val & 0xf;
	regmap_write(priv->regmap, MT6397_TEST_CON0, sel_pol);
	regmap_read(priv->regmap, MT6397_TEST_OUT_L, &val);
	trim |= (val & BIT(0)) << 4;

	return MT6397_SPK_OFFSET_MODE |
	       FIELD_PREP(MT6397_SPK_LEAD_SW, !!(val & BIT(1))) |
	       FIELD_PREP(MT6397_SPK_OFFSET_SW, trim);
}

/*
 * Run the speaker amplifiers' DC offset calibration once, and keep the
 * result to program on every power-up.
 */
static void mt6391_spk_auto_trim(struct mt6391_priv *priv)
{
	struct regmap *map = priv->regmap;
	unsigned int val;
	int ret;

	regmap_set_bits(map, MT6397_AFUNC_AUD_CON2, MT6397_CCI_AUD_DAC_ANA_MUTE);
	mt6391_dl_analog_on(priv);
	mt6391_iv_buffer_on(priv);
	mt6391_dac_on(priv);
	regmap_clear_bits(map, MT6397_AFUNC_AUD_CON2,
			  MT6397_CCI_AUD_DAC_ANA_MUTE);

	regmap_clear_bits(map, MT6397_SPK_CON1, MT6397_SPK_TRIM);
	regmap_clear_bits(map, MT6397_SPK_CON4, MT6397_SPK_TRIM);
	regmap_write(map, MT6397_SPK_CON9, 0x0018);
	regmap_write(map, MT6397_SPK_CON0, MT6397_SPK_TRIM_EN);
	regmap_write(map, MT6397_SPK_CON3, MT6397_SPK_TRIM_EN);
	regmap_update_bits(map, MT6397_SPK_CON0, MT6397_RG_SPK_GAIN,
			   FIELD_PREP(MT6397_RG_SPK_GAIN, 3));
	regmap_update_bits(map, MT6397_SPK_CON3, MT6397_RG_SPK_GAIN,
			   FIELD_PREP(MT6397_RG_SPK_GAIN, 3));
	regmap_update_bits(map, MT6397_SPK_CON9, MT6397_RG_SPKPGA_GAINL,
			   FIELD_PREP(MT6397_RG_SPKPGA_GAINL, 1));
	regmap_update_bits(map, MT6397_SPK_CON5, MT6397_RG_SPKPGA_GAINR,
			   FIELD_PREP(MT6397_RG_SPKPGA_GAINR, 1));
	regmap_set_bits(map, MT6397_SPK_CON0, MT6397_SPK_EN);
	regmap_set_bits(map, MT6397_SPK_CON3, MT6397_SPK_EN);

	/* MediaTek measured the calibration as taking about 13 ms */
	usleep_range(15000, 16000);
	ret = regmap_read_poll_timeout(map, MT6397_SPK_CON1, val,
				       val & MT6397_SPK_TRIM_DONE, 100, 5000);
	if (!ret)
		ret = regmap_read_poll_timeout(map, MT6397_SPK_CON4, val,
					       val & MT6397_SPK_TRIM_DONE,
					       100, 5000);
	if (ret)
		dev_warn(priv->dev, "speaker offset trim did not complete\n");

	regmap_write(map, MT6397_SPK_CON9, 0);
	regmap_clear_bits(map, MT6397_SPK_CON5, MT6397_RG_SPKPGA_GAINR);
	regmap_clear_bits(map, MT6397_SPK_CON0, MT6397_SPK_EN);
	regmap_clear_bits(map, MT6397_SPK_CON3, MT6397_SPK_EN);

	priv->spk_trim[0] = mt6391_read_spk_trim(priv, MT6397_TEST_SPKL_TRIM,
						 MT6397_TEST_SPKL_TRIM_POL);
	priv->spk_trim[1] = mt6391_read_spk_trim(priv, MT6397_TEST_SPKR_TRIM,
						 MT6397_TEST_SPKR_TRIM_POL);

	/* The first revision's right channel calibration is not usable */
	regmap_read(map, MT6397_CID, &val);
	if (val == MT6397_CID_E1)
		priv->spk_trim[1] = MT6397_SPK_OFFSET_MODE;

	regmap_set_bits(map, MT6397_AFUNC_AUD_CON2, MT6397_CCI_AUD_DAC_ANA_MUTE);
	regmap_write(map, MT6397_SPK_CON0, 0);
	regmap_write(map, MT6397_SPK_CON3, 0);
	regmap_write(map, MT6397_SPK_CON11, 0);
	regmap_clear_bits(map, MT6397_ZCD_CON0,
			  MT6397_RG_AUDZCDMUXSEL | MT6397_RG_AUDZCDENABLE);
	regmap_clear_bits(map, MT6397_AUDCLKGEN_CFG0, MT6397_RG_RSTB_DECODER);
	regmap_write(map, MT6397_AUDDAC_CON0, 0);
	regmap_write(map, MT6397_AUD_IV_CFG0, 0);
	regmap_write(map, MT6397_IBIASDIST_CFG0,
		     MT6397_AUDBIASADJ | MT6397_RG_AUDIBIASPWRDN);
	mt6391_dl_analog_off(priv);
	regmap_clear_bits(map, MT6397_AFUNC_AUD_CON2,
			  MT6397_CCI_AUD_DAC_ANA_MUTE);
}

static void mt6391_init_reg(struct mt6391_priv *priv)
{
	struct regmap *map = priv->regmap;

	regmap_set_bits(map, MT6397_AFUNC_AUD_CON2, MT6397_CCI_AUD_DAC_ANA_MUTE);
	mt6391_hp_write_gain(priv, MT6391_HP_GAIN_MUTE, MT6391_HP_GAIN_MUTE);
	regmap_clear_bits(map, MT6397_AUDBUF_CFG0,
			  MT6397_RG_AUDHPRMUXINPUTSEL |
			  MT6397_RG_AUDHPLMUXINPUTSEL |
			  MT6397_RG_AUDHPRPWRUP | MT6397_RG_AUDHPLPWRUP |
			  MT6397_RG_AUDHSPWRUP);
	regmap_write(map, MT6397_IBIASDIST_CFG0,
		     MT6397_AUDBIASADJ | MT6397_RG_AUDIBIASPWRDN);
	regmap_write(map, MT6397_AUDDAC_CON0, 0);
	regmap_clear_bits(map, MT6397_AUDCLKGEN_CFG0, MT6397_RG_RSTB_DECODER);
	mt6391_dl_analog_off(priv);
	regmap_clear_bits(map, MT6397_AFUNC_AUD_CON2,
			  MT6397_CCI_AUD_DAC_ANA_MUTE);
	regmap_write(map, MT6397_ZCD_CON0, 0);
	regmap_set_bits(map, MT6397_AFE_PMIC_NEWIF_CFG2, MT6397_NEWIF_SCK_INV);
}

static void mt6391_accdet_clear_irq(struct mt6391_priv *priv)
{
	unsigned int val;

	regmap_set_bits(priv->regmap, MT6397_ACCDET_CON11,
			MT6397_ACCDET_IRQ_CLR);
	regmap_read_poll_timeout(priv->regmap, MT6397_ACCDET_CON11, val,
				 !(val & MT6397_ACCDET_IRQ), 100, 2000);
	regmap_clear_bits(priv->regmap, MT6397_ACCDET_CON11,
			  MT6397_ACCDET_IRQ_CLR);
}

static unsigned int mt6391_accdet_ab(struct mt6391_priv *priv)
{
	unsigned int val;

	regmap_read(priv->regmap, MT6397_ACCDET_CON13, &val);
	return FIELD_GET(MT6397_ACCDET_MEM_IN, val);
}

static void mt6391_jack_work(struct work_struct *work)
{
	struct mt6391_priv *priv = container_of(to_delayed_work(work),
						struct mt6391_priv, jack_work);
	int type, btn = 0;

	switch (mt6391_accdet_ab(priv)) {
	case MT6391_ACCDET_AB_OPEN:
		type = 0;
		break;
	case MT6391_ACCDET_AB_HEADSET:
		type = SND_JACK_HEADSET;
		break;
	case MT6391_ACCDET_AB_GROUND:
		if (priv->jack_type == SND_JACK_HEADSET) {
			type = SND_JACK_HEADSET;
			btn = SND_JACK_BTN_0;
		} else {
			type = SND_JACK_HEADPHONE;
		}
		break;
	default:
		return;
	}

	WRITE_ONCE(priv->jack_type, type);
	snd_soc_jack_report(priv->jack, type | btn,
			    SND_JACK_HEADSET | SND_JACK_BTN_0);
}

static irqreturn_t mt6391_accdet_irq(int irq, void *data)
{
	struct mt6391_priv *priv = data;
	unsigned long delay = 0;

	mt6391_accdet_clear_irq(priv);

	if (!READ_ONCE(priv->jack_type) &&
	    mt6391_accdet_ab(priv) != MT6391_ACCDET_AB_OPEN)
		delay = msecs_to_jiffies(MT6391_ACCDET_SETTLE_MS);
	mod_delayed_work(system_power_efficient_wq, &priv->jack_work, delay);

	return IRQ_HANDLED;
}

static void mt6391_accdet_enable(struct mt6391_priv *priv)
{
	struct regmap *map = priv->regmap;

	regmap_write(map, MT6397_TOP_CKPDN_CLR, MT6397_RG_ACCDET_CK_PDN);
	regmap_write(map, MT6397_TOP_RST_CON_SET, MT6397_RG_ACCDET_RST);
	regmap_write(map, MT6397_TOP_RST_CON_CLR, MT6397_RG_ACCDET_RST);

	regmap_write(map, MT6397_ACCDET_CON0, MT6391_ACCDET_RSV_2V8);
	regmap_write(map, MT6397_ACCDET_CON3, MT6391_ACCDET_PWM_WIDTH - 1);
	regmap_write(map, MT6397_ACCDET_CON4, MT6391_ACCDET_PWM_WIDTH - 1);
	regmap_write(map, MT6397_ACCDET_CON5, MT6397_ACCDET_FALL_DELAY |
		     FIELD_PREP(MT6397_ACCDET_RISE_DELAY,
				MT6391_ACCDET_RISE_DELAY));
	regmap_write(map, MT6397_ACCDET_CON6, MT6391_ACCDET_DEBOUNCE);
	regmap_write(map, MT6397_ACCDET_CON7, MT6391_ACCDET_DEBOUNCE);
	regmap_write(map, MT6397_ACCDET_CON9, MT6391_ACCDET_DEBOUNCE_OPEN);
	regmap_write(map, MT6397_ACCDET_CON2,
		     MT6397_ACCDET_CMP_EN | MT6397_ACCDET_VTH_EN |
		     MT6397_ACCDET_MBIAS_EN | MT6397_ACCDET_CMP_PWM_IDLE |
		     MT6397_ACCDET_VTH_PWM_IDLE | MT6397_ACCDET_MBIAS_PWM_IDLE);
	regmap_write(map, MT6397_ACCDET_CON1, MT6397_ACCDET_EN);

	/* An empty jack stays at the reset state and raises no interrupt */
	priv->jack_type = 0;
	enable_irq(priv->accdet_irq);
	mod_delayed_work(system_power_efficient_wq, &priv->jack_work,
			 msecs_to_jiffies(MT6391_ACCDET_SETTLE_MS));
}

static void mt6391_accdet_disable(struct mt6391_priv *priv)
{
	disable_irq(priv->accdet_irq);
	cancel_delayed_work_sync(&priv->jack_work);

	regmap_write(priv->regmap, MT6397_ACCDET_CON1, 0);
	regmap_write(priv->regmap, MT6397_ACCDET_CON2, 0);
	mt6391_accdet_clear_irq(priv);
	regmap_write(priv->regmap, MT6397_TOP_CKPDN_SET,
		     MT6397_RG_ACCDET_CK_PDN);
}

static int mt6391_set_jack(struct snd_soc_component *cmpnt,
			   struct snd_soc_jack *jack, void *data)
{
	struct mt6391_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	if (!priv->accdet_irq)
		return 0;

	if (priv->jack)
		mt6391_accdet_disable(priv);

	priv->jack = jack;
	if (jack)
		mt6391_accdet_enable(priv);

	return 0;
}

static int mt6391_codec_probe(struct snd_soc_component *cmpnt)
{
	struct mt6391_priv *priv = snd_soc_component_get_drvdata(cmpnt);
	unsigned int clks = MT6397_RG_AUD_26M_PDN | MT6397_RG_AUD_13M_PDN |
			    MT6397_RG_SPK_CK_PDN | MT6397_RG_SPK_PWM_DIV_PDN |
			    MT6397_RG_SPK_DIV_PDN;

	snd_soc_component_init_regmap(cmpnt, priv->regmap);

	regmap_set_bits(priv->regmap, MT6397_TOP_CKCON1, MT6397_RG_CLKSQ_EN);
	regmap_write(priv->regmap, MT6397_TOP_CKPDN_CLR, clks);

	mt6391_init_reg(priv);
	mt6391_read_hp_trim(priv);
	mt6391_spk_auto_trim(priv);

	regmap_clear_bits(priv->regmap, MT6397_TOP_CKCON1, MT6397_RG_CLKSQ_EN);
	regmap_write(priv->regmap, MT6397_TOP_CKPDN_SET, clks);

	return 0;
}

static void mt6391_codec_remove(struct snd_soc_component *cmpnt)
{
	mt6391_set_jack(cmpnt, NULL, NULL);
}

/* The bias is off in suspend; resume classifies the jack again */
static int mt6391_codec_suspend(struct snd_soc_component *cmpnt)
{
	struct mt6391_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	if (priv->jack)
		mt6391_accdet_disable(priv);

	return 0;
}

static int mt6391_codec_resume(struct snd_soc_component *cmpnt)
{
	struct mt6391_priv *priv = snd_soc_component_get_drvdata(cmpnt);

	if (priv->jack)
		mt6391_accdet_enable(priv);

	return 0;
}

static const struct snd_soc_component_driver mt6391_component_driver = {
	.probe = mt6391_codec_probe,
	.remove = mt6391_codec_remove,
	.suspend = mt6391_codec_suspend,
	.resume = mt6391_codec_resume,
	.set_jack = mt6391_set_jack,
	.controls = mt6391_snd_controls,
	.num_controls = ARRAY_SIZE(mt6391_snd_controls),
	.dapm_widgets = mt6391_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt6391_dapm_widgets),
	.dapm_routes = mt6391_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(mt6391_dapm_routes),
	.endianness = 1,
};

static int mt6391_platform_probe(struct platform_device *pdev)
{
	struct mt6397_chip *mt6397 = dev_get_drvdata(pdev->dev.parent);
	struct mt6391_priv *priv;
	int ret;

	priv = devm_kzalloc_obj(&pdev->dev, *priv);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	priv->regmap = mt6397->regmap;
	priv->dl_rate_mode = mt6391_dl_rate_mode(48000);
	priv->hp_gain[0] = MT6391_HP_GAIN_MUTE;
	priv->hp_gain[1] = MT6391_HP_GAIN_MUTE;
	INIT_DELAYED_WORK(&priv->jack_work, mt6391_jack_work);
	platform_set_drvdata(pdev, priv);

	ret = platform_get_irq_byname_optional(pdev, "accdet");
	if (ret > 0) {
		priv->accdet_irq = ret;
		ret = devm_request_threaded_irq(&pdev->dev, priv->accdet_irq,
						NULL, mt6391_accdet_irq,
						IRQF_ONESHOT | IRQF_NO_AUTOEN,
						"mt6391-accdet", priv);
		if (ret)
			return ret;
	} else if (ret == -ENXIO) {
		dev_info(&pdev->dev, "no ACCDET interrupt, no jack detection\n");
	} else {
		return ret;
	}

	return devm_snd_soc_register_component(&pdev->dev,
					       &mt6391_component_driver,
					       mt6391_dai_driver,
					       ARRAY_SIZE(mt6391_dai_driver));
}

static const struct of_device_id mt6391_of_match[] = {
	{ .compatible = "mediatek,mt6397-codec" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt6391_of_match);

static struct platform_driver mt6391_platform_driver = {
	.driver = {
		.name = "mt6397-codec",
		.of_match_table = mt6391_of_match,
	},
	.probe = mt6391_platform_probe,
};
module_platform_driver(mt6391_platform_driver);

MODULE_DESCRIPTION("MediaTek MT6391 PMIC audio codec driver");
MODULE_LICENSE("GPL");
