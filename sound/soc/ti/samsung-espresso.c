// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASoC machine driver for the Samsung Galaxy Tab 2 (espresso): a WM1811 codec
 * on McBSP3 of the OMAP4430, with the codec as bit and frame clock provider.
 *
 * Based on the Midas WM1811 driver (sound/soc/samsung/midas_wm1811.c).
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/consumer.h>
#include <linux/input-event-codes.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "../codecs/wm8994.h"

#define ESPRESSO_DEFAULT_FLL1_RATE	11289600U

struct espresso_priv {
	struct clk *mclk1;
	unsigned int mclk1_rate;
	unsigned int fll1_rate;
	bool mclk1_enabled;
	bool aif1clk_forced;

	struct gpio_desc *gpio_headset_detect;
	struct gpio_desc *gpio_headset_key;
	struct iio_channel *adc_headset_detect;
	struct snd_soc_jack headset_jack;
	/* 3-pole, 4-pole, 3-pole by ADC value; "Media", "Volume Up/Down" keys */
	struct snd_soc_jack_zone headset_jack_zones[3];
	struct snd_soc_jack_zone headset_key_zones[3];
	struct snd_soc_jack_gpio headset_gpios[2];
};

static struct snd_soc_jack_pin espresso_headset_jack_pins[] = {
	{
		.pin = "HP",
		.mask = SND_JACK_HEADPHONE,
	},
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},
};

/*
 * Like Midas: the jack switch is on a GPIO, and the voltage of the headset
 * microphone line, read with an ADC while its bias is on, tells a 4-pole
 * headset from headphones and which headset key is pressed.
 */
static int espresso_headset_jack_check(void *data)
{
	struct snd_soc_component *codec = data;
	struct snd_soc_dapm_context *dapm = snd_soc_component_to_dapm(codec);
	struct espresso_priv *priv = snd_soc_card_get_drvdata(codec->card);
	int adc, ret;
	int jack_type;

	if (!gpiod_get_value_cansleep(priv->gpio_headset_detect))
		return 0;

	/* the ADC reads the microphone line only with its bias on */
	ret = snd_soc_dapm_force_enable_pin(dapm, "headset-mic-bias");
	if (ret < 0) {
		dev_err(codec->card->dev,
			"Failed to enable the headset mic bias (%d), assuming headphones\n",
			ret);
		return SND_JACK_HEADPHONE;
	}
	snd_soc_dapm_sync(dapm);

	/* let the voltage settle */
	msleep(20);

	ret = iio_read_channel_processed(priv->adc_headset_detect, &adc);
	if (ret < 0) {
		dev_err(codec->card->dev,
			"Failed to read the ADC (%d), assuming headphones\n", ret);
		jack_type = SND_JACK_HEADPHONE;
	} else {
		dev_dbg(codec->card->dev, "headset detect ADC: %d mV\n", adc);
		jack_type = snd_soc_jack_get_type(&priv->headset_jack, adc);
	}

	snd_soc_dapm_disable_pin(dapm, "headset-mic-bias");
	snd_soc_dapm_sync(dapm);

	return jack_type;
}

static int espresso_headset_key_check(void *data)
{
	struct snd_soc_component *codec = data;
	struct espresso_priv *priv = snd_soc_card_get_drvdata(codec->card);
	int adc, i, ret;

	if (!gpiod_get_value_cansleep(priv->gpio_headset_key))
		return 0;

	/* only a 4-pole headset has keys */
	if (!(priv->headset_jack.status & SND_JACK_MICROPHONE))
		return 0;

	ret = iio_read_channel_processed(priv->adc_headset_detect, &adc);
	if (ret < 0) {
		dev_err(codec->card->dev,
			"Failed to read the ADC (%d), can't tell the key\n", ret);
		return 0;
	}
	dev_dbg(codec->card->dev, "headset key ADC: %d mV\n", adc);

	for (i = 0; i < ARRAY_SIZE(priv->headset_key_zones); i++)
		if (adc >= priv->headset_key_zones[i].min_mv &&
		    adc <= priv->headset_key_zones[i].max_mv)
			return priv->headset_key_zones[i].jack_type;

	return 0;
}

static int espresso_start_fll1(struct snd_soc_pcm_runtime *rtd,
			       unsigned int rate)
{
	struct snd_soc_card *card = rtd->card;
	struct espresso_priv *priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *aif1_dai = snd_soc_rtd_to_codec(rtd, 0);
	int ret;

	if (!rate)
		rate = priv->fll1_rate;
	/*
	 * If no new rate is requested, set FLL1 to a sane default for jack
	 * detection.
	 */
	if (!rate)
		rate = ESPRESSO_DEFAULT_FLL1_RATE;

	if (rate != priv->fll1_rate && priv->fll1_rate) {
		/* while reconfiguring, switch to MCLK1 for SYSCLK */
		ret = snd_soc_dai_set_sysclk(aif1_dai, WM8994_SYSCLK_MCLK1,
					     priv->mclk1_rate,
					     SND_SOC_CLOCK_IN);
		if (ret < 0) {
			dev_err(card->dev, "Unable to switch to MCLK1: %d\n",
				ret);
			return ret;
		}
	}

	ret = snd_soc_dai_set_pll(aif1_dai, WM8994_FLL1, WM8994_FLL_SRC_MCLK1,
				  priv->mclk1_rate, rate);
	if (ret < 0) {
		dev_err(card->dev, "Failed to set FLL1 rate: %d\n", ret);
		return ret;
	}
	priv->fll1_rate = rate;

	ret = snd_soc_dai_set_sysclk(aif1_dai, WM8994_SYSCLK_FLL1,
				     priv->fll1_rate, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(card->dev, "Failed to set SYSCLK source: %d\n", ret);
		return ret;
	}

	return 0;
}

static int espresso_stop_fll1(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct espresso_priv *priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *aif1_dai = snd_soc_rtd_to_codec(rtd, 0);
	int ret;

	ret = snd_soc_dai_set_sysclk(aif1_dai, WM8994_SYSCLK_MCLK1,
				     priv->mclk1_rate, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(card->dev, "Unable to switch to MCLK1: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(aif1_dai, WM8994_FLL1, 0, 0, 0);
	if (ret < 0) {
		dev_err(card->dev, "Unable to stop FLL1: %d\n", ret);
		return ret;
	}

	priv->fll1_rate = 0;

	return 0;
}

static int espresso_aif1_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct espresso_priv *priv = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *aif1_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_dapm_context *dapm =
		snd_soc_component_to_dapm(aif1_dai->component);
	unsigned int pll_out;
	int ret;

	/* AIF1CLK should be at least 3MHz for "optimal performance" */
	if (params_rate(params) == 8000 || params_rate(params) == 11025)
		pll_out = params_rate(params) * 512;
	else
		pll_out = params_rate(params) * 256;

	ret = espresso_start_fll1(rtd, pll_out);
	if (ret < 0)
		return ret;

	/*
	 * The codec provides BCLK and LRCLK, but DAPM powers AIF1CLK only for
	 * a complete path. Without an enabled input or output, McBSP would
	 * get no clocks and the stream would time out, so keep AIF1CLK on
	 * while the stream is set up.
	 */
	if (!priv->aif1clk_forced) {
		ret = snd_soc_dapm_force_enable_pin(dapm, "AIF1CLK");
		if (ret < 0)
			return ret;
		priv->aif1clk_forced = true;
	}

	return 0;
}

static int espresso_aif1_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct espresso_priv *priv = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *aif1_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_dapm_context *dapm =
		snd_soc_component_to_dapm(aif1_dai->component);

	/* both directions share the clocks: release them with the last one */
	if (!priv->aif1clk_forced || snd_soc_dai_active(aif1_dai) > 1)
		return 0;

	priv->aif1clk_forced = false;
	return snd_soc_dapm_disable_pin(dapm, "AIF1CLK");
}

static const struct snd_soc_ops espresso_aif1_ops = {
	.hw_params = espresso_aif1_hw_params,
	.hw_free = espresso_aif1_hw_free,
};

static const struct snd_kcontrol_new espresso_controls[] = {
	SOC_DAPM_PIN_SWITCH("HP"),
	SOC_DAPM_PIN_SWITCH("SPK"),
	SOC_DAPM_PIN_SWITCH("RCV"),
	SOC_DAPM_PIN_SWITCH("LINE"),

	SOC_DAPM_PIN_SWITCH("Main Mic"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
};

static const struct snd_soc_dapm_widget espresso_dapm_widgets[] = {
	SND_SOC_DAPM_HP("HP", NULL),
	SND_SOC_DAPM_SPK("SPK", NULL),
	SND_SOC_DAPM_SPK("RCV", NULL),
	SND_SOC_DAPM_LINE("LINE", NULL),

	SND_SOC_DAPM_MIC("Main Mic", NULL),
	SND_SOC_DAPM_REGULATOR_SUPPLY("mic-bias", 0, 0),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_REGULATOR_SUPPLY("headset-mic-bias", 0, 0),
};

/* Default routing; supplemented by the audio-routing DT property */
static const struct snd_soc_dapm_route espresso_dapm_routes[] = {
	/* Bind microphones with their respective regulator supplies */
	{ "Main Mic", NULL, "mic-bias" },
	{ "Headset Mic", NULL, "headset-mic-bias" },
};

static int espresso_set_bias_level(struct snd_soc_card *card,
				   struct snd_soc_dapm_context *dapm,
				   enum snd_soc_bias_level level)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_get_pcm_runtime(card,
						  &card->dai_link[0]);
	struct snd_soc_dai *aif1_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct espresso_priv *priv = snd_soc_card_get_drvdata(card);
	int ret;

	if (snd_soc_dapm_to_dev(dapm) != aif1_dai->dev)
		return 0;

	switch (level) {
	case SND_SOC_BIAS_STANDBY:
		/*
		 * MCLK1 is the FLL1 reference and SYSCLK whenever FLL1 is
		 * stopped or reprogrammed. The codec takes a reference on its
		 * SYSCLK source only when AIF1CLK powers up, and drops the one
		 * for the FLL1 reference while it reprograms FLL1, so hold
		 * MCLK1 as long as the codec is biased: otherwise the gated
		 * 26 MHz clock stops under SYSCLK and the codec loses
		 * register writes.
		 */
		if (!priv->mclk1_enabled) {
			ret = clk_prepare_enable(priv->mclk1);
			if (ret)
				return ret;
			priv->mclk1_enabled = true;
		}
		/*
		 * Stop FLL1 only on the way down: coming up from OFF,
		 * hw_params has already set it up for the stream.
		 */
		if (snd_soc_dapm_get_bias_level(dapm) != SND_SOC_BIAS_PREPARE)
			return 0;
		return espresso_stop_fll1(rtd);
	case SND_SOC_BIAS_PREPARE:
		return espresso_start_fll1(rtd, 0);
	case SND_SOC_BIAS_OFF:
		/*
		 * A stream with no complete path only powers AIF1CLK, a
		 * supply, which takes the bias to STANDBY but never to
		 * PREPARE, so FLL1 is still running here.
		 */
		if (priv->fll1_rate) {
			ret = espresso_stop_fll1(rtd);
			if (ret)
				return ret;
		}
		if (priv->mclk1_enabled) {
			clk_disable_unprepare(priv->mclk1);
			priv->mclk1_enabled = false;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int espresso_late_probe(struct snd_soc_card *card)
{
	struct espresso_priv *priv = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_runtime *rtd = snd_soc_get_pcm_runtime(card,
						  &card->dai_link[0]);
	struct snd_soc_dai *aif1_dai = snd_soc_rtd_to_codec(rtd, 0);
	int ret;

	/* Use MCLK1 as SYSCLK for boot */
	ret = snd_soc_dai_set_sysclk(aif1_dai, WM8994_SYSCLK_MCLK1,
				     priv->mclk1_rate, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(aif1_dai->dev, "Failed to switch to MCLK1: %d\n", ret);
		return ret;
	}

	if (!priv->gpio_headset_detect)
		return 0;

	ret = snd_soc_card_jack_new_pins(card, "Headset",
					 SND_JACK_HEADSET | SND_JACK_BTN_0 |
					 SND_JACK_BTN_1 | SND_JACK_BTN_2,
					 &priv->headset_jack,
					 espresso_headset_jack_pins,
					 ARRAY_SIZE(espresso_headset_jack_pins));
	if (ret)
		return ret;

	ret = snd_soc_jack_add_zones(&priv->headset_jack,
				     ARRAY_SIZE(priv->headset_jack_zones),
				     priv->headset_jack_zones);
	if (ret)
		return ret;

	snd_jack_set_key(priv->headset_jack.jack, SND_JACK_BTN_0, KEY_MEDIA);
	snd_jack_set_key(priv->headset_jack.jack, SND_JACK_BTN_1, KEY_VOLUMEUP);
	snd_jack_set_key(priv->headset_jack.jack, SND_JACK_BTN_2,
			 KEY_VOLUMEDOWN);

	priv->headset_gpios[0] = (struct snd_soc_jack_gpio) {
		.name = "Headset Jack",
		.report = SND_JACK_HEADSET,
		.debounce_time = 150,
		.jack_status_check = espresso_headset_jack_check,
		.data = aif1_dai->component,
		.desc = priv->gpio_headset_detect,
	};
	priv->headset_gpios[1] = (struct snd_soc_jack_gpio) {
		.name = "Headset Key",
		.report = SND_JACK_BTN_0 | SND_JACK_BTN_1 | SND_JACK_BTN_2,
		.debounce_time = 30,
		.jack_status_check = espresso_headset_key_check,
		.data = aif1_dai->component,
		.desc = priv->gpio_headset_key,
	};

	return snd_soc_jack_add_gpios(&priv->headset_jack,
				      ARRAY_SIZE(priv->headset_gpios),
				      priv->headset_gpios);
}

static int espresso_parse_jack(struct device *dev, struct espresso_priv *priv)
{
	struct device_node *np = dev->of_node;
	enum iio_chan_type type;
	u32 fourpole[2], buttons[3];
	int ret, i;

	priv->gpio_headset_detect =
		devm_gpiod_get_optional(dev, "headset-detect", GPIOD_IN);
	if (IS_ERR(priv->gpio_headset_detect))
		return dev_err_probe(dev, PTR_ERR(priv->gpio_headset_detect),
				     "Failed to get the headset detect GPIO\n");
	if (!priv->gpio_headset_detect)
		return 0;

	priv->gpio_headset_key = devm_gpiod_get(dev, "headset-key", GPIOD_IN);
	if (IS_ERR(priv->gpio_headset_key))
		return dev_err_probe(dev, PTR_ERR(priv->gpio_headset_key),
				     "Failed to get the headset key GPIO\n");

	priv->adc_headset_detect = devm_iio_channel_get(dev, "headset-detect");
	if (IS_ERR(priv->adc_headset_detect))
		return dev_err_probe(dev, PTR_ERR(priv->adc_headset_detect),
				     "Failed to get the headset detect ADC\n");

	ret = iio_get_channel_type(priv->adc_headset_detect, &type);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get the ADC type\n");
	if (type != IIO_VOLTAGE)
		return dev_err_probe(dev, -EINVAL,
				     "The headset detect ADC is not a voltage\n");

	/* in mV despite the property names, as on Midas */
	ret = of_property_read_u32_array(np, "samsung,headset-4pole-threshold-microvolt",
					 fourpole, ARRAY_SIZE(fourpole));
	if (ret || fourpole[0] > fourpole[1])
		return dev_err_probe(dev, -EINVAL,
				     "Invalid 4-pole detection thresholds\n");

	ret = of_property_read_u32_array(np, "samsung,headset-button-threshold-microvolt",
					 buttons, ARRAY_SIZE(buttons));
	if (ret || buttons[0] > buttons[1] || buttons[1] > buttons[2])
		return dev_err_probe(dev, -EINVAL,
				     "Invalid headset key thresholds\n");

	priv->headset_jack_zones[0] = (struct snd_soc_jack_zone) {
		.max_mv = fourpole[0],
		.jack_type = SND_JACK_HEADPHONE,
	};
	priv->headset_jack_zones[1] = (struct snd_soc_jack_zone) {
		.min_mv = fourpole[0] + 1,
		.max_mv = fourpole[1],
		.jack_type = SND_JACK_HEADSET,
	};
	priv->headset_jack_zones[2] = (struct snd_soc_jack_zone) {
		.min_mv = fourpole[1] + 1,
		.max_mv = UINT_MAX,
		.jack_type = SND_JACK_HEADPHONE,
	};

	for (i = 0; i < ARRAY_SIZE(buttons); i++) {
		priv->headset_key_zones[i].min_mv = buttons[i];
		priv->headset_key_zones[i].max_mv =
			i < ARRAY_SIZE(buttons) - 1 ? buttons[i + 1] - 1 : UINT_MAX;
		priv->headset_key_zones[i].jack_type = SND_JACK_BTN_0 >> i;
	}

	return 0;
}

SND_SOC_DAILINK_DEFS(wm1811_hifi,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "wm8994-aif1")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link espresso_dai[] = {
	{
		.name = "WM1811 AIF1",
		.stream_name = "HiFi Primary",
		.ops = &espresso_aif1_ops,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBP_CFP,
		SND_SOC_DAILINK_REG(wm1811_hifi),
	},
};

static struct snd_soc_card espresso_card = {
	.name = "Galaxy Tab 2 WM1811",
	.owner = THIS_MODULE,

	.dai_link = espresso_dai,
	.num_links = ARRAY_SIZE(espresso_dai),
	.controls = espresso_controls,
	.num_controls = ARRAY_SIZE(espresso_controls),
	.dapm_widgets = espresso_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(espresso_dapm_widgets),
	.dapm_routes = espresso_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(espresso_dapm_routes),

	.set_bias_level = espresso_set_bias_level,
	.late_probe = espresso_late_probe,
};

static void espresso_put_clk(void *data)
{
	clk_put(data);
}

static int espresso_probe(struct platform_device *pdev)
{
	struct device_node *cpu_dai_node, *codec_dai_node, *np;
	struct snd_soc_card *card = &espresso_card;
	struct device *dev = &pdev->dev;
	struct espresso_priv *priv;
	struct clk *mclk1;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	snd_soc_card_set_drvdata(card, priv);
	card->dev = dev;

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret < 0)
		return dev_err_probe(dev, ret, "Card name is not specified\n");

	ret = espresso_parse_jack(dev, priv);
	if (ret)
		return ret;

	ret = snd_soc_of_parse_audio_routing(card, "audio-routing");
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Audio routing invalid/unspecified\n");

	np = of_get_child_by_name(dev->of_node, "cpu");
	if (!np)
		return dev_err_probe(dev, -EINVAL, "Missing cpu node\n");
	cpu_dai_node = of_parse_phandle(np, "sound-dai", 0);
	of_node_put(np);
	if (!cpu_dai_node)
		return dev_err_probe(dev, -EINVAL,
				     "Parsing cpu/sound-dai failed\n");

	np = of_get_child_by_name(dev->of_node, "codec");
	if (!np) {
		ret = dev_err_probe(dev, -EINVAL, "Missing codec node\n");
		goto put_cpu_dai_node;
	}
	codec_dai_node = of_parse_phandle(np, "sound-dai", 0);
	of_node_put(np);
	if (!codec_dai_node) {
		ret = dev_err_probe(dev, -EINVAL,
				    "Parsing codec/sound-dai failed\n");
		goto put_cpu_dai_node;
	}

	mclk1 = of_clk_get_by_name(codec_dai_node, "MCLK1");
	if (IS_ERR(mclk1)) {
		ret = dev_err_probe(dev, PTR_ERR(mclk1),
				    "Failed to get the codec's MCLK1\n");
		goto put_codec_dai_node;
	}
	ret = devm_add_action_or_reset(dev, espresso_put_clk, mclk1);
	if (ret)
		goto put_codec_dai_node;
	priv->mclk1 = mclk1;
	priv->mclk1_rate = clk_get_rate(mclk1);
	if (!priv->mclk1_rate) {
		ret = dev_err_probe(dev, -EINVAL, "MCLK1 has no rate\n");
		goto put_codec_dai_node;
	}

	espresso_dai[0].codecs->of_node = codec_dai_node;
	espresso_dai[0].cpus->of_node = cpu_dai_node;
	espresso_dai[0].platforms->of_node = cpu_dai_node;

	ret = devm_snd_soc_register_card(dev, card);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Failed to register card\n");
		goto put_codec_dai_node;
	}

	return 0;

put_codec_dai_node:
	of_node_put(codec_dai_node);
put_cpu_dai_node:
	of_node_put(cpu_dai_node);
	return ret;
}

static const struct of_device_id espresso_of_match[] = {
	{ .compatible = "samsung,espresso-audio" },
	{ }
};
MODULE_DEVICE_TABLE(of, espresso_of_match);

static struct platform_driver espresso_driver = {
	.driver = {
		.name = "espresso-audio",
		.of_match_table = espresso_of_match,
		.pm = &snd_soc_pm_ops,
	},
	.probe = espresso_probe,
};
module_platform_driver(espresso_driver);

MODULE_DESCRIPTION("ASoC support for the Samsung Galaxy Tab 2 (espresso)");
MODULE_LICENSE("GPL");
