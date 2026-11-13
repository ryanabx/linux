// SPDX-License-Identifier: GPL-2.0-only
//
// mt8173-mt6391.c  --  MT8173 machine driver for boards using the MT6391
// PMIC codec (the audio block of the MT6397 PMIC)
//
// The codec is reached over the SoC's internal ADDA interface, so there is
// no I2S format or system clock to configure for it.

#include <linux/input.h>
#include <linux/module.h>
#include <linux/of.h>
#include <sound/jack.h>
#include <sound/soc.h>

#include "../../codecs/rt5514.h"

enum {
	DAI_LINK_PLAYBACK,
	DAI_LINK_CAPTURE,
	DAI_LINK_CODEC,
	DAI_LINK_RT5514,
};

SND_SOC_DAILINK_DEFS(playback,
	DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture,
	DAILINK_COMP_ARRAY(COMP_CPU("VUL")),
	DAILINK_COMP_ARRAY(COMP_DUMMY()),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(codec,
	DAILINK_COMP_ARRAY(COMP_CPU("ADDA")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "mt6397-codec-aif1")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(rt5514_codec,
	DAILINK_COMP_ARRAY(COMP_CPU("I2S")),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "rt5514-aif1")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_jack mt8173_mt6391_jack;

static struct snd_soc_jack_pin mt8173_mt6391_jack_pins[] = {
	{
		.pin = "Headphone",
		.mask = SND_JACK_HEADPHONE,
	},
};

static const struct snd_soc_dapm_widget mt8173_mt6391_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_HP("Headphone", NULL),
};

static const struct snd_soc_dapm_route mt8173_mt6391_routes[] = {
	{ "Speaker", NULL, "SPKL" },
	{ "Speaker", NULL, "SPKR" },
	{ "Headphone", NULL, "HPL" },
	{ "Headphone", NULL, "HPR" },
};

static const struct snd_kcontrol_new mt8173_mt6391_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Headphone"),
};

static int mt8173_mt6391_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *codec = snd_soc_rtd_to_codec(rtd, 0)->component;
	struct snd_soc_jack *jack = &mt8173_mt6391_jack;
	int ret;

	ret = snd_soc_card_jack_new_pins(rtd->card, "Headset Jack",
					 SND_JACK_HEADSET | SND_JACK_BTN_0,
					 jack, mt8173_mt6391_jack_pins,
					 ARRAY_SIZE(mt8173_mt6391_jack_pins));
	if (ret)
		return ret;

	snd_jack_set_key(jack->jack, SND_JACK_BTN_0, KEY_PLAYPAUSE);

	return snd_soc_component_set_jack(codec, jack, NULL);
}

/* The RT5514 runs from the I2S MCLK, which the AFE drives at 256fs */
static int mt8173_rt5514_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai;
	unsigned int rate = params_rate(params);
	int i, ret;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		ret = snd_soc_dai_set_sysclk(codec_dai, RT5514_SCLK_S_MCLK,
					     rate * 256, SND_SOC_CLOCK_IN);
		if (ret)
			return ret;
	}
	return 0;
}

static const struct snd_soc_ops mt8173_rt5514_ops = {
	.hw_params = mt8173_rt5514_hw_params,
};

/*
 * The microphone is on the RT5514's AMICL input. Added from the link's init,
 * so that a board without an RT5514 has no route to a missing widget.
 */
static const struct snd_soc_dapm_widget mt8173_rt5514_widgets[] = {
	SND_SOC_DAPM_MIC("Int Mic", NULL),
};

static const struct snd_soc_dapm_route mt8173_rt5514_routes[] = {
	{ "AMICL", NULL, "Int Mic" },
};

static int mt8173_rt5514_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	int ret;

	ret = snd_soc_dapm_new_controls(card->dapm, mt8173_rt5514_widgets,
					ARRAY_SIZE(mt8173_rt5514_widgets));
	if (ret)
		return ret;

	return snd_soc_dapm_add_routes(card->dapm, mt8173_rt5514_routes,
				       ARRAY_SIZE(mt8173_rt5514_routes));
}

static struct snd_soc_dai_link mt8173_mt6391_dais[] = {
	/* Front End DAI links */
	[DAI_LINK_PLAYBACK] = {
		.name = "MT6391 Playback",
		.stream_name = "MT6391 Playback",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dynamic = 1,
		.playback_only = 1,
		SND_SOC_DAILINK_REG(playback),
	},
	[DAI_LINK_CAPTURE] = {
		.name = "MT6391 Capture",
		.stream_name = "MT6391 Capture",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dynamic = 1,
		.capture_only = 1,
		SND_SOC_DAILINK_REG(capture),
	},
	/* Back End DAI links */
	[DAI_LINK_CODEC] = {
		.name = "Primary Codec",
		.no_pcm = 1,
		.init = mt8173_mt6391_init,
		.playback_only = 1,
		SND_SOC_DAILINK_REG(codec),
	},
	/* Last, so that a board without an RT5514 can drop it */
	[DAI_LINK_RT5514] = {
		.name = "RT5514 Codec",
		.no_pcm = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBC_CFC,
		.ops = &mt8173_rt5514_ops,
		.init = mt8173_rt5514_init,
		.capture_only = 1,
		.ignore_pmdown_time = 1,
		SND_SOC_DAILINK_REG(rt5514_codec),
	},
};

static struct snd_soc_card mt8173_mt6391_card = {
	.name = "mt8173-mt6391",
	.owner = THIS_MODULE,
	.dai_link = mt8173_mt6391_dais,
	.num_links = ARRAY_SIZE(mt8173_mt6391_dais),
	.controls = mt8173_mt6391_controls,
	.num_controls = ARRAY_SIZE(mt8173_mt6391_controls),
	.dapm_widgets = mt8173_mt6391_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt8173_mt6391_widgets),
	.dapm_routes = mt8173_mt6391_routes,
	.num_dapm_routes = ARRAY_SIZE(mt8173_mt6391_routes),
};

static int mt8173_mt6391_dev_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &mt8173_mt6391_card;
	struct device_node *codec_node, *platform_node, *capture_node;
	struct snd_soc_dai_link *dai_link;
	int ret, i;

	platform_node = of_parse_phandle(pdev->dev.of_node,
					 "mediatek,platform", 0);
	if (!platform_node) {
		dev_err(&pdev->dev, "Property 'platform' missing or invalid\n");
		return -EINVAL;
	}
	for_each_card_prelinks(card, i, dai_link) {
		if (dai_link->platforms->name)
			continue;
		dai_link->platforms->of_node = platform_node;
	}

	codec_node = of_parse_phandle(pdev->dev.of_node,
				      "mediatek,audio-codec", 0);
	if (!codec_node) {
		dev_err(&pdev->dev,
			"Property 'audio-codec' missing or invalid\n");
		ret = -EINVAL;
		goto put_platform_node;
	}
	/* The front ends' dummy codec is filled in by the core */
	mt8173_mt6391_dais[DAI_LINK_CODEC].codecs[0].of_node = codec_node;

	capture_node = of_parse_phandle(pdev->dev.of_node,
					"mediatek,audio-codec", 1);
	if (capture_node) {
		mt8173_mt6391_dais[DAI_LINK_RT5514].codecs[0].of_node =
			capture_node;
	} else {
		dev_dbg(&pdev->dev, "no capture codec, playback only\n");
		card->num_links = DAI_LINK_RT5514;
	}

	card->dev = &pdev->dev;

	ret = devm_snd_soc_register_card(&pdev->dev, card);

	of_node_put(capture_node);
	of_node_put(codec_node);

put_platform_node:
	of_node_put(platform_node);
	return ret;
}

static const struct of_device_id mt8173_mt6391_dt_match[] = {
	{ .compatible = "mediatek,mt8173-mt6391", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt8173_mt6391_dt_match);

static struct platform_driver mt8173_mt6391_driver = {
	.driver = {
		   .name = "mt8173-mt6391",
		   .of_match_table = mt8173_mt6391_dt_match,
		   .pm = &snd_soc_pm_ops,
	},
	.probe = mt8173_mt6391_dev_probe,
};

module_platform_driver(mt8173_mt6391_driver);

MODULE_DESCRIPTION("MT8173 MT6391 ALSA SoC machine driver");
MODULE_LICENSE("GPL");
