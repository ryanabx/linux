// SPDX-License-Identifier: GPL-2.0
/*
 * mt8173-mt6391.c  --  MT8173 machine driver for boards using the
 * MT6391 PMIC codec (the audio block of the MT6397 PMIC), such as
 * the Amazon Fire HD 10 2017 (suez).
 *
 * Modeled on mt8173-max98090.c. The codec is reached over the SoC's
 * internal ADDA/PMIC interface, so there is no I2S format or sysclk
 * to configure here.
 */

#include <linux/module.h>
#include <linux/of.h>
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

/*
 * The internal microphone is an analog part on the RT5514's AMICL input; the
 * RT5514 digitises it and hands it to the SoC over I2S, where the AFE carries
 * it on I17/I18 (I03/I04, the other source offered for "I2S Capture", is the
 * SoC's internal ADC, to which nothing is connected on these boards).
 *
 * The RT5514 takes its system clock straight from the I2S MCLK, which the AFE
 * drives at 256fs for every rate, so no PLL is involved.
 */
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
 * "Int Mic" gives the microphone a name to route from, following the idiom in
 * mt8173-rt5650-rt5514.c. Only the left input is populated, so capture is
 * mono.
 *
 * These live in the link's .init rather than on the card so they are only
 * registered when the RT5514 link is, since a board without one has no AMICL
 * widget to route to and card registration would fail.
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
		SND_SOC_DAILINK_REG(codec),
	},
	/*
	 * Kept LAST so probe can simply shorten num_links when the board has
	 * no RT5514 -- see mt8173_mt6391_dev_probe().
	 */
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
	/*
	 * Only the back end names the codec. The front ends use
	 * COMP_DUMMY(), which is empty at this point and filled in by the
	 * core during registration -- assigning an of_node to them too
	 * fails the card sanity check with "Both Component name/of_node
	 * are set".
	 */
	mt8173_mt6391_dais[DAI_LINK_CODEC].codecs[0].of_node = codec_node;

	/*
	 * The RT5514 capture back end is optional. A board without one drops
	 * the last link instead of failing to register, so playback is
	 * unaffected either way -- which is why DAI_LINK_RT5514 is last.
	 */
	capture_node = of_parse_phandle(pdev->dev.of_node,
					"mediatek,capture-codec", 0);
	if (capture_node) {
		mt8173_mt6391_dais[DAI_LINK_RT5514].codecs[0].of_node =
			capture_node;
	} else {
		dev_info(&pdev->dev,
			 "no 'mediatek,capture-codec'; capture back end disabled\n");
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
	{ }
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

/* Module information */
MODULE_DESCRIPTION("MT8173 MT6391 ALSA SoC machine driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mt8173-mt6391");
