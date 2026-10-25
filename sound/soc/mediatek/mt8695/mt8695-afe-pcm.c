// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek ALSA SoC AFE platform driver for MT8695: the HDMI output.
 *
 * The AFE has MT2701's register layout. Its HDMI path is a memory
 * interface (memif) with its own interrupt (IRQ5), an interconnect and
 * an 8-channel I2S engine wired to the HDMI transmitter's audio input.
 * Unlike MT2701, the I2S engine takes its clock straight from APLL1 (the
 * 48 kHz family) or APLL2 (44.1 kHz), through a divider in AUDIO_TOP_CON3.
 *
 * Based on mt2701-afe-pcm.c and the MT8695 vendor kernel.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "mt8695-reg.h"
#include "../common/mtk-afe-fe-dai.h"
#include "../common/mtk-afe-platform-driver.h"
#include "../common/mtk-base-afe.h"

enum {
	MT8695_MEMIF_HDMI,
	MT8695_MEMIF_NUM,
	MT8695_IO_HDMI = MT8695_MEMIF_NUM,
};

enum {
	MT8695_IRQ_5,
	MT8695_IRQ_NUM,
};

/*
 * Clocks kept running while the AFE is resumed. The "afe" gate stays on
 * from probe to removal, as on MT2701 and in the vendor driver.
 */
static const char * const mt8695_afe_clk_names[] = {
	"afe_conn", "a1sys", "apll",
};

struct mt8695_afe_private {
	struct clk_bulk_data clks[ARRAY_SIZE(mt8695_afe_clk_names)];
	struct clk *hdmi_clk;
	struct clk *apll1_clk;
	struct clk *apll2_clk;
	struct clk *a2sys_clk;
	bool apll2_on;
};

static const struct snd_pcm_hardware mt8695_afe_hardware = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_MMAP_VALID,
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min = 256,
	.period_bytes_max = 512 * 1024,
	.periods_min = 2,
	.periods_max = 256,
	.buffer_bytes_max = 1024 * 1024,
	.fifo_size = 0,
};

/* The HDMI memif has no sample rate field and no own interrupt rate */
static int mt8695_memif_fs(struct snd_pcm_substream *substream,
			   unsigned int rate)
{
	return 0;
}

static int mt8695_irq_fs(struct snd_pcm_substream *substream,
			 unsigned int rate)
{
	return 0;
}

static int mt8695_afe_hdmi_startup(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	struct mt8695_afe_private *afe_priv = afe->platform_priv;

	return clk_prepare_enable(afe_priv->hdmi_clk);
}

static void mt8695_afe_hdmi_shutdown(struct snd_pcm_substream *substream,
				     struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	struct mt8695_afe_private *afe_priv = afe->platform_priv;

	clk_disable_unprepare(afe_priv->hdmi_clk);
}

static void mt8695_afe_hdmi_apll2_off(struct mt8695_afe_private *afe_priv)
{
	if (!afe_priv->apll2_on)
		return;

	clk_disable_unprepare(afe_priv->a2sys_clk);
	clk_disable_unprepare(afe_priv->apll2_clk);
	afe_priv->apll2_on = false;
}

static int mt8695_afe_hdmi_apll2_on(struct mt8695_afe_private *afe_priv)
{
	int ret;

	if (afe_priv->apll2_on)
		return 0;

	ret = clk_prepare_enable(afe_priv->apll2_clk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(afe_priv->a2sys_clk);
	if (ret) {
		clk_disable_unprepare(afe_priv->apll2_clk);
		return ret;
	}

	afe_priv->apll2_on = true;
	return 0;
}

static int mt8695_afe_hdmi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params,
				     struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);
	struct mt8695_afe_private *afe_priv = afe->platform_priv;
	unsigned int channels = params_channels(params);
	unsigned int rate = params_rate(params);
	unsigned long apll_rate;
	unsigned int div;
	bool apll2;
	int ret;

	if (rate % 8000 == 0)
		apll2 = false;
	else if (rate % 11025 == 0)
		apll2 = true;
	else
		return -EINVAL;

	if (apll2) {
		ret = mt8695_afe_hdmi_apll2_on(afe_priv);
		if (ret)
			return ret;
		apll_rate = clk_get_rate(afe_priv->apll2_clk);
	} else {
		mt8695_afe_hdmi_apll2_off(afe_priv);
		apll_rate = clk_get_rate(afe_priv->apll1_clk);
	}

	/* The I2S engine runs at 128 fs */
	div = apll_rate / (128 * rate);
	if (!div || div > 64 || apll_rate % (128 * rate)) {
		dev_err(afe->dev, "%u Hz can't be made from %lu Hz\n",
			rate, apll_rate);
		return -EINVAL;
	}

	regmap_update_bits(afe->regmap, AUDIO_TOP_CON0,
			   AUDIO_TOP_CON0_8CH_I2S_APLL2,
			   apll2 ? AUDIO_TOP_CON0_8CH_I2S_APLL2 : 0);
	regmap_update_bits(afe->regmap, AUDIO_TOP_CON3,
			   AUDIO_TOP_CON3_HDMI_BCK_DIV,
			   FIELD_PREP(AUDIO_TOP_CON3_HDMI_BCK_DIV, div - 1));

	regmap_update_bits(afe->regmap, AFE_HDMI_OUT_CON0,
			   AFE_HDMI_OUT_CON0_CH_NUM | AFE_HDMI_OUT_CON0_32BIT,
			   FIELD_PREP(AFE_HDMI_OUT_CON0_CH_NUM, channels) |
			   (params_physical_width(params) == 32 ?
			    AFE_HDMI_OUT_CON0_32BIT : 0));

	regmap_write(afe->regmap, AFE_HDMI_CONN0, AFE_HDMI_CONN0_IDENTITY);

	/* I2S framing with 32-bit slots, whatever the sample width */
	regmap_write(afe->regmap, AFE_8CH_I2S_OUT_CON,
		     FIELD_PREP(AFE_8CH_I2S_OUT_CON_WLEN,
				AFE_8CH_I2S_OUT_CON_WLEN_32BIT) |
		     AFE_8CH_I2S_OUT_CON_I2S_DELAY |
		     AFE_8CH_I2S_OUT_CON_BCK_INV);

	return 0;
}

static int mt8695_afe_hdmi_hw_free(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);

	mt8695_afe_hdmi_apll2_off(afe->platform_priv);

	return 0;
}

static int mt8695_afe_hdmi_trigger(struct snd_pcm_substream *substream,
				   int cmd, struct snd_soc_dai *dai)
{
	struct mtk_base_afe *afe = snd_soc_dai_get_drvdata(dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		regmap_set_bits(afe->regmap, AFE_8CH_I2S_OUT_CON,
				AFE_8CH_I2S_OUT_CON_EN);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		regmap_clear_bits(afe->regmap, AFE_8CH_I2S_OUT_CON,
				  AFE_8CH_I2S_OUT_CON_EN);
		return 0;
	}

	return -EINVAL;
}

static const struct snd_soc_dai_ops mt8695_afe_hdmi_ops = {
	.startup	= mt8695_afe_hdmi_startup,
	.shutdown	= mt8695_afe_hdmi_shutdown,
	.hw_params	= mt8695_afe_hdmi_hw_params,
	.hw_free	= mt8695_afe_hdmi_hw_free,
	.trigger	= mt8695_afe_hdmi_trigger,
};

#define MT8695_HDMI_RATES (SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
			   SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
			   SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
			   SNDRV_PCM_RATE_192000)

static struct snd_soc_dai_driver mt8695_afe_pcm_dais[] = {
	/* FE DAI: the memory interface */
	{
		.name = "PCM_HDMI",
		.id = MT8695_MEMIF_HDMI,
		.playback = {
			.stream_name = "HDMI Multich",
			.channels_min = 2,
			.channels_max = 8,
			.rates = MT8695_HDMI_RATES,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &mtk_afe_fe_ops,
	},
	/* BE DAI: the 8-channel I2S engine feeding the HDMI transmitter */
	{
		.name = "HDMI I2S",
		.id = MT8695_IO_HDMI,
		.playback = {
			.stream_name = "HDMI 8CH I2S Playback",
			.channels_min = 2,
			.channels_max = 8,
			.rates = MT8695_HDMI_RATES,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &mt8695_afe_hdmi_ops,
	},
};

static const struct snd_soc_dapm_route mt8695_afe_pcm_routes[] = {
	{ "HDMI 8CH I2S Playback", NULL, "HDMI Multich" },
};

static const struct snd_soc_component_driver mt8695_afe_pcm_dai_component = {
	.name = "mt8695-afe-pcm-dai",
	.dapm_routes = mt8695_afe_pcm_routes,
	.num_dapm_routes = ARRAY_SIZE(mt8695_afe_pcm_routes),
};

static const struct mtk_base_memif_data memif_data[MT8695_MEMIF_NUM] = {
	{
		/*
		 * The channel count and sample width share AFE_HDMI_OUT_CON0
		 * with the enable bit; the back end sets them.
		 */
		.name = "HDMI",
		.id = MT8695_MEMIF_HDMI,
		.reg_ofs_base = AFE_HDMI_OUT_BASE,
		.reg_ofs_cur = AFE_HDMI_OUT_CUR,
		.reg_ofs_end = AFE_HDMI_OUT_END,
		.fs_reg = -1,
		.fs_shift = -1,
		.mono_reg = -1,
		.mono_shift = -1,
		.enable_reg = AFE_HDMI_OUT_CON0,
		.enable_shift = 0,
		.hd_reg = -1,
		.hd_shift = -1,
		.hd_align_reg = -1,
		.agent_disable_reg = -1,
		.msb_reg = -1,
	},
};

static const struct mtk_base_irq_data irq_data[MT8695_IRQ_NUM] = {
	{
		.id = MT8695_IRQ_5,
		.irq_cnt_reg = AFE_IRQ_CNT5,
		.irq_cnt_shift = 0,
		.irq_cnt_maskbit = 0x3ffff,
		.irq_fs_reg = -1,
		.irq_fs_shift = -1,
		.irq_en_reg = AFE_IRQ_CON,
		.irq_en_shift = AFE_IRQ_CON_IRQ5_ON_SHIFT,
		.irq_clr_reg = AFE_IRQ_CLR,
		.irq_clr_shift = AFE_IRQ_HDMI_SHIFT,
	},
};

static irqreturn_t mt8695_afe_irq_handler(int irq, void *dev_id)
{
	struct mtk_base_afe *afe = dev_id;
	struct mtk_base_afe_memif *memif = &afe->memif[MT8695_MEMIF_HDMI];
	unsigned int status;

	regmap_read(afe->regmap, AFE_IRQ_STATUS, &status);
	status &= AFE_IRQ_CLR_ALL;
	if (!status)
		return IRQ_NONE;

	regmap_write(afe->regmap, AFE_IRQ_CLR, status);

	if (status & BIT(AFE_IRQ_HDMI_SHIFT) && memif->substream)
		snd_pcm_period_elapsed(memif->substream);

	return IRQ_HANDLED;
}

static int mt8695_afe_runtime_suspend(struct device *dev)
{
	struct mtk_base_afe *afe = dev_get_drvdata(dev);
	struct mt8695_afe_private *afe_priv = afe->platform_priv;

	regmap_clear_bits(afe->regmap, AFE_DAC_CON0, AFE_DAC_CON0_AFE_ON);
	regmap_clear_bits(afe->regmap, ASYS_TOP_CON, ASYS_TOP_CON_TIMING_ON);
	clk_bulk_disable_unprepare(ARRAY_SIZE(afe_priv->clks), afe_priv->clks);

	return 0;
}

static int mt8695_afe_runtime_resume(struct device *dev)
{
	struct mtk_base_afe *afe = dev_get_drvdata(dev);
	struct mt8695_afe_private *afe_priv = afe->platform_priv;
	int ret;

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(afe_priv->clks), afe_priv->clks);
	if (ret)
		return ret;

	regmap_set_bits(afe->regmap, ASYS_TOP_CON, ASYS_TOP_CON_TIMING_ON);
	regmap_set_bits(afe->regmap, AFE_DAC_CON0, AFE_DAC_CON0_AFE_ON);
	regmap_set_bits(afe->regmap, AFE_MEMIF_PBUF_SIZE,
			AFE_MEMIF_PBUF_SIZE_HDMI);

	return 0;
}

static int mt8695_afe_init_clocks(struct mtk_base_afe *afe)
{
	struct mt8695_afe_private *afe_priv = afe->platform_priv;
	struct device *dev = afe->dev;
	struct clk *clk;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(afe_priv->clks); i++)
		afe_priv->clks[i].id = mt8695_afe_clk_names[i];

	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(afe_priv->clks), afe_priv->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clocks\n");

	clk = devm_clk_get_enabled(dev, "afe");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to enable the afe clock\n");

	afe_priv->hdmi_clk = devm_clk_get(dev, "hdmi");
	if (IS_ERR(afe_priv->hdmi_clk))
		return dev_err_probe(dev, PTR_ERR(afe_priv->hdmi_clk),
				     "failed to get the hdmi clock\n");

	afe_priv->a2sys_clk = devm_clk_get(dev, "a2sys");
	if (IS_ERR(afe_priv->a2sys_clk))
		return dev_err_probe(dev, PTR_ERR(afe_priv->a2sys_clk),
				     "failed to get the a2sys clock\n");

	afe_priv->apll1_clk = devm_clk_get(dev, "apll1");
	if (IS_ERR(afe_priv->apll1_clk))
		return dev_err_probe(dev, PTR_ERR(afe_priv->apll1_clk),
				     "failed to get the apll1 clock\n");

	afe_priv->apll2_clk = devm_clk_get(dev, "apll2");
	if (IS_ERR(afe_priv->apll2_clk))
		return dev_err_probe(dev, PTR_ERR(afe_priv->apll2_clk),
				     "failed to get the apll2 clock\n");

	return 0;
}

static int mt8695_afe_pcm_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt8695_afe_private *afe_priv;
	struct mtk_base_afe *afe;
	int i, irq, ret;

	afe = devm_kzalloc(dev, sizeof(*afe), GFP_KERNEL);
	if (!afe)
		return -ENOMEM;

	afe_priv = devm_kzalloc(dev, sizeof(*afe_priv), GFP_KERNEL);
	if (!afe_priv)
		return -ENOMEM;

	afe->platform_priv = afe_priv;
	afe->dev = dev;

	/* The memif takes 32-bit physical addresses */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	afe->regmap = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(afe->regmap))
		return dev_err_probe(dev, PTR_ERR(afe->regmap),
				     "could not get regmap from parent\n");

	mutex_init(&afe->irq_alloc_lock);

	afe->memif_size = MT8695_MEMIF_NUM;
	afe->memif = devm_kcalloc(dev, afe->memif_size, sizeof(*afe->memif),
				  GFP_KERNEL);
	if (!afe->memif)
		return -ENOMEM;

	for (i = 0; i < afe->memif_size; i++) {
		afe->memif[i].data = &memif_data[i];
		afe->memif[i].irq_usage = -1;
	}

	afe->irqs_size = MT8695_IRQ_NUM;
	afe->irqs = devm_kcalloc(dev, afe->irqs_size, sizeof(*afe->irqs),
				 GFP_KERNEL);
	if (!afe->irqs)
		return -ENOMEM;

	for (i = 0; i < afe->irqs_size; i++)
		afe->irqs[i].irq_data = &irq_data[i];

	afe->mtk_afe_hardware = &mt8695_afe_hardware;
	afe->memif_fs = mt8695_memif_fs;
	afe->irq_fs = mt8695_irq_fs;
	afe->runtime_resume = mt8695_afe_runtime_resume;
	afe->runtime_suspend = mt8695_afe_runtime_suspend;

	ret = mt8695_afe_init_clocks(afe);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, afe);

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;

	/* Nothing may be left pending from the boot loader */
	regmap_clear_bits(afe->regmap, AFE_IRQ_CON,
			  BIT(AFE_IRQ_CON_IRQ5_ON_SHIFT));
	regmap_write(afe->regmap, AFE_IRQ_CLR, AFE_IRQ_CLR_ALL);

	irq = platform_get_irq_byname(pdev, "afe");
	if (irq < 0) {
		ret = irq;
		goto err_pm_put;
	}

	ret = devm_request_irq(dev, irq, mt8695_afe_irq_handler, 0,
			       "mt8695-afe", afe);
	if (ret) {
		dev_err_probe(dev, ret, "could not request the afe irq\n");
		goto err_pm_put;
	}

	ret = devm_snd_soc_register_component(dev, &mtk_afe_pcm_platform,
					      NULL, 0);
	if (ret)
		goto err_pm_put;

	ret = devm_snd_soc_register_component(dev,
					      &mt8695_afe_pcm_dai_component,
					      mt8695_afe_pcm_dais,
					      ARRAY_SIZE(mt8695_afe_pcm_dais));

err_pm_put:
	pm_runtime_put_sync(dev);
	return ret;
}

static const struct of_device_id mt8695_afe_pcm_dt_match[] = {
	{ .compatible = "mediatek,mt8695-audio" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt8695_afe_pcm_dt_match);

static const struct dev_pm_ops mt8695_afe_pm_ops = {
	RUNTIME_PM_OPS(mt8695_afe_runtime_suspend,
		       mt8695_afe_runtime_resume, NULL)
};

static struct platform_driver mt8695_afe_pcm_driver = {
	.driver = {
		.name = "mt8695-audio",
		.of_match_table = mt8695_afe_pcm_dt_match,
		.pm = pm_ptr(&mt8695_afe_pm_ops),
	},
	.probe = mt8695_afe_pcm_dev_probe,
};
module_platform_driver(mt8695_afe_pcm_driver);

MODULE_DESCRIPTION("MediaTek ALSA SoC AFE platform driver for MT8695");
MODULE_LICENSE("GPL");
