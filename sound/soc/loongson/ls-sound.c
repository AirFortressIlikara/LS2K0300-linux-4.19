#include <linux/module.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <linux/pci_ids.h>
#include <linux/pci.h>
#include <linux/acpi.h>


static int loongson_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
#ifdef SND_SOC_ES8336
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int err;
	unsigned int mclk_rate;
	unsigned int rate = params_rate(params);


	/*
	 * According to CS4271 datasheet we use MCLK/LRCK=256 for
	 * rates below 50kHz and 128 for higher sample rates
	 */
	if (rate < 50000)
		mclk_rate = rate * 64 * 4;
	else
		mclk_rate = rate * 64 * 2;

	err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk_rate,
				     SND_SOC_CLOCK_IN);
	if (err)
		return err;

	return snd_soc_dai_set_sysclk(cpu_dai, 0, mclk_rate,
					SND_SOC_CLOCK_OUT);
#else/*original*/
	return 0;
#endif

	return 0;

}

static struct snd_soc_ops loongson_ops = {
	.hw_params = loongson_hw_params,
};

static int loongson_es8388_init_paiftx(struct snd_soc_pcm_runtime *rtd)
{
	/* Enabling the microphone requires the fitting of a 0R
	 * resistor to connect the line from the microphone jack.
	 */
	snd_soc_dapm_disable_pin(&rtd->card->dapm, "MicIn");
	return 0;
}

enum {
	PRI_PLAYBACK = 0,
	PRI_CAPTURE,
};
/*
 *SND_SOC_DAIFMT_CBS_CFS  play ok, volume ok
 *SND_SOC_DAIFMT_CBM_CFM  Codec's BCLK is Master，LRCLK is Master
 */
#define LOONGSON_DAI_FMT (SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_IB_NF | \
	SND_SOC_DAIFMT_CBS_CFS)

static struct snd_soc_dai_link loongson_dai[] = {
	[PRI_PLAYBACK] = {
		/* Primary Playback i/f */
		.name = "ES8388 PAIF RX",
		.stream_name = "Playback",
		.cpu_dai_name = "loongson-i2s-dai",
		.codec_dai_name = "ES8323 HiFi",
		.platform_name = "loongson-i2s",
		.dai_fmt = LOONGSON_DAI_FMT,
		.ops = &loongson_ops,
	},
	[PRI_CAPTURE] = {
		/* Primary Capture i/f */
		.name = "ES8388 PAIF TX",
		.stream_name = "Capture",
		.cpu_dai_name = "loongson-i2s-dai",
		.codec_dai_name = "ES8323 HiFi",
		.platform_name = "loongson-i2s",
		.dai_fmt = LOONGSON_DAI_FMT,
		.init = loongson_es8388_init_paiftx,
		.ops = &loongson_ops,
	},
};

static struct snd_soc_card loongson = {
	.name = "LOONGSON-I2S",
	.owner = THIS_MODULE,
	.dai_link = loongson_dai,
	.num_links = ARRAY_SIZE(loongson_dai),

};

static struct platform_device *loongson_snd_device;

static int ls_sound_drv_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *np;

	loongson_snd_device = platform_device_alloc("soc-audio", -1);
	if (!loongson_snd_device)
		return -ENOMEM;

	platform_set_drvdata(loongson_snd_device, &loongson);
	if(ACPI_COMPANION(&pdev->dev)){
	    loongson_dai[0].name = "ES8323 PAIF RX";
	    loongson_dai[0].stream_name = "Playback";
	    loongson_dai[0].codec_dai_name = "ES8323 HiFi";
	    loongson_dai[0].codec_name = "i2c-ESSX8323:00";
	    loongson_dai[1].name = "ES8323 PAIF TX";
	    loongson_dai[1].stream_name = "Capture";
	    loongson_dai[1].codec_dai_name = "ES8323 HiFi";
	    loongson_dai[1].codec_name = "i2c-ESSX8323:00";
	}
	else if((np = pdev->dev.of_node))
	{
	    of_property_read_string_index(np, "codec-names", 0 , &loongson_dai[0].name);
	    of_property_read_string_index(np, "codec-names", 1 , &loongson_dai[0].stream_name);
	    of_property_read_string_index(np, "codec-names", 2, &loongson_dai[0].codec_dai_name);
	    of_property_read_string_index(np, "codec-names", 3, &loongson_dai[0].codec_name);
	    of_property_read_string_index(np, "codec-names", 4 , &loongson_dai[1].name);
	    of_property_read_string_index(np, "codec-names", 5 , &loongson_dai[1].stream_name);
	    of_property_read_string_index(np, "codec-names", 6, &loongson_dai[1].codec_dai_name);
	    of_property_read_string_index(np, "codec-names", 7, &loongson_dai[1].codec_name);
	}

	ret = platform_device_add(loongson_snd_device);

	if (ret)
		platform_device_put(loongson_snd_device);

	return ret;
}

static int ls_sound_drv_remove(struct platform_device *pdev)
{
	platform_device_unregister(loongson_snd_device);
	return 0;
}
static const struct of_device_id snd_ls_sound_dt_match[] = {
	{ .compatible = "loongson,ls-sound", },
	{ .compatible = "loongson,loongson2-sound", },
	{ .compatible = "loongson,ls7a-sound", },
	{},
};
MODULE_DEVICE_TABLE(of, snd_ls_sound_dt_match);

static struct platform_driver ls_sound_driver = {
	.probe = ls_sound_drv_probe,
	.remove = ls_sound_drv_remove,
	.driver = {
		.name = "ls-sound",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(snd_ls_sound_dt_match),
	},
};

static int __init loongson_audio_init(void)
{

	return platform_driver_register(&ls_sound_driver);
}
module_init(loongson_audio_init);

static void __exit loongson_audio_exit(void)
{
	platform_driver_unregister(&ls_sound_driver);
}
module_exit(loongson_audio_exit);

MODULE_SOFTDEP("pre: ls_pcm ls_i2s snd_soc_es8323_i2c");
MODULE_AUTHOR("loongson");
MODULE_DESCRIPTION("ALSA SoC loongson");
MODULE_LICENSE("GPL");
