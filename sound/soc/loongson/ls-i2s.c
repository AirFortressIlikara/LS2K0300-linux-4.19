/*
 * Freescale MPC5200 PSC in I2S mode
 * ALSA SoC Digital Audio Interface (DAI) driver
 *
 * Copyright (C) 2008 Secret Lab Technologies Ltd.
 * Copyright (C) 2009 Jon Smirl, Digispeaker
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include "ls-lib.h"
#include <uapi/linux/pci.h>

#include <linux/pci_ids.h>
#include <linux/pci.h>
#include <linux/acpi.h>

#define PCM_STEREO_OUT_NAME "I2S PCM Stereo out"
#define PCM_STEREO_IN_NAME "I2S PCM Stereo in"

#define IISRXADDR       0x0c
#define IISTXADDR       0x10


uint32_t clk_rate;

static int clk_ena;
#define PSC_I2S_RATES SNDRV_PCM_RATE_8000_96000

#define PSC_I2S_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE)

static struct ls_pcm_dma_params ls_i2s_pcm_stereo_out;
static struct ls_pcm_dma_params ls_i2s_pcm_stereo_in;
static unsigned int revision_id;



static int ls_i2s_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	return 0;
}

static int ls_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct ls_pcm_dma_params *dma_data;
	void *i2sbase;
	unsigned int rat_cddiv;
	unsigned int rat_bitdiv;
	unsigned int bclk_ratio;
	unsigned int wlen = 16;
	unsigned int depth = 16;
	unsigned int fs = 44100;
	unsigned int xfs = 256;
	unsigned long long int mclk_ratio;
	unsigned long long int mclk_ratio_frac;

	i2sbase = dai->dev->platform_data;
	rat_bitdiv = DIV_ROUND_CLOSEST(clk_rate, (depth * 2 * fs * 2)) - 1;
	rat_cddiv = DIV_ROUND_CLOSEST(clk_rate, (xfs * fs * 2)) - 1;
	bclk_ratio = DIV_ROUND_CLOSEST(fs * xfs, (depth * 2 * fs * 2)) - 1;
	mclk_ratio = clk_rate / (xfs * fs);
	mclk_ratio_frac = DIV_ROUND_CLOSEST((u64)clk_rate * 65536, (u64)fs * (u64)xfs) - mclk_ratio * 65536;

	clk_ena = 1;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_data = &ls_i2s_pcm_stereo_out;
	else
		dma_data = &ls_i2s_pcm_stereo_in;

	snd_soc_dai_set_dma_data(dai, substream, dma_data);

	if (revision_id == 0) {
		writel((wlen<<24) | (depth<<16) | (rat_bitdiv<<8) | (rat_cddiv<<0), i2sbase + 0x4);
	} else if (revision_id == 1) {
		writel((wlen<<24) | (depth<<16) | (bclk_ratio<<8) | depth, i2sbase + 0x4);
		writel((mclk_ratio_frac << 16) | mclk_ratio, i2sbase + 0x14);
	} else
		printk("ERROR: IIS revision id is illegal.");

	return 0;
}

static int ls_i2s_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
			      int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static int ls_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai, unsigned int format)
{
	return 0;
}

static int ls_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			      struct snd_soc_dai *dai)
{
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void ls_i2s_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{

}


#ifdef CONFIG_PM
static int ls_i2s_suspend(struct snd_soc_dai *dai)
{
	return 0;
}

static int ls_i2s_resume(struct snd_soc_dai *dai)
{
	return 0;
}

#else
#define ls_i2s_suspend	NULL
#define ls_i2s_resume	NULL
#endif

static int ls_i2s_probe(struct snd_soc_dai *dai)
{
	return 0;
}

static int  ls_i2s_remove(struct snd_soc_dai *dai)
{
	return 0;
}

static const struct snd_soc_dai_ops psc_i2s_dai_ops = {
	.startup	= ls_i2s_startup,
	.shutdown	= ls_i2s_shutdown,
	.trigger	= ls_i2s_trigger,
	.hw_params	= ls_i2s_hw_params,
	.set_fmt	= ls_i2s_set_dai_fmt,
	.set_sysclk	= ls_i2s_set_dai_sysclk,
};

static struct snd_soc_dai_driver psc_i2s_dai[] = {{
	.name = "loongson-i2s-dai",
	.probe = ls_i2s_probe,
	.remove = ls_i2s_remove,
	.suspend = ls_i2s_suspend,
	.resume = ls_i2s_resume,
	.playback = {
		.stream_name = "I2S Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = PSC_I2S_RATES,
		.formats = PSC_I2S_FORMATS,
	}, .capture = {
		.stream_name = "I2S Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = PSC_I2S_RATES,
		.formats = PSC_I2S_FORMATS,
	},
	.ops = &psc_i2s_dai_ops,
	.symmetric_rates = 1,
} };

static const struct snd_soc_component_driver psc_i2s_component = {
	.name		= "loongson-i2s-dai",
};

static int ls_i2s_drv_probe(struct platform_device *pdev)
{
	uint64_t IISRxData, IISTxData;
	struct pci_dev *pci_dev;

	if (ACPI_COMPANION(&pdev->dev) || (pci_dev = pci_get_device(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_LOONGSON_I2S, NULL))) {
		resource_size_t mmio_base, mmio_size;
		struct pci_dev *ppdev;
		static void __iomem *pci_i2s_reg;
		int ret;

		ppdev = pci_get_device(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_LOONGSON_I2S, NULL);

		if (ppdev) {
			ret = pci_enable_device(ppdev);

			mmio_base = pci_resource_start(ppdev, 0);
			mmio_size = pci_resource_len(ppdev, 0);
			pci_i2s_reg = ioremap(mmio_base, mmio_size);
			revision_id = ppdev->revision;
		}

		pdev->dev.platform_data = pci_i2s_reg;
		IISRxData = (uint64_t)pci_i2s_reg | IISRXADDR;
		IISTxData = (uint64_t)pci_i2s_reg | IISTXADDR;
	} else {
		struct device_node *np = pdev->dev.of_node;
		struct resource *r;
		r = platform_get_resource(pdev, IORESOURCE_MEM, 0);

		if (r == NULL) {
			dev_err(&pdev->dev, "no IO memory resource defined\n");
			return -ENODEV;
		}

		pdev->dev.platform_data = ioremap_nocache(r->start, r->end - r->start + 1);;
		IISRxData = r->start | IISRXADDR;
		IISTxData = r->start | IISTXADDR;

		if (of_device_is_compatible(np, "loongson,ls2k0300-i2s")){
			*(volatile uint32_t *)0x8000000016000138 |= (0x2<<16);
			revision_id = 1;
		}
	}

	device_property_read_u32(&pdev->dev, "clock-frequency", &clk_rate);
	pdev->dev.kobj.name = "loongson-i2s-dai";

	ls_i2s_pcm_stereo_out.name = PCM_STEREO_OUT_NAME;
	ls_i2s_pcm_stereo_out.dev_addr = IISTxData;

	ls_i2s_pcm_stereo_in.name = PCM_STEREO_IN_NAME;
	ls_i2s_pcm_stereo_in.dev_addr = IISRxData;

	return snd_soc_register_component(&pdev->dev, &psc_i2s_component,
	psc_i2s_dai, 1);
}

static int ls_i2s_drv_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

static const struct of_device_id snd_ls_i2s_dt_match[] = {
    { .compatible = "loongson,ls-i2s", },
    { .compatible = "loongson,ls2k0300-i2s", },
    { .compatible = "loongson,loongson2-i2s", },
    { .compatible = "loongson,ls7a-i2s", },
    {},
};
MODULE_DEVICE_TABLE(of, snd_ls_i2s_dt_match);

static struct platform_driver ls_i2s_driver = {
	.probe = ls_i2s_drv_probe,
	.remove = ls_i2s_drv_remove,
	.driver = {
		.name = "loongson-i2sp",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(snd_ls_i2s_dt_match),
	},
};

static int __init loongson_i2s_init(void)
{
	return platform_driver_register(&ls_i2s_driver);
}

static void __exit loongson_i2s_exit(void)
{
	platform_driver_unregister(&ls_i2s_driver);
}

module_init(loongson_i2s_init)
module_exit(loongson_i2s_exit)

MODULE_AUTHOR("loongson");
MODULE_DESCRIPTION("Loongson I2S mode ASoC Driver");
MODULE_LICENSE("GPL");

