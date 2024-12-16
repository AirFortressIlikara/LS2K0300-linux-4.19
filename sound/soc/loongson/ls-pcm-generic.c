/*
 * sound/soc/loongson/ls-pcm-generic.c -- ALSA PCM interface for the Loongson chip with generic dma.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/of_dma.h>

#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/soc.h>
#include "ls-lib.h"

#include <linux/interrupt.h>
#include "ls-pcm.h"

#include <linux/module.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include <linux/interrupt.h>
#include <uapi/linux/pci.h>

#include <linux/pci_ids.h>
#include <linux/pci.h>
#include <linux/acpi.h>

static struct ls_runtime_data generic_prtd;
static const struct snd_pcm_hardware ls_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_RESUME |
				  SNDRV_PCM_INFO_PAUSE,
	.formats		= (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE),
	.rates			= SNDRV_PCM_RATE_8000_96000,
	.channels_min		= 2,
	.channels_max		= 2,
	.period_bytes_min	= 128,
	.period_bytes_max	= 128*1024,
	.periods_min		= 1,
	.periods_max		= PAGE_SIZE/sizeof(ls_dma_desc),
	.buffer_bytes_max	= 1024 * 1024,
};
static void ls_pcm_generic_dma_irq(void *dev_id);

static int __ls_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct ls_runtime_data *rtd = runtime->private_data;
	size_t totsize = params_buffer_bytes(params);
	size_t period = params_period_bytes(params);
	dma_addr_t dma_buff_phys;
	struct dma_async_tx_descriptor *desc = NULL;
	dma_cookie_t cookie;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	rtd->totsize = totsize;
	runtime->dma_bytes = totsize;
	dma_buff_phys = runtime->dma_addr;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		rtd->tx_dma_buf = dma_buff_phys;
		desc = dmaengine_prep_dma_cyclic(rtd->tx_ch,
										 rtd->tx_dma_buf,
										 totsize, period, DMA_MEM_TO_DEV,
										 DMA_PREP_INTERRUPT);
	} else {
		rtd->rx_dma_buf = dma_buff_phys;
		desc = dmaengine_prep_dma_cyclic(rtd->rx_ch,
										 rtd->rx_dma_buf,
										 totsize, period, DMA_DEV_TO_MEM,
										 DMA_PREP_INTERRUPT);
	}

	if (!desc) {
		printk("dma prep cyclic failed.\n");
		return -ENODEV;
    }

	desc->callback = ls_pcm_generic_dma_irq;
	desc->callback_param = substream;

	cookie = dmaengine_submit(desc);

	return 0;
}

static int __ls_pcm_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static int ls_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct ls_runtime_data *prtd = substream->runtime->private_data;
	struct dma_chan *chan;
	int ret = 0;
	uint32_t data;

	chan = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?  prtd->tx_ch : prtd->rx_ch;
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		dma_async_issue_pending(chan);

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			data = readl(prtd->i2s_ctl_base);
			data &= ~(0x1 << 12);
			writel(data, prtd->i2s_ctl_base);
			data = readl(prtd->i2s_ctl_base);
			data |= 0xc010 | (1 << 7) | (1 << 12);
			writel(data, prtd->i2s_ctl_base);
		} else {
		   data = readl(prtd->i2s_ctl_base);
		   data |= 0xc010 | (1 << 11) | (1 << 13);
		   writel(data, prtd->i2s_ctl_base);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		dmaengine_terminate_all(chan);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dmaengine_terminate_all(chan);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dma_async_issue_pending(chan);
		udelay(1000);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t ls_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct ls_runtime_data *prtd = runtime->private_data;

	snd_pcm_uframes_t x;
	u64 addr;
	enum dma_status status;
	struct dma_tx_state state;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		status = dmaengine_tx_status(prtd->tx_ch, prtd->tx_ch->cookie, &state);
	else
		status = dmaengine_tx_status(prtd->rx_ch, prtd->rx_ch->cookie, &state);

	addr = prtd->totsize - state.residue;

	x = bytes_to_frames(runtime, addr);

	if (x == runtime->buffer_size)
		x = 0;

	return x;
}

static int __ls_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct ls_runtime_data *prtd = substream->runtime->private_data;

	if (!prtd || !prtd->params)
		return 0;

	return 0;
}

static void ls_pcm_generic_dma_irq(void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;

	snd_pcm_period_elapsed(substream);
}

static int __ls_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct ls_runtime_data *prtd;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret;
	int irq;

	runtime->hw = ls_pcm_hardware;

	if (substream->pcm->device & 1) {
		runtime->hw.info &= ~SNDRV_PCM_INFO_INTERLEAVED;
		runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;
	}
	if (substream->pcm->device & 2)
		runtime->hw.info &= ~(SNDRV_PCM_INFO_MMAP |
				      SNDRV_PCM_INFO_MMAP_VALID);
	/*
	 * For mysterious reasons (and despite what the manual says)
	 * playback samples are lost if the DMA count is not a multiple
	 * of the DMA burst size.  Let's add a rule to enforce that.
	 */
	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 128);
	if (ret)
		goto out;

	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 128);
	if (ret)
		goto out;

	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		goto out;

	ret = -ENOMEM;
	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (!prtd)
		goto out;

	runtime->private_data = prtd;

	prtd->base_phys = generic_prtd.base_phys;
	prtd->base = generic_prtd.base;

	prtd->tx_ch = generic_prtd.tx_ch;
	prtd->tx_dma_buf = generic_prtd.tx_dma_buf;
	prtd->tx_buf = generic_prtd.tx_buf;

	prtd->rx_ch = generic_prtd.rx_ch;
	prtd->rx_dma_buf = generic_prtd.rx_dma_buf;
	prtd->rx_buf = generic_prtd.rx_buf;

	prtd->i2s_ctl_base = generic_prtd.i2s_ctl_base;

	return 0;

 err1:
	kfree(rtd);
 out:
	return ret;
}

static int __ls_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct ls_runtime_data *prtd = runtime->private_data;

	kfree(prtd);

	return 0;
}

static int ls_pcm_mmap(struct snd_pcm_substream *substream,
			 struct vm_area_struct *vma)
{
	return remap_pfn_range(vma, vma->vm_start,
		       substream->dma_buffer.addr >> PAGE_SHIFT,
		       vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

static int ls_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = ls_pcm_hardware.buffer_bytes_max;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;

	buf->area = dma_alloc_coherent(pcm->card->dev, size,
					   &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	return 0;
}

static void ls_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;
		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;
		dma_free_coherent(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);
		buf->area = NULL;
	}
}

static int ls_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct ls_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct ls_pcm_dma_params *dma = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	if (!dma)
		return 0;

	if (prtd->params != dma || prtd->params == NULL) {
		prtd->params = dma;
	}

	return __ls_pcm_hw_params(substream, params);
}

static int ls_pcm_hw_free(struct snd_pcm_substream *substream)
{
	__ls_pcm_hw_free(substream);

	return 0;
}

static struct snd_pcm_ops ls_pcm_ops = {
	.open		= __ls_pcm_open,
	.close		= __ls_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= ls_pcm_hw_params,
	.hw_free	= ls_pcm_hw_free,
	.prepare	= __ls_pcm_prepare,
	.trigger	= ls_pcm_trigger,
	.pointer	= ls_pcm_pointer,
	.mmap		= ls_pcm_mmap,
};

static int ls_soc_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	int ret = 0;

    card->dev->coherent_dma_mask = DMA_MASK;

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		ret = ls_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		ret = ls_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
 out:
	return ret;
}

struct snd_soc_component_driver ls_soc_platform_generic = {
	.ops 		= &ls_pcm_ops,
	.pcm_new	= ls_soc_pcm_new,
	.pcm_free	= ls_pcm_free_dma_buffers,
};

static int ls_snd_soc_register_component(struct platform_device *pdev)
{
    pdev->dev.coherent_dma_mask = DMA_MASK;
    pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	pdev->dev.kobj.name = "loongson-i2s";

	return snd_soc_register_component(&pdev->dev, &ls_soc_platform_generic, NULL, 0);
}


static int ls_of_dma_rx_probe(struct ls_runtime_data *ls_data, struct platform_device *pdev)
{
	struct dma_slave_config config;
	struct dma_async_tx_descriptor *desc = NULL;
	struct device *dev = &pdev->dev;
	dma_cookie_t cookie;
	int ret;

	ls_data->rx_ch = dma_request_slave_channel(dev, "i2s_record");
	if (!ls_data->rx_ch) {
		dev_err(dev, "rx dma alloc failed\n");
		return -ENODEV;
	}

	ls_data->rx_buf = dma_alloc_coherent(dev, ls_pcm_hardware.buffer_bytes_max,
						 &ls_data->rx_dma_buf,
						 GFP_KERNEL);
	if (!ls_data->rx_buf) {
		ret = -ENOMEM;
		goto alloc_err;
	}

	memset(&config, 0, sizeof(config));

	config.src_addr = ls_data->base_phys + 0xc;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

	ret = dmaengine_slave_config(ls_data->rx_ch, &config);
	if (ret < 0) {
		dev_err(dev, "rx dma channel config failed\n");
		ret = -ENODEV;
		goto config_err;
	}

	return 0;

config_err:
	dma_free_coherent(dev,
			  ls_pcm_hardware.buffer_bytes_max, ls_data->rx_buf,
			  ls_data->rx_dma_buf);

alloc_err:
	dma_release_channel(ls_data->rx_ch);
	ls_data->rx_ch = NULL;

	return ret;
}

static int ls_of_dma_tx_probe(struct ls_runtime_data *ls_data, struct platform_device *pdev)
{
	struct dma_slave_config config;
	struct dma_async_tx_descriptor *desc = NULL;
	struct device *dev = &pdev->dev;
	dma_cookie_t cookie;
	int ret;

	ls_data->tx_ch = dma_request_slave_channel(dev, "i2s_play");
	if (!ls_data->tx_ch) {
		dev_err(dev, "tx dma alloc failed\n");
		return -ENODEV;
	}

	ls_data->tx_buf = dma_alloc_coherent(dev, ls_pcm_hardware.buffer_bytes_max,
						 &ls_data->tx_dma_buf,
						 GFP_KERNEL);
	if (!ls_data->tx_buf) {
		ret = -ENOMEM;
		goto alloc_err;
	}

	memset(&config, 0, sizeof(config));

	config.dst_addr = ls_data->base_phys + 0x10;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

	ret = dmaengine_slave_config(ls_data->tx_ch, &config);
	if (ret < 0) {
		dev_err(dev, "tx dma channel config failed\n");
		ret = -ENODEV;
		goto config_err;
	}

	return 0;

config_err:
	dma_free_coherent(dev,
			  ls_pcm_hardware.buffer_bytes_max, ls_data->tx_buf,
			  ls_data->tx_dma_buf);

alloc_err:
	dma_release_channel(ls_data->tx_ch);
	ls_data->tx_ch = NULL;

	return ret;
}

static int ls_soc_probe(struct platform_device *pdev)
{
	int ret = 0;
	int timeout = 20000;

	struct resource *r;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no IO memory resource defined\n");
		ret = -ENODEV;
	}

	generic_prtd.base_phys = r->start;
	generic_prtd.base = (uint64_t *)ioremap(r->start, 0x14);
	generic_prtd.i2s_ctl_base = generic_prtd.base + 0x8;

	ret = ls_of_dma_rx_probe(&generic_prtd, pdev);
	if (ret)
		dev_info(&pdev->dev, "interrupt mode used for rx (no dma)\n");

	ret = ls_of_dma_tx_probe(&generic_prtd, pdev);
	if (ret)
		dev_info(&pdev->dev, "interrupt mode used for tx (no dma)\n");

	writel(0x8, generic_prtd.i2s_ctl_base);
	while ((!(readl(generic_prtd.i2s_ctl_base) & 0x10000)) && timeout--)
		udelay(5);
	writel(0x8008, generic_prtd.i2s_ctl_base);
	while ((!(readl(generic_prtd.i2s_ctl_base) & 0x100)) && timeout--)
		udelay(5);
	return ls_snd_soc_register_component(pdev);
}

static int ls_soc_remove(struct platform_device *pdev)
{

	if (generic_prtd.rx_ch)
		dma_release_channel(generic_prtd.rx_ch);

	if (generic_prtd.rx_dma_buf)
		dma_free_coherent(&pdev->dev,
						  ls_pcm_hardware.buffer_bytes_max, generic_prtd.rx_buf,
						  generic_prtd.rx_dma_buf);

	if (generic_prtd.tx_ch)
		dma_release_channel(generic_prtd.tx_ch);

	if (generic_prtd.tx_dma_buf)
		dma_free_coherent(&pdev->dev,
						  ls_pcm_hardware.buffer_bytes_max, generic_prtd.tx_buf,
						  generic_prtd.tx_dma_buf);

	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id snd_ls_dt_match[] = {
	{ .compatible = "loongson,ls-pcm-generic-audio", },
	{},
};
MODULE_DEVICE_TABLE(of, snd_ls_dt_match);
#endif

static struct platform_driver ls_generic_pcm_driver = {
	.driver = {
			.name = "ls-generic-pcm-audio",
			.owner = THIS_MODULE,
#ifdef CONFIG_OF
      .of_match_table = of_match_ptr(snd_ls_dt_match),
#endif
	},
	.probe = ls_soc_probe,
	.remove = ls_soc_remove,
};

static int __init ls_generic_pcm_init(void)
{
	return platform_driver_register(&ls_generic_pcm_driver);
}

static void __exit ls_generic_pcm_exit(void)
{
	platform_driver_unregister(&ls_generic_pcm_driver);
}

module_init(ls_generic_pcm_init)
module_exit(ls_generic_pcm_exit)

MODULE_AUTHOR("loongson");
MODULE_DESCRIPTION("Loongson generic pcm driver");
MODULE_LICENSE("GPL");

