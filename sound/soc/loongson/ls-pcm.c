/*
 * linux/sound/mips/ls-pcm.c -- ALSA PCM interface for the Loongson chip
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

#define DMA1ADDR 0x100
#define DMA2ADDR 0x110


unsigned long long i2s_ctl;

static struct ls_runtime_data g_prtd;
static struct device *dev;
static struct platform_device *plat_dev;
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
static unsigned int revision_id;

int __ls_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct ls_runtime_data *rtd = runtime->private_data;
	size_t totsize = params_buffer_bytes(params);
	size_t period = params_period_bytes(params);
	ls_dma_desc *dma_desc;
	dma_addr_t dma_buff_phys, next_desc_phys;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;

	dma_desc = rtd->dma_desc_array;
	next_desc_phys = rtd->dma_desc_array_phys;
	dma_buff_phys = runtime->dma_addr;
	do {
		next_desc_phys += sizeof(ls_dma_desc);
		dma_desc->ordered = (next_desc_phys | 0x1);
		dma_desc->ordered_hi = (next_desc_phys >> 32);

		dma_desc->saddr = dma_buff_phys;
		dma_desc->saddr_hi = (dma_buff_phys >> 32);
		dma_desc->daddr = rtd->params->dev_addr;
		dma_desc->cmd = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
				0x00001001 : 0x00000001;

		if (period > totsize)
			period = totsize;
		dma_desc->length = 8;
		dma_desc->step_length = 0;
		dma_desc->step_times = period >> 5;

		dma_desc++;
		dma_buff_phys += period;

	} while (totsize -= period);
	dma_desc[-1].ordered = (rtd->dma_desc_array_phys | 0x1);
	dma_desc[-1].ordered_hi = ((rtd->dma_desc_array_phys) >> 32);

	return 0;
}
EXPORT_SYMBOL(__ls_pcm_hw_params);

int __ls_pcm_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}
EXPORT_SYMBOL(__ls_pcm_hw_free);

int ls_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct ls_runtime_data *prtd = substream->runtime->private_data;
	struct device *dev = substream->pcm->card->dev;
	int ret = 0;
	u64 val, dma_order;
	int timeout = 20000;
	void *order_addr;
	uint32_t data;

	order_addr = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?  prtd->order_addr1 : prtd->order_addr2;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:

		val = (prtd->dma_desc_array_phys & ~0x1fUL) | 0x8UL;

		if (dev->coherent_dma_mask == DMA_BIT_MASK(64))
				val |= 0x1UL;
		else
				val &= ~0x1UL;

		dma_order = (readq(order_addr) & 0xfUL) | val;
		writeq(dma_order, order_addr);

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			if (revision_id == 0) {
				data = readl(i2s_ctl);
				data &= ~(0x1 << 12);
				writel(data, i2s_ctl);
				data = readl(i2s_ctl);
				data |= 0xd081;
				writel(data, i2s_ctl);
			} else if (revision_id == 1) {
				data = readl(i2s_ctl);
				data &= ~(0x1 << 12);
				writel(data, i2s_ctl);
				data = readl(i2s_ctl);
				data |= 0xc010 | (1 << 7) | (1 << 12);
				writel(data, i2s_ctl);
			} else
				printk("ERROR: IIS revision id is illegal.");
		} else {
			if (revision_id == 0) {
				data = readl(i2s_ctl);
				data |= 0xe802;
				writel(data, i2s_ctl);
			} else if (revision_id == 1) {
				data = readl(i2s_ctl);
				data |= 0xc010 | (1 << 11) | (1 << 13);
				writel(data, i2s_ctl);
			} else
				printk("ERROR: IIS revision id is illegal.");
		}

		while ((readl(order_addr) & 8) && timeout--)
				udelay(5);

		break;

	case SNDRV_PCM_TRIGGER_STOP:
		writeq(0x10UL | (readq(order_addr) & 0x1fUL), order_addr);
		udelay(1000);
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		val = (prtd->dma_desc_ready_phys & ~0x1fUL) | 0x4UL;

		if (dev->coherent_dma_mask == DMA_BIT_MASK(64))
				val |= 0x1UL;
		else
				val &= ~0x1UL;

		dma_order = (readq(order_addr) & 0x1fUL) | val;
		writeq(dma_order, order_addr);
		while (readl(order_addr)&4)
			;
		writel(0x10UL | (readq(order_addr) & 0x1fUL), order_addr);
		udelay(1000);
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		val = (prtd->dma_desc_ready_phys &  ~0x1fUL) | 0x8UL;

		if (dev->coherent_dma_mask == DMA_BIT_MASK(64))
				val |= 0x1UL;
		else
				val &= ~0x1UL;

		while ((readl(order_addr) & 8) && timeout--)
				udelay(5);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}
EXPORT_SYMBOL(ls_pcm_trigger);

snd_pcm_uframes_t
ls_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct ls_runtime_data *prtd = runtime->private_data;
	struct device *dev = substream->pcm->card->dev;

	snd_pcm_uframes_t x;
	u64 dma_order, val;
	u64 addr;
	void *order_addr;

	order_addr = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?  prtd->order_addr1 : prtd->order_addr2;
	val = (prtd->dma_position_desc_phys &  ~0x1fUL) | 0x4UL;

	if (dev->coherent_dma_mask == DMA_BIT_MASK(64))
		val |= 0x1UL;
	else
		val &= ~0x1UL;

	dma_order = (readq(order_addr) & 0x1fUL) | val;
	writeq(dma_order, order_addr);
	while (readl(order_addr) & 4)
		;

	addr = prtd->dma_position_desc->saddr_hi;
	addr = ((addr << 32) | (prtd->dma_position_desc->saddr & 0xffffffff));

	x = bytes_to_frames(runtime, addr - runtime->dma_addr);

	if (x == runtime->buffer_size)
		x = 0;
	return x;
}
EXPORT_SYMBOL(ls_pcm_pointer);

int __ls_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct ls_runtime_data *prtd = substream->runtime->private_data;

	if (!prtd || !prtd->params)
		return 0;

	return 0;
}
EXPORT_SYMBOL(__ls_pcm_prepare);

irqreturn_t ls_pcm_dma_irq(int irq, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	snd_pcm_period_elapsed(substream);

	return IRQ_HANDLED;
}
EXPORT_SYMBOL(ls_pcm_dma_irq);

int __ls_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct ls_runtime_data *prtd;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret;
	int irq;

	unsigned long int val;

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
	prtd->dma_desc_array =
		dma_alloc_coherent(substream->pcm->card->dev, PAGE_SIZE,
				       &prtd->dma_desc_array_phys, GFP_KERNEL);
	if (!prtd->dma_desc_array)
		goto err1;
	prtd->dma_desc_ready =
		dma_alloc_coherent(substream->pcm->card->dev, sizeof(ls_dma_desc),
					&prtd->dma_desc_ready_phys, GFP_KERNEL);
	if (!prtd->dma_desc_ready)
		goto err1;
	prtd->dma_position_desc =
		dma_alloc_coherent(substream->pcm->card->dev, sizeof(ls_dma_desc),
					&prtd->dma_position_desc_phys, GFP_KERNEL);
	if (!prtd->dma_position_desc)
		goto err1;

	runtime->private_data = prtd;

	prtd->base_phys = g_prtd.base_phys;
	prtd->base = g_prtd.base;

	prtd->order_addr1_phys = g_prtd.order_addr1_phys;
	prtd->order_addr1 = g_prtd.order_addr1;

	prtd->order_addr2_phys = g_prtd.order_addr2_phys;
	prtd->order_addr2 = g_prtd.order_addr2;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		irq = platform_get_irq(plat_dev, 0);
		ret = request_irq(irq, ls_pcm_dma_irq, IRQF_TRIGGER_RISING,
				"dma-write", substream);
		if (ret < 0) {
			printk("%s request_irq failed. ret=0x%x\r\n", __func__, ret);
			return ret;
		}
	} else {
		irq = platform_get_irq(plat_dev, 1);
		ret = request_irq(irq, ls_pcm_dma_irq, IRQF_TRIGGER_RISING,
				"dma-read", substream);
		if (ret < 0) {
			printk("%s request_irq failed. ret=0x%x\r\n", __func__, ret);
			return ret;
		}
	}

	return 0;

 err1:
	kfree(rtd);
 out:
	return ret;
}
EXPORT_SYMBOL(__ls_pcm_open);

int __ls_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct ls_runtime_data *prtd = runtime->private_data;
	int irq;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		irq = platform_get_irq(plat_dev, 0);
		free_irq(irq, substream);
	} else {
		irq = platform_get_irq(plat_dev, 1);
		free_irq(irq, substream);
	}

	dma_free_coherent(substream->pcm->card->dev, PAGE_SIZE,
			      prtd->dma_desc_array, prtd->dma_desc_array_phys);
	kfree(prtd);
	return 0;
}
EXPORT_SYMBOL(__ls_pcm_close);

static int ls_pcm_mmap(struct snd_pcm_substream *substream,
			 struct vm_area_struct *vma)
{
	return remap_pfn_range(vma, vma->vm_start,
		       substream->dma_buffer.addr >> PAGE_SHIFT,
		       vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

int ls_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
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
EXPORT_SYMBOL(ls_pcm_preallocate_dma_buffer);

void ls_pcm_free_dma_buffers(struct snd_pcm *pcm)
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
EXPORT_SYMBOL(ls_pcm_free_dma_buffers);

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

	if (!card->dev->dma_mask)
			card->dev->dma_mask = (uint64_t *)dev->coherent_dma_mask;
	card->dev->coherent_dma_mask = dev->coherent_dma_mask;

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

struct snd_soc_component_driver ls_soc_platform = {
	.ops 		= &ls_pcm_ops,
	.pcm_new	= ls_soc_pcm_new,
	.pcm_free	= ls_pcm_free_dma_buffers,
};
EXPORT_SYMBOL_GPL(ls_soc_platform);;

static int ls_snd_soc_register_component(struct platform_device *pdev)
{
	uint32_t val;

	dev = &pdev->dev;
	plat_dev = pdev;

	device_property_read_u32(dev, "dma-mask", &val);
	pdev->dev.coherent_dma_mask = val;

	if (pdev->dev.dma_mask)
		*(pdev->dev.dma_mask) = pdev->dev.coherent_dma_mask;
	else
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	pdev->dev.kobj.name = "loongson-i2s";

	return snd_soc_register_component(&pdev->dev, &ls_soc_platform, NULL, 0);
}

static int ls_soc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct pci_dev *pci_dev;
	resource_size_t dma_1;
	resource_size_t dma_2;
	int timeout = 20000;

	if (ACPI_COMPANION(&pdev->dev) || (pci_dev = pci_get_device(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_LOONGSON_I2S, NULL))) {
		resource_size_t mmio_base, mmio_size;
		struct pci_dev *ppdev;
		static void __iomem *pci_i2s_reg;

		ppdev = pci_get_device(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_LOONGSON_I2S, NULL);

		if (ppdev) {
			ret = pci_enable_device(ppdev);

			mmio_base = pci_resource_start(ppdev, 0);
			mmio_size = pci_resource_len(ppdev, 0);
			pci_i2s_reg = ioremap(mmio_base, mmio_size);
			revision_id = ppdev->revision;
		}

		dma_1 = ((uint64_t)pci_i2s_reg | DMA1ADDR);
		dma_2 = ((uint64_t)pci_i2s_reg | DMA2ADDR);

		i2s_ctl = pci_i2s_reg + 0x8;

		g_prtd.base_phys = (dma_addr_t)pci_i2s_reg;
		g_prtd.base = pci_i2s_reg;

		g_prtd.order_addr1_phys = (dma_addr_t)dma_1;
		g_prtd.order_addr1 = (uint64_t *)dma_1;

		g_prtd.order_addr2_phys = (dma_addr_t)dma_2;
		g_prtd.order_addr2 = (uint64_t *)dma_2;
	} else {
		struct dma_chan *chan;
		__be32 *of_property = NULL;
		struct resource *r;
		r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (r == NULL) {
			dev_err(&pdev->dev, "no IO memory resource defined\n");
			ret = -ENODEV;
		}

		chan = of_dma_request_slave_channel(pdev->dev.of_node, "i2s_play");
		if (chan == NULL) {
			dev_err(&pdev->dev, "no APBDMA-2 resource defined\n");
			return -ENODEV;
		}

		of_property = (__be32 *)of_get_property(chan->device->dev->of_node, "reg", NULL);
		if (of_property != 0)
			dma_1 = of_read_number(of_property, 2);

		chan = of_dma_request_slave_channel(pdev->dev.of_node, "i2s_record");
		if (chan == NULL) {
			dev_err(&pdev->dev, "no APBDMA-3 resource defined\n");
			return -ENODEV;
		}

		of_property = (__be32 *)of_get_property(chan->device->dev->of_node, "reg", NULL);
		if (of_property != 0)
			dma_2 = of_read_number(of_property, 2);

		g_prtd.base_phys = r->start;
		g_prtd.base = (uint64_t *)ioremap(r->start, 0x10);
		i2s_ctl = g_prtd.base + 0x8;

		g_prtd.order_addr1_phys = dma_1;
		g_prtd.order_addr1 = (uint64_t *)ioremap(dma_1, 8);

		g_prtd.order_addr2_phys = dma_2;
		g_prtd.order_addr2 = (uint64_t *)ioremap(dma_2, 8);
	}

	if (revision_id == 1) {
		writel(0x8, i2s_ctl);
		while ((!(readl(i2s_ctl) & 0x10000)) && timeout--)
			udelay(5);
		writel(0x8008, i2s_ctl);
		while ((!(readl(i2s_ctl) & 0x100)) && timeout--)
			udelay(5);
	}

	return ls_snd_soc_register_component(pdev);
}

static int ls_soc_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id snd_ls_dt_match[] = {
	{ .compatible = "loongson,ls-pcm-audio", },
	{ .compatible = "loongson,loongson2-pcm-audio", },
	{ .compatible = "loongson,ls7a-pcm-audio", },
	{},
};
MODULE_DEVICE_TABLE(of, snd_ls_dt_match);
#endif

static struct platform_driver loongson_pcm_driver = {
	.driver = {
			.name = "loongson-pcm-audio",
			.owner = THIS_MODULE,
#ifdef CONFIG_OF
      .of_match_table = of_match_ptr(snd_ls_dt_match),
#endif
	},
	.probe = ls_soc_probe,
	.remove = ls_soc_remove,
};

static int __init loongson_pcm_init(void)
{
	return platform_driver_register(&loongson_pcm_driver);
}

static void __exit loongson_pcm_exit(void)
{
	platform_driver_unregister(&loongson_pcm_driver);
}

module_init(loongson_pcm_init)
module_exit(loongson_pcm_exit)

MODULE_AUTHOR("loongson");
MODULE_DESCRIPTION("Loongson sound driver");
MODULE_LICENSE("GPL");

