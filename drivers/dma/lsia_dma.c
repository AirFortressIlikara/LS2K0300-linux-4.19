/*
 * LOONGSON LSIA DMA controller
 *
 * Copyright (C) 2024 Loongson Technology Corporation Limited
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "virt-dma.h"

#define DMA_ISR			0x0000 /* DMA Int Status Reg */
#define DMA_IFCR		0x0004 /* DMA Int Flag Clear Reg */
#define DMA_TCI			BIT(1) /* Transfer Complete Interrupt */
#define DMA_HTI			BIT(2) /* Half Transfer Interrupt */
#define DMA_TEI			BIT(3) /* Transfer Error Interrupt */
#define DMA_MASKI		(DMA_TCI \
				 | DMA_HTI \
				 | DMA_TEI)

/* DMA channel x Configuration Register */
#define DMA_CCR(x)		(0x0008 + 0x14 * (x))
#define DMA_CCR_M2M		BIT(14)
#define DMA_CCR_PL_MASK		GENMASK(13, 12)
#define DMA_CCR_PL(n)		((n & 0x3) << 12)
#define DMA_CCR_MSIZE_MASK	GENMASK(11, 10)
#define DMA_CCR_MSIZE(n)	((n & 0x3) << 10)
#define DMA_CCR_PSIZE_MASK	GENMASK(9, 8)
#define DMA_CCR_PSIZE(n)	((n & 0x3) << 8)
#define DMA_CCR_MINC		BIT(7) /* Memory increment mode */
#define DMA_CCR_PINC		BIT(6) /* Peripheral increment mode */
#define DMA_CCR_CIRC		BIT(5) /* Circular mode */
#define DMA_CCR_DIR		BIT(4)
#define DMA_CCR_TEIE		BIT(3) /* Transfer Error Int Enable */
#define DMA_CCR_HTIE		BIT(2) /* Half Transfer Complete Int Enable */
#define DMA_CCR_TCIE		BIT(1) /* Transfer Complete Int Enable */
#define DMA_CCR_EN		BIT(0) /* Stream Enable */
#define DMA_CCR_CFG_MASK	(DMA_CCR_PINC \
				| DMA_CCR_MINC \
				| DMA_CCR_PL_MASK)
#define DMA_CCR_IRQ_MASK	(DMA_CCR_TCIE \
				| DMA_CCR_HTIE \
				| DMA_CCR_TEIE)

#define DMA_CNDTR(x)		(0x000c + 0x14 * (x))
#define DMA_CPAR(x)		(0x0010 + 0x14 * (x))
#define DMA_CMAR(x)		(0x0014 + 0x14 * (x))


#define DMA_MAX_CHANNELS	8

#define LS_DMA_BUSWIDTHS	(BIT(DMA_SLAVE_BUSWIDTH_1_BYTE)  | \
				 BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) | \
				 BIT(DMA_SLAVE_BUSWIDTH_4_BYTES))

struct lsia_dma_chan_reg {
	u32 dma_ccr;
	u32 dma_cndtr;
	u32 dma_cpar;
	u32 dma_cmar;
};

struct lsia_dma_sg_req {
	u32 len;
	struct lsia_dma_chan_reg chan_reg;
};

struct lsia_dma_desc {
	struct virt_dma_desc vdesc;
	bool cyclic;
	u32 num_sgs;
	struct lsia_dma_sg_req sg_req[];
};

struct lsia_dma_chan {
	struct virt_dma_chan vchan;
	u32 id;
	u32 irq;
	struct lsia_dma_desc *desc;
	u32 next_sg;
	struct dma_slave_config	dma_sconfig;
	struct lsia_dma_chan_reg chan_reg;
};

struct lsia_dma_device {
	struct dma_device ddev;
	void __iomem *base;
	u32 dma_channels;
	struct lsia_dma_chan chan[DMA_MAX_CHANNELS];
};

static struct lsia_dma_device *lsia_dma_get_dev(struct lsia_dma_chan *chan)
{
	return container_of(chan->vchan.chan.device, struct lsia_dma_device,
			    ddev);
}

static struct lsia_dma_chan *to_lsia_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct lsia_dma_chan, vchan.chan);
}

static struct lsia_dma_desc *to_lsia_dma_desc(struct virt_dma_desc *vdesc)
{
	return container_of(vdesc, struct lsia_dma_desc, vdesc);
}

static struct device *chan2dev(struct lsia_dma_chan *chan)
{
	return &chan->vchan.chan.dev->device;
}

static u32 lsia_dma_read(struct lsia_dma_device *dmadev, u32 reg)
{
	return readl_relaxed(dmadev->base + reg);
}

static void lsia_dma_write(struct lsia_dma_device *dmadev, u32 reg, u32 val)
{
	writel_relaxed(val, dmadev->base + reg);
}

static struct lsia_dma_desc *lsia_dma_alloc_desc(u32 num_sgs)
{
	return kzalloc(sizeof(struct lsia_dma_desc) +
		       sizeof(struct lsia_dma_sg_req) * num_sgs, GFP_NOWAIT);
}

static int lsia_dma_slave_config(struct dma_chan *c,
				  struct dma_slave_config *config)
{
	struct lsia_dma_chan *chan = to_lsia_dma_chan(c);

	memcpy(&chan->dma_sconfig, config, sizeof(*config));

	return 0;
}

static void lsia_dma_irq_clear(struct lsia_dma_chan *chan, u32 flags)
{
	struct lsia_dma_device *dmadev = lsia_dma_get_dev(chan);
	u32 dma_ifcr;

	dma_ifcr = flags << (4 * chan->id);
	lsia_dma_write(dmadev, DMA_IFCR, dma_ifcr);
}

static void lsia_dma_stop(struct lsia_dma_chan *chan)
{
	struct lsia_dma_device *dmadev = lsia_dma_get_dev(chan);
	u32 dma_ccr;

	dma_ccr = lsia_dma_read(dmadev, DMA_CCR(chan->id));
	dma_ccr &= ~(DMA_CCR_IRQ_MASK | DMA_CCR_EN);
	lsia_dma_write(dmadev, DMA_CCR(chan->id), dma_ccr);

	lsia_dma_irq_clear(chan, DMA_MASKI);
}

static int lsia_dma_terminate_all(struct dma_chan *c)
{
	struct lsia_dma_chan *chan = to_lsia_dma_chan(c);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&chan->vchan.lock, flags);
	if (chan->desc) {
		vchan_terminate_vdesc(&chan->desc->vdesc);
		lsia_dma_stop(chan);
		chan->desc = NULL;
	}
	vchan_get_all_descriptors(&chan->vchan, &head);
	spin_unlock_irqrestore(&chan->vchan.lock, flags);
	vchan_dma_desc_free_list(&chan->vchan, &head);

	return 0;
}

static void lsia_dma_synchronize(struct dma_chan *c)
{
	struct lsia_dma_chan *chan = to_lsia_dma_chan(c);

	vchan_synchronize(&chan->vchan);
}

static void lsia_dma_dump_reg(struct lsia_dma_chan *chan)
{
	struct lsia_dma_device *dev = lsia_dma_get_dev(chan);
	u32 id = chan->id;

	dev_dbg(chan2dev(chan), "CR:  %x\n", lsia_dma_read(dev, DMA_CCR(id)));
	dev_dbg(chan2dev(chan), "NDTR:%x\n", lsia_dma_read(dev, DMA_CNDTR(id)));
	dev_dbg(chan2dev(chan), "PAR: %x\n", lsia_dma_read(dev, DMA_CPAR(id)));
	dev_dbg(chan2dev(chan), "MAR: %x\n", lsia_dma_read(dev, DMA_CMAR(id)));
}

static void lsia_dma_start_transfer(struct lsia_dma_chan *chan)
{
	struct lsia_dma_device *dmadev = lsia_dma_get_dev(chan);
	struct virt_dma_desc *vdesc;
	struct lsia_dma_sg_req *sg_req;
	struct lsia_dma_chan_reg *reg;

	lsia_dma_stop(chan);

	if (!chan->desc) {
		vdesc = vchan_next_desc(&chan->vchan);
		if (!vdesc)
			return;
		list_del(&vdesc->node);
		chan->desc = to_lsia_dma_desc(vdesc);
		chan->next_sg = 0;
	}

	if (chan->next_sg == chan->desc->num_sgs)
		chan->next_sg = 0;

	sg_req = &chan->desc->sg_req[chan->next_sg];
	reg = &sg_req->chan_reg;

	lsia_dma_write(dmadev, DMA_CCR(chan->id), reg->dma_ccr);
	lsia_dma_write(dmadev, DMA_CNDTR(chan->id), reg->dma_cndtr);
	lsia_dma_write(dmadev, DMA_CPAR(chan->id), reg->dma_cpar);
	lsia_dma_write(dmadev, DMA_CMAR(chan->id), reg->dma_cmar);

	chan->next_sg++;

	lsia_dma_dump_reg(chan);

	/* Start DMA */
	reg->dma_ccr |= DMA_CCR_EN;
	lsia_dma_write(dmadev, DMA_CCR(chan->id), reg->dma_ccr);

	dev_dbg(chan2dev(chan), "vchan %pK: started\n", &chan->vchan);
}

static void lsia_dma_configure_next_sg(struct lsia_dma_chan *chan)
{
	struct lsia_dma_device *dmadev = lsia_dma_get_dev(chan);
	struct lsia_dma_sg_req *sg_req;
	u32 dma_cmar, id;
	u32 dma_ccr;

	id = chan->id;

	if (chan->next_sg == chan->desc->num_sgs)
		chan->next_sg = 0;

	/* stop to update mem addr */
	dma_ccr = lsia_dma_read(dmadev, DMA_CCR(id));
	dma_ccr &= ~DMA_CCR_EN;
	lsia_dma_write(dmadev, DMA_CCR(id), dma_ccr);

	sg_req = &chan->desc->sg_req[chan->next_sg];
	dma_cmar = sg_req->chan_reg.dma_cmar;
	lsia_dma_write(dmadev, DMA_CMAR(id), dma_cmar);

	/* start transition */
	dma_ccr |= DMA_CCR_EN;
	lsia_dma_write(dmadev, DMA_CCR(id), dma_ccr);
}

static void lsia_dma_handle_chan_done(struct lsia_dma_chan *chan)
{
	if (chan->desc) {
		if (chan->desc->cyclic) {
			vchan_cyclic_callback(&chan->desc->vdesc);
			/* DMA_CCR_CIRC mode don't need update register */
			if (chan->desc->num_sgs == 1)
				return;
			lsia_dma_configure_next_sg(chan);
			chan->next_sg++;
		} else {
			if (chan->next_sg == chan->desc->num_sgs) {
				vchan_cookie_complete(&chan->desc->vdesc);
				chan->desc = NULL;
			}
			lsia_dma_start_transfer(chan);
		}
	}
}

static irqreturn_t lsia_dma_chan_irq(int irq, void *devid)
{
	struct lsia_dma_chan *chan = devid;
	struct lsia_dma_device *dmadev = lsia_dma_get_dev(chan);
	u32 status, scr;

	spin_lock(&chan->vchan.lock);

	status = (lsia_dma_read(dmadev, DMA_ISR) >> (4 * chan->id)) & DMA_MASKI;
	scr = lsia_dma_read(dmadev, DMA_CCR(chan->id));
	status &= scr;

	if (status & DMA_TCI)
		lsia_dma_handle_chan_done(chan);

	if (status & DMA_HTI)
		lsia_dma_irq_clear(chan, DMA_HTI);

	if (status & DMA_TEI)
		dev_err(chan2dev(chan), "Trans Error\n");

	lsia_dma_irq_clear(chan, status);

	spin_unlock(&chan->vchan.lock);

	return IRQ_HANDLED;
}

static void lsia_dma_issue_pending(struct dma_chan *c)
{
	struct lsia_dma_chan *chan = to_lsia_dma_chan(c);
	unsigned long flags;

	spin_lock_irqsave(&chan->vchan.lock, flags);
	if (vchan_issue_pending(&chan->vchan) && !chan->desc) {
		dev_dbg(chan2dev(chan), "vchan %pK: issued\n", &chan->vchan);
		lsia_dma_start_transfer(chan);

	}
	spin_unlock_irqrestore(&chan->vchan.lock, flags);
}

static int lsia_dma_set_xfer_param(struct lsia_dma_chan *chan,
				    enum dma_transfer_direction direction,
				    enum dma_slave_buswidth *buswidth,
				    u32 buf_len)
{
	enum dma_slave_buswidth dev_width;
	int mem_size, periph_size;
	u32 dma_ccr;

	switch (direction) {
	case DMA_MEM_TO_DEV:
		dev_width = chan->dma_sconfig.dst_addr_width;
		chan->chan_reg.dma_cpar = chan->dma_sconfig.dst_addr;
		dma_ccr = DMA_CCR_DIR;
		break;
	case DMA_DEV_TO_MEM:
		dev_width = chan->dma_sconfig.src_addr_width;
		chan->chan_reg.dma_cpar = chan->dma_sconfig.src_addr;
		dma_ccr = DMA_CCR_MINC;
		break;
	default:
		return -EINVAL;
	}
	*buswidth = dev_width;
	if ((dev_width > 4) || (dev_width < 0))
		return -EINVAL;

	periph_size = (dev_width >> 1) & 0x3;
	/* Set memory data size to 4 bytes */
	mem_size = 0x2;
	dma_ccr |= DMA_CCR_PSIZE(periph_size) | DMA_CCR_MSIZE(mem_size);

	/* Set DMA control register */
	chan->chan_reg.dma_ccr &= ~(DMA_CCR_PSIZE_MASK | DMA_CCR_MSIZE_MASK);
	chan->chan_reg.dma_ccr |= dma_ccr;

	return 0;
}

static struct dma_async_tx_descriptor *lsia_dma_prep_slave_sg(
	struct dma_chan *c, struct scatterlist *sgl,
	u32 sg_len, enum dma_transfer_direction direction,
	unsigned long flags, void *context)
{
	struct lsia_dma_chan *chan = to_lsia_dma_chan(c);
	struct lsia_dma_desc *desc;
	struct scatterlist *sg;
	enum dma_slave_buswidth buswidth;
	u32 nb_data_items;
	int i;

	desc = lsia_dma_alloc_desc(sg_len);
	if (!desc)
		return NULL;

	for_each_sg(sgl, sg, sg_len, i) {
		if (lsia_dma_set_xfer_param(chan, direction, &buswidth,
					  sg_dma_len(sg)))
			return NULL;

		desc->sg_req[i].len = sg_dma_len(sg);

		nb_data_items = desc->sg_req[i].len / buswidth;
		if (nb_data_items >= (1 << 16)) {
			dev_err(chan2dev(chan), "nb items not supported\n");
			kfree(desc);
			return NULL;
		}
		desc->sg_req[i].chan_reg.dma_ccr = chan->chan_reg.dma_ccr;
		desc->sg_req[i].chan_reg.dma_cpar = chan->chan_reg.dma_cpar;
		desc->sg_req[i].chan_reg.dma_cmar = sg_dma_address(sg);
		desc->sg_req[i].chan_reg.dma_cndtr = nb_data_items;
	}

	desc->num_sgs = sg_len;
	desc->cyclic = false;

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static struct dma_async_tx_descriptor *lsia_dma_prep_dma_cyclic(
	struct dma_chan *c, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction direction,
	unsigned long flags)
{
	struct lsia_dma_chan *chan = to_lsia_dma_chan(c);
	struct lsia_dma_desc *desc;
	enum dma_slave_buswidth buswidth;
	u32 num_periods, nb_data_items;
	int i;

	if (buf_len % period_len)
		return NULL;

	if (lsia_dma_set_xfer_param(chan, direction, &buswidth, period_len))
		return NULL;

	nb_data_items = period_len / buswidth;
	if (nb_data_items >= (1 << 16)) {
		dev_err(chan2dev(chan), "number of items not supported\n");
		return NULL;
	}

	/* Enable Circular mode */
	if (buf_len == period_len)
		chan->chan_reg.dma_ccr |= DMA_CCR_CIRC;

	num_periods = buf_len / period_len;

	desc = lsia_dma_alloc_desc(num_periods);
	if (!desc)
		return NULL;

	for (i = 0; i < num_periods; i++) {
		desc->sg_req[i].len = period_len;
		desc->sg_req[i].chan_reg.dma_ccr = chan->chan_reg.dma_ccr;
		desc->sg_req[i].chan_reg.dma_cpar = chan->chan_reg.dma_cpar;
		desc->sg_req[i].chan_reg.dma_cmar = buf_addr;
		desc->sg_req[i].chan_reg.dma_cndtr = nb_data_items;
		buf_addr += period_len;
	}

	desc->num_sgs = num_periods;
	desc->cyclic = true;

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static size_t lsia_dma_desc_residue(struct lsia_dma_chan *chan,
				     struct lsia_dma_desc *desc,
				     u32 next_sg)
{
	u32 residue, width, ndtr;
	int i;
	struct lsia_dma_device *dmadev = lsia_dma_get_dev(chan);

	width = (lsia_dma_read(dmadev, DMA_CCR(chan->id)) >> 8) & 0x3;
	ndtr = lsia_dma_read(dmadev, DMA_CNDTR(chan->id));
	residue = ndtr << width;

	if (chan->desc->cyclic && next_sg == 0)
		return residue;

	for (i = next_sg; i < desc->num_sgs; i++)
		residue += desc->sg_req[i].len;

	return residue;
}

static enum dma_status lsia_dma_tx_status(struct dma_chan *c,
					   dma_cookie_t cookie,
					   struct dma_tx_state *state)
{
	struct lsia_dma_chan *chan = to_lsia_dma_chan(c);
	struct virt_dma_desc *vdesc;
	enum dma_status status;
	unsigned long flags;

	status = dma_cookie_status(c, cookie, state);
	if (status == DMA_COMPLETE || !state)
		return status;

	spin_lock_irqsave(&chan->vchan.lock, flags);
	vdesc = vchan_find_desc(&chan->vchan, cookie);
	if (chan->desc && cookie == chan->desc->vdesc.tx.cookie)
		state->residue = lsia_dma_desc_residue(chan, chan->desc,
						 chan->next_sg);
	else if (vdesc)
		state->residue = lsia_dma_desc_residue(chan,
						 to_lsia_dma_desc(vdesc), 0);

	spin_unlock_irqrestore(&chan->vchan.lock, flags);

	return status;
}

static int lsia_dma_alloc_chan_resources(struct dma_chan *c)
{
	struct lsia_dma_chan *chan = to_lsia_dma_chan(c);

	lsia_dma_stop(chan);
	return 0;
}

static void lsia_dma_free_chan_resources(struct dma_chan *c)
{
	vchan_free_chan_resources(to_virt_chan(c));
}

static void lsia_dma_desc_free(struct virt_dma_desc *vdesc)
{
	kfree(container_of(vdesc, struct lsia_dma_desc, vdesc));
}

static struct dma_chan *lsia_dma_of_xlate(struct of_phandle_args *dma_spec,
					   struct of_dma *ofdma)
{
	struct lsia_dma_device *dmadev = ofdma->of_dma_data;
	struct device *dev = dmadev->ddev.dev;
	struct lsia_dma_chan *chan;
	struct dma_chan *c;
	u32 channel_id, stream_config;

	if (dma_spec->args_count < 2)
		return NULL;

	channel_id = dma_spec->args[0];
	stream_config = dma_spec->args[1];

	if (channel_id >= DMA_MAX_CHANNELS) {
		dev_err(dev, "Bad channel\n");
		return NULL;
	}

	chan = &dmadev->chan[channel_id];

	c = dma_get_slave_channel(&chan->vchan.chan);
	if (!c) {
		dev_err(dev, "No more channels available\n");
		return NULL;
	}

	memset(&chan->chan_reg, 0, sizeof(struct lsia_dma_chan_reg));
	chan->chan_reg.dma_ccr = stream_config & (DMA_CCR_CFG_MASK
				 | DMA_CCR_IRQ_MASK);

	return c;
}

static int lsia_dma_probe(struct platform_device *pdev)
{
	struct lsia_dma_chan *chan;
	struct lsia_dma_device *dmadev;
	struct dma_device *dd;
	struct resource *res; /* IO mem resources */
	int chls, i, ret, irq;

	dmadev = devm_kzalloc(&pdev->dev, sizeof(*dmadev), GFP_KERNEL);
	if (!dmadev)
		return -ENOMEM;

	dd = &dmadev->ddev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dmadev->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dmadev->base))
		return PTR_ERR(dmadev->base);

	dd->dev = &pdev->dev;

	dma_cap_set(DMA_SLAVE, dd->cap_mask);
	dma_cap_set(DMA_PRIVATE, dd->cap_mask);
	dma_cap_set(DMA_CYCLIC, dd->cap_mask);
	dd->device_alloc_chan_resources = lsia_dma_alloc_chan_resources;
	dd->device_config = lsia_dma_slave_config;
	dd->device_prep_slave_sg = lsia_dma_prep_slave_sg;
	dd->device_prep_dma_cyclic = lsia_dma_prep_dma_cyclic;
	dd->device_issue_pending = lsia_dma_issue_pending;
	dd->device_synchronize = lsia_dma_synchronize;
	dd->device_tx_status = lsia_dma_tx_status;
	dd->device_terminate_all = lsia_dma_terminate_all;
	dd->device_free_chan_resources = lsia_dma_free_chan_resources;

	dd->src_addr_widths = LS_DMA_BUSWIDTHS;
	dd->dst_addr_widths = LS_DMA_BUSWIDTHS;
	dd->directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);

	INIT_LIST_HEAD(&dd->channels);

	if (of_property_read_u32((&pdev->dev)->of_node, "dma-channels", &chls))
		chls = DMA_MAX_CHANNELS;
	dmadev->dma_channels = chls;

	for (i = 0; i < chls; i++) {
		chan = &dmadev->chan[i];
		chan->id = i;
		chan->vchan.desc_free = lsia_dma_desc_free;
		vchan_init(&chan->vchan, dd);
	}

	ret = dma_async_device_register(dd);
	if (ret)
		return ret;

	for (i = 0; i < chls; i++) {
		chan = &dmadev->chan[i];
		irq = platform_get_irq(pdev, i);
		if (irq < 0) {
			dev_err(&pdev->dev, "failed to get IRQ: %d\n", irq);
			ret = -EINVAL;
			goto err_unregister;
		}
		chan->irq = irq;
		ret = request_irq(chan->irq, lsia_dma_chan_irq, 0,
				  dev_name(chan2dev(chan)), chan);
		if (ret) {
			dev_err(&pdev->dev, "request %d err %d\n", irq, ret);
			goto err_unregister;
		}
	}

	ret = of_dma_controller_register(pdev->dev.of_node,
					 lsia_dma_of_xlate, dmadev);
	if (ret) {
		dev_err(&pdev->dev, "DMA registration failed %d\n", ret);
		goto err_unregister;
	}

	platform_set_drvdata(pdev, dmadev);

	dev_info(&pdev->dev, "DMA driver registered\n");

	return 0;

err_unregister:
	dma_async_device_unregister(dd);

	return ret;
}

static int lsia_dma_remove(struct platform_device *pdev)
{
	struct lsia_dma_device *dmadev = platform_get_drvdata(pdev);
	int i;

	of_dma_controller_free(pdev->dev.of_node);

	for (i = 0; i < dmadev->dma_channels; i++) {
		free_irq(dmadev->chan[i].irq, dmadev);
		tasklet_kill(&dmadev->chan[i].vchan.task);
	}
	dma_async_device_unregister(&dmadev->ddev);

	return 0;
}

static const struct of_device_id lsia_dma_of_match[] = {
	{ .compatible = "loongson,lsia-dma", },
	{},
};
MODULE_DEVICE_TABLE(of, lsia_dma_of_match);

static struct platform_driver lsia_dma_driver = {
	.driver = {
		.name = "loongson-lsia-dma",
		.of_match_table = lsia_dma_of_match,
	},
	.probe		= lsia_dma_probe,
	.remove		= lsia_dma_remove,
};

static int __init lsia_dma_init(void)
{
	return platform_driver_register(&lsia_dma_driver);
}
subsys_initcall(lsia_dma_init);

static void __exit lsia_dma_exit(void)
{
	platform_driver_unregister(&lsia_dma_driver);
}
module_exit(lsia_dma_exit);

MODULE_AUTHOR("Loongson Technology Corporation Limited");
MODULE_DESCRIPTION("Looongson LSIA DMA Controller driver");
MODULE_LICENSE("GPL");
