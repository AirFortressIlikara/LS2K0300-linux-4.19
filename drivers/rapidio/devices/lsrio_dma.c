/*
 * DMA Engine support for Loongson Rapid IO controller
 *
 * Copyright 2023 Loongson Technology, Inc.
 * Yinggang Gu <guyinggang@loongson.cn>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called COPYING.
 */

#include <linux/io.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include "../../dma/dmaengine.h"

#include "lsrio.h"

static int lsrio_submit_sg(struct lsrio_tx_desc *desc);

static unsigned int dma_desc_per_channel = 64;
module_param(dma_desc_per_channel, uint, 0444);
MODULE_PARM_DESC(dma_desc_per_channel,
		 "Number of DMA descriptors per channel (default: 64)");

static unsigned int dma_queue_sz = 16;
module_param(dma_queue_sz, uint, 0444);
MODULE_PARM_DESC(dma_queue_sz,
		 "DMA Transactions Queue Size (default: 16)");

static inline struct lsrio_dma_chan *to_lsrio_chan(struct dma_chan *chan)
{
	return container_of(chan, struct lsrio_dma_chan, dchan);
}

static inline struct lsrio_device *to_lsrio(struct dma_device *ddev)
{
	return container_of(ddev, struct rio_mport, dma)->priv;
}

static inline
struct lsrio_tx_desc *to_lsrio_desc(struct dma_async_tx_descriptor *txd)
{
	return container_of(txd, struct lsrio_tx_desc, txd);
}

static void lsrio_dma_reset(struct lsrio_dma_chan *chan, u8 dir)
{
	struct lsrio_device *priv = to_lsrio(chan->dchan.device);
	u32 reg_val, reset_mask = 0;

	if (dir & LSRIO_DMA_DIR_W)
		reset_mask |= (1 << LSRIO_RAB_RST_CTRL_WDMA_SHIFT) << chan->id;

	if (dir & LSRIO_DMA_DIR_R)
		reset_mask |= (1 << LSRIO_RAB_RST_CTRL_RDMA_SHIFT) << chan->id;

	reg_val = lsrio_apb_readl(priv, LSRIO_RAB_RST_CTRL);
	reg_val |= reset_mask;
	lsrio_apb_writel(priv, LSRIO_RAB_RST_CTRL, reg_val);
	reg_val &= (~reset_mask);
	lsrio_apb_writel(priv, LSRIO_RAB_RST_CTRL, reg_val);
}

static void lsrio_clr_stat(struct lsrio_dma_chan *ldma_chan, u8 dir)
{
	struct lsrio_device *priv = to_lsrio(ldma_chan->dchan.device);

	if (dir & LSRIO_DMA_DIR_W)
		lsrio_apb_writel(priv, LSRIO_RAB_WDMA_STAT(ldma_chan->id),
					LSRIO_RAB_DMA_STAT_CLA);

	if (dir & LSRIO_DMA_DIR_R)
		lsrio_apb_writel(priv, LSRIO_RAB_RDMA_STAT(ldma_chan->id),
					LSRIO_RAB_DMA_STAT_CLA);
}

static int lsrio_dma_ch_init(struct lsrio_dma_chan *ldma_chan)
{
	struct lsrio_device *priv = to_lsrio(ldma_chan->dchan.device);
	struct lsrio_dma_desc *desc_ptr;
	dma_addr_t desc_phys;

	ls_debug(DMA, &ldma_chan->dchan.dev->device, "DMAC%d", ldma_chan->id);

	/*
	 * Allocate space for DMA descriptors
	 * (add an extra element for link descriptor)
	 */
	desc_ptr = dma_pool_alloc(priv->dma_desc_pool, GFP_ATOMIC, &desc_phys);
	if (!desc_ptr)
		return -ENOMEM;

	ldma_chan->tx_desc_num	= dma_queue_sz;
	ldma_chan->tx_desc_phys = desc_phys;
	ldma_chan->tx_desc_base = desc_ptr;

	desc_ptr = dma_pool_alloc(priv->dma_desc_pool, GFP_ATOMIC, &desc_phys);
	if (!desc_ptr) {
		dma_pool_free(priv->dma_desc_pool, ldma_chan->tx_desc_base,
			ldma_chan->tx_desc_phys);
		return -ENOMEM;
	}

	ldma_chan->rx_desc_num	= dma_queue_sz;
	ldma_chan->rx_desc_phys = desc_phys;
	ldma_chan->rx_desc_base = desc_ptr;

	/* Rest engine */
	lsrio_dma_reset(ldma_chan, LSRIO_DMA_DIR_ALL);

	/* Clear Status */
	lsrio_clr_stat(ldma_chan, LSRIO_DMA_DIR_ALL);

	return 0;
}

static void lsrio_dma_ch_free(struct lsrio_dma_chan *ldma_chan)
{
	struct lsrio_device *priv = to_lsrio(ldma_chan->dchan.device);

	if (!ldma_chan->tx_desc_base && !ldma_chan->rx_desc_base)
		return;

	lsrio_clr_stat(ldma_chan, LSRIO_DMA_DIR_ALL);

	dma_pool_free(priv->dma_desc_pool, ldma_chan->tx_desc_base,
			ldma_chan->tx_desc_phys);
	dma_pool_free(priv->dma_desc_pool, ldma_chan->rx_desc_base,
			ldma_chan->rx_desc_phys);

	ldma_chan->tx_desc_base = NULL;
	ldma_chan->rx_desc_base = NULL;
}

static void lsrio_dma_interrupt_enable(struct lsrio_dma_chan *dma_chan,
									int enable)
{
	struct lsrio_device *priv = to_lsrio(dma_chan->dchan.device);
	u32 reg_val, reg_off, reg_mask;

	/* Do not need decriptor finish interrupt */
	reg_mask = 0x10001 << dma_chan->id;

	reg_off = LSRIO_RAB_INTR_ENAB_WDMA;
	reg_val = lsrio_apb_readl(priv, reg_off);

	if (enable)
		reg_val |= reg_mask;
	else
		reg_val &= (~reg_mask);

	lsrio_apb_writel(priv, reg_off, reg_val);

	reg_off = LSRIO_RAB_INTR_ENAB_RDMA;
	reg_val = lsrio_apb_readl(priv, reg_off);

	if (enable)
		reg_val |= reg_mask;
	else
		reg_val &= (~reg_mask);

	lsrio_apb_writel(priv, reg_off, reg_val);
}

static bool lsrio_dma_is_idle(struct lsrio_dma_chan *ldma_chan, u8 dir)
{
	struct lsrio_device *priv = to_lsrio(ldma_chan->dchan.device);
	int ch = ldma_chan->id;
	u32 sts = 0;

	if (dir & LSRIO_DMA_DIR_W)
		sts |= lsrio_apb_readl(priv, LSRIO_RAB_WDMA_STAT(ch));

	if (dir & LSRIO_DMA_DIR_R)
		sts |= lsrio_apb_readl(priv, LSRIO_RAB_RDMA_STAT(ch));

	return (sts & LSRIO_RAB_DMA_STAT_BUSY) == 0;
}

static void lsrio_start_dma(struct lsrio_dma_chan *ldma_chan,
			struct lsrio_tx_desc *desc)
{
	struct lsrio_device *priv = to_lsrio(ldma_chan->dchan.device);
	u32 reg_val;

	reg_val = LSRIO_RAB_DMA_CTRL_ST |
		(desc->destid << LSRIO_RAB_DMA_CTRL_DID_SHITF);

	if (desc->dir == LSRIO_DMA_DIR_W) {
		lsrio_apb_writel(priv, LSRIO_RAB_WDMA_ADDR(ldma_chan->id),
			((u64)ldma_chan->tx_desc_base >> 2) & LSRIO_RAB_DMA_ADDR_MASK);
		lsrio_apb_writel(priv, LSRIO_RAB_WDMA_ADDR_EXT(ldma_chan->id), 0);
		lsrio_apb_writel(priv, LSRIO_RAB_WDMA_CTRL(ldma_chan->id), reg_val);
	} else {
		lsrio_apb_writel(priv, LSRIO_RAB_RDMA_ADDR(ldma_chan->id),
			((u64)ldma_chan->rx_desc_base >> 2) & LSRIO_RAB_DMA_ADDR_MASK);
		lsrio_apb_writel(priv, LSRIO_RAB_RDMA_ADDR_EXT(ldma_chan->id), 0);
		lsrio_apb_writel(priv, LSRIO_RAB_RDMA_CTRL(ldma_chan->id), reg_val);
	}
}

static int lsrio_desc_fill(struct lsrio_tx_desc *desc, int *idx,
			struct scatterlist *sg, bool last)
{
	struct lsrio_dma_chan *ldma_chan = to_lsrio_chan(desc->txd.chan);
	struct lsrio_dma_desc *desc_ptr;
	u32 sg_len = sg_dma_len(sg);
	u32 dma_len;
	int desc_id = *idx;
	int move = 0;
	dma_addr_t next;

	/* The remaining descriptors are not sufficient */
	if (sg_len > ((dma_desc_per_channel - desc_id) * LSRIO_DMA_MAX_BCOUNT))
		return -ENOMEM;

	if (desc->dir == LSRIO_DMA_DIR_W) {
		desc_ptr = &ldma_chan->tx_desc_base[desc_id];
		next = ldma_chan->tx_desc_phys +
			(desc_id + 1) * sizeof(struct lsrio_dma_desc);
	} else {
		desc_ptr = &ldma_chan->rx_desc_base[desc_id];
		next = ldma_chan->rx_desc_phys +
			(desc_id + 1) * sizeof(struct lsrio_dma_desc);
	}

	while (sg_len) {
		if (sg_len > LSRIO_DMA_MAX_BCOUNT)
			dma_len = LSRIO_DMA_MAX_BCOUNT;
		else
			dma_len = sg_len;

		sg_len -= dma_len;
		desc_ptr->desc_ctrl = LSRIO_DMAD_VALID | LSRIO_DMAD_NDVAL |
			((dma_len >> 2) << LSRIO_DMAD_WRCNT_SHIFT);

		if (desc->dir == LSRIO_DMA_DIR_W) {
			desc_ptr->src_addr = (sg_dma_address(sg) + move) >> 2;
			desc_ptr->dst_addr = (desc->rio_addr + move) >> 2;
		} else {
			desc_ptr->src_addr = (desc->rio_addr + move) >> 2;
			desc_ptr->dst_addr = (sg_dma_address(sg) + move) >> 2;
		}

		if (((sg_len == 0) && last) || (desc_id == (dma_desc_per_channel - 1)))
			desc_ptr->nxtd_addr = 0;
		else
			desc_ptr->nxtd_addr = next >> 3;

		wmb();

		move += dma_len;
		desc_id++;
		desc_ptr++;
		next += sizeof(struct lsrio_dma_desc);
	}

	*idx = desc_id;

	return 0;
}

static void lsrio_dma_tx_err(struct lsrio_dma_chan *dma_chan,
			      struct lsrio_tx_desc *desc)
{
	struct dma_async_tx_descriptor *txd = &desc->txd;
	dma_async_tx_callback callback = txd->callback;
	void *param = txd->callback_param;

	list_move(&desc->desc_node, &dma_chan->free_list);

	if (callback)
		callback(param);
}

static int lsrio_submit_sg(struct lsrio_tx_desc *desc)
{
	struct dma_chan *chan = desc->txd.chan;
	struct lsrio_dma_chan *ldma_chan = to_lsrio_chan(chan);
	struct device *ch_dev = &chan->dev->device;
	struct lsrio_dma_desc *desc_base = NULL;
	struct scatterlist *sg = desc->sg;
	u64 rio_addr = 0;
	int i, err = 0, idx = 0;

	if (!lsrio_dma_is_idle(ldma_chan, desc->dir)) {
		ls_err(ch_dev, "DMAC%d ERR: Attempt to use non-idle channel",
			ldma_chan->id);
		return -EIO;
	}

	/*
	 * Fill DMA channel's hardware buffer descriptors.
	 */
	rio_addr = desc->rio_addr;

	if (desc->dir == LSRIO_DMA_DIR_W)
		desc_base = ldma_chan->tx_desc_base;
	else if (desc->dir == LSRIO_DMA_DIR_R)
		desc_base = ldma_chan->rx_desc_base;
	else {
		ls_err(ch_dev, "DMAC%d ERR: Descriptor direction invalid",
			ldma_chan->id);
		return -EINVAL;
	}

	/* Initialize DMA descriptor */
	for_each_sg(desc->sg, sg, desc->sg_len, i) {
		ls_debug(DMAV, ch_dev, "DMAC%d sg%d/%d addr: 0x%llx len: %d",
			ldma_chan->id, i, desc->sg_len,
			(unsigned long long)sg_dma_address(sg), sg_dma_len(sg));

		desc->rio_addr = rio_addr;

		if (sg_is_last(sg)) {
			err = lsrio_desc_fill(desc, &idx, sg, true);
			if (!err)
				desc->sg_len = 0;
		} else
			err = lsrio_desc_fill(desc, &idx, sg, false);

		if (err < 0) {
			/* Last descriptor should be NULL */
			if (idx > 0) {
				desc_base[idx-1].nxtd_addr = 0;
				err = 0;
			}

			desc->sg_len -= i;
			desc->sg = sg;
			ls_debug(DMAV, ch_dev, "Descriptors is full");
			break;
		}

		rio_addr += sg_dma_len(sg);
	}

	return err;
}

static void lsrio_advance_work(struct lsrio_dma_chan *ldma_chan,
				struct lsrio_tx_desc *desc)
{
	struct lsrio_tx_desc *_d;
	u8 dir_wait = 0;
	int err;

	ls_debug(DMA, &ldma_chan->dchan.dev->device, "DMAC%d", ldma_chan->id);

	if (!desc && !list_empty(&ldma_chan->queue)) {
		if (lsrio_dma_is_idle(ldma_chan, LSRIO_DMA_DIR_W)
				&& !ldma_chan->active_tx)
			dir_wait |= LSRIO_DMA_DIR_W;

		if (lsrio_dma_is_idle(ldma_chan, LSRIO_DMA_DIR_R)
				&& !ldma_chan->active_rx)
			dir_wait |= LSRIO_DMA_DIR_R;

		if (dir_wait) {
			list_for_each_entry_safe(desc, _d, &ldma_chan->queue, desc_node) {
				if (desc->dir & dir_wait)
					break;
			}

			if (desc->dir & dir_wait) {
				if (desc->dir == LSRIO_DMA_DIR_W)
					ldma_chan->active_tx = desc;
				else if (desc->dir == LSRIO_DMA_DIR_R)
					ldma_chan->active_rx = desc;

				list_del_init((&desc->desc_node));
			} else
				desc = NULL;
		}
	}

	if (desc) {
		if (!lsrio_dma_is_idle(ldma_chan, desc->dir))
			return;

		err = lsrio_submit_sg(desc);
		if (!err)
			lsrio_start_dma(ldma_chan, desc);
		else {
			lsrio_dma_tx_err(ldma_chan, desc);
			ls_debug(DMA, &ldma_chan->dchan.dev->device,
				"DMAC%d ERR: lsrio_submit_sg failed with err=%d",
				ldma_chan->id, err);
		}
	}

	ls_debug(DMA, &ldma_chan->dchan.dev->device, "DMAC%d Exit", ldma_chan->id);
}

static void lsrio_dma_clean_desc(struct lsrio_dma_chan *ldma_chan, u8 dir)
{
	struct lsrio_device *priv = to_lsrio(ldma_chan->dchan.device);
	struct lsrio_tx_desc **active;
	struct lsrio_tx_desc *desc;
	struct lsrio_dma_desc *desc_base;
	u32 dma_sts, desc_num;

	if (dir == LSRIO_DMA_DIR_W) {
		dma_sts = lsrio_apb_readl(priv, LSRIO_RAB_WDMA_STAT(ldma_chan->id));
		active = &ldma_chan->active_tx;
		desc_base = ldma_chan->tx_desc_base;
		desc_num = ldma_chan->tx_desc_num;
	} else {
		dma_sts = lsrio_apb_readl(priv, LSRIO_RAB_RDMA_STAT(ldma_chan->id));
		active = &ldma_chan->active_rx;
		desc_base = ldma_chan->rx_desc_base;
		desc_num = ldma_chan->rx_desc_num;
	}

	desc = *active;

	if (!desc || !dma_sts)
		return;

	ls_debug(DMA, &ldma_chan->dchan.dev->device,
			"DMAC%d_STS = 0x%x did=%d raddr=0x%llx",
			ldma_chan->id, dma_sts, desc->destid, desc->rio_addr);

	lsrio_clr_stat(ldma_chan, dir);

	if (dma_sts & LSRIO_RAB_DMA_STAT_ERR) {
		spin_lock(&ldma_chan->lock);

		lsrio_dma_reset(ldma_chan, dir);
		desc->status = DMA_ERROR;
		dma_cookie_complete(&desc->txd);
		list_add(&desc->desc_node, &ldma_chan->free_list);
		*active = NULL;
		if (ldma_chan->active)
			lsrio_advance_work(ldma_chan, NULL);
		spin_unlock(&ldma_chan->lock);
	}

	if (dma_sts & LSRIO_RAB_DMA_STAT_CTC) {
		spin_lock(&ldma_chan->lock);

		if (desc->sg_len == 0) {
			dma_async_tx_callback callback = NULL;
			void *param = NULL;

			desc->status = DMA_COMPLETE;
			dma_cookie_complete(&desc->txd);
			if (desc->txd.flags & DMA_PREP_INTERRUPT) {
				callback = desc->txd.callback;
				param = desc->txd.callback_param;
			}
			list_add(&desc->desc_node, &ldma_chan->free_list);
			*active = NULL;
			if (ldma_chan->active)
				lsrio_advance_work(ldma_chan, NULL);
			spin_unlock(&ldma_chan->lock);
			if (callback)
				callback(param);
		} else {
			if (ldma_chan->active)
				lsrio_advance_work(ldma_chan, *active);
			spin_unlock(&ldma_chan->lock);
		}
	}
}

static void lsrio_dma_tasklet(unsigned long data)
{
	struct lsrio_dma_chan *ldma_chan = (struct lsrio_dma_chan *)data;

	lsrio_dma_clean_desc(ldma_chan, LSRIO_DMA_DIR_W);
	lsrio_dma_clean_desc(ldma_chan, LSRIO_DMA_DIR_R);

	lsrio_dma_interrupt_enable(ldma_chan, 1);
}

void lsrio_dma_handler(struct lsrio_dma_chan *ldma_chan)
{
	lsrio_dma_interrupt_enable(ldma_chan, 0);

	if (ldma_chan->active)
		tasklet_schedule(&ldma_chan->tasklet);
}

static dma_cookie_t lsrio_tx_submit(struct dma_async_tx_descriptor *txd)
{
	struct lsrio_tx_desc *desc = to_lsrio_desc(txd);
	struct lsrio_dma_chan *ldma_chan = to_lsrio_chan(txd->chan);
	dma_cookie_t cookie;

	/* Check if the descriptor is detached from any lists */
	if (!list_empty(&desc->desc_node)) {
		ls_err(&ldma_chan->dchan.dev->device,
			"DMAC%d wrong state of descriptor %p",
			ldma_chan->id, txd);
		return -EIO;
	}

	spin_lock_bh(&ldma_chan->lock);

	if (!ldma_chan->active) {
		spin_unlock_bh(&ldma_chan->lock);
		return -ENODEV;
	}

	cookie = dma_cookie_assign(txd);
	desc->status = DMA_IN_PROGRESS;

	list_add_tail(&desc->desc_node, &ldma_chan->queue);
	lsrio_advance_work(ldma_chan, NULL);

	spin_unlock_bh(&ldma_chan->lock);

	return cookie;
}

static int lsrio_alloc_chan_resources(struct dma_chan *dchan)
{
	struct lsrio_dma_chan *ldma_chan = to_lsrio_chan(dchan);
	struct lsrio_tx_desc *desc;
	int i;

	ls_debug(DMA, &dchan->dev->device, "DMAC%d", ldma_chan->id);
	/* Initialize DMA channel */
	if (lsrio_dma_ch_init(ldma_chan)) {
		ls_err(&dchan->dev->device, "Unable to initialize DMAC%d",
			ldma_chan->id);
		return -ENODEV;
	}

	/* Allocate queue of transaction descriptors */
	desc = kcalloc(dma_queue_sz, sizeof(struct lsrio_tx_desc), GFP_ATOMIC);
	if (!desc) {
		lsrio_dma_ch_free(ldma_chan);
		return -ENOMEM;
	}

	for (i = 0; i < dma_queue_sz; i++) {
		dma_async_tx_descriptor_init(&desc[i].txd, dchan);
		desc[i].txd.tx_submit = lsrio_tx_submit;
		desc[i].txd.flags = DMA_CTRL_ACK;
		list_add(&desc[i].desc_node, &ldma_chan->free_list);
	}

	dma_cookie_init(dchan);
	ldma_chan->active = true;
	lsrio_dma_interrupt_enable(ldma_chan, 1);

	return dma_queue_sz;
}

static void lsrio_sync_dma_irq(struct lsrio_dma_chan *ldma_chan)
{
	struct lsrio_device *priv = to_lsrio(ldma_chan->dchan.device);

	synchronize_irq(priv->pdev->irq);
}

static void lsrio_free_chan_resources(struct dma_chan *dchan)
{
	struct lsrio_dma_chan *ldma_chan = to_lsrio_chan(dchan);

	ls_debug(DMA, &dchan->dev->device, "DMAC%d", ldma_chan->id);

	if (!ldma_chan->tx_desc_base || !ldma_chan->rx_desc_base)
		return;

	lsrio_dma_interrupt_enable(ldma_chan, 0);
	ldma_chan->active = false;
	lsrio_sync_dma_irq(ldma_chan);
	tasklet_kill(&ldma_chan->tasklet);
	INIT_LIST_HEAD(&ldma_chan->free_list);
	lsrio_dma_ch_free(ldma_chan);
}

static enum dma_status lsrio_tx_status(struct dma_chan *dchan,
				dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	return dma_cookie_status(dchan, cookie, txstate);
}

static void lsrio_issue_pending(struct dma_chan *dchan)
{
	struct lsrio_dma_chan *ldma_chan = to_lsrio_chan(dchan);

	ls_debug(DMA, &dchan->dev->device, "DMAC%d", ldma_chan->id);

	spin_lock_bh(&ldma_chan->lock);

	if (lsrio_dma_is_idle(ldma_chan, LSRIO_DMA_DIR_ALL) && ldma_chan->active)
		lsrio_advance_work(ldma_chan, NULL);

	spin_unlock_bh(&ldma_chan->lock);
}

static struct dma_async_tx_descriptor *lsrio_prep_rio_sg(struct dma_chan *dchan,
			struct scatterlist *sgl, unsigned int sg_len,
			enum dma_transfer_direction dir, unsigned long flags,
			void *tinfo)
{
	struct lsrio_dma_chan *ldma_chan = to_lsrio_chan(dchan);
	struct lsrio_tx_desc *desc;
	struct rio_dma_ext *rext = tinfo;
	struct dma_async_tx_descriptor *txd = NULL;

	if (!sgl || !sg_len) {
		ls_err(&dchan->dev->device, "DMAC%d No SG list",
			ldma_chan->id);
		return ERR_PTR(-EINVAL);
	}

	ls_debug(DMA, &dchan->dev->device, "DMAC%d %s", ldma_chan->id,
		  (dir == DMA_DEV_TO_MEM)?"READ":"WRITE");

	spin_lock_bh(&ldma_chan->lock);

	if (!list_empty(&ldma_chan->free_list)) {
		desc = list_first_entry(&ldma_chan->free_list,
				struct lsrio_tx_desc, desc_node);
		list_del_init(&desc->desc_node);
		desc->destid	= rext->destid;
		desc->rio_addr	= rext->rio_addr;
		desc->sg_len	= sg_len;
		desc->sg		= sgl;

		if (dir == DMA_DEV_TO_MEM)
			desc->dir = LSRIO_DMA_DIR_R;
		else if (dir == DMA_MEM_TO_DEV)
			desc->dir = LSRIO_DMA_DIR_W;

		txd				= &desc->txd;
		txd->flags		= flags;
	}

	spin_unlock_bh(&ldma_chan->lock);

	if (!txd) {
		ls_debug(DMA, &dchan->dev->device,
			  "DMAC%d free TXD is not available", ldma_chan->id);
		return ERR_PTR(-EBUSY);
	}

	return txd;
}

static int lsrio_terminate_all(struct dma_chan *dchan)
{
	struct lsrio_dma_chan *ldma_chan = to_lsrio_chan(dchan);
	struct lsrio_tx_desc *desc, *_d;
	LIST_HEAD(list);

	ls_debug(DMA, &dchan->dev->device, "DMAC%d", ldma_chan->id);

	spin_lock_bh(&ldma_chan->lock);

	while (!lsrio_dma_is_idle(ldma_chan, LSRIO_DMA_DIR_ALL))
		udelay(5);

	ldma_chan->active = false;

	if (ldma_chan->active_tx)
		list_add(&ldma_chan->active_tx->desc_node, &list);
	if (ldma_chan->active_rx)
		list_add(&ldma_chan->active_rx->desc_node, &list);

	list_splice_init(&ldma_chan->queue, &list);

	list_for_each_entry_safe(desc, _d, &list, desc_node)
		lsrio_dma_tx_err(ldma_chan, desc);

	spin_unlock_bh(&ldma_chan->lock);

	return 0;
}

void lsrio_dma_stop_all(struct lsrio_device *priv)
{
	u32 reg_val;

	reg_val = lsrio_apb_readl(priv, LSRIO_RAB_CTRL);
	reg_val &= ~(LSRIO_RAB_CTRL_WDMA_EN | LSRIO_RAB_CTRL_RDMA_EN |
			LSRIO_RAB_CTRL_DMA_1_PF);
	lsrio_apb_writel(priv, LSRIO_RAB_CTRL, reg_val);
}

int lsrio_register_dma(struct lsrio_device *priv)
{
	struct rio_mport *mport = &priv->mport;
	int nr_channels = 0;
	int err = 0;
	int i = 0;
	u32 reg_val;

	/*
	 * enable RAB WDMA and RDMA
	 * Default: arrange Prefetch Size = 1 of DMA Descriptors Array
	 */
	reg_val = lsrio_apb_readl(priv, LSRIO_RAB_CTRL);
	reg_val |= (LSRIO_RAB_CTRL_WDMA_EN | LSRIO_RAB_CTRL_RDMA_EN |
			LSRIO_RAB_CTRL_DMA_1_PF);
	lsrio_apb_writel(priv, LSRIO_RAB_CTRL, reg_val);

	reg_val = lsrio_apb_readl(priv, LSRIO_RAB_CAPA);
	priv->dma_num = min((reg_val >> 12) & 0xf, (reg_val >> 8) & 0xf);
	nr_channels = priv->dma_num;

	INIT_LIST_HEAD(&mport->dma.channels);
	for (i = 0; i < nr_channels; i++) {
		struct lsrio_dma_chan *lsrio_dchan = &priv->dma[i];

		lsrio_dchan->dchan.device = &mport->dma;
		lsrio_dchan->dchan.cookie = 1;
		lsrio_dchan->dchan.chan_id = i;
		lsrio_dchan->id = i;
		lsrio_dchan->active = false;

		/* init a lock to sychronize the use of associated channel */
		spin_lock_init(&lsrio_dchan->lock);
		/* point current active transfer descriptor on a channel */
		lsrio_dchan->active_tx = NULL;
		lsrio_dchan->active_rx = NULL;
		/* init a queue per channel for transfer */
		INIT_LIST_HEAD(&lsrio_dchan->queue);
		INIT_LIST_HEAD(&lsrio_dchan->free_list);

		tasklet_init(&lsrio_dchan->tasklet, lsrio_dma_tasklet,
				(unsigned long)lsrio_dchan);
		/* Add a channel to DMA device */
		list_add_tail(&lsrio_dchan->dchan.device_node, &mport->dma.channels);
	}

	priv->dma_desc_pool = dma_pool_create("Lsrio dma", &priv->pdev->dev,
				dma_desc_per_channel * sizeof(struct lsrio_dma_desc),
				16, 0);
	if (!priv->dma_desc_pool)
		return -ENOMEM;

	mport->dma.chancnt = nr_channels;

	dma_cap_zero(mport->dma.cap_mask);
	dma_cap_set(DMA_PRIVATE, mport->dma.cap_mask);
	dma_cap_set(DMA_SLAVE, mport->dma.cap_mask);
	mport->dma.dev = &priv->pdev->dev;
	mport->dma.device_alloc_chan_resources = lsrio_alloc_chan_resources;
	mport->dma.device_free_chan_resources = lsrio_free_chan_resources;
	mport->dma.device_tx_status = lsrio_tx_status;
	mport->dma.device_prep_slave_sg = lsrio_prep_rio_sg;
	mport->dma.device_issue_pending = lsrio_issue_pending;
	mport->dma.device_terminate_all = lsrio_terminate_all;

	err = dma_async_device_register(&mport->dma);
	if (err) {
		dma_pool_destroy(priv->dma_desc_pool);
		ls_err(&priv->pdev->dev, "Failed to register DMA device");
	}

	return err;
}

void lsrio_unregister_dma(struct lsrio_device *priv)
{
	struct rio_mport *mport = &priv->mport;
	struct dma_chan *chan, *_c;
	struct lsrio_dma_chan *ldma_chan;

	lsrio_dma_stop_all(priv);
	dma_async_device_unregister(&mport->dma);

	list_for_each_entry_safe(chan, _c, &mport->dma.channels,
					device_node) {
		ldma_chan = to_lsrio_chan(chan);
		if (ldma_chan->active) {
			lsrio_dma_interrupt_enable(ldma_chan, 0);
			ldma_chan->active = false;
			lsrio_sync_dma_irq(ldma_chan);
			tasklet_kill(&ldma_chan->tasklet);
			INIT_LIST_HEAD(&ldma_chan->free_list);
			lsrio_dma_ch_free(ldma_chan);
		}

		list_del(&chan->device_node);
	}

	dma_pool_destroy(priv->dma_desc_pool);
}
