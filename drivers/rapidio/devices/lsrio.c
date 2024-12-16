/*
 * RapidIO mport driver for Loongson Rapid IO controller
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
#include <linux/delay.h>

#include "lsrio.h"

#ifdef DEBUG
u32 ls_dbg_level = DBG_ALL;
module_param_named(dbg_level, ls_dbg_level, uint, 0644);
MODULE_PARM_DESC(dbg_level, "Debugging output level (default 0 = none)");
#endif

/* fixup hbdidlcsr */
#define LSRIO_HBDIDLCSR_DEF	0xffff
static u32 lsrio_hbdidlcsr_local = LSRIO_HBDIDLCSR_DEF;
static u32 lsrio_hbdidlcsr_hop[32];
static u32 lsrio_hbdidlcsr_id[0x100];

static void lsrio_omsg_handler(struct lsrio_device *priv, int ch);
static void lsrio_imsg_handler(struct lsrio_device *priv, int ch);

static inline bool lsrio_msg_desc_unhandled(struct lsrio_msg_desc *desc)
{
	return (desc->desc_ctrl & LSRIO_DME_DES_HANDLE_MASK) == 0;
}

void lsrio_set_apb_csr(struct lsrio_device *priv, u32 offset)
{
	u32 reg_val, page_sel;

	reg_val = readl(priv->apb_base +
		(LSRIO_RAB_APB_CSR & (LSRIO_REG_PAGESIZE - 1)));
	reg_val &= ~(0x1fff0000);
	page_sel = (offset >> 11) & 0x1fff;

	writel(reg_val | (page_sel << 16),
		priv->apb_base + (LSRIO_RAB_APB_CSR & (LSRIO_REG_PAGESIZE - 1)));
}

void lsrio_apb_writel(struct lsrio_device *priv, u32 offset, u32 value)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->apb_lock, flags);

	if ((offset >= LSRIO_RAB_REGBASE) &&
		(offset < (LSRIO_RAB_REGBASE + LSRIO_REG_PAGESIZE))) {
		writel(value, priv->apb_base + (offset & (LSRIO_REG_PAGESIZE - 1)));
		goto out;
	}

	lsrio_set_apb_csr(priv, offset);

	if (offset < LSRIO_RAB_REGBASE)
		value = cpu_to_be32(value);

	writel(value, priv->apb_base + LSRIO_REG_PAGESIZE +
		   (offset & (LSRIO_REG_PAGESIZE - 1)));

out:
	spin_unlock_irqrestore(&priv->apb_lock, flags);
}

u32 lsrio_apb_readl(struct lsrio_device *priv, u32 offset)
{
	unsigned long flags;
	u32 ret;

	spin_lock_irqsave(&priv->apb_lock, flags);

	if ((offset >= LSRIO_RAB_REGBASE) &&
			(offset < (LSRIO_RAB_REGBASE + LSRIO_REG_PAGESIZE))) {
		ret = readl(priv->apb_base + (offset & (LSRIO_REG_PAGESIZE - 1)));
		goto out;
	}

	lsrio_set_apb_csr(priv, offset);

	if (offset < LSRIO_RAB_REGBASE)
		ret = be32_to_cpu(readl(priv->apb_base + LSRIO_REG_PAGESIZE +
					(offset & (LSRIO_REG_PAGESIZE - 1))));
	else
		ret = readl(priv->apb_base + LSRIO_REG_PAGESIZE +
					(offset & (LSRIO_REG_PAGESIZE - 1)));

out:
	spin_unlock_irqrestore(&priv->apb_lock, flags);

	return ret;
}

static void lsrio_map_maint_window(struct lsrio_device *priv, u16 destid,
						u8 hopcount, u32 dst_base, u32 size)
{
	u32 amap_ctrl, amap_size, amap_abar, amap_rbar;

	amap_ctrl = LSRIO_RAB_APIO_AMAP_CTRL_EN |
				(AMAP_MAINT << LSRIO_RAB_APIO_AMAP_CTRL_TYPE_SHIFT) |
				(destid << LSRIO_RAB_APIO_AMAP_CTRL_DSTID_SHIFT);
	amap_size = size & LSRIO_RAB_APIO_AMAP_SIZE_MASK;
	amap_abar = (priv->ob_win[0].base >> 10) & LSRIO_RAB_APIO_AMAP_ABAR_MASK;
	amap_rbar = ((dst_base >> 10) |
				(hopcount << LSRIO_RAB_APIO_AMAP_RBAR_HC_SHIFT))
				& LSRIO_RAB_APIO_AMAP_RBAR_MASK;

	lsrio_apb_writel(priv, LSRIO_RAB_APIO_AMAP_CTRL(0), amap_ctrl);
	lsrio_apb_writel(priv, LSRIO_RAB_APIO_AMAP_SIZE(0), amap_size);
	lsrio_apb_writel(priv, LSRIO_RAB_APIO_AMAP_ABAR(0), amap_abar);
	lsrio_apb_writel(priv, LSRIO_RAB_APIO_AMAP_RBAR(0), amap_rbar);
}

static void lsrio_map_apio_window(struct lsrio_device *priv,
					struct lsrio_ob_win *win, u8 win_id)
{
	u32 amap_ctrl, amap_size, amap_abar, amap_rbar;

	amap_ctrl = LSRIO_RAB_APIO_AMAP_CTRL_EN |
				(win->type << LSRIO_RAB_APIO_AMAP_CTRL_TYPE_SHIFT) |
				(win->destid << LSRIO_RAB_APIO_AMAP_CTRL_DSTID_SHIFT);
	amap_size = win->size & LSRIO_RAB_APIO_AMAP_SIZE_MASK;
	amap_abar = (win->base >> 10) & LSRIO_RAB_APIO_AMAP_ABAR_MASK;
	amap_rbar = (win->rstart >> 10) & LSRIO_RAB_APIO_AMAP_RBAR_MASK;

	lsrio_apb_writel(priv, LSRIO_RAB_APIO_AMAP_CTRL(win_id), amap_ctrl);
	lsrio_apb_writel(priv, LSRIO_RAB_APIO_AMAP_SIZE(win_id), amap_size);
	lsrio_apb_writel(priv, LSRIO_RAB_APIO_AMAP_ABAR(win_id), amap_abar);
	lsrio_apb_writel(priv, LSRIO_RAB_APIO_AMAP_RBAR(win_id), amap_rbar);
}

static inline void lsrio_unmap_apio_window(struct lsrio_device *priv, u8 win_id)
{
	lsrio_apb_writel(priv, LSRIO_RAB_APIO_AMAP_CTRL(win_id), 0);
}

/**
 * lsrio_disable_ints - disables all device interrupts
 * @priv: pointer to lsrio private data
 */
static inline void lsrio_disable_ints(struct lsrio_device *priv)
{
	lsrio_apb_writel(priv, LSRIO_RAB_INTR_ENAB_MISC, 0);
	lsrio_apb_writel(priv, LSRIO_RAB_INTR_ENAB_GNRL, 0);
}

/**
 * lsrio_enable_ints - enables all device interrupts
 * @priv: pointer to lsrio private data
 */
static inline void lsrio_enable_ints(struct lsrio_device *priv)
{
	lsrio_apb_writel(priv, LSRIO_RAB_INTR_ENAB_MISC, LSRIO_RAB_INTR_MISC_ENALL);
	lsrio_apb_writel(priv, LSRIO_RAB_INTR_ENAB_GNRL, LSRIO_RAB_INTR_GNRL_ENALL);
}

/**
 * lsrio_lcread - read from local SREP config space
 * @mport: RapidIO master port info
 * @index: ID of RapdiIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be read into
 *
 * Generates a local SREP space read. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int lsrio_lcread(struct rio_mport *mport, int index, u32 offset,
			 int len, u32 *data)
{
	struct lsrio_device *priv = mport->priv;

	if (len != sizeof(u32))
		return -EINVAL; /* only 32-bit access is supported */

	if (offset == RIO_HOST_DID_LOCK_CSR) {
		*data = lsrio_hbdidlcsr_local;
		return 0;
	}

	*data = lsrio_apb_readl(priv, offset);

	return 0;
}

/**
 * lsrio_lcwrite - write into local SREP config space
 * @mport: RapidIO master port info
 * @index: ID of RapdiIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be written
 *
 * Generates a local write into SREP configuration space. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int lsrio_lcwrite(struct rio_mport *mport, int index, u32 offset,
			  int len, u32 data)
{
	struct lsrio_device *priv = mport->priv;

	if (len != sizeof(u32))
		return -EINVAL; /* only 32-bit access is supported */

	if (offset == RIO_HOST_DID_LOCK_CSR) {
		if (lsrio_hbdidlcsr_local == LSRIO_HBDIDLCSR_DEF)
			lsrio_hbdidlcsr_local = data;
		else if (lsrio_hbdidlcsr_local == data)
			lsrio_hbdidlcsr_local = LSRIO_HBDIDLCSR_DEF;

		return 0;
	}

	lsrio_apb_writel(priv, offset, data);

	return 0;
}

/**
 * lsrio_cread - Generate a RapidIO maintenance read transaction
 * @mport: RapidIO master port control structure
 * @index: ID of RapdiIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Location to be read into
 *
 * Generates a RapidIO maintenance read transaction.
 * Returns %0 on success and %-EINVAL or %-EFAULT on failure.
 */
static int lsrio_cread(struct rio_mport *mport, int index, u16 destid,
			u8 hopcount, u32 offset, int len, u32 *data)
{
	struct lsrio_device *priv = mport->priv;
	u32 win_size = priv->ob_win[0].size;
	u32 reg_val;

	if (offset == RIO_HOST_DID_LOCK_CSR) {
		if (hopcount == 0xff)
			*data = lsrio_hbdidlcsr_id[destid];
		else if (hopcount <= 31)
			*data = lsrio_hbdidlcsr_hop[hopcount];
		else
			return -EFAULT;

		return 0;
	}

	/* Clear status */
	reg_val = lsrio_apb_readl(priv, LSRIO_RAB_APIO_STAT);
	lsrio_apb_writel(priv, LSRIO_RAB_APIO_STAT, reg_val);

	lsrio_map_maint_window(priv, destid, hopcount,
					round_down(offset, win_size), win_size);

	*data = cpu_to_be32(readl(priv->axi_base + priv->ob_win[0].base
					+ (offset % win_size)));

	reg_val = lsrio_apb_readl(priv, LSRIO_RAB_APIO_STAT);
	if (reg_val) {
		lsrio_apb_writel(priv, LSRIO_RAB_APIO_STAT, reg_val);
		ls_err(&priv->pdev->dev, "RIO Cread failed");
		ls_debug(MAINT, &priv->pdev->dev,
			"Maint Read Hop %d id %d off 0x%x, status 0x%x",
			hopcount, destid, offset, reg_val);
		return -EFAULT;
	}

	return 0;
}

/**
 * lsrio_cwrite - Generate a RapidIO maintenance write transaction
 * @mport: RapidIO master port control structure
 * @index: ID of RapdiIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Value to be written
 *
 * Generates a RapidIO maintenance write transaction.
 * Returns %0 on success and %-EINVAL or %-EFAULT on failure.
 */
static int lsrio_cwrite(struct rio_mport *mport, int index, u16 destid,
			 u8 hopcount, u32 offset, int len, u32 data)
{
	struct lsrio_device *priv = mport->priv;
	u32 win_size = priv->ob_win[0].size;
	u32 reg_val;

	if (offset == RIO_DID_CSR)
		lsrio_hbdidlcsr_id[data] = lsrio_hbdidlcsr_hop[hopcount];

	if (offset == RIO_HOST_DID_LOCK_CSR) {
		u32 *lsrio_hbdidlcsr;

		if (hopcount == 0xff)
			lsrio_hbdidlcsr = &lsrio_hbdidlcsr_id[destid];
		else if (hopcount <= 31)
			lsrio_hbdidlcsr = &lsrio_hbdidlcsr_hop[hopcount];
		else
			return -EFAULT;

		if (*lsrio_hbdidlcsr == LSRIO_HBDIDLCSR_DEF)
			*lsrio_hbdidlcsr = data;
		else if (*lsrio_hbdidlcsr == data)
			*lsrio_hbdidlcsr = LSRIO_HBDIDLCSR_DEF;

		return 0;
	}

	/* Clear status */
	reg_val = lsrio_apb_readl(priv, LSRIO_RAB_APIO_STAT);
	lsrio_apb_writel(priv, LSRIO_RAB_APIO_STAT, reg_val);

	lsrio_map_maint_window(priv, destid, hopcount,
				round_down(offset, win_size), win_size);

	writel(be32_to_cpu(data), priv->axi_base + priv->ob_win[0].base
				+ (offset % win_size));

	reg_val = lsrio_apb_readl(priv, LSRIO_RAB_APIO_STAT);
	if (reg_val) {
		lsrio_apb_writel(priv, LSRIO_RAB_APIO_STAT, reg_val);
		ls_err(&priv->pdev->dev, "RIO CWrite failed");
		ls_debug(MAINT, &priv->pdev->dev,
			"Maint Write Hop %d id %d off 0x%x, status 0x%x",
			hopcount, destid, offset, reg_val);
		return -EFAULT;
	}

	return 0;
}

/**
 * lsrio_dsend - Send a RapidIO doorbell
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @destid: Destination ID of target device
 * @data: 16-bit info field of RapidIO doorbell
 *
 * Sends a RapidIO doorbell message. Always returns %0.
 */
static int lsrio_dsend(struct rio_mport *mport, int index,
			u16 destid, u16 data)
{
	struct lsrio_device *priv = mport->priv;

	ls_debug(DBELL, &priv->pdev->dev,
		  "Send Doorbell 0x%04x to destID 0x%x", data, destid);

	lsrio_apb_writel(priv, LSRIO_RAB_OBDB_INFO(0), data);
	lsrio_apb_writel(priv, LSRIO_RAB_OBDB_CTRL(0),
			LSRIO_RAB_OBDB_SEND | destid << LSRIO_RAB_OBDB_DID_SHIFT);

	wait_for_completion(&priv->odb_complete);
	if (lsrio_apb_readl(priv, LSRIO_RAB_OBDB_CTRL(0))
		& LSRIO_RAB_OBDB_CTRL_STAT) {
		return -EBUSY;
	}

	return 0;
}

/**
 * lsrio_dbell_handler - Lsrio doorbell interrupt handler
 * @priv: lsrio device-specific data structure
 *
 * Handles inbound doorbell interrupts. Copies doorbell entry from an internal
 * buffer into DB message FIFO and schedules deferred  routine to process
 * queued DBs.
 */
static void lsrio_dbell_handler(struct lsrio_device *priv)
{
	u32 reg_val;

	/* Disable IDB interrupts */
	reg_val = lsrio_apb_readl(priv, LSRIO_RAB_INTR_ENAB_MISC);
	reg_val &= ~LSRIO_RAB_INTR_MISC_IBDR;
	lsrio_apb_writel(priv, LSRIO_RAB_INTR_ENAB_MISC, reg_val);

	schedule_work(&priv->idb_work);
}

static void lsrio_db_dpc(struct work_struct *work)
{
	struct lsrio_device *priv = container_of(work, struct lsrio_device,
						    idb_work);
	struct rio_mport *mport;
	struct rio_dbell *dbell;
	u32 cnt, info, reg_val;
	u16 srcid;
	int found = 0;

	/*
	 * Process queued inbound doorbells
	 */
	mport = &priv->mport;
	cnt = lsrio_apb_readl(priv, LSRIO_RAB_IBDB_CSR) >> LSRIO_RAB_IBDB_NUM_SHIFT;

	while (cnt--) {
		info = lsrio_apb_readl(priv, LSRIO_RAB_IBDB_INFO);
		srcid = info >> LSRIO_RAB_IBDB_SRCID_SHIFT;

		/* Process one doorbell */
		list_for_each_entry(dbell, &mport->dbells, node) {
			if ((dbell->res->start <= srcid) &&
				(dbell->res->end >= srcid)) {
				found = 1;
				break;
			}
		}

		if (found)
			dbell->dinb(mport, dbell->dev_id, srcid,
					mport->host_deviceid, info & 0xffff);
	}

	/* Re-enable IDB interrupts */
	reg_val = lsrio_apb_readl(priv, LSRIO_RAB_INTR_ENAB_MISC);
	reg_val |= LSRIO_RAB_INTR_MISC_IBDR;
	lsrio_apb_writel(priv, LSRIO_RAB_INTR_ENAB_MISC, reg_val);
}

/**
 * lsrio_irqhandler - Lsrio interrupt handler
 * @irq: Linux interrupt number
 * @ptr: Pointer to interrupt-specific data (lsrio_device structure)
 *
 * Handles Lsrio interrupts signaled using INTA. Checks reported
 * interrupt events and calls an event-specific handler(s).
 */
static irqreturn_t lsrio_irqhandler(int irq, void *ptr)
{
	struct lsrio_device *priv = (struct lsrio_device *)ptr;
	u32 gnrl_int, sec_int, reg_val;
	int ch;

	gnrl_int = lsrio_apb_readl(priv, LSRIO_RAB_INTR_STAT_GNRL);
	if (!gnrl_int)
		return IRQ_NONE;

	if (gnrl_int & LSRIO_RAB_INTR_GNRL_APIO) {
		sec_int = lsrio_apb_readl(priv, LSRIO_RAB_INTR_STAT_APIO);
		/* Apio int do not need handle */
		lsrio_apb_writel(priv, LSRIO_RAB_INTR_STAT_APIO, sec_int);
	}
#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	if (gnrl_int & (LSRIO_RAB_INTR_GNRL_WDMA | LSRIO_RAB_INTR_GNRL_RDMA)) {
		u32 ch_mask = 0x10101;
		u32 sum_int = 0;
		int ch = 0;

		sec_int = lsrio_apb_readl(priv, LSRIO_RAB_INTR_STAT_WDMA);
		lsrio_apb_writel(priv, LSRIO_RAB_INTR_STAT_WDMA, sec_int);
		sum_int |= sec_int;

		sec_int = lsrio_apb_readl(priv, LSRIO_RAB_INTR_STAT_RDMA);
		lsrio_apb_writel(priv, LSRIO_RAB_INTR_STAT_RDMA, sec_int);
		sum_int |= sec_int;

		while (sum_int) {
			if (sum_int & ch_mask)
				lsrio_dma_handler(&priv->dma[ch]);

			ch++;
			sum_int &= (~ch_mask);
			sum_int = sum_int >> 1;
		}
	}
#endif
	if (gnrl_int & LSRIO_RAB_INTR_GNRL_IDME) {
		sec_int = lsrio_apb_readl(priv, LSRIO_RAB_INTR_STAT_IDME);
		lsrio_apb_writel(priv, LSRIO_RAB_INTR_STAT_IDME, sec_int);
		reg_val = lsrio_apb_readl(priv, LSRIO_RAB_INTR_ENAB_IDME);
		reg_val &= (~sec_int);
		lsrio_apb_writel(priv, LSRIO_RAB_INTR_ENAB_IDME, reg_val);

		while (sec_int) {
			ch = __ffs(sec_int);
			lsrio_imsg_handler(priv, ch);
			sec_int &= (~BIT(ch));
		}
	}

	if (gnrl_int & LSRIO_RAB_INTR_GNRL_ODME) {
		sec_int = lsrio_apb_readl(priv, LSRIO_RAB_INTR_STAT_ODME);
		lsrio_apb_writel(priv, LSRIO_RAB_INTR_STAT_ODME, sec_int);
		reg_val = lsrio_apb_readl(priv, LSRIO_RAB_INTR_ENAB_ODME);
		reg_val &= (~sec_int);
		lsrio_apb_writel(priv, LSRIO_RAB_INTR_ENAB_ODME, reg_val);

		while (sec_int) {
			ch = __ffs(sec_int);
			lsrio_omsg_handler(priv, ch);
			sec_int &= (~BIT(ch));
		}
	}

	if (gnrl_int & LSRIO_RAB_INTR_GNRL_MISC) {
		sec_int = lsrio_apb_readl(priv, LSRIO_RAB_INTR_STAT_MISC);

		if (sec_int & LSRIO_RAB_INTR_MISC_OBDD)
			complete(&priv->odb_complete);
		if (sec_int & LSRIO_RAB_INTR_MISC_IBDR)
			lsrio_dbell_handler(priv);
		lsrio_apb_writel(priv, LSRIO_RAB_INTR_STAT_MISC, sec_int);
	}

	return IRQ_HANDLED;
}

static int lsrio_request_irq(struct lsrio_device *priv)
{
	int err;

	err = request_irq(priv->pdev->irq, lsrio_irqhandler, IRQF_SHARED,
				DRV_NAME, (void *)priv);

	if (err)
		ls_err(&priv->pdev->dev,
			"Unable to allocate interrupt, err=%d", err);

	return err;
}

static void lsrio_free_irq(struct lsrio_device *priv)
{
	free_irq(priv->pdev->irq, (void *)priv);
}

static int lsrio_map_outb_win(struct rio_mport *mport, u16 destid, u64 rstart,
			u32 size, u32 flags, dma_addr_t *laddr)
{
	struct lsrio_device *priv = mport->priv;
	struct lsrio_axi_res *res = &priv->axi_res;
	struct lsrio_ob_win *ob_win;
	int i;

	ls_debug(OBW, &priv->pdev->dev,
		  "did=%d ra=0x%llx sz=0x%x", destid, rstart, size);

	if ((size & (LSRIO_AMAP_BAR_ALIGN - 1)) ||
		(rstart & (LSRIO_AMAP_BAR_ALIGN - 1)))
		return -EINVAL;

	if (priv->obwin_cnt == 0)
		return -EBUSY;

	if (res->free < size)
		return -ENOMEM;

	for (i = 1; i < LSRIO_OBWIN_NUM; i++) {
		if (!(priv->ob_win[i].active)) {
			ob_win = &priv->ob_win[i];

			ob_win->active = true;
			ob_win->base = res->size - res->free;
			ob_win->size = size;
			ob_win->axi_base = res->base + ob_win->base;
			/* Default set Nread/Nwrite */
			ob_win->type = AMAP_NW;
			ob_win->rstart = rstart;
			ob_win->destid = destid;

			lsrio_map_apio_window(priv, ob_win, i);
			break;
		}
	}

	priv->obwin_cnt--;
	res->free -= size;

	*laddr = ob_win->axi_base;
	return 0;
}

static void lsrio_unmap_outb_win(struct rio_mport *mport,
				  u16 destid, u64 rstart)
{
	struct lsrio_device *priv = mport->priv;
	struct lsrio_ob_win *ob_win;
	int i;

	ls_debug(OBW, &priv->pdev->dev, "did=%d ra=0x%llx", destid, rstart);

	for (i = 1; i < LSRIO_OBWIN_NUM; i++) {
		ob_win = &priv->ob_win[i];

		if (ob_win->active &&
			ob_win->destid == destid && ob_win->rstart == rstart) {
			ls_debug(OBW, &priv->pdev->dev,
				  "free OBW%d @%llx", i, ob_win->base);
			ob_win->active = false;
			lsrio_unmap_apio_window(priv, i);
			priv->obwin_cnt++;
			priv->axi_res.free += ob_win->size;
			break;
		}
	}
}

static void lsrio_close_outb_win(struct lsrio_device *priv)
{
	struct lsrio_ob_win *ob_win;
	int i;

	for (i = 0; i < LSRIO_OBWIN_NUM; i++) {
		ob_win = &priv->ob_win[i];
		ob_win->active = false;
		lsrio_unmap_apio_window(priv, i);
		priv->obwin_cnt++;
		priv->axi_res.free += ob_win->size;
	}
}

/**
 * lsrio_map_inb_mem -- Mapping inbound memory region.
 * @mport: RapidIO master port
 * @lstart: Local memory space start address.
 * @rstart: RapidIO space start address.
 * @size: The mapping region size.
 * @flags: Flags for mapping. 0 for using default flags.
 *
 * Return: 0 -- Success.
 *
 * This function will create the inbound mapping
 * from rstart to lstart.
 */
static int lsrio_map_inb_mem(struct rio_mport *mport, dma_addr_t lstart,
		u64 rstart, u64 size, u32 flags)
{
	struct lsrio_device *priv = mport->priv;
	u32 ibw_id, reg_val;

	/* TODO only accept direct map */
	if (lstart != rstart)
		return -EINVAL;

	ibw_id = (rstart >> 28) & 0xf;

	if (priv->ib_win[ibw_id].ref == 0) {
		reg_val = lsrio_apb_readl(priv, LSRIO_RAB_RPIO_AMAP_LUT(ibw_id));
		reg_val |= LSRIO_RAB_RPIO_AMAP_LUT_EN;
		lsrio_apb_writel(priv, LSRIO_RAB_RPIO_AMAP_LUT(ibw_id), reg_val);
	}

	priv->ib_win[ibw_id].ref++;

	return 0;
}

/**
 * lsrio_unmap_inb_mem -- Unmapping inbound memory region.
 * @mport: RapidIO master port
 * @lstart: Local memory space start address.
 */
static void lsrio_unmap_inb_mem(struct rio_mport *mport,
				dma_addr_t lstart)
{
	struct lsrio_device *priv = mport->priv;
	u32 ibw_id, reg_val;

	ibw_id = (lstart >> 28) & 0xf;

	if (--priv->ib_win[ibw_id].ref == 0) {
		reg_val = lsrio_apb_readl(priv, LSRIO_RAB_RPIO_AMAP_LUT(ibw_id));
		reg_val &= (~LSRIO_RAB_RPIO_AMAP_LUT_EN);
		lsrio_apb_writel(priv, LSRIO_RAB_RPIO_AMAP_LUT(ibw_id), reg_val);
	}
}

static void lsrio_close_inb_win(struct lsrio_device *priv)
{
	int i;
	u32 reg_val;

	for (i = 0; i < LSRIO_IBWIN_NUM; i++) {
		reg_val = lsrio_apb_readl(priv, LSRIO_RAB_RPIO_AMAP_LUT(i));
		reg_val &= (~LSRIO_RAB_RPIO_AMAP_LUT_EN);
		lsrio_apb_writel(priv, LSRIO_RAB_RPIO_AMAP_LUT(i), reg_val);
	}
}

static int lsrio_doorbell_init(struct lsrio_device *priv)
{
	init_completion(&priv->odb_complete);
	/* Initialize Inbound Doorbell processing DPC and queue */
	INIT_WORK(&priv->idb_work, lsrio_db_dpc);

	lsrio_apb_writel(priv, LSRIO_RAB_IBDB_CSR, LSRIO_RAB_IBDB_EN);

	return 0;
}

/* Enable Inbound Messaging interrupts */
static void lsrio_imsg_interrupt_enable(struct lsrio_device *priv, int ch)
{
	u32 reg_val;

	reg_val = lsrio_apb_readl(priv, LSRIO_RAB_INTR_ENAB_IDME);
	reg_val |= LSRIO_RAB_INTR_IDME_CH(ch);
	lsrio_apb_writel(priv, LSRIO_RAB_INTR_ENAB_IDME, reg_val);
}

/* Disable Inbound Messaging interrupts */
static void lsrio_imsg_interrupt_disable(struct lsrio_device *priv, int ch)
{
	u32 reg_val;

	reg_val = lsrio_apb_readl(priv, LSRIO_RAB_INTR_ENAB_IDME);
	reg_val &= (~LSRIO_RAB_INTR_IDME_CH(ch));
	lsrio_apb_writel(priv, LSRIO_RAB_INTR_ENAB_IDME, reg_val);
}

/* Enable Outbound Messaging interrupts */
static void lsrio_omsg_interrupt_enable(struct lsrio_device *priv, int ch)
{
	u32 reg_val;

	reg_val = lsrio_apb_readl(priv, LSRIO_RAB_INTR_ENAB_ODME);
	reg_val |= LSRIO_RAB_INTR_ODME_CH(ch);
	lsrio_apb_writel(priv, LSRIO_RAB_INTR_ENAB_ODME, reg_val);
}

/* Disable Outbound Messaging interrupts */
static void lsrio_omsg_interrupt_disable(struct lsrio_device *priv, int ch)
{
	u32 reg_val;

	reg_val = lsrio_apb_readl(priv, LSRIO_RAB_INTR_ENAB_ODME);
	reg_val &= (~LSRIO_RAB_INTR_ODME_CH(ch));
	lsrio_apb_writel(priv, LSRIO_RAB_INTR_ENAB_ODME, reg_val);
}

/**
 * lsrio_add_outb_message - Add message to the Lsrio outbound message queue
 * @mport: Master port with outbound message queue
 * @rdev: Target of outbound message
 * @mbox: Outbound mailbox
 * @buffer: Message to add to outbound queue
 * @len: Length of message
 */
static int lsrio_add_outb_message(struct rio_mport *mport, struct rio_dev *rdev,
			int mbox, void *buffer, size_t len)
{
	struct lsrio_device *priv = mport->priv;
	struct lsrio_msg_desc *desc;
	u32 w_ptr, r_ptr, reg_val;
	unsigned long flags;

	if (!priv->omsg_init[mbox] ||
	    len > LSRIO_MSG_MAX_SIZE || len < 8)
		return -EINVAL;

	if ((mbox >= 2 && len > LSRIO_SMSG_BUFSIZE))
		return -EINVAL;

	spin_lock_irqsave(&priv->omsg_ring[mbox].lock, flags);

	w_ptr = priv->omsg_ring[mbox].w_ptr;
	r_ptr = priv->omsg_ring[mbox].r_ptr;

	if ((w_ptr == (r_ptr - 1)) ||
		((w_ptr == (priv->omsg_ring[mbox].size - 1)) && (r_ptr == 0)))
		return -ENOMEM;

	/* Copy copy message into transfer buffer */
	memcpy(priv->omsg_ring[mbox].omq_base[w_ptr], buffer, len);

	if (len & 0x7)
		len += 8;

	/* Build descriptor associated with buffer */
	desc = &priv->omsg_ring[mbox].omd_base[w_ptr];

	desc->desc_ctrl = LSRIO_DME_DES_VALID | LSRIO_DME_DES_NDPV |
		LSRIO_DME_DES_INTEN | (rdev->destid << LSRIO_DME_DES_DSTID_SHIFT);
	desc->msg_info = ((mbox << 2) & LSRIO_DME_DES_MBOX_MASK) |
		((len >> 3) << LSRIO_DME_DES_MLEN_SHIFT) | LSRIO_OBDME_DES_SSIZE_256;
	desc->buf_addr = (priv->omsg_ring[mbox].omq_phys[w_ptr] >> 8) &
		LSRIO_DME_DES_ADDRL_MASK;

	/* Go to next descriptor */
	if (++w_ptr == priv->omsg_ring[mbox].size) {
		w_ptr &= priv->omsg_ring[mbox].size - 1;
		desc->nxtd = ((u64)(&priv->omsg_ring[mbox].omd_base[0]) >> 4)
			& LSRIO_DME_NXTDL_MASK;
	} else
		desc->nxtd = (((u64)desc + sizeof(struct lsrio_msg_desc)) >> 4)
			& LSRIO_DME_NXTDL_MASK;

	priv->omsg_ring[mbox].w_ptr = w_ptr;

	wmb();

	/* Wake up engine */
	reg_val = lsrio_apb_readl(priv, LSRIO_RAB_OBDME_CTRL(mbox));
	lsrio_apb_writel(priv, LSRIO_RAB_OBDME_CTRL(mbox),
				reg_val | LSRIO_RAB_OBDME_CTRL_WAKE);
	spin_unlock_irqrestore(&priv->omsg_ring[mbox].lock, flags);

	return 0;
}

/**
 * lsrio_omsg_handler - Outbound Message Interrupt Handler
 * @priv: pointer to lsrio private data
 * @ch:   number of OB MSG channel to service
 *
 * Services channel interrupts from outbound messaging engine.
 */
static void lsrio_omsg_handler(struct lsrio_device *priv, int ch)
{
	struct rio_mport *mport = &priv->mport;
	u32 w_ptr, r_ptr, ring_size, omsg_stat;
	bool do_callback = false;
	bool axi_error = false;

	spin_lock(&priv->omsg_ring[ch].lock);
	omsg_stat = lsrio_apb_readl(priv, LSRIO_RAB_OBDME_STAT(ch));

	w_ptr = priv->omsg_ring[ch].w_ptr;
	r_ptr = priv->omsg_ring[ch].r_ptr;
	ring_size = priv->omsg_ring[ch].size;

	if ((omsg_stat == LSRIO_RAB_OBDME_STAT_SLP) || (r_ptr == w_ptr))
		goto no_sts;

	if (omsg_stat & LSRIO_RAB_OBDME_STAT_AXIE)
		axi_error = true;

	while (r_ptr != w_ptr) {
		struct lsrio_msg_desc *desc;
		u32 desc_ctrl;

		desc = &priv->omsg_ring[ch].omd_base[r_ptr];
		desc_ctrl = desc->desc_ctrl;

		/* Find last descriptor */
		if (lsrio_msg_desc_unhandled(desc))
			break;

		r_ptr = (r_ptr + 1) & (ring_size - 1);

		/* Clear Descriptor */
		memset((void *)desc, 0, sizeof(struct lsrio_msg_desc));

		if (desc_ctrl & LSRIO_DME_DES_AXIE) {
			w_ptr = r_ptr;
			break;
		}
	}

	/* Inform upper layer about transfer completion */
	if (r_ptr == w_ptr)
		do_callback = true;

	/* Update read pointer */
	priv->omsg_ring[ch].r_ptr = r_ptr;

	/* If axi bus error happened, it is necessary to restart engine */
	if (unlikely(axi_error)) {
		priv->omsg_ring[ch].w_ptr = w_ptr;
		/* Disable engine */
		lsrio_apb_writel(priv, LSRIO_RAB_OBDME_CTRL(ch), 0);

		/* Setup Outbound Message descriptor pointer */
		lsrio_apb_writel(priv, LSRIO_RAB_OBDME_DADDR(ch),
					(u32)(priv->omsg_ring[ch].omd_phys +
					w_ptr * sizeof(struct lsrio_msg_desc)) >> 4);

		/* Enable Outbound Message engine */
		lsrio_apb_writel(priv, LSRIO_RAB_OBDME_CTRL(ch),
					LSRIO_RAB_OBDME_CTRL_EN);
	}

no_sts:
	lsrio_apb_writel(priv, LSRIO_RAB_OBDME_STAT(ch), omsg_stat);
	lsrio_omsg_interrupt_enable(priv, ch);
	spin_unlock(&priv->omsg_ring[ch].lock);

	if (mport->outb_msg[ch].mcback && do_callback)
		mport->outb_msg[ch].mcback(mport, priv->omsg_ring[ch].dev_id,
						ch, r_ptr);
}

/**
 * lsrio_open_outb_mbox - Initialize Lsrio outbound mailbox
 * @mport: Master port implementing Outbound Messaging Engine
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the outbound mailbox ring
 */
static int lsrio_open_outb_mbox(struct rio_mport *mport, void *dev_id,
				 int mbox, int entries)
{
	struct lsrio_device *priv = mport->priv;
	int i, buff_size;
	int timeout = 0x1000;
	int rc = 0;
	u32 reg_val;

	/* Mbox 0,1 for multi-segment, Mbox 2,3 for single-segment */
	if ((entries < LSRIO_OMSGD_MIN_RING_SIZE) ||
	    (entries > (LSRIO_OMSGD_RING_SIZE)) ||
	    (!is_power_of_2(entries)) || mbox >= RIO_MAX_MBOX) {
		rc = -EINVAL;
		goto out;
	}

	priv->omsg_ring[mbox].dev_id = dev_id;
	priv->omsg_ring[mbox].size = entries;
	spin_lock_init(&priv->omsg_ring[mbox].lock);

	/*
	 * Outbound Msg Buffer allocation based on
	 * the number of maximum descriptor entries
	 */
	if (mbox < 2)
		buff_size = LSRIO_MMSG_BUFSIZE;
	else
		buff_size = LSRIO_SMSG_BUFSIZE;

	for (i = 0; i < entries; i++) {
		priv->omsg_ring[mbox].omq_base[i] =
			dma_alloc_coherent(
				&priv->pdev->dev, buff_size,
				&priv->omsg_ring[mbox].omq_phys[i],
				GFP_KERNEL);

		if (priv->omsg_ring[mbox].omq_base[i] == NULL) {
			ls_debug(OMSG, &priv->pdev->dev,
				  "ENOMEM for OB_MSG_%d data buffer", mbox);
			rc = -ENOMEM;
			goto out_buf;
		}
	}

	/* Outbound message descriptor allocation */
	priv->omsg_ring[mbox].omd_base = dma_zalloc_coherent(
				&priv->pdev->dev,
				entries * sizeof(struct lsrio_msg_desc),
				&priv->omsg_ring[mbox].omd_phys, GFP_KERNEL);
	if (priv->omsg_ring[mbox].omd_base == NULL) {
		ls_debug(OMSG, &priv->pdev->dev,
			"ENOMEM for OB_MSG_%d descriptor memory", mbox);
		rc = -ENOMEM;
		goto out_buf;
	}

	priv->omsg_ring[mbox].w_ptr = 0;
	priv->omsg_ring[mbox].r_ptr = 0;

	/*
	 * Configure Outbound Messaging Engine
	 */
	do {
		reg_val = lsrio_apb_readl(priv, LSRIO_RAB_OBDME_CTRL(mbox));
		timeout--;
	} while ((reg_val & LSRIO_RAB_OBDME_STAT_TP) && timeout);

	if (timeout <= 0)
		goto out_des;

	/* Setup Outbound Message descriptor pointer */
	lsrio_apb_writel(priv, LSRIO_RAB_OBDME_DADDR(mbox),
				(u32)priv->omsg_ring[mbox].omd_phys >> 4);

	/* Clear Status */
	lsrio_apb_writel(priv, LSRIO_RAB_OBDME_STAT(mbox),
				LSRIO_RAB_OBDME_STAT_CLA);

	/* Enable Outbound Message engine */
	lsrio_apb_writel(priv, LSRIO_RAB_OBDME_CTRL(mbox),
				LSRIO_RAB_OBDME_CTRL_EN);

	/* Enable interrupts */
	lsrio_omsg_interrupt_enable(priv, mbox);

	priv->omsg_init[mbox] = 1;

	return 0;
out_des:
	dma_free_coherent(&priv->pdev->dev,
				entries * sizeof(struct lsrio_msg_desc),
				priv->omsg_ring[mbox].omd_base,
				priv->omsg_ring[mbox].omd_phys);
out_buf:
	for (i = 0; i < priv->omsg_ring[mbox].size; i++) {
		if (priv->omsg_ring[mbox].omq_base[i]) {
			dma_free_coherent(&priv->pdev->dev,
				buff_size + 16,
				priv->omsg_ring[mbox].omq_base[i],
				priv->omsg_ring[mbox].omq_phys[i]);

			priv->omsg_ring[mbox].omq_base[i] = NULL;
		}
	}

out:
	return rc;
}

/**
 * lsrio_close_outb_mbox - Close Lsrio outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @mbox: Mailbox to close
 */
static void lsrio_close_outb_mbox(struct rio_mport *mport, int mbox)
{
	struct lsrio_device *priv = mport->priv;
	u32 i, buff_size;

	if (!priv->omsg_init[mbox])
		return;
	priv->omsg_init[mbox] = 0;

	/* Disable Interrupts */
	lsrio_omsg_interrupt_disable(priv, mbox);

	/* Disale engine */
	lsrio_apb_writel(priv, LSRIO_RAB_OBDME_CTRL(mbox), 0);

	/* Free OMSG descriptors */
	dma_free_coherent(&priv->pdev->dev,
		priv->omsg_ring[mbox].size *
			sizeof(struct lsrio_msg_desc),
		priv->omsg_ring[mbox].omd_base,
		priv->omsg_ring[mbox].omd_phys);

	priv->omsg_ring[mbox].omd_base = NULL;

	if (mbox < 2)
		buff_size = LSRIO_MMSG_BUFSIZE;
	else
		buff_size = LSRIO_SMSG_BUFSIZE;

	/* Free message buffers */
	for (i = 0; i < priv->omsg_ring[mbox].size; i++) {
		if (priv->omsg_ring[mbox].omq_base[i]) {
			dma_free_coherent(&priv->pdev->dev,
				buff_size,
				priv->omsg_ring[mbox].omq_base[i],
				priv->omsg_ring[mbox].omq_phys[i]);

			priv->omsg_ring[mbox].omq_base[i] = NULL;
		}
	}
}

static void lsrio_imsg_desc_refill(struct lsrio_device *priv, int ch,
						u32 s_ptr, int len)
{
	struct lsrio_msg_desc *desc;
	int i;

	for (i = 0; i < len; i++) {
		u32 nxt_ptr = (s_ptr + 1) & (priv->imsg_ring[ch].size - 1);

		desc = &priv->imsg_ring[ch].imd_base[s_ptr];
		desc->desc_ctrl = LSRIO_DME_DES_VALID | LSRIO_DME_DES_NDPV |
			LSRIO_DME_DES_INTEN | LSRIO_IBDME_DES_BSIZE_4K;
		desc->buf_addr = ((u64)(priv->imsg_ring[ch].buf_base +
			s_ptr * LSRIO_MMSG_BUFSIZE) >> 8) & LSRIO_DME_DES_ADDRL_MASK;
		desc->nxtd = ((u64)(&priv->imsg_ring[ch].imd_base[nxt_ptr]) >> 4)
			& LSRIO_DME_NXTDL_MASK;

		s_ptr = nxt_ptr;
	}
}

/**
 * lsrio_imsg_handler - Inbound Message Interrupt Handler
 * @priv: pointer to lsrio private data
 * @ch: inbound message channel number to service
 *
 * Services channel interrupts from inbound messaging engine.
 */
static void lsrio_imsg_handler(struct lsrio_device *priv, int ch)
{
	struct rio_mport *mport = &priv->mport;
	struct lsrio_msg_desc *desc;
	u32 imsg_stat, size, r_ptr;
	bool axi_error = false;
	int i;

	spin_lock(&priv->imsg_ring[ch].lock);

	imsg_stat = lsrio_apb_readl(priv, LSRIO_RAB_IBDME_STAT(ch));
	size = priv->imsg_ring[ch].size;
	r_ptr = priv->imsg_ring[ch].r_ptr;

	if (imsg_stat & LSRIO_RAB_IBDME_STAT_SLP)
		priv->imsg_ring[ch].sleeping = true;
	else if (priv->imsg_ring[ch].sleeping)
		priv->imsg_ring[ch].sleeping = false;

	if (imsg_stat & LSRIO_RAB_IBDME_STAT_AXIE)
		axi_error = true;

	/* Clear IB channel interrupts */
	if (imsg_stat & LSRIO_RAB_IBDME_STAT_DTC) {
		for (i = 0; i < size; i++) {
			desc = &priv->imsg_ring[ch].imd_base[r_ptr];
			if (lsrio_msg_desc_unhandled(desc) ||
				(desc->desc_ctrl & LSRIO_DME_DES_AXIE))
				break;

			desc->desc_ctrl &= (~(LSRIO_DME_DES_VALID | LSRIO_DME_DES_NDPV));
			r_ptr = (r_ptr + 1) & (size - 1);
		}

		priv->imsg_ring[ch].r_ptr = r_ptr;

		/* If an IB Msg is received notify the upper layer */
		if (mport->inb_msg[ch].mcback)
			mport->inb_msg[ch].mcback(mport,
				priv->imsg_ring[ch].dev_id, ch, -1);
	}

	if (axi_error) {
		/* Disable engine */
		lsrio_apb_writel(priv, LSRIO_RAB_IBDME_CTRL(ch), 0);

		lsrio_imsg_desc_refill(priv, ch, 0, size);

		/* Setup Outbound Message descriptor pointer */
		lsrio_apb_writel(priv, LSRIO_RAB_IBDME_DADDR(ch),
					(u32)(priv->omsg_ring[ch].omd_phys) >> 4);

		/* Enable Outbound Message engine */
		lsrio_apb_writel(priv, LSRIO_RAB_IBDME_CTRL(ch),
			LSRIO_RAB_IBDME_CTRL_EN | (ch << LSRIO_RAB_IBDME_CTRL_MBOX_SHIFT));
	}

	lsrio_apb_writel(priv, LSRIO_RAB_IBDME_STAT(ch), imsg_stat);
	lsrio_imsg_interrupt_enable(priv, ch);
	spin_unlock(&priv->imsg_ring[ch].lock);
}

/**
 * lsrio_open_inb_mbox - Initialize Lsrio inbound mailbox
 * @mport: Master port implementing the Inbound Messaging Engine
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the inbound mailbox ring
 */
static int lsrio_open_inb_mbox(struct rio_mport *mport, void *dev_id,
				int mbox, int entries)
{
	struct lsrio_device *priv = mport->priv;
	int timeout = 0x1000;
	int rc = 0;
	int i;
	u32 reg_val;

	if ((entries < LSRIO_IMSGD_MIN_RING_SIZE) ||
	    (entries > LSRIO_IMSGD_RING_SIZE) ||
	    (!is_power_of_2(entries)) || mbox >= RIO_MAX_MBOX) {
		rc = -EINVAL;
		goto out;
	}

	/* Initialize IB Messaging Ring */
	priv->imsg_ring[mbox].dev_id = dev_id;
	priv->imsg_ring[mbox].size = entries;
	priv->imsg_ring[mbox].rx_slot = 0;
	priv->imsg_ring[mbox].r_ptr = 0;
	/* Sleeping interrupt have a lag, so we need enforce wake up once */
	priv->imsg_ring[mbox].sleeping = true;
	for (i = 0; i < priv->imsg_ring[mbox].size; i++)
		priv->imsg_ring[mbox].imq_base[i] = NULL;
	spin_lock_init(&priv->imsg_ring[mbox].lock);

	/* Engine 0 ~ 3 surrport multi-segemnt */
	priv->imsg_ring[mbox].buf_base =
		dma_alloc_coherent(&priv->pdev->dev,
				   entries * LSRIO_MMSG_BUFSIZE,
				   &priv->imsg_ring[mbox].buf_phys,
				   GFP_KERNEL);

	if (priv->imsg_ring[mbox].buf_base == NULL) {
		ls_err(&priv->pdev->dev,
			"Failed to allocate buffers for IB MBOX%d", mbox);
		rc = -ENOMEM;
		goto out;
	}

	/* Allocate memory for Inbound message descriptors */
	priv->imsg_ring[mbox].imd_base =
		dma_alloc_coherent(&priv->pdev->dev,
				   entries * sizeof(struct lsrio_msg_desc),
				   &priv->imsg_ring[mbox].imd_phys, GFP_KERNEL);

	if (priv->imsg_ring[mbox].imd_base == NULL) {
		ls_err(&priv->pdev->dev,
			"Failed to allocate descriptor memory for IB MBOX%d",
			mbox);
		rc = -ENOMEM;
		goto out_buf;
	}

	wmb();

	/*
	 * Configure Inbound Messaging channel
	 */
	/* Initialize Inbound Message Engine */
	do {
		reg_val = lsrio_apb_readl(priv, LSRIO_RAB_IBDME_CTRL(mbox));
		timeout--;
	} while ((reg_val & LSRIO_RAB_IBDME_STAT_TP) && timeout);

	if (timeout <= 0)
		goto out_des;

	/* Setup Inbound Message descriptor pointer */
	lsrio_apb_writel(priv, LSRIO_RAB_IBDME_DADDR(mbox),
		(u32)priv->imsg_ring[mbox].imd_phys >> 4);

	/* Clear Status */
	lsrio_apb_writel(priv, LSRIO_RAB_IBDME_STAT(mbox),
				LSRIO_RAB_IBDME_STAT_CLA);

	/* Enable Inbound Message engine */
	lsrio_apb_writel(priv, LSRIO_RAB_IBDME_CTRL(mbox),
		LSRIO_RAB_IBDME_CTRL_EN | (mbox << LSRIO_RAB_IBDME_CTRL_MBOX_SHIFT));

	/* Enable interrupts */
	lsrio_imsg_interrupt_enable(priv, mbox);

	priv->imsg_init[mbox] = 1;
	return 0;

out_des:
	dma_free_coherent(&priv->pdev->dev,
		entries * sizeof(struct lsrio_msg_desc),
		priv->imsg_ring[mbox].imd_base,
		priv->imsg_ring[mbox].imd_phys);

out_buf:
	dma_free_coherent(&priv->pdev->dev,
		priv->imsg_ring[mbox].size * LSRIO_MSG_BUFFER_SIZE,
		priv->imsg_ring[mbox].buf_base,
		priv->imsg_ring[mbox].buf_phys);

	priv->imsg_ring[mbox].buf_base = NULL;

out:
	return rc;
}

/**
 * lsrio_close_inb_mbox - Shut down Lsrio inbound mailbox
 * @mport: Master port implementing the Inbound Messaging Engine
 * @mbox: Mailbox to close
 */
static void lsrio_close_inb_mbox(struct rio_mport *mport, int mbox)
{
	struct lsrio_device *priv = mport->priv;
	u32 rx_slot;

	if (!priv->imsg_init[mbox]) /* mbox isn't initialized yet */
		return;
	priv->imsg_init[mbox] = 0;

	/* Disable Inbound Messaging Engine */
	lsrio_apb_writel(priv, LSRIO_RAB_IBDME_CTRL(mbox), 0);

	/* Disable Interrupts */
	lsrio_imsg_interrupt_disable(priv, mbox);

	/* Clear Inbound Buffer Queue */
	for (rx_slot = 0; rx_slot < priv->imsg_ring[mbox].size; rx_slot++)
		priv->imsg_ring[mbox].imq_base[rx_slot] = NULL;

	/* Free memory allocated for message buffers */
	dma_free_coherent(&priv->pdev->dev,
		priv->imsg_ring[mbox].size * LSRIO_MMSG_BUFSIZE,
		priv->imsg_ring[mbox].buf_base,
		priv->imsg_ring[mbox].buf_phys);

	priv->imsg_ring[mbox].buf_base = NULL;

	/* Free memory allocated for RX descriptors */
	dma_free_coherent(&priv->pdev->dev,
		priv->imsg_ring[mbox].size * sizeof(struct lsrio_msg_desc),
		priv->imsg_ring[mbox].imd_base,
		priv->imsg_ring[mbox].imd_phys);

	priv->imsg_ring[mbox].imd_base = NULL;
}

/**
 * lsrio_add_inb_buffer - Add buffer to the Lsrio inbound message queue
 * @mport: Master port implementing the Inbound Messaging Engine
 * @mbox: Inbound mailbox number
 * @buf: Buffer to add to inbound queue
 */
static int lsrio_add_inb_buffer(struct rio_mport *mport, int mbox, void *buf)
{
	struct lsrio_device *priv = mport->priv;
	u32 rx_slot;
	int rc = 0;

	rx_slot = priv->imsg_ring[mbox].rx_slot;
	if (priv->imsg_ring[mbox].imq_base[rx_slot]) {
		ls_err(&priv->pdev->dev,
			"Error adding inbound buffer %d, buffer exists",
			rx_slot);
		rc = -EINVAL;
		goto out;
	}

	priv->imsg_ring[mbox].imq_base[rx_slot] = buf;

	lsrio_imsg_desc_refill(priv, mbox, rx_slot, 1);

	if (priv->imsg_ring[mbox].sleeping) {
		u32 reg_val;

		reg_val = lsrio_apb_readl(priv, LSRIO_RAB_IBDME_CTRL(mbox));
		lsrio_apb_writel(priv, LSRIO_RAB_IBDME_CTRL(mbox),
			reg_val | LSRIO_RAB_IBDME_CTRL_WAKE);

		reg_val = lsrio_apb_readl(priv, LSRIO_RAB_IBDME_STAT(mbox));
		if ((reg_val & LSRIO_RAB_IBDME_STAT_SLP) == 0)
			priv->imsg_ring[mbox].sleeping = false;
		else
			ls_err(&priv->pdev->dev, "Wake up inbound message engine failed!\n");
	}

	if (++priv->imsg_ring[mbox].rx_slot == priv->imsg_ring[mbox].size)
		priv->imsg_ring[mbox].rx_slot = 0;

out:
	return rc;
}

/**
 * lsrio_get_inb_message - Fetch inbound message from the Lsrio MSG Queue
 * @mport: Master port implementing the Inbound Messaging Engine
 * @mbox: Inbound mailbox number
 *
 * Returns pointer to the message on success or NULL on failure.
 */
static void *lsrio_get_inb_message(struct rio_mport *mport, int mbox)
{
	struct lsrio_device *priv = mport->priv;
	struct lsrio_msg_desc *desc;
	u32 rx_slot;
	void *rx_virt = NULL;
	void *buf = NULL;
	int msg_size;

	if (!priv->imsg_init[mbox])
		return NULL;

	rx_slot = priv->imsg_ring[mbox].rx_slot;
	desc = &priv->imsg_ring[mbox].imd_base[rx_slot];

	if (lsrio_msg_desc_unhandled(desc))
		goto out;

	while (priv->imsg_ring[mbox].imq_base[rx_slot] == NULL)
		rx_slot = (rx_slot + 1) & (priv->imsg_ring[mbox].size - 1);

	buf = priv->imsg_ring[mbox].imq_base[rx_slot];

	if ((desc->desc_ctrl & LSRIO_DME_DES_DONE) == 0) {
		ls_info(&priv->pdev->dev, "Msg receive failed!\n");
		ls_debug(IMSG, &priv->pdev->dev,
			"Desc word0 is 0x%x\n", desc->desc_ctrl);
		goto out;
	}

	rx_virt = priv->imsg_ring[mbox].buf_base + rx_slot * LSRIO_MMSG_BUFSIZE;
	msg_size = ((desc->msg_info & LSRIO_DME_DES_MLEN_MASK)
				>> LSRIO_DME_DES_MLEN_SHIFT) << 3;

	memcpy(buf, rx_virt, msg_size);

	priv->imsg_ring[mbox].imq_base[rx_slot] = NULL;
out:
	return buf;
}

/**
 * lsrio_messages_init - Initialization of Messaging Engine
 * @priv: pointer to lsrio private data
 *
 * Configures Lsrio messaging engine.
 */
static int lsrio_messages_init(struct lsrio_device *priv)
{
	return 0;
}

static int lsrio_query_mport(struct rio_mport *mport,
			      struct rio_mport_attr *attr)
{
	struct lsrio_device *priv = mport->priv;
	u32 reg_val;

	reg_val = lsrio_apb_readl(priv, 0x100 + RIO_PORT_N_ERR_STS_CSR(0, 0));

	if (reg_val & RIO_PORT_N_ERR_STS_PORT_OK) {
		reg_val = lsrio_apb_readl(priv, 0x100 + RIO_PORT_N_CTL2_CSR(0, 0));
		/* BDE already set */
		if (reg_val & BIT(26))
			attr->link_speed = (reg_val & RIO_PORT_N_CTL2_SEL_BAUD) >> 28;
		reg_val = lsrio_apb_readl(priv, 0x100 + RIO_PORT_N_CTL_CSR(0, 0));
		attr->link_width = (reg_val & RIO_PORT_N_CTL_IPW) >> 27;
	} else
		attr->link_speed = RIO_LINK_DOWN;

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	attr->flags = RIO_MPORT_DMA | RIO_MPORT_DMA_SG;
	attr->dma_max_sge = 64;
	attr->dma_max_size = LSRIO_DMA_MAX_BCOUNT;
	attr->dma_align = 16;
#else
	attr->flags = 0;
#endif
	return 0;
}

/**
 * lsrio_amap_init - init apio and rpio window
 * @priv: pointer to lsrio private data
 */
static void lsrio_amap_init(struct lsrio_device *priv)
{
	int i;

	lsrio_apb_writel(priv, LSRIO_RAB_CTRL, LSRIO_RAB_CTRL_DEF);

	lsrio_apb_writel(priv, LSRIO_RAB_APIO_CTRL,
			LSRIO_APIO_PIO_EN | LSRIO_APIO_MEM_EN | LSRIO_APIO_MAINT_EN);
	lsrio_apb_writel(priv, LSRIO_RAB_RPIO_CTRL, LSRIO_RPIO_PIO_EN);
	lsrio_apb_writel(priv, LSRIO_RAB_AMST_CACHE, LSRIO_RAB_AMST_CACHE_EN);

	/* Win 0 always for maintanence */
	priv->ob_win[0].base = 0;
	priv->ob_win[0].size = LSRIO_MAINT_WIN_SIZE;
	priv->ob_win[0].type = AMAP_MAINT;
	priv->ob_win[0].axi_base = priv->axi_res.base;
	priv->ob_win[0].active = true;
	priv->axi_res.free -= LSRIO_MAINT_WIN_SIZE;

	for (i = 1; i < LSRIO_OBWIN_NUM; i++)
		priv->ob_win[i].active = false;

	priv->obwin_cnt = LSRIO_OBWIN_NUM - 1;

	/* IDSL set bit[31:28] */
	lsrio_apb_writel(priv, LSRIO_RAB_RPIO_AMAP_IDSL, 0x4);
	lsrio_apb_writel(priv, LSRIO_RAB_RPIO_AMAP_BYPS,
				LSRIO_RAB_RPIO_AMAP_BYPS_RAS);

	/* Set inbound windows direct map for 32 bit address */
	for (i = 0; i < LSRIO_IBWIN_NUM; i++) {
		lsrio_apb_writel(priv, LSRIO_RAB_RPIO_AMAP_LUT(i),
				LSRIO_RAB_RPIO_AMAP_LUT_256 | (i << 22));
		priv->ib_win[i].ref = 0;
	}
}

static void lsrio_amap_remove(struct lsrio_device *priv)
{
	lsrio_apb_writel(priv, LSRIO_RAB_CTRL, 0);

	lsrio_apb_writel(priv, LSRIO_RAB_APIO_CTRL, 0);
	lsrio_apb_writel(priv, LSRIO_RAB_RPIO_CTRL, 0);
	lsrio_apb_writel(priv, LSRIO_RAB_AMST_CACHE, 0);

	lsrio_close_outb_win(priv);
	lsrio_close_inb_win(priv);
}

static struct rio_ops lsrio_rio_ops = {
	.lcread			= lsrio_lcread,
	.lcwrite		= lsrio_lcwrite,
	.cread			= lsrio_cread,
	.cwrite			= lsrio_cwrite,
	.dsend			= lsrio_dsend,
	.open_inb_mbox		= lsrio_open_inb_mbox,
	.close_inb_mbox		= lsrio_close_inb_mbox,
	.open_outb_mbox		= lsrio_open_outb_mbox,
	.close_outb_mbox	= lsrio_close_outb_mbox,
	.add_outb_message	= lsrio_add_outb_message,
	.add_inb_buffer		= lsrio_add_inb_buffer,
	.get_inb_message	= lsrio_get_inb_message,
	.map_inb		= lsrio_map_inb_mem,
	.unmap_inb		= lsrio_unmap_inb_mem,
	.query_mport	= lsrio_query_mport,
	.map_outb		= lsrio_map_outb_win,
	.unmap_outb		= lsrio_unmap_outb_win,
};

static void lsrio_mport_release(struct device *dev)
{
	struct rio_mport *mport = to_rio_mport(dev);

	ls_debug(EXIT, dev, "%s id=%d", mport->name, mport->id);
}

/**
 * lsrio_setup_mport - Setup Lsrio as RapidIO subsystem master port
 * @priv: pointer to lsrio private data
 *
 * Configures Lsrio as RapidIO master port.
 */
static int lsrio_setup_mport(struct lsrio_device *priv)
{
	struct pci_dev *pdev = priv->pdev;
	int err = 0;
	struct rio_mport *mport = &priv->mport;

	err = rio_mport_initialize(mport);
	if (err)
		return err;

	mport->ops = &lsrio_rio_ops;
	mport->index = 0;
	mport->sys_size = 0; /* small system */
	mport->priv = (void *)priv;
	mport->phys_efptr = 0x100;
	mport->phys_rmap = 1;
	mport->dev.parent = &pdev->dev;
	mport->dev.release = lsrio_mport_release;

	INIT_LIST_HEAD(&mport->dbells);
	rio_init_dbell_res(&mport->riores[RIO_DOORBELL_RESOURCE], 0, 0xffff);
	rio_init_mbox_res(&mport->riores[RIO_INB_MBOX_RESOURCE], 0, 3);
	rio_init_mbox_res(&mport->riores[RIO_OUTB_MBOX_RESOURCE], 0, 3);
	snprintf(mport->name, RIO_MAX_MPORT_NAME, "%s(%s)",
		 dev_driver_string(&pdev->dev), dev_name(&pdev->dev));

	err = lsrio_request_irq(priv);
	if (err)
		return err;

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	err = lsrio_register_dma(priv);
	if (err)
		goto err_exit;
#endif

	err = rio_register_mport(mport);
	if (err) {
#ifdef CONFIG_RAPIDIO_DMA_ENGINE
		lsrio_unregister_dma(priv);
#endif
		goto err_exit;
	}

	return 0;

err_exit:
	lsrio_free_irq(priv);
	return err;
}

static int lsrio_probe(struct pci_dev *pdev,
				  const struct pci_device_id *id)
{
	struct lsrio_device *priv;
	int err, i;

	priv = kzalloc(sizeof(struct lsrio_device), GFP_KERNEL);
	if (!priv) {
		err = -ENOMEM;
		goto err_exit;
	}

	err = pci_enable_device(pdev);
	if (err) {
		ls_err(&pdev->dev, "Failed to enable PCI device");
		goto err_clean;
	}

	priv->pdev = pdev;

	err = pci_request_regions(pdev, DRV_NAME);
	if (err) {
		ls_err(&pdev->dev, "Unable to obtain PCI resources");
		goto err_disable_pdev;
	}

	pci_set_master(pdev);
	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));

	priv->axi_res.base = (u64)pci_resource_start(pdev, BAR_0);
	priv->axi_res.size = pci_resource_len(pdev, BAR_0);
	priv->axi_res.free = priv->axi_res.size;

	priv->axi_base = pci_ioremap_bar(pdev, BAR_0);
	if (!priv->axi_base) {
		ls_err(&pdev->dev, "Unable to map axi registers space");
		err = -ENOMEM;
		goto err_free_res;
	}

	priv->apb_base = pci_ioremap_bar(pdev, BAR_2);
	if (!priv->apb_base) {
		ls_err(&pdev->dev, "Unable to map apb registers space");
		err = -ENOMEM;
		goto err_unmap_bars;
	}

	spin_lock_init(&priv->apb_lock);
	lsrio_disable_ints(priv);

	/* Fixup hbdidlcsr */
	for (i = 0; i < 32; i++)
		lsrio_hbdidlcsr_hop[i] = LSRIO_HBDIDLCSR_DEF;

	lsrio_amap_init(priv);
	lsrio_doorbell_init(priv);
	lsrio_messages_init(priv);

	err = lsrio_setup_mport(priv);
	if (err)
		goto err_unmap_bars;

	pci_set_drvdata(pdev, priv);

	lsrio_enable_ints(priv);

	return 0;

err_unmap_bars:
	if (priv->apb_base)
		iounmap(priv->apb_base);
	if (priv->apb_base)
		iounmap(priv->apb_base);

err_free_res:
	pci_release_regions(pdev);
	pci_clear_master(pdev);
err_disable_pdev:
	pci_disable_device(pdev);
err_clean:
	kfree(priv);
err_exit:
	return err;
}

static void lsrio_remove(struct pci_dev *pdev)
{
	struct lsrio_device *priv = pci_get_drvdata(pdev);

	ls_debug(EXIT, &pdev->dev, "enter");

	lsrio_disable_ints(priv);
	lsrio_free_irq(priv);
	flush_scheduled_work();
	rio_unregister_mport(&priv->mport);

	lsrio_amap_remove(priv);
#ifdef CONFIG_RAPIDIO_DMA_ENGINE
	lsrio_unregister_dma(priv);
#endif

	if (priv->axi_base)
		iounmap(priv->axi_base);
	if (priv->apb_base)
		iounmap(priv->apb_base);

	pci_release_regions(pdev);
	pci_clear_master(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
	kfree(priv);
	ls_debug(EXIT, &pdev->dev, "exit");
}

static void lsrio_shutdown(struct pci_dev *pdev)
{
	struct lsrio_device *priv = pci_get_drvdata(pdev);

	ls_debug(EXIT, &pdev->dev, "enter");

	lsrio_disable_ints(priv);
	lsrio_dma_stop_all(priv);
	pci_clear_master(pdev);
	pci_disable_device(pdev);
}

static const struct pci_device_id lsrio_pci_tbl[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_LOONGSON_RIO) },
	{ 0, }	/* terminate list */
};

MODULE_DEVICE_TABLE(pci, lsrio_pci_tbl);

static struct pci_driver lsrio_driver = {
	.name		= "lsrio",
	.id_table	= lsrio_pci_tbl,
	.probe		= lsrio_probe,
	.remove		= lsrio_remove,
	.shutdown	= lsrio_shutdown,
};

module_pci_driver(lsrio_driver);

MODULE_DESCRIPTION("Loongson Rapid IO driver");
MODULE_AUTHOR("Loongson Technology, Inc.");
MODULE_LICENSE("GPL");
