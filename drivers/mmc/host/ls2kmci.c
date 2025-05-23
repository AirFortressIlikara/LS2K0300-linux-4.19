/*
 *  linux/drivers/mmc/ls2k_mci.h - Loongson ls2k MCI driver
 *
 *  Copyright (C) loongson.
 *
 * Current driver maintained by Ben Dooks and Simtec Electronics
 *  Copyright (C) 2008 Simtec Electronics <ben-linux@fluff.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/mmc/host.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/dmaengine.h>

#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/of_dma.h>
#include <linux/bitrev.h>
#include <linux/pci.h>
#include <linux/of_irq.h>

#include "ls2kmci.h"
#include <linux/gpio.h>

#define DRIVER_NAME "ls2k_sdio"
#define DEF_CLKRATE 125000000

enum dbg_channels {
	dbg_err   = (1 << 0),
	dbg_debug = (1 << 1),
	dbg_info  = (1 << 2),
	dbg_irq   = (1 << 3),
	dbg_sg    = (1 << 4),
	dbg_dma   = (1 << 5),
	dbg_pio   = (1 << 6),
	dbg_fail  = (1 << 7),
	dbg_conf  = (1 << 8),
};

struct mmc_gpio {
	struct gpio_desc *ro_gpio;
	struct gpio_desc *cd_gpio;
	bool override_ro_active_level;
	bool override_cd_active_level;
	irqreturn_t (*cd_gpio_isr)(int irq, void *dev_id);
	char *ro_label;
	u32 cd_debounce_delay_ms;
	char cd_label[];
};

static const int dbgmap_err   = dbg_fail | dbg_err;
static const int dbgmap_info  = dbg_info | dbg_conf;
static const int dbgmap_debug = dbg_err | dbg_debug;

/* Used to fix the problem that DMA copies are small in length. */
static u64 dma_buf;

static void __iomem *cd_gpio_inten_base;
#define dbg(host, channels, args...)		  \
	do {					  \
	if (dbgmap_err & channels) 		  \
		dev_err(&host->pdev->dev, args);  \
	else if (dbgmap_info & channels)	  \
		dev_info(&host->pdev->dev, args); \
	else if (dbgmap_debug & channels)	  \
		dev_dbg(&host->pdev->dev, args);  \
	} while (0)

static void finalize_request(struct ls2k_mci_host *host);
static void ls2k_mci_send_request(struct mmc_host *mmc);

#ifdef CONFIG_MMC_DEBUG

static void dbg_dumpregs(struct ls2k_mci_host *host, char *prefix)
{
	u32 con, pre, cmdarg, cmdcon, cmdsta, r0, r1, r2, r3, timer, bsize;
	u32 datcon, datcnt, datsta, fsta, imask;

	con 	= readl(host->base + SDICON);
	pre 	= readl(host->base + SDIPRE);
	cmdarg 	= readl(host->base + SDICMDARG);
	cmdcon 	= readl(host->base + SDICMDCON);
	cmdsta 	= readl(host->base + SDICMDSTAT);
	r0 	= readl(host->base + SDIRSP0);
	r1 	= readl(host->base + SDIRSP1);
	r2 	= readl(host->base + SDIRSP2);
	r3 	= readl(host->base + SDIRSP3);
	timer 	= readl(host->base + SDITIMER);
	bsize 	= readl(host->base + SDIBSIZE);
	datcon 	= readl(host->base + SDIDCON);
	datcnt 	= readl(host->base + SDIDCNT);
	datsta 	= readl(host->base + SDIDSTA);
	fsta 	= readl(host->base + SDIFSTA);
	imask   = readl(host->base + host->sdiimsk);

	dbg(host, dbg_debug, "%s  CON:[%08x]  PRE:[%08x]  TMR:[%08x]\n",
				prefix, con, pre, timer);

	dbg(host, dbg_debug, "%s CCON:[%08x] CARG:[%08x] CSTA:[%08x]\n",
				prefix, cmdcon, cmdarg, cmdsta);

	dbg(host, dbg_debug, "%s DCON:[%08x] FSTA:[%08x]"
			       " DSTA:[%08x] DCNT:[%08x]\n",
				prefix, datcon, fsta, datsta, datcnt);

	dbg(host, dbg_debug, "%s   R0:[%08x]   R1:[%08x]"
			       "   R2:[%08x]   R3:[%08x]\n",
				prefix, r0, r1, r2, r3);
}

static void prepare_dbgmsg(struct ls2k_mci_host *host, struct mmc_command *cmd,
			   int stop)
{
	snprintf(host->dbgmsg_cmd, 300,
		 "#%u%s op:%i arg:0x%08x flags:0x08%x retries:%u",
		 host->ccnt, (stop ? " (STOP)" : ""),
		 cmd->opcode, cmd->arg, cmd->flags, cmd->retries);

	if (cmd->data) {
		snprintf(host->dbgmsg_dat, 300,
			 "#%u bsize:%u blocks:%u bytes:%u",
			 host->dcnt, cmd->data->blksz,
			 cmd->data->blocks,
			 cmd->data->blocks * cmd->data->blksz);
	} else {
		host->dbgmsg_dat[0] = '\0';
	}
}

static void dbg_dumpcmd(struct ls2k_mci_host *host, struct mmc_command *cmd,
			int fail)
{
	unsigned int dbglvl = fail ? dbg_fail : dbg_debug;

	if (!cmd)
		return;

	if (cmd->error == 0) {
		dbg(host, dbglvl, "CMD[OK] %s R0:0x%08x\n",
			host->dbgmsg_cmd, cmd->resp[0]);
	} else {
		dbg(host, dbglvl, "CMD[ERR %i] %s Status:%s\n",
			cmd->error, host->dbgmsg_cmd, host->status);
	}

	if (!cmd->data)
		return;

	if (cmd->data->error == 0) {
		dbg(host, dbglvl, "DAT[OK] %s\n", host->dbgmsg_dat);
	} else {
		dbg(host, dbglvl, "DAT[ERR %i] %s DCNT:0x%08x\n",
			cmd->data->error, host->dbgmsg_dat,
			readl(host->base + SDIDCNT));
	}
}
#else
#define dbg_dumpcmd	NULL
#define prepare_dbgmsg	NULL
#define dbg_dumpregs	NULL
#endif /* CONFIG_MMC_DEBUG */

#define CARD_DETECT_IRQ		(64+12) /* gpio12 */

/*
 * ls2k_mci_enable_irq - enable IRQ, after having disabled it.
 * @host: The device state.
 * @more: True if more IRQs are expected from transfer.
 *
 * Enable the main IRQ if needed after it has been disabled.
 *
 * The IRQ can be one of the following states:
 *	- disabled during IDLE
 *	- disabled whilst processing data
 *	- enabled during transfer
 *	- enabled whilst awaiting SDIO interrupt detection
 */
static void ls2k_mci_enable_irq(struct ls2k_mci_host *host, bool more)
{
	unsigned long flags;
	bool enable = false;

	local_irq_save(flags);

	host->irq_enabled = more;
	host->irq_disabled = false;

	enable = more | host->sdio_irqen;

	if (host->irq_state != enable) {
		host->irq_state = enable;

		if (enable)
			enable_irq(host->irq);
		else
			disable_irq(host->irq);
	}

	local_irq_restore(flags);
}


static inline bool ls2k_mci_host_usedma(struct ls2k_mci_host *host)
{
	return true;
}

static void ls2k_mci_disable_irq(struct ls2k_mci_host *host, bool transfer)
{
	unsigned long flags;

	local_irq_save(flags);
	host->irq_disabled = transfer;

	if (transfer && host->irq_state) {
		host->irq_state = false;
		disable_irq(host->irq);
	}
	local_irq_restore(flags);
}

static inline void clear_imask(struct ls2k_mci_host *host)
{
	u32 mask = readl(host->base + host->sdiimsk);

	/* preserve the SDIO IRQ mask state */
	writel(mask, host->base + host->sdiimsk);
}

static void finalize_request(struct ls2k_mci_host *host)
{
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd;
	int debug_as_failure = 0;
	if (host->complete_what != COMPLETION_FINALIZE)
		return;

	if (!mrq)
		return;
	cmd = host->cmd_is_stop ? mrq->stop : mrq->cmd;

	if (cmd->data && (cmd->error == 0) &&
	    (cmd->data->error == 0)) {
		if (ls2k_mci_host_usedma(host) && (!host->dma_complete)) {
			dbg(host, dbg_dma, "DMA Missing (%d)!\n",
			    host->dma_complete);
			return;
		}
	}
	/* Read response from controller. */
	cmd->resp[0] = readl(host->base + SDIRSP0);
	cmd->resp[1] = readl(host->base + SDIRSP1);
	cmd->resp[2] = readl(host->base + SDIRSP2);
	cmd->resp[3] = readl(host->base + SDIRSP3);

	if (cmd->error)
		debug_as_failure = 1;

	if (cmd->data && cmd->data->error)
		debug_as_failure = 1;

	/* Cleanup controller */
	writel(0, host->base + SDICMDARG);
	writel(0, host->base + SDICMDCON);
	clear_imask(host);

	if (cmd->data && cmd->error)
		cmd->data->error = cmd->error;

	if (cmd->data && cmd->data->stop && (!host->cmd_is_stop)) {
		host->cmd_is_stop = 1;
		ls2k_mci_send_request(host->mmc);
		return;
	}

	/* If we have no data transfer we are finished here */
	if (!mrq->data)
		goto request_done;

	/* Calculate the amout of bytes transfer if there was no error */
	if (mrq->data->error == 0) {
		mrq->data->bytes_xfered =
			(mrq->data->blocks * mrq->data->blksz);
	} else {
		mrq->data->bytes_xfered = 0;
	}

	/* If we had an error while transferring data we flush the
	 * DMA channel and the fifo to clear out any garbage.
	 */
	if (mrq->data->error != 0) {
		if (ls2k_mci_host_usedma(host))
			printk(" dbg-ms: ls2k_mci dma data error!\r\n");
	}

request_done:
	host->complete_what = COMPLETION_NONE;
	host->mrq = NULL;
	mmc_request_done(host->mmc, mrq);
}

static void pio_tasklet(unsigned long data)
{
	struct ls2k_mci_host *host = (struct ls2k_mci_host *) data;

	ls2k_mci_disable_irq(host, true);

	if (host->complete_what == COMPLETION_FINALIZE) {
		clear_imask(host);

		ls2k_mci_enable_irq(host, false);
		finalize_request(host);
	} else
		ls2k_mci_enable_irq(host, true);
}

static void ls2k_cmd_data_fix(struct ls2k_mci_host *host,
						struct mmc_command *cmd)
{
	struct mmc_data *mdata = cmd->data;
	int i, j;
	u32 *data;

	if (host->pdata->version == LOONGSON_SDIO_EMMC_VER_1_2) {
		if (cmd->opcode != SD_SWITCH)
			return;
	} else {
		if (host->app_cmd != SD_APP_SEND_SCR &&
			host->app_cmd != SD_APP_SEND_NUM_WR_BLKS &&
			host->app_cmd != SD_APP_SD_STATUS &&
			cmd->opcode != MMC_SEND_WRITE_PROT &&
			cmd->opcode != SD_SWITCH)
			return;
	}
	if (!(cmd->flags & MMC_CMD_ADTC))
		return;

	for (i = 0; i < mdata->sg_len; i++) {
		data = sg_virt(&mdata->sg[i]);
		for (j = 0; j < (sg_dma_len(&mdata->sg[i]) / 4); j++)
			if (cmd->opcode == SD_SWITCH)
				data[j] = (bitrev8x4(data[j]));
			else
				data[j] = cpu_to_be32(data[j]);
	}
}

static irqreturn_t ls2k_mci_irq(int irq, void *dev_id)
{
	struct ls2k_mci_host *host = dev_id;
	struct mmc_command *cmd;
	u32 mci_csta, mci_dsta, mci_imsk;

	/* Used to fix the problem that DMA copies are small in length. */
	u64 sg_dma_buf;
	u32 dma_length = 0x0;
	int i, j;

	mci_csta = readl(host->base + SDICMDSTA);
	mci_imsk = readl(host->base + SDIINTMSK);
	mci_dsta = readl(host->base + SDIDSTA);

	if ((host->complete_what == COMPLETION_NONE) ||
	    (host->complete_what == COMPLETION_FINALIZE)) {
		host->status = "nothing to complete";
		clear_imask(host);
		goto irq_out;
	}
	if (!host->mrq) {
		host->status = "no active mrq";
		clear_imask(host);
		goto irq_out;
	}

	cmd = host->cmd_is_stop ? host->mrq->stop : host->mrq->cmd;

	if (!cmd) {
		host->status = "no active cmd";
		clear_imask(host);
		goto irq_out;
	}

	cmd->error = 0;

	if (mci_imsk & SDIIMSK_CMDTIMEOUT) {
#ifdef CONFIG_MMC_DEBUG
		dbg(host, dbg_conf, "CMDSTAT: error CMDTIMEOUT\n");
#endif
		cmd->error = -ETIMEDOUT;
		host->status = "error: command timeout";
		goto fail_transfer;
	}

	if ((mci_imsk & SDIIMSK_CMDSENT)) {
		if (host->complete_what == COMPLETION_CMDSENT) {
			host->status = "ok: command sent";
			goto close_transfer;
		}
	}

	if (mci_imsk & SDIIMSK_RESPONSECRC) {
		if (cmd->flags & MMC_RSP_CRC) {
			if (host->mrq->cmd->flags & MMC_RSP_136) {
				dbg(host, dbg_irq,
				    "fixup: ignore CRC fail with long rsp\n");
			}
		}
	}

	if ((mci_imsk & SDIIMSK_CMDSENT)) {
		if (host->complete_what == COMPLETION_RSPFIN) {
			host->status = "ok: command response received";
			goto close_transfer;
		}
		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN)
			host->complete_what = COMPLETION_XFERFINISH;
	}

	if (!cmd->data)
		goto clear_status_bits;

	if (mci_imsk & SDIIMSK_RXCRCFAIL) {
		dbg(host, dbg_err, "bad data crc (outgoing)\n");
		cmd->data->error = -EILSEQ;
		host->status = "error: bad data crc (outgoing)";
		goto fail_transfer;
	}

	if (mci_imsk & SDIIMSK_TXCRCFAIL) {
		dbg(host, dbg_err, "bad data crc (incoming)\n");
		cmd->data->error = -EILSEQ;
		host->status = "error: bad data crc (incoming)";
		goto fail_transfer;
	}

	if (mci_imsk & SDIIMSK_DATATIMEOUT) {
		dbg(host, dbg_err, "data timeout\n");
		cmd->data->error = -ETIMEDOUT;
		host->status = "error: data timeout";
		goto fail_transfer;
	}

	if ((mci_imsk & SDIIMSK_DATAFINISH)) {
		if (host->complete_what == COMPLETION_XFERFINISH) {
			host->status = "ok: data transfer completed";
			host->dma_complete = 1;
			goto close_transfer;
		}

		if (host->complete_what == COMPLETION_XFERFINISH_RSPFIN)
			host->complete_what = COMPLETION_RSPFIN;
	}

clear_status_bits:
	goto irq_out;

fail_transfer:
	host->pio_active = XFER_NONE;

close_transfer:
	/* Used to fix the problem that DMA copies are small in length. */
	if (dma_buf && cmd->data && cmd->data->sg) {
		dma_length = sg_dma_len(cmd->data->sg);

		if (dma_length == 8 || dma_length == 12) {
			/* Organize the messy data and return the correct data to the kernel stack. */
			sg_dma_buf = TO_CAC(sg_dma_address(cmd->data->sg));
			for (i = 0, j = dma_length / 4; i < j; i++)
				((volatile __be32 *)sg_dma_buf)[i] = ((volatile __be32 *)dma_buf)[i * 4 + i];

			kfree((void *)dma_buf);
			dma_buf = 0;
		}
	}

	host->complete_what = COMPLETION_FINALIZE;

	if (host->pdata->irq_fixup)
		host->pdata->irq_fixup(host, cmd);

	writel(mci_imsk, host->base + SDIINTMSK);
	clear_imask(host);
	tasklet_schedule(&host->pio_tasklet);
	if (cmd->data && cmd->data->sg)
		dma_unmap_sg(mmc_dev(host->mmc), cmd->data->sg, cmd->data->sg_len,
			(cmd->data->flags & MMC_DATA_WRITE) ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	goto irq_out;

irq_out:
	writel(mci_imsk, host->base + SDIINTMSK);
	return IRQ_HANDLED;
}

static void ls2k_mci_set_clk(struct ls2k_mci_host *host, struct mmc_ios *ios)
{
	u32 mci_psc;

	/* Set clock */
	for (mci_psc = 1; mci_psc < 255; mci_psc++) {
		host->real_rate = host->clk_rate / mci_psc;

		if (host->real_rate <= ios->clock)
			break;
	}
	if (mci_psc > 255)
		mci_psc = 255;

	host->prescaler = mci_psc;
	writel(host->prescaler | SDIPRE_REVCLOCK, host->base + SDIPRE);

	/* If requested clock is 0, real_rate will be 0, too */
	if (ios->clock == 0)
		host->real_rate = 0;
}

static noinline void
ls2k_mci_send_command(struct ls2k_mci_host *host, struct mmc_command *cmd)
{
	u32 ccon;

	if (cmd->data) {
		host->complete_what = COMPLETION_XFERFINISH_RSPFIN;
	} else if (cmd->flags & MMC_RSP_PRESENT)
		host->complete_what = COMPLETION_RSPFIN;
	else
		host->complete_what = COMPLETION_CMDSENT;

	writel(cmd->arg, host->base + SDICMDARG);
	ccon  = cmd->opcode & SDICMDCON_INDEX;
	ccon |= SDICMDCON_SENDERHOST | SDICMDCON_CMDSTART;

	/* Emmc cmd6 do not need data transfer */
	if (host->pdata->version > LOONGSON_SDIO_EMMC_VER_1_0 &&
			cmd->opcode == SD_SWITCH && cmd->data)
		ccon |= SDICMDCON_CMD6DATA;

	if (cmd->flags & MMC_RSP_PRESENT)
		ccon |= SDICMDCON_WAITRSP;

	if (cmd->flags & MMC_RSP_136)
		ccon |= SDICMDCON_LONGRSP;

#ifdef CONFIG_AIC_WLAN_SUPPORT
	unsigned int val;
	if (cmd->opcode == 53){
		val = readl(host->base + SDIDCON);
		val |= (0x3 << 24);                        
		writel(val, host->base + SDIDCON);
	}
#endif

	writel(ccon, host->base + SDICMDCON);
}

static int ls2k_mci_prepare_dma(struct ls2k_mci_host *host, struct mmc_data *data)
{
	void __iomem *dma_order_reg;
	int dma_len, i;
	u64 dma_order;
	int rw = data->flags & MMC_DATA_WRITE;

	dma_len = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
			     rw ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	if (dma_len == 0)
		return -ENOMEM;

	host->dma_complete = 0;
	host->dmatogo = dma_len;

	for (i = 0; i < dma_len; i++) {
		host->sg_cpu[i].length = sg_dma_len(&data->sg[i])  / 4;
		host->sg_cpu[i].step_length = 0;
		host->sg_cpu[i].step_times = 1;

		/* 
		 * If the transmission length is 8 bytes or 12 bytes, the DMA 
		 * transmission is out of order, so a new piece of memory is 
		 * allocated to store the messy data.
		 */
		if (sg_dma_len(&data->sg[i]) == 8 || sg_dma_len(&data->sg[i]) == 12) {
			dma_buf = (u64)kmalloc(48, GFP_KERNEL);
			if (!dma_buf)
				return -ENOMEM;

			memset((void *)dma_buf, 0x00, 48);
			host->sg_cpu[i].saddr    = dma_buf;
			host->sg_cpu[i].saddr_hi = dma_buf >> 32;
		} else {
			host->sg_cpu[i].saddr    = sg_dma_address(&data->sg[i]);
			host->sg_cpu[i].saddr_hi = ((sg_dma_address(&data->sg[i])) >> 32);
		}
		host->sg_cpu[i].daddr = host->phys_base+0x40;
		if (data->flags & MMC_DATA_READ) {
			host->sg_cpu[i].cmd = 0x1<<0;
			dma_order_reg = host->rdma_order_reg;
		} else {
			host->sg_cpu[i].cmd = ((0x1<<12) | (0x1<<0));
			dma_order_reg = host->wdma_order_reg;
		}

		host->sg_cpu[i].order_addr = host->sg_dma+(i+1)*sizeof(struct ls2k_dma_desc);
		host->sg_cpu[i].order_addr_hi = ((host->sg_dma+(i+1)*sizeof(struct ls2k_dma_desc)) >> 32);
		host->sg_cpu[i].order_addr |= 0x1<<0;
	}

	host->sg_cpu[dma_len-1].order_addr &= ~(0x1<<0);

	dma_order = (readq(dma_order_reg) & 0xfUL) | ((host->sg_dma & ~0x1fUL) | 0x8UL);

	if (host->pdev->dev.coherent_dma_mask == DMA_BIT_MASK(64))
		dma_order |= 0x1UL;
	else
		dma_order &= ~0x1UL;

	if (host->pdata->version == LOONGSON_SDIO_EMMC_VER_1_0)
		writeq(dma_order, dma_order_reg);
	else {
		writel(dma_order >> 32, dma_order_reg + 4);
		writel(dma_order & 0xffffffff, dma_order_reg);
	}

	return 0;
}

static int ls2k_mci_setup_data(struct ls2k_mci_host *host, struct mmc_data *data)
{
	u32 dcon;

	/* write DCON register */
	if (!data) {
		writel(0, host->base + SDIDCON);
		return 0;
	}

	if ((data->blksz & 3) != 0) {
		if (data->blocks > 1)
			return -EINVAL;
	}

	dcon  = data->blocks & SDIDCON_BLKNUM_MASK;
	if (host->bus_width == MMC_BUS_WIDTH_4) {
		dcon |= SDIDCON_4BIT_BUS;
	} else if (host->bus_width == MMC_BUS_WIDTH_8) {
		dcon |= SDIDCON_8BIT_BUS;
	}

	dcon |= 3 << 14;
	writel(dcon, host->base + SDIDCON);
	writel(data->blksz, host->base + SDIBSIZE);

	/* write TIMER register */
	writel(0xFFFFFFFF, host->base + SDITIMER);

	return 0;
}


static void ls2k_mci_set_dll(struct ls2k_mci_host *host, u32 dll)
{
	//u32 value = (dll << 8);     //only for rd dll value
	u32 value = dll;
	writel(0x30000000, host->base + 0xf4);    //set rd dll
	writel(value, host->base + 0xf8);
}


static void ls2k_mci_send_request(struct mmc_host *mmc)
{
	int res;
	struct ls2k_mci_host *host = mmc_priv(mmc);
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd = host->cmd_is_stop ? mrq->stop : mrq->cmd;

	host->ccnt++;

	if (cmd->data) {
		host->dcnt++;
			res = ls2k_mci_setup_data(host, cmd->data);
			if (res) {
				dbg(host, dbg_err, "setup data error %d\n", res);
				cmd->error = res;
				cmd->data->error = res;
				mmc_request_done(mmc, mrq);
				return;
			}
			res = ls2k_mci_prepare_dma(host, cmd->data);
			if (res) {
				dbg(host, dbg_err, "data prepare error %d\n", res);
				cmd->error = res;
				cmd->data->error = res;
				mmc_request_done(mmc, mrq);
				return;
			}
	}

#if 1   /* to solve 2k1500 emmc bug for losing interrupt when send cmd25 */
	u32 s_flag, t_flag, tmp;
	int retry_n = 1000;
        if (cmd->opcode == 0x19) {  //for cmd25  multiple blocks write
		while (retry_n--) {
		        tmp = readl(host->base + 0x2c);  
			s_flag = tmp & (0x1 << 14);
			tmp = readl(host->base + 0x38);
			t_flag = tmp & (0x1 << 11);

			if (s_flag && t_flag)  //tx fifo full
				break;
			if (retry_n < 500)
				udelay(10);
		}	
#ifdef CONFIG_MMC_DEBUG
		if (retry_n < 0)
	        	dbg(host, dbg_conf, "mci warning: may losing int when tx fifo is not full\n");
#endif
	}
#endif

	/* Send command */
	ls2k_mci_send_command(host, cmd);

	/* Note the app cmd opcode */
	if (cmd->opcode == MMC_APP_CMD || host->app_cmd == MMC_APP_CMD)
		host->app_cmd = cmd->opcode;
	else
		host->app_cmd = 0;

	/* Enable Interrupt */
	ls2k_mci_enable_irq(host, true);

	/* fix deselect card no irq */
	if (cmd->opcode == MMC_SELECT_CARD && cmd->arg == 0) {
		cmd->error = 0;
		mmc_request_done(mmc, mrq);
	}
}

static void ls2k_mci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct ls2k_mci_host *host = mmc_priv(mmc);
	struct platform_device *pdev = host->pdev;
	u32 mci_con;
	/* Set the power state */
	mci_con = readl(host->base + SDICON);
	u32 dll;
	int ret;


	ls2k_mci_disable_irq(host, true);
	switch (ios->power_mode) {
	case MMC_POWER_ON:
	case MMC_POWER_UP:
		writel(SDICON_RESET, host->base + SDICON);
		mdelay(100);
		writel(0x3, host->base + SDICON);
		if (host->pdata->version >= LOONGSON_SDIO_EMMC_VER_1_1)
			writel(SDIINT_CLEARALL, host->base + SDIINTMSK);
		writel(SDIINT_ENALL, host->base + SDIINTEN);
		break;
	case MMC_POWER_OFF:
	default:
		mci_con |= SDICON_RESET;
		break;
	}
	ls2k_mci_set_clk(host, ios);

	/* Set CLOCK_ENABLE */
	if (ios->clock)
		mci_con |= SDICON_CLOCKTYPE;
	writel(mci_con, host->base + SDICON);

#ifdef CONFIG_MMC_DEBUG
	if ((ios->power_mode == MMC_POWER_ON) ||
	    (ios->power_mode == MMC_POWER_UP)) {
		dbg(host, dbg_conf, "running at %lukHz (requested: %ukHz).\n",
			host->real_rate/1000, ios->clock/1000);
	} else {
		dbg(host, dbg_conf, "powered down.\n");
	}
#endif
	host->bus_width = ios->bus_width;

	ret = of_property_read_u32(pdev->dev.of_node, "dll", &dll);
        if (ret == 0) 
                ls2k_mci_set_dll(host, dll);
	mdelay(100);
	ls2k_mci_enable_irq(host, true);
}

static int ls2k_mci_card_present(struct mmc_host *mmc)
{
	return 1;
}

static void ls2k_mci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct ls2k_mci_host *host = mmc_priv(mmc);
	host->status = "mmc request";
	host->cmd_is_stop = 0;
	host->mrq = mrq;

	if (ls2k_mci_card_present(mmc) == 0) {
		dbg(host, dbg_err, "%s: no medium present\n", __func__);
		host->mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
	} else
		ls2k_mci_send_request(mmc);
}

static int ls2k_mci_get_ro(struct mmc_host *mmc)
{
	return 0;
}

static int ls2k_mci_gpio_get_cd(struct mmc_host *host)
{
	int value, virq, chip_irq, wait_fall;  
	struct mmc_gpio *ctx;
	struct gpio_chip *gc;
	
	ctx = host->slot.handler_priv;

	if (!ctx) {
		printk("err: get cd, invalid mmc gpio\n");
		return -EINVAL;
	}

	virq = gpiod_to_irq(ctx->cd_gpio);
	value = mmc_gpio_get_cd(host);

	gc = gpiod_to_chip(ctx->cd_gpio);
	if (!gc) {
		printk("err: get cd, can't find gpio chip\n");
		return -EINVAL;
	}

	gpiod_direction_input(ctx->cd_gpio);

	if (value == 1) {                          //insert card	
		irq_set_irq_type(virq, IRQ_TYPE_EDGE_RISING);
	} else {                                   //pull out card 
		irq_set_irq_type(virq, IRQ_TYPE_EDGE_FALLING);
	}

	return value;
}

static struct mmc_host_ops ls2k_mci_ops = {
	.request	= ls2k_mci_request,
	.set_ios	= ls2k_mci_set_ios,
	.get_ro		= ls2k_mci_get_ro,
	.get_cd		= ls2k_mci_gpio_get_cd,
};

static struct ls2k_mci_pdata ls2k_mci_v1_0_pdata = {
	 .no_wprotect = 0,
	 .no_detect = 0,
	 .irq_fixup = ls2k_cmd_data_fix,
	 .version = LOONGSON_SDIO_EMMC_VER_1_0,
};

static struct ls2k_mci_pdata ls2k_mci_v1_1_pdata = {
	 .irq_fixup = ls2k_cmd_data_fix,
	 .version = LOONGSON_SDIO_EMMC_VER_1_1,
};

static struct ls2k_mci_pdata ls2k_mci_v1_2_pdata = {
	 .irq_fixup = ls2k_cmd_data_fix,
	 .version = LOONGSON_SDIO_EMMC_VER_1_2,
};

static struct ls2k_mci_host *hotpug_host;
static int ls2k_mci_hotplug_set(const char *val, const struct kernel_param *kp)
{
	mmc_detect_change(hotpug_host->mmc, msecs_to_jiffies(500));
	return 0;
}

module_param_call(hotplug, ls2k_mci_hotplug_set, NULL, NULL, 0664);

static int ls2k_mci_probe(struct platform_device *pdev)
{
	struct ls2k_mci_host *host;
	struct ls2k_mci_pdata *pdata;
	struct mmc_host	*mmc;
	int ret, addr_cells;
	unsigned long flags;
	struct resource *r;
	struct dma_chan *chan;
	__be32 *of_property = NULL;
	struct mmc_gpio *ctx;
	int cd_gpio_num;
	u64 val;

	if (dev_fwnode(&pdev->dev)) {
		if (!device_property_read_u64(&pdev->dev, "dma-mask",
					(u64 *)&pdev->dev.coherent_dma_mask)) {
			if (pdev->dev.dma_mask)
				*(pdev->dev.dma_mask) = pdev->dev.coherent_dma_mask;
			else
				pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
		}
	}

	mmc = mmc_alloc_host(sizeof(struct ls2k_mci_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto probe_free_host;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->pdev = pdev;

	pdata = (struct ls2k_mci_pdata *)device_get_match_data(&pdev->dev);
	if (!pdata) {
		ret = -EINVAL;
		goto probe_free_host;
	}

	platform_device_add_data(pdev, pdata, sizeof(*pdata));
	host->pdata = pdata;

	spin_lock_init(&host->complete_lock);
	tasklet_init(&host->pio_tasklet, pio_tasklet, (unsigned long) host);
	host->sdiimsk		= 0x3c;
	host->sdidata		= 0x40;
	host->clk_div		= 0x1;
	host->clk_rate		= DEF_CLKRATE;
	host->complete_what	= COMPLETION_NONE;
	host->pio_active	= XFER_NONE;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no IO memory resource defined\n");
		ret = -ENODEV;
		goto probe_free_host;
	}
	host->phys_base = r->start;

	if (r->start == LS2K1000_SDIO_REG_BASE)
		cd_gpio_inten_base = (void *)LS2K1000_GPIO_INTEN_BASE;
	else if (r->start == LS2K0500_SDIO0_REG_BASE)
		cd_gpio_inten_base = (void *)LS2K0500_SDIO0_GPIO_INTEN_BASE;
	else if (r->start == LS2K0500_SDIO1_REG_BASE)
		cd_gpio_inten_base = (void *)LS2K0500_SDIO1_GPIO_INTEN_BASE;

	host->base = ioremap(r->start, r->end - r->start + 1);

	ret = of_property_read_u32(pdev->dev.of_node, "#address-cells", &addr_cells);
	if (ret) {
		dev_err(&pdev->dev, "missing #address-cells property\n");
		addr_cells = 1;
	}

	chan = of_dma_request_slave_channel(pdev->dev.of_node, "sdio_rw");

	if (IS_ERR(chan)) {
		dev_info(&pdev->dev, "Use exclusive dma engine.\n");
		host->wdma_order_reg = host->base + WDMA_OFFSET;
		host->rdma_order_reg = host->base + RDMA_OFFSET;
	} else {
		if (addr_cells == 2) {
			of_property = (__be32 *)of_get_property(chan->device->dev->of_node, "reg", NULL);
			if (of_property != 0)
				r->start = of_read_number(of_property, 2);
		} else {
			of_property_read_u32(chan->device->dev->of_node, "reg", (u32 *)&r->start);
		}
		host->wdma_order_reg = ioremap(r->start, 8);
		host->rdma_order_reg = host->wdma_order_reg;
	}

	writel(SDICON_RESET, host->base + SDICON);
	mdelay(1);
	writel(SDIINT_ENALL, host->base + SDIINTEN);
	writel(0x2, host->base + SDICON);

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq == 0) {
		dev_err(&pdev->dev, "failed to get interrupt resouce.\n");
		ret = -EINVAL;
		goto probe_iounmap;
	}

	hotpug_host = host;
	local_irq_save(flags);
	if (request_irq(host->irq, ls2k_mci_irq, 0, DRIVER_NAME, host)) {
		dev_err(&pdev->dev, "failed to request mci interrupt.\n");
		ret = -ENOENT;
		goto probe_iounmap;
	}

	disable_irq(host->irq);
	host->irq_state = false;
	local_irq_restore(flags);

	if (ls2k_mci_host_usedma(host)) {
		host->dma = 0;
	}

	host->sg_cpu = dma_alloc_coherent(&pdev->dev, 0x100*28, &host->sg_dma, GFP_KERNEL);
	memset(host->sg_cpu, 0, 0x100*28);

	if (dev_fwnode(&pdev->dev)) {
		device_property_read_u64(&pdev->dev, "clock-frequency", (u64 *)&host->clk_rate);
	}

	mmc->ops	= &ls2k_mci_ops;
	mmc->ocr_avail	= MMC_VDD_32_33|MMC_VDD_33_34;

	mmc->f_min	= host->clk_rate / (host->clk_div * 256);
	mmc->f_max	= host->clk_rate / host->clk_div;

	if (host->pdata->ocr_avail)
		mmc->ocr_avail = host->pdata->ocr_avail;

	mmc->max_blk_count	= 4095;
	mmc->max_blk_size	= 4095;
	mmc->max_req_size	= mmc->max_blk_count * mmc->max_blk_size;
	mmc->max_segs		= 1;
	mmc->max_seg_size	= mmc->max_req_size;
	dbg(host, dbg_debug, "mapped mci_base:%p irq:%u irq_cd:%u dma:%u.\n", host->base, host->irq, host->irq_cd, host->dma);

	ret = mmc_of_parse(mmc);

	if (ret) {
		dev_err(mmc_dev(mmc), "failed to parse device node!\n");
		ret = -EINVAL;
		goto probe_iounmap;
	}

	ctx = mmc->slot.handler_priv;

	if (!(mmc->caps & MMC_CAP_NONREMOVABLE) &&
	    !(mmc->caps & MMC_CAP_NEEDS_POLL) && cd_gpio_inten_base) {
		cd_gpio_num = desc_to_gpio(ctx->cd_gpio);
		local_irq_save(flags);
		val = readq((void *)cd_gpio_inten_base);
		val |= (0x1 << cd_gpio_num);
		writeq(val, (void *)cd_gpio_inten_base);
		local_irq_restore(flags);
		ls2k_mci_gpio_get_cd(mmc);
	}


	ret = mmc_add_host(mmc);
	if (ret) {
		dev_err(&pdev->dev, "failed to add mmc host.\n");
		goto free_cpufreq;
	}
	platform_set_drvdata(pdev, mmc);

	return 0;

free_cpufreq:
	free_irq(host->irq, host);

probe_iounmap:
	iounmap(host->base);

probe_free_host:
	mmc_free_host(mmc);
	return ret;
}

static void ls2k_mci_shutdown(struct platform_device *pdev)
{
	struct mmc_host	*mmc = platform_get_drvdata(pdev);

	mmc_remove_host(mmc);
}

static int ls2k_mci_remove(struct platform_device *pdev)
{
	struct mmc_host		*mmc  = platform_get_drvdata(pdev);
	struct ls2k_mci_host	*host = mmc_priv(mmc);

	ls2k_mci_shutdown(pdev);

	tasklet_disable(&host->pio_tasklet);

	free_irq(host->irq, host);

	iounmap(host->base);

	mmc_free_host(mmc);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sdio_ls2k_dt_match[] = {
	{.compatible = "loongson,ls2k_sdio", .data = &ls2k_mci_v1_0_pdata},
	{.compatible = "loongson,ls2k_sdio_1.1", .data = &ls2k_mci_v1_1_pdata},
    {.compatible = "loongson,ls2k_sdio_1.2", .data = &ls2k_mci_v1_2_pdata},
	{},
};
MODULE_DEVICE_TABLE(of, sdio_ls2k_dt_match);
#endif

static struct platform_driver ls2k_mci_driver = {
	.driver	= {
		.name	= "ls2k_sdio",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(sdio_ls2k_dt_match),
#endif
	},
	.probe		= ls2k_mci_probe,
	.remove		= ls2k_mci_remove,
	.shutdown	= ls2k_mci_shutdown,
};

static int __init ls2k_mci_init(void)
{
	return platform_driver_register(&ls2k_mci_driver);
}

static void __exit ls2k_mci_exit(void)
{
	platform_driver_unregister(&ls2k_mci_driver);
}

module_init(ls2k_mci_init);
module_exit(ls2k_mci_exit);
static void ls2k_mci_pci_remove(struct pci_dev *pci)
{
	struct platform_device	*ls2k_mci = pci_get_drvdata(pci);

	platform_device_unregister(ls2k_mci);
	pci_set_drvdata(pci, NULL);
}
static int ls2k_mci_pci_probe(struct pci_dev *pci,
			  const struct pci_device_id *id)
{
	struct resource		res[2];
	struct platform_device_info plat_info;
	struct platform_device	*ls2k_mci;
	int			ret;
	struct device		*dev = &pci->dev;

	ret = pcim_enable_device(pci);
	if (ret) {
		dev_err(dev, "failed to enable pci device\n");
		return -ENODEV;
	}

	pci_set_master(pci);

	memset(res, 0x00, sizeof(struct resource) * ARRAY_SIZE(res));
	res[0].start	= pci_resource_start(pci, 0);
	res[0].end	= pci_resource_end(pci, 0);
	res[0].name	= "ls2k_sdio";
	res[0].flags	= IORESOURCE_MEM;
	res[1].start = pci->irq ? : of_irq_get(dev->of_node, 0);
	res[1].name	= "ls2k_sdio";
	res[1].flags	= IORESOURCE_IRQ;

	ls2k_mci = platform_device_alloc("ls2k_sdio", PLATFORM_DEVID_AUTO);
	ret = platform_device_add_resources(ls2k_mci, res, ARRAY_SIZE(res));
	if (ret) {
		dev_err(dev, "couldn't add resources to ls2k_mci device\n");
		goto err;
	}
	ls2k_mci->dev.parent = dev;
	ls2k_mci->dev.fwnode = dev->fwnode;
	ls2k_mci->dev.of_node = of_node_get(dev->of_node);
	if (dev->dma_mask)
		ls2k_mci->dev.dma_mask = dev->dma_mask;
	ls2k_mci->dev.coherent_dma_mask = dev->coherent_dma_mask;

	ret = platform_device_add(ls2k_mci);

	if (ret) {
		dev_err(dev, "failed to register ls2k_mci device\n");
		goto err;
	}

	pci_set_drvdata(pci, ls2k_mci);

	return 0;
err:
	platform_device_put(ls2k_mci);
	return ret;
}

static const struct pci_device_id ls2k_mci_pci_ids[] = {
	{
	    PCI_VENDOR_ID_LOONGSON, 0x7a48
	},
	 
	{
	    PCI_VENDOR_ID_LOONGSON, 0x7a88
	},
	{ /* end: all zeroes */ }
};
MODULE_DEVICE_TABLE(pci, ls2k_mci_pci_ids);
static struct pci_driver ls2k_mci_pci_driver = {
	.name = "ls2k_sdio",
	.id_table = ls2k_mci_pci_ids,
	.probe = ls2k_mci_pci_probe,
	.remove = ls2k_mci_pci_remove,
};

module_pci_driver(ls2k_mci_pci_driver);


MODULE_DESCRIPTION("Loongson ls2k MMC/SD Card Interface driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(" Loongson kernel-development team");
