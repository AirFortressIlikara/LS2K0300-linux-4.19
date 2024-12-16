/*******************************************************************************
  Copyright 2023 Loongson Technology, Inc.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".
*******************************************************************************/

#include <linux/io.h>
#include <linux/iopoll.h>
#include "common.h"
#include "dwmac_loongson_dma.h"

int dwmac_loongson_dma_reset(void __iomem *ioaddr)
{
	u32 value = readl(ioaddr + DMA_BUS_MODE);
	int cnt = 5;
	int err;

	/* DMA SW reset */
	do {
		value |= DMA_BUS_MODE_SFT_RESET;
		writel(value, ioaddr + DMA_BUS_MODE);

		err = readl_poll_timeout(ioaddr + DMA_BUS_MODE, value,
					 !(value & DMA_BUS_MODE_SFT_RESET),
					 10000, 200000);
	} while (cnt-- && err);

	if (err)
		return -EBUSY;

	return 0;
}

/* CSR1 enables the transmit DMA to check for new descriptor */
void dwmac_loongson_enable_dma_transmission_chan(void __iomem *ioaddr, u32 chan)
{
	writel(1, ioaddr + DMA_XMT_POLL_DEMAND + chan * DMA_CHAN_OFFSET);
}

void dwmac_loongson_enable_dma_irq(void __iomem *ioaddr, u32 chan)
{
	writel(DMA_INTR_DEFAULT_MASK, ioaddr + DMA_INTR_ENA
										+ chan * DMA_CHAN_OFFSET);
}

void dwmac_loongson_disable_dma_irq(void __iomem *ioaddr, u32 chan)
{
	writel(0, ioaddr + DMA_INTR_ENA + chan * DMA_CHAN_OFFSET);
}

void dwmac_loongson_dma_start_tx(void __iomem *ioaddr, u32 chan)
{
	u32 value = readl(ioaddr + DMA_CONTROL + chan * DMA_CHAN_OFFSET);

	value |= DMA_CONTROL_ST;
	writel(value, ioaddr + DMA_CONTROL + chan * DMA_CHAN_OFFSET);
}

void dwmac_loongson_dma_stop_tx(void __iomem *ioaddr, u32 chan)
{
	u32 value = readl(ioaddr + DMA_CONTROL + chan * DMA_CHAN_OFFSET);

	value &= ~DMA_CONTROL_ST;
	writel(value, ioaddr + DMA_CONTROL + chan * DMA_CHAN_OFFSET);
}

void dwmac_loongson_dma_start_rx(void __iomem *ioaddr, u32 chan)
{
	u32 value = readl(ioaddr + DMA_CONTROL + chan * DMA_CHAN_OFFSET);

	value |= DMA_CONTROL_SR;
	writel(value, ioaddr + DMA_CONTROL + chan * DMA_CHAN_OFFSET);
}

void dwmac_loongson_dma_stop_rx(void __iomem *ioaddr, u32 chan)
{
	u32 value = readl(ioaddr + DMA_CONTROL + chan * DMA_CHAN_OFFSET);

	value &= ~DMA_CONTROL_SR;
	writel(value, ioaddr + DMA_CONTROL + chan * DMA_CHAN_OFFSET);
}

#ifdef DWMAC_DMA_DEBUG
static void show_tx_process_state(unsigned int status)
{
	unsigned int state;
	state = (status & DMA_STATUS_TS_MASK) >> DMA_STATUS_TS_SHIFT;

	switch (state) {
	case 0:
		pr_debug("- TX (Stopped): Reset or Stop command\n");
		break;
	case 1:
		pr_debug("- TX (Running): Fetching the Tx desc\n");
		break;
	case 2:
		pr_debug("- TX (Running): Waiting for end of tx\n");
		break;
	case 3:
		pr_debug("- TX (Running): Reading the data "
		       "and queuing the data into the Tx buf\n");
		break;
	case 6:
		pr_debug("- TX (Suspended): Tx Buff Underflow "
		       "or an unavailable Transmit descriptor\n");
		break;
	case 7:
		pr_debug("- TX (Running): Closing Tx descriptor\n");
		break;
	default:
		break;
	}
}

static void show_rx_process_state(unsigned int status)
{
	unsigned int state;

	state = (status & DMA_STATUS_RS_MASK) >> DMA_STATUS_RS_SHIFT;

	switch (state) {
	case 0:
		pr_debug("- RX (Stopped): Reset or Stop command\n");
		break;
	case 1:
		pr_debug("- RX (Running): Fetching the Rx desc\n");
		break;
	case 2:
		pr_debug("- RX (Running): Checking for end of pkt\n");
		break;
	case 3:
		pr_debug("- RX (Running): Waiting for Rx pkt\n");
		break;
	case 4:
		pr_debug("- RX (Suspended): Unavailable Rx buf\n");
		break;
	case 5:
		pr_debug("- RX (Running): Closing Rx descriptor\n");
		break;
	case 6:
		pr_debug("- RX(Running): Flushing the current frame"
		       " from the Rx buf\n");
		break;
	case 7:
		pr_debug("- RX (Running): Queuing the Rx frame"
		       " from the Rx buf into memory\n");
		break;
	default:
		break;
	}
}
#endif

int dwmac_loongson_dma_interrupt(void __iomem *ioaddr,
			struct stmmac_extra_stats *x, u32 chan)
{
	int ret = 0;

	/* read the status register (CSR5) */
	u32 intr_status = readl(ioaddr + DMA_STATUS + chan * DMA_CHAN_OFFSET);
#ifdef DWMAC_DMA_DEBUG
	/* Enable it to monitor DMA rx/tx status in case of critical problems */
	pr_debug("%s: [CSR5: 0x%08x]\n", __func__, intr_status);
	show_tx_process_state(intr_status);
	show_rx_process_state(intr_status);
#endif
	/* ABNORMAL interrupts */
	if (unlikely((intr_status & DMA_STATUS_TX_AIS) ||
			(intr_status & DMA_STATUS_RX_AIS))) {
		if (unlikely(intr_status & DMA_STATUS_UNF)) {
			ret = tx_hard_error_bump_tc;
			x->tx_undeflow_irq++;
		}
		if (unlikely(intr_status & DMA_STATUS_TJT))
			x->tx_jabber_irq++;

		if (unlikely(intr_status & DMA_STATUS_OVF))
			x->rx_overflow_irq++;

		if (unlikely(intr_status & DMA_STATUS_RU))
			x->rx_buf_unav_irq++;
		if (unlikely(intr_status & DMA_STATUS_RPS))
			x->rx_process_stopped_irq++;
		if (unlikely(intr_status & DMA_STATUS_RWT))
			x->rx_watchdog_irq++;
		if (unlikely(intr_status & DMA_STATUS_ETI))
			x->tx_early_irq++;
		if (unlikely(intr_status & DMA_STATUS_TPS)) {
			x->tx_process_stopped_irq++;
			ret = tx_hard_error;
		}
		if (unlikely((intr_status & DMA_STATUS_RX_FBI) ||
				(intr_status & DMA_STATUS_TX_FBI))) {
			x->fatal_bus_error_irq++;
			ret = tx_hard_error;
		}
	}
	/* TX/RX NORMAL interrupts */
	if (likely((intr_status & DMA_STATUS_TX_NIS) ||
			(intr_status & DMA_STATUS_RX_NIS))) {
		x->normal_irq_n++;
		if (likely(intr_status & DMA_STATUS_RI)) {
			u32 value = readl(ioaddr + DMA_INTR_ENA);
			/* to schedule NAPI on real RIE event. */
			if (likely(value & DMA_INTR_ENA_RIE)) {
				x->rx_normal_irq_n++;
				ret |= handle_rx;
			}
		}
		if (likely(intr_status & DMA_STATUS_TI)) {
			x->tx_normal_irq_n++;
			ret |= handle_tx;
		}
		if (unlikely(intr_status & DMA_STATUS_ERI))
			x->rx_early_irq++;
	}

	/* Clear the interrupt by writing a logic 1 to the CSR5[15-0] */
	writel((intr_status & 0x7ffff), ioaddr + DMA_STATUS
										+ chan * DMA_CHAN_OFFSET);

	return ret;
}
