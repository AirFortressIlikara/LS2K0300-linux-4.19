/*******************************************************************************
  Specialised functions for managing Ring mode

  Copyright(C) 2011  STMicroelectronics Ltd

  It defines all the functions used to handle the normal/enhanced
  descriptors in case of the DMA is configured to work in chained or
  in ring mode.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#include "stmmac.h"

static int jumbo_frm(void *p, struct sk_buff *skb, int csum)
{
	struct stmmac_tx_queue *tx_q = (struct stmmac_tx_queue *)p;
	unsigned int nopaged_len = skb_headlen(skb);
	struct stmmac_priv *priv = tx_q->priv_data;
	unsigned int entry = tx_q->cur_tx;
	unsigned int bmax, len;
	struct dma_extended_desc *edesc = (tx_q->dma_etx + entry);
	struct dma_desc *desc = &edesc->basic;
	dma_addr_t des2;

	bmax = BUF_SIZE_8KiB;

	len = nopaged_len - bmax*2;


	if (nopaged_len > bmax*2) {

		desc->des2 = des2 = dma_map_single(priv->device, skb->data,
					    bmax*2, DMA_TO_DEVICE);
		desc->des3 = des2 >> 32;
		if (dma_mapping_error(priv->device, des2))
			return -1;

		tx_q->tx_skbuff_dma[entry].buf = des2;
		tx_q->tx_skbuff_dma[entry].len = bmax*2;
		tx_q->tx_skbuff_dma[entry].is_jumbo = true;

		edesc->des6 = des2 + bmax;
		edesc->des7 = (des2 + bmax) >> 32;
		stmmac_prepare_tx_desc(priv, desc, 1, bmax, csum,
				STMMAC_RING_MODE, 1, false, skb->len);

		tx_q->tx_skbuff[entry] = NULL;
		entry = STMMAC_GET_ENTRY(entry, DMA_TX_SIZE);
		edesc = tx_q->dma_etx + entry;
		desc = &edesc->basic;

		desc->des2 = des2 = dma_map_single(priv->device, skb->data + bmax,
					    len, DMA_TO_DEVICE);
		desc->des3 = des2 >> 32;
		if (dma_mapping_error(priv->device, des2))
			return -1;
		tx_q->tx_skbuff_dma[entry].buf = des2;
		tx_q->tx_skbuff_dma[entry].len = len;
		tx_q->tx_skbuff_dma[entry].is_jumbo = true;

		edesc->des6 = des2 + bmax;
		edesc->des7 = (des2 + bmax) >> 32;
		stmmac_prepare_tx_desc(priv, desc, 0, len, csum,
						STMMAC_RING_MODE, 1, true, skb->len);
	} else {
		desc->des2 = des2 = dma_map_single(priv->device, skb->data,
					    nopaged_len, DMA_TO_DEVICE);
		desc->des3 = des2 >> 32;
		if (dma_mapping_error(priv->device, des2))
			return -1;
		tx_q->tx_skbuff_dma[entry].buf = des2;
		tx_q->tx_skbuff_dma[entry].len = nopaged_len;
		tx_q->tx_skbuff_dma[entry].is_jumbo = true;
		edesc->des6 = des2 + bmax;
		edesc->des7 = (des2 + bmax) >> 32;
		stmmac_prepare_tx_desc(priv, desc, 1, nopaged_len, csum,
						STMMAC_RING_MODE, 1, true, skb->len);
	}

	tx_q->cur_tx = entry;

	return entry;
}

static unsigned int is_jumbo_frm(int len, int enh_desc)
{
	unsigned int ret = 0;

	if (len >= BUF_SIZE_4KiB)
		ret = 1;

	return ret;
}

static void refill_desc3(void *priv_ptr, struct dma_desc *desc)
{
	struct stmmac_rx_queue *rx_q = priv_ptr;
	struct stmmac_priv *priv = rx_q->priv_data;
	struct dma_extended_desc *edesc = (struct dma_extended_desc *)desc;

		/* Fill DES3 in case of RING mode */
		if (priv->dma_buf_sz >= BUF_SIZE_8KiB) {
			edesc->des6 = edesc->basic.des2 + BUF_SIZE_8KiB;
			edesc->des7 = edesc->basic.des3;
		}
}

/* In ring mode we need to fill the desc3 because it is used as buffer */
static void init_desc3(struct dma_desc *desc)
{
	struct dma_extended_desc *edesc = (struct dma_extended_desc *)desc;
	edesc->des6 = edesc->basic.des2 + BUF_SIZE_8KiB;
	edesc->des7 = edesc->basic.des3;
}

static void clean_desc3(void *priv_ptr, struct dma_desc *p)
{
	struct stmmac_tx_queue *tx_q = (struct stmmac_tx_queue *)priv_ptr;
	struct stmmac_priv *priv = tx_q->priv_data;
	unsigned int entry = tx_q->dirty_tx;
	if (unlikely(tx_q->tx_skbuff_dma[entry].is_jumbo)) {
		struct dma_extended_desc *edesc = (struct dma_extended_desc *)p;
		edesc->des6 = 0;
		edesc->des7 = 0;
	}
}

static int set_16kib_bfsize(int mtu)
{
	int ret = 0;
	if (unlikely(mtu >= BUF_SIZE_8KiB))
		ret = BUF_SIZE_16KiB;
	return ret;
}

const struct stmmac_mode_ops ring_mode64_ops = {
	.is_jumbo_frm = is_jumbo_frm,
	.jumbo_frm = jumbo_frm,
	.refill_desc3 = refill_desc3,
	.init_desc3 = init_desc3,
	.clean_desc3 = clean_desc3,
	.set_16kib_bfsize = set_16kib_bfsize,
};
