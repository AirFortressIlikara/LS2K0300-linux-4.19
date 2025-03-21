/*******************************************************************************
  Specialised functions for managing Chained mode

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
	unsigned int bmax;
	unsigned int i = 1, len;
	struct dma_desc *desc;
	dma_addr_t des2;

	desc = tx_q->dma_tx + entry;

	if (priv->plat->enh_desc)
		bmax = BUF_SIZE_8KiB;
	else
		bmax = BUF_SIZE_2KiB;

	len = nopaged_len - bmax;

	desc->des2 = des2 = dma_map_single(priv->device, skb->data,
				    bmax, DMA_TO_DEVICE);
	desc->des3 = des2 >> 32;

	if (dma_mapping_error(priv->device, des2))
		return -1;
	tx_q->tx_skbuff_dma[entry].buf = des2;
	tx_q->tx_skbuff_dma[entry].len = bmax;
	/* do not close the descriptor and do not set own bit */
	stmmac_prepare_tx_desc(priv, desc, 1, bmax, csum, STMMAC_CHAIN_MODE,
			0, false, skb->len);

	while (len != 0) {
		tx_q->tx_skbuff[entry] = NULL;
		entry = STMMAC_GET_ENTRY(entry, DMA_TX_SIZE);
		desc = tx_q->dma_tx + entry;

		if (len > bmax) {
			desc->des2 = des2 = dma_map_single(priv->device,
						    (skb->data + bmax * i),
						    bmax, DMA_TO_DEVICE);
			desc->des3 = des2 >> 32;
			if (dma_mapping_error(priv->device, des2))
				return -1;
			tx_q->tx_skbuff_dma[entry].buf = des2;
			tx_q->tx_skbuff_dma[entry].len = bmax;
			stmmac_prepare_tx_desc(priv, desc, 0, bmax, csum, STMMAC_CHAIN_MODE,
							0, false, skb->len);
			len -= bmax;
			i++;
		} else {
			desc->des2 = des2 = dma_map_single(priv->device,
						    (skb->data + bmax * i), len,
						    DMA_TO_DEVICE);
			desc->des3 = des2 >> 32;
			if (dma_mapping_error(priv->device, des2))
				return -1;
			tx_q->tx_skbuff_dma[entry].buf = des2;
			tx_q->tx_skbuff_dma[entry].len = len;
			/* last descriptor can be set now */
			stmmac_prepare_tx_desc(priv, desc, 0, len, csum,
					STMMAC_CHAIN_MODE, 1, true, skb->len);
			len = 0;
		}
	}

	tx_q->cur_tx = entry;

	return entry;
}

static unsigned int is_jumbo_frm(int len, int enh_desc)
{
	unsigned int ret = 0;

	if ((enh_desc && (len > BUF_SIZE_8KiB)) ||
	    (!enh_desc && (len > BUF_SIZE_2KiB))) {
		ret = 1;
	}

	return ret;
}

static void init_dma_chain(void *des, dma_addr_t phy_addr,
				  unsigned int size, unsigned int extend_desc)
{
	/*
	 * In chained mode the des3 points to the next element in the ring.
	 * The latest element has to point to the head.
	 */
	int i;
	dma_addr_t dma_phy = phy_addr;

	struct dma_extended_desc *p = (struct dma_extended_desc *)des;
	for (i = 0; i < (size - 1); i++) {
		dma_phy += sizeof(struct dma_extended_desc);
		p->des6 = (unsigned int)dma_phy;
		p->des7 = dma_phy >> 32;
		p++;
	}
	p->des6 = (unsigned int)phy_addr;
	p->des7 = phy_addr>>32;

}

static void refill_desc3(void *priv_ptr, struct dma_desc *p)
{
}

static void clean_desc3(void *priv_ptr, struct dma_desc *p)
{
}

const struct stmmac_mode_ops chain_mode64_ops = {
	.init = init_dma_chain,
	.is_jumbo_frm = is_jumbo_frm,
	.jumbo_frm = jumbo_frm,
	.refill_desc3 = refill_desc3,
	.clean_desc3 = clean_desc3,
};
