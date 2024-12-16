/*
 * linux/sound/mips/ls-pcm.h -- ALSA PCM interface for the Loongson chip
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _LS_PCM_H
#define _LS_PCM_H
#include <asm/delay.h>

#define DMA_MASK	0xffffffffffffffff

typedef struct ls_dma_desc {
	u32 ordered;
	u32 saddr;
	u32 daddr;
	u32 length;
	u32 step_length;
	u32 step_times;
	u32 cmd;
	u32 stats;
	u32 ordered_hi;
	u32 saddr_hi;
	u32 dummy[6];
} ls_dma_desc;

struct ls_runtime_data {
	int dma_ch;
	u64 totsize;
	u64 i2s_ctl_base;
	struct ls_pcm_dma_params *params;
	ls_dma_desc *dma_desc_array;
	dma_addr_t dma_desc_array_phys;

	ls_dma_desc *dma_desc_ready;
	dma_addr_t dma_desc_ready_phys;

	ls_dma_desc *dma_position_desc;
	dma_addr_t dma_position_desc_phys;

	dma_addr_t base_phys;
	void	*base;
	dma_addr_t order_addr1_phys;
	void	*order_addr1;
	dma_addr_t order_addr2_phys;
	void	*order_addr2;
    struct dma_chan         *chan;

	struct dma_chan *tx_ch;  /* dma rx channel            */
	dma_addr_t tx_dma_buf;   /* dma rx buffer bus address */
	unsigned int *tx_buf;   /* dma rx buffer cpu address */
	struct dma_chan *rx_ch;  /* dma rx channel            */
	dma_addr_t rx_dma_buf;   /* dma rx buffer bus address */
	unsigned int *rx_buf;   /* dma rx buffer cpu address */
};

struct ls_pcm_client {
	struct ls_pcm_dma_params *playback_params;
	struct ls_pcm_dma_params *capture_params;
	int (*startup)(struct snd_pcm_substream *);
	void (*shutdown)(struct snd_pcm_substream *);
	int (*prepare)(struct snd_pcm_substream *);
};

extern int ls_pcm_new(struct snd_card *, struct ls_pcm_client *, struct snd_pcm **);

extern struct snd_soc_component_driver ls_soc_platform;

static inline u32 read_reg(volatile u32 *reg)
{
	return *reg;
}
static inline void write_reg(volatile u32 *reg, u32 val)
{
	*(reg) = (val);
}

#endif
