/*
 * Header file for the Loongson APB DMA Controller driver
 *
 * Copyright (C) 2017 Loongson Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef LS_APB_DMA_H
#define	LS_APB_DMA_H

extern int hw_coherentio;

#define	LS2K0500_APB_DMA_CONF	0x1fe10100

#define APBSEL_WIDTH	3
#define APBSEL_MASK	0x7
#define SEL_MASK_SHIFT(sel)	(~(APBSEL_MASK << ((sel) * APBSEL_WIDTH)))	/*clear sel region value*/
#define SEL_VAL(nr, sel)	((nr) << ((sel) * APBSEL_WIDTH))	/*set nr into sel region*/

/**
 * struct ls_dma_chan - internal representation of an Loongson APBDMAC channel
 * @chan_common: common dmaengine channel object members
 * @device: parent device
 * @ch_regs: memory mapped register base
 * @mask: channel index in a mask
 * @per_if: peripheral interface
 * @mem_if: memory interface
 * @status: transmit status information from irq/prep* functions
 *                to tasklet (use atomic operations)
 * @tasklet: bottom half to finish transaction work
 * @save_cfg: configuration register that is saved on suspend/resume cycle
 * @save_dscr: for cyclic operations, preserve next descriptor address in
 *             the cyclic list on suspend/resume cycle
 * @dma_sconfig: configuration for slave transfers, passed via DMA_SLAVE_CONFIG
 * @lock: serializes enqueue/dequeue operations to descriptors lists
 * @active_list: list of descriptors dmaengine is being running on
 * @queue: list of descriptors ready to be submitted to engine
 * @free_list: list of descriptors usable by the channel
 * @descs_allocated: records the actual size of the descriptor pool
 */
struct ls_dma_chan {
    struct dma_chan chan_common;
    struct at_dma *device;
    void __iomem *ch_regs;
    u8 mask;
    u8 per_if;
    u8 mem_if;
    unsigned long status;
    struct tasklet_struct tasklet;
    u32 save_cfg;
    u32 save_dscr;
    struct dma_slave_config dma_sconfig;

    spinlock_t lock;

    /* these other elements are all protected by lock */
    struct list_head active_list;
    struct list_head queue;
    struct list_head free_list;
    unsigned int descs_allocated;
};

/**
 * struct ls_apbdma - internal representation of an Loongson APBDMA Controller
 * @chan_common: common dmaengine dma_device object members
 * @apbdma_devtype: identifier of DMA controller compatibility
 * @ch_regs: memory mapped register base
 * @clk: dma controller clock
 * @save_imr: interrupt mask register that is saved on suspend/resume cycle
 * @all_chan_mask: all channels availlable in a mask
 * @dma_desc_pool: base of DMA descriptor region (DMA address)
 * @chan: channels table to store ls_dma_chan structures
 */
struct ls_apbdma {
    struct dma_device   dma_common;
    void __iomem        *regs;
    struct clk      	*clk;
    u32         		save_imr;
    u8          		all_chan_mask;
    struct dma_pool     *dma_desc_pool;
    /* THE END channels table */
    struct ls_dma_chan  chan[0];
};

/**
 * struct ls_dma_platform_data - Controller configuration parameters
 * @nr_channels: Number of channels supported by hardware (max 8)
 * @dma_sel: Order number of APB sel region(max 9)
 * @apb_dma_nr: Order number of APB DMA controller(max 4)
 * @common:Pointer to the APB DMA common platform device
 * @is_private: The device channels should be marked as private and not for
 *	by the general purpose DMA channel allocator.
 * @chan_allocation_order: Allocate channels starting from 0 or 7
 * @chan_priority: Set channel priority increasing from 0 to 7 or 7 to 0.
 * @block_size: Maximum block size supported by the controller
 * @nr_masters: Number of APB masters supported by the controller
 * @data_width: Maximum data width supported by hardware per APB master
 * @sd: slave specific data. Used for configuring channels
 * @sd_count: count of slave data structures passed.
 */
struct ls_apbdma_platform_data {
	unsigned int			nr_channels;
	unsigned int			dma_sel;
	unsigned int			apb_dma_nr;
	struct platform_device 	*common;
	bool					is_private;
#define CHAN_ALLOCATION_ASCENDING	0	/* zero to seven */
#define CHAN_ALLOCATION_DESCENDING	1	/* seven to zero */
	unsigned char			chan_allocation_order;
#define CHAN_PRIORITY_ASCENDING		0	/* chan0 highest */
#define CHAN_PRIORITY_DESCENDING	1	/* chan7 highest */
	unsigned char			chan_priority;
	unsigned short			block_size;
	unsigned char			nr_masters;
	unsigned char			data_width[4];
	struct regmap			*regmap;
	unsigned int			conf_reg_mask;
	unsigned int			conf_reg_value;
};

static inline struct ls_dma_chan *to_ls_dma_chan(struct dma_chan *dchan)
{
    return container_of(dchan, struct ls_dma_chan, chan_common);
}

#endif /* LS_APB_DMA_H */
