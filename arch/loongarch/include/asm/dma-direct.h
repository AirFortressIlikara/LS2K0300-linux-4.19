/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 *
 */
#ifndef _LOONGARCH_DMA_DIRECT_H
#define _LOONGARCH_DMA_DIRECT_H 1

static inline bool dma_capable(struct device *dev, dma_addr_t addr, size_t size)
{
	if (!dev->dma_mask)
		return false;

	return addr + size - 1 <= *dev->dma_mask;
}

static inline dma_addr_t translate_phys_to_dma(struct device *dev,
		phys_addr_t paddr)
{
	const struct bus_dma_region *m;

	for (m = dev->archdata.dma_range_map; m->size; m++)
		if (paddr >= m->cpu_start && paddr - m->cpu_start < m->size)
			return (dma_addr_t)paddr - m->offset;

	/* make sure dma_capable fails when no translation is available */
	return (dma_addr_t)-1;
}

static inline phys_addr_t translate_dma_to_phys(struct device *dev,
		dma_addr_t dma_addr)
{
	const struct bus_dma_region *m;

	for (m = dev->archdata.dma_range_map; m->size; m++)
		if (dma_addr >= m->dma_start && dma_addr - m->dma_start < m->size)
			return (phys_addr_t)dma_addr + m->offset;

	return (phys_addr_t)-1;
}

static inline dma_addr_t __phys_to_dma(struct device *dev, phys_addr_t paddr)
{
	unsigned long mask;

	if (dev->archdata.dma_range_map)
		return translate_phys_to_dma(dev, paddr);
	else if (dev->archdata.dma_node_off) {
		mask = dev->archdata.dma_node_mask << dev->archdata.dma_node_off;
		return (paddr & ~mask) | ((paddr & mask) >> dev->archdata.dma_node_off);
	} else
		return (dma_addr_t)paddr;
}

static inline phys_addr_t __dma_to_phys(struct device *dev, dma_addr_t dev_addr)
{
	unsigned long mask;

	if (dev->archdata.dma_range_map)
		return translate_dma_to_phys(dev, dev_addr);
	else if (dev->archdata.dma_node_off) {
		mask = dev->archdata.dma_node_mask;
		return (dev_addr & ~mask) | ((dev_addr & mask) << dev->archdata.dma_node_off);
	} else
		return (phys_addr_t)dev_addr;
}

#endif /* _LOONGARCH_DMA_DIRECT_H */
