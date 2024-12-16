/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 *
 */
#ifndef __ASM_DMA_H
#define __ASM_DMA_H

#include <linux/dma-direction.h>
#include <linux/dma-contiguous.h>

#define MAX_DMA_ADDRESS	PAGE_OFFSET
#define MAX_DMA32_PFN	(1UL << (32 - PAGE_SHIFT))

extern int isa_dma_bridge_buggy;

static inline int dev_is_coherent(struct device *dev)
{
	return 1;
}

static inline void dma_sync_virt(void *addr, size_t size,
		enum dma_data_direction dir)
{
	switch (dir) {
	case DMA_TO_DEVICE:
		dma_cache_wback((unsigned long)addr, size);
		break;

	case DMA_FROM_DEVICE:
		dma_cache_inv((unsigned long)addr, size);
		break;

	case DMA_BIDIRECTIONAL:
		dma_cache_wback_inv((unsigned long)addr, size);
		break;

	default:
		BUG();
	}
}

#endif
