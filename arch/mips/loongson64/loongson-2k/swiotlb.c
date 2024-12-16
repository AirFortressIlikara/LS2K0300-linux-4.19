/*
 * swiotlb.c - Add loongson-2K swiotlb support.
 *
 * Copyright (C) 2017, 2020, Loongson Technology Corporation Limited, Inc.
 *
 * Authors Pei Huang <huangpei@loongson.cn>
 * Authors Ming Wang <wangming01@loongson.cn>
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/dma-direct.h>
#include <linux/init.h>
#include <linux/sizes.h>
#include <linux/dma-direct.h>
#include <linux/dma-mapping.h>
#include <linux/dma-noncoherent.h>
#include <linux/scatterlist.h>
#include <linux/swiotlb.h>

#include <asm/bootinfo.h>
#include <asm/dma-coherence.h>
#include <loongson-pch.h>

static inline void *dma_to_virt(struct device *dev,
	dma_addr_t dma_addr)
{
	return phys_to_virt(__dma_to_phys(dev, dma_addr));
}

static void *loongson_dma_alloc_coherent(struct device *dev, size_t size,
				dma_addr_t *dma_handle, gfp_t gfp, unsigned long attrs)
{
	void *ret;

	/* ignore region specifiers */
	gfp &= ~(__GFP_DMA | __GFP_DMA32 | __GFP_HIGHMEM);

#ifdef CONFIG_ZONE_DMA
	if (dev == NULL)
		gfp |= __GFP_DMA;
	else if (dev->coherent_dma_mask <= DMA_BIT_MASK(24))
		gfp |= __GFP_DMA;
	else
		;
#endif

#ifdef CONFIG_ZONE_DMA32
	if (dev == NULL)
		gfp |= __GFP_DMA32;
	else if (dev->coherent_dma_mask <= DMA_BIT_MASK(32))
		gfp |= __GFP_DMA32;
	else
		;
#endif

	gfp |= __GFP_NORETRY | __GFP_NOWARN;

	ret = swiotlb_alloc(dev, size, dma_handle, gfp, attrs);

	if (!dev_is_coherent(dev)) {
		dma_cache_wback_inv((unsigned long)dma_to_virt(dev, *dma_handle), size);
		ret = (void *)UNCAC_ADDR(ret);
	}

	mb();

	return ret;
}

static void loongson_dma_free_coherent(struct device *dev, size_t size,
				void *vaddr, dma_addr_t dma_handle, unsigned long attrs)
{
	if (!dev_is_coherent(dev)) {
		vaddr = (void *)CAC_ADDR(vaddr);
		dma_cache_wback_inv((unsigned long)dma_to_virt(dev, dma_handle), size);
	}

	swiotlb_free(dev, size, vaddr, dma_handle, attrs);
}

static dma_addr_t loongson_dma_map_page(struct device *dev, struct page *page,
				unsigned long offset, size_t size,
				enum dma_data_direction dir,
				unsigned long attrs)
{
	dma_addr_t daddr;

	daddr = swiotlb_map_page(dev, page, offset, size, dir, attrs);
	if (!dev_is_coherent(dev) && !(attrs & DMA_ATTR_SKIP_CPU_SYNC))
		dma_sync_virt(dma_to_virt(dev, daddr), size, dir);
	mb();

	return daddr;
}

static void loongson_dma_unmap_page(struct device *dev, dma_addr_t dev_addr,
			size_t size, enum dma_data_direction dir,
			unsigned long attrs)
{
	if (!dev_is_coherent(dev) && !(attrs & DMA_ATTR_SKIP_CPU_SYNC))
		dma_sync_virt(dma_to_virt(dev, dev_addr), size, dir);

	swiotlb_unmap_page(dev, dev_addr, size, dir, attrs);
}

static int loongson_dma_map_sg(struct device *dev, struct scatterlist *sgl,
				int nents, enum dma_data_direction dir,
				unsigned long attrs)
{
	struct scatterlist *sg;
	int i;
	int r = swiotlb_map_sg_attrs(dev, sgl, nents, dir, attrs);

	if (!dev_is_coherent(dev) && !(attrs & DMA_ATTR_SKIP_CPU_SYNC)) {
		for_each_sg(sgl, sg, nents, i)
			dma_sync_virt(dma_to_virt(dev, sg->dma_address), sg->length, dir);
	}

	mb();

	return r;
}

static void loongson_dma_unmap_sg(struct device *dev, struct scatterlist *sgl,
			int nelems, enum dma_data_direction dir,
			unsigned long attrs)
{
	int i;
	struct scatterlist *sg;

	if (!dev_is_coherent(dev) && !(attrs & DMA_ATTR_SKIP_CPU_SYNC) && dir != DMA_TO_DEVICE) {
		for_each_sg(sgl, sg, nelems, i)
			dma_sync_virt(dma_to_virt(dev, sg->dma_address), sg->length, dir);
	}

	swiotlb_unmap_sg_attrs(dev, sgl, nelems, dir, attrs);
}

static void loongson_dma_sync_single_for_cpu(struct device *dev, dma_addr_t dev_addr,
			    size_t size, enum dma_data_direction dir)
{
	if (!dev_is_coherent(dev))
		dma_sync_virt(dma_to_virt(dev, dev_addr), size, dir);


	swiotlb_sync_single_for_cpu(dev, dev_addr, size, dir);
}

static void loongson_dma_sync_single_for_device(struct device *dev,
				dma_addr_t dma_handle, size_t size,
				enum dma_data_direction dir)
{
	swiotlb_sync_single_for_device(dev, dma_handle, size, dir);
	if (!dev_is_coherent(dev))
		dma_sync_virt(dma_to_virt(dev, dma_handle), size, dir);
	mb();
}

static void loongson_dma_sync_sg_for_cpu(struct device *dev, struct scatterlist *sgl,
			int nelems, enum dma_data_direction dir)
{
	struct scatterlist *sg;
	int i;

	if (!dev_is_coherent(dev)) {
		for_each_sg(sgl, sg, nelems, i) {
			dma_cache_sync(dev, dma_to_virt(dev,
					sg_dma_address(sg)), sg_dma_len(sg), dir);
		}
	}

	swiotlb_sync_sg_for_cpu(dev, sgl, nelems, dir);
}

static void loongson_dma_sync_sg_for_device(struct device *dev,
				struct scatterlist *sgl, int nents,
				enum dma_data_direction dir)
{
	struct scatterlist *sg;
	int i;

	swiotlb_sync_sg_for_device(dev, sgl, nents, dir);
	if (!dev_is_coherent(dev)) {
		for_each_sg(sgl, sg, nents, i) {
			dma_cache_sync(dev, dma_to_virt(dev,
					sg_dma_address(sg)), sg_dma_len(sg), dir);
		}
	}

	mb();
}

static int loongson_dma_mmap(struct device *dev, struct vm_area_struct *vma,
	void *cpu_addr, dma_addr_t dma_addr, size_t size, unsigned long attrs)
{
	int ret = -ENXIO;
	unsigned long user_count = vma_pages(vma);
	unsigned long count = PAGE_ALIGN(size) >> PAGE_SHIFT;
	unsigned long pfn = page_to_pfn(virt_to_page(cpu_addr));
	unsigned long off = vma->vm_pgoff;

	if (!dev_is_coherent(dev)) {
		if (attrs & DMA_ATTR_WRITE_COMBINE)
			vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
		else
			vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	} else {
		unsigned long prot = pgprot_val(vma->vm_page_prot);

		prot = (prot & ~_CACHE_MASK) | _page_cachable_default;
		vma->vm_page_prot = __pgprot(prot);
	}

	if (dma_mmap_from_dev_coherent(dev, vma, cpu_addr, size, &ret))
		return ret;

	if (off < count && user_count <= (count - off)) {
		ret = remap_pfn_range(vma, vma->vm_start, pfn + off,
				      user_count << PAGE_SHIFT, vma->vm_page_prot);
	}

	return ret;
}

const struct dma_map_ops loongson_dma_ops = {
		.alloc = loongson_dma_alloc_coherent,
		.free = loongson_dma_free_coherent,
		.mmap = loongson_dma_mmap,
		.map_page = loongson_dma_map_page,
		.unmap_page = loongson_dma_unmap_page,
		.map_sg = loongson_dma_map_sg,
		.unmap_sg = loongson_dma_unmap_sg,
		.sync_single_for_cpu = loongson_dma_sync_single_for_cpu,
		.sync_single_for_device = loongson_dma_sync_single_for_device,
		.sync_sg_for_cpu = loongson_dma_sync_sg_for_cpu,
		.sync_sg_for_device = loongson_dma_sync_sg_for_device,
		.dma_supported = swiotlb_dma_supported,
		.cache_sync = arch_dma_cache_sync,
		.mapping_error = swiotlb_dma_mapping_error,
};
EXPORT_SYMBOL(loongson_dma_ops);

static char *vstart;
static size_t swiotlbsize;
#define SLABS_PER_PAGE (1 << (PAGE_SHIFT - IO_TLB_SHIFT))
#define IO_TLB_MIN_SLABS ((1<<20) >> IO_TLB_SHIFT)

static __init void ls2k_swiotlb_init(int verbose)
{
	unsigned long swiotlb_nslabs;
	swiotlbsize = 64 * (1<<20);
	swiotlb_nslabs = swiotlbsize >> IO_TLB_SHIFT;
	swiotlb_nslabs = ALIGN(swiotlb_nslabs, IO_TLB_SEGSIZE);
	swiotlbsize = swiotlb_nslabs << IO_TLB_SHIFT;

	/* Get IO TLB memory from the low pages */
	vstart = alloc_bootmem_low_pages_nopanic(PAGE_ALIGN(swiotlbsize));
	if (vstart && !swiotlb_init_with_tbl(vstart, swiotlb_nslabs, verbose))
		return;

	if (vstart)
		free_bootmem(virt_to_phys(vstart),
				 PAGE_ALIGN(swiotlbsize));
	vstart = NULL;

	pr_warn("Cannot allocate buffer");
}

int swiotlb_late_init_with_default_size(size_t default_size);

static int __init late_swiotlb_setup(void)
{
	unsigned long swiotlb_nslabs;
	unsigned int order;
	int rc = 0;

	if (vstart)
		return 0;

	/*
	 * Get IO TLB memory from the low pages
	 */
	order = get_order(swiotlbsize);
	swiotlb_nslabs = SLABS_PER_PAGE << order;
	swiotlbsize = swiotlb_nslabs << IO_TLB_SHIFT;

	while ((SLABS_PER_PAGE << order) > IO_TLB_MIN_SLABS) {
		vstart = (void *)__get_free_pages(GFP_DMA32 | __GFP_NOWARN,
						  order);
		if (vstart)
			break;
		order--;
	}

	if (!vstart) {
		return -ENOMEM;
	}
	if (order != get_order(swiotlbsize)) {
		pr_warn("only able to allocate %ld MB\n",
			(PAGE_SIZE << order) >> 20);
		swiotlb_nslabs = SLABS_PER_PAGE << order;
	}
	rc = swiotlb_late_init_with_tbl(vstart, swiotlb_nslabs);
	if (rc)
		free_pages((unsigned long)vstart, order);

	return rc;
}

__define_initcall(late_swiotlb_setup, rootfss);

void __init plat_swiotlb_setup(void)
{
	pr_info("swiotlb:restricted 32bit dma!\n");

	ls2k_swiotlb_init(1);
}
