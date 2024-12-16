// SPDX-License-Identifier: GPL-2.0
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 *
 */
#include <linux/dma-direct.h>
#include <linux/init.h>
#include <linux/sizes.h>
#include <linux/acpi.h>
#include <linux/dma-direct.h>
#include <linux/dma-mapping.h>
#include <linux/dma-noncoherent.h>
#include <linux/scatterlist.h>
#include <linux/swiotlb.h>

#include <asm/dma.h>
#include <asm/numa.h>
#include <loongson.h>
#include <loongson-pch.h>
#include <linux/acpi.h>

#ifdef CONFIG_ACPI
static int acpi_dma_get_ranges(struct device *dev, struct bus_dma_region **map)
{
	struct acpi_device *adev;
	LIST_HEAD(list);
	struct resource_entry *rentry;
	int ret;
	struct device *dma_dev = dev;
	struct bus_dma_region *r;

	/*
	 * Walk the device tree chasing an ACPI companion with a _DMA
	 * object while we go. Stop if we find a device with an ACPI
	 * companion containing a _DMA method.
	 */
	do {
		adev = ACPI_COMPANION(dma_dev);
		if (adev && acpi_has_method(adev->handle, METHOD_NAME__DMA))
			break;

		dma_dev = dma_dev->parent;
	} while (dma_dev);

	if (!dma_dev)
		return -ENODEV;

	if (!acpi_has_method(adev->handle, METHOD_NAME__CRS)) {
		acpi_handle_warn(adev->handle, "_DMA is valid only if _CRS is present\n");
		return -EINVAL;
	}

	ret = acpi_dev_get_dma_resources(adev, &list);
	if (ret > 0) {
		r = kcalloc(ret + 1, sizeof(*r), GFP_KERNEL);
		if (!r) {
			ret = -ENOMEM;
			goto out;
		}

		*map = r;

		list_for_each_entry(rentry, &list, node) {
			if (rentry->res->start >= rentry->res->end) {
				ret = -EINVAL;
				dev_dbg(dma_dev, "Invalid DMA regions configuration\n");
				goto out;
			}

			r->cpu_start = rentry->res->start;
			r->dma_start = rentry->res->start - rentry->offset;
			r->size = rentry->res->end - rentry->res->start + 1;
			r->offset = rentry->offset;
			r++;
		}

	}
 out:
	acpi_dev_free_resource_list(&list);

	return ret >= 0 ? 0 : ret;
}

void acpi_dma_map_setup(struct device *dev)
{
	int ret;
	u64 mask, end = 0;
	struct bus_dma_region *map = NULL;

	ret = acpi_dma_get_ranges(dev, &map);
	if (!ret && map) {
		struct bus_dma_region *r = map;

		for (end = 0; r->size; r++) {
			if (r->dma_start + r->size - 1 > end)
				end = r->dma_start + r->size - 1;
		}

		mask = DMA_BIT_MASK(ilog2(end) + 1);
		dev->archdata.dma_range_map = map;
		dev->coherent_dma_mask = min(dev->coherent_dma_mask, mask);
		*dev->dma_mask = min(*dev->dma_mask, mask);
	}
}
#endif

int node_id_offset;

static inline void *dma_to_virt(struct device *dev, dma_addr_t dma_addr)
{
	return phys_to_virt(__dma_to_phys(dev, dma_addr));
}

static void *loongson_dma_alloc_coherent(struct device *dev, size_t size,
		dma_addr_t *dma_handle, gfp_t gfp, unsigned long attrs)
{
	void *ret = swiotlb_alloc(dev, size, dma_handle, gfp, attrs);

	return ret;
}

static void loongson_dma_free_coherent(struct device *dev, size_t size,
		void *vaddr, dma_addr_t dma_handle, unsigned long attrs)
{
	/* use dma address directly since dma addr == phy addr for swiotlb */
	if (!is_swiotlb_buffer(dma_handle))
		dma_direct_free(dev, size, vaddr, dma_handle, attrs);
	else
		swiotlb_free(dev, size, vaddr, dma_handle, attrs);
}

static int loongson_dma_mmap(struct device *dev, struct vm_area_struct *vma,
	void *cpu_addr, dma_addr_t dma_addr, size_t size, unsigned long attrs)
{
	int ret = -ENXIO;
	unsigned long user_count = vma_pages(vma);
	unsigned long count = PAGE_ALIGN(size) >> PAGE_SHIFT;
	unsigned long pfn = page_to_pfn(virt_to_page(cpu_addr));
	unsigned long off = vma->vm_pgoff;

	unsigned long prot = pgprot_val(vma->vm_page_prot);

	prot = (prot & ~_CACHE_MASK) | _CACHE_CC;
	vma->vm_page_prot = __pgprot(prot);

	if (dma_mmap_from_dev_coherent(dev, vma, cpu_addr, size, &ret))
		return ret;

	if (off < count && user_count <= (count - off)) {
		ret = remap_pfn_range(vma, vma->vm_start, pfn + off,
				      user_count << PAGE_SHIFT, vma->vm_page_prot);
	}

	return ret;
}

static dma_addr_t loongson_dma_map_page(struct device *dev, struct page *page,
				unsigned long offset, size_t size,
				enum dma_data_direction dir,
				unsigned long attrs)
{
	dma_addr_t daddr;

	daddr = swiotlb_map_page(dev, page, offset, size, dir, attrs);
	return daddr;
}

static void loongson_dma_unmap_page(struct device *dev, dma_addr_t dev_addr,
			size_t size, enum dma_data_direction dir,
			unsigned long attrs)
{
	swiotlb_unmap_page(dev, dev_addr, size, dir, attrs);
}

static int loongson_dma_map_sg(struct device *dev, struct scatterlist *sgl,
				int nents, enum dma_data_direction dir,
				unsigned long attrs)
{
	int  r;

	r = swiotlb_map_sg_attrs(dev, sgl, nents, dir, attrs);
	return r;
}

static void loongson_dma_unmap_sg(struct device *dev, struct scatterlist *sgl,
			int nelems, enum dma_data_direction dir,
			unsigned long attrs)
{
	swiotlb_unmap_sg_attrs(dev, sgl, nelems, dir, attrs);
}

static void loongson_dma_sync_single_for_cpu(struct device *dev, dma_addr_t dev_addr,
			size_t size, enum dma_data_direction dir)
{
	/* use dma address directly since dma addr == phy addr for swiotlb */
	if (is_swiotlb_buffer(dev_addr))
		swiotlb_sync_single_for_cpu(dev, dev_addr, size, dir);
}

static void loongson_dma_sync_single_for_device(struct device *dev,
				dma_addr_t dma_handle, size_t size,
				enum dma_data_direction dir)
{
	swiotlb_sync_single_for_device(dev, dma_handle, size, dir);
	/*
	 * There maybe exist write-buffer, device can not get cpu's write buffer
	 * need flush data from write-buffer to cache
	 */
	mb();
}

static void loongson_dma_sync_sg_for_cpu(struct device *dev,
				struct scatterlist *sgl, int nents,
				enum dma_data_direction dir)
{
	swiotlb_sync_sg_for_cpu(dev, sgl, nents, dir);
}

static void loongson_dma_sync_sg_for_device(struct device *dev,
				struct scatterlist *sgl, int nents,
				enum dma_data_direction dir)
{
	swiotlb_sync_sg_for_device(dev, sgl, nents, dir);
	/*
	 * There maybe exist write-buffer, device can not get cpu's write buffer
	 * need flush data from write-buffer to cache
	 */
	mb();
}

static int loongson_dma_supported(struct device *dev, u64 mask)
{
	return swiotlb_dma_supported(dev, mask);
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
	.dma_supported = loongson_dma_supported,
	.cache_sync = arch_dma_cache_sync,
	.mapping_error = swiotlb_dma_mapping_error,
};
EXPORT_SYMBOL(loongson_dma_ops);

void __init plat_swiotlb_setup(void)
{
	swiotlb_init(1);

	if (efi_bp) {
		if (loongson_sysconf.is_soc_cpu)
			node_id_offset = 0;
		else
			node_id_offset = ((readl(LS7A_DMA_CFG) & LS7A_DMA_NODE_MASK) >> LS7A_DMA_NODE_SHF) + 36;
	}
}
