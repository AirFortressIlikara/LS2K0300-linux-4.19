/*
 * Arch specific extensions to struct device
 *
 * This file is released under the GPLv2
 */
#ifndef _ASM_MIPS_DEVICE_H
#define _ASM_MIPS_DEVICE_H

struct dev_archdata {
	unsigned long dma_attrs;
#ifdef CONFIG_DMA_PERDEV_COHERENT
	/* Non-zero if DMA is coherent with CPU caches */
	bool dma_coherent;
#endif
#if defined(CONFIG_LOONGSON_IOMMU)
	/* hook for IOMMU specific extension */
	void *iommu;
#endif
	bool cpu_device;
};

struct pdev_archdata {
};

#endif /* _ASM_MIPS_DEVICE_H*/
