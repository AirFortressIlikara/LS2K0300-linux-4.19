// SPDX-License-Identifier: GPL-2.0
/*
 * Loongson PCI Host Controller Driver
 *
 * Copyright (C) 2020 Chong Qiao <qiaochong@loongson.cn>
 */

#include <linux/of_device.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/pci_regs.h>
#include <linux/of_gpio.h>
#include <asm/cacheflush.h>
#include <linux/swiotlb.h>

#include "../pci.h"

#define FLAG_CFG0	BIT(0)
#define FLAG_CFG1	BIT(1)
#define FLAG_DEV_FIX	BIT(2)

struct loongson_pci {
	void __iomem *reg_base;
	void __iomem *pciheader_base;
	void __iomem *pcicfg_base;
	struct platform_device *pdev;
	struct dma_domain dma_domain;
};

#define OFFS_PXARB_CFG 		0x0c
#define OFFS_PXARB_STATUS 	0x10
#define OFFS_PCIMAP		0x14
#define OFFS_PCIMAP_CFG		0x20

#define ID_SEL_BEGIN 11
#define MAX_DEV_NUM (31 - ID_SEL_BEGIN)
#define LOONGSON_PCICMD_MABORT_CLR	0x20000000
#define LOONGSON_PCICMD_MTABORT_CLR	0x10000000
static int pci_irq;

static void __iomem *ppci_loongson_map_bus(struct pci_bus *bus, unsigned int devfn,
			       int where)
{
	unsigned char busnum = bus->number;
	int device = PCI_SLOT(devfn);
	int function = PCI_FUNC(devfn);
	u32 addr, type;
	u32 dummy;
	int reg = where & ~3;
	void *addrp;
	struct pci_host_bridge *bridge = pci_find_host_bridge(bus);
	struct loongson_pci *priv =  pci_host_bridge_priv(bridge);

	/*
	 * Do not read more than one device on the bus other than
	 * the host bus. For our hardware the root bus is always bus 0.
	 */
	if (!bus->parent)
		busnum = 0;

	if (busnum == 0) {
		/* Type 0 configuration for onboard PCI bus */
		if (device > MAX_DEV_NUM)
			return NULL;

		addr = (1 << (device + ID_SEL_BEGIN)) | (function << 8) | reg;
		type = 0;
	} else {
		/* Type 1 configuration for offboard PCI bus */
		addr = (busnum << 16) | (device << 11) | (function << 8) | reg;
		type = 0x10000;
	}

	/* Clear aborts */
	dummy = readl(priv->pciheader_base + PCI_COMMAND);
	dummy |= LOONGSON_PCICMD_MABORT_CLR | \
				LOONGSON_PCICMD_MTABORT_CLR;

	writel(dummy, priv->pciheader_base + PCI_COMMAND);
	writel((addr >> 16) | type, priv->reg_base + OFFS_PCIMAP_CFG);

	/* Flush Bonito register block */
	(void)readl(priv->reg_base + OFFS_PCIMAP_CFG);
	mmiowb();

	addrp = priv->pcicfg_base + (addr & 0xffff);
	return addrp;
}

/* H/w only accept 32-bit PCI operations */
static struct pci_ops loongson_ppci_ops = {
	.map_bus = ppci_loongson_map_bus,
	.read	= pci_generic_config_read32,
	.write	= pci_generic_config_write32,
};

static const struct of_device_id loongson_ppci_of_match[] = {
	{ .compatible = "loongson,ls2k500-pci",
		.data = NULL, },
	{}
};

static phys_addr_t ls2k500_dma_to_phys(struct device *dev, dma_addr_t daddr)
{
	return (daddr > 0x8fffffffUL) ? daddr : (daddr & 0x0fffffff);
}
dma_addr_t ls2k500_phys_to_dma(struct device *dev, phys_addr_t paddr)
{
	return paddr|0x80000000;
}

#define CACHELINE_SIZE (1 << CONFIG_L1_CACHE_SHIFT)
static void ls_dma_cache_wback_inv(unsigned long addr, int size)
{
	int i;
	for (i = 0; i < size; i += CACHELINE_SIZE)
		flush_cache_line_hit(addr + i);
}

static void ls_dma_cache_inv(unsigned long addr, int size)
{
	int i;
	for (i = 0; i < size; i += CACHELINE_SIZE)
		invalid_cache_line_hit(addr + i);
}

static inline void dma_sync_virtual(struct device *dev, void *addr, size_t size,
	enum dma_data_direction direction)
{
	switch (direction) {
	case DMA_TO_DEVICE:
		ls_dma_cache_wback_inv((unsigned long)addr, size);
		break;

	case DMA_FROM_DEVICE:
		ls_dma_cache_inv((unsigned long)addr, size);
		break;

	case DMA_BIDIRECTIONAL:
		ls_dma_cache_wback_inv((unsigned long)addr, size);
		break;

	default:
		BUG();
	}
}


static inline void *dma_to_virt(struct device *dev, dma_addr_t dma_addr)
{
	return phys_to_virt(ls2k500_dma_to_phys(dev, dma_addr));
}

static inline void *dma_to_virt1(struct device *dev, dma_addr_t dma_addr)
{
	return phys_to_virt(ls2k500_dma_to_phys(dev, dma_addr)|0x80000000);
}


static void *ls2k500_pci_dma_alloc_coherent(struct device *dev, size_t size,
				dma_addr_t *dma_handle, gfp_t gfp, unsigned long attrs)
{
	void *ret;
	ret = swiotlb_alloc(dev, size, dma_handle, gfp, attrs);
	*dma_handle = ls2k500_phys_to_dma(dev, *dma_handle);

	if (*dma_handle < 0x90000000UL) {
		ret = dma_to_virt1(dev, *dma_handle);
		ls_dma_cache_wback_inv((unsigned long)dma_to_virt(dev, *dma_handle), size);
	}
	mb();

	return ret;
}

static void ls2k500_pci_dma_free_coherent(struct device *dev, size_t size,
				void *vaddr, dma_addr_t dma_handle, unsigned long attrs)
{
	if (dma_handle < 0x90000000UL) {
		ls_dma_cache_wback_inv((unsigned long)dma_to_virt1(dev, dma_handle), size);
		ls_dma_cache_inv((unsigned long)dma_to_virt(dev, dma_handle), size);
	}

	vaddr = dma_to_virt(dev, dma_handle);
	swiotlb_free(dev, size, vaddr, dma_handle, attrs);
}

static dma_addr_t ls2k500_pci_dma_map_page(struct device *dev, struct page *page,
				unsigned long offset, size_t size,
				enum dma_data_direction dir,
				unsigned long attrs)
{
	dma_addr_t daddr;

	daddr = swiotlb_map_page(dev, page, offset, size, dir, attrs);

	if (daddr < 0x90000000UL) {
		dma_sync_virtual(dev, dma_to_virt1(dev, daddr), size, DMA_FROM_DEVICE);
		dma_sync_virtual(dev, dma_to_virt(dev, daddr), size, dir);
	}
	mb();

	return daddr;
}

static void ls2k500_pci_dma_unmap_page(struct device *dev, dma_addr_t dev_addr,
			size_t size, enum dma_data_direction dir,
			unsigned long attrs)
{
	if (dev_addr < 0x90000000UL) {
		dma_sync_virtual(dev, dma_to_virt1(dev, dev_addr), size, DMA_FROM_DEVICE);
		dma_sync_virtual(dev, dma_to_virt(dev, dev_addr), size, dir);
	}
	swiotlb_unmap_page(dev, dev_addr, size, dir, attrs);
}

static int ls2k500_pci_dma_map_sg(struct device *dev, struct scatterlist *sgl,
				int nents, enum dma_data_direction dir,
				unsigned long attrs)
{
	int i, r;
	struct scatterlist *sg;

	r = swiotlb_map_sg_attrs(dev, sgl, nents, dir,
					attrs);
	for_each_sg(sgl, sg, nents, i) {
		if (sg->dma_address < 0x90000000UL) {
			dma_sync_virtual(dev, dma_to_virt1(dev, sg->dma_address), sg->length, DMA_FROM_DEVICE);
			dma_sync_virtual(dev, dma_to_virt(dev, sg->dma_address), sg->length, dir);
		}
	}
	mb();

	return r;
}

static void ls2k500_pci_dma_unmap_sg(struct device *dev, struct scatterlist *sgl,
			int nelems, enum dma_data_direction dir,
			unsigned long attrs)
{
	int i;
	struct scatterlist *sg;

	if (dir != DMA_TO_DEVICE) {
		for_each_sg(sgl, sg, nelems, i) {
			if (sg->dma_address < 0x90000000UL) {
				dma_sync_virtual(dev, dma_to_virt1(dev, sg->dma_address), sg->length, DMA_FROM_DEVICE);
				dma_sync_virtual(dev, dma_to_virt(dev, sg->dma_address), sg->length, dir);
			}
		}
	}

	swiotlb_unmap_sg_attrs(dev, sgl, nelems, dir, attrs);
}

static void ls2k500_pci_dma_sync_single_for_cpu(struct device *dev, dma_addr_t dev_addr,
			size_t size, enum dma_data_direction dir)
{
	if (dev_addr < 0x90000000UL) {
		dma_sync_virtual(dev, dma_to_virt1(dev, dev_addr), size, DMA_FROM_DEVICE);
		dma_sync_virtual(dev, dma_to_virt(dev, dev_addr), size, dir);
	}
	swiotlb_sync_single_for_cpu(dev, dev_addr, size, dir);
}

static void ls2k500_pci_dma_sync_single_for_device(struct device *dev,
				dma_addr_t dma_handle, size_t size,
				enum dma_data_direction dir)
{
	swiotlb_sync_single_for_device(dev, dma_handle, size, dir);
	if (dma_handle < 0x90000000UL) {
	dma_sync_virtual(dev, dma_to_virt1(dev, dma_handle), size, DMA_FROM_DEVICE);
	dma_sync_virtual(dev, dma_to_virt(dev, dma_handle), size, dir);
	}
	mb();
}

static void ls2k500_pci_dma_sync_sg_for_cpu(struct device *dev,
				struct scatterlist *sgl, int nents,
				enum dma_data_direction dir)
{
	int i;
	struct scatterlist *sg;

	for_each_sg(sgl, sg, nents, i) {
		if (sg->dma_address < 0x90000000UL) {
		dma_sync_virtual(dev, dma_to_virt1(dev,
					sg->dma_address), sg->length, DMA_FROM_DEVICE);
		dma_sync_virtual(dev, dma_to_virt(dev,
					sg->dma_address), sg->length, dir);
	}
	}
	swiotlb_sync_sg_for_cpu(dev, sgl, nents, dir);
}

static void ls2k500_pci_dma_sync_sg_for_device(struct device *dev,
				struct scatterlist *sgl, int nents,
				enum dma_data_direction dir)
{
	int i;
	struct scatterlist *sg;

	swiotlb_sync_sg_for_device(dev, sgl, nents, dir);
	for_each_sg(sgl, sg, nents, i) {
		if (sg->dma_address < 0x90000000UL) {
		dma_sync_virtual(dev, dma_to_virt1(dev,
					sg->dma_address), sg->length, DMA_FROM_DEVICE);
		dma_sync_virtual(dev, dma_to_virt(dev,
					sg->dma_address), sg->length, dir);
	}
	}
	mb();
}

static void ls2k500_pci_dma_cache_sync(struct device *dev, void *vaddr, size_t size,
		enum dma_data_direction direction)
{
	dma_addr_t daddr;
	BUG_ON(direction == DMA_NONE);

	daddr = ls2k500_phys_to_dma(dev, virt_to_phys(vaddr));
	if (daddr < 0x90000000UL) {
		dma_sync_virtual(dev, dma_to_virt1(dev, daddr), size, DMA_FROM_DEVICE);
		dma_sync_virtual(dev, dma_to_virt(dev, daddr), size, direction);
	}
}

static struct dma_map_ops ls2k500_pci_dma_map_ops = {
		.alloc = ls2k500_pci_dma_alloc_coherent,
		.free = ls2k500_pci_dma_free_coherent,
		.map_page = ls2k500_pci_dma_map_page,
		.unmap_page = ls2k500_pci_dma_unmap_page,
		.map_sg = ls2k500_pci_dma_map_sg,
		.unmap_sg = ls2k500_pci_dma_unmap_sg,
		.sync_single_for_cpu = ls2k500_pci_dma_sync_single_for_cpu,
		.sync_single_for_device = ls2k500_pci_dma_sync_single_for_device,
		.sync_sg_for_cpu = ls2k500_pci_dma_sync_sg_for_cpu,
		.sync_sg_for_device = ls2k500_pci_dma_sync_sg_for_device,
		.mapping_error = swiotlb_dma_mapping_error,
		.dma_supported = swiotlb_dma_supported,
		.cache_sync		= ls2k500_pci_dma_cache_sync,
};

static int loongson_ppci_probe(struct platform_device *pdev)
{
	struct loongson_pci *priv;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct pci_host_bridge *bridge;
	struct resource *regs;
	struct resource *bus_range;
	struct dma_domain *domain;
	LIST_HEAD(pci_res);
	int gpio, ret, i;
	if (!node)
		return -ENODEV;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*priv));
	if (!bridge)
		return -ENODEV;

	priv = pci_host_bridge_priv(bridge);
	priv->pdev = pdev;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(dev, "missing mem resources for cfg0\n");
		return -EINVAL;
	}

	priv->pcicfg_base = devm_pci_remap_cfg_resource(dev, regs);
	if (IS_ERR(priv->pcicfg_base))
		return PTR_ERR(priv->pcicfg_base);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!regs)
		dev_info(dev, "missing mem resource for cfg1\n");
	else {
		priv->pciheader_base = devm_pci_remap_cfg_resource(dev, regs);
		if (IS_ERR(priv->pciheader_base))
			priv->pciheader_base = NULL;
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!regs)
		dev_info(dev, "missing mem resource for cfg1\n");
	else {
		priv->reg_base = devm_pci_remap_cfg_resource(dev, regs);
		if (IS_ERR(priv->reg_base))
			priv->reg_base = NULL;
	}

	i = of_gpio_named_count(node, "pci-gpios");
	while (i) {
		gpio = of_get_named_gpio(node, "pci-gpios", i - 1);
		if (gpio_is_valid(gpio)) {
			int err;
			struct irq_data *data;
			struct irq_chip *chip;

			err = devm_gpio_request(&pdev->dev, gpio,
			"pci-irq-gpio");
			if (err) {
				dev_err(&pdev->dev,
					"can't request vbus gpio %d, err: %d\n",
					gpio, err);
				return err;
			}
			gpio_direction_input(gpio);
			pci_irq = gpio_to_irq(gpio);
			data = irq_get_irq_data(pci_irq);
			chip = irq_data_get_irq_chip(data);
			if (chip && chip->irq_set_type)
				chip->irq_set_type(data, IRQ_TYPE_LEVEL_LOW);
		}
		i--;
	}

	if (pci_parse_request_of_pci_ranges(&pdev->dev, &pci_res, &bus_range))
		return -ENODEV;

	list_splice_init(&pci_res, &bridge->windows);
	bridge->dev.parent = dev;
	bridge->dev.of_node = node;
	bridge->sysdata = priv;
	bridge->ops = &loongson_ppci_ops;
	bridge->map_irq = of_irq_parse_and_map_pci;
	bridge->busnr = bus_range->start - 1;

	ret = pci_host_probe(bridge);

	if (!ret) {
		domain = &priv->dma_domain;
		domain->domain_nr = pci_domain_nr(bridge->bus);
		domain->dma_ops = &ls2k500_pci_dma_map_ops;
		add_dma_domain(domain);
	}
	return ret;
}

static struct platform_driver loongson_ppci_driver = {
	.driver = {
		.name = "loongson,ls2k500-pci",
		.of_match_table = loongson_ppci_of_match,
	},
	.probe = loongson_ppci_probe,
};
builtin_platform_driver(loongson_ppci_driver);
