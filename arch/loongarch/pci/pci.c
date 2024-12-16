// SPDX-License-Identifier: GPL-2.0
/*
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#include <asm/cacheflush.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/acpi.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/of_address.h>
#include <loongson.h>
#include <loongson-pch.h>
static void set_dma_domain_ops(struct pci_dev *pdev);
extern struct fwnode_handle *msi_irqdomain_handle(int id);

int raw_pci_read(unsigned int domain, unsigned int bus, unsigned int devfn,int reg, int len, u32 *val)
{
	struct pci_bus * bus_tmp = pci_find_bus(domain, bus);
	if (bus_tmp)
		return bus_tmp->ops->read(bus_tmp, devfn, reg, len, val);
	return -EINVAL;
}

int raw_pci_write(unsigned int domain, unsigned int bus, unsigned int devfn,
						int reg, int len, u32 val)
{
	struct pci_bus * bus_tmp = pci_find_bus(domain, bus);
	if (bus_tmp)
		return bus_tmp->ops->write(bus_tmp, devfn, reg, len, val);
	return -EINVAL;
}

static int __init pcibios_init(void)
{
	unsigned int lsize;

	/*
	 * Set PCI cacheline size to that of the last level in the
	 * cache hierarchy.
	 */
	lsize = cpu_last_level_cache_line_size();

	BUG_ON(!lsize);

	pci_dfl_cache_line_size = lsize >> 2;

	pr_debug("PCI: pci_cache_line_size set to %d bytes\n", lsize);
	return 0;
}

void pci_resource_to_user(const struct pci_dev *dev, int bar,
			  const struct resource *rsrc, resource_size_t *start,
			  resource_size_t *end)
{
	phys_addr_t size = resource_size(rsrc);

	*start = rsrc->start;
	*end = rsrc->start + size - 1;
}

phys_addr_t mcfg_addr_init(int node)
{
	return (((u64) node << 44) | MCFG_EXT_PCICFG_BASE);
}

subsys_initcall(pcibios_init);

int pcibios_dev_init(struct pci_dev *dev)
{
	set_dma_domain_ops(dev);
#ifdef CONFIG_ACPI
	if (acpi_disabled)
		return 0;
	if (pci_dev_msi_enabled(dev))
		return 0;
	return acpi_pci_irq_enable(dev);
#endif
}

int pcibios_enable_device(struct pci_dev *dev, int mask)
{
	int err;

	err = pci_enable_resources(dev, mask);
	if (err < 0)
		return err;

	return pcibios_dev_init(dev);
}

extern int node_id_offset;
int pcibios_add_device(struct pci_dev *dev)
{
	struct irq_domain *msi_domain;
	int id = pci_domain_nr(dev->bus);

	msi_domain = irq_find_matching_fwnode(msi_irqdomain_handle(id), DOMAIN_BUS_PCI_MSI);
	dev_set_msi_domain(&dev->dev, msi_domain);
	if (efi_bp && !loongson_sysconf.is_soc_cpu) {
		dev->dev.archdata.dma_node_mask = 0xFULL << node_id_offset;
		dev->dev.archdata.dma_node_off  = 44 - node_id_offset;
	}
	return 0;
}

static LIST_HEAD(dma_domain_list);
static DEFINE_SPINLOCK(dma_domain_list_lock);

void add_dma_domain(struct dma_domain *domain)
{
	spin_lock(&dma_domain_list_lock);
	list_add(&domain->node, &dma_domain_list);
	spin_unlock(&dma_domain_list_lock);
}
EXPORT_SYMBOL_GPL(add_dma_domain);

void del_dma_domain(struct dma_domain *domain)
{
	spin_lock(&dma_domain_list_lock);
	list_del(&domain->node);
	spin_unlock(&dma_domain_list_lock);
}
EXPORT_SYMBOL_GPL(del_dma_domain);

static void set_dma_domain_ops(struct pci_dev *pdev)
{
	struct dma_domain *domain;

	spin_lock(&dma_domain_list_lock);
	list_for_each_entry(domain, &dma_domain_list, node) {
		if (pci_domain_nr(pdev->bus) == domain->domain_nr) {
			pdev->dev.dma_ops = domain->dma_ops;
			break;
		}
	}
	spin_unlock(&dma_domain_list_lock);
}
