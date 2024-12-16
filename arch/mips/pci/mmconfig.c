// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 *
 */
#include <linux/pci.h>
#include <linux/acpi.h>
#include <linux/init.h>
#include <linux/dmi.h>
#include <linux/slab.h>
#include <linux/pci-acpi.h>
#include <asm/pci.h>
#include <loongson.h>

#define PREFIX "PCI: "

static DEFINE_MUTEX(pci_mmcfg_lock);

LIST_HEAD(pci_mmcfg_list);

struct pci_mmcfg_region *pci_mmconfig_lookup(int segment, int bus)
{
	struct pci_mmcfg_region *cfg;

	list_for_each_entry(cfg, &pci_mmcfg_list, list)
		if (cfg->segment == segment &&
		    cfg->start_bus <= bus && bus <= cfg->end_bus)
			return cfg;

	return NULL;
}

static int __init pci_parse_mcfg(struct acpi_table_header *header)
{
	struct acpi_table_mcfg *mcfg;
	struct acpi_mcfg_allocation *cfg_table, *cfg;
	struct pci_mmcfg_region *new, *arr;
	unsigned long i;
	int entries;

	if (header->length < sizeof(struct acpi_table_mcfg))
		return -EINVAL;

	/* how many config structures do we have */
	entries = (header->length - sizeof(struct acpi_table_mcfg)) /
				sizeof(struct acpi_mcfg_allocation);


	mcfg = (struct acpi_table_mcfg *)header;
	cfg_table = (struct acpi_mcfg_allocation *) &mcfg[1];

	arr = kcalloc(entries, sizeof(*arr), GFP_KERNEL);
	if (!arr)
		return -ENOMEM;

	for (i = 0, new = arr; i < entries; i++, new++) {
		cfg = &cfg_table[i];
		new->address = cfg->address;
		new->segment = cfg->pci_segment;
		new->start_bus = cfg->start_bus_number;
		new->end_bus = cfg->end_bus_number;

		list_add(&new->list, &pci_mmcfg_list);
		pr_info(PREFIX
		       "MMCONFIG for domain %04x [bus %02x-%02x]"
		       "(base %#lx)\n",
		       new->segment, new->start_bus, new->end_bus, (unsigned long)(new->address));
	}

	return 0;
}

void __init pci_mmcfg_late_init(void)
{
	int err;

	err = acpi_table_parse(ACPI_SIG_MCFG, pci_parse_mcfg);
	if (err)
		pr_err("Failed to parse MCFG (%d)\n", err);
}

phys_addr_t pci_mmconfig_addr(u16 seg, u8 start)
{
	struct pci_mmcfg_region *cfg;

	cfg = pci_mmconfig_lookup(seg, start);
	if (!cfg)
		return NULL;
	return cfg->address;
}

/* Delete MMCFG information for host bridges */
int pci_mmconfig_delete(u16 seg, u8 start, u8 end)
{
	struct pci_mmcfg_region *cfg;

	list_for_each_entry(cfg, &pci_mmcfg_list, list)
		if (cfg->segment == seg && cfg->start_bus == start &&
		    cfg->end_bus == end) {
			list_del(&cfg->list);
			kfree(cfg);
			return 0;
		}

	return -ENOENT;
}
