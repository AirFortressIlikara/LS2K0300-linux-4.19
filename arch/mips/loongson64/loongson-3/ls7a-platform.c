/*
 *  Copyright (C) 2013, Loongson Technology Corporation Limited, Inc.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 */
#include <linux/init.h>
#include <asm/io.h>
#include <pci.h>
#include <boot_param.h>
#include <loongson-pch.h>
#include <loongson.h>

#include <linux/serial_8250.h>
#include <linux/platform_device.h>

#include <linux/i2c.h>
#include <linux/platform_data/i2c-gpio.h>
#include <linux/acpi.h>

#ifndef loongson_ls7a_decode_year
#define loongson_ls7a_decode_year(year) ((year) + 1900)
#endif

#define RTC_TOYREAD0    0x2C
#define RTC_YEAR        0x30

u32 node_id_offset;

unsigned long ls7a_rwflags;
DEFINE_RWLOCK(ls7a_rwlock);
EXPORT_SYMBOL(ls7a_rwflags);
EXPORT_SYMBOL(ls7a_rwlock);

void pci_no_msi(void);

#define LS7A_DMA_CFG	(void *)TO_UNCAC(LS7A_CHIPCFG_REG_BASE + 0x041c)

static void ls7a_early_config(void)
{
	struct cpuinfo_mips *c = &boot_cpu_data;

	node_id_offset = ((readl(LS7A_DMA_CFG) & 0x1f00) >> 8) + 36;

	if (current_cpu_type()!=CPU_LOONGSON3_COMP && 
		(c->processor_id & PRID_REV_MASK) < PRID_REV_LOONGSON3A_R2_1)
		pci_no_msi();
}

unsigned long loongson_ls7a_get_rtc_time(void)
{
	unsigned int year, mon, day, hour, min, sec;
	unsigned int value;

	value = ls7a_readl(LS7A_RTC_REG_BASE + RTC_TOYREAD0);
	sec = (value >> 4) & 0x3f;
	min = (value >> 10) & 0x3f;
	hour = (value >> 16) & 0x1f;
	day = (value >> 21) & 0x1f;
	mon = (value >> 26) & 0x3f;
	year = ls7a_readl(LS7A_RTC_REG_BASE + RTC_YEAR);

	year = loongson_ls7a_decode_year(year);

	return mktime(year, mon, day, hour, min, sec);
}

phys_addr_t mcfg_addr_init(int domain)
{
	return (((u64) domain << 44) | HT1LO_EXT_PCICFG_BASE_FIX);
}

static struct resource pci_mem_resource = {
	.name	= "pci memory space",
	.flags	= IORESOURCE_MEM,
};

static struct resource pci_io_resource = {
	.name	= "pci io space",
	.flags	= IORESOURCE_IO,
};

static struct pci_controller ls7a_pci_controller = {
	.pci_ops	= &ls7a_pci_ops,
	.io_resource	= &pci_io_resource,
	.mem_resource	= &pci_mem_resource,
	.mem_offset	= 0x00000000UL,
	.io_offset	= 0x00000000UL,
};

static void __init ls7a_arch_initcall(void)
{
	pci_mem_resource.start = loongson_sysconf.pci_mem_start_addr;
	pci_mem_resource.end   = loongson_sysconf.pci_mem_end_addr;
	pci_io_resource.start  = LOONGSON_PCI_IO_START;
	pci_io_resource.end    = IO_SPACE_LIMIT;
	ioport_resource.end    = IO_SPACE_LIMIT;

	ls7a_pci_controller.io_map_base = mips_io_port_base;
	ls7a_pci_controller.mcfg_addr = mcfg_addr_init(0);
	if (acpi_disabled)
		register_pci_controller(&ls7a_pci_controller);
}

static void get_suspend_addr(void)
{
	acpi_status status;
	unsigned long long suspend_addr = 0;

	status = acpi_evaluate_integer(NULL, "\\SADR", NULL, &suspend_addr);
	if (ACPI_FAILURE(status) || !suspend_addr) {
		pr_err("No supspend address provided!! S3 will not be allowed!!\n");
		return;
	}
	loongson_sysconf.suspend_addr = suspend_addr;
}
static void __init ls7a_device_initcall(void)
{
	if (!acpi_disabled)
		get_suspend_addr();
}

static void ls7a_fixup_pci_pin(struct pci_dev *dev)
{
	if (dev->vendor == PCI_VENDOR_ID_LOONGSON
			&& (dev->device & 0xff00) == 0x7a00) {
		u8 fun = dev->devfn & 7;
		dev->pin = 1 + (fun & 3);
	}
}

static int create_irqdomain_map_legacy(struct pci_dev *dev)
{
	if (!pci_dev_msi_enabled(dev)){
		if (!dev->irq_managed &&
				dev->irq > 0 && dev->irq != 255) {
			struct irq_fwspec fwspec;
			fwspec.fwnode = NULL;
			fwspec.param[0] = dev->irq;
			fwspec.param_count = 1;
			dev->irq = irq_create_fwspec_mapping(&fwspec);
			dev->irq_managed = 1;
		}
	}
	return 0;
}

int ls7a_pcibios_dev_init(struct pci_dev *dev)
{
#ifdef CONFIG_ACPI
	acpi_status status;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	acpi_handle handle = NULL;

	if (dev->bus->bridge)
		handle = ACPI_HANDLE(dev->bus->bridge);

	if (!acpi_disabled && handle) {
		/* 'handle' is the _PRT's parent (root bridge or PCI-PCI bridge) */
		status = acpi_get_irq_routing_table(handle, &buffer);
		kfree(buffer.pointer);
		if (ACPI_SUCCESS(status)) {
			if (!pci_dev_msi_enabled(dev)) {
				if (dev->multifunction)
					ls7a_fixup_pci_pin(dev);
				return acpi_pci_irq_enable(dev);
			}
		}
	}
#endif
	return create_irqdomain_map_legacy(dev);
}

struct pci_controller *ls7a_getroot_controller(void)
{
	return &ls7a_pci_controller;
}

struct platform_controller_hub ls7a_pch = {
	.type			= LS7A,
	.pcidev_max_funcs 	= 7,
	.early_config		= ls7a_early_config,
	.init_irq		= setup_PCH_PIC,
	.pch_arch_initcall	= ls7a_arch_initcall,
	.pch_device_initcall	= ls7a_device_initcall,
	.pcibios_dev_init	= ls7a_pcibios_dev_init,
	.get_root_controller	= ls7a_getroot_controller,
};

void get_pci_root_info (struct pci_controller **controller, struct list_head *resources, bool is_use_crs)
{
	if(is_use_crs) {
		*controller = loongson_pch->get_root_controller();
	} else {
		pci_add_resource(resources, &pci_mem_resource);
		pci_add_resource(resources, &pci_io_resource);
	}
}
