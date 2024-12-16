// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/init.h>
#include <asm/io.h>
#include <pci.h>
#include <boot_param.h>
#include <loongson-pch.h>
#include <loongson.h>
#include <linux/serial_8250.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <asm/mach-loongson64/loongson_cpu.h>
#include <asm/irq_cpu.h>

#define MSI_IRQ_NR_224		224
extern void (*do_cascade)(void);
extern void do_pch(void);
extern void setup_pch_irqs(int model);
extern void of_setup_pch_irqs(int model);
extern void pch_msi_domain_init(int start, int count);
extern void pch_lpc_domain_init(void);

static struct resource pci_mem_resource = {
	.name   = "pci memory space",
	.flags  = IORESOURCE_MEM,
};

static struct resource pci_io_resource = {
	.name   = "pci io space",
	.flags  = IORESOURCE_IO,
};

static struct pci_controller kvm_pci_controller = {
	.pci_ops        = &ls7a_guest_pci_ops,
	.io_resource    = &pci_io_resource,
	.mem_resource   = &pci_mem_resource,
	.mem_offset     = 0x00000000UL,
	.io_offset      = 0x00000000UL,
};

static void __init kvm_arch_initcall(void)
{
	pci_mem_resource.start = loongson_sysconf.pci_mem_start_addr;
	pci_mem_resource.end   = loongson_sysconf.pci_mem_end_addr;
	pci_io_resource.start  = LOONGSON_PCI_IO_START;
	pci_io_resource.end    = IO_SPACE_LIMIT;
	ioport_resource.end    = IO_SPACE_LIMIT;

	kvm_pci_controller.io_map_base = mips_io_port_base;
	if (acpi_disabled)
		register_pci_controller(&kvm_pci_controller);
}

static int __init kvm_detect(void)
{
	if (!loongson_cpu_has_csr)
		return 0;
	if (csr_readl(LOONGSON_CSR_FEATURES) & LOONGSON_CSRF_VM)
		return 1;
	if (cpu_guestmode)
		return 1;

	return 0;
}

static int kvm_pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	int irq;
	u8 tmp;

	pci_read_config_byte(dev, PCI_INTERRUPT_PIN, &tmp);
	slot = pci_common_swizzle((struct pci_dev *)dev, &tmp);
	irq = LS7A_PCH_IRQ_BASE + 16 + ((slot * 4 + tmp - 1) & 0xf);
	return irq;
}

static void __init kvm_init_irq(void)
{
	int i, start, count;
	struct fwnode_handle *irq_fwnode;
	enum pch_irq_route_model_id model;

	irq_alloc_descs(-1, MIPS_CPU_IRQ_BASE, 8, 0);
	for (i = MIPS_CPU_IRQ_BASE; i < MIPS_CPU_IRQ_BASE + 8; i++)
		irq_set_noprobe(i);
	do_cascade = do_pch;
	mips_cpu_irq_init();
	model = PCH_IRQ_ROUTE_EXT_GUEST;

	writel(GUEST_NEW_IRQMODEL, LS7A_GUEST_IRQMODEL_REG);
	pr_info("Support EXT interrupt.\n");
#ifdef CONFIG_LOONGSON_EXTIOI
	start = LOONGSON_GUEST_PCH_IRQ_BASE;
	count = MSI_IRQ_NR_224;
	irq_fwnode = irq_domain_alloc_named_fwnode("kvm_eiointc");
	if (irq_fwnode) {
		extioi_vec_init(irq_fwnode, LOONGSON_BRIDGE_IRQ, CSR_EXTIOI_VECTOR_NUM, 0, 0, 0, 0);
	}
	pch_msi_domain_init(start, count);
#endif
	if (acpi_disabled)
		of_setup_pch_irqs(model);
	else
		setup_pch_irqs(model);
}

static int create_irqdomain_map_legacy(struct pci_dev *dev)
{
	if (!pci_dev_msi_enabled(dev)) {
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

int kvm_pcibios_dev_init(struct pci_dev *dev)
{
	int irq = 0;

#ifdef CONFIG_ACPI
	if (!acpi_disabled && !pci_dev_msi_enabled(dev))
		return acpi_pci_irq_enable(dev);
#endif
	if (dev->msi_enabled || dev->msix_enabled)
		return 0;

	if (dev->irq_managed && dev->irq > 0)
		return 0;

	irq = kvm_pcibios_map_irq(dev, dev->devfn >> 3, 1);
	if (irq >= 0) {
		dev->irq = irq;
		dev->irq_managed = 1;
	}
	return create_irqdomain_map_legacy(dev);
}

struct pci_controller *kvm_getroot_controller(void)
{
	return &kvm_pci_controller;
}

static void __init kvm_init_platform(void)
{
	loongson_pch->pch_arch_initcall = kvm_arch_initcall;
	loongson_pch->pcibios_map_irq	= kvm_pcibios_map_irq;
	loongson_pch->pcibios_dev_init  = kvm_pcibios_dev_init;
	loongson_pch->get_root_controller = kvm_getroot_controller;
	loongson_pch->init_irq		= kvm_init_irq;
}

void __init init_hypervisor_platform(void)
{
	int ret;


	ret = kvm_detect();
	if (ret)
		kvm_init_platform();
}
