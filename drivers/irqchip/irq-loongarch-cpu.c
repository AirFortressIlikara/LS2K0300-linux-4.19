// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Loongson Technologies, Inc.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>

#include <asm/irq_cpu.h>
#include <asm/loongarchregs.h>
#include <asm/setup.h>
#include <larchintrin.h>

static struct irq_domain *irq_domain;
static struct fwnode_handle *irq_fwnode;

struct fwnode_handle *coreintc_get_fwnode(void)
{
	return irq_fwnode;
}
static inline void unmask_loongarch_irq(struct irq_data *d)
{
	set_csr_ecfg(ECFGF(d->hwirq));
}

static inline void mask_loongarch_irq(struct irq_data *d)
{
	clear_csr_ecfg(ECFGF(d->hwirq));
}

static struct irq_chip loongarch_cpu_irq_controller = {
	.name		= "COREINTC",
	.irq_ack	= mask_loongarch_irq,
	.irq_mask	= mask_loongarch_irq,
	.irq_mask_ack	= mask_loongarch_irq,
	.irq_unmask	= unmask_loongarch_irq,
	.irq_eoi	= unmask_loongarch_irq,
	.irq_disable	= mask_loongarch_irq,
	.irq_enable	= unmask_loongarch_irq,
};

asmlinkage void __weak plat_irq_dispatch(int irq)
{
	unsigned int virq;

	virq = irq_linear_revmap(irq_domain, irq);
	do_IRQ(virq);
}

static int loongarch_cpu_intc_map(struct irq_domain *d, unsigned int irq,
			     irq_hw_number_t hw)
{
	struct irq_chip *chip;

	chip = &loongarch_cpu_irq_controller;

	if (cpu_has_vint)
		set_vi_handler(hw, plat_irq_dispatch);

	irq_set_chip_and_handler(irq, chip, handle_percpu_irq);

	return 0;
}

static const struct irq_domain_ops loongarch_cpu_intc_irq_domain_ops = {
	.map = loongarch_cpu_intc_map,
	.xlate = irq_domain_xlate_onecell,
};


static inline void loongarch_cpu_register_ipi_domain(struct device_node *of_node) {}

static void __init __loongarch_cpu_irq_init(struct device_node *of_node)
{
	int i;
	int irq_size, irq_base;

	irq_size = EXCCODE_INT_END - EXCCODE_INT_START;
	irq_base = LOONGARCH_CPU_IRQ_BASE;
	if (of_node)
		irq_fwnode = of_node_to_fwnode(of_node);
	else
		irq_fwnode = irq_domain_alloc_named_fwnode("coreintc");

	irq_alloc_descs(-1, irq_base, irq_size, 0);
	for (i = irq_base; i < irq_base + irq_size; i++)
		irq_set_noprobe(i);

	irq_domain = __irq_domain_add(irq_fwnode, irq_size,
				  irq_size, 0, &loongarch_cpu_intc_irq_domain_ops, NULL);
	if (!irq_domain)
		panic("Failed to add irqdomain for loongarch CPU");
	irq_domain_associate_many(irq_domain, irq_base, 0, irq_size);
}

void __init loongarch_cpu_irq_init(void)
{
	__loongarch_cpu_irq_init(NULL);
}

int __init loongarch_cpu_irq_of_init(struct device_node *of_node,
				struct device_node *parent)
{
	__loongarch_cpu_irq_init(of_node);
	return 0;
}
IRQCHIP_DECLARE(cpu_intc, "loongson,cpu-interrupt-controller", loongarch_cpu_irq_of_init);
#ifdef CONFIG_ACPI

static int __init coreintc_acpi_init_v1(struct acpi_subtable_header *header,
				   const unsigned long end)
{
	if (irq_domain)
		return 0;

	__loongarch_cpu_irq_init(NULL);
	return 0;
}
IRQCHIP_ACPI_DECLARE(coreintc_v1, ACPI_MADT_TYPE_CORE_PIC,
		NULL, ACPI_MADT_CORE_PIC_VERSION_V1,
		coreintc_acpi_init_v1);
#endif
