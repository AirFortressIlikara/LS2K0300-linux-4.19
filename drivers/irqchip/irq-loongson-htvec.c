// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2020, Jiaxun Yang <jiaxun.yang@flygoat.com>
 *			Jianmin Lv <lvjianmin@loongson.cn>
 *  Loongson HyperTransport Interrupt Vector support
 */

#define pr_fmt(fmt) "htvec: " fmt

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/syscore_ops.h>

/* Registers */
#define HTVEC_EN_OFF		0x20
#define VEC_COUNT_PER_REG	64
#define VEC_REG_IDX(irq_id)	((irq_id) / VEC_COUNT_PER_REG)
#define VEC_REG_BIT(irq_id)	((irq_id) % VEC_COUNT_PER_REG)
#define HTPIC_MAX_PARENT_IRQ	8

#ifdef CONFIG_LOONGARCH
#define loongson_cpu_has_msi256 1
#endif

struct htvec {
	void __iomem		*base;
	struct irq_domain	*htvec_domain;
	u64			saved_vec_en[HTPIC_MAX_PARENT_IRQ / 2];
	int			vec_reg_count;
} *htvec_priv;

static struct fwnode_handle *irq_fwnode;

struct fwnode_handle *htvec_get_fwnode(void)
{
	return irq_fwnode;
}

static void htvec_irq_dispatch(struct irq_desc *desc)
{
	int i;
	u64 pending;
	bool handled = false;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct htvec *priv = irq_desc_get_handler_data(desc);
	int cpu = smp_processor_id();
	chained_irq_enter(chip, desc);

	for (i = 0; i < priv->vec_reg_count; i++) {
		pending = readq(priv->base + 8 * i);
		writeq(pending, priv->base + 8 * i);
		while (pending) {
			int bit = __ffs(pending);
			int virq = irq_linear_revmap(priv->htvec_domain,
					bit + VEC_COUNT_PER_REG * i);
			if (virq > 0) handle_virq(virq, cpu);
			pending &= ~BIT(bit);
			handled = true;
		}
	}

	chained_irq_exit(chip, desc);
}

static void htvec_ack_irq(struct irq_data *d)
{
}

static void htvec_mask_irq(struct irq_data *d)
{
}

static void htvec_unmask_irq(struct irq_data *d)
{
}

static struct irq_chip htvec_irq_chip = {
	.name			= "HTINTC",
	.irq_mask		= htvec_mask_irq,
	.irq_unmask		= htvec_unmask_irq,
	.irq_ack		= htvec_ack_irq,
	.irq_set_affinity	= def_set_irq_affinity,
};

static int htvec_domain_translate(struct irq_domain *d,
					struct irq_fwspec *fwspec,
					unsigned long *hwirq,
					unsigned int *type)
{
	if (fwspec->param_count < 1)
		return -EINVAL;
	*hwirq = fwspec->param[0];
	*type = IRQ_TYPE_NONE;
	return 0;
}

static int htvec_domain_alloc(struct irq_domain *domain, unsigned int virq,
				unsigned int nr_irqs, void *arg)
{
	unsigned int type, i;
	unsigned long hwirq = 0;
	struct htvec *priv = domain->host_data;

	htvec_domain_translate(domain, arg, &hwirq, &type);
	for (i = 0; i < nr_irqs; i++) {
		irq_domain_set_info(domain, virq + i, hwirq + i, &htvec_irq_chip,
					priv, handle_edge_irq, NULL, NULL);
	}

	return 0;
}

static void htvec_domain_free(struct irq_domain *domain, unsigned int virq,
				unsigned int nr_irqs)
{
	int i;

	for (i = 0; i < nr_irqs; i++) {
		struct irq_data *d = irq_domain_get_irq_data(domain, virq + i);

		irq_set_handler(virq + i, NULL);
		irq_domain_reset_irq_data(d);
	}
}

static const struct irq_domain_ops htvec_domain_ops = {
	.translate	= htvec_domain_translate,
	.alloc		= htvec_domain_alloc,
	.free		= htvec_domain_free,
};

static void htvec_reset(struct htvec *priv)
{
	u32 idx;

	/* Clear IRQ cause registers, mask all interrupts */
	for (idx = 0; idx < priv->vec_reg_count; idx++) {
		writeq_relaxed((u64)-1, priv->base + 8 * idx);
		writeq_relaxed((u64)-1, priv->base + HTVEC_EN_OFF + 8 * idx);
	}
}

int htvec_init(unsigned long addr,
		unsigned int num_parents,
		unsigned int *parent_irq,
		struct fwnode_handle *irq_handle)
{
	int i;
	int num_vec;

	htvec_priv = kzalloc(sizeof(*htvec_priv), GFP_KERNEL);
	if (!htvec_priv)
		return -ENOMEM;

	htvec_priv->base = (void __iomem *)addr;
	htvec_priv->vec_reg_count = num_parents >> 1;
	num_vec = htvec_priv->vec_reg_count * VEC_COUNT_PER_REG;

	/* Setup IRQ domain */
	htvec_priv->htvec_domain = irq_domain_create_linear(irq_handle, num_vec,
					&htvec_domain_ops, htvec_priv);
	if (!htvec_priv->htvec_domain) {
		pr_err("loongson-htvec: cannot add IRQ domain\n");
		kfree(htvec_priv);
		return -EINVAL;
	}

	htvec_reset(htvec_priv);

	for (i = 0; i < num_parents; i++) {
		irq_set_chained_handler_and_data(parent_irq[i],
						 htvec_irq_dispatch, htvec_priv);
	}
	irq_fwnode = irq_handle;
	return 0;
}

#ifdef CONFIG_PM
static void htvec_irq_resume(void)
{
	int i;
	for (i = 0; i < htvec_priv->vec_reg_count; i++) {
		writeq(htvec_priv->saved_vec_en[i],
				htvec_priv->base + HTVEC_EN_OFF + 8 * i);
	}
}

static int htvec_irq_suspend(void)
{
	int i;
	for (i = 0; i < htvec_priv->vec_reg_count; i++) {
		htvec_priv->saved_vec_en[i] = readq(htvec_priv->base + HTVEC_EN_OFF + 8 * i);
	}
	return 0;
}

#else
#define htvec_irq_suspend NULL
#define htvec_irq_resume NULL
#endif

static struct syscore_ops htvec_irq_syscore_ops = {
	.suspend = htvec_irq_suspend,
	.resume = htvec_irq_resume,
};

static int __init htvec_init_syscore_ops(void)
{
	if (htvec_priv)
		register_syscore_ops(&htvec_irq_syscore_ops);
	return 0;
}
device_initcall(htvec_init_syscore_ops);
#ifdef CONFIG_ACPI
static int __init htvec_acpi_init_v1(struct acpi_subtable_header *header,
				   const unsigned long end)
{
	struct acpi_madt_ht_pic *htvec_entry;
	struct irq_fwspec fwspec;
	struct fwnode_handle *irq_handle, *parent_handle;
	unsigned int parent_irq[HTPIC_MAX_PARENT_IRQ];
	int i, err;
	int num_parents = 0;
	u64 base;
	htvec_entry = (struct acpi_madt_ht_pic *)header;

	base = (u64)ioremap(htvec_entry->address, htvec_entry->size);
	if (!base) {
		return -EINVAL;
	}
	irq_handle = irq_domain_alloc_named_fwnode("htintc");
	if (!irq_handle) {
		goto iounmap_base;
	}

	parent_handle = liointc_get_fwnode();

	for (i = 0; i < HTPIC_MAX_PARENT_IRQ; i++) {
		if (htvec_entry->cascade[i] > 0) {
			fwspec.fwnode = parent_handle;
			fwspec.param[0] = htvec_entry->cascade[i];
			fwspec.param_count = 1;
			parent_irq[i] = irq_create_fwspec_mapping(&fwspec);
			num_parents++;
		}
	}

	if (num_parents == 0) {
		 goto free_handle;
	}

	err = htvec_init(base,
			num_parents,
			parent_irq,
			irq_handle);
	if (err < 0)
		goto free_handle;

	return 0;

free_handle:
	irq_domain_free_fwnode(irq_handle);
iounmap_base:
	iounmap((void *)base);
	return -EINVAL;
}
IRQCHIP_ACPI_DECLARE(htvec_v1, ACPI_MADT_TYPE_HT_PIC,
		NULL, ACPI_MADT_HT_PIC_VERSION_V1,
		htvec_acpi_init_v1);
#endif
