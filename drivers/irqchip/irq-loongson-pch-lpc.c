// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2020, Jianmin Lv <lvjianmin@loongson.cn>
 *  Loongson LPC support
 */

#define pr_fmt(fmt) "lpc: " fmt

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <loongson-pch.h>
#include <linux/syscore_ops.h>

/* Registers */
#define LPC_PARENT_IRQ		83
#define LPC_COUNT		16
#define LPC_INT_CTL		0x0
#define LPC_INT_ENA		0x4
#define LPC_INT_STS		0x8
#define LPC_INT_CLR		0xc
#define LPC_INT_POL		0x10

struct pch_lpc {
	void __iomem		*base;
	struct fwnode_handle	*domain_handle;
	u32			saved_reg_ctl;
	u32			saved_reg_ena;
	u32			saved_reg_pol;
	raw_spinlock_t		lpc_lock;
} *pch_lpc_priv;

struct fwnode_handle *pch_lpc_get_fwnode(void)
{
	return pch_lpc_priv ? pch_lpc_priv->domain_handle : NULL;
}

static void ack_lpc_irq(struct irq_data *d)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&pch_lpc_priv->lpc_lock, flags);
	writel(0x1 << d->irq, pch_lpc_priv->base + LPC_INT_CLR);
	raw_spin_unlock_irqrestore(&pch_lpc_priv->lpc_lock, flags);
}
static void mask_lpc_irq(struct irq_data *d)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&pch_lpc_priv->lpc_lock, flags);
	writel(readl(pch_lpc_priv->base + LPC_INT_ENA) & (~(0x1 << (d->irq))), pch_lpc_priv->base + LPC_INT_ENA);
	raw_spin_unlock_irqrestore(&pch_lpc_priv->lpc_lock, flags);
}

static void mask_ack_lpc_irq(struct irq_data *d)
{
}
static void unmask_lpc_irq(struct irq_data *d)
{
	unsigned long flags;
	raw_spin_lock_irqsave(&pch_lpc_priv->lpc_lock, flags);
	writel(readl(pch_lpc_priv->base + LPC_INT_ENA) | (0x1 << (d->irq)),
			pch_lpc_priv->base + LPC_INT_ENA);
	raw_spin_unlock_irqrestore(&pch_lpc_priv->lpc_lock, flags);
}

static int pch_lpc_set_type(struct irq_data *d, unsigned int type)
{
	u32 val;
	u32 mask = 0x1 << (d->hwirq);

	if (type != IRQ_TYPE_LEVEL_HIGH && type != IRQ_TYPE_LEVEL_LOW)
		return 0;

	val = readl(pch_lpc_priv->base + LPC_INT_POL);

	if (type == IRQ_TYPE_LEVEL_HIGH)
		val |= mask;
	else
		val &= ~mask;

	writel(val, pch_lpc_priv->base + LPC_INT_POL);

	return 0;
}

static struct irq_chip pch_lpc_irq_chip = {
	.name			= "LPC",
	.irq_mask		= mask_lpc_irq,
	.irq_unmask		= unmask_lpc_irq,
	.irq_ack		= ack_lpc_irq,
	.irq_mask_ack		= mask_ack_lpc_irq,
	.irq_eoi		= unmask_lpc_irq,
	.irq_set_type		= pch_lpc_set_type,
	.flags			= IRQCHIP_MASK_ON_SUSPEND,
};

static void pch_handle_irq(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 pending;

	chained_irq_enter(chip, desc);

	pending = readl(pch_lpc_priv->base + LPC_INT_ENA);
	pending &= readl(pch_lpc_priv->base + LPC_INT_STS);
	if (!pending)
		spurious_interrupt();

	while (pending) {
		int bit = __ffs(pending);
		generic_handle_irq(bit);
		pending &= ~BIT(bit);
	}
	chained_irq_exit(chip, desc);
}

static int pch_lpc_map(struct irq_domain *d, unsigned int irq,
			irq_hw_number_t hw)
{
	irq_set_chip_and_handler(irq, &pch_lpc_irq_chip, handle_level_irq);
	return 0;
}

static const struct irq_domain_ops pch_lpc_domain_ops = {
	.translate	= irq_domain_translate_twocell,
	.map		= pch_lpc_map,
};

static void pch_lpc_reset(struct pch_lpc *priv)
{
	/* Enable the LPC interrupt, bit31: en  bit30: edge */
	writel(0x80000000, priv->base + LPC_INT_CTL);
	writel(0, priv->base + LPC_INT_ENA);
	/* clear all 18-bit interrpt bit */
	writel(0x3ffff, priv->base + LPC_INT_CLR);
}

static int pch_lpc_disable(void)
{
	return !!((readl(pch_lpc_priv->base + LPC_INT_ENA) == 0xffffffff) && (readl(pch_lpc_priv->base + LPC_INT_STS) == 0xffffffff));
}

int pch_lpc_init(u64 address, u16 size,
		int parent_irq,
		struct fwnode_handle *irq_handle)
{
	struct pch_lpc *priv;
	int err;
	struct irq_domain *lpc_domain;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	pch_lpc_priv = priv;
	priv->domain_handle = irq_handle;
	priv->base = ioremap(address, size);
	if (!priv->base) {
		err = -ENOMEM;
		goto free_priv;
	}

	if (pch_lpc_disable()) {
		pr_err("Failed to get LPC status\n");
		err = -EIO;
		goto bypass;
	}
	raw_spin_lock_init(&priv->lpc_lock);

	lpc_domain = __irq_domain_add(irq_handle, LPC_COUNT,
				  LPC_COUNT, 0, &pch_lpc_domain_ops, priv);
	if (!lpc_domain) {
		pr_err("Failed to create IRQ domain\n");
		err = -ENOMEM;
		goto iounmap_base;
	}
	if (lpc_domain)
		irq_domain_associate_many(lpc_domain, 0, 0, LPC_COUNT);
	pch_lpc_reset(priv);

	irq_set_chained_handler_and_data(parent_irq,
						 pch_handle_irq, priv);
	return 0;

iounmap_base:
	iounmap(priv->base);
free_priv:
	kfree(priv);
bypass:
	return err;
}

#ifdef CONFIG_PM
static void pch_lpc_resume(void)
{
	writel(pch_lpc_priv->saved_reg_pol, pch_lpc_priv->base + LPC_INT_POL);
	writel(pch_lpc_priv->saved_reg_ctl, pch_lpc_priv->base + LPC_INT_CTL);
	writel(pch_lpc_priv->saved_reg_ena, pch_lpc_priv->base + LPC_INT_ENA);
}

static int pch_lpc_suspend(void)
{
	pch_lpc_priv->saved_reg_ctl = readl(pch_lpc_priv->base + LPC_INT_CTL);
	pch_lpc_priv->saved_reg_ena = readl(pch_lpc_priv->base + LPC_INT_ENA);
	pch_lpc_priv->saved_reg_pol = readl(pch_lpc_priv->base + LPC_INT_POL);
	return 0;
}

#else
#define pch_lpc_suspend NULL
#define pch_lpc_resume NULL
#endif

static struct syscore_ops pch_lpc_syscore_ops = {
	.suspend = pch_lpc_suspend,
	.resume = pch_lpc_resume,
};

static int __init pch_lpc_init_syscore_ops(void)
{
	if (pch_lpc_priv)
		register_syscore_ops(&pch_lpc_syscore_ops);
	return 0;
}
device_initcall(pch_lpc_init_syscore_ops);
#ifdef CONFIG_ACPI
static int __init pch_lpc_acpi_init_v1(struct acpi_subtable_header *header,
				   const unsigned long end)
{
	struct acpi_madt_lpc_pic *pch_lpc_entry;
	struct fwnode_handle *irq_handle;
	struct irq_fwspec fwspec;
	int parent_irq;
	pch_lpc_entry = (struct acpi_madt_lpc_pic *)header;

	irq_handle = irq_domain_alloc_named_fwnode("lpcintc");
	if (!irq_handle) {
		pr_err("Unable to allocate domain handle\n");
		return -ENOMEM;
	}

	fwspec.fwnode = NULL;
	fwspec.param[0] = pch_lpc_entry->cascade + pchintc_gsi_base(0);
	fwspec.param_count = 1;
	parent_irq = irq_create_fwspec_mapping(&fwspec);
	pch_lpc_init(pch_lpc_entry->address,
			pch_lpc_entry->size,
			parent_irq,
			irq_handle);
	return 0;
}
IRQCHIP_ACPI_DECLARE(pch_lpc_v1, ACPI_MADT_TYPE_LPC_PIC,
		NULL, ACPI_MADT_LPC_PIC_VERSION_V1,
		pch_lpc_acpi_init_v1);
#endif
