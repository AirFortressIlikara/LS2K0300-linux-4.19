/* SPDX-License-Identifier: GPL-2.0 */
/*
 * =====================================================================================
 *
 *       Filename:  irq.c
 *
 *    Description:  irq handle
 *
 *        Version:  1.0
 *       Revision:  none
 *       Compiler:  gcc
 *
 *        Company:  Loongson Corp.
 *
 * =====================================================================================
 */
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/spinlock_types.h>
#include <asm/irq_cpu.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/syscore_ops.h>
#include "irq-ls2k.h"

#define LS2K_INTISR_REG(i)		(priv->regbase + 0x20 + (((i) > 31) ? 0x40 : 0))
#define LS2K_INTEN_REG(i)		(priv->regbase + 0x24 + (((i) > 31) ? 0x40 : 0))
#define LS2K_INTENSET_REG(i)		(priv->regbase + 0x28 + (((i) > 31) ? 0x40 : 0))
#define LS2K_INTENCLR_REG(i)		(priv->regbase + 0x2c + (((i) > 31) ? 0x40 : 0))
#define LS2K_INTPOL_REG(i)		(priv->regbase + 0x30 + (((i) > 31) ? 0x40 : 0))
#define LS2K_INTEDGE_REG(i)		(priv->regbase + 0x34 + (((i) > 31) ? 0x40 : 0))
#define LS2K_IRQ_ROUTE_REG(i)		(priv->regbase + 0x00 + (((i) > 31) ? 0x40 : 0) + ((i) % 32))
#define LS2K_COREISR_REG0(i)		(priv->isrreg + (i) * 0x100)
#define LS2K_COREISR_REG1(i)		(priv->isrreg + 8 + (i) * 0x100)
/* 2k1000 */
#define LS2K_BOUNCE_REG(i)		(priv->regbase + 0x38 + (((i) > 31) ? 0x40 : 0))
#define LS2K_INTAUTO_REG(i)		(priv->regbase + 0x3c + (((i) > 31) ? 0x40 : 0))

struct iointc *iointc_ptr;

void mask_ls2k_iointc_irq(struct irq_data *data)
{
	struct iointc *priv = (struct iointc *)data->domain->host_data;
	unsigned long hwirq = data->hwirq;
	void *reg = LS2K_INTENCLR_REG(hwirq);
	unsigned long flags;

	raw_spin_lock_irqsave(&priv->lock, flags);
	writel(1<<(hwirq%32), (void *)reg);
	raw_spin_unlock_irqrestore(&priv->lock, flags);

}

void unmask_ls2k_iointc_irq(struct irq_data *data)
{
	struct iointc *priv = (struct iointc *)data->domain->host_data;
	unsigned long hwirq = data->hwirq;
	void *reg = LS2K_INTENSET_REG(hwirq);
	unsigned long flags;

	raw_spin_lock_irqsave(&priv->lock, flags);
	writel(1<<(hwirq % 32), (void *)reg);
	raw_spin_unlock_irqrestore(&priv->lock, flags);
}

int ls2k_iointc_set_type(struct irq_data *d, unsigned int type)
{
	struct iointc *priv = (struct iointc *)d->domain->host_data;
	unsigned long hwirq = d->hwirq;
	unsigned long flags;
	unsigned int pol_val, edge_val;

	raw_spin_lock_irqsave(&priv->lock, flags);
	pol_val = readl((void *)LS2K_INTPOL_REG(hwirq));
	edge_val = readl((void *)LS2K_INTEDGE_REG(hwirq));

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		edge_val |= (1<<(hwirq%32));
		pol_val &= ~(1<<(hwirq%32));
		break;
	case IRQ_TYPE_EDGE_FALLING:
		edge_val |= (1<<(hwirq%32));
		pol_val |= (1<<(hwirq%32));
		break;
	case IRQ_TYPE_LEVEL_LOW:
		edge_val &= ~(1<<(hwirq%32));
		pol_val |= (1<<(hwirq%32));
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		edge_val &= ~(1<<(hwirq%32));
		pol_val &= ~(1<<(hwirq%32));
		break;
	case IRQ_TYPE_EDGE_BOTH:
		pr_warn("Not support both_edge interrupt,\
			 but requester can simulate it\n");
		break;
	case IRQ_TYPE_NONE:
	default:
		raw_spin_unlock_irqrestore(&priv->lock, flags);
		return -EINVAL;
	}

	writel(pol_val, (void *)LS2K_INTPOL_REG(hwirq));
	writel(edge_val, (void *)LS2K_INTEDGE_REG(hwirq));
	raw_spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

void ls2k_iointc_irq_ack(struct irq_data *d)
{
	struct iointc *priv = (struct iointc *)d->domain->host_data;
	unsigned long hwirq = d->hwirq;
	void *reg;
	unsigned long flags;
	unsigned int reg_val;

	raw_spin_lock_irqsave(&priv->lock, flags);

	reg = LS2K_INTEN_REG(hwirq);
	reg_val = readl((void *)reg);

	reg = LS2K_INTENCLR_REG(hwirq);
	writel(1 << (hwirq%32), (void *)reg);

	if (reg_val & (1 << (hwirq%32)))
		reg = LS2K_INTENSET_REG(hwirq);
	else
		reg = LS2K_INTENCLR_REG(hwirq);
	writel(1 << (hwirq%32), (void *)reg);
	raw_spin_unlock_irqrestore(&priv->lock, flags);
}

int ls2k_iointc_irq_domain_xlate(struct irq_domain *d, struct device_node *node,
		const u32 *intspec, unsigned int intsize,
		unsigned long *out_hwirq, unsigned int *out_type)
{
	if (WARN_ON(intsize < 1))
		return -EINVAL;

	*out_hwirq = intspec[0];
	if (intsize > 1)
		*out_type = intspec[1] & IRQ_TYPE_SENSE_MASK;
	else
		*out_type = IRQ_TYPE_NONE;
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static struct ls2k_iointc_irq_registers {
	u32 inten_low;
	u32 intpol_low;
	u32 intedge_low;
	u32 introute_low;
	u32 inten_high;
	u32 intpol_high;
	u32 intedge_high;
	u32 introute_high;
	u8  entry[64];
} ls2k_irq_regs;

static int ls2k_iointc_suspend(void)
{
	int i;
	struct iointc *priv = iointc_ptr;

	ls2k_irq_regs.inten_low = readl((void *)LS2K_INTEN_REG(0));
	ls2k_irq_regs.intpol_low = readl((void *)LS2K_INTPOL_REG(0));
	ls2k_irq_regs.intedge_low = readl((void *)LS2K_INTEDGE_REG(0));
	ls2k_irq_regs.inten_high = readl((void *)LS2K_INTEN_REG(32));
	ls2k_irq_regs.intpol_high = readl((void *)LS2K_INTPOL_REG(32));
	ls2k_irq_regs.intedge_high = readl((void *)LS2K_INTEDGE_REG(32));

	for (i = 0; i < 64; i++)
		ls2k_irq_regs.entry[i] = readb((void *)(LS2K_IRQ_ROUTE_REG(i)));

	return 0;
}

static void ls2k_iointc_resume(void)
{
	int i;
	struct iointc *priv = iointc_ptr;

	writel(ls2k_irq_regs.inten_low, (void *)LS2K_INTENSET_REG(0));
	writel(ls2k_irq_regs.intpol_low, (void *)LS2K_INTPOL_REG(0));
	writel(ls2k_irq_regs.intedge_low, (void *)LS2K_INTEDGE_REG(0));
	writel(ls2k_irq_regs.inten_high, (void *)LS2K_INTENSET_REG(32));
	writel(ls2k_irq_regs.intpol_high, (void *)LS2K_INTPOL_REG(32));
	writel(ls2k_irq_regs.intedge_high, (void *)LS2K_INTEDGE_REG(32));

	for (i = 0; i < 64; i++)
		writeb(ls2k_irq_regs.entry[i], (void *)(LS2K_IRQ_ROUTE_REG(i)));
}

static struct syscore_ops ls2k_iointc_syscore_ops = {
	.suspend	= ls2k_iointc_suspend,
	.resume		= ls2k_iointc_resume,
};

static void ls2k_ioinitc_syscore_init(void)
{
	register_syscore_ops(&ls2k_iointc_syscore_ops);
}
#else
static inline void ls2k_ioinitc_syscore_init(void) {}
#endif

void ls2k500_iointc_set_irq_route(struct iointc *priv, int irq, struct cpumask *core_mask, int line)
{

	if (irq > 63) {
		printk(KERN_ERR "%s: interrupt number error\n", __func__);
		return;
	}

	writeb((1 << (line + 4)), (void *)LS2K_IRQ_ROUTE_REG(irq));
}

int ls2k500_iointc_set_affinity(struct irq_data *d, const struct cpumask *affinity,
		bool force)
{
	struct iointc *priv = (struct iointc *)d->domain->host_data;
	unsigned long hwirq = d->hwirq;
	cpumask_t tmask;
	unsigned long *mask;
	unsigned long flags;
	unsigned long irqline_parent = irq_get_irq_data(priv->cascade)->hwirq;

	cpumask_and(&tmask, affinity, cpu_online_mask);
	cpumask_copy(d->common->affinity, &tmask);

	mask = cpumask_bits(&tmask);
	raw_spin_lock_irqsave(&priv->lock, flags);

	ls2k500_iointc_set_irq_route(priv, hwirq, &tmask, irqline_parent - 2 /* 2 soft IP */);

	raw_spin_unlock_irqrestore(&priv->lock, flags);

	irq_data_update_effective_affinity(d, &tmask);

	return IRQ_SET_MASK_OK_NOCOPY;
}

static struct irq_chip ls2k500_iointc = {
	.name		= "2k-iointc",
	.irq_mask	= mask_ls2k_iointc_irq,
	.irq_unmask	= unmask_ls2k_iointc_irq,
	.irq_set_affinity = ls2k500_iointc_set_affinity,
	.irq_ack	= ls2k_iointc_irq_ack,
	.irq_set_type	= ls2k_iointc_set_type,
	.flags = IRQCHIP_SKIP_SET_WAKE,
};

static void ls2k500_iointc_irq_handler(struct irq_desc *desc)
{
	struct iointc *priv = irq_desc_get_handler_data(desc);
	unsigned long pending;
	u32 reg_val;
	int irq;
	unsigned int virq;

	pending = readl((void *)LS2K_COREISR_REG0(0));
	reg_val = readl((void *)LS2K_COREISR_REG1(0));
	pending |= (unsigned long)reg_val << 32;

	if (!pending)
		spurious_interrupt();

	while (pending) {
		irq = fls64(pending) - 1;
		virq = irq_linear_revmap(priv->domain, irq);
		do_IRQ(virq);
		pending &= ~BIT(irq);
	}
}

static int ls2k500_iointc_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hw)
{
	irq_set_chip_and_handler(irq, &ls2k500_iointc, handle_level_irq);
	return 0;
}

static const struct irq_domain_ops ls2k500_iointc_irq_domain_ops = {
	.map = ls2k500_iointc_map,
	.xlate = ls2k_iointc_irq_domain_xlate,
};

int ls2k500_iointc_irq_of_init(struct device_node *of_node,
				struct device_node *parent)
{
	u32 cascade;
	struct fwnode_handle *fwnode;
	struct iointc *priv = kzalloc(sizeof(struct iointc), GFP_KERNEL);

	iointc_ptr = priv;
	raw_spin_lock_init(&priv->lock);
	priv->cascade = cascade = of_irq_get_byname(of_node, "cascade");
	priv->regbase = of_iomap(of_node, 0);
	priv->isrreg = of_iomap(of_node, 1);
	fwnode = of_node_to_fwnode(of_node);

	priv->domain = irq_domain_create_linear(fwnode, 64,
					&ls2k500_iointc_irq_domain_ops, priv);

	irq_set_chained_handler_and_data(cascade,
					ls2k500_iointc_irq_handler,
					priv);
	ls2k_ioinitc_syscore_init();

	if (!priv->domain)
		panic("Failed to add irqdomain for ls2k io intc");

	return 0;
}
IRQCHIP_DECLARE(ls2k500_iointc, "loongson,2k500-iointc", ls2k500_iointc_irq_of_init);
IRQCHIP_DECLARE(ls2k500_icu, "loongson,2k500-icu", ls2k500_iointc_irq_of_init);

void ls2k1000_iointc_set_irq_route(struct iointc *priv, int irq, struct cpumask *core_mask, int line)
{
	void *auto_reg = LS2K_INTAUTO_REG(irq);
	void *bounce_reg = LS2K_BOUNCE_REG(irq);
	unsigned long *mask;
	unsigned int reg_val;

	if (irq > 63) {
		printk(KERN_ERR "%s: interrupt number error\n", __func__);
		return;
	}

	mask = cpumask_bits(core_mask);
	writeb((1 << (line + 4)) | (*mask & 0x1), (void *)LS2K_IRQ_ROUTE_REG(irq));

	reg_val = readl((void *)auto_reg);
	reg_val &= ~(1 << (irq % 32)); /* auto always set to 0 */
	writel(reg_val, (void *)auto_reg);

	reg_val = readl((void *)bounce_reg);
	reg_val &= ~(1 << (irq % 32));
	writel(reg_val, (void *)bounce_reg);
}

int ls2k1000_iointc_set_affinity(struct irq_data *d, const struct cpumask *affinity,
		bool force)
{
	struct iointc *priv = (struct iointc *)d->domain->host_data;
	unsigned long hwirq = d->hwirq;
	cpumask_t tmask;
	unsigned long *mask;
	unsigned long flags;
	unsigned long irqline_parent = irq_get_irq_data(priv->cascade)->hwirq;

	cpumask_and(&tmask, affinity, cpu_online_mask);
	cpumask_copy(d->common->affinity, &tmask);

	mask = cpumask_bits(&tmask);
	raw_spin_lock_irqsave(&priv->lock, flags);

	ls2k1000_iointc_set_irq_route(priv, hwirq, &tmask, irqline_parent - 2 /* 2 soft IP */);

	raw_spin_unlock_irqrestore(&priv->lock, flags);

	irq_data_update_effective_affinity(d, &tmask);

	return IRQ_SET_MASK_OK_NOCOPY;
}

static struct irq_chip ls2k1000_iointc = {
	.name		= "2k-iointc",
	.irq_mask	= mask_ls2k_iointc_irq,
	.irq_unmask	= unmask_ls2k_iointc_irq,
	.irq_set_affinity = ls2k1000_iointc_set_affinity,
	.irq_ack	= ls2k_iointc_irq_ack,
	.irq_set_type	= ls2k_iointc_set_type,
	.flags = IRQCHIP_SKIP_SET_WAKE,
};

static void ls2k1000_iointc_irq_handler(struct irq_desc *desc)
{
	struct iointc *priv = irq_desc_get_handler_data(desc);
	unsigned long pending;
	int irq, virq;
	int core = cpu_logical_map(smp_processor_id());

	pending = readl((void *)LS2K_COREISR_REG1(core));
	pending <<= 32;
	pending |= readl((void *)LS2K_COREISR_REG0(core));

	if (!pending)
		spurious_interrupt();

	while (pending) {
		irq = fls64(pending) - 1;
		virq = irq_linear_revmap(priv->domain, irq);
		do_IRQ(virq);
		pending &= ~BIT(irq);
	}
}

static int ls2k1000_iointc_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hw)
{
	irq_set_chip_and_handler(irq, &ls2k1000_iointc, handle_level_irq);
	return 0;
}

static const struct irq_domain_ops ls2k1000_iointc_irq_domain_ops = {
	.map = ls2k1000_iointc_map,
	.xlate = ls2k_iointc_irq_domain_xlate,
};

int ls2k1000_iointc_irq_of_init(struct device_node *of_node,
				struct device_node *parent)
{
	u32 cascade;
	struct fwnode_handle *fwnode;
	struct iointc *priv = kzalloc(sizeof(struct iointc), GFP_KERNEL);

	iointc_ptr = priv;
	raw_spin_lock_init(&priv->lock);
	priv->cascade = cascade = of_irq_get_byname(of_node, "cascade");
	priv->regbase = of_iomap(of_node, 0);
	priv->isrreg = of_iomap(of_node, 1);
	fwnode = of_node_to_fwnode(of_node);

	priv->domain = irq_domain_create_linear(fwnode, 64,
					&ls2k1000_iointc_irq_domain_ops, priv);

	irq_set_chained_handler_and_data(cascade,
					ls2k1000_iointc_irq_handler,
					priv);
	if (!priv->domain)
		panic("Failed to add irqdomain for ls2k io intc");

	ls2k_ioinitc_syscore_init();

	irq_set_default_host(priv->domain);

	return 0;
}
IRQCHIP_DECLARE(ls2k1000_iointc, "loongson,2k1000-iointc", ls2k1000_iointc_irq_of_init);
IRQCHIP_DECLARE(ls2k1000_icu, "loongson,2k1000-icu", ls2k1000_iointc_irq_of_init);
