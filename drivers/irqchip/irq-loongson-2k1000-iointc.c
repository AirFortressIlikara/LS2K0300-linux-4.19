/* SPDX-License-Identifier: GPL-2.0 */
/*
 * =====================================================================================
 *
 *       Filename:  irq.c
 *
 *    Description:  irq handle
 *
 *        Version:  1.0
 *        Created:  03/16/2017 10:52:40 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  hp (Huang Pei), huangpei@loongson.cn
 *        Company:  Loongson Corp.
 *
 * =====================================================================================
 */
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/spinlock_types.h>
#include <asm/irq_cpu.h>
#include <loongson_cpu.h>
#include <loongson.h>
#include <loongson-2k.h>
#include <linux/syscore_ops.h>

DEFINE_RAW_SPINLOCK(ls2k_irq_lock);

struct irq_domain *ls2k_iointc_irq_domain;

#ifdef CONFIG_PM
struct ls2k_iointc_irq_registers {
	u32 Inten_0;
	u32 Intpol_0;
	u32 Intedge_0;
	u32 Intbounce_0;
	u32 Intauto_0;
	u32 Inten_1;
	u32 Intpol_1;
	u32 Intedge_1;
	u32 Intbounce_1;
	u32 Intauto_1;
	u8  Entry[64];
};

static struct ls2k_iointc_irq_registers ls2k_irq_regs;
#endif

#ifdef CONFIG_PM
int ls2k_iointc_suspend_irq(void)
{
	unsigned long base;
	int index;

	base = CKSEG1ADDR(CONF_BASE) + INT_LO_OFF;

	ls2k_irq_regs.Inten_0 = ls64_conf_read32((void *)(base + INT_EN_OFF));
	ls2k_irq_regs.Intpol_0 = ls64_conf_read32((void *)(base + INT_PLE_OFF));
	ls2k_irq_regs.Intedge_0 = ls64_conf_read32((void *)(base + INT_EDG_OFF));
	ls2k_irq_regs.Intbounce_0 = ls64_conf_read32((void *)(base + INT_BCE_OFF));
	ls2k_irq_regs.Intauto_0 = ls64_conf_read32((void *)(base + INT_AUTO_OFF));

	ls2k_irq_regs.Inten_1 = ls64_conf_read32((void *)(base + 0x40 + INT_EN_OFF));
	ls2k_irq_regs.Intpol_1 = ls64_conf_read32((void *)(base + 0x40 + INT_PLE_OFF));
	ls2k_irq_regs.Intedge_1 = ls64_conf_read32((void *)(base + 0x40 + INT_EDG_OFF));
	ls2k_irq_regs.Intbounce_1 = ls64_conf_read32((void *)(base + 0x40 + INT_BCE_OFF));
	ls2k_irq_regs.Intauto_1 = ls64_conf_read32((void *)(base + 0x40 + INT_AUTO_OFF));

	for (index = 0; index < 32; index++)
		ls2k_irq_regs.Entry[index] = ls64_conf_read8((void *)(base + index));
	for (index = 0; index < 32; index++)
		ls2k_irq_regs.Entry[index + 32] = ls64_conf_read8((void *)(base + 0x40 + index));

	return 0;
}

void ls2k_iointc_resume_irq(void)
{
	unsigned long base;
	int index;

	base = CKSEG1ADDR(CONF_BASE) + INT_LO_OFF;

	ls64_conf_write32(ls2k_irq_regs.Inten_0, (void *)(base + INT_SET_OFF));
	ls64_conf_write32(ls2k_irq_regs.Intpol_0,   (void *)(base + INT_PLE_OFF));
	ls64_conf_write32(ls2k_irq_regs.Intedge_0,  (void *)(base + INT_EDG_OFF));
	ls64_conf_write32(ls2k_irq_regs.Intbounce_0, (void *)(base + INT_BCE_OFF));
	ls64_conf_write32(ls2k_irq_regs.Intauto_0,   (void *)(base + INT_AUTO_OFF));

	ls64_conf_write32(ls2k_irq_regs.Inten_1, (void *)(base + 0x40 + INT_SET_OFF));
	ls64_conf_write32(ls2k_irq_regs.Intpol_1,   (void *)(base + 0x40 + INT_PLE_OFF));
	ls64_conf_write32(ls2k_irq_regs.Intedge_1,  (void *)(base + 0x40 + INT_EDG_OFF));
	ls64_conf_write32(ls2k_irq_regs.Intbounce_1, (void *)(base + 0x40 + INT_BCE_OFF));
	ls64_conf_write32(ls2k_irq_regs.Intauto_1,   (void *)(base + 0x40 + INT_AUTO_OFF));

	for (index = 0; index < 32; index++)
		ls64_conf_write8(ls2k_irq_regs.Entry[index], (void *)(base + index));
	for (index = 0; index < 32; index++)
		ls64_conf_write8(ls2k_irq_regs.Entry[index + 32], (void *)(base + 0x40 + index));
}
#else
#define ls2k_iointc_suspend_irq NULL
#define ls2k_iointc_resume_irq NULL
#endif

void ls2k_iointc_set_irq_route(int irq, struct cpumask *core_mask, int line)
{
	unsigned long bounce_reg = LS2K_INTBOUNCE_REG(irq);
	unsigned long auto_reg = LS2K_INTAUTO_REG(irq);
	unsigned long *mask;
	unsigned int weight;
	unsigned int reg_val;

	if (irq > 63) {
		printk(KERN_ERR "%s: interrupt number error\n", __func__);
		return;
	}

	mask = cpumask_bits(core_mask);
	ls64_conf_write8((1 << (line+4)) | (*mask & 0xf), (void *)LS2K_IRQ_ROUTE_REG(irq));

	reg_val = ls64_conf_read32((void *)auto_reg);
	reg_val &= ~(1 << (irq % 32)); /* auto always set to 0 */
	ls64_conf_write32(reg_val, (void *)auto_reg);

	reg_val = ls64_conf_read32((void *)bounce_reg);

	weight = cpumask_weight(core_mask);
	if (weight > 1)
		reg_val |= (1 << (irq % 32));
	else
		reg_val &= ~(1 << (irq % 32));
	ls64_conf_write32(reg_val, (void *)bounce_reg);
}

void mask_ls2k_iointc_irq(struct irq_data *data)
{
	unsigned long hwirq = data->hwirq - LS2K_IOINTC_HWIRQ_BASE;
	unsigned long reg = LS2K_INTENCLR_REG(hwirq);
	unsigned long flags;

	raw_spin_lock_irqsave(&ls2k_irq_lock, flags);
	ls64_conf_write32(1<<(hwirq%32), (void *)reg);
	raw_spin_unlock_irqrestore(&ls2k_irq_lock, flags);

}

void unmask_ls2k_iointc_irq(struct irq_data *data)
{
	unsigned long hwirq = data->hwirq - LS2K_IOINTC_HWIRQ_BASE;
	unsigned long reg = LS2K_INTENSET_REG(hwirq);
	unsigned long flags;

	raw_spin_lock_irqsave(&ls2k_irq_lock, flags);
	ls64_conf_write32(1<<(hwirq % 32), (void *)reg);
	raw_spin_unlock_irqrestore(&ls2k_irq_lock, flags);
}

int ls2k_iointc_set_affinity(struct irq_data *d, const struct cpumask *affinity,
		bool force)
{
	unsigned long hwirq = d->hwirq - LS2K_IOINTC_HWIRQ_BASE;
	cpumask_t tmask;
	unsigned long *mask;
	unsigned long flags;

	cpumask_and(&tmask, affinity, cpu_online_mask);
	cpumask_copy(d->common->affinity, &tmask);

	mask = cpumask_bits(&tmask);
	raw_spin_lock_irqsave(&ls2k_irq_lock, flags);

	ls2k_iointc_set_irq_route(hwirq, &tmask, LS2K_IOINTC_IRQ_LINE);

	raw_spin_unlock_irqrestore(&ls2k_irq_lock, flags);

	irq_data_update_effective_affinity(d, &tmask);

	return IRQ_SET_MASK_OK_NOCOPY;
}

int ls2k_iointc_set_type(struct irq_data *d, unsigned int type)
{
	unsigned long hwirq = d->hwirq - LS2K_IOINTC_HWIRQ_BASE;
	unsigned long flags;
	unsigned int pol_val, edge_val;

	raw_spin_lock_irqsave(&ls2k_irq_lock, flags);
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
		raw_spin_unlock_irqrestore(&ls2k_irq_lock, flags);
		return -EINVAL;
	}

	writel(pol_val, (void *)LS2K_INTPOL_REG(hwirq));
	writel(edge_val, (void *)LS2K_INTEDGE_REG(hwirq));
	raw_spin_unlock_irqrestore(&ls2k_irq_lock, flags);

	return 0;
}

void ls2k_iointc_irq_ack(struct irq_data *d)
{
	unsigned long hwirq = d->hwirq - LS2K_IOINTC_HWIRQ_BASE;
	unsigned long reg;
	unsigned long flags;
	unsigned int reg_val;

	raw_spin_lock_irqsave(&ls2k_irq_lock, flags);

	reg = LS2K_INTEN_REG(hwirq);
	reg_val = readl((void *)reg);

	reg = LS2K_INTENCLR_REG(hwirq);
	writel(1<<(hwirq%32), (void *)reg);

	if (reg_val & (1 << (hwirq%32)))
		reg = LS2K_INTENSET_REG(hwirq);
	else
		reg = LS2K_INTENCLR_REG(hwirq);
	writel(1<<(hwirq%32), (void *)reg);
	raw_spin_unlock_irqrestore(&ls2k_irq_lock, flags);
}

static struct irq_chip ls2k_iointc = {
	.name		= "2k-iointc",
	.irq_mask	= mask_ls2k_iointc_irq,
	.irq_unmask	= unmask_ls2k_iointc_irq,
	.irq_set_affinity = ls2k_iointc_set_affinity,
	.irq_ack	= ls2k_iointc_irq_ack,
	.irq_set_type = ls2k_iointc_set_type,
	.flags = IRQCHIP_SKIP_SET_WAKE,
};

static int ls2k_iointc_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hw)
{
	irq_set_chip_and_handler(irq, &ls2k_iointc, handle_level_irq);
	return 0;
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

static const struct irq_domain_ops ls2k_iointc_irq_domain_ops = {
	.map = ls2k_iointc_map,
	.xlate = ls2k_iointc_irq_domain_xlate,
};

int ls2k_iointc_irq_of_init(struct device_node *of_node,
				struct device_node *parent)
{
	unsigned int virq;

	virq = irq_alloc_descs(LS2K_IRQ_BASE, LS2K_IRQ_BASE, 64, 0);
	ls2k_iointc_irq_domain = irq_domain_add_legacy(of_node, 64, virq, LS2K_IOINTC_HWIRQ_BASE,
							&ls2k_iointc_irq_domain_ops, NULL);
	if (!ls2k_iointc_irq_domain)
		panic("Failed to add irqdomain for ls2k io intc");

	return 0;
}
EXPORT_SYMBOL(ls2k_iointc_irq_of_init);

static struct syscore_ops ls2k_iointc_irq_syscore_ops = {
	.suspend	= ls2k_iointc_suspend_irq,
	.resume		= ls2k_iointc_resume_irq,
};

static int __init ls2k_iointc_init_syscore_ops(void)
{
	register_syscore_ops(&ls2k_iointc_irq_syscore_ops);
	return 0;
}
device_initcall(ls2k_iointc_init_syscore_ops);
IRQCHIP_DECLARE(ls2k_iointc, "loongson,2k1000-iointc", ls2k_iointc_irq_of_init);
IRQCHIP_DECLARE(ls2k_icu, "loongson,2k-icu", ls2k_iointc_irq_of_init);
