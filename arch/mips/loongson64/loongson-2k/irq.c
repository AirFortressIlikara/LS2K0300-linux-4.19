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

extern struct irq_domain *ls2k_iointc_irq_domain;

#ifdef CONFIG_SMP
extern void ls64_ipi_interrupt(struct pt_regs *);
#endif

extern void ls2k_iointc_set_irq_route(int, struct cpumask *, int);

#ifdef CONFIG_HOTPLUG_CPU
void handle_irq_affinity(void)
{
	struct irq_desc *desc;
	struct irq_chip *chip;
	unsigned int irq;
	unsigned long flags;
	struct cpumask *affinity;

	for_each_active_irq(irq) {
		desc = irq_to_desc(irq);
		if (!desc)
			continue;

		raw_spin_lock_irqsave(&desc->lock, flags);

		affinity = desc->irq_data.common->affinity;

		if (!cpumask_intersects(affinity, cpu_online_mask))
			cpumask_copy(affinity, cpu_online_mask);

		chip = irq_data_get_irq_chip(&desc->irq_data);
		if (chip && chip->irq_set_affinity)
			chip->irq_set_affinity(&desc->irq_data, affinity, true);
		raw_spin_unlock_irqrestore(&desc->lock, flags);
	}
}

void fixup_irqs(void)
{
	handle_irq_affinity();
	irq_cpu_offline();
	clear_c0_status(ST0_IM);
}
#endif

static int ls2k_iointc_irq_handler(void)
{
	unsigned long pending, enabled;
	u32 reg_val;
	int irq;
	unsigned int virq;
	unsigned int core = read_c0_ebase() & 0x3ff;

	pending = readl((void *)LS2K_COREx_INTISR0_REG(core));
	reg_val = readl((void *)LS2K_COREx_INTISR1_REG(core));
	pending |= (unsigned long)reg_val << 32;

	enabled = readl((void *)LS2K_INTEN_REG(0));
	reg_val = readl((void *)LS2K_INTEN_REG(32));
	enabled |= (unsigned long)reg_val << 32;

	pending &= enabled;
	if (!pending)
		spurious_interrupt();

	while (pending) {
		irq = fls64(pending) - 1;
		virq = irq_linear_revmap(ls2k_iointc_irq_domain, irq + LS2K_IOINTC_HWIRQ_BASE);
		do_IRQ(virq);
		pending &= ~BIT(irq);
	}
	return 0;
}
static unsigned long ls2k_irq_count;

void mach_irq_dispatch(unsigned int pending)
{
	ls2k_irq_count++;
	if (pending & CAUSEF_IP2)
		do_IRQ(MIPS_CPU_IRQ_BASE + 2);
	if (pending & CAUSEF_IP3)
		ls2k_iointc_irq_handler();
#ifdef CONFIG_SMP
	if (pending & CAUSEF_IP6)
		ls64_ipi_interrupt(NULL);
#endif
	if (pending & CAUSEF_IP7)
		do_IRQ(MIPS_CPU_IRQ_BASE + 7);
}

void __init mach_init_irq(void)
{
	unsigned long reg, irq;
	struct cpumask core_mask;

	/* init mips cpu interrupt */
	irq_alloc_descs(MIPS_CPU_IRQ_BASE, MIPS_CPU_IRQ_BASE, 8, 0);
	mips_cpu_irq_init();
	set_c0_status(STATUSF_IP2 | STATUSF_IP3 | STATUSF_IP6);

	/* disable all peripheral interrupts */
	reg = LS2K_INTENCLR_REG(0);
	ls64_conf_write32(0xffffffff, (void *)reg);
	reg = LS2K_INTENCLR_REG(32);
	ls64_conf_write32(0xffffffff, (void *)reg);

	/* all peripheral interrupts route to core 0 IP3 */
	cpumask_clear(&core_mask);
	cpumask_set_cpu(0, &core_mask);

	for (irq = 0; irq < 64; irq++)
		ls2k_iointc_set_irq_route(irq, &core_mask, LS2K_IOINTC_IRQ_LINE);

	irqchip_init();
}

asmlinkage void plat_irq_dispatch(void)
{
	unsigned int pending;

	pending = read_c0_cause() & read_c0_status() & ST0_IM;

	/* machine-specific plat_irq_dispatch */
	mach_irq_dispatch(pending);
}

void __init arch_init_irq(void)
{
	/*
	 * Clear all of the interrupts while we change the able around a bit.
	 * int-handler is not on bootstrap
	 */
	clear_c0_status(ST0_IM | ST0_BEV);

	/* no steer */
	LOONGSON_INTSTEER = 0;

	/*
	 * Mask out all interrupt by writing "1" to all bit position in
	 * the interrupt reset reg.
	 */
	LOONGSON_INTENCLR = ~0;

	/* machine specific irq init */
	mach_init_irq();

}
