// SPDX-License-Identifier: GPL-2.0
#include <loongson.h>
#include <irq.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>

#include <asm/irq_cpu.h>
#include <asm/i8259.h>
#include <asm/mipsregs.h>
#include <asm/setup.h>
#include <boot_param.h>
#include <loongson-pch.h>
#include <loongson.h>
#include "smp.h"

void do_dispatch(void);
void do_cpu_cascade(void);
void (*do_cascade)(void) = do_dispatch;
void (*do_liointc)(void) = do_cpu_cascade;

DEFINE_SPINLOCK(bitmap_lock);
DECLARE_BITMAP(ipi_irq_in_use, MAX_DIRQS);

unsigned int irq_cpu[MAX_IRQS] = {[0 ... MAX_IRQS-1] = -1};
unsigned int ipi_irq2pos[MAX_IRQS] = { [0 ... MAX_IRQS-1] = -1 };
unsigned int ipi_pos2irq[MAX_DIRQS] = { [0 ... MAX_DIRQS-1] = -1 };

void do_cpu_cascade(void)
{
	do_IRQ(LOONGSON_LINTC_IRQ);
}

void do_cpu_dispatch(void)
{
	unsigned int irq = LOONGSON_INT_ROUTER_INTSTS;
	if (irq & (1 << LOONGSON_CPU_UART0_VEC))
		do_IRQ(LOONGSON_LINTC_IRQ);
	if (irq & (1 << LOONGSON_CPU_THSENS_VEC))
		do_IRQ(LOONGSON_THSENS_IRQ);
}

void do_dispatch(void)
{
	loongson_pch->irq_dispatch();
}

void do_pch(void)
{
	do_IRQ(LOONGSON_BRIDGE_IRQ);
}

int create_ipi_dirq(unsigned int irq)
{
	int pos;

	pos = find_first_zero_bit(ipi_irq_in_use, MAX_DIRQS);

	if (pos == MAX_DIRQS)
		return -ENOSPC;

	ipi_pos2irq[pos] = irq;
	ipi_irq2pos[irq] = pos;
	set_bit(pos, ipi_irq_in_use);
	return 0;
}

void destroy_ipi_dirq(unsigned int irq)
{
	int pos;

	pos = ipi_irq2pos[irq];

	if (pos < 0)
		return;

	ipi_irq2pos[irq] = -1;
	ipi_pos2irq[pos] = -1;
	clear_bit(pos, ipi_irq_in_use);
}

static DEFINE_SPINLOCK(affinity_lock);

int def_set_irq_affinity(struct irq_data *d, const struct cpumask *affinity,
			  bool force)
{
	struct cpumask new_affinity;
	unsigned long flags;

	if (!IS_ENABLED(CONFIG_SMP))
		return -EPERM;

	if(ipi_irq2pos[d->irq] == -1)
		return -EINVAL;

	spin_lock_irqsave(&affinity_lock, flags);

	if (!cpumask_intersects(affinity, cpu_online_mask)) {
		spin_unlock_irqrestore(&affinity_lock, flags);
		return -EINVAL;
	} else {
		cpumask_and(&new_affinity, affinity, cpu_online_mask);
	}

	cpumask_copy(d->common->affinity, &new_affinity);

	spin_unlock_irqrestore(&affinity_lock, flags);
	irq_data_update_effective_affinity(d, &new_affinity);
	return IRQ_SET_MASK_OK_NOCOPY;
}

#define UNUSED_IPS_GUEST (CAUSEF_IP0)
#define UNUSED_IPS (CAUSEF_IP5 | CAUSEF_IP4 | CAUSEF_IP0)
static void ip2_irqdispatch(void)
{
	do_liointc();
}

static void ip3_irqdispatch(void)
{
	do_cascade();
}

#ifdef CONFIG_SMP
static void ip6_irqdispatch(void)
{
	loongson3_ipi_interrupt(NULL);
}
#endif

static void ip7_irqdispatch(void)
{
	do_IRQ(LOONGSON_TIMER_IRQ);
}

void mach_irq_dispatch(unsigned int pending)
{
	if (pending & CAUSEF_IP7)
		do_IRQ(LOONGSON_TIMER_IRQ);
#if defined(CONFIG_SMP)
	if (pending & CAUSEF_IP6)
		loongson3_ipi_interrupt(NULL);
#endif
	if (pending & CAUSEF_IP3)
		do_cascade();
	if (pending & CAUSEF_IP2)
		do_liointc();
	if (pending & UNUSED_IPS) {
		pr_err("%s : spurious interrupt\n", __func__);
		spurious_interrupt();
	}
}

void __init mach_init_irq(void)
{
	loongson_pch->init_irq();

	if (cpu_has_vint) {
		pr_info("Setting up vectored interrupts\n");
		set_vi_handler(2, ip2_irqdispatch);
		set_vi_handler(3, ip3_irqdispatch);
#ifdef CONFIG_SMP
		set_vi_handler(6, ip6_irqdispatch);
#endif
		set_vi_handler(7, ip7_irqdispatch);
	}

	if (cpu_guestmode)
		set_c0_status(STATUSF_IP2 | STATUSF_IP3 |
					STATUSF_IP1 | STATUSF_IP6);
	else {
		set_c0_status(STATUSF_IP2 | STATUSF_IP3 | STATUSF_IP6);
		if (current_cpu_type() == CPU_LOONGSON3_COMP) {
			if (((*(volatile unsigned int *)0x900000003ff00404) & 0xf00000) != 0xf00000)
				(*(volatile unsigned int *)0x900000003ff00404) |= 0xf00000;
		}
	}
}

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
			chip->irq_set_affinity(&desc->irq_data, desc->irq_data.common->affinity, true);
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
