// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2013, Loongson Technology Corporation Limited, Inc.
 *  Copyright (C) 2014-2017, Lemote, Inc.
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irqchip.h>

#include <asm/irq_cpu.h>
#include <asm/i8259.h>
#include <asm/mipsregs.h>

#include <irq.h>
#include <loongson.h>
#include <loongson-pch.h>
#include "smp.h"

extern const struct plat_smp_ops *mp_ops;
extern void (*do_cascade)(void);
extern void (*do_liointc)(void);
extern void do_dispatch(void);
extern void do_cpu_dispatch(void);

static unsigned int ht_irq[] = {0, 1, 3, 4, 5, 6, 7, 8, 12, 14, 15};

void rs780_irq_dispatch(void)
{
	unsigned int i, irq;
	struct irq_data *irqd;
	struct cpumask affinity;

	irq = LOONGSON_HT1_INT_VECTOR(0);
	LOONGSON_HT1_INT_VECTOR(0) = irq; /* Acknowledge the IRQs */

	for (i = 0; i < ARRAY_SIZE(ht_irq); i++) {
		if (!(irq & (0x1 << ht_irq[i])))
			continue;

		/* handled by local core */
		if (ipi_irq2pos[ht_irq[i]] == -1) {
			do_IRQ(ht_irq[i]);
			continue;
		}

		irqd = irq_get_irq_data(ht_irq[i]);
		cpumask_and(&affinity, irqd->common->affinity, cpu_active_mask);
		if (cpumask_empty(&affinity)) {
			do_IRQ(ht_irq[i]);
			continue;
		}

		irq_cpu[ht_irq[i]] = cpumask_next(irq_cpu[ht_irq[i]], &affinity);
		if (irq_cpu[ht_irq[i]] >= nr_cpu_ids)
			irq_cpu[ht_irq[i]] = cpumask_first(&affinity);

		if (irq_cpu[ht_irq[i]] == 0) {
			do_IRQ(ht_irq[i]);
			continue;
		}

		/* balanced by other cores */
		mp_ops->send_ipi_single(irq_cpu[ht_irq[i]], (0x1 << (ipi_irq2pos[ht_irq[i]])) << IPI_IRQ_OFFSET);
	}
}

void rs780_msi_irq_dispatch(void)
{
	struct irq_data *irqd;
	struct cpumask affinity;
	unsigned int i, irq, irqs;

	/* Handle normal IRQs */
	for (i = 0; i < 4; i++) {
		irqs = LOONGSON_HT1_INT_VECTOR(i);
		LOONGSON_HT1_INT_VECTOR(i) = irqs; /* Acknowledge the IRQs */

		while (irqs) {
			irq = ffs(irqs) - 1;
			irqs &= ~(1 << irq);
			irq = (i * 32) + irq;

			if (irq >= MIPS_CPU_IRQ_BASE && irq < (MIPS_CPU_IRQ_BASE + 8)) {
				pr_err("spurious interrupt: IRQ%d.\n", irq);
				spurious_interrupt();
				continue;
			}

			/* handled by local core */
			if (ipi_irq2pos[irq] == -1) {
				do_IRQ(irq);
				continue;
			}

			irqd = irq_get_irq_data(irq);
			cpumask_and(&affinity, irqd->common->affinity, cpu_active_mask);
			if (cpumask_empty(&affinity)) {
				do_IRQ(irq);
				continue;
			}

			irq_cpu[irq] = cpumask_next(irq_cpu[irq], &affinity);
			if (irq_cpu[irq] >= nr_cpu_ids)
				irq_cpu[irq] = cpumask_first(&affinity);

			if (irq_cpu[irq] == 0) {
				do_IRQ(irq);
				continue;
			}

			/* balanced by other cores */
			mp_ops->send_ipi_single(irq_cpu[irq], (0x1 << (ipi_irq2pos[irq])) << IPI_IRQ_OFFSET);
		}
	}
}

static inline void mask_loongson_irq(struct irq_data *d) { }
static inline void unmask_loongson_irq(struct irq_data *d) { }

/* For MIPS IRQs which shared by all cores */
static struct irq_chip loongson_irq_chip = {
	.name           = "Loongson",
	.irq_ack        = mask_loongson_irq,
	.irq_mask       = mask_loongson_irq,
	.irq_mask_ack   = mask_loongson_irq,
	.irq_unmask     = unmask_loongson_irq,
	.irq_eoi        = unmask_loongson_irq,
};

void rs780_init_irq(void)
{
	int i;
	struct irq_chip *chip;

	do_cascade = do_dispatch;
	do_liointc = do_cpu_dispatch;
	clear_c0_status(ST0_IM | ST0_BEV);

	irq_alloc_descs(-1, MIPS_CPU_IRQ_BASE, 8, 0);
	for (i = MIPS_CPU_IRQ_BASE; i < MIPS_CPU_IRQ_BASE + 8; i++)
		irq_set_noprobe(i);
	irqchip_init();

	/* Route LPC and THM int to cpu Core0 INT0 */
	LOONGSON_INT_ROUTER_THM = LOONGSON_INT_COREx_INTy(loongson_sysconf.boot_cpu_id, 0);
	LOONGSON_INT_ROUTER_LPC = LOONGSON_INT_COREx_INTy(loongson_sysconf.boot_cpu_id, 0);
	/* Route HT1 int0 ~ int3 to cpu Core0 INT1 */
	for (i = 0; i < 4; i++) {
		LOONGSON_INT_ROUTER_HT1(i) = LOONGSON_INT_COREx_INTy(loongson_sysconf.boot_cpu_id, 1);
		/* Enable HT1 interrupts */
		LOONGSON_HT1_INTN_EN(i) = 0xffffffff;
	}
	/* Enable router interrupt intenset */
	LOONGSON_INT_ROUTER_INTENSET =
		LOONGSON_INT_ROUTER_INTEN | (0xffff << 16) | 0x1 << 10 | 0x1 << 14;

	/* Clear HT1 original interrupts */
	LOONGSON_HT1_INT_VECTOR(1) = 0xffffffff;
	LOONGSON_HT1_INT_VECTOR(2) = 0xffffffff;
	LOONGSON_HT1_INT_VECTOR(3) = 0xffffffff;

	chip = irq_get_chip(I8259A_IRQ_BASE);
	chip->irq_set_affinity = def_set_irq_affinity;

	if (pci_msi_enabled())
		loongson_pch->irq_dispatch = rs780_msi_irq_dispatch;
	else {
		for (i = 0; i < MAX_IRQS; i++)
			ipi_irq2pos[i] = -1;
		for (i = 0; i < MAX_DIRQS; i++)
			ipi_pos2irq[i] = -1;
		create_ipi_dirq(3);
		create_ipi_dirq(4);
		create_ipi_dirq(5);
		create_ipi_dirq(6);
		create_ipi_dirq(14);
		create_ipi_dirq(15);
	}

	irq_set_chip_and_handler(LOONGSON_THSENS_IRQ,
		&loongson_irq_chip, handle_percpu_irq);
	irq_set_chip_and_handler(LOONGSON_LINTC_IRQ,
		&loongson_irq_chip, handle_percpu_irq);
	irq_set_chip_and_handler(LOONGSON_BRIDGE_IRQ,
		&loongson_irq_chip, handle_percpu_irq);

}
