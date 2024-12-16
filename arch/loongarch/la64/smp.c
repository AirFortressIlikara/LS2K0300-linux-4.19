// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2010, 2011, 2012, Lemote, Inc.
 * Author: Chen Huacai, chenhc@lemote.com
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/sched/hotplug.h>
#include <linux/sched/task_stack.h>
#include <linux/seq_file.h>
#include <linux/smp.h>
#include <linux/syscore_ops.h>
#include <linux/acpi.h>
#include <linux/tracepoint.h>
#include <asm/processor.h>
#include <asm/time.h>
#include <asm/tlbflush.h>
#include <asm/cacheflush.h>
#include <asm/numa.h>
#include <loongson.h>
#include <asm/delay.h>
#include <loongson-pch.h>

#include <larchintrin.h>

static DEFINE_PER_CPU(int, cpu_state);

#define MAX_CPUS min(64, NR_CPUS)

static u32 (*ipi_read_clear)(int cpu);
static void (*ipi_write_action)(int cpu, u32 action);

enum ipi_msg_type {
	IPI_RESCHEDULE,
	IPI_CALL_FUNC,
};

static const char *ipi_types[NR_IPI] __tracepoint_string = {
	[IPI_RESCHEDULE] = "Rescheduling interrupts",
	[IPI_CALL_FUNC] = "Call Function interrupts",
};

void show_ipi_list(struct seq_file *p, int prec)
{
	unsigned int cpu, i;

	for (i = 0; i < NR_IPI; i++) {
		seq_printf(p, "%*s%u:%s", prec - 1, "IPI", i,
			   prec >= 4 ? " " : "");
		for_each_online_cpu(cpu)
			seq_printf(p, "%10u ",
				   __get_irq_stat(cpu, ipi_irqs[i]));
		seq_printf(p, "      %s\n", ipi_types[i]);
	}
}

/* Send mail buffer via Mail_Send */
static void csr_mail_send(uint64_t data, int cpu, int mailbox)
{
	uint64_t val;

	/* Send high 32 bits */
	val = IOCSR_MBUF_SEND_BLOCKING;
	val |= (IOCSR_MBUF_SEND_BOX_HI(mailbox) << IOCSR_MBUF_SEND_BOX_SHIFT);
	val |= (cpu << IOCSR_MBUF_SEND_CPU_SHIFT);
	val |= (data & IOCSR_MBUF_SEND_H32_MASK);
	iocsr_write64(val, LOONGARCH_IOCSR_MBUF_SEND);

	/* Send low 32 bits */
	val = IOCSR_MBUF_SEND_BLOCKING;
	val |= (IOCSR_MBUF_SEND_BOX_LO(mailbox) << IOCSR_MBUF_SEND_BOX_SHIFT);
	val |= (cpu << IOCSR_MBUF_SEND_CPU_SHIFT);
	val |= (data << IOCSR_MBUF_SEND_BUF_SHIFT);
	iocsr_write64(val, LOONGARCH_IOCSR_MBUF_SEND);
};

static u32 csr_ipi_read_clear(int cpu)
{
	u32 action;

	/* Load the ipi register to figure out what we're supposed to do */
	action = iocsr_read32(LOONGARCH_IOCSR_IPI_STATUS);
	/* Clear the ipi register to clear the interrupt */
	iocsr_write32(action, LOONGARCH_IOCSR_IPI_CLEAR);

	return action;
}

static void csr_ipi_write_action(int cpu, u32 action)
{
	unsigned int irq = 0;

	while ((irq = ffs(action))) {
		uint32_t val = IOCSR_IPI_SEND_BLOCKING;
		val |= (irq - 1);
		val |= (cpu << IOCSR_IPI_SEND_CPU_SHIFT);
		iocsr_write32(val, LOONGARCH_IOCSR_IPI_SEND);
		action &= ~BIT(irq - 1);
	}
}

static void ipi_method_init(void)
{
	ipi_read_clear = csr_ipi_read_clear;
	ipi_write_action = csr_ipi_write_action;
}

/*
 * Simple enough, just poke the appropriate ipi register
 */
static void loongson3_send_ipi_single(int cpu, unsigned int action)
{
	ipi_write_action(cpu_logical_map(cpu), (u32)action);
}

static void
loongson3_send_ipi_mask(const struct cpumask *mask, unsigned int action)
{
	unsigned int i;

	for_each_cpu(i, mask)
		ipi_write_action(cpu_logical_map(i), (u32)action);
}

void loongson3_ipi_interrupt(int irq)
{
	unsigned int action;
	unsigned int cpu = smp_processor_id();

	action = ipi_read_clear(cpu_logical_map(cpu));

	mb();

	if (action & SMP_RESCHEDULE) {
		__inc_irq_stat(cpu, ipi_irqs[IPI_RESCHEDULE]);
		scheduler_ipi();
	}

	if (action & SMP_CALL_FUNCTION) {
		__inc_irq_stat(cpu, ipi_irqs[IPI_CALL_FUNC]);
		irq_enter();
		generic_smp_call_function_interrupt();
		irq_exit();
	}
}

/*
 * SMP init and finish on secondary CPUs
 */
static void loongson3_init_secondary(void)
{
	unsigned int cpu = smp_processor_id();
	unsigned int imask = ECFGF_TIMER | ECFGF_IPI | ECFGF_IP2 | ECFGF_IP1 | ECFGF_IP0 | ECFGF_PC;

	/* Set interrupt mask, but don't enable */
	change_csr_ecfg(ECFG0_IM, imask);

	iocsr_write32(0xffffffff, LOONGARCH_IOCSR_IPI_EN);
	per_cpu(cpu_state, cpu) = CPU_ONLINE;
	cpu_data[cpu].core =
		     cpu_logical_map(cpu) % loongson_sysconf.cores_per_package;
	cpu_data[cpu].package =
			cpu_logical_map(cpu) / loongson_sysconf.cores_per_package;

	if (cpu_has_extioi)
		extioi_init();
}

static void loongson3_smp_finish(void)
{

	local_irq_enable();

	iocsr_write64(0, LOONGARCH_IOCSR_MBUF0);
	pr_info("CPU#%d finished\n", smp_processor_id());
}

static void __init loongson3_smp_setup(void)
{
	int i = 0, num = 0; /* i: physical id, num: logical id */

	if (acpi_disabled) {
		init_cpu_possible(cpu_none_mask);

		while (i < MAX_CPUS) {
			if (loongson_sysconf.reserved_cpus_mask & (0x1UL << i)) {
				/* Reserved physical CPU cores */
				__cpu_number_map[i] = -1;
			} else {
				__cpu_number_map[i] = num;
				__cpu_logical_map[num] = i;
				set_cpu_possible(num, true);
				num++;
			}
			i++;
		}
		pr_info("Detected %i available CPU(s)\n", num);

		while (num < MAX_CPUS) {
			__cpu_logical_map[num] = -1;
			num++;
		}
	}

	ipi_method_init();

	iocsr_write32(0xffffffff, LOONGARCH_IOCSR_IPI_EN);

	cpu_data[0].core = cpu_logical_map(0) % loongson_sysconf.cores_per_package;
	cpu_data[0].package = cpu_logical_map(0) / loongson_sysconf.cores_per_package;
}

static void __init loongson3_prepare_cpus(unsigned int max_cpus)
{
	int i = 0;

	for (i = 0; i < loongson_sysconf.nr_cpus; i++) {
		set_cpu_present(i, true);

		csr_mail_send(0, __cpu_logical_map[i], 0);
	}

	per_cpu(cpu_state, smp_processor_id()) = CPU_ONLINE;
}

/*
 * Setup the PC, SP, and TP of a secondary processor and start it runing!
 */
static int loongson3_boot_secondary(int cpu, struct task_struct *idle)
{
	unsigned long entry;

	pr_info("Booting CPU#%d...\n", cpu);

	/* write PC entry in hw mailbox for secondary CPU */
	entry = (unsigned long)&smp_bootstrap;

	if (!efi_bp)
		entry = __pa_symbol(entry);

	csr_mail_send(entry, cpu_logical_map(cpu), 0);

	/* send ipi to secondary processor */
	loongson3_send_ipi_single(cpu, SMP_BOOT_CPU);
	return 0;
}

#ifdef CONFIG_HOTPLUG_CPU
static bool is_unplug_cpu(int cpu)
{
	int i, node, logical_cpu;

	if (cpu == 0)
		return true;

	for (i = 1; i < nr_pch_pics; i++) {
		node = eiointc_get_node(i);
		logical_cpu = cpu_number_map(node * CORES_PER_EXTIOI_NODE);
		if (cpu == logical_cpu)
			return true;
	}

	return false;
}

static int loongson3_cpu_disable(void)
{
	unsigned long flags;
	unsigned int cpu = smp_processor_id();

	if (is_unplug_cpu(cpu)) {
		pr_warn("CPU %u is master cpu of node group. Cannot disable CPU\n", cpu);
		return -EBUSY;
	}

	numa_remove_cpu(cpu);
	set_cpu_online(cpu, false);
	calculate_cpu_foreign_map();
	local_irq_save(flags);
	fixup_irqs();
	local_irq_restore(flags);
	local_flush_tlb_all();

	return 0;
}


static void loongson3_cpu_die(unsigned int cpu)
{
	while (per_cpu(cpu_state, cpu) != CPU_DEAD)
		cpu_relax();

	mb();
}

void play_dead(void)
{
	unsigned int action;
	void (*boot_cpu)(void);

	idle_task_exit();
	/* Tell __cpu_die() that this CPU is now safe to dispose of */
	__this_cpu_write(cpu_state, CPU_DEAD);

	/* enable ipi interrupt*/
	local_irq_enable();
	set_csr_ecfg(ECFGF_IPI);

	do {
		asm volatile("idle 0\n\t");
		boot_cpu = (void *)((u64)iocsr_read32(LOONGARCH_IOCSR_MBUF0));
	} while (boot_cpu == 0);
	boot_cpu = (void *)TO_UNCAC(iocsr_read64(LOONGARCH_IOCSR_MBUF0));

	/* clear ipi interrupt */
	action = iocsr_read32(LOONGARCH_IOCSR_IPI_STATUS);
	iocsr_write32(action, LOONGARCH_IOCSR_IPI_CLEAR);

	boot_cpu();
	unreachable();
}
#endif

const struct plat_smp_ops loongson3_smp_ops = {
	.send_ipi_single = loongson3_send_ipi_single,
	.send_ipi_mask = loongson3_send_ipi_mask,
	.smp_setup = loongson3_smp_setup,
	.prepare_cpus = loongson3_prepare_cpus,
	.boot_secondary = loongson3_boot_secondary,
	.init_secondary = loongson3_init_secondary,
	.smp_finish = loongson3_smp_finish,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_disable = loongson3_cpu_disable,
	.cpu_die = loongson3_cpu_die,
#endif
};

/*
 * Power management
 */
#ifdef CONFIG_PM

static int loongson3_ipi_suspend(void)
{
        return 0;
}

static void loongson3_ipi_resume(void)
{
	iocsr_write32(0xffffffff, LOONGARCH_IOCSR_IPI_EN);
}

static struct syscore_ops loongson3_ipi_syscore_ops = {
	.resume         = loongson3_ipi_resume,
	.suspend        = loongson3_ipi_suspend,
};

/*
 * Enable boot cpu ipi before enabling nonboot cpus
 * during syscore_resume.
 * */
static int __init ipi_pm_init(void)
{
	register_syscore_ops(&loongson3_ipi_syscore_ops);
        return 0;
}

core_initcall(ipi_pm_init);
#endif
