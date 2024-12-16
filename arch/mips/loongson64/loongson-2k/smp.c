/*
 * smp.c - Add loongson-2K smp support.
 *
 * Copyright (C) 2020, Loongson Technology Corporation Limited, Inc.
 *
 * Authors Pei Huang <huangpei@loongson.cn>
 * Authors Ming Wang <wangming01@loongson.cn>
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/cpufreq.h>
#include <asm/processor.h>
#include <asm/time.h>
#include <asm/clock.h>
#include <asm/tlbflush.h>
#include <asm/smp.h>
#include <linux/sched/task_stack.h>
#include <loongson.h>
#include <loongson-2k.h>
#include <linux/sched/hotplug.h>
#include <linux/sched/task_stack.h>
#include <linux/cpufreq.h>
#include <linux/syscore_ops.h>
#include <asm/tlbflush.h>
#include <asm/cacheflush.h>
#include <irq.h>

DEFINE_PER_CPU(int, cpu_state);
static uint32_t core0_c0count[NR_CPUS];

/*
 * Simple enough, just poke the appropriate ipi register
 */
static void loongson_send_ipi_single(int cpu, unsigned int action)
{
	unsigned long base = IPI_BASE_OF(cpu_logical_map(cpu));
	ls64_conf_write32((u32)action, (void *)(base + IPI_OFF_SET));
}

static void loongson_send_ipi_mask(const struct cpumask *mask, unsigned int action)
{
	unsigned int i;
	for_each_cpu(i, mask) {
	unsigned long base = IPI_BASE_OF(cpu_logical_map(i));
		ls64_conf_write32((u32)action, (void *)(base + IPI_OFF_SET));
	}
}

void ls64_ipi_interrupt(struct pt_regs *regs)
{
	int i, cpu = smp_processor_id();
	unsigned long base = IPI_BASE_OF(cpu_logical_map(cpu));
	unsigned int action, c0count;

	/* Load the ipi register to figure out what we're supposed to do */
	action = ls64_conf_read32((void *)(base + IPI_OFF_STATUS));

	/* Clear the ipi register to clear the interrupt */
	ls64_conf_write32((u32)action, (void *)(base + IPI_OFF_CLEAR));

	if (action & SMP_RESCHEDULE_YOURSELF) {
		scheduler_ipi();
	}

	if (action & SMP_CALL_FUNCTION) {
		irq_enter();
		generic_smp_call_function_interrupt();
		irq_exit();
	}

	if (action & SMP_ASK_C0COUNT) {
		BUG_ON(cpu != 0);
		c0count = read_c0_count();
		c0count = c0count ? c0count : 1;
		for (i = 1; i < num_possible_cpus(); i++)
			core0_c0count[i] = c0count;
		__sync();
	}
}

#define MAX_LOOPS 800
/*
 * SMP init and finish on secondary CPUs
 */
void  loongson_init_secondary(void)
{
	int i;
	uint32_t initcount;
	unsigned int cpu = smp_processor_id();
	unsigned int imask = STATUSF_IP7 | STATUSF_IP6 | STATUSF_IP5 |
			     STATUSF_IP4 | STATUSF_IP3 | STATUSF_IP2;

	/* Set interrupt mask, but don't enable */
	change_c0_status(ST0_IM, imask);

	for (i = 0; i < num_possible_cpus(); i++) {
		unsigned long base = IPI_BASE_OF(cpu_logical_map(i));
		ls64_conf_write32(0xffffffff, (void *)(base + IPI_OFF_ENABLE));
	}

	per_cpu(cpu_state, cpu) = CPU_ONLINE;

	i = 0;
	core0_c0count[cpu] = 0;
	loongson_send_ipi_single(0, SMP_ASK_C0COUNT);
	while (!core0_c0count[cpu]) {
		i++;
		cpu_relax();
	}

	if (i > MAX_LOOPS)
		i = MAX_LOOPS;
	if (cpu_data[cpu].package)
		initcount = core0_c0count[cpu] + i;
	else /* Local access is faster for loops */
		initcount = core0_c0count[cpu] + i/2;
	write_c0_count(initcount);
}

void  loongson_smp_finish(void)
{
	int cpu = smp_processor_id();
	unsigned long base = IPI_BASE_OF(cpu_logical_map(cpu));
	write_c0_compare(read_c0_count() + mips_hpt_frequency/HZ);
	local_irq_enable();
	ls64_conf_write64(0, (void *)(base + IPI_OFF_MAILBOX0));
	if (system_state == SYSTEM_BOOTING)
		printk("CPU#%d finished, CP0_ST=%x\n",
			smp_processor_id(), read_c0_status());
}

void __init loongson_smp_setup(void)
{
	int i = 0;
	int core_id = (read_c0_ebase() & 0x3ff);

	init_cpu_possible(cpu_none_mask);

	/* For unified kernel, NR_CPUS is the maximum possible value,
	 * nr_cpus_loongson is the really present value */
	while (i < NR_CPUS) {
		__cpu_number_map[i] = i;
		__cpu_logical_map[i] = i;
		set_cpu_possible(i, true);
		i++;
	}
	__cpu_logical_map[0] = core_id;
	__cpu_number_map[0] = core_id;
	__cpu_logical_map[core_id] = 0;
	__cpu_number_map[core_id] = 0;

	printk(KERN_INFO "Detected %i available CPU(s)\n", i);
}

void __init loongson_prepare_cpus(unsigned int max_cpus)
{
	init_cpu_present(cpu_possible_mask);
	per_cpu(cpu_state, smp_processor_id()) = CPU_ONLINE;
}

/*
 * Setup the PC, SP, and GP of a secondary processor and start it runing!
 */
int loongson_boot_secondary(int cpu, struct task_struct *idle)
{
	volatile unsigned long startargs[4];
	unsigned long coreid = cpu_logical_map(cpu);
	unsigned long base = IPI_BASE_OF(coreid);

	if (system_state == SYSTEM_BOOTING)
		printk("Booting CPU#%d...\n", cpu);

	/* startargs[] are initial PC, SP and GP for secondary CPU */
	startargs[0] = (unsigned long)&smp_bootstrap;
	startargs[1] = (unsigned long)__KSTK_TOS(idle);
	startargs[2] = (unsigned long)task_thread_info(idle);
	startargs[3] = 0;

	if (system_state == SYSTEM_BOOTING)
		printk("CPU#%d, func_pc=%lx, sp=%lx, gp=%lx\n",
			cpu, startargs[0], startargs[1], startargs[2]);

	ls64_conf_write64(startargs[3], (void *)(base + IPI_OFF_MAILBOX3));
	ls64_conf_write64(startargs[2], (void *)(base + IPI_OFF_MAILBOX2));
	ls64_conf_write64(startargs[1], (void *)(base + IPI_OFF_MAILBOX1));
	ls64_conf_write64(startargs[0], (void *)(base + IPI_OFF_MAILBOX0));

	return 0;
}

#ifdef CONFIG_HOTPLUG_CPU

extern void fixup_irqs(void);
extern void (*flush_cache_all)(void);

static int loongson_cpu_disable(void)
{
	unsigned long flags;
	unsigned int cpu = smp_processor_id();

	if (cpu == 0)
		return -EBUSY;

	set_cpu_online(cpu, false);
	calculate_cpu_foreign_map();
	local_irq_save(flags);
	fixup_irqs();
	local_irq_restore(flags);
	flush_cache_all();
	local_flush_tlb_all();

	return 0;
}

static void loongson_cpu_die(unsigned int cpu)
{
	while (per_cpu(cpu_state, cpu) != CPU_DEAD)
		cpu_relax();

	mb();
}

static void loongson2k_play_dead(int *state_addr)
{
	register int val;
	register long cpuid, core, node, count;
	register void *addr, *base, *initfunc;

	__asm__ __volatile__(
		"   .set push                     \n"
		"   .set noreorder                \n"
		"   li %[addr], 0x80000000        \n" /* KSEG0 */
		"1: cache 0, 0(%[addr])           \n" /* flush L1 ICache */
		"   cache 0, 1(%[addr])           \n"
		"   cache 0, 2(%[addr])           \n"
		"   cache 0, 3(%[addr])           \n"
		"   cache 1, 0(%[addr])           \n" /* flush L1 DCache */
		"   cache 1, 1(%[addr])           \n"
		"   cache 1, 2(%[addr])           \n"
		"   cache 1, 3(%[addr])           \n"
		"   addiu %[sets], %[sets], -1    \n"
		"   bnez  %[sets], 1b             \n"
		"   addiu %[addr], %[addr], 0x40  \n"
		"   li    %[val], 0x7             \n" /* *state_addr = CPU_DEAD; */
		"   sw    %[val], (%[state_addr]) \n"
		"   sync                          \n"
		"   cache 21, (%[state_addr])     \n" /* flush entry of *state_addr */
		"   .set pop                      \n"
		: [addr] "=&r" (addr), [val] "=&r" (val)
		: [state_addr] "r" (state_addr),
		  [sets] "r" (cpu_data[smp_processor_id()].dcache.sets));

	__asm__ __volatile__(
		"   .set push                         \n"
		"   .set noreorder                    \n"
		"   .set mips64                       \n"
		"   mfc0  %[cpuid], $15, 1            \n"
		"   andi  %[cpuid], 0x3ff             \n"
		"   dli   %[base], 0x900000001fe11000 \n"
		"   andi  %[core], %[cpuid], 0x3      \n"
		"   sll   %[core], 8                  \n" /* get core id */
		"   or    %[base], %[base], %[core]   \n"
		"   andi  %[node], %[cpuid], 0xc      \n"
		"   dsll  %[node], 42                 \n" /* get node id */
		"   or    %[base], %[base], %[node]   \n"
		"1: li    %[count], 0x100             \n" /* wait for init loop */
		"2: bnez  %[count], 2b                \n" /* limit mailbox access */
		"   addiu %[count], -1                \n"
		"   ld    %[initfunc], 0x20(%[base])  \n" /* get PC via mailbox */
		"   beqz  %[initfunc], 1b             \n"
		"   nop                               \n"
		"   ld    $sp, 0x28(%[base])          \n" /* get SP via mailbox */
		"   ld    $gp, 0x30(%[base])          \n" /* get GP via mailbox */
		"   ld    $a1, 0x38(%[base])          \n"
		"   jr    %[initfunc]                 \n" /* jump to initial PC */
		"   nop                               \n"
		"   .set pop                          \n"
		: [core] "=&r" (core), [node] "=&r" (node),
		  [base] "=&r" (base), [cpuid] "=&r" (cpuid),
		  [count] "=&r" (count), [initfunc] "=&r" (initfunc)
		: /* No Input */
		: "a1");
}

void play_dead(void)
{
	int *state_addr;
	unsigned int cpu = smp_processor_id();
	void (*play_dead_at_ckseg1)(int *);

	idle_task_exit();
	switch (current_cpu_type()) {
	case CPU_LOONGSON2K:
	default:
		play_dead_at_ckseg1 = (void *)CKSEG1ADDR((unsigned long)loongson2k_play_dead);
		break;
	}

	state_addr = &per_cpu(cpu_state, cpu);
	mb();
	play_dead_at_ckseg1(state_addr);
}
#endif

struct plat_smp_ops loongson_smp_ops = {
	.send_ipi_single = loongson_send_ipi_single,
	.send_ipi_mask = loongson_send_ipi_mask,
	.init_secondary = loongson_init_secondary,
	.smp_finish = loongson_smp_finish,
	.boot_secondary = loongson_boot_secondary,
	.smp_setup = loongson_smp_setup,
	.prepare_cpus = loongson_prepare_cpus,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_disable = loongson_cpu_disable,
	.cpu_die = loongson_cpu_die,
#endif
};
