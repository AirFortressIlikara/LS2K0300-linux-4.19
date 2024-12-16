/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General
 * Public License.  See the file "COPYING" in the main directory of this
 * archive for more details.
 * Author: Huacai Chen <chenhuacai@loongson.cn>
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */
#ifndef __ASM_SMP_H
#define __ASM_SMP_H

#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/linkage.h>
#include <linux/smp.h>
#include <linux/threads.h>
#include <linux/cpumask.h>
#include <linux/errno.h>
#include <linux/cpumask.h>

#ifdef CONFIG_SMP

struct task_struct;

struct plat_smp_ops {
	void (*send_ipi_single)(int cpu, unsigned int action);
	void (*send_ipi_mask)(const struct cpumask *mask, unsigned int action);
	void (*init_secondary)(void);
	void (*smp_finish)(void);
	int (*boot_secondary)(int cpu, struct task_struct *idle);
	void (*smp_setup)(void);
	void (*prepare_cpus)(unsigned int max_cpus);
#ifdef CONFIG_HOTPLUG_CPU
	int (*cpu_disable)(void);
	void (*cpu_die)(unsigned int cpu);
#endif
};

extern struct plat_smp_ops *mp_ops;
void register_smp_ops(const struct plat_smp_ops *ops);

static inline void plat_smp_setup(void)
{
	mp_ops->smp_setup();
}

#else /* !CONFIG_SMP */

struct plat_smp_ops;

static inline void plat_smp_setup(void)
{
	/* UP, nothing to do ...  */
}

static inline void register_smp_ops(const struct plat_smp_ops *ops)
{
}

#endif /* !CONFIG_SMP */

extern int smp_num_siblings;
extern int num_processors;
extern int disabled_cpus;
extern cpumask_t cpu_sibling_map[];
extern cpumask_t cpu_core_map[];
extern cpumask_t cpu_foreign_map[];

static inline int raw_smp_processor_id(void)
{
#if defined(__VDSO__)
	extern int vdso_smp_processor_id(void)
		__compiletime_error("VDSO should not call smp_processor_id()");
	return vdso_smp_processor_id();
#else
	return current_thread_info()->cpu;
#endif
}
#define raw_smp_processor_id raw_smp_processor_id

/* Map from cpu id to sequential logical cpu number.  This will only
   not be idempotent when cpus failed to come on-line.	*/
extern int __cpu_number_map[NR_CPUS];
#define cpu_number_map(cpu)  __cpu_number_map[cpu]

/* The reverse map from sequential logical cpu number to cpu id.  */
extern int __cpu_logical_map[NR_CPUS];
#define cpu_logical_map(cpu)  __cpu_logical_map[cpu]

#define cpu_physical_id(cpu)	cpu_logical_map(cpu)
#define SMP_BOOT_CPU		0x1
#define SMP_RESCHEDULE		0x2
#define SMP_CALL_FUNCTION	0x4

/*
 * Initial data for bringing up a secondary CPU.
 * @stack	- sp for the secondary CPU
 * @thread_info - threadinfo for the secondary CPU
 */
struct secondary_data {
	unsigned long stack;
	unsigned long thread_info;
};
extern struct secondary_data cpuboot_data;

/* Mask of CPUs which are currently definitely operating coherently */
extern cpumask_t cpu_coherent_mask;

extern asmlinkage void smp_bootstrap(void);

extern void calculate_cpu_foreign_map(void);

/*
 * generate IPI list text
 */
extern void show_ipi_list(struct seq_file *p, int prec);

/*
 * this function sends a 'reschedule' IPI to another CPU.
 * it goes straight through and wastes no time serializing
 * anything. Worst case is that we lose a reschedule ...
 */
static inline void smp_send_reschedule(int cpu)
{
	mp_ops->send_ipi_single(cpu, SMP_RESCHEDULE);
}

#ifdef CONFIG_HOTPLUG_CPU
static inline int __cpu_disable(void)
{
	return mp_ops->cpu_disable();
}

static inline void __cpu_die(unsigned int cpu)
{
	mp_ops->cpu_die(cpu);
}

extern void play_dead(void);
#endif

static inline void arch_send_call_function_single_ipi(int cpu)
{
	mp_ops->send_ipi_single(cpu, SMP_CALL_FUNCTION);
}

static inline void arch_send_call_function_ipi_mask(const struct cpumask *mask)
{
	mp_ops->send_ipi_mask(mask, SMP_CALL_FUNCTION);
}

#endif /* __ASM_SMP_H */
