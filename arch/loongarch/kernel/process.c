// SPDX-License-Identifier: GPL-2.0
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994 - 1999, 2000 by Ralf Baechle and others.
 * Copyright (C) 2005, 2006 by Ralf Baechle (ralf@linux-mips.org)
 * Copyright (C) 1999, 2000 Silicon Graphics, Inc.
 * Copyright (C) 2004 Thiemo Seufer
 * Copyright (C) 2013  Imagination Technologies Ltd.
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/sched/debug.h>
#include <linux/sched/task.h>
#include <linux/sched/task_stack.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/stddef.h>
#include <linux/unistd.h>
#include <linux/export.h>
#include <linux/ptrace.h>
#include <linux/mman.h>
#include <linux/personality.h>
#include <linux/sys.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/kallsyms.h>
#include <linux/random.h>
#include <linux/prctl.h>
#include <linux/nmi.h>
#include <linux/cpu.h>

#include <asm/asm.h>
#include <asm/bootinfo.h>
#include <asm/cpu.h>
#include <asm/fpu.h>
#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/loongarchregs.h>
#include <asm/processor.h>
#include <asm/reg.h>
#include <linux/uaccess.h>
#include <asm/io.h>
#include <asm/elf.h>
#include <asm/inst.h>
#include <asm/stacktrace.h>
#include <asm/irq_regs.h>
#include <asm/unwind.h>
#include <asm/vdso.h>
#include <asm/lbt.h>

#ifdef CONFIG_HOTPLUG_CPU
void arch_cpu_idle_dead(void)
{
	play_dead();
}
#endif

asmlinkage void ret_from_fork(void);
asmlinkage void ret_from_kernel_thread(void);

void start_thread(struct pt_regs * regs, unsigned long pc, unsigned long sp)
{
	unsigned long crmd;
	unsigned long prmd;
	unsigned long euen;

	/* New thread loses kernel privileges. */
	crmd = regs->csr_crmd & ~(PLV_MASK);
	crmd |= PLV_USER;
	regs->csr_crmd = crmd;

	prmd = regs->csr_prmd & ~(PLV_MASK);
	prmd |= PLV_USER;
	regs->csr_prmd = prmd;

	euen = regs->csr_euen & ~(CSR_EUEN_FPEN);
	regs->csr_euen = euen;
	lose_fpu(0);
	lose_lbt(0);

	__clear_thread_flag(TIF_LSX_CTX_LIVE);
	__clear_thread_flag(TIF_LASX_CTX_LIVE);
	__clear_thread_flag(TIF_LBT_CTX_LIVE);
	clear_used_math();
	regs->csr_era = pc;
	regs->regs[3] = sp;

	current_thread_info()->is_exec = 1;
	smp_wmb();
}

/*
 * Idle related variables and functions
 */

unsigned long boot_option_idle_override = IDLE_NO_OVERRIDE;
EXPORT_SYMBOL(boot_option_idle_override);
void exit_thread(struct task_struct *tsk)
{
}

int arch_dup_task_struct(struct task_struct *dst, struct task_struct *src)
{
	/*
	 * Save any process state which is live in hardware registers to the
	 * parent context prior to duplication. This prevents the new child
	 * state becoming stale if the parent is preempted before copy_thread()
	 * gets a chance to save the parent's live hardware registers to the
	 * child context.
	 */
	preempt_disable();

	if (is_fpu_owner()) {
		if (is_lasx_enabled())
			save_lasx(current);
		else if (is_lsx_enabled())
			save_lsx(current);
		else
			save_fp(current);
	}

	preempt_enable();

	/* do not copy fpr registers if have not used math */
	if (used_math())
		memcpy(dst, src, sizeof(struct task_struct));
	else
		memcpy(dst, src, offsetof(struct task_struct, thread.fpu.fpr));

	return 0;
}

/*
 * Copy architecture-specific thread state
 */
int copy_thread_tls(unsigned long clone_flags, unsigned long usp,
	unsigned long kthread_arg, struct task_struct *p, unsigned long tls)
{
	struct thread_info *ti = task_thread_info(p);
	struct pt_regs *childregs, *regs = current_pt_regs();
	unsigned long childksp;
	unsigned long csr_crmd;
	unsigned long csr_euen;
	unsigned long csr_prmd;
	unsigned long csr_ecfg;

	childksp = (unsigned long)task_stack_page(p) + THREAD_SIZE;

	/* set up new TSS. */
	childregs = (struct pt_regs *) childksp - 1;
	/*  Put the stack after the struct pt_regs.  */
	childksp = (unsigned long) childregs;
	/* Clear __schedule() sched_cfa */
	p->thread.sched_cfa = 0;
	if (unlikely(p->flags & PF_KTHREAD)) {
		/* kernel thread */
		p->thread.csr_euen = 0;
		p->thread.csr_crmd = csr_read32(LOONGARCH_CSR_CRMD);
		p->thread.csr_prmd = csr_read32(LOONGARCH_CSR_PRMD) | CSR_PRMD_PIE;
		p->thread.csr_ecfg = csr_read32(LOONGARCH_CSR_ECFG);
		csr_crmd = p->thread.csr_crmd;
		csr_euen = p->thread.csr_euen;
		csr_prmd = p->thread.csr_prmd;
		csr_ecfg = p->thread.csr_ecfg;

		memset(childregs, 0, sizeof(struct pt_regs));
		ti->addr_limit = KERNEL_DS;
		p->thread.reg23 = usp; /* fn */
		p->thread.reg24 = kthread_arg;
		p->thread.reg03 = childksp;
		p->thread.reg01 = (unsigned long) ret_from_kernel_thread;
		p->thread.sched_ra = (unsigned long) ret_from_kernel_thread;
		csr_crmd &= ~CSR_CRMD_IE;
		childregs->csr_euen = csr_euen;
		childregs->csr_prmd = csr_prmd;
		childregs->csr_ecfg = csr_ecfg;
		childregs->csr_crmd = csr_crmd;
		return 0;
	}

	/* user thread */
	*childregs = *regs;
	childregs->csr_prmd &= ~CSR_PRMD_PPLV;
	childregs->csr_prmd |= PLV_USER;
	childregs->csr_crmd &= ~CSR_CRMD_PLV;
	childregs->csr_crmd |= PLV_USER;

	childregs->regs[4] = 0; /* Child gets zero as return value */
	if (usp)
		childregs->regs[3] = usp;
	ti->addr_limit = USER_DS;

	p->thread.reg03 = (unsigned long) childregs;
	p->thread.reg01 = (unsigned long) ret_from_fork;
	p->thread.sched_ra = (unsigned long) ret_from_fork;

	/*
	 * New tasks lose permission to use the fpu. This accelerates context
	 * switching for most programs since they don't use the fpu.
	 */
	childregs->csr_euen = 0;

	__clear_tsk_thread_flag(p, TIF_USEDFPU);
	__clear_tsk_thread_flag(p, TIF_USEDSIMD);
	__clear_tsk_thread_flag(p, TIF_USEDLBT);
	__clear_tsk_thread_flag(p, TIF_LSX_CTX_LIVE);
	__clear_tsk_thread_flag(p, TIF_LASX_CTX_LIVE);
	__clear_tsk_thread_flag(p, TIF_LBT_CTX_LIVE);
	task_thread_info(p)->is_exec = 0;

	if (clone_flags & CLONE_SETTLS)
		childregs->regs[2] = tls;

	smp_wmb();
	return 0;
}

#ifdef CONFIG_STACKPROTECTOR
#include <linux/stackprotector.h>
unsigned long __stack_chk_guard __read_mostly;
EXPORT_SYMBOL(__stack_chk_guard);
#endif

bool in_task_stack(unsigned long stack, struct task_struct *task,
			struct stack_info *info)
{
	unsigned long begin = (unsigned long)task_stack_page(task);
	unsigned long end = begin + THREAD_SIZE;

	if (stack < begin || stack >= end)
		return false;

	info->type = STACK_TYPE_TASK;
	info->begin = begin;
	info->end = end;
	info->next_sp = 0;

	return true;
}

bool in_irq_stack(unsigned long stack, struct stack_info *info)
{
	unsigned long nextsp;
	unsigned long begin = (unsigned long)this_cpu_read(irq_stack);
	unsigned long end = begin + IRQ_STACK_START;

	if (stack < begin || stack >= end)
		return false;

	nextsp = *(unsigned long *)end;
	if (nextsp & (SZREG - 1))
		return false;

	info->type = STACK_TYPE_IRQ;
	info->begin = begin;
	info->end = end;
	info->next_sp = nextsp;

	return true;
}

int get_stack_info(unsigned long stack, struct task_struct *task,
		   struct stack_info *info)
{
	task = task ? : current;

	if (!stack || stack & (SZREG - 1))
		goto unknown;

	if (in_task_stack(stack, task, info))
		return 0;

	if (task != current)
		goto unknown;

	if (in_irq_stack(stack, info))
		return 0;

unknown:
	info->type = STACK_TYPE_UNKNOWN;
	return -EINVAL;
}

unsigned long get_wchan(struct task_struct *task)
{
	unsigned long stack_page;
	unsigned long pc;
	unsigned long stack_page_end __maybe_unused;
	unsigned long frame __maybe_unused;
	struct unwind_state state __maybe_unused;

	if (!task || task == current || task->state == TASK_RUNNING)
		return 0;

	stack_page = (unsigned long)try_get_task_stack(task);
	if (!stack_page)
		return 0;

	pc = thread_saved_ra(task);

#ifdef CONFIG_UNWINDER_GUESS
	/*
	 * Use "guess" unwinder get wchan is not reliable.
	 * Just calculate a simpler wchan value here.
	 * Do nothing here.
	 */
	return 0;
#endif

	stack_page_end = stack_page + THREAD_SIZE;
	frame = thread_saved_fp(task);

	if (frame >= stack_page_end || frame < stack_page)
		goto out;

	memset(&state, 0, sizeof(state));
	state.task = task;
	state.sp = frame;
	state.stack_info.type = STACK_TYPE_TASK;
	state.stack_info.begin = stack_page;
	state.stack_info.end = stack_page_end;
	state.stack_info.next_sp = 0;
	state.pc = pc;
#ifdef CONFIG_UNWINDER_PROLOGUE
	state.enable = true;
	state.first = true;
#endif

	while (in_sched_functions(pc)) {
		if (!unwind_next_frame(&state)) {
			pc = 0;
			goto out;
		}
		pc = unwind_get_return_address(&state);
	}

out:
	put_task_stack(task);
	return pc;
}

unsigned long loongarch_stack_top(void)
{
	unsigned long top = TASK_SIZE & PAGE_MASK;

	/* Space for the VDSO, data page & GIC user page */
	top -= PAGE_ALIGN(current->thread.vdso->size);
	top -= PAGE_SIZE;

	/* Space to randomize the VDSO base */
	if (current->flags & PF_RANDOMIZE)
		top -= VDSO_RANDOMIZE_SIZE;

	return top;
}

/*
 * Don't forget that the stack pointer must be aligned on a 8 bytes
 * boundary for 32-bits ABI and 16 bytes for 64-bits ABI.
 */
unsigned long arch_align_stack(unsigned long sp)
{
	if (!(current->personality & ADDR_NO_RANDOMIZE) && randomize_va_space)
		sp -= get_random_int() & ~PAGE_MASK;

	return sp & STACK_ALIGN;
}

static DEFINE_PER_CPU(call_single_data_t, backtrace_csd);
static struct cpumask backtrace_csd_busy;

static void handle_backtrace(void *info)
{
	nmi_cpu_backtrace(get_irq_regs());
	cpumask_clear_cpu(smp_processor_id(), &backtrace_csd_busy);
}

static void raise_backtrace(cpumask_t *mask)
{
	call_single_data_t *csd;
	int cpu;

	for_each_cpu(cpu, mask) {
		/*
		 * If we previously sent an IPI to the target CPU & it hasn't
		 * cleared its bit in the busy cpumask then it didn't handle
		 * our previous IPI & it's not safe for us to reuse the
		 * call_single_data_t.
		 */
		if (cpumask_test_and_set_cpu(cpu, &backtrace_csd_busy)) {
			pr_warn("Unable to send backtrace IPI to CPU%u - perhaps it hung?\n",
				cpu);
			continue;
		}

		csd = &per_cpu(backtrace_csd, cpu);
		csd->func = handle_backtrace;
		smp_call_function_single_async(cpu, csd);
	}
}

void arch_trigger_cpumask_backtrace(const cpumask_t *mask, bool exclude_self)
{
	nmi_trigger_cpumask_backtrace(mask, exclude_self, raise_backtrace);
}

int loongarch_get_process_fp_mode(struct task_struct *task)
{
	return PR_FP_MODE_FR;
}

int loongarch_set_process_fp_mode(struct task_struct *task, unsigned int value)
{
	return 0;
}

#ifdef CONFIG_64BIT
void loongarch_dump_regs64(u64 *uregs, const struct pt_regs *regs)
{
	unsigned int i;

	for (i = LOONGARCH_EF_R0; i <= LOONGARCH_EF_R31; i++) {
		uregs[i] = regs->regs[i];
	}

	uregs[LOONGARCH_EF_CSR_ERA] = regs->csr_era;
	uregs[LOONGARCH_EF_CSR_BADV] = regs->csr_badv;
	uregs[LOONGARCH_EF_CSR_CRMD] = regs->csr_crmd;
	uregs[LOONGARCH_EF_CSR_PRMD] = regs->csr_prmd;
	uregs[LOONGARCH_EF_CSR_EUEN] = regs->csr_euen;
	uregs[LOONGARCH_EF_CSR_ECFG] = regs->csr_ecfg;
	uregs[LOONGARCH_EF_CSR_ESTAT] = regs->csr_estat;
}
#endif /* CONFIG_64BIT */
