/* SPDX-License-Identifier: GPL-2.0 */
/* thread_info.h: LoongArch low-level thread information
 *
 * Copyright (C) 2002  David Howells (dhowells@redhat.com)
 * - Incorporating suggestions made by Linus Torvalds and Dave Miller
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */

#ifndef _ASM_THREAD_INFO_H
#define _ASM_THREAD_INFO_H

#ifdef __KERNEL__

#ifndef __ASSEMBLY__

#include <asm/processor.h>

/*
 * low level task data that entry.S needs immediate access to
 * - this struct should fit entirely inside of one cache line
 * - this struct shares the supervisor stack pages
 * - if the contents of this structure are changed, the assembly constants
 *   must also be changed
 */
struct thread_info {
	struct task_struct	*task;		/* main task structure */
	unsigned long		flags;		/* low level flags */
#ifdef CONFIG_CPU_HAS_LBT
	unsigned long		tp_value;	/* thread pointer */
#endif
	__u32			cpu;		/* current CPU */
	int			preempt_count;	/* 0 => preemptable, <0 => BUG */
	mm_segment_t		addr_limit;	/*
						 * thread address space limit:
						 * 0x7fffffff for user-thead
						 * 0xffffffff for kernel-thread
						 */
	struct pt_regs		*regs;
	long			syscall;	/* syscall number */

	unsigned long		will_wait:1;
	unsigned long		is_exec:1;
	unsigned long		will_wake:1;

	struct task_struct	*last_waker;
	unsigned long		wakee_times;
	unsigned long		last_waketime;

	unsigned long		wait_num;
	unsigned long		last_waittime;
};

/*
 * macros/functions for gaining access to the thread information structure
 */
#define INIT_THREAD_INFO(tsk)			\
{						\
	.task		= &tsk,			\
	.flags		= _TIF_FIXADE,		\
	.cpu		= 0,			\
	.preempt_count	= INIT_PREEMPT_COUNT,	\
	.addr_limit	= KERNEL_DS,		\
}

/* How to get the thread information struct from C. */
register struct thread_info *__current_thread_info __asm__("$r2");

static inline struct thread_info *current_thread_info(void)
{
	return __current_thread_info;
}

#endif /* !__ASSEMBLY__ */

/* thread information allocation */
#if defined(CONFIG_PAGE_SIZE_4KB) && defined(CONFIG_32BIT)
#define THREAD_SIZE_ORDER (1)
#endif
#if defined(CONFIG_PAGE_SIZE_4KB) && defined(CONFIG_64BIT)
#define THREAD_SIZE_ORDER (2)
#endif
#ifdef CONFIG_PAGE_SIZE_16KB
#define THREAD_SIZE_ORDER (0)
#endif
#ifdef CONFIG_PAGE_SIZE_64KB
#define THREAD_SIZE_ORDER (0)
#endif

#define THREAD_SIZE (PAGE_SIZE << THREAD_SIZE_ORDER)
#define THREAD_MASK (THREAD_SIZE - 1UL)

#define STACK_WARN	(THREAD_SIZE / 8)

/*
 * thread information flags
 * - these are process state flags that various assembly files may need to
 *   access
 * - pending work-to-be-done flags are in LSW
 * - other flags in MSW
 */
#define TIF_SIGPENDING		1	/* signal pending */
#define TIF_NEED_RESCHED	2	/* rescheduling necessary */
#define TIF_NOTIFY_RESUME	3	/* callback before returning to user */
#define TIF_RESTORE_SIGMASK	4	/* restore signal mask in do_signal() */
#define TIF_NOHZ		5	/* in adaptive nohz mode */
#define TIF_SYSCALL_AUDIT	6	/* syscall auditing active */
#define TIF_SYSCALL_TRACE	7	/* syscall trace active */
#define TIF_SYSCALL_TRACEPOINT	8	/* syscall tracepoint instrumentation */
#define TIF_SECCOMP		9	/* secure computing */
#define TIF_UPROBE		10	/* breakpointed or singlestepping */
#define TIF_USEDFPU		11	/* FPU was used by this task this quantum (SMP) */
#define TIF_USEDSIMD		12	/* SIMD has been used this quantum */
#define TIF_MEMDIE		13	/* is terminating due to OOM killer */
#define TIF_FIXADE		14	/* Fix address errors in software */
#define TIF_LOGADE		15	/* Log address errors to syslog */
#define TIF_32BIT_REGS		16	/* 32-bit general purpose registers */
#define TIF_32BIT_ADDR		17	/* 32-bit address space */
#define TIF_FPUBOUND		18	/* thread bound to FPU-full CPU set */
#define TIF_LOAD_WATCH		19	/* If set, load watch registers */
#define TIF_LSX_CTX_LIVE	20	/* LSX context must be preserved */
#define TIF_LASX_CTX_LIVE	21	/* LASX context must be preserved */
#define TIF_PATCH_PENDING	22	/* pending live patching update */
#define TIF_USEDLBT		23	/* LBT has been used */
#define TIF_LBT_CTX_LIVE	24	/* LBT context */
#define TIF_SINGLESTEP		25

#define _TIF_SIGPENDING		(1<<TIF_SIGPENDING)
#define _TIF_NEED_RESCHED	(1<<TIF_NEED_RESCHED)
#define _TIF_NOTIFY_RESUME	(1<<TIF_NOTIFY_RESUME)
#define _TIF_NOHZ		(1<<TIF_NOHZ)
#define _TIF_SYSCALL_AUDIT	(1<<TIF_SYSCALL_AUDIT)
#define _TIF_SYSCALL_TRACE	(1<<TIF_SYSCALL_TRACE)
#define _TIF_SYSCALL_TRACEPOINT	(1<<TIF_SYSCALL_TRACEPOINT)
#define _TIF_SECCOMP		(1<<TIF_SECCOMP)
#define _TIF_UPROBE		(1<<TIF_UPROBE)
#define _TIF_USEDFPU		(1<<TIF_USEDFPU)
#define _TIF_USEDSIMD		(1<<TIF_USEDSIMD)
#define _TIF_FIXADE		(1<<TIF_FIXADE)
#define _TIF_LOGADE		(1<<TIF_LOGADE)
#define _TIF_32BIT_REGS		(1<<TIF_32BIT_REGS)
#define _TIF_32BIT_ADDR		(1<<TIF_32BIT_ADDR)
#define _TIF_FPUBOUND		(1<<TIF_FPUBOUND)
#define _TIF_LOAD_WATCH		(1<<TIF_LOAD_WATCH)
#define _TIF_LSX_CTX_LIVE	(1<<TIF_LSX_CTX_LIVE)
#define _TIF_LASX_CTX_LIVE	(1<<TIF_LASX_CTX_LIVE)
#define _TIF_PATCH_PENDING	(1<<TIF_PATCH_PENDING)
#define _TIF_USEDLBT		(1<<TIF_USEDLBT)
#define _TIF_LBT_CTX_LIVE	(1<<TIF_LBT_CTX_LIVE)
#define _TIF_SINGLESTEP		(1<<TIF_SINGLESTEP)

#define _TIF_WORK_SYSCALL_ENTRY	(_TIF_NOHZ | _TIF_SYSCALL_TRACE |	\
				 _TIF_SYSCALL_AUDIT | \
				 _TIF_SYSCALL_TRACEPOINT | _TIF_SECCOMP)

/* work to do in syscall_trace_leave() */
#define _TIF_WORK_SYSCALL_EXIT	(_TIF_NOHZ | _TIF_SYSCALL_TRACE |	\
				 _TIF_SYSCALL_AUDIT | _TIF_SYSCALL_TRACEPOINT)

/* work to do on interrupt/exception return */
#define _TIF_WORK_MASK		\
	(_TIF_SIGPENDING | _TIF_NEED_RESCHED | _TIF_NOTIFY_RESUME |	\
	 _TIF_UPROBE)
/* work to do on any return to u-space */
#define _TIF_ALLWORK_MASK	(_TIF_NOHZ | _TIF_WORK_MASK |		\
				 _TIF_WORK_SYSCALL_EXIT |		\
				 _TIF_SYSCALL_TRACEPOINT)

#endif /* __KERNEL__ */
#endif /* _ASM_THREAD_INFO_H */
