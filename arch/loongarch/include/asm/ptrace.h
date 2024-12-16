/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994, 95, 96, 97, 98, 99, 2000 by Ralf Baechle
 * Copyright (C) 1999, 2000 Silicon Graphics, Inc.
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#ifndef _ASM_PTRACE_H
#define _ASM_PTRACE_H


#include <linux/compiler.h>
#include <linux/linkage.h>
#include <linux/types.h>
#include <asm/page.h>
#include <asm/thread_info.h>
#include <uapi/asm/ptrace.h>

/*
 * This struct defines the way the registers are stored on the stack during a
 * system call/exception.
 *
 * If you add a register here, also add it to regoffset_table[] in
 * arch/loongarch/kernel/ptrace.c.
 */
struct pt_regs {
	/* Saved main processor registers. */
	unsigned long regs[32];

	/* Saved special registers. */
	unsigned long csr_era;
	unsigned long csr_badv;
	unsigned long csr_crmd;
	unsigned long csr_prmd;
	unsigned long csr_euen;
	unsigned long csr_ecfg;
	unsigned long csr_estat;
	unsigned long orig_a0;
	unsigned long __last[];
} __attribute__ ((aligned (8)));


static inline unsigned long kernel_stack_pointer(struct pt_regs *regs)
{
	//TODO kernel_stack_pointer is ra?
	return regs->regs[3];
}

/*
 * Don't use asm-generic/ptrace.h it defines FP accessors that don't make
 * sense on LoongArch.  We rather want an error if they get invoked.
 */

static inline void instruction_pointer_set(struct pt_regs *regs,
                                           unsigned long val)
{
	regs->csr_era = val;
}

/* Query offset/name of register from its name/offset */
extern int regs_query_register_offset(const char *name);
#define MAX_REG_OFFSET (offsetof(struct pt_regs, __last))

/**
 * regs_get_register() - get register value from its offset
 * @regs:       pt_regs from which register value is gotten.
 * @offset:     offset number of the register.
 *
 * regs_get_register returns the value of a register. The @offset is the
 * offset of the register in struct pt_regs address which specified by @regs.
 * If @offset is bigger than MAX_REG_OFFSET, this returns 0.
 */
static inline unsigned long regs_get_register(struct pt_regs *regs,
                                              unsigned int offset)
{
	if (unlikely(offset > MAX_REG_OFFSET))
		return 0;

	return *(unsigned long *)((unsigned long)regs + offset);
}

/**
 * regs_within_kernel_stack() - check the address in the stack
 * @regs:       pt_regs which contains kernel stack pointer.
 * @addr:       address which is checked.
 *
 * regs_within_kernel_stack() checks @addr is within the kernel stack page(s).
 * If @addr is within the kernel stack, it returns true. If not, returns false.
 */
static inline int regs_within_kernel_stack(struct pt_regs *regs,
                                           unsigned long addr)
{
	return ((addr & ~(THREAD_SIZE - 1))  ==
		(kernel_stack_pointer(regs) & ~(THREAD_SIZE - 1)));
}

/**
 * regs_get_kernel_stack_nth() - get Nth entry of the stack
 * @regs:       pt_regs which contains kernel stack pointer.
 * @n:          stack entry number.
 *
 * regs_get_kernel_stack_nth() returns @n th entry of the kernel stack which
 * is specified by @regs. If the @n th entry is NOT in the kernel stack,
 * this returns 0.
 */
static inline unsigned long regs_get_kernel_stack_nth(struct pt_regs *regs,
                                                      unsigned int n)
{
	unsigned long *addr = (unsigned long *)kernel_stack_pointer(regs);

	addr += n;
	if (regs_within_kernel_stack(regs, (unsigned long)addr))
		return *addr;
	else
		return 0;
}

struct task_struct;

/*
 * Does the process account for user or for system time?
 */
#define user_mode(regs) (((regs)->csr_prmd & PLV_MASK) == PLV_USER)

static inline long regs_return_value(struct pt_regs *regs)
{
	return regs->regs[4];
}

#define instruction_pointer(regs) ((regs)->csr_era)
#define profile_pc(regs) instruction_pointer(regs)

extern asmlinkage long syscall_trace_enter(struct pt_regs *regs, long syscall);
extern asmlinkage void syscall_trace_leave(struct pt_regs *regs);

extern void die(const char *, struct pt_regs *) __noreturn;

static inline void die_if_kernel(const char *str, struct pt_regs *regs)
{
	if (unlikely(!user_mode(regs)))
		die(str, regs);
}

#define current_pt_regs()						\
({									\
	unsigned long sp = (unsigned long)__builtin_frame_address(0);	\
	(struct pt_regs *)((sp | (THREAD_SIZE - 1)) + 1) - 1;		\
})

/* Helpers for working with the user stack pointer */

static inline unsigned long user_stack_pointer(struct pt_regs *regs)
{
	return regs->regs[3];
}

static inline void user_stack_pointer_set(struct pt_regs *regs,
	unsigned long val)
{
	regs->regs[3] = val;
}

#define arch_has_single_step()  (1)

#endif /* _ASM_PTRACE_H */
