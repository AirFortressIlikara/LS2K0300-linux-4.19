/* SPDX-License-Identifier: GPL-2.0+ */
/*
* Copyright (C) 2020 Loongson Technology Corporation Limited
*
* Author: Hanlu Li <lihanlu@loongson.cn>
*/

#ifndef __ASM_LOONGARCH_SYSCALL_H
#define __ASM_LOONGARCH_SYSCALL_H

#include <linux/compiler.h>
#include <uapi/linux/audit.h>
#include <linux/elf-em.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <asm/ptrace.h>
#include <asm/unistd.h>

extern void *sys_call_table[];

static inline long syscall_get_nr(struct task_struct *task,
				  struct pt_regs *regs)
{
	return regs->regs[11];
}

static inline void syscall_set_nr(struct task_struct *task,
				  struct pt_regs *regs,
				  int syscall)
{
	regs->regs[11] = syscall;
}

static inline long syscall_get_return_value(struct task_struct *task,
					    struct pt_regs *regs)
{
	return regs->regs[4];
}

static inline void syscall_rollback(struct task_struct *task,
				    struct pt_regs *regs)
{
	/* Do nothing */
}

static inline void syscall_set_return_value(struct task_struct *task,
					    struct pt_regs *regs,
					    int error, long val)
{
	regs->regs[4] = (long) error ? error : val;
}

static inline void syscall_get_arguments(struct task_struct *task,
					 struct pt_regs *regs,
					 unsigned int i, unsigned int n,
					 unsigned long *args)
{
	BUG_ON(i + n > 6);
	if (!n)
		return;
	if (i == 0) {
		args[0] = regs->orig_a0;
		args++;
		n--;
	} else {
		i--;
	}
	memcpy(args, &regs->regs[5] + i, n * sizeof(args[0]));
}

static inline int syscall_get_arch(void)
{
	return AUDIT_ARCH_LOONGARCH64;
}

#endif	/* __ASM_LOONGARCH_SYSCALL_H */
