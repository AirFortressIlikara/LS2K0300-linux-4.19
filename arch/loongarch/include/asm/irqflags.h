/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */
#ifndef _ASM_IRQFLAGS_H
#define _ASM_IRQFLAGS_H

#ifndef __ASSEMBLY__

#include <linux/compiler.h>
#include <linux/stringify.h>
#include <asm/loongarchregs.h>

static inline void arch_local_irq_disable(void)
{
	csr_xchg32(0, CSR_CRMD_IE, LOONGARCH_CSR_CRMD);
}

static inline void arch_local_irq_enable(void)
{
	csr_xchg32(CSR_CRMD_IE, CSR_CRMD_IE, LOONGARCH_CSR_CRMD);
}

static inline unsigned long arch_local_irq_save(void)
{
	return csr_xchg32(0, CSR_CRMD_IE, LOONGARCH_CSR_CRMD);
}

static inline void arch_local_irq_restore(unsigned long flags)
{
	if (flags & CSR_CRMD_IE)
		csr_xchg32(flags, CSR_CRMD_IE, LOONGARCH_CSR_CRMD);
}

static inline unsigned long arch_local_save_flags(void)
{
	return csr_read32(LOONGARCH_CSR_CRMD);
}

static inline int arch_irqs_disabled_flags(unsigned long flags)
{
	return !(flags & CSR_CRMD_IE);
}

#endif /* #ifndef __ASSEMBLY__ */

/*
 * Do the CPU's IRQ-state tracing from assembly code.
 */
#ifdef CONFIG_TRACE_IRQFLAGS
/* Reload some registers clobbered by trace_hardirqs_on */
# define TRACE_IRQS_RELOAD_REGS						\
	LONG_L	$r11, sp, PT_R11;					\
	LONG_L	$r10, sp, PT_R10;					\
	LONG_L	$r9, sp, PT_R9;						\
	LONG_L	$r8, sp, PT_R8;						\
	LONG_L	$r7, sp, PT_R7;						\
	LONG_L	$r6, sp, PT_R6;						\
	LONG_L	$r5, sp, PT_R5;						\
	LONG_L	$r4, sp, PT_R4
# define TRACE_IRQS_ON							\
	CLI;	/* make sure trace_hardirqs_on() is called in kernel level */ \
	la.abs	t0, trace_hardirqs_on;					\
	jirl    ra, t0, 0
# define TRACE_IRQS_ON_RELOAD						\
	TRACE_IRQS_ON;							\
	TRACE_IRQS_RELOAD_REGS
# define TRACE_IRQS_OFF							\
	la.abs	t0, trace_hardirqs_off;					\
	jirl    ra, t0, 0
#else
# define TRACE_IRQS_ON
# define TRACE_IRQS_ON_RELOAD
# define TRACE_IRQS_OFF
#endif

#endif /* _ASM_IRQFLAGS_H */
