/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2014 Imagination Technologies Ltd
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * PM helper macros for CPU power off (e.g. Suspend-to-RAM).
 */

#ifndef __ASM_PM_H
#define __ASM_PM_H

#ifdef __ASSEMBLY__

#include <asm/asm-offsets.h>
#include <asm/asm.h>
#include <asm/loongarchregs.h>
#include <asm/regdef.h>

/* Save CPU state to stack for suspend to RAM */
.macro SUSPEND_SAVE_REGS
	subu	sp, PT_SIZE
	/* Call preserved GPRs */
	LONG_S	$16, sp, PT_R16
	LONG_S	$17, sp, PT_R17
	LONG_S	$18, sp, PT_R18
	LONG_S	$19, sp, PT_R19
	LONG_S	$20, sp, PT_R20
	LONG_S	$21, sp, PT_R21
	LONG_S	$22, sp, PT_R22
	LONG_S	$23, sp, PT_R23
	LONG_S	$28, sp, PT_R28
	LONG_S	$30, sp, PT_R30
	LONG_S	$31, sp, PT_R31
	/* A couple of CP0 registers with space in pt_regs */
	mfc0	k0, CP0_STATUS
	LONG_S	k0, sp, PT_STATUS
.endm

/* Restore CPU state from stack after resume from RAM */
.macro RESUME_RESTORE_REGS_RETURN
	/* A couple of CP0 registers with space in pt_regs */
	LONG_L	k0, sp, PT_STATUS
	mtc0	k0, CP0_STATUS
	/* Call preserved GPRs */
	LONG_L	$16, sp, PT_R16
	LONG_L	$17, sp, PT_R17
	LONG_L	$18, sp, PT_R18
	LONG_L	$19, sp, PT_R19
	LONG_L	$20, sp, PT_R20
	LONG_L	$21, sp, PT_R21
	LONG_L	$22, sp, PT_R22
	LONG_L	$23, sp, PT_R23
	LONG_L	$28, sp, PT_R28
	LONG_L	$30, sp, PT_R30
	LONG_L	$31, sp, PT_R31
	/* Pop and return */
	jr	ra
	 addiu	sp, PT_SIZE
.endm

/* Get address of static suspend state into t1 */
.macro LA_STATIC_SUSPEND
	la	t1, loongarch_static_suspend_state
.endm

/* Save important CPU state for early restoration to global data */
.macro SUSPEND_SAVE_STATIC
	/* save stack pointer (pointing to GPRs) */
	LONG_S	sp, t1, SSS_SP
.endm

/* Restore important CPU state early from global data */
.macro RESUME_RESTORE_STATIC
	/* restore stack pointer (pointing to GPRs) */
	LONG_L	sp, SSS_SP(t1)
.endm

/* flush caches to make sure context has reached memory */
.macro SUSPEND_CACHE_FLUSH
	.extern	__wback_cache_all
	la	t1, __wback_cache_all
	LONG_L	t0, 0(t1)
	jalr	t0
	 nop
 .endm

/* Save suspend state and flush data caches to RAM */
.macro SUSPEND_SAVE
	SUSPEND_SAVE_REGS
	LA_STATIC_SUSPEND
	SUSPEND_SAVE_STATIC
	SUSPEND_CACHE_FLUSH
.endm

/* Restore saved state after resume from RAM and return */
.macro RESUME_RESTORE_RETURN
	LA_STATIC_SUSPEND
	RESUME_RESTORE_STATIC
	RESUME_RESTORE_REGS_RETURN
.endm

#else /* __ASSEMBLY__ */

/**
 * struct loongarch_static_suspend_state - Core saved CPU state across S2R.
 * @segctl:	CP0 Segment control registers.
 * @sp:		Stack frame where TP register context is saved.
 *
 * This structure contains minimal CPU state that must be saved in static kernel
 * data in order to be able to restore the rest of the state. This includes
 * segmentation configuration in the case of EVA being enabled, as they must be
 * restored prior to any kmalloc'd memory being referenced (even the stack
 * pointer).
 */
struct loongarch_static_suspend_state {
	unsigned long sp;
};

#endif /* !__ASSEMBLY__ */

#endif /* __ASM_PM_HELPERS_H */
