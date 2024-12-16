/* SPDX-License-Identifier: GPL-2.0 */
/*
 * asmmacro.h: Assembler macros to make things easier to read.
 *
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#ifndef _ASM_ASMMACRO_64_H
#define _ASM_ASMMACRO_64_H

#include <asm/asm-offsets.h>
#include <asm/regdef.h>
#include <asm/fpregdef.h>
#include <asm/loongarchregs.h>

	.macro	cpu_save_nonscratch thread
	stptr.d	s0, \thread, THREAD_REG23
	stptr.d	s1, \thread, THREAD_REG24
	stptr.d	s2, \thread, THREAD_REG25
	stptr.d	s3, \thread, THREAD_REG26
	stptr.d	s4, \thread, THREAD_REG27
	stptr.d	s5, \thread, THREAD_REG28
	stptr.d	s6, \thread, THREAD_REG29
	stptr.d	s7, \thread, THREAD_REG30
	stptr.d	s8, \thread, THREAD_REG31
	stptr.d	sp, \thread, THREAD_REG03
	stptr.d	fp, \thread, THREAD_REG22
	.endm

	.macro	cpu_restore_nonscratch thread
	ldptr.d	s0, \thread, THREAD_REG23
	ldptr.d	s1, \thread, THREAD_REG24
	ldptr.d	s2, \thread, THREAD_REG25
	ldptr.d	s3, \thread, THREAD_REG26
	ldptr.d	s4, \thread, THREAD_REG27
	ldptr.d	s5, \thread, THREAD_REG28
	ldptr.d	s6, \thread, THREAD_REG29
	ldptr.d	s7, \thread, THREAD_REG30
	ldptr.d	s8, \thread, THREAD_REG31
	ldptr.d	sp, \thread, THREAD_REG03
	ldptr.d	fp, \thread, THREAD_REG22
	ldptr.d	ra, \thread, THREAD_REG01
	.endm

#endif /* _ASM_ASMMACRO_64_H */
