/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 *
 * Some useful macros for LoongArch assembler code
 *
 * Some of the routines below contain useless nops that will be optimized
 * away by gas in -O mode. These nops are however required to fill delay
 * slots in noreorder mode.
 */
#ifndef __ASM_ASM_H
#define __ASM_ASM_H

#include <asm/abidefs.h>

#ifndef CAT
#ifdef __STDC__
#define __CAT(str1, str2) str1##str2
#else
#define __CAT(str1, str2) str1/**/str2
#endif
#define CAT(str1, str2) __CAT(str1, str2)
#endif

/*
 * PIC specific declarations
 * Not used for the kernel but here seems to be the right place.
 */
#ifdef __PIC__
#define CPRESTORE(register)				\
		.cprestore register
#define CPADD(register)					\
		.cpadd	register
#define CPLOAD(register)				\
		.cpload register
#else
#define CPRESTORE(register)
#define CPADD(register)
#define CPLOAD(register)
#endif

/*
 * ABS - export absolute symbol
 */
#define ABS(symbol,value)				\
		.globl	symbol;				\
symbol		=	value

/*
 * Build text tables
 */
#define TTABLE(string)					\
		.pushsection .text;			\
		.word	1f;				\
		.popsection				\
		.pushsection .data;			\
1:		.asciz string;				\
		.popsection

/*
 * LoongArch pref instruction.
 * Use with .set noreorder only!
 *
 * LoongArch implementations are free to treat this as a nop.  Loongson-3
 * is one of them.  So we should have an option not to use this instruction.
 */
#ifdef CONFIG_CPU_HAS_PREFETCH

#define PREF(hint, addr, offs)				\
		preld	hint, addr, offs;		\

#define PREFX(hint, addr, index)			\
		preldx	hint, addr, index;		\

#else /* !CONFIG_CPU_HAS_PREFETCH */

#define PREF(hint, addr, offs)
#define PREFX(hint, addr, index)

#endif /* !CONFIG_CPU_HAS_PREFETCH */

#define MOVN(rd, rs, rt)				\
		maskeqz	rd, rs, rt
#define MOVZ(rd, rs, rt)				\
		masknez	rd, rs, rt

/*
 * Stack alignment
 */
#define STACK_ALIGN	~(0xf)

/*
 * Macros to handle different pointer/register sizes for 32/64-bit code
 */

/*
 * Size of a register
 */
#ifdef __loongarch64
#define SZREG	8
#else
#define SZREG	4
#endif

/*
 * Use the following macros in assemblercode to load/store registers,
 * pointers etc.
 */
#if (_LOONGARCH_SIM == _LOONGARCH_SIM_ABILP64)
#define REG_S		st.d
#define REG_L		ld.d
#define REG_SUBU	sub.d
#define REG_ADDU	add.d
#endif

/*
 * How to add/sub/load/store/shift C int variables.
 */
#if (_LOONGARCH_SZINT == 32)
#define INT_ADD		add.w
#define INT_ADDI	addi.w
#define INT_SUB		sub.w
#define INT_L		ld.w
#define INT_S		st.w
#define INT_SLL		slli.w
#define INT_SLLV	sll.w
#define INT_SRL		srli.w
#define INT_SRLV	srl.w
#define INT_SRA		srai.w
#define INT_SRAV	sra.w
#endif

#if (_LOONGARCH_SZINT == 64)
#define INT_ADD		add.d
#define INT_ADDI	addi.d
#define INT_SUB		sub.d
#define INT_L		ld.d
#define INT_S		st.d
#define INT_SLL		slli.d
#define INT_SLLV	sll.d
#define INT_SRL		srli.d
#define INT_SRLV	srl.d
#define INT_SRA		sra.w
#define INT_SRAV	sra.d
#endif

/*
 * How to add/sub/load/store/shift C long variables.
 */
#if (_LOONGARCH_SZLONG == 32)
#define LONG_ADD	add.w
#define LONG_ADDI	addi.w
#define LONG_SUB	sub.w
#define LONG_L		ld.w
#define LONG_S		st.w
#define LONG_SLL	slli.w
#define LONG_SLLV	sll.w
#define LONG_SRL	srli.w
#define LONG_SRLV	srl.w
#define LONG_SRA	srai.w
#define LONG_SRAV	sra.w

#ifdef __ASSEMBLY__
#define LONG		.word
#endif
#define LONGSIZE	4
#define LONGMASK	3
#define LONGLOG		2
#endif

#if (_LOONGARCH_SZLONG == 64)
#define LONG_ADD	add.d
#define LONG_ADDI	addi.d
#define LONG_SUB	sub.d
#define LONG_L		ld.d
#define LONG_S		st.d
#define LONG_SLL	slli.d
#define LONG_SLLV	sll.d
#define LONG_SRL	srli.d
#define LONG_SRLV	srl.d
#define LONG_SRA	sra.w
#define LONG_SRAV	sra.d

#ifdef __ASSEMBLY__
#define LONG		.dword
#endif
#define LONGSIZE	8
#define LONGMASK	7
#define LONGLOG		3
#endif

/*
 * How to add/sub/load/store/shift pointers.
 */
#if (_LOONGARCH_SZPTR == 32)
#define PTR_ADD		add.w
#define PTR_ADDI	addi.w
#define PTR_SUB		sub.w
#define PTR_L		ld.w
#define PTR_S		st.w
#define PTR_LI		li.w
#define PTR_SLL		slli.w
#define PTR_SLLV	sll.w
#define PTR_SRL		srli.w
#define PTR_SRLV	srl.w
#define PTR_SRA		srai.w
#define PTR_SRAV	sra.w

#define PTR_SCALESHIFT	2

#define PTR		.word
#define PTRSIZE		4
#define PTRLOG		2
#endif

#if (_LOONGARCH_SZPTR == 64)
#define PTR_ADD		add.d
#define PTR_ADDI	addi.d
#define PTR_SUB		sub.d
#define PTR_L		ld.d
#define PTR_S		st.d
#define PTR_LI		li.d
#define PTR_SLL		slli.d
#define PTR_SLLV	sll.d
#define PTR_SRL		srli.d
#define PTR_SRLV	srl.d
#define PTR_SRA		srai.d
#define PTR_SRAV	sra.d

#define PTR_SCALESHIFT	3

#define PTR		.dword
#define PTRSIZE		8
#define PTRLOG		3
#endif

#define SSNOP		sll.w zero, zero, 1

#ifdef CONFIG_SGI_IP28
/* Inhibit speculative stores to volatile (e.g.DMA) or invalid addresses. */
#include <asm/cacheops.h>
#define R10KCBARRIER(addr)  cacop   Cache_Barrier, addr;
#else
#define R10KCBARRIER(addr)
#endif

#endif /* __ASM_ASM_H */
