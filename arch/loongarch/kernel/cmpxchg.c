// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017 Imagination Technologies
 * Author: Paul Burton <paul.burton@mips.com>
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/bug.h>
#include <asm/cmpxchg.h>

unsigned long __xchg_small(volatile void *ptr, unsigned long val, unsigned int size)
{
	u32 old32, mask, temp;
	volatile u32 *ptr32;
	unsigned int shift;

	/* Check that ptr is naturally aligned */
	WARN_ON((unsigned long)ptr & (size - 1));

	/* Mask value to the correct size. */
	mask = GENMASK((size * BITS_PER_BYTE) - 1, 0);
	val &= mask;

	/*
	 * Calculate a shift & mask that correspond to the value we wish to
	 * exchange within the naturally aligned 4 byte integerthat includes
	 * it.
	 */
	shift = (unsigned long)ptr & 0x3;
	shift *= BITS_PER_BYTE;
	mask <<= shift;

	/*
	 * Calculate a pointer to the naturally aligned 4 byte integer that
	 * includes our byte of interest, and load its value.
	 */
	ptr32 = (volatile u32 *)((unsigned long)ptr & ~0x3);

	asm volatile (
	"1:		ll.w	%0, %3		\n"
	"		andn	%1, %0, %4	\n"
	"		or	%1, %1, %5	\n"
	"		sc.w	%1, %2		\n"
	"		beqz	%1, 1b		\n"
	: "=&r" (old32), "=&r" (temp), "=ZC" (*ptr32)
	: "ZC" (*ptr32), "Jr" (mask), "Jr" (val << shift)
	: "memory");

	return (old32 & mask) >> shift;
}

unsigned long __cmpxchg_small(volatile void *ptr, unsigned long old,
			      unsigned long new, unsigned int size)
{
	u32 old32, mask, temp;
	volatile u32 *ptr32;
	unsigned int shift;

	/* Check that ptr is naturally aligned */
	WARN_ON((unsigned long)ptr & (size - 1));

	/* Mask inputs to the correct size. */
	mask = GENMASK((size * BITS_PER_BYTE) - 1, 0);
	old &= mask;
	new &= mask;

	/*
	 * Calculate a shift & mask that correspond to the value we wish to
	 * compare & exchange within the naturally aligned 4 byte integer
	 * that includes it.
	 */
	shift = (unsigned long)ptr & 0x3;
	shift *= BITS_PER_BYTE;
	old <<= shift;
	new <<= shift;
	mask <<= shift;

	/*
	 * Calculate a pointer to the naturally aligned 4 byte integer that
	 * includes our byte of interest, and load its value.
	 */
	ptr32 = (volatile u32 *)((unsigned long)ptr & ~0x3);

	asm volatile (
	"1:	ll.w		%0, %3		\n"
	"	and		%1, %0, %4	\n"
	"	bne		%1, %5, 2f	\n"
	"	andn		%1, %0, %4	\n"
	"	or		%1, %1, %6	\n"
	"	sc.w		%1, %2		\n"
	"	beqz		%1, 1b		\n"
	"	b		3f		\n"
	"2:					\n"
	__WEAK_LLSC_MB
	"3:					\n"
	: "=&r" (old32), "=&r" (temp), "=ZC" (*ptr32)
	: "ZC" (*ptr32), "Jr" (mask), "Jr" (old), "Jr" (new)
	: "memory");

	return (old32 & mask) >> shift;
}
