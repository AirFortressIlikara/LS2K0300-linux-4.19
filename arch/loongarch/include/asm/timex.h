/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1998, 1999, 2003 by Ralf Baechle
 * Copyright (C) 2014 by Maciej W. Rozycki
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */
#ifndef _ASM_TIMEX_H
#define _ASM_TIMEX_H

#ifdef __KERNEL__

#include <linux/compiler.h>

#include <asm/cpu.h>
#include <asm/cpu-features.h>
#include <asm/loongarchregs.h>

/*
 * Standard way to access the cycle counter.
 * Currently only used on SMP for scheduling.
 *
 * We know that all SMP capable CPUs have cycle counters.
 */

typedef unsigned long cycles_t;

static inline cycles_t get_cycles(void)
{
	return drdtime();
}

#endif /* __KERNEL__ */

#endif /*  _ASM_TIMEX_H */
