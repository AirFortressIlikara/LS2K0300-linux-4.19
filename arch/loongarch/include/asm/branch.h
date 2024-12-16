/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1996, 1997, 1998, 2001 by Ralf Baechle
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#ifndef _ASM_BRANCH_H
#define _ASM_BRANCH_H

#include <asm/cpu-features.h>
#include <asm/loongarchregs.h>
#include <asm/ptrace.h>
#include <asm/inst.h>

static inline unsigned long exception_era(struct pt_regs *regs)
{
	return regs->csr_era;
}

static inline int compute_return_era(struct pt_regs *regs)
{
	regs->csr_era += 4;
	return 0;
}

#endif /* _ASM_BRANCH_H */
