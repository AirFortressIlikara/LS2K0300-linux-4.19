/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1999, 2000, 06 Ralf Baechle (ralf@linux-mips.org)
 * Copyright (C) 1999, 2000 Silicon Graphics, Inc.
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */
#ifndef _ASM_SPINLOCK_H
#define _ASM_SPINLOCK_H

/* How long a lock should spin before we consider blocking */
#define SPIN_THRESHOLD  (1 << 15)

#include <asm/processor.h>
#include <asm/qrwlock.h>
#include <asm/qspinlock.h>

#endif /* _ASM_SPINLOCK_H */
