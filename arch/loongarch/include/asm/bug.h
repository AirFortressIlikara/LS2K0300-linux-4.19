/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _ARCH_LOONGARCH_ASM_BUG_H
#define _ARCH_LOONGARCH_ASM_BUG_H

#include <linux/stringify.h>

#include <asm/asm-bug.h>

#define __BUG_FLAGS(flags)				\
	asm volatile (__stringify(ASM_BUG_FLAGS(flags)));

#define BUG() do {					\
	__BUG_FLAGS(0);					\
	unreachable();					\
} while (0)

#define __WARN_FLAGS(flags)					\
do {								\
	__BUG_FLAGS(BUGFLAG_WARNING|(flags));			\
	annotate_reachable();					\
} while (0)

#define HAVE_ARCH_BUG

#include <asm-generic/bug.h>

#endif /* ! _ARCH_LOONGARCH_ASM_BUG_H */
