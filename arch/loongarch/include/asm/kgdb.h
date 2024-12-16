/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_KGDB_H_
#define __ASM_KGDB_H_

#ifdef __KERNEL__

#include <asm/abidefs.h>

#if (_LOONGARCH_ISA == _LOONGARCH_ISA_LOONGARCH32)

#define KGDB_GDB_REG_SIZE	32
#define GDB_SIZEOF_REG		sizeof(u32)

#elif (_LOONGARCH_ISA == _LOONGARCH_ISA_LOONGARCH64)

#ifdef CONFIG_32BIT
#define KGDB_GDB_REG_SIZE	32
#define GDB_SIZEOF_REG		sizeof(u32)
#else /* CONFIG_CPU_32BIT */
#define KGDB_GDB_REG_SIZE	64
#define GDB_SIZEOF_REG		sizeof(u64)
#endif
#else
#error "Need to set KGDB_GDB_REG_SIZE for LoongArch ISA"
#endif /* _LOONGARCH_ISA */

#define BUFMAX			2048
#define DBG_MAX_REG_NUM		34
#define DBG_FP_FCC0_REGNO	(DBG_MAX_REG_NUM + 32)
#define DBG_FP_FSR_REGNO	(DBG_FP_FCC0_REGNO + 8)
#define DBG_FP_SCR0_REGNO	(DBG_FP_FSR_REGNO + 1)
#define DBG_ALL_REG_NUM		(DBG_FP_SCR0_REGNO + 4)
#define NUMREGBYTES		(DBG_MAX_REG_NUM * sizeof(GDB_SIZEOF_REG))
#define NUMCRITREGBYTES		(12 * sizeof(GDB_SIZEOF_REG))
#define BREAK_INSTR_SIZE	4
#define CACHE_FLUSH_IS_SAFE	0

extern void arch_kgdb_breakpoint(void);
extern void *saved_vectors[32];
extern void handle_exception(struct pt_regs *regs);
extern void breakinst(void);
extern int kgdb_ll_trap(int cmd, const char *str,
			struct pt_regs *regs, long err, int trap, int sig);

#endif				/* __KERNEL__ */

#endif /* __ASM_KGDB_H_ */
