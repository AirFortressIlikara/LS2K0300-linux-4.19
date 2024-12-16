/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive for
 * more details.
 *
 * Copyright (C) 2009 DSLab, Lanzhou University, China
 * Author: Wu Zhangjin <wuzhangjin@gmail.com>
 */

#ifndef _ASM_MIPS_FTRACE_H
#define _ASM_MIPS_FTRACE_H

#ifdef CONFIG_FUNCTION_TRACER

#define MCOUNT_ADDR ((unsigned long)(_mcount))
#define MCOUNT_INSN_SIZE 4		/* sizeof mcount call */

#ifndef __ASSEMBLY__
extern void _mcount(void);
#define mcount _mcount

#define safe_load(load, src, dst, error)		\
do {							\
	asm volatile (					\
		"1: " load " %[tmp_dst], 0(%[tmp_src])\n"	\
		"   li %[tmp_err], 0\n"			\
		"2: .insn\n"				\
							\
		".section .fixup, \"ax\"\n"		\
		"3: li %[tmp_err], 1\n"			\
		"   j 2b\n"				\
		".previous\n"				\
							\
		".section\t__ex_table,\"a\"\n\t"	\
		STR(PTR) "\t1b, 3b\n\t"			\
		".previous\n"				\
							\
		: [tmp_dst] "=&r" (dst), [tmp_err] "=r" (error)\
		: [tmp_src] "r" (src)			\
		: "memory"				\
	);						\
} while (0)

#define safe_store(store, src, dst, error)	\
do {						\
	asm volatile (				\
		"1: " store " %[tmp_src], 0(%[tmp_dst])\n"\
		"   li %[tmp_err], 0\n"		\
		"2: .insn\n"			\
						\
		".section .fixup, \"ax\"\n"	\
		"3: li %[tmp_err], 1\n"		\
		"   j 2b\n"			\
		".previous\n"			\
						\
		".section\t__ex_table,\"a\"\n\t"\
		STR(PTR) "\t1b, 3b\n\t"		\
		".previous\n"			\
						\
		: [tmp_err] "=r" (error)	\
		: [tmp_dst] "r" (dst), [tmp_src] "r" (src)\
		: "memory"			\
	);					\
} while (0)

#define safe_load_code(dst, src, error) \
	safe_load(STR(lw), src, dst, error)
#define safe_store_code(src, dst, error) \
	safe_store(STR(sw), src, dst, error)

#define safe_load_stack(dst, src, error) \
	safe_load(STR(PTR_L), src, dst, error)

#define safe_store_stack(src, dst, error) \
	safe_store(STR(PTR_S), src, dst, error)


#ifdef CONFIG_DYNAMIC_FTRACE
static inline unsigned long ftrace_call_adjust(unsigned long addr)
{
	return addr;
}

struct dyn_arch_ftrace {
};

#endif /*  CONFIG_DYNAMIC_FTRACE */
#endif /* __ASSEMBLY__ */
#endif /* CONFIG_FUNCTION_TRACER */

#ifdef CONFIG_DYNAMIC_FTRACE_WITH_REGS
#define ARCH_SUPPORTS_FTRACE_OPS	1
struct dyn_ftrace;
int ftrace_init_nop(struct module *mod, struct dyn_ftrace *rec);
#define ftrace_init_nop ftrace_init_nop
#endif

#endif /* _ASM_MIPS_FTRACE_H */
