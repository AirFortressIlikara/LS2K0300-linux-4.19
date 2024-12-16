// SPDX-License-Identifier: GPL-2.0
/*
 * Code for replacing ftrace calls with jumps.
 *
 * Copyright (C) 2007-2008 Steven Rostedt <srostedt@redhat.com>
 * Copyright (C) 2009, 2010 DSLab, Lanzhou University, China
 * Author: Wu Zhangjin <wuzhangjin@gmail.com>
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 *
 * Thanks goes to Steven Rostedt for writing the original x86 version.
 */

#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/ftrace.h>
#include <linux/syscalls.h>

#include <asm/asm.h>
#include <asm/asm-offsets.h>
#include <asm/cacheflush.h>
#include <asm/syscall.h>
#include <asm/unistd.h>
#include <asm/inst.h>
#include <asm/loongarchregs.h>

#include <asm-generic/sections.h>

#ifdef CONFIG_FUNCTION_GRAPH_TRACER

/*
 * As `call _mcount` follows LoongArch psABI, ra-saved operation and
 * stack operation can be found before this insn.
 */
static inline bool is_ra_save_ins(union loongarch_instruction *ip)
{
	/* st.d $ra, $sp, offset */
	return ip->reg2i12_format.opcode == std_op &&
		ip->reg2i12_format.rj == 3 &&
		ip->reg2i12_format.rd == 1;
}

static inline bool is_stack_open_ins(union loongarch_instruction *ip)
{
	/* addi.d $sp, $sp, -imm */
	return ip->reg2i12_format.opcode == addid_op &&
		ip->reg2i12_format.rj == 3 &&
		ip->reg2i12_format.rd == 3 &&
		ip->reg2i12_format.simmediate < 0;
}

static int ftrace_get_parent_ra_addr(unsigned long insn_addr, int *ra_off)
{
	union loongarch_instruction *insn;
	int limit = 32;

	insn = (union loongarch_instruction *)insn_addr;

	do {
		insn--;
		limit--;

		if (is_ra_save_ins(insn))
			*ra_off = insn->reg2i12_format.simmediate;

	} while (!is_stack_open_ins(insn) && limit);

	if (!limit)
		return -EINVAL;

	return 0;
}

void prepare_ftrace_return(unsigned long self_addr,
		unsigned long callsite_sp, unsigned long old)
{
	int ra_off;
	unsigned long return_hooker = (unsigned long)&return_to_handler;

	if (unlikely(ftrace_graph_is_dead()))
		return;

	if (unlikely(atomic_read(&current->tracing_graph_pause)))
		return;

	if (ftrace_get_parent_ra_addr(self_addr, &ra_off))
		goto out;

	if (!function_graph_enter(old, self_addr, 0, NULL)) {
		*(unsigned long *)(callsite_sp + ra_off) = return_hooker;
	}

	return;

out:
	ftrace_graph_stop();
	WARN_ON(1);
	return;
}
#endif	/* CONFIG_FUNCTION_GRAPH_TRACER */
