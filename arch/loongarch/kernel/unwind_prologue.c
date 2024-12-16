// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#include <linux/ftrace.h>
#include <linux/kallsyms.h>

#include <asm/inst.h>
#include <asm/ptrace.h>
#include <asm/unwind.h>

unsigned long unwind_get_return_address(struct unwind_state *state)
{
	if (unwind_done(state))
		return 0;

	return state->pc;
}
EXPORT_SYMBOL_GPL(unwind_get_return_address);

static inline bool is_stack_open_ins(union loongarch_instruction *ip)
{
	/* addi.d $sp, $sp, -imm */
	return ip->reg2i12_format.opcode == addid_op &&
		ip->reg2i12_format.rj == 3 &&
		ip->reg2i12_format.rd == 3 &&
		ip->reg2i12_format.simmediate < 0;
}

static inline bool is_ra_save_ins(union loongarch_instruction *ip)
{
	/* st.d $ra, $sp, offset */
	return ip->reg2i12_format.opcode == std_op &&
		  ip->reg2i12_format.rj == 3 &&
		  ip->reg2i12_format.rd == 1;
}

static inline bool is_branch_ins(union loongarch_instruction *ip)
{
	return is_branch_insn(*ip);
}

static inline void unwind_state_fixup(struct unwind_state *state)
{
#ifdef CONFIG_DYNAMIC_FTRACE
	static unsigned long ftrace_case = (unsigned long)ftrace_call + 4;

	if (state->pc == ftrace_case)
		state->is_ftrace = true;
#endif
}

static bool unwind_by_prologue(struct unwind_state *state)
{
	struct stack_info *info = &state->stack_info;
	union loongarch_instruction *ip, *ip_end;
	unsigned long frame_size = 0, frame_ra = -1;
	unsigned long size, offset, pc = state->pc;
	struct pt_regs *regs;

	if (state->sp >= info->end || state->sp < info->begin)
		return false;

	if (state->is_ftrace) {
		/*
		 * As we meet ftrace_regs_entry, reset first flag like first doing
		 * tracing, Prologue analysis will stop soon because PC is at entry.
		 */
		regs = (struct pt_regs *)state->sp;
		state->pc = regs->csr_era;
		state->ra = regs->regs[1];
		state->sp = regs->regs[3];
		state->first = true;
		state->is_ftrace = false;
		return true;
	}

	if (!kallsyms_lookup_size_offset(pc, &size, &offset))
		return false;

	ip = (union loongarch_instruction *)(pc - offset);
	ip_end = (union loongarch_instruction *)pc;

	while (ip < ip_end) {
		if (is_stack_open_ins(ip)) {
			frame_size = -ip->reg2i12_format.simmediate;
			ip++;
			break;
		}
		ip++;
	}

	if (!frame_size) {
		if (state->first)
			goto first;

		return false;
	}

	while (ip < ip_end) {
		if (is_ra_save_ins(ip)) {
			frame_ra = ip->reg2i12_format.simmediate;
			break;
		}
		if (is_branch_ins(ip))
			break;
		ip++;
	}

	if (frame_ra < 0) {
		if (state->first) {
			state->sp = state->sp + frame_size;
			goto first;
		}
		return false;
	}

	if (state->first)
		state->first = false;

	state->pc = *(unsigned long *)(state->sp + frame_ra);
	state->sp = state->sp + frame_size;
	goto out;

first:
	state->first = false;
	if (state->pc == state->ra)
		return false;

	state->pc = state->ra;

out:
	unwind_state_fixup(state);
	return !!__kernel_text_address(state->pc);
}

static bool unwind_by_guess(struct unwind_state *state)
{
	struct stack_info *info = &state->stack_info;
	unsigned long addr;

	for (state->sp += sizeof(unsigned long);
	     state->sp < info->end;
	     state->sp += sizeof(unsigned long)) {
		addr = *(unsigned long *)(state->sp);
		state->pc = unwind_graph_addr(state, addr, state->sp + 8);
		if (__kernel_text_address(state->pc))
			return true;
	}

	return false;
}

bool unwind_next_frame(struct unwind_state *state)
{
	struct stack_info *info = &state->stack_info;
	struct pt_regs *regs;
	unsigned long pc;

	if (unwind_done(state))
		return false;

	do {
		if (state->enable) {
			if (unwind_by_prologue(state)) {
				state->pc = unwind_graph_addr(state, state->pc, state->sp);
				return true;
			}
			if (info->type == STACK_TYPE_IRQ &&
				info->end == state->sp) {
				regs = (struct pt_regs *)info->next_sp;
				pc = regs->csr_era;
				if (user_mode(regs) || !__kernel_text_address(pc))
					return false;

				state->pc = pc;
				state->sp = regs->regs[3];
				state->ra = regs->regs[1];
				state->first = true;
				get_stack_info(state->sp, state->task, info);

				return true;
			}
		} else {
			if (state->first)
				state->first = false;

			if (unwind_by_guess(state))
				return true;
		}

		state->sp = info->next_sp;

	} while (!get_stack_info(state->sp, state->task, info));

	return false;
}
EXPORT_SYMBOL_GPL(unwind_next_frame);

void unwind_start(struct unwind_state *state, struct task_struct *task,
		    struct pt_regs *regs)
{
	memset(state, 0, sizeof(*state));

	if (__kernel_text_address(regs->csr_era)) {
		state->enable = true;
	}

	state->task = task;
	state->pc = regs->csr_era;
	state->sp = regs->regs[3];
	state->ra = regs->regs[1];
	state->first = true;
	state->pc = unwind_graph_addr(state, state->pc, state->sp);
	get_stack_info(state->sp, state->task, &state->stack_info);

	if (!unwind_done(state) && !__kernel_text_address(state->pc))
		unwind_next_frame(state);
}
EXPORT_SYMBOL_GPL(unwind_start);
