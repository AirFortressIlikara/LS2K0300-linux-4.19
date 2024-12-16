/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Most of this ideas comes from x86.
 *
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */
#ifndef _ASM_UNWIND_H
#define _ASM_UNWIND_H

#include <linux/module.h>
#include <linux/ftrace.h>
#include <linux/sched.h>

#include <asm/ptrace.h>
#include <asm/stacktrace.h>

struct unwind_state {
	struct stack_info stack_info;
	struct task_struct *task;
	int graph_idx;
#if defined(CONFIG_UNWINDER_PROLOGUE)
	unsigned long sp, pc, ra;
	bool enable;
	bool first;
#elif defined(CONFIG_UNWINDER_ORC)
	unsigned long sp, pc, fp, ra;
#else /* CONFIG_UNWINDER_GUESS */
	unsigned long sp, pc;
	bool first;
#endif
	bool error, is_ftrace;
};

void unwind_start(struct unwind_state *state, struct task_struct *task,
		      struct pt_regs *regs);
bool unwind_next_frame(struct unwind_state *state);
unsigned long unwind_get_return_address(struct unwind_state *state);

static inline bool unwind_done(struct unwind_state *state)
{
	return state->stack_info.type == STACK_TYPE_UNKNOWN;
}

static inline bool unwind_error(struct unwind_state *state)
{
	return state->error;
}

#ifdef CONFIG_UNWINDER_ORC
void unwind_init(void);
void unwind_module_init(struct module *mod, void *orc_ip, size_t orc_ip_size,
			void *orc, size_t orc_size);
#else
static inline void unwind_init(void) {}
static inline void unwind_module_init(struct module *mod, void *orc_ip,
			size_t orc_ip_size, void *orc, size_t orc_size) {}
#endif /* CONFIG_UNWINDER_ORC */

#define GRAPH_FAKE_OFFSET (sizeof(struct pt_regs) - offsetof(struct pt_regs, regs[1]))
static inline unsigned long unwind_graph_addr(struct unwind_state *state,
					unsigned long pc, unsigned long cfa)
{
	return ftrace_graph_ret_addr(state->task, &state->graph_idx,
				     pc, (unsigned long *)(cfa - GRAPH_FAKE_OFFSET));
}
#endif /* _ASM_UNWIND_H */
