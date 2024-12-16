// SPDX-License-Identifier: GPL-2.0
/*
 * Based on arch/x86/kernel/unwind_orc.c
 *
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */

#include <linux/cpu.h>
#include <linux/kallsyms.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched/task_stack.h>
#include <linux/sort.h>

#include <asm/loongarchregs.h>
#include <asm/irq.h>
#include <asm/orc_lookup.h>
#include <asm/orc_types.h>
#include <asm/sections.h>
#include <asm/stacktrace.h>
#include <asm/tlb.h>
#include <asm/unwind.h>
#include <asm/setup.h>
#include <linux/ftrace.h>

#define orc_warn(fmt, ...) \
	printk_deferred_once(KERN_WARNING "WARNING: " fmt, ##__VA_ARGS__)

#define orc_warn_current(args...)					\
({									\
	if (state->task == current)					\
		orc_warn(args);						\
})

extern int __start_orc_unwind_ip[];
extern int __stop_orc_unwind_ip[];
extern struct orc_entry __start_orc_unwind[];
extern struct orc_entry __stop_orc_unwind[];

static bool orc_init __ro_after_init;
static unsigned int lookup_num_blocks __ro_after_init;

static int *cur_orc_ip_table = __start_orc_unwind_ip;
static struct orc_entry *cur_orc_table = __start_orc_unwind;

static inline unsigned long orc_ip(const int *ip)
{
	return (unsigned long)ip + *ip;
}

static struct orc_entry *__orc_find(int *ip_table, struct orc_entry *u_table,
				    unsigned int num_entries, unsigned long ip)
{
	int *first = ip_table;
	int *last = ip_table + num_entries - 1;
	int *mid = first, *found = first;

	if (!num_entries)
		return NULL;

	/*
	 * Do a binary range search to find the rightmost duplicate of a given
	 * starting address.  Some entries are section terminators which are
	 * "weak" entries for ensuring there are no gaps.  They should be
	 * ignored when they conflict with a real entry.
	 */
	while (first <= last) {
		mid = first + ((last - first) / 2);

		if (orc_ip(mid) <= ip) {
			found = mid;
			first = mid + 1;
		} else
			last = mid - 1;
	}

	return u_table + (found - ip_table);
}

#ifdef CONFIG_MODULES
static struct orc_entry *orc_module_find(unsigned long ip)
{
	struct module *mod;

	mod = __module_address(ip);
	if (!mod || !mod->arch.orc_unwind || !mod->arch.orc_unwind_ip)
		return NULL;
	return __orc_find(mod->arch.orc_unwind_ip, mod->arch.orc_unwind,
			  mod->arch.num_orcs, ip);
}
#else
static struct orc_entry *orc_module_find(unsigned long ip)
{
	return NULL;
}
#endif

/* Fake frame pointer entry -- used as a fallback for generated code */
static struct orc_entry orc_fp_entry = {
	.type		= ORC_TYPE_CALL,
	.sp_reg		= ORC_REG_FP,
	.sp_offset	= 16,
	.fp_reg		= ORC_REG_PREV_SP,
	.fp_offset	= -16,
	.ra_reg		= ORC_REG_PREV_SP,
	.ra_offset	= -8,
	.end		= 0,
};

static struct orc_entry *orc_find(unsigned long ip)
{
	struct orc_entry *orc = NULL;

	/* Although exception occurs, CSR_ERA is not 0. */
	if (ip == 0)
		return NULL;

	/* For non-init vmlinux addresses, use the fast lookup table: */
	if (ip >= LOOKUP_START_IP && ip < LOOKUP_STOP_IP) {
		unsigned int idx, start, stop;

		idx = (ip - LOOKUP_START_IP) / LOOKUP_BLOCK_SIZE;

		if (unlikely((idx >= lookup_num_blocks - 1))) {
			orc_warn("WARNING: bad lookup idx: idx=%u num=%u ip=%pB\n",
				 idx, lookup_num_blocks, (void *)ip);
			return NULL;
		}

		start = orc_lookup[idx];
		stop = orc_lookup[idx + 1] + 1;

		if (unlikely((__start_orc_unwind + start >= __stop_orc_unwind) ||
			     (__start_orc_unwind + stop > __stop_orc_unwind))) {
			orc_warn("WARNING: bad lookup value: idx=%u num=%u start=%u stop=%u ip=%pB\n",
				 idx, lookup_num_blocks, start, stop, (void *)ip);
			return NULL;
		}

		return __orc_find(__start_orc_unwind_ip + start,
				  __start_orc_unwind + start, stop - start, ip);
	}

	/* vmlinux .init slow lookup: */
	if (init_kernel_text(ip))
		return __orc_find(__start_orc_unwind_ip, __start_orc_unwind,
				  __stop_orc_unwind_ip - __start_orc_unwind_ip, ip);

	/* Module lookup: */
	orc = orc_module_find(ip);
	if (orc)
		return orc;

	return NULL;
}

static void orc_sort_swap(void *_a, void *_b, int size)
{
	struct orc_entry *orc_a, *orc_b;
	struct orc_entry orc_tmp;
	int *a = _a, *b = _b, tmp;
	int delta = _b - _a;

	/* Swap the .orc_unwind_ip entries: */
	tmp = *a;
	*a = *b + delta;
	*b = tmp - delta;

	/* Swap the corresponding .orc_unwind entries: */
	orc_a = cur_orc_table + (a - cur_orc_ip_table);
	orc_b = cur_orc_table + (b - cur_orc_ip_table);
	orc_tmp = *orc_a;
	*orc_a = *orc_b;
	*orc_b = orc_tmp;
}

static int orc_sort_cmp(const void *_a, const void *_b)
{
	struct orc_entry *orc_a;
	const int *a = _a, *b = _b;
	unsigned long a_val = orc_ip(a);
	unsigned long b_val = orc_ip(b);

	if (a_val > b_val)
		return 1;
	if (a_val < b_val)
		return -1;

	/*
	 * The "weak" section terminator entries need to always be on the left
	 * to ensure the lookup code skips them in favor of real entries.
	 * These terminator entries exist to handle any gaps created by
	 * whitelisted .o files which didn't get objtool generation.
	 */
	orc_a = cur_orc_table + (a - cur_orc_ip_table);
	return orc_a->sp_reg == ORC_REG_UNDEFINED && !orc_a->end ? -1 : 1;
}

#ifdef CONFIG_MODULES
static DEFINE_MUTEX(sort_mutex);

void unwind_module_init(struct module *mod, void *_orc_ip, size_t orc_ip_size,
			void *_orc, size_t orc_size)
{
	int *orc_ip = _orc_ip;
	struct orc_entry *orc = _orc;
	unsigned int num_entries = orc_ip_size / sizeof(int);

	WARN_ON_ONCE(orc_ip_size % sizeof(int) != 0 ||
		     orc_size % sizeof(*orc) != 0 ||
		     num_entries != orc_size / sizeof(*orc));

	/*
	 * The 'cur_orc_*' globals allow the orc_sort_swap() callback to
	 * associate an .orc_unwind_ip table entry with its corresponding
	 * .orc_unwind entry so they can both be swapped.
	 */
	mutex_lock(&sort_mutex);
	cur_orc_ip_table = orc_ip;
	cur_orc_table = orc;
	sort(orc_ip, num_entries, sizeof(int), orc_sort_cmp, orc_sort_swap);
	mutex_unlock(&sort_mutex);

	mod->arch.orc_unwind_ip = orc_ip;
	mod->arch.orc_unwind = orc;
	mod->arch.num_orcs = num_entries;
}
#endif

void __init unwind_init(void)
{
	size_t orc_ip_size = (void *)__stop_orc_unwind_ip - (void *)__start_orc_unwind_ip;
	size_t orc_size = (void *)__stop_orc_unwind - (void *)__start_orc_unwind;
	size_t num_entries = orc_ip_size / sizeof(int);
	struct orc_entry *orc;
	int i;

	if (!num_entries || orc_ip_size % sizeof(int) != 0 ||
	    orc_size % sizeof(struct orc_entry) != 0 ||
	    num_entries != orc_size / sizeof(struct orc_entry)) {
		orc_warn("WARNING: Bad or missing .orc_unwind table.  Disabling unwinder.\n");
		return;
	}

	/* Sort the .orc_unwind and .orc_unwind_ip tables: */
	sort(__start_orc_unwind_ip, num_entries, sizeof(int), orc_sort_cmp,
	     orc_sort_swap);

	/* Initialize the fast lookup table: */
	lookup_num_blocks = orc_lookup_end - orc_lookup;
	for (i = 0; i < lookup_num_blocks - 1; i++) {
		orc = __orc_find(__start_orc_unwind_ip, __start_orc_unwind,
				 num_entries,
				 LOOKUP_START_IP + (LOOKUP_BLOCK_SIZE * i));
		if (!orc) {
			orc_warn("WARNING: Corrupt .orc_unwind table.  Disabling unwinder.\n");
			return;
		}

		orc_lookup[i] = orc - __start_orc_unwind;
	}

	/* Initialize the ending block: */
	orc = __orc_find(__start_orc_unwind_ip, __start_orc_unwind, num_entries,
			 LOOKUP_STOP_IP);
	if (!orc) {
		orc_warn("WARNING: Corrupt .orc_unwind table.  Disabling unwinder.\n");
		return;
	}
	orc_lookup[lookup_num_blocks-1] = orc - __start_orc_unwind;

	orc_init = true;
}

static inline bool task_on_another_cpu(struct task_struct *task)
{
#ifdef CONFIG_SMP
	return task != current && task->on_cpu;
#else
	return false;
#endif
}

void unwind_start(struct unwind_state *state, struct task_struct *task,
		      struct pt_regs *regs)
{
	memset(state, 0, sizeof(*state));
	state->task = task;

	if (!orc_init)
		goto err;

	if (task_on_another_cpu(task))
		goto err;

	state->pc = regs->csr_era;
	state->ra = regs->regs[1];
	state->sp = regs->regs[3];
	state->fp = regs->regs[22];

	if (get_stack_info(state->sp, state->task, &state->stack_info))
		goto err;

	return;

err:
	state->error = true;
	state->stack_info.type = STACK_TYPE_UNKNOWN;
}

#define INSN_LINK_OFFSET 4

extern void *exception_table[];
extern void *vector_table[];

static inline unsigned long bt_address(unsigned long ra)
{
	extern unsigned long eentry;

	if (__kernel_text_address(ra))
		return ra;

	/* We are in preempt_disable() here */
	if (__module_text_address(ra))
		return ra;

	if (ra >= eentry && ra < eentry +  EXCCODE_INT_END * VECSIZE) {
		unsigned long type = (ra - eentry) / VECSIZE;
		unsigned long offset = (ra - eentry) % VECSIZE;
		unsigned long func;

		if (type >= 0 && type < EXCCODE_INT_START)
			func = (unsigned long)exception_table[type];
		else if (type >= EXCCODE_INT_START && type < EXCCODE_INT_END)
			func = (unsigned long)vector_table[type - EXCCODE_INT_START];
		else
			return 0;

		return func + offset;
	}

	return ra;
}

static inline bool on_stack(struct stack_info *info, unsigned long addr,
			    size_t len)
{
	unsigned long begin = info->begin;
	unsigned long end = info->end;

	return (info->type != STACK_TYPE_UNKNOWN &&
		addr >= begin && addr < end &&
		addr + len > begin && addr + len <= end);
}

static bool stack_access_ok(struct unwind_state *state, unsigned long addr,
			    size_t len)
{
	struct stack_info *info = &state->stack_info;

	if (!on_stack(info, addr, len) &&
	    (get_stack_info(addr, state->task, info)))
		return false;

	return true;
}

static bool is_entry_func(unsigned long addr)
{
	extern u32 kernel_entry;
	extern u32 kernel_entry_end;

	return addr >= (unsigned long)&kernel_entry &&
		addr < (unsigned long)&kernel_entry_end;
}

bool unwind_next_frame(struct unwind_state *state)
{
	struct stack_info *info = &state->stack_info;
	struct orc_entry *orc;
	struct pt_regs *regs;
	unsigned long *p, pc;

	if (unwind_done(state))
		return false;

	/* Don't let modules unload while we're reading their ORC data. */
	preempt_disable();

	if (is_entry_func(state->pc))
		goto end;

	orc = orc_find(state->pc);
	if (!orc) {
		orc = &orc_fp_entry;
		state->error = true;
	}

	switch (orc->sp_reg) {
	case ORC_REG_SP:
		state->sp = state->sp + orc->sp_offset;
		break;
	case ORC_REG_FP:
		state->sp = state->fp;
		break;
	default:
		orc_warn("unknown SP base reg %d at %pB\n",
			orc->sp_reg, (void *)state->pc);
		goto err;
	}

	switch (orc->fp_reg) {
	case ORC_REG_PREV_SP:
		p = (unsigned long *)(state->sp + orc->fp_offset);
		if (!stack_access_ok(state, (unsigned long)p, sizeof(unsigned long)))
			goto err;

		state->fp = *p;
		break;
	case ORC_REG_UNDEFINED:
		/* Nothing. */
		break;
	default:
		orc_warn("unknown FP base reg %d at %pB\n",
			orc->fp_reg, (void *)state->pc);
		goto err;
	}

	switch (orc->type) {
	case ORC_TYPE_CALL:
		if (orc->ra_reg == ORC_REG_PREV_SP) {
			p = (unsigned long *)(state->sp + orc->ra_offset);
			if (!stack_access_ok(state, (unsigned long)p, sizeof(unsigned long)))
				goto err;

			pc = unwind_graph_addr(state, *p, state->sp);
			pc -= INSN_LINK_OFFSET;
		} else if (orc->ra_reg == ORC_REG_UNDEFINED) {
			if (!state->ra || state->ra == state->pc)
				goto err;

			pc = unwind_graph_addr(state, state->ra, state->sp);
			pc -=  INSN_LINK_OFFSET;
			state->ra = 0;
		} else {
			orc_warn("unknown ra base reg %d at %pB\n",
				orc->ra_reg, (void *)state->pc);
			goto err;
		}
		break;
	case ORC_TYPE_REGS:
		if (state->stack_info.type == STACK_TYPE_IRQ && state->sp == info->end)
			regs = (struct pt_regs *)info->next_sp;
		else
			regs = (struct pt_regs *)state->sp;

		if (!stack_access_ok(state, (unsigned long)regs, sizeof(*regs)))
			goto err;

		if ((info->end == (unsigned long)regs + sizeof(*regs)) &&
		    !regs->regs[3] && !regs->regs[1])
			goto end;

		if (user_mode(regs))
			goto end;

		pc = regs->csr_era;
		if (!__kernel_text_address(pc))
			goto err;

		state->sp = regs->regs[3];
		state->ra = regs->regs[1];
		state->fp = regs->regs[22];
		get_stack_info(state->sp, state->task, info);

		break;
	default:
		orc_warn("unknown .orc_unwind entry type %d at %pB\n",
			 orc->type, (void *)state->pc);
		goto err;
	}

	state->pc = bt_address(pc);
	if (!state->pc) {
		pr_err("cannot find unwind pc at %pK\n", (void *)pc);
		goto err;
	}

	preempt_enable();
	return true;

err:
	state->error = true;

end:
	preempt_enable();
	state->stack_info.type = STACK_TYPE_UNKNOWN;
	return false;
}

unsigned long unwind_get_return_address(struct unwind_state *state)
{
	if (unwind_done(state))
		return 0;

	return __kernel_text_address(state->pc) ? state->pc : 0;
}
