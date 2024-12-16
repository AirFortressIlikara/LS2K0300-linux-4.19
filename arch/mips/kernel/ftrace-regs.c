// SPDX-License-Identifier: GPL-2.0-only
/*
 * arch/mips/kernel/ftrace.c
 *
 * Copyright (C) 2021 Loongson Limited Corp.
 * Author: Huang Pei <huangpei@loongson.cn>
 */

#include <linux/ftrace.h>
#include <linux/kprobes.h>
#include <linux/module.h>
#include <linux/swab.h>
#include <linux/uaccess.h>

#include <asm/ftrace.h>
#include <asm/asm.h>
#include <asm/asm-offsets.h>
#include <asm/cacheflush.h>
#include <asm/uasm.h>


#define INSN_NOP	0x00000000	/* nop */
#define INSN_JALR_AT2	0x00200809	/* jalr at, at */
#define INSN_LI_0	0x240c0000	/* li t0, 0 */
#define INSN_LI_1	0x240c0001	/* li t0, 1 */
#define FTRACE_CALL_IP	((unsigned long)(&ftrace_call))
#define JAL		0x0c000000	/* jump & link: ip --> ra, jump to target */
#define ADDR_MASK	0x03ffffff	/*  op_code|addr : 31...26|25 ....0 */
#define INSN_JAL(addr)	\
	((unsigned int)(JAL | (((addr) >> 2) & ADDR_MASK)))

extern void ftrace_graph_call(void);

static unsigned int insn_lui __read_mostly;

/* Arch override because MIPS doesn't need to run this from stop_machine() */
void arch_ftrace_update_code(int command)
{
	ftrace_modify_all_code(command);
}

static int ftrace_modify_code(unsigned long ip, unsigned int new_code)
{
	int faulted;
	mm_segment_t old_fs;

	/* *(unsigned int *)ip = new_code; */
	safe_store_code(new_code, ip, faulted);

	if (unlikely(faulted))
		return -EFAULT;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	flush_icache_range(ip, ip + 8);
	set_fs(old_fs);

	return 0;
}

int ftrace_update_ftrace_func(ftrace_func_t func)
{
	unsigned int new;

	new = INSN_JAL((unsigned long)func);

	return ftrace_modify_code(FTRACE_CALL_IP, new);
}

/*
 * enable tracing by replacing the middle nop with jalr, like
 *
 * lui	at, %hi(ftrace_regs_all)
 * jalr	at, at
 * li	t0, 0
 */
int ftrace_make_call(struct dyn_ftrace *rec, unsigned long addr)
{
	unsigned long ip = rec->ip;

	ftrace_modify_code(ip + 4, INSN_JALR_AT2);
	return 0;
}

/*
 * disable recording regs by replacing
 *
 *  li	t0, 0
 *
 * with
 *
 *  li, t0, 1
 *
 * or vesa
 */
int ftrace_modify_call(struct dyn_ftrace *rec, unsigned long old_addr,
			unsigned long addr)
{
	unsigned long ip = rec->ip;

	if (abs(old_addr - addr) == 4) {
		if (addr == (unsigned long)ftrace_regs_caller)
			return ftrace_modify_code(ip + 4, INSN_LI_0);

		if (addr == (unsigned long)ftrace_caller)
			return ftrace_modify_code(ip + 4, INSN_LI_1);

	}

	/* we do not support direct call or trampoline now */

	return -1;

}

/*
 * replace all three nop at the entry with
 *
 * lui	at, %hi(ftrace_regs_all)
 * nop
 * li	t0, 1
 */

int ftrace_init_nop(struct module *mod, struct dyn_ftrace *rec)
{
	unsigned long ip = rec->ip;

	ftrace_modify_code(ip, insn_lui);

	ftrace_modify_code(ip + 8, INSN_LI_1);
	return 0;
}



/*
 * disable tracing by replacing
 *
 *  jalr at, at
 *
 * with
 *
 *  nop
 *
 */

int ftrace_make_nop(struct module *mod, struct dyn_ftrace *rec,
			unsigned long addr)

{
	unsigned int new = INSN_NOP;
	unsigned long ip = rec->ip + 4;

	return ftrace_modify_code(ip, new);
}

int __init ftrace_dyn_arch_init(void)
{
	u32 *buf;
	int reg;

	reg = 1;
	/* lui at, %hi(ftrace_regs_all) */
	buf = (u32 *)&insn_lui;
	uasm_i_lui(&buf, reg, uasm_rel_hi((long)ftrace_regs_caller));

	return 0;
}

void kprobe_ftrace_handler(unsigned long ip, unsigned long parent_ip,
			   struct ftrace_ops *ops, struct pt_regs *regs)
{
	struct kprobe *p;
	struct kprobe_ctlblk *kcb;
	unsigned long orig_ip;

	p = get_kprobe((kprobe_opcode_t *)ip);
	if (unlikely(!p) || kprobe_disabled(p))
		return;

	kcb = get_kprobe_ctlblk();
	if (kprobe_running()) {
		kprobes_inc_nmissed_count(p);
	} else {
		/*
		 * pre_handler need epc point to the kprobe
		 *
		 */
		orig_ip = instruction_pointer(regs);
		instruction_pointer_set(regs, ip);
		__this_cpu_write(current_kprobe, p);
		kcb->kprobe_status = KPROBE_HIT_ACTIVE;
		if (!p->pre_handler || !p->pre_handler(p, regs)) {
			/*
			 * Emulate singlestep (and also recover regs->cp0_epc)
			 * as if there is a nop
			 */
			instruction_pointer_set(regs, ip + 4);
			if (unlikely(p->post_handler)) {
				kcb->kprobe_status = KPROBE_HIT_SSDONE;
				p->post_handler(p, regs, 0);
			}
			instruction_pointer_set(regs, orig_ip);
		}
		/*
		 * If pre_handler returns !0, we have to
		 * skip emulating post_handler.
		 */
		__this_cpu_write(current_kprobe, NULL);
	}

}

int arch_prepare_kprobe_ftrace(struct kprobe *p)
{
	p->ainsn.insn = NULL;
	return 0;
}

#ifdef CONFIG_FUNCTION_GRAPH_TRACER
unsigned long prepare_ftrace_return(unsigned long parent, unsigned long self_ra,
			   unsigned long fp)
{
	unsigned long return_hooker = (unsigned long)&return_to_handler;

	if (unlikely(ftrace_graph_is_dead()))
		goto out;

	if (unlikely(atomic_read(&current->tracing_graph_pause)))
		goto out;

	self_ra -=  8;
	if (!function_graph_enter(parent, self_ra, fp, NULL))
		parent = return_hooker;
out:
	return parent;
}

/*
 * Turn on/off the call to ftrace_graph_caller() in ftrace_caller()
 * depending on @enable.
 */
static int ftrace_modify_graph_caller(bool enable)
{
	unsigned long pc = (unsigned long)ftrace_graph_call;
	unsigned int new;

	if (enable)
		new = INSN_JAL((unsigned long)ftrace_graph_caller);
	else
		new = INSN_NOP;

	return ftrace_modify_code(pc, new);
}

int ftrace_enable_ftrace_graph_caller(void)
{
	return ftrace_modify_graph_caller(true);
}

int ftrace_disable_ftrace_graph_caller(void)
{
	return ftrace_modify_graph_caller(false);
}

#endif /* CONFIG_FUNCTION_GRAPH_TRACER */
