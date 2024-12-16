/*
 *  Kernel Probes (KProbes)
 *  arch/loongarch/kernel/kprobes.c
 *
 *  Copyright 2006 Sony Corp.
 *  Copyright 2010 Cavium Networks
 *
 *  Some portions copied from the powerpc version.
 *
 *   Copyright (C) IBM Corporation, 2002, 2004
 *
 *  Copyright (C) 2020 Loongson Technology Corporation Limited
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kprobes.h>
#include <linux/preempt.h>
#include <linux/uaccess.h>
#include <linux/kdebug.h>
#include <linux/slab.h>

#include <asm/ptrace.h>
#include <asm/branch.h>
#include <asm/break.h>

static const union loongarch_instruction breakpoint_insn = {
	.reg0i15_format = {
		.opcode = break_op,
		.simmediate = BRK_KPROBE_BP,
		}
};

static const union loongarch_instruction breakpoint2_insn = {
	.reg0i15_format = {
		.opcode = break_op,
		.simmediate = BRK_KPROBE_SSTEPBP,
		}
};

DEFINE_PER_CPU(struct kprobe *, current_kprobe);
DEFINE_PER_CPU(struct kprobe_ctlblk, kprobe_ctlblk);

#define SS_FAIL		-1
#define SS_NONE		0
#define SS_BRANCH	1
#define SS_PC		2

/*
 * insn_has_ll_or_sc function checks whether instruction is ll or sc
 * one; putting breakpoint on top of atomic ll/sc pair is bad idea;
 * so we need to prevent it and refuse kprobes insertion for such
 * instructions; cannot do much about breakpoint in the middle of
 * ll/sc pair; it is upto user to avoid those places
 */
static int __kprobes insn_has_ll_or_sc(union loongarch_instruction insn)
{
	int ret = 0;

	switch (insn.reg2i14_format.opcode) {
	case llw_op:
	case lld_op:
	case scw_op:
	case scd_op:
		ret = 1;
		break;
	}
	return ret;
}

int __kprobes arch_prepare_kprobe(struct kprobe *p)
{
	union loongarch_instruction insn;
	int ret = 0;

	insn = p->addr[0];

	if (insn_has_ll_or_sc(insn)) {
		pr_notice("Kprobes for ll and sc instructions are not"
			  "supported\n");
		ret = -EINVAL;
		goto out;
	}

	if (insn.reg1i21_format.opcode == bceqz_op) {
		pr_notice("Kprobes for bceqz and bcnez instructions are not"
			  "supported\n");
		ret = -EINVAL;
		goto out;
	}

	/* insn: must be on special executable page on loongarch. */
	p->ainsn.insn = get_insn_slot();
	if (!p->ainsn.insn) {
		ret = -ENOMEM;
		goto out;
	}

	/*
	 * In the kprobe->ainsn.insn[] array we store the original
	 * instruction at index zero and a break trap instruction at
	 * index one.
	 *
	 */
	memcpy(&p->ainsn.insn[0], p->addr, sizeof(kprobe_opcode_t));

	p->ainsn.insn[1] = breakpoint2_insn;
	p->opcode = *p->addr;

out:
	return ret;
}

void __kprobes arch_arm_kprobe(struct kprobe *p)
{
	*p->addr = breakpoint_insn;
	flush_insn_slot(p);
}

void __kprobes arch_disarm_kprobe(struct kprobe *p)
{
	*p->addr = p->opcode;
	flush_insn_slot(p);
}

void __kprobes arch_remove_kprobe(struct kprobe *p)
{
	if (p->ainsn.insn) {
		free_insn_slot(p->ainsn.insn, 0);
		p->ainsn.insn = NULL;
	}
}

static void save_previous_kprobe(struct kprobe_ctlblk *kcb)
{
	kcb->prev_kprobe.kp = kprobe_running();
	kcb->prev_kprobe.status = kcb->kprobe_status;
	kcb->prev_kprobe.old_SR = kcb->kprobe_old_SR;
	kcb->prev_kprobe.saved_SR = kcb->kprobe_saved_SR;
	kcb->prev_kprobe.saved_era = kcb->kprobe_saved_era;
}

static void restore_previous_kprobe(struct kprobe_ctlblk *kcb)
{
	__this_cpu_write(current_kprobe, kcb->prev_kprobe.kp);
	kcb->kprobe_status = kcb->prev_kprobe.status;
	kcb->kprobe_old_SR = kcb->prev_kprobe.old_SR;
	kcb->kprobe_saved_SR = kcb->prev_kprobe.saved_SR;
	kcb->kprobe_saved_era = kcb->prev_kprobe.saved_era;
}

static void set_current_kprobe(struct kprobe *p, struct pt_regs *regs,
			       struct kprobe_ctlblk *kcb)
{
	__this_cpu_write(current_kprobe, p);
	kcb->kprobe_saved_SR = kcb->kprobe_old_SR = (regs->csr_prmd & CSR_PRMD_PIE);
	kcb->kprobe_saved_era = regs->csr_era;
}

static int prepare_singlestep(struct kprobe *p, struct pt_regs *regs)
{
	if (is_branch_insn(p->opcode)) {
		if (!simu_branch(regs, p->opcode))
			return SS_BRANCH;
	} else if (is_pc_insn(p->opcode)) {
		if (!simu_pc(regs, p->opcode))
			return SS_PC;
	} else {
		regs->csr_era = (unsigned long)&p->ainsn.insn[0];
		return SS_NONE;
	}

	pr_notice("Kprobes: Error in simulate insn\n");
	regs->csr_era = (unsigned long)&p->ainsn.insn[0];
	return SS_FAIL;
}

static void setup_singlestep(struct kprobe *p, struct pt_regs *regs,
			     struct kprobe_ctlblk *kcb, int reenter)
{
	int ss;

	if (reenter) {
		save_previous_kprobe(kcb);
		set_current_kprobe(p, regs, kcb);
		kcb->kprobe_status = KPROBE_REENTER;
	} else {
		kcb->kprobe_status = KPROBE_HIT_SS;
	}
	/* single step inline if the instruction is an break */
	if (p->ainsn.insn->word == breakpoint_insn.word) {
		regs->csr_prmd &= ~CSR_PRMD_PIE;
		regs->csr_prmd |= kcb->kprobe_saved_SR;
		preempt_enable_no_resched();
	} else {
		regs->csr_prmd &= ~CSR_PRMD_PIE;
		ss = prepare_singlestep(p, regs);
		if (ss == SS_NONE) {
			kcb->kprobe_status = KPROBE_HIT_SS;
		} else if (ss == SS_BRANCH || ss == SS_PC) {
			kcb->kprobe_status = KPROBE_HIT_SSDONE;
			if (p->post_handler)
				p->post_handler(p, regs, 0);
			reset_current_kprobe();
			preempt_enable_no_resched();
		} else {
			if (p->fault_handler)
				p->fault_handler(p, regs, 0);
			reset_current_kprobe();
			preempt_enable_no_resched();
		}
	}
}

static int reenter_kprobe(struct kprobe *p, struct pt_regs *regs,
			  struct kprobe_ctlblk *kcb)
{
	switch (kcb->kprobe_status) {
	case KPROBE_HIT_SSDONE:
	case KPROBE_HIT_ACTIVE:
		kprobes_inc_nmissed_count(p);
		setup_singlestep(p, regs, kcb, 1);
		break;
	case KPROBE_HIT_SS:
	case KPROBE_REENTER:
		pr_err("Unrecoverable kprobe detected. \n");
		BUG();
		break;
	default:
		/* impossible cases */
		WARN_ON(1);
		return 0;
	}

	return 1;
}

static int __kprobes kprobe_handler(struct pt_regs *regs)
{
	struct kprobe *p;
	kprobe_opcode_t *addr;
	struct kprobe_ctlblk *kcb;

	addr = (kprobe_opcode_t *) regs->csr_era;

	/*
	 * We don't want to be preempted for the entire
	 * duration of kprobe processing
	 */
	preempt_disable();
	kcb = get_kprobe_ctlblk();

	p = get_kprobe(addr);
	if (p) {
		if (kprobe_running()) {
			if (reenter_kprobe(p, regs, kcb))
				return 1;
		} else {
			set_current_kprobe(p, regs, kcb);
			kcb->kprobe_status = KPROBE_HIT_ACTIVE;
			if (p->pre_handler && p->pre_handler(p, regs)) {
			/* handler has already set things up, so skip ss setup */
				reset_current_kprobe();
				preempt_enable_no_resched();
				return 1;
			} else {
				setup_singlestep(p, regs, kcb, 0);
				return 1;
			}
		}
	} else {
		if (addr->word != breakpoint_insn.word) {
			/*
			 * The breakpoint instruction was removed right
			 * after we hit it.  Another cpu has removed
			 * either a probepoint or a debugger breakpoint
			 * at this address.  In either case, no further
			 * handling of this interrupt is appropriate.
			 */
		 preempt_enable_no_resched();
		 return 1;
		}
		/* Not one of ours: let kernel handle it */
	}
	preempt_enable_no_resched();
	return 0;
}

static inline int post_kprobe_handler(struct pt_regs *regs)
{
	struct kprobe *cur = kprobe_running();
	struct kprobe_ctlblk *kcb = get_kprobe_ctlblk();

	if (!cur)
		return 0;

	if ((kcb->kprobe_status != KPROBE_REENTER) && cur->post_handler) {
		kcb->kprobe_status = KPROBE_HIT_SSDONE;
		cur->post_handler(cur, regs, 0);
	}

	regs->csr_era = kcb->kprobe_saved_era + LOONGARCH_INSN_SIZE;
	regs->csr_prmd |= kcb->kprobe_saved_SR;

	/* Restore back the original saved kprobes variables and continue. */
	if (kcb->kprobe_status == KPROBE_REENTER) {
		restore_previous_kprobe(kcb);
		goto out;
	}
	reset_current_kprobe();
out:
	preempt_enable_no_resched();

	return 1;
}

static inline int kprobe_fault_handler(struct pt_regs *regs, int trapnr)
{
	struct kprobe *cur = kprobe_running();
	struct kprobe_ctlblk *kcb = get_kprobe_ctlblk();

	if (cur->fault_handler && cur->fault_handler(cur, regs, trapnr))
		return 1;

	if (kcb->kprobe_status & KPROBE_HIT_SS) {
		regs->csr_era = kcb->kprobe_saved_era + LOONGARCH_INSN_SIZE;
		regs->csr_prmd |= kcb->kprobe_old_SR;

		reset_current_kprobe();
		preempt_enable_no_resched();
	}
	return 0;
}

/*
 * Wrapper routine for handling exceptions.
 */
int __kprobes kprobe_exceptions_notify(struct notifier_block *self,
				       unsigned long val, void *data)
{

	struct die_args *args = (struct die_args *)data;
	int ret = NOTIFY_DONE;

	switch (val) {
	case DIE_BREAK:
		if (kprobe_handler(args->regs))
			ret = NOTIFY_STOP;
		break;
	case DIE_SSTEPBP:
		if (post_kprobe_handler(args->regs))
			ret = NOTIFY_STOP;
		break;

	case DIE_PAGE_FAULT:
		/* kprobe_running() needs smp_processor_id() */
		preempt_disable();

		if (kprobe_running()
		    && kprobe_fault_handler(args->regs, args->trapnr))
			ret = NOTIFY_STOP;
		preempt_enable();
		break;
	default:
		break;
	}
	return ret;
}

/*
 * Function return probe trampoline:
 *	- init_kprobes() establishes a probepoint here
 *	- When the probed function returns, this probe causes the
 *	  handlers to fire
 */
static void __used kretprobe_trampoline_holder(void)
{
	asm volatile(
		/* Keep the assembler from reordering and placing JR here. */
		"nop\n\t"
		".global kretprobe_trampoline\n"
		"kretprobe_trampoline:\n\t"
		"nop\n\t"
		: : : "memory");
}

void kretprobe_trampoline(void);

void __kprobes arch_prepare_kretprobe(struct kretprobe_instance *ri,
				      struct pt_regs *regs)
{
	ri->ret_addr = (kprobe_opcode_t *) regs->regs[1];

	/* Replace the return addr with trampoline addr */
	regs->regs[1] = (unsigned long)kretprobe_trampoline;
}

/*
 * Called when the probe at kretprobe trampoline is hit
 */
static int __kprobes trampoline_probe_handler(struct kprobe *p,
						struct pt_regs *regs)
{
	struct kretprobe_instance *ri = NULL;
	struct hlist_head *head, empty_rp;
	struct hlist_node *tmp;
	unsigned long flags, orig_ret_address = 0;
	unsigned long trampoline_address = (unsigned long)kretprobe_trampoline;

	INIT_HLIST_HEAD(&empty_rp);
	kretprobe_hash_lock(current, &head, &flags);

	/*
	 * It is possible to have multiple instances associated with a given
	 * task either because an multiple functions in the call path
	 * have a return probe installed on them, and/or more than one return
	 * return probe was registered for a target function.
	 *
	 * We can handle this because:
	 *     - instances are always inserted at the head of the list
	 *     - when multiple return probes are registered for the same
	 *	 function, the first instance's ret_addr will point to the
	 *	 real return address, and all the rest will point to
	 *	 kretprobe_trampoline
	 */
	hlist_for_each_entry_safe(ri, tmp, head, hlist) {
		if (ri->task != current)
			/* another task is sharing our hash bucket */
			continue;

		if (ri->rp && ri->rp->handler)
			ri->rp->handler(ri, regs);

		orig_ret_address = (unsigned long)ri->ret_addr;
		recycle_rp_inst(ri, &empty_rp);

		if (orig_ret_address != trampoline_address)
			/*
			 * This is the real return address. Any other
			 * instances associated with this task are for
			 * other calls deeper on the call stack
			 */
			break;
	}

	kretprobe_assert(ri, orig_ret_address, trampoline_address);
	instruction_pointer(regs) = orig_ret_address;

	kretprobe_hash_unlock(current, &flags);

	hlist_for_each_entry_safe(ri, tmp, &empty_rp, hlist) {
		hlist_del(&ri->hlist);
		kfree(ri);
	}
	/*
	 * By returning a non-zero value, we are telling
	 * kprobe_handler() that we don't want the post_handler
	 * to run (and have re-enabled preemption)
	 */
	return 1;
}

int __kprobes arch_trampoline_kprobe(struct kprobe *p)
{
	if (p->addr == (kprobe_opcode_t *)kretprobe_trampoline)
		return 1;

	return 0;
}

static struct kprobe trampoline_p = {
	.addr = (kprobe_opcode_t *)kretprobe_trampoline,
	.pre_handler = trampoline_probe_handler
};

int __init arch_init_kprobes(void)
{
	return register_kprobe(&trampoline_p);
}
