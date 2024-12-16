// SPDX-License-Identifier: GPL-2.0+
/*
* Copyright (C) 2020 Loongson Technology Corporation Limited
*
* Author: Hanlu Li <lihanlu@loongson.cn>
*/
#include <linux/audit.h>
#include <linux/cache.h>
#include <linux/context_tracking.h>
#include <linux/irqflags.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/personality.h>
#include <linux/smp.h>
#include <linux/kernel.h>
#include <linux/signal.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/ptrace.h>
#include <linux/unistd.h>
#include <linux/compiler.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/tracehook.h>
#include <linux/livepatch.h>

#include <asm/asm.h>
#include <asm/cacheflush.h>
#include <asm/fpu.h>
#include <asm/ucontext.h>
#include <asm/cpu-features.h>
#include <asm/inst.h>
#include <asm/lbt.h>
#include <asm/vdso.h>
#include <asm/unwind_hints.h>

#include "signal-common.h"

static int (*save_fp_context)(struct sigcontext __user *sc);
static int (*restore_fp_context)(struct sigcontext __user *sc);

struct rt_sigframe {
	u32 rs_ass[4];		/* argument save space for o32 */
	u32 rs_pad[2];		/* Was: signal trampoline */
	struct siginfo rs_info;
	struct ucontext rs_uc;
};

/*
 * Thread saved context copy to/from a signal context presumed to be on the
 * user stack, and therefore accessed with appropriate macros from uaccess.h.
 */
static int copy_fp_to_sigcontext(struct sigcontext __user *sc)
{
	int i;
	int err = 0;
	int inc = 1;
	uint64_t __user *fcc = &sc->sc_fcc;
	uint32_t __user *csr = &sc->sc_fcsr;
	uint64_t __user *fpregs = (uint64_t *)&sc->sc_fpregs;

	for (i = 0; i < NUM_FPU_REGS; i += inc) {
		err |=
		    __put_user(get_fpr64(&current->thread.fpu.fpr[i], 0),
			       &fpregs[4*i]);
	}
	err |= __put_user(current->thread.fpu.fcsr, csr);
	err |= __put_user(current->thread.fpu.fcc, fcc);

	return err;
}

static int copy_lsx_upper_to_sigcontext(struct sigcontext __user *sc)
{
	int i;
	int err = 0;
	int inc = 1;
	uint64_t __user *fpregs = (uint64_t *)&sc->sc_fpregs;

	for (i = 0; i < NUM_FPU_REGS; i += inc) {
		err |=
		    __put_user(get_fpr64(&current->thread.fpu.fpr[i], 1),
			       &fpregs[4*i+1]);
	}

	return err;
}

static int copy_lasx_upper_to_sigcontext(struct sigcontext __user *sc)
{
	int i;
	int err = 0;
	int inc = 1;
	uint64_t __user *fpregs = (uint64_t *)&sc->sc_fpregs;

	for (i = 0; i < NUM_FPU_REGS; i += inc) {
		err |=
		    __put_user(get_fpr64(&current->thread.fpu.fpr[i], 2),
			       &fpregs[4*i+2]);
		err |=
		    __put_user(get_fpr64(&current->thread.fpu.fpr[i], 3),
			       &fpregs[4*i+3]);
	}

	return err;
}

static int copy_fp_from_sigcontext(struct sigcontext __user *sc)
{
	int i;
	int err = 0;
	int inc = 1;
	u64 fpr_val;
	uint64_t __user *fcc = &sc->sc_fcc;
	uint32_t __user *csr = &sc->sc_fcsr;
	uint64_t __user *fpregs = (uint64_t *)&sc->sc_fpregs;

	for (i = 0; i < NUM_FPU_REGS; i += inc) {
		err |= __get_user(fpr_val, &fpregs[4*i]);
		set_fpr64(&current->thread.fpu.fpr[i], 0, fpr_val);
	}
	err |= __get_user(current->thread.fpu.fcsr, csr);
	err |= __get_user(current->thread.fpu.fcc, fcc);

	return err;
}

static int copy_lsx_upper_from_sigcontext(struct sigcontext __user *sc)
{
	int i;
	int err = 0;
	int inc = 1;
	u64 fpr_val;
	uint64_t __user *fpregs = (uint64_t *)&sc->sc_fpregs;

	for (i = 0; i < NUM_FPU_REGS; i += inc) {
		err |= __get_user(fpr_val, &fpregs[4*i+1]);
		set_fpr64(&current->thread.fpu.fpr[i], 1, fpr_val);
	}

	return err;
}

static int copy_lasx_upper_from_sigcontext(struct sigcontext __user *sc)
{
	int i;
	int err = 0;
	int inc = 1;
	u64 fpr_val;
	uint64_t __user *fpregs = (uint64_t *)&sc->sc_fpregs;

	for (i = 0; i < NUM_FPU_REGS; i += inc) {
		err |= __get_user(fpr_val, &fpregs[4*i+2]);
		set_fpr64(&current->thread.fpu.fpr[i], 2, fpr_val);
		err |= __get_user(fpr_val, &fpregs[4*i+3]);
		set_fpr64(&current->thread.fpu.fpr[i], 3, fpr_val);
	}

	return err;
}

/*
 * Wrappers for the assembly _{save,restore}_fp_context functions.
 */
static int save_hw_fp_context(struct sigcontext __user *sc)
{
	uint64_t __user *fcc = &sc->sc_fcc;
	uint32_t __user *fcsr = &sc->sc_fcsr;
	uint64_t __user *fpregs = (uint64_t *)&sc->sc_fpregs;

	return _save_fp_context(fpregs, fcc, fcsr);
}

static int restore_hw_fp_context(struct sigcontext __user *sc)
{
	uint64_t __user *fcc = &sc->sc_fcc;
	uint32_t __user *csr = &sc->sc_fcsr;
	uint64_t __user *fpregs = (uint64_t *)&sc->sc_fpregs;

	return _restore_fp_context(fpregs, fcc, csr);
}

static int save_lasx_context(struct sigcontext __user *sc)
{
	uint64_t __user *fcc = &sc->sc_fcc;
	uint32_t __user *fcsr = &sc->sc_fcsr;
	uint64_t __user *fpregs = (uint64_t *)&sc->sc_fpregs;

	return _save_lasx_context(fpregs, fcc, fcsr);
}

static int restore_lasx_context(struct sigcontext __user *sc)
{
	uint64_t __user *fcc = &sc->sc_fcc;
	uint32_t __user *fcsr = &sc->sc_fcsr;
	uint64_t __user *fpregs = (uint64_t *)&sc->sc_fpregs;

	return _restore_lasx_context(fpregs, fcc, fcsr);
}

static int save_lsx_context(struct sigcontext __user *sc)
{
	uint64_t __user *fcc = &sc->sc_fcc;
	uint32_t __user *fcsr = &sc->sc_fcsr;
	uint64_t __user *fpregs = (uint64_t *)&sc->sc_fpregs;

	return _save_lsx_context(fpregs, fcc, fcsr);
}

static int restore_lsx_context(struct sigcontext __user *sc)
{
	uint64_t __user *fcc = &sc->sc_fcc;
	uint32_t __user *fcsr = &sc->sc_fcsr;
	uint64_t __user *fpregs = (uint64_t *)&sc->sc_fpregs;

	return _restore_lsx_context(fpregs, fcc, fcsr);
}

#if defined(CONFIG_CPU_HAS_LBT)
static int copy_lbt_to_sigcontext(struct sigcontext __user *sc)
{
	uint64_t __user *scrregs = (uint64_t *)&sc->sc_scr;
	uint32_t __user *eflags = (uint32_t *)&sc->sc_reserved;
	int err = 0;

	err |= __put_user(current->thread.lbt.scr0, &scrregs[0]);
	err |= __put_user(current->thread.lbt.scr1, &scrregs[1]);
	err |= __put_user(current->thread.lbt.scr2, &scrregs[2]);
	err |= __put_user(current->thread.lbt.scr3, &scrregs[3]);
	err |= __put_user(current->thread.lbt.eflag, eflags);

	return err;
}

static int copy_lbt_from_sigcontext(struct sigcontext __user *sc)
{
	int err = 0;
	uint64_t __user *scrregs = (uint64_t *)&sc->sc_scr;
	uint32_t __user *eflags = (uint32_t *)&sc->sc_reserved;

	err |= __get_user(current->thread.lbt.scr0, &scrregs[0]);
	err |= __get_user(current->thread.lbt.scr1, &scrregs[1]);
	err |= __get_user(current->thread.lbt.scr2, &scrregs[2]);
	err |= __get_user(current->thread.lbt.scr3, &scrregs[3]);
	err |= __get_user(current->thread.lbt.eflag, eflags);

	return err;
}

static int save_lbt_context(struct sigcontext __user *sc)
{
	uint64_t __user *scrregs = (uint64_t *)&sc->sc_scr;
	uint32_t __user *eflags = (uint32_t *)&sc->sc_reserved;

	return _save_scr_context(scrregs, eflags);
}

static int restore_lbt_context(struct sigcontext __user *sc)
{
	uint64_t __user *scrregs = (uint64_t *)&sc->sc_scr;
	uint32_t __user *eflags = (uint32_t *)&sc->sc_reserved;

	return _restore_scr_context(scrregs, eflags);
}

static int protected_save_lbt_context(struct sigcontext __user *sc)
{
	int err = 0;
	uint64_t __user *scrregs = (uint64_t *)&sc->sc_scr;
	uint32_t __user *eflags = (uint32_t *)&sc->sc_reserved;

	while (1) {
		lock_fpu_owner();
		if (thread_lbt_context_live()) {
			if (is_lbt_owner()) {
				save_lbt_context(sc);
			} else {
				err |= copy_lbt_to_sigcontext(sc);
			}
		}

		unlock_fpu_owner();
		if (likely(!err))
			break;
		/* touch the sigcontext and try again */
		err = __put_user(0, &scrregs[0]) | __put_user(0, eflags);

		if (err)
			return err;
	}

	return err;
}

static int protected_restore_lbt_context(struct sigcontext __user *sc)
{
	int err = 0, tmp;
	uint64_t __user *scrregs = (uint64_t *)&sc->sc_scr;
	uint32_t __user *eflags = (uint32_t *)&sc->sc_reserved;

	while (1) {
		lock_fpu_owner();
		if (thread_lbt_context_live()) {
			if (is_lbt_owner()) {
				restore_lbt_context(sc);
			} else {
				err |= copy_lbt_from_sigcontext(sc);
			}
		}

		unlock_fpu_owner();
		if (likely(!err))
			break;
		/* touch the sigcontext and try again */
		err = __get_user(tmp, &scrregs[0]) | __get_user(tmp, eflags);

		if (err)
			return err;
	}

	return err;
}
#endif

/*
 * Helper routines
 */
static int protected_save_fp_context(struct sigcontext __user *sc)
{
	int err = 0;
	unsigned int used;
	uint32_t __user *fcc = &sc->sc_fcsr;
	uint32_t __user *fcsr = &sc->sc_fcsr;
	uint32_t __user *flags = &sc->sc_flags;
	uint64_t __user *fpregs = (uint64_t *)&sc->sc_fpregs;

	used = used_math() ? USED_FP : 0;
	if (!used)
		goto fp_done;

	while (1) {
		lock_fpu_owner();
		if (thread_lasx_context_live()) {
			if (is_lasx_enabled()) {
				err = save_lasx_context(sc);
				goto finish;
			} else {
				err |= copy_lasx_upper_to_sigcontext(sc);
				/* LASX contains LSX */
				BUG_ON(!thread_lsx_context_live());
			}
		}
		if (thread_lsx_context_live()) {
			if (is_lsx_enabled()) {
				err = save_lsx_context(sc);
				goto finish;
			} else {
				err |= copy_lsx_upper_to_sigcontext(sc);
			}
		}
		if (is_fpu_owner())
			err = save_fp_context(sc);
		else
			err |= copy_fp_to_sigcontext(sc);
finish:
		unlock_fpu_owner();
		if (likely(!err))
			break;
		/* touch the sigcontext and try again */
		err = __put_user(0, &fpregs[0]) |
			__put_user(0, &fpregs[32*4-1]) |
			__put_user(0, fcc) |
			__put_user(0, fcsr);
		if (err)
			return err;	/* really bad sigcontext */
	}

fp_done:
	return __put_user(used, flags);
}

static int protected_restore_fp_context(struct sigcontext __user *sc)
{
	unsigned int used;
	int err, sig = 0, tmp __maybe_unused;
	uint32_t __user *fcc = &sc->sc_fcsr;
	uint32_t __user *fcsr = &sc->sc_fcsr;
	uint32_t __user *flags = &sc->sc_flags;
	uint64_t __user *fpregs = (uint64_t *)&sc->sc_fpregs;

	err = __get_user(used, flags);
	conditional_used_math(used & USED_FP);

	/*
	 * The signal handler may have used FPU; give it up if the program
	 * doesn't want it following sigreturn.
	 */
	if (err || !(used & USED_FP))
		lose_fpu(0);
	if (err)
		return err;
	if (!(used & USED_FP))
		goto fp_done;

	err = sig = fpcsr_pending(fcsr);
	if (err < 0)
		return err;

	err = 0;

	while (1) {
		lock_fpu_owner();
		if (thread_lasx_context_live()) {
			if (is_lasx_enabled()) {
				err = restore_lasx_context(sc);
				goto finish;
			} else {
				err |= copy_lasx_upper_from_sigcontext(sc);
				/* LASX contains LSX */
				BUG_ON(!thread_lsx_context_live());
			}
		}
		if (thread_lsx_context_live()) {
			if (is_lsx_enabled()) {
				err = restore_lsx_context(sc);
				goto finish;
			} else {
				err |= copy_lsx_upper_from_sigcontext(sc);
				/* LSX contains FP */
				BUG_ON(!used_math());
			}
		}
		if (is_fpu_owner())
			err = restore_fp_context(sc);
		else
			err |= copy_fp_from_sigcontext(sc);
finish:
		unlock_fpu_owner();
		if (likely(!err))
			break;
		/* touch the sigcontext and try again */
		err = __get_user(tmp, &fpregs[0]) |
			__get_user(tmp, &fpregs[32*4-1]) |
			__get_user(tmp, fcc) |
			__get_user(tmp, fcsr);
		if (err)
			break;	/* really bad sigcontext */
	}

fp_done:
	return err ?: sig;
}

static int setup_sigcontext(struct pt_regs *regs, struct sigcontext __user *sc)
{
	int i, err = 0;
	unsigned int sc_flags = 0;

	err |= __put_user(regs->csr_era, &sc->sc_pc);

	err |= __put_user(0, &sc->sc_regs[0]);
	for (i = 1; i < 32; i++)
		err |= __put_user(regs->regs[i], &sc->sc_regs[i]);

	/*
	 * Save FPU state to signal context. Signal handler
	 * will "inherit" current FPU state.
	 */
	err |= protected_save_fp_context(sc);

#if defined(CONFIG_CPU_HAS_LBT)
	err |= protected_save_lbt_context(sc);
#endif
	/*
	 * save load/store access flags to sc_flags of
	 * struct sigcontext when trigger address error.
	 */
	if (current->thread.error_code == 1) {
		err |= __get_user(sc_flags, &sc->sc_flags);
		sc_flags |= ADRERR_RD;
		err |= __put_user(sc_flags, &sc->sc_flags);
	} else if (current->thread.error_code == 2) {
		err |= __get_user(sc_flags, &sc->sc_flags);
		sc_flags |= ADRERR_WR;
		err |= __put_user(sc_flags, &sc->sc_flags);
	}

	return err;
}

int fpcsr_pending(unsigned int __user *fpcsr)
{
	int err, sig = 0;
	unsigned int csr, enabled;

	err = __get_user(csr, fpcsr);
	enabled = ((csr & FPU_CSR_ALL_E) << 24);
	/*
	 * If the signal handler set some FPU exceptions, clear it and
	 * send SIGFPE.
	 */
	if (csr & enabled) {
		csr &= ~enabled;
		err |= __put_user(csr, fpcsr);
		sig = SIGFPE;
	}
	return err ?: sig;
}

static int restore_sigcontext(struct pt_regs *regs, struct sigcontext __user *sc)
{
	int i, err = 0;

	/* Always make any pending restarted system calls return -EINTR */
	current->restart_block.fn = do_no_restart_syscall;

	err |= __get_user(regs->csr_era, &sc->sc_pc);

	for (i = 1; i < 32; i++)
		err |= __get_user(regs->regs[i], &sc->sc_regs[i]);

#if defined(CONFIG_CPU_HAS_LBT)
	err |= protected_restore_lbt_context(sc);
#endif
	return err ?: protected_restore_fp_context(sc);
}

void __user *get_sigframe(struct ksignal *ksig, struct pt_regs *regs,
			  size_t frame_size)
{
	unsigned long sp;

	/* Default to using normal stack */
	sp = regs->regs[3];

	/*
	 * If we are on the alternate signal stack and would overflow it, don't.
	 * Return an always-bogus address instead so we will die with SIGSEGV.
	 */
	if (on_sig_stack(sp) && !likely(on_sig_stack(sp - frame_size)))
		return (void __user __force *)(-1UL);

	sp = sigsp(sp, ksig);

	return (void __user *)((sp - frame_size) & STACK_ALIGN);
}

/*
 * Atomically swap in the new signal mask, and wait for a signal.
 */

asmlinkage void sys_rt_sigreturn(void)
{
	int sig;
	sigset_t set;
	struct pt_regs *regs;
	struct rt_sigframe __user *frame;

	regs = current_pt_regs();
	frame = (struct rt_sigframe __user *)regs->regs[3];
	if (!access_ok(VERIFY_READ, frame, sizeof(*frame)))
		goto badframe;
	if (__copy_from_user(&set, &frame->rs_uc.uc_sigmask, sizeof(set)))
		goto badframe;

	set_current_blocked(&set);

	sig = restore_sigcontext(regs, &frame->rs_uc.uc_mcontext);
	if (sig < 0)
		goto badframe;
	else if (sig)
		force_sig(sig, current);

	if (restore_altstack(&frame->rs_uc.uc_stack))
		goto badframe;

	/*
	 * Don't let your children do this ...
	 */
	__asm__ __volatile__(
		"or\t$sp, $zero, %0\n\t"
		NOT_SIBLING_CALL_HINT
		"b\tsyscall_exit"
		: /* no outputs */
		: "r" (regs)
		: "$sp");
	unreachable();

badframe:
	force_sig(SIGSEGV, current);
}

static int setup_rt_frame(void *sig_return, struct ksignal *ksig,
			  struct pt_regs *regs, sigset_t *set)
{
	int err = 0;
	struct rt_sigframe __user *frame;

	frame = get_sigframe(ksig, regs, sizeof(*frame));
	if (!access_ok(VERIFY_WRITE, frame, sizeof (*frame)))
		return -EFAULT;

	/* Create siginfo.  */
	err |= copy_siginfo_to_user(&frame->rs_info, &ksig->info);

	/* Create the ucontext.	 */
	err |= __put_user(0, &frame->rs_uc.uc_flags);
	err |= __put_user(NULL, &frame->rs_uc.uc_link);
	err |= __save_altstack(&frame->rs_uc.uc_stack, regs->regs[3]);
	err |= setup_sigcontext(regs, &frame->rs_uc.uc_mcontext);
	err |= __copy_to_user(&frame->rs_uc.uc_sigmask, set, sizeof(*set));

	if (err)
		return -EFAULT;

	/*
	 * Arguments to signal handler:
	 *
	 *   a0 = signal number
	 *   a1 = 0 (should be cause)
	 *   a2 = pointer to ucontext
	 *
	 * csr_era point to the signal handler, $3 points to
	 * the struct rt_sigframe.
	 */
#ifdef CONFIG_CPU_HAS_LBT
	if (current_thread_info()->tp_value)
		regs->regs[2] = current_thread_info()->tp_value;
#endif
	regs->regs[4] = ksig->sig;
	regs->regs[5] = (unsigned long) &frame->rs_info;
	regs->regs[6] = (unsigned long) &frame->rs_uc;
	regs->regs[3] = (unsigned long) frame;
	regs->regs[1] = (unsigned long) sig_return;
	regs->csr_era = (unsigned long) ksig->ka.sa.sa_handler;

	DEBUGP("SIG deliver (%s:%d): sp=0x%p pc=0x%lx ra=0x%lx\n",
	       current->comm, current->pid,
	       frame, regs->csr_era, regs->regs[1]);

	return 0;
}

static void handle_signal(struct ksignal *ksig, struct pt_regs *regs)
{
	sigset_t *oldset = sigmask_to_save();
	int ret;
	void *vdso = current->mm->context.vdso;

	/* Are we from a system call? */
	if (regs->regs[0]) {
		switch(regs->regs[4]) {
		case -ERESTART_RESTARTBLOCK:
		case -ERESTARTNOHAND:
			regs->regs[4] = -EINTR;
			break;
		case -ERESTARTSYS:
			if (!(ksig->ka.sa.sa_flags & SA_RESTART)) {
				regs->regs[4] = -EINTR;
				break;
			}
		/* fallthrough */
		case -ERESTARTNOINTR:
			regs->regs[4] = regs->orig_a0;
			regs->csr_era -= 4;
		}

		regs->regs[0] = 0;		/* Don't deal with this again.	*/
	}

	rseq_signal_deliver(ksig, regs);

	ret = setup_rt_frame(vdso + current->thread.vdso->off_rt_sigreturn, ksig, regs, oldset);

	signal_setup_done(ret, ksig, 0);
}

static void do_signal(struct pt_regs *regs)
{
	struct ksignal ksig;

	if (get_signal(&ksig)) {
		/* Whee!  Actually deliver the signal.	*/
		handle_signal(&ksig, regs);
		return;
	}

	/* Are we from a system call? */
	if (regs->regs[0]) {
		switch (regs->regs[4]) {
		case -ERESTARTNOHAND:
		case -ERESTARTSYS:
		case -ERESTARTNOINTR:
			regs->regs[4] = regs->orig_a0;
			regs->csr_era -= 4;
			break;

		case -ERESTART_RESTARTBLOCK:
			regs->regs[4] = regs->orig_a0;
			regs->regs[11] = __NR_restart_syscall;
			regs->csr_era -= 4;
			break;
		}
		regs->regs[0] = 0;	/* Don't deal with this again.	*/
	}

	/*
	 * If there's no signal to deliver, we just put the saved sigmask
	 * back
	 */
	restore_saved_sigmask();
}

/*
 * notification of userspace execution resumption
 * - triggered by the TIF_WORK_MASK flags
 */
asmlinkage void do_notify_resume(struct pt_regs *regs, void *unused,
	__u32 thread_info_flags)
{
	local_irq_enable();

	user_exit();

	if (thread_info_flags & _TIF_UPROBE)
		uprobe_notify_resume(regs);

	if (thread_info_flags & _TIF_PATCH_PENDING)
		klp_update_patch_state(current);

	/* deal with pending signal delivery */
	if (thread_info_flags & _TIF_SIGPENDING)
		do_signal(regs);

	if (thread_info_flags & _TIF_NOTIFY_RESUME) {
		clear_thread_flag(TIF_NOTIFY_RESUME);
		tracehook_notify_resume(regs);
		rseq_handle_notify_resume(NULL, regs);
	}

	user_enter();
}

static int signal_setup(void)
{
	if (cpu_has_fpu) {
		save_fp_context = save_hw_fp_context;
		restore_fp_context = restore_hw_fp_context;
	} else {
		save_fp_context = copy_fp_to_sigcontext;
		restore_fp_context = copy_fp_from_sigcontext;
	}

	return 0;
}

arch_initcall(signal_setup);
