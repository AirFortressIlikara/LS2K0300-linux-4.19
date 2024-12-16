/* SPDX-License-Identifier: GPL-2.0+ */
/*
* Copyright (C) 2020 Loongson Technology Corporation Limited
*
* Author: Hanlu Li <lihanlu@loongson.cn>
*/

#ifndef __SIGNAL_COMMON_H
#define __SIGNAL_COMMON_H

/* #define DEBUG_SIG */

#ifdef DEBUG_SIG
#  define DEBUGP(fmt, args...) printk("%s: " fmt, __func__, ##args)
#else
#  define DEBUGP(fmt, args...)
#endif

/*
 * Determine which stack to use..
 */
extern void __user *get_sigframe(struct ksignal *ksig, struct pt_regs *regs,
				 size_t frame_size);
/* Check and clear pending FPU exceptions in saved CSR */
extern int fpcsr_pending(unsigned int __user *fpcsr);

/* Make sure we will not lose FPU ownership */
#define lock_fpu_owner()	({ preempt_disable(); pagefault_disable(); })
#define unlock_fpu_owner()	({ pagefault_enable(); preempt_enable(); })

/* Assembly functions to move context to/from the FPU */
extern asmlinkage int
_save_fp_context(void __user *fpregs, void __user *fcc, void __user *csr);
extern asmlinkage int
_restore_fp_context(void __user *fpregs, void __user *fcc, void __user *csr);
extern asmlinkage int
_save_lsx_context(void __user *fpregs, void __user *fcc, void __user *fcsr);
extern asmlinkage int
_restore_lsx_context(void __user *fpregs, void __user *fcc, void __user *fcsr);
extern asmlinkage int
_save_lasx_context(void __user *fpregs, void __user *fcc, void __user *fcsr);
extern asmlinkage int
_restore_lasx_context(void __user *fpregs, void __user *fcc, void __user *fcsr);
#if defined(CONFIG_CPU_HAS_LBT)
extern asmlinkage int
_save_scr_context(void __user *scr, void __user *eflags);
extern asmlinkage int
_restore_scr_context(void __user *scr, void __user *eflags);
#endif

#endif	/* __SIGNAL_COMMON_H */
