// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (C) 2020 Loongson Technology Corporation Limited
*
* Author: Hanlu Li <lihanlu@loongson.cn>
*/
#include <linux/compiler.h>
#include <linux/context_tracking.h>
#include <linux/elf.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/sched/task_stack.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/ptrace.h>
#include <linux/regset.h>
#include <linux/smp.h>
#include <linux/security.h>
#include <linux/stddef.h>
#include <linux/tracehook.h>
#include <linux/audit.h>
#include <linux/seccomp.h>
#include <linux/ftrace.h>

#include <asm/byteorder.h>
#include <asm/cpu.h>
#include <asm/cpu-info.h>
#include <asm/fpu.h>
#include <asm/loongarchregs.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/processor.h>
#include <asm/syscall.h>
#include <linux/uaccess.h>
#include <asm/bootinfo.h>
#include <asm/reg.h>
#include <asm/watch.h>
#include <asm/ptrace.h>

#define CREATE_TRACE_POINTS
#include <trace/events/syscalls.h>

static void init_fp_ctx(struct task_struct *target)
{
	/* The target already has context */
	if (tsk_used_math(target))
		return;

	/* Begin with data registers set to all 1s... */
	memset(&target->thread.fpu.fpr, ~0, sizeof(target->thread.fpu.fpr));
	set_stopped_child_used_math(target);
}

/*
 * Called by kernel/ptrace.c when detaching..
 *
 * Make sure single step bits etc are not set.
 */
void ptrace_disable(struct task_struct *child)
{
	/* Don't load the watchpoint registers for the ex-child. */
	clear_tsk_thread_flag(child, TIF_LOAD_WATCH);
	clear_tsk_thread_flag(child, TIF_SINGLESTEP);

}

/* regset get/set implementations */

static int gpr_get(struct task_struct *target,
		   const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   void *kbuf, void __user *ubuf)
{
	int err;
	int era_start = sizeof(u64) * GPR_NUM;
	int badv_start = era_start + sizeof(u64);
	struct pt_regs *regs = task_pt_regs(target);

	err = user_regset_copyout(&pos, &count, &kbuf, &ubuf,
				  &regs->regs,
				  0, era_start);
	err |= user_regset_copyout(&pos, &count, &kbuf, &ubuf,
				   &regs->csr_era,
				   era_start, era_start + sizeof(u64));
	err |= user_regset_copyout(&pos, &count, &kbuf, &ubuf,
				   &regs->csr_badv,
				   badv_start, badv_start + sizeof(u64));

	return err;
}

static int gpr_set(struct task_struct *target,
		   const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   const void *kbuf, const void __user *ubuf)
{
	int err;
	int era_start = sizeof(u64) * GPR_NUM;
	struct pt_regs *regs = task_pt_regs(target);

	err = user_regset_copyin(&pos, &count, &kbuf, &ubuf,
				 &regs->regs,
				 0, era_start);
	err |= user_regset_copyin(&pos, &count, &kbuf, &ubuf,
				  &regs->csr_era,
				  era_start, era_start + sizeof(u64));

	return err;
}


/*
 * Get the general floating-point registers.
 */
static int gfpr_get(struct task_struct *target,
		    unsigned int *pos, unsigned int *count,
		    void **kbuf, void __user **ubuf)
{
	return user_regset_copyout(pos, count, kbuf, ubuf,
				   &target->thread.fpu.fpr,
				   0, sizeof(elf_fpreg_t) * NUM_FPU_REGS);
}

static int gfpr_get_simd(struct task_struct *target,
			 unsigned int *pos, unsigned int *count,
			 void **kbuf, void __user **ubuf)
{
	unsigned int i;
	u64 fpr_val;
	int err;

	BUILD_BUG_ON(sizeof(fpr_val) != sizeof(elf_fpreg_t));
	for (i = 0; i < NUM_FPU_REGS; i++) {
		fpr_val = get_fpr64(&target->thread.fpu.fpr[i], 0);
		err = user_regset_copyout(pos, count, kbuf, ubuf,
					  &fpr_val, i * sizeof(elf_fpreg_t),
					  (i + 1) * sizeof(elf_fpreg_t));
		if (err)
			return err;
	}

	return 0;
}

/*
 * Choose the appropriate helper for general registers, and then copy
 * the FCC and FCSR registers separately.
 */
static int fpr_get(struct task_struct *target,
		   const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   void *kbuf, void __user *ubuf)
{
	const int fcc_start = NUM_FPU_REGS * sizeof(elf_fpreg_t);
	const int fcsr_start = fcc_start + sizeof(u64);
	int err;

	if (sizeof(target->thread.fpu.fpr[0]) == sizeof(elf_fpreg_t))
		err = gfpr_get(target, &pos, &count, &kbuf, &ubuf);
	else
		err = gfpr_get_simd(target, &pos, &count, &kbuf, &ubuf);

	err |= user_regset_copyout(&pos, &count, &kbuf, &ubuf,
				   &target->thread.fpu.fcc,
				   fcc_start, fcsr_start);
	err |= user_regset_copyout(&pos, &count, &kbuf, &ubuf,
				   &target->thread.fpu.fcsr,
				   fcsr_start, fcsr_start + sizeof(u32));

	return err;
}

static int gfpr_set(struct task_struct *target,
		    unsigned int *pos, unsigned int *count,
		    const void **kbuf, const void __user **ubuf)
{
	return user_regset_copyin(pos, count, kbuf, ubuf,
				  &target->thread.fpu.fpr,
				  0, NUM_FPU_REGS * sizeof(elf_fpreg_t));
}

static int gfpr_set_simd(struct task_struct *target,
		       unsigned int *pos, unsigned int *count,
		       const void **kbuf, const void __user **ubuf)
{
	unsigned int i;
	u64 fpr_val;
	int err;

	BUILD_BUG_ON(sizeof(fpr_val) != sizeof(elf_fpreg_t));
	for (i = 0; i < NUM_FPU_REGS && *count > 0; i++) {
		err = user_regset_copyin(pos, count, kbuf, ubuf,
					 &fpr_val, i * sizeof(elf_fpreg_t),
					 (i + 1) * sizeof(elf_fpreg_t));
		if (err)
			return err;
		set_fpr64(&target->thread.fpu.fpr[i], 0, fpr_val);
	}

	return 0;
}

/*
 * Choose the appropriate helper for general registers, and then copy
 * the FCC register separately.
 */
static int fpr_set(struct task_struct *target,
		   const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   const void *kbuf, const void __user *ubuf)
{
	const int fcc_start = NUM_FPU_REGS * sizeof(elf_fpreg_t);
	const int fcsr_start = fcc_start + sizeof(u64);
	int err;

	BUG_ON(count % sizeof(elf_fpreg_t));
	if (pos + count > sizeof(elf_fpregset_t))
		return -EIO;

	init_fp_ctx(target);

	if (sizeof(target->thread.fpu.fpr[0]) == sizeof(elf_fpreg_t))
		err = gfpr_set(target, &pos, &count, &kbuf, &ubuf);
	else
		err = gfpr_set_simd(target, &pos, &count, &kbuf, &ubuf);
	if (err)
		return err;

	err |= user_regset_copyin(&pos, &count, &kbuf, &ubuf,
				  &target->thread.fpu.fcc, fcc_start,
				  fcc_start + sizeof(u64));
	err |= user_regset_copyin(&pos, &count, &kbuf, &ubuf,
				  &target->thread.fpu.fcsr, fcsr_start,
				  fcsr_start + sizeof(u32));

	return err;
}

static int cfg_get(struct task_struct *target,
		   const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   void *kbuf, void __user *ubuf)
{
	int i = 0;
	u32 cfg_val;
	int err;

	while (count > 0) {
		cfg_val = read_cpucfg(i++);
		err = user_regset_copyout(&pos, &count, &kbuf, &ubuf,
					  &cfg_val,
					  (i-1) * sizeof(u32), i * sizeof(u32));
		if (err)
			return err;
	}

	return err;
}

/*
 * CFG registers are read-only.
 */
static int cfg_set(struct task_struct *target,
		   const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   const void *kbuf, const void __user *ubuf)
{
	return 0;
}

#ifdef CONFIG_CPU_HAS_LBT
static int lbt_get(struct task_struct *target,
		   const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   void *kbuf, void __user *ubuf)
{
	int err;
	const int ftop_start = (regset->n - 1) * regset->size;
	err = user_regset_copyout(&pos, &count, &kbuf, &ubuf,
				   &target->thread.lbt.scr0,
				   0, ftop_start);
	err |= user_regset_copyout(&pos, &count, &kbuf, &ubuf,
				   &target->thread.fpu.ftop,
				   ftop_start, ftop_start + sizeof(u64));
	return err;
}

static int lbt_set(struct task_struct *target,
		   const struct user_regset *regset,
		   unsigned int pos, unsigned int count,
		   const void *kbuf, const void __user *ubuf)
{
	int err;
	const int ftop_start = (regset->n - 1) * regset->size;

	err = user_regset_copyin(&pos, &count, &kbuf, &ubuf,
				 &target->thread.lbt.scr0, 0, ftop_start);
	err |= user_regset_copyin(&pos, &count, &kbuf, &ubuf,
				  &target->thread.fpu.ftop, ftop_start,
				  ftop_start + sizeof(u64));
	return err;
}
#endif

#ifdef CONFIG_CPU_HAS_LSX

static int copy_pad_fprs(struct task_struct *target,
			 const struct user_regset *regset,
			 unsigned int *ppos, unsigned int *pcount,
			 void **pkbuf, void __user **pubuf,
			 unsigned int live_sz)
{
	int i, j, start, start_pad, err;
	unsigned long long fill = ~0ull;
	unsigned int cp_sz, pad_sz;

	cp_sz = min(regset->size, live_sz);
	pad_sz = regset->size - cp_sz;
	WARN_ON(pad_sz % sizeof(fill));

	i = start = err = 0;
	for (; i < NUM_FPU_REGS; i++, start += regset->size) {
		err |= user_regset_copyout(ppos, pcount, pkbuf, pubuf,
					   &target->thread.fpu.fpr[i],
					   start, start + cp_sz);

		start_pad = start + cp_sz;
		for (j = 0; j < (pad_sz / sizeof(fill)); j++) {
			err |= user_regset_copyout(ppos, pcount, pkbuf, pubuf,
						   &fill, start_pad,
						   start_pad + sizeof(fill));
			start_pad += sizeof(fill);
		}
	}

	return err;
}

static int simd_get(struct task_struct *target,
		    const struct user_regset *regset,
		    unsigned int pos, unsigned int count,
		    void *kbuf, void __user *ubuf)
{
	const unsigned int wr_size = NUM_FPU_REGS * regset->size;
	int err;

	if (!tsk_used_math(target)) {
		/* The task hasn't used FP or LSX, fill with 0xff */
		err = copy_pad_fprs(target, regset, &pos, &count,
				    &kbuf, &ubuf, 0);
	} else if (!test_tsk_thread_flag(target, TIF_LSX_CTX_LIVE)) {
		/* Copy scalar FP context, fill the rest with 0xff */
		err = copy_pad_fprs(target, regset, &pos, &count,
				    &kbuf, &ubuf, 8);
#ifdef CONFIG_CPU_HAS_LASX
	} else if (!test_tsk_thread_flag(target, TIF_LASX_CTX_LIVE)) {
		/* Copy LSX 128 Bit context, fill the rest with 0xff */
		err = copy_pad_fprs(target, regset, &pos, &count,
				    &kbuf, &ubuf, 16);
#endif

	} else if (sizeof(target->thread.fpu.fpr[0]) == regset->size) {
		/* Trivially copy the vector registers */
		err = user_regset_copyout(&pos, &count, &kbuf, &ubuf,
					  &target->thread.fpu.fpr,
					  0, wr_size);
	} else {
		/* Copy as much context as possible, fill the rest with 0xff */
		err = copy_pad_fprs(target, regset, &pos, &count,
				    &kbuf, &ubuf,
				    sizeof(target->thread.fpu.fpr[0]));
	}

	return err;
}

static int simd_set(struct task_struct *target,
		    const struct user_regset *regset,
		    unsigned int pos, unsigned int count,
		    const void *kbuf, const void __user *ubuf)
{
	const unsigned int wr_size = NUM_FPU_REGS * regset->size;
	unsigned int cp_sz;
	int i, err, start;

	init_fp_ctx(target);

	if (sizeof(target->thread.fpu.fpr[0]) == regset->size) {
		/* Trivially copy the vector registers */
		err = user_regset_copyin(&pos, &count, &kbuf, &ubuf,
					 &target->thread.fpu.fpr,
					 0, wr_size);
	} else {
		/* Copy as much context as possible */
		cp_sz = min_t(unsigned int, regset->size,
			      sizeof(target->thread.fpu.fpr[0]));

		i = start = err = 0;
		for (; i < NUM_FPU_REGS; i++, start += regset->size) {
			err |= user_regset_copyin(&pos, &count, &kbuf, &ubuf,
						  &target->thread.fpu.fpr[i],
						  start, start + cp_sz);
		}
	}

	return err;
}

#endif /* CONFIG_CPU_HAS_LSX */

struct pt_regs_offset {
	const char *name;
	int offset;
};

#define REG_OFFSET_NAME(n, r) {.name = #n, .offset = offsetof(struct pt_regs, r)}
#define REG_OFFSET_END {.name = NULL, .offset = 0}

static const struct pt_regs_offset regoffset_table[] = {
	REG_OFFSET_NAME(r0, regs[0]),
	REG_OFFSET_NAME(r1, regs[1]),
	REG_OFFSET_NAME(r2, regs[2]),
	REG_OFFSET_NAME(r3, regs[3]),
	REG_OFFSET_NAME(r4, regs[4]),
	REG_OFFSET_NAME(r5, regs[5]),
	REG_OFFSET_NAME(r6, regs[6]),
	REG_OFFSET_NAME(r7, regs[7]),
	REG_OFFSET_NAME(r8, regs[8]),
	REG_OFFSET_NAME(r9, regs[9]),
	REG_OFFSET_NAME(r10, regs[10]),
	REG_OFFSET_NAME(r11, regs[11]),
	REG_OFFSET_NAME(r12, regs[12]),
	REG_OFFSET_NAME(r13, regs[13]),
	REG_OFFSET_NAME(r14, regs[14]),
	REG_OFFSET_NAME(r15, regs[15]),
	REG_OFFSET_NAME(r16, regs[16]),
	REG_OFFSET_NAME(r17, regs[17]),
	REG_OFFSET_NAME(r18, regs[18]),
	REG_OFFSET_NAME(r19, regs[19]),
	REG_OFFSET_NAME(r20, regs[20]),
	REG_OFFSET_NAME(r21, regs[21]),
	REG_OFFSET_NAME(r22, regs[22]),
	REG_OFFSET_NAME(r23, regs[23]),
	REG_OFFSET_NAME(r24, regs[24]),
	REG_OFFSET_NAME(r25, regs[25]),
	REG_OFFSET_NAME(r26, regs[26]),
	REG_OFFSET_NAME(r27, regs[27]),
	REG_OFFSET_NAME(r28, regs[28]),
	REG_OFFSET_NAME(r29, regs[29]),
	REG_OFFSET_NAME(r30, regs[30]),
	REG_OFFSET_NAME(r31, regs[31]),
	REG_OFFSET_NAME(csr_era, csr_era),
	REG_OFFSET_NAME(csr_badv, csr_badv),
	REG_OFFSET_NAME(csr_crmd, csr_crmd),
	REG_OFFSET_NAME(csr_prmd, csr_prmd),
	REG_OFFSET_NAME(csr_euen, csr_euen),
	REG_OFFSET_NAME(csr_ecfg, csr_ecfg),
	REG_OFFSET_NAME(csr_estat, csr_estat),
	REG_OFFSET_NAME(orig_a0, orig_a0),
	REG_OFFSET_END,
};

/**
 * regs_query_register_offset() - query register offset from its name
 * @name:       the name of a register
 *
 * regs_query_register_offset() returns the offset of a register in struct
 * pt_regs from its name. If the name is invalid, this returns -EINVAL;
 */
int regs_query_register_offset(const char *name)
{
	const struct pt_regs_offset *roff;
	for (roff = regoffset_table; roff->name != NULL; roff++)
		if (!strcmp(roff->name, name))
			return roff->offset;
	return -EINVAL;
}

enum loongarch_regset {
	REGSET_GPR,
	REGSET_FPR,
	REGSET_CPUCFG,
#ifdef CONFIG_CPU_HAS_LBT
	REGSET_LBT,
#endif
#ifdef CONFIG_CPU_HAS_LSX
	REGSET_LSX,
#endif
#ifdef CONFIG_CPU_HAS_LASX
	REGSET_LASX,
#endif
};

static const struct user_regset loongarch64_regsets[] = {
	[REGSET_GPR] = {
		.core_note_type	= NT_PRSTATUS,
		.n		= ELF_NGREG,
		.size		= sizeof(elf_greg_t),
		.align		= sizeof(elf_greg_t),
		.get		= gpr_get,
		.set		= gpr_set,
	},
	[REGSET_FPR] = {
		.core_note_type	= NT_PRFPREG,
		.n		= ELF_NFPREG,
		.size		= sizeof(elf_fpreg_t),
		.align		= sizeof(elf_fpreg_t),
		.get		= fpr_get,
		.set		= fpr_set,
	},
	[REGSET_CPUCFG] = {
		.core_note_type	= NT_LOONGARCH_CPUCFG,
		.n		= 64,
		.size		= sizeof(u32),
		.align		= sizeof(u32),
		.get		= cfg_get,
		.set		= cfg_set,
	},
#ifdef CONFIG_CPU_HAS_LBT
	[REGSET_LBT] = {
		.core_note_type	= NT_LOONGARCH_LBT,
		.n		= 6,
		.size		= sizeof(u64),
		.align		= sizeof(u64),
		.get		= lbt_get,
		.set		= lbt_set,
	},
#endif
#ifdef CONFIG_CPU_HAS_LSX
	[REGSET_LSX] = {
		.core_note_type	= NT_LOONGARCH_LSX,
		.n		= NUM_FPU_REGS,
		.size		= 16,
		.align		= 16,
		.get		= simd_get,
		.set		= simd_set,
	},
#endif
#ifdef CONFIG_CPU_HAS_LASX
	[REGSET_LASX] = {
		.core_note_type	= NT_LOONGARCH_LASX,
		.n		= NUM_FPU_REGS,
		.size		= 32,
		.align		= 32,
		.get		= simd_get,
		.set		= simd_set,
	},
#endif
};

static const struct user_regset_view user_loongarch64_view = {
	.name		= "loongarch64",
	.e_machine	= ELF_ARCH,
	.regsets	= loongarch64_regsets,
	.n		= ARRAY_SIZE(loongarch64_regsets),
};


const struct user_regset_view *task_user_regset_view(struct task_struct *task)
{
	return &user_loongarch64_view;
}

static inline int read_user(struct task_struct *target, unsigned long addr,
			    unsigned long __user *data)
{
	unsigned long tmp = 0;

	switch (addr) {
	case 0 ... 31:
		tmp = task_pt_regs(target)->regs[addr];
		break;
	case PC:
		tmp = task_pt_regs(target)->csr_era;
		break;
	case BADVADDR:
		tmp = task_pt_regs(target)->csr_badv;
		break;
	default:
		return -EIO;
	}

	return put_user(tmp, data);
}

static inline int write_user(struct task_struct *target, unsigned long addr,
			    unsigned long data)
{
	switch (addr) {
	case 0 ... 31:
		task_pt_regs(target)->regs[addr] = data;
		break;
	case PC:
		task_pt_regs(target)->csr_era = data;
		break;
	case BADVADDR:
		task_pt_regs(target)->csr_badv = data;
		break;
	default:
		return -EIO;
	}

	return 0;
}

int ptrace_get_watch_regs(struct task_struct *child,
			  struct pt_watch_regs __user *addr)
{
	enum pt_watch_style style;
	int i;
	unsigned int cnt;

	if (!cpu_has_watch || boot_cpu_data.watch_reg_use_cnt == 0)
		return -EIO;
	if (!access_ok(VERIFY_WRITE, addr, sizeof(struct pt_watch_regs)))
		return -EIO;

#ifdef CONFIG_32BIT
	style = pt_watch_style_la32;
#define WATCH_STYLE la32
#else
	style = pt_watch_style_la64;
#define WATCH_STYLE la64
#endif

	loongarch_update_watch_registers(child);

	/* Reserve the first instruction watchpoint if TIF_SINGLESTEP is set. */
	if (unlikely(test_thread_flag(TIF_SINGLESTEP)))
		child->thread.watch.loongarch3264.irwmask[boot_cpu_data.watch_dreg_count] = 0;

	__get_user(cnt, &addr->max_valid);
	cnt = min(boot_cpu_data.watch_reg_use_cnt, cnt);
	__put_user(cnt, &addr->num_valid);
	__put_user(style, &addr->style);
	for (i = 0; i < cnt; i++) {
		__put_user(child->thread.watch.loongarch3264.addr[i],
			   &addr->WATCH_STYLE[i].addr);
		__put_user(child->thread.watch.loongarch3264.mask[i],
			   &addr->WATCH_STYLE[i].mask);
		__put_user(child->thread.watch.loongarch3264.irw[i],
			   &addr->WATCH_STYLE[i].irw);
		__put_user(child->thread.watch.loongarch3264.irwstat[i],
			   &addr->WATCH_STYLE[i].irwstat);
		__put_user(child->thread.watch.loongarch3264.irwmask[i],
			   &addr->WATCH_STYLE[i].irwmask);
	}

	return 0;
}

int ptrace_set_watch_regs(struct task_struct *child,
			  struct pt_watch_regs __user *addr)
{
	int i;
	unsigned int cnt;
	int watch_active = 0;
	unsigned long addrt[NUM_WATCH_REGS];
	unsigned long maskt[NUM_WATCH_REGS];
	unsigned char irwt[NUM_WATCH_REGS];

	if (!cpu_has_watch || boot_cpu_data.watch_reg_use_cnt == 0)
		return -EIO;
	if (!access_ok(VERIFY_READ, addr, sizeof(struct pt_watch_regs)))
		return -EIO;

	__get_user(cnt, &addr->max_valid);
	cnt = min(boot_cpu_data.watch_reg_use_cnt, cnt);
	/* Check the values. */
	for (i = 0; i < cnt; i++) {
		__get_user(addrt[i], &addr->WATCH_STYLE[i].addr);
#ifdef CONFIG_32BIT
		if (addrt[i] & __UA_LIMIT)
			return -EINVAL;
#else
		if (test_tsk_thread_flag(child, TIF_32BIT_ADDR)) {
			if (addrt[i] & 0xffffffff80000000UL)
				return -EINVAL;
		} else {
			if (addrt[i] & __UA_LIMIT)
				return -EINVAL;
		}
#endif
		__get_user(maskt[i], &addr->WATCH_STYLE[i].mask);
		__get_user(irwt[i], &addr->WATCH_STYLE[i].irw);
	}
	/* Install them. */
	for (i = 0; i < boot_cpu_data.watch_reg_use_cnt; i++) {
		if (irwt[i] & LA_WATCH_IRW)
			watch_active = 1;
		child->thread.watch.loongarch3264.addr[i] = addrt[i];
		child->thread.watch.loongarch3264.mask[i] = maskt[i];
		child->thread.watch.loongarch3264.irw[i] = irwt[i];
	}

	if (watch_active)
		set_tsk_thread_flag(child, TIF_LOAD_WATCH);
	else
		clear_tsk_thread_flag(child, TIF_LOAD_WATCH);

	return 0;
}


long arch_ptrace(struct task_struct *child, long request,
		 unsigned long addr, unsigned long data)
{
	int ret;
	void __user *addrp = (void __user *) addr;
	unsigned long __user *datap = (void __user *) data;

	switch (request) {
	case PTRACE_PEEKUSR:
		ret = read_user(child, addr, datap);
		break;

	case PTRACE_POKEUSR:
		ret = write_user(child, addr, data);
		break;

	case PTRACE_GET_WATCH_REGS:
		ret = ptrace_get_watch_regs(child, addrp);
		break;

	case PTRACE_SET_WATCH_REGS:
		ret = ptrace_set_watch_regs(child, addrp);
		break;

	default:
		ret = ptrace_request(child, request, addr, data);
		break;
	}

	return ret;
}

asmlinkage long syscall_trace_enter(struct pt_regs *regs, long syscall)
{
	user_exit();

	if (test_thread_flag(TIF_SYSCALL_TRACE))
		if (tracehook_report_syscall_entry(regs))
			syscall_set_nr(current, regs, -1);

#ifdef CONFIG_SECCOMP
	if (secure_computing(NULL) == -1)
		return -1;
#endif

	if (unlikely(test_thread_flag(TIF_SYSCALL_TRACEPOINT)))
		trace_sys_enter(regs, syscall);

	audit_syscall_entry(syscall, regs->regs[4], regs->regs[5],
			    regs->regs[6], regs->regs[7]);

	/* set errno for negative syscall number. */
	if (syscall < 0)
		syscall_set_return_value(current, regs, -ENOSYS, 0);
	return syscall;
}

asmlinkage void syscall_trace_leave(struct pt_regs *regs)
{
	/*
	 * We may come here right after calling schedule_user()
	 * or do_notify_resume(), in which case we can be in RCU
	 * user mode.
	 */
	user_exit();

	audit_syscall_exit(regs);

	if (unlikely(test_thread_flag(TIF_SYSCALL_TRACEPOINT)))
		trace_sys_exit(regs, regs_return_value(regs));

	if (test_thread_flag(TIF_SYSCALL_TRACE))
		tracehook_report_syscall_exit(regs, 0);

	user_enter();
}

void user_enable_single_step(struct task_struct *task)
{
	struct thread_info *ti = task_thread_info(task);
	int i = boot_cpu_data.watch_dreg_count;

	task->thread.watch.loongarch3264.addr[i] = task_pt_regs(task)->csr_era;
	task->thread.watch.loongarch3264.mask[i] = -1UL;
	task->thread.watch.loongarch3264.irw[i] = LA_WATCH_I;
	task->thread.single_step = task_pt_regs(task)->csr_era;
	set_ti_thread_flag(ti, TIF_SINGLESTEP);
}
EXPORT_SYMBOL(user_enable_single_step);


void user_disable_single_step(struct task_struct *task)
{
	clear_tsk_thread_flag(task, TIF_SINGLESTEP);
}
EXPORT_SYMBOL(user_disable_single_step);
