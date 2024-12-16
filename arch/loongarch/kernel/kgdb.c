/*
 *  Originally written by Glenn Engel, Lake Stevens Instrument Division
 *
 *  Contributed by HP Systems
 *
 *  Modified for Linux/LoongArch (and LoongArch in general) by Andreas Busse
 *  Send complaints, suggestions etc. to <andy@waldorf-gmbh.de>
 *
 *  Copyright (C) 1995 Andreas Busse
 *
 *  Copyright (C) 2003 MontaVista Software Inc.
 *  Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 *
 *  Copyright (C) 2004-2005 MontaVista Software Inc.
 *  Author: Manish Lachwani, mlachwani@mvista.com or manish@koffee-break.com
 *
 *  Copyright (C) 2007-2008 Wind River Systems, Inc.
 *  Author/Maintainer: Jason Wessel, jason.wessel@windriver.com
 *
 *  Copyright (C) 2020 Loongson Technology Corporation Limited
 *  This file is licensed under the terms of the GNU General Public License
 *  version 2. This program is licensed "as is" without any warranty of any
 *  kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/ptrace.h>		/* for linux pt_regs struct */
#include <linux/kgdb.h>
#include <linux/kdebug.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <asm/inst.h>
#include <asm/fpu.h>
#include <asm/cacheflush.h>
#include <asm/processor.h>
#include <asm/sigcontext.h>
#include <asm/irq_regs.h>
#include <asm/ptrace.h>
#include <asm/watch.h>
#include <asm/asm-offsets.h>
static int kgdb_watch_dcount;
static int kgdb_watch_icount;
int kgdb_watch_activated;

int param_set_dcount(const char *val, const struct kernel_param *kp)
{
	int dbcn, d, ret;
	ret = kstrtoint(val, 0, &d);
	if (ret < 0)
		return ret;
	dbcn = csr_read32(LOONGARCH_CSR_MWPC) & 0x3f;
	if (d > dbcn)
		return -EINVAL;
	boot_cpu_data.watch_dreg_count = dbcn - d;
	*(int *)kp->arg = d;
	return 0;
}

int param_set_icount(const char *val, const struct kernel_param *kp)
{
	int ibcn, d, ret;
	ret = kstrtoint(val, 0, &d);
	if (ret < 0)
		return ret;
	ibcn = csr_read32(LOONGARCH_CSR_FWPC) & 0x3f;
	if (d > ibcn)
		return -EINVAL;
	boot_cpu_data.watch_ireg_count = ibcn - d;
	*(int *)kp->arg = d;
	return 0;
}

const struct kernel_param_ops param_ops_dcount = {
	.set = param_set_dcount,
	.get = param_get_int,
};

const struct kernel_param_ops param_ops_icount = {
	.set = param_set_icount,
	.get = param_get_int,
};

module_param_cb(kgdb_watch_dcount, &param_ops_dcount, &kgdb_watch_dcount, 0644);
module_param_cb(kgdb_watch_icount, &param_ops_icount, &kgdb_watch_icount, 0644);

static struct hard_trap_info {
	unsigned char tt;	/* Trap type code for LoongArch */
	unsigned char signo;	/* Signal that we map this trap into */
} hard_trap_info[] = {
	{ 1, SIGBUS },
	{ 2, SIGBUS },
	{ 3, SIGBUS },
	{ 4, SIGBUS },
	{ 5, SIGBUS },
	{ 6, SIGBUS },
	{ 7, SIGBUS },
	{ 8, SIGBUS },
	{ 9, SIGBUS },
	{ 10, SIGBUS },
	{ 12, SIGTRAP },		/* break */
	{ 13, SIGBUS },
	{ 14, SIGBUS },
	{ 15, SIGFPE },
	{ 16, SIGFPE },
	{ 17, SIGFPE },
	{ 18, SIGFPE },
	{ 0, 0}			/* Must be last */
};

struct dbg_reg_def_t dbg_reg_def[DBG_ALL_REG_NUM] = {
	{ "r0", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[0]) },
	{ "r1", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[1]) },
	{ "r2", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[2]) },
	{ "r3", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[3]) },
	{ "r4", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[4]) },
	{ "r5", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[5]) },
	{ "r6", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[6]) },
	{ "r7", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[7]) },
	{ "r8", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[8]) },
	{ "r9", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[9]) },
	{ "r10", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[10]) },
	{ "r11", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[11]) },
	{ "r12", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[12]) },
	{ "r13", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[13]) },
	{ "r14", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[14]) },
	{ "r15", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[15]) },
	{ "r16", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[16]) },
	{ "r17", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[17]) },
	{ "r18", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[18]) },
	{ "r19", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[19]) },
	{ "r20", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[20]) },
	{ "r21", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[21]) },
	{ "r22", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[22]) },
	{ "r23", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[23]) },
	{ "r24", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[24]) },
	{ "r25", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[25]) },
	{ "r26", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[26]) },
	{ "r27", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[27]) },
	{ "r28", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[28]) },
	{ "r29", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[29]) },
	{ "r30", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[30]) },
	{ "r31", GDB_SIZEOF_REG, offsetof(struct pt_regs, regs[31]) },
	{ "pc", GDB_SIZEOF_REG, offsetof(struct pt_regs, csr_era) },
	{ "badv", GDB_SIZEOF_REG, offsetof(struct pt_regs, csr_badv) },
	{ "f0", GDB_SIZEOF_REG, 0 },
	{ "f1", GDB_SIZEOF_REG, 1 },
	{ "f2", GDB_SIZEOF_REG, 2 },
	{ "f3", GDB_SIZEOF_REG, 3 },
	{ "f4", GDB_SIZEOF_REG, 4 },
	{ "f5", GDB_SIZEOF_REG, 5 },
	{ "f6", GDB_SIZEOF_REG, 6 },
	{ "f7", GDB_SIZEOF_REG, 7 },
	{ "f8", GDB_SIZEOF_REG, 8 },
	{ "f9", GDB_SIZEOF_REG, 9 },
	{ "f10", GDB_SIZEOF_REG, 10 },
	{ "f11", GDB_SIZEOF_REG, 11 },
	{ "f12", GDB_SIZEOF_REG, 12 },
	{ "f13", GDB_SIZEOF_REG, 13 },
	{ "f14", GDB_SIZEOF_REG, 14 },
	{ "f15", GDB_SIZEOF_REG, 15 },
	{ "f16", GDB_SIZEOF_REG, 16 },
	{ "f17", GDB_SIZEOF_REG, 17 },
	{ "f18", GDB_SIZEOF_REG, 18 },
	{ "f19", GDB_SIZEOF_REG, 19 },
	{ "f20", GDB_SIZEOF_REG, 20 },
	{ "f21", GDB_SIZEOF_REG, 21 },
	{ "f22", GDB_SIZEOF_REG, 22 },
	{ "f23", GDB_SIZEOF_REG, 23 },
	{ "f24", GDB_SIZEOF_REG, 24 },
	{ "f25", GDB_SIZEOF_REG, 25 },
	{ "f26", GDB_SIZEOF_REG, 26 },
	{ "f27", GDB_SIZEOF_REG, 27 },
	{ "f28", GDB_SIZEOF_REG, 28 },
	{ "f29", GDB_SIZEOF_REG, 29 },
	{ "f30", GDB_SIZEOF_REG, 30 },
	{ "f31", GDB_SIZEOF_REG, 31 },
	{ "fcc0", 1, 0 },
	{ "fcc1", 1, 1 },
	{ "fcc2", 1, 2 },
	{ "fcc3", 1, 3 },
	{ "fcc4", 1, 4 },
	{ "fcc5", 1, 5 },
	{ "fcc6", 1, 6 },
	{ "fcc7", 1, 7 },
	{ "fcsr", GDB_SIZEOF_REG, 0 },
	{ "scr0", GDB_SIZEOF_REG, THREAD_SCR0 },
	{ "scr1", GDB_SIZEOF_REG, THREAD_SCR1 },
	{ "scr2", GDB_SIZEOF_REG, THREAD_SCR2 },
	{ "scr3", GDB_SIZEOF_REG, THREAD_SCR3 },
};

int dbg_set_reg(int regno, void *mem, struct pt_regs *regs)
{
	int fp_reg;

	if (regno < 0 || regno >= DBG_ALL_REG_NUM)
		return -EINVAL;

	if (dbg_reg_def[regno].offset != -1 && regno < 34) {
		memcpy((void *)regs + dbg_reg_def[regno].offset, mem,
		       dbg_reg_def[regno].size);
	} else if (current && dbg_reg_def[regno].offset != -1 && regno < 79) {
		/* FP registers 34 -> 78 */
		if (!(regs->csr_euen & CSR_EUEN_FPEN))
			return 0;
		if (regno == 74) {
			/* Process the fcsr/fsr (register 74) */
			memcpy((void *)&current->thread.fpu.fcsr, mem,
			       dbg_reg_def[regno].size);
		} else if (regno >= 66 && regno < 74) {
			/* Process the fcc */
			fp_reg = dbg_reg_def[regno].offset;
			memcpy((char *)&current->thread.fpu.fcc + fp_reg, mem,
			       dbg_reg_def[regno].size);
		} else if (regno >= 75 && regno < 78) {
			/* Process the scr */
			memcpy((void *)&current->thread.lbt + dbg_reg_def[regno].offset, mem,
			       dbg_reg_def[regno].size);
		} else {
		fp_reg = dbg_reg_def[regno].offset;
		memcpy((void *)&current->thread.fpu.fpr[fp_reg], mem,
		       dbg_reg_def[regno].size);
		}

		restore_fp(current);
	}

	return 0;
}

char *dbg_get_reg(int regno, void *mem, struct pt_regs *regs)
{
	int fp_reg;

	if (regno >= DBG_ALL_REG_NUM || regno < 0)
		return NULL;

	if (dbg_reg_def[regno].offset != -1 && regno < DBG_MAX_REG_NUM) {
		/* First 32 registers, epc, badv */
		memcpy(mem, (void *)regs + dbg_reg_def[regno].offset,
		       dbg_reg_def[regno].size);
	} else if (current && dbg_reg_def[regno].offset != -1 && regno < DBG_ALL_REG_NUM) {
		/* FP registers 34 -> 78 */
		if (!(regs->csr_euen & CSR_EUEN_FPEN))
			goto out;
		save_fp(current);
		if (regno == DBG_FP_FSR_REGNO) {
			/* Process the fcsr/fsr (register 74) */
			memcpy(mem, (void *)&current->thread.fpu.fcsr,
			       dbg_reg_def[regno].size);
		} else if (regno >= DBG_FP_FCC0_REGNO && regno < DBG_FP_FSR_REGNO) {
			/* Process the fcc */
			fp_reg = dbg_reg_def[regno].offset;
			memcpy(mem, (char *)&current->thread.fpu.fcc + fp_reg,
			       dbg_reg_def[regno].size);
		} else if (regno >= DBG_FP_SCR0_REGNO && regno < DBG_ALL_REG_NUM) {
			/* Process the scr */
			memcpy(mem, (void *)&current->thread + dbg_reg_def[regno].offset,
			       dbg_reg_def[regno].size);
		} else {
		fp_reg = dbg_reg_def[regno].offset;
		memcpy(mem, (void *)&current->thread.fpu.fpr[fp_reg],
		       dbg_reg_def[regno].size);
		}
	}

out:
	return dbg_reg_def[regno].name;

}

void arch_kgdb_breakpoint(void)
{
	__asm__ __volatile__(
		".globl breakinst\n\t"
		"nop\n"
		"breakinst:\tbreak 0\n\t");

	annotate_reachable();
}

static void kgdb_call_nmi_hook(void *ignored)
{
	kgdb_nmicallback(raw_smp_processor_id(), get_irq_regs());
}

void kgdb_roundup_cpus(unsigned long flags)
{
	local_irq_enable();
	smp_call_function(kgdb_call_nmi_hook, NULL, 0);
	local_irq_disable();
}

static int compute_signal(int tt)
{
	struct hard_trap_info *ht;

	for (ht = hard_trap_info; ht->tt && ht->signo; ht++)
		if (ht->tt == tt)
			return ht->signo;

	return SIGTRAP;		/* default for things we don't know about */
}

/*
 * Similar to regs_to_gdb_regs() except that process is sleeping and so
 * we may not be able to get all the info.
 */
void sleeping_thread_to_gdb_regs(unsigned long *gdb_regs, struct task_struct *p)
{
	int reg;
#if (KGDB_GDB_REG_SIZE == 32)
	u32 *ptr = (u32 *)gdb_regs, *gdbregs = ptr;
#else
	u64 *ptr = (u64 *)gdb_regs, *gdbregs = ptr;
#endif

	*(ptr++) = 0;
	*(ptr++) = p->thread.reg01;
	*(ptr++) = (long)p;
	*(ptr++) = p->thread.reg03;
	for (reg = 4; reg < 23; reg++)
		*(ptr++) = 0;

	/* S0 - S8 */
	*(ptr++) = p->thread.reg23;
	*(ptr++) = p->thread.reg24;
	*(ptr++) = p->thread.reg25;
	*(ptr++) = p->thread.reg26;
	*(ptr++) = p->thread.reg27;
	*(ptr++) = p->thread.reg28;
	*(ptr++) = p->thread.reg29;
	*(ptr++) = p->thread.reg30;
	*(ptr++) = p->thread.reg31;

	/*
	 * PC
	 * use return address (RA), i.e. the moment after return from resume()
	 */
	*(ptr++) = p->thread.reg01;

	ptr = gdbregs + 73;
	*(ptr++) = p->thread.lbt.scr0;
	*(ptr++) = p->thread.lbt.scr1;
	*(ptr++) = p->thread.lbt.scr2;
	*(ptr++) = p->thread.lbt.scr3;
}

void kgdb_arch_set_pc(struct pt_regs *regs, unsigned long pc)
{
	regs->csr_era = pc;
}

/*
 * Calls linux_debug_hook before the kernel dies. If KGDB is enabled,
 * then try to fall into the debugger
 */
static int kgdb_loongarch_notify(struct notifier_block *self, unsigned long cmd,
			    void *ptr)
{
	struct die_args *args = (struct die_args *)ptr;
	struct pt_regs *regs = args->regs;
	int trap = read_csr_excode();

#ifdef CONFIG_KPROBES
	/*
	 * Return immediately if the kprobes fault notifier has set
	 * DIE_PAGE_FAULT.
	 */
	if (cmd == DIE_PAGE_FAULT)
		return NOTIFY_DONE;
#endif /* CONFIG_KPROBES */

	/* Userspace events, ignore. */
	if (user_mode(regs))
		return NOTIFY_DONE;

	if (atomic_read(&kgdb_active) != -1)
		kgdb_nmicallback(smp_processor_id(), regs);

	if (kgdb_handle_exception(trap, compute_signal(trap), cmd, regs))
		return NOTIFY_DONE;

	if (atomic_read(&kgdb_setting_breakpoint))
		if ((regs->csr_era == (unsigned long)breakinst))
			regs->csr_era += 4;

	/* In SMP mode, __flush_cache_all does IPI */
	local_irq_enable();
	flush_cache_all();

	return NOTIFY_STOP;
}

#ifdef CONFIG_KGDB_LOW_LEVEL_TRAP
int kgdb_ll_trap(int cmd, const char *str,
		 struct pt_regs *regs, long err, int trap, int sig)
{
	struct die_args args = {
		.regs	= regs,
		.str	= str,
		.err	= err,
		.trapnr = trap,
		.signr	= sig,

	};

	if (!kgdb_io_module_registered)
		return NOTIFY_DONE;

	return kgdb_loongarch_notify(NULL, cmd, &args);
}
#endif /* CONFIG_KGDB_LOW_LEVEL_TRAP */

static struct notifier_block kgdb_notifier = {
	.notifier_call = kgdb_loongarch_notify,
};

/*
 * Handle the 'c' command
 */
int kgdb_arch_handle_exception(int vector, int signo, int err_code,
			       char *remcom_in_buffer, char *remcom_out_buffer,
			       struct pt_regs *regs)
{
	char *ptr;
	unsigned long address;

	regs->csr_prmd |= CSR_PRMD_PWE;

	switch (remcom_in_buffer[0]) {
	case 'c':
		/* handle the optional parameter */
		ptr = &remcom_in_buffer[1];
		if (kgdb_hex2long(&ptr, &address))
			regs->csr_era = address;

		return 0;
	}

	return -1;
}

static struct hw_breakpoint {
	unsigned		enabled;
	unsigned long		addr;
	int			len;
	int			type;
	struct perf_event	* __percpu *pev;
} dbreakinfo[NUM_WATCH_REGS], ibreakinfo[NUM_WATCH_REGS];

static int
kgdb_set_hw_break(unsigned long addr, int len, enum kgdb_bptype bptype)
{
	int i;
	struct hw_breakpoint *breakinfo = (bptype == BP_HARDWARE_BREAKPOINT) ?
	ibreakinfo : dbreakinfo;
	int count = (bptype == BP_HARDWARE_BREAKPOINT) ? kgdb_watch_icount :
	kgdb_watch_dcount;

	for (i = 0; i < count; i++)
		if (!breakinfo[i].enabled)
			break;
	if (i == count)
		return -1;

	breakinfo[i].type = bptype;
	breakinfo[i].len = len;
	breakinfo[i].addr = addr;
	breakinfo[i].enabled |= 1;

	return 0;
}


static int
kgdb_remove_hw_break(unsigned long addr, int len, enum kgdb_bptype bptype)
{
	int i;
	struct hw_breakpoint *breakinfo = (bptype == BP_HARDWARE_BREAKPOINT) ?
	ibreakinfo : dbreakinfo;
	int count = (bptype == BP_HARDWARE_BREAKPOINT) ? kgdb_watch_icount :
	kgdb_watch_dcount;

	for (i = 0; i < count; i++)
		if (breakinfo[i].addr == addr && breakinfo[i].enabled)
			break;
	if (i == count)
		return -1;

	breakinfo[i].enabled &= ~1;

	return 0;
}

static void kgdb_disable_hw_debug(struct pt_regs *regs)
{
	csr_xchg32(0, CSR_CRMD_WE, LOONGARCH_CSR_CRMD);
	regs->csr_prmd &= ~CSR_PRMD_PWE;
}

static void kgdb_remove_all_hw_break(void)
{
	int i, j, mask;

	for (mask = 0, i = 0, j = boot_cpu_data.watch_ireg_count;
	     i < kgdb_watch_icount; i++, j++) {
		if (!(ibreakinfo[i].enabled & 2))
			continue;
		ibreakinfo[i].enabled = 0;
		watch_csrwr(0, LOONGARCH_CSR_IB0ADDR + 8 * j);
		watch_csrwr(0, LOONGARCH_CSR_IB0MASK + 8 * j);
		watch_csrwr(0, LOONGARCH_CSR_IB0ASID + 8 * j);
		watch_csrwr(0, LOONGARCH_CSR_IB0CTL + 8 * j);
		mask |= 1 << j;
	}
	watch_csrwr(mask, LOONGARCH_CSR_FWPS);

	for (mask = 0, i = 0, j = boot_cpu_data.watch_dreg_count; i < kgdb_watch_dcount;
	     i++, j++) {
		if (!(dbreakinfo[i].enabled & 2))
			continue;
		dbreakinfo[i].enabled = 0;
		watch_csrwr(0, LOONGARCH_CSR_DB0ADDR + 8 * j);
		watch_csrwr(0, LOONGARCH_CSR_DB0MASK + 8 * j);
		watch_csrwr(0, LOONGARCH_CSR_DB0ASID + 8 * j);
		watch_csrwr(0, LOONGARCH_CSR_DB0CTL + 8 * j);
		mask |= 1 << j;
	}
	watch_csrwr(mask, LOONGARCH_CSR_MWPS);

	csr_xchg32(0, CSR_CRMD_WE, LOONGARCH_CSR_CRMD);

	kgdb_watch_activated = 0;
}

static void kgdb_correct_hw_break(void)
{
	int i, j, dbc, activated = 0;

	for (i = 0, j = boot_cpu_data.watch_ireg_count; i < kgdb_watch_icount; i++, j++) {
		if ((ibreakinfo[i].enabled & 3) == 2) {
			watch_csrwr(0, LOONGARCH_CSR_IB0CTL + 8*j);
			ibreakinfo[i].enabled = 0;
			continue;
		} else if (!ibreakinfo[i].enabled)
			continue;
		ibreakinfo[i].enabled |= 2;
		watch_csrwr(ibreakinfo[i].addr, LOONGARCH_CSR_IB0ADDR + 8*j);
		watch_csrwr(0, LOONGARCH_CSR_IB0MASK + 8*j);
		watch_csrwr(0, LOONGARCH_CSR_IB0ASID + 8*j);
		watch_csrwr(0x1e, LOONGARCH_CSR_IB0CTL + 8*j);
		watch_csrwr(0x10000, LOONGARCH_CSR_FWPS);
		activated = 1;
	}

	for (i = 0, j = boot_cpu_data.watch_dreg_count; i < kgdb_watch_dcount; i++, j++) {
		if ((dbreakinfo[i].enabled & 3) == 2) {
			watch_csrwr(0, LOONGARCH_CSR_DB0CTL + 8*j);
			dbreakinfo[i].enabled = 0;
			continue;
		} else if (!dbreakinfo[i].enabled)
			continue;
		dbreakinfo[i].enabled |= 2;
		dbc = 0x1e;
		switch (dbreakinfo[i].len) {
		case 8:
			break;
		case 4:
			dbc |= (1<<10);
			break;
		case 2:
			dbc |= (2<<10);
			break;
		case 1:
			dbc |= (3<<10);
			break;
		default:
			break;
		}

		if (dbreakinfo[i].type == BP_WRITE_WATCHPOINT) {
			dbc |= 1<<9;
		} else if (BP_READ_WATCHPOINT) {
			dbc |= 1<<8;
		} else {
			dbc |= 3<<8;
		}

		watch_csrwr(dbreakinfo[i].addr, LOONGARCH_CSR_DB0ADDR + 8*j);
		watch_csrwr(0, LOONGARCH_CSR_DB0MASK + 8*j);
		watch_csrwr(0, LOONGARCH_CSR_DB0ASID + 8*j);
		watch_csrwr(dbc, LOONGARCH_CSR_DB0CTL + 8*j);
		activated = 1;
	}

	csr_xchg32(activated ? CSR_CRMD_WE : 0, CSR_CRMD_WE, LOONGARCH_CSR_CRMD);
	kgdb_watch_activated = activated;
}

struct kgdb_arch arch_kgdb_ops = {
	.flags			= KGDB_HW_BREAKPOINT,
	.set_hw_breakpoint	= kgdb_set_hw_break,
	.remove_hw_breakpoint	= kgdb_remove_hw_break,
	.disable_hw_break	= kgdb_disable_hw_debug,
	.remove_all_hw_break	= kgdb_remove_all_hw_break,
	.correct_hw_break	= kgdb_correct_hw_break,
};

int kgdb_arch_init(void)
{
	int ibcn, dbcn;
	union loongarch_instruction insn = {
/*0x002a0000 break 0*/
	.reg0i15_format = {
		.opcode = break_op,
		.simmediate = 0,
		}
	};
	memcpy(arch_kgdb_ops.gdb_bpt_instr, insn.byte, BREAK_INSTR_SIZE);

	register_die_notifier(&kgdb_notifier);
	dbcn = csr_read32(LOONGARCH_CSR_MWPC) & 0x3f;
	ibcn = csr_read32(LOONGARCH_CSR_FWPC) & 0x3f;
	boot_cpu_data.watch_dreg_count = dbcn - kgdb_watch_dcount;
	boot_cpu_data.watch_ireg_count = ibcn - kgdb_watch_icount;
	return 0;
}

/*
 *	kgdb_arch_exit - Perform any architecture specific uninitalization.
 *
 *	This function will handle the uninitalization of any architecture
 *	specific callbacks, for dynamic registration and unregistration.
 */
void kgdb_arch_exit(void)
{
	unregister_die_notifier(&kgdb_notifier);
}
