/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 *
 * Author: Chong Qiao <qiaochong@loongson.cn>
 */

#include <linux/sched.h>

#include <asm/processor.h>
#include <asm/watch.h>
#include <asm/ptrace.h>
#include <linux/sched/task_stack.h>

unsigned long watch_csrrd(unsigned int reg)
{
	switch (reg) {
	case LOONGARCH_CSR_IB0ADDR:
		return csr_read64(LOONGARCH_CSR_IB0ADDR);
	case LOONGARCH_CSR_IB1ADDR:
		return csr_read64(LOONGARCH_CSR_IB1ADDR);
	case LOONGARCH_CSR_IB2ADDR:
		return csr_read64(LOONGARCH_CSR_IB2ADDR);
	case LOONGARCH_CSR_IB3ADDR:
		return csr_read64(LOONGARCH_CSR_IB3ADDR);
	case LOONGARCH_CSR_IB4ADDR:
		return csr_read64(LOONGARCH_CSR_IB4ADDR);
	case LOONGARCH_CSR_IB5ADDR:
		return csr_read64(LOONGARCH_CSR_IB5ADDR);
	case LOONGARCH_CSR_IB6ADDR:
		return csr_read64(LOONGARCH_CSR_IB6ADDR);
	case LOONGARCH_CSR_IB7ADDR:
		return csr_read64(LOONGARCH_CSR_IB7ADDR);

	case LOONGARCH_CSR_IB0MASK:
		return csr_read64(LOONGARCH_CSR_IB0MASK);
	case LOONGARCH_CSR_IB1MASK:
		return csr_read64(LOONGARCH_CSR_IB1MASK);
	case LOONGARCH_CSR_IB2MASK:
		return csr_read64(LOONGARCH_CSR_IB2MASK);
	case LOONGARCH_CSR_IB3MASK:
		return csr_read64(LOONGARCH_CSR_IB3MASK);
	case LOONGARCH_CSR_IB4MASK:
		return csr_read64(LOONGARCH_CSR_IB4MASK);
	case LOONGARCH_CSR_IB5MASK:
		return csr_read64(LOONGARCH_CSR_IB5MASK);
	case LOONGARCH_CSR_IB6MASK:
		return csr_read64(LOONGARCH_CSR_IB6MASK);
	case LOONGARCH_CSR_IB7MASK:
		return csr_read64(LOONGARCH_CSR_IB7MASK);

	case LOONGARCH_CSR_IB0ASID:
		return csr_read64(LOONGARCH_CSR_IB0ASID);
	case LOONGARCH_CSR_IB1ASID:
		return csr_read64(LOONGARCH_CSR_IB1ASID);
	case LOONGARCH_CSR_IB2ASID:
		return csr_read64(LOONGARCH_CSR_IB2ASID);
	case LOONGARCH_CSR_IB3ASID:
		return csr_read64(LOONGARCH_CSR_IB3ASID);
	case LOONGARCH_CSR_IB4ASID:
		return csr_read64(LOONGARCH_CSR_IB4ASID);
	case LOONGARCH_CSR_IB5ASID:
		return csr_read64(LOONGARCH_CSR_IB5ASID);
	case LOONGARCH_CSR_IB6ASID:
		return csr_read64(LOONGARCH_CSR_IB6ASID);
	case LOONGARCH_CSR_IB7ASID:
		return csr_read64(LOONGARCH_CSR_IB7ASID);

	case LOONGARCH_CSR_IB0CTL:
		return csr_read64(LOONGARCH_CSR_IB0CTL);
	case LOONGARCH_CSR_IB1CTL:
		return csr_read64(LOONGARCH_CSR_IB1CTL);
	case LOONGARCH_CSR_IB2CTL:
		return csr_read64(LOONGARCH_CSR_IB2CTL);
	case LOONGARCH_CSR_IB3CTL:
		return csr_read64(LOONGARCH_CSR_IB3CTL);
	case LOONGARCH_CSR_IB4CTL:
		return csr_read64(LOONGARCH_CSR_IB4CTL);
	case LOONGARCH_CSR_IB5CTL:
		return csr_read64(LOONGARCH_CSR_IB5CTL);
	case LOONGARCH_CSR_IB6CTL:
		return csr_read64(LOONGARCH_CSR_IB6CTL);
	case LOONGARCH_CSR_IB7CTL:
		return csr_read64(LOONGARCH_CSR_IB7CTL);

	case LOONGARCH_CSR_DB0ADDR:
		return csr_read64(LOONGARCH_CSR_DB0ADDR);
	case LOONGARCH_CSR_DB1ADDR:
		return csr_read64(LOONGARCH_CSR_DB1ADDR);
	case LOONGARCH_CSR_DB2ADDR:
		return csr_read64(LOONGARCH_CSR_DB2ADDR);
	case LOONGARCH_CSR_DB3ADDR:
		return csr_read64(LOONGARCH_CSR_DB3ADDR);
	case LOONGARCH_CSR_DB4ADDR:
		return csr_read64(LOONGARCH_CSR_DB4ADDR);
	case LOONGARCH_CSR_DB5ADDR:
		return csr_read64(LOONGARCH_CSR_DB5ADDR);
	case LOONGARCH_CSR_DB6ADDR:
		return csr_read64(LOONGARCH_CSR_DB6ADDR);
	case LOONGARCH_CSR_DB7ADDR:
		return csr_read64(LOONGARCH_CSR_DB7ADDR);

	case LOONGARCH_CSR_DB0MASK:
		return csr_read64(LOONGARCH_CSR_DB0MASK);
	case LOONGARCH_CSR_DB1MASK:
		return csr_read64(LOONGARCH_CSR_DB1MASK);
	case LOONGARCH_CSR_DB2MASK:
		return csr_read64(LOONGARCH_CSR_DB2MASK);
	case LOONGARCH_CSR_DB3MASK:
		return csr_read64(LOONGARCH_CSR_DB3MASK);
	case LOONGARCH_CSR_DB4MASK:
		return csr_read64(LOONGARCH_CSR_DB4MASK);
	case LOONGARCH_CSR_DB5MASK:
		return csr_read64(LOONGARCH_CSR_DB5MASK);
	case LOONGARCH_CSR_DB6MASK:
		return csr_read64(LOONGARCH_CSR_DB6MASK);
	case LOONGARCH_CSR_DB7MASK:
		return csr_read64(LOONGARCH_CSR_DB7MASK);

	case LOONGARCH_CSR_DB0ASID:
		return csr_read64(LOONGARCH_CSR_DB0ASID);
	case LOONGARCH_CSR_DB1ASID:
		return csr_read64(LOONGARCH_CSR_DB1ASID);
	case LOONGARCH_CSR_DB2ASID:
		return csr_read64(LOONGARCH_CSR_DB2ASID);
	case LOONGARCH_CSR_DB3ASID:
		return csr_read64(LOONGARCH_CSR_DB3ASID);
	case LOONGARCH_CSR_DB4ASID:
		return csr_read64(LOONGARCH_CSR_DB4ASID);
	case LOONGARCH_CSR_DB5ASID:
		return csr_read64(LOONGARCH_CSR_DB5ASID);
	case LOONGARCH_CSR_DB6ASID:
		return csr_read64(LOONGARCH_CSR_DB6ASID);
	case LOONGARCH_CSR_DB7ASID:
		return csr_read64(LOONGARCH_CSR_DB7ASID);

	case LOONGARCH_CSR_DB0CTL:
		return csr_read64(LOONGARCH_CSR_DB0CTL);
	case LOONGARCH_CSR_DB1CTL:
		return csr_read64(LOONGARCH_CSR_DB1CTL);
	case LOONGARCH_CSR_DB2CTL:
		return csr_read64(LOONGARCH_CSR_DB2CTL);
	case LOONGARCH_CSR_DB3CTL:
		return csr_read64(LOONGARCH_CSR_DB3CTL);
	case LOONGARCH_CSR_DB4CTL:
		return csr_read64(LOONGARCH_CSR_DB4CTL);
	case LOONGARCH_CSR_DB5CTL:
		return csr_read64(LOONGARCH_CSR_DB5CTL);
	case LOONGARCH_CSR_DB6CTL:
		return csr_read64(LOONGARCH_CSR_DB6CTL);
	case LOONGARCH_CSR_DB7CTL:
		return csr_read64(LOONGARCH_CSR_DB7CTL);

	case LOONGARCH_CSR_FWPS:
		return csr_read64(LOONGARCH_CSR_FWPS);
	case LOONGARCH_CSR_MWPS:
		return csr_read64(LOONGARCH_CSR_MWPS);
	case LOONGARCH_CSR_FWPC:
		return csr_read64(LOONGARCH_CSR_FWPC);
	case LOONGARCH_CSR_MWPC:
		return csr_read64(LOONGARCH_CSR_MWPC);
	default:
		printk("read watch register number error %d\n", reg);
	}
	return 0;
}
EXPORT_SYMBOL(watch_csrrd);

void watch_csrwr(unsigned long val, unsigned int reg)
{
	switch (reg) {
	case LOONGARCH_CSR_IB0ADDR:
		csr_write64(val, LOONGARCH_CSR_IB0ADDR);
		break;
	case LOONGARCH_CSR_IB1ADDR:
		csr_write64(val, LOONGARCH_CSR_IB1ADDR);
		break;
	case LOONGARCH_CSR_IB2ADDR:
		csr_write64(val, LOONGARCH_CSR_IB2ADDR);
		break;
	case LOONGARCH_CSR_IB3ADDR:
		csr_write64(val, LOONGARCH_CSR_IB3ADDR);
		break;
	case LOONGARCH_CSR_IB4ADDR:
		csr_write64(val, LOONGARCH_CSR_IB4ADDR);
		break;
	case LOONGARCH_CSR_IB5ADDR:
		csr_write64(val, LOONGARCH_CSR_IB5ADDR);
		break;
	case LOONGARCH_CSR_IB6ADDR:
		csr_write64(val, LOONGARCH_CSR_IB6ADDR);
		break;
	case LOONGARCH_CSR_IB7ADDR:
		csr_write64(val, LOONGARCH_CSR_IB7ADDR);
		break;

	case LOONGARCH_CSR_IB0MASK:
		csr_write64(val, LOONGARCH_CSR_IB0MASK);
		break;
	case LOONGARCH_CSR_IB1MASK:
		csr_write64(val, LOONGARCH_CSR_IB1MASK);
		break;
	case LOONGARCH_CSR_IB2MASK:
		csr_write64(val, LOONGARCH_CSR_IB2MASK);
		break;
	case LOONGARCH_CSR_IB3MASK:
		csr_write64(val, LOONGARCH_CSR_IB3MASK);
		break;
	case LOONGARCH_CSR_IB4MASK:
		csr_write64(val, LOONGARCH_CSR_IB4MASK);
		break;
	case LOONGARCH_CSR_IB5MASK:
		csr_write64(val, LOONGARCH_CSR_IB5MASK);
		break;
	case LOONGARCH_CSR_IB6MASK:
		csr_write64(val, LOONGARCH_CSR_IB6MASK);
		break;
	case LOONGARCH_CSR_IB7MASK:
		csr_write64(val, LOONGARCH_CSR_IB7MASK);
		break;

	case LOONGARCH_CSR_IB0ASID:
		csr_write64(val, LOONGARCH_CSR_IB0ASID);
		break;
	case LOONGARCH_CSR_IB1ASID:
		csr_write64(val, LOONGARCH_CSR_IB1ASID);
		break;
	case LOONGARCH_CSR_IB2ASID:
		csr_write64(val, LOONGARCH_CSR_IB2ASID);
		break;
	case LOONGARCH_CSR_IB3ASID:
		csr_write64(val, LOONGARCH_CSR_IB3ASID);
		break;
	case LOONGARCH_CSR_IB4ASID:
		csr_write64(val, LOONGARCH_CSR_IB4ASID);
		break;
	case LOONGARCH_CSR_IB5ASID:
		csr_write64(val, LOONGARCH_CSR_IB5ASID);
		break;
	case LOONGARCH_CSR_IB6ASID:
		csr_write64(val, LOONGARCH_CSR_IB6ASID);
		break;
	case LOONGARCH_CSR_IB7ASID:
		csr_write64(val, LOONGARCH_CSR_IB7ASID);
		break;

	case LOONGARCH_CSR_IB0CTL:
		csr_write64(val, LOONGARCH_CSR_IB0CTL);
		break;
	case LOONGARCH_CSR_IB1CTL:
		csr_write64(val, LOONGARCH_CSR_IB1CTL);
		break;
	case LOONGARCH_CSR_IB2CTL:
		csr_write64(val, LOONGARCH_CSR_IB2CTL);
		break;
	case LOONGARCH_CSR_IB3CTL:
		csr_write64(val, LOONGARCH_CSR_IB3CTL);
		break;
	case LOONGARCH_CSR_IB4CTL:
		csr_write64(val, LOONGARCH_CSR_IB4CTL);
		break;
	case LOONGARCH_CSR_IB5CTL:
		csr_write64(val, LOONGARCH_CSR_IB5CTL);
		break;
	case LOONGARCH_CSR_IB6CTL:
		csr_write64(val, LOONGARCH_CSR_IB6CTL);
		break;
	case LOONGARCH_CSR_IB7CTL:
		csr_write64(val, LOONGARCH_CSR_IB7CTL);
		break;

	case LOONGARCH_CSR_DB0ADDR:
		csr_write64(val, LOONGARCH_CSR_DB0ADDR);
		break;
	case LOONGARCH_CSR_DB1ADDR:
		csr_write64(val, LOONGARCH_CSR_DB1ADDR);
		break;
	case LOONGARCH_CSR_DB2ADDR:
		csr_write64(val, LOONGARCH_CSR_DB2ADDR);
		break;
	case LOONGARCH_CSR_DB3ADDR:
		csr_write64(val, LOONGARCH_CSR_DB3ADDR);
		break;
	case LOONGARCH_CSR_DB4ADDR:
		csr_write64(val, LOONGARCH_CSR_DB4ADDR);
		break;
	case LOONGARCH_CSR_DB5ADDR:
		csr_write64(val, LOONGARCH_CSR_DB5ADDR);
		break;
	case LOONGARCH_CSR_DB6ADDR:
		csr_write64(val, LOONGARCH_CSR_DB6ADDR);
		break;
	case LOONGARCH_CSR_DB7ADDR:
		csr_write64(val, LOONGARCH_CSR_DB7ADDR);
		break;

	case LOONGARCH_CSR_DB0MASK:
		csr_write64(val, LOONGARCH_CSR_DB0MASK);
		break;
	case LOONGARCH_CSR_DB1MASK:
		csr_write64(val, LOONGARCH_CSR_DB1MASK);
		break;
	case LOONGARCH_CSR_DB2MASK:
		csr_write64(val, LOONGARCH_CSR_DB2MASK);
		break;
	case LOONGARCH_CSR_DB3MASK:
		csr_write64(val, LOONGARCH_CSR_DB3MASK);
		break;
	case LOONGARCH_CSR_DB4MASK:
		csr_write64(val, LOONGARCH_CSR_DB4MASK);
		break;
	case LOONGARCH_CSR_DB5MASK:
		csr_write64(val, LOONGARCH_CSR_DB5MASK);
		break;
	case LOONGARCH_CSR_DB6MASK:
		csr_write64(val, LOONGARCH_CSR_DB6MASK);
		break;
	case LOONGARCH_CSR_DB7MASK:
		csr_write64(val, LOONGARCH_CSR_DB7MASK);
		break;

	case LOONGARCH_CSR_DB0ASID:
		csr_write64(val, LOONGARCH_CSR_DB0ASID);
		break;
	case LOONGARCH_CSR_DB1ASID:
		csr_write64(val, LOONGARCH_CSR_DB1ASID);
		break;
	case LOONGARCH_CSR_DB2ASID:
		csr_write64(val, LOONGARCH_CSR_DB2ASID);
		break;
	case LOONGARCH_CSR_DB3ASID:
		csr_write64(val, LOONGARCH_CSR_DB3ASID);
		break;
	case LOONGARCH_CSR_DB4ASID:
		csr_write64(val, LOONGARCH_CSR_DB4ASID);
		break;
	case LOONGARCH_CSR_DB5ASID:
		csr_write64(val, LOONGARCH_CSR_DB5ASID);
		break;
	case LOONGARCH_CSR_DB6ASID:
		csr_write64(val, LOONGARCH_CSR_DB6ASID);
		break;
	case LOONGARCH_CSR_DB7ASID:
		csr_write64(val, LOONGARCH_CSR_DB7ASID);
		break;

	case LOONGARCH_CSR_DB0CTL:
		csr_write64(val, LOONGARCH_CSR_DB0CTL);
		break;
	case LOONGARCH_CSR_DB1CTL:
		csr_write64(val, LOONGARCH_CSR_DB1CTL);
		break;
	case LOONGARCH_CSR_DB2CTL:
		csr_write64(val, LOONGARCH_CSR_DB2CTL);
		break;
	case LOONGARCH_CSR_DB3CTL:
		csr_write64(val, LOONGARCH_CSR_DB3CTL);
		break;
	case LOONGARCH_CSR_DB4CTL:
		csr_write64(val, LOONGARCH_CSR_DB4CTL);
		break;
	case LOONGARCH_CSR_DB5CTL:
		csr_write64(val, LOONGARCH_CSR_DB5CTL);
		break;
	case LOONGARCH_CSR_DB6CTL:
		csr_write64(val, LOONGARCH_CSR_DB6CTL);
		break;
	case LOONGARCH_CSR_DB7CTL:
		csr_write64(val, LOONGARCH_CSR_DB7CTL);
		break;

	case LOONGARCH_CSR_FWPS:
		csr_write64(val, LOONGARCH_CSR_FWPS);
		break;
	case LOONGARCH_CSR_MWPS:
		csr_write64(val, LOONGARCH_CSR_MWPS);
		break;
	default:
		printk("write watch register number error %d\n", reg);
	}
}
EXPORT_SYMBOL(watch_csrwr);

/*
 * Install the watch registers for the current thread.
 */
void loongarch_install_watch_registers(struct task_struct *t)
{
	int i, j, dbcn;
	struct loongarch3264_watch_reg_state *watches = &t->thread.watch.loongarch3264;

	dbcn = boot_cpu_data.watch_dreg_count;
	for (i = 0; i < boot_cpu_data.watch_reg_use_cnt; i++) {
		if (watches->irw[i] & (LA_WATCH_R | LA_WATCH_W)) {
			unsigned int dbc = 0x1e;

			if ((watches->irw[i] & LA_WATCH_R))
				dbc |= 1<<8;
			if ((watches->irw[i] & LA_WATCH_W))
				dbc |= 1<<9;

			watch_csrwr(watches->addr[i], LOONGARCH_CSR_DB0ADDR + 8*i);
			watch_csrwr(watches->mask[i], LOONGARCH_CSR_DB0MASK + 8*i);
			watch_csrwr(0, LOONGARCH_CSR_DB0ASID + 8*i);
			watch_csrwr(dbc, LOONGARCH_CSR_DB0CTL + 8*i);

		} else if (watches->irw[i] & LA_WATCH_I) {
			j = i - dbcn;
			watch_csrwr(watches->addr[i], LOONGARCH_CSR_IB0ADDR + 8*j);
			watch_csrwr(watches->mask[i], LOONGARCH_CSR_IB0MASK + 8*j);
			watch_csrwr(0, LOONGARCH_CSR_IB0ASID + 8*j);
			watch_csrwr(0x1e, LOONGARCH_CSR_IB0CTL + 8*j);
			if ((task_pt_regs(t)->csr_era & ~watches->mask[i]) == (watches->addr[i] & ~watches->mask[i]))
				watch_csrwr(0x10000, LOONGARCH_CSR_FWPS);
		} else if (watches->irwmask[i]) {
			if (i < dbcn) {
				watch_csrwr(0x0, LOONGARCH_CSR_DB0CTL + 8*i);
			} else {
				j = i - dbcn;
				watch_csrwr(0x0, LOONGARCH_CSR_IB0CTL + 8*j);
			}
		}
	}
}
EXPORT_SYMBOL(loongarch_install_watch_registers);

#define PRMD_WE (1UL<<3) /*prmd watch enable, restore to crmd when eret*/
void loongarch_clear_prev_watch_registers(struct task_struct *prev)
{
	struct pt_regs *regs = task_pt_regs(prev);

	loongarch_clear_watch_registers();
	prev->thread.csr_prmd &= ~PRMD_WE;
	regs->csr_prmd &= ~PRMD_WE;
}

void loongarch_install_next_watch_registers(struct task_struct *next)
{
	struct pt_regs *regs = task_pt_regs(next);

	loongarch_install_watch_registers(next);
	next->thread.csr_prmd |= PRMD_WE;
	regs->csr_prmd |= PRMD_WE;
}

#define CTRL_WE 1 /*breakpoint only can be written by jtag*/
void loongarch_update_watch_registers(struct task_struct *t)
{
	struct loongarch3264_watch_reg_state *watches =
		&t->thread.watch.loongarch3264;
	int i, j, dbcn, ctl;

	dbcn = boot_cpu_data.watch_dreg_count;
	for (i = 0; i < boot_cpu_data.watch_reg_use_cnt; i++) {
		if (i < dbcn) {
			ctl = watch_csrrd(LOONGARCH_CSR_DB0CTL + 8*i);
			if (ctl & CTRL_WE)
				watches->irwmask[i] = 0;
			else
				watches->irwmask[i] = LA_WATCH_R | LA_WATCH_W;
		} else {
			j = i - dbcn;
			ctl = watch_csrrd(LOONGARCH_CSR_IB0CTL + 8*j);
			if (ctl & CTRL_WE)
				watches->irwmask[i] = 0;
			else
				watches->irwmask[i] = LA_WATCH_I;
		}
	}
}
/*
 * Read back the watchhi registers so the user space debugger has
 * access to the I, R, and W bits.
 */
void loongarch_read_watch_registers(struct pt_regs *regs)
{
	struct loongarch3264_watch_reg_state *watches =
		&current->thread.watch.loongarch3264;
	unsigned int i, j, dbcn, ibst, dbst;
	dbcn = boot_cpu_data.watch_dreg_count;
	ibst = watch_csrrd(LOONGARCH_CSR_FWPS);
	dbst = watch_csrrd(LOONGARCH_CSR_MWPS);

	for (i = 0; i < boot_cpu_data.watch_reg_use_cnt; i++) {
		if (i < dbcn) {
			watches->addr[i] = watch_csrrd(LOONGARCH_CSR_DB0ADDR + 8*i);
			watches->mask[i] = watch_csrrd(LOONGARCH_CSR_DB0MASK + 8*i);
			if (dbst & (1u << i)) {
				int dbc = watch_csrrd(LOONGARCH_CSR_DB0CTL + i*8);

				watches->irwstat[i] = 0;
				if (dbc & (1<<8))
					watches->irwstat[i] |= LA_WATCH_R;
				if (dbc & (1<<9))
					watches->irwstat[i] |= LA_WATCH_W;
			}
		} else {
			j = i - dbcn;
			watches->addr[i] = watch_csrrd(LOONGARCH_CSR_IB0ADDR + 8*j);
			watches->mask[i] = watch_csrrd(LOONGARCH_CSR_IB0MASK + 8*j);
			watches->irwstat[i] = 0;
			if (ibst & (1u << j))
				watches->irwstat[i] |= LA_WATCH_I;
		}
	}

	if (ibst & 0xffff)
		watch_csrwr(ibst, LOONGARCH_CSR_FWPS);
}

/*
 * Disable all watch registers.
 */
void loongarch_clear_watch_registers(void)
{
	unsigned int i, j, dbcn, ibst, dbst;

	ibst = watch_csrrd(LOONGARCH_CSR_FWPS);
	dbst = watch_csrrd(LOONGARCH_CSR_MWPS);
	dbcn = boot_cpu_data.watch_dreg_count;

	/*
	 *  bit 16: skip next event
	 *  bit 0-15: breakpoint triggered, W1C
	 */

	if (ibst & 0x1ffff)
		watch_csrwr(ibst&~0x10000, LOONGARCH_CSR_FWPS);
	if (dbst & 0x1ffff)
		watch_csrwr(dbst&~0x10000, LOONGARCH_CSR_MWPS);

	for (i = 0; i < boot_cpu_data.watch_reg_use_cnt; i++) {
		if (i < dbcn) {
			watch_csrwr(0, LOONGARCH_CSR_DB0ADDR + 8*i);
			watch_csrwr(0, LOONGARCH_CSR_DB0MASK + 8*i);
			watch_csrwr(0, LOONGARCH_CSR_DB0ASID + 8*i);
			watch_csrwr(0, LOONGARCH_CSR_DB0CTL + 8*i);
		} else {
			j = i - dbcn;
			watch_csrwr(0, LOONGARCH_CSR_IB0ADDR + 8*j);
			watch_csrwr(0, LOONGARCH_CSR_IB0MASK + 8*j);
			watch_csrwr(0, LOONGARCH_CSR_IB0ASID + 8*j);
			watch_csrwr(0, LOONGARCH_CSR_IB0CTL + 8*j);
		}
	}
}
EXPORT_SYMBOL(loongarch_clear_watch_registers);

void loongarch_probe_watch_registers(struct cpuinfo_loongarch *c)
{
	unsigned int ibcn, dbcn, total;

	ibcn = watch_csrrd(LOONGARCH_CSR_FWPC) & 0x3f;
	dbcn = watch_csrrd(LOONGARCH_CSR_MWPC) & 0x3f;
	c->watch_dreg_count = dbcn;
	c->watch_ireg_count = ibcn;
	total =  ibcn + dbcn;
	c->watch_reg_use_cnt = (total < NUM_WATCH_REGS) ? total : NUM_WATCH_REGS;
	if (total)
		c->options |= LOONGARCH_CPU_WATCH;
}
