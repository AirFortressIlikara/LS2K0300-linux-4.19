// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */
#include <linux/sizes.h>
#include <linux/uaccess.h>

#include <asm/inst.h>
#include <asm/kprobes.h>

#define __SIGNEX(X, SIDX) ((X) >= (1 << SIDX) ? ~((1 << SIDX) - 1) | (X) : (X))
#define SIGNEX26(X) __SIGNEX(((unsigned long)(X)), 25)
#define SIGNEX21(X) __SIGNEX(((unsigned long)(X)), 20)
#define SIGNEX20(X) __SIGNEX(((unsigned long)(X)), 19)
#define SIGNEX16(X) __SIGNEX(((unsigned long)(X)), 15)

unsigned long bs_dest_26(unsigned long now, unsigned int h, unsigned int l)
{
	return now + (SIGNEX26(h << 16 | l) << 2);
}

unsigned long bs_dest_21(unsigned long now, unsigned int h,
					 unsigned int l)
{
	return now + (SIGNEX21(h << 16 | l) << 2);
}

unsigned long bs_dest_16(unsigned long now, unsigned int si)
{
	return now + (SIGNEX16(si) << 2);
}

int simu_branch(struct pt_regs *regs, union loongarch_instruction insn)
{
	unsigned int si, si_l, si_h, rd, rj;
	unsigned long era = regs->csr_era;

	if (era & 3)
		return -EFAULT;

	si_l = insn.reg0i26_format.simmediate_l;
	si_h = insn.reg0i26_format.simmediate_h;
	switch (insn.reg0i26_format.opcode) {
	case b_op:
		regs->csr_era = bs_dest_26(era, si_h, si_l);
		return 0;
	case bl_op:
		regs->csr_era = bs_dest_26(era, si_h, si_l);
		regs->regs[1] = era + LOONGARCH_INSN_SIZE;
		return 0;
	}

	si_l = insn.reg1i21_format.simmediate_l;
	si_h = insn.reg1i21_format.simmediate_h;
	rj = insn.reg1i21_format.rj;
	switch (insn.reg1i21_format.opcode) {
	case beqz_op:
		if (cond_beqz(regs, rj))
			regs->csr_era = bs_dest_21(era, si_h, si_l);
		else
			regs->csr_era += LOONGARCH_INSN_SIZE;
		return 0;
	case bnez_op:
		if (cond_bnez(regs, rj))
			regs->csr_era = bs_dest_21(era, si_h, si_l);
		else
			regs->csr_era += LOONGARCH_INSN_SIZE;
		return 0;
	}

	si = insn.reg2i16_format.simmediate;
	rj = insn.reg2i16_format.rj;
	rd = insn.reg2i16_format.rd;
	switch (insn.reg2i16_format.opcode) {
	case jirl_op:
		/* Use bs_dest_16() to calc the dest. */
		regs->csr_era = bs_dest_16(regs->regs[rj], si);
		regs->regs[rd] = era + LOONGARCH_INSN_SIZE;
		break;
	case beq_op:
		if (cond_beq(regs, rj, rd))
			regs->csr_era = bs_dest_16(era, si);
		else
			regs->csr_era += LOONGARCH_INSN_SIZE;
		break;
	case bne_op:
		if (cond_bne(regs, rj, rd))
			regs->csr_era = bs_dest_16(era, si);
		else
			regs->csr_era += LOONGARCH_INSN_SIZE;
		break;
	case blt_op:
		if (cond_blt(regs, rj, rd))
			regs->csr_era = bs_dest_16(era, si);
		else
			regs->csr_era += LOONGARCH_INSN_SIZE;
		break;
	case bge_op:
		if (cond_bge(regs, rj, rd))
			regs->csr_era = bs_dest_16(era, si);
		else
			regs->csr_era += LOONGARCH_INSN_SIZE;
		break;
	case bltu_op:
		if (cond_bltu(regs, rj, rd))
			regs->csr_era = bs_dest_16(era, si);
		else
			regs->csr_era += LOONGARCH_INSN_SIZE;
		break;
	case bgeu_op:
		if (cond_bgeu(regs, rj, rd))
			regs->csr_era = bs_dest_16(era, si);
		else
			regs->csr_era += LOONGARCH_INSN_SIZE;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int simu_pc(struct pt_regs *regs, union loongarch_instruction insn)
{
	unsigned long era = regs->csr_era;
	unsigned int rd = insn.reg1i20_format.rd;
	unsigned int si = insn.reg1i20_format.simmediate;

	if (era & 3)
		return -EFAULT;

	switch (insn.reg1i20_format.opcode) {
	case pcaddi_op:
		regs->regs[rd] = era + (SIGNEX20(si) << 2);
		break;
	case pcalau12i_op:
		regs->regs[rd] = era + (SIGNEX20(si) << 12);
		regs->regs[rd] &= ~((1 << 12) - 1);
		break;
	case pcaddu12i_op:
		regs->regs[rd] = era + (SIGNEX20(si) << 12);
		break;
	case pcaddu18i_op:
		regs->regs[rd] = era + (SIGNEX20(si) << 18);
		break;
	default:
		return -EINVAL;
	}

	regs->csr_era += LOONGARCH_INSN_SIZE;

	return 0;
}

static DEFINE_RAW_SPINLOCK(patch_lock);

int la64_insn_read(void *addr, u32 *insnp)
{
	int ret;
	u32 val;

	ret = probe_kernel_read(&val, addr, LOONGARCH_INSN_SIZE);
	if (!ret)
		*insnp = val;

	return ret;
}

int la64_insn_write(void *addr, u32 insn)
{
	int ret;
	unsigned long flags = 0;

	raw_spin_lock_irqsave(&patch_lock, flags);
	ret = probe_kernel_write(addr, &insn, LOONGARCH_INSN_SIZE);
	raw_spin_unlock_irqrestore(&patch_lock, flags);

	return ret;
}

int la64_insn_patch_text(void *addr, u32 insn)
{
	int ret;
	u32 *tp = addr;

	if ((unsigned long)tp & 3)
		return -EINVAL;

	ret = la64_insn_write(tp, insn);
	if (!ret)
		flush_icache_range((unsigned long)tp,
				   (unsigned long)tp + LOONGARCH_INSN_SIZE);

	return ret;
}

u32 la64_insn_gen_nop(void)
{
	return INSN_NOP;
}

u32 la64_insn_gen_b(unsigned long pc, unsigned long dest)
{
	unsigned int simmediate_l, simmediate_h;
	union loongarch_instruction insn;
	long offset = dest - pc;

	if ((offset & 3) || offset < -SZ_128M || offset >= SZ_128M) {
		pr_warn("The generated b instruction is out of range.\n");
		return INSN_BREAK;
	}

	offset >>= 2;

	simmediate_l = offset & 0xffff;
	offset >>= 16;
	simmediate_h = offset & 0x3ff;

	insn.reg0i26_format.opcode = b_op;
	insn.reg0i26_format.simmediate_l = simmediate_l;
	insn.reg0i26_format.simmediate_h = simmediate_h;

	return insn.word;
}

u32 la64_insn_gen_bl(unsigned long pc, unsigned long dest)
{
	unsigned int simmediate_l, simmediate_h;
	union loongarch_instruction insn;
	long offset = dest - pc;

	if ((offset & 3) || offset < -SZ_128M || offset >= SZ_128M) {
		pr_warn("The generated bl instruction is out of range.\n");
		return INSN_BREAK;
	}

	offset >>= 2;

	simmediate_l = offset & 0xffff;
	offset >>= 16;
	simmediate_h = offset & 0x3ff;

	insn.reg0i26_format.opcode = bl_op;
	insn.reg0i26_format.simmediate_l = simmediate_l;
	insn.reg0i26_format.simmediate_h = simmediate_h;

	return insn.word;
}

u32 la64_insn_gen_addu16id(enum loongarch_gpr rd, enum loongarch_gpr rj, int imm)
{
	union loongarch_instruction insn;

	insn.reg2i16_format.opcode = addu16id_op;
	insn.reg2i16_format.rd = rd;
	insn.reg2i16_format.rj = rj;
	insn.reg2i16_format.simmediate = imm;

	return insn.word;
}

u32 la64_insn_gen_lu32id(enum loongarch_gpr rd, int imm)
{
	union loongarch_instruction insn;

	insn.reg1i20_format.opcode = lu32id_op;
	insn.reg1i20_format.rd = rd;
	insn.reg1i20_format.simmediate = imm;

	return insn.word;
}

u32 la64_insn_gen_lu52id(enum loongarch_gpr rd, enum loongarch_gpr rj, int imm)
{
	union loongarch_instruction insn;

	insn.reg2i12_format.opcode = lu52id_op;
	insn.reg2i12_format.rd = rd;
	insn.reg2i12_format.rj = rj;
	insn.reg2i12_format.simmediate = imm;

	return insn.word;
}

u32 la64_insn_gen_jirl(enum loongarch_gpr rd, enum loongarch_gpr rj,
			unsigned long pc, unsigned long dest)
{
	union loongarch_instruction insn;
	long offset = dest - pc;

	if ((offset & 3) || offset < -SZ_128K || offset >= SZ_128K) {
		pr_warn("The generated jirl instruction is out of range.\n");
		return INSN_BREAK;
	}

	insn.reg2i16_format.opcode = jirl_op;
	insn.reg2i16_format.rd = rd;
	insn.reg2i16_format.rj = rj;
	insn.reg2i16_format.simmediate = offset >> 2;

	return insn.word;
}

u32 la64_insn_gen_or(enum loongarch_gpr rd, enum loongarch_gpr rj,
			enum loongarch_gpr rk)
{
	union loongarch_instruction insn;

	insn.reg3_format.opcode = or_op;
	insn.reg3_format.rd = rd;
	insn.reg3_format.rj = rj;
	insn.reg3_format.rk = rk;

	return insn.word;
}

u32 la64_insn_gen_move(enum loongarch_gpr rd, enum loongarch_gpr rj)
{
	return la64_insn_gen_or(rd, rj, 0);
}
