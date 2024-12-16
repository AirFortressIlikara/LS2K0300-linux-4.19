/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Format of an instruction in memory.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1996, 2000 by Ralf Baechle
 * Copyright (C) 2006 by Thiemo Seufer
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#ifndef _ASM_INST_H
#define _ASM_INST_H
#include <linux/types.h>

#include <asm/asm.h>
#include <asm/errno.h>
#include <asm/ptrace.h>

#include <uapi/asm/inst.h>

/* HACHACHAHCAHC ...  */

/* In case some other massaging is needed, keep LOONGARCHInst as wrapper */

#define LOONGARCHInst(x) x

#define I_OPCODE_SFT	26
#define LOONGARCHInst_OPCODE(x) (LOONGARCHInst(x) >> I_OPCODE_SFT)

#define I_JTARGET_SFT	0
#define LOONGARCHInst_JTARGET(x) (LOONGARCHInst(x) & 0x03ffffff)

#define I_RS_SFT	21
#define LOONGARCHInst_RS(x) ((LOONGARCHInst(x) & 0x03e00000) >> I_RS_SFT)

#define I_RT_SFT	16
#define LOONGARCHInst_RT(x) ((LOONGARCHInst(x) & 0x001f0000) >> I_RT_SFT)

#define I_IMM_SFT	0
#define LOONGARCHInst_SIMM(x) ((int)((short)(LOONGARCHInst(x) & 0xffff)))
#define LOONGARCHInst_UIMM(x) (LOONGARCHInst(x) & 0xffff)

#define Inst_UncondBranchSIMM(x) \
	((int)((((LOONGARCHInst(x) & 0x3ff) | ((LOONGARCHInst(x) & 0x200) ? 0xfffffc00 : 0)) << 16) \
	| ((LOONGARCHInst(x) & 0x3fffc00) >> 10)))

#define I_CACHEOP_SFT	18
#define LOONGARCHInst_CACHEOP(x) ((LOONGARCHInst(x) & 0x001c0000) >> I_CACHEOP_SFT)

#define I_CACHESEL_SFT	16
#define LOONGARCHInst_CACHESEL(x) ((LOONGARCHInst(x) & 0x00030000) >> I_CACHESEL_SFT)

#define I_RD_SFT	11
#define LOONGARCHInst_RD(x) ((LOONGARCHInst(x) & 0x0000f800) >> I_RD_SFT)

#define I_RE_SFT	6
#define LOONGARCHInst_RE(x) ((LOONGARCHInst(x) & 0x000007c0) >> I_RE_SFT)

#define I_FUNC_SFT	0
#define LOONGARCHInst_FUNC(x) (LOONGARCHInst(x) & 0x0000003f)

#define I_FFMT_SFT	21
#define LOONGARCHInst_FFMT(x) ((LOONGARCHInst(x) & 0x01e00000) >> I_FFMT_SFT)

#define I_FT_SFT	16
#define LOONGARCHInst_FT(x) ((LOONGARCHInst(x) & 0x001f0000) >> I_FT_SFT)

#define I_FS_SFT	11
#define LOONGARCHInst_FS(x) ((LOONGARCHInst(x) & 0x0000f800) >> I_FS_SFT)

#define I_FD_SFT	6
#define LOONGARCHInst_FD(x) ((LOONGARCHInst(x) & 0x000007c0) >> I_FD_SFT)

#define I_FR_SFT	21
#define LOONGARCHInst_FR(x) ((LOONGARCHInst(x) & 0x03e00000) >> I_FR_SFT)

#define I_FMA_FUNC_SFT	2
#define LOONGARCHInst_FMA_FUNC(x) ((LOONGARCHInst(x) & 0x0000003c) >> I_FMA_FUNC_SFT)

#define I_FMA_FFMT_SFT	0
#define LOONGARCHInst_FMA_FFMT(x) (LOONGARCHInst(x) & 0x00000003)

typedef unsigned int loongarch_instruction;

/* Recode table from 16-bit register notation to 32-bit GPR. Do NOT export!!! */
extern const int reg16to32[];

#define LOONGARCH_INSN_SIZE	sizeof(union loongarch_instruction)

enum loongarch_gpr {
	LOONGARCH_GPR_ZERO = 0,
	LOONGARCH_GPR_RA = 1,
	LOONGARCH_GPR_TP = 2,
	LOONGARCH_GPR_SP = 3,
	LOONGARCH_GPR_A0 = 4,
	LOONGARCH_GPR_A1,
	LOONGARCH_GPR_A2,
	LOONGARCH_GPR_A3,
	LOONGARCH_GPR_A4,
	LOONGARCH_GPR_A5,
	LOONGARCH_GPR_A6,
	LOONGARCH_GPR_A7,
	LOONGARCH_GPR_V0 = 4,
	LOONGARCH_GPR_V1 = 5,
	LOONGARCH_GPR_T0 = 12,
	LOONGARCH_GPR_T1,
	LOONGARCH_GPR_T2,
	LOONGARCH_GPR_T3,
	LOONGARCH_GPR_T4,
	LOONGARCH_GPR_T5,
	LOONGARCH_GPR_T6,
	LOONGARCH_GPR_T7,
	LOONGARCH_GPR_T8,
	LOONGARCH_GPR_FP = 22,
	LOONGARCH_GPR_S0 = 23,
	LOONGARCH_GPR_S1,
	LOONGARCH_GPR_S2,
	LOONGARCH_GPR_S3,
	LOONGARCH_GPR_S4,
	LOONGARCH_GPR_S5,
	LOONGARCH_GPR_S6,
	LOONGARCH_GPR_S7,
	LOONGARCH_GPR_S8,
	LOONGARCH_GPR_MAX
};

#define INSN_NOP 0x03400000
#define INSN_BREAK 0x002a0000

#define ADDR_IMMMASK_ADDU16ID	0x00000000FFFF0000
#define ADDR_IMMMASK_LU32ID	0x000FFFFF00000000
#define ADDR_IMMMASK_LU52ID	0xFFF0000000000000

#define ADDR_IMMSHIFT_ADDU16ID	16
#define ADDR_IMMSHIFT_LU32ID	32
#define ADDR_IMMSHIFT_LU52ID	52

#define ADDR_IMM(addr, INSN)	((addr & ADDR_IMMMASK_##INSN) >> ADDR_IMMSHIFT_##INSN)

static inline bool cond_beqz(struct pt_regs *regs, int rj)
{
	return regs->regs[rj] == 0;
}

static inline bool cond_bnez(struct pt_regs *regs, int rj)
{
	return regs->regs[rj] != 0;
}

static inline bool cond_beq(struct pt_regs *regs, int rj, int rd)
{
	return regs->regs[rj] == regs->regs[rd];
}

static inline bool cond_bne(struct pt_regs *regs, int rj, int rd)
{
	return regs->regs[rj] != regs->regs[rd];
}

static inline bool cond_blt(struct pt_regs *regs, int rj, int rd)
{
	return (long)regs->regs[rj] < (long)regs->regs[rd];
}

static inline bool cond_bge(struct pt_regs *regs, int rj, int rd)
{
	return (long)regs->regs[rj] >= (long)regs->regs[rd];
}

static inline bool cond_bltu(struct pt_regs *regs, int rj, int rd)
{
	return regs->regs[rj] < regs->regs[rd];
}

static inline bool cond_bgeu(struct pt_regs *regs, int rj, int rd)
{
	return regs->regs[rj] >= regs->regs[rd];
}

static inline bool is_branch_insn(union loongarch_instruction insn)
{
	return insn.reg1i21_format.opcode >= beqz_op &&
			insn.reg1i21_format.opcode <= bgeu_op;
}

static inline bool is_pc_insn(union loongarch_instruction insn)
{
	return insn.reg1i20_format.opcode >= pcaddi_op &&
			insn.reg1i20_format.opcode <= pcaddu18i_op;
}

unsigned long bs_dest_26(unsigned long now, unsigned int h, unsigned int l);
unsigned long bs_dest_21(unsigned long now, unsigned int h, unsigned int l);
unsigned long bs_dest_16(unsigned long now, unsigned int si);

int simu_branch(struct pt_regs *regs, union loongarch_instruction insn);
int simu_pc(struct pt_regs *regs, union loongarch_instruction insn);

int la64_insn_read(void *addr, u32 *insnp);
int la64_insn_write(void *addr, u32 insn);
int la64_insn_patch_text(void *addr, u32 insn);

u32 la64_insn_gen_nop(void);
u32 la64_insn_gen_b(unsigned long pc, unsigned long dest);
u32 la64_insn_gen_bl(unsigned long pc, unsigned long dest);

u32 la64_insn_gen_addu16id(enum loongarch_gpr rd, enum loongarch_gpr rj, int imm);
u32 la64_insn_gen_lu32id(enum loongarch_gpr rd, int imm);
u32 la64_insn_gen_lu52id(enum loongarch_gpr rd, enum loongarch_gpr rj, int imm);

u32 la64_insn_gen_jirl(enum loongarch_gpr rd, enum loongarch_gpr rj,
			unsigned long pc, unsigned long dest);

u32 la64_insn_gen_or(enum loongarch_gpr rd, enum loongarch_gpr rj,
			enum loongarch_gpr rk);
u32 la64_insn_gen_move(enum loongarch_gpr rd, enum loongarch_gpr rj);

#endif /* _ASM_INST_H */
