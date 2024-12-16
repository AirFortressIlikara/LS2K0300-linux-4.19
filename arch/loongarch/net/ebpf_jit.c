// SPDX-License-Identifier: GPL-2.0-only
/*
 * BPF JIT compiler for LoongArch
 *
 * Copyright (C) 2021 Loongson Technology Corporation Limited
 */
#include "ebpf_jit.h"

#define TMP_REG_1       (MAX_BPF_JIT_REG + 0)
#define TMP_REG_2       (MAX_BPF_JIT_REG + 1)
#define TMP_REG_3       (MAX_BPF_JIT_REG + 2)
#define REG_TCC         (MAX_BPF_JIT_REG + 3)
#define TCC_SAVED       (MAX_BPF_JIT_REG + 4)

#define SAVE_RA         BIT(0)
#define SAVE_TCC        BIT(1)

static const int regmap[] = {
	/* return value from in-kernel function, and exit value for eBPF program */
	[BPF_REG_0] = LOONGARCH_GPR_A5,
	/* arguments from eBPF program to in-kernel function */
	[BPF_REG_1] = LOONGARCH_GPR_A0,
	[BPF_REG_2] = LOONGARCH_GPR_A1,
	[BPF_REG_3] = LOONGARCH_GPR_A2,
	[BPF_REG_4] = LOONGARCH_GPR_A3,
	[BPF_REG_5] = LOONGARCH_GPR_A4,
	/* callee saved registers that in-kernel function will preserve */
	[BPF_REG_6] = LOONGARCH_GPR_S0,
	[BPF_REG_7] = LOONGARCH_GPR_S1,
	[BPF_REG_8] = LOONGARCH_GPR_S2,
	[BPF_REG_9] = LOONGARCH_GPR_S3,
	/* read-only frame pointer to access stack */
	[BPF_REG_FP] = LOONGARCH_GPR_S4,
	/* temporary register for blinding constants */
	[BPF_REG_AX] = LOONGARCH_GPR_T0,
	/* temporary register for internal BPF JIT */
	[TMP_REG_1] = LOONGARCH_GPR_T1,
	[TMP_REG_2] = LOONGARCH_GPR_T2,
	[TMP_REG_3] = LOONGARCH_GPR_T3,
	/* tail call */
	[REG_TCC] = LOONGARCH_GPR_A6,
	/* store A6 in S5 if program do calls */
	[TCC_SAVED] = LOONGARCH_GPR_S5,
};

static void mark_call(struct jit_ctx *ctx)
{
	ctx->flags |= SAVE_RA;
}

static void mark_tail_call(struct jit_ctx *ctx)
{
	ctx->flags |= SAVE_TCC;
}

static bool seen_call(struct jit_ctx *ctx)
{
	return (ctx->flags & SAVE_RA);
}

static bool seen_tail_call(struct jit_ctx *ctx)
{
	return (ctx->flags & SAVE_TCC);
}

static u8 tail_call_reg(struct jit_ctx *ctx)
{
	if (seen_call(ctx))
		return regmap[TCC_SAVED];

	return regmap[REG_TCC];
}

/*
 * eBPF prog stack layout:
 *
 *                                        high
 * original $sp ------------> +-------------------------+ <--LOONGARCH_GPR_FP
 *                            |           $ra           |
 *                            +-------------------------+
 *                            |           $fp           |
 *                            +-------------------------+
 *                            |           $s0           |
 *                            +-------------------------+
 *                            |           $s1           |
 *                            +-------------------------+
 *                            |           $s2           |
 *                            +-------------------------+
 *                            |           $s3           |
 *                            +-------------------------+
 *                            |           $s4           |
 *                            +-------------------------+
 *                            |           $s5           |
 *                            +-------------------------+ <--BPF_REG_FP
 *                            |  prog->aux->stack_depth |
 *                            |        (optional)       |
 * current $sp -------------> +-------------------------+
 *                                        low
 */
static void build_prologue(struct jit_ctx *ctx)
{
	int stack_adjust = 0, store_offset, bpf_stack_adjust;

	bpf_stack_adjust = round_up(ctx->prog->aux->stack_depth, 16);

	stack_adjust += sizeof(long); /* LOONGARCH_GPR_RA */
	stack_adjust += sizeof(long); /* LOONGARCH_GPR_FP */
	stack_adjust += sizeof(long); /* LOONGARCH_GPR_S0 */
	stack_adjust += sizeof(long); /* LOONGARCH_GPR_S1 */
	stack_adjust += sizeof(long); /* LOONGARCH_GPR_S2 */
	stack_adjust += sizeof(long); /* LOONGARCH_GPR_S3 */
	stack_adjust += sizeof(long); /* LOONGARCH_GPR_S4 */
	stack_adjust += sizeof(long); /* LOONGARCH_GPR_S5 */

	stack_adjust = round_up(stack_adjust, 16);
	stack_adjust += bpf_stack_adjust;

	/*
	 * First instruction initializes the tail call count (TCC).
	 * On tail call we skip this instruction, and the TCC is
	 * passed in REG_TCC from the caller.
	 */
	emit_insn(ctx, addid, regmap[REG_TCC], LOONGARCH_GPR_ZERO, MAX_TAIL_CALL_CNT);

	emit_insn(ctx, addid, LOONGARCH_GPR_SP, LOONGARCH_GPR_SP, -stack_adjust);

	store_offset = stack_adjust - sizeof(long);
	emit_insn(ctx, std, LOONGARCH_GPR_RA, LOONGARCH_GPR_SP, store_offset);

	store_offset -= sizeof(long);
	emit_insn(ctx, std, LOONGARCH_GPR_FP, LOONGARCH_GPR_SP, store_offset);

	store_offset -= sizeof(long);
	emit_insn(ctx, std, LOONGARCH_GPR_S0, LOONGARCH_GPR_SP, store_offset);

	store_offset -= sizeof(long);
	emit_insn(ctx, std, LOONGARCH_GPR_S1, LOONGARCH_GPR_SP, store_offset);

	store_offset -= sizeof(long);
	emit_insn(ctx, std, LOONGARCH_GPR_S2, LOONGARCH_GPR_SP, store_offset);

	store_offset -= sizeof(long);
	emit_insn(ctx, std, LOONGARCH_GPR_S3, LOONGARCH_GPR_SP, store_offset);

	store_offset -= sizeof(long);
	emit_insn(ctx, std, LOONGARCH_GPR_S4, LOONGARCH_GPR_SP, store_offset);

	store_offset -= sizeof(long);
	emit_insn(ctx, std, LOONGARCH_GPR_S5, LOONGARCH_GPR_SP, store_offset);

	emit_insn(ctx, addid, LOONGARCH_GPR_FP, LOONGARCH_GPR_SP, stack_adjust);

	if (bpf_stack_adjust)
		emit_insn(ctx, addid, regmap[BPF_REG_FP], LOONGARCH_GPR_SP, bpf_stack_adjust);

	/*
	 * Program contains calls and tail calls, so REG_TCC need
	 * to be saved across calls.
	 */
	if (seen_tail_call(ctx) && seen_call(ctx))
		move_reg(ctx, regmap[TCC_SAVED], regmap[REG_TCC]);

	ctx->stack_size = stack_adjust;
}

static void __build_epilogue(struct jit_ctx *ctx, bool is_tail_call)
{
	int stack_adjust = ctx->stack_size;
	int load_offset;

	load_offset = stack_adjust - sizeof(long);
	emit_insn(ctx, ldd, LOONGARCH_GPR_RA, LOONGARCH_GPR_SP, load_offset);

	load_offset -= sizeof(long);
	emit_insn(ctx, ldd, LOONGARCH_GPR_FP, LOONGARCH_GPR_SP, load_offset);

	load_offset -= sizeof(long);
	emit_insn(ctx, ldd, LOONGARCH_GPR_S0, LOONGARCH_GPR_SP, load_offset);

	load_offset -= sizeof(long);
	emit_insn(ctx, ldd, LOONGARCH_GPR_S1, LOONGARCH_GPR_SP, load_offset);

	load_offset -= sizeof(long);
	emit_insn(ctx, ldd, LOONGARCH_GPR_S2, LOONGARCH_GPR_SP, load_offset);

	load_offset -= sizeof(long);
	emit_insn(ctx, ldd, LOONGARCH_GPR_S3, LOONGARCH_GPR_SP, load_offset);

	load_offset -= sizeof(long);
	emit_insn(ctx, ldd, LOONGARCH_GPR_S4, LOONGARCH_GPR_SP, load_offset);

	load_offset -= sizeof(long);
	emit_insn(ctx, ldd, LOONGARCH_GPR_S5, LOONGARCH_GPR_SP, load_offset);

	emit_insn(ctx, addid, LOONGARCH_GPR_SP, LOONGARCH_GPR_SP, stack_adjust);

	if (!is_tail_call) {
		/* Set return value */
		move_reg(ctx, LOONGARCH_GPR_V0, regmap[BPF_REG_0]);
		/* Return to the caller */
		emit_insn(ctx, jirl, LOONGARCH_GPR_ZERO, LOONGARCH_GPR_RA, 0);
	} else {
		/*
		 * Call the next bpf prog and skip the first instruction
		 * of TCC initialization.
		 */
		emit_insn(ctx, jirl, LOONGARCH_GPR_ZERO, regmap[TMP_REG_3], 1);
	}
}

void build_epilogue(struct jit_ctx *ctx)
{
	__build_epilogue(ctx, false);
}

/* initialized on the first pass of build_body() */
static int out_offset = -1;
static int emit_bpf_tail_call(struct jit_ctx *ctx)
{
	int off;
	u8 tcc = tail_call_reg(ctx);
	u8 a1 = LOONGARCH_GPR_A1;
	u8 a2 = LOONGARCH_GPR_A2;
	u8 tmp1 = regmap[TMP_REG_1];
	u8 tmp2 = regmap[TMP_REG_2];
	u8 tmp3 = regmap[TMP_REG_3];
	const int idx0 = ctx->idx;

#define cur_offset (ctx->idx - idx0)
#define jmp_offset (out_offset - (cur_offset))

	/*
	 * a0: &ctx
	 * a1: &array
	 * a2: index
	 *
	 * if (index >= array->map.max_entries)
	 *	 goto out;
	 */
	off = offsetof(struct bpf_array, map.max_entries);
	emit_insn(ctx, ldwu, tmp1, a1, off);
	/* bgeu $a2, $t1, jmp_offset */
	emit_tailcall_jump(ctx, BPF_JGE, a2, tmp1, jmp_offset);

	/*
	 * if (TCC-- < 0)
	 *	 goto out;
	 */
	emit_insn(ctx, addid, tmp1, tcc, -1);
	emit_tailcall_jump(ctx, BPF_JSLT, tcc, LOONGARCH_GPR_ZERO, jmp_offset);

	/*
	 * prog = array->ptrs[index];
	 * if (!prog)
	 *	 goto out;
	 */
	emit_insn(ctx, sllid, tmp2, a2, 3);
	emit_insn(ctx, addd, tmp2, tmp2, a1);
	off = offsetof(struct bpf_array, ptrs);
	emit_insn(ctx, ldd, tmp2, tmp2, off);
	/* beq $t2, $zero, jmp_offset */
	emit_tailcall_jump(ctx, BPF_JEQ, tmp2, LOONGARCH_GPR_ZERO, jmp_offset);

	/* goto *(prog->bpf_func + 4); */
	off = offsetof(struct bpf_prog, bpf_func);
	emit_insn(ctx, ldd, tmp3, tmp2, off);
	move_reg(ctx, tcc, tmp1);
	__build_epilogue(ctx, true);

	/* out: */
	if (out_offset == -1)
		out_offset = cur_offset;
	if (cur_offset != out_offset) {
		pr_err_once("tail_call out_offset = %d, expected %d!\n",
			     cur_offset, out_offset);
		return -1;
	}

	return 0;
#undef cur_offset
#undef jmp_offset
}

/* backport from upstream commit e2c95a61656d in kernel/bpf/core.c */
static int bpf_jit_get_func_addr(const struct bpf_prog *prog,
			  const struct bpf_insn *insn, bool extra_pass,
			  u64 *func_addr, bool *func_addr_fixed)
{
	s16 off = insn->off;
	s32 imm = insn->imm;
	u8 *addr;

	*func_addr_fixed = insn->src_reg != BPF_PSEUDO_CALL;
	if (!*func_addr_fixed) {
		/* Place-holder address till the last pass has collected
		 * all addresses for JITed subprograms in which case we
		 * can pick them up from prog->aux.
		 */
		if (!extra_pass)
			addr = NULL;
		else if (prog->aux->func &&
			 off >= 0 && off < prog->aux->func_cnt)
			addr = (u8 *)prog->aux->func[off]->bpf_func;
		else
			return -EINVAL;
	} else {
		/* Address of a BPF helper call. Since part of the core
		 * kernel, it's always at a fixed location. __bpf_call_base
		 * and the helper with imm relative to it are both in core
		 * kernel.
		 */
		addr = (u8 *)__bpf_call_base + imm;
	}

	*func_addr = (unsigned long)addr;
	return 0;
}

static int build_insn(const struct bpf_insn *insn, struct jit_ctx *ctx, bool extra_pass)
{
	bool is32 = (BPF_CLASS(insn->code) == BPF_ALU);
	const u8 code = insn->code;
	const u8 cond = BPF_OP(code);
	const u8 dst = regmap[insn->dst_reg];
	const u8 src = regmap[insn->src_reg];
	const u8 tmp = regmap[TMP_REG_1];
	const u8 tmp2 = regmap[TMP_REG_2];
	const s16 off = insn->off;
	const s32 imm = insn->imm;
	int i = insn - ctx->prog->insnsi;
	int jmp_offset;
	bool func_addr_fixed;
	u64 func_addr;
	u64 imm64;
	int ret;

	switch (code) {
	/* dst = src */
	case BPF_ALU | BPF_MOV | BPF_X:
	case BPF_ALU64 | BPF_MOV | BPF_X:
		move_reg(ctx, dst, src);
		emit_zext_32(ctx, dst, is32);
		break;
	/* dst = imm */
	case BPF_ALU | BPF_MOV | BPF_K:
	case BPF_ALU64 | BPF_MOV | BPF_K:
		move_imm32(ctx, dst, imm, is32);
		break;

	/* dst = dst + src */
	case BPF_ALU | BPF_ADD | BPF_X:
	case BPF_ALU64 | BPF_ADD | BPF_X:
		emit_insn(ctx, addd, dst, dst, src);
		emit_zext_32(ctx, dst, is32);
		break;
	/* dst = dst + imm */
	case BPF_ALU | BPF_ADD | BPF_K:
	case BPF_ALU64 | BPF_ADD | BPF_K:
		if (is_signed_imm12(imm)) {
			emit_insn(ctx, addid, dst, dst, imm);
		} else {
			move_imm32(ctx, tmp, imm, is32);
			emit_insn(ctx, addd, dst, dst, tmp);
		}
		emit_zext_32(ctx, dst, is32);
		break;

	/* dst = dst - src */
	case BPF_ALU | BPF_SUB | BPF_X:
	case BPF_ALU64 | BPF_SUB | BPF_X:
		emit_insn(ctx, subd, dst, dst, src);
		emit_zext_32(ctx, dst, is32);
		break;
	/* dst = dst - imm */
	case BPF_ALU | BPF_SUB | BPF_K:
	case BPF_ALU64 | BPF_SUB | BPF_K:
		if (is_signed_imm12(-imm)) {
			emit_insn(ctx, addid, dst, dst, -imm);
		} else {
			move_imm32(ctx, tmp, imm, is32);
			emit_insn(ctx, subd, dst, dst, tmp);
		}
		emit_zext_32(ctx, dst, is32);
		break;

	/* dst = dst * src */
	case BPF_ALU | BPF_MUL | BPF_X:
	case BPF_ALU64 | BPF_MUL | BPF_X:
		emit_insn(ctx, muld, dst, dst, src);
		emit_zext_32(ctx, dst, is32);
		break;
	/* dst = dst * imm */
	case BPF_ALU | BPF_MUL | BPF_K:
	case BPF_ALU64 | BPF_MUL | BPF_K:
		move_imm32(ctx, tmp, imm, is32);
		emit_insn(ctx, muld, dst, dst, tmp);
		emit_zext_32(ctx, dst, is32);
		break;

	/* dst = dst / src */
	case BPF_ALU | BPF_DIV | BPF_X:
	case BPF_ALU64 | BPF_DIV | BPF_X:
		emit_insn(ctx, divdu, dst, dst, src);
		emit_zext_32(ctx, dst, is32);
		break;
	/* dst = dst / imm */
	case BPF_ALU | BPF_DIV | BPF_K:
	case BPF_ALU64 | BPF_DIV | BPF_K:
		move_imm32(ctx, tmp, imm, is32);
		emit_insn(ctx, divdu, dst, dst, tmp);
		emit_zext_32(ctx, dst, is32);
		break;

	/* dst = dst % src */
	case BPF_ALU | BPF_MOD | BPF_X:
	case BPF_ALU64 | BPF_MOD | BPF_X:
		emit_insn(ctx, moddu, dst, dst, src);
		emit_zext_32(ctx, dst, is32);
		break;
	/* dst = dst % imm */
	case BPF_ALU | BPF_MOD | BPF_K:
	case BPF_ALU64 | BPF_MOD | BPF_K:
		move_imm32(ctx, tmp, imm, is32);
		emit_insn(ctx, moddu, dst, dst, tmp);
		emit_zext_32(ctx, dst, is32);
		break;

	/* dst = -dst */
	case BPF_ALU | BPF_NEG:
	case BPF_ALU64 | BPF_NEG:
		move_imm32(ctx, tmp, imm, is32);
		emit_insn(ctx, subd, dst, LOONGARCH_GPR_ZERO, dst);
		emit_zext_32(ctx, dst, is32);
		break;

	/* dst = dst & src */
	case BPF_ALU | BPF_AND | BPF_X:
	case BPF_ALU64 | BPF_AND | BPF_X:
		emit_insn(ctx, and, dst, dst, src);
		emit_zext_32(ctx, dst, is32);
		break;
	/* dst = dst & imm */
	case BPF_ALU | BPF_AND | BPF_K:
	case BPF_ALU64 | BPF_AND | BPF_K:
		if (is_unsigned_imm12(imm)) {
			emit_insn(ctx, andi, dst, dst, imm);
		} else {
			move_imm32(ctx, tmp, imm, is32);
			emit_insn(ctx, and, dst, dst, tmp);
		}
		emit_zext_32(ctx, dst, is32);
		break;

	/* dst = dst | src */
	case BPF_ALU | BPF_OR | BPF_X:
	case BPF_ALU64 | BPF_OR | BPF_X:
		emit_insn(ctx, or, dst, dst, src);
		emit_zext_32(ctx, dst, is32);
		break;
	/* dst = dst | imm */
	case BPF_ALU | BPF_OR | BPF_K:
	case BPF_ALU64 | BPF_OR | BPF_K:
		if (is_unsigned_imm12(imm)) {
			emit_insn(ctx, ori, dst, dst, imm);
		} else {
			move_imm32(ctx, tmp, imm, is32);
			emit_insn(ctx, or, dst, dst, tmp);
		}
		emit_zext_32(ctx, dst, is32);
		break;

	/* dst = dst ^ src */
	case BPF_ALU | BPF_XOR | BPF_X:
	case BPF_ALU64 | BPF_XOR | BPF_X:
		emit_insn(ctx, xor, dst, dst, src);
		emit_zext_32(ctx, dst, is32);
		break;
	/* dst = dst ^ imm */
	case BPF_ALU | BPF_XOR | BPF_K:
	case BPF_ALU64 | BPF_XOR | BPF_K:
		if (is_unsigned_imm12(imm)) {
			emit_insn(ctx, xori, dst, dst, imm);
		} else {
			move_imm32(ctx, tmp, imm, is32);
			emit_insn(ctx, xor, dst, dst, tmp);
		}
		emit_zext_32(ctx, dst, is32);
		break;

	/* dst = dst << src (logical) */
	case BPF_ALU | BPF_LSH | BPF_X:
		emit_insn(ctx, sllw, dst, dst, src);
		emit_zext_32(ctx, dst, is32);
		break;
	case BPF_ALU64 | BPF_LSH | BPF_X:
		emit_insn(ctx, slld, dst, dst, src);
		break;
	/* dst = dst << imm (logical) */
	case BPF_ALU | BPF_LSH | BPF_K:
		emit_insn(ctx, slliw, dst, dst, imm);
		emit_zext_32(ctx, dst, is32);
		break;
	case BPF_ALU64 | BPF_LSH | BPF_K:
		emit_insn(ctx, sllid, dst, dst, imm);
		break;

	/* dst = dst >> src (logical) */
	case BPF_ALU | BPF_RSH | BPF_X:
		emit_insn(ctx, srlw, dst, dst, src);
		emit_zext_32(ctx, dst, is32);
		break;
	case BPF_ALU64 | BPF_RSH | BPF_X:
		emit_insn(ctx, srld, dst, dst, src);
		break;
	/* dst = dst >> imm (logical) */
	case BPF_ALU | BPF_RSH | BPF_K:
		emit_insn(ctx, srliw, dst, dst, imm);
		emit_zext_32(ctx, dst, is32);
		break;
	case BPF_ALU64 | BPF_RSH | BPF_K:
		emit_insn(ctx, srlid, dst, dst, imm);
		break;

	/* dst = dst >> src (arithmetic) */
	case BPF_ALU | BPF_ARSH | BPF_X:
		emit_insn(ctx, sraw, dst, dst, src);
		emit_zext_32(ctx, dst, is32);
		break;
	case BPF_ALU64 | BPF_ARSH | BPF_X:
		emit_insn(ctx, srad, dst, dst, src);
		break;
	/* dst = dst >> imm (arithmetic) */
	case BPF_ALU | BPF_ARSH | BPF_K:
		emit_insn(ctx, sraiw, dst, dst, imm);
		emit_zext_32(ctx, dst, is32);
		break;
	case BPF_ALU64 | BPF_ARSH | BPF_K:
		emit_insn(ctx, sraid, dst, dst, imm);
		break;

	/* dst = BSWAP##imm(dst) */
	case BPF_ALU | BPF_END | BPF_FROM_LE:
		switch (imm) {
		case 16:
			/* zero-extend 16 bits into 64 bits */
			emit_insn(ctx, sllid, dst, dst, 48);
			emit_insn(ctx, srlid, dst, dst, 48);
			break;
		case 32:
			/* zero-extend 32 bits into 64 bits */
			emit_zext_32(ctx, dst, is32);
			break;
		case 64:
			/* do nothing */
			break;
		}
		break;
	case BPF_ALU | BPF_END | BPF_FROM_BE:
		switch (imm) {
		case 16:
			emit_insn(ctx, revb2h, dst, dst);
			/* zero-extend 16 bits into 64 bits */
			emit_insn(ctx, sllid, dst, dst, 48);
			emit_insn(ctx, srlid, dst, dst, 48);
			break;
		case 32:
			emit_insn(ctx, revb2w, dst, dst);
			/* zero-extend 32 bits into 64 bits */
			emit_zext_32(ctx, dst, is32);
			break;
		case 64:
			emit_insn(ctx, revbd, dst, dst);
			break;
		}
		break;

	/* PC += off if dst cond src */
	case BPF_JMP | BPF_JEQ | BPF_X:
	case BPF_JMP | BPF_JNE | BPF_X:
	case BPF_JMP | BPF_JGT | BPF_X:
	case BPF_JMP | BPF_JGE | BPF_X:
	case BPF_JMP | BPF_JLT | BPF_X:
	case BPF_JMP | BPF_JLE | BPF_X:
	case BPF_JMP | BPF_JSGT | BPF_X:
	case BPF_JMP | BPF_JSGE | BPF_X:
	case BPF_JMP | BPF_JSLT | BPF_X:
	case BPF_JMP | BPF_JSLE | BPF_X:
		jmp_offset = bpf2la_offset(i, off, ctx);
		emit_cond_jump(ctx, cond, dst, src, jmp_offset);
		break;

	/* PC += off if dst cond imm */
	case BPF_JMP | BPF_JEQ | BPF_K:
	case BPF_JMP | BPF_JNE | BPF_K:
	case BPF_JMP | BPF_JGT | BPF_K:
	case BPF_JMP | BPF_JGE | BPF_K:
	case BPF_JMP | BPF_JLT | BPF_K:
	case BPF_JMP | BPF_JLE | BPF_K:
	case BPF_JMP | BPF_JSGT | BPF_K:
	case BPF_JMP | BPF_JSGE | BPF_K:
	case BPF_JMP | BPF_JSLT | BPF_K:
	case BPF_JMP | BPF_JSLE | BPF_K:
		jmp_offset = bpf2la_offset(i, off, ctx);
		move_imm32(ctx, tmp, imm, is32);
		emit_cond_jump(ctx, cond, dst, tmp, jmp_offset);
		break;

	/* PC += off if dst & src */
	case BPF_JMP | BPF_JSET | BPF_X:
		jmp_offset = bpf2la_offset(i, off, ctx);
		emit_insn(ctx, and, tmp, dst, src);
		emit_cond_jump(ctx, cond, tmp, LOONGARCH_GPR_ZERO, jmp_offset);
		break;
	/* PC += off if dst & imm */
	case BPF_JMP | BPF_JSET | BPF_K:
		jmp_offset = bpf2la_offset(i, off, ctx);
		move_imm32(ctx, tmp, imm, is32);
		emit_insn(ctx, and, tmp, dst, tmp);
		emit_cond_jump(ctx, cond, tmp, LOONGARCH_GPR_ZERO, jmp_offset);
		break;

	/* PC += off */
	case BPF_JMP | BPF_JA:
		jmp_offset = bpf2la_offset(i, off, ctx);
		emit_uncond_jump(ctx, jmp_offset, false);
		break;

	/* function call */
	case BPF_JMP | BPF_CALL:
		mark_call(ctx);
		ret = bpf_jit_get_func_addr(ctx->prog, insn, extra_pass,
					    &func_addr, &func_addr_fixed);
		if (ret < 0)
			return ret;

		move_imm64(ctx, tmp, func_addr, is32);
		emit_insn(ctx, jirl, LOONGARCH_GPR_RA, tmp, 0);
		move_reg(ctx, regmap[BPF_REG_0], LOONGARCH_GPR_V0);
		break;

	/* tail call */
	case BPF_JMP | BPF_TAIL_CALL:
		mark_tail_call(ctx);
		if (emit_bpf_tail_call(ctx))
			return -EINVAL;
		break;

	/* function return */
	case BPF_JMP | BPF_EXIT:
		emit_sext_32(ctx, regmap[BPF_REG_0]);
		/*
		 * Optimization: when last instruction is EXIT,
		 * simply fallthrough to epilogue.
		 */
		if (i == ctx->prog->len - 1)
			break;

		jmp_offset = epilogue_offset(ctx);
		emit_uncond_jump(ctx, jmp_offset, true);
		break;

	/* dst = imm64 */
	case BPF_LD | BPF_IMM | BPF_DW:
		imm64 = (u64)(insn + 1)->imm << 32 | (u32)insn->imm;
		move_imm64(ctx, dst, imm64, is32);
		return 1;

	/* dst = *(size *)(src + off) */
	case BPF_LDX | BPF_MEM | BPF_B:
	case BPF_LDX | BPF_MEM | BPF_H:
	case BPF_LDX | BPF_MEM | BPF_W:
	case BPF_LDX | BPF_MEM | BPF_DW:
		if (is_signed_imm12(off)) {
			switch (BPF_SIZE(code)) {
			case BPF_B:
				emit_insn(ctx, ldbu, dst, src, off);
				break;
			case BPF_H:
				emit_insn(ctx, ldhu, dst, src, off);
				break;
			case BPF_W:
				emit_insn(ctx, ldwu, dst, src, off);
				break;
			case BPF_DW:
				emit_insn(ctx, ldd, dst, src, off);
				break;
			}
		} else {
			move_imm32(ctx, tmp, off, is32);
			switch (BPF_SIZE(code)) {
			case BPF_B:
				emit_insn(ctx, ldxbu, dst, src, tmp);
				break;
			case BPF_H:
				emit_insn(ctx, ldxhu, dst, src, tmp);
				break;
			case BPF_W:
				emit_insn(ctx, ldxwu, dst, src, tmp);
				break;
			case BPF_DW:
				emit_insn(ctx, ldxd, dst, src, tmp);
				break;
			}
		}
		break;

	/* *(size *)(dst + off) = imm */
	case BPF_ST | BPF_MEM | BPF_B:
	case BPF_ST | BPF_MEM | BPF_H:
	case BPF_ST | BPF_MEM | BPF_W:
	case BPF_ST | BPF_MEM | BPF_DW:
		move_imm32(ctx, tmp, imm, is32);
		if (is_signed_imm12(off)) {
			switch (BPF_SIZE(code)) {
			case BPF_B:
				emit_insn(ctx, stb, tmp, dst, off);
				break;
			case BPF_H:
				emit_insn(ctx, sth, tmp, dst, off);
				break;
			case BPF_W:
				emit_insn(ctx, stw, tmp, dst, off);
				break;
			case BPF_DW:
				emit_insn(ctx, std, tmp, dst, off);
				break;
			}
		} else {
			move_imm32(ctx, tmp2, off, is32);
			switch (BPF_SIZE(code)) {
			case BPF_B:
				emit_insn(ctx, stxb, tmp, dst, tmp2);
				break;
			case BPF_H:
				emit_insn(ctx, stxh, tmp, dst, tmp2);
				break;
			case BPF_W:
				emit_insn(ctx, stxw, tmp, dst, tmp2);
				break;
			case BPF_DW:
				emit_insn(ctx, stxd, tmp, dst, tmp2);
				break;
			}
		}
		break;

	/* *(size *)(dst + off) = src */
	case BPF_STX | BPF_MEM | BPF_B:
	case BPF_STX | BPF_MEM | BPF_H:
	case BPF_STX | BPF_MEM | BPF_W:
	case BPF_STX | BPF_MEM | BPF_DW:
		if (is_signed_imm12(off)) {
			switch (BPF_SIZE(code)) {
			case BPF_B:
				emit_insn(ctx, stb, src, dst, off);
				break;
			case BPF_H:
				emit_insn(ctx, sth, src, dst, off);
				break;
			case BPF_W:
				emit_insn(ctx, stw, src, dst, off);
				break;
			case BPF_DW:
				emit_insn(ctx, std, src, dst, off);
				break;
			}
		} else {
			move_imm32(ctx, tmp, off, is32);
			switch (BPF_SIZE(code)) {
			case BPF_B:
				emit_insn(ctx, stxb, src, dst, tmp);
				break;
			case BPF_H:
				emit_insn(ctx, stxh, src, dst, tmp);
				break;
			case BPF_W:
				emit_insn(ctx, stxw, src, dst, tmp);
				break;
			case BPF_DW:
				emit_insn(ctx, stxd, src, dst, tmp);
				break;
			}
		}
		break;

	/* atomic_add: lock *(size *)(dst + off) += src */
	case BPF_STX | BPF_XADD | BPF_W:
	case BPF_STX | BPF_XADD | BPF_DW:
		if (insn->imm != BPF_ADD) {
			pr_err_once("unknown atomic op code %02x\n", insn->imm);
			return -EINVAL;
		}

		move_imm32(ctx, tmp, off, is32);
		emit_insn(ctx, addd, tmp, dst, tmp);
		switch (BPF_SIZE(insn->code)) {
		case BPF_W:
			emit_insn(ctx, amaddw, tmp2, src, tmp);
			break;
		case BPF_DW:
			emit_insn(ctx, amaddd, tmp2, src, tmp);
			break;
		}
		break;

	default:
		pr_err("bpf_jit: unknown opcode %02x\n", code);
		return -EINVAL;
	}

	return 0;
}

static int build_body(struct jit_ctx *ctx, bool extra_pass)
{
	const struct bpf_prog *prog = ctx->prog;
	int i;

	for (i = 0; i < prog->len; i++) {
		const struct bpf_insn *insn = &prog->insnsi[i];
		int ret;

		if (!ctx->image)
			ctx->offset[i] = ctx->idx;

		ret = build_insn(insn, ctx, extra_pass);
		if (ret > 0) {
			i++;
			if (!ctx->image)
				ctx->offset[i] = ctx->idx;
			continue;
		}
		if (ret)
			return ret;
	}

	if (!ctx->image)
		ctx->offset[i] = ctx->idx;

	return 0;
}

static inline void bpf_flush_icache(void *start, void *end)
{
	flush_icache_range((unsigned long)start, (unsigned long)end);
}

/* Fill space with illegal instructions */
static void jit_fill_hole(void *area, unsigned int size)
{
	u32 *ptr;

	/* We are guaranteed to have aligned memory */
	for (ptr = area; size >= sizeof(u32); size -= sizeof(u32))
		*ptr++ = INSN_BREAK;
}

static int validate_code(struct jit_ctx *ctx)
{
	int i;
	union loongarch_instruction insn;

	for (i = 0; i < ctx->idx; i++) {
		insn = ctx->image[i];
		/* Check INSN_BREAK */
		if (insn.word == INSN_BREAK)
			return -1;
	}

	return 0;
}

struct bpf_prog *bpf_int_jit_compile(struct bpf_prog *prog)
{
	struct bpf_prog *tmp, *orig_prog = prog;
	struct bpf_binary_header *header;
	struct jit_data *jit_data;
	struct jit_ctx ctx;
	bool tmp_blinded = false;
	bool extra_pass = false;
	int image_size;
	u8 *image_ptr;

	/*
	 * If BPF JIT was not enabled then we must fall back to
	 * the interpreter.
	 */
	if (!prog->jit_requested)
		return orig_prog;

	tmp = bpf_jit_blind_constants(prog);
	/*
	 * If blinding was requested and we failed during blinding,
	 * we must fall back to the interpreter. Otherwise, we save
	 * the new JITed code.
	 */
	if (IS_ERR(tmp))
		return orig_prog;
	if (tmp != prog) {
		tmp_blinded = true;
		prog = tmp;
	}

	jit_data = prog->aux->jit_data;
	if (!jit_data) {
		jit_data = kzalloc(sizeof(*jit_data), GFP_KERNEL);
		if (!jit_data) {
			prog = orig_prog;
			goto out;
		}
		prog->aux->jit_data = jit_data;
	}
	if (jit_data->ctx.offset) {
		ctx = jit_data->ctx;
		image_ptr = jit_data->image;
		header = jit_data->header;
		extra_pass = true;
		image_size = sizeof(u32) * ctx.idx;
		goto skip_init_ctx;
	}

	memset(&ctx, 0, sizeof(ctx));
	ctx.prog = prog;

	ctx.offset = kcalloc(prog->len + 1, sizeof(*ctx.offset), GFP_KERNEL);
	if (!ctx.offset) {
		prog = orig_prog;
		goto out_off;
	}

	/* 1. Initial fake pass to compute ctx->idx and set ctx->flags */
	if (build_body(&ctx, extra_pass)) {
		prog = orig_prog;
		goto out_off;
	}
	build_prologue(&ctx);
	ctx.epilogue_offset = ctx.idx;
	build_epilogue(&ctx);

	/* Now we know the actual image size.
	 * As each LoongArch instruction is of length 32bit,
	 * we are translating number of JITed intructions into
	 * the size required to store these JITed code.
	 */
	image_size = sizeof(u32) * ctx.idx;
	/* Now we know the size of the structure to make */
	header = bpf_jit_binary_alloc(image_size, &image_ptr,
				      sizeof(u32), jit_fill_hole);
	if (!header) {
		prog = orig_prog;
		goto out_off;
	}

	/* 2. Now, the actual pass to generate final JIT code */
	ctx.image = (union loongarch_instruction *)image_ptr;
skip_init_ctx:
	ctx.idx = 0;

	build_prologue(&ctx);
	if (build_body(&ctx, extra_pass)) {
		bpf_jit_binary_free(header);
		prog = orig_prog;
		goto out_off;
	}
	build_epilogue(&ctx);

	/* 3. Extra pass to validate JITed code */
	if (validate_code(&ctx)) {
		bpf_jit_binary_free(header);
		prog = orig_prog;
		goto out_off;
	}

	/* And we're done */
	if (bpf_jit_enable > 1)
		bpf_jit_dump(prog->len, image_size, 2, ctx.image);

	/* Update the icache */
	bpf_flush_icache(header, ctx.image + ctx.idx);

	if (!prog->is_func || extra_pass) {
		if (extra_pass && ctx.idx != jit_data->ctx.idx) {
			pr_err_once("multi-func JIT bug %d != %d\n",
				    ctx.idx, jit_data->ctx.idx);
			bpf_jit_binary_free(header);
			prog->bpf_func = NULL;
			prog->jited = 0;
			goto out_off;
		}
		bpf_jit_binary_lock_ro(header);
	} else {
		jit_data->ctx = ctx;
		jit_data->image = image_ptr;
		jit_data->header = header;
	}
	prog->bpf_func = (void *)ctx.image;
	prog->jited = 1;
	prog->jited_len = image_size;

	if (!prog->is_func || extra_pass) {
out_off:
		kfree(ctx.offset);
		kfree(jit_data);
		prog->aux->jit_data = NULL;
	}
out:
	if (tmp_blinded)
		bpf_jit_prog_release_other(prog, prog == orig_prog ?
					   tmp : orig_prog);

	out_offset = -1;
	return prog;
}
