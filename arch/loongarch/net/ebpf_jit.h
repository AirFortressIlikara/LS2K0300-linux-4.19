/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * BPF JIT compiler for LoongArch
 *
 * Copyright (C) 2021 Loongson Technology Corporation Limited
 */
#include <linux/bpf.h>
#include <linux/filter.h>
#include <asm/cacheflush.h>
#include <asm/inst.h>

struct jit_ctx {
	const struct bpf_prog *prog;
	unsigned int idx;
	unsigned int flags;
	unsigned int epilogue_offset;
	u32 *offset;
	union loongarch_instruction *image;
	u32 stack_size;
};

struct jit_data {
	struct bpf_binary_header *header;
	u8 *image;
	struct jit_ctx ctx;
};

#define emit_insn(ctx, func, ...)						\
do {										\
	if (ctx->image != NULL) {						\
		union loongarch_instruction *insn = &ctx->image[ctx->idx];	\
		emit_##func(insn, ##__VA_ARGS__);				\
	}									\
	ctx->idx++;								\
} while (0)

static inline bool is_unsigned_imm(unsigned long val, unsigned int bit)
{
	return val < (1UL << bit);
}

static inline bool is_signed_imm(long val, unsigned int bit)
{
	return -(1L << (bit - 1)) <= val && val < (1L << (bit - 1));
}

#define is_signed_imm12(val) is_signed_imm(val, 12)
#define is_signed_imm16(val) is_signed_imm(val, 16)
#define is_signed_imm26(val) is_signed_imm(val, 26)
#define is_signed_imm32(val) is_signed_imm(val, 32)
#define is_signed_imm52(val) is_signed_imm(val, 52)
#define is_unsigned_imm12(val) is_unsigned_imm(val, 12)
#define is_unsigned_imm32(val) is_unsigned_imm(val, 32)

static inline int bpf2la_offset(int bpf_insn, int off, const struct jit_ctx *ctx)
{
	/* BPF JMP offset is relative to the next instruction */
	bpf_insn++;
	/*
	 * Whereas la64 branch instructions encode the offset
	 * from the branch itself, so we must subtract 1 from the
	 * instruction offset.
	 */
	return (ctx->offset[bpf_insn + off] - (ctx->offset[bpf_insn] - 1));
}

static inline int epilogue_offset(const struct jit_ctx *ctx)
{
	int to = ctx->epilogue_offset;
	int from = ctx->idx;

	return (to - from);
}

static inline void emit_ldbu(union loongarch_instruction *insn,
			     enum loongarch_gpr rd, enum loongarch_gpr rj, int imm)
{
	insn->reg2i12_format.opcode = ldbu_op;
	insn->reg2i12_format.simmediate = imm;
	insn->reg2i12_format.rd = rd;
	insn->reg2i12_format.rj = rj;
}

static inline void emit_ldhu(union loongarch_instruction *insn,
			     enum loongarch_gpr rd, enum loongarch_gpr rj, int imm)
{
	insn->reg2i12_format.opcode = ldhu_op;
	insn->reg2i12_format.simmediate = imm;
	insn->reg2i12_format.rd = rd;
	insn->reg2i12_format.rj = rj;
}

static inline void emit_ldwu(union loongarch_instruction *insn,
			     enum loongarch_gpr rd, enum loongarch_gpr rj, int imm)
{
	insn->reg2i12_format.opcode = ldwu_op;
	insn->reg2i12_format.simmediate = imm;
	insn->reg2i12_format.rd = rd;
	insn->reg2i12_format.rj = rj;
}

static inline void emit_ldd(union loongarch_instruction *insn,
			    enum loongarch_gpr rd, enum loongarch_gpr rj, int imm)
{
	insn->reg2i12_format.opcode = ldd_op;
	insn->reg2i12_format.simmediate = imm;
	insn->reg2i12_format.rd = rd;
	insn->reg2i12_format.rj = rj;
}

static inline void emit_stb(union loongarch_instruction *insn,
			    enum loongarch_gpr rd, enum loongarch_gpr rj, int imm)
{
	insn->reg2i12_format.opcode = stb_op;
	insn->reg2i12_format.simmediate = imm;
	insn->reg2i12_format.rd = rd;
	insn->reg2i12_format.rj = rj;
}

static inline void emit_sth(union loongarch_instruction *insn,
			    enum loongarch_gpr rd, enum loongarch_gpr rj, int imm)
{
	insn->reg2i12_format.opcode = sth_op;
	insn->reg2i12_format.simmediate = imm;
	insn->reg2i12_format.rd = rd;
	insn->reg2i12_format.rj = rj;
}

static inline void emit_stw(union loongarch_instruction *insn,
			    enum loongarch_gpr rd, enum loongarch_gpr rj, int imm)
{
	insn->reg2i12_format.opcode = stw_op;
	insn->reg2i12_format.simmediate = imm;
	insn->reg2i12_format.rd = rd;
	insn->reg2i12_format.rj = rj;
}

static inline void emit_std(union loongarch_instruction *insn,
			    enum loongarch_gpr rd, enum loongarch_gpr rj, int imm)
{
	insn->reg2i12_format.opcode = std_op;
	insn->reg2i12_format.simmediate = imm;
	insn->reg2i12_format.rd = rd;
	insn->reg2i12_format.rj = rj;
}

static inline void emit_ldxbu(union loongarch_instruction *insn, enum loongarch_gpr rd,
			      enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = ldxbu_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_ldxhu(union loongarch_instruction *insn, enum loongarch_gpr rd,
			      enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = ldxhu_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_ldxwu(union loongarch_instruction *insn, enum loongarch_gpr rd,
			      enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = ldxwu_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_ldxd(union loongarch_instruction *insn, enum loongarch_gpr rd,
			     enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = ldxd_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_stxb(union loongarch_instruction *insn, enum loongarch_gpr rd,
			     enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = stxb_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_stxh(union loongarch_instruction *insn, enum loongarch_gpr rd,
			     enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = stxh_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_stxw(union loongarch_instruction *insn, enum loongarch_gpr rd,
			     enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = stxw_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_stxd(union loongarch_instruction *insn, enum loongarch_gpr rd,
			     enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = stxd_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_amaddw(union loongarch_instruction *insn, enum loongarch_gpr rd,
			       enum loongarch_gpr rk, enum loongarch_gpr rj)
{
	insn->reg3_format.opcode = amaddw_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rk = rk;
	insn->reg3_format.rj = rj;
}

static inline void emit_amaddd(union loongarch_instruction *insn, enum loongarch_gpr rd,
			       enum loongarch_gpr rk, enum loongarch_gpr rj)
{
	insn->reg3_format.opcode = amaddd_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rk = rk;
	insn->reg3_format.rj = rj;
}

static inline void emit_addd(union loongarch_instruction *insn, enum loongarch_gpr rd,
			     enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = addd_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_addiw(union loongarch_instruction *insn,
			      enum loongarch_gpr rd, enum loongarch_gpr rj, int imm)
{
	insn->reg2i12_format.opcode = addiw_op;
	insn->reg2i12_format.simmediate = imm;
	insn->reg2i12_format.rd = rd;
	insn->reg2i12_format.rj = rj;
}

static inline void emit_addid(union loongarch_instruction *insn,
			      enum loongarch_gpr rd, enum loongarch_gpr rj, int imm)
{
	insn->reg2i12_format.opcode = addid_op;
	insn->reg2i12_format.simmediate = imm;
	insn->reg2i12_format.rd = rd;
	insn->reg2i12_format.rj = rj;
}

static inline void emit_subd(union loongarch_instruction *insn, enum loongarch_gpr rd,
			     enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = subd_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_muld(union loongarch_instruction *insn, enum loongarch_gpr rd,
			     enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = muld_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_divdu(union loongarch_instruction *insn, enum loongarch_gpr rd,
			     enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = divdu_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_moddu(union loongarch_instruction *insn, enum loongarch_gpr rd,
			     enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = moddu_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_and(union loongarch_instruction *insn, enum loongarch_gpr rd,
			    enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = and_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_andi(union loongarch_instruction *insn,
			     enum loongarch_gpr rd, enum loongarch_gpr rj, u32 imm)
{
	insn->reg2ui12_format.opcode = andi_op;
	insn->reg2ui12_format.simmediate = imm;
	insn->reg2ui12_format.rd = rd;
	insn->reg2ui12_format.rj = rj;
}

static inline void emit_or(union loongarch_instruction *insn, enum loongarch_gpr rd,
			   enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = or_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_ori(union loongarch_instruction *insn,
			    enum loongarch_gpr rd, enum loongarch_gpr rj, u32 imm)
{
	insn->reg2ui12_format.opcode = ori_op;
	insn->reg2ui12_format.simmediate = imm;
	insn->reg2ui12_format.rd = rd;
	insn->reg2ui12_format.rj = rj;
}

static inline void emit_xor(union loongarch_instruction *insn, enum loongarch_gpr rd,
			    enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = xor_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_xori(union loongarch_instruction *insn,
			     enum loongarch_gpr rd, enum loongarch_gpr rj, u32 imm)
{
	insn->reg2ui12_format.opcode = xori_op;
	insn->reg2ui12_format.simmediate = imm;
	insn->reg2ui12_format.rd = rd;
	insn->reg2ui12_format.rj = rj;
}

static inline void emit_lu12iw(union loongarch_instruction *insn,
			       enum loongarch_gpr rd, int imm)
{
	insn->reg1i20_format.opcode = lu12iw_op;
	insn->reg1i20_format.simmediate = imm;
	insn->reg1i20_format.rd = rd;
}

static inline void emit_lu32id(union loongarch_instruction *insn,
			       enum loongarch_gpr rd, int imm)
{
	insn->reg1i20_format.opcode = lu32id_op;
	insn->reg1i20_format.simmediate = imm;
	insn->reg1i20_format.rd = rd;
}

static inline void emit_lu52id(union loongarch_instruction *insn,
			       enum loongarch_gpr rd, enum loongarch_gpr rj, int imm)
{
	insn->reg2i12_format.opcode = lu52id_op;
	insn->reg2i12_format.simmediate = imm;
	insn->reg2i12_format.rd = rd;
	insn->reg2i12_format.rj = rj;
}

static inline void emit_sllw(union loongarch_instruction *insn, enum loongarch_gpr rd,
			     enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = sllw_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_slliw(union loongarch_instruction *insn,
			      enum loongarch_gpr rd, enum loongarch_gpr rj, u32 imm)
{
	insn->reg2ui5_format.opcode = slliw_op;
	insn->reg2ui5_format.simmediate = imm;
	insn->reg2ui5_format.rd = rd;
	insn->reg2ui5_format.rj = rj;
}

static inline void emit_slld(union loongarch_instruction *insn, enum loongarch_gpr rd,
			     enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = slld_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_sllid(union loongarch_instruction *insn,
			      enum loongarch_gpr rd, enum loongarch_gpr rj, u32 imm)
{
	insn->reg2ui6_format.opcode = sllid_op;
	insn->reg2ui6_format.simmediate = imm;
	insn->reg2ui6_format.rd = rd;
	insn->reg2ui6_format.rj = rj;
}

static inline void emit_srlw(union loongarch_instruction *insn, enum loongarch_gpr rd,
			     enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = srlw_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_srliw(union loongarch_instruction *insn,
			      enum loongarch_gpr rd, enum loongarch_gpr rj, u32 imm)
{
	insn->reg2ui5_format.opcode = srliw_op;
	insn->reg2ui5_format.simmediate = imm;
	insn->reg2ui5_format.rd = rd;
	insn->reg2ui5_format.rj = rj;
}

static inline void emit_srld(union loongarch_instruction *insn, enum loongarch_gpr rd,
			     enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = srld_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_srlid(union loongarch_instruction *insn,
			      enum loongarch_gpr rd, enum loongarch_gpr rj, u32 imm)
{
	insn->reg2ui6_format.opcode = srlid_op;
	insn->reg2ui6_format.simmediate = imm;
	insn->reg2ui6_format.rd = rd;
	insn->reg2ui6_format.rj = rj;
}

static inline void emit_sraw(union loongarch_instruction *insn, enum loongarch_gpr rd,
			     enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = sraw_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_sraiw(union loongarch_instruction *insn,
			      enum loongarch_gpr rd, enum loongarch_gpr rj, u32 imm)
{
	insn->reg2ui5_format.opcode = sraid_op;
	insn->reg2ui5_format.simmediate = imm;
	insn->reg2ui5_format.rd = rd;
	insn->reg2ui5_format.rj = rj;
}

static inline void emit_srad(union loongarch_instruction *insn, enum loongarch_gpr rd,
			     enum loongarch_gpr rj, enum loongarch_gpr rk)
{
	insn->reg3_format.opcode = srad_op;
	insn->reg3_format.rd = rd;
	insn->reg3_format.rj = rj;
	insn->reg3_format.rk = rk;
}

static inline void emit_sraid(union loongarch_instruction *insn,
			      enum loongarch_gpr rd, enum loongarch_gpr rj, u32 imm)
{
	insn->reg2ui6_format.opcode = sraid_op;
	insn->reg2ui6_format.simmediate = imm;
	insn->reg2ui6_format.rd = rd;
	insn->reg2ui6_format.rj = rj;
}

static inline void emit_beq(union loongarch_instruction *insn,
			    enum loongarch_gpr rj, enum loongarch_gpr rd, int offset)
{
	insn->reg2i16_format.opcode = beq_op;
	insn->reg2i16_format.simmediate = offset;
	insn->reg2i16_format.rj = rj;
	insn->reg2i16_format.rd = rd;
}

static inline void emit_bne(union loongarch_instruction *insn,
			    enum loongarch_gpr rj, enum loongarch_gpr rd, int offset)
{
	insn->reg2i16_format.opcode = bne_op;
	insn->reg2i16_format.simmediate = offset;
	insn->reg2i16_format.rj = rj;
	insn->reg2i16_format.rd = rd;
}

static inline void emit_blt(union loongarch_instruction *insn,
			    enum loongarch_gpr rj, enum loongarch_gpr rd, int offset)
{
	insn->reg2i16_format.opcode = blt_op;
	insn->reg2i16_format.simmediate = offset;
	insn->reg2i16_format.rj = rj;
	insn->reg2i16_format.rd = rd;
}

static inline void emit_bge(union loongarch_instruction *insn,
			    enum loongarch_gpr rj, enum loongarch_gpr rd, int offset)
{
	insn->reg2i16_format.opcode = bge_op;
	insn->reg2i16_format.simmediate = offset;
	insn->reg2i16_format.rj = rj;
	insn->reg2i16_format.rd = rd;
}

static inline void emit_bltu(union loongarch_instruction *insn,
			     enum loongarch_gpr rj, enum loongarch_gpr rd, int offset)
{
	insn->reg2i16_format.opcode = bltu_op;
	insn->reg2i16_format.simmediate = offset;
	insn->reg2i16_format.rj = rj;
	insn->reg2i16_format.rd = rd;
}

static inline void emit_bgeu(union loongarch_instruction *insn,
			     enum loongarch_gpr rj, enum loongarch_gpr rd, int offset)
{
	insn->reg2i16_format.opcode = bgeu_op;
	insn->reg2i16_format.simmediate = offset;
	insn->reg2i16_format.rj = rj;
	insn->reg2i16_format.rd = rd;
}

static inline void emit_b(union loongarch_instruction *insn, int offset)
{
	unsigned int simmediate_l, simmediate_h;

	simmediate_l = offset & 0xffff;
	offset >>= 16;
	simmediate_h = offset & 0x3ff;

	insn->reg0i26_format.opcode = b_op;
	insn->reg0i26_format.simmediate_l = simmediate_l;
	insn->reg0i26_format.simmediate_h = simmediate_h;
}

static inline void emit_jirl(union loongarch_instruction *insn,
			     enum loongarch_gpr rd, enum loongarch_gpr rj, int offset)
{
	insn->reg2i16_format.opcode = jirl_op;
	insn->reg2i16_format.simmediate = offset;
	insn->reg2i16_format.rd = rd;
	insn->reg2i16_format.rj = rj;
}

static inline void emit_pcaddu18i(union loongarch_instruction *insn,
				  enum loongarch_gpr rd, int imm)
{
	insn->reg1i20_format.opcode = pcaddu18i_op;
	insn->reg1i20_format.simmediate = imm;
	insn->reg1i20_format.rd = rd;
}

static inline void emit_revb2h(union loongarch_instruction *insn,
			    enum loongarch_gpr rd, enum loongarch_gpr rj)
{
	insn->reg2_format.opcode = revb2h_op;
	insn->reg2_format.rd = rd;
	insn->reg2_format.rj = rj;
}

static inline void emit_revb2w(union loongarch_instruction *insn,
			    enum loongarch_gpr rd, enum loongarch_gpr rj)
{
	insn->reg2_format.opcode = revb2w_op;
	insn->reg2_format.rd = rd;
	insn->reg2_format.rj = rj;
}

static inline void emit_revbd(union loongarch_instruction *insn,
			    enum loongarch_gpr rd, enum loongarch_gpr rj)
{
	insn->reg2_format.opcode = revbd_op;
	insn->reg2_format.rd = rd;
	insn->reg2_format.rj = rj;
}

/* Zero-extend 32 bits into 64 bits */
static inline void emit_zext_32(struct jit_ctx *ctx, enum loongarch_gpr reg, bool is32)
{
	if (!is32)
		return;

	/* Clear the upper 32 bits */
	emit_insn(ctx, lu32id, reg, 0);
}

/* Signed-extend 32 bits into 64 bits */
static inline void emit_sext_32(struct jit_ctx *ctx, enum loongarch_gpr reg)
{
	emit_insn(ctx, addiw, reg, reg, 0);
}

static inline void move_imm32(struct jit_ctx *ctx, enum loongarch_gpr rd,
			      int imm32, bool is32)
{
	int si20;
	u32 ui12;

	/* or rd, $zero, $zero */
	if (imm32 == 0) {
		emit_insn(ctx, or, rd, LOONGARCH_GPR_ZERO, LOONGARCH_GPR_ZERO);
		return;
	}

	/* addiw rd, $zero, imm_11_0(signed) */
	if (is_signed_imm12(imm32)) {
		emit_insn(ctx, addiw, rd, LOONGARCH_GPR_ZERO, imm32);
		goto zext;
	}

	/* ori rd, $zero, imm_11_0(unsigned) */
	if (is_unsigned_imm12(imm32)) {
		emit_insn(ctx, ori, rd, LOONGARCH_GPR_ZERO, imm32);
		goto zext;
	}

	/* lu12iw rd, imm_31_12(signed) */
	si20 = (imm32 >> 12) & 0xfffff;
	emit_insn(ctx, lu12iw, rd, si20);

	/* ori rd, rd, imm_11_0(unsigned) */
	ui12 = imm32 & 0xfff;
	if (ui12 != 0)
		emit_insn(ctx, ori, rd, rd, ui12);

zext:
	emit_zext_32(ctx, rd, is32);
}

static inline void move_imm64(struct jit_ctx *ctx, enum loongarch_gpr rd,
			      long imm64, bool is32)
{
	int imm32, si20, si12;
	long imm52;

	si12 = (imm64 >> 52) & 0xfff;
	imm52 = imm64 & 0xfffffffffffff;
	/* lu52id rd, $zero, imm_63_52(signed) */
	if (si12 != 0 && imm52 == 0) {
		emit_insn(ctx, lu52id, rd, LOONGARCH_GPR_ZERO, si12);
		return;
	}

	imm32 = imm64 & 0xffffffff;
	move_imm32(ctx, rd, imm32, is32);

	if (!is_signed_imm32(imm64)) {
		if (imm52 != 0) {
			/* lu32id rd, imm_51_32(signed) */
			si20 = (imm64 >> 32) & 0xfffff;
			emit_insn(ctx, lu32id, rd, si20);
		}

		/* lu52id rd, rd, imm_63_52(signed) */
		if (!is_signed_imm52(imm64))
			emit_insn(ctx, lu52id, rd, rd, si12);
	}
}

static inline void move_reg(struct jit_ctx *ctx, enum loongarch_gpr rd,
			    enum loongarch_gpr rj)
{
	emit_insn(ctx, or, rd, rj, LOONGARCH_GPR_ZERO);
}

static inline int invert_jump_cond(u8 cond)
{
	switch (cond) {
	case BPF_JEQ:
		return BPF_JNE;
	case BPF_JNE:
	case BPF_JSET:
		return BPF_JEQ;
	case BPF_JGT:
		return BPF_JLE;
	case BPF_JGE:
		return BPF_JLT;
	case BPF_JLT:
		return BPF_JGE;
	case BPF_JLE:
		return BPF_JGT;
	case BPF_JSGT:
		return BPF_JSLE;
	case BPF_JSGE:
		return BPF_JSLT;
	case BPF_JSLT:
		return BPF_JSGE;
	case BPF_JSLE:
		return BPF_JSGT;
	}
	return -1;
}

static inline void cond_jump_offs16(struct jit_ctx *ctx, u8 cond, enum loongarch_gpr rj,
				    enum loongarch_gpr rd, int jmp_offset)
{
	switch (cond) {
	case BPF_JEQ:
		/* PC += jmp_offset if rj == rd */
		emit_insn(ctx, beq, rj, rd, jmp_offset);
		return;
	case BPF_JNE:
	case BPF_JSET:
		/* PC += jmp_offset if rj != rd */
		emit_insn(ctx, bne, rj, rd, jmp_offset);
		return;
	case BPF_JGT:
		/* PC += jmp_offset if rj > rd (unsigned) */
		emit_insn(ctx, bltu, rd, rj, jmp_offset);
		return;
	case BPF_JLT:
		/* PC += jmp_offset if rj < rd (unsigned) */
		emit_insn(ctx, bltu, rj, rd, jmp_offset);
		return;
	case BPF_JGE:
		/* PC += jmp_offset if rj >= rd (unsigned) */
		emit_insn(ctx, bgeu, rj, rd, jmp_offset);
		return;
	case BPF_JLE:
		/* PC += jmp_offset if rj <= rd (unsigned) */
		emit_insn(ctx, bgeu, rd, rj, jmp_offset);
		return;
	case BPF_JSGT:
		/* PC += jmp_offset if rj > rd (signed) */
		emit_insn(ctx, blt, rd, rj, jmp_offset);
		return;
	case BPF_JSLT:
		/* PC += jmp_offset if rj < rd (signed) */
		emit_insn(ctx, blt, rj, rd, jmp_offset);
		return;
	case BPF_JSGE:
		/* PC += jmp_offset if rj >= rd (signed) */
		emit_insn(ctx, bge, rj, rd, jmp_offset);
		return;
	case BPF_JSLE:
		/* PC += jmp_offset if rj <= rd (signed) */
		emit_insn(ctx, bge, rd, rj, jmp_offset);
		return;
	}
}

static inline void cond_jump_offs26(struct jit_ctx *ctx, u8 cond, enum loongarch_gpr rj,
				    enum loongarch_gpr rd, int jmp_offset)
{
	cond = invert_jump_cond(cond);
	cond_jump_offs16(ctx, cond, rj, rd, 2);
	emit_insn(ctx, b, jmp_offset);
}

static inline void cond_jump_offs32(struct jit_ctx *ctx, u8 cond, enum loongarch_gpr rj,
				    enum loongarch_gpr rd, int jmp_offset)
{
	s64 upper, lower;

	upper = (jmp_offset + (1 << 15)) >> 16;
	lower = jmp_offset & 0xffff;

	cond = invert_jump_cond(cond);
	cond_jump_offs16(ctx, cond, rj, rd, 3);

	/*
	 * jmp_addr = jmp_offset << 2
	 * tmp2 = PC + jmp_addr[31, 18] + 18'b0
	 */
	emit_insn(ctx, pcaddu18i, LOONGARCH_GPR_T2, upper << 2);

	/* jump to (tmp2 + jmp_addr[17, 2] + 2'b0) */
	emit_insn(ctx, jirl, LOONGARCH_GPR_ZERO, LOONGARCH_GPR_T2, lower + 1);
}

static inline void uncond_jump_offs26(struct jit_ctx *ctx, int jmp_offset)
{
	emit_insn(ctx, b, jmp_offset);
}

static inline void uncond_jump_offs32(struct jit_ctx *ctx, int jmp_offset, bool is_exit)
{
	s64 upper, lower;

	upper = (jmp_offset + (1 << 15)) >> 16;
	lower = jmp_offset & 0xffff;

	if (is_exit)
		lower -= 1;

	/*
	 * jmp_addr = jmp_offset << 2;
	 * tmp1 = PC + jmp_addr[31, 18] + 18'b0
	 */
	emit_insn(ctx, pcaddu18i, LOONGARCH_GPR_T1, upper << 2);

	/* jump to (tmp1 + jmp_addr[17, 2] + 2'b0) */
	emit_insn(ctx, jirl, LOONGARCH_GPR_ZERO, LOONGARCH_GPR_T1, lower + 1);
}

static inline void emit_cond_jump(struct jit_ctx *ctx, u8 cond, enum loongarch_gpr rj,
				  enum loongarch_gpr rd, int jmp_offset)
{
	if (is_signed_imm16(jmp_offset))
		cond_jump_offs16(ctx, cond, rj, rd, jmp_offset);
	else if (is_signed_imm26(jmp_offset))
		cond_jump_offs26(ctx, cond, rj, rd, jmp_offset);
	else
		cond_jump_offs32(ctx, cond, rj, rd, jmp_offset);
}

static inline void emit_uncond_jump(struct jit_ctx *ctx, int jmp_offset, bool is_exit)
{
	if (is_signed_imm26(jmp_offset))
		uncond_jump_offs26(ctx, jmp_offset);
	else
		uncond_jump_offs32(ctx, jmp_offset, is_exit);
}

static inline void emit_tailcall_jump(struct jit_ctx *ctx, u8 cond, enum loongarch_gpr rj,
				      enum loongarch_gpr rd, int jmp_offset)
{
	if (is_signed_imm16(jmp_offset))
		cond_jump_offs16(ctx, cond, rj, rd, jmp_offset);
	else if (is_signed_imm26(jmp_offset))
		cond_jump_offs26(ctx, cond, rj, rd, jmp_offset - 1);
	else
		cond_jump_offs32(ctx, cond, rj, rd, jmp_offset - 2);
}
