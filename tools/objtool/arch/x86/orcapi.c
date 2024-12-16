// SPDX-License-Identifier: GPL-2.0-or-later
#include <string.h>
#include <stdlib.h>

#include "../../orc.h"
#include "../../warn.h"

static struct orc_entry empty = {
	.sp_reg = ORC_REG_UNDEFINED,
	.bp_reg  = ORC_REG_UNDEFINED,
	.type    = ORC_TYPE_CALL,
};

int create_orc(struct objtool_file *file)
{
	struct instruction *insn;

	for_each_insn(file, insn) {
		struct orc_entry *orc = &insn->orc;
		struct cfi_reg *cfa = &insn->state.cfa;
		struct cfi_reg *bp = &insn->state.regs[CFI_BP];

		orc->end = insn->state.end;

		if (cfa->base == CFI_UNDEFINED) {
			orc->sp_reg = ORC_REG_UNDEFINED;
			continue;
		}

		switch (cfa->base) {
		case CFI_SP:
			orc->sp_reg = ORC_REG_SP;
			break;
		case CFI_SP_INDIRECT:
			orc->sp_reg = ORC_REG_SP_INDIRECT;
			break;
		case CFI_BP:
			orc->sp_reg = ORC_REG_BP;
			break;
		case CFI_BP_INDIRECT:
			orc->sp_reg = ORC_REG_BP_INDIRECT;
			break;
		case CFI_R10:
			orc->sp_reg = ORC_REG_R10;
			break;
		case CFI_R13:
			orc->sp_reg = ORC_REG_R13;
			break;
		case CFI_DI:
			orc->sp_reg = ORC_REG_DI;
			break;
		case CFI_DX:
			orc->sp_reg = ORC_REG_DX;
			break;
		default:
			WARN_FUNC("unknown CFA base reg %d",
				  insn->sec, insn->offset, cfa->base);
			return -1;
		}

		switch (bp->base) {
		case CFI_UNDEFINED:
			orc->bp_reg = ORC_REG_UNDEFINED;
			break;
		case CFI_CFA:
			orc->bp_reg = ORC_REG_PREV_SP;
			break;
		case CFI_BP:
			orc->bp_reg = ORC_REG_BP;
			break;
		default:
			WARN_FUNC("unknown BP base reg %d",
				  insn->sec, insn->offset, bp->base);
			return -1;
		}

		orc->sp_offset = cfa->offset;
		orc->bp_offset = bp->offset;
		orc->type = insn->state.type;
	}

	return 0;
}

int arch_create_orc_entry(struct section *u_sec, struct section *ip_relasec,
				unsigned int idx, struct section *insn_sec,
				unsigned long insn_off, struct orc_entry *o)
{
	struct orc_entry *orc;
	struct rela *rela;

	/* populate ORC data */
	orc = (struct orc_entry *)u_sec->data->d_buf + idx;
	memcpy(orc, o, sizeof(*orc));

	/* populate rela for ip */
	rela = malloc(sizeof(*rela));
	if (!rela) {
		perror("malloc");
		return -1;
	}
	memset(rela, 0, sizeof(*rela));

	if (insn_sec->sym) {
		rela->sym = insn_sec->sym;
		rela->addend = insn_off;
	} else {
		/*
		 * The Clang assembler doesn't produce section symbols, so we
		 * have to reference the function symbol instead:
		 */
		rela->sym = find_symbol_containing(insn_sec, insn_off);
		if (!rela->sym) {
			/*
			 * Hack alert.  This happens when we need to reference
			 * the NOP pad insn immediately after the function.
			 */
			rela->sym = find_symbol_containing(insn_sec,
							   insn_off - 1);
		}
		if (!rela->sym) {
			WARN("missing symbol for insn at offset 0x%lx\n",
			     insn_off);
			return -1;
		}

		rela->addend = insn_off - rela->sym->offset;
	}

	rela->type = R_X86_64_PC32;
	rela->offset = idx * sizeof(int);

	list_add_tail(&rela->list, &ip_relasec->rela_list);
	hash_add(ip_relasec->rela_hash, &rela->hash, rela->offset);

	return 0;
}

int arch_create_orc_entry_empty(struct section *u_sec, struct section *ip_relasec,
				unsigned int idx, struct section *insn_sec,
				unsigned long insn_off)
{
	return arch_create_orc_entry(u_sec, ip_relasec, idx,
				      insn_sec, insn_off, &empty);
}

static const char *reg_name(unsigned int reg)
{
	switch (reg) {
	case ORC_REG_PREV_SP:
		return "prevsp";
	case ORC_REG_DX:
		return "dx";
	case ORC_REG_DI:
		return "di";
	case ORC_REG_BP:
		return "bp";
	case ORC_REG_SP:
		return "sp";
	case ORC_REG_R10:
		return "r10";
	case ORC_REG_R13:
		return "r13";
	case ORC_REG_BP_INDIRECT:
		return "bp(ind)";
	case ORC_REG_SP_INDIRECT:
		return "sp(ind)";
	default:
		return "?";
	}
}

static const char *orc_type_name(unsigned int type)
{
	switch (type) {
	case ORC_TYPE_CALL:
		return "call";
	case ORC_TYPE_REGS:
		return "regs";
	case ORC_TYPE_REGS_IRET:
		return "iret";
	default:
		return "?";
	}
}

static void print_reg(unsigned int reg, int offset)
{
	if (reg == ORC_REG_BP_INDIRECT)
		printf("(bp%+d)", offset);
	else if (reg == ORC_REG_SP_INDIRECT)
		printf("(sp%+d)", offset);
	else if (reg == ORC_REG_UNDEFINED)
		printf("(und)");
	else
		printf("%s%+d", reg_name(reg), offset);
}

void arch_print_reg(struct orc_entry orc)
{
	printf(" sp:");

	print_reg(orc.sp_reg, orc.sp_offset);

	printf(" bp:");

	print_reg(orc.bp_reg, orc.bp_offset);

	printf(" type:%s end:%d\n",
	       orc_type_name(orc.type), orc.end);
}
