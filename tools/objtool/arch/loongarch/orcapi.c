// SPDX-License-Identifier: GPL-2.0-or-later
#include <string.h>
#include <stdlib.h>

#include "../../orc.h"
#include "../../warn.h"

static struct orc_entry empty = {
	.sp_reg = ORC_REG_UNDEFINED,
	.type = ORC_TYPE_CALL,
};

int create_orc(struct objtool_file *file)
{
	struct instruction *insn;

	for_each_insn(file, insn) {
		struct orc_entry *orc = &insn->orc;
		struct cfi_reg *cfa = &insn->state.cfa;
		struct cfi_reg *fp = &insn->state.regs[CFI_FP];
		struct cfi_reg *ra = &insn->state.regs[CFI_RA];

		orc->end = insn->state.end;

		if (cfa->base == CFI_UNDEFINED) {
			orc->sp_reg = ORC_REG_UNDEFINED;
			continue;
		}

		switch (cfa->base) {
		case CFI_SP:
			orc->sp_reg = ORC_REG_SP;
			break;
		case CFI_FP:
			orc->sp_reg = ORC_REG_FP;
			break;
		default:
			WARN_FUNC("unknown CFA base reg %d",
				  insn->sec, insn->offset, cfa->base);
			return -1;
		}

		switch (fp->base) {
		case CFI_UNDEFINED:
			orc->fp_reg = ORC_REG_UNDEFINED;
			orc->fp_offset = 0;
			break;
		case CFI_CFA:
			orc->fp_reg = ORC_REG_PREV_SP;
			orc->fp_offset = fp->offset;
			break;
		default:
			WARN_FUNC("unknown FP base reg %d",
					insn->sec, insn->offset, fp->base);
		}

		switch (ra->base) {
		case CFI_UNDEFINED:
			orc->ra_reg = ORC_REG_UNDEFINED;
			orc->ra_offset = 0;
			break;
		case CFI_CFA:
			orc->ra_reg = ORC_REG_PREV_SP;
			orc->ra_offset = ra->offset;
			break;
		default:
			WARN_FUNC("unknown RA base reg %d",
				  insn->sec, insn->offset, ra->base);
		}

		orc->sp_offset = cfa->offset;
		orc->type = insn->state.type;
	}

	return 0;
}

int arch_create_orc_entry(struct section *u_sec, struct section *ip_relasec,
				unsigned int idx, struct section *insn_sec,
				unsigned long insn_off, struct orc_entry *o)
{
	struct orc_entry *orc;
	struct rela *rela, *next;

	/* populate ORC data */
	orc = (struct orc_entry *)u_sec->data->d_buf + idx;
	memcpy(orc, o, sizeof(*orc));

	/* populate rela for ip
	 * Since loongarch64 has no PC32 at present, fake as follows,
	 * R_LARCH_ADD32 ip
	 * R_LARCH_SUB32 orc
	 */
	rela = malloc(sizeof(*rela) * 2);
	if (!rela) {
		perror("malloc");
		return -1;
	}
	memset(rela, 0, sizeof(*rela) * 2);

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

	next = rela + 1;
	rela->type = R_LARCH_ADD32;
	rela->offset = idx * sizeof(int);
	rela->next = next;
	next->type = R_LARCH_SUB32;
	next->offset = idx * sizeof(int);
	next->sym = ip_relasec->base->sym;
	next->addend = idx * sizeof(int);

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
	case ORC_REG_SP:
		return "sp";
	case ORC_REG_FP:
		return "fp";
	case ORC_REG_PREV_SP:
		return "prevsp";
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
	default:
		return "?";
	}
}

static void print_reg(unsigned int reg, int offset)
{
	if (reg == ORC_REG_UNDEFINED)
		printf(" (und) ");
	else
		printf("%s + %3d", reg_name(reg), offset);
}

void arch_print_reg(struct orc_entry orc)
{
	printf(" sp:");

	print_reg(orc.sp_reg, orc.sp_offset);

	printf(" fp:");

	print_reg(orc.fp_reg, orc.fp_offset);

	printf(" ra:");
	print_reg(orc.ra_reg, orc.ra_offset);

	printf(" type:%s end:%d\n",
	       orc_type_name(orc.type), orc.end);
}
