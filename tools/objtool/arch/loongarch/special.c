// SPDX-License-Identifier: GPL-2.0-or-later
#include <stdlib.h>
#include <asm/inst.h>
#include <string.h>

#include "../../special.h"
#include "../../builtin.h"
#include "../../check.h"
#include "../../warn.h"

/* Architecture specific "noreturn" function */
const char * const arch_noreturns[] = {
	"die",
};
int arch_noreturns_size = ARRAY_SIZE(arch_noreturns);

void arch_handle_alternative(unsigned short feature, struct special_alt *alt)
{
}

void arch_mark_func_jump_tables(struct objtool_file *file,
				    struct symbol *func)
{
	struct instruction *insn;
	struct rela *rela;
	union loongarch_instruction code;

	func_for_each_insn_all(file, func, insn) {
		code = *(union loongarch_instruction *)(insn->sec->data->d_buf + insn->offset);

		if (code.reg1i20_format.opcode != pcaddu12i_op)
			continue;

		rela = find_rela_by_dest(insn->sec, insn->offset);

		if (!rela || (strncmp(rela->sym->name, ".L", 2) &&
				    strncmp(rela->sym->name, "jumptable", 9)))
			continue;

		rela = find_rela_by_dest(rela->sym->sec, rela->sym->offset);

		if (!rela || strncmp(rela->sym->name, ".L", 2))
			continue;

		rela->jump_table_start = true;
	}
}

static int dynamic_add_jump_table(struct objtool_file *file,
			    struct instruction *insn, struct rela *table)
{
	struct rela *reloc = table;
	struct instruction *dest_insn;
	struct alternative *alt;
	struct symbol *pfunc = insn->func->pfunc;

	list_for_each_entry_from(reloc, &table->rela_sec->rela_list, list) {

		/* Check for the end of the table: */
		if (reloc != table && reloc->jump_table_start)
			break;

		/* Detect function pointers from contiguous objects: */
		if (reloc->sym->sec == pfunc->sec &&
		    reloc->sym->offset == pfunc->offset)
			break;

		dest_insn = find_insn(file, reloc->sym->sec, reloc->sym->offset);
		if (!dest_insn)
			break;

		/* Make sure the destination is in the same function: */
		if (!dest_insn->func || dest_insn->func->pfunc != pfunc)
			break;

		alt = malloc(sizeof(*alt));
		if (!alt) {
			WARN("malloc failed");
			return -1;
		}

		alt->insn = dest_insn;
		list_add_tail(&alt->list, &insn->alts);
	}

	return 0;
}

/*
 * Switch jump could be thought of as having two stagestwo stages:
 * Stage1: la (la.pcrel or la.abs)
 *   1) lui12i.w ori lu32i.d lu52i.d
 *   2) pcaddu12i addi.d
 * Stage2: ld (addr + idx)
 *   1) alsl.d ldptr.d/ld.d
 *   2) (alsl.d) ldx.d
 *   3) add.d ldptr.d/ld.d
 *    ...
 */

int arch_dynamic_add_jump_table_alts(struct list_head *p_orbit_list, struct objtool_file *file,
				 struct symbol *func, struct instruction *insn)
{
	int dest_reg;
	int stage = 0, instack = 0;
	int i = 0;
	int lu52id = 0, ldptrd = 0, ldd = 0;
	struct instruction *orbit, *n;
	struct rela *reloc, *rodata_reloc;
	union loongarch_instruction code;

	if (list_empty(p_orbit_list)) {
		WARN_FUNC("BUG: why do I have no insn track?", insn->sec, insn->offset);
		return 1;
	}

	if (func_last_orbit(p_orbit_list) != insn) {
		WARN_FUNC("BUG: insn is not expected.", insn->sec, insn->offset);
		return 1;
	}

	n = list_next_entry(insn, orbit_node);
	code = *(union loongarch_instruction *)(n->sec->data->d_buf + n->offset);
	if (code.reg2i12_format.opcode == addid_op &&
	      code.reg2i12_format.rj == CFI_SP &&
	      code.reg2i12_format.rd == CFI_SP) {
		insn->type = INSN_RETURN;
		return 0;
	}

	code = *(union loongarch_instruction *)(insn->sec->data->d_buf + insn->offset);
	if (code.reg2i16_format.opcode != jirl_op) {
		WARN_FUNC("BUG: first insn track is not expected.", insn->sec, insn->offset);
		return 1;
	}

	dest_reg = code.reg2i16_format.rj;

	list_for_each_entry(orbit, p_orbit_list, orbit_node) {
		code = *(union loongarch_instruction *)(orbit->sec->data->d_buf + orbit->offset);
		switch (i) {
		case 1:
			if (code.reg2i12_format.opcode != lu52id_op ||
				code.reg2i12_format.rj != dest_reg ||
				code.reg2i12_format.rd != dest_reg)
				i = 5;
			break;
		case 2:
			if (code.reg1i20_format.opcode != lu32id_op ||
				code.reg1i20_format.rd != dest_reg)
				i = 5;
			break;
		case 3:
			if (code.reg2ui12_format.opcode != ori_op ||
				code.reg2ui12_format.rj != dest_reg ||
				code.reg2ui12_format.rd != dest_reg)
				i = 5;
			break;
		case 4:
			if (code.reg1i20_format.opcode != lu12iw_op ||
				code.reg1i20_format.rd != dest_reg) {
				i = 5;
				break;
			}
			reloc = find_rela_by_dest(orbit->sec, orbit->offset);
			if (!reloc) {
				WARN_FUNC("BUG: lu12i.w has no reloc.", orbit->sec, orbit->offset);
				return 1;
			}
			n = find_insn(file, reloc->sym->sec, reloc->sym->offset);
			if (!n) {
				/* Global symbol */
				insn->type = INSN_RETURN;
				break;
			}
			insn->jump_dest = n;
			return 0;
		default:
			/* i == 0, skip jirl insn. */
			break;
		}
		if (i == 5)
			break;

		i++;
	}

	list_for_each_entry(orbit, p_orbit_list, orbit_node) {
		i++;
		code = *(union loongarch_instruction *)(orbit->sec->data->d_buf + orbit->offset);
		if (instack) {
			if ((code.reg2i12_format.opcode == std_op || code.reg2i12_format.opcode == stptrd_op) &&
			     code.reg2i12_format.simmediate == instack && code.reg2i12_format.rj == CFI_SP) {
				dest_reg = code.reg2i12_format.rd;
				instack = 0;
			}
			continue;
		}
		if ((code.reg2i12_format.opcode == ldd_op || code.reg2i12_format.opcode == ldptrd_op) &&
		     code.reg2i12_format.rd == dest_reg && code.reg2i12_format.rj == CFI_SP) {
			instack = code.reg2i12_format.simmediate;
			continue;
		}

		switch (stage) {
		case 0:
			/* alsl.d */
			if (code.reg3sa2_format.opcode == alsld_op &&
			      code.reg3sa2_format.rd == dest_reg) {
				dest_reg = code.reg3sa2_format.rk;
				stage = 1;
			}
			/* ldptr.d */
			if (code.reg2i14_format.opcode == ldptrd_op &&
			      code.reg2i14_format.rd == dest_reg) {
				dest_reg = code.reg2i14_format.rj;
				ldptrd = i;
			}
			/* ld.d */
			if (code.reg2i12_format.opcode == ldd_op &&
				code.reg2i12_format.rd == dest_reg) {
				dest_reg = code.reg2i12_format.rj;
				ldd = i;
			}
			/* ldx.d */
			if (code.reg3_format.opcode == ldxd_op &&
			      code.reg3_format.rd == dest_reg) {
				dest_reg = code.reg3_format.rj;
				stage = 1;
			}
			/* add.d */
			if ((ldptrd || ldd) &&
			      code.reg3_format.opcode == addd_op &&
			      code.reg3_format.rd == dest_reg) {
				dest_reg = code.reg3_format.rj;
				stage = 1;
			}
			/* ~ lu52i.d */
			if (code.reg2i12_format.opcode == lu52id_op &&
			      code.reg2i12_format.rj == dest_reg &&
			      code.reg2i12_format.rd == dest_reg) {
				insn->type = INSN_RETURN;
				return 0;
			}
			/* ~ addi.d */
			if (code.reg2i12_format.opcode == addid_op &&
			      code.reg2i12_format.rd == dest_reg) {
				insn->type = INSN_RETURN;
				return 0;
			}
			break;
		case 1:
			/* pcaddu12i */
			if (!lu52id &&
			      code.reg1i20_format.opcode == pcaddu12i_op &&
			      code.reg1i20_format.rd == dest_reg) {
				reloc = find_rela_by_dest(orbit->sec, orbit->offset);
				if (!reloc) {
					WARN_FUNC("BUG: pcaddu12i has no reloc.", orbit->sec, orbit->offset);
					return 1;
				}

				if (!strncmp(reloc->sym->name, ".L", 2) ||
				      !strncmp(reloc->sym->name, "jumptable", 9)) {
					rodata_reloc = find_rela_by_dest(reloc->sym->sec, reloc->sym->offset);

					if (!rodata_reloc) {
						WARN_FUNC("BUG: rodata has no reloc.", orbit->sec, orbit->offset);
						return 1;
					}

					insn->jump_table = rodata_reloc;

					return dynamic_add_jump_table(file, insn, insn->jump_table);
				} else if (reloc->sym->bind == STB_LOCAL && reloc->sym->type == STT_OBJECT) {
					insn->type = INSN_RETURN;
					return 0;
				} else if (reloc->sym->bind == STB_GLOBAL) {
					insn->type = INSN_RETURN;
					return 0;
				}

				WARN("BUG here");
				return 1;
			}
			if (lu52id &&
			      code.reg1i20_format.opcode == lu12iw_op &&
			      code.reg1i20_format.rd == dest_reg) {
				reloc = find_rela_by_dest(orbit->sec, orbit->offset);
				if (!reloc) {
					WARN_FUNC("BUG: lu12i.w has no reloc.", orbit->sec, orbit->offset);
					return 1;
				}

				if (!strncmp(reloc->sym->name, ".L", 2) ||
				      !strncmp(reloc->sym->name, "jumptable", 9)) {
					rodata_reloc = find_rela_by_dest(reloc->sym->sec, reloc->sym->offset);

					if (!rodata_reloc) {
						WARN_FUNC("BUG: rodata has no reloc.", orbit->sec, orbit->offset);
						return 1;
					}

					insn->jump_table = rodata_reloc;

					return dynamic_add_jump_table(file, insn, insn->jump_table);
				} else if (reloc->sym->bind == STB_LOCAL && reloc->sym->type == STT_OBJECT) {
					insn->type = INSN_RETURN;
					return 0;
				} else if (reloc->sym->bind == STB_GLOBAL) {
					insn->type = INSN_RETURN;
					return 0;
				}

				WARN("BUG here");
				return 1;
			}
			/* addi.d */
			if (code.reg2i12_format.opcode == addid_op &&
			      code.reg2i12_format.rd == dest_reg) {
				dest_reg = code.reg2i12_format.rj;
			}
			/* lu52i.d */
			if (code.reg2i12_format.opcode == lu52id_op &&
			      code.reg2i12_format.rj == dest_reg &&
			      code.reg2i12_format.rd == dest_reg) {
				lu52id = i;
			}
			break;
		default:
			WARN_FUNC("BUG: why am I here?", orbit->sec, orbit->offset);
			return 1;
		}
	}

	insn->type = INSN_RETURN;
	return 0;
}
