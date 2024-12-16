/*
 * Copyright (C) 2015-2017 Josh Poimboeuf <jpoimboe@redhat.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <stdlib.h>

#include "builtin.h"
#include "cfi.h"
#include "arch.h"
#include "check.h"
#include "special.h"
#include "warn.h"

#include <linux/hashtable.h>
#include <linux/kernel.h>

#define FAKE_JUMP_OFFSET -1

const char *objname;
struct cfi_state initial_func_cfi;
struct list_head orbit_list;

struct instruction *find_insn(struct objtool_file *file,
			      struct section *sec, unsigned long offset)
{
	struct instruction *insn;

	hash_for_each_possible(file->insn_hash, insn, hash, offset)
		if (insn->sec == sec && insn->offset == offset)
			return insn;

	return NULL;
}

struct instruction *next_insn_same_sec(struct objtool_file *file,
					      struct instruction *insn)
{
	struct instruction *next = list_next_entry(insn, list);

	if (!next || &next->list == &file->insn_list || next->sec != insn->sec)
		return NULL;

	return next;
}

struct instruction *next_insn_same_func(struct objtool_file *file,
					       struct instruction *insn)
{
	struct instruction *next = list_next_entry(insn, list);
	struct symbol *func = insn->func;

	if (!func)
		return NULL;

	if (&next->list != &file->insn_list && next->func == func)
		return next;

	/* Check if we're already in the subfunction: */
	if (func == func->cfunc)
		return NULL;

	/* Move to the subfunction: */
	return find_insn(file, func->cfunc->sec, func->cfunc->offset);
}

/*
 * Check if the function has been manually whitelisted with the
 * STACK_FRAME_NON_STANDARD macro, or if it should be automatically whitelisted
 * due to its use of a context switching instruction.
 */
static bool ignore_func(struct objtool_file *file, struct symbol *func)
{
	struct rela *rela;

	/* check for STACK_FRAME_NON_STANDARD */
	if (file->whitelist && file->whitelist->rela)
		list_for_each_entry(rela, &file->whitelist->rela->rela_list, list) {
			if (rela->sym->type == STT_SECTION &&
			    rela->sym->sec == func->sec &&
			    rela->addend == func->offset)
				return true;
			if (rela->sym->type == STT_FUNC && rela->sym == func)
				return true;
		}

	return false;
}

/*
 * This checks to see if the given function is a "noreturn" function.
 *
 *
 * For global functions which are outside the scope of this object file, we
 * have to keep a manual list of them.
 *
 * For local functions, we have to detect them manually by simply looking for
 * the lack of a return instruction.
 *
 * Returns:
 *  -1: error
 *   0: no dead end
 *   1: dead end
 */
static int __dead_end_function(struct objtool_file *file, struct symbol *func,
			       int recursion)
{
	int i;
	struct instruction *insn;
	bool empty = true;

	/*
	 * Unfortunately these have to be hard coded because the noreturn
	 * attribute isn't provided in ELF data.
	 */
	static const char * const global_noreturns[] = {
		"__stack_chk_fail",
		"panic",
		"do_exit",
		"do_task_dead",
		"__module_put_and_exit",
		"complete_and_exit",
		"kvm_spurious_fault",
		"__reiserfs_panic",
		"lbug_with_loc",
		"fortify_panic",
		"usercopy_abort",
		"machine_real_restart",
		"rewind_stack_do_exit",
	};

	if (!func)
		return 0;

	if (func->bind == STB_WEAK)
		return 0;

	if (func->bind == STB_GLOBAL) {
		for (i = 0; i < ARRAY_SIZE(global_noreturns); i++)
			if (!strcmp(func->name, global_noreturns[i]))
				return 1;

		for (i = 0; i < arch_noreturns_size; i++)
			if (!strcmp(func->name, arch_noreturns[i]))
				return 1;
	}

	if (!func->len)
		return 0;

	insn = find_insn(file, func->sec, func->offset);

	if (!insn->func)
		return 0;

	func_for_each_insn_all(file, func, insn) {
		empty = false;

		if (insn->type == INSN_RETURN)
			return 0;
	}

	if (empty)
		return 0;

	/*
	 * A function can have a sibling call instead of a return.  In that
	 * case, the function's dead-end status depends on whether the target
	 * of the sibling call returns.
	 */
	func_for_each_insn_all(file, func, insn) {
		if (insn->type == INSN_JUMP_UNCONDITIONAL) {
			struct instruction *dest = insn->jump_dest;

			if (!dest)
				/* sibling call to another file */
				return 0;

			if (dest->func && dest->func->pfunc != insn->func->pfunc) {

				/* local sibling call */
				if (recursion == 5) {
					/*
					 * Infinite recursion: two functions
					 * have sibling calls to each other.
					 * This is a very rare case.  It means
					 * they aren't dead ends.
					 */
					return 0;
				}

				return __dead_end_function(file, dest->func,
							   recursion + 1);
			}
		}

		if (insn->type == INSN_JUMP_DYNAMIC && list_empty(&insn->alts))
			/* sibling call */
			return 0;
	}

	return 1;
}

static int dead_end_function(struct objtool_file *file, struct symbol *func)
{
	return __dead_end_function(file, func, 0);
}

static void clear_insn_state(struct insn_state *state)
{
	int i;

	memset(state, 0, sizeof(*state));
	state->cfa.base = CFI_UNDEFINED;
	for (i = 0; i < CFI_NUM_REGS; i++) {
		state->regs[i].base = CFI_UNDEFINED;
		state->vals[i].base = CFI_UNDEFINED;
	}
	state->drap_reg = CFI_UNDEFINED;
	state->drap_offset = -1;
}

/*
 * Call the arch-specific instruction decoder for all the instructions and add
 * them to the global instruction list.
 */
static int decode_instructions(struct objtool_file *file)
{
	struct section *sec;
	struct symbol *func;
	unsigned long offset;
	struct instruction *insn;
	int ret;

	for_each_sec(file, sec) {

		if (!(sec->sh.sh_flags & SHF_EXECINSTR))
			continue;

		if (strcmp(sec->name, ".altinstr_replacement") &&
		    strcmp(sec->name, ".altinstr_aux") &&
		    strncmp(sec->name, ".discard.", 9))
			sec->text = true;

		for (offset = 0; offset < sec->len; offset += insn->len) {
			insn = malloc(sizeof(*insn));
			if (!insn) {
				WARN("malloc failed");
				return -1;
			}
			memset(insn, 0, sizeof(*insn));
			INIT_LIST_HEAD(&insn->alts);
			clear_insn_state(&insn->state);

			insn->sec = sec;
			insn->offset = offset;

			ret = arch_decode_instruction(file->elf, sec, offset,
						      sec->len - offset,
						      &insn->len, &insn->type,
						      &insn->immediate,
						      &insn->stack_op);
			if (ret)
				goto err;

			if (!insn->type || insn->type > INSN_LAST) {
				WARN_FUNC("invalid instruction type %d",
					  insn->sec, insn->offset, insn->type);
				ret = -1;
				goto err;
			}

			hash_add(file->insn_hash, &insn->hash, insn->offset);
			list_add_tail(&insn->list, &file->insn_list);
		}

		list_for_each_entry(func, &sec->symbol_list, list) {
			if (func->type != STT_FUNC)
				continue;

			if (!find_insn(file, sec, func->offset)) {
				WARN("%s(): can't find starting instruction",
				     func->name);
				return -1;
			}

			func_for_each_insn(file, func, insn)
				if (!insn->func)
					insn->func = func;
		}
	}

	return 0;

err:
	free(insn);
	return ret;
}

/*
 * Mark "ud2" instructions and manually annotated dead ends.
 */
static int add_dead_ends(struct objtool_file *file)
{
	struct section *sec;
	struct rela *rela;
	struct instruction *insn;
	bool found;

	/*
	 * By default, "ud2" is a dead end unless otherwise annotated, because
	 * GCC 7 inserts it for certain divide-by-zero cases.
	 */
	for_each_insn(file, insn)
		if (insn->type == INSN_BUG)
			insn->dead_end = true;

	/*
	 * Check for manually annotated dead ends.
	 */
	sec = find_section_by_name(file->elf, ".rela.discard.unreachable");
	if (!sec)
		goto reachable;

	list_for_each_entry(rela, &sec->rela_list, list) {
#ifdef __loongarch__
		if (!rela->next || rela->type != R_LARCH_ADD32 || rela->next->type != R_LARCH_SUB32) {
#else
		if (rela->sym->type != STT_SECTION) {
#endif
			WARN("unexpected relocation symbol type in %s", sec->name);
			return -1;
		}
#ifdef __loongarch__
		insn = find_insn(file, rela->sym->sec, rela->sym->offset);
#else
		insn = find_insn(file, rela->sym->sec, rela->addend);
#endif
		if (insn)
			insn = list_prev_entry(insn, list);
#ifdef __loongarch__
		else if (rela->sym->offset == rela->sym->sec->len) {
#else
		else if (rela->addend == rela->sym->sec->len) {
#endif
			found = false;
			list_for_each_entry_reverse(insn, &file->insn_list, list) {
				if (insn->sec == rela->sym->sec) {
					found = true;
					break;
				}
			}

			if (!found) {
				WARN("can't find unreachable insn at %s+0x%x",
				     rela->sym->sec->name, rela->addend);
				return -1;
			}
		} else {
			WARN("can't find unreachable insn at %s+0x%x",
			     rela->sym->sec->name, rela->addend);
			return -1;
		}

		insn->dead_end = true;
	}

reachable:
	/*
	 * These manually annotated reachable checks are needed for GCC 4.4,
	 * where the Linux unreachable() macro isn't supported.  In that case
	 * GCC doesn't know the "ud2" is fatal, so it generates code as if it's
	 * not a dead end.
	 */
	sec = find_section_by_name(file->elf, ".rela.discard.reachable");
	if (!sec)
		return 0;

	list_for_each_entry(rela, &sec->rela_list, list) {
#ifdef __loongarch__
		if (!rela->next || rela->type != R_LARCH_ADD32 || rela->next->type != R_LARCH_SUB32) {
#else
		if (rela->sym->type != STT_SECTION) {
#endif
			WARN("unexpected relocation symbol type in %s", sec->name);
			return -1;
		}
#ifdef __loongarch__
		insn = find_insn(file, rela->sym->sec, rela->sym->offset);
#else
		insn = find_insn(file, rela->sym->sec, rela->addend);
#endif
		if (insn)
			insn = list_prev_entry(insn, list);
#ifdef __loongarch__
		else if (rela->sym->offset == rela->sym->sec->len) {
#else
		else if (rela->addend == rela->sym->sec->len) {
#endif
			found = false;
			list_for_each_entry_reverse(insn, &file->insn_list, list) {
				if (insn->sec == rela->sym->sec) {
					found = true;
					break;
				}
			}

			if (!found) {
				WARN("can't find reachable insn at %s+0x%x",
				     rela->sym->sec->name, rela->addend);
				return -1;
			}
		} else {
			WARN("can't find reachable insn at %s+0x%x",
			     rela->sym->sec->name, rela->addend);
			return -1;
		}

		insn->dead_end = false;
	}

	return 0;
}

/*
 * Mark not sibling call instructions.
 */
static int add_not_sibling_call(struct objtool_file *file)
{
	struct section *sec;
	struct rela *rela;
	struct instruction *insn;

	sec = find_section_by_name(file->elf, ".rela.discard.not_sibling_call");
	if (!sec)
		return 0;

	list_for_each_entry(rela, &sec->rela_list, list) {
#ifdef __loongarch__
		if (!rela->next || rela->type != R_LARCH_ADD32 || rela->next->type != R_LARCH_SUB32) {
#else
		if (rela->sym->type != STT_SECTION) {
#endif
			WARN("unexpected relocation symbol type in %s", sec->name);
			return -1;
		}

#ifdef __loongarch__
		insn = find_insn(file, rela->sym->sec, rela->sym->offset);
#else
		insn = find_insn(file, rela->sym->sec, rela->addend);
#endif

		if (!insn) {
			WARN("unexpected relocation symbol offset at %s: %lx",
				rela->sym->sec->name, rela->sym->offset);
			return -1;
		}
		insn->not_sibling_call = true;
	}

	return 0;
}

/*
 * Warnings shouldn't be reported for ignored functions.
 */
static void add_ignores(struct objtool_file *file)
{
	struct instruction *insn;
	struct section *sec;
	struct symbol *func;

	for_each_sec(file, sec) {
		list_for_each_entry(func, &sec->symbol_list, list) {
			if (func->type != STT_FUNC)
				continue;

			if (!ignore_func(file, func))
				continue;

			func_for_each_insn_all(file, func, insn)
				insn->ignore = true;
		}
	}
}

/*
 * FIXME: For now, just ignore any alternatives which add retpolines.  This is
 * a temporary hack, as it doesn't allow ORC to unwind from inside a retpoline.
 * But it at least allows objtool to understand the control flow *around* the
 * retpoline.
 */
static int add_nospec_ignores(struct objtool_file *file)
{
	struct section *sec;
	struct rela *rela;
	struct instruction *insn;

	sec = find_section_by_name(file->elf, ".rela.discard.nospec");
	if (!sec)
		return 0;

	list_for_each_entry(rela, &sec->rela_list, list) {
		if (rela->sym->type != STT_SECTION) {
			WARN("unexpected relocation symbol type in %s", sec->name);
			return -1;
		}

		insn = find_insn(file, rela->sym->sec, rela->addend);
		if (!insn) {
			WARN("bad .discard.nospec entry");
			return -1;
		}

		insn->ignore_alts = true;
	}

	return 0;
}

/*
 * Find the destination instructions for all jumps.
 */
static int add_jump_destinations(struct objtool_file *file)
{
	struct instruction *insn;
	struct rela *rela;
	struct section *dest_sec;
	unsigned long dest_off;

	for_each_insn(file, insn) {
		if (insn->type != INSN_JUMP_CONDITIONAL &&
		    insn->type != INSN_JUMP_UNCONDITIONAL)
			continue;

		if (insn->offset == FAKE_JUMP_OFFSET)
			continue;

		rela = find_rela_by_dest_range(insn->sec, insn->offset,
					       insn->len);
		if (!rela) {
			dest_sec = insn->sec;
			dest_off = arch_jump_destination(insn);
		} else if (rela->sym->type == STT_SECTION) {
			dest_sec = rela->sym->sec;
			dest_off = arch_dest_rela_offset(rela->addend);
#ifdef __loongarch__
		} else if (!strncmp(rela->sym->name, ".L", 2)) {
			/* '.L' in LA compiler means target name prefix */
			dest_sec = rela->sym->sec;
			dest_off = rela->sym->offset + rela->addend;
			/* In asm file, jump dest maybe a label without '.L' prefix */
		} else if (rela->sym->type == STT_NOTYPE && rela->sym->bind == STB_LOCAL) {
			dest_sec = rela->sym->sec;
			dest_off = rela->sym->offset + rela->addend;
#endif
		} else if (rela->sym->sec->idx) {
			dest_sec = rela->sym->sec;
			dest_off = rela->sym->sym.st_value +
				   arch_dest_rela_offset(rela->addend);
		} else if (strstr(rela->sym->name, "_indirect_thunk_")) {
			/*
			 * Retpoline jumps are really dynamic jumps in
			 * disguise, so convert them accordingly.
			 */
			insn->type = INSN_JUMP_DYNAMIC;
			insn->retpoline_safe = true;
			continue;
		} else {
			/* sibling call */
			insn->jump_dest = 0;
			continue;
		}

		insn->jump_dest = find_insn(file, dest_sec, dest_off);
		if (!insn->jump_dest) {

			/*
			 * This is a special case where an alt instruction
			 * jumps past the end of the section.  These are
			 * handled later in handle_group_alt().
			 */
			if (!strcmp(insn->sec->name, ".altinstr_replacement"))
				continue;

			WARN_FUNC("can't find jump dest instruction at %s+0x%lx",
				  insn->sec, insn->offset, dest_sec->name,
				  dest_off);
			return -1;
		}

		/*
		 * For GCC 8+, create parent/child links for any cold
		 * subfunctions.  This is _mostly_ redundant with a similar
		 * initialization in read_symbols().
		 *
		 * If a function has aliases, we want the *first* such function
		 * in the symbol table to be the subfunction's parent.  In that
		 * case we overwrite the initialization done in read_symbols().
		 *
		 * However this code can't completely replace the
		 * read_symbols() code because this doesn't detect the case
		 * where the parent function's only reference to a subfunction
		 * is through a switch table.
		 */
		if (insn->func && insn->jump_dest->func &&
		    insn->func != insn->jump_dest->func &&
		    !strstr(insn->func->name, ".cold.") &&
		    strstr(insn->jump_dest->func->name, ".cold.")) {
			insn->func->cfunc = insn->jump_dest->func;
			insn->jump_dest->func->pfunc = insn->func;
		}
	}

	return 0;
}

/*
 * Find the destination instructions for all calls.
 */
static int add_call_destinations(struct objtool_file *file)
{
	struct instruction *insn;
	unsigned long dest_off;
	struct rela *rela;

	for_each_insn(file, insn) {
		if (insn->type != INSN_CALL)
			continue;

		rela = find_rela_by_dest_range(insn->sec, insn->offset,
					       insn->len);
		if (!rela) {
			dest_off = arch_jump_destination(insn);
			insn->call_dest = find_symbol_by_offset(insn->sec,
								dest_off);

			if (!insn->call_dest && !insn->ignore) {
				WARN_FUNC("unsupported intra-function call",
					  insn->sec, insn->offset);
				if (retpoline)
					WARN("If this is a retpoline, please patch it in with alternatives and annotate it with ANNOTATE_NOSPEC_ALTERNATIVE.");
				return -1;
			}

		} else if (rela->sym->type == STT_SECTION) {
			dest_off = arch_dest_rela_offset(rela->addend);
			insn->call_dest = find_symbol_by_offset(rela->sym->sec,
								dest_off);
			if (!insn->call_dest ||
			    insn->call_dest->type != STT_FUNC) {
				WARN_FUNC("can't find call dest symbol at %s+0x%lx",
					  insn->sec, insn->offset,
					  rela->sym->sec->name,
					  dest_off);
				return -1;
			}
		} else
			insn->call_dest = rela->sym;
	}

	return 0;
}

/*
 * The .alternatives section requires some extra special care, over and above
 * what other special sections require:
 *
 * 1. Because alternatives are patched in-place, we need to insert a fake jump
 *    instruction at the end so that validate_branch() skips all the original
 *    replaced instructions when validating the new instruction path.
 *
 * 2. An added wrinkle is that the new instruction length might be zero.  In
 *    that case the old instructions are replaced with noops.  We simulate that
 *    by creating a fake jump as the only new instruction.
 *
 * 3. In some cases, the alternative section includes an instruction which
 *    conditionally jumps to the _end_ of the entry.  We have to modify these
 *    jumps' destinations to point back to .text rather than the end of the
 *    entry in .altinstr_replacement.
 *
 * 4. It has been requested that we don't validate the !POPCNT feature path
 *    which is a "very very small percentage of machines".
 */
static int handle_group_alt(struct objtool_file *file,
			    struct special_alt *special_alt,
			    struct instruction *orig_insn,
			    struct instruction **new_insn)
{
	struct instruction *last_orig_insn, *last_new_insn, *insn, *fake_jump = NULL;
	unsigned long dest_off;

	last_orig_insn = NULL;
	insn = orig_insn;
	sec_for_each_insn_from(file, insn) {
		if (insn->offset >= special_alt->orig_off + special_alt->orig_len)
			break;

		if (special_alt->skip_orig)
			insn->type = INSN_NOP;

		insn->alt_group = true;
		last_orig_insn = insn;
	}

	if (next_insn_same_sec(file, last_orig_insn)) {
		fake_jump = malloc(sizeof(*fake_jump));
		if (!fake_jump) {
			WARN("malloc failed");
			return -1;
		}
		memset(fake_jump, 0, sizeof(*fake_jump));
		INIT_LIST_HEAD(&fake_jump->alts);
		clear_insn_state(&fake_jump->state);

		fake_jump->sec = special_alt->new_sec;
		fake_jump->offset = FAKE_JUMP_OFFSET;
		fake_jump->type = INSN_JUMP_UNCONDITIONAL;
		fake_jump->jump_dest = list_next_entry(last_orig_insn, list);
		fake_jump->func = orig_insn->func;
	}

	if (!special_alt->new_len) {
		if (!fake_jump) {
			WARN("%s: empty alternative at end of section",
			     special_alt->orig_sec->name);
			return -1;
		}

		*new_insn = fake_jump;
		return 0;
	}

	last_new_insn = NULL;
	insn = *new_insn;
	sec_for_each_insn_from(file, insn) {
		if (insn->offset >= special_alt->new_off + special_alt->new_len)
			break;

		last_new_insn = insn;

		insn->ignore = orig_insn->ignore_alts;

		if (insn->type != INSN_JUMP_CONDITIONAL &&
		    insn->type != INSN_JUMP_UNCONDITIONAL)
			continue;

		if (!insn->immediate)
			continue;

		dest_off = arch_jump_destination(insn);
		if (dest_off == special_alt->new_off + special_alt->new_len) {
			if (!fake_jump) {
				WARN("%s: alternative jump to end of section",
				     special_alt->orig_sec->name);
				return -1;
			}
			insn->jump_dest = fake_jump;
		}

		if (!insn->jump_dest) {
			WARN_FUNC("can't find alternative jump destination",
				  insn->sec, insn->offset);
			return -1;
		}
	}

	if (!last_new_insn) {
		WARN_FUNC("can't find last new alternative instruction",
			  special_alt->new_sec, special_alt->new_off);
		return -1;
	}

	if (fake_jump)
		list_add(&fake_jump->list, &last_new_insn->list);

	return 0;
}

/*
 * A jump table entry can either convert a nop to a jump or a jump to a nop.
 * If the original instruction is a jump, make the alt entry an effective nop
 * by just skipping the original instruction.
 */
static int handle_jump_alt(struct objtool_file *file,
			   struct special_alt *special_alt,
			   struct instruction *orig_insn,
			   struct instruction **new_insn)
{
	if (orig_insn->type == INSN_NOP)
		return 0;

	if (orig_insn->type != INSN_JUMP_UNCONDITIONAL) {
		WARN_FUNC("unsupported instruction at jump label",
			  orig_insn->sec, orig_insn->offset);
		return -1;
	}

	*new_insn = list_next_entry(orig_insn, list);
	return 0;
}

/*
 * Read all the special sections which have alternate instructions which can be
 * patched in or redirected to at runtime.  Each instruction having alternate
 * instruction(s) has them added to its insn->alts list, which will be
 * traversed in validate_branch().
 */
static int add_special_section_alts(struct objtool_file *file)
{
	struct list_head special_alts;
	struct instruction *orig_insn, *new_insn;
	struct special_alt *special_alt, *tmp;
	struct alternative *alt;
	int ret;

	ret = special_get_alts(file->elf, &special_alts);
	if (ret)
		return ret;

	list_for_each_entry_safe(special_alt, tmp, &special_alts, list) {

		orig_insn = find_insn(file, special_alt->orig_sec,
				      special_alt->orig_off);
		if (!orig_insn) {
			WARN_FUNC("special: can't find orig instruction",
				  special_alt->orig_sec, special_alt->orig_off);
			ret = -1;
			goto out;
		}

		new_insn = NULL;
		if (!special_alt->group || special_alt->new_len) {
			new_insn = find_insn(file, special_alt->new_sec,
					     special_alt->new_off);
			if (!new_insn) {
				WARN_FUNC("special: can't find new instruction",
					  special_alt->new_sec,
					  special_alt->new_off);
				ret = -1;
				goto out;
			}
		}

		if (special_alt->group) {
			if (!special_alt->orig_len) {
				WARN_FUNC("empty alternative entry",
					  orig_insn->sec, orig_insn->offset);
				continue;
			}

			ret = handle_group_alt(file, special_alt, orig_insn,
					       &new_insn);
			if (ret)
				goto out;
		} else if (special_alt->jump_or_nop) {
			ret = handle_jump_alt(file, special_alt, orig_insn,
					      &new_insn);
			if (ret)
				goto out;
		}

		alt = malloc(sizeof(*alt));
		if (!alt) {
			WARN("malloc failed");
			ret = -1;
			goto out;
		}

		alt->insn = new_insn;
		list_add_tail(&alt->list, &orig_insn->alts);

		list_del(&special_alt->list);
		free(special_alt);
	}

out:
	return ret;
}

static int add_switch_table(struct objtool_file *file, struct instruction *insn,
			    struct rela *table)
{
	struct rela *rela = table;
	struct instruction *alt_insn;
	struct alternative *alt;
	struct symbol *pfunc = insn->func->pfunc;
	unsigned int prev_offset = 0;

	list_for_each_entry_from(rela, &table->rela_sec->rela_list, list) {

		/* Check for the end of the table: */
		if (rela != table && rela->jump_table_start)
			break;

		/* Make sure the switch table entries are consecutive: */
		if (prev_offset && rela->offset != prev_offset + 8)
			break;

		/* Detect function pointers from contiguous objects: */
		if (rela->sym->sec == pfunc->sec &&
		    rela->addend == pfunc->offset)
			break;

		alt_insn = find_insn(file, rela->sym->sec, rela->addend);
		if (!alt_insn)
			break;

		/* Make sure the jmp dest is in the function or subfunction: */
		if (alt_insn->func->pfunc != pfunc)
			break;

		alt = malloc(sizeof(*alt));
		if (!alt) {
			WARN("malloc failed");
			return -1;
		}

		alt->insn = alt_insn;
		list_add_tail(&alt->list, &insn->alts);
		prev_offset = rela->offset;
	}

	if (!prev_offset) {
		WARN_FUNC("can't find switch jump table",
			  insn->sec, insn->offset);
		return -1;
	}

	return 0;
}

static int add_func_switch_tables(struct objtool_file *file,
				  struct symbol *func)
{
	struct instruction *insn;
	int ret;

	func_for_each_insn_all(file, func, insn) {
		if (!insn->jump_table)
			continue;

		ret = add_switch_table(file, insn, insn->jump_table);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * For some switch statements, gcc generates a jump table in the .rodata
 * section which contains a list of addresses within the function to jump to.
 * This finds these jump tables and adds them to the insn->alts lists.
 */
static int add_switch_table_alts(struct objtool_file *file)
{
	struct section *sec;
	struct symbol *func;
	int ret;

	if (!file->rodata)
		return 0;

	for_each_sec(file, sec) {
		list_for_each_entry(func, &sec->symbol_list, list) {
			if (func->type != STT_FUNC)
				continue;

			arch_mark_func_jump_tables(file, func);
			ret = add_func_switch_tables(file, func);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int read_unwind_hints(struct objtool_file *file)
{
	struct section *sec, *relasec;
	struct rela *rela;
	struct unwind_hint *hint;
	struct instruction *insn;
	struct cfi_reg *cfa;
	int i;

	sec = find_section_by_name(file->elf, ".discard.unwind_hints");
	if (!sec)
		return 0;

	relasec = sec->rela;
	if (!relasec) {
		WARN("missing .rela.discard.unwind_hints section");
		return -1;
	}

	if (sec->len % sizeof(struct unwind_hint)) {
		WARN("struct unwind_hint size mismatch");
		return -1;
	}

	file->hints = true;

	for (i = 0; i < sec->len / sizeof(struct unwind_hint); i++) {
		hint = (struct unwind_hint *)sec->data->d_buf + i;

		rela = find_rela_by_dest(sec, i * sizeof(*hint));
		if (!rela) {
			WARN("can't find rela for unwind_hints[%d]", i);
			return -1;
		}

#ifdef __loongarch__
		insn = find_insn(file, rela->sym->sec, rela->sym->offset);
#else
		insn = find_insn(file, rela->sym->sec, rela->addend);
#endif
		if (!insn) {
			WARN("can't find insn for unwind_hints[%d]", i);
			return -1;
		}

		cfa = &insn->state.cfa;

		if (hint->type == UNWIND_HINT_TYPE_SAVE) {
			insn->save = true;
			continue;

		} else if (hint->type == UNWIND_HINT_TYPE_RESTORE) {
			insn->restore = true;
			insn->hint = true;
			continue;
		}

		insn->hint = true;

		if (arch_decode_hint_reg(cfa, hint->sp_reg)) {
			WARN_FUNC("unsupported unwind_hint sp base reg %d",
				  insn->sec, insn->offset, hint->sp_reg);
			return -1;
		}

		cfa->offset = hint->sp_offset;
		insn->state.type = hint->type;
		insn->state.end = hint->end;
	}

	return 0;
}

static int read_retpoline_hints(struct objtool_file *file)
{
	struct section *sec;
	struct instruction *insn;
	struct rela *rela;

	sec = find_section_by_name(file->elf, ".rela.discard.retpoline_safe");
	if (!sec)
		return 0;

	list_for_each_entry(rela, &sec->rela_list, list) {
		if (rela->sym->type != STT_SECTION) {
			WARN("unexpected relocation symbol type in %s", sec->name);
			return -1;
		}

		insn = find_insn(file, rela->sym->sec, rela->addend);
		if (!insn) {
			WARN("bad .discard.retpoline_safe entry");
			return -1;
		}

		if (insn->type != INSN_JUMP_DYNAMIC &&
		    insn->type != INSN_CALL_DYNAMIC) {
			WARN_FUNC("retpoline_safe hint not an indirect jump/call",
				  insn->sec, insn->offset);
			return -1;
		}

		insn->retpoline_safe = true;
	}

	return 0;
}

static void mark_rodata(struct objtool_file *file)
{
	struct section *sec;
	bool found = false;

	/*
	 * This searches for the .rodata section or multiple .rodata.func_name
	 * sections if -fdata-sections is being used. The .str.1.1 and .str.1.8
	 * rodata sections are ignored as they don't contain jump tables.
	 */
	for_each_sec(file, sec) {
		if (!strncmp(sec->name, ".rodata", 7) &&
		    !strstr(sec->name, ".str1.")) {
			sec->rodata = true;
			found = true;
		}
	}

	file->rodata = found;
}

static int decode_sections(struct objtool_file *file)
{
	int ret;

	mark_rodata(file);

	ret = decode_instructions(file);
	if (ret)
		return ret;

	ret = add_dead_ends(file);
	if (ret)
		return ret;

	ret = add_not_sibling_call(file);
	if (ret)
		return ret;

	add_ignores(file);

	ret = add_nospec_ignores(file);
	if (ret)
		return ret;

	ret = add_jump_destinations(file);
	if (ret)
		return ret;

	ret = add_special_section_alts(file);
	if (ret)
		return ret;

	ret = add_call_destinations(file);
	if (ret)
		return ret;

	ret = add_switch_table_alts(file);
	if (ret)
		return ret;

	ret = read_unwind_hints(file);
	if (ret)
		return ret;

	ret = read_retpoline_hints(file);
	if (ret)
		return ret;

	return 0;
}

static bool is_fentry_call(struct instruction *insn)
{
	if (insn->type == INSN_CALL &&
	    insn->call_dest->type == STT_NOTYPE &&
	    !strcmp(insn->call_dest->name, "__fentry__"))
		return true;

	return false;
}

static bool has_modified_stack_frame(struct insn_state *state)
{
	int i;

	if (state->cfa.base != initial_func_cfi.cfa.base ||
	    state->cfa.offset != initial_func_cfi.cfa.offset ||
	    state->stack_size != initial_func_cfi.cfa.offset ||
	    state->drap)
		return true;

	for (i = 0; i < CFI_NUM_REGS; i++)
		if (state->regs[i].base != initial_func_cfi.regs[i].base ||
		    state->regs[i].offset != initial_func_cfi.regs[i].offset)
			return true;

	return false;
}

static bool insn_state_match(struct instruction *insn, struct insn_state *state)
{
	struct insn_state *state1 = &insn->state, *state2 = state;
	int i;

	if (memcmp(&state1->cfa, &state2->cfa, sizeof(state1->cfa))) {
		WARN_FUNC("stack state mismatch: cfa1=%d%+d cfa2=%d%+d",
			  insn->sec, insn->offset,
			  state1->cfa.base, state1->cfa.offset,
			  state2->cfa.base, state2->cfa.offset);

	} else if (memcmp(&state1->regs, &state2->regs, sizeof(state1->regs))) {
		for (i = 0; i < CFI_NUM_REGS; i++) {
			if (!memcmp(&state1->regs[i], &state2->regs[i],
				    sizeof(struct cfi_reg)))
				continue;

			WARN_FUNC("stack state mismatch: reg1[%d]=%d%+d reg2[%d]=%d%+d",
				  insn->sec, insn->offset,
				  i, state1->regs[i].base, state1->regs[i].offset,
				  i, state2->regs[i].base, state2->regs[i].offset);
			break;
		}

	} else if (state1->type != state2->type) {
		WARN_FUNC("stack state mismatch: type1=%d type2=%d",
			  insn->sec, insn->offset, state1->type, state2->type);

	} else if (state1->drap != state2->drap ||
		 (state1->drap && state1->drap_reg != state2->drap_reg) ||
		 (state1->drap && state1->drap_offset != state2->drap_offset)) {
		WARN_FUNC("stack state mismatch: drap1=%d(%d,%d) drap2=%d(%d,%d)",
			  insn->sec, insn->offset,
			  state1->drap, state1->drap_reg, state1->drap_offset,
			  state2->drap, state2->drap_reg, state2->drap_offset);

	} else
		return true;

	return false;
}

/*
 * Follow the branch starting at the given instruction, and recursively follow
 * any other branches (jumps).  Meanwhile, track the frame pointer state at
 * each instruction and validate all the rules described in
 * tools/objtool/Documentation/stack-validation.txt.
 */
static int validate_branch(struct objtool_file *file, struct symbol *func,
			   struct instruction *first, struct insn_state state)
{
	struct alternative *alt;
	struct instruction *insn, *next_insn;
	struct section *sec;
	int ret;

	insn = first;
	sec = insn->sec;

	if (insn->alt_group && list_empty(&insn->alts)) {
		WARN_FUNC("don't know how to handle branch to middle of alternative instruction group",
			  sec, insn->offset);
		return 1;
	}

	while (1) {
		next_insn = next_insn_same_sec(file, insn);

		if (file->c_file && func && insn->func && func != insn->func->pfunc) {
			WARN("%s() falls through to next function %s()",
			     func->name, insn->func->name);
			return 1;
		}

		if (func && insn->ignore) {
			WARN_FUNC("BUG: why am I validating an ignored function?",
				  sec, insn->offset);
			return 1;
		}

		if (insn->visited) {
			if (!insn->hint && !insn_state_match(insn, &state))
				return 1;

			return 0;
		}

		if (insn->hint) {
			if (insn->restore) {
				struct instruction *save_insn, *i;

				i = insn;
				save_insn = NULL;
				func_for_each_insn_continue_reverse(file, func, i) {
					if (i->save) {
						save_insn = i;
						break;
					}
				}

				if (!save_insn) {
					WARN_FUNC("no corresponding CFI save for CFI restore",
						  sec, insn->offset);
					return 1;
				}

				if (!save_insn->visited) {
					/*
					 * Oops, no state to copy yet.
					 * Hopefully we can reach this
					 * instruction from another branch
					 * after the save insn has been
					 * visited.
					 */
					if (insn == first)
						return 0;

					WARN_FUNC("objtool isn't smart enough to handle this CFI save/restore combo",
						  sec, insn->offset);
					return 1;
				}

				insn->state = save_insn->state;
			}

			state = insn->state;

		} else
			insn->state = state;

		insn->visited = true;

		list_add(&insn->orbit_node, &orbit_list);

		if (insn->type == INSN_JUMP_DYNAMIC &&
		      arch_dynamic_add_jump_table_alts(&orbit_list, file, func, insn))
			return 1;

		if (!insn->ignore_alts && !list_empty(&insn->alts)) {
			list_for_each_entry(alt, &insn->alts, list) {
				ret = validate_branch(file, func, alt->insn, state);
				if (ret) {
					if (backtrace)
						BT_FUNC("(alt)", insn);
					return ret;
				}
				while (func_last_orbit(&orbit_list) &&
					func_last_orbit(&orbit_list)->offset != insn->offset) {
					list_del(&func_last_orbit(&orbit_list)->orbit_node);
				}
			}
		}

		switch (insn->type) {

		case INSN_RETURN:
			if (func && has_modified_stack_frame(&state)) {
				WARN_FUNC("return with modified stack frame",
					  sec, insn->offset);
				return 1;
			}

			if (state.bp_scratch) {
				WARN("%s uses BP as a scratch register",
				     func->name);
				return 1;
			}

			return 0;

		case INSN_CALL:
		case INSN_CALL_DYNAMIC:
			if (insn->type == INSN_CALL_DYNAMIC)
				arch_try_find_call(&orbit_list, file, func, insn);

			if (is_fentry_call(insn))
				break;

			ret = dead_end_function(file, insn->call_dest);
			if (ret == 1)
				return 0;
			if (ret == -1)
				return 1;

			if (!no_fp && func && !arch_has_valid_stack_frame(&state)) {
				WARN_FUNC("call without frame pointer save/setup",
					  sec, insn->offset);
				return 1;
			}
			break;

		case INSN_JUMP_CONDITIONAL:
		case INSN_JUMP_UNCONDITIONAL:
			if (insn->jump_dest &&
			    (!func || !insn->jump_dest->func ||
			     insn->jump_dest->func->pfunc == func)) {
				ret = validate_branch(file, func,
						      insn->jump_dest, state);
				if (ret) {
					if (backtrace)
						BT_FUNC("(branch)", insn);
					return ret;
				}

				while (func_last_orbit(&orbit_list) &&
					func_last_orbit(&orbit_list)->offset != insn->offset) {
					list_del(&func_last_orbit(&orbit_list)->orbit_node);
				}

			} else if (!insn->not_sibling_call && func
					&& has_modified_stack_frame(&state)) {
				WARN_FUNC("sibling call from callable instruction with modified stack frame",
					  sec, insn->offset);
				return 1;
			}

			if (insn->type == INSN_JUMP_UNCONDITIONAL)
				return 0;

			break;

		case INSN_JUMP_DYNAMIC:
			if (func && list_empty(&insn->alts) &&
			    has_modified_stack_frame(&state)) {
				WARN_FUNC("sibling call from callable instruction with modified stack frame",
					  sec, insn->offset);
				return 1;
			}

			return 0;

		case INSN_CONTEXT_SWITCH:
			if (func && (!next_insn || !next_insn->hint)) {
				WARN_FUNC("unsupported instruction in callable function",
					  sec, insn->offset);
				return 1;
			}
			return 0;

		case INSN_STACK:
			if (arch_update_insn_state(insn, &state, no_fp))
				return 1;

			break;

		default:
			break;
		}

		if (insn->dead_end)
			return 0;

		if (!next_insn) {
			if (state.cfa.base == CFI_UNDEFINED)
				return 0;
			WARN("%s: unexpected end of section", sec->name);
			return 1;
		}

		insn = next_insn;
	}

	return 0;
}

static int validate_unwind_hints(struct objtool_file *file)
{
	struct instruction *insn;
	int ret, warnings = 0;
	struct insn_state state;

	if (!file->hints)
		return 0;

	clear_insn_state(&state);

	for_each_insn(file, insn) {
		if (insn->hint && !insn->visited) {
			ret = validate_branch(file, insn->func, insn, state);
			if (ret && backtrace)
				BT_FUNC("<=== (hint)", insn);
			warnings += ret;
			while (!list_empty(&orbit_list)) {
				list_del(&func_last_orbit(&orbit_list)->orbit_node);
			}
		}
	}

	return warnings;
}

static int validate_retpoline(struct objtool_file *file)
{
	struct instruction *insn;
	int warnings = 0;

	for_each_insn(file, insn) {
		if (insn->type != INSN_JUMP_DYNAMIC &&
		    insn->type != INSN_CALL_DYNAMIC)
			continue;

		if (insn->retpoline_safe)
			continue;

		/*
		 * .init.text code is ran before userspace and thus doesn't
		 * strictly need retpolines, except for modules which are
		 * loaded late, they very much do need retpoline in their
		 * .init.text
		 */
		if (!strcmp(insn->sec->name, ".init.text") && !module)
			continue;

		WARN_FUNC("indirect %s found in RETPOLINE build",
			  insn->sec, insn->offset,
			  insn->type == INSN_JUMP_DYNAMIC ? "jump" : "call");

		warnings++;
	}

	return warnings;
}

static bool is_kasan_insn(struct instruction *insn)
{
	return (insn->type == INSN_CALL &&
		!strcmp(insn->call_dest->name, "__asan_handle_no_return"));
}

static bool is_ubsan_insn(struct instruction *insn)
{
	return (insn->type == INSN_CALL &&
		!strcmp(insn->call_dest->name,
			"__ubsan_handle_builtin_unreachable"));
}

static bool ignore_unreachable_insn(struct instruction *insn)
{
	int i;

	if (insn->ignore || insn->type == INSN_NOP)
		return true;

	/*
	 * Ignore any unused exceptions.  This can happen when a whitelisted
	 * function has an exception table entry.
	 *
	 * Also ignore alternative replacement instructions.  This can happen
	 * when a whitelisted function uses one of the ALTERNATIVE macros.
	 */
	if (!strcmp(insn->sec->name, ".fixup") ||
	    !strcmp(insn->sec->name, ".altinstr_replacement") ||
	    !strcmp(insn->sec->name, ".altinstr_aux"))
		return true;

	if (insn->type == INSN_JUMP_UNCONDITIONAL && insn->offset == FAKE_JUMP_OFFSET)
		return true;

	if (!insn->func)
		return false;

	/*
	 * CONFIG_UBSAN_TRAP inserts a UD2 when it sees
	 * __builtin_unreachable().  The BUG() macro has an unreachable() after
	 * the UD2, which causes GCC's undefined trap logic to emit another UD2
	 * (or occasionally a JMP to UD2).
	 */
	if (list_prev_entry(insn, list)->dead_end &&
	    (insn->type == INSN_BUG ||
	     (insn->type == INSN_JUMP_UNCONDITIONAL &&
	      insn->jump_dest && insn->jump_dest->type == INSN_BUG)))
		return true;

	/*
	 * Check if this (or a subsequent) instruction is related to
	 * CONFIG_UBSAN or CONFIG_KASAN.
	 *
	 * End the search at 5 instructions to avoid going into the weeds.
	 */
	for (i = 0; i < 5; i++) {

		if (is_kasan_insn(insn) || is_ubsan_insn(insn))
			return true;

		if (insn->type == INSN_JUMP_UNCONDITIONAL) {
			if (insn->jump_dest &&
			    insn->jump_dest->func == insn->func) {
				insn = insn->jump_dest;
				continue;
			}

			break;
		}

		if (insn->offset + insn->len >= insn->func->offset + insn->func->len)
			break;

		insn = list_next_entry(insn, list);
	}

	return false;
}

static int validate_functions(struct objtool_file *file)
{
	struct section *sec;
	struct symbol *func;
	struct instruction *insn;
	struct insn_state state;
	int ret, warnings = 0;

	clear_insn_state(&state);

	state.cfa = initial_func_cfi.cfa;
	memcpy(&state.regs, &initial_func_cfi.regs,
	       CFI_NUM_REGS * sizeof(struct cfi_reg));
	state.stack_size = initial_func_cfi.cfa.offset;

	for_each_sec(file, sec) {
		list_for_each_entry(func, &sec->symbol_list, list) {
			if (func->type != STT_FUNC || func->pfunc != func)
				continue;

			insn = find_insn(file, sec, func->offset);
			if (!insn || insn->ignore || insn->visited)
				continue;

			ret = validate_branch(file, func, insn, state);
			if (ret && backtrace)
				BT_FUNC("<=== (func)", insn);
			warnings += ret;
			while (!list_empty(&orbit_list)) {
				list_del(&func_last_orbit(&orbit_list)->orbit_node);
			}
		}
	}

	return warnings;
}

static int validate_reachable_instructions(struct objtool_file *file)
{
	struct instruction *insn;

	if (file->ignore_unreachables)
		return 0;

	for_each_insn(file, insn) {
		if (insn->visited || ignore_unreachable_insn(insn))
			continue;

		WARN_FUNC("unreachable instruction", insn->sec, insn->offset);
		return 1;
	}

	return 0;
}

static void cleanup(struct objtool_file *file)
{
	struct instruction *insn, *tmpinsn;
	struct alternative *alt, *tmpalt;

	list_for_each_entry_safe(insn, tmpinsn, &file->insn_list, list) {
		list_for_each_entry_safe(alt, tmpalt, &insn->alts, list) {
			list_del(&alt->list);
			free(alt);
		}
		list_del(&insn->list);
		hash_del(&insn->hash);
		free(insn);
	}
	elf_close(file->elf);
}

static struct objtool_file file;

int check(const char *_objname, bool orc)
{
	int ret, warnings = 0;

	objname = _objname;

	file.elf = elf_open(objname, orc ? O_RDWR : O_RDONLY);
	if (!file.elf)
		return 1;

	INIT_LIST_HEAD(&file.insn_list);
	INIT_LIST_HEAD(&orbit_list);
	hash_init(file.insn_hash);
	file.whitelist = find_section_by_name(file.elf, ".discard.func_stack_frame_non_standard");
	file.c_file = find_section_by_name(file.elf, ".comment");
	file.ignore_unreachables = no_unreachable;
	file.hints = false;

	arch_initial_func_cfi_state(&initial_func_cfi);

	ret = decode_sections(&file);
	if (ret < 0)
		goto out;
	warnings += ret;

	if (list_empty(&file.insn_list))
		goto out;

	if (retpoline) {
		ret = validate_retpoline(&file);
		if (ret < 0)
			return ret;
		warnings += ret;
	}

	ret = validate_functions(&file);
	if (ret < 0)
		goto out;
	warnings += ret;

	ret = validate_unwind_hints(&file);
	if (ret < 0)
		goto out;
	warnings += ret;

	if (!warnings) {
		ret = validate_reachable_instructions(&file);
		if (ret < 0)
			goto out;
		warnings += ret;
	}

	if (orc) {
		ret = create_orc(&file);
		if (ret < 0)
			goto out;

		ret = create_orc_sections(&file);
		if (ret < 0)
			goto out;

		ret = elf_write(file.elf);
		if (ret < 0)
			goto out;
	}

out:
	cleanup(&file);

	/* ignore warnings for now until we get all the code cleaned up */
	if (ret || warnings)
		return 0;
	return 0;
}
