/*
 * Copyright (C) 2017 Josh Poimboeuf <jpoimboe@redhat.com>
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

#ifndef _CHECK_H
#define _CHECK_H

#include <stdbool.h>
#include "cfi.h"
#include "arch.h"

struct insn_state {
	struct cfi_reg cfa;
	struct cfi_reg regs[CFI_NUM_REGS];
	int stack_size;
	unsigned char type;
	bool bp_scratch;
	bool drap, end;
	int drap_reg, drap_offset;
	struct cfi_reg vals[CFI_NUM_REGS];
};

struct instruction {
	struct list_head list;
	struct hlist_node hash;
	struct list_head orbit_node;
	struct section *sec;
	unsigned long offset;
	unsigned int len;
	unsigned char type;
	unsigned long immediate;
	bool alt_group, visited, dead_end, ignore, hint, save, restore, ignore_alts;
	bool retpoline_safe, not_sibling_call;
	struct symbol *call_dest;
	struct instruction *jump_dest;
	struct instruction *first_jump_src;
	struct rela *jump_table;
	struct list_head alts;
	struct symbol *func;
	struct stack_op stack_op;
	struct insn_state state;
	struct orc_entry orc;
};

struct instruction *find_insn(struct objtool_file *file,
			      struct section *sec, unsigned long offset);

struct instruction *next_insn_same_sec(struct objtool_file *file,
					      struct instruction *insn);

struct instruction *next_insn_same_func(struct objtool_file *file,
					       struct instruction *insn);

#define for_each_insn(file, insn)					\
	list_for_each_entry(insn, &file->insn_list, list)

#define sec_for_each_insn(file, sec, insn)				\
	for (insn = find_insn(file, sec, 0);				\
	     insn && &insn->list != &file->insn_list &&			\
			insn->sec == sec;				\
	     insn = list_next_entry(insn, list))

#define func_for_each_insn_all(file, func, insn)			\
	for (insn = find_insn(file, func->sec, func->offset);		\
	     insn;							\
	     insn = next_insn_same_func(file, insn))

#define func_for_each_insn(file, func, insn)				\
	for (insn = find_insn(file, func->sec, func->offset);		\
	     insn && &insn->list != &file->insn_list &&			\
		insn->sec == func->sec &&				\
		insn->offset < func->offset + func->len;		\
	     insn = list_next_entry(insn, list))

#define func_for_each_insn_continue_reverse(file, func, insn)		\
	for (insn = list_prev_entry(insn, list);			\
	     &insn->list != &file->insn_list &&				\
		insn->sec == func->sec && insn->offset >= func->offset;	\
	     insn = list_prev_entry(insn, list))

#define sec_for_each_insn_from(file, insn)				\
	for (; insn; insn = next_insn_same_sec(file, insn))

#define sec_for_each_insn_continue(file, insn)				\
	for (insn = next_insn_same_sec(file, insn); insn;		\
	     insn = next_insn_same_sec(file, insn))

#define func_last_orbit(p)						\
	  (list_first_entry_or_null(p, struct instruction, orbit_node))

bool arch_has_valid_stack_frame(struct insn_state *state);

int arch_update_insn_state(struct instruction *insn, struct insn_state *state, int no_fp);

#endif /* _CHECK_H */
