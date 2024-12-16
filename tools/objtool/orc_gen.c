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

#include <stdlib.h>
#include <string.h>

#include "check.h"
#include "warn.h"
#include "orc.h"

int create_orc_sections(struct objtool_file *file)
{
	struct instruction *insn, *prev_insn;
	struct section *sec, *ip_sec, *u_sec, *ip_relasec;
	unsigned int idx;

	sec = find_section_by_name(file->elf, ".orc_unwind");
	if (sec) {
		WARN("file already has .orc_unwind section, skipping");
		return -1;
	}

	/* count the number of needed orcs */
	idx = 0;
	for_each_sec(file, sec) {
		if (!sec->text)
			continue;

		prev_insn = NULL;
		sec_for_each_insn(file, sec, insn) {
			if (!prev_insn ||
			    memcmp(&insn->orc, &prev_insn->orc,
				   sizeof(struct orc_entry))) {
				idx++;
			}
			prev_insn = insn;
		}

		/* section terminator */
		if (prev_insn)
			idx++;
	}
	if (!idx)
		return -1;


	/* create .orc_unwind_ip section */
	ip_sec = elf_create_section(file->elf, ".orc_unwind_ip", sizeof(int), idx);
	if (!ip_sec)
		return -1;

	/* create .rela.orc_unwind_ip section */
	ip_relasec = elf_create_rela_section(file->elf, ip_sec);
	if (!ip_relasec)
		return -1;

	/* create .orc_unwind section */
	u_sec = elf_create_section(file->elf, ".orc_unwind",
				   sizeof(struct orc_entry), idx);

	/* populate sections */
	idx = 0;
	for_each_sec(file, sec) {
		if (!sec->text)
			continue;

		prev_insn = NULL;
		sec_for_each_insn(file, sec, insn) {
			if (!prev_insn || memcmp(&insn->orc, &prev_insn->orc,
						 sizeof(struct orc_entry))) {

				if (arch_create_orc_entry(u_sec, ip_relasec, idx,
						     insn->sec, insn->offset,
						     &insn->orc))
					return -1;

				idx++;
			}
			prev_insn = insn;
		}

		/* section terminator */
		if (prev_insn) {
			if (arch_create_orc_entry_empty(u_sec, ip_relasec, idx,
					     prev_insn->sec,
					     prev_insn->offset + prev_insn->len))
				return -1;

			idx++;
		}
	}

	if (elf_rebuild_rela_section(ip_relasec))
		return -1;

	return 0;
}
