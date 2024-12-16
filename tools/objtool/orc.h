// SPDX-License-Identifier: GPL-2.0-or-later

#include "elf.h"
#include "check.h"

int arch_create_orc_entry(struct section *u_sec, struct section *ip_relasec,
				unsigned int idx, struct section *insn_sec,
				unsigned long insn_off, struct orc_entry *o);

int arch_create_orc_entry_empty(struct section *u_sec, struct section *ip_relasec,
				unsigned int idx, struct section *insn_sec,
				unsigned long insn_off);

void arch_print_reg(struct orc_entry orc);
