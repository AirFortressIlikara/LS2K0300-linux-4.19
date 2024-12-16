// SPDX-License-Identifier: GPL-2.0
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */

#include <linux/jump_label.h>
#include <linux/kernel.h>
#include <linux/memory.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/cpu.h>

#include <asm/cacheflush.h>
#include <asm/inst.h>

void arch_jump_label_transform(struct jump_entry *e,
			       enum jump_label_type type)
{

	u32 insn, *insn_p;

	insn_p = (u32 *)e->code;

	if (type == JUMP_LABEL_JMP)
		insn = la64_insn_gen_b((unsigned long)e->code, (unsigned long)e->target);
	else
		insn = la64_insn_gen_nop();

	mutex_lock(&text_mutex);
	*insn_p = insn;

	flush_icache_range((unsigned long)insn_p,
			   (unsigned long)insn_p + sizeof(*insn_p));

	mutex_unlock(&text_mutex);
}
