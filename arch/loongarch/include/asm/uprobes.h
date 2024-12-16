/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#ifndef __ASM_UPROBES_H
#define __ASM_UPROBES_H

#include <linux/notifier.h>
#include <linux/types.h>

#include <asm/break.h>
#include <asm/inst.h>

/*
 * We want this to be defined as union loongarch_instruction but that makes the
 * generic code blow up.
 */
typedef u32 uprobe_opcode_t;

#define MAX_UINSN_BYTES			8
#define UPROBE_XOL_SLOT_BYTES		128	/* Max. cache line size */

#define UPROBE_BRK_UPROBE		0x002a000c	/* break 12 */
#define UPROBE_BRK_UPROBE_XOL		0x002a000d	/* break 13 */

#define UPROBE_SWBP_INSN		UPROBE_BRK_UPROBE
#define UPROBE_SWBP_INSN_SIZE		4

struct arch_uprobe {
	unsigned long	resume_era;
	u32	insn[2];
	u32	ixol[2];
};

struct arch_uprobe_task {
	unsigned long saved_trap_nr;
};

#endif /* __ASM_UPROBES_H */
