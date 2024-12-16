#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/stringify.h>
#include <linux/mm.h>
#include <linux/memory.h>
#include <linux/stop_machine.h>
#include <linux/slab.h>
#include <linux/kdebug.h>
#include <asm/alternative.h>
#include <asm/cacheflush.h>
#include <asm/inst.h>
#include <asm/pgtable.h>
#include <asm/sections.h>
#include <asm/tlbflush.h>
#include <asm/io.h>
#include <asm/fixmap.h>

int __read_mostly alternatives_patched;

EXPORT_SYMBOL_GPL(alternatives_patched);

#define MAX_PATCH_SIZE (((u8)(-1)) / LOONGARCH_INSN_SIZE)

static int __initdata_or_module debug_alternative;

static int __init debug_alt(char *str)
{
	debug_alternative = 1;
	return 1;
}
__setup("debug-alternative", debug_alt);

#define DPRINTK(fmt, args...)						\
do {									\
	if (debug_alternative)						\
		printk(KERN_DEBUG "%s: " fmt "\n", __func__, ##args);	\
} while (0)

#define DUMP_WORDS(buf, count, fmt, args...)				\
do {									\
	if (unlikely(debug_alternative)) {				\
		int _j;							\
		union loongarch_instruction *_buf = buf;		\
									\
		if (!(count))						\
			break;						\
									\
		printk(KERN_DEBUG fmt, ##args);				\
		for (_j = 0; _j < count - 1; _j++)			\
			printk(KERN_CONT "<%08x> ", _buf[_j].word);	\
		printk(KERN_CONT "<%08x>\n", _buf[_j].word);		\
	}								\
} while (0)

/* Use this to add nops to a buffer, then text_poke the whole buffer. */
static void __init_or_module add_nops(union loongarch_instruction *insn,
					int count)
{
	while (count--) {
		insn->word = INSN_NOP;
		insn++;
	}
}

extern struct alt_instr __alt_instructions[], __alt_instructions_end[];
void *text_poke_early(union loongarch_instruction *insn,
		union loongarch_instruction *buf, unsigned int nr);

/* Is the jump addr in local .altinstructions */
static inline bool in_alt_jump(unsigned long jump, void *start, void *end)
{
	return jump >= (unsigned long)start && jump < (unsigned long)end;
}

static void __init_or_module recompute_jump(union loongarch_instruction *buf,
		union loongarch_instruction *dest, union loongarch_instruction *src,
		void *start, void *end)
{
	unsigned int si, si_l, si_h;
	unsigned long cur_pc, jump_addr, pc;
	long offset;

	cur_pc = (unsigned long)src;
	pc = (unsigned long)dest;

	si_l = src->reg0i26_format.simmediate_l;
	si_h = src->reg0i26_format.simmediate_h;
	switch (src->reg0i26_format.opcode) {
	case b_op:
	case bl_op:
		jump_addr = bs_dest_26(cur_pc, si_h, si_l);
		if (in_alt_jump(jump_addr, start, end))
			return;
		offset = jump_addr - pc;
		BUG_ON(offset < -SZ_128M || offset >= SZ_128M);
		offset >>= 2;
		buf->reg0i26_format.simmediate_h = offset >> 16;
		buf->reg0i26_format.simmediate_l = offset;
		return;
	}

	si_l = src->reg1i21_format.simmediate_l;
	si_h = src->reg1i21_format.simmediate_h;
	switch (src->reg1i21_format.opcode) {
	case beqz_op:
	case bnez_op:
	case bceqz_op:
		jump_addr = bs_dest_21(cur_pc, si_h, si_l);
		if (in_alt_jump(jump_addr, start, end))
			return;
		offset = jump_addr - pc;
		BUG_ON(offset < -SZ_4M || offset >= SZ_4M);
		offset >>= 2;
		buf->reg1i21_format.simmediate_h = offset >> 16;
		buf->reg1i21_format.simmediate_l = offset;
		return;
	}

	si = src->reg2i16_format.simmediate;
	switch (src->reg2i16_format.opcode) {
	case beq_op:
	case bne_op:
	case blt_op:
	case bge_op:
	case bltu_op:
	case bgeu_op:
		jump_addr = bs_dest_16(cur_pc, si);
		if (in_alt_jump(jump_addr, start, end))
			return;
		offset = jump_addr - pc;
		BUG_ON(offset < -SZ_128K || offset >= SZ_128K);
		offset >>= 2;
                buf->reg2i16_format.simmediate = offset;
		return;
	}
}

static int __init_or_module copy_alt_insns(union loongarch_instruction *buf,
	union loongarch_instruction *dest, union loongarch_instruction *src, int nr)
{
	int i;

	for (i = 0; i < nr; i++) {
		buf[i].word = src[i].word;

		if (is_pc_insn(src[i])) {
			pr_err("Not support pcrel instruction at present!");
			return -EINVAL;
		}


		if (is_branch_insn(src[i]) &&
		    src[i].reg2i16_format.opcode != jirl_op) {
			recompute_jump(&buf[i], &dest[i], &src[i], src, src + nr);
		}
	}

	return 0;
}

/*
 * Replace instructions with better alternatives for this CPU type. This runs
 * before SMP is initialized to avoid SMP problems with self modifying code.
 * This implies that asymmetric systems where APs have less capabilities than
 * the boot processor are not handled. Tough. Make sure you disable such
 * features by hand.
 *
 * Marked "noinline" to cause control flow change and thus insn cache
 * to refetch changed I$ lines.
 */
noinline void __init_or_module apply_alternatives(struct alt_instr *start,
						  struct alt_instr *end)
{
	struct alt_instr *a;
	union loongarch_instruction *instr, *replacement;
	union loongarch_instruction insnbuf[MAX_PATCH_SIZE];
	unsigned int nr_instr, nr_repl, nr_insnbuf;

	DPRINTK("alt table %px, -> %px", start, end);
	/*
	 * The scan order should be from start to end. A later scanned
	 * alternative code can overwrite previously scanned alternative code.
	 * Some kernel functions (e.g. memcpy, memset, etc) use this order to
	 * patch code.
	 *
	 * So be careful if you want to change the scan order to any other
	 * order.
	 */
	for (a = start; a < end; a++) {
		nr_insnbuf = 0;

		instr = (void *)&a->instr_offset + a->instr_offset;
		replacement = (void *)&a->replace_offset + a->replace_offset;

		BUG_ON(a->instrlen > sizeof(insnbuf));
		BUG_ON(a->instrlen & 0x3);
		BUG_ON(a->replacementlen & 0x3);

		nr_instr = a->instrlen / LOONGARCH_INSN_SIZE;
		nr_repl = a->replacementlen / LOONGARCH_INSN_SIZE;

		if (!cpu_has(a->cpuid)) {
			DPRINTK("feat not exist: %d, old: (%px len: %d), repl: (%px, len: %d)",
				a->cpuid,
				instr, a->instrlen,
				replacement, a->replacementlen);

			continue;
		}

		DPRINTK("feat: %d, old: (%px len: %d), repl: (%px, len: %d)",
			a->cpuid,
			instr, a->instrlen,
			replacement, a->replacementlen);

		DUMP_WORDS(instr, nr_instr, "%px: old_insn: ", instr);
		DUMP_WORDS(replacement, nr_repl, "%px: rpl_insn: ", replacement);

		copy_alt_insns(insnbuf, instr, replacement, nr_repl);
		nr_insnbuf = nr_repl;

		if (nr_instr > nr_repl) {
			add_nops(insnbuf + nr_repl, nr_instr - nr_repl);
			nr_insnbuf += nr_instr - nr_repl;
		}
		DUMP_WORDS(insnbuf, nr_insnbuf, "%px: final_insn: ", instr);

		text_poke_early(instr, insnbuf, nr_insnbuf);
	}
}

void __init alternative_instructions(void)
{
	/*
	 * Don't stop machine check exceptions while patching.
	 * MCEs only happen when something got corrupted and in this
	 * case we must do something about the corruption.
	 * Ignoring it is worse than a unlikely patching race.
	 * Also machine checks tend to be broadcast and if one CPU
	 * goes into machine check the others follow quickly, so we don't
	 * expect a machine check to cause undue problems during to code
	 * patching.
	 */

	apply_alternatives(__alt_instructions, __alt_instructions_end);

	alternatives_patched = 1;
}

/**
 * text_poke_early - Update instructions on a live kernel at boot time
 * @addr: address to modify
 * @opcode: source of the copy
 * @len: length to copy
 *
 * When you use this code to patch more than one byte of an instruction
 * you need to make sure that other CPUs cannot execute this code in parallel.
 * Also no thread must be currently preempted in the middle of these
 * instructions. And on the local CPU you need to be protected again NMI or MCE
 * handlers seeing an inconsistent instruction while you patch.
 */
void *__init_or_module text_poke_early(union loongarch_instruction *insn,
			union loongarch_instruction *buf, unsigned int nr)
{
	unsigned long flags;
	int i;

	local_irq_save(flags);
	wmb();

	for (i = 0; i < nr; i++)
		insn[i].word = buf[i].word;

	wmb();
	local_irq_restore(flags);
	flush_icache_range((unsigned long)insn, (unsigned long)(insn + nr));

	return insn;
}
