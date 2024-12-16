#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/stringify.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/memory.h>
#include <linux/stop_machine.h>
#include <linux/slab.h>
#include <linux/kdebug.h>
#include <asm/alternative.h>
#include <asm/sections.h>
#include <asm/pgtable.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/io.h>
#include <asm/fixmap.h>

#ifdef CONFIG_CPU_LOONGSON3
int __read_mostly alternatives_patched;

EXPORT_SYMBOL_GPL(alternatives_patched);

#define MAX_PATCH_LEN (255-1)

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

#define DUMP_BYTES(buf, len, fmt, args...)				\
do {									\
	if (unlikely(debug_alternative)) {				\
		int j;							\
									\
		if (!(len))						\
			break;						\
									\
		printk(KERN_DEBUG fmt, ##args);				\
		for (j = 0; j < (len) - 1; j++)				\
			printk(KERN_CONT "%02hhx ", buf[j]);		\
		printk(KERN_CONT "%02hhx\n", buf[j]);			\
	}								\
} while (0)

/* Use this to add nops to a buffer, then text_poke the whole buffer. */
static void __init_or_module add_nops(void *insns, unsigned int len)
{
	memset(insns, 0, len);
}

extern struct alt_instr __alt_instructions[], __alt_instructions_end[];
void *text_poke_early(void *addr, const void *opcode, size_t len);

/*
 * Are we looking at a near JMP with a 1 or 4-byte displacement.
 */
static inline bool is_jmp(const u32 instr)
{
	switch (MIPSInst_OPCODE(instr)) {
	case bcond_op:
		switch (MIPSInst_RT(instr)) {
		case bltzal_op:
		case bltzall_op:
		case bltzl_op:
		case bltz_op:
		case bgezal_op:
		case bgezall_op:
		case bgezl_op:
		case bgez_op:
			return true;
		default:
			return false;
		}
	case beql_op:
	case beq_op:
	case bnel_op:
	case bne_op:
	case blezl_op:
	case blez_op:
	case bgtzl_op:
	case bgtz_op:
		return true;
	default:
		return false;
	}
}

static void __init_or_module
recompute_jump(struct alt_instr *a, u8 *orig_insn, u8 *repl_insn, u8 *insnbuf)
{
	u8 *next_rip, *tgt_rip;
	s16 n_dspl, o_dspl;
	u32 insn;

	o_dspl = MIPSInst_SIMM(*(u32 *)repl_insn);

	/* next_rip of the replacement JMP */
	next_rip = repl_insn + a->replacementlen;
	/* target rip of the replacement JMP */
	tgt_rip  = next_rip + (o_dspl << 2);
	n_dspl = (tgt_rip - (orig_insn + a->replacementlen)) >> 2;

	DPRINTK("target RIP: %px, new_displ: 0x%x", tgt_rip, n_dspl);

	insn = *(u32 *)insnbuf;
	insn = (insn & 0xffff0000) | (0x0000ffff & n_dspl);
	*(u32 *)insnbuf = insn;

	DPRINTK("final displ: 0x%08x, JMP 0x%lx",
		n_dspl, (unsigned long)orig_insn + n_dspl);
}

/*
 * "noinline" to cause control flow change and thus invalidate I$ and
 * cause refetch after modification.
 */
static noinline void __init_or_module optimize_nops(struct alt_instr *a, u8 *instr)
{
	unsigned long flags;

	local_irq_save(flags);
	add_nops(instr + (a->instrlen - a->padlen), a->padlen);
	local_irq_restore(flags);

	DUMP_BYTES(instr, a->instrlen, "%px: [%d:%d) optimized NOPs: ",
		   instr, a->instrlen - a->padlen, a->padlen);
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
	u8 *instr, *replacement;
	u8 insnbuf[MAX_PATCH_LEN];

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
		int insnbuf_sz = 0;

		instr = (u8 *)&a->instr_offset + a->instr_offset;
		replacement = (u8 *)&a->repl_offset + a->repl_offset;
		BUG_ON(a->instrlen > sizeof(insnbuf));
		if (!boot_cpu_has(a->cpuid)) {
			DPRINTK("feat not exist: %d, old: (%px len: %d), repl: (%px, len: %d), pad: %d",
				a->cpuid,
				instr, a->instrlen,
				replacement, a->replacementlen, a->padlen);
			if (a->padlen > 1)
				optimize_nops(a, instr);

			continue;
		}

		DPRINTK("feat: %d, old: (%px len: %d), repl: (%px, len: %d), pad: %d",
			a->cpuid,
			instr, a->instrlen,
			replacement, a->replacementlen, a->padlen);

		DUMP_BYTES(instr, a->instrlen, "%px: old_insn: ", instr);
		DUMP_BYTES(replacement, a->replacementlen, "%px: rpl_insn: ", replacement);

		memcpy(insnbuf, replacement, a->replacementlen);
		insnbuf_sz = a->replacementlen;

		if (a->replacementlen && is_jmp(*(u32 *)replacement))
			recompute_jump(a, instr, replacement, insnbuf);

		if (a->instrlen > a->replacementlen) {
			add_nops(insnbuf + a->replacementlen,
				 a->instrlen - a->replacementlen);
			insnbuf_sz += a->instrlen - a->replacementlen;
		}
		DUMP_BYTES(insnbuf, insnbuf_sz, "%px: final_insn: ", instr);

		text_poke_early(instr, insnbuf, insnbuf_sz);
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
void *__init_or_module text_poke_early(void *addr, const void *opcode,
				       size_t len)
{
	unsigned long flags;
	int i;

	local_irq_save(flags);
	wmb();
	for (i = 0; i < len; ++i) {
		((u8 *)addr)[i] = ((u8 *)opcode)[i];
	}
	wmb();
	local_irq_restore(flags);
	flush_icache_range((unsigned long)addr, (unsigned long)(addr + len));

	return addr;
}
#endif /* CONFIG_CPU_LOONGSON3 */
