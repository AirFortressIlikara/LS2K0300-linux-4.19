// SPDX-License-Identifier: GPL-2.0+
/*
* Copyright (C) 2020 Loongson Technology Corporation Limited
*
* Author: Hanlu Li <lihanlu@loongson.cn>
*/

#undef DEBUG

#include <linux/moduleloader.h>
#include <linux/elf.h>
#include <linux/ftrace.h>
#include <linux/mm.h>
#include <linux/numa.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <asm/alternative.h>
#include <asm/inst.h>
#include <asm/unwind.h>
#include <asm/module.h>

static inline bool unsigned_imm_check(unsigned long val, unsigned int bit)
{
	return val < (1UL << bit);
}

static inline bool signed_imm_check(long val, unsigned int bit)
{
	return -(1L << (bit - 1)) <= val && val < (1L << (bit - 1));
}

static int la_rela_stack_push(s64 stack_value, s64 *la_rela_stack, size_t *la_rela_stack_top)
{
	if (LA_RELA_STACK_DEPTH <= *la_rela_stack_top)
		return -ENOEXEC;

	la_rela_stack[(*la_rela_stack_top)++] = stack_value;
	pr_debug("%s stack_value = 0x%llx\n", __func__, stack_value);

	return 0;
}

static int la_rela_stack_pop(s64 *stack_value, s64 *la_rela_stack, size_t *la_rela_stack_top)
{
	if (*la_rela_stack_top == 0)
		return -ENOEXEC;

	*stack_value = la_rela_stack[--(*la_rela_stack_top)];
	pr_debug("%s stack_value = 0x%llx\n", __func__, *stack_value);

	return 0;
}

static int apply_r_larch_error(struct module *me, u32 *location, Elf_Addr v,
				  s64 *la_rela_stack, size_t *la_rela_stack_top, unsigned int type)
{
	pr_err("%s: Unsupport relocation type %u, please add its support.\n", me->name, type);
	return -EINVAL;
}

static int apply_r_larch_none(struct module *me, u32 *location, Elf_Addr v,
			      s64 *la_rela_stack, size_t *la_rela_stack_top, unsigned int type)
{
	return 0;
}

static int apply_r_larch_32(struct module *me, u32 *location, Elf_Addr v,
			    s64 *la_rela_stack, size_t *la_rela_stack_top, unsigned int type)
{
	*location = v;
	return 0;
}

static int apply_r_larch_64(struct module *me, u32 *location, Elf_Addr v,
			    s64 *la_rela_stack, size_t *la_rela_stack_top, unsigned int type)
{
	*(Elf_Addr *)location = v;
	return 0;
}

static int apply_r_larch_sop_push_pcrel(struct module *me, u32 *location, Elf_Addr v,
					s64 *la_rela_stack, size_t *la_rela_stack_top, unsigned int type)
{
	return la_rela_stack_push(v - (u64)location, la_rela_stack, la_rela_stack_top);
}

static int apply_r_larch_sop_push_absolute(struct module *me, u32 *location, Elf_Addr v,
					   s64 *la_rela_stack, size_t *la_rela_stack_top, unsigned int type)
{
	int err = 0;

	err = la_rela_stack_push(v, la_rela_stack, la_rela_stack_top);
	return err;
}

static int apply_r_larch_sop_push_plt_pcrel(struct module *me, Elf_Shdr *sechdrs, u32 *location, Elf_Addr v,
					    s64 *la_rela_stack, size_t *la_rela_stack_top, unsigned int type)
{
	ptrdiff_t offset = (void *)v - (void *)location;

	if (offset < -SZ_128M || offset >= SZ_128M) {
		ptrdiff_t plt_entry;
		plt_entry = module_emit_plt_entry(me, sechdrs, v);
		offset = (void *)location - (void *)plt_entry;
		if (offset < -SZ_128M || offset >= SZ_128M) {
			pr_err("%s: plt entry too far\n", me->name);
			return -EINVAL;
		}
		return apply_r_larch_sop_push_pcrel(me, location, plt_entry,
				la_rela_stack, la_rela_stack_top, type);
	}
	return apply_r_larch_sop_push_pcrel(me, location, v, la_rela_stack, la_rela_stack_top, type);
}

static int apply_r_larch_sop(struct module *me, u32 *location, Elf_Addr v,
			     s64 *la_rela_stack, size_t *la_rela_stack_top, unsigned int type)
{
	int err = 0;
	s64 opr1, opr2, opr3;

	if (type == R_LARCH_SOP_IF_ELSE) {
		err = la_rela_stack_pop(&opr3, la_rela_stack, la_rela_stack_top);
		if (err)
			return err;
	}

	err = la_rela_stack_pop(&opr2, la_rela_stack, la_rela_stack_top);
	if (err)
		return err;
	err = la_rela_stack_pop(&opr1, la_rela_stack, la_rela_stack_top);
	if (err)
		return err;

	switch (type) {
	case R_LARCH_SOP_SUB:
		err = la_rela_stack_push(opr1 - opr2, la_rela_stack, la_rela_stack_top);
		break;
	case R_LARCH_SOP_SL:
		err = la_rela_stack_push(opr1 << opr2, la_rela_stack, la_rela_stack_top);
		break;
	case R_LARCH_SOP_SR:
		err = la_rela_stack_push(opr1 >> opr2, la_rela_stack, la_rela_stack_top);
		break;
	case R_LARCH_SOP_ADD:
		err = la_rela_stack_push(opr1 + opr2, la_rela_stack, la_rela_stack_top);
		break;
	case R_LARCH_SOP_AND:
		err = la_rela_stack_push(opr1 & opr2, la_rela_stack, la_rela_stack_top);
		break;
	case R_LARCH_SOP_IF_ELSE:
		err = la_rela_stack_push(opr1 ? opr2 : opr3, la_rela_stack, la_rela_stack_top);
		break;
	default:
		pr_err("%s: Unsupport relocation type %u handler\n", me->name, type);
		return -EINVAL;
	}

	return err;
}

static int apply_r_larch_sop_imm_field(struct module *me, u32 *location, Elf_Addr v,
				       s64 *la_rela_stack, size_t *la_rela_stack_top, unsigned int type)
{
	int err = 0;
	s64 opr1;
	union loongarch_instruction *insn = (union loongarch_instruction *)location;

	err = la_rela_stack_pop(&opr1, la_rela_stack, la_rela_stack_top);
	if (err)
		return err;

	switch (type) {
	case R_LARCH_SOP_POP_32_U_10_12:
		if (!unsigned_imm_check(opr1, 12))
			goto overflow;

		insn->reg2ui12_format.simmediate = opr1 & 0xfff;
		return 0;
	case R_LARCH_SOP_POP_32_S_10_12:
		if (!signed_imm_check(opr1, 12))
			goto overflow;

		insn->reg2i12_format.simmediate = opr1 & 0xfff;
		return 0;
	case R_LARCH_SOP_POP_32_S_10_16:
		if (!signed_imm_check(opr1, 16))
			goto overflow;

		insn->reg2i16_format.simmediate = opr1 & 0xffff;
		return 0;
	case R_LARCH_SOP_POP_32_S_10_16_S2:
		if (opr1 % 4)
			goto unaligned;

		if (!signed_imm_check(opr1, 18))
			goto overflow;

		insn->reg2i16_format.simmediate = (opr1 >> 2) & 0xffff;
		return 0;
	case R_LARCH_SOP_POP_32_S_5_20:
		if (!signed_imm_check(opr1, 20))
			goto overflow;

		insn->reg1i20_format.simmediate = (opr1) & 0xfffff;
		return 0;
	case R_LARCH_SOP_POP_32_S_0_5_10_16_S2:
		if (opr1 % 4)
			goto unaligned;

		if (!signed_imm_check(opr1, 23))
			goto overflow;

		opr1 >>= 2;
		insn->reg1i21_format.simmediate_l = opr1 & 0xffff;
		insn->reg1i21_format.simmediate_h = (opr1 >> 16) & 0x1f;
		return 0;
	case R_LARCH_SOP_POP_32_S_0_10_10_16_S2:
		if (opr1 % 4)
			goto unaligned;

		if (!signed_imm_check(opr1, 28))
			goto overflow;

		opr1 >>= 2;
		insn->reg0i26_format.simmediate_l = opr1 & 0xffff;
		insn->reg0i26_format.simmediate_h = (opr1 >> 16) & 0x3ff;
		return 0;
	case R_LARCH_SOP_POP_32_U:
		if (!unsigned_imm_check(opr1, 32))
			goto overflow;

		/* (*(uint32_t *) PC) = opr */
		*location = (u32)opr1;
		return 0;
	default:
		pr_err("%s: Unsupport relocation type %u handler\n", me->name, type);
		return -EINVAL;
	}

overflow:
	pr_err("module %s: opr1 = 0x%llx overflow! dangerous %s (%u) relocation\n",
		me->name, opr1, __func__, type);
	return -ENOEXEC;

unaligned:
	pr_err("module %s: opr1 = 0x%llx unaligned! dangerous %s (%u) relocation\n",
		me->name, opr1, __func__, type);
	return -ENOEXEC;
}

static int apply_r_larch_add_sub(struct module *me, u32 *location, Elf_Addr v,
			       s64 *la_rela_stack, size_t *la_rela_stack_top, unsigned int type)
{
	switch (type) {
	case R_LARCH_ADD32:
		*(s32 *)location += v;
		return 0;
	case R_LARCH_ADD64:
		*(s64 *)location += v;
		return 0;
	case R_LARCH_SUB32:
		*(s32 *)location -= v;
		return 0;
	case R_LARCH_SUB64:
		*(s64 *)location -= v;
		return 0;
	default:
		pr_err("%s: Unsupport relocation type %u handler\n", me->name, type);
		return -EINVAL;
	}
}

/*
 * reloc_handlers_rela() - Apply a particular relocation to a module
 * @me: the module to apply the reloc to
 * @location: the address at which the reloc is to be applied
 * @v: the value of the reloc, with addend for RELA-style
 * @la_rela_stack: the stack used for store relocation info, LOCAL to THIS module
 * @la_rela_stac_top: where the stack operation(pop/push) applies to
 *
 * Return: 0 upon success, else -ERRNO
 */
typedef int (*reloc_rela_handler)(struct module *me, u32 *location, Elf_Addr v,
		s64 *la_rela_stack, size_t *la_rela_stack_top, unsigned int type);

/* The handlers for known reloc types */
static reloc_rela_handler reloc_rela_handlers[] = {
	[R_LARCH_NONE ... R_LARCH_SUB64]			= apply_r_larch_error,

	[R_LARCH_NONE]						= apply_r_larch_none,
	[R_LARCH_32]						= apply_r_larch_32,
	[R_LARCH_64]						= apply_r_larch_64,
	[R_LARCH_MARK_LA]					= apply_r_larch_none,
	[R_LARCH_MARK_PCREL]					= apply_r_larch_none,
	[R_LARCH_SOP_PUSH_PCREL]				= apply_r_larch_sop_push_pcrel,
	[R_LARCH_SOP_PUSH_ABSOLUTE]				= apply_r_larch_sop_push_absolute,
	[R_LARCH_SOP_SUB ... R_LARCH_SOP_IF_ELSE]		= apply_r_larch_sop,
	[R_LARCH_SOP_POP_32_S_10_5 ... R_LARCH_SOP_POP_32_U]	= apply_r_larch_sop_imm_field,
	[R_LARCH_ADD32 ... R_LARCH_SUB64]			= apply_r_larch_add_sub,
};

int apply_relocate_add(Elf_Shdr *sechdrs, const char *strtab,
		       unsigned int symindex, unsigned int relsec,
		       struct module *me)
{
	s64 la_rela_stack[LA_RELA_STACK_DEPTH];
	size_t la_rela_stack_top = 0;
	Elf_Rela *rel = (void *) sechdrs[relsec].sh_addr;
	reloc_rela_handler handler;
	Elf_Sym *sym;
	void *location;
	unsigned int i, type;
	Elf_Addr v;
	int err;

	pr_debug("%s: Applying relocate section %u to %u\n", __func__, relsec,
	       sechdrs[relsec].sh_info);

	la_rela_stack_top = 0;
	for (i = 0; i < sechdrs[relsec].sh_size / sizeof(*rel); i++) {
		/* This is where to make the change */
		location = (void *)sechdrs[sechdrs[relsec].sh_info].sh_addr + rel[i].r_offset;
		/* This is the symbol it is referring to */
		sym = (Elf_Sym *)sechdrs[symindex].sh_addr + ELF_R_SYM(rel[i].r_info);
		if (IS_ERR_VALUE(sym->st_value)) {
			/* Ignore unresolved weak symbol */
			if (ELF_ST_BIND(sym->st_info) == STB_WEAK)
				continue;
			pr_warn("%s: Unknown symbol %s\n", me->name, strtab + sym->st_name);
			return -ENOENT;
		}

		type = ELF_R_TYPE(rel[i].r_info);

		if (type < ARRAY_SIZE(reloc_rela_handlers))
			handler = reloc_rela_handlers[type];
		else
			handler = NULL;

		if (!handler) {
			pr_err("%s: Unknown relocation type %u\n", me->name, type);
			return -EINVAL;
		}

		pr_debug("type %d st_value %llx r_addend %llx loc %llx\n",
		       (int)ELF_R_TYPE(rel[i].r_info),
		       sym->st_value, rel[i].r_addend, (u64)location);

		v = sym->st_value + rel[i].r_addend;
		if (type == R_LARCH_SOP_PUSH_PLT_PCREL) {
			err = apply_r_larch_sop_push_plt_pcrel(me, sechdrs, location,
						v, la_rela_stack, &la_rela_stack_top, type);
		} else {
			err = handler(me, location, v, la_rela_stack, &la_rela_stack_top, type);
		}
		if (err)
			return err;
	}

	return 0;
}

static int module_init_ftrace_plt(const Elf_Ehdr *hdr,
				  const Elf_Shdr *sechdrs,
				  struct module *mod)
{
#ifdef CONFIG_DYNAMIC_FTRACE
	struct plt_entry *ftrace_plts;

	ftrace_plts = (void *)sechdrs->sh_addr;

	ftrace_plts[FTRACE_PLT_IDX] = emit_plt_entry(FTRACE_ADDR);

	if (IS_ENABLED(CONFIG_DYNAMIC_FTRACE_WITH_REGS))
		ftrace_plts[FTRACE_REGS_PLT_IDX] = emit_plt_entry(FTRACE_REGS_ADDR);

	mod->arch.ftrace_trampolines = ftrace_plts;
#endif
	return 0;
}

int module_finalize(const Elf_Ehdr *hdr,
		    const Elf_Shdr *sechdrs,
		    struct module *me)
{
	char *secstrings;
	const Elf_Shdr *s, *orc = NULL, *orc_ip = NULL, *alt = NULL, *ftrace = NULL;

	secstrings = (void *)hdr + sechdrs[hdr->e_shstrndx].sh_offset;

	for (s = sechdrs; s < sechdrs + hdr->e_shnum; s++) {
		if (!strcmp(".altinstructions", secstrings + s->sh_name))
			alt = s;
		if (!strcmp(".orc_unwind", secstrings + s->sh_name))
			orc = s;
		if (!strcmp(".orc_unwind_ip", secstrings + s->sh_name))
			orc_ip = s;
		if (!strcmp(".ftrace_trampoline", secstrings + s->sh_name))
			ftrace = s;
	}

	if (alt) {
		/* patch .altinstructions */
		void *aseg = (void *)alt->sh_addr;
		apply_alternatives(aseg, aseg + alt->sh_size);
	}

	if (orc && orc_ip)
		unwind_module_init(me, (void *)orc_ip->sh_addr, orc_ip->sh_size,
				   (void *)orc->sh_addr, orc->sh_size);

	return module_init_ftrace_plt(hdr, ftrace, me);

}

void *module_alloc(unsigned long size)
{
	return __vmalloc_node_range(size, 1, MODULES_VADDR,
				    MODULES_END, GFP_KERNEL,
				    PAGE_KERNEL_EXEC, 0, NUMA_NO_NODE,
				    __builtin_return_address(0));
}
