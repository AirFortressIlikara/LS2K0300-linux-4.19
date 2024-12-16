/*
 * Copyright (C) 2015 Josh Poimboeuf <jpoimboe@redhat.com>
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

#include <stdio.h>
#include <stdlib.h>

#define unlikely(cond) (cond)
#include <asm/insn.h>
#include "lib/inat.c"
#include "lib/insn.c"

#include "../../check.h"
#include "../../elf.h"
#include "../../arch.h"
#include "../../warn.h"
#include <asm/orc_types.h>

static unsigned char op_to_cfi_reg[][2] = {
	{CFI_AX, CFI_R8},
	{CFI_CX, CFI_R9},
	{CFI_DX, CFI_R10},
	{CFI_BX, CFI_R11},
	{CFI_SP, CFI_R12},
	{CFI_BP, CFI_R13},
	{CFI_SI, CFI_R14},
	{CFI_DI, CFI_R15},
};

static int is_x86_64(struct elf *elf)
{
	switch (elf->ehdr.e_machine) {
	case EM_X86_64:
		return 1;
	case EM_386:
		return 0;
	default:
		WARN("unexpected ELF machine type %d", elf->ehdr.e_machine);
		return -1;
	}
}

bool arch_callee_saved_reg(unsigned char reg)
{
	switch (reg) {
	case CFI_BP:
	case CFI_BX:
	case CFI_R12:
	case CFI_R13:
	case CFI_R14:
	case CFI_R15:
		return true;

	case CFI_AX:
	case CFI_CX:
	case CFI_DX:
	case CFI_SI:
	case CFI_DI:
	case CFI_SP:
	case CFI_R8:
	case CFI_R9:
	case CFI_R10:
	case CFI_R11:
	case CFI_RA:
	default:
		return false;
	}
}

unsigned long arch_dest_rela_offset(int addend)
{
	return addend + 4;
}

unsigned long arch_jump_destination(struct instruction *insn)
{
	return insn->offset + insn->len + insn->immediate;
}

int arch_decode_instruction(struct elf *elf, struct section *sec,
			    unsigned long offset, unsigned int maxlen,
			    unsigned int *len, unsigned char *type,
			    unsigned long *immediate, struct stack_op *op)
{
	struct insn insn;
	int x86_64, sign;
	unsigned char op1, op2, rex = 0, rex_b = 0, rex_r = 0, rex_w = 0,
		      rex_x = 0, modrm = 0, modrm_mod = 0, modrm_rm = 0,
		      modrm_reg = 0, sib = 0;

	x86_64 = is_x86_64(elf);
	if (x86_64 == -1)
		return -1;

	insn_init(&insn, sec->data->d_buf + offset, maxlen, x86_64);
	insn_get_length(&insn);

	if (!insn_complete(&insn)) {
		WARN_FUNC("can't decode instruction", sec, offset);
		return -1;
	}

	*len = insn.length;
	*type = INSN_OTHER;

	if (insn.vex_prefix.nbytes)
		return 0;

	op1 = insn.opcode.bytes[0];
	op2 = insn.opcode.bytes[1];

	if (insn.rex_prefix.nbytes) {
		rex = insn.rex_prefix.bytes[0];
		rex_w = X86_REX_W(rex) >> 3;
		rex_r = X86_REX_R(rex) >> 2;
		rex_x = X86_REX_X(rex) >> 1;
		rex_b = X86_REX_B(rex);
	}

	if (insn.modrm.nbytes) {
		modrm = insn.modrm.bytes[0];
		modrm_mod = X86_MODRM_MOD(modrm);
		modrm_reg = X86_MODRM_REG(modrm);
		modrm_rm = X86_MODRM_RM(modrm);
	}

	if (insn.sib.nbytes)
		sib = insn.sib.bytes[0];

	switch (op1) {

	case 0x1:
	case 0x29:
		if (rex_w && !rex_b && modrm_mod == 3 && modrm_rm == 4) {

			/* add/sub reg, %rsp */
			*type = INSN_STACK;
			op->src.type = OP_SRC_ADD;
			op->src.reg = op_to_cfi_reg[modrm_reg][rex_r];
			op->dest.type = OP_DEST_REG;
			op->dest.reg = CFI_SP;
		}
		break;

	case 0x50 ... 0x57:

		/* push reg */
		*type = INSN_STACK;
		op->src.type = OP_SRC_REG;
		op->src.reg = op_to_cfi_reg[op1 & 0x7][rex_b];
		op->dest.type = OP_DEST_PUSH;

		break;

	case 0x58 ... 0x5f:

		/* pop reg */
		*type = INSN_STACK;
		op->src.type = OP_SRC_POP;
		op->dest.type = OP_DEST_REG;
		op->dest.reg = op_to_cfi_reg[op1 & 0x7][rex_b];

		break;

	case 0x68:
	case 0x6a:
		/* push immediate */
		*type = INSN_STACK;
		op->src.type = OP_SRC_CONST;
		op->dest.type = OP_DEST_PUSH;
		break;

	case 0x70 ... 0x7f:
		*type = INSN_JUMP_CONDITIONAL;
		break;

	case 0x81:
	case 0x83:
		if (rex != 0x48)
			break;

		if (modrm == 0xe4) {
			/* and imm, %rsp */
			*type = INSN_STACK;
			op->src.type = OP_SRC_AND;
			op->src.reg = CFI_SP;
			op->src.offset = insn.immediate.value;
			op->dest.type = OP_DEST_REG;
			op->dest.reg = CFI_SP;
			break;
		}

		if (modrm == 0xc4)
			sign = 1;
		else if (modrm == 0xec)
			sign = -1;
		else
			break;

		/* add/sub imm, %rsp */
		*type = INSN_STACK;
		op->src.type = OP_SRC_ADD;
		op->src.reg = CFI_SP;
		op->src.offset = insn.immediate.value * sign;
		op->dest.type = OP_DEST_REG;
		op->dest.reg = CFI_SP;
		break;

	case 0x89:
		if (rex_w && !rex_r && modrm_mod == 3 && modrm_reg == 4) {

			/* mov %rsp, reg */
			*type = INSN_STACK;
			op->src.type = OP_SRC_REG;
			op->src.reg = CFI_SP;
			op->dest.type = OP_DEST_REG;
			op->dest.reg = op_to_cfi_reg[modrm_rm][rex_b];
			break;
		}

		if (rex_w && !rex_b && modrm_mod == 3 && modrm_rm == 4) {

			/* mov reg, %rsp */
			*type = INSN_STACK;
			op->src.type = OP_SRC_REG;
			op->src.reg = op_to_cfi_reg[modrm_reg][rex_r];
			op->dest.type = OP_DEST_REG;
			op->dest.reg = CFI_SP;
			break;
		}

		/* fallthrough */
	case 0x88:
		if (!rex_b &&
		    (modrm_mod == 1 || modrm_mod == 2) && modrm_rm == 5) {

			/* mov reg, disp(%rbp) */
			*type = INSN_STACK;
			op->src.type = OP_SRC_REG;
			op->src.reg = op_to_cfi_reg[modrm_reg][rex_r];
			op->dest.type = OP_DEST_REG_INDIRECT;
			op->dest.reg = CFI_BP;
			op->dest.offset = insn.displacement.value;

		} else if (rex_w && !rex_b && modrm_rm == 4 && sib == 0x24) {

			/* mov reg, disp(%rsp) */
			*type = INSN_STACK;
			op->src.type = OP_SRC_REG;
			op->src.reg = op_to_cfi_reg[modrm_reg][rex_r];
			op->dest.type = OP_DEST_REG_INDIRECT;
			op->dest.reg = CFI_SP;
			op->dest.offset = insn.displacement.value;
		}

		break;

	case 0x8b:
		if (rex_w && !rex_b && modrm_mod == 1 && modrm_rm == 5) {

			/* mov disp(%rbp), reg */
			*type = INSN_STACK;
			op->src.type = OP_SRC_REG_INDIRECT;
			op->src.reg = CFI_BP;
			op->src.offset = insn.displacement.value;
			op->dest.type = OP_DEST_REG;
			op->dest.reg = op_to_cfi_reg[modrm_reg][rex_r];

		} else if (rex_w && !rex_b && sib == 0x24 &&
			   modrm_mod != 3 && modrm_rm == 4) {

			/* mov disp(%rsp), reg */
			*type = INSN_STACK;
			op->src.type = OP_SRC_REG_INDIRECT;
			op->src.reg = CFI_SP;
			op->src.offset = insn.displacement.value;
			op->dest.type = OP_DEST_REG;
			op->dest.reg = op_to_cfi_reg[modrm_reg][rex_r];
		}

		break;

	case 0x8d:
		if (sib == 0x24 && rex_w && !rex_b && !rex_x) {

			*type = INSN_STACK;
			if (!insn.displacement.value) {
				/* lea (%rsp), reg */
				op->src.type = OP_SRC_REG;
			} else {
				/* lea disp(%rsp), reg */
				op->src.type = OP_SRC_ADD;
				op->src.offset = insn.displacement.value;
			}
			op->src.reg = CFI_SP;
			op->dest.type = OP_DEST_REG;
			op->dest.reg = op_to_cfi_reg[modrm_reg][rex_r];

		} else if (rex == 0x48 && modrm == 0x65) {

			/* lea disp(%rbp), %rsp */
			*type = INSN_STACK;
			op->src.type = OP_SRC_ADD;
			op->src.reg = CFI_BP;
			op->src.offset = insn.displacement.value;
			op->dest.type = OP_DEST_REG;
			op->dest.reg = CFI_SP;

		} else if (rex == 0x49 && modrm == 0x62 &&
			   insn.displacement.value == -8) {

			/*
			 * lea -0x8(%r10), %rsp
			 *
			 * Restoring rsp back to its original value after a
			 * stack realignment.
			 */
			*type = INSN_STACK;
			op->src.type = OP_SRC_ADD;
			op->src.reg = CFI_R10;
			op->src.offset = -8;
			op->dest.type = OP_DEST_REG;
			op->dest.reg = CFI_SP;

		} else if (rex == 0x49 && modrm == 0x65 &&
			   insn.displacement.value == -16) {

			/*
			 * lea -0x10(%r13), %rsp
			 *
			 * Restoring rsp back to its original value after a
			 * stack realignment.
			 */
			*type = INSN_STACK;
			op->src.type = OP_SRC_ADD;
			op->src.reg = CFI_R13;
			op->src.offset = -16;
			op->dest.type = OP_DEST_REG;
			op->dest.reg = CFI_SP;
		}

		break;

	case 0x8f:
		/* pop to mem */
		*type = INSN_STACK;
		op->src.type = OP_SRC_POP;
		op->dest.type = OP_DEST_MEM;
		break;

	case 0x90:
		*type = INSN_NOP;
		break;

	case 0x9c:
		/* pushf */
		*type = INSN_STACK;
		op->src.type = OP_SRC_CONST;
		op->dest.type = OP_DEST_PUSH;
		break;

	case 0x9d:
		/* popf */
		*type = INSN_STACK;
		op->src.type = OP_SRC_POP;
		op->dest.type = OP_DEST_MEM;
		break;

	case 0x0f:

		if (op2 >= 0x80 && op2 <= 0x8f) {

			*type = INSN_JUMP_CONDITIONAL;

		} else if (op2 == 0x05 || op2 == 0x07 || op2 == 0x34 ||
			   op2 == 0x35) {

			/* sysenter, sysret */
			*type = INSN_CONTEXT_SWITCH;

		} else if (op2 == 0x0b || op2 == 0xb9) {

			/* ud2 */
			*type = INSN_BUG;

		} else if (op2 == 0x0d || op2 == 0x1f) {

			/* nopl/nopw */
			*type = INSN_NOP;

		} else if (op2 == 0xa0 || op2 == 0xa8) {

			/* push fs/gs */
			*type = INSN_STACK;
			op->src.type = OP_SRC_CONST;
			op->dest.type = OP_DEST_PUSH;

		} else if (op2 == 0xa1 || op2 == 0xa9) {

			/* pop fs/gs */
			*type = INSN_STACK;
			op->src.type = OP_SRC_POP;
			op->dest.type = OP_DEST_MEM;
		}

		break;

	case 0xc9:
		/*
		 * leave
		 *
		 * equivalent to:
		 * mov bp, sp
		 * pop bp
		 */
		*type = INSN_STACK;
		op->dest.type = OP_DEST_LEAVE;

		break;

	case 0xe3:
		/* jecxz/jrcxz */
		*type = INSN_JUMP_CONDITIONAL;
		break;

	case 0xe9:
	case 0xeb:
		*type = INSN_JUMP_UNCONDITIONAL;
		break;

	case 0xc2:
	case 0xc3:
		*type = INSN_RETURN;
		break;

	case 0xca: /* retf */
	case 0xcb: /* retf */
	case 0xcf: /* iret */
		*type = INSN_CONTEXT_SWITCH;
		break;

	case 0xe8:
		*type = INSN_CALL;
		break;

	case 0xff:
		if (modrm_reg == 2 || modrm_reg == 3)

			*type = INSN_CALL_DYNAMIC;

		else if (modrm_reg == 4)

			*type = INSN_JUMP_DYNAMIC;

		else if (modrm_reg == 5)

			/* jmpf */
			*type = INSN_CONTEXT_SWITCH;

		else if (modrm_reg == 6) {

			/* push from mem */
			*type = INSN_STACK;
			op->src.type = OP_SRC_CONST;
			op->dest.type = OP_DEST_PUSH;
		}

		break;

	default:
		break;
	}

	*immediate = insn.immediate.nbytes ? insn.immediate.value : 0;

	return 0;
}

void arch_initial_func_cfi_state(struct cfi_state *state)
{
	int i;

	for (i = 0; i < CFI_NUM_REGS; i++) {
		state->regs[i].base = CFI_UNDEFINED;
		state->regs[i].offset = 0;
	}

	/* initial CFA (call frame address) */
	state->cfa.base = CFI_SP;
	state->cfa.offset = 8;

	/* initial RA (return address) */
	state->regs[16].base = CFI_CFA;
	state->regs[16].offset = -8;
}

int arch_decode_hint_reg(struct cfi_reg *cfa, u8 sp_reg)
{
	switch (sp_reg) {
	case ORC_REG_UNDEFINED:
		cfa->base = CFI_UNDEFINED;
		break;
	case ORC_REG_SP:
		cfa->base = CFI_SP;
		break;
	case ORC_REG_BP:
		cfa->base = CFI_BP;
		break;
	case ORC_REG_SP_INDIRECT:
		cfa->base = CFI_SP_INDIRECT;
		break;
	case ORC_REG_R10:
		cfa->base = CFI_R10;
		break;
	case ORC_REG_R13:
		cfa->base = CFI_R13;
		break;
	case ORC_REG_DI:
		cfa->base = CFI_DI;
		break;
	case ORC_REG_DX:
		cfa->base = CFI_DX;
		break;
	default:
		return -1;
	}

	return 0;
}

bool arch_has_valid_stack_frame(struct insn_state *state)
{
	if (state->cfa.base == CFI_BP && state->regs[CFI_BP].base == CFI_CFA &&
	    state->regs[CFI_BP].offset == -16)
		return true;

	if (state->drap && state->regs[CFI_BP].base == CFI_BP)
		return true;

	return false;
}

static int update_insn_state_regs(struct instruction *insn, struct insn_state *state)
{
	struct cfi_reg *cfa = &state->cfa;
	struct stack_op *op = &insn->stack_op;

	if (cfa->base != CFI_SP && cfa->base != CFI_SP_INDIRECT)
		return 0;

	/* push */
	if (op->dest.type == OP_DEST_PUSH)
		cfa->offset += 8;

	/* pop */
	if (op->src.type == OP_SRC_POP)
		cfa->offset -= 8;

	/* add immediate to sp */
	if (op->dest.type == OP_DEST_REG && op->src.type == OP_SRC_ADD &&
	    op->dest.reg == CFI_SP && op->src.reg == CFI_SP)
		cfa->offset -= op->src.offset;

	return 0;
}

static void save_reg(struct insn_state *state, unsigned char reg, int base,
		     int offset)
{
	if (arch_callee_saved_reg(reg) &&
	    state->regs[reg].base == CFI_UNDEFINED) {
		state->regs[reg].base = base;
		state->regs[reg].offset = offset;
	}
}

static void restore_reg(struct insn_state *state, unsigned char reg)
{
	state->regs[reg].base = CFI_UNDEFINED;
	state->regs[reg].offset = 0;
}

/*
 * A note about DRAP stack alignment:
 *
 * GCC has the concept of a DRAP register, which is used to help keep track of
 * the stack pointer when aligning the stack.  r10 or r13 is used as the DRAP
 * register.  The typical DRAP pattern is:
 *
 *   4c 8d 54 24 08		lea    0x8(%rsp),%r10
 *   48 83 e4 c0		and    $0xffffffffffffffc0,%rsp
 *   41 ff 72 f8		pushq  -0x8(%r10)
 *   55				push   %rbp
 *   48 89 e5			mov    %rsp,%rbp
 *				(more pushes)
 *   41 52			push   %r10
 *				...
 *   41 5a			pop    %r10
 *				(more pops)
 *   5d				pop    %rbp
 *   49 8d 62 f8		lea    -0x8(%r10),%rsp
 *   c3				retq
 *
 * There are some variations in the epilogues, like:
 *
 *   5b				pop    %rbx
 *   41 5a			pop    %r10
 *   41 5c			pop    %r12
 *   41 5d			pop    %r13
 *   41 5e			pop    %r14
 *   c9				leaveq
 *   49 8d 62 f8		lea    -0x8(%r10),%rsp
 *   c3				retq
 *
 * and:
 *
 *   4c 8b 55 e8		mov    -0x18(%rbp),%r10
 *   48 8b 5d e0		mov    -0x20(%rbp),%rbx
 *   4c 8b 65 f0		mov    -0x10(%rbp),%r12
 *   4c 8b 6d f8		mov    -0x8(%rbp),%r13
 *   c9				leaveq
 *   49 8d 62 f8		lea    -0x8(%r10),%rsp
 *   c3				retq
 *
 * Sometimes r13 is used as the DRAP register, in which case it's saved and
 * restored beforehand:
 *
 *   41 55			push   %r13
 *   4c 8d 6c 24 10		lea    0x10(%rsp),%r13
 *   48 83 e4 f0		and    $0xfffffffffffffff0,%rsp
 *				...
 *   49 8d 65 f0		lea    -0x10(%r13),%rsp
 *   41 5d			pop    %r13
 *   c3				retq
 */
int arch_update_insn_state(struct instruction *insn, struct insn_state *state, int no_fp)
{
	struct stack_op *op = &insn->stack_op;
	struct cfi_reg *cfa = &state->cfa;
	struct cfi_reg *regs = state->regs;

	/* stack operations don't make sense with an undefined CFA */
	if (cfa->base == CFI_UNDEFINED) {
		if (insn->func) {
			WARN_FUNC("undefined stack state", insn->sec, insn->offset);
			return -1;
		}
		return 0;
	}

	if (state->type == ORC_TYPE_REGS || state->type == ORC_TYPE_REGS_IRET)
		return update_insn_state_regs(insn, state);

	switch (op->dest.type) {

	case OP_DEST_REG:
		switch (op->src.type) {

		case OP_SRC_REG:
			if (op->src.reg == CFI_SP && op->dest.reg == CFI_BP &&
			    cfa->base == CFI_SP &&
			    regs[CFI_BP].base == CFI_CFA &&
			    regs[CFI_BP].offset == -cfa->offset) {

				/* mov %rsp, %rbp */
				cfa->base = op->dest.reg;
				state->bp_scratch = false;
			}

			else if (op->src.reg == CFI_SP &&
				 op->dest.reg == CFI_BP && state->drap) {

				/* drap: mov %rsp, %rbp */
				regs[CFI_BP].base = CFI_BP;
				regs[CFI_BP].offset = -state->stack_size;
				state->bp_scratch = false;
			}

			else if (op->src.reg == CFI_SP && cfa->base == CFI_SP) {

				/*
				 * mov %rsp, %reg
				 *
				 * This is needed for the rare case where GCC
				 * does:
				 *
				 *   mov    %rsp, %rax
				 *   ...
				 *   mov    %rax, %rsp
				 */
				state->vals[op->dest.reg].base = CFI_CFA;
				state->vals[op->dest.reg].offset = -state->stack_size;
			}

			else if (op->src.reg == CFI_BP && op->dest.reg == CFI_SP &&
				 cfa->base == CFI_BP) {

				/*
				 * mov %rbp, %rsp
				 *
				 * Restore the original stack pointer (Clang).
				 */
				state->stack_size = -state->regs[CFI_BP].offset;
			}

			else if (op->dest.reg == cfa->base) {

				/* mov %reg, %rsp */
				if (cfa->base == CFI_SP &&
				    state->vals[op->src.reg].base == CFI_CFA) {

					/*
					 * This is needed for the rare case
					 * where GCC does something dumb like:
					 *
					 *   lea    0x8(%rsp), %rcx
					 *   ...
					 *   mov    %rcx, %rsp
					 */
					cfa->offset = -state->vals[op->src.reg].offset;
					state->stack_size = cfa->offset;

				} else {
					cfa->base = CFI_UNDEFINED;
					cfa->offset = 0;
				}
			}

			break;

		case OP_SRC_ADD:
			if (op->dest.reg == CFI_SP && op->src.reg == CFI_SP) {

				/* add imm, %rsp */
				state->stack_size -= op->src.offset;
				if (cfa->base == CFI_SP)
					cfa->offset -= op->src.offset;
				break;
			}

			if (op->dest.reg == CFI_SP && op->src.reg == CFI_BP) {

				/* lea disp(%rbp), %rsp */
				state->stack_size = -(op->src.offset + regs[CFI_BP].offset);
				break;
			}

			if (op->src.reg == CFI_SP && cfa->base == CFI_SP) {

				/* drap: lea disp(%rsp), %drap */
				state->drap_reg = op->dest.reg;

				/*
				 * lea disp(%rsp), %reg
				 *
				 * This is needed for the rare case where GCC
				 * does something dumb like:
				 *
				 *   lea    0x8(%rsp), %rcx
				 *   ...
				 *   mov    %rcx, %rsp
				 */
				state->vals[op->dest.reg].base = CFI_CFA;
				state->vals[op->dest.reg].offset = \
					-state->stack_size + op->src.offset;

				break;
			}

			if (state->drap && op->dest.reg == CFI_SP &&
			    op->src.reg == state->drap_reg) {

				 /* drap: lea disp(%drap), %rsp */
				cfa->base = CFI_SP;
				cfa->offset = state->stack_size = -op->src.offset;
				state->drap_reg = CFI_UNDEFINED;
				state->drap = false;
				break;
			}

			if (op->dest.reg == state->cfa.base) {
				WARN_FUNC("unsupported stack register modification",
					  insn->sec, insn->offset);
				return -1;
			}

			break;

		case OP_SRC_AND:
			if (op->dest.reg != CFI_SP ||
			    (state->drap_reg != CFI_UNDEFINED && cfa->base != CFI_SP) ||
			    (state->drap_reg == CFI_UNDEFINED && cfa->base != CFI_BP)) {
				WARN_FUNC("unsupported stack pointer realignment",
					  insn->sec, insn->offset);
				return -1;
			}

			if (state->drap_reg != CFI_UNDEFINED) {
				/* drap: and imm, %rsp */
				cfa->base = state->drap_reg;
				cfa->offset = state->stack_size = 0;
				state->drap = true;
			}

			/*
			 * Older versions of GCC (4.8ish) realign the stack
			 * without DRAP, with a frame pointer.
			 */

			break;

		case OP_SRC_POP:
			if (!state->drap && op->dest.type == OP_DEST_REG &&
			    op->dest.reg == cfa->base) {

				/* pop %rbp */
				cfa->base = CFI_SP;
			}

			if (state->drap && cfa->base == CFI_BP_INDIRECT &&
			    op->dest.type == OP_DEST_REG &&
			    op->dest.reg == state->drap_reg &&
			    state->drap_offset == -state->stack_size) {

				/* drap: pop %drap */
				cfa->base = state->drap_reg;
				cfa->offset = 0;
				state->drap_offset = -1;

			} else if (regs[op->dest.reg].offset == -state->stack_size) {

				/* pop %reg */
				restore_reg(state, op->dest.reg);
			}

			state->stack_size -= 8;
			if (cfa->base == CFI_SP)
				cfa->offset -= 8;

			break;

		case OP_SRC_REG_INDIRECT:
			if (state->drap && op->src.reg == CFI_BP &&
			    op->src.offset == state->drap_offset) {

				/* drap: mov disp(%rbp), %drap */
				cfa->base = state->drap_reg;
				cfa->offset = 0;
				state->drap_offset = -1;
			}

			if (state->drap && op->src.reg == CFI_BP &&
			    op->src.offset == regs[op->dest.reg].offset) {

				/* drap: mov disp(%rbp), %reg */
				restore_reg(state, op->dest.reg);

			} else if (op->src.reg == cfa->base &&
			    op->src.offset == regs[op->dest.reg].offset + cfa->offset) {

				/* mov disp(%rbp), %reg */
				/* mov disp(%rsp), %reg */
				restore_reg(state, op->dest.reg);
			}

			break;

		default:
			WARN_FUNC("unknown stack-related instruction",
				  insn->sec, insn->offset);
			return -1;
		}

		break;

	case OP_DEST_PUSH:
		state->stack_size += 8;
		if (cfa->base == CFI_SP)
			cfa->offset += 8;

		if (op->src.type != OP_SRC_REG)
			break;

		if (state->drap) {
			if (op->src.reg == cfa->base && op->src.reg == state->drap_reg) {

				/* drap: push %drap */
				cfa->base = CFI_BP_INDIRECT;
				cfa->offset = -state->stack_size;

				/* save drap so we know when to restore it */
				state->drap_offset = -state->stack_size;

			} else if (op->src.reg == CFI_BP && cfa->base == state->drap_reg) {

				/* drap: push %rbp */
				state->stack_size = 0;

			} else if (regs[op->src.reg].base == CFI_UNDEFINED) {

				/* drap: push %reg */
				save_reg(state, op->src.reg, CFI_BP, -state->stack_size);
			}

		} else {

			/* push %reg */
			save_reg(state, op->src.reg, CFI_CFA, -state->stack_size);
		}

		/* detect when asm code uses rbp as a scratch register */
		if (!no_fp && insn->func && op->src.reg == CFI_BP &&
		    cfa->base != CFI_BP)
			state->bp_scratch = true;
		break;

	case OP_DEST_REG_INDIRECT:

		if (state->drap) {
			if (op->src.reg == cfa->base && op->src.reg == state->drap_reg) {

				/* drap: mov %drap, disp(%rbp) */
				cfa->base = CFI_BP_INDIRECT;
				cfa->offset = op->dest.offset;

				/* save drap offset so we know when to restore it */
				state->drap_offset = op->dest.offset;
			}

			else if (regs[op->src.reg].base == CFI_UNDEFINED) {

				/* drap: mov reg, disp(%rbp) */
				save_reg(state, op->src.reg, CFI_BP, op->dest.offset);
			}

		} else if (op->dest.reg == cfa->base) {

			/* mov reg, disp(%rbp) */
			/* mov reg, disp(%rsp) */
			save_reg(state, op->src.reg, CFI_CFA,
				 op->dest.offset - state->cfa.offset);
		}

		break;

	case OP_DEST_LEAVE:
		if ((!state->drap && cfa->base != CFI_BP) ||
		    (state->drap && cfa->base != state->drap_reg)) {
			WARN_FUNC("leave instruction with modified stack frame",
				  insn->sec, insn->offset);
			return -1;
		}

		/* leave (mov %rbp, %rsp; pop %rbp) */

		state->stack_size = -state->regs[CFI_BP].offset - 8;
		restore_reg(state, CFI_BP);

		if (!state->drap) {
			cfa->base = CFI_SP;
			cfa->offset -= 8;
		}

		break;

	case OP_DEST_MEM:
		if (op->src.type != OP_SRC_POP) {
			WARN_FUNC("unknown stack-related memory operation",
				  insn->sec, insn->offset);
			return -1;
		}

		/* pop mem */
		state->stack_size -= 8;
		if (cfa->base == CFI_SP)
			cfa->offset -= 8;

		break;

	default:
		WARN_FUNC("unknown stack-related instruction",
			  insn->sec, insn->offset);
		return -1;
	}

	return 0;
}

void arch_try_find_call(struct list_head *p_orbit_list, struct objtool_file *file,
			struct symbol *func, struct instruction *insn)
{
}
