/* SPDX-License-Identifier: GPL-2.0 */
#ifndef RELOCS_H
#define RELOCS_H

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <elf.h>
#include <byteswap.h>
#define USE_BSD
#include <endian.h>
#include <regex.h>

void die(char *fmt, ...);

#ifndef EM_LOONGARCH
#define EM_LOONGARCH	258
#endif
#define R_LARCH_NONE				0
#define R_LARCH_32 				1
#define R_LARCH_64				2
#define R_LARCH_MARK_LA				20
#define R_LARCH_MARK_PCREL			21
#define R_LARCH_SOP_PUSH_PCREL			22
#define R_LARCH_SOP_PUSH_ABSOLUTE		23
#define R_LARCH_SOP_PUSH_PLT_PCREL		29
#define R_LARCH_SOP_SUB				32
#define R_LARCH_SOP_SL				33
#define R_LARCH_SOP_SR				34
#define R_LARCH_SOP_ADD				35
#define R_LARCH_SOP_AND				36
#define R_LARCH_SOP_IF_ELSE			37
#define R_LARCH_SOP_POP_32_U_10_12		39
#define R_LARCH_SOP_POP_32_S_10_12		40
#define R_LARCH_SOP_POP_32_S_10_16		41
#define R_LARCH_SOP_POP_32_S_10_16_S2		42
#define R_LARCH_SOP_POP_32_S_5_20		43
#define R_LARCH_SOP_POP_32_S_0_5_10_16_S2	44
#define R_LARCH_SOP_POP_32_S_0_10_10_16_S2	45
#define R_LARCH_SOP_POP_32_U			46
#define R_LARCH_ADD32				50
#define R_LARCH_ADD64				51
#define R_LARCH_SUB32				55
#define R_LARCH_SUB64				56

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

enum symtype {
	S_ABS,
	S_REL,
	S_SEG,
	S_LIN,
	S_NSYMTYPES
};

void process_32(FILE *fp, int as_text, int as_bin,
		int show_reloc_info, int keep_relocs);
void process_64(FILE *fp, int as_text, int as_bin,
		int show_reloc_info, int keep_relocs);
#endif /* RELOCS_H */
