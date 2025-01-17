// SPDX-License-Identifier: GPL-2.0
#include "relocs.h"

#define ELF_BITS 32

#define ELF_MACHINE		EM_LOONGARCH
#define ELF_MACHINE_NAME	"LOONGARCH32"
#define SHT_REL_TYPE		SHT_RELA
#define Elf_Rel			ElfW(Rela)

#define ELF_CLASS		ELFCLASS32
#define ELF_R_SYM(val)		ELF32_R_SYM(val)
#define ELF_R_TYPE(val)		ELF32_R_TYPE(val)
#define ELF_ST_TYPE(o)		ELF32_ST_TYPE(o)
#define ELF_ST_BIND(o)		ELF32_ST_BIND(o)
#define ELF_ST_VISIBILITY(o)	ELF32_ST_VISIBILITY(o)

#include "relocs.c"
