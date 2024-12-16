/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2003, 2004 Ralf Baechle
 * Copyright (C) 2004  Maciej W. Rozycki
 * Copyright (C) 2020 Loongson Technology Corporation Limited
 */
#ifndef __ASM_CPU_FEATURES_H
#define __ASM_CPU_FEATURES_H

#include <asm/cpu.h>
#include <asm/cpu-info.h>

#define cpu_opt(opt)			(cpu_data[0].options & (opt))
#define cpu_has(feat)			(cpu_data[0].options & BIT_ULL(feat))

#define cpu_has_loongarch		(cpu_has_loongarch32 | cpu_has_loongarch64)
#define cpu_has_loongarch32		(cpu_data[0].isa_level & LOONGARCH_CPU_ISA_32BIT)
#define cpu_has_loongarch64		(cpu_data[0].isa_level & LOONGARCH_CPU_ISA_64BIT)

#ifdef CONFIG_32BIT
# define cpu_has_64bits			(cpu_data[0].isa_level & LOONGARCH_CPU_ISA_64BIT)
# define cpu_vabits			31
# define cpu_pabits			31
#endif

#ifdef CONFIG_64BIT
# define cpu_has_64bits			1
# define cpu_vabits			cpu_data[0].vabits
# define cpu_pabits			cpu_data[0].pabits
# define __NEED_ADDRBITS_PROBE
#endif

/*
 * SMP assumption: Options of CPU 0 are a superset of all processors.
 * This is true for all known LoongArch systems.
 */
#define cpu_has_cpucfg		cpu_opt(LOONGARCH_CPU_CPUCFG)
#define cpu_has_lam	    	cpu_opt(LOONGARCH_CPU_LAM)
#define cpu_has_ual	    	cpu_opt(LOONGARCH_CPU_UAL)
#define cpu_has_fpu		cpu_opt(LOONGARCH_CPU_FPU)
#define cpu_has_lsx		cpu_opt(LOONGARCH_CPU_LSX)
#define cpu_has_lasx		cpu_opt(LOONGARCH_CPU_LASX)
#define cpu_has_crc32		cpu_opt(LOONGARCH_CPU_CRC32)
#define cpu_has_complex		cpu_opt(LOONGARCH_CPU_COMPLEX)
#define cpu_has_crypto		cpu_opt(LOONGARCH_CPU_CRYPTO)
#define cpu_has_lvz		cpu_opt(LOONGARCH_CPU_LVZ)
#define cpu_has_lbt_x86		cpu_opt(LOONGARCH_CPU_LBT_X86)
#define cpu_has_lbt_arm		cpu_opt(LOONGARCH_CPU_LBT_ARM)
#define cpu_has_lbt_mips	cpu_opt(LOONGARCH_CPU_LBT_MIPS)
#define cpu_has_lbt		(cpu_has_lbt_x86|cpu_has_lbt_arm|cpu_has_lbt_mips)
#define cpu_has_csr		cpu_opt(LOONGARCH_CPU_CSR)
#define cpu_has_tlb		cpu_opt(LOONGARCH_CPU_TLB)
#define cpu_has_watch		cpu_opt(LOONGARCH_CPU_WATCH)
#define cpu_has_vint		cpu_opt(LOONGARCH_CPU_VINT)
#define cpu_has_csripi		cpu_opt(LOONGARCH_CPU_CSRIPI)
#define cpu_has_extioi		cpu_opt(LOONGARCH_CPU_EXTIOI)
#define cpu_has_prefetch	cpu_opt(LOONGARCH_CPU_PREFETCH)
#define cpu_has_pmp		cpu_opt(LOONGARCH_CPU_PMP)
#define cpu_has_perf		cpu_opt(LOONGARCH_CPU_PMP)
#define cpu_has_scalefreq	cpu_opt(LOONGARCH_CPU_SCALEFREQ)
#define cpu_has_guestid		cpu_opt(LOONGARCH_CPU_GUESTID)
#define cpu_has_hypervisor	cpu_opt(LOONGARCH_CPU_HYPERVISOR)
#define cpu_has_intdecode	cpu_opt(LOONGARCH_CPU_EIODECODE)
#define cpu_has_flatmode	cpu_opt(LOONGARCH_CPU_FLATMODE)
#define cpu_has_ptw		cpu_opt(LOONGARCH_CPU_PTW)





#define cpu_has_matc_guest	(cpu_data[0].guest_cfg & (1 << 0))
#define cpu_has_matc_root	(cpu_data[0].guest_cfg & (1 << 1))
#define cpu_has_matc_nest	(cpu_data[0].guest_cfg & (1 << 2))
#define cpu_has_sitp		(cpu_data[0].guest_cfg & (1 << 6))
#define cpu_has_titp		(cpu_data[0].guest_cfg & (1 << 8))
#define cpu_has_toep		(cpu_data[0].guest_cfg & (1 << 10))
#define cpu_has_topp		(cpu_data[0].guest_cfg & (1 << 12))
#define cpu_has_torup		(cpu_data[0].guest_cfg & (1 << 14))
#define cpu_has_gcip_all	(cpu_data[0].guest_cfg & (1 << 16))
#define cpu_has_gcip_hit	(cpu_data[0].guest_cfg & (1 << 17))
#define cpu_has_gcip_secure	(cpu_data[0].guest_cfg & (1 << 18))

/*
 * Guest capabilities
 */
#define cpu_guest_has_conf1	(cpu_data[0].guest.conf & (1 << 1))
#define cpu_guest_has_conf2	(cpu_data[0].guest.conf & (1 << 2))
#define cpu_guest_has_conf3	(cpu_data[0].guest.conf & (1 << 3))
#define cpu_guest_has_fpu	(cpu_data[0].guest.options & LOONGARCH_CPU_FPU)
#define cpu_guest_has_perf	(cpu_data[0].guest.options & LOONGARCH_CPU_PMP)
#define cpu_guest_has_watch	(cpu_data[0].guest.options & LOONGARCH_CPU_WATCH)
#define cpu_guest_has_lsx	(cpu_data[0].guest.ases & LOONGARCH_ASE_LSX)
#define cpu_guest_has_kscr(n)	(cpu_data[0].guest.kscratch_mask & (1u << (n)))

/*
 * Guest dynamic capabilities
 */
#define cpu_guest_has_dyn_fpu	(cpu_data[0].guest.options_dyn & LOONGARCH_CPU_FPU)
#define cpu_guest_has_dyn_perf	(cpu_data[0].guest.options_dyn & LOONGARCH_CPU_PMP)
#define cpu_guest_has_dyn_lsx	(cpu_data[0].guest.ases_dyn & LOONGARCH_ASE_LSX)

#endif /* __ASM_CPU_FEATURES_H */
