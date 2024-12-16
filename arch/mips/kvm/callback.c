/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2012  MIPS Technologies, Inc.  All rights reserved.
 * Authors: Yann Le Du <ledu@kymasys.com>
 */

#include <linux/export.h>
#include <linux/kvm_host.h>
#include <asm/fpu.h>
#include <asm/tlbflush.h>
#include <asm/time.h>
#include <asm/cacheflush.h>
#include <asm/pgtable-64.h>
#include <asm/tlbex.h>
#include <asm/msa.h>

struct kvm_mips_callbacks *kvm_mips_callbacks;
EXPORT_SYMBOL_GPL(kvm_mips_callbacks);
struct kvm_timer_callbacks *kvm_timer_callbacks;
EXPORT_SYMBOL_GPL(kvm_timer_callbacks);
extern void flush_tlb_all(void);
EXPORT_SYMBOL(flush_tlb_all);
extern unsigned int mips_hpt_frequency;
extern unsigned long ebase;
extern void (*flush_icache_range)(unsigned long start, unsigned long end);
extern void (*local_flush_icache_range)(unsigned long start, unsigned long end);
#ifndef __PAGETABLE_PMD_FOLDED
extern pmd_t invalid_pmd_table[PTRS_PER_PMD];
#endif
 extern u32 tlbmiss_handler_setup_pgd[];
extern void __kvm_save_fpu(struct kvm_vcpu_arch *vcpu);
extern void __kvm_restore_fpu(struct kvm_vcpu_arch *vcpu);
extern void __kvm_restore_fcsr(struct kvm_vcpu_arch *vcpu);
EXPORT_SYMBOL(__kvm_save_fpu);
EXPORT_SYMBOL(__kvm_restore_fpu);
EXPORT_SYMBOL(__kvm_restore_fcsr);
#ifdef CONFIG_CPU_HAS_MSA
extern void __kvm_restore_msacsr(struct kvm_vcpu_arch *vcpu);
EXPORT_SYMBOL(__kvm_restore_msacsr);
extern void __kvm_restore_msa(struct kvm_vcpu_arch *vcpu);
EXPORT_SYMBOL(__kvm_restore_msa);
extern void __kvm_save_msa(struct kvm_vcpu_arch *vcpu);
EXPORT_SYMBOL(__kvm_save_msa);
extern void __kvm_restore_msa_upper(struct kvm_vcpu_arch *vcpu);
EXPORT_SYMBOL(__kvm_restore_msa_upper);
#endif
#ifdef CONFIG_CPU_HAS_LASX
EXPORT_SYMBOL(_save_lasx);
extern void __kvm_restore_lasx(struct kvm_vcpu_arch *vcpu);
EXPORT_SYMBOL(__kvm_restore_lasx);
extern void __kvm_save_lasx(struct kvm_vcpu_arch *vcpu);
EXPORT_SYMBOL(__kvm_save_lasx);
extern void __kvm_restore_lasx_upper(struct kvm_vcpu_arch *vcpu);
EXPORT_SYMBOL(__kvm_restore_lasx_upper);
extern void __kvm_restore_lasx_uppest(struct kvm_vcpu_arch *vcpu);
EXPORT_SYMBOL(__kvm_restore_lasx_uppest);
#endif
#ifdef CONFIG_MIPS_HUGE_TLB_SUPPORT
extern int pmd_huge(pmd_t pmd);
EXPORT_SYMBOL(pmd_huge);
extern int pud_huge(pud_t pud);
EXPORT_SYMBOL(pud_huge);
#endif
