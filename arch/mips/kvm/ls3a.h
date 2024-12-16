/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * KVM/LS_VZ: Support for LS3A2000/LS3A3000 hardware virtualization extensions
 *
 * Copyright (C) 2017 Loongson Corp.
 * Authors: Huang Pei <huangpei@loongon.cn>
 * Author: Xing Li, lixing@loongson.cn
 */
#ifndef __KVM_MIPS_LS3A3000_H__
#define __KVM_MIPS_LS3A3000_H__

#include <linux/kernel.h>
#include <linux/kvm_host.h>
#include <linux/kvm_para.h>
#include "ls3a_ipi.h"
#include "ls7a_irq.h"
#include "ls3a_ext_irq.h"

void kvm_mips_tlbw(struct kvm_mips_tlb *ptlb);
int _kvm_mips_map_page_fast(struct kvm_vcpu *vcpu, unsigned long gpa,
				bool write_fault,
				pte_t *out_entry, pte_t *out_buddy);
int kvm_lsvz_map_page(struct kvm_vcpu *vcpu, unsigned long gpa,
				bool write_fault,unsigned long prot_bits,
				pte_t *out_entry, pte_t *out_buddy);
pte_t *kvm_mips_pte_for_gpa(struct kvm *kvm,
				struct kvm_mmu_memory_cache *cache,
				unsigned long addr);

int kvm_mips_guesttlb_lookup(struct kvm_vcpu *vcpu, unsigned long entryhi);
int kvm_mips_gva_to_hpa(struct kvm_vcpu *vcpu, unsigned long gva,
					       unsigned long *phpa);
int kvm_mips_handle_ls3a3000_vz_root_tlb_fault(unsigned long badvaddr,
                                      struct kvm_vcpu *vcpu, bool write_fault);
#endif
