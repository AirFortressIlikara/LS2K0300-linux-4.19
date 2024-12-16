/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * KVM/MIPS: Hypercall handling.
 *
 * Copyright (C) 2015  Imagination Technologies Ltd.
 */

#include <linux/kernel.h>
#include <linux/kvm_host.h>
#include <linux/kvm_para.h>
#include "ls3a.h"

#define MAX_HYPCALL_ARGS	8

enum emulation_result kvm_mips_emul_hypcall(struct kvm_vcpu *vcpu,
					    union mips_instruction inst)
{
	unsigned int code = (inst.co_format.code >> 5) & 0x3ff;

	kvm_debug("[%#lx] HYPCALL %#03x\n", vcpu->arch.pc, code);

	switch (code) {
	case 0:
		return EMULATE_HYPERCALL;
	case 5:
		return EMULATE_DEBUG;
	default:
		return EMULATE_FAIL;
	};
}

static int kvm_mips_hypercall(struct kvm_vcpu *vcpu, unsigned long num,
			      const unsigned long *args, unsigned long *hret)
{
	struct kvm_run *run = vcpu->run;
	int ret;

	run->hypercall.nr = num;
	run->hypercall.args[0] = args[0];
	run->hypercall.args[1] = args[1];
	run->hypercall.args[2] = args[2];
	run->hypercall.args[3] = args[3];
	run->hypercall.args[4] = args[4];
	run->hypercall.args[5] = args[5];
	run->exit_reason = KVM_EXIT_HYPERCALL;
	ret = RESUME_HOST;
	return ret;
}

int kvm_mips_handle_hypcall(struct kvm_vcpu *vcpu)
{
	unsigned long num, args[MAX_HYPCALL_ARGS];

	/* read hypcall number and arguments */
	num = vcpu->arch.gprs[2];	/* v0 */
	args[0] = vcpu->arch.gprs[4];	/* a0 */
	args[1] = vcpu->arch.gprs[5];	/* a1 */
	args[2] = vcpu->arch.gprs[6];	/* a2 */
	args[3] = vcpu->arch.gprs[7];	/* a3 */
	args[4] = vcpu->arch.gprs[2];	/* tlb_miss/tlbl/tlbs/tlbm */
	args[5] = vcpu->arch.gprs[3];	/* EXCCODE/_TLBL/_TLBS/_MOD */

	return kvm_mips_hypercall(vcpu, num,
				  args, &vcpu->arch.gprs[2] /* v0 */);
}
