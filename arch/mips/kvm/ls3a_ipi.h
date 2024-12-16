/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * KVM/LS_VZ: Support for LS3A3000 IPI interrupt virtualization extensions
 *
 * Copyright (C) 2019 Loongson Corp.
 * Author: Chen Zhu <zhuchen@loongon.cn>
 */

#ifndef __LS3A_KVM_IPI_H
#define __LS3A_KVM_IPI_H

#include <linux/mm_types.h>
#include <linux/hrtimer.h>
#include <linux/kvm_host.h>
#include <linux/spinlock.h>

#include <kvm/iodev.h>

typedef struct gipi_single {
	uint32_t status;
	uint32_t en;
	uint32_t set;
	uint32_t clear;
	uint64_t buf[4];
} gipi_single;

typedef struct gipiState {
	gipi_single core[16];
} gipiState;

struct ls3a_kvm_ipi;

typedef struct ipi_io_device {
	struct ls3a_kvm_ipi *ipi;
	struct kvm_io_device device;
	int nodeNum;
} ipi_io_device;

struct ls3a_kvm_ipi {
	spinlock_t lock;
	struct kvm *kvm;
	gipiState ls3a_gipistate;
	int nodeNum;
	ipi_io_device dev_ls3a_ipi[4];
};

#define SMP_MAILBOX            0x3ff01000ULL

#define CORE0_STATUS_OFF       0x000
#define CORE0_EN_OFF           0x004
#define CORE0_SET_OFF          0x008
#define CORE0_CLEAR_OFF        0x00c
#define CORE0_BUF_20           0x020
#define CORE0_BUF_28           0x028
#define CORE0_BUF_30           0x030
#define CORE0_BUF_38           0x038

#define CORE1_STATUS_OFF       0x100
#define CORE1_EN_OFF           0x104
#define CORE1_SET_OFF          0x108
#define CORE1_CLEAR_OFF        0x10c
#define CORE1_BUF_20           0x120
#define CORE1_BUF_28           0x128
#define CORE1_BUF_30           0x130
#define CORE1_BUF_38           0x138

#define CORE2_STATUS_OFF       0x200
#define CORE2_EN_OFF           0x204
#define CORE2_SET_OFF          0x208
#define CORE2_CLEAR_OFF        0x20c
#define CORE2_BUF_20           0x220
#define CORE2_BUF_28           0x228
#define CORE2_BUF_30           0x230
#define CORE2_BUF_38           0x238

#define CORE3_STATUS_OFF       0x300
#define CORE3_EN_OFF           0x304
#define CORE3_SET_OFF          0x308
#define CORE3_CLEAR_OFF        0x30c
#define CORE3_BUF_20           0x320
#define CORE3_BUF_28           0x328
#define CORE3_BUF_30           0x330
#define CORE3_BUF_38           0x338

static inline struct ls3a_kvm_ipi *ls3a_ipi_irqchip(struct kvm *kvm)
{
	return kvm->arch.v_gipi;
}

static inline int ls3a_ipi_in_kernel(struct kvm *kvm)
{
	int ret;

	ret = (ls3a_ipi_irqchip(kvm) != NULL);
	return ret;
}

struct ls3a_kvm_ipi * kvm_create_ls3a_ipi(struct kvm *kvm);
int kvm_set_ls3a_ipi(struct kvm *kvm, struct loongson_gipiState *state);
int kvm_get_ls3a_ipi(struct kvm *kvm, struct loongson_gipiState *state);
void kvm_destroy_ls3a_ipi(struct ls3a_kvm_ipi *vipi);
#endif
