/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * currently used for Loongson3A Virtual EXT interrupt.
 * Mapped into the guest kernel @ KVM_GUEST_COMMPAGE_ADDR.
 *
 * Copyright (C) 2019  Loongson Technologies, Inc.  All rights reserved.
 * Authors: Chen Zhu <zhuchen@loongson.cn>
 */
#ifndef __LS3A_KVM_EXT_IRQ_H
#define __LS3A_KVM_EXT_IRQ_H

#include <linux/mm_types.h>
#include <linux/hrtimer.h>
#include <linux/kvm_host.h>
#include <linux/spinlock.h>

#include <kvm/iodev.h>
#define EXT_IRQ_OFFSET                  0x14A0
#define LS3A_EXT_IRQ_BASE		(0x1FE00000ULL + EXT_IRQ_OFFSET)

#define EXTIOI_NODETYPE_START		0x14A0
#define EXTIOI_NODETYPE_END		0x14C0
#define EXTIOI_IPMAP_START		0x14C0
#define EXTIOI_IPMAP_END		0x14C8
#define EXTIOI_ENABLE_START		0x1600
#define EXTIOI_ENABLE_END		0x1620
#define EXTIOI_BOUNCE_START		0x1680
#define EXTIOI_BOUNCE_END		0x16A0
#define EXTIOI_ISR_START		0x1700
#define EXTIOI_ISR_END			0x1720
#define EXTIOI_COREISR_START		0x1800
#define EXTIOI_COREISR_END		0x1B20
#define EXTIOI_COREMAP_START		0x1C00
#define EXTIOI_COREMAP_END		0x1D00

#define LS3A_INTC_IP			8
#define LS3A_NODES			4
#define MAX_CORES			4
#define EXTIOI_IRQS			(256)
#define EXTIOI_IRQS_BITMAP_SIZE		(256 / 8)
/* map to ipnum per 32 irqs */
#define EXTIOI_IRQS_IPMAP_SIZE		(256 / 32)
#define EXTIOI_IRQS_COREMAP_SIZE	(EXTIOI_IRQS)
#define EXTIOI_IRQS_NODETYPE_SIZE	16

typedef struct kvm_ls3a_extirq_state {
	union ext_en {
		uint64_t reg_u64[EXTIOI_IRQS_BITMAP_SIZE / 8];
		uint32_t reg_u32[EXTIOI_IRQS_BITMAP_SIZE / 4];
		uint8_t reg_u8[EXTIOI_IRQS_BITMAP_SIZE];
	} ext_en;
	union bounce {
		uint64_t reg_u64[EXTIOI_IRQS_BITMAP_SIZE / 8];
		uint32_t reg_u32[EXTIOI_IRQS_BITMAP_SIZE / 4];
		uint8_t reg_u8[EXTIOI_IRQS_BITMAP_SIZE];
	} bounce;
	union ext_isr {
		uint64_t reg_u64[EXTIOI_IRQS_BITMAP_SIZE / 8];
		uint32_t reg_u32[EXTIOI_IRQS_BITMAP_SIZE / 4];
		uint8_t reg_u8[EXTIOI_IRQS_BITMAP_SIZE];
	} ext_isr;
	union ext_core_isr {
		uint64_t reg_u64[MAX_CORES][EXTIOI_IRQS_BITMAP_SIZE / 8];
		uint32_t reg_u32[MAX_CORES][EXTIOI_IRQS_BITMAP_SIZE / 4];
		uint8_t reg_u8[MAX_CORES][EXTIOI_IRQS_BITMAP_SIZE];
	} ext_core_isr;
	union ip_map {
		uint64_t reg_u64;
		uint32_t reg_u32[EXTIOI_IRQS_IPMAP_SIZE / 4];
		uint8_t reg_u8[EXTIOI_IRQS_IPMAP_SIZE];
	} ip_map;
	union core_map {
		uint64_t reg_u64[EXTIOI_IRQS_COREMAP_SIZE / 8];
		uint32_t reg_u32[EXTIOI_IRQS_COREMAP_SIZE / 4];
		uint8_t reg_u8[EXTIOI_IRQS_COREMAP_SIZE];
	} core_map;
	union node_type {
		uint64_t reg_u64[EXTIOI_IRQS_NODETYPE_SIZE / 4];
		uint32_t reg_u32[EXTIOI_IRQS_NODETYPE_SIZE / 2];
		uint16_t reg_u16[EXTIOI_IRQS_NODETYPE_SIZE];
		uint8_t reg_u8[EXTIOI_IRQS_NODETYPE_SIZE * 2];
	} node_type;

	/*software state */
	uint8_t ext_sw_ipmap[EXTIOI_IRQS];
	uint8_t ext_sw_coremap[EXTIOI_IRQS];
	uint8_t ext_sw_ipisr[MAX_CORES][LS3A_INTC_IP][EXTIOI_IRQS_BITMAP_SIZE];
} LS3AExtirqState;

typedef struct ext_irq_io_device {
	struct ls3a_kvm_extirq *extirq;
	struct kvm_io_device device;
	int nodeNum;
} ext_irq_io_device;

struct ls3a_kvm_extirq {
	spinlock_t lock;
	struct kvm *kvm;
	ext_irq_io_device dev_ls3a_ext_irq[4];
	struct kvm_ls3a_extirq_state ls3a_ext_irq;
};


static inline struct ls3a_kvm_extirq *ls3a_ext_irqchip(struct kvm *kvm)
{
	return kvm->arch.v_extirq;
}

static inline int ls3a_extirq_in_kernel(struct kvm *kvm)
{
	int ret;

	ret = (ls3a_ext_irqchip(kvm) != NULL);
	return ret;
}

void ext_irq_handler(struct kvm *kvm,int irq,int level);
struct ls3a_kvm_extirq *kvm_create_ls3a_ext_irq(struct kvm *kvm);
int kvm_get_ls3a_extirq(struct kvm *kvm,
			struct kvm_loongson_ls3a_extirq_state *state);
int kvm_set_ls3a_extirq(struct kvm *kvm,
			struct kvm_loongson_ls3a_extirq_state *state);
int kvm_get_ls3a_ipmask(struct kvm *kvm, uint16_t *state);
int kvm_set_ls3a_ipmask(struct kvm *kvm, uint16_t *state);
void kvm_destroy_ls3a_ext_irq(struct ls3a_kvm_extirq *v_extirq);
void msi_irq_handler(struct kvm *kvm, int irq, int level);
#endif
