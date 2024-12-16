/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * commpage, currently used for Virtual Loongson7A IOAPIC.
 *
 * Copyright (C) 2019 Loongson Technologies, Inc.  All rights reserved.
 * Authors: Chen Zhu <zhuchen@loongson.cn>
 */

#include <linux/highmem.h>
#include <linux/mm.h>
#include "ls7a_irq.h"
#include "ls3a.h"

void ls7a_ioapic_lock(struct ls7a_kvm_ioapic *s, unsigned long *flags)
{
	unsigned long tmp;
	spin_lock_irqsave(&s->lock, tmp);
	*flags = tmp;
}

void ls7a_ioapic_unlock(struct ls7a_kvm_ioapic *s, unsigned long *flags)
{
	unsigned long tmp;
	tmp = *flags;
	spin_unlock_irqrestore(&s->lock, tmp);
}

static void kvm_ls7a_ioapic_raise(struct kvm *kvm, unsigned long mask)
{
	unsigned long irqnum, val;
	struct ls7a_kvm_ioapic *s = ls7a_ioapic_irqchip(kvm);
	struct kvm_ls7a_ioapic_state *state;
	struct kvm_mips_interrupt irq;
	int i;

	state = &s->ls7a_ioapic;
	irq.cpu = -1;
	val = mask & state->intirr & (~state->int_mask);
	for_each_set_bit(i, &val, 64) {
		if ((state->intisr & (0x1ULL << i)) == 0) {
			state->intisr |= 0x1ULL << i;
			irqnum = state->htmsi_vector[i];
			kvm_debug("msi_irq_handler,%ld,up\n", irqnum);
			msi_irq_handler(kvm, irqnum, 1);
		}
	}
	kvm->stat.lsvz_kvm_ls7a_ioapic_update++;
}

static void kvm_ls7a_ioapic_lower(struct kvm *kvm, unsigned long mask)
{
	unsigned long irqnum, val;
	struct ls7a_kvm_ioapic *s = ls7a_ioapic_irqchip(kvm);
	struct kvm_ls7a_ioapic_state *state;
	struct kvm_mips_interrupt irq;
	int i;

	state = &s->ls7a_ioapic;
	irq.cpu = -1;
	val = mask & state->intisr;
	for_each_set_bit(i, &val, 64) {
		if (state->intisr & (0x1ULL << i)) {
			state->intisr &= ~(0x1ULL << i);
			irqnum = state->htmsi_vector[i];
			kvm_debug("msi_irq_handler,%ld,down\n", irqnum);
			msi_irq_handler(kvm, irqnum, 0);
		}
	}
	kvm->stat.lsvz_kvm_ls7a_ioapic_update++;
}

int kvm_ls7a_set_msi(struct kvm_kernel_irq_routing_entry *e,
		struct kvm *kvm, int irq_source_id, int level, bool line_status)
{
	unsigned long flags;
	if (!level)
		return -1;

	kvm_debug("msi data is 0x%x",e->msi.data);
	ls7a_ioapic_lock(ls7a_ioapic_irqchip(kvm), &flags);
	msi_irq_handler(kvm, e->msi.data, 1);
	ls7a_ioapic_unlock(ls7a_ioapic_irqchip(kvm), &flags);

	return 0;
}

int kvm_ls7a_send_userspace_msi(struct kvm *kvm, struct kvm_msi *msi)
{
	struct kvm_kernel_irq_routing_entry route;

	if (msi->flags != 0)
		return -EINVAL;

	kvm->stat.lsvz_kvm_ls7a_msi_irq++;
	route.msi.address_lo = msi->address_lo;
	route.msi.address_hi = msi->address_hi;
	route.msi.data = msi->data;

	kvm_debug("msi data is 0x%x",route.msi.data);
	return kvm_ls7a_set_msi(&route, kvm, KVM_USERSPACE_IRQ_SOURCE_ID, 1, false);

}

int kvm_ls7a_ioapic_set_irq(struct kvm *kvm, int irq, int level)
{
	struct ls7a_kvm_ioapic *s;
	struct kvm_ls7a_ioapic_state *state;
	uint64_t mask = 1ULL << irq;
	s = ls7a_ioapic_irqchip(kvm);
	state = &s->ls7a_ioapic;
	BUG_ON(irq < 0 || irq >= LS7A_IOAPIC_NUM_PINS);

	if (state->intedge & mask) {
		/* edge triggered */
		if (level) {
			if ((state->last_intirr & mask) == 0) {
				state->intirr |= mask;
				kvm_ls7a_ioapic_raise(kvm, mask);
			}
			state->last_intirr |= mask;
		} else
			state->last_intirr &= ~mask;
	} else {
		/* level triggered */
		if (!!level) {
			if ((state->intirr & mask) == 0) {
				state->intirr |= mask;
				kvm_ls7a_ioapic_raise(kvm, mask);
			}
		} else {
			if (state->intirr & mask) {
				state->intirr &= ~mask;
				kvm_ls7a_ioapic_lower(kvm, mask);
			}
		}
	}
	kvm->stat.lsvz_kvm_ls7a_ioapic_set_irq++;
	return 0;
}

static int ls7a_ioapic_reg_write(struct ls7a_kvm_ioapic *s,
			  gpa_t addr, int len, const void *val,
			  unsigned long *flags)
{
	struct kvm *kvm;
	struct kvm_ls7a_ioapic_state *state;

        int64_t offset_tmp;
        uint64_t offset;
	uint64_t data, old;
	unsigned long bit;

        offset = addr&0xfff;
	kvm = s->kvm;
	state = &(s->ls7a_ioapic);

	if (offset & (len -1 )) {
		printk("%s(%d):unaligned address access %llx size %d \n",
			__FUNCTION__, __LINE__, addr, len);
		return 0;
	}

        if (8 == len) {
		data = *(uint64_t *)val;
		switch (offset) {
		case LS7A_INT_MASK_OFFSET:
			old = state->int_mask;
			state->int_mask = data;
			if (old & ~data)
				kvm_ls7a_ioapic_raise(kvm, old & ~data);
			else if (~old & data)
				kvm_ls7a_ioapic_lower(kvm, ~old & data);
			break;
		case LS7A_INT_STATUS_OFFSET:
			state->intisr = data;
			break;
		case LS7A_INT_EDGE_OFFSET:
			state->intedge = data;
			break;
		case LS7A_INT_CLEAR_OFFSET:
			state->intirr &= (~(data & state->intedge));
			kvm_ls7a_ioapic_lower(kvm, data);
			for_each_set_bit(offset_tmp, val, 64) {
				ls7a_ioapic_unlock(
					ls7a_ioapic_irqchip(kvm), flags);
				kvm_notify_acked_irq(kvm, 0, offset_tmp);
				ls7a_ioapic_lock(
					ls7a_ioapic_irqchip(kvm), flags);
			}
			break;
		case LS7A_INT_POL_OFFSET:
			state->int_polarity = data;
			break;
		case LS7A_HTMSI_EN_OFFSET:
			old = state->htmsi_en;
			state->htmsi_en = data;
			bit = (~old) & data & state->intirr;
			for_each_set_bit(offset_tmp, &bit, 64) {
				kvm_ls7a_ioapic_raise(kvm, 1ULL << offset_tmp);
			}
			break;
		case LS7A_AUTO_CTRL0_OFFSET:
		case LS7A_AUTO_CTRL1_OFFSET:
			break;
		default:
			WARN_ONCE(1,"Abnormal address access:addr 0x%llx,len %d\n",addr,len);
			break;
		}
	} else if (1 == len) {
		data = *(unsigned char *)val;
                if (offset >= LS7A_HTMSI_VEC_OFFSET) {
                        offset_tmp = offset - LS7A_HTMSI_VEC_OFFSET;
                        if (offset_tmp >= 0 && offset_tmp < 64) {
				state->htmsi_vector[offset_tmp] =
							(uint8_t)(data & 0xff);
				if (state->intirr & state->htmsi_en
						& (1ULL << offset_tmp)) {
					kvm_ls7a_ioapic_raise(kvm,
							1ULL << offset_tmp);
				}
			}
                } else if (offset >=  LS7A_ROUTE_ENTRY_OFFSET) {
                        offset_tmp = offset - LS7A_ROUTE_ENTRY_OFFSET;
                        if (offset_tmp >= 0 && offset_tmp < 64) {
				state->route_entry[offset_tmp] =
							(uint8_t)(data & 0xff);
			}
		} else {
			WARN_ONCE(1, "Abnormal access:addr 0x%llx,len %d\n",
								addr, len);
		}
	} else {
		WARN_ONCE(1, "Abnormal address access:addr 0x%llx,len %d\n",
								addr, len);
	}
	kvm->stat.lsvz_ls7a_ioapic_reg_write++;
	return 0;
}

static inline struct ls7a_kvm_ioapic *to_ioapic(struct kvm_io_device *dev)
{
	return container_of(dev, struct ls7a_kvm_ioapic, dev_ls7a_ioapic);
}

static int kvm_ls7a_ioapic_write(struct kvm_vcpu *vcpu,
				 struct kvm_io_device * this,
				 gpa_t addr, int len, const void *val)
{
	struct ls7a_kvm_ioapic *s = to_ioapic(this);
	unsigned long flags;

	ls7a_ioapic_lock(s->kvm->arch.v_ioapic, &flags);
	ls7a_ioapic_reg_write(s, addr, len, val, &flags);
	ls7a_ioapic_unlock(s->kvm->arch.v_ioapic, &flags);

	return 0;
}

static int ls7a_ioapic_reg_read(struct ls7a_kvm_ioapic *s,
		       gpa_t addr, int len, void *val)
{
        uint64_t offset,offset_tmp;
	struct kvm *kvm;
	struct kvm_ls7a_ioapic_state *state;
	uint64_t result=0;

	state = &(s->ls7a_ioapic);
	kvm = s->kvm;
        offset = addr&0xfff;

	if (offset & (len -1 )) {
		printk("%s(%d):unaligned address access %llx size %d \n",
			__FUNCTION__, __LINE__, addr, len);
		return 0;
	}

	if (8 == len) {
		switch (offset) {
		case LS7A_INT_MASK_OFFSET:
			result = state->int_mask;
			break;
		case LS7A_INT_STATUS_OFFSET:
			result = state->intisr & (~state->int_mask);
			break;
		case LS7A_INT_EDGE_OFFSET:
			result = state->intedge;
			break;
		case LS7A_INT_POL_OFFSET:
			result = state->int_polarity;
			break;
		case LS7A_HTMSI_EN_OFFSET:
			result = state->htmsi_en;
			break;
		case LS7A_AUTO_CTRL0_OFFSET:
		case LS7A_AUTO_CTRL1_OFFSET:
			break;
		case LS7A_INT_ID_OFFSET:
			result = LS7A_INT_ID_VER;
			result = (result << 32) + LS7A_INT_ID_VAL;
			break;
		default:
			WARN_ONCE(1, "Abnormal access:addr 0x%llx,len %d\n",
								addr, len);
			break;
		}
		if(val != NULL)
			*(uint64_t *)val = result;
	} else if(1 == len) {
		if (offset >= LS7A_HTMSI_VEC_OFFSET) {
                        offset_tmp = offset - LS7A_HTMSI_VEC_OFFSET;
                        if (offset_tmp >= 0 && offset_tmp < 64) {
				result = state->htmsi_vector[offset_tmp];
                        }
                }else if(offset >=  LS7A_ROUTE_ENTRY_OFFSET){
			offset_tmp = offset - LS7A_ROUTE_ENTRY_OFFSET;
			if (offset_tmp >= 0 && offset_tmp < 64) {
				result = state->route_entry[offset_tmp];
			}
		} else {
			WARN_ONCE(1, "Abnormal access:addr 0x%llx,len %d\n",
								addr, len);
		}
		if(val != NULL)
			*(unsigned char *)val = result;
	} else {
		WARN_ONCE(1, "Abnormal access:addr 0x%llx,len %d\n",
								addr, len);
	}
	kvm->stat.lsvz_ls7a_ioapic_reg_read++;
	return result;
}

static int kvm_ls7a_ioapic_read(struct kvm_vcpu *vcpu,
				struct kvm_io_device *this,
				gpa_t addr, int len, void *val)
{
	struct ls7a_kvm_ioapic *s = to_ioapic(this);
	uint64_t result=0;
	unsigned long flags;

	ls7a_ioapic_lock(s->kvm->arch.v_ioapic, &flags);
	result = ls7a_ioapic_reg_read(s,addr,len,val);
	ls7a_ioapic_unlock(s->kvm->arch.v_ioapic, &flags);
	return 0;
}

static const struct kvm_io_device_ops kvm_ls7a_ioapic_ops = {
	.read     = kvm_ls7a_ioapic_read,
	.write    = kvm_ls7a_ioapic_write,
};

static int kvm_ls7a_ioapic_alias_read(struct kvm_vcpu *vcpu,
				struct kvm_io_device *this,
				gpa_t addr, int len, void *val)
{
	struct ls7a_kvm_ioapic *s;
	unsigned long flags;

	s = container_of(this, struct ls7a_kvm_ioapic, ls7a_ioapic_alias);
	ls7a_ioapic_lock(s->kvm->arch.v_ioapic, &flags);
	ls7a_ioapic_reg_read(s, addr, len, val);
	ls7a_ioapic_unlock(s->kvm->arch.v_ioapic, &flags);
	return 0;
}

static int kvm_ls7a_ioapic_alias_write(struct kvm_vcpu *vcpu,
				struct kvm_io_device *this,
				gpa_t addr, int len, const void *val)
{
	struct ls7a_kvm_ioapic *s;
	unsigned long flags;

	s = container_of(this, struct ls7a_kvm_ioapic, ls7a_ioapic_alias);
	ls7a_ioapic_lock(s->kvm->arch.v_ioapic, &flags);
	ls7a_ioapic_reg_write(s, addr, len, val, &flags);
	ls7a_ioapic_unlock(s->kvm->arch.v_ioapic, &flags);

	return 0;
}

static const struct kvm_io_device_ops kvm_ls7a_ioapic_ops_alias = {
	.read     = kvm_ls7a_ioapic_alias_read,
	.write    = kvm_ls7a_ioapic_alias_write,
};

struct ls7a_kvm_ioapic *kvm_create_ls7a_ioapic(struct kvm *kvm)
{
	struct ls7a_kvm_ioapic *s;
	int ret;
	unsigned long ls7a_ioapic_reg_base;

	s = kzalloc(sizeof(struct ls7a_kvm_ioapic), GFP_KERNEL);
	if (!s)
		return NULL;
	spin_lock_init(&s->lock);
	s->kvm = kvm;

	/*
	 * Initialize MMIO device
	 */
	ls7a_ioapic_reg_base = LS7A_IOAPIC_GUEST_REG_BASE;
	kvm_iodevice_init(&s->dev_ls7a_ioapic, &kvm_ls7a_ioapic_ops);
	mutex_lock(&kvm->slots_lock);
	ret = kvm_io_bus_register_dev(kvm, KVM_MMIO_BUS, ls7a_ioapic_reg_base,
					0x1000, &s->dev_ls7a_ioapic);
	if (ret < 0)
		goto fail_unlock;

	ls7a_ioapic_reg_base = LS7A_IOAPIC_GUEST_REG_BASE_ALIAS;
	kvm_iodevice_init(&s->ls7a_ioapic_alias, &kvm_ls7a_ioapic_ops_alias);
	ret = kvm_io_bus_register_dev(kvm, KVM_MMIO_BUS, ls7a_ioapic_reg_base,
					0x1000, &s->ls7a_ioapic_alias);
	if (ret < 0) {
		kvm_io_bus_unregister_dev(kvm, KVM_MMIO_BUS,
				&s->dev_ls7a_ioapic);
		goto fail_unlock;
	}

	mutex_unlock(&kvm->slots_lock);
	return s;

fail_unlock:
	mutex_unlock(&kvm->slots_lock);
	kfree(s);

	return NULL;
}

int kvm_get_ls7a_ioapic(struct kvm *kvm,
			struct ls7a_ioapic_state *state)
{
	struct ls7a_kvm_ioapic *ls7a_ioapic = ls7a_ioapic_irqchip(kvm);
	struct kvm_ls7a_ioapic_state *ioapic_state =
					    &(ls7a_ioapic->ls7a_ioapic);
	unsigned long flags;

	ls7a_ioapic_lock(ls7a_ioapic, &flags);
	memcpy(state, ioapic_state, sizeof(struct kvm_ls7a_ioapic_state));
	ls7a_ioapic_unlock(ls7a_ioapic, &flags);
	kvm->stat.lsvz_kvm_get_ls7a_ioapic++;
	return 0;
}

int kvm_set_ls7a_ioapic(struct kvm *kvm,
			struct ls7a_ioapic_state *state)
{
	struct ls7a_kvm_ioapic *ls7a_ioapic = ls7a_ioapic_irqchip(kvm);
	struct kvm_ls7a_ioapic_state *ioapic_state =
					    &(ls7a_ioapic->ls7a_ioapic);
	unsigned long flags;
	if (!ls7a_ioapic)
		return -EINVAL;

	ls7a_ioapic_lock(ls7a_ioapic, &flags);
	memcpy(ioapic_state, state, sizeof(struct kvm_ls7a_ioapic_state));
	ls7a_ioapic_unlock(ls7a_ioapic, &flags);
	kvm->stat.lsvz_kvm_set_ls7a_ioapic++;
	return 0;
}

void kvm_destroy_ls7a_ioapic(struct ls7a_kvm_ioapic *vpic)
{
	kvm_io_bus_unregister_dev(vpic->kvm, KVM_MMIO_BUS,
					&vpic->dev_ls7a_ioapic);
	kvm_io_bus_unregister_dev(vpic->kvm, KVM_MMIO_BUS,
					&vpic->ls7a_ioapic_alias);
	kfree(vpic);
}
