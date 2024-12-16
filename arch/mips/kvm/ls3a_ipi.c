/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * currently used for Loongson3A Virtual IPI interrupt.
 * Mapped into the guest kernel @ KVM_GUEST_COMMPAGE_ADDR.
 *
 * Copyright (C) 2019  Loongson Technologies, Inc.  All rights reserved.
 * Authors: Chen Zhu <zhuchen@loongson.cn>
 */

#include "ls3a_ipi.h"
#include "ls3a.h"

extern int kvm_vcpu_ioctl_interrupt(struct kvm_vcpu *vcpu,
			     struct kvm_mips_interrupt *irq);

#define ls3a_gipi_lock(s, flags)	spin_lock_irqsave(&s->lock, flags)
#define ls3a_gipi_unlock(s, flags)	spin_unlock_irqrestore(&s->lock, flags)

static int ls3a_gipi_writel(struct ls3a_kvm_ipi * ipi, gpa_t addr,
						int len, const void *val)
{
	uint64_t data,offset;
	struct kvm_mips_interrupt irq;
	gipiState * s = &(ipi->ls3a_gipistate);
	uint32_t coreno = (addr >> 8) & 3;
	uint32_t node, no;
	struct kvm *kvm;
	void *pbuf;

	kvm = ipi->kvm;
	node = (addr >> kvm->arch.node_shift) & 3;
	no = coreno + node * 4;

	data = *(uint64_t *)val;
	offset = addr & 0xFF;

	BUG_ON(offset & (len - 1));

	switch (offset) {
	case CORE0_STATUS_OFF:
		printk("CORE0_SET_OFF Can't be write\n");

		break;
	case CORE0_EN_OFF:
		s->core[no].en = data;

		break;
	case CORE0_SET_OFF:
		s->core[no].status |= data;
		irq.cpu = no;
		irq.irq = 6;

		kvm_vcpu_ioctl_interrupt(kvm->vcpus[no],&irq);
		kvm_vcpu_kick(kvm->vcpus[no]);
		break;
	case CORE0_CLEAR_OFF:
		s->core[no].status &= ~data;
		if (!s->core[no].status) {
			irq.cpu = no;
			irq.irq = -6;
			kvm_vcpu_ioctl_interrupt(kvm->vcpus[no],&irq);
		}

		break;
	case 0x20 ... 0x3c:
		pbuf =  (void*)s->core[no].buf + (offset - 0x20);
		if (len == 1) {
			*(unsigned char*)pbuf = (unsigned char)data;
		} else if (len == 2) {
			*(unsigned short*)pbuf = (unsigned short)data;
		} else if (len == 4) {
			*(unsigned int*)pbuf = (unsigned int)data;
		} else if (len == 8) {
			*(unsigned long*)pbuf = (unsigned long)data;
		}

		break;
	default:
		printk("ls3a_gipi_writel with unknown addr %llx \n", addr);
		break;
	}
	return 0;
}

static uint64_t ls3a_gipi_readl(struct ls3a_kvm_ipi * ipi,
				gpa_t addr, int len, void* val)
{
	uint64_t offset;
	uint64_t ret = 0;

	gipiState * s = &(ipi->ls3a_gipistate);
	uint32_t node, no;
	uint32_t coreno;
	void *pbuf;

	coreno = (addr >> 8 ) & 3;
	node = (addr >> ipi->kvm->arch.node_shift) & 3;
	no = coreno + node *4;

	offset = addr & 0xFF;

	BUG_ON(offset & (len - 1));
	switch (offset) {
	case CORE0_STATUS_OFF:
		ret = s->core[no].status;
		break;
	case CORE0_EN_OFF:
		ret = s->core[no].en;
		break;
	case CORE0_SET_OFF:
		ret = 0;
		break;
	case CORE0_CLEAR_OFF:
		ret = 0;
		break;
	case 0x20 ... 0x3c:
		pbuf =  (void*)s->core[no].buf + (offset - 0x20);
		if (len == 1) {
			ret  = *(unsigned char*)pbuf;
		} else if (len == 2) {
			ret = *(unsigned short*)pbuf;
		} else if (len == 4) {
			ret = *(unsigned int*)pbuf;
		} else if (len == 8) {
			ret = *(unsigned long*)pbuf;
		}
		break;
	default:
		printk("ls3a_gipi_readl with unknown addr %llx \n", addr);
		break;
	}

	*(uint64_t *)val = ret;

	return ret;
}

static int kvm_ls3a_ipi_write(struct kvm_vcpu *vcpu,
			      struct kvm_io_device * dev,
			      gpa_t addr, int len, const void *val)
{
	struct ls3a_kvm_ipi *ipi;
	ipi_io_device *ipi_device;
	unsigned long flags;

	ipi_device = container_of(dev, ipi_io_device, device);
	ipi = ipi_device->ipi;
	ipi->kvm->stat.lsvz_kvm_pip_write_exits++;

	ls3a_gipi_lock(ipi, flags);
	ls3a_gipi_writel(ipi, addr, len, val);
	ls3a_gipi_unlock(ipi, flags);
	return 0;
}


static int kvm_ls3a_ipi_read(struct kvm_vcpu *vcpu,
			     struct kvm_io_device *dev,
			     gpa_t addr, int len, void *val)
{
	struct ls3a_kvm_ipi *ipi;
	ipi_io_device *ipi_device;
	unsigned long flags;

	ipi_device = container_of(dev, ipi_io_device, device);
	ipi = ipi_device->ipi;
	ipi->kvm->stat.lsvz_kvm_pip_read_exits++;

	ls3a_gipi_lock(ipi, flags);
	ls3a_gipi_readl(ipi, addr, len, val);
	ls3a_gipi_unlock(ipi, flags);
	return 0;
}


static const struct kvm_io_device_ops kvm_ls3a_ipi_ops = {
	.read     = kvm_ls3a_ipi_read,
	.write    = kvm_ls3a_ipi_write,
};

void kvm_destroy_ls3a_ipi(struct ls3a_kvm_ipi *vipi)
{
	int i;
	struct kvm_io_device *device;

	for (i=0; i<vipi->nodeNum; i++) {
		device = &vipi->dev_ls3a_ipi[i].device;
		kvm_io_bus_unregister_dev(vipi->kvm, KVM_MMIO_BUS, device);
	}
	kfree(vipi);
}

struct ls3a_kvm_ipi * kvm_create_ls3a_ipi(struct kvm *kvm)
{
	struct ls3a_kvm_ipi *s;
	unsigned long addr;
	struct kvm_io_device *device;
	int ret, i;

	s = kzalloc(sizeof(struct ls3a_kvm_ipi), GFP_KERNEL);
	if (!s)
		return NULL;
	spin_lock_init(&s->lock);
	s->kvm = kvm;
	s->nodeNum = 0;

	/*
	 * Initialize MMIO device
	 */
	for (i=0; i<4; i++) {
		device = &s->dev_ls3a_ipi[i].device;
		kvm_iodevice_init(device, &kvm_ls3a_ipi_ops);
		addr = (((unsigned long)i) << kvm->arch.node_shift) +
								SMP_MAILBOX;
		mutex_lock(&kvm->slots_lock);
		ret = kvm_io_bus_register_dev(kvm, KVM_MMIO_BUS,
						addr, 0x400, device);
		mutex_unlock(&kvm->slots_lock);
		if (ret < 0)
			break;
		s->nodeNum++;
		s->dev_ls3a_ipi[i].ipi = s;
		s->dev_ls3a_ipi[i].nodeNum = i;
	}

	if (ret == 0)
		return s;

	kvm_destroy_ls3a_ipi(s);
	return NULL;
}

int kvm_get_ls3a_ipi(struct kvm *kvm, struct loongson_gipiState *state)
{
	struct ls3a_kvm_ipi *ipi = ls3a_ipi_irqchip(kvm);
	gipiState *ipi_state =  &(ipi->ls3a_gipistate);
	unsigned long flags;

	ls3a_gipi_lock(ipi, flags);
	memcpy(state, ipi_state, sizeof(gipiState));
	ls3a_gipi_unlock(ipi, flags);
	return 0;
}

int kvm_set_ls3a_ipi(struct kvm *kvm, struct loongson_gipiState *state)
{
	struct ls3a_kvm_ipi *ipi = ls3a_ipi_irqchip(kvm);
	gipiState *ipi_state =  &(ipi->ls3a_gipistate);
	unsigned long flags;

	if (!ipi)
		return -EINVAL;

	ls3a_gipi_lock(ipi, flags);
	memcpy(ipi_state, state, sizeof(gipiState));
	ls3a_gipi_unlock(ipi, flags);
	return 0;
}
