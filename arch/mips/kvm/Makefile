# SPDX-License-Identifier: GPL-2.0
# Makefile for KVM support for MIPS
#

common-objs-y = $(addprefix ../../../virt/kvm/, kvm_main.o coalesced_mmio.o irqchip.o eventfd.o)

KVM := ../../../virt/kvm
common-objs-$(CONFIG_KVM_VFIO) += $(KVM)/vfio.o

EXTRA_CFLAGS += -Ivirt/kvm -Iarch/mips/kvm

kvm-objs := $(common-objs-y) mips.o emulate.o entry.o \
	    interrupt.o stats.o commpage.o
kvm-objs += hypcall.o
kvm-objs += mmu.o

ifdef CONFIG_KVM_MIPS_VZ
kvm-objs		+= vz.o ls3acomp-vz.o ls7a_irq.o \
				ls3a_ipi.o \
				stable_timer.o ls3a_ext_irq.o ls7a_irqfd.o
else
kvm-objs		+= dyntrans.o
kvm-objs		+= trap_emul.o
endif
obj-$(CONFIG_KVM)	+= kvm.o
obj-y			+= callback.o tlb.o fpu.o msa.o lasx.o
