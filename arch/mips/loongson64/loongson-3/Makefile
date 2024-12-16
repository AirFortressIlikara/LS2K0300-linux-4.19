#
# Makefile for Loongson-3 family machines
#
obj-y			+= irq.o cop2-ex.o platform.o acpi_init.o dma.o clock.o ec_wpce775l.o workarounds.o \
					ls2h-irq.o ls2h-platform.o pch-irq.o ls7a-platform.o rs780-irq.o rs780-platform.o \
					stable_counter.o ec_npce985x.o ec_it8528.o

toolchain-loongson-ext2	:= $(call cc-option-yn,$(mips-cflags) -mloongson-ext2)
obj-$(toolchain-loongson-ext2)	+= loongson3-memcpy.o loongson3-memset.o

obj-$(CONFIG_SMP)	+= smp.o

obj-$(CONFIG_NUMA)	+= numa.o

obj-$(CONFIG_SUSPEND)	+= pm.o sleep.o

obj-$(CONFIG_GS464E_NODE_COUNTER) += node_counter.o

obj-$(CONFIG_LOONGSON_HPET) += hpet.o

obj-$(CONFIG_PCI_MSI)  += pci_msi.o

obj-$(CONFIG_LOONGSON3_CPUAUTOPLUG) += loongson3_cpuautoplug.o

obj-$(CONFIG_CPU_HAS_LASX)	+= lasx.o
