obj-y += init.o cmdline.o machtype.o setup.o irq.o early_printk.o time.o reset.o  cop2-ex.o

obj-$(CONFIG_SMP) += smp.o

obj-$(CONFIG_SWIOTLB) += swiotlb.o

obj-$(CONFIG_CPU_FREQ) += platform.o

obj-$(CONFIG_LOONGSON_HPET) += hpet.o
obj-$(CONFIG_PM) += pm.o sleep.o
toolchain-loongson-ext2 := $(call cc-option-yn,$(mips-cflags) -mloongson-ext2)
obj-$(toolchain-loongson-ext2)  += loongson3-memcpy.o loongson3-memset.o
