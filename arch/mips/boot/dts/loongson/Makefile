# SPDX-License-Identifier: GPL-2.0
dtb-$(CONFIG_CPU_LOONGSON3)	+= loongson3_ls2h.dtb loongson3_ls7a.dtb loongson3_rs780.dtb
dtb-$(CONFIG_CPU_LOONGSON2K)	+= loongson2_2k1000.dtb

obj-y				+= $(patsubst %.dtb, %.dtb.o, $(dtb-y))
