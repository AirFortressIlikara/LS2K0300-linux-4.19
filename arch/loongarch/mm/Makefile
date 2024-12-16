# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Linux/LoongArch-specific parts of the memory manager.
#

obj-y				+= init.o cache.o tlbex.o extable.o fault.o \
				   gup.o mmap.o pgtable.o tlb.o ioremap.o

obj-$(CONFIG_64BIT)		+= pgtable-64.o
obj-$(CONFIG_HUGETLB_PAGE)	+= hugetlbpage.o
obj-$(CONFIG_KASAN)		+= kasan_init.o
