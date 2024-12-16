/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#ifndef _ASM_PCI_H
#define _ASM_PCI_H

#include <linux/mm.h>

#ifdef __KERNEL__

/*
 * This file essentially defines the interface between board specific
 * PCI code and LoongArch common PCI code. Should potentially put into
 * include/asm/pci.h file.
 */

#include <linux/ioport.h>
#include <linux/list.h>

extern phys_addr_t mcfg_addr_init(int node);

/* Can be used to override the logic in pci_scan_bus for skipping
   already-configured bus numbers - to be used for buggy BIOSes
   or architectures with incomplete PCI setup by the loader */
static inline unsigned int pcibios_assign_all_busses(void)
{
	return 0;
}

#define PCIBIOS_MIN_IO		0x4000
#define PCIBIOS_MIN_MEM		0x20000000

#define PCIBIOS_MIN_CARDBUS_IO	0x4000

#define HAVE_PCI_MMAP
#define ARCH_GENERIC_PCI_MMAP_RESOURCE
#define HAVE_ARCH_PCI_RESOURCE_TO_USER

/*
 * Dynamic DMA mapping stuff.
 * LoongArch has everything mapped statically.
 */

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/string.h>
#include <asm/io.h>

static inline int pci_proc_domain(struct pci_bus *bus)
{
	return 1; /* always show the domain in /proc */
}

#endif /* __KERNEL__ */

/* generic pci stuff */
#include <asm-generic/pci.h>

#endif /* _ASM_PCI_H */
