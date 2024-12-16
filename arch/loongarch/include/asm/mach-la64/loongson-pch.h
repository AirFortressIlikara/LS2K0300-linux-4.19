/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  Copyright (C) 2014-2017, Lemote, Inc.
 *  Copyright (C) 2018, Loongson Technology Corporation Limited, Inc.
 */
#ifndef _LOONGSON_PCH_H
#define _LOONGSON_PCH_H

#include <linux/msi.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <asm/addrspace.h>

/* ============== LS7A registers =============== */
#define LS7A_PCH_REG_BASE		0x10000000UL
/* CHIPCFG regs */
#define LS7A_CHIPCFG_REG_BASE		(LS7A_PCH_REG_BASE + 0x00010000)
/* MISC reg base */
#define LS7A_MISC_REG_BASE		(LS7A_PCH_REG_BASE + 0x00080000)
/* RTC regs */
#define LS7A_RTC_REG_BASE		(LS7A_MISC_REG_BASE + 0x00050100)

#define LS7A_DMA_CFG			((volatile void *)TO_UNCAC(LS7A_CHIPCFG_REG_BASE + 0x041c))
#define LS7A_DMA_NODE_SHF		8
#define LS7A_DMA_NODE_MASK		0x1F00

#define LS7A_LPC_INT_BASE		(LS7A_PCH_REG_BASE + 0x2000)
#define LS7A_LPC_INT_SIZE		0x1000
#define LS7A_LPC_CASCADE_IRQ		83

/*PCI Configration Space Base*/
#define MCFG_EXT_PCICFG_BASE		0xefe00000000UL

/*PCH OFFSET*/
#define HT1LO_OFFSET		0xe0000000000UL

/* REG ACCESS*/
#define ls7a_readb(addr)		(*(volatile unsigned char  *)TO_UNCAC(addr))
#define ls7a_readw(addr)		(*(volatile unsigned short *)TO_UNCAC(addr))
#define ls7a_readl(addr)		(*(volatile unsigned int   *)TO_UNCAC(addr))
#define ls7a_readq(addr)		(*(volatile unsigned long  *)TO_UNCAC(addr))
#define ls7a_writeb(val, addr)		(*(volatile unsigned char  *)TO_UNCAC(addr) = (val))
#define ls7a_writew(val, addr)		(*(volatile unsigned short *)TO_UNCAC(addr) = (val))
#define ls7a_writel(val, addr)		(*(volatile unsigned int *)TO_UNCAC(addr) = (val))
#define ls7a_writeq(val, addr)		(*(volatile unsigned long *)TO_UNCAC(addr) = (val))

#define ls7a_write(val, addr)		ls7a_write_type(val, addr, uint64_t)

extern unsigned long ls7a_rwflags;
extern rwlock_t ls7a_rwlock;
#define ls7a_read(val, addr)        					  \
    do {                                				  \
        read_lock_irqsave(&ls7a_rwlock,flags); 			          \
        val = *(volatile unsigned long __force *)TO_UNCAC(addr);          \
        read_unlock_irqrestore(&ls7a_rwlock,flags); 		          \
    }while(0)

#define ls7a_write_type(val, addr, type)          					  \
    do {                                				  \
        write_lock_irqsave(&ls7a_rwlock,ls7a_rwflags);          	  \
        *(volatile type __force *)TO_UNCAC(addr) = (val);        \
        write_unlock_irqrestore(&ls7a_rwlock,ls7a_rwflags);               \
    }while(0)

/* ============== Data structrues =============== */

/* gpio data */
struct platform_gpio_data {
	u32 gpio_conf;
	u32 gpio_out;
	u32 gpio_in;
	u32 in_start_bit;
	u32 support_irq;
	char *label;
	int gpio_base;
	int ngpio;
};

#endif
