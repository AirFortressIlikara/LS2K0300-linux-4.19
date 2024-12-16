/*
 *  Copyright (C) 2020, Loongson Technology Corporation Limited, Inc.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
*/

#ifndef __ASM_MACH_LOONGSON_2K_H_
#define __ASM_MACH_LOONGSON_2K_H_

#include <linux/io.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <boot_param.h>

/* we need to read back to ensure the write is accepted by conf bus */

#define LS2K_IO_REG_BASE		0x1f000000

/* CHIP CONFIG regs */
#define LS2K_CHIP_CFG_REG_BASE		(LS2K_IO_REG_BASE + 0x00e10000)
#define LS2K_GEN_CONFIG0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x420)

#define CONF_BASE 			0x1fe00000

#define PLL_SYS0_OFF 			0x480
#define PLL_SYS1_OFF 			0x488

/* HPET regs */
#define HPET_CFG			0x010
#define HPET_STATUS			0x020
#define HPET_COUNTER			0x0f0
#define HPET_T0_IRS			0x001
#define HPET_T0_CFG			0x100
#define HPET_T0_CMP			0x108
#define HPET_CFG_ENABLE			0x001
#define HPET_TN_LEVEL			0x0002
#define HPET_TN_ENABLE			0x0004
#define HPET_TN_PERIODIC		0x0008
#define HPET_TN_SETVAL			0x0040
#define HPET_TN_32BIT			0x0100

/* REG ACCESS*/
#define ls2k_readb(addr)		(*(volatile unsigned char *)TO_UNCAC(addr))
#define ls2k_readw(addr)		(*(volatile unsigned short *)TO_UNCAC(addr))
#define ls2k_readl(addr)		(*(volatile unsigned int *)TO_UNCAC(addr))
#define ls2k_readq(addr)		(*(volatile unsigned long *)TO_UNCAC(addr))
#define ls2k_writeb(val, addr)		(*(volatile unsigned char *)TO_UNCAC(addr) = (val))
#define ls2k_writew(val, addr)		(*(volatile unsigned short *)TO_UNCAC(addr) = (val))
#define ls2k_writel(val, addr)		(*(volatile unsigned int *)TO_UNCAC(addr) = (val))
#define ls2k_writeq(val, addr)		(*(volatile unsigned long *)TO_UNCAC(addr) = (val))

#define DVFS_CFG			0x410
#define DVFS_STS			0x414
#define DVFS_CNT			0x418

#define MIN_FREQ_LEVEL 			1
#define DVFS_STS_STATUS_MASK		(0x01)

#define DVFS_CNT_START			(0x01)
#define DVFS_CNT_UPDATE_EN		(0x01 << 22)
#define DVFS_CNT_DIV(div)		(div << 16)
#define DVFS_CNT_POL			(0x01 << 2)
#define DVFS_CNT_VID_EN			(0x01 << 1)
#define DVFS_CNT_POL_UP			(0x01 << 2)
#define DVFS_CNT_POL_DOWN		(~(0x01 << 2))
#define DVFS_CNT_VID(vid)		(vid << 4)
#define DVFS_CNT_VID_UPDATE_EN		(0x1 << 1)
#define DVFS_CNT_VID_UPDATE_OFF		(~(0x1 << 1))
#define DVFS_CNT_VID_TGT_MASK   	GENMASK(9, 4)
#define DVFS_CNT_FEQ_MASK   		GENMASK(21, 16)

#define DVFS_CFG_DVFS_EN		(0x01)
#define DVFS_CFG_BACKUP_CLOCK		(0x01 << 4)
#define DVFS_CFG_PROTECT_CLOCK   	(0x02 << 6)
#define DVFS_CFG_PROTECT_FULL   	(0x03 << 6)
#define DVFS_CFG_CTRL0_MASK   		GENMASK(5, 4)
#define DVFS_CFG_CTRL1_MASK   		GENMASK(7, 6)

#define DVFS_STS_FEQ_STS_OFFSET 	16
#define DVFS_STS_FEQ_STS_MASK   	GENMASK(21, 16)

#define APB_BASE			0x1fe20000
#define ACPI_OFF			0x7000

/* ACPI ACCESS */
#define ACPI_BASE 			(APB_BASE + ACPI_OFF)
#define acpi_readb(offset)		ls2k_readb((ACPI_BASE + offset))
#define acpi_readw(offset) 		ls2k_readw((ACPI_BASE + offset))
#define acpi_readl(offset) 		ls2k_readl((ACPI_BASE + offset))
#define acpi_readq(offset) 		ls2k_readq((ACPI_BASE + offset))
#define acpi_writeb(val, offset)	ls2k_writeb((val), (ACPI_BASE + offset))
#define acpi_writew(val, offset) 	ls2k_writew((val), (ACPI_BASE + offset))
#define acpi_writel(val, offset) 	ls2k_writel((val), (ACPI_BASE + offset))
#define acpi_writeq(val, offset) 	ls2k_writeq((val), (ACPI_BASE + offset))

#endif /* __ASM_MACH_LOONGSON_2K_H_ */
