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
#define CONF_BASE 0x1fe10000

#define CPU_WBASE0_OFF 0x0
#define CPU_WBASE1_OFF 0x8
#define CPU_WBASE2_OFF 0x10
#define CPU_WBASE3_OFF 0x18
#define CPU_WBASE4_OFF 0x20
#define CPU_WBASE5_OFF 0x28
#define CPU_WBASE6_OFF 0x30
#define CPU_WBASE7_OFF 0x38

#define CPU_WMASK0_OFF 0x40
#define CPU_WMASK1_OFF 0x48
#define CPU_WMASK2_OFF 0x50
#define CPU_WMASK3_OFF 0x58
#define CPU_WMASK4_OFF 0x60
#define CPU_WMASK5_OFF 0x68
#define CPU_WMASK6_OFF 0x70
#define CPU_WMASK7_OFF 0x78

#define CPU_WMMAP0_OFF 0x80
#define CPU_WMMAP1_OFF 0x88
#define CPU_WMMAP2_OFF 0x90
#define CPU_WMMAP3_OFF 0x98
#define CPU_WMMAP4_OFF 0xa0
#define CPU_WMMAP5_OFF 0xa8
#define CPU_WMMAP6_OFF 0xb0
#define CPU_WMMAP7_OFF 0xb8

#define SCACHE_LA0_OFF 0x200
#define SCACHE_LA1_OFF 0x208
#define SCACHE_LA2_OFF 0x210
#define SCACHE_LA3_OFF 0x218

#define SCACHE_LM0_OFF 0x240
#define SCACHE_LM1_OFF 0x248
#define SCACHE_LM2_OFF 0x250
#define SCACHE_LM3_OFF 0x258

#define SIGNAL_OFF 0x400

#define GMAC_OFF 0x420
#define UART_OFF 0x428
#define GPU_OFF 0x430
#define APBDMA_OFF 0x438
#define USB_PHY01_OFF 0x440
#define USB_PHY23_OFF 0x448
#define SATA_OFF 0x450
#define SATA_PHY_OFF 0x458
#define MON_WRDMA_OFF 0x460

#define PLL_SYS0_OFF 0x480
#define PLL_SYS1_OFF 0x488
#define PLL_DLL0_OFF 0x490
#define PLL_DLL1_OFF 0x498
#define PLL_DC0_OFF 0x4a0
#define PLL_DC1_OFF 0x4a8
#define PLL_PIX00_OFF 0x4b0
#define PLL_PIX01_OFF 0x4b8
#define PLL_PIX10_OFF 0x4c0
#define PLL_PIX11_OFF 0x4c8
#define PLL_FREQ_SC 0x4d0

#define GPIO0_OEN_OFF 0x500
#define GPIO1_OEN_OFF 0x508
#define GPIO0_O_OFF 0x510
#define GPIO1_O_OFF 0x518
#define GPIO0_I_OFF 0x520
#define GPIO1_I_OFF 0x528
#define GPIO0_INT_OFF 0x528
#define GPIO1_INT_OFF 0x530

#define CONF_DMA0_OFF 0xc00

#define CONF_DMA1_OFF 0xc10

#define CONF_DMA2_OFF 0xc20

#define CONF_DMA3_OFF 0xc30

#define CONF_DMA4_OFF 0xc40

#define C0_IPISR_OFF 0x1000
#define C0_IPIEN_OFF 0x1004
#define C0_IPISET_OFF 0x1008
#define C0_IPICLR_OFF 0x100c

#define C0_MAIL0_OFF 0x1020
#define C0_MAIL1_OFF 0x1028
#define C0_MAIL2_OFF 0x1030
#define C0_MAIL3_OFF 0x1038

#define INTSR0_OFF 0x1040
#define INTSR1_OFF 0x1048

#define C1_IPISR_OFF 0x1100
#define C1_IPIEN_OFF 0x1104
#define C1_IPISET_OFF 0x1108
#define C1_IPICLR_OFF 0x110c

#define C1_MAIL0_OFF 0x1120
#define C1_MAIL1_OFF 0x1128
#define C1_MAIL2_OFF 0x1130
#define C1_MAIL3_OFF 0x1138

#define INT_LO_OFF 0x1400
#define INT_HI_OFF 0x1440

#define INT_RTEBASE_OFF 0x0
#define INT_SR_OFF 0x20
#define INT_EN_OFF 0x24
#define INT_SET_OFF 0x28
#define INT_CLR_OFF 0x2c
#define INT_PLE_OFF 0x30
#define INT_EDG_OFF 0x34
#define INT_BCE_OFF 0x38
#define INT_AUTO_OFF 0x3c

#define THSENS_INT_CTL_HI_OFF 0x1500
#define THSENS_INT_CTL_LO_OFF 0x1508
#define THSENS_INT_CTL_SR_CLR_OFF 0x1510

#define THSENS_SCAL_OFF 0x1520

#define W4_BASE0_OFF 0x2400
#define W4_BASE1_OFF 0x2408
#define W4_BASE2_OFF 0x2410
#define W4_BASE3_OFF 0x2418
#define W4_BASE4_OFF 0x2420
#define W4_BASE5_OFF 0x2428
#define W4_BASE6_OFF 0x2430
#define W4_BASE7_OFF 0x2438

#define W4_MASK0_OFF 0x2440
#define W4_MASK1_OFF 0x2448
#define W4_MASK2_OFF 0x2450
#define W4_MASK3_OFF 0x2458
#define W4_MASK4_OFF 0x2460
#define W4_MASK5_OFF 0x2468
#define W4_MASK6_OFF 0x2470
#define W4_MASK7_OFF 0x2478

#define W4_MMAP0_OFF 0x2480
#define W4_MMAP1_OFF 0x2488
#define W4_MMAP2_OFF 0x2490
#define W4_MMAP3_OFF 0x2498
#define W4_MMAP4_OFF 0x24a0
#define W4_MMAP5_OFF 0x24a8
#define W4_MMAP6_OFF 0x24b0
#define W4_MMAP7_OFF 0x24b8

#define w5_BASE0_OFF 0x2500
#define w5_BASE1_OFF 0x2508
#define w5_BASE2_OFF 0x2510
#define w5_BASE3_OFF 0x2518
#define w5_BASE4_OFF 0x2520
#define w5_BASE5_OFF 0x2528
#define w5_BASE6_OFF 0x2530
#define w5_BASE7_OFF 0x2538

#define w5_MASK0_OFF 0x2540
#define w5_MASK1_OFF 0x2548
#define w5_MASK2_OFF 0x2550
#define w5_MASK3_OFF 0x2558
#define w5_MASK4_OFF 0x2560
#define w5_MASK5_OFF 0x2568
#define w5_MASK6_OFF 0x2570
#define w5_MASK7_OFF 0x2578

#define w5_MMAP0_OFF 0x2580
#define w5_MMAP1_OFF 0x2588
#define w5_MMAP2_OFF 0x2590
#define w5_MMAP3_OFF 0x2598
#define w5_MMAP4_OFF 0x25a0
#define w5_MMAP5_OFF 0x25a8
#define w5_MMAP6_OFF 0x25b0
#define w5_MMAP7_OFF 0x25b8

#define PCI_HEAD20_OFF 0x3000

#define PCI_HEAD30_OFF 0x3040

#define PCI_HEAD31_OFF 0x3080

#define PCI_HEAD40_OFF 0x30c0

#define PCI_HEAD41_OFF 0x3100

#define PCI_HEAD42_OFF 0x3140

#define PCI_HEAD50_OFF 0x3180

#define PCI_HEAD60_OFF 0x31c0

#define PCI_HEAD70_OFF 0x3200

#define PCI_HEAD80_OFF 0x3240

#define PCI_HEADF0_OFF 0x32c0

#define PCI_CFG20_OFF 0x3800
#define PCI_CFG30_OFF 0x3808
#define PCI_CFG31_OFF 0x3810
#define PCI_CFG40_OFF 0x3818
#define PCI_CFG41_OFF 0x3820
#define PCI_CFG42_OFF 0x3828
#define PCI_CFG50_OFF 0x3830
#define PCI_CFG60_OFF 0x3838
#define PCI_CFG70_OFF 0x3840
#define PCI_CFG80_OFF 0x3848
#define PCI_CFGF0_OFF 0x3850

#define CHIP_ID_OFF 0x3ff8
/* end of conf bus */

#define APB_BASE	0x1fe00000

#define UART0_OFF	0x0
#define UART1_OFF	0x100
#define UART2_OFF	0x200
#define UART3_OFF	0x300
#define UART4_OFF	0x400
#define UART5_OFF	0x500
#define UART6_OFF	0x600
#define UART7_OFF	0x700
#define UART8_OFF	0x800
#define UART9_OFF	0x900
#define UARTA_OFF	0xa00
#define UARTB_OFF	0xb00

#define I2C0_OFF	0x1000
#define I2C1_OFF	0x1800

#define PWM_OFF		0x2000

#define HPET_OFF	0x4000
#define AC97_OFF	0x5000
#define NAND_OFF	0x6000

#define ACPI_OFF	0x7000

#define RTC_OFF		0x7800

#define DES_OFF		0x8000
#define AES_OFF		0x9000
#define RSA_OFF		0xa000
#define RNG_OFF		0xb000
#define SDIO_OFF	0xc000
#define I2S_OFF		0xd000

#define IPI_BASE_OF(i)	CKSEG1ADDR((CONF_BASE+0x100*i))

#define IPI_OFF_STATUS 		0x1000
#define IPI_OFF_ENABLE 		0x1004
#define IPI_OFF_SET	 	0x1008
#define IPI_OFF_CLEAR		0x100c
#define IPI_OFF_MAILBOX0	0x1020
#define IPI_OFF_MAILBOX1	0x1028
#define IPI_OFF_MAILBOX2	0x1030
#define IPI_OFF_MAILBOX3	0x1038

#define VRAM_TYPE_SP	0
#define VRAM_TYPE_UMA	1
#define VRAM_TYPE_SP		0	/*GPU use his private memory with special address space*/
#define VRAM_TYPE_UMA_SP	1	/*GPU use his private memory with unified address space*/
#define VRAM_TYPE_UMA_LOW	2	/*GPU use low address syetem memory with unified address space*/
#define VRAM_TYPE_UMA_HIGH	3	/*GPU use high address syetem memory with unified address space*/
#define VRAM_TYPE_SP_LOW	4	/*GPU use low address syetem memory with special address space*/
#define VRAM_TYPE_SP_HIGH	5	/*GPU use high address syetem memory with special address space*/

#define LS2K_IO_REG_BASE		0x1f000000

/* CHIP CONFIG regs */
#define LS2K_CHIP_CFG_REG_BASE		(LS2K_IO_REG_BASE + 0x00e10000)
#define LS2K_APBDMA_CONFIG_REG		(LS2K_CHIP_CFG_REG_BASE + 0x438)
#define LS2K_GEN_CONFIG0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x420)
#define LS2K_GEN_CONFIG1_REG		(LS2K_CHIP_CFG_REG_BASE + 0x428)

#define LS2K_PIX0_PLL			(LS2K_CHIP_CFG_REG_BASE + 0x4b0)
#define LS2K_PIX1_PLL			(LS2K_CHIP_CFG_REG_BASE + 0x4c0)

#define LS2K_INT_REG_BASE		(LS2K_CHIP_CFG_REG_BASE + 0x1420)

#define LS2K_INT_ISR0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x1420)
#define LS2K_INT_IEN0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x1424)
#define LS2K_INT_SET0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x1428)
#define LS2K_INT_CLR0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x142c)
#define LS2K_INT_POL0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x1430)
#define LS2K_INT_EDGE0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x1434)
#define LS2K_INT_BOUNCE_REG		(LS2K_CHIP_CFG_REG_BASE + 0x1438)
#define LS2K_INT_AUTO_REG		(LS2K_CHIP_CFG_REG_BASE + 0x143c)

#define LS2K_GPIO0_OEN_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0500)
#define LS2K_GPIO1_OEN_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0508)
#define LS2K_GPIO0_O_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0510)
#define LS2K_GPIO1_O_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0518)
#define LS2K_GPIO0_I_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0520)
#define LS2K_GPIO1_I_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0528)
#define LS2K_GPIO0_INT_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0530)
#define LS2K_GPIO1_INT_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0538)

#define LS2K_DMA_ORDER_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0c00)
#define LS2K_CHIP_CFG0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0200)
#define LS2K_CHIP_CFG1_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0204)
#define LS2K_CHIP_CFG2_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0208)
#define LS2K_CHIP_CFG3_REG		(LS2K_CHIP_CFG_REG_BASE + 0x020c)
#define LS2K_CHIP_SAMP0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0210)
#define LS2K_CHIP_SAMP1_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0214)
#define LS2K_CHIP_SAMP2_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0218)
#define LS2K_CHIP_SAMP3_REG		(LS2K_CHIP_CFG_REG_BASE + 0x021c)
#define LS2K_CLK_CTRL0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0220)
#define LS2K_CLK_CTRL1_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0224)
#define LS2K_CLK_CTRL2_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0228)
#define LS2K_CLK_CTRL3_REG		(LS2K_CHIP_CFG_REG_BASE + 0x022c)
#define LS2K_PIXCLK0_CTRL0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0230)
#define LS2K_PIXCLK0_CTRL1_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0234)
#define LS2K_PIXCLK1_CTRL0_REG		(LS2K_CHIP_CFG_REG_BASE + 0x0238)
#define LS2K_PIXCLK1_CTRL1_REG		(LS2K_CHIP_CFG_REG_BASE + 0x023c)

#define LS2K_WIN_CFG_BASE		(LS2K_CHIP_CFG_REG_BASE + 0x80000)
#define LS2K_M4_WIN0_BASE_REG		(LS2K_WIN_CFG_BASE + 0x0400)
#define LS2K_M4_WIN0_MASK_REG		(LS2K_WIN_CFG_BASE + 0x0440)
#define LS2K_M4_WIN0_MMAP_REG		(LS2K_WIN_CFG_BASE + 0x0480)

/* USB regs */
#define LS2K_EHCI_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00000)
#define LS2K_OHCI_REG_BASE		(LS2K_IO_REG_BASE + 0x00e08000)

/* GMAC regs */
#define LS2K_GMAC0_REG_BASE		(LS2K_IO_REG_BASE + 0x00e10000)
#define LS2K_GMAC1_REG_BASE		(LS2K_IO_REG_BASE + 0x00e18000)

/* HDA regs */
#define LS2K_HDA_REG_BASE		(LS2K_IO_REG_BASE + 0x00e20000)

/* SATAregs */
#define LS2K_SATA_REG_BASE		(LS2K_IO_REG_BASE + 0x00e30000)

/* OTG regs */
#define LS2K_OTG_REG_BASE		(LS2K_IO_REG_BASE + 0x00e60000)

/* UART regs */
#define LS2K_UART0_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00000)
#define LS2K_UART1_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00100)
#define LS2K_UART2_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00200)
#define LS2K_UART3_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00300)
#define LS2K_UART4_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00400)
#define LS2K_UART5_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00500)
#define LS2K_UART6_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00600)
#define LS2K_UART7_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00700)
#define LS2K_UART8_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00800)
#define LS2K_UART9_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00900)
#define LS2K_UART10_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00a00)
#define LS2K_UART11_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00b00)

#define LS2K_CAN0_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00c00)
#define LS2K_CAN1_REG_BASE		(LS2K_IO_REG_BASE + 0x00e00d00)

/* I2C regs */
#define LS2K_I2C0_REG_BASE		(LS2K_IO_REG_BASE + 0x00e01000)
#define LS2K_I2C0_PRER_LO_REG		(LS2K_I2C0_REG_BASE + 0x0)
#define LS2K_I2C0_PRER_HI_REG		(LS2K_I2C0_REG_BASE + 0x1)
#define LS2K_I2C0_CTR_REG   		(LS2K_I2C0_REG_BASE + 0x2)
#define LS2K_I2C0_TXR_REG   		(LS2K_I2C0_REG_BASE + 0x3)
#define LS2K_I2C0_RXR_REG    		(LS2K_I2C0_REG_BASE + 0x3)
#define LS2K_I2C0_CR_REG     		(LS2K_I2C0_REG_BASE + 0x4)
#define LS2K_I2C0_SR_REG     		(LS2K_I2C0_REG_BASE + 0x4)

#define LS2K_I2C1_REG_BASE		(LS2K_IO_REG_BASE + 0x00e01800)
#define LS2K_I2C1_PRER_LO_REG		(LS2K_I2C1_REG_BASE + 0x0)
#define LS2K_I2C1_PRER_HI_REG		(LS2K_I2C1_REG_BASE + 0x1)
#define LS2K_I2C1_CTR_REG    		(LS2K_I2C1_REG_BASE + 0x2)
#define LS2K_I2C1_TXR_REG    		(LS2K_I2C1_REG_BASE + 0x3)
#define LS2K_I2C1_RXR_REG    		(LS2K_I2C1_REG_BASE + 0x3)
#define LS2K_I2C1_CR_REG     		(LS2K_I2C1_REG_BASE + 0x4)
#define LS2K_I2C1_SR_REG     		(LS2K_I2C1_REG_BASE + 0x4)

/* RTC regs */
#define LS2K_RTC_REG_BASE		(LS2K_IO_REG_BASE + 0x00e07800)
#define	LS2K_TOY_TRIM_REG		(LS2K_RTC_REG_BASE + 0x0020)
#define	LS2K_TOY_WRITE0_REG		(LS2K_RTC_REG_BASE + 0x0024)
#define	LS2K_TOY_WRITE1_REG		(LS2K_RTC_REG_BASE + 0x0028)
#define	LS2K_TOY_READ0_REG		(LS2K_RTC_REG_BASE + 0x002c)
#define	LS2K_TOY_READ1_REG		(LS2K_RTC_REG_BASE + 0x0030)
#define	LS2K_TOY_MATCH0_REG		(LS2K_RTC_REG_BASE + 0x0034)
#define	LS2K_TOY_MATCH1_REG		(LS2K_RTC_REG_BASE + 0x0038)
#define	LS2K_TOY_MATCH2_REG		(LS2K_RTC_REG_BASE + 0x003c)
#define	LS2K_RTC_CTRL_REG		(LS2K_RTC_REG_BASE + 0x0040)
#define	LS2K_RTC_TRIM_REG		(LS2K_RTC_REG_BASE + 0x0060)
#define	LS2K_RTC_WRITE0_REG		(LS2K_RTC_REG_BASE + 0x0064)
#define	LS2K_RTC_READ0_REG		(LS2K_RTC_REG_BASE + 0x0068)
#define	LS2K_RTC_MATCH0_REG		(LS2K_RTC_REG_BASE + 0x006c)
#define	LS2K_RTC_MATCH1_REG		(LS2K_RTC_REG_BASE + 0x0070)
#define	LS2K_RTC_MATCH2_REG		(LS2K_RTC_REG_BASE + 0x0074)

#define LS2K_SDIO_REG_BASE		(LS2K_IO_REG_BASE + 0x00e0c000)
#define LS2K_I2S_REG_BASE		(LS2K_IO_REG_BASE + 0x00e0d000)


#define LS2K_IOINTC_HWIRQ_BASE 8  /* should keep consistent with dts  */
#define LS2K_INT_MSI_TRIGGER_0     (LS2K_CHIP_CFG_REG_BASE + 0x14b0)
#define LS2K_INT_MSI_TRIGGER_EN_0  (LS2K_CHIP_CFG_REG_BASE + 0x14b4)
#define LS2K_INT_MSI_TRIGGER_1     (LS2K_CHIP_CFG_REG_BASE + 0x14f0)
#define LS2K_INT_MSI_TRIGGER_EN_1  (LS2K_CHIP_CFG_REG_BASE + 0x14f4)
#define LS2K_IRQ_MASK              0xffffff3fbffff3ff

#define LS2K_INTISR_REG(i)	TO_UNCAC(0x1fe11420 | (((i) > 31) ? 0x40 : 0))
#define LS2K_INTEN_REG(i)	TO_UNCAC(0x1fe11424 | (((i) > 31) ? 0x40 : 0))
#define LS2K_INTENSET_REG(i)	TO_UNCAC(0x1fe11428 | (((i) > 31) ? 0x40 : 0))
#define LS2K_INTENCLR_REG(i)	TO_UNCAC(0x1fe1142c | (((i) > 31) ? 0x40 : 0))
#define LS2K_INTPOL_REG(i)	TO_UNCAC(0x1fe11430 | (((i) > 31) ? 0x40 : 0))
#define LS2K_INTEDGE_REG(i)	TO_UNCAC(0x1fe11434 | (((i) > 31) ? 0x40 : 0))
#define LS2K_INTBOUNCE_REG(i)	TO_UNCAC(0x1fe11438 | (((i) > 31) ? 0x40 : 0))
#define LS2K_INTAUTO_REG(i)	TO_UNCAC(0x1fe1143c | (((i) > 31) ? 0x40 : 0))
#define LS2K_COREx_INTISR0_REG(x)	TO_UNCAC(0x1fe11040 + (x)*0x100)
#define LS2K_COREx_INTISR1_REG(x)	TO_UNCAC(0x1fe11048 + (x)*0x100)
#define LS2K_IRQ_ROUTE_REG(i)	TO_UNCAC((0x1fe11400 | (((i) > 31) ? 0x40 : 0))\
				+ ((i) % 32))
#define LS2K_IOINTC_IRQ_LINE   1  /* IP3 */

/* REG ACCESS*/
#define ls2k_readb(addr)		(*(volatile unsigned char *)TO_UNCAC(addr))
#define ls2k_readw(addr)		(*(volatile unsigned short *)TO_UNCAC(addr))
#define ls2k_readl(addr)		(*(volatile unsigned int *)TO_UNCAC(addr))
#define ls2k_readq(addr)		(*(volatile unsigned long *)TO_UNCAC(addr))
#define ls2k_writeb(val, addr)	(*(volatile unsigned char *)TO_UNCAC(addr) = (val))
#define ls2k_writew(val, addr)	(*(volatile unsigned short *)TO_UNCAC(addr) = (val))
#define ls2k_writel(val, addr)	(*(volatile unsigned int *)TO_UNCAC(addr) = (val))
#define ls2k_writeq(val, addr)	(*(volatile unsigned long *)TO_UNCAC(addr) = (val))

enum ACPI_Sx {
	ACPI_S3 = 5,
	ACPI_S4 = 6,
	ACPI_S5 = 7,
};

#define ACPI_BASE 	(APB_BASE + ACPI_OFF)
#define S_GPMCR		0
#define R_GPMCR		0x4
#define RTC_GPMCR	0x8
#define PM1_STS		0xc
#define PM1_SR		0x10
#define PM1_CTR		0x14
#define PM1_TIMER	0x18
#define PM_PCTR		0x1c

#define GPE0_STS	0x28
#define GPE0_SR		0x2c
#define RST_CTR		0x30
#define WD_CR		0x34
#define WD_TIMER	0x38

#define THER_SCR	0x4c
#define G_RTC_1		0x50
#define G_RTC_2		0x54

#define DPM_CfG		0x400
#define DPM_STS		0x404
#define DPM_CTR		0x408

#define DVFS_CFG	0x410
#define DVFS_STS	0x414
#define DVFS_CNT	0x418

#define MIN_FREQ_LEVEL 1
#define DVFS_STS_STATUS_MASK	(0x01)

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

#define DVFS_STS_FEQ_STS_OFFSET 16
#define DVFS_STS_FEQ_STS_MASK   GENMASK(21, 16)

/* ACPI ACCESS */
#define acpi_readb(offset)			ls2k_readb((ACPI_BASE + offset))
#define acpi_readw(offset) 			ls2k_readw((ACPI_BASE + offset))
#define acpi_readl(offset) 			ls2k_readl((ACPI_BASE + offset))
#define acpi_readq(offset) 			ls2k_readq((ACPI_BASE + offset))
#define acpi_writeb(val, offset)	ls2k_writeb((val), (ACPI_BASE + offset))
#define acpi_writew(val, offset) 	ls2k_writew((val), (ACPI_BASE + offset))
#define acpi_writel(val, offset) 	ls2k_writel((val), (ACPI_BASE + offset))
#define acpi_writeq(val, offset) 	ls2k_writeq((val), (ACPI_BASE + offset))

#endif /* __ASM_MACH_LOONGSON_2K_H_ */
