/*
 *  linux/drivers/mmc/ls2k_mci.h - Loongson ls2k MCI driver
 *
 *  Copyright (C) loongson, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FCR_SDIO_H_
#define FCR_SDIO_H_

#include <linux/interrupt.h>

#define _REG(x) 	((volatile u32 *)(x))
#define _REG2(b, o) 	((volatile u32 *)((b)+(o)))

/*
 * sdio_controler
 */
#ifndef CONFIG_LOONGARCH
#define LS2K1000_SDIO_REG_BASE	0x1fe0c000
#define LS2K1000_GPIO_INTEN_BASE  TO_UNCAC(0x1fe10530)
#else
#define LS2K1000_SDIO_REG_BASE	0x1fe2c000
#define LS2K1000_GPIO_INTEN_BASE  TO_UNCAC(0x1fe00530)
#endif

#define LS2K0500_SDIO0_REG_BASE	0x1ff64000
#define LS2K0500_SDIO0_GPIO_INTEN_BASE  TO_UNCAC(0x1fe104e4) /* GPIO32~63 */
#define LS2K0500_SDIO1_REG_BASE	0x1ff66000
#define LS2K0500_SDIO1_GPIO_INTEN_BASE  TO_UNCAC(0x1fe104e8) /* GPIO64~95 */

#define SDIO_DEV_ADDR   (host->base+0x40)
#define WDMA_OFFSET		0x400
#define RDMA_OFFSET		0x800

#define SDICON     0x00
#define SDIPRE     0x04
#define SDICMDARG  0x08
#define SDICMDCON  0x0c
#define SDICMDSTA  0x10
#define SDIRSP0    0x14
#define SDIRSP1    0x18
#define SDIRSP2    0x1C
#define SDIRSP3    0x20
#define SDIDTIMER  0x24
#define SDIBSIZE   0x28
#define SDIDATCON  0x2C
#define SDIDATCNT  0x30
#define SDIDSTA    0x34
#define SDIFSTA    0x38
#define SDIINTMSK  0x3C
#define SDIWRDAT   0x40
#define SDISTAADD0 0x44
#define SDISTAADD1 0x48
#define SDISTAADD2 0x4c
#define SDISTAADD3 0x50
#define SDISTAADD4 0x54
#define SDISTAADD5 0x58
#define SDISTAADD6 0x5c
#define SDISTAADD7 0x60
#define SDIINTEN   0x64

#define DAT_4_WIRE 1
#define ERASE_START_ADDR 0x5000
#define ERASE_End_ADDR   0x5000

#define SDICMDSTAT            (0x10)
#define SDITIMER              (0x24)
#define SDIDCON               (0x2C)
#define SDIDCNT               (0x30)

#define SDIDATA               (0x3C)
#define SDIIMSK               (0x40)

#define SDICON_RESET          (1<<8)
#define SDICON_MMCCLOCK       (1<<5)
#define SDICON_BYTEORDER      (1<<4)
#define SDICON_SDIOIRQ        (1<<3)
#define SDICON_RWAITEN        (1<<2)
#define SDICON_FIFORESET      (1<<1)
#define SDICON_CLOCKTYPE      (1<<0)

#define SDIPRE_REVCLOCK       (1<<31)

#define SDICMDCON_CMD6DATA    (1<<18)
#define SDICMDCON_ABORT       (1<<12)
#define SDICMDCON_WITHDATA    (1<<11)
#define SDICMDCON_LONGRSP     (1<<10)
#define SDICMDCON_WAITRSP     (1<<9)
#define SDICMDCON_CMDSTART    (1<<8)
#define SDICMDCON_SENDERHOST  (1<<6)
#define SDICMDCON_INDEX       (0x3f)

#define SDICMDSTAT_CRCFAIL    (1<<12)
#define SDICMDSTAT_CMDSENT    (1<<11)
#define SDICMDSTAT_CMDTIMEOUT (1<<10)
#define SDICMDSTAT_RSPFIN     (1<<9)
#define SDICMDSTAT_XFERING    (1<<8)
#define SDICMDSTAT_INDEX      (0xff)

#define SDIDCON_8BIT_BUS      (1<<26)
#define SDIDCON_IRQPERIOD     (1<<21)
#define SDIDCON_TXAFTERRESP   (1<<20)
#define SDIDCON_RXAFTERCMD    (1<<19)
#define SDIDCON_BUSYAFTERCMD  (1<<18)
#define SDIDCON_BLOCKMODE     (1<<17)
#define SDIDCON_4BIT_BUS      (1<<16)
#define MISC_CTRL_SDIO_DMAEN  (1<<15)
#define SDIDCON_STOP          (1<<14)
#define SDIDCON_DATMODE	      (3<<12)
#define SDIDCON_BLKNUM        (0x7ff)

/* constants for SDIDCON_DATMODE */
#define SDIDCON_XFER_READY    (0<<12)
#define SDIDCON_XFER_CHKSTART (1<<12)
#define SDIDCON_XFER_RXSTART  (2<<12)
#define SDIDCON_XFER_TXSTART  (3<<12)

#define SDIDCON_BLKNUM_MASK   (0xFFF)
#define SDIDCNT_BLKNUM_SHIFT  (12)

#define SDIDSTA_RDYWAITREQ    (1<<10)
#define SDIDSTA_SDIOIRQDETECT (1<<9)
#define SDIDSTA_FIFOFAIL      (1<<8)
#define SDIDSTA_CRCFAIL       (1<<7)
#define SDIDSTA_RXCRCFAIL     (1<<6)
#define SDIDSTA_DATATIMEOUT   (1<<5)
#define SDIDSTA_XFERFINISH    (1<<4)
#define SDIDSTA_BUSYFINISH    (1<<3)
#define SDIDSTA_SBITERR       (1<<2)
#define SDIDSTA_TXDATAON      (1<<1)
#define SDIDSTA_RXDATAON      (1<<0)

#define SDIFSTA_TFDET          (1<<13)
#define SDIFSTA_RFDET          (1<<12)
#define SDIFSTA_TFFULL         (1<<11)
#define SDIFSTA_TFEMPTY        (1<<10)
#define SDIFSTA_RFLAST         (1<<9)
#define SDIFSTA_RFFULL         (1<<8)
#define SDIFSTA_RFEMPTY        (1<<7)
#define SDIFSTA_COUNTMASK      (0x7f)

#define SDIIMSK_RESPONSEND     (1<<14)
#define SDIIMSK_READWAIT       (1<<13)
#define SDIIMSK_SDIOIRQ        (1<<12)
#define SDIIMSK_FIFOFAIL       (1<<11)


#define SDIIMSK_RESPONSECRC    (1<<8)
#define SDIIMSK_CMDTIMEOUT     (1<<7)
#define SDIIMSK_CMDSENT        (1<<6)
#define SDIIMSK_PROGERR        (1<<4)
#define SDIIMSK_TXCRCFAIL      (1<<3)
#define SDIIMSK_RXCRCFAIL      (1<<2)
#define SDIIMSK_DATATIMEOUT    (1<<1)
#define SDIIMSK_DATAFINISH     (1<<0)

#define SDIIMSK_BUSYFINISH     (1<<6)
#define SDIIMSK_SBITERR        (1<<5)
#define SDIIMSK_TXFIFOHALF     (1<<4)
#define SDIIMSK_TXFIFOEMPTY    (1<<3)
#define SDIIMSK_RXFIFOLAST     (1<<2)
#define SDIIMSK_RXFIFOFULL     (1<<1)
#define SDIIMSK_RXFIFOHALF     (1<<0)

#define SDIINT_CLEARALL        0x1ff
#define SDIINT_ENALL           0x1ff

#define nr_strtol strtoul

enum ls2k_mci_waitfor {
	COMPLETION_NONE,
	COMPLETION_FINALIZE,
	COMPLETION_CMDSENT,
	COMPLETION_RSPFIN,
	COMPLETION_XFERFINISH,
	COMPLETION_XFERFINISH_RSPFIN,
};

typedef struct ls2k_dma_desc{
	volatile u32 order_addr;
	volatile u32 saddr;
	volatile u32 daddr;
	volatile u32 length;
	volatile u32 step_length;
	volatile u32 step_times;
	volatile u32 cmd;
	volatile u32 dummy;
	volatile u32 order_addr_hi;
	volatile u32 saddr_hi;
} ls2k_dma_desc;

struct ls2k_mci_host {
	struct platform_device	*pdev;
	struct ls2k_mci_pdata *pdata;
	struct mmc_host		*mmc;
	struct resource		*mem;
	struct clk		*clk;
	void __iomem		*base;
	dma_addr_t		phys_base;
	void __iomem		*wdma_order_reg;
	void __iomem		*rdma_order_reg;
	int			irq;
	int			irq_cd;
	int			dma;
	unsigned long dma_send_vir_mem;
	unsigned long dma_send_phy_mem;
	unsigned long dma_rec_vir_mem;
	unsigned long dma_rec_phy_mem;
	struct ls2k_dma_desc *sg_cpu;
	dma_addr_t sg_dma;

	unsigned long		clk_rate;
	unsigned long		clk_div;
	unsigned long		real_rate;
	u8			prescaler;

	int			is2440;
	unsigned		sdiimsk;
	unsigned		sdidata;
	int			dodma;
	int			dmatogo;

	bool			irq_disabled;
	bool			irq_enabled;
	bool			irq_state;
	int			sdio_irqen;

	struct mmc_request	*mrq;
	int			cmd_is_stop;

	spinlock_t		complete_lock;
	enum ls2k_mci_waitfor	complete_what;

	int			dma_complete;

	u32			pio_sgptr;
	u32			pio_bytes;
	u32			pio_count;
	u32			*pio_ptr;
#define XFER_NONE 0
#define XFER_READ 1
#define XFER_WRITE 2
	u32			pio_active;

	int			bus_width;

	char 			dbgmsg_cmd[301];
	char 			dbgmsg_dat[301];
	char			*status;

	unsigned int		ccnt, dcnt;
	struct tasklet_struct	pio_tasklet;

#ifdef CONFIG_DEBUG_FS
	struct dentry		*debug_root;
	struct dentry		*debug_state;
	struct dentry		*debug_regs;
#endif

#ifdef CONFIG_CPU_FREQ
	struct notifier_block	freq_transition;
#endif
	struct dma_chan         *chan;
	unsigned int		app_cmd;
};

struct ls2k_mci_pdata {
	unsigned int	no_wprotect : 1;
	unsigned int	no_detect : 1;
	unsigned int	wprotect_invert : 1;
	unsigned int	detect_invert : 1;   /* set => detect active high. */
	unsigned int	use_dma : 1;

	unsigned int	gpio_detect;
	unsigned int	gpio_wprotect;
#define LOONGSON_SDIO_EMMC_VER_1_0	0x100
#define LOONGSON_SDIO_EMMC_VER_1_1	0x110
#define LOONGSON_SDIO_EMMC_VER_1_2	0x120
	unsigned int	version;
	unsigned long	ocr_avail;
	void	(*set_power)(unsigned char power_mode,
				     unsigned short vdd);
	void	(*irq_fixup)(struct ls2k_mci_host *host,
					struct mmc_command *cmd);
};
#endif
