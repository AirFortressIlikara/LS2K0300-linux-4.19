/*
 * Loongson RapidIO controller definitions
 *
 * Copyright 2023, Loongson Technology, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef __LS_RIO_H
#define __LS_RIO_H

/* Debug output filtering masks */
enum {
	DBG_NONE	= 0,
	DBG_INIT	= BIT(0), /* driver init */
	DBG_EXIT	= BIT(1), /* driver exit */
	DBG_MPORT	= BIT(2), /* mport add/remove */
	DBG_MAINT	= BIT(3), /* maintenance ops messages */
	DBG_DMA		= BIT(4), /* DMA transfer messages */
	DBG_DMAV	= BIT(5), /* verbose DMA transfer messages */
	DBG_IBW		= BIT(6), /* inbound window */
	DBG_EVENT	= BIT(7), /* event handling messages */
	DBG_OBW		= BIT(8), /* outbound window messages */
	DBG_DBELL	= BIT(9), /* doorbell messages */
	DBG_OMSG	= BIT(10), /* doorbell messages */
	DBG_IMSG	= BIT(11), /* doorbell messages */
	DBG_ALL		= ~0,
};

#ifdef DEBUG
extern u32 ls_dbg_level;

#define ls_debug(level, dev, fmt, arg...)				\
	do {								\
		if (DBG_##level & ls_dbg_level)				\
			dev_dbg(dev, "%s: " fmt "\n", __func__, ##arg);	\
	} while (0)
#else
#define ls_debug(level, dev, fmt, arg...) \
		no_printk(KERN_DEBUG "%s: " fmt "\n", __func__, ##arg)
#endif

#define ls_info(dev, fmt, arg...) \
	dev_info(dev, "%s: " fmt "\n", __func__, ##arg)

#define ls_warn(dev, fmt, arg...) \
	dev_warn(dev, "%s: WARNING " fmt "\n", __func__, ##arg)

#define ls_err(dev, fmt, arg...) \
	dev_err(dev, "%s: ERROR " fmt "\n", __func__, ##arg)

#define DRV_NAME	"lsrio"

#define BAR_0	0
#define BAR_1	1
#define BAR_2	2
#define BAR_4	4

/* Register definitions */

#define LSRIO_STD_REGBASE		0x0
#define LSRIO_GRIO_REGBASE		0x10000
#define LSRIO_RAB_REGBASE		0x20000
#define LSRIO_REG_PAGESIZE		0x800

#define LSRIO_RAB_CAPA				0x20004
#define LSRIO_RAB_CTRL				0x20008
#define LSRIO_RAB_CTRL_APIO_EN		BIT(0)
#define LSRIO_RAB_CTRL_RPIO_EN		BIT(1)
#define LSRIO_RAB_CTRL_WDMA_EN		BIT(2)
#define LSRIO_RAB_CTRL_RDMA_EN		BIT(3)
#define LSRIO_RAB_CTRL_ABSS_MASK	GENMASK(5, 4)
#define LSRIO_RAB_CTRL_ABSS_NS		0
#define LSRIO_RAB_CTRL_ABSS_SD		0x10
#define LSRIO_RAB_CTRL_ABSS_SW		0x20
#define LSRIO_RAB_CTRL_ABSS_SH		0x30
#define LSRIO_RAB_CTRL_DMA_1_PF		0
#define LSRIO_RAB_CTRL_DMA_2_PF		0x100
#define LSRIO_RAB_CTRL_DMA_4_PF		0x200
#define LSRIO_RAB_CTRL_DMA_8_PF		0x300

#define LSRIO_RAB_CTRL_DEF			(LSRIO_RAB_CTRL_APIO_EN | \
										LSRIO_RAB_CTRL_RPIO_EN)

#define LSRIO_RAB_RST_CTRL			0x20018
#define LSRIO_RAB_RST_CTRL_WDMA_SHIFT	16
#define LSRIO_RAB_RST_CTRL_RDMA_SHIFT	24

#define LSRIO_RAB_APB_CSR			0x20030
#define LSRIO_APB_CSR_BSN			0
#define LSRIO_APB_CSR_BSW			0x2
#define LSRIO_APB_CSR_BSHW			0x3

#define LSRIO_RAB_INTR_GNRL_APIO	BIT(0)
#define LSRIO_RAB_INTR_GNRL_RPIO	BIT(1)
#define LSRIO_RAB_INTR_GNRL_WDMA	BIT(2)
#define LSRIO_RAB_INTR_GNRL_RDMA	BIT(3)
#define LSRIO_RAB_INTR_GNRL_IDME	BIT(4)
#define LSRIO_RAB_INTR_GNRL_ODME	BIT(5)
#define LSRIO_RAB_INTR_GNRL_MISC	BIT(6)
#define LSRIO_RAB_INTR_GNRL_IBDS	BIT(7)
#define LSRIO_RAB_INTR_GNRL_OBDS	BIT(8)

#define LSRIO_RAB_INTR_ENAB_GNRL	0x20040

#define LSRIO_RAB_INTR_GNRL_ENALL	(LSRIO_RAB_INTR_GNRL_APIO | \
										LSRIO_RAB_INTR_GNRL_RPIO | \
										LSRIO_RAB_INTR_GNRL_WDMA | \
										LSRIO_RAB_INTR_GNRL_RDMA | \
										LSRIO_RAB_INTR_GNRL_IDME | \
										LSRIO_RAB_INTR_GNRL_ODME | \
										LSRIO_RAB_INTR_GNRL_MISC | \
										LSRIO_RAB_INTR_GNRL_IBDS | \
										LSRIO_RAB_INTR_GNRL_OBDS)

#define LSRIO_RAB_INTR_ENAB_APIO	0x20044
#define LSRIO_RAB_INTR_APIO_ENALL	0x101

#define LSRIO_RAB_INTR_ENAB_RPIO	0x20048
#define LSRIO_RAB_INTR_RPIO_ENALL	0x101

#define LSRIO_RAB_INTR_ENAB_WDMA	0x2004c
#define LSRIO_RAB_INTR_WDMA_ENALL	0xf0f0f

#define LSRIO_RAB_INTR_ENAB_RDMA	0x20050
#define LSRIO_RAB_INTR_RDMA_ENALL	0xf0f0f

#define LSRIO_RAB_INTR_ENAB_IDME	0x20054
#define LSRIO_RAB_INTR_IDME_CH(n)	BIT(n)
#define LSRIO_RAB_INTR_IDME_ENALL	0xffffffff

#define LSRIO_RAB_INTR_ENAB_ODME	0x20058
#define LSRIO_RAB_INTR_ODME_CH(n)	BIT(n)
#define LSRIO_RAB_INTR_ODME_ENALL	0xf

#define LSRIO_RAB_INTR_ENAB_MISC	0x2005c
#define LSRIO_RAB_INTR_MISC_ENALL	0xfff

#define LSRIO_RAB_INTR_ENAB_IBDS	0x22a04
#define LSRIO_RAB_INTR_IBDS_ENALL	0xffff

#define LSRIO_RAB_INTR_ENAB_OBDS	0x22a0c
#define LSRIO_RAB_INTR_OBDS_ENALL	0xffff

#define LSRIO_RAB_INTR_STAT_GNRL	0x20060

#define LSRIO_RAB_INTR_STAT_APIO	0x20064
#define LSRIO_RAB_INTR_STAT_RPIO	0x20068
#define LSRIO_RAB_INTR_STAT_WDMA	0x2006c
#define LSRIO_RAB_INTR_STAT_RDMA	0x20070
#define LSRIO_RAB_INTR_STAT_IDME	0x20074
#define LSRIO_RAB_INTR_STAT_ODME	0x20078

#define LSRIO_RAB_INTR_STAT_MISC	0x2007c
#define LSRIO_RAB_INTR_MISC_IBDR	BIT(0)
#define LSRIO_RAB_INTR_MISC_OBDD	BIT(1)

#define LSRIO_RAB_INTR_STAT_IBDS	0x22a10
#define LSRIO_RAB_INTR_STAT_OBDS	0x22a18

#define LSRIO_RAB_RPIO_CTRL						0x20080
#define LSRIO_RPIO_PIO_EN						BIT(0)
#define LSRIO_RPIO_RLX_ORD						BIT(1)

#define LSRIO_RAB_RPIO_STAT						0x20084
#define LSRIO_RAB_RPIO_AMAP_BASE				0x20100
#define LSRIO_RAB_RPIO_AMAP_LUT(n)				(LSRIO_RAB_RPIO_AMAP_BASE + n * 0x4)
#define LSRIO_RAB_RPIO_AMAP_LUT_EN				BIT(0)
#define LSRIO_RAB_RPIO_AMAP_LUT_256				(8 << 1)
#define LSRIO_RAB_RPIO_AMAP_LUT_WSIZE_SHIFT		1
#define LSRIO_RAB_RPIO_AMAP_LUT_AXIUP_SHIFT		14
#define LSRIO_RAB_RPIO_AMAP_IDSL				0x20140
#define LSRIO_RAB_RPIO_AMAP_BYPS				0x20144
#define LSRIO_RAB_RPIO_AMAP_BYPS_RAS			BIT(0)

#define LSRIO_RAB_APIO_CTRL						0x20180
#define LSRIO_APIO_PIO_EN						BIT(0)
#define LSRIO_APIO_MEM_EN						BIT(1)
#define LSRIO_APIO_MAINT_EN						BIT(2)
#define LSRIO_APIO_CCP_EN						BIT(3)

#define LSRIO_RAB_APIO_STAT						0x20184

#define LSRIO_RAB_APIO_AMAP_BASE				0x20200
#define LSRIO_RAB_APIO_AMAP_BAR(n)				(LSRIO_RAB_APIO_AMAP_BASE + n * 0x10)
#define LSRIO_RAB_APIO_AMAP_CTRL(n)				(LSRIO_RAB_APIO_AMAP_BAR(n) + 0x0)
#define LSRIO_RAB_APIO_AMAP_SIZE(n)				(LSRIO_RAB_APIO_AMAP_BAR(n) + 0x4)
#define LSRIO_RAB_APIO_AMAP_ABAR(n)				(LSRIO_RAB_APIO_AMAP_BAR(n) + 0x8)
#define LSRIO_RAB_APIO_AMAP_RBAR(n)				(LSRIO_RAB_APIO_AMAP_BAR(n) + 0xc)
#define LSRIO_AMAP_BAR_ALIGN					0x400

#define LSRIO_RAB_APIO_AMAP_CTRL_EN				BIT(0)

#define LSRIO_RAB_APIO_AMAP_CTRL_TYPE_MASK		GENMASK(2, 1)
#define LSRIO_RAB_APIO_AMAP_CTRL_TYPE_SHIFT		1

#define LSRIO_RAB_APIO_AMAP_CTRL_PRI_SHIFT		3
#define LSRIO_RAB_APIO_AMAP_CTRL_PRI_MASK		GENMASK(4, 3)

#define LSRIO_RAB_APIO_AMAP_CTRL_XADDR_SHIFT	5
#define LSRIO_RAB_APIO_AMAP_CTRL_XADDR_MASK		GENMASK(6, 5)

#define LSRIO_RAB_APIO_AMAP_CTRL_CRF			BIT(7)

#define LSRIO_RAB_APIO_AMAP_CTRL_DSTID_SHIFT	16
#define LSRIO_RAB_APIO_AMAP_CTRL_DSTID_MASK		GENMASK(31, 16)

#define LSRIO_RAB_APIO_AMAP_SIZE_SHIFT			10
#define LSRIO_RAB_APIO_AMAP_SIZE_MASK			GENMASK(31, 10)

#define LSRIO_RAB_APIO_AMAP_ABAR_SHIFT			0
#define LSRIO_RAB_APIO_AMAP_ABAR_MASK			GENMASK(27, 0)

#define LSRIO_RAB_APIO_AMAP_RBAR_HC_SHIFT		14
#define LSRIO_RAB_APIO_AMAP_RBAR_MASK			GENMASK(21, 0)

#define LSRIO_RAB_OBDB_BASE				0x20400
#define LSRIO_RAB_OBDB_BAR(n)			(LSRIO_RAB_OBDB_BASE + n * 0x8)
#define LSRIO_RAB_OBDB_CTRL(n)			(LSRIO_RAB_OBDB_BAR(n) + 0x0)
#define LSRIO_RAB_OBDB_INFO(n)			(LSRIO_RAB_OBDB_BAR(n) + 0x4)
#define LSRIO_RAB_OBDB_CTRL_STAT		GENMASK(3, 1)

#define LSRIO_RAB_IBDB_CSR				0x20480
#define LSRIO_RAB_IBDB_EN				BIT(0)
#define LSRIO_RAB_IBDB_NUM_SHIFT		16
#define LSRIO_RAB_IBDB_INFO				0x20484
#define LSRIO_RAB_IBDB_SRCID_SHIFT		16

#define LSRIO_RAB_OBDB_SEND				BIT(0)
#define LSRIO_RAB_OBDB_DID_SHIFT		16

#define LSRIO_RAB_OBDME_BASE			0x20500
#define LSRIO_RAB_OBDME_BAR(n)			(LSRIO_RAB_OBDME_BASE + n * 0x10)
#define LSRIO_RAB_OBDME_CTRL(n)			(LSRIO_RAB_OBDME_BAR(n) + 0x0)
#define LSRIO_RAB_OBDME_DADDR(n)		(LSRIO_RAB_OBDME_BAR(n) + 0x4)
#define LSRIO_RAB_OBDME_STAT(n)			(LSRIO_RAB_OBDME_BAR(n) + 0x8)
#define LSRIO_RAB_OBDME_DESC(n)			(LSRIO_RAB_OBDME_BAR(n) + 0xc)

#define LSRIO_RAB_OBDME_CTRL_EN				BIT(0)
#define LSRIO_RAB_OBDME_CTRL_WAKE			BIT(1)
#define LSRIO_RAB_OBDME_CTRL_BFT_SHIFT		4
#define LSRIO_RAB_OBDME_CTRL_BFT_MASK		GENMASK(5, 4)
#define LSRIO_RAB_OBDME_CTRL_DIADS_SHIFT	24
#define LSRIO_RAB_OBDME_CTRL_DIADS_MASK		GENMASK(27, 24)
#define LSRIO_RAB_OBDME_CTRL_DIAWS_SHIFT	28
#define LSRIO_RAB_OBDME_CTRL_DIAWS_MASK		GENMASK(29, 28)
#define LSRIO_RAB_OBDME_CTRL_DCADH_SHIFT	30
#define LSRIO_RAB_OBDME_CTRL_DCADH_MASK		GENMASK(31, 30)

#define LSRIO_RAB_OBDME_STAT_DCTC		BIT(0)
#define LSRIO_RAB_OBDME_STAT_DTC		BIT(1)
#define LSRIO_RAB_OBDME_STAT_DFE		BIT(2)
#define LSRIO_RAB_OBDME_STAT_DE			BIT(3)
#define LSRIO_RAB_OBDME_STAT_DUE		BIT(4)
#define LSRIO_RAB_OBDME_STAT_DTE		BIT(5)
#define LSRIO_RAB_OBDME_STAT_MRE		BIT(6)
#define LSRIO_RAB_OBDME_STAT_MRT		BIT(7)
#define LSRIO_RAB_OBDME_STAT_TP			BIT(8)
#define LSRIO_RAB_OBDME_STAT_SLP		BIT(9)
#define LSRIO_RAB_OBDME_STAT_AXIE		GENMASK(5, 2)
#define LSRIO_RAB_OBDME_STAT_CLA		GENMASK(7, 0)

#define LSRIO_RAB_IBDME_BASE			0x20600
#define LSRIO_RAB_IBDME_BAR(n)			(LSRIO_RAB_IBDME_BASE + n * 0x10)
#define LSRIO_RAB_IBDME_CTRL(n)			(LSRIO_RAB_IBDME_BAR(n) + 0x0)
#define LSRIO_RAB_IBDME_DADDR(n)		(LSRIO_RAB_IBDME_BAR(n) + 0x4)
#define LSRIO_RAB_IBDME_STAT(n)			(LSRIO_RAB_IBDME_BAR(n) + 0x8)
#define LSRIO_RAB_IBDME_DESC(n)			(LSRIO_RAB_IBDME_BAR(n) + 0xc)

#define LSRIO_RAB_IBDME_CTRL_EN				BIT(0)
#define LSRIO_RAB_IBDME_CTRL_WAKE			BIT(1)
#define LSRIO_RAB_IBDME_CTRL_LETER_SHIFT	4
#define LSRIO_RAB_IBDME_CTRL_LETER_MASK		GENMASK(5, 4)
#define LSRIO_RAB_IBDME_CTRL_MBOX_SHIFT		6
#define LSRIO_RAB_IBDME_CTRL_MBOX_MASK		GENMASK(7, 6)
#define LSRIO_RAB_IBDME_CTRL_XMBOX_SHIFT	8
#define LSRIO_RAB_IBDME_CTRL_XMBOX_MASK		GENMASK(11, 8)
#define LSRIO_RAB_IBDME_CTRL_DIAWS_SHIFT	28
#define LSRIO_RAB_IBDME_CTRL_DIAWS_MASK		GENMASK(29, 28)
#define LSRIO_RAB_IBDME_CTRL_DCADH_SHIFT	30
#define LSRIO_RAB_IBDME_CTRL_DCADH_MASK		GENMASK(31, 30)

#define LSRIO_RAB_IBDME_STAT_DCTC		BIT(0)
#define LSRIO_RAB_IBDME_STAT_DTC		BIT(1)
#define LSRIO_RAB_IBDME_STAT_DFE		BIT(2)
#define LSRIO_RAB_IBDME_STAT_DE			BIT(3)
#define LSRIO_RAB_IBDME_STAT_DUE		BIT(4)
#define LSRIO_RAB_IBDME_STAT_DTE		BIT(5)
#define LSRIO_RAB_IBDME_STAT_MRE		BIT(6)
#define LSRIO_RAB_IBDME_STAT_MRT		BIT(7)
#define LSRIO_RAB_IBDME_STAT_TP			BIT(8)
#define LSRIO_RAB_IBDME_STAT_SLP		BIT(9)
#define LSRIO_RAB_IBDME_STAT_AXIE		GENMASK(5, 2)
#define LSRIO_RAB_IBDME_STAT_CLA		GENMASK(7, 0)

#define LSRIO_RAB_OBDME_TIDMSK			0x205f0

#define LSRIO_RAB_DMA_CTRL_OFF			0x0
#define LSRIO_RAB_DMA_ADDR_OFF			0x4
#define LSRIO_RAB_DMA_ADDR_EXT_OFF		0x8
#define LSRIO_RAB_DMA_STAT_OFF			0xc

#define LSRIO_RAB_DMA_ADDR_MASK			GENMASK(31, 1)

#define LSRIO_RAB_DMA_CTRL_ST			BIT(0)
#define LSRIO_RAB_DMA_CTRL_SUS			BIT(1)
#define LSRIO_RAB_DMA_CTRL_LDA			BIT(2)
#define LSRIO_RAB_DMA_CTRL_DID_SHITF	16

#define LSRIO_RAB_DMA_STAT_CTC			BIT(0)
#define LSRIO_RAB_DMA_STAT_DTC			BIT(1)
#define LSRIO_RAB_DMA_STAT_DFE			BIT(2)
#define LSRIO_RAB_DMA_STAT_DCE			BIT(3)
#define LSRIO_RAB_DMA_STAT_AXIE			BIT(4)
#define LSRIO_RAB_DMA_STAT_RIOE			BIT(5)
#define LSRIO_RAB_DMA_STAT_BUSY			BIT(8)
#define LSRIO_RAB_DMA_STAT_ERR			GENMASK(5, 2)
#define LSRIO_RAB_DMA_STAT_CLA			GENMASK(5, 0)

#define LSRIO_RAB_WDMA_BASE				0x20800
#define LSRIO_RAB_WDMA_BAR(n)			(LSRIO_RAB_WDMA_BASE + n * 0x10)
#define LSRIO_RAB_WDMA_CTRL(n)			(LSRIO_RAB_WDMA_BAR(n) + 0x0)
#define LSRIO_RAB_WDMA_ADDR(n)			(LSRIO_RAB_WDMA_BAR(n) + 0x4)
#define LSRIO_RAB_WDMA_STAT(n)			(LSRIO_RAB_WDMA_BAR(n) + 0x8)
#define LSRIO_RAB_WDMA_ADDR_EXT(n)		(LSRIO_RAB_WDMA_BAR(n) + 0xc)

#define LSRIO_RAB_RDMA_BASE				0x20880
#define LSRIO_RAB_RDMA_BAR(n)			(LSRIO_RAB_RDMA_BASE + n * 0x10)
#define LSRIO_RAB_RDMA_CTRL(n)			(LSRIO_RAB_RDMA_BAR(n) + 0x0)
#define LSRIO_RAB_RDMA_ADDR(n)			(LSRIO_RAB_RDMA_BAR(n) + 0x4)
#define LSRIO_RAB_RDMA_STAT(n)			(LSRIO_RAB_RDMA_BAR(n) + 0x8)
#define LSRIO_RAB_RDMA_ADDR_EXT(n)		(LSRIO_RAB_RDMA_BAR(n) + 0xc)


#define LSRIO_RAB_AMST_CACHE			0x23480
#define LSRIO_RAB_AMST_CACHE_EN			0xf

/*
 * Messaging definitions
 */
#define LSRIO_MMSG_BUFSIZE		RIO_MAX_MSG_SIZE
#define LSRIO_SMSG_BUFSIZE		0x100
#define LSRIO_MSG_BUFFER_SIZE	RIO_MAX_MSG_SIZE
#define LSRIO_MSG_MAX_SIZE		RIO_MAX_MSG_SIZE
#define LSRIO_IMSG_MAXCH		8
#define LSRIO_IMSG_CHNUM		LSRIO_IMSG_MAXCH
#define LSRIO_IMSGD_MIN_RING_SIZE	32
#define LSRIO_IMSGD_RING_SIZE		512

#define LSRIO_OMSG_CHNUM			4 /* One channel per MBOX */
#define LSRIO_OMSGD_MIN_RING_SIZE	0
#define LSRIO_OMSGD_RING_SIZE		512

enum lsrio_amap_type {
	AMAP_MAINT,
	AMAP_NW,
	AMAP_NW_R,
	AMAP_SW,
};

/*
 * Block DMA Descriptors
 */

struct lsrio_dma_desc {
	__le32 desc_ctrl;

#define LSRIO_DMAD_VALID		BIT(0)
#define LSRIO_DMAD_NDVAL		BIT(1)
#define LSRIO_DMAD_WRCNT_SHIFT	5
#define LSRIO_DMAD_WRCNT_MASK	GENMASK(22, 5)
#define LSRIO_DMAD_DONE			BIT(24)
#define LSRIO_DMAD_AXIE			BIT(25)
#define LSRIO_DMAD_RIOE			BIT(26)
#define LSRIO_DMAD_FETE			BIT(27)
#define LSRIO_DMAD_UPDE			BIT(28)

	__le32 src_addr;

	__le32 dst_addr;

	__le32 nxtd_addr;
} __aligned(16);

/*
 * Outbound Messaging Descriptor
 */
struct lsrio_msg_desc {
	__le32 desc_ctrl;

#define LSRIO_DME_DES_VALID			BIT(0)
#define LSRIO_DME_DES_NDPV			BIT(1)
#define LSRIO_DME_DES_EOC			BIT(2)
#define LSRIO_DME_DES_INTEN			BIT(3)
#define LSRIO_DME_DES_DONE			BIT(4)
#define LSRIO_DME_DES_TOUT			BIT(5)
#define LSRIO_DME_DES_AXIE			BIT(6)
#define LSRIO_DME_DES_RIOE			BIT(7)
#define LSRIO_IBDME_DES_BSIZE_SHIFT	4
#define LSRIO_IBDME_DES_BSIZE_MASK	GENMASK(5, 4)
#define LSRIO_IBDME_DES_BSIZE_4K	0x30
#define LSRIO_DME_DES_HANDLE_MASK	GENMASK(11, 8)
#define LSRIO_DME_DES_DSTID_SHIFT	16
#define LSRIO_DME_DES_DSTID_MASK	GENMASK(31, 16)

	__le32 msg_info;

#define LSRIO_DME_DES_LETER_SHIFT	0
#define LSRIO_DME_DES_LETER_MASK	GENMASK(1, 0)
#define LSRIO_DME_DES_MBOX_SHIFT	2
#define LSRIO_DME_DES_MBOX_MASK		GENMASK(3, 2)
#define LSRIO_DME_DES_XMBOX_SHIFT	4
#define LSRIO_DME_DES_XMBOX_MASK	GENMASK(7, 4)
#define LSRIO_DME_DES_MLEN_SHIFT	8
#define LSRIO_DME_DES_MLEN_MASK		GENMASK(17, 8)
#define LSRIO_OBDME_DES_SSIZE_SHIFT	18
#define LSRIO_OBDME_DES_SSIZE_MASK	GENMASK(20, 18)
#define LSRIO_OBDME_DES_SSIZE_256	0x180000
#define LSRIO_DME_DES_ADDRH_SHIFT	21
#define LSRIO_DME_DES_ADDRH_MASK	GENMASK(28, 21)
#define LSRIO_DME_DES_CRF			BIT(29)
#define LSRIO_DME_DES_PRIO_SHIFT	30
#define LSRIO_DME_DES_PRIO_MASK		GENMASK(31, 30)

	__le32 buf_addr;

#define LSRIO_DME_DES_ADDRL_SHIFT	0
#define LSRIO_DME_DES_ADDRL_MASK	GENMASK(29, 0)
#define LSRIO_DME_DES_NXTDH_SHIFT	30
#define LSRIO_DME_DES_NXTDH_MASK	GENMASK(31, 30)

	__le32 nxtd;

#define LSRIO_DME_NXTDL_SHIFT		0
#define LSRIO_DME_NXTDL_MASK		GENMASK(31, 0)
} __aligned(16);

/* Structures */

#ifdef CONFIG_RAPIDIO_DMA_ENGINE

#define LSRIO_DMA_MAX_BCOUNT	(GENMASK(17, 0) * 4)

struct lsrio_tx_desc {
	struct dma_async_tx_descriptor	txd;
	u16				destid;
	/* low 64-bits of 66-bit RIO address */
	u64				rio_addr;
	/* upper 2-bits of 66-bit RIO address */
	u8				rio_addr_u;
#define LSRIO_DMA_DIR_W		BIT(0)
#define LSRIO_DMA_DIR_R		BIT(1)
#define LSRIO_DMA_DIR_ALL	(LSRIO_DMA_DIR_W | LSRIO_DMA_DIR_R)
	u8				dir;
	struct list_head		desc_node;
	struct scatterlist		*sg;
	unsigned int			sg_len;
	enum dma_status			status;
};

struct lsrio_dma_chan {
	u32						id;
	int						tx_desc_num;
	struct lsrio_dma_desc	*tx_desc_base;
	dma_addr_t				tx_desc_phys;
	int						rx_desc_num;
	struct lsrio_dma_desc	*rx_desc_base;
	dma_addr_t				rx_desc_phys;

	struct dma_chan			dchan;
	spinlock_t				lock;
	bool					active;
	struct list_head		queue;
	struct list_head		free_list;
	struct lsrio_tx_desc	*active_tx;
	struct lsrio_tx_desc	*active_rx;
	struct tasklet_struct	tasklet;
};

#endif /* CONFIG_RAPIDIO_DMA_ENGINE */

struct lsrio_imsg_ring {
	u32		size;
	/* VA/PA of data buffers for incoming messages */
	void		*buf_base;
	dma_addr_t	buf_phys;
	/* VA/PA of Inbound message descriptors */
	struct lsrio_msg_desc *imd_base;
	dma_addr_t	imd_phys;
	 /* Inbound Queue buffer pointers */
	void		*imq_base[LSRIO_IMSGD_RING_SIZE];

	u32		rx_slot;
	void		*dev_id;
	u32		r_ptr;
	bool	sleeping;
	spinlock_t	lock;
};

struct lsrio_omsg_ring {
	u32		size;
	/* VA/PA of OB Msg descriptors */
	struct lsrio_msg_desc	*omd_base;
	dma_addr_t	omd_phys;
	/* VA/PA of OB Msg data buffers */
	void		*omq_base[LSRIO_OMSGD_RING_SIZE];
	dma_addr_t	omq_phys[LSRIO_OMSGD_RING_SIZE];


	u32		w_ptr;
	u32		r_ptr;
	void		*dev_id;
	spinlock_t	lock;
};

struct lsrio_ib_win {
	u64		rio_base;
	u64		axi_base;
	u64		ref;
	u32		size;
	bool	active;
};

struct lsrio_axi_res {
	u64		base;
	u64		size;
	u64		free;
};

struct lsrio_ob_win {
	u64		base;
	u32		size;
	u16		type;
	u16		destid;
	u64		rstart;
	u64		axi_base;
	bool	active;
};

struct lsrio_device {
	struct pci_dev	*pdev;
	struct rio_mport mport;
	u32		flags;

	void __iomem	*axi_base;
	void __iomem	*apb_base;
	struct lsrio_axi_res axi_res;

	spinlock_t	apb_lock;
	void		*idb_base;
	dma_addr_t	idb_dma;
	struct work_struct idb_work;
	struct completion odb_complete;

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
#define LSRIO_DMA_MAXCH			8
#define LSRIO_DMA_CHNUM			LSRIO_DMA_MAXCH
	struct lsrio_dma_chan dma[LSRIO_DMA_CHNUM];
	int dma_num;
	struct dma_pool *dma_desc_pool;
#endif

	/* Inbound Messaging */
	int		imsg_init[LSRIO_IMSG_CHNUM];
	struct lsrio_imsg_ring imsg_ring[LSRIO_IMSG_CHNUM];

	/* Outbound Messaging */
	int		omsg_init[LSRIO_OMSG_CHNUM];
	struct lsrio_omsg_ring	omsg_ring[LSRIO_OMSG_CHNUM];

	/* Inbound Mapping Windows */
#define LSRIO_IBWIN_NUM	16
#define LSRIO_IBWIN_RADDR_GAP_DEFAULT	0x40000000
	u32		ibwin_stat;
	struct lsrio_ib_win ib_win[LSRIO_IBWIN_NUM];

	/* Outbound Mapping Windows */
#define LSRIO_OBWIN_NUM	16
#define	LSRIO_MAINT_WIN_SIZE	0x100000
	struct lsrio_ob_win  ob_win[LSRIO_OBWIN_NUM];
	int		obwin_cnt;
};

#ifdef CONFIG_RAPIDIO_DMA_ENGINE
extern void lsrio_dma_handler(struct lsrio_dma_chan *dma_chan);
extern int lsrio_register_dma(struct lsrio_device *priv);
extern void lsrio_unregister_dma(struct lsrio_device *priv);
extern void lsrio_dma_stop_all(struct lsrio_device *priv);
#else
#define lsrio_dma_stop_all(priv) do {} while (0)
#define lsrio_unregister_dma(priv) do {} while (0)
#endif

void lsrio_set_apb_csr(struct lsrio_device *priv, u32 offset);
void lsrio_apb_writel(struct lsrio_device *priv, u32 offset, u32 value);
u32 lsrio_apb_readl(struct lsrio_device *priv, u32 offset);
#endif
