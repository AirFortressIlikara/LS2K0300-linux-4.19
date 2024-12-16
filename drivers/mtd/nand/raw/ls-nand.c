// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <asm/dma.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/mtd/nand_bch.h>

#define DRIVER_NAME	"ls-nand"
#define ALIGN_DMA(x)	((x + 3)/4)
#define REG(reg)	(info->mmio_base + reg)

#define MAX_BUFF_SIZE		0x10000
#define STATUS_TIME_LOOP_R	300
#define STATUS_TIME_LOOP_WS	100
#define STATUS_TIME_LOOP_WM	60
#define STATUS_TIME_LOOP_E	10000

/* Register offset */
#define NAND_CMD_REG		0x00
#define NAND_ADDRC_REG		0x04
#define NAND_ADDRR_REG		0x08
#define NAND_TIM_REG		0x0c
#define NAND_IDL_REG		0x10
#define NAND_IDH_REG		0x14
#define NAND_STA_REG		0x14
#define NAND_PARAM_REG		0x18
#define NAND_OP_NUM_REG		0x1c
#define NAND_CS_RDY_REG		0x20
#define NAND_DMA_ADDR_REG	0x40

#define DMA_OFFSET		0x100

/* NAND_CMD_REG */
#define CMD_VALID		(1 << 0)	/* command valid */
#define CMD_RD_OP		(1 << 1)	/* read operation */
#define CMD_WR_OP		(1 << 2)	/* write operation */
#define CMD_ER_OP		(1 << 3)	/* erase operation */
#define CMD_BER_OP		(1 << 4)	/* blocks erase operation */
#define CMD_RD_ID		(1 << 5)	/* read id */
#define CMD_RESET		(1 << 6)	/* reset */
#define CMD_RD_STATUS		(1 << 7)	/* read status */
#define CMD_MAIN		(1 << 8)	/* operation in main region */
#define CMD_SPARE		(1 << 9)	/* operation in spare region */
#define CMD_DONE		(1 << 10)	/* operation done */
#define CMD_RS_RD		(1 << 11)	/* ecc is enable when reading */
#define CMD_RS_WR		(1 << 12)	/* ecc is enable when writing */
#define CMD_INT_EN		(1 << 13)	/* interrupt enable */
#define CMD_WAIT_RS		(1 << 14)	/* waiting ecc read done */
#define CMD_ECC_DMA_REQ		(1 << 30)	/* dma request in ecc mode */
#define CMD_DMA_REQ		(1 << 31)	/* dma request in normal mode */
#define CMD_RDY_SHIF		16		/* four bits for chip ready */
#define CMD_CE_SHIF		20		/* four bits for chip enable */

/* NAND_PARAM_REG */
#define CHIP_CAP_SHIFT		8
#define ID_NUM_SHIFT		12
#define OP_SCOPE_SHIFT		16
/* DMA COMMAND REG */
#define DMA_INT_MASK		(1 << 0)	/* dma interrupt mask */
#define DMA_INT			(1 << 1)	/* dma interrupt */
#define DMA_SIN_TR_OVER		(1 << 2)	/* a single dma transfer over */
#define DMA_TR_OVER		(1 << 3)	/* all dma transfer over */
#define DMA_RD_WR		(1 << 12)	/* dma operation type */
#define DMA_RD_STU_SHIF		4		/* dma read data status */
#define DMA_WR_STU_SHIF		8		/* dma write data status */

int parse_mtd_partitions(struct mtd_info *master, const char *const *types,
			 struct mtd_partition **pparts,
			 struct mtd_part_parser_data *data);
/* DMA Descripter */
struct ls_nand_dma_desc {
	uint32_t orderad;
	uint32_t saddr;
	uint32_t daddr;
	uint32_t length;
	uint32_t step_length;
	uint32_t step_times;
	uint32_t cmd;
	uint32_t dummy;
	uint32_t order_addr_hi;
	uint32_t saddr_hi;
};

struct ls_nand_plat_data {
	int	enable_arbiter;
	u32	nr_parts;
	u32	chip_ver;
	struct	mtd_partition *parts;
	int	cs;
	u32	csrdy;
	int	chip_cap;
};

struct ls_nand_info {
	struct nand_chip	nand_chip;
	struct platform_device	*pdev;
	spinlock_t		nand_lock;

	/* MTD data control */
	unsigned int		buf_start;
	unsigned int		buf_count;

	void __iomem		*mmio_base;
	unsigned int		irq;
	unsigned int		cmd;

	/* DMA information */
	u64			dma_order_reg;	/* dma controller register */
	unsigned int		apb_data_addr;	/* dma access this address */
	u64			desc_addr;	/* dma descriptor address */
	dma_addr_t		desc_addr_phys;
	size_t			desc_size;
	u64			dma_ask;
	dma_addr_t		dma_ask_phy;

	unsigned char		*data_buff;	/* dma data buffer */
	dma_addr_t		data_buff_phys;
	size_t			data_buff_size;

	struct timer_list	test_timer;

	size_t			data_size;	/* data size in FIFO */
	struct completion	cmd_complete;
	unsigned int		seqin_column;
	unsigned int		seqin_page_addr;
	u32			chip_version;
	int			cs, cs0;
	u32			csrdy;
	int			chip_cap;
};
static unsigned int bch = 4;
module_param(bch,	     uint, 0400);
MODULE_PARM_DESC(bch,		 "Enable BCH ecc and set how many bits should "
				 "be correctable in 512-byte blocks");

static void wait_nand_done(struct ls_nand_info *info, int timeout);
static int ls_nand_init_buff(struct ls_nand_info *info)
{
	struct platform_device *pdev = info->pdev;

	info->data_buff = dma_alloc_coherent(&pdev->dev, MAX_BUFF_SIZE,
					     &info->data_buff_phys, GFP_KERNEL);
	if (info->data_buff == NULL) {
		dev_err(&pdev->dev, "failed to allocate dma buffer\n");
		return -ENOMEM;
	}

	info->data_buff_size = MAX_BUFF_SIZE;
	return 0;
}

static int ls_nand_ecc_calculate(struct mtd_info *mtd,
				const uint8_t *dat, uint8_t *ecc_code)
{
	return 0;
}

static int ls_nand_ecc_correct(struct mtd_info *mtd,
				uint8_t *dat, uint8_t *read_ecc,
				uint8_t *calc_ecc)
{
	/*
	 * Any error include ERR_SEND_CMD, ERR_DBERR, ERR_BUSERR, we
	 * consider it as a ecc error which will tell the caller the
	 * read fail We have distinguish all the errors, but the
	 * nand_read_ecc only check this function return value
	 */
	return 0;
}

static void ls_nand_ecc_hwctl(struct mtd_info *mtd, int mode)
{

}

static int ls_nand_get_ready(struct mtd_info *mtd)
{
	struct ls_nand_info *info = mtd->priv;
	unsigned char status;

	writel(CMD_RD_STATUS | CMD_VALID, REG(NAND_CMD_REG));
	wait_nand_done(info, STATUS_TIME_LOOP_R);
	status = readl(REG(NAND_IDH_REG))>>16;
	return status;
}

static int ls_nand_waitfunc(struct mtd_info *mtd, struct nand_chip *this)
{
	struct ls_nand_info *info = mtd->priv;
	unsigned char status;

	status = readl(REG(NAND_IDH_REG))>>16;
	return status;
}

static void ls_nand_select_chip(struct mtd_info *mtd, int chip)
{
	struct ls_nand_info *info = mtd->priv;

	info->cs = (chip == -1) ? info->cs0 : info->cs0 + chip;
}

static int ls_nand_dev_ready(struct mtd_info *mtd)
{
	struct ls_nand_info *info = mtd->priv;

	return	!!(readl(REG(NAND_CMD_REG)) & (1<<(info->cs+16)));
}

static const char cap2cs[16] = {[0] = 16, [1]  = 17, [2]  = 18, [3] = 19,
				[4] = 19, [5]  = 19, [6]  = 20, [7] = 21,
				[9] = 14, [10] = 15, [11] = 16, [12] = 17,
				[13] = 18};

static void nand_setup(struct ls_nand_info *info,
		int cmd, int addr_c, int addr_r, int param, int op_num)
{
	unsigned int addr_cs = info->cs*(1UL<<cap2cs[(param>>CHIP_CAP_SHIFT)&0xf]);

	writel(param, REG(NAND_PARAM_REG));
	writel(op_num, REG(NAND_OP_NUM_REG));
	writel(addr_c, REG(NAND_ADDRC_REG));
	writel(addr_r|addr_cs, REG(NAND_ADDRR_REG));
	writel(0, REG(NAND_CMD_REG));
	writel(0, REG(NAND_CMD_REG));
	writel(cmd, REG(NAND_CMD_REG));
}

static void wait_nand_done(struct ls_nand_info *info, int timeout)
{
	int t, status_times = timeout;

	do {
		t = readl(REG(NAND_CMD_REG)) & CMD_DONE;
		if (!(status_times--)) {
			writel(0x0, REG(NAND_CMD_REG));
			writel(0x0, REG(NAND_CMD_REG));
			writel(CMD_RESET | CMD_VALID, REG(NAND_CMD_REG));
			break;
		}
		udelay(50);
	} while (t == 0);

	writel(0x0, REG(NAND_CMD_REG));
}

void dma_desc_init(struct ls_nand_info *info)
{
	volatile struct ls_nand_dma_desc *dma_base =
		(volatile struct ls_nand_dma_desc *)(info->desc_addr);

	dma_base->orderad = 0;
	dma_base->saddr = info->data_buff_phys;
	dma_base->daddr = info->apb_data_addr;
	dma_base->step_length = 0;
	dma_base->step_times = 0x1;
	dma_base->length = 0;
	dma_base->cmd = 0;
	dma_base->order_addr_hi = 0;
	dma_base->saddr_hi = ((info->data_buff_phys) >> 32);

}

static void dma_setup(struct ls_nand_info *info, int dma_cmd, int dma_cnt)
{
	u64 dma_order, t;
	volatile struct ls_nand_dma_desc *dma_base =
		(volatile struct ls_nand_dma_desc *)(info->desc_addr);
	dma_base->orderad = 0;
	dma_base->saddr = info->data_buff_phys;
	dma_base->saddr_hi = ((info->data_buff_phys) >> 32);
	dma_base->daddr = info->apb_data_addr;
	dma_base->step_length = 0;
	dma_base->step_times = 0x1;
	dma_base->length = dma_cnt;
	dma_base->cmd = dma_cmd;

	t = ((info->desc_addr_phys) & ~0x1fUL) | (0x1UL << 3);
	if (info->pdev->dev.coherent_dma_mask == DMA_BIT_MASK(64))
		t |= 0x1UL;
	else
		t &= ~0x1UL;

	dma_order = readl((void *)info->dma_order_reg);
	dma_order |= (readl((void *)(info->dma_order_reg + 4)) << 32);
	dma_order = (dma_order & 0xfUL) | t;

	writel((dma_order >> 32), (void *)(info->dma_order_reg + 4));
	writel(dma_order, (void *)info->dma_order_reg);

	t = STATUS_TIME_LOOP_R;
	while ((readl((void *)info->dma_order_reg) & 0x8) && t) {
		t--;
		udelay(50);
	};

	if (t == 0)
		pr_info("nand dma timeout!\n");

	wait_nand_done(info, STATUS_TIME_LOOP_R);
}

static int get_chip_capa_num(uint64_t  chipsize, int pagesize)
{
	int size_mb = chipsize >> 20;

	if (pagesize == 4096)
		return 4;
	else if (pagesize == 2048)
		switch (size_mb) {
		case (1 << 7):		/* 1Gb */
			return 0;
		case (1 << 8):		/* 2Gb */
			return 1;
		case (1 << 9):		/* 4Gb */
			return 2;
		case (1 << 10):		/* 8Gb */
		default:
			return 3;
		}
	else if (pagesize == 8192)

		switch (size_mb) {
		case (1 << 12):		/* 32Gb */
			return 5;
		case (1 << 13):		/* 64Gb */
			return 6;
		case (1 << 14):		/* 128Gb */
		default:
			return 7;
		}
	else if (pagesize == 512)

		switch (size_mb) {
		case (1 << 3):		/* 64Mb */
			return 9;
		case (1 << 4):		/* 128Mb */
			return 0xa;
		case (1 << 5):		/* 256Mb */
			return 0xb;
		case (1 << 6):		/* 512Mb */
		default:
			return 0xc;
		}
	else
		return 0;
}

static void ls_read_id(struct ls_nand_info *info)
{
	unsigned int id_l, id_h;
	unsigned char *data = (unsigned char *)(info->data_buff);
	unsigned int addr_cs, chip_cap;
	struct mtd_info *mtd = nand_to_mtd(&info->nand_chip);

	if (mtd->writesize) {
		chip_cap = get_chip_capa_num(info->nand_chip.chipsize, mtd->writesize);
		addr_cs = info->cs*(1UL<<cap2cs[chip_cap]);
		info->chip_cap  = chip_cap;
	} else  {
		addr_cs = info->cs*(1UL<<cap2cs[info->chip_cap]);
	}

	writel((6 << ID_NUM_SHIFT) | (info->chip_cap << CHIP_CAP_SHIFT), REG(NAND_PARAM_REG));
	writel(addr_cs, REG(NAND_ADDRR_REG));
	writel((CMD_RD_ID | CMD_VALID), REG(NAND_CMD_REG));
	wait_nand_done(info, 100);
	id_l = readl(REG(NAND_IDL_REG));
	id_h = readl(REG(NAND_IDH_REG));
	pr_debug("id_l: %08x, id_h:%08x\n", id_l, id_h);
	data[0] = ((id_h >> 8) & 0xff);
	data[1] = (id_h & 0xff);
	data[2] = (id_l >> 24) & 0xff;
	data[3] = (id_l >> 16) & 0xff;
	data[4] = (id_l >> 8) & 0xff;
	data[5] = id_l & 0xff;
	data[6] = 0;
	data[7] = 0;
}

static void ls_nand_cmdfunc(struct mtd_info *mtd, unsigned int command,
			      int column, int page_addr)
{
	struct ls_nand_info *info = mtd->priv;
	int chip_cap, oobsize, pagesize;
	int cmd, addrc, addrr, op_num, param;
	int dma_cmd, dma_cnt;
	unsigned long flags;

	info->cmd = command;
	oobsize = mtd->oobsize;
	pagesize = mtd->writesize;
	chip_cap = get_chip_capa_num(info->nand_chip.chipsize, pagesize);
	spin_lock_irqsave(&info->nand_lock, flags);
	switch (command) {
	case NAND_CMD_READOOB:
		info->buf_count = oobsize;
		if (info->buf_count <= 0)
			break;
		info->buf_start = 0;
		addrc = pagesize;
		addrr = page_addr;
		param = (oobsize << OP_SCOPE_SHIFT)
			| (chip_cap << CHIP_CAP_SHIFT);
		op_num = oobsize;
		cmd = CMD_VALID | CMD_SPARE | CMD_RD_OP;
		nand_setup(info, cmd, addrc, addrr, param, op_num);

		dma_cmd = DMA_INT_MASK;
		dma_cnt = ALIGN_DMA(op_num);
		dma_setup(info, dma_cmd, dma_cnt);
		break;
	case NAND_CMD_READ0:
		addrc = 0;
		addrr = page_addr;
		op_num = oobsize + pagesize;
		param = (op_num << OP_SCOPE_SHIFT) | (chip_cap << CHIP_CAP_SHIFT);
		cmd = CMD_VALID | CMD_SPARE | CMD_RD_OP | CMD_MAIN;
		info->buf_count = op_num;
		info->buf_start = column;
		nand_setup(info, cmd, addrc, addrr, param, op_num);

		dma_cmd = DMA_INT_MASK;
		dma_cnt = ALIGN_DMA(op_num);
		dma_setup(info, dma_cmd, dma_cnt);
		break;
	case NAND_CMD_SEQIN:
		info->buf_count = oobsize + pagesize - column;
		info->buf_start = 0;
		info->seqin_column = column;
		info->seqin_page_addr = page_addr;
		break;
	case NAND_CMD_PAGEPROG:
		addrc = info->seqin_column;
		addrr = info->seqin_page_addr;
		op_num = info->buf_start;
		param = ((pagesize + oobsize) << OP_SCOPE_SHIFT)
			| (chip_cap << CHIP_CAP_SHIFT);
		cmd = CMD_VALID | CMD_SPARE | CMD_WR_OP;
		if (addrc < pagesize)
			cmd |= CMD_MAIN;
		nand_setup(info, cmd, addrc, addrr, param, op_num);

		dma_cmd = DMA_INT_MASK | DMA_RD_WR;
		dma_cnt = ALIGN_DMA(op_num);
		dma_setup(info, dma_cmd, dma_cnt);
		break;
	case NAND_CMD_RESET:
		nand_setup(info, (CMD_RESET | CMD_VALID), 0, 0, 0, 0);
		wait_nand_done(info, STATUS_TIME_LOOP_R);
		break;
	case NAND_CMD_ERASE1:
		addrc = 0;
		addrr = page_addr;
		op_num = 1;
		param = ((pagesize + oobsize) << OP_SCOPE_SHIFT)
			| (chip_cap << CHIP_CAP_SHIFT);
		cmd = CMD_ER_OP | CMD_VALID;
		nand_setup(info, cmd, addrc, addrr, param, op_num);
		wait_nand_done(info, STATUS_TIME_LOOP_E);
		break;
	case NAND_CMD_STATUS:
		info->buf_count = 0x1;
		info->buf_start = 0x0;
		*(unsigned char *)info->data_buff =
			ls_nand_get_ready(mtd);
		break;
	case NAND_CMD_READID:
		info->buf_count = 0x6;
		info->buf_start = 0;
		ls_read_id(info);
		break;
	case NAND_CMD_ERASE2:
	case NAND_CMD_READ1:
		break;
	case NAND_CMD_RNDOUT:
		info->buf_start = column;
		break;
	default:
		printk(KERN_ERR "non-supported command.\n");
		break;
	}

	spin_unlock_irqrestore(&info->nand_lock, flags);
}

static u16 ls_nand_read_word(struct mtd_info *mtd)
{
	struct ls_nand_info *info = mtd->priv;
	unsigned long flags;
	u16 retval = 0xFFFF;

	spin_lock_irqsave(&info->nand_lock, flags);

	if (!(info->buf_start & 0x1) && info->buf_start < info->buf_count)
		retval = *(u16 *) (info->data_buff + info->buf_start);
	info->buf_start += 2;

	spin_unlock_irqrestore(&info->nand_lock, flags);

	return retval;
}

static uint8_t ls_nand_read_byte(struct mtd_info *mtd)
{
	struct ls_nand_info *info = mtd->priv;
	unsigned long flags;
	char retval = 0xFF;

	spin_lock_irqsave(&info->nand_lock, flags);

	if (info->buf_start < info->buf_count)
		retval = info->data_buff[(info->buf_start)++];

	spin_unlock_irqrestore(&info->nand_lock, flags);
	return retval;
}

static void ls_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct ls_nand_info *info = mtd->priv;
	int real_len = min_t(size_t, len, info->buf_count - info->buf_start);
	unsigned long flags;

	spin_lock_irqsave(&info->nand_lock, flags);

	memcpy(buf, info->data_buff + info->buf_start, real_len);

	info->buf_start += real_len;
	spin_unlock_irqrestore(&info->nand_lock, flags);
}

static void ls_nand_write_buf(struct mtd_info *mtd, const uint8_t *buf,
				int len)
{
	struct ls_nand_info *info = mtd->priv;
	int real_len = min_t(size_t, len, info->buf_count - info->buf_start);
	unsigned long flags;

	spin_lock_irqsave(&info->nand_lock, flags);

	memcpy(info->data_buff + info->buf_start, buf, real_len);
	info->buf_start += real_len;

	spin_unlock_irqrestore(&info->nand_lock, flags);
}

static void ls_nand_init_mtd(struct mtd_info *mtd,
			       struct ls_nand_info *info)
{
	struct nand_chip *this = &info->nand_chip;

	this->options		= 8;
	this->waitfunc		= ls_nand_waitfunc;
	this->select_chip	= ls_nand_select_chip;
	this->dev_ready		= ls_nand_dev_ready;
	this->cmdfunc		= ls_nand_cmdfunc;
	this->read_word		= ls_nand_read_word;
	this->read_byte		= ls_nand_read_byte;
	this->read_buf		= ls_nand_read_buf;
	this->write_buf		= ls_nand_write_buf;

	this->ecc.mode		= NAND_ECC_SOFT;
	this->ecc.algo		= NAND_ECC_BCH;
	this->ecc.hwctl		= ls_nand_ecc_hwctl;
	this->ecc.calculate	= ls_nand_ecc_calculate;
	this->ecc.correct	= ls_nand_ecc_correct;
	this->ecc.size		= 256;
	this->ecc.bytes		= 3;
	mtd->owner = THIS_MODULE;
}

static void test_handler(struct timer_list *t)
{
	u64 dma_order, val;
	struct ls_nand_info *info = from_timer(info, t, test_timer);

	mod_timer(&info->test_timer, jiffies + 1);
	val = (info->dma_ask_phy & ~0x1fUL) | 0x4;

	dma_order = readl((void *)info->dma_order_reg);
	dma_order |= (readl((void *)(info->dma_order_reg + 4)) << 32);
	dma_order = (dma_order & 0x1fUL) | val;

	writel((dma_order >> 32), (void *)(info->dma_order_reg + 4));
	writel(dma_order, (void *)info->dma_order_reg);
	udelay(1000);
}

static void ls_nand_init_info(struct ls_nand_info *info)
{
	info->buf_start = 0;
	info->buf_count = 0;
	info->seqin_column = 0;
	info->seqin_page_addr = 0;
	spin_lock_init(&info->nand_lock);
	writel(0x412, REG(NAND_TIM_REG));
	writel(info->csrdy, REG(NAND_CS_RDY_REG));

	info->test_timer.expires = jiffies + 10;
	timer_setup(&info->test_timer, test_handler, 0);
}

static int ls_nand_probe(struct platform_device *pdev)
{
	struct ls_nand_plat_data *pdata;
	struct ls_nand_info *info;
	struct nand_chip *this;
	struct mtd_info *mtd;
	struct resource *r;
	int ret = 0, irq;
	struct dma_chan *chan;
	int data;
#ifdef CONFIG_MTD_CMDLINE_PARTS
	const char *part_probes[] = { "cmdlinepart", NULL };
#endif

#ifdef CONFIG_MTD_CMDLINE_PARTS
	struct mtd_partition *partitions = NULL;
	int num_partitions = 0;
#endif

	__be32 *of_property = NULL;

	if (pdev->dev.of_node) {
		of_property = (__be32 *)of_get_property(pdev->dev.of_node, "dma-mask", NULL);
		if (of_property != 0) {
			pdev->dev.coherent_dma_mask = of_read_number(of_property, 2);
			if (pdev->dev.dma_mask)
				*(pdev->dev.dma_mask) = pdev->dev.coherent_dma_mask;
			else
				pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
		}
	}

	pdata = devm_kzalloc(&pdev->dev, sizeof(struct ls_nand_plat_data), GFP_KERNEL);

	if (!pdata) {
		dev_err(&pdev->dev, "fialed to allocate memory\n");
		return -ENODEV;
	}

	pdata->cs = 2;
	pdata->csrdy = 0x88442200;
	pdata->enable_arbiter = 1;

	if (pdev->dev.of_node) {
		if (!of_property_read_u32(pdev->dev.of_node, "nand-cs", &data))
			pdata->cs = data;

		if (!of_property_read_u32(pdev->dev.of_node, "chip_cap", &data))
			pdata->chip_cap  = data;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(struct ls_nand_info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "fialed to allocate memory\n");
		return -ENOMEM;
	}

	info->pdev = pdev;
	info->cs0 = info->cs = pdata->cs;
	info->csrdy = pdata->csrdy;
	info->chip_cap = pdata->chip_cap;

	this = &info->nand_chip;
	mtd = nand_to_mtd(&info->nand_chip);
	mtd->priv = info;

	info->desc_addr = (u64) dma_alloc_coherent(&pdev->dev,
			MAX_BUFF_SIZE, &info->desc_addr_phys, GFP_KERNEL);
	info->dma_ask = (u64) dma_alloc_coherent(&pdev->dev,
			MAX_BUFF_SIZE, &info->dma_ask_phy, GFP_KERNEL);

	if (!info->desc_addr) {
		dev_err(&pdev->dev, "fialed to allocate memory\n");
		ret = -ENOMEM;
		goto fail_free_mtd;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (r == NULL) {
		dev_err(&pdev->dev, "no IO memory resource defined\n");
		ret = -ENODEV;
		goto fail_free_buf;
	}

	r = request_mem_region(r->start, r->end - r->start + 1, pdev->name);
	if (r == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto fail_free_buf;
	}

	info->mmio_base = ioremap(r->start, r->end - r->start + 1);
	if (info->mmio_base == NULL) {
		dev_err(&pdev->dev, "ioremap() failed\n");
		ret = -ENODEV;
		goto fail_free_res;
	}

	chan = of_dma_request_slave_channel(pdev->dev.of_node, "nand_rw");
	if (IS_ERR(chan)) {
		pr_debug("use exclusive nand dma engine.\n");
		info->dma_order_reg = (u64)ioremap((r->start + DMA_OFFSET), 8);
	} else {
		if (chan == NULL) {
			dev_err(&pdev->dev, "no nand APBDMA resource defined\n");
			return -ENODEV;
		}

		ret = of_property_read_u32(pdev->dev.of_node, "#address-cells", &data);
		if (ret) {
			dev_err(&pdev->dev, "missing #address-cells property\n");
			data = 1;
		}

		if (data == 2) {
			of_property = (__be32 *)of_get_property(chan->device->dev->of_node, "reg", NULL);
			if (of_property != 0)
				r->start = of_read_number(of_property, 2);
		} else {
			of_property_read_u32(chan->device->dev->of_node, "reg", &data);
			r->start = data;
		}

		info->dma_order_reg = (u64)ioremap(r->start, 8);
	}

	pr_debug("info->dma_order_reg = %llx\n", info->dma_order_reg);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no DMA access address\n");
		ret = -ENODEV;
		goto fail_free_res;
	}
	info->apb_data_addr = r->start;
	pr_debug("info->apb_data_addr= %x\n", info->apb_data_addr);

	ret = ls_nand_init_buff(info);
	if (ret)
		goto fail_free_io;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no IRQ resource defined\n");
		ret = -ENXIO;
		goto fail_free_io;
	}
	info->irq = irq;

	ls_nand_init_mtd(mtd, info);
	if (pdev->dev.of_node) {
		const char *pm;
		struct nand_chip *this = &info->nand_chip;

		if (!of_property_read_string(pdev->dev.of_node, "nand-ecc-algo", &pm)) {
			if (!strcmp(pm, "none")) {
				bch = 0;
				this->ecc.mode		= NAND_ECC_NONE;
			} else if (!strcmp(pm, "bch")) {
				if (!of_property_read_u32(pdev->dev.of_node, "nand-ecc-strength", &data))
					bch = data;
			} else
				bch = 0;
		}
	}

	ls_nand_init_info(info);
	dma_desc_init(info);
	platform_set_drvdata(pdev, mtd);

	if (bch) {
		this->ecc.mode = NAND_ECC_SOFT;
		this->ecc.algo = NAND_ECC_BCH;
		this->ecc.size = 512;
		this->ecc.strength = bch;
		pr_info("using %u-bit/%u bytes BCH ECC\n", bch, this->ecc.size);
	}

	if (nand_scan(this, 4)) {
		dev_err(&pdev->dev, "failed to scan nand\n");
		ret = -ENXIO;
		goto fail_free_io;
	}

#ifdef CONFIG_MTD_CMDLINE_PARTS
	mtd->name = "nand-flash";
	num_partitions = parse_mtd_partitions(mtd, part_probes, &partitions, 0);
#endif
	mtd->dev.of_node = pdev->dev.of_node;
	ret = mtd_device_parse_register(mtd, NULL, NULL, NULL, 0);

	return ret;

fail_free_io:
	iounmap(info->mmio_base);
fail_free_res:
	release_mem_region(r->start, r->end - r->start + 1);
fail_free_buf:
	dma_free_coherent(&pdev->dev, info->data_buff_size,
			  info->data_buff, info->data_buff_phys);
fail_free_mtd:
	return ret;
}

static int ls_nand_remove(struct platform_device *pdev)
{
	struct mtd_info *mtd = platform_get_drvdata(pdev);
	struct ls_nand_info *info = mtd->priv;

	platform_set_drvdata(pdev, NULL);

	mtd_device_unregister(mtd);
	kfree((void *)info->desc_addr);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id ls_nand_dt_match[] = {
	{ .compatible = "loongson,ls-nand", },
	{},
};
MODULE_DEVICE_TABLE(of, ls_nand_dt_match);
#endif

static struct platform_driver ls_nand_driver = {
	.probe		= ls_nand_probe,
	.remove		= ls_nand_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
	.of_match_table = of_match_ptr(ls_nand_dt_match),
#endif
	},
};

static int __init ls_nand_init(void)
{
	pr_info("%s driver initializing\n", DRIVER_NAME);
	return platform_driver_register(&ls_nand_driver);
}

static void __exit ls_nand_exit(void)
{
	platform_driver_unregister(&ls_nand_driver);
}

module_init(ls_nand_init);
module_exit(ls_nand_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Loongson NAND controller driver");
