/*
 * Driver for loongson quadspi controller
 *
 * Copyright (C) 2024
 * Author(s):  <zhaorui@loongson.cn>.
 *
 * License terms: GPL V2.0.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * This program. If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/bsearch.h>

#define QSPI_CR				    0x0
#define QSPI_CSR			    0x1
#define QSPI_DR				    0x2
#define QSPI_SR				    0x3
#define QSPI_RSSR		            0xf

/* QSPI CONFIG register*/
#define	QSPI_CR_DUXEN                       BIT(2)
#define QSPI_CR_QTYPE_SHIFET		    3
#define QSPI_CR_QTYPE_MASK		    GENMASK(4,3)
#define QSPI_CR_DUMMY			    BIT(5)
#define QSPI_CR_QMODE_SHIFET		    6
#define QSPI_CR_QMODE_MASK                  GENMASK(7, 6)
#define QSPI_CR_QMODE_1                     (0U << 3)
#define QSPI_CR_QMODE_2                     (1U << 3)
#define QSPI_CR_QMODE_4                     (2U << 3)

/* QSPI CONFIG &STATUS register*/
#define QSPI_CSR_QBUSY			    BIT(2)
#define QSPI_CSR_QCS                        BIT(3)
#define QSPI_CSR_CLK_DIV_SHIFET             4
#define QSPI_CSR_CLK_DIV_MASK               GENMASK(7, 4)

/* QSPI SWITCH register*/
#define QSPI_SR_SWITCH                      BIT(0)

/* QSPI REGISERT SPACE SWITCH register*/
#define QSPI_RSSR_SPACE                     BIT(0)

#define FMODE_INDW		    0U
#define FMODE_INDR		    1U
#define QSPI_BUSY_TIMEOUT_US        100000
#define LS_MAX_NORCHIP              1

struct ls_qspi_flash {
        struct spi_nor nor;
        struct ls_qspi *qspi;
        u32 cs;
        u32 presc;
        bool registered;
};

struct ls_qspi {
        struct device *dev;
        void __iomem *io_base;
        u32 nor_num;
        struct clk *clk;
        u32 clk_rate;
        struct ls_qspi_flash flash[LS_MAX_NORCHIP];
        struct completion cmd_completion;

        /*
         * to protect device configuration, could be different between
         * 2 flash access (bk1, bk2)
         */
        struct mutex lock;
};

struct ls_qspi_cmd {
        u8 addr_width;
        u8 dummy;
        bool tx_data;
        u8 opcode;
        u32 framemode;
        u32 qspimode;
        u32 addr;
        size_t len;
        void *buf;
};

static struct spretoregs_t
{
        unsigned int spre;
        unsigned short div;
} spretoregs[] = {
	{2,0x0},
	{4,0x1},
	{16,0x2},
	{32,0x3},
	{8,0x4},
	{64,0x5},
	{128,0x6},
	{256,0x7},
	{512,0x8},
	{1024,0x9},
	{2048,0xa},
	{4096,0xb},
};

static  inline void qspi_writeb(struct ls_qspi *qspi,unsigned short offset, u8  value)
{
        writeb(value,qspi->io_base+offset);
}

static inline u32 qspi_readb(struct ls_qspi *qspi, unsigned short offset)
{
        return readb(qspi->io_base+offset);
}

static inline void qspi_set_bit(struct ls_qspi *qspi, unsigned short offset,u8 value)
{
        u8 tmp;

        tmp =qspi_readb(qspi,offset);
        tmp |= value;
        qspi_writeb(qspi,offset,tmp);

}

static inline void qspi_clr_bit(struct ls_qspi *qspi, unsigned short offset,u8 value)
{
        u8 tmp;

        tmp = qspi_readb(qspi,offset);
        tmp &= ~value;
        qspi_writeb(qspi,offset,tmp);

}

static inline void dump_reg(struct ls_qspi *qspi)
{
	printk("CR    :%08x\n",qspi_readb(qspi,QSPI_CR));
	printk("CSR   :%08x\n",qspi_readb(qspi,QSPI_CSR));
	printk("SR    :%08x\n",qspi_readb(qspi,QSPI_SR));
}

static int qspi_setting_spre(const void *key,const void *elt)
{
        unsigned long *d = (unsigned long*)key;
        struct spretoregs_t *b = (struct spretoregs_t *)elt;
        if(*d > b->spre)
                return 1;
        else if(*d < b->spre)
                return -1;
        return 0;
}

static struct spretoregs_t *search_divisor(unsigned int spre)
{
        struct spretoregs_t *b = NULL;

        if(spre <= spretoregs[ARRAY_SIZE(spretoregs) - 1].spre)
                b = (struct spretoregs_t *)bsearch((const void*)&spre,(const void*)spretoregs,ARRAY_SIZE(spretoregs),
                                                       sizeof(struct spretoregs_t),qspi_setting_spre);

        return b;
}

static int ls_qspi_wait_nobusy(struct ls_qspi *qspi)
{
        u32 sr;

        return readb_relaxed_poll_timeout(qspi->io_base + QSPI_CSR, sr,
                                          !(sr & QSPI_CSR_QBUSY), 10,
                                          QSPI_BUSY_TIMEOUT_US);
}

static void ls_qspi_set_framemode(struct spi_nor *nor,
                                     struct ls_qspi_cmd *cmd, bool read)
{
        u32 dmode = QSPI_CR_QMODE_1;

	if (read) {
                switch (nor->read_proto) {
                default:
                case SNOR_PROTO_1_1_1:
                        dmode = QSPI_CR_QMODE_1;
                        break;
		case SNOR_PROTO_1_1_2:
			dmode = QSPI_CR_QMODE_2;
			break;
		case SNOR_PROTO_1_1_4:
		case SNOR_PROTO_1_4_4:
			dmode = QSPI_CR_QMODE_4;
			break;
		}
	}

	cmd->framemode = dmode;

}

static void ls_qspi_read_fifo(u8 *val, struct ls_qspi *qspi)
{
        *val = qspi_readb(qspi,QSPI_DR);
}

static void ls_qspi_write_fifo(u8 *val, struct ls_qspi *qspi)
{
	qspi_writeb(qspi,QSPI_DR,*val);
}

void qspi_set_dummy_cycles(struct ls_qspi *qspi,u8 cycles)
{
	qspi_set_bit(qspi,QSPI_CR,QSPI_CR_DUMMY);

	qspi_writeb(qspi,QSPI_DR,cycles-1);

	qspi_clr_bit(qspi,QSPI_CR,QSPI_CR_DUMMY);
}

static int ls_qspi_tx_poll(struct ls_qspi *qspi,
                              const struct ls_qspi_cmd *cmd)
{
        void (*tx_fifo)(u8 * ,struct ls_qspi *);
        u32 len = cmd->len;
        u8 *buf = cmd->buf;
	u8 setb,clrb;

	qspi_clr_bit(qspi,QSPI_CSR,QSPI_CSR_QCS);

	qspi_writeb(qspi,QSPI_DR,cmd->opcode);

	if(cmd->addr_width){
		if(cmd->addr_width ==3){
			qspi_writeb(qspi,QSPI_DR,(uint8_t)(cmd->addr >>16));
			qspi_writeb(qspi,QSPI_DR,(uint8_t)(cmd->addr >>8));
			qspi_writeb(qspi,QSPI_DR,(uint8_t)(cmd->addr));
		}else if(cmd->addr_width ==4){
			qspi_writeb(qspi,QSPI_DR,(uint8_t)(cmd->addr >>24));
			qspi_writeb(qspi,QSPI_DR,(uint8_t)(cmd->addr >>16));
			qspi_writeb(qspi,QSPI_DR,(uint8_t)(cmd->addr >>8));
			qspi_writeb(qspi,QSPI_DR,(uint8_t)(cmd->addr));
		}
	}

	if(cmd->dummy)
		qspi_set_dummy_cycles(qspi,8);

	setb = (cmd->framemode)  &QSPI_CR_QTYPE_MASK;
	clrb = QSPI_CR_QTYPE_MASK;
	qspi_writeb(qspi,QSPI_CR,qspi_readb(qspi,QSPI_CR) &~clrb |setb);

	if (cmd->qspimode == FMODE_INDW)
                tx_fifo = ls_qspi_write_fifo;
	else
                tx_fifo = ls_qspi_read_fifo;
        while (len--) {

                tx_fifo(buf++, qspi);
        }

	setb = QSPI_CR_QMODE_1 &QSPI_CR_QMODE_MASK;
	clrb = QSPI_CR_QTYPE_MASK;
	qspi_writeb(qspi,QSPI_CR,qspi_readb(qspi,QSPI_CR) &~clrb |setb);

	qspi_set_bit(qspi,QSPI_CSR,QSPI_CSR_QCS);

        return 0;
}

static int ls_qspi_tx(struct ls_qspi *qspi,
                         const struct ls_qspi_cmd *cmd)
{

        return ls_qspi_tx_poll(qspi, cmd);
}

static int ls_qspi_send(struct ls_qspi_flash *flash,
                           const struct ls_qspi_cmd *cmd)
{
        struct ls_qspi *qspi = flash->qspi;
        int err;

        err = ls_qspi_wait_nobusy(qspi);
        if (err)
                goto abort;

        err = ls_qspi_tx(qspi, cmd);
        if (err)
                goto abort;

	return err;

abort:

        if (err)
                dev_err(qspi->dev, "%s abort err:%d\n", __func__, err);

        return err;

}

static int ls_qspi_read_reg(struct spi_nor *nor,
                               u8 opcode, u8 *buf, int len)
{
        struct ls_qspi_flash *flash = nor->priv;
        struct device *dev = flash->qspi->dev;
        struct ls_qspi_cmd cmd;

        dev_dbg(dev, "read_reg: cmd:%#.2x buf:%pK len:%#x\n", opcode, buf, len);

        memset(&cmd, 0, sizeof(cmd));
        cmd.opcode = opcode;
        cmd.tx_data = true;
        cmd.len = len;
        cmd.buf = buf;
        cmd.qspimode = FMODE_INDR;

        ls_qspi_set_framemode(nor, &cmd, false);

        return ls_qspi_send(flash, &cmd);
}

static int ls_qspi_write_reg(struct spi_nor *nor, u8 opcode,
                                u8 *buf, int len)
{
        struct ls_qspi_flash *flash = nor->priv;
        struct device *dev = flash->qspi->dev;
        struct ls_qspi_cmd cmd;

        dev_dbg(dev, "write_reg: cmd:%#.2x buf:%pK len:%#x\n", opcode, buf, len);

        memset(&cmd, 0, sizeof(cmd));
        cmd.opcode = opcode;
        cmd.tx_data = !!(buf && len > 0);
        cmd.len = len;
        cmd.buf = buf;
        cmd.qspimode = FMODE_INDW;

        ls_qspi_set_framemode(nor, &cmd, false);

        return ls_qspi_send(flash, &cmd);
}

static ssize_t ls_qspi_read(struct spi_nor *nor, loff_t from, size_t len,
                               u_char *buf)
{
        struct ls_qspi_flash *flash = nor->priv;
        struct ls_qspi *qspi = flash->qspi;
        struct ls_qspi_cmd cmd;
        int err;

        dev_dbg(qspi->dev, "read(%#.2x): buf:%pK from:%#.8x len:%#zx\n",
                nor->read_opcode, buf, (u32)from, len);

        memset(&cmd, 0, sizeof(cmd));
        cmd.opcode = nor->read_opcode;
        cmd.addr_width = nor->addr_width;
        cmd.addr = (u32)from;
        cmd.tx_data = true;
        cmd.dummy = nor->read_dummy;
        cmd.len = len;
        cmd.buf = buf;
        cmd.qspimode = FMODE_INDR;

        ls_qspi_set_framemode(nor, &cmd, true);
        err = ls_qspi_send(flash, &cmd);

        return err ? err : len;
}

static ssize_t ls_qspi_write(struct spi_nor *nor, loff_t to, size_t len,
                                const u_char *buf)
{
        struct ls_qspi_flash *flash = nor->priv;
        struct device *dev = flash->qspi->dev;
        struct ls_qspi_cmd cmd;
        int err;

        dev_dbg(dev, "write(%#.2x): buf:%p to:%#.8x len:%#zx\n",
                nor->program_opcode, buf, (u32)to, len);

        memset(&cmd, 0, sizeof(cmd));
        cmd.opcode = nor->program_opcode;
        cmd.addr_width = nor->addr_width;
        cmd.addr = (u32)to;
        cmd.tx_data = true;
        cmd.len = len;
        cmd.buf = (void *)buf;
        cmd.qspimode = FMODE_INDW;

        ls_qspi_set_framemode(nor, &cmd, false);
        err = ls_qspi_send(flash, &cmd);

        return err ? err : len;
}

static int ls_qspi_erase(struct spi_nor *nor, loff_t offs)
{
        struct ls_qspi_flash *flash = nor->priv;
        struct device *dev = flash->qspi->dev;
        struct ls_qspi_cmd cmd;

        dev_dbg(dev, "erase(%#.2x):offs:%#x\n", nor->erase_opcode, (u32)offs);

        memset(&cmd, 0, sizeof(cmd));
        cmd.opcode = nor->erase_opcode;
        cmd.addr_width = nor->addr_width;
        cmd.addr = (u32)offs;
        cmd.qspimode = FMODE_INDW;

        ls_qspi_set_framemode(nor, &cmd, false);

        return ls_qspi_send(flash, &cmd);
}

static int ls_qspi_prep(struct spi_nor *nor, enum spi_nor_ops ops)
{
        struct ls_qspi_flash *flash = nor->priv;
        struct ls_qspi *qspi = flash->qspi;

        mutex_lock(&qspi->lock);
        return 0;
}

static void ls_qspi_unprep(struct spi_nor *nor, enum spi_nor_ops ops)
{
        struct ls_qspi_flash *flash = nor->priv;
        struct ls_qspi *qspi = flash->qspi;

        mutex_unlock(&qspi->lock);
}

static int ls_qspi_flash_setup(struct ls_qspi *qspi,
                                  struct device_node *np)
{
        struct spi_nor_hwcaps hwcaps = {
                .mask = SNOR_HWCAPS_READ |
                        SNOR_HWCAPS_READ_FAST |
                        SNOR_HWCAPS_PP,
        };
        u32 width, presc, cs_num, max_rate = 0;
	u8  setb=0,clrb=0;
	struct ls_qspi_flash *flash;
        struct mtd_info *mtd;
        int ret;
	struct spretoregs_t *st;

        of_property_read_u32(np, "reg", &cs_num);
        if (cs_num >= LS_MAX_NORCHIP)
                return -EINVAL;

        of_property_read_u32(np, "spi-max-frequency", &max_rate);
        if (!max_rate)
                return -EINVAL;

        presc = DIV_ROUND_UP(100000000, max_rate);
	st = search_divisor(presc);
	if(st){
		presc =st->div;
	}else{
		return -1;
	}

        if (of_property_read_u32(np, "spi-rx-bus-width", &width))
                width = 1;

	if (width == 4)
		hwcaps.mask |= SNOR_HWCAPS_READ_1_1_4;
	else if (width == 2)
		hwcaps.mask |= SNOR_HWCAPS_READ_1_1_2;
	else if (width != 1)
		return -EINVAL;

	flash = &qspi->flash[cs_num];
	flash->qspi = qspi;
	flash->cs = cs_num;
	flash->presc = presc;

	flash->nor.dev = qspi->dev;
	spi_nor_set_flash_node(&flash->nor, np);
	flash->nor.priv = flash;
	mtd = &flash->nor.mtd;

	flash->nor.read = ls_qspi_read;
	flash->nor.write = ls_qspi_write;
	flash->nor.erase = ls_qspi_erase;
	flash->nor.read_reg = ls_qspi_read_reg;
	flash->nor.write_reg = ls_qspi_write_reg;
	flash->nor.prepare = ls_qspi_prep;
	flash->nor.unprepare = ls_qspi_unprep;

	/*Switch to QSPI mode*/
	qspi_set_bit(qspi,QSPI_RSSR,QSPI_RSSR_SPACE);
	qspi_set_bit(qspi,QSPI_SR,QSPI_SR_SWITCH);

	/*Set clk div */
	setb |= (flash->presc << QSPI_CSR_CLK_DIV_SHIFET)  &QSPI_CSR_CLK_DIV_MASK;
	clrb |= QSPI_CSR_CLK_DIV_MASK;
	qspi_writeb(qspi,QSPI_CSR,qspi_readb(qspi,QSPI_CSR) &~clrb |setb);


	ret = spi_nor_scan(&flash->nor, NULL, &hwcaps);
	if(ret) {
		dev_err(qspi->dev, "device scan failed\n");
		return ret;
	}


	ret = mtd_device_register(mtd, NULL, 0);
	if (ret) {
		dev_err(qspi->dev, "mtd device parse failed\n");
		return ret;
	}

	flash->registered = true;

	dev_dbg(qspi->dev, "cs:%d bus:%d\n", cs_num, width);

	return 0;

}

static void ls_qspi_mtd_free(struct ls_qspi *qspi)
{
        int i;

        for (i = 0; i < LS_MAX_NORCHIP; i++)
                if (qspi->flash[i].registered)
                        mtd_device_unregister(&qspi->flash[i].nor.mtd);
}

static int ls_qspi_probe(struct platform_device *pdev)
{
        struct device *dev = &pdev->dev;
        struct device_node *flash_np;
        struct ls_qspi *qspi;
        struct resource *res;
        int ret;

	qspi = devm_kzalloc(dev, sizeof(*qspi), GFP_KERNEL);
	if (!qspi)
		return -ENOMEM;

	qspi->nor_num = of_get_child_count(dev->of_node);
	if (!qspi->nor_num || qspi->nor_num > LS_MAX_NORCHIP)
		return -ENODEV;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "qspi");
	qspi->io_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(qspi->io_base))
		return PTR_ERR(qspi->io_base);

	init_completion(&qspi->cmd_completion);

	qspi->dev = dev;
	platform_set_drvdata(pdev, qspi);
	mutex_init(&qspi->lock);

	for_each_available_child_of_node(dev->of_node, flash_np) {
		ret = ls_qspi_flash_setup(qspi, flash_np);
		if (ret) {
			dev_err(dev, "unable to setup flash chip\n");
			goto err_flash;
		}
	}

	return 0;

err_flash:
        mutex_destroy(&qspi->lock);
        ls_qspi_mtd_free(qspi);

	return ret;
}

static int ls_qspi_remove(struct platform_device *pdev)
{
        struct ls_qspi *qspi = platform_get_drvdata(pdev);

        /* disable qspi */

        ls_qspi_mtd_free(qspi);
        mutex_destroy(&qspi->lock);

        return 0;
}

static const struct of_device_id loongson_qspi_match[] = {
        {.compatible = "ls,ls-qspi"},
        {}
};
MODULE_DEVICE_TABLE(of, loongson_qspi_match);

static struct platform_driver ls_qspi_driver = {
        .probe  = ls_qspi_probe,
        .remove = ls_qspi_remove,
        .driver = {
                .name = "ls-quadspi",
                .of_match_table = loongson_qspi_match,
        },
};
module_platform_driver(ls_qspi_driver);

MODULE_AUTHOR("ruizhao  <zhaorui@loongson.cn>");
MODULE_DESCRIPTION("loongson quad spi driver");
MODULE_LICENSE("GPL v2");


