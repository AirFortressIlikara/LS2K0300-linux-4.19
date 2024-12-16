#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/iopoll.h>
#include <linux/of_platform.h>


#define SPI_CR1	    0x00
#define SPI_CR2     0x04
#define SPI_CR3	    0x08
#define SPI_CR4	    0x0c
#define SPI_IER     0x10
#define SPI_SR1     0x14
#define SPI_SR2     0x18
#define SPI_CFG1    0x20
#define SPI_CFG2    0x24
#define SPI_CFG3    0x28
#define SPI_CRC1    0x30
#define SPI_CRC2    0x34
#define SPI_DR      0x40

/*2P500 SPICR1 bit fields*/
#define SPI_CR1_SSREV		BIT(8)
#define	SPI_CR1_AUTOSUS		BIT(2)
#define SPI_CR1_CSTART		BIT(1)
#define SPI_CR1_SPE		BIT(0)

/*2P500 SPICR2 bit fields*/
#define SPI_CR2_TXDMAEN		BIT(15)
#define SPI_CR2_TXFTHLV_SHIFT	8
#define SPI_CR2_TXFTHLV		GENMASK(9,8)
#define SPI_CR2_RXDMAEN		BIT(7)
#define SPI_CR2_RXFTHLV_SHIFT   0
#define SPI_CR2_RXFTHLV		GENMASK(1,0)

/*2P500 SPICR3 bit fields*/
#define SPI_CR3_TSIZE_SHIFT	0
#define SPI_CR3_TSIZE		GENMASK(15,0)

/*2P500 SPICR4 bit fields*/
#define SPI_CTSIZE_SHIFT	0
#define SPI_CTSIZE		GENMASK(15,0)

/*2P500 SPIIER bit fields*/
#define SPI_IER_EOTIE		BIT(15)
#define SPI_IER_MODFIE		BIT(11)
#define SPI_IER_CRCEIE		BIT(10)
#define SPI_IER_UDRIE		BIT(9)
#define SPI_IER_OVRIE		BIT(8)
#define SPI_IER_SUSPIE		BIT(7)
#define SPI_IER_TXEIE		BIT(6)
#define SPI_IER_RXEIE		BIT(4)
#define SPI_IER_DXAIE		BIT(2)
#define SPI_IER_TXAIE		BIT(1)
#define SPI_IER_RXAIE		BIT(0)

/*2P500 SPISR1 bit fields*/
#define SPI_SR1_EOT		BIT(15)
#define SPI_SR1_MODF		BIT(11)
#define SPI_SR1_CRCE		BIT(10)
#define SPI_SR1_UDR		BIT(9)
#define SPI_SR1_OVR		BIT(8)
#define SPI_SR1_SUSP		BIT(7)
#define SPI_SR1_TXE		BIT(6)
#define SPI_SR1_RXE		BIT(4)
#define SPI_SR1_DXA		BIT(2)
#define SPI_SR1_TXA		BIT(1)
#define SPI_SR1_RXA		BIT(0)

/*2P500 SPISR2 bit fields*/
#define SPI_SR2_TXFLV_SHIFT	8
#define SPI_SR2_TXFLV		GENMASK(10,8)
#define SPI_SR2_RXFLV_SHIFT	0
#define SPI_SR2_RXFLV		GENMASK(2,0)

/*2P500 CFG1 bit fields*/
#define SPI_CFG1_DSIZE_SHIFT	8
#define SPI_CFG1_DSIZE		GENMASK(12,8)
#define SPI_CFG1_LSBFRST	BIT(7)
#define	SPI_CFG1_CPHA		BIT(1)
#define SPI_CFG1_CPOL		BIT(0)

/*2P500 CFG2 bit fields*/
#define SPI_CFG2_BRINT_SHIFT    8
#define SPI_CFG2_BRINT		GENMASK(15,8)
#define SPI_CFG2_BRDEC_SHIFT    2
#define SPI_CFG2_BRDEC		GENMASK(7,2)

/*2P500 CFG3 bit fields*/
#define SPI_CFG3_SSMODE_SHIFT   8
#define SPI_CFG3_SSMODE		GENMASK(9,8)
#define SPI_CFG3_DOE		BIT(3)
#define SPI_CFG3_DIE		BIT(2)
#define SPI_CFG3_DIOSWP		BIT(1)
#define SPI_CFG3_MSTR		BIT(0)

#define MODE (SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST)

struct ls_spi_info{
	unsigned int bus_num;		/*spi_master bus_num*/
	unsigned int num_chipselect;    /*spi_master num_chipselect*/
	unsigned long max_clk;		/*spi max serial clock*/
	struct spi_board_info	*board_info;   /*link to spi device*/
};

typedef enum{
	RX,
	TX
}DATATYPE;

struct ls_spi{
	struct device	       *dev;
	struct spi_master      *master;
	void   __iomem         *base;
	int			irq;
	spinlock_t              lock;

	u8                      spi_mode;
	u8			tx_trigger;
	u8			rx_trigger;
	u32			max_clk;
	u32			cur_speed;
	u32			cur_bpw;
	u32			cur_fthlv;
	u32			cur_xferlen;
	u32			cs_change;

	const void		*tx_buf;
	void			*rx_buf;
	int			tx_len;
	int			rx_len;
	struct ls_spi_info      *pdata;
};


static	inline void spi_writel(struct ls_spi *spi,unsigned short offset, unsigned int value)
{
	writel(value,spi->base+offset);
}
static inline u32 spi_readl(struct ls_spi *spi, unsigned short offset)
{
	return readl(spi->base+offset);
}

static inline void spi_set_bit(struct ls_spi *spi, unsigned short offset,unsigned int value)
{
	u32 tmp;

	tmp =spi_readl(spi,offset);
	tmp |= value;
	spi_writel(spi,offset,tmp);

}

static inline void spi_clr_bit(struct ls_spi *spi, unsigned short offset,unsigned int value)
{
	u32 tmp;

	tmp = spi_readl(spi,offset);
	tmp &= ~value;
	spi_writel(spi,offset,tmp);

}

static inline void dump_reg(struct ls_spi *spi)
{
	printk("CR1   :%08x\n",spi_readl(spi,SPI_CR1));
	printk("CR2   :%08x\n",spi_readl(spi,SPI_CR2));
	printk("CR3   :%08x\n",spi_readl(spi,SPI_CR3));
	printk("CR4   :%08x\n",spi_readl(spi,SPI_CR4));
	printk("IER   :%08x\n",spi_readl(spi,SPI_IER));
	printk("SR1   :%08x\n",spi_readl(spi,SPI_SR1));
	printk("SR2   :%08x\n",spi_readl(spi,SPI_SR2));
	printk("CFG1  :%08x\n",spi_readl(spi,SPI_CFG1));
	printk("CFG2  :%08x\n",spi_readl(spi,SPI_CFG2));
	printk("CFG3  :%08x\n",spi_readl(spi,SPI_CFG3));
	printk("\n\n========dump_reg end============\n\n");
}

static int ls_spi_get_bpw_mask(struct ls_spi *spi)
{
	unsigned long flags;
	u32 cfg1,max_bpw;

	spin_lock_irqsave(&spi->lock, flags);

	spi_set_bit(spi,SPI_CFG1,SPI_CFG1_DSIZE);

	cfg1 = spi_readl(spi,SPI_CFG1);
	max_bpw = (cfg1 & SPI_CFG1_DSIZE) >> SPI_CFG1_DSIZE_SHIFT;
	max_bpw +=1;

	spin_unlock_irqrestore(&spi->lock, flags);

	dev_dbg(spi->dev, "%d-bit maximum data frame\n", max_bpw);

	return SPI_BPW_RANGE_MASK(4, max_bpw);
}

static int ls_spi_prepare_br(struct ls_spi *spi, u32 speed_hz)
{
	u32 div;

	div=DIV_ROUND_UP(125000000 & ~0x1, speed_hz);

	if(div < 2 || div > 255){
		return -EINVAL;
	}

	spi->cur_speed = 125000000 / div;
	return div;
}

static u32 ls_spi_prepare_fthlv(struct ls_spi *spi,DATATYPE data_type)
{
	u32 fthlv;

	if(data_type == RX){
		if(spi->rx_len > 3)
			fthlv =4;
		else if(spi->rx_len > 1)
			fthlv =2;
		else
			fthlv =1;
	}else{
		if(spi->tx_len > 3)
			fthlv =4;
		else if(spi->tx_len > 1)
			fthlv =2;
		else
			fthlv =1;

	}
	return fthlv;

}

static void ls_spi_write_txfifo(struct ls_spi *spi)
{

	while ((spi->tx_len > 0) && spi_readl(spi,SPI_SR1) & SPI_SR1_TXA){
		u32 offs = spi->cur_xferlen -spi->tx_len;

		if(spi->tx_len >= sizeof(u32)) {
			const u32 * tx_buf32=(const u32 *)(spi->tx_buf + offs);

			writel(*tx_buf32, spi->base +SPI_DR);
			spi->tx_len -= sizeof(u32);
		}else if (spi->tx_len >= sizeof(u16)){
			const u16 * tx_buf16 =(const u16 *)(spi->tx_buf + offs);

			writew(*tx_buf16, spi->base +SPI_DR);;
			spi->tx_len -=sizeof(u16);
		}else{
			const u8 * tx_buf8=(const u8 *)(spi->tx_buf + offs);

			writeb(*tx_buf8, spi->base +SPI_DR);
			spi->tx_len -=sizeof(u8);
		}
	}


	dev_dbg(spi->dev,"%s %d bytes left\n",__func__,spi->tx_len);
}


static void ls_spi_read_fifo(struct ls_spi *spi, bool flush)
{
	u32 sr =spi_readl(spi,SPI_SR1);
	u32 rxflv = spi_readl(spi,SPI_SR2) & SPI_SR2_RXFLV >> SPI_SR2_RXFLV_SHIFT;
	u32 cr2_setb =0,cr2_clrb=0;

	while((spi->rx_len >0)&&
	      (sr &SPI_SR1_RXA)||
	      (flush && (rxflv >0 ))){
		u32 offs = spi->cur_xferlen -spi->rx_len;
		if((spi->rx_len >=sizeof(u32))){
			 u32 *rx_buf32 = (u32 *)(spi->rx_buf + offs);

			*rx_buf32 = readl_relaxed(spi->base + SPI_DR);
			 spi->rx_len -= sizeof(u32);
		}else if((spi->rx_len >=sizeof(u16))||
			(flush && (rxflv >= 2 || spi->cur_bpw > 8))){
			u16 *rx_buf16 = (u16 *)(spi->rx_buf + offs);

			*rx_buf16 = readw_relaxed(spi->base + SPI_DR);
			spi->rx_len -= sizeof(u16);
		}else{
			u8 *rx_buf8 = (u8 *)(spi->rx_buf + offs);

			*rx_buf8 = readb_relaxed(spi->base + SPI_DR);
			spi->rx_len -= sizeof(u8);
		}

		if(spi->rx_len >3){
			cr2_setb = (3 << SPI_CR2_RXFTHLV_SHIFT)  &SPI_CR2_RXFTHLV;
			cr2_clrb = SPI_CR2_RXFTHLV;
			spi_writel(spi,SPI_CR2,spi_readl(spi,SPI_CR2) &~cr2_clrb |cr2_setb);
		}
		else if(spi->rx_len >1){
			cr2_setb = (1 << SPI_CR2_RXFTHLV_SHIFT)  &SPI_CR2_RXFTHLV;
			cr2_clrb = SPI_CR2_RXFTHLV;
			spi_writel(spi,SPI_CR2,spi_readl(spi,SPI_CR2) &~cr2_clrb |cr2_setb);

		}
		else{
			cr2_setb = (0 << SPI_CR2_RXFTHLV_SHIFT)  &SPI_CR2_RXFTHLV;
			cr2_clrb = SPI_CR2_RXFTHLV;
			spi_writel(spi,SPI_CR2,spi_readl(spi,SPI_CR2) &~cr2_clrb |cr2_setb);
		}

		sr =spi_readl(spi,SPI_SR1);
		rxflv = spi_readl(spi,SPI_SR2) & SPI_SR2_RXFLV >> SPI_SR2_RXFLV_SHIFT;
	}
	dev_dbg(spi->dev,"%s%s: %d bytes left\n",__func__,flush ? "flush" : "",spi->rx_len);
}

static void ls_spi_enable(struct ls_spi *spi)
{
	dev_dbg(spi->dev,"enable spi controller\n");

	spi_set_bit(spi,SPI_CR1,SPI_CR1_SPE);
}

static void ls_spi_disable(struct ls_spi *spi)
{
	unsigned long flags;
	u32 cr1,sr;

	dev_dbg(spi->dev,"disable spi controller\n");

	spin_lock_irqsave(&spi->lock,flags);

	cr1 =spi_readl(spi,SPI_CR1);
	if(!(cr1 &SPI_CR1_SPE)){
		spin_unlock_irqrestore(&spi->lock, flags);
		return;
	}

	/*wait eot or suspend*/
	if (readl_relaxed_poll_timeout_atomic(spi->base+SPI_SR1, sr, !(sr &SPI_SR1_EOT), 10 ,100000) < 0){
		if(cr1 & SPI_CR1_CSTART){
			spi_writel(spi,SPI_CR1,cr1|SPI_CR1_AUTOSUS);
			if(readl_relaxed_poll_timeout_atomic(spi->base+SPI_SR1,sr,!(sr & SPI_SR1_SUSP),10,100000) < 0){
				dev_warn(spi->dev, "Suspend request timeout\n");
			}
		}

	}

	if (spi->rx_buf && (spi->rx_len > 0)){
		ls_spi_read_fifo(spi, true);
	}

	if(!spi->cs_change)
		spi_clr_bit(spi,SPI_CR1,SPI_CR1_SPE);

	/*Disable interrupt and clear status flags*/
	spi_writel(spi,SPI_IER,0);
	spi_writel(spi,SPI_SR1,0xffffffff);

	spin_unlock_irqrestore(&spi->lock,flags);
}

/*ls SPI irq handler for SPI controller events*/
static irqreturn_t ls_spi_irq(int irq,void *dev)
{
	struct spi_master *master = dev;
	struct ls_spi *spi =spi_master_get_devdata(master);
	u32 sr, ier, mask;
	unsigned long flags;
	bool end =false;

	spin_lock_irqsave(&spi->lock, flags);

	sr = spi_readl(spi,SPI_SR1);
	ier = spi_readl(spi,SPI_IER);

	mask = ier;

	if(!(sr &mask)){
		dev_dbg(spi->dev,"interrupt (sr =0x%x, ire = 0x%x)\n",sr, ier);
		spin_unlock_irqrestore(&spi->lock, flags);
		return IRQ_NONE;
	}

	if(sr & mask  &SPI_SR1_SUSP){
		dev_warn(spi->dev,"SPI suspend\n");
		if(spi->rx_buf &&(spi->rx_len >0))
			ls_spi_read_fifo(spi,false);
	}

	if(sr &SPI_SR1_MODF){
		dev_warn(spi->dev,"Mode failt: transfer aborted\n");
		end =true;
		spi_set_bit(spi,SPI_SR1,SPI_SR1_MODF);
	}

	if(sr &SPI_SR1_OVR){
		dev_warn(spi->dev, "Overrun!\n");
		if(spi->rx_buf &&(spi->rx_len) > 0)
			ls_spi_read_fifo(spi,false);

		spi_set_bit(spi,SPI_SR1,SPI_SR1_OVR);
	}

	if(sr & mask &SPI_SR1_EOT){
		if(spi->rx_buf &&(spi->rx_len > 0)){
			ls_spi_read_fifo(spi,true);
		}
		end = true;

		spi_set_bit(spi,SPI_SR1,SPI_SR1_EOT);
	}

	if(sr &  mask & SPI_SR1_RXA){
		if(spi->rx_buf &&(spi->rx_len > 0))
			ls_spi_read_fifo(spi, false);
	}

	if(sr & mask &SPI_SR1_TXA){
		if(spi->tx_buf &&(spi->tx_len > 0 ))
			ls_spi_write_txfifo(spi);
	}

	spin_unlock_irqrestore(&spi->lock,flags);

	if(end){
		spi_finalize_current_transfer(master);
		ls_spi_disable(spi);
	}

	return IRQ_HANDLED;
}


static int ls_spi_setup(struct spi_device *spi_dev)
{
	int ret =0;
	struct ls_spi *spi;
	spi= spi_master_get_devdata(spi_dev->master);
#if 0
	if (!gpio_is_valid(spi_dev->cs_gpio)) {
		dev_err(&spi_dev->dev, "%d is not a valid gpio\n",
			spi_dev->cs_gpio);
		return -EINVAL;
	}

	dev_dbg(&spi_dev->dev, "%s: set gpio%d output %s\n", __func__,
		spi_dev->cs_gpio,
		(spi_dev->mode & SPI_CS_HIGH) ? "low" : "high");

	ret = gpio_direction_output(spi_dev->cs_gpio,
				    !(spi_dev->mode & SPI_CS_HIGH));
#endif

	return 0;
}

static int ls_spi_prepare_message(struct spi_master *master, struct spi_message *msg)
{
	struct ls_spi *spi =spi_master_get_devdata(master);
	struct spi_device *spi_dev = msg->spi;

	unsigned long flags;

	u32 cfg1_setb,cfg1_clrb;

	if(spi_dev->mode &SPI_CPOL)
		cfg1_setb |= SPI_CFG1_CPOL;
	else
		cfg1_clrb |= SPI_CFG1_CPOL;


	if(spi_dev->mode & SPI_CPHA)
		cfg1_setb |=SPI_CFG1_CPHA;
	else
		cfg1_clrb |=SPI_CFG1_CPHA;

	if(spi_dev->mode &SPI_LSB_FIRST)
		cfg1_setb |=SPI_CFG1_LSBFRST;
	else
		cfg1_clrb |=SPI_CFG1_LSBFRST;

	dev_dbg(spi->dev,"cpol = %d cpha =%d lsb_first = %d cs_high =%d\n",spi_dev->mode&SPI_CPOL,spi_dev->mode & SPI_CPOL,spi_dev->mode &SPI_LSB_FIRST,spi_dev->mode &SPI_CS_HIGH);

	spin_lock_irqsave(&spi->lock,flags);

	if(cfg1_setb || cfg1_clrb)
		spi_writel(spi,SPI_CFG1,spi_readl(spi,SPI_CFG1) & ~cfg1_clrb|cfg1_setb);

	spin_unlock_irqrestore(&spi->lock, flags);

	return 0;
}

static int ls_spi_transfer_one_irq(struct ls_spi *spi)
{
	unsigned long flags;
	u32 ier =0;

	if(spi->tx_buf &&spi->rx_buf){
		ier |= SPI_IER_DXAIE;
	}
	if(spi->rx_buf){
		ier |= SPI_IER_RXAIE;
	}
	if(spi->tx_buf){
		ier |= SPI_IER_TXAIE;
	}
	ier |= SPI_IER_EOTIE | SPI_IER_OVRIE | SPI_IER_MODFIE |SPI_IER_RXAIE | SPI_IER_TXAIE;

	spin_lock_irqsave(&spi->lock, flags);

	ls_spi_enable(spi);

	if(spi->tx_buf)
		ls_spi_write_txfifo(spi);

	spi_set_bit(spi,SPI_CR1,SPI_CR1_CSTART);

	spi_writel(spi,SPI_IER,ier);

	spin_unlock_irqrestore(&spi->lock, flags);

	return 1;
}

static int ls_spi_transfer_one_setup(struct ls_spi *spi,struct spi_device *spi_dev,struct spi_transfer *transfer)
{
	unsigned long flags;
	u32 cfg1_setb =0,cfg1_clrb =0,cr2_setb=0, cr2_clrb=0, cfg2_setb =0,cfg2_clrb =0;

	u32 nb_words;
	int ret =0;

	spin_lock_irqsave(&spi->lock, flags);

	spi_set_bit(spi,SPI_CR1,SPI_CR1_AUTOSUS);

	u32 bpw,fthlv;

	spi->cur_bpw = transfer->bits_per_word;
	bpw = spi->cur_bpw -1;

	cfg1_clrb |=SPI_CFG1_DSIZE;
	cfg1_setb |=(bpw << SPI_CFG1_DSIZE_SHIFT) & SPI_CFG1_DSIZE;

	spi->cur_fthlv = ls_spi_prepare_fthlv(spi,1);
	fthlv =spi->cur_fthlv -1;
	cr2_clrb |= SPI_CR2_TXFTHLV;
	cr2_setb |= (fthlv << SPI_CR2_TXFTHLV_SHIFT)  &SPI_CR2_TXFTHLV;

	spi->cur_fthlv = ls_spi_prepare_fthlv(spi,0);
	fthlv =spi->cur_fthlv -1;
	cr2_clrb |= SPI_CR2_RXFTHLV;
	cr2_setb |= (fthlv << SPI_CR2_RXFTHLV_SHIFT)  &SPI_CR2_RXFTHLV;

	spi_writel(spi,SPI_CFG1,spi_readl(spi,SPI_CFG1) &~cfg1_clrb |cfg1_setb);
	spi_writel(spi,SPI_CR2,spi_readl(spi,SPI_CR2) &~cr2_clrb |cr2_setb);

	if(spi->cur_speed != transfer->speed_hz){
		int br;

		br = ls_spi_prepare_br(spi,transfer->speed_hz);
		if(br <0){
			ret =-EMSGSIZE;
			goto out;
		}

		transfer->speed_hz =spi->cur_speed;

		cfg2_clrb |=SPI_CFG2_BRINT;
		cfg2_setb |=((u32)br << SPI_CFG2_BRINT_SHIFT) &SPI_CFG2_BRINT;

	}

	spi_writel(spi,SPI_CFG2,spi_readl(spi,SPI_CFG2) &~cfg2_clrb |cfg2_setb);

	spi_set_bit(spi,SPI_CFG3,SPI_CFG3_MSTR);

	if(transfer->tx_buf && transfer->rx_buf){
		spi_clr_bit(spi,SPI_CFG3,SPI_CFG3_DIOSWP);
		spi_set_bit(spi,SPI_CFG3,SPI_CFG3_DIE);
		spi_set_bit(spi,SPI_CFG3,SPI_CFG3_DOE);
	}else if(transfer->tx_buf){
		spi_clr_bit(spi,SPI_CFG3,SPI_CFG3_DIOSWP);
		spi_set_bit(spi,SPI_CFG3,SPI_CFG3_DOE);
		spi_clr_bit(spi,SPI_CFG3,SPI_CFG3_DIE);
	}else if(transfer->rx_buf){
		spi_clr_bit(spi,SPI_CFG3,SPI_CFG3_DIOSWP);
		spi_set_bit(spi,SPI_CFG3,SPI_CFG3_DIE);
		spi_clr_bit(spi,SPI_CFG3,SPI_CFG3_DOE);
	}

	if(spi->cur_bpw <= 8)
		nb_words = transfer->len;
	else if(spi->cur_bpw <= 16)
	        nb_words = DIV_ROUND_UP(transfer->len * 8, 16);
	else
		nb_words = DIV_ROUND_UP(transfer->len * 8, 32);
	nb_words <<= SPI_CR3_TSIZE_SHIFT;

	if(nb_words <= SPI_CR3_TSIZE){
		nb_words =nb_words-1;
		spi_writel(spi,SPI_CR3,nb_words);
		spi_writel(spi,SPI_CR4,0);
	}else{
		ret= -EMSGSIZE;
		goto out;
	}

	spi->cur_xferlen = transfer->len;

	dev_dbg(spi->dev,"data frame of %d bit,data packet of %d data frame",spi->cur_bpw,spi->cur_fthlv);
	dev_dbg(spi->dev,"speed set to %d Hz\n",spi->cur_speed);
	dev_dbg(spi->dev,"transfer of %d bytes (%d data frames)\n",spi->cur_xferlen,nb_words);

out:
	spin_unlock_irqrestore(&spi->lock, flags);

	return ret;
}

static int ls_spi_transfer_one(struct spi_master *master,struct spi_device *spi_dev,struct spi_transfer *transfer)
{
	struct ls_spi *spi =spi_master_get_devdata(master);
	int ret;

	if(transfer->len == 0)
		return 0;

	spi->tx_buf = transfer->tx_buf;
	spi->rx_buf = transfer->rx_buf;
	spi->tx_len =spi->tx_buf ? transfer->len : 0;
	spi->rx_len =spi->rx_buf ? transfer->len : 0;
	spi->cs_change = transfer->cs_change;

	ret = ls_spi_transfer_one_setup(spi,spi_dev,transfer);
	if(ret){
		dev_err(spi->dev,"SPI transfer setup failed\n");
		return ret;
	}

	return ls_spi_transfer_one_irq(spi);
}

static int ls_spi_unprepare_msg(struct spi_master *master, struct spi_message *msg)
{
	struct ls_spi *spi =spi_master_get_devdata(master);

	ls_spi_disable(spi);

	return 0;
}

static int ls_spi_config(struct ls_spi *spi)
{

	spi_clr_bit(spi,SPI_CFG3,SPI_CFG3_SSMODE);

	return 0;
}


#ifdef CONFIG_OF
static struct ls_spi_info *ls_spi_parse_dt(struct ls_spi *ls_spi)
{
	struct ls_spi_info *lsi;
	struct device *dev =ls_spi->dev;
	unsigned int value;

	lsi = devm_kzalloc(dev, sizeof(*lsi), GFP_KERNEL);
	if(!lsi){
		return ERR_PTR(-ENOMEM);
	}

	if(of_property_read_u32(dev->of_node, "spi-max-frequency", &value)) {
		dev_warn(dev, "spi-max-frequency not specified\n");
		lsi->max_clk = 0;
	} else {
		lsi->max_clk = value;
	}

	if (of_property_read_u32(dev->of_node, "num-cs", &value)) {
		dev_warn(dev, "num_cs not specified\n");
		lsi->num_chipselect = 0;
	} else {
		lsi->num_chipselect = value;
	}

	if (of_property_read_u32(dev->of_node, "ls,bus_num", &value)) {
		dev_warn(dev, "loongson,bus_num not specified\n");
		lsi->bus_num = 0;
	} else {
		lsi->bus_num = value;
	}

	return lsi;
}
#else
static struct ls_spi_info *ls_spi_parse_dt(struct device *dev)
{
	return dev_get_platdata(dev);
}
#endif

static const struct of_device_id ls_spi_match[] = {
	{.compatible = "loongson,ls-spi-v1",},
	{},
};
MODULE_DEVICE_TABLE(of, ls_spi_id_table);

static int  ls_spi_probe(struct platform_device *pdev)
{
	struct ls_spi      *ls_spi;
	struct spi_master  *master;
	struct resource    *res;
	struct ls_spi_info *pdata =dev_get_platdata(&pdev->dev);
	struct device_node *np  = pdev->dev.of_node;
	int ret =0;

	master =spi_alloc_master(&pdev->dev,sizeof(ls_spi));
	if(!master){
		dev_err(&pdev->dev,"Unable to alloc SPI Master\n");
		return -ENOMEM;
	}

	ls_spi = spi_master_get_devdata(master);
	ls_spi->dev = &pdev->dev;
	ls_spi->master = master;

	if(!pdata && np){
		pdata =ls_spi_parse_dt(ls_spi);
		if(IS_ERR(pdata))
			return PTR_ERR(pdata);
	}
	if(!pdata){
		dev_err(&pdev->dev,"platform data missing\n");
		return -ENODEV;
	}

	ls_spi->pdata =pdata;
	master->bus_num =(unsigned int)ls_spi->pdata->bus_num;
	if(master->bus_num <0 || master->bus_num >4){
		dev_err(&pdev->dev,"No this channel, bus_num = %d\n",master->bus_num);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ls_spi->base = devm_ioremap_resource(&pdev->dev, res);
	if(IS_ERR(ls_spi->base)){
		ret =PTR_ERR(ls_spi->base);
		goto err_no_iomap;
	}

	ls_spi->irq = platform_get_irq(pdev, 0);
	if(ls_spi->irq <= 0){
		dev_err(&pdev->dev,"err irq : %d\n",ls_spi->irq);
		ret = -ENOENT;
		goto err_no_irq;
	}

	ret =devm_request_irq(&pdev->dev,ls_spi->irq,ls_spi_irq,0,pdev->name,master);
	if(ret){
		dev_err(&pdev->dev, "irq%d request failed: %d\n", ls_spi->irq,ret);
		goto err_register;
	}

	ls_spi->max_clk =ls_spi->pdata->max_clk;

	platform_set_drvdata(pdev,ls_spi);
	spin_lock_init(&ls_spi->lock);

	//ls_spi_config(ls_spi);

	master->mode_bits = MODE;
	master->num_chipselect =ls_spi->pdata->num_chipselect;
#ifdef CONFIG_OF
	master->dev.of_node = pdev->dev.of_node;
#endif
	master->prepare_message = ls_spi_prepare_message;
	master->unprepare_message = ls_spi_unprepare_msg;
	master->transfer_one = ls_spi_transfer_one;
	master->setup = ls_spi_setup;
	master->bits_per_word_mask = ls_spi_get_bpw_mask(ls_spi);

	ret = spi_register_master(master);
	if(ret < 0 ){
		dev_err(&pdev->dev, "spi master registration failed: %d\n",
			ret);
		goto err_register;
	}
	dev_info(&pdev->dev,"driver initialized\n");

	return 0;

err_register:
	free_irq(ls_spi->irq,ls_spi);
err_no_irq:
	iounmap(ls_spi->base);
err_no_iomap:
	release_resource(ls_spi->base);
	kfree(ls_spi->base);

	spi_master_put(master);
	return 0;
}

static int ls_spi_remove(struct platform_device *dev)
{
	struct ls_spi *ls_spi =platform_get_drvdata(dev);

	spi_master_put(ls_spi->master);

	platform_set_drvdata(dev,NULL);

	free_irq(ls_spi->irq,ls_spi);
	iounmap(ls_spi->base);
	release_resource(ls_spi->base);
	kfree(ls_spi->base);

	kfree(ls_spi);

	dev_info(&dev->dev,"driver remove\n");

	return 0;
}

#ifdef CONFIG_PM

static int ls_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
        struct ls_spi *ls_spi = platform_get_drvdata(pdev);
        unsigned long flags;

        spin_lock_irqsave(&ls_spi->lock, flags);

	spin_unlock_irqrestore(&ls_spi->lock, flags);

        return 0;
}

static int ls_spi_resume(struct platform_device *pdev)
{
        struct ls_spi *ls_spi = platform_get_drvdata(pdev);
        unsigned long   flags;


        spin_lock_irqsave(&ls_spi->lock, flags);

	spin_unlock_irqrestore(&ls_spi->lock, flags);

        return 0;
}

#else
#define ls_spi_suspend NULL
#define ls_spi_resume  NULL
#endif

static struct platform_driver ls_spidrv = {
        .probe      = ls_spi_probe,
        .remove         = ls_spi_remove,
        .suspend        = ls_spi_suspend,
        .resume         = ls_spi_resume,
        .driver         = {
                .name   = "ls-spi-v1",
                .of_match_table = ls_spi_match,
                .owner  = THIS_MODULE,
        },
};

module_platform_driver(ls_spidrv);

MODULE_AUTHOR("Rui Zhao <zhaorui@loongson.cn>");
MODULE_DESCRIPTION("Loongson SPI driver");
MODULE_LICENSE("GPL");

