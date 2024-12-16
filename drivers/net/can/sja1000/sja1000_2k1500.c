#include <linux/acpi.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/can/dev.h>
#include <linux/can/platform/sja1000.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#ifdef __loongarch__
#include <asm/loongarchregs.h>
#endif

#include "sja1000.h"

#define DRV_NAME "sja1000_2k1500"
#define LS2K1500_CAN_CLOCK  50000000
#define SJA1000_FRAC		0x1E
#define SJA1000_INTL		0x1F
#define SJA1000_INTH		0x20

static int nrxdescs = 0x10000;
static int ntxdescs = 0x3000;
static int snd_wait_time = 0x1000;
static int fractiming = 1;
module_param(nrxdescs, int, 0664);
module_param(ntxdescs, int, 0664);
module_param(snd_wait_time, int, 0664);
module_param(fractiming, int, 0664);
static int restart_ms = 100;
module_param(restart_ms, int, 0664);
static int canewl = 0x1;
module_param(canewl, int, 0664);
static int skipnrxonerr = 1;
module_param(skipnrxonerr, int, 0664);
static int resetcnt;
static int fixup = 1;
module_param(fixup, int, 0664);

MODULE_AUTHOR("Chong Qiao <qiaochong@loongson.cn>");
MODULE_DESCRIPTION("LS2K1500-CAN driver for SJA1000 on the platform bus");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_LICENSE("GPL v2");

static void sja1000_dma_rx(struct net_device *dev);
static void sja1000_dma_tx(struct net_device *dev);
irqreturn_t sja1000_interrupt(int irq, void *dev_id);
static netdev_tx_t ls2k1500_start_norm_xmit(struct sk_buff *skb,
					struct net_device *dev);
static void sja1000_write_cmdreg(struct sja1000_priv *priv, u8 val);
static struct net_device_ops ls2k1500_netdev_ops;
static void reset_can_devices(void);
enum {
	DEBUG_CLRERR_ONERR = 1,
	DEBUG_NOBUSOFF = 2,
	DEBUG_ERRFRAME = 4,
	DEBUG_CLRERR_ONIRQ = 8,
	DEBUG_NOAT_TIMEOUT = 0x10,
	DEBUG_PRINTK_TIMEOUT = 0x20,
	DEBUG_RESET = 0x40,
	DEBUG_TIMEOUT = 0x80,
};

typedef struct nb_dma_desc_regs {
	volatile uint32_t desc0;
	volatile uint32_t desc1;
	volatile uint32_t tx_spec;
	volatile uint32_t rx_spec;
	volatile uint64_t tx_addr;
	volatile uint64_t rx_addr;
} __attribute__ ((__aligned__(8))) nb_dma_desc_regs_t;

typedef struct dma_ctrl_conf_regs {
	volatile uint64_t version;
	volatile uint32_t INT_enable;
	volatile uint32_t INT_status;
	volatile uint32_t reset_ctrl;
} dma_ctrl_conf_regs_t;

#define INT_en_offset 0x10
#define INT_status_offset 0x14
#define reset_ctrl_offset 0x18

typedef union dma_chan_ptr_regs {
	uint32_t d32;
	struct {
		unsigned last_ptr:16;
		unsigned this_ptr:16;
	} b;
} dma_chan_ptr_regs_t;

typedef union dma_overtimer_regs {
	uint32_t d32;
	struct {
		unsigned overtimer_en:1;
		unsigned null:15;
		unsigned overtimer:16;
	} b;
} dma_overtimer_regs_t;

typedef union dma_chan_ctrl_regs {
	uint32_t d32;
	struct {
		volatile unsigned poll:1;
		volatile unsigned stop_mode:1;
		volatile unsigned dma_uncache:1;
		volatile unsigned wait_resp:1;
		volatile unsigned tx_req:1;
		volatile unsigned rx_req:1;
		volatile unsigned null:10;
		volatile unsigned stop_ptr:16;
	} b;
} dma_chan_ctrl_regs_t;

typedef struct dma_chan_conf_regs {
	volatile uint64_t base_addr;
	volatile dma_chan_ptr_regs_t chan_ptr_regs;
	volatile dma_overtimer_regs_t overtimer_regs;
	volatile dma_chan_ctrl_regs_t chan_ctrl_regs;
} dma_chan_conf_regs_t;

typedef union dma_desc0_regs {
	uint32_t d32;
	struct {
		unsigned wait_time:16;
		unsigned int_trigger:1;
		unsigned last_des:1;
		unsigned null:13;
		unsigned owner:1;
	} b;
} dma_desc0_regs_t;

typedef union dma_desc1_regs {
	uint32_t d32;
	struct {
		unsigned tx_stride:16;
		unsigned rx_stride:16;
	} b;
} dma_desc1_regs_t;

typedef union dma_desc2_3_regs {
	uint32_t d32;
	struct {
		unsigned fixed:1;
		unsigned cache:1;
		unsigned null:2;
		unsigned size:4;
		unsigned length:4;
		unsigned outstanding:4;
		unsigned count:16;
	} b;
} dma_desc2_3_regs_t;

static DEFINE_SPINLOCK(can_devices_lock);
static LIST_HEAD(can_devices);

struct ls2k1500_priv {
	struct pci_dev *pdmadev;
	void *dma_reg;
	nb_dma_desc_regs_t *dma_rx_descs;
	nb_dma_desc_regs_t *dma_tx_descs;
	dma_addr_t dma_rx_phys;
	dma_addr_t dma_tx_phys;
	int tx_head, tx_done_head;
	int rx_head;
	int dma_txnr;
	int dma_rxnr;
	int nrxdescs;
	int ntxdescs;
	int rxleft;
	int state;
	int didx;
	struct can_frame *cf;
	struct sk_buff *skb;
	int irq;
	long reg_phys;
	u64 freq;
	int fractiming;
	struct can_bittiming_const btc;
	struct kobject	kobj;
	struct sja1000_priv *sjp;
	struct net_device_ops netdev_ops;
	struct can_frame tcf;
	unsigned int debug;
	uint8_t saved_regs[0x21];
	struct list_head list;
	u8 isrc, status, rxerr, txerr;
	u8 last_isrc, last_status;
	int last_rxerr, last_tx, skipnrxonerr;
	uint64_t last_cnt;
};

int ls2k1500_can_smp_call_function_single(int cpu, void (*func) (void *info), void *info,
				int wait)
{
	if (fixup)
		return smp_call_function_single(cpu, func, info, wait);
	func(info);
	return 0;
}
static int get_tx(struct ls2k1500_priv *lp, int n)
{
	return n % lp->ntxdescs;;
}

static int get_rx(struct ls2k1500_priv *lp, int n)
{
	return n % lp->nrxdescs;;
}

#define DMA_INT_EN 0x10
#define DMA_INT_STS 0x14
#define DMA_INT_RST 0x18

#define BASE_ADDR_OFFSET 0x100
#define CHAN_PTR_OFFSET 0x108
#define OVERTIMER_OFFSET 0x10C
#define CHAN_CTRL_OFFSET 0x110

void nb_dma_channel_ctrl(struct sja1000_priv *priv, unsigned channel,
			uint64_t base_addr_phys,
			unsigned last_ptr,
			unsigned overtimer_en, unsigned overtimer,
			unsigned poll, unsigned stop_mode,
			unsigned dma_uncache, int rreq, int treq,
			unsigned stop_ptr)
{
	struct ls2k1500_priv *lp = priv->priv;
	void *dma_reg = lp->dma_reg;
	dma_chan_conf_regs_t chan_conf_regs;
	void *chan_base_addr_addr =
		dma_reg + (BASE_ADDR_OFFSET | (channel << 5));
	void *chan_ptr_addr = dma_reg + (CHAN_PTR_OFFSET | (channel << 5));
	void *chan_overtimer_addr =
		dma_reg + (OVERTIMER_OFFSET | (channel << 5));
	void *chan_ctrl_addr = dma_reg + (CHAN_CTRL_OFFSET | (channel << 5));

	chan_conf_regs.chan_ptr_regs.d32 = 0;
	chan_conf_regs.overtimer_regs.d32 = 0;
	chan_conf_regs.chan_ctrl_regs.d32 = 0;

	chan_conf_regs.base_addr = base_addr_phys;
	chan_conf_regs.chan_ptr_regs.b.last_ptr = last_ptr;
	chan_conf_regs.overtimer_regs.b.overtimer_en = overtimer_en;
	chan_conf_regs.overtimer_regs.b.overtimer = overtimer;

	chan_conf_regs.chan_ctrl_regs.b.poll = poll;
	chan_conf_regs.chan_ctrl_regs.b.stop_mode = stop_mode;
	chan_conf_regs.chan_ctrl_regs.b.dma_uncache = dma_uncache;
	chan_conf_regs.chan_ctrl_regs.b.wait_resp = 1;
	chan_conf_regs.chan_ctrl_regs.b.rx_req = rreq;
	chan_conf_regs.chan_ctrl_regs.b.tx_req = treq;
	chan_conf_regs.chan_ctrl_regs.b.stop_ptr = stop_ptr;

	pr_info("chan_base_addr_addr=0x%lx, desc=0x%lx\n", (long)chan_base_addr_addr,
		(long)chan_conf_regs.base_addr);
	writeq(chan_conf_regs.base_addr, chan_base_addr_addr);
	writel(chan_conf_regs.chan_ptr_regs.d32, chan_ptr_addr);
	writel(chan_conf_regs.overtimer_regs.d32, chan_overtimer_addr);
	writel(chan_conf_regs.chan_ctrl_regs.d32, chan_ctrl_addr);
}

void nb_dma_channel_poll(struct sja1000_priv *priv, unsigned channel,
			unsigned poll, unsigned stop_mode,
			unsigned dma_uncache, int rreq, int treq,
			unsigned stop_ptr)
{
	dma_chan_conf_regs_t chan_conf_regs;
	struct ls2k1500_priv *lp = priv->priv;
	void *chan_ctrl_addr = lp->dma_reg + (CHAN_CTRL_OFFSET | (channel << 5));

	chan_conf_regs.chan_ctrl_regs.d32 = 0;
	chan_conf_regs.chan_ctrl_regs.b.poll = poll;
	chan_conf_regs.chan_ctrl_regs.b.stop_mode = stop_mode;
	chan_conf_regs.chan_ctrl_regs.b.dma_uncache = dma_uncache;
	chan_conf_regs.chan_ctrl_regs.b.wait_resp = 1;
	chan_conf_regs.chan_ctrl_regs.b.rx_req = rreq;
	chan_conf_regs.chan_ctrl_regs.b.tx_req = treq;
	chan_conf_regs.chan_ctrl_regs.b.stop_ptr = stop_ptr;
	writel(chan_conf_regs.chan_ctrl_regs.d32, chan_ctrl_addr);
}

void nb_dma_init_desc(nb_dma_desc_regs_t *desc, unsigned wait_time, unsigned int_trigger, unsigned last_des, unsigned owner, /* desc0 */
		unsigned tx_stride, unsigned rx_stride,	/* desc1 */
		unsigned tx_fixed, unsigned tx_cache, unsigned tx_size, unsigned tx_length, unsigned tx_outstanding, unsigned tx_count,	/* desc2 */
		unsigned rx_fixed, unsigned rx_cache, unsigned rx_size, unsigned rx_length, unsigned rx_outstanding, unsigned rx_count,	/* desc3 */
		uint64_t tx_addr_phys, uint64_t rx_addr_phys)
{
	dma_desc0_regs_t desc0;
	dma_desc1_regs_t desc1;
	dma_desc2_3_regs_t tx_spec;
	dma_desc2_3_regs_t rx_spec;

	desc0.d32 = 0;
	desc1.d32 = 0;
	tx_spec.d32 = 0;
	rx_spec.d32 = 0;

	desc0.b.wait_time = wait_time;
	desc0.b.int_trigger = int_trigger;
	desc0.b.last_des = last_des;
	desc0.b.owner = owner;

	desc1.b.tx_stride = tx_stride;
	desc1.b.rx_stride = rx_stride;

	tx_spec.b.fixed = tx_fixed;
	tx_spec.b.cache = tx_cache;
	tx_spec.b.size = tx_size;
	tx_spec.b.length = tx_length;
	tx_spec.b.outstanding = tx_outstanding;
	tx_spec.b.count = tx_count;

	rx_spec.b.fixed = rx_fixed;
	rx_spec.b.cache = rx_cache;
	rx_spec.b.size = rx_size;
	rx_spec.b.length = rx_length;
	rx_spec.b.outstanding = rx_outstanding;
	rx_spec.b.count = rx_count;

	desc->desc0 = desc0.d32;
	desc->desc1 = desc1.d32;
	desc->tx_spec = tx_spec.d32;
	desc->rx_spec = rx_spec.d32;
	desc->tx_addr = tx_addr_phys;
	desc->rx_addr = rx_addr_phys;
}

static void _dma_can_rx_init(struct sja1000_priv *priv, int rerun)
{
	struct ls2k1500_priv *lp = priv->priv;
	long canreg = lp->reg_phys + 0x10;
	int dmachan = lp->dma_rxnr;
	int cached = 1;
	int ndesc = lp->nrxdescs;
	nb_dma_desc_regs_t *dma_rx_descs = lp->dma_rx_descs;
	dma_addr_t canrxbuf_phys =
		lp->dma_rx_phys + ndesc * sizeof(dma_rx_descs[0]);
	int i;

	for (i = 0; i < ndesc; i++) {
		nb_dma_init_desc(dma_rx_descs + i, 0, 1, 0, 1,	/* desc0   Int enable, owner DMA */
				0, 0,	/* desc1    stide: tx 1, rx 1 */
				0, cached, 0, 0, 0, 0x0,	/* desc2  tx cached, size 1, length 1, outstanding 1, count 1 */
				1, 0, 0, 0, 0, 0,	/* desc3  rx cached, size 1, length 1, outstanding 1, count 1 */
				canrxbuf_phys + i, canreg);
	}

	if (rerun)
		nb_dma_channel_ctrl(priv, dmachan, lp->dma_rx_phys, ndesc - 1, 0, 0, 1, 0, !cached, 1, 0, ndesc - 1);	/* DMA channel 0 start */
}

static void _dma_can_send_init(struct sja1000_priv *priv)
{
	struct ls2k1500_priv *lp = priv->priv;
	int dmachan = lp->dma_txnr;
	nb_dma_desc_regs_t *dma_tx_descs = lp->dma_tx_descs;
	int cached = 1;
	int i;
	int ndesc = lp->ntxdescs;

	for (i = 0; i < ndesc; i++)
		memset(dma_tx_descs + i, 0, sizeof(*dma_tx_descs));
	nb_dma_channel_ctrl(priv, dmachan, lp->dma_tx_phys, ndesc - 1, 0, 0, 1,
			0, !cached, 0, 0, ndesc - 1);
}

void can_ls2k1500_dma_tx_poll(void *arg)
{
	struct sja1000_priv *priv = arg;
	struct ls2k1500_priv *lp = priv->priv;
	int dmachan = lp->dma_txnr;
	int cached = 1;

	nb_dma_channel_poll(priv, dmachan, 1, 0, !cached, 0, 0, lp->ntxdescs - 1);	/* DMA channel 0 start */
}
static netdev_tx_t ls2k1500_start_dma_xmit(struct sk_buff *skb,
				struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);
	struct ls2k1500_priv *lp = priv->priv;
	struct can_frame *cf = (struct can_frame *)skb->data;
	uint8_t fi;
	uint8_t dlc;
	canid_t id;
	uint8_t dreg;
	int dma_len;
	int i;
	nb_dma_desc_regs_t *dma_descs;
	long canbase = lp->reg_phys;
	char *cantxbuf = (char *)(lp->dma_tx_descs + ntxdescs);
	char *canbuf;
	dma_addr_t cantxbuf_phys =
		lp->dma_tx_phys + ntxdescs * sizeof(lp->dma_tx_descs[0]);
	dma_addr_t canbuf_phys;
	int cached = 1;

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(dev);
	if (get_tx(lp, lp->tx_head + 3) == lp->tx_done_head)
		return NETDEV_TX_BUSY;

	fi = dlc = cf->can_dlc;
	id = cf->can_id;

	if (id & CAN_RTR_FLAG)
		fi |= SJA1000_FI_RTR;

	canbuf = cantxbuf + get_tx(lp, lp->tx_head) * 16;
	canbuf_phys = cantxbuf_phys + get_tx(lp, lp->tx_head) * 16;

	fi = dlc;

	if (id & CAN_RTR_FLAG)
		fi |= SJA1000_FI_RTR;

	dreg = SJA1000_FI;

	if (id & CAN_EFF_FLAG) {
		dma_len = 5 + dlc;
		/* extended frame format (EFF) */
		fi |= SJA1000_FI_FF;
		canbuf[0] = fi;
		canbuf[1] = (id & 0x1fe00000) >> 21;
		canbuf[2] = (id & 0x001fe000) >> 13;
		canbuf[3] = (id & 0x00001fe0) >> 5;
		canbuf[4] = (id & 0x0000001f) << 3;
		for (i = 5; i < dma_len; i++) {
			canbuf[i] = cf->data[i - 5];
		}
	} else {
		dma_len = 3 + dlc;
		/* standard frame format (SFF) */
		canbuf[0] = fi;
		canbuf[1] = (id & 0x000007f8) >> 3;
		canbuf[2] = (id & 0x00000007) << 5;
		for (i = 3; i < dma_len; i++) {
			canbuf[i] = cf->data[i - 3];
		}
	}

	dma_descs = lp->dma_tx_descs + get_tx(lp, lp->tx_head);

	nb_dma_init_desc(dma_descs, snd_wait_time, 1, 0, 1,	/* desc0   Int enable, owner DMA */
			0, 0,	/* desc1    stide: tx 1, rx 1 */
			0, 0, 0, 0, 0, 0,	/* desc2  tx fixed, cached, size 1, length 1, outstanding 1, count 100 */
			0, cached, 0, 0, 0, 0,	/* desc3  rx fixed, cached, size 1, length 1, outstanding 1, count 1 */
			canbase + dreg, canbuf_phys);

	dma_descs = lp->dma_tx_descs + get_tx(lp, lp->tx_head + 1);

	nb_dma_init_desc(dma_descs, 0, 1, 0, 1,	/* desc0   Int enable, owner DMA */
			0, 0,	/* desc1    stide: tx 1, rx 1 */
			0, 0, 0, 0, 0, dma_len - 2,	/* desc2  tx fixed, cached, size 1, length 1, outstanding 1, count 100 */
			0, cached, 0, 0, 0, dma_len - 2,  /* desc3  rx fixed, cached, size 1, length 1, outstanding 1, count 1 */
			canbase + dreg + 1, canbuf_phys + 1);

	dma_descs = lp->dma_tx_descs + get_tx(lp, lp->tx_head + 2);
	canbuf += dma_len;
	canbuf_phys += dma_len;

	canbuf[0] = CMD_TR;
	dma_len = 1;
	dreg = SJA1000_CMR;
	nb_dma_init_desc(dma_descs, 0, 1, 0, 1,	/* desc0   Int enable, owner DMA */
			0, 0,	/* desc1    stide: tx 1, rx 1 */
			0, 0, 0, 0, 0, dma_len - 1,	/* desc2  tx fixed, cached, size 1, length 1, outstanding 1, count 100 */
			0, cached, 0, 0, 0, dma_len - 1, /* desc3  rx fixed, cached, size 1, length 1, outstanding 1, count 1 */
			canbase + dreg, canbuf_phys);
	dma_descs++;

	lp->tx_head = get_tx(lp, lp->tx_head + 3);

	ls2k1500_can_smp_call_function_single(0, can_ls2k1500_dma_tx_poll, priv, 0);
	return NETDEV_TX_OK;
}

static netdev_tx_t ls2k1500_start_norm_xmit(struct sk_buff *skb,
				struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);
	struct ls2k1500_priv *lp = priv->priv;
	struct can_frame *cf = (struct can_frame *)skb->data;
	lp->tcf = *cf;

	return ls2k1500_netdev_ops.ndo_start_xmit(skb, dev);
}


static irqreturn_t sja1000_dma_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct sja1000_priv *priv = netdev_priv(dev);
	struct ls2k1500_priv *lp = priv->priv;
	int sts = readl(lp->dma_reg + DMA_INT_STS);
	int en = readl(lp->dma_reg + DMA_INT_EN);

	sts &= en;
	if (lp->dma_rxnr < 32 && (sts & (2 << (lp->dma_rxnr * 2)))) {
		writel((2 << (lp->dma_rxnr * 2)), lp->dma_reg + DMA_INT_STS);
		sja1000_dma_rx(dev);
	} else if (lp->dma_txnr < 32 && (sts & (2 << (lp->dma_txnr * 2)))) {
		writel((2 << (lp->dma_txnr * 2)), lp->dma_reg + DMA_INT_STS);
		sja1000_dma_tx(dev);
	} else {
		return IRQ_NONE;
	}
	return IRQ_HANDLED;
}


enum {
	STATE_IDLE = 0,
	STATE_EFF_ID1,
	STATE_EFF_ID2,
	STATE_EFF_ID3,
	STATE_EFF_ID4,
	STATE_NORM_ID1,
	STATE_NORM_ID2,
	STATE_DATA,
};

static void sja1000_dma_rx(struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct ls2k1500_priv *lp = priv->priv;
	nb_dma_desc_regs_t *dma_rx_descs = lp->dma_rx_descs;
	int i = lp->rx_head;
	uint8_t *canrxbuf = (char *)(dma_rx_descs + lp->nrxdescs);
	uint8_t d;

	while (1) {
		if ((dma_rx_descs[i].desc0 & 0x80000000) == 0x80000000) {
			lp->rx_head = i;
			break;
		}
		/*
		 * 6'b1_1_1_000  : data_for_fifo = {1'b1, rtr2, 2'h0, data_len};  // extended mode, extended format header
		 * 6'b1_1_1_001  : data_for_fifo = id[28:21];                     // extended mode, extended format header
		 * 6'b1_1_1_010  : data_for_fifo = id[20:13];                     // extended mode, extended format header
		 * 6'b1_1_1_011  : data_for_fifo = id[12:5];                      // extended mode, extended format header
		 * 6'b1_1_1_100  : data_for_fifo = {id[4:0], 3'h0};               // extended mode, extended format header
		 * 6'b1_1_0_000  : data_for_fifo = {1'b0, rtr1, 2'h0, data_len};  // extended mode, standard format header
		 * 6'b1_1_0_001  : data_for_fifo = id[10:3];                      // extended mode, standard format header
		 * 6'b1_1_0_010  : data_for_fifo = {id[2:0], rtr1, 4'h0};         // extended mode, standard format header
		 default       : data_for_fifo = tmp_fifo[data_cnt - {1'b0, header_len}]; // data
		*/
		switch (lp->state) {
		case STATE_IDLE:
			lp->skb = alloc_can_skb(dev, &lp->cf);
			d = canrxbuf[i];
			if (d & SJA1000_FI_FF) {
				lp->cf->can_id = CAN_EFF_FLAG;
				lp->state = STATE_EFF_ID1;
			} else {
				lp->cf->can_id = 0;
				lp->state = STATE_NORM_ID1;
			}

			lp->cf->can_dlc = get_can_dlc(d & 0x0F);
			lp->didx = 0;
			break;
		case STATE_EFF_ID1:
			d = canrxbuf[i];
			lp->cf->can_id |= d << 21;
			lp->state = STATE_EFF_ID2;
			break;
		case STATE_EFF_ID2:
			d = canrxbuf[i];
			lp->cf->can_id |= d << 13;
			lp->state = STATE_EFF_ID3;
			break;
		case STATE_EFF_ID3:
			d = canrxbuf[i];
			lp->cf->can_id |= d << 5;
			lp->state = STATE_EFF_ID4;
			break;
		case STATE_EFF_ID4:
			d = canrxbuf[i];
			lp->cf->can_id |= d >> 3;
			if (lp->cf->can_dlc)
				lp->state = STATE_DATA;
			else
				goto data_done;
			break;
		case STATE_NORM_ID1:
			d = canrxbuf[i];
			lp->cf->can_id |= (d << 3);
			lp->state = STATE_NORM_ID2;
			break;
		case STATE_NORM_ID2:
			d = canrxbuf[i];
			lp->cf->can_id |= (d >> 5);
			if (lp->cf->can_dlc)
				lp->state = STATE_DATA;
			else
				goto data_done;
			break;
		default:
			d = canrxbuf[i];
			lp->cf->data[lp->didx++] = d;
			if (lp->didx == lp->cf->can_dlc) {
			data_done:
				lp->state = STATE_IDLE;
				stats->rx_packets++;
				stats->rx_bytes += lp->cf->can_dlc;
				netif_rx(lp->skb);
				can_led_event(dev, CAN_LED_EVENT_RX);
			}
			break;
		}
		dma_rx_descs[i].desc0 |= 0x80000000;
		i = get_rx(lp, i + 1);
	}
}

static void sja1000_dma_tx(struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct ls2k1500_priv *lp = priv->priv;

	while (lp->tx_done_head != lp->tx_head) {
		int l = get_tx(lp, lp->tx_done_head + 1);
		int i = get_tx(lp, lp->tx_done_head + 2);
		if ((lp->dma_tx_descs[i].desc0 & 0x80000000) == 0x80000000) {
			break;
		}
		stats->tx_bytes += (lp->dma_tx_descs[l].rx_spec >> 16) + 1;
		stats->tx_packets++;
		lp->tx_done_head = get_tx(lp, lp->tx_done_head + 3);
	}
	can_get_echo_skb(dev, 0);
	if (get_tx(lp, lp->tx_head + 3) != lp->tx_done_head)
		netif_wake_queue(dev);
}

static u8 can_ls2k1500_read_reg8(const struct sja1000_priv *priv, int reg)
{
	struct ls2k1500_priv *lp = priv->priv;
	u8 d;
	if (lp->dma_rxnr < 32 && reg >= SJA1000_FI && reg < SJA1000_EFF_BUF + 8)
		return 0;
	d = ioread8(priv->reg_base + reg);
	if (!fixup)
		return d;
	if (reg == SJA1000_IR && (d || lp->last_tx)) {
		u8 isrc = d;
		u8 status = ioread8(priv->reg_base + SJA1000_SR);

		if (lp->last_tx && !(isrc & IRQ_TI)) {
			if ((status & SR_TBS) && netif_queue_stopped(lp->sjp->dev))
				isrc |= IRQ_TI;
		}
		if (isrc & IRQ_TI)  {
			lp->last_tx = 0;
			if (!netif_queue_stopped(lp->sjp->dev))
				isrc &= ~IRQ_TI;
		}
		if (isrc & IRQ_EI)
			lp->last_rxerr = lp->skipnrxonerr;
		if (isrc & IRQ_RI) {
			while ((status & SR_RBS) && lp->last_rxerr) {
				sja1000_write_cmdreg(priv, CMD_RRB);
				lp->last_rxerr--;
				status = ioread8(priv->reg_base + SJA1000_SR);
			}
		}
		if (isrc & IRQ_EI) {
			long flags;
			u8 mode, ecc;

			lp->rxerr = ioread8(priv->reg_base + SJA1000_TXERR);
			lp->txerr = ioread8(priv->reg_base + SJA1000_RXERR);
			spin_lock_irqsave(&priv->cmdreg_lock, flags);
			mode = ioread8(priv->reg_base + SJA1000_MOD) & ~MOD_RM;
			if (status & SR_BS) {
				iowrite8(MOD_RM, priv->reg_base + SJA1000_MOD);
				iowrite8(0xff, priv->reg_base + SJA1000_TXERR);
				iowrite8(mode, priv->reg_base + SJA1000_MOD);
				udelay(1);
			}
			iowrite8(MOD_RM, priv->reg_base + SJA1000_MOD);
			iowrite8(0, priv->reg_base + SJA1000_TXERR);
			iowrite8(0, priv->reg_base + SJA1000_RXERR);
			iowrite8(mode, priv->reg_base + SJA1000_MOD);
			spin_unlock_irqrestore(&priv->cmdreg_lock, flags);
			ecc = ioread8(priv->reg_base + SJA1000_ECC);
			/* reset mode will clear logic need_to_tx, we resend */
			if (lp->last_tx) {
				unsigned long flags;

				spin_lock_irqsave(&priv->cmdreg_lock, flags);
				lp->last_tx = lp->last_tx + 1 ?: 1;
				if (get_cycles() & 1)
					iowrite8(0x80 | CMD_AT, priv->reg_base + SJA1000_CMR);
				iowrite8(0x80 | CMD_TR, priv->reg_base + SJA1000_CMR);
				spin_unlock_irqrestore(&priv->cmdreg_lock, flags);
			}
		}
		d = isrc;
		lp->status |= status;
		lp->isrc |= isrc;
	} else if (reg == SJA1000_SR) {
		lp->status |= d;
	} else if (reg == SJA1000_TXERR) {
		d = (lp->isrc & IRQ_EI) ? lp->txerr : ioread8(priv->reg_base + SJA1000_TXERR);
	} else if (reg == SJA1000_RXERR) {
		d = (lp->isrc & IRQ_EI) ? lp->rxerr : ioread8(priv->reg_base + SJA1000_RXERR);
	}
	return d;
}

static void can_ls2k1500_write_reg8(const struct sja1000_priv *priv, int reg,
				u8 val)
{
	struct ls2k1500_priv *lp = priv->priv;

	if (reg == SJA1000_MOD) {
		val = lp->fractiming ? val | 0x40 : val & ~0x40;
		val = lp->dma_rxnr < 32 ? val | 0x80 : val & ~0x80;
	} else if (reg == SJA1000_IER) {
		if (lp->dma_txnr < 32)
			val &= ~IRQ_TI;
		if (lp->dma_rxnr < 32)
			val &= ~IRQ_RI;
		val &= ~IRQ_WUI;
	} else if (reg == SJA1000_INTL)
		return;
	else if (reg == SJA1000_CMR) {
		if (val & CMD_TR)
			lp->last_tx = 1;
		val |= 0x80;
	}

	if (reg <= 0x20)
		lp->saved_regs[reg] = val;

	iowrite8(val, priv->reg_base + reg);
}

static void ls2k1500_populate_of(struct sja1000_priv *priv,
				struct device_node *of)
{
	int err;
	u32 prop;

	priv->write_reg = can_ls2k1500_write_reg8;
	priv->read_reg = can_ls2k1500_read_reg8;

	err = of_property_read_u32(of, "nxp,external-clock-frequency", &prop);
	if (!err)
		priv->can.clock.freq = prop / 2;
	else
		priv->can.clock.freq = LS2K1500_CAN_CLOCK;	/* default */

	err = of_property_read_u32(of, "nxp,tx-output-mode", &prop);
	if (!err)
		priv->ocr |= prop & OCR_MODE_MASK;
	else
		priv->ocr |= OCR_MODE_NORMAL;	/* default */

	err = of_property_read_u32(of, "nxp,tx-output-config", &prop);
	if (!err)
		priv->ocr |= (prop << OCR_TX_SHIFT) & OCR_TX_MASK;
	else
		priv->ocr |= OCR_TX0_PULLDOWN;	/* default */

	err = of_property_read_u32(of, "nxp,clock-out-frequency", &prop);
	if (!err && prop) {
		u32 divider = priv->can.clock.freq * 2 / prop;

		if (divider > 1)
			priv->cdr |= divider / 2 - 1;
		else
			priv->cdr |= CDR_CLKOUT_MASK;
	} else {
		priv->cdr |= CDR_CLK_OFF;	/* default */
	}

	if (!of_property_read_bool(of, "nxp,no-comparator-bypass"))
		priv->cdr |= CDR_CBP;	/* default */
}

static void ls2k1500_populate(struct sja1000_priv *priv,
			struct sja1000_platform_data *pdata,
			unsigned long resource_mem_flags)
{
	/* The CAN clock frequency is half the oscillator clock frequency */
	priv->can.clock.freq = pdata->osc_freq / 2;
	priv->ocr = pdata->ocr;
	priv->cdr = pdata->cdr;
	priv->write_reg = can_ls2k1500_write_reg8;
	priv->read_reg = can_ls2k1500_read_reg8;
}

static void can_ls2k1500_dma_rx_init(void *arg)
{
	struct sja1000_priv *priv = arg;
	uint32_t val;
	struct ls2k1500_priv *lp = priv->priv;
	dma_chan_ctrl_regs_t chan_ctrl_regs;

	chan_ctrl_regs.d32 = readl(lp->dma_reg + (CHAN_CTRL_OFFSET | (lp->dma_rxnr << 5)));
	if (!chan_ctrl_regs.b.poll) {
		val = 1 << lp->dma_rxnr;
		writel(val, lp->dma_reg + DMA_INT_RST);
		readl(lp->dma_reg + DMA_INT_RST);
		udelay(100);
		writel(0, lp->dma_reg + DMA_INT_RST);
		_dma_can_rx_init(priv, 1);
		lp->rx_head = 0;
		lp->state = STATE_IDLE;
	} else {
		dma_chan_ptr_regs_t chan_ptr_regs;
		_dma_can_rx_init(priv, 1);
		chan_ptr_regs.d32 = readl(lp->dma_reg + (CHAN_PTR_OFFSET | (lp->dma_rxnr << 5)));
		lp->rx_head = chan_ptr_regs.b.this_ptr;
		lp->state = STATE_IDLE;
	}
	val = readl(lp->dma_reg + DMA_INT_EN);
	val |= 2 << (lp->dma_rxnr * 2);
	writel(val, (lp->dma_reg + DMA_INT_EN));
}

static void can_ls2k1500_dma_tx_init(void *arg)
{
	struct sja1000_priv *priv = arg;
	struct ls2k1500_priv *lp = priv->priv;
	uint32_t val;

	val = 1 << lp->dma_txnr;
	writel(val, lp->dma_reg + DMA_INT_RST);
	readl(lp->dma_reg + DMA_INT_RST);
	udelay(100);
	writel(0, lp->dma_reg + DMA_INT_RST);
	_dma_can_send_init(priv);
	lp->tx_head = lp->tx_done_head = 0;
	val = readl(lp->dma_reg + DMA_INT_EN);
	val |= 2 << (lp->dma_txnr * 2);
	writel(val, (lp->dma_reg + DMA_INT_EN));
}

#define CAN_CALC_SYNC_SEG	1
struct can_bittiming_const BTC = {
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max   = 4,
	.brp_min   = 1,
	.brp_max   = 64,
	.brp_inc   = 1,
};

static int sja1000_set_bittiming(struct net_device *dev)
{
	struct sja1000_priv *priv = netdev_priv(dev);
	struct can_bittiming *bt = &priv->can.bittiming;
	u8 btr0, btr1, btrd_inter_h, btrd_inter_l, btrd_frac, val;
	struct ls2k1500_priv *lp = priv->priv;

	if (!lp->fractiming) {
		btr0 = ((bt->brp - 1) & 0x3f) | (((bt->sjw - 1) & 0x3) << 6);
		btr1 = ((bt->prop_seg + bt->phase_seg1 - 1) & 0xf) |
			(((bt->phase_seg2 - 1) & 0x7) << 4);
		if (priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES)
			btr1 |= 0x80;

		netdev_info(dev, "setting BTR0=0x%02x BTR1=0x%02x\n", btr0, btr1);

		val = priv->read_reg(priv, SJA1000_MOD);
		priv->write_reg(priv, SJA1000_MOD, val | 0x1);
		priv->write_reg(priv, SJA1000_BTR0, btr0);
		priv->write_reg(priv, SJA1000_BTR1, btr1);
	} else {
		int div = ((lp->freq / 2 * 0x200) >> 32) + 1;
		u64 brp = bt->brp * div;
		btr0 = (((bt->sjw - 1) & 0x3) << 6);
		btr1 = ((bt->prop_seg + bt->phase_seg1 - 1) & 0xf) |
			(((bt->phase_seg2 - 1) & 0x7) << 4);
		btrd_inter_h = brp >> 16;
		btrd_inter_l = brp >> 8;
		btrd_frac = brp & 0xff;
		val = priv->read_reg(priv, SJA1000_MOD);
		priv->write_reg(priv, SJA1000_MOD, val | 0x41);
		priv->write_reg(priv, SJA1000_INTH, btrd_inter_h);
		lp->saved_regs[SJA1000_INTL] = btrd_inter_l;
		iowrite8(btrd_inter_l, priv->reg_base + SJA1000_INTL);
		priv->write_reg(priv, SJA1000_FRAC, btrd_frac);

		priv->write_reg(priv, SJA1000_BTR0, btr0);
		priv->write_reg(priv, SJA1000_BTR1, btr1);
	}

	return 0;
}
struct fractiming_attribute {
	struct attribute attr;
	ssize_t (*show)(struct ls2k1500_priv *foo, struct fractiming_attribute *attr, char *buf);
	ssize_t (*store)(struct ls2k1500_priv *foo, struct fractiming_attribute *attr, const char *buf, size_t count);
};
#define to_fractiming_attr(x) container_of(x, struct fractiming_attribute, attr)
#define to_ls2k1500_priv(x)  container_of(x, struct ls2k1500_priv, kobj)

static ssize_t dmarx_show(struct ls2k1500_priv *lp,
			struct fractiming_attribute *attr, char *buf)
{
	if (strcmp(attr->attr.name, "dmarx") == 0)
		return sprintf(buf, "%u\n", lp->dma_rxnr);
	else if (strcmp(attr->attr.name, "dmatx") == 0)
		return sprintf(buf, "%u\n", lp->dma_txnr);
	else if (strcmp(attr->attr.name, "restart_ms") == 0)
		return sprintf(buf, "%u\n", lp->sjp->can.restart_ms);
	else if (strcmp(attr->attr.name, "debug") == 0) {
		int i, l = 0;
		const char *dstr[] = { "clrerr_onerr", "nobusoff", "errframe", "clrerr_onirq", "noat_timeout",
			  "printk_timeout", "reset", "timeout" };
			for (i = 0; i < ARRAY_SIZE(dstr); i++)
				l += sprintf (buf + l, "%s = %d,", dstr[i], (lp->debug >> i) & 1);
		l += sprintf(buf + l, "\n0x%08x\n", lp->debug);
		l += sprintf(buf + l, "cnt:0x%llx last_tx=0x%x last_isrc=0x%x last_status=0x%x\n",
			lp->last_cnt, lp->last_tx, lp->last_isrc, lp->last_status);
			return l;
	} else if (strcmp(attr->attr.name, "skipnrxonerr") == 0)
		return sprintf(buf, "%u\n", lp->skipnrxonerr);
	else if (strcmp(attr->attr.name, "bt") == 0) {
		struct can_bittiming *bt = &lp->sjp->can.bittiming;;
		return sprintf(buf, "bitrate=%u, sample_point=%u\n"
			"tq=%u, prop_seg=%u\n"
			"phase_seg1=%u, phase_seg2=%u\n"
			"sjw=%u, brp=%u\n", bt->bitrate,
			bt->sample_point, bt->tq, bt->prop_seg,
			bt->phase_seg1, bt->phase_seg2, bt->sjw,
			bt->brp);
	} else if (strcmp(attr->attr.name, "clock") == 0)
		return sprintf(buf, "%u\n", lp->sjp->can.clock.freq);
	else if (strcmp(attr->attr.name, "brp_max") == 0)
		return sprintf(buf, "%u\n", lp->btc.brp_max);
	else if (strcmp(attr->attr.name, "brp_min") == 0)
		return sprintf(buf, "%u\n", lp->btc.brp_min);
	else if (strcmp(attr->attr.name, "fractiming") == 0) {
		int div = ((lp->freq / 2 * 0x200) >> 32) + 1;
		int var =
			(lp->sjp->can.clock.freq == lp->freq / 2 * 0x200 /div) ? 1 : 0;
		return sprintf(buf, "%d\n", var);
	} else if (strcmp(attr->attr.name, "timeo") == 0)
		return sprintf(buf, "%d\n", jiffies_to_msecs(lp->sjp->dev->watchdog_timeo));
	else if (strcmp(attr->attr.name, "queue") == 0)
		return sprintf(buf, "queue state = %d\n", !netif_queue_stopped(lp->sjp->dev));
	else if (strcmp(attr->attr.name, "carrier") == 0)
		return sprintf(buf, "carrier = %d\n", netif_carrier_ok(lp->sjp->dev));
	else if (strcmp(attr->attr.name, "canewl") == 0)
		return sprintf(buf, "canewl = 0x%x\n", lp->sjp->read_reg(lp->sjp, SJA1000_EWL));
	else if (strcmp(attr->attr.name, "rxhead") == 0)
		return sprintf(buf, "0x%x\n", lp->rx_head);
	else if (strcmp(attr->attr.name, "txhead") == 0)
		return sprintf(buf, "0x%x\n", lp->tx_head);
	else if (strcmp(attr->attr.name, "txdonehead") == 0)
		return sprintf(buf, "0x%x\n", lp->tx_done_head);
	else if (strcmp(attr->attr.name, "canreset") == 0)
		return sprintf(buf, "0x%x\n", resetcnt);
	else
		return sprintf(buf, "error\n");
}

static ssize_t dmarx_store(struct ls2k1500_priv *lp, struct fractiming_attribute *attr,
			 const char *buf, size_t count)
{
	int ret;
	unsigned long long var;
	int olddma, newdma;

	ret = kstrtoull(buf, 0, &var);
	if (ret < 0)
		return ret;
	if (strcmp(attr->attr.name, "restart_ms") == 0)
		lp->sjp->can.restart_ms = var;
	else if (strcmp(attr->attr.name, "debug") == 0)
		lp->debug = var;
	else if (strcmp(attr->attr.name, "skipnrxonerr") == 0)
		lp->skipnrxonerr = var;
	else if (strcmp(attr->attr.name, "clock") == 0) {
		lp->sjp->can.clock.freq = var;
		lp->freq = var * 2;
	} else if (strcmp(attr->attr.name, "brp_max") == 0)
		lp->btc.brp_max = var;
	else if (strcmp(attr->attr.name, "brp_min") == 0)
		lp->btc.brp_min = var;
	else if (strcmp(attr->attr.name, "timeo") == 0)
		lp->sjp->dev->watchdog_timeo = msecs_to_jiffies(var);
	else if (strcmp(attr->attr.name, "queue") == 0) {
		if (var)
			netif_wake_queue(lp->sjp->dev);
		else
			netif_stop_queue(lp->sjp->dev);
	} else if (strcmp(attr->attr.name, "carrier") == 0) {
		if (var)
			netif_carrier_on(lp->sjp->dev);
		else
			netif_carrier_off(lp->sjp->dev);
	}
	else if (strcmp(attr->attr.name, "fractiming") == 0) {
		if (var) {
			int div = ((lp->freq / 2 * 0x200) >> 32) + 1;
			lp->fractiming = 1;
			lp->sjp->can.clock.freq = lp->freq / 2 * 0x200 / div;
			lp->btc.brp_max = 0x1000000 / div;
			lp->btc.brp_min = 0x200 / div;
		} else {
			lp->fractiming = 0;
			lp->sjp->can.clock.freq = lp->freq / 2;
			lp->btc.brp_max = 64;
			lp->btc.brp_min = 1;
		}
	} else if (strncmp(attr->attr.name, "dma", 3) == 0) {
		u8 d;
		olddma = lp->dma_rxnr < 32 || lp->dma_txnr < 32;
		if (strcmp(attr->attr.name, "dmarx") == 0) {
			lp->dma_rxnr = var;
			d = ioread8(lp->sjp->reg_base + SJA1000_MOD);
			lp->sjp->write_reg(lp->sjp, SJA1000_MOD, d | MOD_RM);
			if (lp->dma_rxnr < 32) {
				if (!lp->dma_rx_descs)
					lp->dma_rx_descs =
						dma_alloc_coherent(&lp->pdmadev->
								dev,
								sizeof(*lp->
									dma_rx_descs)
								* nrxdescs +
								nrxdescs,
								&lp->dma_rx_phys,
								GFP_KERNEL);
				ls2k1500_can_smp_call_function_single(0,
							can_ls2k1500_dma_rx_init,
							lp->sjp, 1);
			} else {
			}
			lp->sjp->write_reg(lp->sjp, SJA1000_MOD, d);
		} else {
			lp->dma_txnr = var;
			if (lp->dma_txnr < 32) {
				lp->netdev_ops.ndo_start_xmit =
					ls2k1500_start_dma_xmit;
				if (!lp->dma_tx_descs)
					lp->dma_tx_descs =
						dma_alloc_coherent(&lp->pdmadev->
								dev,
								sizeof(*lp->
									dma_tx_descs)
								* ntxdescs +
								nrxdescs * 16,
								&lp->dma_tx_phys,
								GFP_KERNEL);
				ls2k1500_can_smp_call_function_single(0,
							can_ls2k1500_dma_tx_init,
							lp->sjp, 1);
			} else {
				lp->netdev_ops.ndo_start_xmit =
					ls2k1500_start_norm_xmit;
			}
		}
		d = ioread8(lp->sjp->reg_base + SJA1000_IER);
		lp->sjp->write_reg(lp->sjp, SJA1000_IER, d | IRQ_TI | IRQ_RI);
		newdma = lp->dma_rxnr < 32 || lp->dma_txnr < 32;
		if (!olddma && newdma) {
			if (!request_irq(lp->irq, sja1000_dma_interrupt,
				lp->sjp->irq_flags, lp->sjp->dev->name,
						(void *)lp->sjp->dev))
			irq_set_affinity_hint(lp->irq, get_cpu_mask(0));
		} else if (olddma && !newdma) {
			irq_set_affinity_hint(lp->irq, NULL);
			free_irq(lp->irq, (void *)lp->sjp->dev);
		}
	} else if (strcmp(attr->attr.name, "canewl") == 0) {
		long flags;
		u8 mode;
		struct sja1000_priv *priv = lp->sjp;

		spin_lock_irqsave(&priv->cmdreg_lock, flags);
		mode = priv->read_reg(priv, SJA1000_MOD);
		priv->write_reg(priv, SJA1000_MOD, MOD_RM);
		priv->write_reg(priv, SJA1000_EWL, var);
		priv->write_reg(priv, SJA1000_MOD, mode);
		spin_unlock_irqrestore(&priv->cmdreg_lock, flags);
	} else if (strcmp(attr->attr.name, "canreset") == 0) {
		reset_can_devices();
	} else if (strcmp(attr->attr.name, "rxhead") == 0) {
		lp->rx_head = var;
	} else if (strcmp(attr->attr.name, "txhead") == 0) {
		lp->tx_head = var;
	} else if (strcmp(attr->attr.name, "txdonehead") == 0) {
		lp->tx_done_head = var;
	}

	return count;
}

static ssize_t foo_attr_show(struct kobject *kobj,
			     struct attribute *attr,
			     char *buf)
{
	struct fractiming_attribute *attribute;
	struct ls2k1500_priv *foo;

	attribute = to_fractiming_attr(attr);
	foo = to_ls2k1500_priv(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(foo, attribute, buf);
}

static ssize_t foo_attr_store(struct kobject *kobj,
			      struct attribute *attr,
			      const char *buf, size_t len)
{
	struct fractiming_attribute *attribute;
	struct ls2k1500_priv *foo;

	attribute = to_fractiming_attr(attr);
	foo = to_ls2k1500_priv(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(foo, attribute, buf, len);
}
static const struct sysfs_ops foo_sysfs_ops = {
	.show = foo_attr_show,
	.store = foo_attr_store,
};

static struct fractiming_attribute fractiming_attribute =
	__ATTR(fractiming, 0664, dmarx_show, dmarx_store);
static struct fractiming_attribute clock_attribute =
	__ATTR(clock, 0664, dmarx_show, dmarx_store);
static struct fractiming_attribute brp_max_attribute =
	__ATTR(brp_max, 0664, dmarx_show, dmarx_store);
static struct fractiming_attribute brp_min_attribute =
	__ATTR(brp_min, 0664, dmarx_show, dmarx_store);
static struct fractiming_attribute bt_attribute =
	__ATTR(bt, 0664, dmarx_show, NULL);
static struct fractiming_attribute dmarx_attribute =
	__ATTR(dmarx, 0664, dmarx_show, dmarx_store);
static struct fractiming_attribute dmatx_attribute =
	__ATTR(dmatx, 0664, dmarx_show, dmarx_store);
static struct fractiming_attribute restart_ms_attribute =
	__ATTR(restart_ms, 0664, dmarx_show, dmarx_store);
static struct fractiming_attribute debug_attribute =
	__ATTR(debug, 0664, dmarx_show, dmarx_store);
static struct fractiming_attribute timeo_attribute =
	__ATTR(timeo, 0664, dmarx_show, dmarx_store);
static struct fractiming_attribute skipnrxonerr_attribute =
	__ATTR(skipnrxonerr, 0664, dmarx_show, dmarx_store);
static struct fractiming_attribute queue_attribute =
	__ATTR(queue, 0664, dmarx_show, dmarx_store);
static struct fractiming_attribute carrier_attribute =
	__ATTR(carrier, 0664, dmarx_show, dmarx_store);
static struct fractiming_attribute isr_attribute =
	__ATTR(isr, 0664, NULL, dmarx_store);
static struct fractiming_attribute canewl_attribute =
	__ATTR(canewl, 0664, dmarx_show, dmarx_store);
static struct fractiming_attribute canreset_attribute =
	__ATTR(canreset, 0664, dmarx_show, dmarx_store);
static struct fractiming_attribute rxhead_attribute =
	__ATTR(rxhead, 0664, dmarx_show, dmarx_store);
static struct fractiming_attribute txhead_attribute =
	__ATTR(txhead, 0664, dmarx_show, dmarx_store);
static struct fractiming_attribute txdonehead_attribute =
	__ATTR(txdonehead, 0664, dmarx_show, dmarx_store);

static struct attribute *foo_default_attrs[] = {
	&fractiming_attribute.attr,
	&clock_attribute.attr,
	&brp_max_attribute.attr,
	&brp_min_attribute.attr,
	&bt_attribute.attr,
	&dmarx_attribute.attr,
	&dmatx_attribute.attr,
	&restart_ms_attribute.attr,
	&debug_attribute.attr,
	&timeo_attribute.attr,
	&skipnrxonerr_attribute.attr,
	&queue_attribute.attr,
	&carrier_attribute.attr,
	&isr_attribute.attr,
	&canewl_attribute.attr,
	&canreset_attribute.attr,
	&rxhead_attribute.attr,
	&txhead_attribute.attr,
	&txdonehead_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};
static void foo_release(struct kobject *kobj)
{
	struct ls2k1500_priv *foo;

	foo = to_ls2k1500_priv(kobj);
	kfree(foo);
}
static struct kobj_type foo_ktype = {
	.sysfs_ops = &foo_sysfs_ops,
	.release = foo_release,
	.default_attrs = foo_default_attrs,
};

static void sja1000_write_cmdreg(struct sja1000_priv *priv, u8 val)
{
	unsigned long flags;

	/*
	 * The command register needs some locking and time to settle
	 * the write_reg() operation - especially on SMP systems.
	 */
	spin_lock_irqsave(&priv->cmdreg_lock, flags);
	priv->write_reg(priv, SJA1000_CMR, val);
	priv->read_reg(priv, SJA1000_SR);
	spin_unlock_irqrestore(&priv->cmdreg_lock, flags);
}

static unsigned long mscycles;
static atomic_t waiting_for_canbreak_ipi;
static void ls2k500sfb_canbreak_func(void *unused)
{
	atomic_dec(&waiting_for_canbreak_ipi);

	while (atomic_read(&waiting_for_canbreak_ipi)) {
		cpu_relax();
	}
}

static void canbreak_smp_send_stop(int ms)
{
	/* Wait at most 100 msecond for the other cpus to stop */
	unsigned long max_cycles =  mscycles * ms;
	unsigned long start_time = get_cycles();

	atomic_set(&waiting_for_canbreak_ipi, num_online_cpus());
	smp_call_function(ls2k500sfb_canbreak_func, NULL, false);
	while ((atomic_read(&waiting_for_canbreak_ipi) > 1)
		&& get_cycles() - start_time < max_cycles) {
		cpu_relax();
	}
}

static void reset_can_devices(void)
{
	struct ls2k1500_priv *lp;
	unsigned long flags;
	void *reset_reg = (void *)TO_UNCAC(0x10010434);
	uint32_t reset_val;

	canbreak_smp_send_stop(100);
	reset_val = readl(reset_reg);
	writel(reset_val | (1 << 24), reset_reg);
	udelay(10);
	writel(reset_val, reset_reg);
	udelay(10);
	spin_lock_irqsave(&can_devices_lock, flags);
	list_for_each_entry(lp, &can_devices, list) {
		lp->state = STATE_IDLE;
		iowrite8(lp->saved_regs[SJA1000_MOD] | 0x1,  lp->sjp->reg_base + SJA1000_MOD);
		iowrite8(lp->saved_regs[SJA1000_BTR0], lp->sjp->reg_base + SJA1000_BTR0);
		iowrite8(lp->saved_regs[SJA1000_BTR1], lp->sjp->reg_base + SJA1000_BTR1);
		iowrite8(lp->saved_regs[SJA1000_INTH], lp->sjp->reg_base + SJA1000_INTH);
		iowrite8(lp->saved_regs[SJA1000_INTL], lp->sjp->reg_base + SJA1000_INTL);
		iowrite8(lp->saved_regs[SJA1000_FRAC], lp->sjp->reg_base + SJA1000_FRAC);
		iowrite8(lp->saved_regs[SJA1000_MOD],  lp->sjp->reg_base + SJA1000_MOD);
		iowrite8(lp->saved_regs[SJA1000_IER],  lp->sjp->reg_base + SJA1000_IER);
	}
	spin_unlock_irqrestore(&can_devices_lock, flags);

	atomic_set(&waiting_for_canbreak_ipi, 0);
	wmb();
	resetcnt++;
}

static void ls2k1500_post_irq(const struct sja1000_priv *priv)
{
	struct ls2k1500_priv *lp = priv->priv;

	if (!fixup)
		return;
	if (lp->isrc & (IRQ_DOI | IRQ_EI | IRQ_BEI | IRQ_EPI | IRQ_ALI)) {
		if (lp->status & SR_BS) {
			if (!priv->can.restart_ms)
				netif_carrier_on(priv->dev);
		}
	}
	lp->last_cnt++;
	lp->last_isrc = lp->isrc;
	lp->last_status = lp->status;
	lp->isrc = 0;
	lp->status = 0;
}

static int can_ls2k1500_probe(struct platform_device *pdev)
{
	int err, irq = 0;
	void __iomem *addr;
	struct net_device *dev;
	struct sja1000_priv *priv;
	struct resource *res_mem, *res_irq = NULL;
	struct sja1000_platform_data *pdata;
	struct device_node *of = pdev->dev.of_node;
	size_t priv_sz = 0;
	struct ls2k1500_priv *lp;
	struct pci_dev *pdmadev;
	uint32_t dma_rxnr = 32, dma_txnr = 32;
	u8 d;
	struct sja1000_platform_data data;

#ifdef __loongarch__
	if (iocsr_read64(LOONGARCH_IOCSR_CPUNAME) == 0x0000303035314B32)
		fixup = 0;
#endif
	if (ACPI_COMPANION(&pdev->dev)) {
		memset(&data, 0, sizeof(data));
		pdata = &data;
		device_property_read_u32(&pdev->dev, "clock", &pdata->osc_freq);
		device_property_read_u8(&pdev->dev, "ocr", &pdata->ocr);
		device_property_read_u8(&pdev->dev, "cdr", &pdata->cdr);
		device_property_read_u32(&pdev->dev, "dmarx", &dma_rxnr);
		device_property_read_u32(&pdev->dev, "dmatx", &dma_txnr);
	} else {
		pdata = dev_get_platdata(&pdev->dev);
		if (!pdata && !of) {
			dev_err(&pdev->dev, "No platform data provided!\n");
			return -ENODEV;
		}
		of_property_read_u32(of, "dmarx", &dma_rxnr);
		of_property_read_u32(of, "dmatx", &dma_txnr);
	}

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_mem)
		return -ENODEV;

	if (!devm_request_mem_region(&pdev->dev, res_mem->start,
					resource_size(res_mem), DRV_NAME))
		return -EBUSY;

	addr = devm_ioremap_nocache(&pdev->dev, res_mem->start,
				resource_size(res_mem));
	if (!addr)
		return -ENOMEM;

	if (of)
		irq = irq_of_parse_and_map(of, 0);
	else
		res_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (!irq && !res_irq)
		return -ENODEV;

	priv_sz = sizeof(struct ls2k1500_priv);

	dev = alloc_sja1000dev(priv_sz);
	if (!dev)
		return -ENOMEM;
	priv = netdev_priv(dev);

	if (res_irq) {
		irq = res_irq->start;
		priv->irq_flags = res_irq->flags & IRQF_TRIGGER_MASK;
		if (res_irq->flags & IORESOURCE_IRQ_SHAREABLE)
			priv->irq_flags |= IRQF_SHARED;
	} else {
		priv->irq_flags = IRQF_SHARED;
	}

	dev->irq = irq;
	priv->reg_base = addr;

	if (of) {
		ls2k1500_populate_of(priv, of);
	} else
		ls2k1500_populate(priv, pdata, res_mem->flags);

	lp = priv->priv;
	lp->sjp = priv;
	lp->freq = priv->can.clock.freq * 2;
	pdmadev = pci_get_device(0x0014, 0x7a2f, NULL);
	lp->dma_rxnr = dma_rxnr;
	lp->dma_txnr = dma_txnr;
	lp->pdmadev = pdmadev;

	if (pdmadev) {
		lp->dma_reg = pci_iomap(pdmadev, 0, 0x1000);
	} else {
		lp->dma_rxnr = 32;
		lp->dma_txnr = 32;
	}
	lp->reg_phys = res_mem->start;
	lp->nrxdescs = nrxdescs;
	lp->ntxdescs = ntxdescs;
	if (ntxdescs % 3)
		return -EINVAL;

	platform_set_drvdata(pdev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	err = register_sja1000dev(dev);
	if (err) {
		dev_err(&pdev->dev, "registering %s failed (err=%d)\n",
			DRV_NAME, err);
		goto exit_free;
	}

	if (lp->dma_rxnr < 32) {
		lp->dma_rx_descs =
			dma_alloc_coherent(&lp->pdmadev->dev,
					sizeof(*lp->dma_rx_descs) * nrxdescs +
					nrxdescs, &lp->dma_rx_phys, GFP_KERNEL);
		ls2k1500_can_smp_call_function_single(0, can_ls2k1500_dma_rx_init, priv, 1);
	}
	d = ioread8(priv->reg_base + SJA1000_MOD);
	priv->write_reg(priv, SJA1000_MOD, d | MOD_RM);
	if (!(d & MOD_RM))
		priv->write_reg(priv, SJA1000_MOD, d);

	if (lp->dma_txnr < 32) {
		lp->dma_tx_descs =
			dma_alloc_coherent(&lp->pdmadev->dev,
					sizeof(*lp->dma_tx_descs) * ntxdescs +
					nrxdescs * 16, &lp->dma_tx_phys,
					GFP_KERNEL);
		ls2k1500_can_smp_call_function_single(0, can_ls2k1500_dma_tx_init, priv, 1);
	}
	lp->irq = of_irq_get(of, 1);
	if (lp->dma_rxnr < 32 || lp->dma_txnr < 32) {
		err =
			request_irq(lp->irq, sja1000_dma_interrupt, priv->irq_flags,
				dev->name, (void *)dev);
		irq_set_affinity_hint(lp->irq, get_cpu_mask(0));
	}

	ls2k1500_netdev_ops = *dev->netdev_ops;
	lp->netdev_ops = *dev->netdev_ops;
	dev->netdev_ops = &lp->netdev_ops;
	if (lp->dma_txnr < 32)
		lp->netdev_ops.ndo_start_xmit = ls2k1500_start_dma_xmit;
	else
		lp->netdev_ops.ndo_start_xmit = ls2k1500_start_norm_xmit;

	lp->btc = BTC;
	priv->can.do_set_bittiming = sja1000_set_bittiming;
	priv->can.bittiming_const = &lp->btc;
	lp->fractiming = fractiming;
	if (fractiming) {
		int div = ((lp->freq / 2 * 0x200) >> 32) + 1;
		priv->can.clock.freq = lp->freq/2 * 0x200 / div;
		lp->btc.brp_max = 0x1000000 / div;
		lp->btc.brp_min = 0x200 / div;
	} else {
		priv->can.clock.freq = lp->freq / 2;
		lp->btc.brp_max = 64;
	       lp->btc.brp_min = 1;
	}
	err = kobject_init_and_add(&lp->kobj, &foo_ktype,  &dev->dev.kobj, "canlp");
	if (!err) {
		kobject_uevent(&lp->kobj, KOBJ_ADD);
	}
	lp->debug = DEBUG_NOAT_TIMEOUT;
	priv->can.restart_ms = restart_ms;
	lp->skipnrxonerr = skipnrxonerr;
	priv->post_irq = ls2k1500_post_irq;
	priv->write_reg(priv, SJA1000_EWL, fixup ? canewl : 0x60);
	spin_lock(&can_devices_lock);
	list_add_tail(&lp->list, &can_devices);
	spin_unlock(&can_devices_lock);

	if (!mscycles) {
		local_irq_disable();
		mscycles = get_cycles();
		mdelay(1);
		mscycles = get_cycles() - mscycles;
		local_irq_enable();
	}
	dev_info(&pdev->dev, "%s device registered (reg_base=%p, irq=%d)\n",
		DRV_NAME, priv->reg_base, dev->irq);
	return 0;

exit_free:
	free_sja1000dev(dev);
	return err;
}

static int can_ls2k1500_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);

	unregister_sja1000dev(dev);
	free_sja1000dev(dev);

	return 0;
}

static const struct of_device_id can_ls2k1500_of_table[] = {
	{.compatible = "ls2k1500,sja1000"},
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, can_ls2k1500_of_table);

static struct platform_driver can_ls2k1500_driver = {
	.probe = can_ls2k1500_probe,
	.remove = can_ls2k1500_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = can_ls2k1500_of_table,
	},
};

module_platform_driver(can_ls2k1500_driver);
