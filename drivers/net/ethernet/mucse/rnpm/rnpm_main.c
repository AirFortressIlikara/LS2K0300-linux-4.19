#include <linux/types.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/kthread.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/in.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/sctp.h>
#include <linux/pkt_sched.h>
#include <linux/ipv6.h>
#include <linux/slab.h>
#include <net/checksum.h>
#include <net/ip6_checksum.h>
#include <linux/ethtool.h>
#include <linux/if.h>
#include <linux/if_vlan.h>
#include <linux/if_bridge.h>
#include <linux/prefetch.h>
#include <linux/capability.h>
#include <linux/sort.h>

#include "rnpm.h"
#include "rnpm_common.h"
#include "rnpm_dcb.h"
#include "rnpm_sriov.h"
#include "rnpm_ptp.h"
#include "rnpm_tc.h"
#include "rnpm_mbx.h"
#include "rnpm_mbx_fw.h"

#ifdef HAVE_UDP_ENC_RX_OFFLOAD
#include <net/vxlan.h>
#include <net/udp_tunnel.h>
#endif /* HAVE_UDP_ENC_RX_OFFLOAD */
#ifdef HAVE_VXLAN_RX_OFFLOAD
#include <net/vxlan.h>
#endif /* HAVE_VXLAN_RX_OFFLOAD */

#define TX_IRQ_MISS_REDUCE

char rnpm_driver_name[] = "rnpm";
char rnpm_port_name[] = "enp";

#ifndef NO_NETDEV_PORT
#define ASSIN_PDEV
#endif
static const char rnpm_driver_string[] =
	"mucse 4/8port 1/10 Gigabit PCI Express Network Driver";
static char rnpm_default_device_descr[] __maybe_unused =
	"mucse(R) 4/8port 1/10 Gigabit Network Connection";
#define DRV_VERSION "0.1.6-rc14"
const char rnpm_driver_version[] = DRV_VERSION;
static const char rnpm_copyright[] =
	"Copyright (c) 2020-2022 mucse Corporation.";

extern struct rnpm_info rnpm_vu440_2x10G_info;
extern struct rnpm_info rnpm_vu440_4x10G_info;
extern struct rnpm_info rnpm_vu440_8x10G_info;
extern struct rnpm_info rnpm_n10_info;
extern struct rnpm_info rnpm_n400_4x1G_info;
static struct rnpm_info *rnpm_info_tbl[] = {
	[board_vu440_2x10G] = &rnpm_vu440_2x10G_info,
	[board_vu440_4x10G] = &rnpm_vu440_4x10G_info,
	[board_vu440_8x10G] = &rnpm_vu440_8x10G_info,

	[board_n10] = &rnpm_n10_info,
	[board_n400_4x1G] = &rnpm_n400_4x1G_info,
};

static bool rnpm_alloc_mapped_page(struct rnpm_ring *rx_ring,
								   struct rnpm_rx_buffer *bi);
#ifdef CONFIG_RNPM_DISABLE_PACKET_SPLIT
static bool rnpm_alloc_mapped_skb(struct rnpm_ring *rx_ring,
								  struct rnpm_rx_buffer *bi);
#endif
static void rnpm_pull_tail(struct sk_buff *skb);

static void rnpm_put_rx_buffer(struct rnpm_ring *rx_ring,
							   struct rnpm_rx_buffer *rx_buffer,
							   struct sk_buff *skb);
//#define DEBUG_TX
//#define SIMULATE_TX

// vu440 must select mode type
#ifdef UV440_2PF
#ifdef MODE_4_PORT
#define MODE_TYPE board_vu440_8x10G
#endif

#ifdef MODE_2_PORT
#define MODE_TYPE board_vu440_4x10G
#endif

#ifdef MODE_1_PORT
#define MODE_TYPE board_vu440_2x10G
#endif

#ifndef MODE_TYPE
/* default in 4 ports in 1 pf mode */
#define MODE_TYPE board_vu440_8x10G
#endif
#endif
/* itr can be modified in napi handle */
/* now hw not support this */
#define ITR_TEST 0

//#define DISABLE_RX_IRQ

static struct pci_device_id rnpm_pci_tbl[] = {
	/*
	#ifndef CONFIG_RNPM_DISABLE_PF0
		{PCI_DEVICE(0x1dab, RNPM_DEV_ID_N10_PF0), .driver_data =
	board_vu440_8x10G}, #endif #ifndef CONFIG_RNPM_DISABLE_PF1
		{PCI_DEVICE(0x1dab, RNPM_DEV_ID_N10_PF1), .driver_data =
	board_vu440_8x10G}, #endif
	*/
	// to support old firmware
	//{PCI_DEVICE(0x1dab, 0x7001), .driver_data = board_n10},
	//{PCI_DEVICE(0x1dab, 0x7002), .driver_data = board_n10_8x10G},

	// {PCI_DEVICE(PCI_VENDOR_ID_MUCSE, 0x1000), .driver_data = board_n10},
	{PCI_DEVICE(PCI_VENDOR_ID_MUCSE, 0x1060), .driver_data = board_n10},
	{PCI_DEVICE(PCI_VENDOR_ID_MUCSE, 0x1C60), .driver_data = board_n10}, /* N10C */
	// todo test me
	{PCI_DEVICE(PCI_VENDOR_ID_MUCSE, 0x1020), .driver_data = board_n10},
	{PCI_DEVICE(PCI_VENDOR_ID_MUCSE, 0x1021), .driver_data = board_n400_4x1G}, /* N400 4x1G */
	{PCI_DEVICE(PCI_VENDOR_ID_MUCSE, 0x1c21), .driver_data = board_n400_4x1G},

	/* required last entry */
	{
		0,
	}};

MODULE_DEVICE_TABLE(pci, rnpm_pci_tbl);

static unsigned int mac_loop_en;
module_param(mac_loop_en, uint, 0000);

#ifdef CONFIG_PCI_IOV
static unsigned int max_vfs;
module_param(max_vfs, uint, 0000);
MODULE_PARM_DESC(max_vfs, "Maximum number of virtual functions to allocate per \
		physical function - default is zero and maximum \
		value is 127");
#endif /* CONFIG_PCI_IOV */

#define DEFAULT_MSG_ENABLE (NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK)
static int debug = -1;
module_param(debug, int, 0000);
MODULE_PARM_DESC(debug, "Debug level (0=none,...,16=all)");

static unsigned int pf_msix_counts_set;
module_param(pf_msix_counts_set, uint, 0000);
MODULE_PARM_DESC(pf_msix_counts_set, "set msix count by one pf");

/* just for test */
static unsigned int fix_eth_name;
module_param(fix_eth_name, uint, 0000);
MODULE_PARM_DESC(fix_eth_name, "set eth adapter name to rnpmXX");

#ifndef NO_PTP
static int module_enable_ptp = 1;
module_param(module_enable_ptp, uint, 0000);
MODULE_PARM_DESC(module_enable_ptp, "enable ptp feature, disabled default");
#endif

static int port_valid_pf0 = 0xf;
module_param(port_valid_pf0, uint, 0000);
MODULE_PARM_DESC(port_valid_pf0, "pf0 valid (only in 8 ports mode");

static int port_valid_pf1 = 0xf;
module_param(port_valid_pf1, uint, 0000);
MODULE_PARM_DESC(port_valid_pf1, "pf1 valid (only in 8 ports mode");

static unsigned int port_names_pf0 = 0x03020100;
module_param(port_names_pf0, uint, 0000);
MODULE_PARM_DESC(port_names_pf0, "pf0 names (only in 8 ports mode");

static unsigned int port_names_pf1 = 0x03020100;
module_param(port_names_pf1, uint, 0000);
MODULE_PARM_DESC(port_names_pf1, "pf1 names (only in 8 ports mode");

MODULE_AUTHOR("Mucse Corporation, <mucse@mucse.com>");
MODULE_DESCRIPTION("Mucse(R) 10 Gigabit PCI Express Network Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

static int enable_hi_dma;
extern const struct net_device_ops rnpm_netdev_ops;
extern void rnpm_service_timer(struct timer_list *t);

void rnpm_service_event_schedule(struct rnpm_adapter *adapter)
{
	if (!test_bit(__RNPM_DOWN, &adapter->state) &&
		!test_and_set_bit(__RNPM_SERVICE_SCHED, &adapter->state))
		schedule_work(&adapter->service_task);
}


void rnpm_pf_service_event_schedule(struct rnpm_pf_adapter *pf_adapter)
{
	schedule_work(&pf_adapter->service_task);
}

static void rnpm_service_event_complete(struct rnpm_adapter *adapter)
{
	BUG_ON(!test_bit(__RNPM_SERVICE_SCHED, &adapter->state));

	/* flush memory to make sure state is correct before next watchdog */
	// smp_mb__before_clear_bit();
	clear_bit(__RNPM_SERVICE_SCHED, &adapter->state);
}

void rnpm_release_hw_control(struct rnpm_adapter *adapter)
{
	// u32 ctrl_ext;

	/* Let firmware take over control of h/w */
	// ctrl_ext = RNPM_READ_REG(&adapter->hw, RNPM_CTRL_EXT);
	// RNPM_WRITE_REG(&adapter->hw, RNPM_CTRL_EXT,
	//	ctrl_ext & ~RNPM_CTRL_EXT_DRV_LOAD);
}

void rnpm_get_hw_control(struct rnpm_adapter *adapter)
{
	// u32 ctrl_ext;

	/* Let firmware know the driver has taken over */
}

/**
 * rnpm_set_ivar - set the ring_vector registers,
 * mapping interrupt causes to vectors
 * @adapter: pointer to adapter struct
 * @queue: queue to map the corresponding interrupt to
 * @msix_vector: the vector to map to the corresponding queue
 *
 */
static void rnpm_set_ring_vector(struct rnpm_adapter *adapter,
								 u8 rnpm_queue,
								 u8 rnpm_msix_vector)
{
	struct rnpm_hw *hw = &adapter->hw;
	// struct net_device *netdev = adapter->netdev;
	u32 data = 0;

	data = hw->pfvfnum << 24;
	data |= (rnpm_msix_vector << 8);
	data |= (rnpm_msix_vector << 0);

	DPRINTK(IFUP,
			INFO,
			"Set Ring-Vector queue:%d (reg:0x%x) <-- Rx-MSIX:%d, Tx-MSIX:%d\n",
			rnpm_queue,
			RING_VECTOR(rnpm_queue),
			rnpm_msix_vector,
			rnpm_msix_vector);

	rnpm_wr_reg(hw->ring_msix_base + RING_VECTOR(rnpm_queue), data);
}

static inline void rnpm_irq_rearm_queues(struct rnpm_adapter *adapter,
										 u64 qmask)
{
	// u32 mask;
}

void rnpm_unmap_and_free_tx_resource(struct rnpm_ring *ring,
									 struct rnpm_tx_buffer *tx_buffer)
{
	if (tx_buffer->skb) {
		dev_kfree_skb_any(tx_buffer->skb);
		if (dma_unmap_len(tx_buffer, len))
			dma_unmap_single(ring->dev,
							 dma_unmap_addr(tx_buffer, dma),
							 dma_unmap_len(tx_buffer, len),
							 DMA_TO_DEVICE);
	} else if (dma_unmap_len(tx_buffer, len)) {
		dma_unmap_page(ring->dev,
					   dma_unmap_addr(tx_buffer, dma),
					   dma_unmap_len(tx_buffer, len),
					   DMA_TO_DEVICE);
	}
	tx_buffer->next_to_watch = NULL;
	tx_buffer->skb = NULL;
	dma_unmap_len_set(tx_buffer, len, 0);
	/* tx_buffer must be completely set up in the transmit path */
}

static u64 rnpm_get_tx_completed(struct rnpm_ring *ring)
{
	return ring->stats.packets;
}

static u64 rnpm_get_tx_pending(struct rnpm_ring *ring)
{
	struct rnpm_adapter *adapter = netdev_priv(ring->netdev);
	struct rnpm_hw *hw = &adapter->hw;

	u32 head = rd32(hw, RNPM_DMA_REG_TX_DESC_BUF_HEAD(ring->rnpm_queue_idx));
	u32 tail = rd32(hw, RNPM_DMA_REG_TX_DESC_BUF_TAIL(ring->rnpm_queue_idx));

	if (head != tail)
		return (head < tail) ? tail - head : (tail + ring->count - head);

	return 0;
}

static inline bool rnpm_check_tx_hang(struct rnpm_ring *tx_ring)
{
	u32 tx_done = rnpm_get_tx_completed(tx_ring);
	u32 tx_done_old = tx_ring->tx_stats.tx_done_old;
	u32 tx_pending = rnpm_get_tx_pending(tx_ring);
	bool ret = false;

	clear_check_for_tx_hang(tx_ring);

	/*
	 * Check for a hung queue, but be thorough. This verifies
	 * that a transmit has been completed since the previous
	 * check AND there is at least one packet pending. The
	 * ARMED bit is set to indicate a potential hang. The
	 * bit is cleared if a pause frame is received to remove
	 * false hang detection due to PFC or 802.3x frames. By
	 * requiring this to fail twice we avoid races with
	 * pfc clearing the ARMED bit and conditions where we
	 * run the check_tx_hang logic with a transmit completion
	 * pending but without time to complete it yet.
	 */
	if ((tx_done_old == tx_done) && tx_pending) {
		/* make sure it is true for two checks in a row */
		ret = test_and_set_bit(__RNPM_HANG_CHECK_ARMED, &tx_ring->state);
	} else {
		/* update completed stats and continue */
		tx_ring->tx_stats.tx_done_old = tx_done;
		/* reset the countdown */
		clear_bit(__RNPM_HANG_CHECK_ARMED, &tx_ring->state);
	}
	return ret;
}

/**
 * rnpm_tx_timeout_reset - initiate reset due to Tx timeout
 * @adapter: driver private struct
 **/
static void rnpm_tx_timeout_reset(struct rnpm_adapter *adapter)
{
	/* Do the reset outside of interrupt context */
	if (!test_bit(__RNPM_DOWN, &adapter->state)) {
		adapter->flags2 |= RNPM_FLAG2_RESET_REQUESTED;
		e_warn(drv, "initiating reset due to tx timeout\n");
		rnpm_dbg("initiating reset due to tx timeout\n");
		rnpm_service_event_schedule(adapter);
	}
}

static void rnpm_check_restart_tx(struct rnpm_q_vector *q_vector,
								  struct rnpm_ring *tx_ring)
{
	struct rnpm_adapter *adapter = q_vector->adapter;

#define TX_WAKE_THRESHOLD (DESC_NEEDED * 2)
	if (likely(netif_carrier_ok(tx_ring->netdev) &&
			   (rnpm_desc_unused(tx_ring) >= TX_WAKE_THRESHOLD))) {
		/* Make sure that anybody stopping the queue after this
		 * sees the new next_to_clean.
		 */
		smp_mb();
		if (__netif_subqueue_stopped(tx_ring->netdev, tx_ring->queue_index) &&
			!test_bit(__RNPM_DOWN, &adapter->state)) {
			netif_wake_subqueue(tx_ring->netdev, tx_ring->queue_index);
			++tx_ring->tx_stats.restart_queue;
		}
	}
}

/**
 * rnpm_clean_tx_irq - Reclaim resources after transmit completes
 * @q_vector: structure containing interrupt and ring information
 * @tx_ring: tx ring to clean
 **/
static bool rnpm_clean_tx_irq(struct rnpm_q_vector *q_vector,
							  struct rnpm_ring *tx_ring,
							  int napi_budget)
{
	struct rnpm_adapter *adapter = q_vector->adapter;
	struct rnpm_tx_buffer *tx_buffer;
	struct rnpm_tx_desc *tx_desc;
	unsigned int total_bytes = 0, total_packets = 0;
	unsigned int budget = q_vector->tx.work_limit;
	unsigned int i = tx_ring->next_to_clean;

	if (test_bit(__RNPM_DOWN, &adapter->state))
		return true;
	tx_ring->tx_stats.poll_count++;
	tx_buffer = &tx_ring->tx_buffer_info[i];
	tx_desc = RNPM_TX_DESC(tx_ring, i);
	i -= tx_ring->count;

	do {
		struct rnpm_tx_desc *eop_desc = tx_buffer->next_to_watch;

		/* if next_to_watch is not set then there is no work pending */
		if (!eop_desc)
			break;

		/* prevent any other reads prior to eop_desc */
		// read_barrier_depends();
		smp_mb();

		/* if eop DD is not set pending work has not been completed */
		if (!(eop_desc->vlan_cmd & cpu_to_le32(RNPM_TXD_STAT_DD)))
			break;

		/* clear next_to_watch to prevent false hangs */
		tx_buffer->next_to_watch = NULL;

		/* update the statistics for this packet */
		total_bytes += tx_buffer->bytecount;
		total_packets += tx_buffer->gso_segs;

		/* free the skb */
		napi_consume_skb(tx_buffer->skb, napi_budget);
		/* unmap skb header data */
		dma_unmap_single(tx_ring->dev,
						 dma_unmap_addr(tx_buffer, dma),
						 dma_unmap_len(tx_buffer, len),
						 DMA_TO_DEVICE);

		/* clear tx_buffer data */
		tx_buffer->skb = NULL;
		dma_unmap_len_set(tx_buffer, len, 0);

		// printk("start clean\n");
		/* unmap remaining buffers */
		while (tx_desc != eop_desc) {
			/* print desc */
			buf_dump_line(
				"desc %d  ", i + tx_ring->count, tx_desc, sizeof(*tx_desc));

			tx_buffer++;
			tx_desc++;
			i++;
			if (unlikely(!i)) {
				i -= tx_ring->count;
				tx_buffer = tx_ring->tx_buffer_info;
				tx_desc = RNPM_TX_DESC(tx_ring, 0);
			}

			/* unmap any remaining paged data */
			if (dma_unmap_len(tx_buffer, len)) {
				dma_unmap_page(tx_ring->dev,
							   dma_unmap_addr(tx_buffer, dma),
							   dma_unmap_len(tx_buffer, len),
							   DMA_TO_DEVICE);
				dma_unmap_len_set(tx_buffer, len, 0);
			}
		}

		/* move us one more past the eop_desc for start of next pkt */
		tx_buffer++;
		tx_desc++;
		i++;
		if (unlikely(!i)) {
			i -= tx_ring->count;
			tx_buffer = tx_ring->tx_buffer_info;
			tx_desc = RNPM_TX_DESC(tx_ring, 0);
		}

		/* issue prefetch for next Tx descriptor */
		prefetch(tx_desc);

		/* update budget accounting */
		budget--;
	} while (likely(budget));

	i += tx_ring->count;
	tx_ring->next_to_clean = i;
	u64_stats_update_begin(&tx_ring->syncp);
	tx_ring->stats.bytes += total_bytes;
	tx_ring->stats.packets += total_packets;
	u64_stats_update_end(&tx_ring->syncp);
	/* maybe nouse ?*/
	// q_vector->tx.total_bytes += total_bytes;
	// q_vector->tx.total_packets += total_packets;
	tx_ring->tx_stats.send_done_bytes += total_bytes;

#if 0
	if (check_for_tx_hang(tx_ring) && rnpm_check_tx_hang(tx_ring)) {
		printk(" error: Tx unit hang\n"
			"  Queue id             <%d>\n"
			"  next_to_use          <%x>\n"
			"  next_to_clean        <%x>\n"
			"tx_buffer_info[next_to_clean]\n"
			"  time_stamp           <%lx>\n"
			"  jiffies              <%lx>\n",
			tx_ring->queue_index,
			tx_ring->next_to_use, i,
			tx_ring->tx_buffer_info[i].time_stamp, jiffies);
		rnpm_tx_timeout_reset(adapter);
	}
#endif

	netdev_tx_completed_queue(txring_txq(tx_ring), total_packets, total_bytes);
#ifndef TX_IRQ_MISS_REDUCE
#define TX_WAKE_THRESHOLD (DESC_NEEDED * 2)
	if (likely(netif_carrier_ok(tx_ring->netdev) &&
			   (rnpm_desc_unused(tx_ring) >= TX_WAKE_THRESHOLD))) {
		/* Make sure that anybody stopping the queue after this
		 * sees the new next_to_clean.
		 */
		smp_mb();
		if (__netif_subqueue_stopped(tx_ring->netdev, tx_ring->queue_index) &&
			!test_bit(__RNPM_DOWN, &adapter->state)) {
			netif_wake_subqueue(tx_ring->netdev, tx_ring->queue_index);
			++tx_ring->tx_stats.restart_queue;
		}
	}
#endif

	return !!budget;
}

static inline void rnpm_rx_hash(struct rnpm_ring *ring,
								union rnpm_rx_desc *rx_desc,
								struct sk_buff *skb)
{
	int rss_type;

	if (!(ring->netdev->features & NETIF_F_RXHASH))
		return;
#define RNPM_RSS_TYPE_MASK 0xc0
	rss_type = rx_desc->wb.cmd & RNPM_RSS_TYPE_MASK;
	skb_set_hash(skb,
				 le32_to_cpu(rx_desc->wb.rss_hash),
				 rss_type ? PKT_HASH_TYPE_L4 : PKT_HASH_TYPE_L3);
}

/**
 * rnpm_rx_checksum - indicate in skb if hw indicated a good cksum
 * @ring: structure containing ring specific data
 * @rx_desc: current Rx descriptor being processed
 * @skb: skb currently being received and modified
 **/
static inline void rnpm_rx_checksum(struct rnpm_ring *ring,
									union rnpm_rx_desc *rx_desc,
									struct sk_buff *skb)
{
	bool encap_pkt = false;

	skb_checksum_none_assert(skb);
	/* Rx csum disabled */
	if (!(ring->netdev->features & NETIF_F_RXCSUM))
		return;

	/* vxlan packet handle ? */
	if (rnpm_get_stat(rx_desc, RNPM_RXD_STAT_TUNNEL_MASK) ==
		RNPM_RXD_STAT_TUNNEL_VXLAN) {
		encap_pkt = true;
#if defined(HAVE_UDP_ENC_RX_OFFLOAD) || defined(HAVE_VXLAN_RX_OFFLOAD)
		skb->encapsulation = 1;
#endif
		/* HAVE_UDP_ENC_RX_OFFLOAD || HAVE_VXLAN_RX_OFFLOAD */
		skb->ip_summed = CHECKSUM_NONE;
	}

	/* if outer L3/L4  error */
	/* must in promisc mode */
	if (rnpm_test_staterr(rx_desc, RNPM_RXD_STAT_ERR_MASK) &&
		!ignore_veb_pkg_err(ring->q_vector->adapter, rx_desc)) {
		// ring->rx_stats.csum_err++;
		return;
	}

	ring->rx_stats.csum_good++;
	/* at least it is a ip packet which has ip checksum */

	/* It must be a TCP or UDP packet with a valid checksum */
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	if (encap_pkt) {
#ifdef HAVE_SKBUFF_CSUM_LEVEL
		/* If we checked the outer header let the stack know */
		skb->csum_level = 1;
#endif /* HAVE_SKBUFF_CSUM_LEVEL */
	}
}

static inline void rnpm_update_rx_tail(struct rnpm_ring *rx_ring, u32 val)
{
	rx_ring->next_to_use = val;
#ifndef CONFIG_RNPM_DISABLE_PACKET_SPLIT
	/* update next to alloc since we have filled the ring */
	rx_ring->next_to_alloc = val;
#endif
	/*
	 * Force memory writes to complete before letting h/w
	 * know there are new descriptors to fetch.  (Only
	 * applicable for weak-ordered memory model archs,
	 * such as IA-64).
	 */
	wmb();
	if (likely(!(rx_ring->ring_flags & RNPM_RING_FLAG_DELAY_SETUP_RX_LEN))) {
		rnpm_wr_reg(rx_ring->tail, val);
	} else if (val < RNPM_MIN_RXD) {
		/* if rx_ring in delay setup mode,
		 * don't update next_to_use to hw large than RNPM_MIN_RXD*/
		rnpm_wr_reg(rx_ring->tail, val);
	}
}

/**
 * rnpm_alloc_rx_buffers - Replace used receive buffers
 * @rx_ring: ring to place buffers on
 * @cleaned_count: number of buffers to replace
 **/
void rnpm_alloc_rx_buffers(struct rnpm_ring *rx_ring, u16 cleaned_count)
{
	union rnpm_rx_desc *rx_desc;
	struct rnpm_rx_buffer *bi;
	u16 i = rx_ring->next_to_use;
	u64 fun_id = ((u64)(rx_ring->pfvfnum) << (32 + 24));
#ifndef CONFIG_RNPM_DISABLE_PACKET_SPLIT
	u16 bufsz;
#endif
	/* nothing to do */
	if (!cleaned_count)
		return;

	rx_desc = RNPM_RX_DESC(rx_ring, i);
	BUG_ON(rx_desc == NULL);
	bi = &rx_ring->rx_buffer_info[i];
	BUG_ON(bi == NULL);
	i -= rx_ring->count;
#ifndef CONFIG_RNPM_DISABLE_PACKET_SPLIT
	bufsz = rnpm_rx_bufsz(rx_ring);
#endif

	do {
#ifdef CONFIG_RNPM_DISABLE_PACKET_SPLIT
		if (!rnpm_alloc_mapped_skb(rx_ring, bi))
			break;
#else
		if (!rnpm_alloc_mapped_page(rx_ring, bi))
			break;
		dma_sync_single_range_for_device(
			rx_ring->dev, bi->dma, bi->page_offset, bufsz, DMA_FROM_DEVICE);
#endif

			/*
			 * Refresh the desc even if buffer_addrs didn't change
			 * because each write-back erases this info.
			 */
#ifdef CONFIG_RNPM_DISABLE_PACKET_SPLIT
		rx_desc->pkt_addr = cpu_to_le64(bi->dma + fun_id);
#else
		rx_desc->pkt_addr = cpu_to_le64(bi->dma + bi->page_offset + fun_id);
#endif
		/* clean dd */
		// rx_desc->resv_cmd = cpu_to_le16(RNPM_RXD_FLAG_RS);
		// rx_desc->resv_cmd = cpu_to_le32(0);
		rx_desc->resv_cmd = 0;

		rx_desc++;
		bi++;
		i++;
		if (unlikely(!i)) {
			rx_desc = RNPM_RX_DESC(rx_ring, 0);
			bi = rx_ring->rx_buffer_info;
			i -= rx_ring->count;
		}

		/* clear the hdr_addr for the next_to_use descriptor */
		// rx_desc->cmd = 0;
		cleaned_count--;
	} while (cleaned_count);

	i += rx_ring->count;

	if (rx_ring->next_to_use != i)
		rnpm_update_rx_tail(rx_ring, i);
}
/**
 * rnpm_get_headlen - determine size of header for RSC/LRO/GRO/FCOE
 * @data: pointer to the start of the headers
 * @max_len: total length of section to find headers in
 *
 * This function is meant to determine the length of headers that will
 * be recognized by hardware for LRO, GRO, and RSC offloads.  The main
 * motivation of doing this is to only perform one pull for IPv4 TCP
 * packets so that we can do basic things like calculating the gso_size
 * based on the average data per packet.
 **/
static unsigned int rnpm_get_headlen(unsigned char *data, unsigned int max_len)
{
	union {
		unsigned char *network;
		/* l2 headers */
		struct ethhdr *eth;
		struct vlan_hdr *vlan;
		/* l3 headers */
		struct iphdr *ipv4;
		struct ipv6hdr *ipv6;
	} hdr;
	__be16 protocol;
	u8 nexthdr = 0; /* default to not TCP */
	u8 hlen;

	/* this should never happen, but better safe than sorry */
	if (max_len < ETH_HLEN)
		return max_len;

	/* initialize network frame pointer */
	hdr.network = data;

	/* set first protocol and move network header forward */
	protocol = hdr.eth->h_proto;
	hdr.network += ETH_HLEN;

	/* handle any vlan tag if present */
	if (protocol == __constant_htons(ETH_P_8021Q)) {
		if ((hdr.network - data) > (max_len - VLAN_HLEN))
			return max_len;

		protocol = hdr.vlan->h_vlan_encapsulated_proto;
		hdr.network += VLAN_HLEN;
	}

	/* handle L3 protocols */
	if (protocol == __constant_htons(ETH_P_IP)) {
		if ((hdr.network - data) > (max_len - sizeof(struct iphdr)))
			return max_len;

		/* access ihl as a u8 to avoid unaligned access on ia64 */
		hlen = (hdr.network[0] & 0x0F) << 2;

		/* verify hlen meets minimum size requirements */
		if (hlen < sizeof(struct iphdr))
			return hdr.network - data;

		/* record next protocol if header is present */
		if (!(hdr.ipv4->frag_off & htons(IP_OFFSET)))
			nexthdr = hdr.ipv4->protocol;
	} else if (protocol == __constant_htons(ETH_P_IPV6)) {
		if ((hdr.network - data) > (max_len - sizeof(struct ipv6hdr)))
			return max_len;

		/* record next protocol */
		nexthdr = hdr.ipv6->nexthdr;
		hlen = sizeof(struct ipv6hdr);
	} else {
		return hdr.network - data;
	}

	/* relocate pointer to start of L4 header */
	hdr.network += hlen;

	/* finally sort out TCP/UDP */
	if (nexthdr == IPPROTO_TCP) {
		if ((hdr.network - data) > (max_len - sizeof(struct tcphdr)))
			return max_len;

		/* access doff as a u8 to avoid unaligned access on ia64 */
		hlen = (hdr.network[12] & 0xF0) >> 2;

		/* verify hlen meets minimum size requirements */
		if (hlen < sizeof(struct tcphdr))
			return hdr.network - data;

		hdr.network += hlen;
	} else if (nexthdr == IPPROTO_UDP) {
		if ((hdr.network - data) > (max_len - sizeof(struct udphdr)))
			return max_len;

		hdr.network += sizeof(struct udphdr);
	}

	/*
	 * If everything has gone correctly hdr.network should be the
	 * data section of the packet and will be the end of the header.
	 * If not then it probably represents the end of the last recognized
	 * header.
	 */
	if ((hdr.network - data) < max_len)
		return hdr.network - data;
	else
		return max_len;
}

static void rnpm_set_rsc_gso_size(struct rnpm_ring *ring, struct sk_buff *skb)
{
	u16 hdr_len = skb_headlen(skb);

	/* set gso_size to avoid messing up TCP MSS */
	skb_shinfo(skb)->gso_size =
		DIV_ROUND_UP((skb->len - hdr_len), RNPM_CB(skb)->append_cnt);
	skb_shinfo(skb)->gso_type = SKB_GSO_TCPV4;
}

__maybe_unused static void rnpm_update_rsc_stats(struct rnpm_ring *rx_ring,
												 struct sk_buff *skb)
{
	/* if append_cnt is 0 then frame is not RSC */
	if (!RNPM_CB(skb)->append_cnt)
		return;

	rx_ring->rx_stats.rsc_count += RNPM_CB(skb)->append_cnt;
	rx_ring->rx_stats.rsc_flush++;

	rnpm_set_rsc_gso_size(rx_ring, skb);

	/* gso_size is computed using append_cnt so always clear it last */
	RNPM_CB(skb)->append_cnt = 0;
}
static void rnpm_rx_vlan(struct rnpm_ring *rx_ring,
						 union rnpm_rx_desc *rx_desc,
						 struct sk_buff *skb)
{
#ifdef NETIF_F_HW_VLAN_CTAG_RX
	if ((netdev_ring(rx_ring)->features & NETIF_F_HW_VLAN_CTAG_RX) &&
#else
	if ((netdev_ring(rx_ring)->features & NETIF_F_HW_VLAN_RX) &&
#endif
		rnpm_test_staterr(rx_desc, RNPM_RXD_STAT_VLAN_VALID)) {
		rx_ring->rx_stats.vlan_remove++;
#ifndef HAVE_VLAN_RX_REGISTER
		__vlan_hwaccel_put_tag(
			skb, htons(ETH_P_8021Q), le16_to_cpu(rx_desc->wb.vlan));
	}
#else
		RNPM_CB(skb)->vid = le16_to_cpu(rx_desc->wb.vlan);
	} else {
		RNPM_CB(skb)->vid = 0;
	}

#endif
}
/**
 * rnpm_process_skb_fields - Populate skb header fields from Rx descriptor
 * @rx_ring: rx descriptor ring packet is being transacted on
 * @rx_desc: pointer to the EOP Rx descriptor
 * @skb: pointer to current skb being populated
 *
 * This function checks the ring, descriptor, and packet information in
 * order to populate the hash, checksum, VLAN, timestamp, protocol, and
 * other fields within the skb.
 **/
static void rnpm_process_skb_fields(struct rnpm_ring *rx_ring,
									union rnpm_rx_desc *rx_desc,
									struct sk_buff *skb)
{
	struct net_device *dev = rx_ring->netdev;

	// rnpm_update_rsc_stats(rx_ring, skb);
	rnpm_rx_hash(rx_ring, rx_desc, skb);

	rnpm_rx_checksum(rx_ring, rx_desc, skb);

	rnpm_rx_vlan(rx_ring, rx_desc, skb);

	skb_record_rx_queue(skb, rx_ring->queue_index);
	// printk("record %d\n", rx_ring->queue_index);
	// printk("now %d\n", skb->queue_mapping);

	skb->protocol = eth_type_trans(skb, dev);
}
#ifdef HAVE_VLAN_RX_REGISTER
static void rnpm_receive_skb(struct rnpm_q_vector *q_vector,
							 struct sk_buff *skb)
{
	u16 vlan_tag = RNPM_CB(skb)->vid;

#if defined(NETIF_F_HW_VLAN_TX) || defined(NETIF_F_HW_VLAN_CTAG_TX)
	if (vlan_tag & VLAN_VID_MASK) {
		/* by placing vlgrp at start of structure we can alias it */
		struct vlan_group **vlgrp = netdev_priv(skb->dev);

		if (!*vlgrp)
			dev_kfree_skb_any(skb);
		else
			vlan_gro_receive(&q_vector->napi, *vlgrp, vlan_tag, skb);
	} else {
#endif /* NETIF_F_HW_VLAN_TX || NETIF_F_HW_VLAN_CTAG_TX */
		// if (q_vector->netpoll_rx)
		//		netif_rx(skb);
		//	else
		napi_gro_receive(&q_vector->napi, skb);
#if defined(NETIF_F_HW_VLAN_TX) || defined(NETIF_F_HW_VLAN_CTAG_TX)
	}
#endif /* NETIF_F_HW_VLAN_TX || NETIF_F_HW_VLAN_CTAG_TX */
}
#endif
static void rnpm_rx_skb(struct rnpm_q_vector *q_vector, struct sk_buff *skb)
{
#ifdef HAVE_VLAN_RX_REGISTER
	rnpm_receive_skb(q_vector, skb);
#else

	napi_gro_receive(&q_vector->napi, skb);
#endif
}

#ifdef CONFIG_RNPM_DISABLE_PACKET_SPLIT
/**
 * rnpm_merge_active_tail - merge active tail into lro skb
 * @tail: pointer to active tail in frag_list
 *
 * This function merges the length and data of an active tail into the
 * skb containing the frag_list.  It resets the tail's pointer to the head,
 * but it leaves the heads pointer to tail intact.
 **/
static inline struct sk_buff *rnpm_merge_active_tail(struct sk_buff *tail)
{
	struct sk_buff *head = RNPM_CB(tail)->head;

	if (!head)
		return tail;

	head->len += tail->len;
	head->data_len += tail->len;
	head->truesize += tail->truesize;

	RNPM_CB(tail)->head = NULL;

	return head;
}

/**
 * rnpm_add_active_tail - adds an active tail into the skb frag_list
 * @head: pointer to the start of the skb
 * @tail: pointer to active tail to add to frag_list
 *
 * This function adds an active tail to the end of the frag list.  This tail
 * will still be receiving data so we cannot yet ad it's stats to the main
 * skb.  That is done via rnpm_merge_active_tail.
 **/
static inline void rnpm_add_active_tail(struct sk_buff *head,
										struct sk_buff *tail)
{
	struct sk_buff *old_tail = RNPM_CB(head)->tail;

	if (old_tail) {
		rnpm_merge_active_tail(old_tail);
		old_tail->next = tail;
	} else {
		skb_shinfo(head)->frag_list = tail;
	}

	RNPM_CB(tail)->head = head;
	RNPM_CB(head)->tail = tail;
}

/**
 * rnpm_close_active_frag_list - cleanup pointers on a frag_list skb
 * @head: pointer to head of an active frag list
 *
 * This function will clear the frag_tail_tracker pointer on an active
 * frag_list and returns true if the pointer was actually set
 **/
static inline bool rnpm_close_active_frag_list(struct sk_buff *head)
{
	struct sk_buff *tail = RNPM_CB(head)->tail;

	if (!tail)
		return false;

	rnpm_merge_active_tail(tail);

	RNPM_CB(head)->tail = NULL;

	return true;
}

#endif

/**
 * rnpm_is_non_eop - process handling of non-EOP buffers
 * @rx_ring: Rx ring being processed
 * @rx_desc: Rx descriptor for current buffer
 * @skb: Current socket buffer containing buffer in progress
 *
 * This function updates next to clean.  If the buffer is an EOP buffer
 * this function exits returning false, otherwise it will place the
 * sk_buff in the next buffer to be chained and return true indicating
 * that this is in fact a non-EOP buffer.
 **/
static bool rnpm_is_non_eop(struct rnpm_ring *rx_ring,
							union rnpm_rx_desc *rx_desc,
							struct sk_buff *skb)
{
	u32 ntc = rx_ring->next_to_clean + 1;
#ifdef CONFIG_RNPM_DISABLE_PACKET_SPLIT
	struct sk_buff *next_skb;
#endif
	/* fetch, update, and store next to clean */
	ntc = (ntc < rx_ring->count) ? ntc : 0;
	rx_ring->next_to_clean = ntc;

	prefetch(RNPM_RX_DESC(rx_ring, ntc));

	/* if we are the last buffer then there is nothing else to do */
	if (likely(rnpm_test_staterr(rx_desc, RNPM_RXD_STAT_EOP)))
		return false;
#ifdef CONFIG_RNPM_RNPM_DISABLE_PACKET_SPLIT
	next_skb = rx_ring->rx_buffer_info[ntc].skb;

	rnpm_add_active_tail(skb, next_skb);
	RNPM_CB(next_skb)->head = skb;
#else
	/* place skb in next buffer to be received */
	rx_ring->rx_buffer_info[ntc].skb = skb;
#endif
	rx_ring->rx_stats.non_eop_descs++;

	return true;
}

// try to add new method

#if (PAGE_SIZE < 8192)
#define RNPM_MAX_2K_FRAME_BUILD_SKB (RNPM_RXBUFFER_1536 - NET_IP_ALIGN)
#define RNPM_2K_TOO_SMALL_WITH_PADDING \
	((NET_SKB_PAD + RNPM_RXBUFFER_1536) > SKB_WITH_OVERHEAD(RNPM_RXBUFFER_2K))

static inline int rnpm_compute_pad(int rx_buf_len)
{
	int page_size, pad_size;

	page_size = ALIGN(rx_buf_len, PAGE_SIZE / 2);
	pad_size = SKB_WITH_OVERHEAD(page_size) - rx_buf_len;

	return pad_size;
}

static inline int rnpm_skb_pad(void)
{
	int rx_buf_len;

	/* If a 2K buffer cannot handle a standard Ethernet frame then
	 * optimize padding for a 3K buffer instead of a 1.5K buffer.
	 *
	 * For a 3K buffer we need to add enough padding to allow for
	 * tailroom due to NET_IP_ALIGN possibly shifting us out of
	 * cache-line alignment.
	 */
	if (RNPM_2K_TOO_SMALL_WITH_PADDING)
		rx_buf_len = RNPM_RXBUFFER_3K + SKB_DATA_ALIGN(NET_IP_ALIGN);
	else
		rx_buf_len = RNPM_RXBUFFER_1536;

	/* if needed make room for NET_IP_ALIGN */
	rx_buf_len -= NET_IP_ALIGN;
	return rnpm_compute_pad(rx_buf_len);
}

#define RNPM_SKB_PAD rnpm_skb_pad()
#else
#define RNPM_SKB_PAD (NET_SKB_PAD + NET_IP_ALIGN)
#endif

#ifndef CONFIG_RNPM_DISABLE_PACKET_SPLIT

static inline unsigned int rnpm_rx_offset(struct rnpm_ring *rx_ring)
{
	return ring_uses_build_skb(rx_ring) ? RNPM_SKB_PAD : 0;
}

static bool rnpm_alloc_mapped_page(struct rnpm_ring *rx_ring,
								   struct rnpm_rx_buffer *bi)
{
	struct page *page = bi->page;
	dma_addr_t dma;
#if defined(HAVE_STRUCT_DMA_ATTRS) && defined(HAVE_SWIOTLB_SKIP_CPU_SYNC)
	DEFINE_DMA_ATTRS(attrs);

	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
	dma_set_attr(DMA_ATTR_WEAK_ORDERING, &attrs);
#endif

	/* since we are recycling buffers we should seldom need to alloc */
	if (likely(page))
		return true;
	// printk("numa is %d\n", numa_node_id());

	/* alloc new page for storage */
	page = alloc_pages(GFP_ATOMIC | __GFP_COLD | __GFP_COMP,
					   rnpm_rx_pg_order(rx_ring));
	if (unlikely(!page)) {
		rx_ring->rx_stats.alloc_rx_page_failed++;
		return false;
	}

	/* map page for use */
	dma = dma_map_page_attrs(rx_ring->dev,
							 page,
							 0,
							 rnpm_rx_pg_size(rx_ring),
							 DMA_FROM_DEVICE,
#if defined(HAVE_STRUCT_DMA_ATTRS) && defined(HAVE_SWIOTLB_SKIP_CPU_SYNC)
							 &attrs);
#else
							 RNPM_RX_DMA_ATTR);
#endif

	/*
	 * if mapping failed free memory back to system since
	 * there isn't much point in holding memory we can't use
	 */
	if (dma_mapping_error(rx_ring->dev, dma)) {
		__free_pages(page, rnpm_rx_pg_order(rx_ring));

		rx_ring->rx_stats.alloc_rx_page_failed++;
		return false;
	}
	/* used temp */
	// rx_ring->rx_stats.alloc_rx_page_failed++;
	bi->dma = dma;
	bi->page = page;
	bi->page_offset = rnpm_rx_offset(rx_ring);
#ifdef HAVE_PAGE_COUNT_BULK_UPDATE
	page_ref_add(page, USHRT_MAX - 1);
	bi->pagecnt_bias = USHRT_MAX;
#else
	bi->pagecnt_bias = 1;
#endif
	rx_ring->rx_stats.alloc_rx_page++;

	return true;
}
/**
 * rnpm_pull_tail - rnpm specific version of skb_pull_tail
 * @skb: pointer to current skb being adjusted
 *
 * This function is an rnpm specific version of __pskb_pull_tail.  The
 * main difference between this version and the original function is that
 * this function can make several assumptions about the state of things
 * that allow for significant optimizations versus the standard function.
 * As a result we can do things like drop a frag and maintain an accurate
 * truesize for the skb.
 */
static void rnpm_pull_tail(struct sk_buff *skb)
{
	skb_frag_t *frag = &skb_shinfo(skb)->frags[0];
	unsigned char *va;
	unsigned int pull_len;

	/*
	 * it is valid to use page_address instead of kmap since we are
	 * working with pages allocated out of the lomem pool per
	 * alloc_page(GFP_ATOMIC)
	 */
	va = skb_frag_address(frag);

	/*
	 * we need the header to contain the greater of either ETH_HLEN or
	 * 60 bytes if the skb->len is less than 60 for skb_pad.
	 */
	pull_len = rnpm_get_headlen(va, RNPM_RX_HDR_SIZE);
	/* align pull length to size of long to optimize memcpy performance */
	skb_copy_to_linear_data(skb, va, ALIGN(pull_len, sizeof(long)));
	/* update all of the pointers */
	skb_frag_size_sub(frag, pull_len);
	skb_frag_off_add(frag, pull_len);
	skb->data_len -= pull_len;
	skb->tail += pull_len;
}

/**
 * rnpm_dma_sync_frag - perform DMA sync for first frag of SKB
 * @rx_ring: rx descriptor ring packet is being transacted on
 * @skb: pointer to current skb being updated
 *
 * This function provides a basic DMA sync up for the first fragment of an
 * skb.  The reason for doing this is that the first fragment cannot be
 * unmapped until we have reached the end of packet descriptor for a buffer
 * chain.
 */
__maybe_unused static void rnpm_dma_sync_frag(struct rnpm_ring *rx_ring,
											  struct sk_buff *skb)
{
#if defined(HAVE_STRUCT_DMA_ATTRS) && defined(HAVE_SWIOTLB_SKIP_CPU_SYNC)
	DEFINE_DMA_ATTRS(attrs);

	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
	dma_set_attr(DMA_ATTR_WEAK_ORDERING, &attrs);

#endif
	/* if the page was released unmap it, else just sync our portion */
	if (unlikely(RNPM_CB(skb)->page_released)) {
		dma_unmap_page_attrs(rx_ring->dev,
							 RNPM_CB(skb)->dma,
							 rnpm_rx_pg_size(rx_ring),
							 DMA_FROM_DEVICE,
#if defined(HAVE_STRUCT_DMA_ATTRS) && defined(HAVE_SWIOTLB_SKIP_CPU_SYNC)
							 &attrs);
#else
							 RNPM_RX_DMA_ATTR);
#endif
	} else if (ring_uses_build_skb(rx_ring)) {
		unsigned long offset = (unsigned long)(skb->data) & ~PAGE_MASK;

		dma_sync_single_range_for_cpu(rx_ring->dev,
									  RNPM_CB(skb)->dma,
									  offset,
									  skb_headlen(skb),
									  DMA_FROM_DEVICE);
	} else {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[0];

		dma_sync_single_range_for_cpu(rx_ring->dev,
									  RNPM_CB(skb)->dma,
									  skb_frag_off(frag),
									  skb_frag_size(frag),
									  DMA_FROM_DEVICE);
	}
}

/* drop this packets if error */
static bool rnpm_check_csum_error(struct rnpm_ring *rx_ring,
								  union rnpm_rx_desc *rx_desc,
								  unsigned int size,
								  unsigned int *driver_drop_packets)
{
	bool err = false;

	struct net_device *netdev = rx_ring->netdev;

	if (netdev->features & NETIF_F_RXCSUM) {
		if (unlikely(rnpm_test_staterr(rx_desc, RNPM_RXD_STAT_ERR_MASK))) {
			rx_debug_printk(
				"rx error: VEB:%s mark:0x%x cmd:0x%x\n",
				(rx_ring->q_vector->adapter->flags & RNP_FLAG_SRIOV_ENABLED)
					? "On"
					: "Off",
				rx_desc->wb.mark,
				rx_desc->wb.cmd);
			/* push this packet to stack if in promisc mode */
			rx_ring->rx_stats.csum_err++;

			if ((!(netdev->flags & IFF_PROMISC) &&
				 (!(netdev->features & NETIF_F_RXALL)))) {
				if (unlikely(
						rnpm_test_staterr(rx_desc, RNPM_RXD_STAT_SCTP_MASK))) {
					if (size > 60) {
						err = true;
						//      return true;
					} else {
						/* sctp less than 60 hw report err by mistake */
						rx_ring->rx_stats.csum_err--;
					}
				} else {
					err = true;
				}
			}
		}
	}
	if (err) {

		struct rnpm_rx_buffer *rx_buffer;
		u32 ntc = rx_ring->next_to_clean + 1;
#if (PAGE_SIZE < 8192)
		unsigned int truesize = rnpm_rx_pg_size(rx_ring) / 2;
#else
		unsigned int truesize = ring_uses_build_skb(rx_ring)
									? SKB_DATA_ALIGN(RNPM_SKB_PAD + size)
									: SKB_DATA_ALIGN(size);
#endif

		// if eop add drop_packets
		if (likely(rnpm_test_staterr(rx_desc, RNPM_RXD_STAT_EOP)))
			*driver_drop_packets = *driver_drop_packets + 1;

		/* we are reusing so sync this buffer for CPU use */
		rx_buffer = &rx_ring->rx_buffer_info[rx_ring->next_to_clean];
		dma_sync_single_range_for_cpu(rx_ring->dev,
									  rx_buffer->dma,
									  rx_buffer->page_offset,
									  RNPM_RXBUFFER_1536,
									  DMA_FROM_DEVICE);

		// rx_buffer->pagecnt_bias--;

#if (PAGE_SIZE < 8192)
		rx_buffer->page_offset ^= truesize;
#else
		rx_buffer->page_offset += truesize;
#endif
		rnpm_put_rx_buffer(rx_ring, rx_buffer, NULL);
		// update to the next desc
		ntc = (ntc < rx_ring->count) ? ntc : 0;
		rx_ring->next_to_clean = ntc;
	}

	return err;
}

/**
 * rnpm_cleanup_headers - Correct corrupted or empty headers
 * @rx_ring: rx descriptor ring packet is being transacted on
 * @rx_desc: pointer to the EOP Rx descriptor
 * @skb: pointer to current skb being fixed
 *
 * Check if the skb is valid. In the XDP case it will be an error pointer.
 * Return true in this case to abort processing and advance to next
 * descriptor.
 *
 * Check for corrupted packet headers caused by senders on the local L2
 * embedded NIC switch not setting up their Tx Descriptors right.  These
 * should be very rare.
 *
 * Also address the case where we are pulling data in on pages only
 * and as such no data is present in the skb header.
 *
 * In addition if skb is not at least 60 bytes we need to pad it so that
 * it is large enough to qualify as a valid Ethernet frame.
 *
 * Returns true if an error was encountered and skb was freed.
 **/
static bool rnpm_cleanup_headers(struct rnpm_ring __maybe_unused *rx_ring,
								 union rnpm_rx_desc *rx_desc,
								 struct sk_buff *skb)
{
	// struct net_device *netdev = rx_ring->netdev;
	/* XDP packets use error pointer so abort at this point */
	if (IS_ERR(skb))
		return true;

	//	if (netdev->features & NETIF_F_RXCSUM) {
	//		if (unlikely(rnpm_test_staterr(rx_desc, RNPM_RXD_STAT_ERR_MASK) &&
	//				!(netdev->features & NETIF_F_RXALL) &&
	//				!ignore_veb_pkg_err(rx_ring->q_vector->adapter,
	//				rx_desc))) {
	//			rx_debug_printk("rx error: VEB:%s mark:0x%x cmd:0x%x\n",
	//			(rx_ring->q_vector->adapter->flags &
	//				RNPM_FLAG_SRIOV_ENABLED) ? "On" : "Off",
	//			rx_desc->wb.mark, rx_desc->wb.cmd);
	//			/* push this packet to stack if in promisc mode */
	//			rx_ring->rx_stats.csum_err++;
	//			if ((!(netdev->flags & IFF_PROMISC) &&
	//				(!(netdev->features & NETIF_F_RXALL)))) {
	//				/*
	//				 * sctp less than 60 hw may report
	//				 * check err by mistake
	//				 */
	//				if (unlikely(rnpm_test_staterr(rx_desc,
	//					RNPM_RXD_STAT_SCTP_MASK))) {
	//					if (skb->len > 60) {
	//						dev_kfree_skb_any(skb);
	//						return true;
	//					}
	//					rx_ring->rx_stats.csum_err--;
	//				} else {
	//					dev_kfree_skb_any(skb);
	//					return true;
	//				}
	//			}
	//		}
	//	}
	/* place header in linear portion of buffer */
	if (!skb_headlen(skb))
		rnpm_pull_tail(skb);

	/* if eth_skb_pad returns an error the skb was freed */
	if (eth_skb_pad(skb))
		return true;

	return false;
}

/**
 * rnpm_reuse_rx_page - page flip buffer and store it back on the ring
 * @rx_ring: rx descriptor ring to store buffers on
 * @old_buff: donor buffer to have page reused
 *
 * Synchronizes page for reuse by the adapter
 **/
static void rnpm_reuse_rx_page(struct rnpm_ring *rx_ring,
							   struct rnpm_rx_buffer *old_buff)
{
	struct rnpm_rx_buffer *new_buff;
	u16 nta = rx_ring->next_to_alloc;

	new_buff = &rx_ring->rx_buffer_info[nta];

	/* update, and store next to alloc */
	nta++;
	rx_ring->next_to_alloc = (nta < rx_ring->count) ? nta : 0;

	/* Transfer page from old buffer to new buffer.
	 * Move each member individually to avoid possible store
	 * forwarding stalls and unnecessary copy of skb.
	 */
	new_buff->dma = old_buff->dma;
	new_buff->page = old_buff->page;
	new_buff->page_offset = old_buff->page_offset;
	new_buff->pagecnt_bias = old_buff->pagecnt_bias;
}

static inline bool rnpm_page_is_reserved(struct page *page)
{
	return (page_to_nid(page) != numa_mem_id()) || page_is_pfmemalloc(page);
}

static bool rnpm_can_reuse_rx_page(struct rnpm_rx_buffer *rx_buffer)
{
	unsigned int pagecnt_bias = rx_buffer->pagecnt_bias;
	struct page *page = rx_buffer->page;

	/* avoid re-using remote pages */
	if (unlikely(rnpm_page_is_reserved(page)))
		return false;
#if (PAGE_SIZE < 8192)
		/* if we are only owner of page we can reuse it */
#ifdef HAVE_PAGE_COUNT_BULK_UPDATE
	if (unlikely((page_ref_count(page) - pagecnt_bias) > 1))
#else
	if (unlikely((page_count(page) - pagecnt_bias) > 1))
#endif
		return false;
#else
		/* The last offset is a bit aggressive in that we assume the
		 * worst case of FCoE being enabled and using a 3K buffer.
		 * However this should have minimal impact as the 1K extra is
		 * still less than one buffer in size.
		 */
#define RNPM_LAST_OFFSET (SKB_WITH_OVERHEAD(PAGE_SIZE) - RNPM_RXBUFFER_2K)
	if (rx_buffer->page_offset > RNPM_LAST_OFFSET)
		return false;
#endif

#ifdef HAVE_PAGE_COUNT_BULK_UPDATE
	/* If we have drained the page fragment pool we need to update
	 * the pagecnt_bias and page count so that we fully restock the
	 * number of references the driver holds.
	 */
	if (unlikely(pagecnt_bias == 1)) {
		page_ref_add(page, USHRT_MAX - 1);
		rx_buffer->pagecnt_bias = USHRT_MAX;
	}
#else
	/* Even if we own the page, we are not allowed to use atomic_set()
	 * This would break get_page_unless_zero() users.
	 */
	if (likely(!pagecnt_bias)) {
		page_ref_inc(page);
		rx_buffer->pagecnt_bias = 1;
	}
#endif

	return true;
}

/**
 * rnpm_add_rx_frag - Add contents of Rx buffer to sk_buff
 * @rx_ring: rx descriptor ring to transact packets on
 * @rx_buffer: buffer containing page to add
 * @skb: sk_buff to place the data into
 * @size: size of data
 *
 * This function will add the data contained in rx_buffer->page to the skb.
 * This is done either through a direct copy if the data in the buffer is
 * less than the skb header size, otherwise it will just attach the page as
 * a frag to the skb.
 *
 * The function will then update the page offset if necessary and return
 * true if the buffer can be reused by the adapter.
 **/
static void rnpm_add_rx_frag(struct rnpm_ring *rx_ring,
							 struct rnpm_rx_buffer *rx_buffer,
							 struct sk_buff *skb,
							 unsigned int size)
{
#if (PAGE_SIZE < 8192)
	unsigned int truesize = rnpm_rx_pg_size(rx_ring) / 2;
#else
	unsigned int truesize = ring_uses_build_skb(rx_ring)
								? SKB_DATA_ALIGN(RNPM_SKB_PAD + size)
								: SKB_DATA_ALIGN(size);
#endif

	skb_add_rx_frag(skb,
					skb_shinfo(skb)->nr_frags,
					rx_buffer->page,
					rx_buffer->page_offset,
					size,
					truesize);

#if (PAGE_SIZE < 8192)
	rx_buffer->page_offset ^= truesize;
#else
	rx_buffer->page_offset += truesize;
#endif
}

static struct rnpm_rx_buffer *rnpm_get_rx_buffer(struct rnpm_ring *rx_ring,
												 union rnpm_rx_desc *rx_desc,
												 struct sk_buff **skb,
												 const unsigned int size)
{
	struct rnpm_rx_buffer *rx_buffer;

	rx_buffer = &rx_ring->rx_buffer_info[rx_ring->next_to_clean];
	prefetchw(rx_buffer->page);
	*skb = rx_buffer->skb;

	rx_buf_dump("rx buf",
				page_address(rx_buffer->page) + rx_buffer->page_offset,
				rx_desc->wb.len);

	/* Delay unmapping of the first packet. It carries the header
	 * information, HW may still access the header after the writeback.
	 * Only unmap it when EOP is reached
	 */

	/* our hw not access the header!! so no need delay unmap */
	/*
	if (!rnpm_test_staterr(rx_desc, RNPM_RXD_STAT_EOP))
	{
		if (!*skb)
			goto skip_sync;
	} else {
		//free the header
		if (*skb)
			rnpm_dma_sync_frag(rx_ring, *skb);
	}
	*/

	/* we are reusing so sync this buffer for CPU use */
	dma_sync_single_range_for_cpu(rx_ring->dev,
								  rx_buffer->dma,
								  rx_buffer->page_offset,
								  size,
								  DMA_FROM_DEVICE);
	// skip_sync:
	rx_buffer->pagecnt_bias--;

	return rx_buffer;
}

static void rnpm_put_rx_buffer(struct rnpm_ring *rx_ring,
							   struct rnpm_rx_buffer *rx_buffer,
							   struct sk_buff *skb)
{
#if defined(HAVE_STRUCT_DMA_ATTRS) && defined(HAVE_SWIOTLB_SKIP_CPU_SYNC)
	DEFINE_DMA_ATTRS(attrs);

	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
	dma_set_attr(DMA_ATTR_WEAK_ORDERING, &attrs);

#endif

	if (!rx_buffer || !rx_buffer->page || !rx_ring) {
		rnpm_info("rnpm rx buffer is null!\n");
		WARN_ON(1);
		return;
	}

	if (rnpm_can_reuse_rx_page(rx_buffer)) {
		/* hand second half of page back to the ring */
		rnpm_reuse_rx_page(rx_ring, rx_buffer);
	} else {
		/* no need to delay unmap */
		//	if (!IS_ERR(skb) && RNPM_CB(skb)->dma == rx_buffer->dma) {
		//		/* the page has been released from the ring */
		//		RNPM_CB(skb)->page_released = true;
		//	} else {
		/* we are not reusing the buffer so unmap it */
		dma_unmap_page_attrs(rx_ring->dev,
							 rx_buffer->dma,
							 rnpm_rx_pg_size(rx_ring),
							 DMA_FROM_DEVICE,
#if defined(HAVE_STRUCT_DMA_ATTRS) && defined(HAVE_SWIOTLB_SKIP_CPU_SYNC)
							 &attrs);
#else
							 RNPM_RX_DMA_ATTR);
#endif
		//		}
		__page_frag_cache_drain(rx_buffer->page, rx_buffer->pagecnt_bias);
	}

	/* clear contents of rx_buffer */
	rx_buffer->page = NULL;
	rx_buffer->skb = NULL;
}

static struct sk_buff *rnpm_construct_skb(struct rnpm_ring *rx_ring,
										  struct rnpm_rx_buffer *rx_buffer,
										  struct xdp_buff *xdp,
										  union rnpm_rx_desc *rx_desc)
{
	unsigned int size = xdp->data_end - xdp->data;
#if (PAGE_SIZE < 8192)
	unsigned int truesize = rnpm_rx_pg_size(rx_ring) / 2;
#else
	unsigned int truesize =
		SKB_DATA_ALIGN(xdp->data_end - xdp->data_hard_start);
#endif
	struct sk_buff *skb;

	/* prefetch first cache line of first page */
	prefetch(xdp->data);
#if L1_CACHE_BYTES < 128
	prefetch(xdp->data + L1_CACHE_BYTES);
#endif
	/* Note, we get here by enabling legacy-rx via:
	 *
	 *    ethtool --set-priv-flags <dev> legacy-rx on
	 *
	 * In this mode, we currently get 0 extra XDP headroom as
	 * opposed to having legacy-rx off, where we process XDP
	 * packets going to stack via rnpm_build_skb(). The latter
	 * provides us currently with 192 bytes of headroom.
	 *
	 * For rnpm_construct_skb() mode it means that the
	 * xdp->data_meta will always point to xdp->data, since
	 * the helper cannot expand the head. Should this ever
	 * change in future for legacy-rx mode on, then lets also
	 * add xdp->data_meta handling here.
	 */

	/* allocate a skb to store the frags */
	skb = napi_alloc_skb(&rx_ring->q_vector->napi, RNPM_RX_HDR_SIZE);
	if (unlikely(!skb))
		return NULL;

	prefetchw(skb->data);

	if (size > RNPM_RX_HDR_SIZE) {
		/*
		if (!rnpm_test_staterr(rx_desc, RNPM_RXD_STAT_EOP))
			RNPM_CB(skb)->dma = rx_buffer->dma;
		*/

		skb_add_rx_frag(skb,
						0,
						rx_buffer->page,
						xdp->data - page_address(rx_buffer->page),
						size,
						truesize);
#if (PAGE_SIZE < 8192)
		rx_buffer->page_offset ^= truesize;
#else
		rx_buffer->page_offset += truesize;
#endif
	} else {
		memcpy(__skb_put(skb, size), xdp->data, ALIGN(size, sizeof(long)));
		rx_buffer->pagecnt_bias++;
	}

	return skb;
}

#ifdef HAVE_SWIOTLB_SKIP_CPU_SYNC
static struct sk_buff *rnpm_build_skb(struct rnpm_ring *rx_ring,
									  struct rnpm_rx_buffer *rx_buffer,
									  struct xdp_buff *xdp,
									  union rnpm_rx_desc *rx_desc)
{
#ifdef HAVE_XDP_BUFF_DATA_META
	unsigned int metasize = xdp->data - xdp->data_meta;
	void *va = xdp->data_meta;
#else
	void *va = xdp->data;
#endif
#if (PAGE_SIZE < 8192)
	unsigned int truesize = rnpm_rx_pg_size(rx_ring) / 2;
#else
	unsigned int truesize =
		SKB_DATA_ALIGN(sizeof(struct skb_shared_info)) +
		SKB_DATA_ALIGN(xdp->data_end - xdp->data_hard_start);
#endif
	struct sk_buff *skb;

	/* prefetch first cache line of first page */
	prefetch(va);
#if L1_CACHE_BYTES < 128
	prefetch(va + L1_CACHE_BYTES);
#endif

	/* build an skb around the page buffer */
	skb = build_skb(xdp->data_hard_start, truesize);
	if (unlikely(!skb))
		return NULL;

	/* update pointers within the skb to store the data */
	skb_reserve(skb, xdp->data - xdp->data_hard_start);
	__skb_put(skb, xdp->data_end - xdp->data);
#ifdef HAVE_XDP_BUFF_DATA_META
	if (metasize)
		skb_metadata_set(skb, metasize);
#endif

		/* we donot need delay free */
		/* record DMA address if this is the start of a chain of buffers */
		/*
		if (!rnpm_test_staterr(rx_desc, RNPM_RXD_STAT_EOP))
			RNPM_CB(skb)->dma = rx_buffer->dma;
		*/

		/* update buffer offset */
#if (PAGE_SIZE < 8192)
	rx_buffer->page_offset ^= truesize;
#else
	rx_buffer->page_offset += truesize;
#endif

	return skb;
}

#endif /* HAVE_SWIOTLB_SKIP_CPU_SYNC */

#define RNPM_XDP_PASS	  0
#define RNPM_XDP_CONSUMED 1
#define RNPM_XDP_TX		  2

static void rnpm_rx_buffer_flip(struct rnpm_ring *rx_ring,
								struct rnpm_rx_buffer *rx_buffer,
								unsigned int size)
{
#if (PAGE_SIZE < 8192)
	unsigned int truesize = rnpm_rx_pg_size(rx_ring) / 2;

	rx_buffer->page_offset ^= truesize;
#else
	unsigned int truesize = ring_uses_build_skb(rx_ring)
								? SKB_DATA_ALIGN(RNPM_SKB_PAD + size)
								: SKB_DATA_ALIGN(size);

	rx_buffer->page_offset += truesize;
#endif
}

/**
 * rnpm_rx_ring_reinit - just reinit rx_ring with new count in ->reset_count
 * @rx_ring: rx descriptor ring to transact packets on
 */
int rnpm_rx_ring_reinit(struct rnpm_adapter *adapter, struct rnpm_ring *rx_ring)
{

	struct rnpm_ring temp_ring;
	int err = 0;
	struct rnpm_hw *hw = &adapter->hw;

	if (rx_ring->count == rx_ring->reset_count)
		return 0;
	/* stop rx queue */

	rnpm_disable_rx_queue(adapter, rx_ring);
	memset(&temp_ring, 0x00, sizeof(struct rnpm_ring));
	/* reinit for this ring */
	memcpy(&temp_ring, rx_ring, sizeof(struct rnpm_ring));
	/* setup new count */
	temp_ring.count = rx_ring->reset_count;
	err = rnpm_setup_rx_resources(&temp_ring, adapter);
	if (err) {
		rnpm_free_rx_resources(&temp_ring);
		goto err_setup;
	}
	rnpm_free_rx_resources(rx_ring);
	memcpy(rx_ring, &temp_ring, sizeof(struct rnpm_ring));
	rnpm_configure_rx_ring(adapter, rx_ring);
err_setup:
	/* start rx */
	wr32(hw, RNPM_DMA_RX_START(rx_ring->rnpm_queue_idx), 1);
	return 0;
}

/**
 * rnpm_clean_rx_irq - Clean completed descriptors from Rx ring - bounce buf
 * @q_vector: structure containing interrupt and ring information
 * @rx_ring: rx descriptor ring to transact packets on
 * @budget: Total limit on number of packets to process
 *
 * This function provides a "bounce buffer" approach to Rx interrupt
 * processing.  The advantage to this is that on systems that have
 * expensive overhead for IOMMU access this provides a means of avoiding
 * it by maintaining the mapping of the page to the syste.
 *
 * Returns amount of work completed.
 **/

static int rnpm_clean_rx_irq(struct rnpm_q_vector *q_vector,
							 struct rnpm_ring *rx_ring,
							 int budget)
{
	unsigned int total_rx_bytes = 0, total_rx_packets = 0;
	unsigned int driver_drop_packets = 0;
	struct rnpm_adapter *adapter = q_vector->adapter;
	u16 cleaned_count = rnpm_desc_unused_rx(rx_ring);
	bool xdp_xmit = false;
	struct xdp_buff xdp;

	xdp.data = NULL;
	xdp.data_end = NULL;

	/*
	#ifdef HAVE_XDP_BUFF_RXQ
		xdp.rxq = &rx_ring->xdp_rxq;
	#endif
	*/
	// rx_ring->rx_stats.poll_count++;
	while (likely(total_rx_packets < budget)) {
		union rnpm_rx_desc *rx_desc;
		struct rnpm_rx_buffer *rx_buffer;
		struct sk_buff *skb;
		unsigned int size;

		/* return some buffers to hardware, one at a time is too slow */
		if (cleaned_count >= RNPM_RX_BUFFER_WRITE) {
			rnpm_alloc_rx_buffers(rx_ring, cleaned_count);
			cleaned_count = 0;
		}
		rx_desc = RNPM_RX_DESC(rx_ring, rx_ring->next_to_clean);

		rx_buf_dump("rx-desc:", rx_desc, sizeof(*rx_desc));
		// buf_dump("rx-desc:", rx_desc, sizeof(*rx_desc));
		rx_debug_printk("  dd set: %s\n",
						(rx_desc->wb.cmd & RNPM_RXD_STAT_DD) ? "Yes" : "No");

		if (!rnpm_test_staterr(rx_desc, RNPM_RXD_STAT_DD))
			break;

		rx_debug_printk(
			"queue:%d  rx-desc:%d has-data len:%d next_to_clean %d\n",
			rx_ring->rnpm_queue_idx,
			rx_ring->next_to_clean,
			rx_desc->wb.len,
			rx_ring->next_to_clean);

		/* handle padding */
		if ((adapter->priv_flags & RNPM_PRIV_FLAG_PCIE_CACHE_ALIGN_PATCH) &&
			(!(adapter->priv_flags & RNPM_PRIV_FLAG_PADDING_DEBUG))) {
			if (likely(rnpm_test_staterr(rx_desc, RNPM_RXD_STAT_EOP))) {
				size = le16_to_cpu(rx_desc->wb.len) -
					   le16_to_cpu(rx_desc->wb.padding_len);
			} else {
				size = le16_to_cpu(rx_desc->wb.len);
			}
		} else {
			/* size should not zero */
			size = le16_to_cpu(rx_desc->wb.len);
		}

		if (!size)
			break;

		if (rnpm_check_csum_error(
				rx_ring, rx_desc, size, &driver_drop_packets)) {
			cleaned_count++;
			continue;
		}

		/* This memory barrier is needed to keep us from reading
		 * any other fields out of the rx_desc until we know the
		 * descriptor has been written back
		 */
		dma_rmb();

		rx_buffer = rnpm_get_rx_buffer(rx_ring, rx_desc, &skb, size);

		if (!skb) {
			xdp.data = page_address(rx_buffer->page) + rx_buffer->page_offset;
#ifdef HAVE_XDP_BUFF_DATA_META
			xdp.data_meta = xdp.data;
#endif
			xdp.data_hard_start = xdp.data - rnpm_rx_offset(rx_ring);
			xdp.data_end = xdp.data + size;
			/* call  xdp hook  use this to support xdp hook */
			// skb = rnpm_run_xdp(adapter, rx_ring, &xdp);
		}

		if (IS_ERR(skb)) {
			if (PTR_ERR(skb) == -RNPM_XDP_TX) {
				xdp_xmit = true;
				rnpm_rx_buffer_flip(rx_ring, rx_buffer, size);
			} else {
				rx_buffer->pagecnt_bias++;
			}
			total_rx_packets++;
			total_rx_bytes += size;
		} else if (skb) {
			rnpm_add_rx_frag(rx_ring, rx_buffer, skb, size);
#ifdef HAVE_SWIOTLB_SKIP_CPU_SYNC
		} else if (ring_uses_build_skb(rx_ring)) {
			skb = rnpm_build_skb(rx_ring, rx_buffer, &xdp, rx_desc);
#endif
		} else {
			skb = rnpm_construct_skb(rx_ring, rx_buffer, &xdp, rx_desc);
		}

		/* exit if we failed to retrieve a buffer */
		if (!skb) {
			rx_ring->rx_stats.alloc_rx_buff_failed++;
			rx_buffer->pagecnt_bias++;
			break;
		}

#ifndef NO_PTP
		if (module_enable_ptp && adapter->ptp_rx_en &&
			adapter->flags2 & RNPM_FLAG2_PTP_ENABLED) {
			rnpm_ptp_get_rx_hwstamp(adapter, rx_desc, skb);
		}
#endif
		rnpm_put_rx_buffer(rx_ring, rx_buffer, skb);
		cleaned_count++;

		/* place incomplete frames back on ring for completion */
		if (rnpm_is_non_eop(rx_ring, rx_desc, skb))
			continue;

		/* verify the packet layout is correct */
		if (rnpm_cleanup_headers(rx_ring, rx_desc, skb))
			continue;

		/* probably a little skewed due to removing CRC */
		total_rx_bytes += skb->len;
		total_rx_packets++;

		/* populate checksum, timestamp, VLAN, and protocol */
		rnpm_process_skb_fields(rx_ring, rx_desc, skb);

		rnpm_rx_skb(q_vector, skb);

		/* update budget accounting */
	}

	//	if (xdp_xmit) {
	//		struct rnpm_ring *ring = adapter->xdp_ring[smp_processor_id()];
	//
	//		/* Force memory writes to complete before letting h/w
	//		 * know there are new descriptors to fetch.
	//		 */
	//		wmb();
	//		writel(ring->next_to_use, ring->tail);
	//
	//		xdp_do_flush_map();
	//	}

	u64_stats_update_begin(&rx_ring->syncp);
	rx_ring->stats.packets += total_rx_packets;
	rx_ring->stats.bytes += total_rx_bytes;
	rx_ring->rx_stats.driver_drop_packets += driver_drop_packets;
	u64_stats_update_end(&rx_ring->syncp);
	// q_vector->rx.total_packets += total_rx_packets;
	// q_vector->rx.total_bytes += total_rx_bytes;

	if (total_rx_packets >= budget)
		rx_ring->rx_stats.poll_again_count++;
	return total_rx_packets;
}
#else /* CONFIG_RNPM_DISABLE_PACKET_SPLIT */

static bool rnpm_alloc_mapped_skb(struct rnpm_ring *rx_ring,
								  struct rnpm_rx_buffer *bi)
{
	struct sk_buff *skb = bi->skb;
	dma_addr_t dma = bi->dma;

	if (unlikely(dma))
		return true;

	if (likely(!skb)) {
		skb = netdev_alloc_skb_ip_align(rx_ring->netdev, rx_ring->rx_buf_len);
		if (unlikely(!skb)) {
			rx_ring->rx_stats.alloc_rx_buff_failed++;
			return false;
		}

		bi->skb = skb;
	}
	dma = dma_map_single(
		rx_ring->dev, skb->data, rx_ring->rx_buf_len, DMA_FROM_DEVICE);

	/*
	 * if mapping failed free memory back to system since
	 * there isn't much point in holding memory we can't use
	 */
	if (dma_mapping_error(rx_ring->dev, dma)) {
		dev_kfree_skb_any(skb);
		bi->skb = NULL;

		rx_ring->rx_stats.alloc_rx_buff_failed++;
		return false;
	}

	bi->dma = dma;
	return true;
}

/**
 * rnpm_clean_rx_irq - Clean completed descriptors from Rx ring - legacy
 * @q_vector: structure containing interrupt and ring information
 * @rx_ring: rx descriptor ring to transact packets on
 * @budget: Total limit on number of packets to process
 *
 * This function provides a legacy approach to Rx interrupt
 * handling.  This version will perform better on systems with a low cost
 * dma mapping API.
 *
 * Returns amount of work completed.
 **/
static int rnpm_clean_rx_irq(struct rnpm_q_vector *q_vector,
							 struct rnpm_ring *rx_ring,
							 int budget)
{
	unsigned int total_rx_bytes = 0, total_rx_packets = 0;
	struct net_device *netdev = rx_ring->netdev;
	u16 len = 0;
	u16 cleaned_count = rnpm_desc_unused_rx(rx_ring);

	while (likely(total_rx_packets < budget)) {
		struct rnpm_rx_buffer *rx_buffer;
		union rnpm_rx_desc *rx_desc;
		struct sk_buff *skb;
		u16 ntc;

		/* return some buffers to hardware, one at a time is too slow */
		if (cleaned_count >= RNPM_RX_BUFFER_WRITE) {
			rnpm_alloc_rx_buffers(rx_ring, cleaned_count);
			cleaned_count = 0;
		}

		ntc = rx_ring->next_to_clean;
		rx_desc = RNPM_RX_DESC(rx_ring, ntc);
		rx_buffer = &rx_ring->rx_buffer_info[ntc];

		if (!rnpm_test_staterr(rx_desc, RNPM_RXD_STAT_DD))
			break;
		/*
		if (!rx_desc->wb.upper.length)
			break;
		*/

		/* This memory barrier is needed to keep us from reading
		 * any other fields out of the rx_desc until we know the
		 * descriptor has been written back
		 */
		dma_rmb();

		skb = rx_buffer->skb;

		prefetch(skb->data);

		len = le16_to_cpu(rx_desc->wb.len);
		/* pull the header of the skb in */
		__skb_put(skb, len);

		/*
		 * Delay unmapping of the first packet. It carries the
		 * header information, HW may still access the header after
		 * the writeback.  Only unmap it when EOP is reached
		 */
		/* no need to delay unmap */
		if (!RNPM_CB(skb)->head) {
			RNPM_CB(skb)->dma = rx_buffer->dma;
		} else {
			skb = rnpm_merge_active_tail(skb);
			dma_unmap_single(rx_ring->dev,
							 rx_buffer->dma,
							 rx_ring->rx_buf_len,
							 DMA_FROM_DEVICE);
		}

		/* clear skb reference in buffer info structure */
		rx_buffer->skb = NULL;
		rx_buffer->dma = 0;

		cleaned_count++;

		if (rnpm_is_non_eop(rx_ring, rx_desc, skb))
			continue;

		/* unmap first */
		dma_unmap_single(rx_ring->dev,
						 RNPM_CB(skb)->dma,
						 rx_ring->rx_buf_len,
						 DMA_FROM_DEVICE);

		RNPM_CB(skb)->dma = 0;

		if (rnpm_close_active_frag_list(skb) && !RNPM_CB(skb)->append_cnt) {
			/* if we got here without RSC the packet is invalid */
			dev_kfree_skb_any(skb);
			continue;
		}

		/* ERR_MASK will only have valid bits if EOP set */
		if (unlikely(rnpm_test_staterr(rx_desc, RNPM_RXD_STAT_ERR_MASK) &&
					 !(netdev->features & NETIF_F_RXALL)) &&
			!ignore_veb_pkg_err(rx_ring->q_vector->adapter, rx_desc)) {
			dev_kfree_skb_any(skb);
			continue;
		}

		/* probably a little skewed due to removing CRC */
		total_rx_bytes += skb->len;

		/* populate checksum, timestamp, VLAN, and protocol */
		rnpm_process_skb_fields(rx_ring, rx_desc, skb);

		rnpm_rx_skb(q_vector, skb);

		/* update budget accounting */
		total_rx_packets++;
	}

	u64_stats_update_begin(&rx_ring->syncp);
	rx_ring->stats.packets += total_rx_packets;
	rx_ring->stats.bytes += total_rx_bytes;
	u64_stats_update_end(&rx_ring->syncp);
	q_vector->rx.total_packets += total_rx_packets;
	q_vector->rx.total_bytes += total_rx_bytes;

	if (cleaned_count)
		rnpm_alloc_rx_buffers(rx_ring, cleaned_count);

	if (total_rx_packets >= budget)
		rx_ring->rx_stats.poll_again_count++;

	// return (total_rx_packets < budget);
	return total_rx_packets;
}

#endif /* CONFIG_RNPM_DISABLE_PACKET_SPLIT */

/**
 * rnpm_configure_msix - Configure MSI-X hardware
 * @adapter: board private structure
 *
 * rnpm_configure_msix sets up the hardware to properly generate MSI-X
 * interrupts.
 **/
static void rnpm_configure_msix(struct rnpm_adapter *adapter)
{
	struct rnpm_q_vector *q_vector;
	int i;
	// u32 mask;

	// rnpm_dbg("[%s] num_q_vectors:%d\n", __func__, adapter->num_q_vectors);

	/*
	 * configure ring-msix Registers table
	 */
	for (i = 0; i < adapter->num_q_vectors; i++) {
		struct rnpm_ring *ring;

		q_vector = adapter->q_vector[i];
		rnpm_for_each_ring(ring, q_vector->rx) rnpm_set_ring_vector(
			adapter, ring->rnpm_queue_idx, q_vector->v_idx);
	}
}

#if ITR_TEST
/**
 * rnpm_update_itr - update the dynamic ITR value based on statistics
 * @q_vector: structure containing interrupt and ring information
 * @ring_container: structure containing ring performance data
 *
 *      Stores a new ITR value based on packets and byte
 *      counts during the last interrupt.  The advantage of per interrupt
 *      computation is faster updates and more accurate ITR for the current
 *      traffic pattern.  Constants in this function were computed
 *      based on theoretical maximum wire speed and thresholds were set based
 *      on testing data as well as attempting to minimize response time
 *      while increasing bulk throughput.
 **/
static void rnpm_update_itr(struct rnpm_q_vector *q_vector,
							struct rnpm_ring_container *ring_container)
{
	unsigned int itr = RNPM_ITR_ADAPTIVE_MIN_USECS | RNPM_ITR_ADAPTIVE_LATENCY;
	unsigned int avg_wire_size, packets, bytes;
	unsigned long next_update = jiffies;

	/* If we don't have any rings just leave ourselves set for maximum
	 * possible latency so we take ourselves out of the equation.
	 */
	if (!ring_container->ring)
		return;

	/* If we didn't update within up to 1 - 2 jiffies we can assume
	 * that either packets are coming in so slow there hasn't been
	 * any work, or that there is so much work that NAPI is dealing
	 * with interrupt moderation and we don't need to do anything.
	 */
	if (time_after(next_update, ring_container->next_update))
		goto clear_counts;

	packets = ring_container->total_packets;

	/* We have no packets to actually measure against. This means
	 * either one of the other queues on this vector is active or
	 * we are a Tx queue doing TSO with too high of an interrupt rate.
	 *
	 * When this occurs just tick up our delay by the minimum value
	 * and hope that this extra delay will prevent us from being called
	 * without any work on our queue.
	 */
	if (!packets) {
		itr = (q_vector->itr >> 2) + RNPM_ITR_ADAPTIVE_MIN_INC;
		if (itr > RNPM_ITR_ADAPTIVE_MAX_USECS)
			itr = RNPM_ITR_ADAPTIVE_MAX_USECS;
		itr += ring_container->itr & RNPM_ITR_ADAPTIVE_LATENCY;
		goto clear_counts;
	}

	bytes = ring_container->total_bytes;

	/* If packets are less than 4 or bytes are less than 9000 assume
	 * insufficient data to use bulk rate limiting approach. We are
	 * likely latency driven.
	 */
	if (packets < 4 && bytes < 9000) {
		itr = RNPM_ITR_ADAPTIVE_LATENCY;
		goto adjust_by_size;
	}

	/* Between 4 and 48 we can assume that our current interrupt delay
	 * is only slightly too low. As such we should increase it by a small
	 * fixed amount.
	 */
	if (packets < 48) {
		itr = (q_vector->itr >> 2) + RNPM_ITR_ADAPTIVE_MIN_INC;
		if (itr > RNPM_ITR_ADAPTIVE_MAX_USECS)
			itr = RNPM_ITR_ADAPTIVE_MAX_USECS;
		goto clear_counts;
	}

	/* Between 48 and 96 is our "goldilocks" zone where we are working
	 * out "just right". Just report that our current ITR is good for us.
	 */
	if (packets < 96) {
		itr = q_vector->itr >> 2;
		goto clear_counts;
	}

	/* If packet count is 96 or greater we are likely looking at a slight
	 * overrun of the delay we want. Try halving our delay to see if that
	 * will cut the number of packets in half per interrupt.
	 */
	if (packets < 256) {
		itr = q_vector->itr >> 3;
		if (itr < RNPM_ITR_ADAPTIVE_MIN_USECS)
			itr = RNPM_ITR_ADAPTIVE_MIN_USECS;
		goto clear_counts;
	}

	/* The paths below assume we are dealing with a bulk ITR since number
	 * of packets is 256 or greater. We are just going to have to compute
	 * a value and try to bring the count under control, though for smaller
	 * packet sizes there isn't much we can do as NAPI polling will likely
	 * be kicking in sooner rather than later.
	 */
	itr = RNPM_ITR_ADAPTIVE_BULK;

adjust_by_size:
	/* If packet counts are 256 or greater we can assume we have a gross
	 * overestimation of what the rate should be. Instead of trying to fine
	 * tune it just use the formula below to try and dial in an exact value
	 * give the current packet size of the frame.
	 */
	avg_wire_size = bytes / packets;

	/* The following is a crude approximation of:
	 *  wmem_default / (size + overhead) = desired_pkts_per_int
	 *  rate / bits_per_byte / (size + ethernet overhead) = pkt_rate
	 *  (desired_pkt_rate / pkt_rate) * usecs_per_sec = ITR value
	 *
	 * Assuming wmem_default is 212992 and overhead is 640 bytes per
	 * packet, (256 skb, 64 headroom, 320 shared info), we can reduce the
	 * formula down to
	 *
	 *  (170 * (size + 24)) / (size + 640) = ITR
	 *
	 * We first do some math on the packet size and then finally bitshift
	 * by 8 after rounding up. We also have to account for PCIe link speed
	 * difference as ITR scales based on this.
	 */
	if (avg_wire_size <= 60) {
		/* Start at 50k ints/sec */
		avg_wire_size = 5120;
	} else if (avg_wire_size <= 316) {
		/* 50K ints/sec to 16K ints/sec */
		avg_wire_size *= 40;
		avg_wire_size += 2720;
	} else if (avg_wire_size <= 1084) {
		/* 16K ints/sec to 9.2K ints/sec */
		avg_wire_size *= 15;
		avg_wire_size += 11452;
	} else if (avg_wire_size <= 1980) {
		/* 9.2K ints/sec to 8K ints/sec */
		avg_wire_size *= 5;
		avg_wire_size += 22420;
	} else {
		/* plateau at a limit of 8K ints/sec */
		avg_wire_size = 32256;
	}

	/* If we are in low latency mode half our delay which doubles the rate
	 * to somewhere between 100K to 16K ints/sec
	 */
	if (itr & RNPM_ITR_ADAPTIVE_LATENCY)
		avg_wire_size >>= 1;

	/* Resultant value is 256 times larger than it needs to be. This
	 * gives us room to adjust the value as needed to either increase
	 * or decrease the value based on link speeds of 10G, 2.5G, 1G, etc.
	 *
	 * Use addition as we have already recorded the new latency flag
	 * for the ITR value.
	 */
	switch (q_vector->adapter->link_speed) {
		case RNPM_LINK_SPEED_10GB_FULL:
		case RNPM_LINK_SPEED_100_FULL:
		default:
			itr +=
				DIV_ROUND_UP(avg_wire_size, RNPM_ITR_ADAPTIVE_MIN_INC * 256) *
				RNPM_ITR_ADAPTIVE_MIN_INC;
			break;
		// case RNPM_LINK_SPEED_2_5GB_FULL:
		case RNPM_LINK_SPEED_1GB_FULL:
			// case RNPM_LINK_SPEED_10_FULL:
			itr += DIV_ROUND_UP(avg_wire_size, RNPM_ITR_ADAPTIVE_MIN_INC * 64) *
				   RNPM_ITR_ADAPTIVE_MIN_INC;
			break;
	}

clear_counts:
	/* write back value */
	ring_container->itr = itr;

	/* next update should occur within next jiffy */
	ring_container->next_update = next_update + 1;

	ring_container->total_bytes = 0;
	ring_container->total_packets = 0;
}
#endif

/**
 * ixgbe_write_eitr - write EITR register in hardware specific way
 * @q_vector: structure containing interrupt and ring information
 *
 * This function is made to be called by ethtool and by the driver
 * when it needs to update EITR registers at runtime.  Hardware
 * specific quirks/differences are taken care of here.
 */
void rnpm_write_eitr(struct rnpm_q_vector *q_vector)
{
	struct rnpm_adapter *adapter = q_vector->adapter;
	struct rnpm_hw *hw = &adapter->hw;
	// int v_idx = q_vector->v_idx;
	u32 itr_reg = q_vector->itr >> 2;
	struct rnpm_ring *ring;

	itr_reg = itr_reg * hw->usecstocount; // 150M
	rnpm_for_each_ring(ring, q_vector->rx)
	{
		wr32(
			hw, RNPM_DMA_REG_TX_INT_DELAY_TIMER(ring->rnpm_queue_idx), itr_reg);
	}
	rnpm_for_each_ring(ring, q_vector->tx)
	{
		wr32(
			hw, RNPM_DMA_REG_RX_INT_DELAY_TIMER(ring->rnpm_queue_idx), itr_reg);
	}
}

#if ITR_TEST
static void rnpm_set_itr(struct rnpm_q_vector *q_vector)
{
	u32 new_itr;

	rnpm_update_itr(q_vector, &q_vector->tx);
	rnpm_update_itr(q_vector, &q_vector->rx);

	/* use the smallest value of new ITR delay calculations */
	new_itr = min(q_vector->rx.itr, q_vector->tx.itr);

	/* Clear latency flag if set, shift into correct position */
	new_itr &= ~RNPM_ITR_ADAPTIVE_LATENCY;
	/* in 2us unit */
	new_itr <<= 2;

	if (new_itr != q_vector->itr) {
		/* save the algorithm value here */
		q_vector->itr = new_itr;
		// printk("update itr to %x\n", q_vector->itr);
		rnpm_write_eitr(q_vector);
	}
}
#endif
enum latency_range {
	lowest_latency = 0,
	low_latency = 1,
	bulk_latency = 2,
	latency_invalid = 255
};
__maybe_unused static void rnpm_check_sfp_event(struct rnpm_adapter *adapter,
												u32 eicr)
{
	// struct rnpm_hw *hw = &adapter->hw;
}

static inline void rnpm_irq_enable_queues(struct rnpm_adapter *adapter,
										  struct rnpm_q_vector *q_vector)
{
	struct rnpm_ring *ring;
	// struct rnpm_hw *hw = &adapter->hw;

	rnpm_for_each_ring(ring, q_vector->rx)
	{
		// clear irq
		// rnpm_wr_reg(ring->dma_int_clr, RX_INT_MASK | TX_INT_MASK);
		// wmb();
#ifdef CONFIG_RNPM_DISABLE_TX_IRQ
		rnpm_wr_reg(ring->dma_int_mask, ~(RX_INT_MASK));
#else
		rnpm_wr_reg(ring->dma_int_mask, ~(RX_INT_MASK | TX_INT_MASK));
		// rnpm_wr_reg(ring->dma_int_mask, ~(RX_INT_MASK));
#endif
	}
}

static inline void rnpm_irq_disable_queues(struct rnpm_q_vector *q_vector)
{
	struct rnpm_ring *ring;

	rnpm_for_each_ring(ring, q_vector->tx)
	{
		//if (q_vector->new_rx_count != q_vector->old_rx_count) {
		//	ring_wr32(ring, RNPM_DMA_REG_RX_INT_DELAY_PKTCNT(ring->rnpm_queue_idx), q_vector->new_rx_count);
		//	q_vector->old_rx_count = q_vector->new_rx_count;
		//}
		rnpm_wr_reg(ring->dma_int_mask, (RX_INT_MASK | TX_INT_MASK));
		// rnpm_wr_reg(ring->dma_int_clr, RX_INT_MASK | TX_INT_MASK);
	}
}
/**
 * rnpm_irq_enable - Enable default interrupt generation settings
 * @adapter: board private structure
 **/
static inline void rnpm_irq_enable(struct rnpm_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_q_vectors; i++)
		rnpm_irq_enable_queues(adapter, adapter->q_vector[i]);
}

static irqreturn_t rnpm_msix_other(int irq, void *data)
{
	struct rnpm_pf_adapter *pf_adapter = data;
	// struct rnpm_hw *hw = &pf_adapter->hw;
	// u32 eicr;

	// cm3 irq handler can be added here

	// fixme
	rnpm_msg_task(pf_adapter);

#if 0
	/* re-enable the original interrupt state, no lsc, no queues */
	if (!test_bit(__RNPM_DOWN, &adapter->state))
		rnpm_irq_enable(adapter);
#endif

	return IRQ_HANDLED;
}

static void rnpm_htimer_start(struct rnpm_q_vector *q_vector)
{
	unsigned long ns = q_vector->irq_check_usecs * NSEC_PER_USEC / 2;

	hrtimer_start_range_ns(
		&q_vector->irq_miss_check_timer, ns_to_ktime(ns), ns, HRTIMER_MODE_REL_PINNED);
}

static void rnpm_htimer_stop(struct rnpm_q_vector *q_vector)
{
	hrtimer_cancel(&q_vector->irq_miss_check_timer);
}

static irqreturn_t rnpm_msix_clean_rings(int irq, void *data)
{
	struct rnpm_q_vector *q_vector = data;

	rnpm_htimer_stop(q_vector);
	/*  disabled interrupts (on this vector) for us */
	rnpm_irq_disable_queues(q_vector);

	if (q_vector->rx.ring || q_vector->tx.ring)
		napi_schedule_irqoff(&q_vector->napi);

	return IRQ_HANDLED;
}

/**
 * rnpm_poll - NAPI Rx polling callback
 * @napi: structure for representing this polling device
 * @budget: how many packets driver is allowed to clean
 *
 * This function is used for legacy and MSI, NAPI mode
 **/
int rnpm_poll(struct napi_struct *napi, int budget)
{
	struct rnpm_q_vector *q_vector =
		container_of(napi, struct rnpm_q_vector, napi);
	struct rnpm_adapter *adapter = q_vector->adapter;
	struct rnpm_hw *hw = &adapter->hw;
	struct rnpm_ring *ring;
	int per_ring_budget, work_done = 0;
	bool clean_complete = true;

#ifdef CONFIG_RNPM_DCA
	if (adapter->flags & RNPM_FLAG_DCA_ENABLED)
		rnpm_update_dca(q_vector);
#endif
	/* Port is down/reset, but napi_schedule_irqoff is exec by watchdog task or irq_miss_check */
	if (test_bit(__RNPM_RESETTING, &adapter->state) || test_bit(__RNPM_DOWN, &adapter->state))
		return budget;

	rnpm_for_each_ring(ring, q_vector->tx) clean_complete &=
		!!rnpm_clean_tx_irq(q_vector, ring, budget);

	/* attempt to distribute budget to each queue fairly, but don't allow
	 * the budget to go below 1 because we'll exit polling */
	if (q_vector->rx.count > 1)
		per_ring_budget = max(budget / q_vector->rx.count, 1);
	else
		per_ring_budget = budget;

	rnpm_for_each_ring(ring, q_vector->rx)
	{
		int cleaned = 0;
		/* this ring is waitting to reset rx_len*/
		/* avoid to deal this ring until reset done */
		if (likely(!(ring->ring_flags & RNPM_RING_FLAG_DO_RESET_RX_LEN)))
			cleaned = rnpm_clean_rx_irq(q_vector, ring, per_ring_budget);
		/* check delay rx setup */
		if (unlikely(ring->ring_flags & RNPM_RING_FLAG_DELAY_SETUP_RX_LEN)) {
			int head;

			head =
				rd32(hw, RNPM_DMA_REG_RX_DESC_BUF_HEAD(ring->rnpm_queue_idx));
			if (head < RNPM_MIN_RXD) {
				/* it is time to delay set */
				/* stop rx */
				rnpm_disable_rx_queue(adapter, ring);
				ring->ring_flags &= (~RNPM_RING_FLAG_DELAY_SETUP_RX_LEN);
				ring->ring_flags |= RNPM_RING_FLAG_DO_RESET_RX_LEN;
			}
		}
		work_done += cleaned;
		if (cleaned >= per_ring_budget)
			clean_complete = false;
		//if ((cleaned < 10) && (cleaned !=0))
		//	q_vector->new_rx_count = 1;
		//else
		//	q_vector->new_rx_count = adapter->rx_frames;
	}

	/* all work done, exit the polling mode */
	// napi_complete(napi);

	/* If all work not completed, return budget and keep polling */
	if (!clean_complete) {
		// check
#ifdef HAVE_IRQ_AFFINITY_NOTIFY
		int cpu_id = smp_processor_id();

		/* It is possible that the interrupt affinity has changed but,
		 * if the cpu is pegged at 100%, polling will never exit while
		 * traffic continues and the interrupt will be stuck on this
		 * cpu.  We check to make sure affinity is correct before we
		 * continue to poll, otherwise we must stop polling so the
		 * interrupt can move to the correct cpu.
		 */
		if (!cpumask_test_cpu(cpu_id, &q_vector->affinity_mask)) {
			/* Tell napi that we are done polling */
			// napi_complete_done(napi, work_done);
			// printk("affinity error\n");

			/* Force an interrupt */
			// i40e_force_wb(vsi, q_vector);
			napi_complete_done(napi, work_done);
			if (!test_bit(__RNPM_DOWN, &adapter->state))
				rnpm_irq_enable_queues(adapter, q_vector);
				/* we need this to ensure riq start before tx start */
#ifdef TX_IRQ_MISS_REDUCE
			smp_mb();
			rnpm_for_each_ring(ring, q_vector->tx)
				rnpm_check_restart_tx(q_vector, ring);
#endif

			if (!test_bit(__RNPM_DOWN, &adapter->state)) {
				rnpm_htimer_start(q_vector);
				/* Return budget-1 so that polling stops */
				return budget - 1;
			}
		}
#endif /* HAVE_IRQ_AFFINITY_NOTIFY */

#ifdef TX_IRQ_MISS_REDUCE
		rnpm_for_each_ring(ring, q_vector->tx)
			rnpm_check_restart_tx(q_vector, ring);
#endif
		/* do poll only state not down */
		if (!test_bit(__RNPM_DOWN, &adapter->state))
			return budget;
	}

	napi_complete_done(napi, work_done);

	/* try to do itr handle */
#if ITR_TEST
	if (adapter->rx_itr_setting == 1)
		rnpm_set_itr(q_vector);
#endif
	/* only open irq if not down */
	if (!test_bit(__RNPM_DOWN, &adapter->state))
		rnpm_irq_enable_queues(adapter, q_vector);
		/* we need this to ensure irq start before tx start */
#ifdef TX_IRQ_MISS_REDUCE
	smp_mb();
	rnpm_for_each_ring(ring, q_vector->tx) {
		rnpm_check_restart_tx(q_vector, ring);
		// update rx count now?
		//if (q_vector->new_rx_count != q_vector->old_rx_count) {
		//	ring_wr32(ring, RNPM_DMA_REG_RX_INT_DELAY_PKTCNT(ring->rnpm_queue_idx), q_vector->new_rx_count);
		//	q_vector->old_rx_count = q_vector->new_rx_count;
		//}
	}
#endif

	/* only open htimer if net not down */ 
	if (!test_bit(__RNPM_DOWN, &adapter->state))
		rnpm_htimer_start(q_vector);

	return min(work_done, budget - 1);
}

#ifdef HAVE_IRQ_AFFINITY_NOTIFY
/**
 * rnp_irq_affinity_notify - Callback for affinity changes
 * @notify: context as to what irq was changed
 * @mask: the new affinity mask
 *
 * This is a callback function used by the irq_set_affinity_notifier function
 * so that we may register to receive changes to the irq affinity masks.
 **/
static void rnpm_irq_affinity_notify(struct irq_affinity_notify *notify,
                                     const cpumask_t *mask)
{

        struct rnpm_q_vector *q_vector =
                container_of(notify, struct rnpm_q_vector, affinity_notify);

        cpumask_copy(&q_vector->affinity_mask, mask);
}

/**
 * rnp_irq_affinity_release - Callback for affinity notifier release
 * @ref: internal core kernel usage
 *
 * This is a callback function used by the irq_set_affinity_notifier function
 * to inform the current notification subscriber that they will no longer
 * receive notifications.
 **/
static void rnpm_irq_affinity_release(struct kref *ref) {}
#endif /* HAVE_IRQ_AFFINITY_NOTIFY */

/**
 * rnpm_request_msix_irqs - Initialize MSI-X interrupts
 * @adapter: board private structure
 *
 * rnpm_request_msix_irqs allocates MSI-X vectors and requests
 * interrupts from the kernel.
 **/
static int rnpm_request_msix_irqs(struct rnpm_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	// struct rnpm_hw *hw = &adapter->hw;
	int err;
	int i = 0;
#ifdef HAVE_IRQ_AFFINITY_HINT
	int cpu;
#endif

#ifdef RNPM_DISABLE_IRQ
	return 0;
#endif
	DPRINTK(IFUP, INFO, "num_q_vectors:%d\n", adapter->num_q_vectors);

	for (i = 0; i < adapter->num_q_vectors; i++) {
		struct rnpm_q_vector *q_vector = adapter->q_vector[i];

		/* use vector_off offset vector */
		//	struct msix_entry *entry =
		//		&adapter->msix_entries[i + adapter->vector_off];
		struct msix_entry *entry = &adapter->msix_entries[i];

		// rnpm_dbg("use irq %d\n", entry->entry);
		if (q_vector->tx.ring && q_vector->rx.ring) {
			snprintf(q_vector->name,
					 sizeof(q_vector->name) - 1,
					 "%s-%s-%d-%d",
					 netdev->name,
					 "TxRx",
					 i,
					 q_vector->v_idx);
		} else {
			WARN(!(q_vector->tx.ring && q_vector->rx.ring),
				 "%s vector%d tx rx is null, v_idx:%d\n",
				 netdev->name,
				 i,
				 q_vector->v_idx);
			/* skip this unused q_vector */
			continue;
		}
		err = request_irq(
			entry->vector, &rnpm_msix_clean_rings, 0, q_vector->name, q_vector);
		if (err) {
			e_err(probe,
				  "%s:request_irq failed for MSIX interrupt:%d "
				  "Error: %d\n",
				  netdev->name,
				  entry->vector,
				  err);
			goto free_queue_irqs;
		}
#ifdef HAVE_IRQ_AFFINITY_NOTIFY
		/* register for affinity change notifications */
		q_vector->affinity_notify.notify = rnpm_irq_affinity_notify;
		q_vector->affinity_notify.release = rnpm_irq_affinity_release;
		irq_set_affinity_notifier(entry->vector, &q_vector->affinity_notify);
#endif /* HAVE_IRQ_AFFINITY_NOTIFY */
#ifdef HAVE_IRQ_AFFINITY_HINT

		/* Spread affinity hints out across online CPUs.
		 *
		 * get_cpu_mask returns a static constant mask with
		 * a permanent lifetime so it's ok to pass to
		 * irq_set_affinity_hint without making a copy.
		 */
		cpu = cpumask_local_spread(q_vector->v_idx, -1);
		irq_set_affinity_hint(entry->vector, get_cpu_mask(cpu));
		//irq_set_affinity_hint(entry->vector, &q_vector->affinity_mask);
		//DPRINTK(IFUP, INFO, "set %s affinity_mask\n", q_vector->name);
#endif
	}

	return 0;

free_queue_irqs:
	while (i) {
		i--;
		irq_set_affinity_hint(adapter->msix_entries[i].vector, NULL);
#ifdef HAVE_IRQ_AFFINITY_NOTIFY
		irq_set_affinity_notifier(adapter->msix_entries[i].vector, NULL);
#endif
#ifdef HAVE_IRQ_AFFINITY_HINT
		irq_set_affinity_hint(adapter->msix_entries[i].vector, NULL);
#endif
		free_irq(adapter->msix_entries[i].vector, adapter->q_vector[i]);
	}

	// pci_disable_msix(adapter->pdev);
	kfree(adapter->msix_entries);
	adapter->msix_entries = NULL;
	return err;
}

#ifdef DISABLE_RX_IRQ
int rx_poll_thread_handler(void *data)
{
	int i;
	struct rnpm_adapter *adapter = data;
	struct net_device *netdev = adapter->netdev;

	dbg("%s  %s running...\n", __func__, netdev->name);

	do {
		for (i = 0; i < adapter->num_q_vectors; i++)
			rnpm_msix_clean_rings(0, adapter->q_vector[i]);

		usleep_range(1, 4);
	} while (!kthread_should_stop() && adapter->quit_poll_thread != true);

	dbg("%s  %s stoped\n", __func__, netdev->name);
	return 0;
}
#endif

/**
 * rnpm_request_irq - initialize interrupts
 * @adapter: board private structure
 *
 * Attempts to configure interrupts using the best available
 * capabilities of the hardware and kernel.
 **/
static int rnpm_request_irq(struct rnpm_adapter *adapter)
{
	int err;

#ifdef DISABLE_RX_IRQ
	adapter->rx_poll_thread =
		kthread_run(rx_poll_thread_handler, adapter, adapter->name);
	if (!adapter->rx_poll_thread) {
		rnpm_err("kthread_run faild!\n");
		return -EIO;
	}
	return 0;
#endif

	err = rnpm_request_msix_irqs(adapter);
	if (err)
		e_err(probe, "request_irq failed, Error %d\n", err);

	return err;
}

static void rnpm_free_irq(struct rnpm_adapter *adapter)
{
	int i;

#ifdef DISABLE_RX_IRQ
	/*
	if (adapter->rx_poll_thread)
		kthread_stop(adapter->rx_poll_thread);
	*/
	// adapter->rx_poll_thread;
	return;
#endif

#ifdef RNPM_DISABLE_IRQ
	return;
#endif

	// rnpm_dbg("[%s] num_q_vectors:%d\n", __func__, adapter->num_q_vectors);

	for (i = 0; i < adapter->num_q_vectors; i++) {
		struct rnpm_q_vector *q_vector = adapter->q_vector[i];
		struct msix_entry *entry = &adapter->msix_entries[i];

		/* free only the irqs that were actually requested */
		if (!q_vector->rx.ring && !q_vector->tx.ring) {
			continue;
		}
#ifdef HAVE_IRQ_AFFINITY_NOTIFY
		/* clear the affinity notifier in the IRQ descriptor */
		irq_set_affinity_notifier(adapter->msix_entries[i].vector, NULL);
#endif
#ifdef HAVE_IRQ_AFFINITY_HINT
		/* clear the affinity_mask in the IRQ descriptor */
		irq_set_affinity_hint(entry->vector, NULL);
#endif
		DPRINTK(IFDOWN, INFO, "free irq %s \n", q_vector->name);
		free_irq(entry->vector, q_vector);
	}
}

/**
 * rnpm_irq_disable - Mask off interrupt generation on the NIC
 * @adapter: board private structure
 **/
static inline void rnpm_irq_disable(struct rnpm_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_q_vectors; i++) {
		rnpm_irq_disable_queues(adapter->q_vector[i]);
		synchronize_irq(adapter->msix_entries[i].vector);
	}
}

int rnpm_xmit_nop_frame_ring(struct rnpm_adapter *adapter,
							 struct rnpm_ring *tx_ring)
{
	u16 i = tx_ring->next_to_use;
	struct rnpm_tx_desc *tx_desc;
	tx_desc = RNPM_TX_DESC(tx_ring, i);

	/* set length to 0 */
	tx_desc->blen_mac_ip_len = 0;
	tx_desc->vlan_cmd = cpu_to_le32(RNPM_TXD_CMD_EOP | RNPM_TXD_CMD_RS);
	/*
	i++;
	if (i == tx_ring->count) {
		i = 0;
	}
	*/
	/* update tail */
	rnpm_wr_reg(tx_ring->tail, 0);
	return 0;
}

int rnpm_xmit_nop_frame_ring_temp(struct rnpm_adapter *adapter,
								  struct rnpm_ring *tx_ring)
{
	u16 i = tx_ring->next_to_use;
	struct rnpm_tx_desc *tx_desc;
	tx_desc = RNPM_TX_DESC(tx_ring, i);

	/* set length to 0 */
	tx_desc->blen_mac_ip_len = 0;
	tx_desc->vlan_cmd = cpu_to_le32(RNPM_TXD_CMD_EOP | RNPM_TXD_CMD_RS);
	/*
	i++;
	if (i == tx_ring->count) {
		i = 0;
	}
	*/
	/* update tail */
	i++;
	tx_desc++;
	if (i == tx_ring->count) {
		i = 0;
	}
	tx_ring->next_to_use = i;
	wmb();
	rnpm_wr_reg(tx_ring->tail, i);
	/* no need clean */
	tx_ring->next_to_clean = i;

	return 0;
}

/**
 * rnpm_tx_maxrate_own - callback to set the maximum per-queue bitrate
 * @netdev: network interface device structure
 * @queue_index: Tx queue to set
 * @maxrate: desired maximum transmit bitrate Mbps
 **/
static int rnpm_tx_maxrate_own(struct rnpm_adapter *adapter, int queue_index)
{
	struct rnpm_ring *tx_ring = adapter->tx_ring[queue_index];
	u64 real_rate = 0;
	u32 maxrate = adapter->max_rate[queue_index];

	if (!maxrate)
		return rnpm_setup_tx_maxrate(adapter->hw.hw_addr,
									 tx_ring,
									 0,
									 adapter->hw.usecstocount * 1000000);
	/* we need turn it to bytes/s */
	real_rate = (maxrate * 1024 * 1024) / 8;
	rnpm_setup_tx_maxrate(adapter->hw.hw_addr,
						  tx_ring,
						  real_rate,
						  adapter->hw.usecstocount * 1000000);

	return 0;
}

/**
 * rnpm_configure_tx_ring - Configure 8259x Tx ring after Reset
 * @adapter: board private structure
 * @ring: structure containing ring specific data
 *
 * Configure the Tx descriptor ring after a reset.
 **/
void rnpm_configure_tx_ring(struct rnpm_adapter *adapter,
							struct rnpm_ring *ring)
{
	struct rnpm_hw *hw = &adapter->hw;
	// int i;
	// u64 desc_dma_phy = ring->dma;
	u8 queue_idx = ring->rnpm_queue_idx;

	/* disable queue to avoid issues while updating state */
	/*
	wr32(hw, RNPM_DMA_TX_START(queue_idx), 0);
	*/

	wr32(hw, RNPM_DMA_REG_TX_DESC_BUF_BASE_ADDR_LO(queue_idx), (u32)ring->dma);
	wr32(hw,
		 RNPM_DMA_REG_TX_DESC_BUF_BASE_ADDR_HI(queue_idx),
		 (u32)(((u64)ring->dma) >> 32) | (hw->pfvfnum << 24));
	wr32(hw, RNPM_DMA_REG_TX_DESC_BUF_LEN(queue_idx), ring->count);

	/* tail <= head */
	ring->next_to_clean = rd32(hw, RNPM_DMA_REG_TX_DESC_BUF_HEAD(queue_idx));
	ring->next_to_use = ring->next_to_clean;
	ring->tail = hw->hw_addr + RNPM_DMA_REG_TX_DESC_BUF_TAIL(queue_idx);
	rnpm_wr_reg(ring->tail, ring->next_to_use);

	//	wr32(hw, RNPM_DMA_REG_TX_DESC_FETCH_CTRL(queue_idx),
	//			(64 << 0)  /*max_water_flow*/
	//			| (TSRN10_TX_DEFAULT_BURST << 16)
	//			/*max-num_descs_peer_read*/
	//	    );
	wr32(hw,
		 RNPM_DMA_REG_TX_DESC_FETCH_CTRL(queue_idx),
		 (8 << 0) /*max_water_flow*/
			 | (TSRN10_TX_DEFAULT_BURST << 16)
		 /*max-num_descs_peer_read*/
	);
	wr32(hw,
		 RNPM_DMA_REG_TX_INT_DELAY_TIMER(queue_idx),
		 adapter->tx_usecs * hw->usecstocount);
	wr32(hw, RNPM_DMA_REG_TX_INT_DELAY_PKTCNT(queue_idx), adapter->tx_frames);

	rnpm_tx_maxrate_own(adapter, ring->queue_index);
	// flow control: bytes-peer-ctrl-tm-clk. 0:no-control
	/* reinitialize flowdirector state */
	if (adapter->flags & RNPM_FLAG_FDIR_HASH_CAPABLE) {
		ring->atr_sample_rate = adapter->atr_sample_rate;
		ring->atr_count = 0;
		set_bit(__RNPM_TX_FDIR_INIT_DONE, &ring->state);
	} else {
		ring->atr_sample_rate = 0;
	}
	/* initialize XPS */
	if (!test_and_set_bit(__RNPM_TX_XPS_INIT_DONE, &ring->state)) {
#ifdef HAVE_IRQ_AFFINITY_NOTIFY
		struct rnpm_q_vector *q_vector = ring->q_vector;

		if (q_vector)
			netif_set_xps_queue(
				adapter->netdev, &q_vector->affinity_mask, ring->queue_index);
#endif
	}

	clear_bit(__RNPM_HANG_CHECK_ARMED, &ring->state);

	// enable queue
	/*
	wmb();
	wr32(hw, RNPM_DMA_TX_START(queue_idx), 1);
	*/

	/* send some length 0 packet */
	/*
	for (i = 0; i < 200; i++)
		rnpm_xmit_nop_frame_ring_temp(adapter, ring);
	*/
}

static void rnpm_setup_mtqc(struct rnpm_adapter *adapter)
{
	// struct rnpm_hw *hw = &adapter->hw;
	// u32 rttdcs, mtqc;
	// u8 tcs = netdev_get_num_tc(adapter->netdev);

#if 0
	/* disable the arbiter while setting MTQC */
	rttdcs = RNPM_READ_REG(hw, RNPM_RTTDCS);
	rttdcs |= RNPM_RTTDCS_ARBDIS;
	RNPM_WRITE_REG(hw, RNPM_RTTDCS, rttdcs);

	/* set transmit pool layout */
	if (adapter->flags & RNPM_FLAG_SRIOV_ENABLED) {
		mtqc = RNPM_MTQC_VT_ENA;
		if (tcs > 4)
			mtqc |= RNPM_MTQC_RT_ENA | RNPM_MTQC_8TC_8TQ;
		else if (tcs > 1)
			mtqc |= RNPM_MTQC_RT_ENA | RNPM_MTQC_4TC_4TQ;
		else if (adapter->ring_feature[RING_F_RSS].indices == 4)
			mtqc |= RNPM_MTQC_32VF;
		else
			mtqc |= RNPM_MTQC_64VF;
	} else {
		if (tcs > 4)
			mtqc = RNPM_MTQC_RT_ENA | RNPM_MTQC_8TC_8TQ;
		else if (tcs > 1)
			mtqc = RNPM_MTQC_RT_ENA | RNPM_MTQC_4TC_4TQ;
		else
			mtqc = RNPM_MTQC_64Q_1PB;
	}

	RNPM_WRITE_REG(hw, RNPM_MTQC, mtqc);

	/* Enable Security TX Buffer IFG for multiple pb */
	if (tcs) {
		u32 sectx = RNPM_READ_REG(hw, RNPM_SECTXMINIFG);

		sectx |= RNPM_SECTX_DCB;
		RNPM_WRITE_REG(hw, RNPM_SECTXMINIFG, sectx);
	}

	/* re-enable the arbiter */
	rttdcs &= ~RNPM_RTTDCS_ARBDIS;
	RNPM_WRITE_REG(hw, RNPM_RTTDCS, rttdcs);
#endif
}

/**
 * rnpm_configure_tx - Configure Transmit Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx unit of the MAC after a reset.
 **/
static void rnpm_configure_tx(struct rnpm_adapter *adapter)
{
	u32 i, dma_axi_ctl;
	struct rnpm_hw *hw = &adapter->hw;

	rnpm_setup_mtqc(adapter);

	/* dma_axi_en.tx_en must be before Tx queues are enabled */
	dma_axi_ctl = rd32(hw, RNPM_DMA_AXI_EN);
	dma_axi_ctl |= TX_AXI_RW_EN;
	wr32(hw, RNPM_DMA_AXI_EN, dma_axi_ctl);

	/* Setup the HW Tx Head and Tail descriptor pointers */
	for (i = 0; i < (adapter->num_tx_queues); i++)
		rnpm_configure_tx_ring(adapter, adapter->tx_ring[i]);
}

/*
   static void rnpm_enable_rx_drop(struct rnpm_adapter *adapter,
   struct rnpm_ring *ring)
   {
   u32 dma_st;
   struct rnpm_hw *hw = &adapter->hw;

   dma_st = rd32(hw, RNPM_DMA_STATUS);
   dma_st |= RNPM_DMA_RX_TIMEOUT_DROP_EN;
   wr32(hw, RNPM_DMA_STATUS, dma_st);
   }

   static void rnpm_disable_rx_drop(struct rnpm_adapter *adapter,
   struct rnpm_ring *ring)
   {
   u32 dma_st;
   struct rnpm_hw *hw = &adapter->hw;

   dma_st = rd32(hw, RNPM_DMA_STATUS);
   dma_st &= ~RNPM_DMA_RX_TIMEOUT_DROP_EN;
   wr32(hw, RNPM_DMA_STATUS, dma_st);
   }
   */

__maybe_unused static void
rnpm_rx_desc_queue_enable(struct rnpm_adapter *adapter, struct rnpm_ring *ring)
{
	// struct rnpm_hw *hw = &adapter->hw;
	// u16 q_idx = ring->rnpm_queue_idx;
}

void rnpm_disable_rx_queue(struct rnpm_adapter *adapter, struct rnpm_ring *ring)
{
	struct rnpm_hw *hw = &adapter->hw;

	wr32(hw, RNPM_DMA_RX_START(ring->rnpm_queue_idx), 0);
}

void rnpm_configure_rx_ring(struct rnpm_adapter *adapter,
							struct rnpm_ring *ring)
{
	struct rnpm_hw *hw = &adapter->hw;
	u64 desc_phy = ring->dma;
	u16 q_idx = ring->rnpm_queue_idx;

	/* disable queue to avoid issues while updating state */
	rnpm_disable_rx_queue(adapter, ring);

	/* set descripts registers*/
	wr32(hw, RNPM_DMA_REG_RX_DESC_BUF_BASE_ADDR_LO(q_idx), (u32)desc_phy);
	wr32(hw,
		 RNPM_DMA_REG_RX_DESC_BUF_BASE_ADDR_HI(q_idx),
		 ((u32)(desc_phy >> 32)) | (hw->pfvfnum << 24));
	wr32(hw, RNPM_DMA_REG_RX_DESC_BUF_LEN(q_idx), ring->count);

	ring->tail = hw->hw_addr + RNPM_DMA_REG_RX_DESC_BUF_TAIL(q_idx);
	ring->next_to_clean = rd32(hw, RNPM_DMA_REG_RX_DESC_BUF_HEAD(q_idx));
	ring->next_to_use = ring->next_to_clean;

	wr32(hw,
		 RNPM_DMA_REG_RX_DESC_FETCH_CTRL(q_idx),
		 0 | (TSRN10_RX_DEFAULT_LINE << 0) /*rx-desc-flow*/
			 | (TSRN10_RX_DEFAULT_BURST << 16)
		 /*max-read-desc-cnt*/
	);
	wr32(hw,
		 RNPM_DMA_REG_RX_INT_DELAY_TIMER(q_idx),
		 adapter->rx_usecs * hw->usecstocount);
	wr32(hw, RNPM_DMA_REG_RX_INT_DELAY_PKTCNT(q_idx), adapter->rx_frames);
	rnpm_alloc_rx_buffers(ring, rnpm_desc_unused_rx(ring));
	/* enable receive descriptor ring */
	// wr32(hw, RNPM_DMA_RX_START(q_idx), 1);
}

static void rnpm_configure_virtualization(struct rnpm_adapter *adapter)
{
	struct rnpm_hw *hw = &adapter->hw;
	//u8 *mac;
	//u32 maclow, machi;
	u32 ring, vfnum = 0;
	//u8 port = adapter->port;

	if (!(adapter->flags & RNPM_FLAG_SRIOV_ENABLED))
		return;

	/* Enable only the PF's pool for Tx/Rx */

	if (adapter->flags2 & RNPM_FLAG2_BRIDGE_MODE_VEB) {
		wr32(
			hw, RNPM_DMA_CONFIG, rd32(hw, RNPM_DMA_CONFIG) & (~DMA_VEB_BYPASS));
		adapter->flags2 |= RNPM_FLAG2_BRIDGE_MODE_VEB;
	}
#if 1
	ring = adapter->tx_ring[0]->rnpm_queue_idx;
	// enable find vf by dest-mac-address
	wr32(hw, RNPM_HOST_FILTER_EN, 1);
	wr32(hw, RNPM_REDIR_EN, 1);
	wr32(hw, RNPM_MRQC_IOV_EN, RNPM_IOV_ENABLED);
	wr32(hw,
		 RNPM_ETH_DMAC_FCTRL,
		 rd32(hw, RNPM_ETH_DMAC_FCTRL) | RNPM_FCTRL_BROADCASE_BYPASS);
	// wr32(hw, RNPM_ETH_DMAC_MCSTCTRL, RNPM_MCSTCTRL_DMAC_47);
	/* Map PF MAC address in RAR Entry 0 to first pool following VFs */
	hw->mac.ops.set_vmdq(hw, 0, ring / 2);
#endif

	// set VEB table, so pf can receive broadcast. and can ping vf
//	if (is_valid_ether_addr(adapter->netdev->dev_addr))
//		mac = adapter->netdev->dev_addr;
//	else
//		mac = hw->mac.perm_addr;
//	dbg("%s %x:%x:%x:%x:%x:%x\n",
//		__func__,
//		mac[0],
//		mac[1],
//		mac[2],
//		mac[3],
//		mac[4],
//		mac[5]);
//	port = 0;
//	if (rd32(hw, RNPM_DMA_VERSION) >= 0x20201231) {
//		for (port = 0; port < 4; port++) {
//			// 4 veb table to same value in 2port mode
//			vfnum = RNPM_MAX_VF_CNT - 1;
//			// use last-vf's table entry. the last
//			maclow = (mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5];
//			machi = (mac[0] << 8) | mac[1];
//			wr32(hw, RNPM_DMA_PORT_VBE_MAC_LO_TBL(port, vfnum), maclow);
//			wr32(hw, RNPM_DMA_PORT_VBE_MAC_HI_TBL(port, vfnum), machi);
//			ring |= ((0x80 | vfnum) << 8);
//			wr32(hw, RNPM_DMA_PORT_VEB_VF_RING_TBL(port, vfnum), ring);
//		}
//	} else {
//		vfnum = RNPM_MAX_VF_CNT - 1;
//		maclow = (mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) | mac[5];
//		machi = (mac[0] << 8) | mac[1];
//		wr32(hw, RNPM_DMA_PORT_VBE_MAC_LO_TBL(port, vfnum), maclow);
//		wr32(hw, RNPM_DMA_PORT_VBE_MAC_HI_TBL(port, vfnum), machi);
//		ring |= ((0x80 | vfnum) << 8);
//		wr32(hw, RNPM_DMA_PORT_VEB_VF_RING_TBL(port, vfnum), ring);
//	}
	/* store vfnum */
	adapter->vf_num_for_pf = 0x80 | vfnum;
}

static void rnpm_set_rx_buffer_len(struct rnpm_adapter *adapter)
{
	// struct rnpm_hw *hw = &adapter->hw;
	struct net_device *netdev = adapter->netdev;
	int max_frame = netdev->mtu + ETH_HLEN + ETH_FCS_LEN;
	struct rnpm_ring *rx_ring;
	int i;
	// u32 mhadd, hlreg0;
	// int max_frame = netdev->mtu + ETH_HLEN + ETH_FCS_LEN;

	if (max_frame < (ETH_FRAME_LEN + ETH_FCS_LEN))
		max_frame = (ETH_FRAME_LEN + ETH_FCS_LEN);

	for (i = 0; i < adapter->num_rx_queues; i++) {
		rx_ring = adapter->rx_ring[i];
#ifndef CONFIG_RNPM_DISABLE_PACKET_SPLIT
		clear_bit(__RNPM_RX_3K_BUFFER, &rx_ring->state);
		clear_bit(__RNPM_RX_BUILD_SKB_ENABLED, &rx_ring->state);
#ifdef HAVE_SWIOTLB_SKIP_CPU_SYNC

		set_bit(__RNPM_RX_BUILD_SKB_ENABLED, &rx_ring->state);
		hw_dbg(&adapter->hw, "set build skb\n");

#if (PAGE_SIZE < 8192)
		if (RNPM_2K_TOO_SMALL_WITH_PADDING ||
			(max_frame > (ETH_FRAME_LEN + ETH_FCS_LEN)))
			;
			//	set_bit(__RNPM_RX_3K_BUFFER, &rx_ring->state);
#endif

#else /* !HAVE_SWIOTLB_SKIP_CPU_SYNC */
		/* FIXME */
		hw_dbg(&adapter->hw, "set construct skb\n");

#endif /* HAVE_SWIOTLB_SKIP_CPU_SYNC */
#else
		rx_ring->rx_buf_len = RNPM_RXBUFFER_2K;
#endif /* CONFIG_RNPM_DISABLE_PACKET_SPLIT */
	}
}

/**
 * rnpm_configure_rx - Configure 8259x Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Rx unit of the MAC after a reset.
 **/
static void rnpm_configure_rx(struct rnpm_adapter *adapter)
{
	struct rnpm_hw *hw = &adapter->hw;
	int i;
	u32 rxctrl = 0, dma_axi_ctl;

	/* disable receives while setting up the descriptors */
#if 0
	rnpm_setup_psrtype(adapter);
	rnpm_setup_rdrxctl(adapter);

	/* Program registers for the distribution of queues */
	rnpm_setup_mrqc(adapter);
#endif

	/* set_rx_buffer_len must be called before ring initialization */
	rnpm_set_rx_buffer_len(adapter);

	/* set_rx_buffer_len must be called before ring initialization */
	// rnpm_set_rx_buffer_len(adapter);

	/*
	 * Setup the HW Rx Head and Tail Descriptor Pointers and
	 * the Base and Length of the Rx Descriptor Ring
	 */
	for (i = 0; i < adapter->num_rx_queues; i++)
		rnpm_configure_rx_ring(adapter, adapter->rx_ring[i]);

	if (adapter->pf_adapter->default_rx_ring > 0) {
		wr32(
			hw, RNPM_ETH_DEFAULT_RX_RING, adapter->pf_adapter->default_rx_ring);
	}

	/* enable all receives */
	rxctrl |= 0;

	dma_axi_ctl = rd32(hw, RNPM_DMA_AXI_EN);
	dma_axi_ctl |= RX_AXI_RW_EN;
	wr32(hw, RNPM_DMA_AXI_EN, dma_axi_ctl);

	hw->mac.ops.enable_rx_dma(hw, rxctrl);
}

#if defined(NETIF_F_HW_VLAN_TX) || defined(NETIF_F_HW_VLAN_CTAG_TX)

#ifdef HAVE_INT_NDO_VLAN_RX_ADD_VID
#ifdef NETIF_F_HW_VLAN_CTAG_TX
static int rnpm_vlan_rx_add_vid(struct net_device *netdev,
								__always_unused __be16 proto,
								u16 vid)
#else  /* !NETIF_F_HW_VLAN_CTAG_TX */
static int rnpm_vlan_rx_add_vid(struct net_device *netdev, u16 vid)
#endif /* NETIF_F_HW_VLAN_CTAG_TX */
#else  /* !HAVE_INT_NDO_VLAN_RX_ADD_VID */
static void rnpm_vlan_rx_add_vid(struct net_device *netdev, u16 vid)
#endif /* HAVE_INT_NDO_VLAN_RX_ADD_VID */
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	struct rnpm_pf_adapter *pf_adapter = adapter->pf_adapter;
	struct rnpm_hw *hw = &adapter->hw;
	int port = 0;
	unsigned long flags;

	if (hw->mac.vlan_location == rnpm_vlan_location_nic) {
		if (hw->mac.ops.set_vfta) {
#ifndef HAVE_VLAN_RX_REGISTER
			if (vid < VLAN_N_VID) {
				set_bit(vid, adapter->active_vlans);
				spin_lock_irqsave(&pf_adapter->vlan_setup_lock, flags);
				set_bit(vid, pf_adapter->active_vlans);
				spin_unlock_irqrestore(&pf_adapter->vlan_setup_lock, flags);
			}
#endif
			/* add VID to filter table */
			spin_lock_irqsave(&pf_adapter->vlan_setup_lock, flags);
			hw->mac.ops.set_vfta(&adapter->hw, vid, VMDQ_P(0), true);
			spin_unlock_irqrestore(&pf_adapter->vlan_setup_lock, flags);
		}
	} else {
		if (hw->mac.ops.set_vfta_mac) {
#ifndef HAVE_VLAN_RX_REGISTER
			if (vid < VLAN_N_VID) {
				set_bit(vid, adapter->active_vlans);
			}
#endif
			hw->mac.ops.set_vfta_mac(&adapter->hw, vid, VMDQ_P(0), true);
		}
	}

	/* todo */
	if (adapter->flags & RNPM_FLAG_SRIOV_ENABLED) {
		u8 vfnum = RNPM_MAX_VF_CNT - 1;
		if (rd32(hw, RNPM_DMA_VERSION) >= 0x20201231) {
			for (port = 0; port < 4; port++)
				wr32(hw, RNPM_DMA_PORT_VEB_VID_TBL(port, vfnum), vid);
		} else {
			wr32(hw, RNPM_DMA_PORT_VEB_VID_TBL(adapter->port, vfnum), vid);
		}
	}
#ifndef HAVE_NETDEV_VLAN_FEATURES
	/*
	 * Copy feature flags from netdev to the vlan netdev for this vid.
	 * This allows things like TSO to bubble down to our vlan device.
	 * Some vlans, such as VLAN 0 for DCB will not have a v_netdev so
	 * we will not have a netdev that needs updating.
	 */
	if (adapter->vlgrp) {
		struct vlan_group *vlgrp = adapter->vlgrp;
		struct net_device *v_netdev = vlan_group_get_device(vlgrp, vid);
		if (v_netdev) {
			v_netdev->features |= netdev->features;
			vlan_group_set_device(vlgrp, vid, v_netdev);
		}
	}
#endif /* HAVE_NETDEV_VLAN_FEATURES */
#ifdef HAVE_INT_NDO_VLAN_RX_ADD_VID
	return 0;
#endif
}

#ifdef HAVE_INT_NDO_VLAN_RX_ADD_VID
#ifdef NETIF_F_HW_VLAN_CTAG_RX
static int rnpm_vlan_rx_kill_vid(struct net_device *netdev,
								 __always_unused __be16 proto,
								 u16 vid)
#else  /* !NETIF_F_HW_VLAN_CTAG_RX */
static int rnpm_vlan_rx_kill_vid(struct net_device *netdev, u16 vid)
#endif /* NETIF_F_HW_VLAN_CTAG_RX */
#else
static void rnpm_vlan_rx_kill_vid(struct net_device *netdev, u16 vid)
#endif
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	struct rnpm_pf_adapter __maybe_unused *pf_adapter = adapter->pf_adapter;
	struct rnpm_hw *hw = &adapter->hw;
	unsigned long flags;
	// int port = adapter->port;

	if (!vid)
#ifdef HAVE_INT_NDO_VLAN_RX_ADD_VID
		return 0;
#else
		return;
#endif

#ifdef HAVE_VLAN_RX_REGISTER
	if (!test_bit(__RNPM_DOWN, &adapter->state))
		rnpm_irq_disable(adapter);

	// vlan_group_set_device(pf_adapter->vlgrp[port], vid, NULL);
	vlan_group_set_device(adapter->vlgrp, vid, NULL);

	if (!test_bit(__RNPM_DOWN, &adapter->state))
		rnpm_irq_enable(adapter);

#endif /* HAVE_VLAN_RX_REGISTER */

	if (hw->mac.ops.set_vfta) {
		/* remove VID from filter table only in no mutiport mode */
		if (!(adapter->flags & RNPM_FLAG_MUTIPORT_ENABLED))
			hw->mac.ops.set_vfta(&adapter->hw, vid, VMDQ_P(0), false);
		// clear_bit(vid, adapter->active_vlans);

		// todo
		//	if ((adapter->flags & RNPM_FLAG_SRIOV_ENABLED)) {
		//		u8 vfnum = RNPM_MAX_VF_CNT - 1;
		//		for_each_set_bit(vid, adapter->active_vlans, VLAN_N_VID) {
		//			if (rd32(hw, RNPM_DMA_VERSION) >= 0x20201231) {
		//				int port;
		//				for (port = 0; port < 4; port++)
		//					wr32(hw, RNPM_DMA_PORT_VEB_VID_TBL(port, vfnum),
		//vid); 			} else { 				wr32(hw, 					 RNPM_DMA_PORT_VEB_VID_TBL(adapter->port,
		//vfnum), 					 vid);
		//			}
		//		}
		//	}
	}
#ifndef HAVE_VLAN_RX_REGISTER
	clear_bit(vid, adapter->active_vlans);

	if (adapter->flags & RNPM_FLAG_MUTIPORT_ENABLED) {
		if (hw->mac.vlan_location == rnpm_vlan_location_nic) {
			/* mutiport mode , only set update*/
			adapter->flags_feature |= RNPM_FLAG_DELAY_UPDATE_VLAN_TABLE;
		} else {
			int i;
			/* if use mac vlan table */
			/* clear hash table */
			wr32(&adapter->hw, RNPM_MAC_VLAN_HASH_TB(adapter->port), 0);
			/* update vlan hash table in mac */
			for_each_set_bit(i, adapter->active_vlans, VLAN_N_VID)
			{
				if (hw->mac.ops.set_vfta_mac) {
					hw->mac.ops.set_vfta_mac(&adapter->hw, i, VMDQ_P(0), true);
				}
			}
		}

	} else {
		spin_lock_irqsave(&pf_adapter->vlan_setup_lock, flags);
		clear_bit(vid, pf_adapter->active_vlans);
		spin_unlock_irqrestore(&pf_adapter->vlan_setup_lock, flags);
	}

#endif
#ifdef HAVE_INT_NDO_VLAN_RX_ADD_VID
	return 0;
#endif
}

#endif

static u32 rnpm_vlan_filter_status_update(struct rnpm_pf_adapter *pf_adapter)
{
	int i;
	u32 status = 1;
	unsigned long flags;

	for (i = 0; i < pf_adapter->adapter_cnt; i++) {
		if (rnpm_port_is_valid(pf_adapter, i)) {
			status &= pf_adapter->vlan_filter_status[i];
		}
	}
	spin_lock_irqsave(&pf_adapter->vlan_filter_lock, flags);
	pf_adapter->vlan_status_true = status;
	spin_unlock_irqrestore(&pf_adapter->vlan_filter_lock, flags);
	return status;
}

/**
 * rnpm_vlan_filter_disable - helper to disable hw vlan filtering
 * @adapter: driver data
 */
static void __maybe_unused
rnpm_vlan_filter_disable(struct rnpm_adapter *adapter)
{
	struct rnpm_hw *hw = &adapter->hw;
	struct rnpm_pf_adapter *pf_adapter = adapter->pf_adapter;
	u8 port = adapter->port;
	pf_adapter->vlan_filter_status[port] = 0;
	if (hw->mac.vlan_location == rnpm_vlan_location_nic) {
		adapter->flags_feature |= RNPM_FLAG_DELAY_UPDATE_VLAN_FILTER;
		/* off vlan filter if any port vlan filter off*/
		if (!rnpm_vlan_filter_status_update(pf_adapter))
			rnpm_vlan_filter_off(hw);
	} else {
		/* mac vlan filter is used */
		u32 value;

		value = rd32(hw, RNPM_MAC_PKT_FLT(port));
		value &= (~RNPM_VLAN_HASH_EN);
		wr32(hw, RNPM_MAC_PKT_FLT(port), value);
		rnpm_vlan_filter_off(hw);
	}
}

/**
 * rnpm_vlan_filter_enable - helper to enable hw vlan filtering
 * @adapter: driver data
 */
static void __maybe_unused rnpm_vlan_filter_enable(struct rnpm_adapter *adapter)
{
	struct rnpm_hw *hw = &adapter->hw;
	struct rnpm_pf_adapter *pf_adapter = adapter->pf_adapter;
	u8 port = adapter->port;
	pf_adapter->vlan_filter_status[port] = 1;
	/* open vlan filter if all port vlan filter on*/
	if (hw->mac.vlan_location == rnpm_vlan_location_nic) {
		adapter->flags_feature |= RNPM_FLAG_DELAY_UPDATE_VLAN_FILTER;
		if (rnpm_vlan_filter_status_update(pf_adapter))
			rnpm_vlan_filter_on(hw);
	} else {
		/* mac vlan filter is used */
		u32 value;

		value = rd32(hw, RNPM_MAC_PKT_FLT(port));
		value |= RNPM_VLAN_HASH_EN;
		wr32(hw, RNPM_MAC_PKT_FLT(port), value);

		rnpm_vlan_filter_off(hw);

		// should set vlan tags registers?
	}
}

/**
 * rnpm_vlan_strip_disable - helper to disable hw vlan stripping
 * @adapter: driver data
 */
static void rnpm_vlan_strip_disable(struct rnpm_adapter *adapter)
{
	int i;
	struct rnpm_ring *tx_ring;
	struct rnpm_hw *hw = &adapter->hw;

	for (i = 0; i < adapter->num_rx_queues; i++) {
		tx_ring = adapter->rx_ring[i];
		hw_queue_strip_rx_vlan(hw, tx_ring->rnpm_queue_idx, false);
	}
}

/**
 * rnpm_vlan_strip_enable - helper to enable hw vlan stripping
 * @adapter: driver data
 */
static void rnpm_vlan_strip_enable(struct rnpm_adapter *adapter)
{
	struct rnpm_hw *hw = &adapter->hw;
	struct rnpm_ring *tx_ring;
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++) {
		tx_ring = adapter->rx_ring[i];
		hw_queue_strip_rx_vlan(hw, tx_ring->rnpm_queue_idx, true);
	}
}

#ifdef HAVE_VLAN_RX_REGISTER
static void rnpm_vlan_mode(struct net_device *netdev, struct vlan_group *grp)
#else
void rnpm_vlan_mode(struct net_device *netdev, u32 features)
#endif
{
#if defined(HAVE_VLAN_RX_REGISTER) || defined(HAVE_8021P_SUPPORT)
	struct rnpm_adapter *adapter = netdev_priv(netdev);
#endif
#ifdef HAVE_8021P_SUPPORT
	bool enable;
#endif

#ifdef HAVE_VLAN_RX_REGISTER
	if (!test_bit(__RNPM_DOWN, &adapter->state))
		rnpm_irq_disable(adapter);

	adapter->vlgrp = grp;
	adapter->pf_adapter->vlgrp[adapter->port] = grp;

	if (!test_bit(__RNPM_DOWN, &adapter->state))
		rnpm_irq_enable(adapter);
#endif
#ifdef HAVE_8021P_SUPPORT
#ifdef HAVE_VLAN_RX_REGISTER
	enable = (grp || (adapter->flags & RNPM_FLAG_DCB_ENABLED));
#else
#ifdef NETIF_F_HW_VLAN_CTAG_RX
	enable = !!(features & NETIF_F_HW_VLAN_CTAG_RX);
#else
	enable = !!(features & NETIF_F_HW_VLAN_RX);
#endif /* NETIF_F_HW_VLAN_CTAG_RX */
#endif /* HAVE_VLAN_RX_REGISTER */
	if (enable)
		/* enable VLAN tag insert/strip */
		rnpm_vlan_strip_enable(adapter);
	else
		/* disable VLAN tag insert/strip */
		rnpm_vlan_strip_disable(adapter);

#endif /* HAVE_8021P_SUPPORT */
}

static void rnpm_restore_vlan(struct rnpm_adapter *adapter)
{
	u16 vid;
	struct rnpm_hw *hw = &adapter->hw;

#ifdef HAVE_VLAN_RX_REGISTER

	rnpm_vlan_mode(adapter->netdev, adapter->vlgrp);

	/*
	 * add vlan ID 0 and enable vlan tag stripping so we
	 * always accept priority-tagged traffic
	 */
#ifdef NETIF_F_HW_VLAN_CTAG_RX
	rnpm_vlan_rx_add_vid(adapter->netdev, htons(ETH_P_8021Q), 0);
#else
	rnpm_vlan_rx_add_vid(adapter->netdev, 0);
#endif
	if (adapter->vlgrp) {
		for (; vid < VLAN_N_VID; vid++) {
			if (!vlan_group_get_device(adapter->vlgrp, vid))
				continue;
#ifdef NETIF_F_HW_VLAN_CTAG_RX
			rnpm_vlan_rx_add_vid(adapter->netdev, htons(ETH_P_8021Q), vid);
#else
			rnpm_vlan_rx_add_vid(adapter->netdev, vid);
#endif
		}
	}
#else /* !HAVE_VLAN_RX_REGISTER */

	rnpm_vlan_rx_add_vid(adapter->netdev, htons(ETH_P_8021Q), 0);

	for_each_set_bit(vid, adapter->active_vlans, VLAN_N_VID)
		rnpm_vlan_rx_add_vid(adapter->netdev, htons(ETH_P_8021Q), vid);

#endif
	/* config vlan mode for mac */
	wr32(hw, RNPM_MAC_TX_VLAN_MODE(adapter->port), 0x00100000);
}

/**
 * rnpm_write_uc_addr_list - write unicast addresses to RAR table
 * @netdev: network interface device structure
 *
 * Writes unicast address list to the RAR table.
 * Returns: -ENOMEM on failure/insufficient address space
 *                0 on no addresses written
 *                X on writing X addresses to the RAR table
 **/
static int rnpm_write_uc_addr_list(struct net_device *netdev)
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	struct rnpm_hw *hw = &adapter->hw;
	// unsigned int rar_entries = hw->mac.num_rar_entries - 1;
	unsigned int rar_entries = adapter->uc_num - 1;
	int count = 0;

	/* In SR-IOV mode significantly less RAR entries are available */
	if (adapter->flags & RNPM_FLAG_SRIOV_ENABLED)
		rar_entries = RNPM_MAX_PF_MACVLANS - 1;

	/* return ENOMEM indicating insufficient memory for addresses */
	if (netdev_uc_count(netdev) > rar_entries)
		return -ENOMEM;

	/* add offset */
	rar_entries += adapter->uc_off;
	if (!netdev_uc_empty(netdev)) {
		struct netdev_hw_addr *ha;

		hw_dbg(hw,
			   "%s: rar_entries:%d, uc_count:%d offset %d\n",
			   __func__,
			   rar_entries,
			   adapter->uc_off,
			   netdev_uc_count(netdev));

		/* return error if we do not support writing to RAR table */
		if (!hw->mac.ops.set_rar)
			return -ENOMEM;
		/* setup mac unicast filters */
		if (hw->mac.mc_location == rnpm_mc_location_mac) {
			/* if use mac multicast */
			if (!hw->mac.ops.set_rar_mac)
				return -ENOMEM;
		}

		netdev_for_each_uc_addr(ha, netdev)
		{
			if (!rar_entries)
				break;
			/* VMDQ_P(0) is num_vfs pf use the last vf in sriov mode  */
			/* that's ok */
			hw->mac.ops.set_rar(
				hw, rar_entries, ha->addr, VMDQ_P(0), RNPM_RAH_AV);

			/* if use mac filter we should also set Unicast to mac */
			if (hw->mac.mc_location == rnpm_mc_location_mac) {
				hw->mac.ops.set_rar_mac(hw,
										rar_entries - adapter->uc_off,
										ha->addr,
										VMDQ_P(0),
										adapter->port);
			}
			rar_entries--;
			count++;
		}
	}
	/* write the addresses in reverse order to avoid write combining */

	hw_dbg(hw,
		   "%s: Clearing RAR[%d - %d]\n",
		   __func__,
		   adapter->uc_off + 1,
		   rar_entries);
	for (; rar_entries > adapter->uc_off; rar_entries--) {
		hw->mac.ops.clear_rar(hw, rar_entries);
		if (hw->mac.mc_location == rnpm_mc_location_mac) {
			hw->mac.ops.clear_rar_mac(
				hw, rar_entries - adapter->uc_off, adapter->port);
		}
	}

	return count;
}

static void rnpm_setup_fctrl(struct rnpm_hw *hw)
{
	struct rnpm_adapter *adapter = (struct rnpm_adapter *)hw->back;
	struct rnpm_pf_adapter *pf_adapter = adapter->pf_adapter;
	int i;
	u32 fctrl = 0;

	for (i = 0; i < pf_adapter->adapter_cnt; i++) {
		if (rnpm_port_is_valid(pf_adapter, i)) {
			fctrl |= pf_adapter->fctrl[i];
		}
	}
	wr32(hw, RNPM_ETH_DMAC_FCTRL, fctrl);
}

/**
 * rnpm_set_rx_mode - Unicast, Multicast and Promiscuous mode set
 * @netdev: network interface device structure
 *
 * The set_rx_method entry point is called whenever the unicast/multicast
 * address list or the network interface flags are updated.  This routine is
 * responsible for configuring the hardware for proper unicast, multicast and
 * promiscuous mode.
 **/
void rnpm_set_rx_mode(struct net_device *netdev)
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	struct rnpm_pf_adapter *pf_adapter = adapter->pf_adapter;
	struct rnpm_hw *hw = &adapter->hw;
	u32 fctrl;
	u32 fctrl_mac = 0;
	netdev_features_t __maybe_unused features = netdev->features;
	int count;
	u8 port = adapter->port;

	hw_dbg(hw, "%s\n", __func__);

	fctrl = pf_adapter->fctrl[port];

	// mcstctrl = rd32(hw, RNPM_ETH_DMAC_MCSTCTRL);

	/* clear the bits we are changing the status of */
	fctrl &= ~(RNPM_FCTRL_UPE | RNPM_FCTRL_MPE);

	/* promisc mode */
	if (netdev->flags & IFF_PROMISC) {
		hw->addr_ctrl.user_set_promisc = true;
		fctrl |= (RNPM_FCTRL_UNICASE_BYPASS | RNPM_FCTRL_MULTICASE_BYPASS |
				  RNPM_FCTRL_BROADCASE_BYPASS);
		// mcstctrl &= ~(RNPM_MCSTCTRL_UNICASE_TBL_EN |
		// RNPM_MCSTCTRL_MULTICASE_TBL_EN);
		fctrl_mac |= RNPM_RX_ALL;
		/* disable hardware filter vlans in promisc mode */
#ifdef NETIF_F_HW_VLAN_CTAG_FILTER
		features &= ~NETIF_F_HW_VLAN_CTAG_FILTER;
#endif
		/* must disable vlan filter in promisc mode */
		// rnpm_vlan_filter_disable(adapter);
		/* close vlan offload */
#ifdef NETIF_F_HW_VLAN_CTAG_RX
		features &= ~NETIF_F_HW_VLAN_CTAG_RX;
#endif
	} else {
		if (netdev->flags & IFF_ALLMULTI) {
			fctrl |= RNPM_FCTRL_MULTICASE_BYPASS;
			fctrl_mac |= RNPM_RX_ALL_MUL;
			// mcstctrl &= ~(RNPM_MCSTCTRL_MULTICASE_TBL_EN);
		} else {
			/* Write addresses to the MTA, if the attempt fails
			 * then we should just turn on promiscuous mode so
			 * that we can at least receive multicast traffic
			 */
			count = hw->mac.ops.update_mc_addr_list(hw, netdev);
			if (count < 0) {
				printk("open mpe\n");
				fctrl |= RNPM_FCTRL_MPE;
				fctrl_mac |= RNPM_RX_ALL_MUL;
				// mcstctrl &= ~RNPM_MCSTCTRL_MULTICASE_TBL_EN;
			} else if (count) {
				// mcstctrl |= RNPM_MCSTCTRL_MULTICASE_TBL_EN;
			}
		}
		hw->addr_ctrl.user_set_promisc = false;
	}

	// test mode
	// fctrl_mac |= RNPM_RX_ALL;
	/*
	 * Write addresses to available RAR registers, if there is not
	 * sufficient space to store all the addresses then enable
	 * unicast promiscuous mode
	 */
	if (rnpm_write_uc_addr_list(netdev) < 0) {
		fctrl |= RNPM_FCTRL_UPE;
		// mcstctrl &= ~RNPM_MCSTCTRL_UNICASE_TBL_EN;
	}

	if (adapter->num_vfs)
		rnpm_restore_vf_multicasts(adapter);

	// force disable Multicast filter why?
	// fctrl |= RNPM_FCTRL_MULTICASE_BYPASS;
	// update multicase & unicast regs
	if (hw->mac.mc_location == rnpm_mc_location_mac) {
		u32 value;

		value = rd32(hw, RNPM_MAC_PKT_FLT(port));
		if(!(adapter->flags & RNPM_FLAG_SWITCH_LOOPBACK_EN)){ // switch-loopback mode mac should rece all  pkgs
			value &= ~(RNPM_RX_ALL | RNPM_RX_ALL_MUL);
		}
		value |= fctrl_mac;
		wr32(hw, RNPM_MAC_PKT_FLT(port), value);
		/* in this mode should always close nic mc uc */
		fctrl |= RNPM_FCTRL_MULTICASE_BYPASS;
		fctrl |= RNPM_FCTRL_UNICASE_BYPASS;
		wr32(hw, RNPM_ETH_DMAC_FCTRL, fctrl);
	} else {
		pf_adapter->fctrl[port] = fctrl;

		rnpm_setup_fctrl(hw);
	}
	/*
	wr32(hw, RNPM_ETH_DMAC_FCTRL, fctrl);
	*/
	// wr32(hw, RNPM_ETH_DMAC_MCSTCTRL, mcstctrl);

	if (adapter->netdev->features & NETIF_F_RXALL) {
	}
#ifdef NETIF_F_HW_VLAN_CTAG_FILTER
	if (features & NETIF_F_HW_VLAN_CTAG_FILTER)
		rnpm_vlan_filter_enable(adapter);
	else
		rnpm_vlan_filter_disable(adapter);
#endif

#ifdef NETIF_F_HW_VLAN_CTAG_RX
	if (features & NETIF_F_HW_VLAN_CTAG_RX)
		rnpm_vlan_strip_enable(adapter);
	else
		rnpm_vlan_strip_disable(adapter);
#endif
	/* features not write back ?*/
	/* no need this */
}

static void rnpm_napi_enable_all(struct rnpm_adapter *adapter)
{
	int q_idx;

	for (q_idx = 0; q_idx < adapter->num_q_vectors; q_idx++)
		napi_enable(&adapter->q_vector[q_idx]->napi);
}

static void rnpm_napi_disable_all(struct rnpm_adapter *adapter)
{
	int q_idx;

	for (q_idx = 0; q_idx < adapter->num_q_vectors; q_idx++) {
		/* stop timer avoid error */
		rnpm_htimer_stop(adapter->q_vector[q_idx]);

		napi_disable(&adapter->q_vector[q_idx]->napi);
	}
}

#ifdef CONFIG_RNPM_DCB
/**
 * rnpm_configure_dcb - Configure DCB hardware
 * @adapter: rnpm adapter struct
 *
 * This is called by the driver on open to configure the DCB hardware.
 * This is also called by the gennetlink interface when reconfiguring
 * the DCB state.
 */
static void rnpm_configure_dcb(struct rnpm_adapter *adapter)
{
	struct rnpm_hw *hw = &adapter->hw;
	int max_frame = adapter->netdev->mtu + ETH_HLEN + ETH_FCS_LEN;

#if 0
	if (!(adapter->flags & RNPM_FLAG_DCB_ENABLED)) {
		if (hw->mac.type == rnpm_mac_82598EB)
			netif_set_gso_max_size(adapter->netdev, 65536);
		return;
	}

	if (hw->mac.type == rnpm_mac_82598EB)
		netif_set_gso_max_size(adapter->netdev, 32768);

	/* reconfigure the hardware */
	if (adapter->dcbx_cap & DCB_CAP_DCBX_VER_CEE) {
		rnpm_dcb_calculate_tc_credits(hw, &adapter->dcb_cfg, max_frame,
						DCB_TX_CONFIG);
		rnpm_dcb_calculate_tc_credits(hw, &adapter->dcb_cfg, max_frame,
						DCB_RX_CONFIG);
		rnpm_dcb_hw_config(hw, &adapter->dcb_cfg);
	} else if (adapter->rnpm_ieee_ets && adapter->rnpm_ieee_pfc) {
		rnpm_dcb_hw_ets(&adapter->hw,
				 adapter->rnpm_ieee_ets,
				 max_frame);
		rnpm_dcb_hw_pfc_config(&adapter->hw,
					adapter->rnpm_ieee_pfc->pfc_en,
					adapter->rnpm_ieee_ets->prio_tc);
	}

	/* Enable RSS Hash per TC */
	if (hw->mac.type != rnpm_mac_82598EB) {
		u32 msb = 0;
		u16 rss_i = adapter->ring_feature[RING_F_RSS].indices - 1;

		while (rss_i) {
			msb++;
			rss_i >>= 1;
		}

		/* write msb to all 8 TCs in one write */
		RNPM_WRITE_REG(hw, RNPM_RQTC, msb * 0x11111111);
	}
#endif
}
#endif

/* Additional bittime to account for RNPM framing */
#define RNPM_ETH_FRAMING 20

/**
 * rnpm_hpbthresh - calculate high water mark for flow control
 *
 * @adapter: board private structure to calculate for
 * @pb: packet buffer to calculate
 */
__maybe_unused static int rnpm_hpbthresh(struct rnpm_adapter *adapter, int pb)
{
	int marker = 0;
#if 0
	struct rnpm_hw *hw = &adapter->hw;
	struct net_device *dev = adapter->netdev;
	int link, tc, kb, marker = 0;
	u32 dv_id, rx_pba;

	/* Calculate max LAN frame size */
	tc = link = dev->mtu + ETH_HLEN + ETH_FCS_LEN + RNPM_ETH_FRAMING;

	/* Calculate delay value for device */
	switch (hw->mac.type) {
	case rnpm_mac_X540:
		dv_id = RNPM_DV_X540(link, tc);
		break;
	default:
		dv_id = RNPM_DV(link, tc);
		break;
	}

	/* Loopback switch introduces additional latency */
	if (adapter->flags & RNPM_FLAG_SRIOV_ENABLED)
		dv_id += RNPM_B2BT(tc);

	/* Delay value is calculated in bit times convert to KB */
	kb = RNPM_BT2KB(dv_id);
	rx_pba = RNPM_READ_REG(hw, RNPM_RXPBSIZE(pb)) >> 10;

	marker = rx_pba - kb;

	/* It is possible that the packet buffer is not large enough
	 * to provide required headroom. In this case throw an error
	 * to user and a do the best we can.
	 */
	if (marker < 0) {
		e_warn(drv, "Packet Buffer(%i) can not provide enough"
			    "headroom to support flow control."
			    "Decrease MTU or number of traffic classes\n", pb);
		marker = tc + 1;
	}
#endif
	return marker;
}

/**
 * rnpm_lpbthresh - calculate low water mark for for flow control
 *
 * @adapter: board private structure to calculate for
 * @pb: packet buffer to calculate
 */
__maybe_unused static int rnpm_lpbthresh(struct rnpm_adapter *adapter)
{
#if 0
	struct rnpm_hw *hw = &adapter->hw;
	struct net_device *dev = adapter->netdev;
	int tc;
	u32 dv_id;
	/* Calculate max LAN frame size */
	tc = dev->mtu + ETH_HLEN + ETH_FCS_LEN;

	/* Calculate delay value for device */
	switch (hw->mac.type) {
	case rnpm_mac_X540:
		dv_id = RNPM_LOW_DV_X540(tc);
		break;
	default:
		dv_id = RNPM_LOW_DV(tc);
		break;
	}

	/* Delay value is calculated in bit times convert to KB */
	return RNPM_BT2KB(dv_id);
#else
	return 0;
#endif
}

/*
 * rnpm_pbthresh_setup - calculate and setup high low water marks
 */
__maybe_unused static void rnpm_pbthresh_setup(struct rnpm_adapter *adapter)
{
#if 0
	struct rnpm_hw *hw = &adapter->hw;
	int num_tc = netdev_get_num_tc(adapter->netdev);
	int i;

	if (!num_tc)
		num_tc = 1;

	hw->fc.low_water = rnpm_lpbthresh(adapter);

	for (i = 0; i < num_tc; i++) {
		hw->fc.high_water[i] = rnpm_hpbthresh(adapter, i);

		/* Low water marks must not be larger than high water marks */
		if (hw->fc.low_water > hw->fc.high_water[i])
			hw->fc.low_water = 0;
	}
#endif
}

static void rnpm_configure_pb(struct rnpm_adapter *adapter)
{
#if 0
	struct rnpm_hw *hw = &adapter->hw;
	int hdrm;
	u8 tc = netdev_get_num_tc(adapter->netdev);

	if (adapter->flags & RNPM_FLAG_FDIR_HASH_CAPABLE ||
	    adapter->flags & RNPM_FLAG_FDIR_PERFECT_CAPABLE)
		hdrm = 32 << adapter->fdir_pballoc;
	else
		hdrm = 0;

	hw->mac.ops.set_rxpba(hw, tc, hdrm, PBA_STRATEGY_EQUAL);
	rnpm_pbthresh_setup(adapter);
#endif
}

static void rnpm_fdir_filter_restore(struct rnpm_adapter *adapter)
{
	struct rnpm_hw *hw = &adapter->hw;
	struct hlist_node *node2;
	struct rnpm_fdir_filter *filter;
	unsigned long flags;

	spin_lock_irqsave(&adapter->fdir_perfect_lock, flags);

	/*
	if (!hlist_empty(&adapter->fdir_filter_list))
		rnpm_fdir_set_input_mask_n10(hw, &adapter->fdir_mask);
	*/
	/* enable tcam if set tcam mode */
	if (adapter->fdir_mode == fdir_mode_tcam) {
		wr32(hw, RNPM_ETH_TCAM_EN, 1);
		wr32(hw, RNPM_TOP_ETH_TCAM_CONFIG_ENABLE, 1);
		wr32(hw, RNPM_TCAM_CACHE_ENABLE, 1);
	}

	/* setup ntuple */
	hlist_for_each_entry_safe(
		filter, node2, &adapter->fdir_filter_list, fdir_node)
	{
		rnpm_fdir_write_perfect_filter(
			adapter->fdir_mode,
			hw,
			&filter->filter,
			filter->hw_idx,
			(filter->action == RNPM_FDIR_DROP_QUEUE)
				? RNPM_FDIR_DROP_QUEUE
				: adapter->rx_ring[filter->action]->rnpm_queue_idx);
	}

	spin_unlock_irqrestore(&adapter->fdir_perfect_lock, flags);
}

// static void rnpm_configure_vlan(struct net_device *netdev,
//								struct rnpm_adapter *adapter)
//{
//	int i;
//	struct rnpm_hw *hw = &adapter->hw;
//
//	/* open vlan filter */
//	if (adapter->netdev->features & (NETIF_F_HW_VLAN_CTAG_FILTER)) {
//		// todo
//		rnpm_vlan_filter_on(hw);
//	} else {
//		// todo
//		rnpm_vlan_filter_off(hw);
//	}
// }

static void rnpm_configure_pause(struct rnpm_adapter *adapter)
{
	struct rnpm_hw *hw = &adapter->hw;
	hw->mac.ops.fc_enable(hw);
}

void rnpm_vlan_stags_flag(struct rnpm_adapter *adapter)
{
	struct rnpm_hw *hw = &adapter->hw;
	u8 port = adapter->port;

	/* stags is added */
	if (adapter->flags2 & RNPM_FLAG2_VLAN_STAGS_ENABLED) {
		/* low 16bits should not all zero */
		// wr32(hw, RNPM_MAC_TX_VLAN_TAG(port), 0xc60ffff);
		wr32(hw,
			 RNPM_MAC_TX_VLAN_TAG(port),
			 RNPM_ERIVLT | RNPM_EDVLP |
				 (RNPM_EVLS_ALWAYS_STRIP << RNPM_EVLS_OFFSET) |
				 RNPM_VL_MODE_ON);
		// wr32(hw, RNPM_MAC_TX_VLAN_MODE(port), 0x180000);
		wr32(hw, RNPM_MAC_TX_VLAN_MODE(port), 0x180000);
		wr32(hw, RNPM_MAC_INNER_VLAN_INCL(port), 0x100000);
	} else {
		/* low 16bits should not all zero */
		// wr32(hw, RNPM_MAC_TX_VLAN_TAG(port), 0x200ffff);
		wr32(hw, RNPM_MAC_TX_VLAN_TAG(port), RNPM_VTHM | RNPM_VL_MODE_ON);
		wr32(hw, RNPM_MAC_TX_VLAN_MODE(port), 0x100000);
		wr32(hw, RNPM_MAC_INNER_VLAN_INCL(port), 0x100000);
	}
}

static void rnpm_configure(struct rnpm_adapter *adapter)
{
	struct rnpm_hw *hw = &adapter->hw;

	rnpm_configure_pb(adapter); // setup high low water

#ifdef CONFIG_RNPM_DCB
	rnpm_configure_dcb(adapter);
#endif

	/*
	 * We must restore virtualization before VLANs or else
	 * the VLVF registers will not be populated
	 */
	rnpm_configure_virtualization(adapter);
	/* init setup pause */
	rnpm_configure_pause(adapter);
	/* Unicast, Multicast and Promiscuous mode set */
	rnpm_set_rx_mode(adapter->netdev);

	/* reset unicast address */
	hw->mac.ops.set_rar(
		hw, adapter->uc_off, hw->mac.addr, VMDQ_P(0), RNPM_RAH_AV);

	/* setup mac unicast filters */
	if (hw->mac.mc_location == rnpm_mc_location_mac) {
		hw->mac.ops.set_rar_mac(hw, 0, hw->mac.addr, VMDQ_P(0), adapter->port);
	}
	/* what conditions should restore vlan ? */
	rnpm_restore_vlan(adapter);
	/* setup rss key and table */
	/* enable all eth filter */
	wr32(hw, RNPM_HOST_FILTER_EN, 1);
	/* open redir */
	wr32(hw, RNPM_REDIR_EN, 1);
	//rnpm_init_rss_key(adapter);
	rnpm_init_rss_table(adapter);

	/* open sctp check en */
	if (hw->feature_flags & RNPM_NET_FEATURE_RX_CHECKSUM)
		wr32(hw, RNPM_ETH_SCTP_CHECKSUM_EN, 1);
	/* test this with stags */
	/* stags is stored in adapter->stags_vid */
	/*
	adapter->flags2 |= RNPM_FLAG2_VLAN_STAGS_ENABLED;
	adapter->stags_vid = htons(0x04);
	*/

	rnpm_vlan_stags_flag(adapter);

	if (adapter->flags & RNPM_FLAG_FDIR_HASH_CAPABLE) {
		// rnpm_init_fdir_signature_n10(&adapter->hw, adapter->fdir_pballoc);
	} else if (adapter->flags & RNPM_FLAG_FDIR_PERFECT_CAPABLE) {
		// rnpm_init_fdir_perfect_n10(&adapter->hw, adapter->fdir_pballoc);
		rnpm_fdir_filter_restore(adapter);
	}

	if (hw->dma_version >= 0x20210108) {
		// mark Multicast as broadcast
		wr32(hw, RNPM_VEB_MAC_MASK_LO, 0xffffffff);
		wr32(hw, RNPM_VEB_MAC_MASK_HI, 0xfeff);
	}

	/* setup rx slic 1536 bytes */
	/* only in page 4k mode ? */
	// rnpm_setup_dma_rx(adapter, RNPM_RXBUFFER_1536/16);
	rnpm_setup_dma_rx(adapter, 96);

	rnpm_configure_tx(adapter);
	rnpm_configure_rx(adapter);
}

static inline bool rnpm_is_sfp(struct rnpm_hw *hw)
{
	return true;
}

/**
 * rnpm_sfp_link_config - set up SFP+ link
 * @adapter: pointer to private adapter struct
 **/
static void rnpm_sfp_link_config(struct rnpm_adapter *adapter)
{
	/*
	 * We are assuming the worst case scenario here, and that
	 * is that an SFP was inserted/removed after the reset
	 * but before SFP detection was enabled.  As such the best
	 * solution is to just start searching as soon as we start
	 */
	adapter->flags2 |= RNPM_FLAG2_SFP_NEEDS_RESET;
}

/**
 * rnpm_non_sfp_link_config - set up non-SFP+ link
 * @hw: pointer to private hardware struct
 *
 * Returns 0 on success, negative on failure
 **/
static int rnpm_non_sfp_link_config(struct rnpm_hw *hw)
{
	u32 ret = RNPM_ERR_LINK_SETUP;
#if 0
	u32 speed;
	bool autoneg, link_up = false;
	if (hw->mac.ops.check_link)
		ret = hw->mac.ops.check_link(hw, &speed, &link_up, false);

	if (ret)
		goto link_cfg_out;

	speed = hw->phy.autoneg_advertised;
	if ((!speed) && (hw->mac.ops.get_link_capabilities))
		ret = hw->mac.ops.get_link_capabilities(hw, &speed,
							&autoneg);
	if (ret)
		goto link_cfg_out;

	if (hw->mac.ops.setup_link)
		ret = hw->mac.ops.setup_link(hw, speed, link_up);
link_cfg_out:
#endif
	return ret;
}

void control_mac_rx(struct rnpm_adapter *adapter, bool on)
{
	struct rnpm_hw *hw = &adapter->hw;
	//struct rnpm_pf_adapter *pf_adapter = adapter->pf_adapter;
	u8 port = adapter->port;
	u32 value = 0;
	u32 count = 0;

	if (on) {
		do {
			wr32(hw,
				 RNPM_MAC_RX_CFG(port),
				 rd32(hw, RNPM_MAC_RX_CFG(port)) | 0x01);
			usleep_range(100, 200);
			value = rd32(hw, RNPM_MAC_RX_CFG(port));
			count++;
			if (count > 1000) {
				printk("setup rx on timeout\n");
				break;
			}
			// printk("wait set rx open %x\n", value);
		} while (!(value & 0x01));

		// clean loop back
		do {
			wr32(hw,
				 RNPM_MAC_RX_CFG(port),
				 rd32(hw, RNPM_MAC_RX_CFG(port)) & (~0x400));
			usleep_range(100, 200);
			value = rd32(hw, RNPM_MAC_RX_CFG(port));
			count++;
			if (count > 1000) {
				printk("setup rx off timeout\n");
				break;
			}
			// printk("wait set rx open %x\n", value);
		} while (value & 0x400);

		wr32(hw, RNPM_ETH_RX_PROGFULL_THRESH_PORT(adapter->port), RECEIVE_ALL_THRESH);
		/* in this mode close mc filter in mac */
		if (hw->mac.mc_location == rnpm_mc_location_nic)
			wr32(hw,
				 RNPM_MAC_PKT_FLT(port),
				 rd32(hw, RNPM_MAC_PKT_FLT(port)) | RNPM_RA);
		else
			wr32(hw,
				 RNPM_MAC_PKT_FLT(port),
				 rd32(hw, RNPM_MAC_PKT_FLT(port)) | RNPM_HPF);
	} else {
		wr32(hw, RNPM_ETH_RX_PROGFULL_THRESH_PORT(adapter->port), DROP_ALL_THRESH);
		// set loopback
		do {
			wr32(hw,
				 RNPM_MAC_RX_CFG(port),
				 rd32(hw, RNPM_MAC_RX_CFG(port)) | 0x400);
			usleep_range(100, 200);
			value = rd32(hw, RNPM_MAC_RX_CFG(port));
			count++;
			if (count > 1000) {
				printk("setup rx on timeout\n");
				break;
			}
			// printk("wait set rx open %x\n", value);
		} while (!(value & 0x400));

		//wr32(hw, RNPM_MAC_PKT_FLT(port), 0x0);
	}
}

static void rnpm_up_complete(struct rnpm_adapter *adapter)
{
	struct rnpm_hw *hw = &adapter->hw;
	int err;
	int i;

	rnpm_get_hw_control(adapter);

	rnpm_configure_msix(adapter);

	/* enable the optics for n10 SFP+ fiber */
	if (hw->mac.ops.enable_tx_laser)
		hw->mac.ops.enable_tx_laser(hw);

	clear_bit(__RNPM_DOWN, &adapter->state);
	rnpm_napi_enable_all(adapter);

	if (rnpm_is_sfp(hw)) {
		rnpm_sfp_link_config(adapter);
	} else {
		err = rnpm_non_sfp_link_config(hw);
		if (err)
			e_err(probe, "link_config FAILED %d\n", err);
	}
	/*clear any pending interrupts*/
	rnpm_irq_enable(adapter);

	/* enable transmits */
	netif_tx_start_all_queues(adapter->netdev);

	/* enable rx transmit */
	for (i = 0; i < adapter->num_rx_queues; i++)
		wr32(hw, RNPM_DMA_RX_START(adapter->rx_ring[i]->rnpm_queue_idx), 1);

#ifdef RNPM_DISABLE_IRQ
	rnpm_mbx_link_event_enable(&adapter->hw, false);
#else
	rnpm_mbx_link_event_enable(&adapter->hw, true);
#endif
	mod_timer(&adapter->service_timer, HZ + jiffies);
	rnpm_mbx_ifup_down(&adapter->hw, MBX_IFUP);
	control_mac_rx(adapter, true);
	/* Set PF Reset Done bit so PF/VF Mail Ops can work */
}

void rnpm_reinit_locked(struct rnpm_adapter *adapter)
{
	WARN_ON(in_interrupt());
	/* put off any impending NetWatchDogTimeout */
	// adapter->netdev->trans_start = jiffies;

	while (test_and_set_bit(__RNPM_RESETTING, &adapter->state))
		usleep_range(1000, 2000);
	rnpm_down(adapter);
	/*
	 * If SR-IOV enabled then wait a bit before bringing the adapter
	 * back up to give the VFs time to respond to the reset.  The
	 * two second wait is based upon the watchdog timer cycle in
	 * the VF driver.
	 */
	if (adapter->flags & RNPM_FLAG_SRIOV_ENABLED)
		msleep(2000);
	rnpm_up(adapter);
	clear_bit(__RNPM_RESETTING, &adapter->state);
}

void rnpm_up(struct rnpm_adapter *adapter)
{
	/* hardware has been reset, we need to reload some things */
	rnpm_configure(adapter);

	rnpm_up_complete(adapter);
}

void rnpm_reset(struct rnpm_adapter *adapter)
{
	struct rnpm_hw *hw = &adapter->hw;
	int err;

	rnpm_dbg("%s\n", __func__);

	/* lock SFP init bit to prevent race conditions with the watchdog */
	while (test_and_set_bit(__RNPM_IN_SFP_INIT, &adapter->state))
		usleep_range(1000, 2000);

	/* clear all SFP and link config related flags while holding SFP_INIT */
	adapter->flags2 &=
		~(RNPM_FLAG2_SEARCH_FOR_SFP | RNPM_FLAG2_SFP_NEEDS_RESET);
	adapter->flags &= ~RNPM_FLAG_NEED_LINK_CONFIG;

	/* if sriov and mutiport port, don’t real reset */
	/*
	if ((adapter->flags & RNPM_FLAG_SRIOV_ENABLED) ||
	   (adapter->flags & RNPM_FLAG_MUTIPORT_ENABLED))
		return;
	*/

	err = hw->mac.ops.init_hw(hw);
	if (err) {
		e_dev_err("init_hw: Hardware Error: err:%d. line:%d\n", err, __LINE__);
	}

	clear_bit(__RNPM_IN_SFP_INIT, &adapter->state);

	/* reprogram the RAR[0] in case user changed it. */
	hw->mac.ops.set_rar(
		hw, adapter->uc_off, hw->mac.addr, VMDQ_P(0), RNPM_RAH_AV);
	/* setup mac unicast filters */
	if (hw->mac.mc_location == rnpm_mc_location_mac) {
		hw->mac.ops.set_rar_mac(hw, 0, hw->mac.addr, VMDQ_P(0), adapter->port);
	}

#ifndef NO_PTP
	if (module_enable_ptp) {
		if (adapter->flags2 & RNPM_FLAG2_PTP_ENABLED &&
			(adapter->ptp_rx_en || adapter->ptp_tx_en))
			rnpm_ptp_reset(adapter);
	}
#endif
}

/**
 * rnpm_clean_rx_ring - Free Rx Buffers per Queue
 * @rx_ring: ring to free buffers from
 **/
static void rnpm_clean_rx_ring(struct rnpm_ring *rx_ring)
{
	u16 i = rx_ring->next_to_clean;
	struct rnpm_rx_buffer *rx_buffer = &rx_ring->rx_buffer_info[i];
#if defined(HAVE_STRUCT_DMA_ATTRS) && defined(HAVE_SWIOTLB_SKIP_CPU_SYNC)
	DEFINE_DMA_ATTRS(attrs);
	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
	dma_set_attr(DMA_ATTR_WEAK_ORDERING, &attrs);
#endif

	/*
	#ifdef HAVE_AF_XDP_ZC_SUPPORT
		if (rx_ring->xsk_umem) {
			rnpm_xsk_clean_rx_ring(rx_ring);
			goto skip_free;
		}

	#endif
	*/
	/* Free all the Rx ring sk_buffs */
#ifdef CONFIG_RNPM_DISABLE_PACKET_SPLIT
	while (i != rx_ring->next_to_use) {
#else
	while (i != rx_ring->next_to_alloc) {
#endif
		if (rx_buffer->skb) {
			struct sk_buff *skb = rx_buffer->skb;
#ifndef CONFIG_RNPM_DISABLE_PACKET_SPLIT
			/* no need this */
			if (RNPM_CB(skb)->page_released)
				dma_unmap_page_attrs(rx_ring->dev,
									 RNPM_CB(skb)->dma,
									 rnpm_rx_pg_size(rx_ring),
									 DMA_FROM_DEVICE,
#if defined(HAVE_STRUCT_DMA_ATTRS) && defined(HAVE_SWIOTLB_SKIP_CPU_SYNC)
									 &attrs);
#else
									 RNPM_RX_DMA_ATTR);
#endif
#else
			/* We need to clean up RSC frag lists */
			skb = rnpm_merge_active_tail(skb);
			if (rnpm_close_active_frag_list(skb))
				dma_unmap_single(rx_ring->dev,
								 RNPM_CB(skb)->dma,
								 rx_ring->rx_buf_len,
								 DMA_FROM_DEVICE);
			RNPM_CB(skb)->dma = 0;
#endif /* CONFIG_RNPM_DISABLE_PACKET_SPLIT */
			dev_kfree_skb(skb);
			rx_buffer->skb = NULL;
		}

#ifndef CONFIG_RNPM_DISABLE_PACKET_SPLIT
		/* Invalidate cache lines that may have been written to by
		 * device so that we avoid corrupting memory.
		 */
		dma_sync_single_range_for_cpu(rx_ring->dev,
									  rx_buffer->dma,
									  rx_buffer->page_offset,
									  rnpm_rx_bufsz(rx_ring),
									  DMA_FROM_DEVICE);

		/* free resources associated with mapping */
		dma_unmap_page_attrs(rx_ring->dev,
							 rx_buffer->dma,
							 rnpm_rx_pg_size(rx_ring),
							 DMA_FROM_DEVICE,
#if defined(HAVE_STRUCT_DMA_ATTRS) && defined(HAVE_SWIOTLB_SKIP_CPU_SYNC)
							 &attrs);
#else
							 RNPM_RX_DMA_ATTR);
#endif

		__page_frag_cache_drain(rx_buffer->page, rx_buffer->pagecnt_bias);
#else  /* CONFIG_RNPM_DISABLE_PACKET_SPLIT */
		if (rx_buffer->dma) {
			dma_unmap_single(rx_ring->dev,
							 rx_buffer->dma,
							 rx_ring->rx_buf_len,
							 DMA_FROM_DEVICE);
			rx_buffer->dma = 0;
		}
#endif /* CONFIG_RNPM_DISABLE_PACKET_SPLIT */
		/* now this page is not used */
		rx_buffer->page = NULL;
		i++;
		rx_buffer++;
		if (i == rx_ring->count) {
			i = 0;
			rx_buffer = rx_ring->rx_buffer_info;
		}
	}

#ifdef HAVE_AF_XDP_ZC_SUPPORT
//skip_free:
#endif
#ifndef CONFIG_RNPM_DISABLE_PACKET_SPLIT
	rx_ring->next_to_alloc = 0;
	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;
#endif
}

/**
 * rnpm_clean_tx_ring - Free Tx Buffers
 * @tx_ring: ring to be cleaned
 **/
static void rnpm_clean_tx_ring(struct rnpm_ring *tx_ring)
{
	unsigned long size;
	u16 i = tx_ring->next_to_clean;
	struct rnpm_tx_buffer *tx_buffer = &tx_ring->tx_buffer_info[i];

	BUG_ON(tx_ring == NULL);

	// printk("next_to_clean %d\n", i);
	// printk("next_to_use %d\n", tx_ring->next_to_use);
	// printk("ring idx is %d\n", tx_ring->rnpm_queue_idx);
	/* ring already cleared, nothing to do */
	if (!tx_ring->tx_buffer_info)
		return;
	while (i != tx_ring->next_to_use) {
		struct rnpm_tx_desc *eop_desc, *tx_desc;

		dev_kfree_skb_any(tx_buffer->skb);
		/* unmap skb header data */
		dma_unmap_single(tx_ring->dev,
						 dma_unmap_addr(tx_buffer, dma),
						 dma_unmap_len(tx_buffer, len),
						 DMA_TO_DEVICE);

		eop_desc = tx_buffer->next_to_watch;
		/*
		if (!eop_desc) {
			printk("eop_desc is null\n");
			tx_buffer++;
			//i++;
			break;
			//continue;

		}
		*/
		tx_desc = RNPM_TX_DESC(tx_ring, i);
		/* unmap remaining buffers */
		while (tx_desc != eop_desc) {
			tx_buffer++;
			tx_desc++;
			i++;
			if (unlikely(i == tx_ring->count)) {
				i = 0;
				tx_buffer = tx_ring->tx_buffer_info;
				tx_desc = RNPM_TX_DESC(tx_ring, 0);
			}

			/* unmap any remaining paged data */
			if (dma_unmap_len(tx_buffer, len)) {
				dma_unmap_page(tx_ring->dev,
							   dma_unmap_addr(tx_buffer, dma),
							   dma_unmap_len(tx_buffer, len),
							   DMA_TO_DEVICE);
				dma_unmap_len_set(tx_buffer, len, 0);
			}
			// printk("free not eop i is %d\n", i);
		}
		/* move us one more past the eop_desc for start of next pkt */
		tx_buffer++;
		i++;
		if (unlikely(i == tx_ring->count)) {
			i = 0;
			tx_buffer = tx_ring->tx_buffer_info;
		}
		// printk("now i is %d\n", i);
	}

	/* Free all the Tx ring sk_buffs */
	/*
	for (i = 0; i < tx_ring->count; i++) {
		tx_buffer_info = &tx_ring->tx_buffer_info[i];
		rnpm_unmap_and_free_tx_resource(tx_ring, tx_buffer_info);
	}
	*/

	netdev_tx_reset_queue(txring_txq(tx_ring));

	size = sizeof(struct rnpm_tx_buffer) * tx_ring->count;
	memset(tx_ring->tx_buffer_info, 0, size);

	/* Zero out the descriptor ring */
	memset(tx_ring->desc, 0, tx_ring->size);

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;
}

/**
 * rnpm_clean_all_rx_rings - Free Rx Buffers for all queues
 * @adapter: board private structure
 **/
static void rnpm_clean_all_rx_rings(struct rnpm_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++)
		rnpm_clean_rx_ring(adapter->rx_ring[i]);
}

/**
 * rnpm_clean_all_tx_rings - Free Tx Buffers for all queues
 * @adapter: board private structure
 **/
static void rnpm_clean_all_tx_rings(struct rnpm_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++)
		rnpm_clean_tx_ring(adapter->tx_ring[i]);
}

static void rnpm_fdir_filter_exit(struct rnpm_adapter *adapter)
{
	struct hlist_node *node2;
	struct rnpm_fdir_filter *filter;
	struct rnpm_hw *hw = &adapter->hw;
	unsigned long flags;

	spin_lock_irqsave(&adapter->fdir_perfect_lock, flags);

	hlist_for_each_entry_safe(
		filter, node2, &adapter->fdir_filter_list, fdir_node)
	{
		/* call earase to hw */
		rnpm_fdir_erase_perfect_filter(
			adapter->fdir_mode, hw, &filter->filter, filter->hw_idx);

		hlist_del(&filter->fdir_node);
		kfree(filter);
	}
	adapter->fdir_filter_count = 0;

	spin_unlock_irqrestore(&adapter->fdir_perfect_lock, flags);
}

void rnpm_down(struct rnpm_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	struct rnpm_hw *hw = &adapter->hw;
	int i;

	rnpm_dbg("%s rnpm_down!!!\n", netdev->name);

	control_mac_rx(adapter, false);

	/* signal that we are down to the interrupt handler */
	set_bit(__RNPM_DOWN, &adapter->state);

	netif_tx_stop_all_queues(netdev);

	netif_carrier_off(netdev);

	netif_tx_disable(netdev);

	// wait all packets loop back
	usleep_range(10000, 20000);

	/* disable all enabled rx queues */
	for (i = 0; i < adapter->num_rx_queues; i++) {
		rnpm_disable_rx_queue(adapter, adapter->rx_ring[i]);
		/* only handle when srio enable or mutiport mode and change rx length
		 * setup */
		if (((adapter->flags & RNPM_FLAG_SRIOV_ENABLED) ||
			 (adapter->flags & RNPM_FLAG_MUTIPORT_ENABLED)) &&
			(adapter->rx_ring[i]->ring_flags & RNPM_RING_FLAG_CHANGE_RX_LEN)) {
			int head;

			head = rd32(hw,
						RNPM_DMA_REG_RX_DESC_BUF_HEAD(
							adapter->rx_ring[i]->rnpm_queue_idx));
			adapter->rx_ring[i]->ring_flags &= (~RNPM_RING_FLAG_CHANGE_RX_LEN);
			/* we should delay setup rx length to wait rx head to 0 */
			if (head >= adapter->rx_ring[i]->reset_count) {
				adapter->rx_ring[i]->ring_flags |=
					RNPM_RING_FLAG_DELAY_SETUP_RX_LEN;
				/* set sw count to head + 1*/
				adapter->rx_ring[i]->temp_count = head + 1;
			}
		}
		/* only down without rx_len change no need handle */
	}
	/* call carrier off first to avoid false dev_watchdog timeouts */

	rnpm_irq_disable(adapter);

	rnpm_napi_disable_all(adapter);

	adapter->flags2 &=
		~(RNPM_FLAG2_FDIR_REQUIRES_REINIT | RNPM_FLAG2_RESET_REQUESTED);
	adapter->flags &= ~RNPM_FLAG_NEED_LINK_UPDATE;

	del_timer_sync(&adapter->service_timer);

	// maybe bug here if call tx real hang reset
	// cancel_work_sync(&adapter->service_task);

	if (adapter->num_vfs) {

		/* Mark all the VFs as inactive */
		/* how to deal this is not sure */
		/*
		for (i = 0 ; i < adapter->num_vfs; i++)
			adapter->vfinfo[i].clear_to_send = false;
		*/
		/* ping all the active vfs to let them know we are going down */
		rnpm_ping_all_vfs(adapter);

		/* Disable all VFTE/VFRE TX/RX */
		rnpm_disable_tx_rx(adapter);
	}

	// wait all packets loop back again， wait napi_schedule_irqoff exit
	usleep_range(10000, 20000);

	/* disable transmits in the hardware now that interrupts are off */
	for (i = 0; i < adapter->num_tx_queues; i++) {
		struct rnpm_hw *hw = &adapter->hw;
		struct rnpm_ring *tx_ring = adapter->tx_ring[i];
		int count = tx_ring->count;
		int head, tail;
		int timeout = 0;
		u32 status = 0;
		/* 1. stop queue */
		// check tx ready
		do {
			status = rd32(hw, RNPM_DMA_TX_READY(tx_ring->rnpm_queue_idx));
			usleep_range(1000, 2000);
			timeout++;
			rnpm_dbg("wait %d tx ready to 1\n", tx_ring->rnpm_queue_idx);
		} while ((status != 1) && (timeout < 100));

		if (timeout >= 100) {
			head = rd32(hw,
			RNPM_DMA_REG_TX_DESC_BUF_HEAD(tx_ring->rnpm_queue_idx));

			tail = rd32(hw,
			RNPM_DMA_REG_TX_DESC_BUF_TAIL(tx_ring->rnpm_queue_idx));
			printk("wait tx ready timeout, name=%s, i=%d queue_idx=%d head=%d tail=%d\n", netdev->name, i, tx_ring->rnpm_queue_idx, head, tail);
		}
		// wr32(hw, RNPM_DMA_TX_START(tx_ring->rnpm_queue_idx), 0);
		// usleep_range(10000, 20000);
		/* 2. try to set tx head to 0 in sriov mode since we don't reset
		   in sriov on or mutiport mode  */
		if ((adapter->flags & RNPM_FLAG_SRIOV_ENABLED) ||
			(adapter->flags & RNPM_FLAG_MUTIPORT_ENABLED)) {
			head = rd32(hw,
						RNPM_DMA_REG_TX_DESC_BUF_HEAD(tx_ring->rnpm_queue_idx));
			if (head != 0) {
				u16 next_to_use = tx_ring->next_to_use;
				if (head != (count - 1)) {
					/* 3 set len head + 1 */
					wr32(hw,
						 RNPM_DMA_REG_TX_DESC_BUF_LEN(tx_ring->rnpm_queue_idx),
						 head + 1);
					// tx_ring->count = head + 1;
				}
				/* set to use head */
				tx_ring->next_to_use = head;
				/* 4 send a len zero packet */
				rnpm_xmit_nop_frame_ring(adapter, tx_ring);
				// wr32(hw, RNPM_DMA_TX_START(tx_ring->rnpm_queue_idx), 1);
				/* 5 wait head to zero */
				while ((head != 0) && (timeout < 1000)) {
					head = rd32(
						hw,
						RNPM_DMA_REG_TX_DESC_BUF_HEAD(tx_ring->rnpm_queue_idx));
					usleep_range(10000, 20000);
					timeout++;
				}
				if (timeout >= 1000) {
					rnpm_dbg("[%s] Wait Rx-ring %d head to zero time out\n",
							 netdev->name,
							 tx_ring->rnpm_queue_idx);
				} else {
					// printk("set %d head to 0 ok\n", tx_ring->rnpm_queue_idx);
				}
				/* 6 stop queue again*/
				// wr32(hw, RNPM_DMA_TX_START(tx_ring->rnpm_queue_idx), 0);

				/* 7 write back next_to_use maybe hw hang */
				tx_ring->next_to_use = next_to_use;
			}
		}
	}

	if (!pci_channel_offline(adapter->pdev)) {
		if (!(adapter->flags & RNPM_FLAG_SRIOV_ENABLED) &&
			(!(adapter->flags & RNPM_FLAG_MUTIPORT_ENABLED))) {
			rnpm_reset(adapter);
		}
	}

	/* power down the optics for n10 SFP+ fiber */
	if (hw->mac.ops.disable_tx_laser)
		hw->mac.ops.disable_tx_laser(hw);

	rnpm_clean_all_tx_rings(adapter);
	rnpm_clean_all_rx_rings(adapter);

#ifdef CONFIG_RNPM_DCA
	/* since we reset the hardware DCA settings were cleared */
	rnpm_setup_dca(adapter);
#endif
}

/**
 * rnpm_tx_timeout - Respond to a Tx Hang
 * @netdev: network interface device structure
 **/
#ifdef HAVE_TX_TIMEOUT_TXQUEUE
static void rnpm_tx_timeout(struct net_device *netdev, unsigned int txqueue)
#else
static void rnpm_tx_timeout(struct net_device *netdev)
#endif
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	/* Do the reset outside of interrupt context */
	int i;
	bool real_tx_hang = false;

#define TX_TIMEO_LIMIT 16000
	for (i = 0; i < adapter->num_tx_queues; i++) {
		struct rnpm_ring *tx_ring = adapter->tx_ring[i];
		if (check_for_tx_hang(tx_ring) && rnpm_check_tx_hang(tx_ring))
			real_tx_hang = true;
	}

	if (real_tx_hang) {
		/* Do the reset outside of interrupt context */
		e_info(drv, "tx real hang\n");
		rnpm_tx_timeout_reset(adapter);
	} else {
		e_info(drv, "Fake Tx hang detected with timeout of %d "
			   "seconds\n",
			   netdev->watchdog_timeo / HZ);

		/* fake Tx hang - increase the kernel timeout */
		if (netdev->watchdog_timeo < TX_TIMEO_LIMIT)
			netdev->watchdog_timeo *= 2;
	}
}

/**
 * rnpm_sw_init - Initialize general software structures (struct rnpm_adapter)
 * @adapter: board private structure to initialize
 *
 * rnpm_sw_init initializes the Adapter private data structure.
 * Fields are initialized based on PCI device information and
 * OS network device settings (MTU size).
 **/
static int rnpm_sw_init(struct rnpm_adapter *adapter)
{
	struct rnpm_hw *hw = &adapter->hw;
	struct pci_dev *pdev = adapter->pdev;
	unsigned int rss = 0, fdir;
	int i;
#ifdef CONFIG_RNPM_DCB
	int j;
	struct tc_configuration *tc;
#endif

	hw->vendor_id = pdev->vendor;
	hw->device_id = pdev->device;
	hw->subsystem_vendor_id = pdev->subsystem_vendor;
	hw->subsystem_device_id = pdev->subsystem_device;

	/* Set common capability flags and settings */
	if (hw->rss_type == rnpm_rss_uv3p) {
		/* Makefile use RNPM_MAX_RINGS to limit ring number */
		rss = min_t(int, adapter->max_ring_pair_counts, num_online_cpus());
	} else {
		rss = min_t(int, adapter->max_ring_pair_counts, num_online_cpus());
	}
#ifdef RNPM_DEFAULT_RINGS_CNT
	rss = min_t(int, rss, RNPM_DEFAULT_RINGS_CNT);
#endif
	// should limit queue since cpu maybe large than vectors number
	rss = min_t(int, rss, adapter->max_msix_counts);
	adapter->ring_feature[RING_F_RSS].limit =
		min_t(int, rss, adapter->max_ring_pair_counts);

	adapter->flags2 |= RNPM_FLAG2_RSC_CAPABLE;
	adapter->flags2 |= RNPM_FLAG2_RSC_ENABLED;
	adapter->atr_sample_rate = 20;

#ifdef RNPM_MAX_RINGS
	fdir = min_t(int, 32, RNPM_MAX_RINGS);
#else
	fdir = min_t(int, 32, num_online_cpus());
#endif
	// no-use this ?
	adapter->ring_feature[RING_F_FDIR].limit = fdir;

	if (hw->feature_flags & RNPM_NET_FEATURE_RX_NTUPLE_FILTER) {
		spin_lock_init(&adapter->fdir_perfect_lock);
		/* init count record */
		adapter->fdir_filter_count = 0;
		adapter->layer2_count = 0;
		adapter->tuple_5_count = 0;
		if (hw->feature_flags & RNPM_NET_FEATURE_TCAM)
			adapter->fdir_mode = fdir_mode_tcam;
		else
			adapter->fdir_mode = fdir_mode_tuple5;

		adapter->fdir_pballoc =
			adapter->layer2_count_max + adapter->tuple_5_count_max;
		// adapter->flags |= RNPM_FLAG_FDIR_PERFECT_CAPABLE;
	}
#ifdef CONFIG_RNPM_DCA
	adapter->flags |= RNPM_FLAG_DCA_CAPABLE;
#endif
	/* itr sw setup here */
	adapter->sample_interval = RNPM_DEFAULT_SAMPLE_INTERVAL;
	adapter->adaptive_rx_coal = RNPM_DEFAULT_ENABLE;
	adapter->adaptive_tx_coal = RNPM_DEFAULT_ENABLE;
	adapter->auto_rx_coal = RNPM_DEFAULT_DISABLE;
	adapter->napi_budge = RNPM_DEFAULT_NAPI_BUDGE;

	/* set default work limits */
	adapter->tx_work_limit = rnpm_info_tbl[adapter->pf_adapter->board_type]->coalesce.tx_work_limit;
	adapter->rx_usecs = rnpm_info_tbl[adapter->pf_adapter->board_type]->coalesce.rx_usecs;
	adapter->rx_frames = rnpm_info_tbl[adapter->pf_adapter->board_type]->coalesce.rx_frames;
	adapter->tx_usecs = rnpm_info_tbl[adapter->pf_adapter->board_type]->coalesce.tx_usecs;
	adapter->tx_frames = rnpm_info_tbl[adapter->pf_adapter->board_type]->coalesce.tx_frames;

	/* Set MAC specific capability flags and exceptions */
	/* port capability is set here */
	switch (hw->mode) {
		case MODE_NIC_MODE_1PORT_40G:
		case MODE_NIC_MODE_1PORT:
			adapter->uc_num = hw->mac.num_rar_entries;
			adapter->uc_off = 0;
			break;
			/* multiple ports use mac */
		case MODE_NIC_MODE_2PORT:
		case MODE_NIC_MODE_4PORT:
			adapter->uc_num = hw->mac.num_rar_entries / 4;
			adapter->uc_off = adapter->uc_num * adapter->port;
			break;
		default:
			break;
	}

		/* n-tuple support exists, always init our spinlock */
		/* init fdir count */

#if 0
#ifdef CONFIG_RNPM_DCB
	adapter->dcb_cfg.num_tcs.pg_tcs = X540_TRAFFIC_CLASS;
	adapter->dcb_cfg.num_tcs.pfc_tcs = X540_TRAFFIC_CLASS;

	/* Configure DCB traffic classes */
	for (j = 0; j < MAX_TRAFFIC_CLASS; j++) {
		tc = &adapter->dcb_cfg.tc_config[j];
		tc->path[DCB_TX_CONFIG].bwg_id = 0;
		tc->path[DCB_TX_CONFIG].bwg_percent = 12 + (j & 1);
		tc->path[DCB_RX_CONFIG].bwg_id = 0;
		tc->path[DCB_RX_CONFIG].bwg_percent = 12 + (j & 1);
		tc->dcb_pfc = pfc_disabled;
	}

	/* Initialize default user to priority mapping, UPx->TC0 */
	tc = &adapter->dcb_cfg.tc_config[0];
	tc->path[DCB_TX_CONFIG].up_to_tc_bitmap = 0xFF;
	tc->path[DCB_RX_CONFIG].up_to_tc_bitmap = 0xFF;

	adapter->dcb_cfg.bw_percentage[DCB_TX_CONFIG][0] = 100;
	adapter->dcb_cfg.bw_percentage[DCB_RX_CONFIG][0] = 100;
	adapter->dcb_cfg.pfc_mode_enable = false;
	adapter->dcb_set_bitmap = 0x00;
	adapter->dcbx_cap = DCB_CAP_DCBX_HOST | DCB_CAP_DCBX_VER_CEE;
	memcpy(&adapter->temp_dcb_cfg, &adapter->dcb_cfg,
	       sizeof(adapter->temp_dcb_cfg));

#endif

	/* default flow control settings */
	hw->fc.requested_mode = rnpm_fc_full;
	hw->fc.current_mode = rnpm_fc_full;	/* init for ethtool output */
	rnpm_pbthresh_setup(adapter);
	hw->fc.pause_time = RNPM_DEFAULT_FCPAUSE;
	hw->fc.send_xon = true;
	hw->fc.disable_fc_autoneg =
		(rnpm_device_supports_autoneg_fc(hw) == 0) ? false : true;
#endif
		/* enable itr by default in dynamic mode */
		// adapter->rx_itr_setting = 1;
		// adapter->tx_itr_setting = 1;

#ifdef CONFIG_PCI_IOV
	adapter->num_vfs = (max_vfs > (RNPM_MAX_VF_FUNCTIONS - 1)) ? 0 : max_vfs;
#endif

	/* set default ring sizes */
	adapter->tx_ring_item_count = rnpm_info_tbl[adapter->pf_adapter->board_type]->queue_depth;
	adapter->rx_ring_item_count = rnpm_info_tbl[adapter->pf_adapter->board_type]->queue_depth;

	/* initialize eeprom parameters */
	if (rnpm_init_eeprom_params_generic(hw)) {
		e_dev_err("EEPROM initialization failed\n");
		return -EIO;
	}

	/*initialization default pause flow */
	hw->fc.requested_mode = rnpm_fc_full;
	// hw->fc.requested_mode = rnpm_fc_none;
	hw->fc.pause_time = RNPM_DEFAULT_FCPAUSE;
	hw->fc.current_mode = rnpm_fc_full;
	// hw->fc.current_mode = rnpm_fc_none;
	for (i = 0; i < RNPM_MAX_TRAFFIC_CLASS; i++) {
		hw->fc.high_water[i] = RNPM_DEFAULT_HIGH_WATER;
		hw->fc.low_water[i] = RNPM_DEFAULT_LOW_WATER;
	}

	set_bit(__RNPM_DOWN, &adapter->state);

	return 0;
}

/**
 * rnpm_setup_tx_resources - allocate Tx resources (Descriptors)
 * @tx_ring:    tx descriptor ring (for a specific queue) to setup
 *
 * Return 0 on success, negative on failure
 **/

int rnpm_setup_tx_resources(struct rnpm_ring *tx_ring,
							struct rnpm_adapter *adapter)
{
	struct device *dev = tx_ring->dev;
	int orig_node = dev_to_node(dev);
	int numa_node = NUMA_NO_NODE;
	int size;

	size = sizeof(struct rnpm_tx_buffer) * tx_ring->count;

	if (tx_ring->q_vector)
		numa_node = tx_ring->q_vector->numa_node;

	tx_ring->tx_buffer_info = vzalloc_node(size, numa_node);
	if (!tx_ring->tx_buffer_info)
		tx_ring->tx_buffer_info = vzalloc(size);
	if (!tx_ring->tx_buffer_info)
		goto err;

	// memset(tx_ring->tx_buffer_info, 0, size);

	/* round up to nearest 4K */
	tx_ring->size = tx_ring->count * sizeof(struct rnpm_tx_desc);
	tx_ring->size = ALIGN(tx_ring->size, 4096);

	set_dev_node(dev, numa_node);
	tx_ring->desc =
		dma_alloc_coherent(dev, tx_ring->size, &tx_ring->dma, GFP_KERNEL);
	set_dev_node(dev, orig_node);
	if (!tx_ring->desc)
		tx_ring->desc =
			dma_alloc_coherent(dev, tx_ring->size, &tx_ring->dma, GFP_KERNEL);
	if (!tx_ring->desc)
		goto err;
	memset(tx_ring->desc, 0, tx_ring->size);

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;

	DPRINTK(IFUP,
			INFO,
			"TxRing:%d,"
			"vector:%d ItemCounts:%d desc:%p(0x%llx) node:%d\n",
			tx_ring->rnpm_queue_idx,
			tx_ring->q_vector->v_idx,
			tx_ring->count,
			tx_ring->desc,
			tx_ring->dma,
			numa_node);
	return 0;
err:
	rnpm_err("%s [SetupTxResources] #%d TxRing:%d,"
			 "vector:%d ItemCounts:%d\n",
			 tx_ring->netdev->name,
			 tx_ring->queue_index,
			 tx_ring->rnpm_queue_idx,
			 tx_ring->q_vector->v_idx,
			 tx_ring->count);
	vfree(tx_ring->tx_buffer_info);
	tx_ring->tx_buffer_info = NULL;
	dev_err(dev, "Unable to allocate memory for the Tx descriptor ring\n");
	return -ENOMEM;
}

/**
 * rnpm_setup_all_tx_resources - allocate all queues Tx resources
 * @adapter: board private structure
 *
 * If this function returns with an error, then it's possible one or
 * more of the rings is populated (while the rest are not).  It is the
 * callers duty to clean those orphaned rings.
 *
 * Return 0 on success, negative on failure
 **/
static int rnpm_setup_all_tx_resources(struct rnpm_adapter *adapter)
{
	int i, err = 0;

	tx_dbg("adapter->num_tx_queues:%d, adapter->tx_ring[0]:%p\n",
		   adapter->num_tx_queues,
		   adapter->tx_ring[0]);

	for (i = 0; i < (adapter->num_tx_queues); i++) {
		BUG_ON(adapter->tx_ring[i] == NULL);
		err = rnpm_setup_tx_resources(adapter->tx_ring[i], adapter);
		if (!err)
			continue;

		e_err(probe, "Allocation for Tx Queue %u failed\n", i);
		goto err_setup_tx;
	}

	return 0;
err_setup_tx:
	/* rewind the index freeing the rings as we go */
	while (i--)
		rnpm_free_tx_resources(adapter->tx_ring[i]);
	return err;
}

/**
 * rnpm_setup_rx_resources - allocate Rx resources (Descriptors)
 * @rx_ring:    rx descriptor ring (for a specific queue) to setup
 *
 * Returns 0 on success, negative on failure
 **/
int rnpm_setup_rx_resources(struct rnpm_ring *rx_ring,
							struct rnpm_adapter *adapter)
{
	struct device *dev = rx_ring->dev;
	int orig_node = dev_to_node(dev);
	int numa_node = -1;
	int size;

	BUG_ON(rx_ring == NULL);

	size = sizeof(struct rnpm_rx_buffer) * rx_ring->count;

	if (rx_ring->q_vector)
		numa_node = rx_ring->q_vector->numa_node;

	rx_ring->rx_buffer_info = vzalloc_node(size, numa_node);
	if (!rx_ring->rx_buffer_info)
		rx_ring->rx_buffer_info = vzalloc(size);
	if (!rx_ring->rx_buffer_info)
		goto err;

	// memset(rx_ring->rx_buffer_info, 0, size);

	/* Round up to nearest 4K */
	rx_ring->size = rx_ring->count * sizeof(union rnpm_rx_desc);
	rx_ring->size = ALIGN(rx_ring->size, 4096);

	set_dev_node(dev, numa_node);
	rx_ring->desc =
		dma_alloc_coherent(dev, rx_ring->size, &rx_ring->dma, GFP_KERNEL);
	set_dev_node(dev, orig_node);
	if (!rx_ring->desc)
		rx_ring->desc =
			dma_alloc_coherent(dev, rx_ring->size, &rx_ring->dma, GFP_KERNEL);
	if (!rx_ring->desc)
		goto err;
	memset(rx_ring->desc, 0, rx_ring->size);

	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;

	DPRINTK(IFUP,
			INFO,
			"RxRing:%d, vector:%d "
			"ItemCounts:%d desc:%p(0x%llx) node:%d\n",
			rx_ring->rnpm_queue_idx,
			rx_ring->q_vector->v_idx,
			rx_ring->count,
			rx_ring->desc,
			rx_ring->dma,
			numa_node);

	return 0;
err:
	rnpm_err("%s [SetupRxResources] #%d RxRing:%d, vector:%d "
			 "ItemCounts:%d error!\n",
			 rx_ring->netdev->name,
			 rx_ring->queue_index,
			 rx_ring->rnpm_queue_idx,
			 rx_ring->q_vector->v_idx,
			 rx_ring->count);

	vfree(rx_ring->rx_buffer_info);
	rx_ring->rx_buffer_info = NULL;
	dev_err(dev, "Unable to allocate memory for the Rx descriptor ring\n");
	return -ENOMEM;
}

/**
 * rnpm_setup_all_rx_resources - allocate all queues Rx resources
 * @adapter: board private structure
 *
 * If this function returns with an error, then it's possible one or
 * more of the rings is populated (while the rest are not).  It is the
 * callers duty to clean those orphaned rings.
 *
 * Return 0 on success, negative on failure
 **/
static int rnpm_setup_all_rx_resources(struct rnpm_adapter *adapter)
{
	int i, err = 0;
	struct rnpm_hw *hw = &adapter->hw;
	u32 head;

	for (i = 0; i < adapter->num_rx_queues; i++) {
		BUG_ON(adapter->rx_ring[i] == NULL);
		/* should check count and head */
		/* in sriov condition may head large than count */
		head = rd32(
			hw,
			RNPM_DMA_REG_RX_DESC_BUF_HEAD(adapter->rx_ring[i]->rnpm_queue_idx));
		if (unlikely(head >= adapter->rx_ring[i]->count)) {
			dbg("[%s] Ring %d head large than count",
				adapter->netdev->name,
				adapter->rx_ring[i]->rnpm_queue_idx);
			adapter->rx_ring[i]->ring_flags |=
				RNPM_RING_FLAG_DELAY_SETUP_RX_LEN;
			adapter->rx_ring[i]->reset_count = adapter->rx_ring[i]->count;
			adapter->rx_ring[i]->count = head + 1;
		}
		err = rnpm_setup_rx_resources(adapter->rx_ring[i], adapter);
		if (!err)
			continue;

		e_err(probe, "Allocation for Rx Queue %u failed\n", i);
		goto err_setup_rx;
	}

	return 0;
err_setup_rx:
	/* rewind the index freeing the rings as we go */
	while (i--)
		rnpm_free_rx_resources(adapter->rx_ring[i]);
	return err;
}

/**
 * rnpm_free_tx_resources - Free Tx Resources per Queue
 * @tx_ring: Tx descriptor ring for a specific queue
 *
 * Free all transmit software resources
 **/
void rnpm_free_tx_resources(struct rnpm_ring *tx_ring)
{
	BUG_ON(tx_ring == NULL);

	rnpm_clean_tx_ring(tx_ring);

	vfree(tx_ring->tx_buffer_info);
	tx_ring->tx_buffer_info = NULL;

	/* if not set, then don't free */
	if (!tx_ring->desc)
		return;

	dma_free_coherent(tx_ring->dev, tx_ring->size, tx_ring->desc, tx_ring->dma);

	tx_ring->desc = NULL;
}

/**
 * rnpm_free_all_tx_resources - Free Tx Resources for All Queues
 * @adapter: board private structure
 *
 * Free all transmit software resources
 **/
static void rnpm_free_all_tx_resources(struct rnpm_adapter *adapter)
{
	int i;

	for (i = 0; i < (adapter->num_tx_queues); i++)
		rnpm_free_tx_resources(adapter->tx_ring[i]);
}

/**
 * rnpm_free_rx_resources - Free Rx Resources
 * @rx_ring: ring to clean the resources from
 *
 * Free all receive software resources
 **/
void rnpm_free_rx_resources(struct rnpm_ring *rx_ring)
{
	BUG_ON(rx_ring == NULL);

	rnpm_clean_rx_ring(rx_ring);

	vfree(rx_ring->rx_buffer_info);
	rx_ring->rx_buffer_info = NULL;

	/* if not set, then don't free */
	if (!rx_ring->desc)
		return;

	dma_free_coherent(rx_ring->dev, rx_ring->size, rx_ring->desc, rx_ring->dma);

	rx_ring->desc = NULL;
}

/**
 * rnpm_free_all_rx_resources - Free Rx Resources for All Queues
 * @adapter: board private structure
 *
 * Free all receive software resources
 **/
static void rnpm_free_all_rx_resources(struct rnpm_adapter *adapter)
{
	int i;

	for (i = 0; i < (adapter->num_rx_queues); i++)
		// if (adapter->rx_ring[i]->desc)
		rnpm_free_rx_resources(adapter->rx_ring[i]);
}

/**
 * rnpm_change_mtu - Change the Maximum Transfer Unit
 * @netdev: network interface device structure
 * @new_mtu: new value for maximum frame size
 *
 * Returns 0 on success, negative on failure
 **/
static int rnpm_change_mtu(struct net_device *netdev, int new_mtu)
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	struct rnpm_pf_adapter *pf_adapter = adapter->pf_adapter;

	int max_frame = new_mtu + ETH_HLEN + 2 * ETH_FCS_LEN;

	/* MTU < 68 is an error and causes problems on some kernels */
	if ((new_mtu < RNPM_MIN_MTU) || (max_frame > RNPM_MAX_JUMBO_FRAME_SIZE))
		return -EINVAL;

	e_info(probe, "changing MTU from %d to %d\n", netdev->mtu, new_mtu);

	/* must set new MTU before calling down or up */
	netdev->mtu = new_mtu;

	pf_adapter->flags |= RNPM_PF_SET_MTU;

	if (netif_running(netdev))
		rnpm_reinit_locked(adapter);

	return 0;
}

#ifdef HAVE_RHEL7_NETDEV_OPS_EXT_NDO_TX_MAXRATE
#define RNPM_TX_MAXRATE_DEFINE
#else
#ifndef NO_TX_MAXRATE
#define RNPM_TX_MAXRATE_DEFINE
#endif
#endif

#ifdef RNPM_TX_MAXRATE_DEFINE
/**
 * rnpm_tx_maxrate - callback to set the maximum per-queue bitrate
 * @netdev: network interface device structure
 * @queue_index: Tx queue to set
 * @maxrate: desired maximum transmit bitrate Mbps
 **/
static int
rnpm_tx_maxrate(struct net_device *netdev, int queue_index, u32 maxrate)
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	struct rnpm_ring *tx_ring = adapter->tx_ring[queue_index];
	u64 real_rate = 0;
	// record this flags
	adapter->max_rate[queue_index] = maxrate;
	// adapter->flags2 |= RNPM_FLAG2_TX_RATE_SETUP;
	if (!maxrate)
		return rnpm_setup_tx_maxrate(adapter->hw.hw_addr,
									 tx_ring,
									 0,
									 adapter->hw.usecstocount * 1000000);
	/* we need turn it to bytes/s */
	real_rate = (maxrate * 1024 * 1024) / 8;
	rnpm_setup_tx_maxrate(adapter->hw.hw_addr,
						  tx_ring,
						  real_rate,
						  adapter->hw.usecstocount * 1000000);

	return 0;
}
#endif
/**
 * rnpm_open - Called when a network interface is made active
 * @netdev: network interface device structure
 *
 * Returns 0 on success, negative value on failure
 *
 * The open entry point is called when a network interface is made
 * active by the system (IFF_UP).  At this point all resources needed
 * for transmit and receive operations are allocated, the interrupt
 * handler is registered with the OS, the watchdog timer is started,
 * and the stack is notified that the interface is ready.
 **/
int rnpm_open(struct net_device *netdev)
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	int err;

	DPRINTK(IFUP, INFO, "ifup\n");

	// rnpm_mbx_ifup_down(&adapter->hw, 1);

	/* disallow open during test */
	if (test_bit(__RNPM_TESTING, &adapter->state))
		return -EBUSY;

	netif_carrier_off(netdev);
	/* allocate transmit descriptors */
	err = rnpm_setup_all_tx_resources(adapter);
	if (err)
		goto err_setup_tx;
	/* allocate receive descriptors */
	err = rnpm_setup_all_rx_resources(adapter);
	if (err)
		goto err_setup_rx;
	rnpm_configure(adapter);
	err = rnpm_request_irq(adapter);
	if (err)
		goto err_req_irq;
	/* Notify the stack of the actual queue counts. */
	err = netif_set_real_num_tx_queues(netdev, adapter->num_tx_queues);
	if (err)
		goto err_set_queues;
	err = netif_set_real_num_rx_queues(netdev, adapter->num_rx_queues);
	if (err)
		goto err_set_queues;
#ifndef NO_PTP
	if (module_enable_ptp)
		rnpm_ptp_register(adapter);
#endif
	rnpm_up_complete(adapter);

	return 0;

err_set_queues:
	rnpm_free_irq(adapter);
err_req_irq:
	rnpm_free_all_rx_resources(adapter);
err_setup_rx:
	rnpm_free_all_tx_resources(adapter);
err_setup_tx:
	rnpm_mbx_ifup_down(&adapter->hw, MBX_IFDOWN);
	rnpm_reset(adapter);

	return err;
}

/**
 * rnpm_close - Disables a network interface
 * @netdev: network interface device structure
 *
 * Returns 0, this is not allowed to fail
 *
 * The close entry point is called when an interface is de-activated
 * by the OS.  The hardware is still under the drivers control, but
 * needs to be disabled.  A global MAC reset is issued to stop the
 * hardware, and all transmit and receive resources are freed.
 **/
int rnpm_close(struct net_device *netdev)
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);

	DPRINTK(IFDOWN, INFO, "ifdown\n");
	rnpm_logd(LOG_FUNC_ENTER, "%s %s\n", __func__, adapter->name);

#ifdef DISABLE_RX_IRQ
	adapter->quit_poll_thread = true;
#endif
	/* should clean adapter->ptp_tx_skb */
	if (adapter->ptp_tx_skb) {
		dev_kfree_skb_any(adapter->ptp_tx_skb);
		adapter->ptp_tx_skb = NULL;
		adapter->tx_hwtstamp_timeouts++;
		netdev_warn(adapter->netdev, "clearing Tx timestamp hang\n");
	}
	// **

#ifndef NO_PTP
	if (module_enable_ptp)
		rnpm_ptp_unregister(adapter);
#endif

	rnpm_down(adapter);

	rnpm_free_irq(adapter);

	rnpm_fdir_filter_exit(adapter);

	rnpm_free_all_tx_resources(adapter);
	rnpm_free_all_rx_resources(adapter);
	rnpm_mbx_ifup_down(&adapter->hw, MBX_IFDOWN);
	// rnpm_release_hw_control(adapter);

	return 0;
}

/**
 * rnpm_update_stats - Update the board statistics counters.
 * @adapter: board private structure
 **/
void rnpm_update_stats(struct rnpm_adapter *adapter)
{
	struct net_device_stats *net_stats = &adapter->netdev->stats;
	struct rnpm_hw *hw = &adapter->hw;
	struct rnpm_hw_stats *hw_stats = &adapter->hw_stats;

	int i, port = adapter->port;
	struct rnpm_ring *ring;
	u64 hw_csum_rx_error = 0;
	u64 hw_csum_rx_good = 0;

	net_stats->tx_packets = 0;
	net_stats->tx_bytes = 0;
	net_stats->rx_packets = 0;
	net_stats->rx_bytes = 0;
	net_stats->rx_errors = 0;

	hw_stats->vlan_strip_cnt = 0;
	hw_stats->vlan_add_cnt = 0;

	if (test_bit(__RNPM_DOWN, &adapter->state) ||
		test_bit(__RNPM_RESETTING, &adapter->state))
		return;

	for (i = 0; i < adapter->num_q_vectors; i++) {
		rnpm_for_each_ring(ring, adapter->q_vector[i]->rx)
		{
			net_stats->rx_packets += ring->stats.packets;
			net_stats->rx_bytes += ring->stats.bytes;
			hw_csum_rx_error += ring->rx_stats.csum_err;
			hw_csum_rx_good += ring->rx_stats.csum_good;
			hw_stats->vlan_strip_cnt += ring->rx_stats.vlan_remove;
		}
		rnpm_for_each_ring(ring, adapter->q_vector[i]->tx)
		{
			net_stats->tx_packets += ring->stats.packets;
			net_stats->tx_bytes += ring->stats.bytes;
			hw_stats->vlan_add_cnt += ring->tx_stats.vlan_add;
		}
	}

	switch (hw->mode) {
		case MODE_NIC_MODE_1PORT_40G:
		case MODE_NIC_MODE_1PORT:
			hw_stats->dma_to_eth =
				rd32(hw, RNPM_DMA_STATS_DMA_TO_DMA_CHANNEL_0) +
				rd32(hw, RNPM_DMA_STATS_DMA_TO_DMA_CHANNEL_1) +
				rd32(hw, RNPM_DMA_STATS_DMA_TO_DMA_CHANNEL_2) +
				rd32(hw, RNPM_DMA_STATS_DMA_TO_DMA_CHANNEL_3);
			break;
		case MODE_NIC_MODE_2PORT:
			hw_stats->dma_to_eth = 0;
			for (i = port * 2; i < (port + 1) * 2; i++) {
				hw_stats->dma_to_eth +=
					rd32(hw, RNPM_DMA_STATS_DMA_TO_DMA_CHANNEL(i));
			}
			break;
		case MODE_NIC_MODE_4PORT:
			hw_stats->dma_to_eth =
				rd32(hw, RNPM_DMA_STATS_DMA_TO_DMA_CHANNEL(port));
			break;
	}

	/* only has unique reg */
	hw_stats->dma_to_switch = rd32(hw, RNPM_DMA_STATS_DMA_TO_SWITCH);
	hw_stats->mac_to_dma = rd32(hw, RNPM_DMA_STATS_MAC_TO_DMA);

	net_stats->rx_crc_errors = rnpm_recalculate_err_pkts(
		rd32(hw, RNPM_RXTRANS_CRC_ERR_PKTS(port)), hw->err_pkts_init.crc[port]);

	net_stats->rx_errors =
		rnpm_recalculate_err_pkts(rd32(hw, RNPM_RXTRANS_WDT_ERR_PKTS(port)),
								  hw->err_pkts_init.wdt[port]) +
		rnpm_recalculate_err_pkts(rd32(hw, RNPM_RXTRANS_CODE_ERR_PKTS(port)),
								  hw->err_pkts_init.code[port]) +
		rnpm_recalculate_err_pkts(rd32(hw, RNPM_RXTRANS_CRC_ERR_PKTS(port)),
								  hw->err_pkts_init.crc[port]) +
		rnpm_recalculate_err_pkts(rd32(hw, RNPM_RXTRANS_SLEN_ERR_PKTS(port)),
								  hw->err_pkts_init.slen[port]) +
		rnpm_recalculate_err_pkts(rd32(hw, RNPM_RXTRANS_GLEN_ERR_PKTS(port)),
								  hw->err_pkts_init.glen[port]) +
		rnpm_recalculate_err_pkts(rd32(hw, RNPM_RXTRANS_IPH_ERR_PKTS(port)),
								  hw->err_pkts_init.iph[port]) +
		rnpm_recalculate_err_pkts(rd32(hw, RNPM_RXTRANS_LEN_ERR_PKTS(port)),
								  hw->err_pkts_init.len[port]) +
		rnpm_recalculate_err_pkts(rd32(hw, RNPM_RXTRANS_CUT_ERR_PKTS(port)),
								  hw->err_pkts_init.cut[port]) +
		hw_csum_rx_error;

	/* only has unique reg */
	//=== drop ===
	// hw_stats->invalid_droped_packets = rd32(hw, RNPM_ETH_INVALID_DROP_PKTS);
	// hw_stats->filter_dropped_packets = rd32(hw, RNPM_ETH_FILTER_DROP_PKTS);
	// hw_stats->host_l2_match_drop = rd32(hw, RNPM_ETH_HOST_L2_DROP_PKTS);
	// hw_stats->redir_input_match_drop =
	//	rd32(hw, RNPM_ETH_REDIR_INPUT_MATCH_DROP_PKTS);
	// hw_stats->redir_etype_match_drop = rd32(hw, RNPM_ETH_ETYPE_DROP_PKTS);
	// hw_stats->redir_tcp_syn_match_drop = rd32(hw,
	// RNPM_ETH_TCP_SYN_DROP_PKTS); hw_stats->redir_tuple5_match_drop = 	rd32(hw,
	//RNPM_ETH_REDIR_TUPLE5_DROP_PKTS); hw_stats->redir_tcam_match_drop =
	// rd32(hw, RNPM_ETH_REDIR_TCAM_DROP_PKTS); hw_stats->bmc_dropped_packets =
	// rd32(hw, RNPM_ETH_DECAP_BMC_DROP_NUM); hw_stats->switch_dropped_packets =
	// rd32(hw, RNPM_ETH_DECAP_SWITCH_DROP_NUM);

	adapter->hw_csum_rx_error = hw_csum_rx_error;
	adapter->hw_csum_rx_good = hw_csum_rx_good;

	hw_stats->mac_rx_broadcast = rd32(hw, RNPM_MAC_STATS_BROADCAST_LOW(port));
	hw_stats->mac_rx_broadcast +=
		((u64)rd32(hw, RNPM_MAC_STATS_BROADCAST_HIGH(port)) << 32);

	// maybe no use
	hw_stats->mac_rx_multicast = rd32(hw, RNPM_MAC_STATS_MULTICAST_LOW(port));
	hw_stats->mac_rx_multicast +=
		((u64)rd32(hw, RNPM_MAC_STATS_MULTICAST_HIGH(port)) << 32);

	/* store to net_stats */
	net_stats->multicast = hw_stats->mac_rx_multicast;
}

#if 0

/**
 * rnpm_fdir_reinit_subtask - worker thread to reinit FDIR filter table
 * @adapter: pointer to the device adapter structure
 **/
static void rnpm_fdir_reinit_subtask(struct rnpm_adapter *adapter)
{
	struct rnpm_hw *hw = &adapter->hw;
	int i;

	if (!(adapter->flags2 & RNPM_FLAG2_FDIR_REQUIRES_REINIT))
		return;

	adapter->flags2 &= ~RNPM_FLAG2_FDIR_REQUIRES_REINIT;

	/* if interface is down do nothing */
	if (test_bit(__RNPM_DOWN, &adapter->state))
		return;

	/* do nothing if we are not using signature filters */
	if (!(adapter->flags & RNPM_FLAG_FDIR_HASH_CAPABLE))
		return;

	adapter->fdir_overflow++;

	if (rnpm_reinit_fdir_tables_n10(hw) == 0) {
		for (i = 0; i < adapter->num_tx_queues; i++)
			set_bit(__RNPM_TX_FDIR_INIT_DONE,
					&(adapter->tx_ring[i]->state));
		/* re-enable flow director interrupts */
		RNPM_WRITE_REG(hw, RNPM_EIMS, RNPM_EIMS_FLOW_DIR);
	} else {
		e_err(probe, "failed to finish FDIR re-initialization, "
		      "ignored adding FDIR ATR filters\n");
	}
}

#endif
/**
 * rnpm_check_hang_subtask - check for hung queues and dropped interrupts
 * @adapter: pointer to the device adapter structure
 *
 * This function serves two purposes.  First it strobes the interrupt lines
 * in order to make certain interrupts are occurring.  Secondly it sets the
 * bits needed to check for TX hangs.  As a result we should immediately
 * determine if a hang has occurred.
 */
static void rnpm_check_hang_subtask(struct rnpm_adapter *adapter)
{

	// struct rnpm_hw *hw = &adapter->hw;
	// u64 eics = 0;
	int i;
	struct rnpm_ring *tx_ring;
	u64 tx_next_to_clean_old;
	u64 tx_next_to_clean;
	u64 tx_next_to_use;
	struct rnpm_ring *rx_ring;
	u64 rx_next_to_clean_old;
	u64 rx_next_to_clean;
	// u64 rx_next_to_use;
	union rnpm_rx_desc *rx_desc;

	/* If we're down or resetting, just bail */
	if (test_bit(__RNPM_DOWN, &adapter->state) ||
		test_bit(__RNPM_RESETTING, &adapter->state))
		return;

	/* Force detection of hung controller */
	if (netif_carrier_ok(adapter->netdev)) {
		for (i = 0; i < adapter->num_tx_queues; i++)
			set_check_for_tx_hang(adapter->tx_ring[i]);
	}

	//	if (!(adapter->flags & RNPM_FLAG_MSIX_ENABLED)) {
	//		/*
	//		 * for legacy and MSI interrupts don't set any bits
	//		 * that are enabled for EIAM, because this operation
	//		 * would set *both* EIMS and EICS for any bit in EIAM
	//		 */
	//		RNPM_WRITE_REG(hw, RNPM_EICS,
	//			(RNPM_EICS_TCP_TIMER | RNPM_EICS_OTHER));
	//	} else {
	//		/* get one bit for every active tx/rx interrupt vector */
	//		for (i = 0; i < adapter->num_q_vectors; i++) {
	//			struct rnpm_q_vector *qv = adapter->q_vector[i];
	//			if (qv->rx.ring || qv->tx.ring)
	//				eics |= ((u64)1 << i);
	//		}
	//	}

	/* Cause software interrupt to ensure rings are cleaned */
	// rnpm_irq_rearm_queues(adapter, eics);
	//  todo fix irq?
	//  check if we lost tx irq ?
	for (i = 0; i < adapter->num_tx_queues; i++) {
		tx_ring = adapter->tx_ring[i];
		/* get the last next_to_clean */
		tx_next_to_clean_old = tx_ring->tx_stats.tx_next_to_clean;
		tx_next_to_clean = tx_ring->next_to_clean;
		tx_next_to_use = tx_ring->next_to_use;

		/* if we have tx desc to clean */
		if (tx_next_to_use != tx_next_to_clean) {

			if (tx_next_to_clean == tx_next_to_clean_old) {
				tx_ring->tx_stats.tx_equal_count++;
				if (tx_ring->tx_stats.tx_equal_count > 2) {
					/* maybe not so good */
					struct rnpm_q_vector *q_vector = tx_ring->q_vector;

					/* stats */
					// printk("maybe tx irq miss happen!!! \n");
					if (q_vector->rx.ring || q_vector->tx.ring) {
						rnpm_irq_disable_queues(q_vector);
						napi_schedule_irqoff(&q_vector->napi);
					}

					tx_ring->tx_stats.tx_irq_miss++;
					tx_ring->tx_stats.tx_equal_count = 0;
				}
			} else {
				tx_ring->tx_stats.tx_equal_count = 0;
			}
			/* update */
			/* record this next_to_clean */
			tx_ring->tx_stats.tx_next_to_clean = tx_next_to_clean;
		} else {
			/* clean record to -1 */
			tx_ring->tx_stats.tx_next_to_clean = -1;
		}
	}

	// check if we lost rx irq
	for (i = 0; i < adapter->num_rx_queues; i++) {
		rx_ring = adapter->rx_ring[i];
		/* get the last next_to_clean */
		rx_next_to_clean_old = rx_ring->rx_stats.rx_next_to_clean;
		/* get the now clean */
		rx_next_to_clean = rx_ring->next_to_clean;

		// if rx clean stopped
		// maybe not so good
		if (rx_next_to_clean == rx_next_to_clean_old) {
			rx_ring->rx_stats.rx_equal_count++;

			if ((rx_ring->rx_stats.rx_equal_count > 2) &&
				(rx_ring->rx_stats.rx_equal_count < 5)) {
				// check if dd in the clean rx desc
				rx_desc = RNPM_RX_DESC(rx_ring, rx_ring->next_to_clean);
				if (rnpm_test_staterr(rx_desc, RNPM_RXD_STAT_DD)) {
					int size;
					struct rnpm_q_vector *q_vector = rx_ring->q_vector;

					size = le16_to_cpu(rx_desc->wb.len);
					if (size) {
						rx_ring->rx_stats.rx_irq_miss++;
						if (q_vector->rx.ring || q_vector->tx.ring) {
							rnpm_irq_disable_queues(q_vector);
							napi_schedule_irqoff(&q_vector->napi);
						}
					}
				}
				// rx_ring->rx_stats.rx_equal_count = 0;
			}
			if (rx_ring->rx_stats.rx_equal_count > 1000)
				rx_ring->rx_stats.rx_equal_count = 0;
		} else {
			rx_ring->rx_stats.rx_equal_count = 0;
		}
		// update new clean
		rx_ring->rx_stats.rx_next_to_clean = rx_next_to_clean;
	}
}

/**
 * rnpm_watchdog_update_link - update the link status
 * @adapter: pointer to the device adapter structure
 * @link_speed: pointer to a u32 to store the link_speed
 **/
static void rnpm_watchdog_update_link(struct rnpm_adapter *adapter)
{
	struct rnpm_hw *hw = &adapter->hw;
	u32 link_speed = adapter->link_speed;
	bool link_up = adapter->link_up;

	if (!(adapter->flags & RNPM_FLAG_NEED_LINK_UPDATE))
		return;

	if (hw->mac.ops.check_link) {
		hw->mac.ops.check_link(hw, &link_speed, &link_up, false);
	} else {
		/* always assume link is up, if no check link function */
		link_speed = RNPM_LINK_SPEED_10GB_FULL;
		link_up = true;
	}

	if (link_up ||
		time_after(jiffies,
				   (adapter->link_check_timeout + RNPM_TRY_LINK_TIMEOUT))) {
		adapter->flags &= ~RNPM_FLAG_NEED_LINK_UPDATE;
	}

	adapter->link_up = link_up;
	adapter->link_speed = link_speed;
}

static void rnpm_update_default_up(struct rnpm_adapter *adapter)
{
#ifdef CONFIG_RNPM_DCB
	struct net_device *netdev = adapter->netdev;
	struct dcb_app app = {
		.selector = IEEE_8021QAZ_APP_SEL_ETHERTYPE,
		.protocol = 0,
	};
	u8 up = 0;

	if (adapter->dcbx_cap & DCB_CAP_DCBX_VER_IEEE)
		up = dcb_ieee_getapp_mask(netdev, &app);

	adapter->default_up = (up > 1) ? (ffs(up) - 1) : 0;
#endif
}

/**
 * rnpm_watchdog_link_is_up - update netif_carrier status and
 *                             print link up message
 * @adapter: pointer to the device adapter structure
 **/
static void rnpm_watchdog_link_is_up(struct rnpm_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	struct rnpm_hw *hw = &adapter->hw;
	u32 link_speed = adapter->link_speed;
	bool flow_rx = true, flow_tx = true;

	rnpm_link_stat_mark(hw, hw->nr_lane, 1);

	/* only continue if link was previously down */
	if (netif_carrier_ok(netdev))
		return;

	adapter->flags2 &= ~RNPM_FLAG2_SEARCH_FOR_SFP;

	e_info(drv,
		   "NIC Link is Up %s, Flow Control: %s\n",
		   (link_speed == RNPM_LINK_SPEED_10GB_FULL
				? "10 Gbps"
				: (link_speed == RNPM_LINK_SPEED_1GB_FULL
					   ? "1 Gbps"
					   : (link_speed == RNPM_LINK_SPEED_100_FULL
							  ? "100 Mbps"
							  : (link_speed == RNPM_LINK_SPEED_10_FULL
									 ? "10 Mbps"
									 : "unknown speed")))),
		   ((flow_rx && flow_tx)
				? "RX/TX"
				: (flow_rx ? "RX" : (flow_tx ? "TX" : "None"))));

	netif_carrier_on(netdev);

	netif_tx_wake_all_queues(netdev);
	// rnpm_check_vf_rate_limit(adapter);

	/* update the default user priority for VFs */
	rnpm_update_default_up(adapter);

	control_mac_rx(adapter, true);
	/* ping all the active vfs to let them know link has changed */
	// rnpm_ping_all_vfs(adapter);
}

/**
 * rnpm_watchdog_link_is_down - update netif_carrier status and
 *                               print link down message
 * @adapter: pointer to the adapter structure
 **/
static void rnpm_watchdog_link_is_down(struct rnpm_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	struct rnpm_hw *hw = &adapter->hw;

	adapter->link_up = false;
	adapter->link_speed = 0;

	rnpm_link_stat_mark(hw, hw->nr_lane, 0);
	control_mac_rx(adapter, false);
	/* only continue if link was up previously */
	if (!netif_carrier_ok(netdev))
		return;

	/* poll for SFP+ cable when link is down */
	if (rnpm_is_sfp(hw))
		adapter->flags2 |= RNPM_FLAG2_SEARCH_FOR_SFP;

	e_info(drv, "NIC Link is Down\n");
	netif_carrier_off(netdev);

	netif_tx_stop_all_queues(netdev);
	/* ping all the active vfs to let them know link has changed */
	// rnpm_ping_all_vfs(adapter);
}

/**
 * rnpm_watchdog_flush_tx - flush queues on link down
 * @adapter: pointer to the device adapter structure
 **/
__maybe_unused static void rnpm_watchdog_flush_tx(struct rnpm_adapter *adapter)
{
	int i;
	int some_tx_pending = 0;

	if (!netif_carrier_ok(adapter->netdev)) {
		for (i = 0; i < adapter->num_tx_queues; i++) {
			struct rnpm_ring *tx_ring = adapter->tx_ring[i];
			if (tx_ring->next_to_use != tx_ring->next_to_clean) {
				some_tx_pending = 1;
				break;
			}
		}

		if (some_tx_pending) {
			/* We've lost link, so the controller stops DMA,
			 * but we've got queued Tx work that's never going
			 * to get done, so reset controller to flush Tx.
			 * (Do the reset outside of interrupt context).
			 */
			rnpm_dbg("initiating reset to clear Tx work after link loss\n");
			e_warn(drv, "initiating reset to clear Tx work after link loss\n");
			adapter->flags2 |= RNPM_FLAG2_RESET_REQUESTED;
		}
	}
}

/**
 * rnpm_watchdog_subtask - check and bring link up
 * @adapter: pointer to the device adapter structure
 **/
static void rnpm_watchdog_subtask(struct rnpm_adapter *adapter)
{
	/* if interface is down do nothing */
	if (test_bit(__RNPM_DOWN, &adapter->state) ||
		test_bit(__RNPM_RESETTING, &adapter->state))
		return;

	rnpm_watchdog_update_link(adapter);

	if ((adapter->link_up)) {
		rnpm_watchdog_link_is_up(adapter);
	} else {
		rnpm_watchdog_link_is_down(adapter);
	}

	rnpm_update_stats(adapter);
	//rnpm_watchdog_flush_tx(adapter);
}

/**
 * rnpm_sfp_detection_subtask - poll for SFP+ cable
 * @adapter: the rnpm adapter structure
 **/
static void rnpm_sfp_detection_subtask(struct rnpm_adapter *adapter)
{
#if 0
	struct rnpm_hw *hw = &adapter->hw;
	s32 err;

	/* not searching for SFP so there is nothing to do here */
	if (!(adapter->flags2 & RNPM_FLAG2_SEARCH_FOR_SFP) &&
	    !(adapter->flags2 & RNPM_FLAG2_SFP_NEEDS_RESET))
		return;

	/* concurent i2c reads are not supported */
	if (test_bit(__RNPM_READ_I2C, &adapter->state))
		return;

	/* someone else is in init, wait until next service event */
	if (test_and_set_bit(__RNPM_IN_SFP_INIT, &adapter->state))
		return;

	err = hw->phy.ops.identify_sfp(hw);
	if (err == RNPM_ERR_SFP_NOT_SUPPORTED)
		goto sfp_out;

	if (err == RNPM_ERR_SFP_NOT_PRESENT) {
		/* If no cable is present, then we need to reset
		 * the next time we find a good cable. */
		adapter->flags2 |= RNPM_FLAG2_SFP_NEEDS_RESET;
	}

	/* exit on error */
	if (err)
		goto sfp_out;

	/* exit if reset not needed */
	if (!(adapter->flags2 & RNPM_FLAG2_SFP_NEEDS_RESET))
		goto sfp_out;

	adapter->flags2 &= ~RNPM_FLAG2_SFP_NEEDS_RESET;

	/*
	 * A module may be identified correctly, but the EEPROM may not have
	 * support for that module.  setup_sfp() will fail in that case, so
	 * we should not allow that module to load.
	 */
	if (hw->mac.type == rnpm_mac_82598EB)
		err = hw->phy.ops.reset(hw);
	else
		err = hw->mac.ops.setup_sfp(hw);

	if (err == RNPM_ERR_SFP_NOT_SUPPORTED)
		goto sfp_out;

	adapter->flags |= RNPM_FLAG_NEED_LINK_CONFIG;
	e_info(probe, "detected SFP+: %d\n", hw->phy.sfp_type);

sfp_out:
	clear_bit(__RNPM_IN_SFP_INIT, &adapter->state);

	if ((err == RNPM_ERR_SFP_NOT_SUPPORTED) &&
	    (adapter->netdev->reg_state == NETREG_REGISTERED)) {
		e_dev_err("failed to initialize because an unsupported "
			  "SFP+ module type was detected.\n");
		e_dev_err("Reload the driver after installing a "
			  "supported module.\n");
		unregister_netdev(adapter->netdev);
	}
#endif
}

/**
 * rnpm_sfp_link_config_subtask - set up link SFP after module install
 * @adapter: the rnpm adapter structure
 **/
static void rnpm_sfp_link_config_subtask(struct rnpm_adapter *adapter)
{
	// struct rnpm_hw *hw = &adapter->hw;
	// u32 speed;
	// bool autoneg = false;

	if (!(adapter->flags & RNPM_FLAG_NEED_LINK_CONFIG))
		return;

#if 0
	/* someone else is in init, wait until next service event */
	if (test_and_set_bit(__RNPM_IN_SFP_INIT, &adapter->state))
		return;

	adapter->flags &= ~RNPM_FLAG_NEED_LINK_CONFIG;

	speed = hw->phy.autoneg_advertised;
	if ((!speed) && (hw->mac.ops.get_link_capabilities))
		hw->mac.ops.get_link_capabilities(hw, &speed, &autoneg);
	if (hw->mac.ops.setup_link)
		hw->mac.ops.setup_link(hw, speed, true);

	adapter->flags |= RNPM_FLAG_NEED_LINK_UPDATE;
	adapter->link_check_timeout = jiffies;
	clear_bit(__RNPM_IN_SFP_INIT, &adapter->state);
#endif
}

#ifdef CONFIG_PCI_IOV
__maybe_unused static void rnpm_check_for_bad_vf(struct rnpm_adapter *adapter)
{
#if 0
	int vf;
	struct rnpm_hw *hw = &adapter->hw;
	struct net_device *netdev = adapter->netdev;
	u32 gpc;
	u32 ciaa, ciad;

	gpc = RNPM_READ_REG(hw, RNPM_TXDGPC);
	if (gpc) /* If incrementing then no need for the check below */
		return;
	/*
	 * Check to see if a bad DMA write target from an errant or
	 * malicious VF has caused a PCIe error.  If so then we can
	 * issue a VFLR to the offending VF(s) and then resume without
	 * requesting a full slot reset.
	 */

	for (vf = 0; vf < adapter->num_vfs; vf++) {
		ciaa = (vf << 16) | 0x80000000;
		/* 32 bit read so align, we really want status at offset 6 */
		ciaa |= PCI_COMMAND;
		RNPM_WRITE_REG(hw, RNPM_CIAA_n10, ciaa);
		ciad = RNPM_READ_REG(hw, RNPM_CIAD_n10);
		ciaa &= 0x7FFFFFFF;
		/* disable debug mode asap after reading data */
		RNPM_WRITE_REG(hw, RNPM_CIAA_n10, ciaa);
		/* Get the upper 16 bits which will be the PCI status reg */
		ciad >>= 16;
		if (ciad & PCI_STATUS_REC_MASTER_ABORT) {
			netdev_err(netdev, "VF %d Hung DMA\n", vf);
			/* Issue VFLR */
			ciaa = (vf << 16) | 0x80000000;
			ciaa |= 0xA8;
			RNPM_WRITE_REG(hw, RNPM_CIAA_n10, ciaa);
			ciad = 0x00008000;  /* VFLR */
			RNPM_WRITE_REG(hw, RNPM_CIAD_n10, ciad);
			ciaa &= 0x7FFFFFFF;
			RNPM_WRITE_REG(hw, RNPM_CIAA_n10, ciaa);
		}
	}
#endif
}

#endif
/**
 * rnpm_pf_service_timer - Timer Call-back
 * @data: pointer to adapter cast into an unsigned long
 **/
void rnpm_pf_service_timer(struct timer_list *t)
{
	struct rnpm_pf_adapter *pf_adapter = from_timer(pf_adapter, t, service_timer);
	unsigned long next_event_offset;

	// we check 2s
	next_event_offset = HZ * 2;
	/* Reset the timer */
	mod_timer(&pf_adapter->service_timer, next_event_offset + jiffies);
	rnpm_pf_service_event_schedule(pf_adapter);
}

/**
 * rnpm_service_timer - Timer Call-back
 * @data: pointer to adapter cast into an unsigned long
 **/
void rnpm_service_timer(struct timer_list *t)
{
	struct rnpm_adapter *adapter = from_timer(adapter, t, service_timer);
	unsigned long next_event_offset;
	bool ready = true;

#ifdef RNPM_DISABLE_IRQ
	rnpm_mbx_get_link(&adapter->hw);
#endif
	/* poll faster when waiting for link */
	if (adapter->flags & RNPM_FLAG_NEED_LINK_UPDATE)
		next_event_offset = HZ / 10;
	else
		next_event_offset = HZ * 2;
#if 0
#ifdef CONFIG_PCI_IOV
	/*
	 * don't bother with SR-IOV VF DMA hang check if there are
	 * no VFs or the link is down
	 */
	if (!adapter->num_vfs ||
	    (adapter->flags & RNPM_FLAG_NEED_LINK_UPDATE))
		goto normal_timer_service;

	/* If we have VFs allocated then we must check for DMA hangs */
	rnpm_check_for_bad_vf(adapter);
	next_event_offset = HZ / 50;
	adapter->timer_event_accumulator++;

	if (adapter->timer_event_accumulator >= 100)
		adapter->timer_event_accumulator = 0;
	else
		ready = false;

normal_timer_service:
#endif
#endif
	/* Reset the timer */
	mod_timer(&adapter->service_timer, next_event_offset + jiffies);

	if (ready)
		rnpm_service_event_schedule(adapter);
}

static void rnpm_fix_dma_tx_status(struct rnpm_pf_adapter *pf_adapter)
{
	int i;

	// set all tx start to 1
	for (i = 0; i < 128; i++)
		wr32(pf_adapter, RNPM_DMA_TX_START(i), 1);
}

static int rnpm_reset_pf(struct rnpm_pf_adapter *pf_adapter)
{
	int times = 0;
	int i = 0;
	u32 status = 0;
	// unsigned long flags;

	wr32(pf_adapter, RNPM_DMA_AXI_EN, 0);
#define TIMEOUT_COUNT (1000)
	/* wait axi ready */
	while ((status != 0xf) && (times < TIMEOUT_COUNT)) {
		status = rd32(pf_adapter, RNPM_DMA_AXI_STAT);
		usleep_range(4000, 8000);
		times++;
		// rnpm_dbg("wait axi ready\n");
	}

	if (times >= TIMEOUT_COUNT) {
		rnpm_warn("wait axi ready timeout\n");
		return -1;
	}
	wr32(pf_adapter, RNPM_TOP_NIC_REST_N, NIC_RESET);
	/*
	 * we need this
	 */
	wmb();

	wr32(pf_adapter, RNPM_TOP_NIC_REST_N, ~NIC_RESET);

#ifdef NO_MBX_VERSION
#define TSRN10_REG_DEBUG_VALUE (0x1a2b3c4d)

	spin_lock_irqsave(&pf_adapter->dummy_setup_lock, flags);
	wr32(pf_adapter, RNPM_DMA_DUMY, TSRN10_REG_DEBUG_VALUE);
	times = 0;
	status = 0;
	while ((status != TSRN10_REG_DEBUG_VALUE + 1) && (times < TIMEOUT_COUNT)) {
		status = rd32(pf_adapter, RNPM_DMA_DUMY);
		times++;
		usleep_range(4000, 8000);
		// rnpm_dbg("wait firmware reset card %x\n", status);
	}
	spin_unlock_irqrestore(&pf_adapter->dummy_setup_lock, flags);

	if (times >= TIMEOUT_COUNT) {
		rnpm_dbg("wait firmware reset card timeout\n");
		return -1;
	}
#else
	rnpm_mbx_fw_reset_phy(&pf_adapter->hw);
#endif

	/* global setup here */
	wr32(pf_adapter, RNPM_TOP_ETH_BUG_40G_PATCH, 1);
	wr32(pf_adapter, RNPM_ETH_TUNNEL_MOD, 1);

	/* set all rx drop */
	for (i = 0; i < 4; i++)
		wr32(pf_adapter, RNPM_ETH_RX_PROGFULL_THRESH_PORT(i), DROP_ALL_THRESH);

	// rnpm_dbg("reset_finish\n");
	/* setup rss key */
	rnpm_init_rss_key(pf_adapter);
	/* tcam setup */
	if (pf_adapter->adapter_cnt == 1) {
		wr32(pf_adapter, RNPM_ETH_TCAM_EN, 1);
		wr32(pf_adapter, RNPM_TOP_ETH_TCAM_CONFIG_ENABLE, 1);
		wr32(pf_adapter, RNPM_TCAM_MODE, 2);
#define TCAM_NUM (4096)
		for (i = 0; i < TCAM_NUM; i++) {
			wr32(pf_adapter, RNPM_TCAM_SDPQF(i), 0);
			wr32(pf_adapter, RNPM_TCAM_DAQF(i), 0);
			wr32(pf_adapter, RNPM_TCAM_SAQF(i), 0);
			wr32(pf_adapter, RNPM_TCAM_APQF(i), 0);

			wr32(pf_adapter, RNPM_TCAM_SDPQF_MASK(i), 0);
			wr32(pf_adapter, RNPM_TCAM_DAQF_MASK(i), 0);
			wr32(pf_adapter, RNPM_TCAM_SAQF_MASK(i), 0);
			wr32(pf_adapter, RNPM_TCAM_APQF_MASK(i), 0);
		}
		wr32(pf_adapter, RNPM_TCAM_MODE, 1);
	}
	// should open all tx
	rnpm_fix_dma_tx_status(pf_adapter);
#define DEFAULT_MIN_SIZE 60
#define DEFAULT_MAX_SIZE 1522
	wr32(pf_adapter, RNPM_ETH_DEFAULT_RX_MIN_LEN, DEFAULT_MIN_SIZE);
	wr32(pf_adapter, RNPM_ETH_DEFAULT_RX_MAX_LEN, DEFAULT_MAX_SIZE);
	//wr32(pf_adapter, RNPM_ETH_ERR_MASK_VECTOR, ETH_ERR_PKT_LEN_ERR | ETH_ERR_HDR_LEN_ERR);

	switch (pf_adapter->hw.mode) {
	case MODE_NIC_MODE_1PORT:
	case MODE_NIC_MODE_4PORT:
		wr32(pf_adapter, RNPM_ETH_TC_PORT_OFFSET_TABLE(0), 0);
		wr32(pf_adapter, RNPM_ETH_TC_PORT_OFFSET_TABLE(1), 1);
		wr32(pf_adapter, RNPM_ETH_TC_PORT_OFFSET_TABLE(2), 2);
		wr32(pf_adapter, RNPM_ETH_TC_PORT_OFFSET_TABLE(3), 3);

	break;
	case MODE_NIC_MODE_2PORT:
		wr32(pf_adapter, RNPM_ETH_TC_PORT_OFFSET_TABLE(0), 0);
		wr32(pf_adapter, RNPM_ETH_TC_PORT_OFFSET_TABLE(1), 2);
	break;
	}
	return 0;
}

__maybe_unused void wait_all_port_resetting(struct rnpm_pf_adapter *pf_adapter)
{
	int i;
	struct rnpm_adapter *adapter;
	// should wait all 
	for (i = 0; i < pf_adapter->adapter_cnt - 1; i++) {
		adapter = pf_adapter->adapter[i];
		while (test_and_set_bit(__RNPM_RESETTING, &adapter->state))
			usleep_range(1000, 2000);
	}
}

__maybe_unused void clean_all_port_resetting(struct rnpm_pf_adapter *pf_adapter)
{
	int i;
	struct rnpm_adapter *adapter;
	// should wait all 
	for (i = 0; i < pf_adapter->adapter_cnt - 1; i++) {
		adapter = pf_adapter->adapter[i];
		clear_bit(__RNPM_RESETTING, &adapter->state);
	}
}

static void rnpm_pf_mtu_subtask(struct rnpm_pf_adapter *pf_adapter)
{
	int i;
	struct rnpm_adapter *adapter;
	struct net_device *netdev;
	int mtu = 0;

	for (i = pf_adapter->adapter_cnt - 1; i >= 0; i--) {
		
		adapter = pf_adapter->adapter[i];
		if (adapter) {
			netdev = adapter->netdev;

			if (mtu < netdev->mtu)
				mtu = netdev->mtu;
			}
	}
	mtu = mtu + ETH_HLEN + 2 * ETH_FCS_LEN;

	wr32(pf_adapter, RNPM_ETH_DEFAULT_RX_MAX_LEN, mtu);
}


static void rnpm_pf_reset_subtask(struct rnpm_pf_adapter *pf_adapter)
{
	int err = 0;
	int i;
	struct rnpm_adapter *adapter;
	struct net_device *netdev;

	while (test_and_set_bit(__RNPM_RESETTING, &pf_adapter->state))
		usleep_range(1000, 2000);
	rnpm_warn("rx hang detected, reset pf \n");

	// try to pf nic reset
	err = rnpm_reset_pf(pf_adapter);

	// first stop all port
	for (i = pf_adapter->adapter_cnt - 1; i >= 0; i--) {
		adapter = pf_adapter->adapter[i];
		if (!adapter) {
			continue;
		}

		netdev = adapter->netdev;
		rtnl_lock();
		netif_device_detach(netdev);
		if (netif_running(netdev)) {
			rnpm_down(adapter);
			rnpm_free_irq(adapter);
			rnpm_free_all_tx_resources(adapter);
			rnpm_free_all_rx_resources(adapter);
			rnpm_mbx_ifup_down(&adapter->hw, MBX_IFDOWN);
		}
		/* free msix */
		//adapter->rm_mode = true;
		rnpm_clear_interrupt_scheme(adapter);
		rtnl_unlock();
	}

	// set all port up
	for (i = 0; i < pf_adapter->adapter_cnt; i++) {
		adapter = pf_adapter->adapter[i];
		if (!adapter) {
			continue;
		}

		netdev = adapter->netdev;
		//rnpm_reset(adapter);
		rtnl_lock();
		err = rnpm_init_interrupt_scheme(adapter);
		if (!err && netif_running(netdev))
			err = rnpm_open(netdev);

		netif_device_attach(netdev);
		rtnl_unlock();
	}

	clear_bit(__RNPM_RESETTING, &pf_adapter->state);
}

static void rnpm_reset_subtask(struct rnpm_adapter *adapter)
{
	if (!(adapter->flags2 & RNPM_FLAG2_RESET_REQUESTED))
		return;

	adapter->flags2 &= ~RNPM_FLAG2_RESET_REQUESTED;

	/* If we're already down or resetting, just bail */
	if (test_bit(__RNPM_DOWN, &adapter->state) ||
		test_bit(__RNPM_RESETTING, &adapter->state))
		return;

	// rnpm_dump(adapter);
	netdev_err(adapter->netdev, "Reset adapter\n");
	adapter->tx_timeout_count++;

	rnpm_reinit_locked(adapter);
}

static void rnpm_rx_len_reset_subtask(struct rnpm_adapter *adapter)
{
	int i;
	struct rnpm_ring *rx_ring;
	// struct net_device *netdev = adapter->netdev;

	for (i = 0; i < adapter->num_tx_queues; i++) {
		rx_ring = adapter->rx_ring[i];
		if (unlikely(rx_ring->ring_flags & RNPM_RING_FLAG_DO_RESET_RX_LEN)) {
			dbg("[%s] Rx-ring %d count reset\n",
				adapter->netdev->name,
				rx_ring->rnpm_queue_idx);
			rnpm_rx_ring_reinit(adapter, rx_ring);
			rx_ring->ring_flags &= (~RNPM_RING_FLAG_DO_RESET_RX_LEN);
		}
	}
}

/* just modify rx itr */
//static void rnpm_auto_itr_moderation(struct rnpm_adapter *adapter)
//{
//	int i;
//	struct rnpm_ring *rx_ring;
//	u64 period = (u64)(jiffies - adapter->last_moder_jiffies);
//	u32 pkt_rate_high, pkt_rate_low;
//	struct rnpm_hw *hw = &adapter->hw;
//	u64 packets;
//	u64 rate;
//	u64 avg_pkt_size;
//	u64 rx_packets;
//	u64 rx_bytes;
//	u64 rx_pkt_diff;
//	u32 itr_reg;
//	int moder_time;
//
//	/* if interface is down do nothing */
//	if (test_bit(__RNPM_DOWN, &adapter->state) ||
//		test_bit(__RNPM_RESETTING, &adapter->state))
//		return;
//
//	if (!adapter->auto_rx_coal)
//		return;
//
//	if (!adapter->adaptive_rx_coal || period < adapter->sample_interval * HZ) {
//		return;
//	}
//	pkt_rate_low = READ_ONCE(adapter->pkt_rate_low);
//	pkt_rate_high = READ_ONCE(adapter->pkt_rate_high);
//
//	/* it is time to check moderation */
//	for (i = 0; i < adapter->num_rx_queues; i++) {
//		rx_ring = adapter->rx_ring[i];
//		rx_packets = READ_ONCE(rx_ring->stats.packets);
//		rx_bytes = READ_ONCE(rx_ring->stats.bytes);
//		rx_pkt_diff =
//			rx_packets - adapter->last_moder_packets[rx_ring->queue_index];
//		packets = rx_pkt_diff;
//		rate = packets * HZ / period;
//
//		avg_pkt_size =
//			packets
//				? (rx_bytes - adapter->last_moder_bytes[rx_ring->queue_index]) /
//					  packets
//				: 0;
//
//		if (rate > (RNPM_RX_RATE_THRESH / adapter->num_rx_queues) &&
//			avg_pkt_size > RNPM_AVG_PKT_SMALL) {
//			if (rate <= pkt_rate_low)
//				moder_time = adapter->rx_usecs_low;
//			else if (rate >= pkt_rate_high)
//				moder_time = adapter->rx_usecs_high;
//			else
//				moder_time =
//					(rate - pkt_rate_low) *
//						(adapter->rx_usecs_high - adapter->rx_usecs_low) /
//						(pkt_rate_high - pkt_rate_low) +
//					adapter->rx_usecs_low;
//		} else {
//			moder_time = adapter->rx_usecs_low;
//		}
//
//		if (moder_time != adapter->last_moder_time[rx_ring->queue_index]) {
//#ifdef UV3P_1PF
//			itr_reg = moder_time * 300; // 150M
//#else
//			itr_reg = moder_time * 125; // 62.5M
//#endif
//			/* setup time to hw */
//			wr32(hw,
//				 RNPM_DMA_REG_RX_INT_DELAY_TIMER(rx_ring->rnpm_queue_idx),
//				 itr_reg);
//			adapter->last_moder_time[rx_ring->queue_index] = moder_time;
//		}
//		/* write back new count */
//		adapter->last_moder_packets[rx_ring->queue_index] = rx_packets;
//		adapter->last_moder_bytes[rx_ring->queue_index] = rx_bytes;
//	}
//}
// todo check lock status ?
int rnpm_check_mc_addr(struct rnpm_adapter *adapter)
{
	struct rnpm_pf_adapter *pf_adapter = adapter->pf_adapter;
	u32 mta_shadow[RNPM_MAX_MTA];
	int i;
	int j;
	int ret = 0;
	struct rnpm_hw *hw;
	/* store old data */
	memcpy(mta_shadow, pf_adapter->mta_shadow, sizeof(u32) * RNPM_MAX_MTA);
	/* caculate new data */
	for (i = 0; i < RNPM_MAX_MTA; i++) {
		pf_adapter->mta_shadow[i] = 0;
		for (j = 0; j < pf_adapter->adapter_cnt; j++) {
			if (rnpm_port_is_valid(pf_adapter, j)) {
				hw = &pf_adapter->adapter[j]->hw;
				pf_adapter->mta_shadow[i] |= hw->mac.mta_shadow[j];
			}
		}
		if (pf_adapter->mta_shadow[i] != mta_shadow[i])
			ret = 1;
	}
	return ret;
}

void update_pf_vlan(struct rnpm_adapter *adapter)
{
#if 0
	struct rnpm_pf_adapter *pf_adapter = adapter->pf_adapter;
	struct rnpm_hw *hw = &adapter->hw;
	unsigned long diff[BITS_TO_LONGS(VLAN_N_VID)];
	unsigned long new[BITS_TO_LONGS(VLAN_N_VID)];
	int num = BITS_TO_LONGS(VLAN_N_VID);
	int i, j;
    unsigned long flags;

	// fix this?
	memset(diff, 0, sizeof(diff));
	memset(new, 0, sizeof(new));
	spin_lock_irqsave(&pf_adapter->vlan_setup_lock,flags);


	/* try to get the new active_vlans */
	for (i = 0; i < num; i++) {
		for (j = 0; j < pf_adapter->adapter_cnt; j++) {
			adapter = pf_adapter->adapter[j];
			new[i] |= adapter->active_vlans[i];
		}
		// different is delete vlans 
		diff[i] = new[i] ^ pf_adapter->active_vlans[i];
		pf_adapter->active_vlans[i] = new[i];
	}
	// update vlan filter table 
	for_each_set_bit(i, diff, VLAN_N_VID) {
		if (hw->mac.ops.set_vfta) {
			/* remove VID from filter table */
			hw->mac.ops.set_vfta(&adapter->hw, i, VMDQ_P(0), false);
		}
	}

	spin_unlock_irqrestore(&pf_adapter->vlan_setup_lock,flags);
#endif
}

__maybe_unused static void
rnpm_update_feature_subtask(struct rnpm_adapter *adapter)
{
	struct rnpm_pf_adapter __maybe_unused *pf_adapter = adapter->pf_adapter;
	u32 changed = 0;
	netdev_features_t features = adapter->netdev->features;
	/* if interface is down do nothing */
	if (test_bit(__RNPM_DOWN, &adapter->state) ||
		test_bit(__RNPM_RESETTING, &adapter->state))
		return;

	/* update vlan filter status
	 * maybe other port update the unique status */
	if (adapter->flags_feature & RNPM_FLAG_DELAY_UPDATE_VLAN_FILTER) {
#ifdef NETIF_F_HW_VLAN_CTAG_FILTER
		if (pf_adapter->vlan_status_true) {
			if (!(features & NETIF_F_HW_VLAN_CTAG_FILTER)) {
				features |= NETIF_F_HW_VLAN_CTAG_FILTER;
				changed = 1;
			}
		} else {
			if (features & NETIF_F_HW_VLAN_CTAG_FILTER) {
				features &= (~NETIF_F_HW_VLAN_CTAG_FILTER);
				changed = 1;
			}
		}
#endif
	}
	if (changed) {
		adapter->netdev->features = features;
	}
	if (adapter->flags_feature & RNPM_FLAG_DELAY_UPDATE_VLAN_TABLE) {
		/* this port try to delete a vlan table */
		// todo
		update_pf_vlan(adapter);

		adapter->flags_feature &= (~RNPM_FLAG_DELAY_UPDATE_VLAN_TABLE);
	}

	if (adapter->flags_feature & RNPM_FLAG_DELAY_UPDATE_MUTICAST_TABLE) {
		// update muticast table
		// todo
		adapter->flags_feature &= (~RNPM_FLAG_DELAY_UPDATE_MUTICAST_TABLE);
	}
}
/**
 * rnpm_pf_service_task - manages and runs subtasks
 * @work: pointer to work_struct containing our data
 **/
void rnpm_pf_service_task(struct work_struct *work)
{
	struct rnpm_pf_adapter *pf_adapter =
		container_of(work, struct rnpm_pf_adapter, service_task);

	// check reset task ?
	if (pf_adapter->flags & RNPM_PF_RESET) {
		pf_adapter->flags &= (~RNPM_PF_RESET);
		rnpm_pf_reset_subtask(pf_adapter);
	}

	// mtu reset task
	if (pf_adapter->flags & RNPM_PF_SET_MTU) {
		// check 
		rnpm_pf_mtu_subtask(pf_adapter);
		pf_adapter->flags &= (~RNPM_PF_SET_MTU);
	}
	

}

/**
 * rnpm_service_task - manages and runs subtasks
 * @work: pointer to work_struct containing our data
 **/
void rnpm_service_task(struct work_struct *work)
{
	struct rnpm_adapter *adapter =
		container_of(work, struct rnpm_adapter, service_task);

	/* mutiport cannot offload vxlan */
	//#if defined(HAVE_UDP_ENC_RX_OFFLOAD) || defined(HAVE_VXLAN_RX_OFFLOAD)
	//#ifndef HAVE_UDP_TUNNEL_NIC_INFO
	//	if (adapter->flags2 & RNPM_FLAG2_UDP_TUN_REREG_NEEDED) {
	//		rtnl_lock();
	//		adapter->flags2 &= ~RNPM_FLAG2_UDP_TUN_REREG_NEEDED;
	//#ifdef HAVE_UDP_ENC_RX_OFFLOAD
	//		udp_tunnel_get_rx_info(adapter->netdev);
	//#else
	//		vxlan_get_rx_port(adapter->netdev);
	//#endif /*HAVE_UDP_ENC_RX_OFFLOAD */
	//		rtnl_unlock();
	//	}
	//#endif /* HAVE_UDP_ENC_RX_OFFLOAD || HAVE_VXLAN_RX_OFFLOAD */
	//#endif
	rnpm_reset_subtask(adapter);
	rnpm_sfp_detection_subtask(adapter);
	rnpm_sfp_link_config_subtask(adapter);
	rnpm_watchdog_subtask(adapter);
	rnpm_rx_len_reset_subtask(adapter);
	//rnpm_auto_itr_moderation(adapter);
	// rnpm_update_feature_subtask(adapter);
	// rnpm_fdir_reinit_subtask(adapter);
	rnpm_check_hang_subtask(adapter);
#if 0
	/* TODO Support */
	if (adapter->flags2 & RNPM_FLAG2_PTP_ENABLED) {
		rnpm_ptp_overflow_check(adapter);
		rnpm_ptp_rx_hang(adapter);
	}
#endif

	rnpm_service_event_complete(adapter);
}

static int
rnpm_tso(struct rnpm_ring *tx_ring, struct rnpm_tx_buffer *first, u8 *hdr_len)
{

	struct sk_buff *skb = first->skb;
	union {
		struct iphdr *v4;
		struct ipv6hdr *v6;
		unsigned char *hdr;
	} ip;
	union {
		struct tcphdr *tcp;
		struct udphdr *udp;
		unsigned char *hdr;
	} l4;
	u32 paylen, l4_offset;
	int err;
	u8 *inner_mac;
	u16 gso_segs, gso_size;
#ifdef FIX_MAC_PADDIN
	u16 gso_need_pad;
#endif

	if (skb->ip_summed != CHECKSUM_PARTIAL)
		return 0;

	if (!skb_is_gso(skb))
		return 0;

	err = skb_cow_head(skb, 0);
	if (err < 0)
		return err;

	inner_mac = skb->data;
	ip.hdr = skb_network_header(skb);
	l4.hdr = skb_transport_header(skb);

	first->tx_flags |= RNPM_TXD_FLAG_TSO | RNPM_TXD_IP_CSUM | RNPM_TXD_L4_CSUM |
					   RNPM_TXD_L4_TYPE_TCP;

	/* initialize outer IP header fields */
	if (ip.v4->version == 4) {
		/* IP header will have to cancel out any data that
		 * is not a part of the outer IP header
		 */
		ip.v4->check = 0x0000;
	} else {
		ip.v6->payload_len = 0;
	}
#ifdef HAVE_ENCAP_TSO_OFFLOAD
	if (skb_shinfo(skb)->gso_type &
		(SKB_GSO_GRE |
#ifdef NETIF_F_GSO_PARTIAL
		 SKB_GSO_GRE_CSUM |
#endif
		 SKB_GSO_UDP_TUNNEL | SKB_GSO_UDP_TUNNEL_CSUM)) {
#ifndef NETIF_F_GSO_PARTIAL
		if (skb_shinfo(skb)->gso_type & SKB_GSO_UDP_TUNNEL_CSUM) {
#else
		if (!(skb_shinfo(skb)->gso_type & SKB_GSO_PARTIAL) &&
			(skb_shinfo(skb)->gso_type & SKB_GSO_UDP_TUNNEL_CSUM)) {
#endif
		}
		/* we should always do this */
		inner_mac = skb_inner_mac_header(skb);

		first->tunnel_hdr_len = (inner_mac - skb->data);
		if (skb_shinfo(skb)->gso_type &
			(SKB_GSO_UDP_TUNNEL | SKB_GSO_UDP_TUNNEL_CSUM)) {
			first->tx_flags |= RNPM_TXD_TUNNEL_VXLAN;
			l4.udp->check = 0;
			tx_dbg("set outer l4.udp to 0\n");
		} else {
			first->tx_flags |= RNPM_TXD_TUNNEL_NVGRE;
		}

		/* reset pointers to inner headers */
		ip.hdr = skb_inner_network_header(skb);
		l4.hdr = skb_inner_transport_header(skb);
	}

#endif /* HAVE_ENCAP_TSO_OFFLOAD */
	if (ip.v4->version == 4) {
		/* IP header will have to cancel out any data that
		 * is not a part of the outer IP header
		 */
		ip.v4->check = 0x0000;

	} else {
		ip.v6->payload_len = 0;
		/* set ipv6 type */

		first->tx_flags |= (RNPM_TXD_FLAG_IPv6);
	}

	/* determine offset of inner transport header */
	l4_offset = l4.hdr - skb->data;

	paylen = skb->len - l4_offset;
	tx_dbg("before l4 checksum is %x\n", l4.tcp->check);

	csum_replace_by_diff(&l4.tcp->check, htonl(paylen));

	tx_dbg("l4 checksum is %x\n", l4.tcp->check);

	first->mac_ip_len = l4.hdr - ip.hdr;
	first->mac_ip_len |= (ip.hdr - inner_mac) << 9;

	/* compute header lengths */
	*hdr_len = (l4.tcp->doff * 4) + l4_offset;
	/* pull values out of skb_shinfo */
	gso_size = skb_shinfo(skb)->gso_size;
	gso_segs = skb_shinfo(skb)->gso_segs;

#ifndef HAVE_NDO_FEATURES_CHECK
	/* too small a TSO segment size causes problems */
	if (gso_size < 64) {
		gso_size = 64;
		gso_segs = DIV_ROUND_UP(skb->len - *hdr_len, 64);
	}
#endif

#ifdef FIX_MAC_PADDIN
	if ((gso_need_pad = (first->skb->len - *hdr_len) % gso_size)) {
		if ((gso_need_pad + *hdr_len) <= 60) {
			gso_need_pad = 60 - (gso_need_pad + *hdr_len);
			first->gso_need_padding = !!gso_need_pad;
		}
	}
#endif

	/* update gso size and bytecount with header size */
	/* to fix tx status */
	first->gso_segs = gso_segs;
	first->bytecount += (first->gso_segs - 1) * *hdr_len;

	first->mss_len_vf_num |= (gso_size | ((l4.tcp->doff * 4) << 24));
	// rnpm_tx_ctxtdesc(tx_ring,skb_shinfo(skb)->gso_size ,l4len, 0, 0,
	// type_tucmd);

	first->ctx_flag = true;
	return 1;
}

__maybe_unused static void set_resevd(struct rnpm_tx_buffer *first)
{
	struct sk_buff *skb = first->skb;
	union {
		struct iphdr *v4;
		struct ipv6hdr *v6;
		unsigned char *hdr;
	} ip;
	union {
		struct tcphdr *tcp;
		struct udphdr *udp;
		unsigned char *hdr;
	} l4 __maybe_unused;

	ip.hdr = skb_network_header(skb);

	if (ip.v4->version == 4) {
		u16 old = ip.v4->frag_off;

		ip.v4->frag_off |= 0x0080;
		// l4_proto = ip.v4->protocol;
		//  first->cmd_flags |= RNP_TXD_FLAG_IPv4;

		csum_replace_by_diff(&ip.v4->check, ip.v4->frag_off - old);
	}
}

static int rnpm_tx_csum(struct rnpm_ring *tx_ring, struct rnpm_tx_buffer *first)
{
	struct sk_buff *skb = first->skb;
	u8 l4_proto = 0;
	u8 ip_len = 0;
	u8 mac_len = 0;
	u8 *inner_mac = skb->data;
	u8 *exthdr;
	__be16 frag_off;
	union {
		struct iphdr *v4;
		struct ipv6hdr *v6;
		unsigned char *hdr;
	} ip;
	union {
		struct tcphdr *tcp;
		struct udphdr *udp;
		unsigned char *hdr;
	} l4;

	if (skb->ip_summed != CHECKSUM_PARTIAL) {
		return 0;
	}

	ip.hdr = skb_network_header(skb);
	l4.hdr = skb_transport_header(skb);

	inner_mac = skb->data;

#ifdef HAVE_ENCAP_CSUM_OFFLOAD
	/* outer protocol */
	if (skb->encapsulation) {
		/* define outer network header type */
		if (ip.v4->version == 4) {
			l4_proto = ip.v4->protocol;
		} else {
			exthdr = ip.hdr + sizeof(*ip.v6);
			l4_proto = ip.v6->nexthdr;
			if (l4.hdr != exthdr)
				ipv6_skip_exthdr(skb, exthdr - skb->data, &l4_proto, &frag_off);
		}

		/* define outer transport */
		switch (l4_proto) {
			case IPPROTO_UDP:
				l4.udp->check = 0;

				first->tx_flags |= RNPM_TXD_TUNNEL_VXLAN;
				break;
#ifdef HAVE_GRE_ENCAP_OFFLOAD
			case IPPROTO_GRE:
				first->tx_flags |= RNPM_TXD_TUNNEL_NVGRE;
				/* There was a long-standing issue in GRE where GSO
				 * was not setting the outer transport header unless
				 * a GRE checksum was requested. This was fixed in
				 * the 4.6 version of the kernel.  In the 4.7 kernel
				 * support for GRE over IPv6 was added to GSO.  So we
				 * can assume this workaround for all IPv4 headers
				 * without impacting later versions of the GRE.
				 */
				if (ip.v4->version == 4)
					l4.hdr = ip.hdr + (ip.v4->ihl * 4);
				break;
#endif
			default:
				skb_checksum_help(skb);
				return -1;
		}

		/* switch IP header pointer from outer to inner header */
		ip.hdr = skb_inner_network_header(skb);
		l4.hdr = skb_inner_transport_header(skb);

		inner_mac = skb_inner_mac_header(skb);
		first->tunnel_hdr_len = inner_mac - skb->data;
		first->ctx_flag = true;
		tx_dbg("tunnel length is %d\n", first->tunnel_hdr_len);
	}
#endif /* HAVE_ENCAP_CSUM_OFFLOAD */

	mac_len = (ip.hdr - inner_mac); // mac length
	tx_dbg("inner checksum needed %d", skb_checksum_start_offset(skb));
	tx_dbg("skb->encapsulation %d\n", skb->encapsulation);
	ip_len = (l4.hdr - ip.hdr);
	if (ip.v4->version == 4) {
		l4_proto = ip.v4->protocol;
		// first->cmd_flags |= RNPM_TXD_FLAG_IPv4;
	} else {
		exthdr = ip.hdr + sizeof(*ip.v6);
		l4_proto = ip.v6->nexthdr;
		if (l4.hdr != exthdr)
			ipv6_skip_exthdr(skb, exthdr - skb->data, &l4_proto, &frag_off);
		first->tx_flags |= RNPM_TXD_FLAG_IPv6;
	}
	/* Enable L4 checksum offloads */
	switch (l4_proto) {
		case IPPROTO_TCP:
			first->tx_flags |= RNPM_TXD_L4_TYPE_TCP | RNPM_TXD_L4_CSUM;
			break;
		case IPPROTO_SCTP:
			tx_dbg("sctp checksum packet \n");
			first->tx_flags |= RNPM_TXD_L4_TYPE_SCTP | RNPM_TXD_L4_CSUM;
			break;
		case IPPROTO_UDP:
			first->tx_flags |= RNPM_TXD_L4_TYPE_UDP | RNPM_TXD_L4_CSUM;
			break;
		default:
			skb_checksum_help(skb);
			return 0;
	}
	tx_dbg("mac length is %d\n", mac_len);
	tx_dbg("ip length is %d\n", ip_len);
	first->mac_ip_len = (mac_len << 9) | ip_len;
	return 0;
}
static int __rnpm_maybe_stop_tx(struct rnpm_ring *tx_ring, u16 size)
{
	tx_dbg("stop subqueue\n");
	netif_stop_subqueue(tx_ring->netdev, tx_ring->queue_index);
	/* Herbert's original patch had:
	 *  smp_mb__after_netif_stop_queue();
	 * but since that doesn't exist yet, just open code it. */
	smp_mb();

	/* We need to check again in a case another CPU has just
	 * made room available. */
	if (likely(rnpm_desc_unused(tx_ring) < size))
		return -EBUSY;

	/* A reprieve! - use start_queue because it doesn't call schedule */
	netif_start_subqueue(tx_ring->netdev, tx_ring->queue_index);
	++tx_ring->tx_stats.restart_queue;
	return 0;
}

static inline int rnpm_maybe_stop_tx(struct rnpm_ring *tx_ring, u16 size)
{
	if (likely(rnpm_desc_unused(tx_ring) >= size))
		return 0;
	return __rnpm_maybe_stop_tx(tx_ring, size);
}

static int rnpm_tx_map(struct rnpm_ring *tx_ring,
					   struct rnpm_tx_buffer *first,
					   const u8 hdr_len)
{
	struct sk_buff *skb = first->skb;
	struct rnpm_tx_buffer *tx_buffer;
	struct rnpm_tx_desc *tx_desc;
	skb_frag_t *frag;
	dma_addr_t dma;
	unsigned int data_len, size;

	u32 tx_flags = first->tx_flags;
	u32 mac_ip_len = (first->mac_ip_len) << 16;
	u16 i = tx_ring->next_to_use;
	u64 fun_id = ((u64)(tx_ring->pfvfnum) << (56));

	tx_desc = RNPM_TX_DESC(tx_ring, i);

	size = skb_headlen(skb);
	data_len = skb->data_len;

	dma = dma_map_single(tx_ring->dev, skb->data, size, DMA_TO_DEVICE);

	tx_buffer = first;

	// printk("next to use is %d\n", i);
	for (frag = &skb_shinfo(skb)->frags[0];; frag++) {
		if (dma_mapping_error(tx_ring->dev, dma))
			goto dma_error;

		/* record length, and DMA address */
		dma_unmap_len_set(tx_buffer, len, size);
		dma_unmap_addr_set(tx_buffer, dma, dma);

		// 1st desc
		tx_desc->pkt_addr = cpu_to_le64(dma | fun_id);

		while (unlikely(size > RNPM_MAX_DATA_PER_TXD)) {
			tx_desc->vlan_cmd = cpu_to_le32(tx_flags);
			tx_desc->blen_mac_ip_len =
				cpu_to_le32(mac_ip_len ^ RNPM_MAX_DATA_PER_TXD);
			//==== desc==
			buf_dump_line("tx0  ", __LINE__, tx_desc, sizeof(*tx_desc));
			i++;
			tx_desc++;
			if (i == tx_ring->count) {
				tx_desc = RNPM_TX_DESC(tx_ring, 0);
				i = 0;
			}
			dma += RNPM_MAX_DATA_PER_TXD;
			size -= RNPM_MAX_DATA_PER_TXD;

			tx_desc->pkt_addr = cpu_to_le64(dma | fun_id);
		}

		buf_dump_line("tx1  ", __LINE__, tx_desc, sizeof(*tx_desc));
		if (likely(!data_len)) // if not sg break
			break;
		tx_desc->vlan_cmd = cpu_to_le32(tx_flags);
		tx_desc->blen_mac_ip_len = cpu_to_le32(mac_ip_len ^ size);
		buf_dump_line("tx2  ", __LINE__, tx_desc, sizeof(*tx_desc));

		//==== frag==
		i++;
		tx_desc++;
		if (i == tx_ring->count) {
			tx_desc = RNPM_TX_DESC(tx_ring, 0);
			i = 0;
		}
		// tx_desc->cmd = RNPM_TXD_CMD_RS;
		// tx_desc->mac_ip_len = 0;

		size = skb_frag_size(frag);

		data_len -= size;

		dma = skb_frag_dma_map(tx_ring->dev, frag, 0, size, DMA_TO_DEVICE);

		tx_buffer = &tx_ring->tx_buffer_info[i];
	}

	/* write last descriptor with RS and EOP bits */
	tx_desc->vlan_cmd =
		cpu_to_le32(tx_flags | RNPM_TXD_CMD_EOP | RNPM_TXD_CMD_RS);
	tx_desc->blen_mac_ip_len = cpu_to_le32(mac_ip_len ^ size);

	// count++;

	buf_dump_line("tx3  ", __LINE__, tx_desc, sizeof(*tx_desc));

	/* set the timestamp */
	first->time_stamp = jiffies;

	// tx_ring->tx_stats.send_bytes += first->bytecount;
	netdev_tx_sent_queue(txring_txq(tx_ring), first->bytecount);

	/*
	 * Force memory writes to complete before letting h/w know there
	 * are new descriptors to fetch.  (Only applicable for weak-ordered
	 * memory model archs, such as IA-64).
	 *
	 * We also need this memory barrier to make certain all of the
	 * status bits have been updated before next_to_watch is written.
	 */

	/* set next_to_watch value indicating a packet is present */
	wmb();
	first->next_to_watch = tx_desc;

	// buf_dump_line("tx4  ", __LINE__, tx_desc, sizeof(*tx_desc));
	i++;
	if (i == tx_ring->count)
		i = 0;
#ifdef SIMULATE_TX
	napi_consume_skb(first->skb, 64);
	dma_unmap_single(tx_ring->dev,
					 dma_unmap_addr(first, dma),
					 dma_unmap_len(first, len),
					 DMA_TO_DEVICE);

	tx_ring->stats.bytes += skb->len;
	tx_ring->stats.packets += 1;
	first->skb = NULL;
#else
	tx_ring->next_to_use = i;

	/* need this */
	rnpm_maybe_stop_tx(tx_ring, DESC_NEEDED);
#ifdef HAVE_SKB_XMIT_MORE
	if (netif_xmit_stopped(txring_txq(tx_ring)) || !netdev_xmit_more()) {
		tx_ring->tx_stats.send_bytes_to_hw += first->bytecount;
		tx_ring->tx_stats.send_bytes_to_hw += tx_ring->tx_stats.todo_update;
		tx_ring->tx_stats.todo_update = 0;
		rnpm_wr_reg(tx_ring->tail, i);
#ifndef SPIN_UNLOCK_IMPLIES_MMIOWB
		/* we need this if more than one processor can write to our tail
		 * at a time, it synchronizes IO on IA64/Altix systems
		 */
		mmiowb();
#endif
	} else {
		tx_ring->tx_stats.todo_update = first->bytecount;
	}
#else
	/* notify HW of packet */
	rnpm_wr_reg(tx_ring->tail, i);

#ifndef SPIN_UNLOCK_IMPLIES_MMIOWB
	/* we need this if more than one processor can write to our tail
	 * at a time, it synchronizes IO on IA64/Altix systems
	 */
	mmiowb();
#endif
#endif /* HAVE_SKB_XMIT_MORE */

#endif
	// printk("now tail is %d\n", i);

	return 0;
dma_error:
	dev_err(tx_ring->dev, "TX DMA map failed\n");

	/* clear dma mappings for failed tx_buffer_info map */
	for (;;) {
		tx_buffer = &tx_ring->tx_buffer_info[i];
		rnpm_unmap_and_free_tx_resource(tx_ring, tx_buffer);
		if (tx_buffer == first)
			break;
		if (i == 0)
			i = tx_ring->count;
		i--;
	}

	tx_ring->next_to_use = i;

	return -1;
}
__maybe_unused static void rnpm_atr(struct rnpm_ring *ring,
									struct rnpm_tx_buffer *first)
{
#if 0
	struct rnpm_q_vector *q_vector = ring->q_vector;
	union rnpm_atr_hash_dword input = { .dword = 0 };
	union rnpm_atr_hash_dword common = { .dword = 0 };
	union {
		unsigned char *network;
		struct iphdr *ipv4;
		struct ipv6hdr *ipv6;
	} hdr;
	struct tcphdr *th;
	__be16 vlan_id;

	/* if ring doesn't have a interrupt vector, cannot perform ATR */
	if (!q_vector)
		return;

	/* do nothing if sampling is disabled */
	if (!ring->atr_sample_rate)
		return;

	ring->atr_count++;

	/* snag network header to get L4 type and address */
	hdr.network = skb_network_header(first->skb);

	/* Currently only IPv4/IPv6 with TCP is supported */
	if ((first->protocol != __constant_htons(ETH_P_IPV6) ||
	     hdr.ipv6->nexthdr != IPPROTO_TCP) &&
	    (first->protocol != __constant_htons(ETH_P_IP) ||
	     hdr.ipv4->protocol != IPPROTO_TCP))
		return;

	th = tcp_hdr(first->skb);

	/* skip this packet since it is invalid or the socket is closing */
	if (!th || th->fin)
		return;

	/* sample on all syn packets or once every atr sample count */
	if (!th->syn && (ring->atr_count < ring->atr_sample_rate))
		return;

	/* reset sample count */
	ring->atr_count = 0;

	vlan_id = htons(first->tx_flags >> RNPM_TX_FLAGS_VLAN_SHIFT);

	/*
	 * src and dst are inverted, think how the receiver sees them
	 *
	 * The input is broken into two sections, a non-compressed section
	 * containing vm_pool, vlan_id, and flow_type.  The rest of the data
	 * is XORed together and stored in the compressed dword.
	 */
	input.formatted.vlan_id = vlan_id;

	/*
	 * since src port and flex bytes occupy the same word XOR them together
	 * and write the value to source port portion of compressed dword
	 */
	if (first->tx_flags & (RNPM_TX_FLAGS_SW_VLAN | RNPM_TX_FLAGS_HW_VLAN))
		common.port.src ^= th->dest ^ __constant_htons(ETH_P_8021Q);
	else
		common.port.src ^= th->dest ^ first->protocol;
	common.port.dst ^= th->source;

	if (first->protocol == __constant_htons(ETH_P_IP)) {
		input.formatted.flow_type = RNPM_ATR_FLOW_TYPE_TCPV4;
		common.ip ^= hdr.ipv4->saddr ^ hdr.ipv4->daddr;
	} else {
		input.formatted.flow_type = RNPM_ATR_FLOW_TYPE_TCPV6;
		common.ip ^= hdr.ipv6->saddr.s6_addr32[0] ^
			     hdr.ipv6->saddr.s6_addr32[1] ^
			     hdr.ipv6->saddr.s6_addr32[2] ^
			     hdr.ipv6->saddr.s6_addr32[3] ^
			     hdr.ipv6->daddr.s6_addr32[0] ^
			     hdr.ipv6->daddr.s6_addr32[1] ^
			     hdr.ipv6->daddr.s6_addr32[2] ^
			     hdr.ipv6->daddr.s6_addr32[3];
	}

	/* This assumes the Rx queue and Tx queue are bound to the same CPU */
	rnpm_fdir_add_signature_filter_n10(&q_vector->adapter->hw,
					      input, common, ring->queue_index);
#endif
}

netdev_tx_t rnpm_xmit_frame_ring(struct sk_buff *skb,
								 struct rnpm_adapter *adapter,
								 struct rnpm_ring *tx_ring)
{
	struct rnpm_tx_buffer *first;
	int tso;
	u32 tx_flags = 0;
	unsigned short f;
	u16 count = TXD_USE_COUNT(skb_headlen(skb));
	__be16 protocol = skb->protocol;
	u8 hdr_len = 0;
	// struct rnpm_pf_adapter *pf_adapter = adapter->pf_adapter;

#ifdef SIMULATE_TX
	// napi_consume_skb(skb, 64);

	dev_kfree_skb_any(skb);
	/*
	dma_unmap_single(tx_ring->dev,
			dma_unmap_addr(first, dma),
			dma_unmap_len(first, len),
			DMA_TO_DEVICE);
	*/
	tx_ring->stats.bytes += skb->len;
	tx_ring->stats.packets += 1;
	// first->skb = NULL;
	return NETDEV_TX_OK;
#endif
	tx_dbg("=== begin ====\n");

	// rnpm_skb_dump(skb, true);

	tx_dbg("skb:%p, skb->len:%d  headlen:%d, data_len:%d\n",
		   skb,
		   skb->len,
		   skb_headlen(skb),
		   skb->data_len);
	/*
	 * need: 1 descriptor per page * PAGE_SIZE/RNPM_MAX_DATA_PER_TXD,
	 *       + 1 desc for skb_headlen/RNPM_MAX_DATA_PER_TXD,
	 *       + 2 desc gap to keep tail from touching head,
	 *       + 1 desc for context descriptor,
	 * otherwise try next time
	 */
	for (f = 0; f < skb_shinfo(skb)->nr_frags; f++) {
		skb_frag_t *frag_temp = &skb_shinfo(skb)->frags[f];
		count += TXD_USE_COUNT(skb_frag_size(frag_temp));
		tx_dbg(" #%d frag: size:%d\n", f, skb_frag_size(frag_temp));
		if (count > 60) {
			/* error detect */
			printk("desc too large, %d\n", count);
			return NETDEV_TX_BUSY;
		}
	}

	if (rnpm_maybe_stop_tx(tx_ring, count + 3)) {
		tx_ring->tx_stats.tx_busy++;
		return NETDEV_TX_BUSY;
	}

	/* record the location of the first descriptor for this packet */
	first = &tx_ring->tx_buffer_info[tx_ring->next_to_use];
	first->skb = skb;
	first->bytecount = skb->len;
	first->gso_segs = 1;
	first->type_tucmd = 0;

	/* default len should not 0 (hw request) */
	first->mac_ip_len = 20;
	first->mss_len_vf_num = 0;
	first->inner_vlan_tunnel_len = 0;

#ifdef RNPM_IOV_VEB_BUG_NOT_FIXED
	first->ctx_flag = (adapter->flags & RNPM_FLAG_SRIOV_ENABLED) ? true : false;
#else
	first->ctx_flag = false;
#endif
#ifdef FIX_MAC_PADDIN
	// for test only 
	first->ctx_flag = true;
#endif

	/* if we have a HW VLAN tag being added default to the HW one */
	/* RNPM_TXD_VLAN_VALID is used for veb */
	if (adapter->flags2 & RNPM_FLAG2_VLAN_STAGS_ENABLED) {
		/* always add a stags for any packets out */
		tx_flags |= adapter->stags_vid;
		tx_flags |= RNPM_TXD_VLAN_CTRL_INSERT_VLAN;
		if (skb_vlan_tag_present(skb)) {
			tx_flags |= RNPM_TXD_VLAN_VALID;
			first->inner_vlan_tunnel_len |= (skb_vlan_tag_get(skb) << 8);
			first->ctx_flag = true;
			/* else if it is a SW VLAN check the next protocol and store the tag
			 */
		} else if (protocol == __constant_htons(ETH_P_8021Q)) {
			struct vlan_hdr *vhdr, _vhdr;
			vhdr = skb_header_pointer(skb, ETH_HLEN, sizeof(_vhdr), &_vhdr);
			if (!vhdr)
				goto out_drop;

			protocol = vhdr->h_vlan_encapsulated_proto;
			// tx_flags |= ntohs(vhdr->h_vlan_TCI);
			tx_flags |= RNPM_TXD_VLAN_VALID;
		}
	} else {
		/* normal mode */
		if (skb_vlan_tag_present(skb)) {
			tx_flags |= skb_vlan_tag_get(skb);
			tx_flags |= RNPM_TXD_VLAN_VALID | RNPM_TXD_VLAN_CTRL_INSERT_VLAN;
			tx_ring->tx_stats.vlan_add++;
			/* else if it is a SW VLAN check the next protocol and store the tag
			 */
		} else if (protocol == __constant_htons(ETH_P_8021Q)) {
			struct vlan_hdr *vhdr, _vhdr;
			vhdr = skb_header_pointer(skb, ETH_HLEN, sizeof(_vhdr), &_vhdr);
			if (!vhdr)
				goto out_drop;

			protocol = vhdr->h_vlan_encapsulated_proto;
			tx_flags |= ntohs(vhdr->h_vlan_TCI);
			tx_flags |= RNPM_TXD_VLAN_VALID;
		}
	}
	protocol = vlan_get_protocol(skb);

	skb_tx_timestamp(skb);
	/* just for test */
	// tx_flags |= RNPM_TXD_FLAG_PTP;
#ifdef SKB_SHARED_TX_IS_UNION
	if (unlikely(skb_tx(skb)->hardware) &&
		adapter->flags2 & RNPM_FLAG2_PTP_ENABLED && adapter->ptp_tx_en) {
		if (!test_and_set_bit_lock(__RNPM_PTP_TX_IN_PROGRESS,
								   &adapter->state)) {
			skb_tx(skb)->in_progress = 1;

#else
	if (unlikely(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
		adapter->flags2 & RNPM_FLAG2_PTP_ENABLED && adapter->ptp_tx_en) {

		if (!test_and_set_bit_lock(__RNPM_PTP_TX_IN_PROGRESS,
								   &adapter->state)) {

			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
#endif
			tx_flags |= RNPM_TXD_FLAG_PTP;
			adapter->ptp_tx_skb = skb_get(skb);
			adapter->tx_hwtstamp_start = jiffies;
			schedule_work(&adapter->tx_hwtstamp_work);

		} else {
			printk("ptp_tx_skb miss\n");
		}
		//	if (adapter->ptp_tx_skb != NULL) {
		//		//dev_kfree_skb_any(adapter->ptp_tx_skb);
		//		//pci_err(adapter->pdev,
		//		//		"drop one ptp pkts when xmit that the clean_tx_irq not
		// excute\n"); 		printk("tx_hwtstamp_skipped\n"); 	} else {
		//		/* schedule check for Tx timestamp */
		//	}
	}
#if 0
	/* DCB maps skb priorities 0-7 onto 3 bit PCP of VLAN tag. */
	if ((adapter->flags & RNPM_FLAG_DCB_ENABLED) &&
			((tx_flags & (RNPM_TX_FLAGS_HW_VLAN | RNPM_TX_FLAGS_SW_VLAN)) ||
			 (skb->priority != TC_PRIO_CONTROL))) {
		tx_flags &= ~RNPM_TX_FLAGS_VLAN_PRIO_MASK;
		tx_flags |= (skb->priority & 0x7) <<
			RNPM_TX_FLAGS_VLAN_PRIO_SHIFT;
		if (tx_flags & RNPM_TX_FLAGS_SW_VLAN) {
			struct vlan_ethhdr *vhdr;
			if (skb_header_cloned(skb) &&
					pskb_expand_head(skb, 0, 0, GFP_ATOMIC))
				goto out_drop;
			vhdr = (struct vlan_ethhdr *)skb->data;
			vhdr->h_vlan_TCI = htons(tx_flags >>
					RNPM_TX_FLAGS_VLAN_SHIFT);
		} else {
			tx_flags |= RNPM_TX_FLAGS_HW_VLAN;
		}
	}
#endif
	/* record initial flags and protocol */
	first->tx_flags = tx_flags;
	first->protocol = protocol;

	tso = rnpm_tso(tx_ring, first, &hdr_len);
	if (tso < 0) {
		goto out_drop;
	} else if (!tso) {
		rnpm_tx_csum(tx_ring, first);
	}
	// set_resevd(first);
	/* check sriov mode */
	/* in this mode pf send msg should with vf_num */
	if (adapter->flags & RNPM_FLAG_SRIOV_ENABLED) {
		first->ctx_flag = true;
		first->mss_len_vf_num |= (adapter->vf_num_for_pf << 16);
	}

	/* send this packet to rpu */
	if (adapter->priv_flags & RNPM_PRIV_FLAG_TO_RPU) {
		// (pf_adapter->priv_flags & RNPM_PRIV_FLAG_SWITCH_LOOPBACK)) {
		first->ctx_flag = true;
		first->type_tucmd |= RNPM_TXD_FLAG_TO_RPU;
	}

	/* add control desc */
	rnpm_maybe_tx_ctxtdesc(tx_ring, first, first->type_tucmd);
	/* add the ATR filter if ATR is on */
	/*
	if (test_bit(__RNPM_TX_FDIR_INIT_DONE, &tx_ring->state))
		rnpm_atr(tx_ring, first);
	*/

	if (rnpm_tx_map(tx_ring, first, hdr_len))
		goto cleanup_tx_tstamp;
	/*
	   dev_kfree_skb_any(skb);
	   tx_ring->stats.bytes += skb->len;
	   tx_ring->stats.packets +=1;
	   */

	// rnpm_maybe_stop_tx(tx_ring, DESC_NEEDED);

	tx_dbg("=== end ====\n\n\n\n");
	return NETDEV_TX_OK;

out_drop:
	dev_kfree_skb_any(first->skb);
	first->skb = NULL;
cleanup_tx_tstamp:

	if (unlikely(tx_flags & RNPM_TXD_FLAG_PTP)) {
		dev_kfree_skb_any(adapter->ptp_tx_skb);
		adapter->ptp_tx_skb = NULL;
		cancel_work_sync(&adapter->tx_hwtstamp_work);
		clear_bit_unlock(__RNPM_PTP_TX_IN_PROGRESS, &adapter->state);
	}

	return NETDEV_TX_OK;
}

static netdev_tx_t rnpm_xmit_frame(struct sk_buff *skb,
								   struct net_device *netdev)
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	struct rnpm_ring *tx_ring;

	if (!netif_carrier_ok(netdev)) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	/*
	 * The minimum packet size for olinfo paylen is 17 so pad the skb
	 * in order to meet this minimum size requirement.
	 */
	// for test only  solf padding 
#ifdef FIX_MAC_PADDIN
	if (skb_put_padto(skb, 60))
		return NETDEV_TX_OK;
#else
	if (skb_put_padto(skb, 17))
		return NETDEV_TX_OK;
#endif
	/* for sctp packet , padding 0 change the crc32c */
	/* mac can padding (17-63) length to 64 */
	tx_ring = adapter->tx_ring[skb->queue_mapping];

	return rnpm_xmit_frame_ring(skb, adapter, tx_ring);
}

/**
 * rnpm_set_mac - Change the Ethernet Address of the NIC
 * @netdev: network interface device structure
 * @p: pointer to an address structure
 *
 * Returns 0 on success, negative on failure
 **/
static int rnpm_set_mac(struct net_device *netdev, void *p)
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	struct rnpm_hw *hw = &adapter->hw;
	struct sockaddr *addr = p;

	dbg("[%s] call set mac\n", netdev->name);

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	eth_hw_addr_set(netdev, addr->sa_data);
	//memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);
	memcpy(hw->mac.addr, addr->sa_data, netdev->addr_len);

	hw->mac.ops.set_rar(
		hw, adapter->uc_off, hw->mac.addr, VMDQ_P(0), RNPM_RAH_AV);

	/* setup mac unicast filters */
	if (hw->mac.mc_location == rnpm_mc_location_mac) {
		hw->mac.ops.set_rar_mac(hw, 0, hw->mac.addr, VMDQ_P(0), adapter->port);
	}

	rnpm_configure_virtualization(adapter);
	return 0;
}

static int
rnpm_mdio_read(struct net_device *netdev, int prtad, int devad, u16 addr)
{
	int rc = -EIO;
#if 0
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	struct rnpm_hw *hw = &adapter->hw;
	u16 value;

	if (prtad != hw->phy.mdio.prtad)
		return -EINVAL;
	rc = hw->phy.ops.read_reg(hw, addr, devad, &value);
	if (!rc)
		rc = value;
#endif
	return rc;
}

static int rnpm_mdio_write(
	struct net_device *netdev, int prtad, int devad, u16 addr, u16 value)
{
#if 0
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	struct rnpm_hw *hw = &adapter->hw;

	if (prtad != hw->phy.mdio.prtad)
		return -EINVAL;
	return hw->phy.ops.write_reg(hw, addr, devad, value);
#endif
	return -EINVAL;
}

static int rnpm_mii_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	struct mii_ioctl_data *mii = (struct mii_ioctl_data *)&ifr->ifr_data;
	int prtad, devad, ret;

	prtad = (mii->phy_id & MDIO_PHY_ID_PRTAD) >> 5;
	devad = (mii->phy_id & MDIO_PHY_ID_DEVAD);

	if (cmd == SIOCGMIIREG) {
		ret = rnpm_mdio_read(netdev, prtad, devad, mii->reg_num);
		if (ret < 0)
			return ret;
		mii->val_out = ret;
		return 0;
	} else {
		return rnpm_mdio_write(netdev, prtad, devad, mii->reg_num, mii->val_in);
	}
}

static int rnpm_ioctl(struct net_device *netdev, struct ifreq *req, int cmd)
{
#ifdef HAVE_PTP_1588_CLOCK
	struct rnpm_adapter *adapter = netdev_priv(netdev);
#endif

	// printk("rnpm ioctl cmd is %x\n", cmd);
	/* ptp 1588 used this */
	switch (cmd) {
#ifdef HAVE_PTP_1588_CLOCK
#ifdef SIOCGHWTSTAMP
		case SIOCGHWTSTAMP:
#ifndef NO_PTP
			if (module_enable_ptp)
				return rnpm_ptp_get_ts_config(adapter, req);
#endif
#endif
			break;
		case SIOCSHWTSTAMP:
#ifndef NO_PTP
			if (module_enable_ptp)
				return rnpm_ptp_set_ts_config(adapter, req);
			break;
#endif
#endif
		case SIOCGMIIREG:
		case SIOCSMIIREG:
			return rnpm_mii_ioctl(netdev, req, cmd);
	}
	return -EINVAL;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 * Polling 'interrupt' - used by things like netconsole to send skbs
 * without having to re-enable interrupts. It's not called while
 * the interrupt routine is executing.
 */
static void rnpm_netpoll(struct net_device *netdev)
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	int i;

	/* if interface is down do nothing */
	if (test_bit(__RNPM_DOWN, &adapter->state))
		return;

	adapter->flags |= RNPM_FLAG_IN_NETPOLL;
	for (i = 0; i < adapter->num_q_vectors; i++)
		rnpm_msix_clean_rings(0, adapter->q_vector[i]);
	adapter->flags &= ~RNPM_FLAG_IN_NETPOLL;
}

#endif

#ifdef HAVE_NDO_GET_STATS64
#ifdef HAVE_VOID_NDO_GET_STATS64
static void rnpm_get_stats64(struct net_device *netdev,
							 struct rtnl_link_stats64 *stats)
#else
static struct rtnl_link_stats64 *
rnpm_get_stats64(struct net_device *netdev, struct rtnl_link_stats64 *stats)
#endif
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	int i;

	rcu_read_lock();

	for (i = 0; i < adapter->num_rx_queues; i++) {
		struct rnpm_ring *ring = READ_ONCE(adapter->rx_ring[i]);
		u64 bytes, packets;
		unsigned int start;

		if (ring) {
			do {
				start = u64_stats_fetch_begin_irq(&ring->syncp);
				packets = ring->stats.packets;
				bytes = ring->stats.bytes;
			} while (u64_stats_fetch_retry(&ring->syncp, start));
			stats->rx_packets += packets;
			stats->rx_bytes += bytes;
		}
	}

	for (i = 0; i < adapter->num_tx_queues; i++) {
		struct rnpm_ring *ring = READ_ONCE(adapter->tx_ring[i]);
		u64 bytes, packets;
		unsigned int start;

		if (ring) {
			do {
				start = u64_stats_fetch_begin_irq(&ring->syncp);
				packets = ring->stats.packets;
				bytes = ring->stats.bytes;
			} while (u64_stats_fetch_retry(&ring->syncp, start));
			stats->tx_packets += packets;
			stats->tx_bytes += bytes;
		}
	}

	rcu_read_unlock();
	/* following stats updated by rnpm_watchdog_task() */
	stats->multicast = netdev->stats.multicast;
	stats->rx_errors = netdev->stats.rx_errors;
	stats->rx_frame_errors = netdev->stats.rx_frame_errors;
	// stats->rx_length_errors	= netdev->stats.rx_length_errors;
	stats->rx_crc_errors = netdev->stats.rx_crc_errors;
	// stats->rx_missed_errors	= netdev->stats.rx_missed_errors;

#ifndef HAVE_VOID_NDO_GET_STATS64
	return stats;
#endif
}
#else
/**
 * rnpm_get_stats - Get System Network Statistics
 * @netdev: network interface device structure
 *
 * Returns the address of the device statistics structure.
 * The statistics are actually updated from the timer callback.
 **/
static struct net_device_stats *rnpm_get_stats(struct net_device *netdev)
{
        struct rnpm_adapter *adapter = netdev_priv(netdev);

        /* update the stats data */
        rnpm_update_stats(adapter);

#ifdef HAVE_NETDEV_STATS_IN_NETDEV
        /* only return the current stats */
        return &netdev->stats;
#else
        /* only return the current stats */
        return &adapter->net_stats;
#endif /* HAVE_NETDEV_STATS_IN_NETDEV */
}

#endif
#ifdef CONFIG_RNPM_DCB
/**
 * rnpm_validate_rtr - verify 802.1Qp to Rx packet buffer mapping is valid.
 * @adapter: pointer to rnpm_adapter
 * @tc: number of traffic classes currently enabled
 *
 * Configure a valid 802.1Qp to Rx packet buffer mapping ie confirm
 * 802.1Q priority maps to a packet buffer that exists.
 */
static void rnpm_validate_rtr(struct rnpm_adapter *adapter, u8 tc)
{
	struct rnpm_hw *hw = &adapter->hw;
	u32 reg, rsave;
	int i;
#if 0
	/* 82598 have a static priority to TC mapping that can not
	 * be changed so no validation is needed.
	 */
	if (hw->mac.type == rnpm_mac_82598EB)
		return;

	reg = RNPM_READ_REG(hw, RNPM_RTRUP2TC);
	rsave = reg;

	for (i = 0; i < MAX_TRAFFIC_CLASS; i++) {
		u8 up2tc = reg >> (i * RNPM_RTRUP2TC_UP_SHIFT);

		/* If up2tc is out of bounds default to zero */
		if (up2tc > tc)
			reg &= ~(0x7 << RNPM_RTRUP2TC_UP_SHIFT);
	}

	if (reg != rsave)
		RNPM_WRITE_REG(hw, RNPM_RTRUP2TC, reg);
#endif
	return;
}

/**
 * rnpm_set_prio_tc_map - Configure netdev prio tc map
 * @adapter: Pointer to adapter struct
 *
 * Populate the netdev user priority to tc map
 */
static void rnpm_set_prio_tc_map(struct rnpm_adapter *adapter)
{
	struct net_device *dev = adapter->netdev;
#if 0
	struct rnpm_dcb_config *dcb_cfg = &adapter->dcb_cfg;
	struct ieee_ets *ets = adapter->rnpm_ieee_ets;
	u8 prio;

	for (prio = 0; prio < MAX_USER_PRIORITY; prio++) {
		u8 tc = 0;

		if (adapter->dcbx_cap & DCB_CAP_DCBX_VER_CEE)
			tc = rnpm_dcb_get_tc_from_up(dcb_cfg, 0, prio);
		else if (ets)
			tc = ets->prio_tc[prio];

		netdev_set_prio_tc_map(dev, prio, tc);
	}
#endif
}

#endif /* CONFIG_RNPM_DCB */
/**
 * rnpm_setup_tc - configure net_device for multiple traffic classes
 *
 * @netdev: net device to configure
 * @tc: number of traffic classes to enable
 */
int rnpm_setup_tc(struct net_device *dev, u8 tc)
{
	int err = 0;
	struct rnpm_adapter *adapter = netdev_priv(dev);
	struct rnpm_hw *hw = &adapter->hw;

	/* Hardware supports up to 8 traffic classes */
	if (tc > RNPM_MAX_TCS_NUM)
		return -EINVAL;

	/* Hardware has to reinitialize queues and interrupts to
	 * match packet buffer alignment. Unfortunately, the
	 * hardware is not flexible enough to do this dynamically.
	 */
	while (test_and_set_bit(__RNPM_RESETTING, &adapter->pf_adapter->state))
		usleep_range(1000, 2000);

	if (netif_running(dev))
		rnpm_close(dev);

	rnpm_clear_interrupt_scheme(adapter);
	hw->mac.ops.clear_hw_cntrs(hw);
	rnpm_update_stats(adapter);

#ifdef CONFIG_RNPM_DCB
	if (tc) {
		netdev_set_num_tc(dev, tc);
		rnpm_set_prio_tc_map(adapter);

		adapter->flags |= RNPM_FLAG_DCB_ENABLED;

		if (adapter->hw.mac.type == rnpm_mac_82598EB) {
			adapter->last_lfc_mode = adapter->hw.fc.requested_mode;
			adapter->hw.fc.requested_mode = rnpm_fc_none;
		}
	} else {
		netdev_reset_tc(dev);

		if (adapter->hw.mac.type == rnpm_mac_82598EB)
			adapter->hw.fc.requested_mode = adapter->last_lfc_mode;

		adapter->flags &= ~RNPM_FLAG_DCB_ENABLED;

		adapter->temp_dcb_cfg.pfc_mode_enable = false;
		adapter->dcb_cfg.pfc_mode_enable = false;
	}

	rnpm_validate_rtr(adapter, tc);

#endif /* CONFIG_RNPM_DCB */
	rnpm_init_interrupt_scheme(adapter);

	/* rss table must reset */
	adapter->rss_tbl_setup_flag = 0;

	if (netif_running(dev))
		err = rnpm_open(dev);
		//return rnpm_open(dev);

	clear_bit(__RNPM_RESETTING, &adapter->pf_adapter->state);
	return err;
}

#ifdef CONFIG_PCI_IOV
void rnpm_sriov_reinit(struct rnpm_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;

	rtnl_lock();
	rnpm_setup_tc(netdev, netdev_get_num_tc(netdev));
	rtnl_unlock();
}
#endif

//#if (defined HAVE_RHEL7_NETDEV_OPS_EXT_NDO_SETUP_TC) || (defined
// NETIF_F_HW_TC)
//
//#ifdef HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV
// static int __rnpm_setup_tc(struct net_device *netdev, enum tc_setup_type
// type, 			  void *type_data) #elif defined(HAVE_NDO_SETUP_TC_CHAIN_INDEX)
// static int __rnpm_setup_tc(struct net_device *netdev, u32 handle, 		u32
// chain_index,
//__be16 proto, 		struct tc_to_netdev *tc) #else static int
//__rnpm_setup_tc(struct
// net_device *netdev, u32 handle, __be16 proto, 		struct tc_to_netdev *tc)
// #endif
//{
//#ifndef HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV
//	unsigned int type = tc->type;
//#ifdef HAVE_NDO_SETUP_TC_CHAIN_INDEX
//	if (chain_index) {
//		dbg("chain_index %d\n", chain_index);
//		return -EOPNOTSUPP;
//	}
//#endif /* HAVE_NDO_SETUP_TC_CHAIN_INDEX */
//#ifdef HAVE_TC_SETUP_CLSU32
//	netdev_info(netdev, " TC_H_MAJ %llu H_MAJ  %llu \n",
//			TC_H_MAJ(handle), TC_H_MAJ(TC_H_INGRESS));
//	if (TC_H_MAJ(handle) == TC_H_MAJ(TC_H_INGRESS) &&
//			type == TC_SETUP_CLSU32) {
//		netdev_info(netdev, "setup_tc type is %d\n", type);
//		return rnpm_setup_tc_cls_u32(dev, proto, tc->cls_u32);
//	}
//#endif /* HAVE_TC_SETUP_CLSU32 */
//#endif /* !HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV */
//	switch (type) {
//#if defined(HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV)
//#if defined(HAVE_TCF_BLOCK)
//	case TC_SETUP_BLOCK:
//			return rnpm_setup_tc_block(netdev, type_data);
//#else
//#ifdef HAVE_TC_SETUP_CLSU32
//	case TC_SETUP_CLSU32:
//			return rnpm_setup_tc_cls_u32(netdev, type_data);
//#endif /* HAVE_TC_SETUP_CLSU32 */
//#endif /* HAVE_TCF_BLOCK */
//#endif /* HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV */
///*
//	case TC_SETUP_QDISC_MQPRIO:
//#if defined(HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV)
//		return rnpm_setup_mqprio(netdev, type_data);
//#endif
//*/
//	default:
//		return -EOPNOTSUPP;
//	}
//
//	return 0;
//}
//#endif
void rnpm_do_reset(struct net_device *netdev)
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);

	if (netif_running(netdev))
		rnpm_reinit_locked(adapter);
	else
		rnpm_reset(adapter);
}

#ifdef HAVE_NDO_SET_FEATURES
#ifdef HAVE_RHEL6_NET_DEVICE_OPS_EXT
static u32 rnpm_fix_features(struct net_device *netdev, u32 features)
#else
static netdev_features_t rnpm_fix_features(struct net_device *netdev,
										   netdev_features_t features)
#endif
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	// struct rnpm_pf_adapter *pf_adapter = adapter->pf_adapter;
	// u8 port = adapter->port;

	/* If Rx checksum is disabled, then RSC/LRO should also be disabled */
	if (!(features & NETIF_F_RXCSUM))
		features &= ~NETIF_F_LRO;

	/* close rx csum when rx fcs on */
	if (features & NETIF_F_RXFCS)
		features &= (~NETIF_F_RXCSUM);
	/* Turn off LRO if not RSC capable */
	if (!(adapter->flags2 & RNPM_FLAG2_RSC_CAPABLE))
		features &= ~NETIF_F_LRO;

	/* in multiports mode we not support ntuple */
	/*
	if (adapter->flags & RNPM_FLAG_MUTIPORT_ENABLED) {
		if (features & NETIF_F_NTUPLE) {
			features &= ~NETIF_F_NTUPLE;
		}
	}
	*/

	return features;
}

#ifdef HAVE_RHEL6_NET_DEVICE_OPS_EXT
static int rnpm_set_features(struct net_device *netdev, u32 features)
#else
static int rnpm_set_features(struct net_device *netdev,
							 netdev_features_t features)
#endif
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);
	// struct rnpm_pf_adapter *pf_adapter = adapter->pf_adapter;
	netdev_features_t changed = netdev->features ^ features;
	bool need_reset = false;
	struct rnpm_hw *hw = &adapter->hw;
	// int i;
	// u8 port = adapter->port;

#if 0
	/* Make sure RSC matches LRO, reset if change */
	/* we don't support rsc */
	if (!(features & NETIF_F_LRO)) {
		if (adapter->flags2 & RNPM_FLAG2_RSC_ENABLED)
			need_reset = true;
		adapter->flags2 &= ~RNPM_FLAG2_RSC_ENABLED;
	} else if ((adapter->flags2 & RNPM_FLAG2_RSC_CAPABLE) &&
			!(adapter->flags2 & RNPM_FLAG2_RSC_ENABLED)) {
		if (adapter->rx_itr_setting == 1 ||
				adapter->rx_itr_setting > RNPM_MIN_RSC_ITR) {
			adapter->flags2 |= RNPM_FLAG2_RSC_ENABLED;
			need_reset = true;
		} else if ((changed ^ features) & NETIF_F_LRO) {
			e_info(probe, "rx-usecs set too low, "
					"disabling RSC\n");
		}
	}
#endif

	switch (features & NETIF_F_NTUPLE) {
		case NETIF_F_NTUPLE:
			/* turn off ATR, enable perfect filters and reset */
			if (!(adapter->flags & RNPM_FLAG_FDIR_PERFECT_CAPABLE))
				need_reset = true;

			adapter->flags &= ~RNPM_FLAG_FDIR_HASH_CAPABLE;
			adapter->flags |= RNPM_FLAG_FDIR_PERFECT_CAPABLE;
			break;
		default:
			/* turn off perfect filters, enable ATR and reset */
			if (adapter->flags & RNPM_FLAG_FDIR_PERFECT_CAPABLE)
				need_reset = true;

			adapter->flags &= ~RNPM_FLAG_FDIR_PERFECT_CAPABLE;

			/* We cannot enable ATR if SR-IOV is enabled */
			if (adapter->flags & RNPM_FLAG_SRIOV_ENABLED)
				break;

			/* We cannot enable ATR if we have 2 or more traffic classes */
			if (netdev_get_num_tc(netdev) > 1)
				break;

			/* We cannot enable ATR if RSS is disabled */
			// if (adapter->ring_feature[RING_F_RSS].limit <= 1)
			//     break;

			/* A sample rate of 0 indicates ATR disabled */
			if (!adapter->atr_sample_rate)
				break;

			adapter->flags |= RNPM_FLAG_FDIR_HASH_CAPABLE;
			break;
	}

		/* vlan filter changed */
#ifdef NETIF_F_HW_VLAN_CTAG_FILTER
	if (changed & (NETIF_F_HW_VLAN_CTAG_FILTER)) {
		if (features & NETIF_F_HW_VLAN_CTAG_FILTER)
			rnpm_vlan_filter_enable(adapter);
		else
			rnpm_vlan_filter_disable(adapter);
	}
#endif /* NETIF_F_HW_VLAN_CTAG_FILTER */
	/* rss hash changed */
	/* should set rss table to all 0 */
	if (changed & (NETIF_F_RXHASH)) {
		if (adapter->flags & RNPM_FLAG_MUTIPORT_ENABLED) {
			/* in mutiport mode ,use rss table to zero instead close hw flags */
			if (features & (NETIF_F_RXHASH)) {
				adapter->flags &= (~RNPM_FLAG_RXHASH_DISABLE);
				rnpm_store_reta(adapter);
			} else {
				adapter->flags |= RNPM_FLAG_RXHASH_DISABLE;
				rnpm_store_reta(adapter);
			}

		} else {
			u32 iov_en = (adapter->flags & RNPM_FLAG_SRIOV_ENABLED)
							 ? RNPM_IOV_ENABLED
							 : 0;
			/* close rxhash will lead all rx packets to ring 0 */
			if (features & (NETIF_F_RXHASH))
				wr32(hw,
					 RNPM_ETH_RSS_CONTROL,
					 RNPM_ETH_ENABLE_RSS_ONLY | iov_en);
			else
				wr32(hw, RNPM_ETH_RSS_CONTROL, RNPM_ETH_DISABLE_RSS | iov_en);
		}
	}

	/* rx fcs changed */
	/* in this mode rx l4/sctp checksum will get error */
	if (changed & NETIF_F_RXFCS) {
		u32 old_value;

		rnpm_msg_post_status(adapter, PF_FCS_STATUS);

		old_value = rd32(hw, RNPM_MAC_RX_CFG(adapter->port));

#define FCS_MASK (0x6)
		if (features & NETIF_F_RXFCS) {
			old_value &= (~FCS_MASK);
			/* if in rx fcs mode , hw rxcsum may error, close rxcusm */
		} else {
			old_value |= FCS_MASK;
		}
		wr32(hw, RNPM_MAC_RX_CFG(adapter->port), old_value);
	}

	if (changed & NETIF_F_RXALL)
		need_reset = true;

#ifdef NETIF_F_HW_VLAN_CTAG_RX
	if (changed & NETIF_F_HW_VLAN_CTAG_RX) {
		if (features & NETIF_F_HW_VLAN_CTAG_RX)
			rnpm_vlan_strip_enable(adapter);
		else
			rnpm_vlan_strip_disable(adapter);
	}
#endif

	/* set up active feature */
	netdev->features = features;

	if (need_reset)
		rnpm_do_reset(netdev);

	return 0;
}
#endif /* HAVE_NDO_SET_FEATURES */

//#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 1, 0))
// static int rnpm_ndo_fdb_add(struct ndmsg *ndm, struct nlattr *tb[],
//		struct net_device *dev, const unsigned char *addr,
//		u16 vid, u16 flags, struct netlink_ext_ack *extack)
//#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0))
// static int rnpm_ndo_fdb_add(struct ndmsg *ndm, struct nlattr *tb[],
//		struct net_device *dev,
//		const unsigned char *addr,
//		u16 flags)
//#else
// static int rnpm_ndo_fdb_add(struct ndmsg *ndm, struct net_device *dev,
//		unsigned char *addr, u16 flags)
//#endif
//{
//	struct rnpm_adapter *adapter = netdev_priv(dev);
//	int err = 0;
//
//
//	if (!(adapter->flags & RNPM_FLAG_SRIOV_ENABLED))
//#if defined(RHEL_RELEASE_CODE)
//		return ndo_dflt_fdb_add(ndm, tb, dev, addr, 0, flags);
//#else
//	return ndo_dflt_fdb_add(ndm, tb, dev, addr, flags);
//#endif
//
//
//	/* Hardware does not support aging addresses so if a
//	 * ndm_state is given only allow permanent addresses
//	 */
//	if (ndm->ndm_state && !(ndm->ndm_state & NUD_PERMANENT)) {
//		pr_info("%s: FDB only supports static addresses\n",
//				rnpm_driver_name);
//		return -EINVAL;
//	}
//
//	if (is_unicast_ether_addr(addr) || is_link_local_ether_addr(addr)) {
//		u32 rar_uc_entries = RNPM_MAX_PF_MACVLANS;
//
//		if (netdev_uc_count(dev) < rar_uc_entries)
//			err = dev_uc_add_excl(dev, addr);
//		else
//			err = -ENOMEM;
//	} else if (is_multicast_ether_addr(addr)) {
//		err = dev_mc_add_excl(dev, addr);
//	} else {
//		err = -EINVAL;
//	}
//
//	/* Only return duplicate errors if NLM_F_EXCL is set */
//	if (err == -EEXIST && !(flags & NLM_F_EXCL))
//		err = 0;
//
//	return err;
//}

#ifdef HAVE_BRIDGE_ATTRIBS
#ifdef HAVE_NDO_BRIDGE_SETLINK_EXTACK
static int rnpm_ndo_bridge_setlink(struct net_device *dev,
								   struct nlmsghdr *nlh,
								   __always_unused u16 flags,
								   struct netlink_ext_ack __always_unused *ext)
#elif defined(HAVE_NDO_BRIDGE_SET_DEL_LINK_FLAGS)
static int rnpm_ndo_bridge_setlink(struct net_device *dev,
								   struct nlmsghdr *nlh,
								   __always_unused u16 flags)
#else
static int rnpm_ndo_bridge_setlink(struct net_device *dev, struct nlmsghdr *nlh)
#endif /* HAVE_NDO_BRIDGE_SETLINK_EXTACK */
{
	struct rnpm_adapter *adapter = netdev_priv(dev);
	struct rnpm_hw *hw = &adapter->hw;
	struct nlattr *attr, *br_spec;
	int rem;

	if (!(adapter->flags & RNPM_FLAG_SRIOV_ENABLED))
		return -EOPNOTSUPP;

	br_spec = nlmsg_find_attr(nlh, sizeof(struct ifinfomsg), IFLA_AF_SPEC);

	nla_for_each_nested(attr, br_spec, rem)
	{
		__u16 mode;
		// u32 reg = 0;

		if (nla_type(attr) != IFLA_BRIDGE_MODE)
			continue;

		mode = nla_get_u16(attr);
		if (mode == BRIDGE_MODE_VEPA) {
			adapter->flags2 &= ~RNPM_FLAG2_BRIDGE_MODE_VEB;
			wr32(hw,
				 RNPM_DMA_CONFIG,
				 rd32(hw, RNPM_DMA_CONFIG) | DMA_VEB_BYPASS);
		} else if (mode == BRIDGE_MODE_VEB) {
			adapter->flags2 |= RNPM_FLAG2_BRIDGE_MODE_VEB;
			wr32(hw,
				 RNPM_DMA_CONFIG,
				 rd32(hw, RNPM_DMA_CONFIG) & (~DMA_VEB_BYPASS));

		} else
			return -EINVAL;

		e_info(drv,
			   "enabling bridge mode: %s\n",
			   mode == BRIDGE_MODE_VEPA ? "VEPA" : "VEB");
	}

	return 0;
}

#ifdef HAVE_NDO_BRIDGE_GETLINK_NLFLAGS
static int rnpm_ndo_bridge_getlink(struct sk_buff *skb,
								   u32 pid,
								   u32 seq,
								   struct net_device *dev,
								   u32 __maybe_unused filter_mask,
								   int nlflags)
#elif defined(HAVE_BRIDGE_FILTER)
static int rnpm_ndo_bridge_getlink(struct sk_buff *skb,
								   u32 pid,
								   u32 seq,
								   struct net_device *dev,
								   u32 __always_unused filter_mask)
#else
static int rnpm_ndo_bridge_getlink(struct sk_buff *skb,
								   u32 pid,
								   u32 seq,
								   struct net_device *dev)
#endif /* HAVE_NDO_BRIDGE_GETLINK_NLFLAGS */
{
	struct rnpm_adapter *adapter = netdev_priv(dev);
	u16 mode;

	if (!(adapter->flags & RNPM_FLAG_SRIOV_ENABLED))
		return 0;

	if (adapter->flags2 & RNPM_FLAG2_BRIDGE_MODE_VEB)
		mode = BRIDGE_MODE_VEB;
	else
		mode = BRIDGE_MODE_VEPA;

#ifdef HAVE_NDO_DFLT_BRIDGE_GETLINK_VLAN_SUPPORT
	return ndo_dflt_bridge_getlink(
		skb, pid, seq, dev, mode, 0, 0, nlflags, filter_mask, NULL);
#elif defined(HAVE_NDO_BRIDGE_GETLINK_NLFLAGS)
	return ndo_dflt_bridge_getlink(skb, pid, seq, dev, mode, 0, 0, nlflags);
#elif defined(HAVE_NDO_FDB_ADD_VID) || \
	defined NDO_DFLT_BRIDGE_GETLINK_HAS_BRFLAGS
	return ndo_dflt_bridge_getlink(skb, pid, seq, dev, mode, 0, 0);
#else
	return ndo_dflt_bridge_getlink(skb, pid, seq, dev, mode);
#endif /* HAVE_NDO_DFLT_BRIDGE_GETLINK_VLAN_SUPPORT */
}

#endif /* HAVE_BRIDGE_ATTRIBS */

#if defined(HAVE_UDP_ENC_RX_OFFLOAD) || defined(HAVE_VXLAN_RX_OFFLOAD)
void rnpm_clear_udp_tunnel_port(struct rnpm_adapter *adapter)
{
	struct rnpm_hw *hw = &adapter->hw;
	// u32 vxlanctrl;

	if (!(adapter->flags & (RNPM_FLAG_VXLAN_OFFLOAD_CAPABLE)))
		return;

	wr32(hw, RNPM_ETH_VXLAN_PORT, 0);
	adapter->vxlan_port = 0;
	// vxlanctrl = IXGBE_READ_REG(hw, IXGBE_VXLANCTRL) & ~mask;
	// IXGBE_WRITE_REG(hw, IXGBE_VXLANCTRL, vxlanctrl);

	// if (mask & RNPM_VXLANCTRL_VXLAN_UDPPORT_MASK)
	//	adapter->vxlan_port = 0;
	//#ifdef HAVE_UDP_ENC_RX_OFFLOAD
	// if (mask & RNPM_VXLANCTRL_GENEVE_UDPPORT_MASK)
	//	adapter->geneve_port = 0;
	//#endif /* HAVE_UDP_ENC_RX_OFFLOAD */
}
#endif /* HAVE_UDP_ENC_RX_OFFLOAD || HAVE_VXLAN_RX_OFFLOAD */

#ifdef HAVE_UDP_ENC_RX_OFFLOAD
/**
 * rnpm_add_udp_tunnel_port - Get notifications about adding UDP tunnel ports
 * @dev: The port's netdev
 * @ti: Tunnel endpoint information
 **/
__maybe_unused static void rnpm_add_udp_tunnel_port(struct net_device *dev,
													struct udp_tunnel_info *ti)
{
	struct rnpm_adapter *adapter = netdev_priv(dev);
	struct rnpm_hw *hw = &adapter->hw;
	__be16 port = ti->port;
	// u32 port_shift = 0;
	// u32 reg;

	if (ti->sa_family != AF_INET)
		return;

	switch (ti->type) {
		case UDP_TUNNEL_TYPE_VXLAN:
			if (!(adapter->flags & RNPM_FLAG_VXLAN_OFFLOAD_CAPABLE))
				return;

			if (adapter->vxlan_port == port)
				return;

			if (adapter->vxlan_port) {
				netdev_info(dev,
							"VXLAN port %d set, not adding port %d\n",
							ntohs(adapter->vxlan_port),
							ntohs(port));
				return;
			}

			adapter->vxlan_port = port;
			break;
		/*
		case UDP_TUNNEL_TYPE_GENEVE:
			if (!(adapter->flags & IXGBE_FLAG_GENEVE_OFFLOAD_CAPABLE))
				return;

			if (adapter->geneve_port == port)
				return;

			if (adapter->geneve_port) {
				netdev_info(dev,
						"GENEVE port %d set, not adding port %d\n",
						ntohs(adapter->geneve_port),
						ntohs(port));
				return;
			}

			port_shift = RNPM_VXLANCTRL_GENEVE_UDPPORT_SHIFT;
			adapter->geneve_port = port;
			break;
		*/
		default:
			return;
	}

	// reg = IXGBE_READ_REG(hw, IXGBE_VXLANCTRL) | ntohs(port) << port_shift;
	// IXGBE_WRITE_REG(hw, IXGBE_VXLANCTRL, reg);
	wr32(hw, RNPM_ETH_VXLAN_PORT, adapter->vxlan_port);
}

/**
 * rnpm_del_udp_tunnel_port - Get notifications about removing UDP tunnel ports
 * @dev: The port's netdev
 * @ti: Tunnel endpoint information
 **/
__maybe_unused static void rnpm_del_udp_tunnel_port(struct net_device *dev,
													struct udp_tunnel_info *ti)
{
	struct rnpm_adapter *adapter = netdev_priv(dev);
	// u32 port_mask;

	if (ti->type != UDP_TUNNEL_TYPE_VXLAN)
		//    ti->type != UDP_TUNNEL_TYPE_GENEVE)
		return;

	if (ti->sa_family != AF_INET)
		return;

	switch (ti->type) {
		case UDP_TUNNEL_TYPE_VXLAN:
			if (!(adapter->flags & RNPM_FLAG_VXLAN_OFFLOAD_CAPABLE))
				return;

			if (adapter->vxlan_port != ti->port) {
				netdev_info(dev, "VXLAN port %d not found\n", ntohs(ti->port));
				return;
			}

			// port_mask = IXGBE_VXLANCTRL_VXLAN_UDPPORT_MASK;
			break;
		/*
		case UDP_TUNNEL_TYPE_GENEVE:
			if (!(adapter->flags & IXGBE_FLAG_GENEVE_OFFLOAD_CAPABLE))
				return;

			if (adapter->geneve_port != ti->port) {
				netdev_info(dev, "GENEVE port %d not found\n",
						ntohs(ti->port));
				return;
			}

			port_mask = IXGBE_VXLANCTRL_GENEVE_UDPPORT_MASK;
			break;
		*/
		default:
			return;
	}

	rnpm_clear_udp_tunnel_port(adapter);
	adapter->flags2 |= RNPM_FLAG2_UDP_TUN_REREG_NEEDED;
}
#elif defined(HAVE_VXLAN_RX_OFFLOAD)
/**
 * rnpm_add_vxlan_port - Get notifications about VXLAN ports that come up
 * @dev: The port's netdev
 * @sa_family: Socket Family that VXLAN is notifiying us about
 * @port: New UDP port number that VXLAN started listening to
 */
static void __maybe_unused rnpm_add_vxlan_port(struct net_device *dev,
											   sa_family_t sa_family,
											   __be16 port)
{
	struct rnpm_adapter *adapter = netdev_priv(dev);
	struct rnpm_hw *hw = &adapter->hw;

	if (sa_family != AF_INET)
		return;

	if (!(adapter->flags & RNPM_FLAG_VXLAN_OFFLOAD_ENABLE))
		return;

	if (adapter->vxlan_port == port)
		return;

	if (adapter->vxlan_port) {
		netdev_info(dev,
					"Hit Max num of VXLAN ports, not adding port %d\n",
					ntohs(port));
		return;
	}

	adapter->vxlan_port = port;
	wr32(hw, RNPM_ETH_VXLAN_PORT, adapter->vxlan_port);
	// IXGBE_WRITE_REG(hw, IXGBE_VXLANCTRL, ntohs(port));
}

/**
 * rnpm_del_vxlan_port - Get notifications about VXLAN ports that go away
 * @dev: The port's netdev
 * @sa_family: Socket Family that VXLAN is notifying us about
 * @port: UDP port number that VXLAN stopped listening to
 */
static void __maybe_unused rnpm_del_vxlan_port(struct net_device *dev,
											   sa_family_t sa_family,
											   __be16 port)
{
	struct rnpm_adapter *adapter = netdev_priv(dev);

	if (!(adapter->flags & RNPM_FLAG_VXLAN_OFFLOAD_ENABLE))
		return;

	if (sa_family != AF_INET)
		return;

	if (adapter->vxlan_port != port) {
		netdev_info(dev, "Port %d was not found, not deleting\n", ntohs(port));
		return;
	}

	rnpm_clear_udp_tunnel_port(adapter);
	adapter->flags2 |= RNPM_FLAG2_UDP_TUN_REREG_NEEDED;
}
#endif /* HAVE_VXLAN_RX_OFFLOAD */

#ifdef HAVE_NDO_FEATURES_CHECK
#define RNPM_MAX_TUNNEL_HDR_LEN 80
#ifdef NETIF_F_GSO_PARTIAL
#define RNPM_MAX_MAC_HDR_LEN	 127
#define RNPM_MAX_NETWORK_HDR_LEN 511

static netdev_features_t rnpm_features_check(struct sk_buff *skb,
											 struct net_device *dev,
											 netdev_features_t features)
{
	unsigned int network_hdr_len, mac_hdr_len;

	/* Make certain the headers can be described by a context descriptor */
	mac_hdr_len = skb_network_header(skb) - skb->data;
	if (unlikely(mac_hdr_len > RNPM_MAX_MAC_HDR_LEN))
		return features &
			   ~(NETIF_F_HW_CSUM | NETIF_F_SCTP_CRC | NETIF_F_HW_VLAN_CTAG_TX |
				 NETIF_F_TSO | NETIF_F_TSO6);

	network_hdr_len = skb_checksum_start(skb) - skb_network_header(skb);
	if (unlikely(network_hdr_len > RNPM_MAX_NETWORK_HDR_LEN))
		return features & ~(NETIF_F_HW_CSUM | NETIF_F_SCTP_CRC | NETIF_F_TSO |
							NETIF_F_TSO6);

	/* We can only support IPV4 TSO in tunnels if we can mangle the
	 * inner IP ID field, so strip TSO if MANGLEID is not supported.
	 */
	if (skb->encapsulation && !(features & NETIF_F_TSO_MANGLEID)) {
		/*
		if (features & NETIF_F_TSO)
			printk("encapsulation tso set\n");
		if (features & SKB_GSO_UDP_TUNNEL_CSUM)
			printk("encapsulation csum set\n");
		if (features & SKB_GSO_UDP_TUNNEL)
			printk("encapsulation udp tso set\n");
		*/
		features &= ~NETIF_F_TSO;
	}

	return features;
}
#else
static netdev_features_t rnpm_features_check(struct sk_buff *skb,
											 struct net_device *dev,
											 netdev_features_t features)
{
	if (!skb->encapsulation)
		return features;

	if (unlikely(skb_inner_mac_header(skb) - skb_transport_header(skb) >
				 RNPM_MAX_TUNNEL_HDR_LEN))
		return features & ~NETIF_F_CSUM_MASK;

	return features;
}

#endif /* NETIF_F_GSO_PARTIAL */
#endif /* HAVE_NDO_FEATURES_CHECK */

#ifdef HAVE_NET_DEVICE_OPS
const struct net_device_ops rnpm_netdev_ops = {
	.ndo_open = rnpm_open,
	.ndo_stop = rnpm_close,
	.ndo_start_xmit = rnpm_xmit_frame,
	.ndo_set_rx_mode = rnpm_set_rx_mode,
	.ndo_validate_addr = eth_validate_addr,

	.ndo_do_ioctl = rnpm_ioctl,
#ifdef HAVE_RHEL7_NET_DEVICE_OPS_EXT
	/* RHEL7 requires this to be defined to enable extended ops.
	 * RHEL7 uses the function get_ndo_ext to retrieve offsets for
	 * extended fields from with the net_device_ops struct and
	 * ndo_size is checked to determine whether or not
	 * the offset is valid.
	 */
	.ndo_size = sizeof(struct net_device_ops),
#endif
#ifdef HAVE_RHEL7_EXTENDED_MIN_MAX_MTU
	.extended.ndo_change_mtu = rnpm_change_mtu,
#else
	.ndo_change_mtu = rnpm_change_mtu,
#endif
#ifdef HAVE_NDO_GET_STATS64
	.ndo_get_stats64 = rnpm_get_stats64,
#else
	.ndo_get_stats		= rnpm_get_stats,
#endif
	.ndo_tx_timeout = rnpm_tx_timeout,

#ifdef HAVE_RHEL7_NETDEV_OPS_EXT_NDO_TX_MAXRATE
	.extended.ndo_set_tx_maxrate = rnpm_tx_maxrate,
#else
#ifndef NO_TX_MAXRATE
	.ndo_set_tx_maxrate = rnpm_tx_maxrate,
#endif
#endif

	.ndo_set_mac_address = rnpm_set_mac,
#if defined(NETIF_F_HW_VLAN_TX) || defined(NETIF_F_HW_VLAN_CTAG_TX)
	.ndo_vlan_rx_add_vid = rnpm_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid = rnpm_vlan_rx_kill_vid,
#endif

//#ifdef HAVE_NDO_SET_VF_MIN_MAX_TX_RATE
//	.ndo_set_vf_rate = rnpm_ndo_set_vf_bw,
//#else
//	.ndo_set_vf_tx_rate = rnpm_ndo_set_vf_bw,
//#endif /* HAVE_NDO_SET_VF_MIN_MAX_TX_RATE */

//#if defined(CONFIG_PCI_IOV)
//	.ndo_set_vf_mac = rnpm_ndo_set_vf_mac,
//	//.ndo_set_vf_tx_rate	    = rnpm_ndo_set_vf_ring_rate,
//	.ndo_get_vf_config = rnpm_ndo_get_vf_config,
//#endif

/*
#ifdef HAVE_SETUP_TC
#ifdef HAVE_RHEL7_NETDEV_OPS_EXT_NDO_SETUP_TC
	.extended.ndo_setup_tc_rh = __rnpm_setup_tc,
#else
#ifdef NETIF_F_HW_TC
	.ndo_setup_tc		= __rnpm_setup_tc,
#else
	.ndo_setup_tc		= rnpm_setup_tc,
#endif
#endif
#endif
*/

#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = rnpm_netpoll,
#endif

#ifdef HAVE_FDB_OPS
//.ndo_fdb_add		= rnpm_ndo_fdb_add,
#endif

#ifdef HAVE_VLAN_RX_REGISTER
	.ndo_vlan_rx_register = &rnpm_vlan_mode,
#endif
#ifdef HAVE_BRIDGE_ATTRIBS
	.ndo_bridge_setlink = rnpm_ndo_bridge_setlink,
	.ndo_bridge_getlink = rnpm_ndo_bridge_getlink,
#endif
/* we only has one vxlan port for 1 pf */
/* not support for mutiple ports */
/*
#ifdef HAVE_UDP_ENC_RX_OFFLOAD
#ifdef HAVE_RHEL7_NETDEV_OPS_EXT_NDO_UDP_TUNNEL
	.extended.ndo_udp_tunnel_add = rnpm_add_udp_tunnel_port,
	.extended.ndo_udp_tunnel_del = rnpm_del_udp_tunnel_port,
#else
	.ndo_udp_tunnel_add	= rnpm_add_udp_tunnel_port,
	.ndo_udp_tunnel_del	= rnpm_del_udp_tunnel_port,
#endif
#elif defined(HAVE_VXLAN_RX_OFFLOAD)
	.ndo_add_vxlan_port	= rnpm_add_vxlan_port,
	.ndo_del_vxlan_port	= rnpm_del_vxlan_port,
#endif // HAVE_UDP_ENC_RX_OFFLOAD
*/

#ifdef HAVE_NDO_FEATURES_CHECK
	.ndo_features_check = rnpm_features_check,
#endif /* HAVE_NDO_FEATURES_CHECK */

#ifdef HAVE_RHEL6_NET_DEVICE_OPS_EXT

};

/* RHEL6 keeps these operations in a separate structure */
static const struct net_device_ops_ext rnpm_netdev_ops_ext = {
	.size = sizeof(struct net_device_ops_ext),
#endif /* HAVE_RHEL6_NET_DEVICE_OPS_EXT */
#ifdef HAVE_NDO_SET_FEATURES
	.ndo_set_features = rnpm_set_features,
	.ndo_fix_features = rnpm_fix_features,
#endif /* HAVE_NDO_SET_FEATURES */
};
#endif /* HAVE_NET_DEVICE_OPS */

void rnpm_assign_netdev_ops(struct net_device *dev)
{
#ifdef HAVE_NET_DEVICE_OPS
	dev->netdev_ops = &rnpm_netdev_ops;
#ifdef HAVE_RHEL6_NET_DEVICE_OPS_EXT
	set_netdev_ops_ext(dev, &rnpm_netdev_ops_ext);
#endif /* HAVE_RHEL6_NET_DEVICE_OPS_EXT */
#else  /* HAVE_NET_DEVICE_OPS */
	dev->open = &rnpm_open;
	dev->stop = &rnpm_close;
	dev->hard_start_xmit = &rnpm_xmit_frame;
	// dev->get_stats = &rnpm_get_stats;
#ifdef HAVE_SET_RX_MODE
	dev->set_rx_mode = &rnpm_set_rx_mode;
#endif
	dev->set_multicast_list = &rnpm_set_rx_mode;
	dev->set_mac_address = &rnpm_set_mac;
	dev->change_mtu = &rnpm_change_mtu;
	dev->do_ioctl = &rnpm_ioctl;
#ifdef HAVE_TX_TIMEOUT
	dev->tx_timeout = &rnpm_tx_timeout;
#endif
#if defined(NETIF_F_HW_VLAN_TX) || defined(NETIF_F_HW_VLAN_CTAG_TX)
	dev->vlan_rx_register = &rnpm_vlan_mode; // todo
	dev->vlan_rx_add_vid = &rnpm_vlan_rx_add_vid;
	dev->vlan_rx_kill_vid = &rnpm_vlan_rx_kill_vid;
#endif
#ifdef CONFIG_NET_POLL_CONTROLLER
	dev->poll_controller = &rnpm_netpoll;
#endif
#ifdef HAVE_NETDEV_SELECT_QUEUE
	dev->select_queue = &__netdev_pick_tx;
#endif /* HAVE_NETDEV_SELECT_QUEUE */
#endif /* HAVE_NET_DEVICE_OPS */

#ifdef HAVE_RHEL6_NET_DEVICE_EXTENDED
#ifdef HAVE_NDO_BUSY_POLL
	// netdev_extended(dev)->ndo_busy_poll             = rnpm_busy_poll_recv; //
	// todo
#endif /* HAVE_NDO_BUSY_POLL */
#endif /* HAVE_RHEL6_NET_DEVICE_EXTENDED */

	rnpm_set_ethtool_ops(dev);
	dev->watchdog_timeo = 5 * HZ;
}

/**
 * rnpm_wol_supported - Check whether device supports WoL
 * @hw: hw specific details
 * @device_id: the device ID
 * @subdev_id: the subsystem device ID
 *
 * This function is used by probe and ethtool to determine
 * which devices have WoL support
 *
 **/
int rnpm_wol_supported(struct rnpm_adapter *adapter,
					   u16 device_id,
					   u16 subdevice_id)
{
	int is_wol_supported = 0;
#if 0

	struct rnpm_hw *hw = &adapter->hw;
	u16 wol_cap = adapter->eeprom_cap & RNPM_DEVICE_CAPS_WOL_MASK;

	switch (device_id) {
	case RNPM_DEV_ID_N10_SFP:
		/* Only these subdevices could supports WOL */
		switch (subdevice_id) {
		case RNPM_SUBDEV_ID_n10_560FLR:
			/* only support first port */
			if (hw->bus.func != 0)
				break;
		case RNPM_SUBDEV_ID_n10_SP_560FLR:
		case RNPM_SUBDEV_ID_n10_SFP:
		case RNPM_SUBDEV_ID_n10_RNDC:
		case RNPM_SUBDEV_ID_n10_ECNA_DP:
		case RNPM_SUBDEV_ID_n10_LOM_SFP:
			is_wol_supported = 1;
			break;
		}
		break;
	case RNPM_DEV_ID_n10EN_SFP:
		/* Only this subdevice supports WOL */
		switch (subdevice_id) {
		case RNPM_SUBDEV_ID_n10EN_SFP_OCP1:
				is_wol_supported = 1;
				break;
		}
		break;
	case RNPM_DEV_ID_n10_COMBO_BACKPLANE:
		/* All except this subdevice support WOL */
		if (subdevice_id != RNPM_SUBDEV_ID_n10_KX4_KR_MEZZ)
			is_wol_supported = 1;
		break;
	case RNPM_DEV_ID_n10_KX4:
		is_wol_supported = 1;
		break;
	case RNPM_DEV_ID_X540T:
	case RNPM_DEV_ID_X540T1:
		/* check eeprom to see if enabled wol */
		if ((wol_cap == RNPM_DEVICE_CAPS_WOL_PORT0_1) ||
				((wol_cap == RNPM_DEVICE_CAPS_WOL_PORT0) &&
				 (hw->bus.func == 0))) {
			is_wol_supported = 1;
		}
		break;
	}

#endif
	return is_wol_supported;
}

static inline unsigned long rnpm_tso_features(void)
{
	unsigned long features = 0;

#ifdef NETIF_F_TSO
	features |= NETIF_F_TSO;
#endif /* NETIF_F_TSO */
#ifdef NETIF_F_TSO6
	features |= NETIF_F_TSO6;
#endif /* NETIF_F_TSO6 */
#ifdef NETIF_F_GSO_PARTIAL
	features |= NETIF_F_GSO_PARTIAL | RNPM_GSO_PARTIAL_FEATURES;
#endif

	return features;
}

static int rnpm_rm_adpater(struct rnpm_adapter *adapter)
{
	struct net_device *netdev;
	netdev = adapter->netdev;

	rnpm_info("= remove adapter:%s =\n", netdev->name);
	// rnpm_logd(LOG_FUNC_ENTER,"= %s: %s\n", __func__, netdev->name);

	rnpm_dbg_adapter_exit(adapter);

	netif_carrier_off(netdev);

	set_bit(__RNPM_DOWN, &adapter->state);

	/* should clean all tx schedule_work */
#ifndef NO_PTP
	if (module_enable_ptp) {
		// should wait ptp timeout
		while (test_bit(__RNPM_PTP_TX_IN_PROGRESS, &adapter->state)) {
			usleep_range(10000, 20000);
		}
		cancel_work_sync(&adapter->tx_hwtstamp_work);
	}
#endif

	cancel_work_sync(&adapter->service_task);

#ifdef CONFIG_RNPM_DCA
	if (adapter->flags & RNPM_FLAG_DCA_ENABLED) {
		adapter->flags &= ~RNPM_FLAG_DCA_ENABLED;
		dca_remove_requester(&pdev->dev);
		wr32(&adapter->hw + RNPM_DCA_CTRL, 1);
	}
#endif
	rnpm_sysfs_exit(adapter);

	if (netdev->reg_state == NETREG_REGISTERED)
		unregister_netdev(netdev);

	/* set this used in 4 ports in 1pf mode */
	// adapter->netdev = NULL;
	// adapter->rm_mode = true;

	rnpm_clear_interrupt_scheme(adapter);

	rnpm_info("remove %s  complete\n", netdev->name);
	// rnpm_logd(LOG_FUNC_ENTER,"= remove  %s done\n", netdev->name);

	free_netdev(netdev);

	return 0;
}

/* read from hw */
void rnpm_fix_queue_number(struct rnpm_hw *hw)
{
	struct rnpm_adapter *adapter = hw->back;
	u32 count;

	//count = rd32(hw, RNPM_DMA_STATUS);
	//count = (count & DMA_RING_NUM) >> 24;
	count = rnpm_info_tbl[adapter->pf_adapter->board_type]->total_queue_pair_cnts;
	switch (hw->mode) {
		case MODE_NIC_MODE_1PORT:
			count = count;
			break;
		case MODE_NIC_MODE_1PORT_40G:
			count = count;
			break;
		case MODE_NIC_MODE_2PORT:
			count = count / 2;
			break;
		case MODE_NIC_MODE_4PORT:
			count = count / 4;
			break;
	}

	if (count != adapter->max_ring_pair_counts) {
		printk("reset max_ring_pair_counts from %d to %d\n",
			   adapter->max_ring_pair_counts,
			   count);
		adapter->max_ring_pair_counts = count;
	}

#ifdef RNPM_MAX_RINGS
	adapter->max_ring_pair_counts = RNPM_MAX_RINGS;
#endif

	// for test only
	// adapter->max_ring_pair_counts = 1;
	// printk("count now is %d\n", count);
}

int Hamming_weight_1(u32 n)
{
	int count_ = 0;

	while (n != 0) {
		n &= (n - 1);
		count_++;
	}
	return count_;
}

static int check_valid_mode(struct rnpm_pf_adapter *pf_adapter)
{
	int err = 0;
	switch (pf_adapter->board_type) {
		case board_n10: // port_valid should be valid
		case board_n400_4x1G:
			return 0;
		case board_vu440_2x10G:
			// case board_n10_2x10G:
			if (pf_adapter->port_valid & (~0x01))
				err = -1;
			break;
		case board_vu440_4x10G:
			// case board_n10_4x10G:
			if (pf_adapter->port_valid & (~0x03))
				err = -1;
			break;
		case board_vu440_8x10G:
			// case board_n10_8x10G:
			if (pf_adapter->port_valid & (~0x0f))
				err = -1;
			break;
		default:
			rnpm_dbg("board mode error\n");
			err = -1;
			break;
	}

	return err;
}

static int rnpm_init_msix_pf_adapter(struct rnpm_pf_adapter *pf_adapter)
{
	int total_msix_counts;
	int valid_port = Hamming_weight_1(pf_adapter->port_valid);
	int vector, vectors = 0, err, max_msix_counts_per_port;
	int min_vectors = valid_port + 1;
	int remain, i;
#ifdef NO_PCI_MSIX_COUNT
	total_msix_counts = 64;
#else
	total_msix_counts = pci_msix_vec_count(pf_adapter->pdev);
#endif

	// reset max vectors if set by kconfig
#ifdef RNPM_N10M_MSIX_VECTORS
	total_msix_counts = RNPM_N10M_MSIX_VECTORS;
#endif
	if (pf_msix_counts_set) {
		total_msix_counts = pf_msix_counts_set < 5 ? 5 : pf_msix_counts_set;
	}
	total_msix_counts -= 1; // one for mailbox
	total_msix_counts = min_t(int, rnpm_info_tbl[pf_adapter->board_type]->total_queue_pair_cnts,
									   total_msix_counts);
	max_msix_counts_per_port = total_msix_counts / valid_port;

	remain = total_msix_counts - max_msix_counts_per_port * valid_port;

	/* decide max msix for each port */
	for (i = 0; i < MAX_PORT_NUM; i++) {
		/* this port is valid */
		if (pf_adapter->port_valid & (1 << i)) {
			if (remain) {
				pf_adapter->max_msix_counts[i] = max_msix_counts_per_port + 1;
				remain--;
			} else {
				pf_adapter->max_msix_counts[i] = max_msix_counts_per_port;
			}
		}
		pf_adapter->max_msix_counts[i] =
			min_t(int, pf_adapter->max_msix_counts[i], num_online_cpus());
		rnpm_dbg(
			"port %d, max_msix_counts %d\n", i, pf_adapter->max_msix_counts[i]);
		vectors += pf_adapter->max_msix_counts[i];
	}
	pf_adapter->other_irq = 0; // mbx use vector0
	vectors += 1;

	pf_adapter->msix_entries =
		kcalloc(vectors, sizeof(struct msix_entry), GFP_KERNEL);
	if (!pf_adapter->msix_entries) {
		rnpm_err("alloc msix_entries faild!\n");
		return -ENOMEM;
	}

	for (vector = 0; vector < vectors; vector++)
		pf_adapter->msix_entries[vector].entry = vector;

	err = pci_enable_msix_range(
		pf_adapter->pdev, pf_adapter->msix_entries, min_vectors, vectors);

	if (err < 0) {
		rnpm_err("pci_enable_msix faild: req:%d err:%d\n", vectors, err);
		kfree(pf_adapter->msix_entries);
		pf_adapter->msix_entries = NULL;
		return -EINVAL;
	} else if ((err > 0) && (err != vectors)) {
		// should reset msix for each port
		printk("we get msix count %d\n", err);
		total_msix_counts = err;
		total_msix_counts -= 1; // one for mailbox

		max_msix_counts_per_port = total_msix_counts / valid_port;
		remain = total_msix_counts - max_msix_counts_per_port * valid_port;

		/* decide max msix for each port */
		for (i = 0; i < MAX_PORT_NUM; i++) {
			/* this port is valid */
			if (pf_adapter->port_valid & (1 << i)) {
				if (remain) {
					pf_adapter->max_msix_counts[i] =
						max_msix_counts_per_port + 1;
					remain--;
				} else {
					pf_adapter->max_msix_counts[i] = max_msix_counts_per_port;
				}
			}
			pf_adapter->max_msix_counts[i] =
				min_t(int, pf_adapter->max_msix_counts[i], num_online_cpus());
			printk("port %d, max_msix_counts %d\n",
				   i,
				   pf_adapter->max_msix_counts[i]);
			// vectors += pf_adapter->max_msix_counts[i];
		}
	}

	return 0;
}

static int rnpm_rm_msix_pf_adapter(struct rnpm_pf_adapter *pf_adapter)
{
	// free other_irq
	pci_disable_msix(pf_adapter->pdev);
	if (pf_adapter->msix_entries)
		kfree(pf_adapter->msix_entries);
	pf_adapter->msix_entries = 0;
	return 0;
}

int rnpm_set_clause73_autoneg_enable(struct net_device *netdev, int enable)
{
	struct rnpm_adapter *adapter = netdev_priv(netdev);

	if(!adapter){
		return -EINVAL;
	}

	if (test_bit(__RNPM_DOWN, &adapter->state) ||
		test_bit(__RNPM_RESETTING, &adapter->state))
		return -EBUSY;

	return rnpm_hw_set_clause73_autoneg_enable(&adapter->hw, enable);
}
EXPORT_SYMBOL(rnpm_set_clause73_autoneg_enable);

static void rnpm_rm_mbx_irq(struct rnpm_pf_adapter *pf_adapter)
{
	pf_adapter->hw.mbx.ops.configure(
		&pf_adapter->hw,
		pf_adapter->msix_entries[pf_adapter->other_irq].entry,
		false);

#ifdef RNPM_DISABLE_IRQ
	return;
#endif
	free_irq(pf_adapter->msix_entries[pf_adapter->other_irq].vector,
			 pf_adapter);
}

static void rnpm_request_mbx_irq(struct rnpm_pf_adapter *pf_adapter)
{
	int err;

#ifdef RNPM_DISABLE_IRQ
	return;
#endif

	snprintf(pf_adapter->name,
			 20,
			 "rnpm%d%d-other%d",
			 rnpm_is_pf1(pf_adapter->pdev),
			 pf_adapter->bd_number,
			 pf_adapter->other_irq);
	err = request_irq(pf_adapter->msix_entries[pf_adapter->other_irq].vector,
					  rnpm_msix_other,
					  0,
					  pf_adapter->name,
					  pf_adapter);

	pf_adapter->hw.mbx.ops.configure(
		&pf_adapter->hw,
		pf_adapter->msix_entries[pf_adapter->other_irq].entry,
		true);
}

static int rnpm_add_pf_adapter(struct pci_dev *pdev,
		struct rnpm_pf_adapter **ppf_adapter,
		const struct pci_device_id *id)
{
	/*alloc pf_adapter and set it to pdev priv */
	struct rnpm_pf_adapter *pf_adapter;
	int i, err = 0;
#ifdef FT_PADDING
	u32 data;
#endif
	static int pf0_cards_found;
	static int pf1_cards_found;
	struct rnpm_hw *hw;
	struct rnpm_info *ii = rnpm_info_tbl[(int)id->driver_data];

	pf_adapter = devm_kzalloc(&pdev->dev, sizeof(*pf_adapter), GFP_KERNEL);
	if (pf_adapter) {
		*ppf_adapter = pf_adapter;
	} else {
		err = -ENOMEM;
		goto err_pf_alloc;
	}

	pf_adapter->board_type = (int)id->driver_data;
	pf_adapter->pdev = pdev;
	pci_set_drvdata(pdev, pf_adapter);
	/* map pcie bar */
#define RNPM_NIC_BAR 4
	pf_adapter->hw_addr = pcim_iomap(pdev, RNPM_NIC_BAR, 0);
	if (!pf_adapter->hw_addr) {
		err = -EIO;
		goto err_ioremap4;
	}

#if 0
	pf_adapter->hw_bar2 = pcim_iomap(pdev, 2, 0);
	if (!pf_adapter->hw_bar2) {
		err = -EIO;
		goto err_ioremap2;
	}

	pf_adapter->hw_bar0 = pcim_iomap(pdev,0, 0);
	dbg("[bar%d]:%p %llx len=%d MB\n", RNPM_NIC_BAR,
			pf_adapter->hw_addr,
			(unsigned long long)pci_resource_start(pdev, RNPM_NIC_BAR),
			(int)pci_resource_len(pdev, RNPM_NIC_BAR) / 1024 / 1024);
#endif
	if (rnpm_is_pf1(pdev)) {
		pf_adapter->bd_number = pf0_cards_found++;
	} else {
		pf_adapter->bd_number = pf1_cards_found++;
	}
	mutex_init(&pf_adapter->mbx_lock);

	/* mailbox here */
	hw = &pf_adapter->hw;
	hw->hw_addr = pf_adapter->hw_addr;
	hw->pdev = pf_adapter->pdev;
	hw->mbx.lock = &pf_adapter->mbx_lock;
	rnpm_init_mbx_params_pf(hw);
	memcpy(&hw->mbx.ops, ii->mbx_ops, sizeof(hw->mbx.ops));
#ifdef NO_MBX_VERSION
	/* in this mode; we set mode munaly */
	ii->mac = rnp_mac_n10g_x8_10G;
	pf_adapter->adapter_cnt = ii->adapter_cnt;
	if (rnpm_is_pf1(pdev)) {
		pf_adapter->port_valid = port_valid_pf0;
		pf_adapter->port_names = port_names_pf0;
	} else {
		pf_adapter->port_valid = port_valid_pf1;
		pf_adapter->port_names = port_names_pf1;
	}
	// pf_adapter->hw.mac_type = ii->mac;
	pf_adapter->hw.phy_type = PHY_TYPE_10G_BASE_SR;
#else
	spin_lock_init(&pf_adapter->vlan_setup_lock);
	spin_lock_init(&pf_adapter->drop_setup_lock);
	spin_lock_init(&pf_adapter->dummy_setup_lock);
	/* setup priv_flags */
	spin_lock_init(&pf_adapter->priv_flags_lock);

	rnpm_mbx_link_event_enable_nolock(hw, 0);
	if (rnpm_mbx_get_capability(hw, ii)) {
		dev_err(&pdev->dev, "rnp_mbx_get_capablity faild!\n");
		err = -EIO;
		goto err_mbx_capability;
	}
	pf_adapter->port_valid = hw->lane_mask;
	if (hw->port_ids != 0xffffffff)
		pf_adapter->port_names = hw->port_ids; // port_names_pf0;
	else {
		pf_adapter->port_names = port_names_pf0;
	}

	pf_adapter->adapter_cnt = ii->adapter_cnt;
	pf_adapter->hw.axi_mhz  = hw->axi_mhz;

#endif

	/* some global var init here */
	spin_lock_init(&pf_adapter->key_setup_lock);
	pf_adapter->default_rx_ring = 0;
	spin_lock_init(&pf_adapter->mc_setup_lock);

	pf_adapter->mc_location = rnpm_mc_location_nic;

	// fixme n10 can get from device id vu440 cannot
	// pf_adapter->board_type = MODE_TYPE;
	// todo vu440 must decide mode_type

/* vu440 can select board_type manul */
#ifdef UV440_2PF
	pf_adapter->board_type = MODE_TYPE;
#endif
	switch (pf_adapter->hw.mode) {
	case MODE_NIC_MODE_1PORT:
		pf_adapter->mcft_size = 128;
		break;
	case MODE_NIC_MODE_2PORT:
	case MODE_NIC_MODE_4PORT:
		pf_adapter->mcft_size = 8;
		break;
	default:
		pf_adapter->mcft_size = 128;
		break;
	}


	pf_adapter->mc_filter_type = rnpm_mc_filter_type0;
	spin_lock_init(&pf_adapter->vlan_filter_lock);

	for (i = 0; i < MAX_PORT_NUM; i++) {
		/* set this is true */
		pf_adapter->vlan_filter_status[i] = 1;
		/* broadcast bypass should always set */
		pf_adapter->fctrl[i] = RNPM_FCTRL_BROADCASE_BYPASS;
	}
	pf_adapter->vlan_status_true = 1;

	pf_adapter->priv_flags = 0;
#ifdef FT_PADDING
	rnpm_dbg("ft padding status on\n");
	pf_adapter->priv_flags |= RNPM_PRIV_FLAG_PCIE_CACHE_ALIGN_PATCH;
	data = rd32(pf_adapter, RNPM_DMA_CONFIG);
	SET_BIT(padding_enable, data);
	wr32(pf_adapter, RNPM_DMA_CONFIG, data);
#endif

	err = check_valid_mode(pf_adapter);
	if (err)
		goto err_msix;

	err = rnpm_init_msix_pf_adapter(pf_adapter);

	if (err)
		goto err_msix;

	/* reset card */
	err = rnpm_reset_pf(pf_adapter);
	if (err)
		goto err_reset;

	rnpm_request_mbx_irq(pf_adapter);

	/* setup rss key */
	// rnpm_init_rss_key(pf_adapter);

	/* tcam setup */
//	if (pf_adapter->adapter_cnt == 1) {
//		wr32(pf_adapter, RNPM_ETH_TCAM_EN, 1);
//		wr32(pf_adapter, RNPM_TOP_ETH_TCAM_CONFIG_ENABLE, 1);
//		wr32(pf_adapter, RNPM_TCAM_MODE, 2);
//#define TCAM_NUM (4096)
//		for (i = 0; i < TCAM_NUM; i++) {
//			wr32(pf_adapter, RNPM_TCAM_SDPQF(i), 0);
//			wr32(pf_adapter, RNPM_TCAM_DAQF(i), 0);
//			wr32(pf_adapter, RNPM_TCAM_SAQF(i), 0);
//			wr32(pf_adapter, RNPM_TCAM_APQF(i), 0);
//
//			wr32(pf_adapter, RNPM_TCAM_SDPQF_MASK(i), 0);
//			wr32(pf_adapter, RNPM_TCAM_DAQF_MASK(i), 0);
//			wr32(pf_adapter, RNPM_TCAM_SAQF_MASK(i), 0);
//			wr32(pf_adapter, RNPM_TCAM_APQF_MASK(i), 0);
//		}
//		wr32(pf_adapter, RNPM_TCAM_MODE, 1);
//	}
//	// should open all tx
//	rnpm_fix_dma_tx_status(pf_adapter);
	// should init timer service
	timer_setup(&pf_adapter->service_timer, rnpm_pf_service_timer, 0);
	INIT_WORK(&pf_adapter->service_task, rnpm_pf_service_task);

	return 0;
err_reset:
	dev_err(&pdev->dev, "error: err_reset!\n");
	rnpm_rm_mbx_irq(pf_adapter);
	rnpm_rm_msix_pf_adapter(pf_adapter);

err_msix:
	dev_err(&pdev->dev, "error: err_msix!\n");
err_mbx_capability:
	pcim_iounmap(pdev, pf_adapter->hw_addr);
err_ioremap4:
	devm_kfree(&pdev->dev, pf_adapter);
	dev_err(&pdev->dev, "error: err_ioremap4!\n");
err_pf_alloc:
	dev_err(&pdev->dev, "error: err_pf_alloc!\n");
	return err;
}

static int rnpm_rm_pf_adapter(struct pci_dev *pdev,
		struct rnpm_pf_adapter **ppf_adapter)
{
	struct rnpm_pf_adapter *pf_adapter = *ppf_adapter;

	if (pf_adapter->service_timer.function)
		del_timer_sync(&pf_adapter->service_timer);
	cancel_work_sync(&pf_adapter->service_task);

	rnpm_rm_mbx_irq(*ppf_adapter);
	rnpm_rm_msix_pf_adapter(*ppf_adapter);
	pcim_iounmap(pdev, pf_adapter->hw_addr);

	if (pf_adapter) {
		devm_kfree(&pdev->dev, pf_adapter);
	}

	return 0;
}

static int rnpm_add_adpater(struct pci_dev *pdev,
		const struct rnpm_info *ii,
		struct rnpm_adapter **padapter,
		struct rnpm_pf_adapter *pf_adapter,
		int port,
		int msix_offset,
		int port_name)
{
	int i, err = 0;
	struct rnpm_adapter *adapter = NULL;
	struct net_device *netdev;
	struct rnpm_hw *hw;
	unsigned int queues;
	unsigned int indices;
	int adapter_cnt = pf_adapter->adapter_cnt;
	// netdev_features_t hw_enc_features = 0;
#ifndef NETIF_F_GSO_PARTIAL
#ifdef HAVE_NDO_SET_FEATURES
#ifndef HAVE_RHEL6_NET_DEVICE_OPS_EXT
	netdev_features_t hw_features;
#else
	u32 hw_features;
#endif
#endif
#endif /* NETIF_F_GSO_PARTIAL */
	queues = ii->total_queue_pair_cnts / adapter_cnt;
	indices = queues;
	pr_info("====  add adapter queues:%d table %d ===",
			queues,
			pf_adapter->max_msix_counts[port]);

	netdev = alloc_etherdev_mq(sizeof(struct rnpm_adapter), indices);
	if (!netdev) {
		rnpm_err("alloc etherdev errors\n");
		return -ENOMEM;
	}

	adapter = netdev_priv(netdev);
	adapter->netdev = netdev;
	adapter->pdev = pdev;

	adapter->bd_number = pf_adapter->bd_number;
	adapter->port = port;
	adapter->lane = port;

	adapter->max_ring_pair_counts = queues;
	adapter->vector_off = msix_offset;

	adapter->max_msix_counts = pf_adapter->max_msix_counts[port];
	adapter->max_q_vectors = adapter->max_msix_counts;

	// todo maybe usefull for not full ports valid in 8ports mode
	adapter->layer2_count_max = ii->total_layer2_count / adapter_cnt;
	adapter->layer2_offset = adapter->layer2_count_max * adapter->port;
	adapter->tuple_5_count_max = ii->total_tuple5_count / adapter_cnt;
	adapter->tuple_5_offset = adapter->tuple_5_count_max * adapter->port;

	adapter->priv_flags = pf_adapter->priv_flags;

#ifdef RNPM_NAME_BY_LANES
	snprintf(adapter->name,
			 sizeof(netdev->name),
			 "%s%ds%df%d",
			 rnpm_port_name,
			 pdev->bus->number,
			 rnpm_is_pf1(pdev),
			 adapter->port);
#else
	snprintf(adapter->name,
			 sizeof(netdev->name),
			 "%s%ds%df%d",
			 rnpm_port_name,
			 pdev->bus->number,
			 rnpm_is_pf1(pdev),
			 port_name);
#endif
	if (padapter) {
		*padapter = adapter;
		(*padapter)->pf_adapter = pf_adapter;
	}
	hw = &adapter->hw;
	hw->back = adapter;
	hw->nr_lane = hw->num = adapter->port;
	hw->pdev = pdev;
	hw->mode = pf_adapter->hw.mode;
	hw->lane_mask = pf_adapter->hw.lane_mask;
	hw->fw_version = pf_adapter->hw.fw_version;
	hw->fw_uid = pf_adapter->hw.fw_uid;
	// hw->mac_type = pf_adapter->hw.mac_type;
	hw->phy.media_type = hw->phy_type = pf_adapter->hw.phy_type;
	hw->axi_mhz = pf_adapter->hw.axi_mhz;

	/* not so good ? */
	memcpy(&hw->mbx, &pf_adapter->hw.mbx, sizeof(pf_adapter->hw.mbx));
	memcpy(
		&hw->mac.ops, &pf_adapter->hw.mac.ops, sizeof(pf_adapter->hw.mac.ops));

	adapter->msg_enable = netif_msg_init(debug,
										 NETIF_MSG_DRV
#ifdef MSG_PROBE_ENABLE
											 | NETIF_MSG_PROBE
#endif
#ifdef MSG_IFUP_ENABLE
											 | NETIF_MSG_IFUP
#endif
#ifdef MSG_IFDOWN_ENABLE
											 | NETIF_MSG_IFDOWN
#endif
	);

	if (rnpm_is_pf1(pdev)) {
		hw->pfvfnum = PF_NUM(1);
	} else {
		hw->pfvfnum = PF_NUM(0);
	}

	/* adapter hw->mode to decide flags */
	switch (hw->mode) {
		case MODE_NIC_MODE_1PORT_40G:
		case MODE_NIC_MODE_1PORT:
			adapter->flags &= (~RNPM_FLAG_MUTIPORT_ENABLED);
			break;
		case MODE_NIC_MODE_2PORT:
			adapter->flags |= RNPM_FLAG_MUTIPORT_ENABLED;
			break;
		case MODE_NIC_MODE_4PORT:
			adapter->flags |= RNPM_FLAG_MUTIPORT_ENABLED;
			break;
		default:
			adapter->flags |= RNPM_FLAG_MUTIPORT_ENABLED;
			break;
	}

		/* this is relative with netdev name */
		/* in mutiport mode not support this */
		// if (!(adapter->flags & RNPM_FLAG_MUTIPORT_ENABLED)) {
#if !defined(NO_ETHDEV_PORT)
	if (adapter->flags & RNPM_FLAG_MUTIPORT_ENABLED) {
		netdev->dev_port = port_name;
	}

	SET_NETDEV_DEV(netdev, &pdev->dev);
#endif

	adapter->portid_of_card = port_name;
	//}
	/* no use now */
	hw->default_rx_queue = 0;

	hw->rss_type = ii->rss_type;
	hw->hw_addr = pf_adapter->hw_addr;
	hw->ring_msix_base = hw->hw_addr + 0xa0000;

	/* fix queue from hw setup */
	rnpm_fix_queue_number(hw);
	/* get version */
	hw->dma_version = rd32(hw, RNPM_DMA_VERSION);
	pr_info("%s %s: dma versioin:0x%x, nic version:0x%x, pfvfnum:0x%x\n",
			adapter->name,
			pci_name(pdev),
			hw->dma_version,
			rd32(hw, RNPM_TOP_NIC_VERSION),
			hw->pfvfnum);

	rnpm_assign_netdev_ops(netdev);
	strncpy(netdev->name, adapter->name, sizeof(netdev->name) - 1);

	/* Setup hw api */
	memcpy(&hw->mac.ops, ii->mac_ops, sizeof(hw->mac.ops));
	//hw->mac.type = ii->mac;

	/* EEPROM */
	if (ii->eeprom_ops)
		memcpy(&hw->eeprom.ops, ii->eeprom_ops, sizeof(hw->eeprom.ops));

	/* PHY */
	memcpy(&hw->phy.ops, ii->phy_ops, sizeof(hw->phy.ops));
	hw->phy.sfp_type = rnpm_sfp_type_unknown;

	/* PCS */
	memcpy(&hw->pcs.ops, ii->pcs_ops, sizeof(hw->pcs.ops));

#if 0
	/* rnpm_identify_phy_generic will set prtad and mmds properly */
	hw->phy.mdio.prtad = MDIO_PRTAD_NONE;
	hw->phy.mdio.mmds = 0;
	hw->phy.mdio.mode_support = MDIO_SUPPORTS_C45 | MDIO_EMULATE_C22;
	hw->phy.mdio.dev = netdev;
	hw->phy.mdio.mdio_read = rnpm_mdio_read;
	hw->phy.mdio.mdio_write = rnpm_mdio_write;
#endif

	ii->get_invariants(hw);

	/* UV440_2PF support other vectors */
	/* only the last port set this */
	/*
	if (!((adapter->port + 1) % ii->adapter_cnt))
		adapter->num_other_vectors = 1;
	*/

	/* setup the private structure */
	/* this private is used only once */
	err = rnpm_sw_init(adapter);
	if (err)
		goto err_sw_init;

	/* Cache if MNG FW is up so we don't have to read the REG later */
	if (hw->mac.ops.mng_fw_enabled)
		hw->mng_fw_enabled = hw->mac.ops.mng_fw_enabled(hw);

	/* Make it possible the adapter to be woken up via WOL */

	/* reset_hw fills in the perm_addr as well */
	err = hw->mac.ops.reset_hw(hw);
	hw->phy.reset_if_overtemp = false;
	if (err) {
		e_dev_err("HW Init failed: %d\n", err);
		goto err_sw_init;
	}
	/* fix dma tx start */
	// rnpm_fix_dma_tx_status(adapter);

#if defined(CONFIG_PCI_IOV)
	if (adapter->num_other_vectors) {
		/* Mailbox */
		// rnpm_init_mbx_params_pf(hw);
		// memcpy(&hw->mbx.ops, ii->mbx_ops, sizeof(hw->mbx.ops));
		//  fixme
		// rnpm_enable_sriov(adapter);
		// pci_sriov_set_totalvfs(pdev, RNPM_MAX_VF_FUNCTIONS - 1);
	}
#endif

#ifdef HAVE_NETDEVICE_MIN_MAX_MTU
	/* MTU range: 68 - 9710 */
#ifdef HAVE_RHEL7_EXTENDED_MIN_MAX_MTU
	netdev->extended->min_mtu = RNPM_MIN_MTU;
	netdev->extended->max_mtu =
		RNPM_MAX_JUMBO_FRAME_SIZE - (ETH_HLEN + 2 * ETH_FCS_LEN);
#else
	netdev->min_mtu = RNPM_MIN_MTU;
	netdev->max_mtu = RNPM_MAX_JUMBO_FRAME_SIZE - (ETH_HLEN + 2 * ETH_FCS_LEN);
#endif
#endif

#ifdef NETIF_F_GSO_PARTIAL

	if (hw->feature_flags & RNPM_NET_FEATURE_SG)
		netdev->features |= NETIF_F_SG;
	if (hw->feature_flags & RNPM_NET_FEATURE_TSO)
		netdev->features |= NETIF_F_TSO | NETIF_F_TSO6;
	if (hw->feature_flags & RNPM_NET_FEATURE_RX_HASH)
		netdev->features |= NETIF_F_RXHASH;
	if (hw->feature_flags & RNPM_NET_FEATURE_RX_CHECKSUM)
		netdev->features |= NETIF_F_RXCSUM;
	if (hw->feature_flags & RNPM_NET_FEATURE_TX_CHECKSUM) {
		netdev->features |= NETIF_F_HW_CSUM | NETIF_F_SCTP_CRC;
	}

	netdev->features |= NETIF_F_HIGHDMA;
	netdev->gso_partial_features = RNPM_GSO_PARTIAL_FEATURES;
	netdev->features |= NETIF_F_GSO_PARTIAL | RNPM_GSO_PARTIAL_FEATURES;

	netdev->hw_features |= netdev->features;

	if (hw->feature_flags & RNPM_NET_FEATURE_VLAN_FILTER)
		netdev->hw_features |= NETIF_F_HW_VLAN_CTAG_FILTER;
	if (hw->feature_flags & RNPM_NET_FEATURE_VLAN_OFFLOAD) {
		netdev->hw_features |=
			NETIF_F_HW_VLAN_CTAG_RX | NETIF_F_HW_VLAN_CTAG_TX;
	}
	netdev->hw_features |= NETIF_F_RXALL;
	if (hw->feature_flags & RNPM_NET_FEATURE_RX_NTUPLE_FILTER)
		netdev->hw_features |= NETIF_F_NTUPLE;
	if (hw->feature_flags & RNPM_NET_FEATURE_RX_FCS)
		netdev->hw_features |= NETIF_F_RXFCS;

	netdev->vlan_features |= netdev->features | NETIF_F_TSO_MANGLEID;
	netdev->hw_enc_features |= netdev->vlan_features;
	netdev->mpls_features |= NETIF_F_HW_CSUM;

	if (hw->feature_flags & RNPM_NET_FEATURE_VLAN_FILTER)
		netdev->features |= NETIF_F_HW_VLAN_CTAG_FILTER;
	if (hw->feature_flags & RNPM_NET_FEATURE_VLAN_OFFLOAD) {
		netdev->features |= NETIF_F_HW_VLAN_CTAG_RX | NETIF_F_HW_VLAN_CTAG_TX;
	}
	netdev->priv_flags |= IFF_UNICAST_FLT;
	netdev->priv_flags |= IFF_SUPP_NOFCS;

	if (adapter->flags2 & RNPM_FLAG2_RSC_CAPABLE)
		netdev->hw_features |= NETIF_F_LRO;

#else /* NETIF_F_GSO_PARTIAL */

	if (hw->feature_flags & RNPM_NET_FEATURE_SG)
		netdev->features |= NETIF_F_SG;
	if (hw->feature_flags & RNPM_NET_FEATURE_TX_CHECKSUM)
		netdev->features |= NETIF_F_IP_CSUM;

	netdev->features |= NETIF_F_HIGHDMA;

	netdev->features |= NETIF_F_GSO_UDP_TUNNEL | NETIF_F_GSO_UDP_TUNNEL_CSUM;

#ifdef NETIF_F_IPV6_CSUM
	if (hw->feature_flags & RNPM_NET_FEATURE_TX_CHECKSUM)
		netdev->features |= NETIF_F_IPV6_CSUM;
#endif /*  NETIF_F_IPV6_CSUM */

#ifdef NETIF_F_HW_VLAN_CTAG_TX
	if (hw->feature_flags & RNPM_NET_FEATURE_VLAN_FILTER)
		netdev->features |= NETIF_F_HW_VLAN_CTAG_FILTER;
	if (hw->feature_flags & RNPM_NET_FEATURE_VLAN_OFFLOAD) {
		netdev->features |= NETIF_F_HW_VLAN_CTAG_RX | NETIF_F_HW_VLAN_CTAG_TX;
	}
#endif /*  NETIF_F_HW_VLAN_CTAG_TX */
	netdev->features |= rnpm_tso_features();

#ifdef NETIF_F_RXHASH
	if (hw->feature_flags & RNPM_NET_FEATURE_RX_HASH)
		netdev->features |= NETIF_F_RXHASH;
#endif /* NETIF_F_RXHASH */

	if (hw->feature_flags & RNPM_NET_FEATURE_RX_CHECKSUM)
		netdev->features |= NETIF_F_RXCSUM;
#ifdef HAVE_NDO_SET_FEATURES
		/* copy netdev features into list of user selectable features */
#ifndef HAVE_RHEL6_NET_DEVICE_OPS_EXT
	hw_features = netdev->hw_features;
#else  /* HAVE_RHEL6_NET_DEVICE_OPS_EXT */
	hw_features = get_netdev_hw_features(netdev);
#endif /* HAVE_RHEL6_NET_DEVICE_OPS_EXT */
	hw_features |= netdev->features;

	/* give us the option of enabling RSC/LRO later */
	if (adapter->flags2 & RNPM_FLAG2_RSC_CAPABLE)
		hw_features |= NETIF_F_LRO;
#else  /* HAVE_NDO_SET_FEATURES */

#ifdef NETIF_F_GRO
	/* this is only needed on kernels prior to 2.6.39 */
	netdev->features |= NETIF_F_GRO;
#endif /* NETIF_F_GRO */
#endif /* HAVE_NDO_SET_FEATURES */


#ifdef HAVE_NDO_SET_FEATURES
	if (hw->feature_flags & RNPM_NET_FEATURE_TX_CHECKSUM)
		hw_features |= NETIF_F_SCTP_CSUM;
	if (hw->feature_flags & RNPM_NET_FEATURE_RX_NTUPLE_FILTER)
		hw_features |= NETIF_F_NTUPLE;

	hw_features |= NETIF_F_RXALL;

	if (hw->feature_flags & RNPM_NET_FEATURE_RX_FCS)
		hw_features |= NETIF_F_RXFCS;
#endif /* HAVE_NDO_SET_FEATURES */

#ifdef HAVE_NDO_SET_FEATURES
#ifdef HAVE_RHEL6_NET_DEVICE_OPS_EXT
	set_netdev_hw_features(netdev, hw_features);
#else  /* HAVE_RHEL6_NET_DEVICE_OPS_EXT */
	netdev->hw_features = hw_features;
#endif /* HAVE_RHEL6_NET_DEVICE_OPS_EXT */
#endif /* HAVE_NDO_SET_FEATURES */

#ifdef HAVE_NETDEV_VLAN_FEATURES
	if (hw->feature_flags & RNPM_NET_FEATURE_SG)
		netdev->vlan_features |= NETIF_F_SG;
	if (hw->feature_flags & RNPM_NET_FEATURE_TX_CHECKSUM)
		netdev->vlan_features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;
	if (hw->feature_flags & RNPM_NET_FEATURE_TSO)
		netdev->vlan_features |= NETIF_F_TSO | NETIF_F_TSO6;
#endif /* HAVE_NETDEV_VLAN_FEATURES */

#ifdef HAVE_ENCAP_CSUM_OFFLOAD
	netdev->hw_enc_features |= NETIF_F_SG;
#endif /* HAVE_ENCAP_CSUM_OFFLOAD */

#ifdef HAVE_VXLAN_RX_OFFLOAD
	if (hw->feature_flags & RNPM_NET_FEATURE_TX_CHECKSUM) {
		netdev->hw_enc_features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;
	}
#endif /* HAVE_VXLAN_RX_OFFLOAD */

#endif /* NETIF_F_GSO_PARTIAL */

#ifdef IFF_UNICAST_FLT
	netdev->priv_flags |= IFF_UNICAST_FLT;
#endif /* IFF_UNICAST_FLT */
#ifdef IFF_SUPP_NOFCS
	netdev->priv_flags |= IFF_SUPP_NOFCS;
#endif /* IFF_SUPP_NOFCS */

#ifdef NET_FEATURE_DCB
#ifdef CONFIG_DCB
	rnpm_dcb_init(netdev, adapter);
#endif /* CONFIG_DCB */
#endif /* NET_FEATURE_DCB */

	if (adapter->flags2 & RNPM_FLAG2_RSC_ENABLED)
		netdev->features |= NETIF_F_LRO;

#if 0
	/* make sure the EEPROM is good */
	if (hw->eeprom.ops.validate_checksum(hw, NULL) < 0) {
		e_dev_err("The EEPROM Checksum Is Not Valid\n");
		err = -EIO;
		goto err_sw_init;
	}
#endif
	eth_hw_addr_set(netdev, hw->mac.perm_addr);
#ifdef ETHTOOL_GPERMADDR
	memcpy(netdev->perm_addr, hw->mac.perm_addr, netdev->addr_len);
#endif
	pr_info("set dev_addr:%pM\n", netdev->dev_addr);

	if (!is_valid_ether_addr(netdev->dev_addr)) {
		e_dev_err("invalid MAC address\n");
		err = -EIO;
		/* handle error not corect */
		goto err_sw_init;
	}
	ether_addr_copy(hw->mac.addr, hw->mac.perm_addr);

	timer_setup(&adapter->service_timer, rnpm_service_timer, 0);

#ifndef NO_PTP
	if (module_enable_ptp) {
		adapter->flags2 |= RNPM_FLAG2_PTP_ENABLED;
		if (adapter->flags2 & RNPM_FLAG2_PTP_ENABLED) {
			adapter->tx_timeout_factor = 10;
			INIT_WORK(&adapter->tx_hwtstamp_work, rnpm_tx_hwtstamp_work);
		}
	}
#endif

	INIT_WORK(&adapter->service_task, rnpm_service_task);
	clear_bit(__RNPM_SERVICE_SCHED, &adapter->state);

	err = rnpm_init_interrupt_scheme(adapter);
	if (err)
		goto err_interrupt_scheme;

	/* WOL not supported for all devices */
	adapter->wol = 0;
#if 0
	hw->eeprom.ops.read(hw, 0x2c, &adapter->eeprom_cap);
	hw->wol_enabled = rnpm_wol_supported(adapter, pdev->device,
			pdev->subsystem_device);
	if (hw->wol_enabled)
		adapter->wol = RNPM_WUFC_MAG;

	device_set_wakeup_enable(&adapter->pdev->dev, adapter->wol);

	/* save off EEPROM version number */
	hw->eeprom.ops.read(hw, 0x2e, &adapter->eeprom_verh);
	hw->eeprom.ops.read(hw, 0x2d, &adapter->eeprom_verl);

	/* pick up the PCI bus settings for reporting later */
	hw->mac.ops.get_bus_info(hw);

	/* print bus type/speed/width info */
	e_dev_info("(PCI Express:%s:%s) %pM\n",
			(hw->bus.speed == rnpm_bus_speed_8000 ? "8.0GT/s" :
			 hw->bus.speed == rnpm_bus_speed_5000 ? "5.0GT/s" :
			 hw->bus.speed == rnpm_bus_speed_2500 ? "2.5GT/s" :
			 "Unknown"),
			(hw->bus.width == rnpm_bus_width_pcie_x8 ? "Width x8" :
			 hw->bus.width == rnpm_bus_width_pcie_x4 ? "Width x4" :
			 hw->bus.width == rnpm_bus_width_pcie_x1 ? "Width x1" :
			 "Unknown"),
			netdev->dev_addr);

	err = rnpm_read_pba_string_generic(hw, part_str, RNPM_PBANUM_LENGTH);
	if (err)
		strncpy(part_str, "Unknown", RNPM_PBANUM_LENGTH);
	if (rnpm_is_sfp(hw) && hw->phy.sfp_type != rnpm_sfp_type_not_present)
		e_dev_info("MAC: %d, PHY: %d, SFP+: %d, PBA No: %s\n",
				hw->mac.type, hw->phy.type, hw->phy.sfp_type,
				part_str);
	else
		e_dev_info("MAC: %d, PHY: %d, PBA No: %s\n",
				hw->mac.type, hw->phy.type, part_str);

	if (hw->bus.width <= rnpm_bus_width_pcie_x4) {
		e_dev_warn("PCI-Express bandwidth available for this card is "
				"not sufficient for optimal performance.\n");
		e_dev_warn("For optimal performance a x8 PCI-Express slot "
				"is required.\n");
	}

#endif
	/* reset the hardware with the new settings */
	err = hw->mac.ops.start_hw(hw);

	strncpy(netdev->name, adapter->name, sizeof(netdev->name) - 1);

	if (fix_eth_name) {
		if (!(adapter->flags & RNPM_FLAG_MUTIPORT_ENABLED)) {
			snprintf(adapter->name,
					 sizeof(netdev->name),
					 "rnp%d%d",
					 rnpm_is_pf1(pdev),
					 adapter->bd_number);
		} else {
			snprintf(adapter->name,
					 sizeof(netdev->name),
					 "rnpm%d%d%d",
					 rnpm_is_pf1(pdev),
					 adapter->bd_number,
					 adapter->port);
		}
		strncpy(netdev->name, adapter->name, sizeof(netdev->name) - 1);
	} else {
#ifdef ASSIN_PDEV
		strcpy(netdev->name, "eth%d");
#else
		if (!(adapter->flags & RNPM_FLAG_MUTIPORT_ENABLED))
			strcpy(netdev->name, "eth%d");

#endif
		/* multiports we can't support eth%d */
	}

	err = register_netdev(netdev);
	if (err) {
		rnpm_err("register_netdev faild! err code %x \n", err);
		goto err_register;
	}

	/* power down the optics for n10 SFP+ fiber */
	if (hw->mac.ops.disable_tx_laser)
		hw->mac.ops.disable_tx_laser(hw);

	/* carrier off reporting is important to ethtool even BEFORE open */
	netif_carrier_off(netdev);

#ifdef CONFIG_RNPM_DCA
	if (dca_add_requester(&pdev->dev) == 0) {
		adapter->flags |= RNPM_FLAG_DCA_ENABLED;
		rnpm_setup_dca(adapter);
	}
#endif

	if (adapter->flags & RNPM_FLAG_SRIOV_ENABLED) {
		e_info(probe, "IOV is enabled with %d VFs\n", adapter->num_vfs);
		for (i = 0; i < adapter->num_vfs; i++)
			rnpm_vf_configuration(pdev, (i | 0x10000000));
	}

	if (rnpm_sysfs_init(adapter))
		e_err(probe, "failed to allocate sysfs resources\n");

	rnpm_dbg_adapter_init(adapter);

	/* Need link setup for MNG FW, else wait for RNPM_UP */
	// if (hw->mng_fw_enabled && hw->mac.ops.setup_link)
	//     hw->mac.ops.setup_link(hw, RNPM_LINK_SPEED_10GB_FULL |
	//     RNPM_LINK_SPEED_1GB_FULL, true);

	return 0;
	e_dev_err("error: unregister_netdev\n");
	unregister_netdev(netdev);
err_register:
	e_dev_err("error: err_register err=%d\n", err);
	rnpm_clear_interrupt_scheme(adapter);
err_interrupt_scheme:
	e_dev_err("error: err_interrupt_scheme err=%d\n", err);
	if (adapter->service_timer.function)
		del_timer_sync(&adapter->service_timer);
err_sw_init:
	e_dev_err("error: err_sw_init err=%d\n", err);
	/* cannot handle right */
	rnpm_disable_sriov(adapter);
	adapter->flags2 &= ~RNPM_FLAG2_SEARCH_FOR_SFP;
	// err_ioremap:
	free_netdev(netdev);
	return err;
}

/**
 * rnpm_probe - Device Initialization Routine
 * @pdev: PCI device information struct
 * @ent: entry in rnpm_pci_tbl
 *
 * Returns 0 on success, negative on failure
 *
 * rnpm_probe initializes an adapter identified by a pci_dev structure.
 * The OS initialization, configuring of the adapter private structure,
 * and a hardware reset occur.
 **/
static int rnpm_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	// struct net_device *netdev;
	// struct rnpm_adapter *adapter;
	struct rnpm_pf_adapter *pf_adapter;
	const struct rnpm_info *ii;
	int i = 0, vector_idx = 0, err;
	int vector_idx_new, port_name, port_name_new, lane_num;
	int valid_port;
	u32 port_valid;

	/* Catch broken hardware that put the wrong VF device ID in
	 * the PCIe SR-IOV capability.
	 */
	if (pdev->is_virtfn) {
		WARN(1,
			 KERN_ERR "%s (%hx:%hx) should not be a VF!\n",
			 pci_name(pdev),
			 pdev->vendor,
			 pdev->device);
		return -EINVAL;
	}
	err = pci_enable_device_mem(pdev);
	if (err) {
		dev_err(&pdev->dev, "pci_enable_device_mem failed 0x%x\n", err);
		return err;
	}
	if (!dma_set_mask(&pdev->dev, DMA_BIT_MASK(56)) &&
		!dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(56))) {
		enable_hi_dma = 1;
	} else {
		err = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
		if (err) {
			err = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
			if (err) {
				dev_err(&pdev->dev, "No usable DMA configuration, aborting\n");
				goto err_dma;
			}
		}
		enable_hi_dma = 0;
	}
	// err = pci_request_selected_regions(pdev, pci_select_bars(pdev,
	//			IORESOURCE_MEM), rnpm_driver_name);
	err = pci_request_mem_regions(pdev, rnpm_driver_name);
	if (err) {
		dev_err(&pdev->dev, "pci_request_selected_regions failed 0x%x\n", err);
		goto err_pci_reg;
	}
	pci_enable_pcie_error_reporting(pdev);
	pci_set_master(pdev);
	pci_save_state(pdev);
	err = rnpm_add_pf_adapter(pdev, &pf_adapter, id);
	if (err) {
		dev_err(&pdev->dev, "rnpm_add_pf_adapter failed 0x%x\n", err);
		goto err_pf_adpater;
	}
	ii = rnpm_info_tbl[pf_adapter->board_type];
	//pf_adapter->adapter_cnt = ii->adapter_cnt;
	memset(pf_adapter->adapter, 0, sizeof(pf_adapter->adapter));
	if (pf_adapter->adapter_cnt > MAX_PORT_NUM) {
		dev_err(&pdev->dev, "invalid adapt cnt:%d\n", pf_adapter->adapter_cnt);
		return -EIO;
	}
	valid_port = Hamming_weight_1(pf_adapter->port_valid);
	port_valid = pf_adapter->port_valid;
	do {
		port_name = -1;
		vector_idx = 1;
		lane_num = 0;
		vector_idx_new = 1;
		// get the min port name
		for (i = 0, vector_idx = 1; i < pf_adapter->adapter_cnt; i++) {
			if (port_valid & (1 << i)) {
				port_name_new = (pf_adapter->port_names >> (i * 8)) & 0xff;
				if ((port_name == -1) || (port_name > port_name_new)) {
					// get the current port name
					port_name = port_name_new;
					lane_num = i;
					vector_idx_new = vector_idx;
				}
			}
			vector_idx += pf_adapter->max_msix_counts[i];
		}
		// do register
		err = rnpm_add_adpater(pdev,
				ii,
				&pf_adapter->adapter[lane_num],
				pf_adapter,
				lane_num,
				vector_idx_new,
				port_name);
		if (err) {
			dev_err(&pdev->dev, "add adpater %d failed, err=%d\n", i, err);
			goto err_adpater;
		}

		// mask valid
		port_valid &= (~(1 << lane_num));
		valid_port--;

	} while (valid_port > 0);

#ifndef NO_MBX_VERSION
	rnpm_mbx_link_event_enable_nolock(&pf_adapter->hw, 1);
#endif
	mod_timer(&pf_adapter->service_timer, HZ + jiffies);

	return 0;

err_adpater:
	dev_err(&pdev->dev, "error: err_adpater!\n");
	rnpm_rm_pf_adapter(pdev, &pf_adapter);
err_pf_adpater:
	pci_release_mem_regions(pdev);
err_dma:
err_pci_reg:
	dev_err(&pdev->dev, "probe err = %d!\n", err);
	return err;
}

/**
 * rnpm_remove - Device Removal Routine
 * @pdev: PCI device information struct
 *
 * rnpm_remove is called by the PCI subsystem to alert the driver
 * that it should release a PCI device.  The could be caused by a
 * Hot-Plug event, or because the driver is going to be removed from
 * memory.
 **/
static void rnpm_remove(struct pci_dev *pdev)
{
	struct rnpm_pf_adapter *pf_adapter = pci_get_drvdata(pdev);
	int i;

	// disable link event to rc
	rnpm_mbx_link_event_enable_nolock(&pf_adapter->hw, 0);

	while (mutex_lock_interruptible(pf_adapter->hw.mbx.lock)) {
		usleep_range(10000, 20000);
	}
	set_bit(__RNPM_DOWN, &pf_adapter->state);
	mutex_unlock(pf_adapter->hw.mbx.lock);

	/* must rm in this order */
	for (i = pf_adapter->adapter_cnt - 1; i >= 0; i--) {
		if (rnpm_port_is_valid(pf_adapter, i)) {
			if (pf_adapter->adapter[i])
				rnpm_rm_adpater(pf_adapter->adapter[i]);
		}
	}

	// disbale mbx-irq
	if (pf_adapter->hw.mbx.ops.configure) {
		pf_adapter->hw.mbx.ops.configure(&pf_adapter->hw, 0, false);
	}

#ifdef CONFIG_PCI_IOV
	/*
	 * Only disable SR-IOV on unload if the user specified the now
	 * deprecated max_vfs module parameter.
	 */
	/* sriov not consider now */
	/*
	if (max_vfs)
		rnpm_disable_sriov(adapter);
	*/
#endif

	rnpm_rm_pf_adapter(pdev, &pf_adapter);
	// pci_release_selected_regions(pdev, pci_select_bars(pdev,
	//			IORESOURCE_MEM));
	dma_free_coherent(&pdev->dev,
						pf_adapter->hw.mbx.reply_dma_size,
						pf_adapter->hw.mbx.reply_dma,
						pf_adapter->hw.mbx.reply_dma_phy);
	pci_release_mem_regions(pdev);
	pci_disable_pcie_error_reporting(pdev);
	pci_disable_device(pdev);
}

/**
 * rnpm_io_error_detected - called when PCI error is detected
 * @pdev: Pointer to PCI device
 * @state: The current pci connection state
 *
 * This function is called after a PCI bus error affecting
 * this device has been detected.
 */
static pci_ers_result_t rnpm_io_error_detected(struct pci_dev *pdev,
											   pci_channel_state_t state)
{
	// struct rnpm_adapter *adapter = pci_get_drvdata(pdev);
	// struct net_device *netdev = adapter->netdev;

#if 0

#ifdef CONFIG_PCI_IOV
	struct pci_dev *bdev, *vfdev;
	u32 dw0, dw1, dw2, dw3;
	int vf, pos;
	u16 req_id, pf_func;

	if (adapter->hw.mac.type == rnpm_mac_82598EB ||
			adapter->num_vfs == 0)
		goto skip_bad_vf_detection;

	bdev = pdev->bus->self;
	while (bdev && (pci_pcie_type(bdev) != PCI_EXP_TYPE_ROOT_PORT))
		bdev = bdev->bus->self;

	if (!bdev)
		goto skip_bad_vf_detection;

	pos = pci_find_ext_capability(bdev, PCI_EXT_CAP_ID_ERR);
	if (!pos)
		goto skip_bad_vf_detection;

	pci_read_config_dword(bdev, pos + PCI_ERR_HEADER_LOG, &dw0);
	pci_read_config_dword(bdev, pos + PCI_ERR_HEADER_LOG + 4, &dw1);
	pci_read_config_dword(bdev, pos + PCI_ERR_HEADER_LOG + 8, &dw2);
	pci_read_config_dword(bdev, pos + PCI_ERR_HEADER_LOG + 12, &dw3);

	req_id = dw1 >> 16;
	/* On the n10 if bit 7 of the requestor ID is set then it's a VF */
	if (!(req_id & 0x0080))
		goto skip_bad_vf_detection;

	pf_func = req_id & 0x01;
	if ((pf_func & 1) == (pdev->devfn & 1)) {
		unsigned int device_id;

		vf = (req_id & 0x7F) >> 1;
		e_dev_err("VF %d has caused a PCIe error\n", vf);
		e_dev_err("TLP: dw0: %8.8x\tdw1: %8.8x\tdw2: "
				"%8.8x\tdw3: %8.8x\n",
				dw0, dw1, dw2, dw3);
		device_id = RNPM_DEV_ID_N10_VF;

		/* Find the pci device of the offending VF */
		vfdev = pci_get_device(PCI_VENDOR_ID_MUCSE, device_id, NULL);
		while (vfdev) {
			if (vfdev->devfn == (req_id & 0xFF))
				break;
			vfdev = pci_get_device(PCI_VENDOR_ID_MUCSE, device_id, vfdev);
		}
		/*
		 * There's a slim chance the VF could have been hot plugged,
		 * so if it is no longer present we don't need to issue the
		 * VFLR.  Just clean up the AER in that case.
		 */
		if (vfdev) {
			e_dev_err("Issuing VFLR to VF %d\n", vf);
			pci_write_config_dword(vfdev, 0xA8, 0x00008000);
			/* Free device reference count */
			pci_dev_put(vfdev);
		}

		pci_cleanup_aer_uncorrect_error_status(pdev);
	}

	/*
	 * Even though the error may have occurred on the other port
	 * we still need to increment the vf error reference count for
	 * both ports because the I/O resume function will be called
	 * for both of them.
	 */
	adapter->vferr_refcount++;

	return PCI_ERS_RESULT_RECOVERED;

skip_bad_vf_detection:
#endif /* CONFIG_PCI_IOV */
	netif_device_detach(netdev);

	if (state == pci_channel_io_perm_failure)
		return PCI_ERS_RESULT_DISCONNECT;

	if (netif_running(netdev))
		rnpm_down(adapter);
	pci_disable_device(pdev);
#endif
	/* Request a slot reset. */
	return PCI_ERS_RESULT_NEED_RESET;
}

/**
 * rnpm_io_slot_reset - called after the pci bus has been reset.
 * @pdev: Pointer to PCI device
 *
 * Restart the card from scratch, as if from a cold-boot.
 */
static pci_ers_result_t rnpm_io_slot_reset(struct pci_dev *pdev)
{
	pci_ers_result_t result = PCI_ERS_RESULT_NONE;
#if 0
	struct rnpm_adapter *adapter = pci_get_drvdata(pdev);
	int err;

	if (pci_enable_device_mem(pdev)) {
		e_err(probe, "Cannot re-enable PCI device after reset.\n");
		result = PCI_ERS_RESULT_DISCONNECT;
	} else {
		pci_set_master(pdev);
		pci_restore_state(pdev);
		pci_save_state(pdev);

		pci_wake_from_d3(pdev, false);

		rnpm_reset(adapter);
		//RNPM_WRITE_REG(&adapter->hw, RNPM_WUS, ~0);
		result = PCI_ERS_RESULT_RECOVERED;
	}

	err = pci_cleanup_aer_uncorrect_error_status(pdev);
	if (err) {
		e_dev_err("pci_cleanup_aer_uncorrect_error_status "
				"failed 0x%0x\n", err);
		/* non-fatal, continue */
	}
#endif

	return result;
}

#ifdef CONFIG_PM
static int rnpm_resume(struct pci_dev *pdev)
{
	struct rnpm_pf_adapter *pf_adapter = pci_get_drvdata(pdev);
	struct rnpm_adapter *adapter;
	struct net_device *netdev;
	int i;
	u32 err;

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	/*
	 * pci_restore_state clears dev->state_saved so call
	 * pci_save_state to restore it.
	 */
	pci_save_state(pdev);

	err = pci_enable_device_mem(pdev);
	if (err) {
		dev_err(&pdev->dev, "Cannot enable PCI device from suspend\n");
		return err;
	}

	pci_set_master(pdev);

	pci_wake_from_d3(pdev, false);

	err = rnpm_init_msix_pf_adapter(pf_adapter);

	rnpm_request_mbx_irq(pf_adapter);

	if (pf_adapter->adapter_cnt == 1) {
		wr32(pf_adapter, RNPM_ETH_TCAM_EN, 1);
		wr32(pf_adapter, RNPM_TOP_ETH_TCAM_CONFIG_ENABLE, 1);
		wr32(pf_adapter, RNPM_TCAM_MODE, 2);
#define TCAM_NUM (4096)
		for (i = 0; i < TCAM_NUM; i++) {
			wr32(pf_adapter, RNPM_TCAM_SDPQF(i), 0);
			wr32(pf_adapter, RNPM_TCAM_DAQF(i), 0);
			wr32(pf_adapter, RNPM_TCAM_SAQF(i), 0);
			wr32(pf_adapter, RNPM_TCAM_APQF(i), 0);

			wr32(pf_adapter, RNPM_TCAM_SDPQF_MASK(i), 0);
			wr32(pf_adapter, RNPM_TCAM_DAQF_MASK(i), 0);
			wr32(pf_adapter, RNPM_TCAM_SAQF_MASK(i), 0);
			wr32(pf_adapter, RNPM_TCAM_APQF_MASK(i), 0);
		}
		wr32(pf_adapter, RNPM_TCAM_MODE, 1);
	}
	// should open all tx
	rnpm_fix_dma_tx_status(pf_adapter);

	for (i = 0; i < pf_adapter->adapter_cnt; i++) {
		if (!rnpm_port_is_valid(pf_adapter, i))
			continue;

		adapter = pf_adapter->adapter[i];
		netdev = adapter->netdev;
		rnpm_reset(adapter);
		rtnl_lock();
		err = rnpm_init_interrupt_scheme(adapter);
		if (!err && netif_running(netdev))
			err = rnpm_open(netdev);

		rtnl_unlock();
		netif_device_attach(netdev);
	}

	// RNPM_WRITE_REG(&adapter->hw, RNPM_WUS, ~0);

	if (err)
		return err;

	return 0;
}
#endif /* CONFIG_PM */

__maybe_unused static int __rnpm_shutdown(struct pci_dev *pdev,
										  bool *enable_wake)
{
	struct rnpm_pf_adapter *pf_adapter = pci_get_drvdata(pdev);
	struct rnpm_adapter *adapter;
	int i;
	struct net_device *netdev;
	struct rnpm_hw *hw;
	u32 wufc;
#ifdef CONFIG_PM
	int retval = 0;
#endif

	for (i = pf_adapter->adapter_cnt - 1; i >= 0; i--) {
		if (!rnpm_port_is_valid(pf_adapter, i))
			continue;

		adapter = pf_adapter->adapter[i];
		netdev = adapter->netdev;
		hw = &adapter->hw;
		rtnl_lock();
		netif_device_detach(netdev);
		if (netif_running(netdev)) {
			rnpm_down(adapter);
			rnpm_free_irq(adapter);
			rnpm_free_all_tx_resources(adapter);
			rnpm_free_all_rx_resources(adapter);
		}
		rtnl_unlock();
		/* free msix */
		//adapter->rm_mode = true;
		rnpm_clear_interrupt_scheme(adapter);

		wufc = adapter->wol;
		if (wufc) {
			rnpm_set_rx_mode(netdev);

			/* enable the optics for n10 SFP+ fiber as we can WoL */
			if (hw->mac.ops.enable_tx_laser)
				hw->mac.ops.enable_tx_laser(hw);

			/* turn on all-multi mode if wake on multicast is enabled */

		} else {
		}
	}

#ifdef CONFIG_PM
	retval = pci_save_state(pdev);
	if (retval)
		return retval;

#endif

	pci_wake_from_d3(pdev, false);
	*enable_wake = false;
	// pci_wake_from_d3(pdev, !!wufc);
	//*enable_wake = !!wufc;

	// rnpm_release_hw_control(adapter);
	rnpm_rm_mbx_irq(pf_adapter);
	rnpm_rm_msix_pf_adapter(pf_adapter);

	pci_disable_device(pdev);

	return 0;
}

#ifdef CONFIG_PM
static int rnpm_suspend(struct pci_dev *pdev, pm_message_t state)
{
	int retval;
	bool wake;

	retval = __rnpm_shutdown(pdev, &wake);
	if (retval)
		return retval;

	if (wake) {
		pci_prepare_to_sleep(pdev);
	} else {
		pci_wake_from_d3(pdev, false);
		pci_set_power_state(pdev, PCI_D3hot);
	}

	return 0;
}
#endif /* CONFIG_PM */

__maybe_unused static void rnpm_shutdown(struct pci_dev *pdev)
{
	bool wake;

	__rnpm_shutdown(pdev, &wake);

	if (system_state == SYSTEM_POWER_OFF) {
		pci_wake_from_d3(pdev, wake);
		pci_set_power_state(pdev, PCI_D3hot);
	}
}

/**
 * rnpm_io_resume - called when traffic can start flowing again.
 * @pdev: Pointer to PCI device
 *
 * This callback is called when the error recovery driver tells us that
 * its OK to resume normal operation.
 */
static void rnpm_io_resume(struct pci_dev *pdev)
{
	struct rnpm_adapter *adapter = pci_get_drvdata(pdev);
	struct net_device *netdev = adapter->netdev;


#ifdef CONFIG_PCI_IOV
	if (adapter->vferr_refcount) {
		e_info(drv, "Resuming after VF err\n");
		adapter->vferr_refcount--;
		return;
	}

#endif
	if (netif_running(netdev))
		rnpm_up(adapter);

	netif_device_attach(netdev);
}

static const struct pci_error_handlers rnpm_err_handler = {
	.error_detected = rnpm_io_error_detected,
	.slot_reset = rnpm_io_slot_reset,
	.resume = rnpm_io_resume,
};

static struct pci_driver rnpm_driver = {
	.name = rnpm_driver_name,
	.id_table = rnpm_pci_tbl,
	.probe = rnpm_probe,
	.remove = rnpm_remove,
#ifdef CONFIG_PM
	.suspend = rnpm_suspend,
	.resume = rnpm_resume,
#endif
	//.shutdown = rnpm_shutdown,
	// .sriov_configure = rnpm_pci_sriov_configure,
	.err_handler = &rnpm_err_handler};

static int __init rnpm_init_module(void)
{
	int ret;

	pr_info("%s - version %s\n", rnpm_driver_string, rnpm_driver_version);
	pr_info("%s\n", rnpm_copyright);
	rnpm_dbg_init();
	ret = pci_register_driver(&rnpm_driver);
	if (ret) {
		rnpm_dbg_exit();
		return ret;
	}

	return 0;
}
module_init(rnpm_init_module);

static void __exit rnpm_exit_module(void)
{
	pci_unregister_driver(&rnpm_driver);

	rnpm_dbg_exit();

	rcu_barrier(); /* Wait for completion of call_rcu()'s */
}

module_exit(rnpm_exit_module);
