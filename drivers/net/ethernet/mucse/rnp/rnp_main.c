#include <linux/capability.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/pci.h>
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
#include <linux/bitops.h>
#include <linux/prefetch.h>
#include <linux/netdevice.h>
#ifdef NETIF_F_HW_TC
#include <net/tc_act/tc_gact.h>
#include <net/tc_act/tc_mirred.h>
#include <net/pkt_cls.h>
#endif

#include "rnp_tc_u32_parse.h"
#include "rnp_common.h"
#include "rnp.h"
#include "rnp_dcb.h"
#include "rnp_sriov.h"
#include "rnp_ptp.h"

//#define NO_MBX_VESION

#ifdef HAVE_UDP_ENC_RX_OFFLOAD
#include <net/vxlan.h>
#include <net/udp_tunnel.h>
#endif /* HAVE_UDP_ENC_RX_OFFLOAD */
#ifdef HAVE_VXLAN_RX_OFFLOAD
#include <net/vxlan.h>
#endif /* HAVE_VXLAN_RX_OFFLOAD */

// for test
#ifdef CONFIG_ARM64
#define NO_BQL_TEST
#endif

#define USE_NUMA_MEMORY
//#define SUPPORT_IRQ_AFFINITY_CHANGE
#define REDUCE_TX_IRQ_MISS

char rnp_driver_name[] = "rnp";
static const char rnp_driver_string[] =
	"mucse 1/10/25/40 Gigabit PCI Express Network Driver";
static char rnp_default_device_descr[] =
	"mucse(R) 2port 10/40 Gigabit Network Connection";
#define DRV_VERSION "0.1.6.rc45"
const char rnp_driver_version[] = DRV_VERSION;
static const char rnp_copyright[] =
	"Copyright (c) 2020-2022 mucse Corporation.";

extern struct rnp_info rnp_vu440_2ports_info;
extern struct rnp_info rnp_n10_2ports_info;
extern struct rnp_info rnp_n400_2ports_info;
extern struct rnp_info rnp_n500_info;
// extern struct rnp_info rnp_n20_2ports_info;

static struct rnp_info *rnp_info_tbl[] = {
	[board_vu440_2ports] = &rnp_vu440_2ports_info,
	[board_n10_2ports] = &rnp_n10_2ports_info,
	[board_n400_2ports] = &rnp_n400_2ports_info,
	//	[board_n20_2ports] = &rnp_n20_2ports_info,
	[board_n500] = &rnp_n500_info,
};

static int register_mbx_irq(struct rnp_adapter *adapter);
static void remove_mbx_irq(struct rnp_adapter *adapter);
static bool rnp_alloc_mapped_page(struct rnp_ring *rx_ring,
								  struct rnp_rx_buffer *bi);
static bool rnp_alloc_mapped_skb(struct rnp_ring *rx_ring,
								 struct rnp_rx_buffer *bi);
static void rnp_pull_tail(struct sk_buff *skb);
static void rnp_put_rx_buffer(struct rnp_ring *rx_ring,
							  struct rnp_rx_buffer *rx_buffer,
							  struct sk_buff *skb);

/* itr can be modified in napi handle */
/* now hw not support this */
#define ITR_TEST 0

static struct pci_device_id rnp_pci_tbl[] = {
	{PCI_DEVICE(0x8848, 0x1000),
	 .driver_data = board_n10_2ports}, // 2x40G 2x10G
	{PCI_DEVICE(0x8848, 0x1001),
	 .driver_data = board_n400_2ports}, // 2x1G
	{PCI_DEVICE(0x8848, 0x1C00),
	 .driver_data = board_n10_2ports},						 // 2x40G 2x10G
//	{PCI_DEVICE(0x8848, 0x8308), .driver_data = board_n500}, // n500
	{PCI_DEVICE(0x1dab, 0x7001), .driver_data = board_vu440_2ports}, // vu440
	{PCI_DEVICE(0x1dab, 0x7002), .driver_data = board_vu440_2ports}, // vu440

	/* required last entry */
	{
		0,
	},
};
MODULE_DEVICE_TABLE(pci, rnp_pci_tbl);

static unsigned int mac_loop_en;
module_param(mac_loop_en, uint, 0000);

#ifdef CONFIG_PCI_IOV
static unsigned int max_vfs;
module_param(max_vfs, uint, 0000);
MODULE_PARM_DESC(
	max_vfs,
	"Maximum number of virtual functions to allocate per physical "
	"function - default is zero and maximum value is 63/7 in n10/n500");
#endif /* CONFIG_PCI_IOV */

#define DEFAULT_MSG_ENABLE (NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK)
static int debug = -1;
module_param(debug, int, 0000);
MODULE_PARM_DESC(debug, "Debug level (0=none,...,16=all)");

static unsigned int fix_eth_name;
module_param(fix_eth_name, uint, 0000);
MODULE_PARM_DESC(fix_eth_name, "set eth adapter name to rnpXX");

static int module_enable_ptp = 1;
module_param(module_enable_ptp, uint, 0000);
MODULE_PARM_DESC(module_enable_ptp, "enable ptp feature, disabled default");

static int pf_msix_counts_set = 0;
module_param(pf_msix_counts_set, uint, 0000);
MODULE_PARM_DESC(pf_msix_counts_set, "set msix count by one pf");

MODULE_AUTHOR("Mucse Corporation, <mucse@mucse.com>");
MODULE_DESCRIPTION("Mucse(R) 1/10/25/40 Gigabit PCI Express Network Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

static struct workqueue_struct *rnp_wq;

static int enable_hi_dma;
extern void rnp_service_timer(struct timer_list *t);

static void rnp_service_event_schedule(struct rnp_adapter *adapter)
{
	if (!test_bit(__RNP_DOWN, &adapter->state) &&
		!test_and_set_bit(__RNP_SERVICE_SCHED, &adapter->state))
		queue_work(rnp_wq, &adapter->service_task);
	//	schedule_work(&adapter->service_task);
}

static void rnp_service_event_complete(struct rnp_adapter *adapter)
{
	BUG_ON(!test_bit(__RNP_SERVICE_SCHED, &adapter->state));

	/* flush memory to make sure state is correct before next watchdog */
	smp_mb__before_atomic();
	clear_bit(__RNP_SERVICE_SCHED, &adapter->state);
}

void rnp_release_hw_control(struct rnp_adapter *adapter)
{
	u32 ctrl_ext;

	/* Let firmware take over control of h/w */
	// ctrl_ext = RNP_READ_REG(&adapter->hw, RNP_CTRL_EXT);
	// RNP_WRITE_REG(&adapter->hw, RNP_CTRL_EXT, ctrl_ext &
	// ~RNP_CTRL_EXT_DRV_LOAD);
}

void rnp_get_hw_control(struct rnp_adapter *adapter)
{
	u32 ctrl_ext;

	/* Let firmware know the driver has taken over */
}

/**
 * rnp_set_ivar - set the ring_vector registers, mapping interrupt causes to
 * vectors
 * @adapter: pointer to adapter struct
 * @queue: queue to map the corresponding interrupt to
 * @msix_vector: the vector to map to the corresponding queue
 *
 */
static void rnp_set_ring_vector(struct rnp_adapter *adapter,
								u8 rnp_queue,
								u8 rnp_msix_vector)
{
	struct rnp_hw *hw = &adapter->hw;
	u32 data = 0;
	struct net_device *netdev = adapter->netdev;

	data = hw->pfvfnum << 24;
	data |= (rnp_msix_vector << 8);
	data |= (rnp_msix_vector << 0);

	DPRINTK(IFUP,
			INFO,
			"Set Ring-Vector queue:%d (reg:0x%x) <-- Rx-MSIX:%d, Tx-MSIX:%d\n",
			rnp_queue,
			RING_VECTOR(rnp_queue),
			rnp_msix_vector,
			rnp_msix_vector);

	rnp_wr_reg(hw->ring_msix_base + RING_VECTOR(rnp_queue), data);
}

static inline void rnp_irq_rearm_queues(struct rnp_adapter *adapter, u64 qmask)
{
	u32 mask;
}

void rnp_unmap_and_free_tx_resource(struct rnp_ring *ring,
									struct rnp_tx_buffer *tx_buffer)
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

static u64 rnp_get_tx_completed(struct rnp_ring *ring)
{
	return ring->stats.packets;
}

static u64 rnp_get_tx_pending(struct rnp_ring *ring)
{
	struct rnp_adapter *adapter = netdev_priv(ring->netdev);
	struct rnp_hw *hw = &adapter->hw;

	u32 head = ring_rd32(ring, RNP_DMA_REG_TX_DESC_BUF_HEAD);
	u32 tail = ring_rd32(ring, RNP_DMA_REG_TX_DESC_BUF_TAIL);

	if (head != tail)
		return (head < tail) ? tail - head : (tail + ring->count - head);

	return 0;
}

static inline bool rnp_check_tx_hang(struct rnp_ring *tx_ring)
{
	u32 tx_done = rnp_get_tx_completed(tx_ring);
	u32 tx_done_old = tx_ring->tx_stats.tx_done_old;
	u32 tx_pending = rnp_get_tx_pending(tx_ring);
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
		ret = test_and_set_bit(__RNP_HANG_CHECK_ARMED, &tx_ring->state);
	} else {
		/* update completed stats and continue */
		tx_ring->tx_stats.tx_done_old = tx_done;
		/* reset the countdown */
		clear_bit(__RNP_HANG_CHECK_ARMED, &tx_ring->state);
	}
	return ret;
}

/**
 * rnp_tx_timeout_reset - initiate reset due to Tx timeout
 * @adapter: driver private struct
 **/
static void rnp_tx_timeout_reset(struct rnp_adapter *adapter)
{
	/* Do the reset outside of interrupt context */
	if (!test_bit(__RNP_DOWN, &adapter->state)) {
		adapter->flags2 |= RNP_FLAG2_RESET_REQUESTED;
		e_warn(drv, "initiating reset due to tx timeout\n");
		rnp_service_event_schedule(adapter);
	}
}

static void rnp_check_restart_tx(struct rnp_q_vector *q_vector,
								 struct rnp_ring *tx_ring)
{

	struct rnp_adapter *adapter = q_vector->adapter;
#define TX_WAKE_THRESHOLD (DESC_NEEDED * 2)
	if (likely(netif_carrier_ok(tx_ring->netdev) &&
			   (rnp_desc_unused(tx_ring) >= TX_WAKE_THRESHOLD))) {
		/* Make sure that anybody stopping the queue after this
		 * sees the new next_to_clean.
		 */
		smp_mb();
#ifdef HAVE_TX_MQ
		if (__netif_subqueue_stopped(tx_ring->netdev, tx_ring->queue_index) &&
			!test_bit(__RNP_DOWN, &adapter->state)) {
			netif_wake_subqueue(tx_ring->netdev, tx_ring->queue_index);
			++tx_ring->tx_stats.restart_queue;
		}
#else
		if (__netif_queue_stopped(tx_ring->netdev) &&
			!test_bit(__RNP_DOWN, &adapter->state)) {
			netif_wake_queue(tx_ring->netdev);
			++tx_ring->tx_stats.restart_queue;
		}

#endif
	}
}

/**
 * rnp_clean_tx_irq - Reclaim resources after transmit completes
 * @q_vector: structure containing interrupt and ring information
 * @tx_ring: tx ring to clean
 **/
static bool rnp_clean_tx_irq(struct rnp_q_vector *q_vector,
							 struct rnp_ring *tx_ring,
							 int napi_budget)
{
	struct rnp_adapter *adapter = q_vector->adapter;
	struct rnp_tx_buffer *tx_buffer;
	struct rnp_tx_desc *tx_desc;
	u64 total_bytes = 0, total_packets = 0;
	int budget = q_vector->tx.work_limit;
	int i = tx_ring->next_to_clean;

	if (test_bit(__RNP_DOWN, &adapter->state))
		return true;
	tx_ring->tx_stats.poll_count++;
	tx_buffer = &tx_ring->tx_buffer_info[i];
	tx_desc = RNP_TX_DESC(tx_ring, i);
	i -= tx_ring->count;

	do {
		struct rnp_tx_desc *eop_desc = tx_buffer->next_to_watch;

		/* if next_to_watch is not set then there is no work pending */
		if (!eop_desc)
			break;

		/* prevent any other reads prior to eop_desc */
		rmb();

		/* if eop DD is not set pending work has not been completed */
		if (!(eop_desc->vlan_cmd & cpu_to_le32(RNP_TXD_STAT_DD)))
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

		/* unmap remaining buffers */
		while (tx_desc != eop_desc) {
			tx_buffer++;
			tx_desc++;
			i++;
			if (unlikely(!i)) {
				i -= tx_ring->count;
				tx_buffer = tx_ring->tx_buffer_info;
				tx_desc = RNP_TX_DESC(tx_ring, 0);
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
			tx_desc = RNP_TX_DESC(tx_ring, 0);
		}

		/* issue prefetch for next Tx descriptor */
		prefetch(tx_desc);

		/* update budget accounting */
		budget--;
	} while (likely(budget));

	// printk("clean tx irq %d\n", budget);
	i += tx_ring->count;
	tx_ring->next_to_clean = i;
	u64_stats_update_begin(&tx_ring->syncp);
	tx_ring->stats.bytes += total_bytes;
	tx_ring->stats.packets += total_packets;
	// update tx clean
	tx_ring->tx_stats.tx_clean_count += total_packets;
	tx_ring->tx_stats.tx_clean_times++;
	if (tx_ring->tx_stats.tx_clean_times > 10) {
		tx_ring->tx_stats.tx_clean_times = 0;
		tx_ring->tx_stats.tx_clean_count = 0;
	}

	u64_stats_update_end(&tx_ring->syncp);
	q_vector->tx.total_bytes += total_bytes;
	q_vector->tx.total_packets += total_packets;

	tx_ring->tx_stats.send_done_bytes += total_bytes;
#ifdef NO_BQL_TEST
#else
	netdev_tx_completed_queue(txring_txq(tx_ring), total_packets, total_bytes);
#endif

#ifndef REDUCE_TX_IRQ_MISS
#define TX_WAKE_THRESHOLD (DESC_NEEDED * 2)
	if (likely(netif_carrier_ok(tx_ring->netdev) &&
			   (rnp_desc_unused(tx_ring) >= TX_WAKE_THRESHOLD))) {
		/* Make sure that anybody stopping the queue after this
		 * sees the new next_to_clean.
		 */
		smp_mb();
		if (__netif_subqueue_stopped(tx_ring->netdev, tx_ring->queue_index) &&
			!test_bit(__RNP_DOWN, &adapter->state)) {
			netif_wake_subqueue(tx_ring->netdev, tx_ring->queue_index);
			++tx_ring->tx_stats.restart_queue;
		}
	}

#endif
	/* now we start tx queue later */
	return !!budget;
}

static inline void rnp_rx_hash(struct rnp_ring *ring,
							   union rnp_rx_desc *rx_desc,
							   struct sk_buff *skb)
{
	int rss_type;

	if (!(ring->netdev->features & NETIF_F_RXHASH))
		return;
#define RNP_RSS_TYPE_MASK 0xc0
	rss_type = rx_desc->wb.cmd & RNP_RSS_TYPE_MASK;
	skb_set_hash(skb,
				 le32_to_cpu(rx_desc->wb.rss_hash),
				 rss_type ? PKT_HASH_TYPE_L4 : PKT_HASH_TYPE_L3);
}

/**
 * rnp_rx_checksum - indicate in skb if hw indicated a good cksum
 * @ring: structure containing ring specific data
 * @rx_desc: current Rx descriptor being processed
 * @skb: skb currently being received and modified
 **/
static inline void rnp_rx_checksum(struct rnp_ring *ring,
								   union rnp_rx_desc *rx_desc,
								   struct sk_buff *skb)
{
	bool encap_pkt = false;

	skb_checksum_none_assert(skb);
	/* Rx csum disabled */
	if (!(ring->netdev->features & NETIF_F_RXCSUM))
		return;

	/* vxlan packet handle ? */
	if (rnp_get_stat(rx_desc, RNP_RXD_STAT_TUNNEL_MASK) ==
		RNP_RXD_STAT_TUNNEL_VXLAN) {
		encap_pkt = true;
#if defined(HAVE_UDP_ENC_RX_OFFLOAD) || defined(HAVE_VXLAN_RX_OFFLOAD)
		skb->encapsulation = 1;
#endif /* HAVE_UDP_ENC_RX_OFFLOAD || HAVE_VXLAN_RX_OFFLOAD */
		skb->ip_summed = CHECKSUM_NONE;
	}

	/* if outer L3/L4  error */
	/* must in promisc mode or rx-all mode */
	if (rnp_test_staterr(rx_desc, RNP_RXD_STAT_ERR_MASK)) {
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

static inline void rnp_update_rx_tail(struct rnp_ring *rx_ring, u32 val)
{
	rx_ring->next_to_use = val;
#ifndef CONFIG_RNP_DISABLE_PACKET_SPLIT
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
	if (likely(!(rx_ring->ring_flags & RNP_RING_FLAG_DELAY_SETUP_RX_LEN))) {
		rnp_wr_reg(rx_ring->tail, val);

	} else if (val < RNP_MIN_RXD) {
		/* if rx_ring in delay setup mode, don't update
		 * next_to_use to hw large than RNP_MIN_RXD
		 */
		rnp_wr_reg(rx_ring->tail, val);
	}
}

/**
 * rnp_alloc_rx_buffers - Replace used receive buffers
 * @rx_ring: ring to place buffers on
 * @cleaned_count: number of buffers to replace
 **/
void rnp_alloc_rx_buffers(struct rnp_ring *rx_ring, u16 cleaned_count)
{
	union rnp_rx_desc *rx_desc;
	struct rnp_rx_buffer *bi;
	u16 i = rx_ring->next_to_use;
	u64 fun_id = ((u64)(rx_ring->pfvfnum) << (32 + 24));
#ifndef CONFIG_RNP_DISABLE_PACKET_SPLIT
	u16 bufsz;
#endif
	/* nothing to do */
	if (!cleaned_count)
		return;

	rx_desc = RNP_RX_DESC(rx_ring, i);

	BUG_ON(rx_desc == NULL);

	bi = &rx_ring->rx_buffer_info[i];

	BUG_ON(bi == NULL);

	i -= rx_ring->count;
#ifndef CONFIG_RNP_DISABLE_PACKET_SPLIT
	bufsz = rnp_rx_bufsz(rx_ring);
#endif

	do {
#ifdef CONFIG_RNP_DISABLE_PACKET_SPLIT
		if (!rnp_alloc_mapped_skb(rx_ring, bi))
			break;
#else
		if (!rnp_alloc_mapped_page(rx_ring, bi))
			break;

		dma_sync_single_range_for_device(
			rx_ring->dev, bi->dma, bi->page_offset, bufsz, DMA_FROM_DEVICE);
#endif

			/*
			 * Refresh the desc even if buffer_addrs didn't change
			 * because each write-back erases this info.
			 */
#ifdef CONFIG_RNP_DISABLE_PACKET_SPLIT
		rx_desc->pkt_addr = cpu_to_le64(bi->dma + fun_id);
#else
		rx_desc->pkt_addr = cpu_to_le64(bi->dma + bi->page_offset + fun_id);

		// printk("%d rx_desc page_offset %x\n", i, bi->page_offset);
#endif
		/* clean dd */
		rx_desc->resv_cmd = 0;

		rx_desc++;
		bi++;
		i++;
		if (unlikely(!i)) {
			rx_desc = RNP_RX_DESC(rx_ring, 0);
			bi = rx_ring->rx_buffer_info;
			i -= rx_ring->count;
		}

		/* clear the hdr_addr for the next_to_use descriptor */
		// rx_desc->cmd = 0;
		cleaned_count--;
	} while (cleaned_count);

	i += rx_ring->count;

	if (rx_ring->next_to_use != i)
		rnp_update_rx_tail(rx_ring, i);
}
/**
 * rnp_get_headlen - determine size of header for RSC/LRO/GRO/FCOE
 * @data: pointer to the start of the headers
 * @max_len: total length of section to find headers in
 *
 * This function is meant to determine the length of headers that will
 * be recognized by hardware for LRO, GRO, and RSC offloads.  The main
 * motivation of doing this is to only perform one pull for IPv4 TCP
 * packets so that we can do basic things like calculating the gso_size
 * based on the average data per packet.
 **/
static unsigned int rnp_get_headlen(unsigned char *data, unsigned int max_len)
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
	if (protocol == htons(ETH_P_8021Q)) {
		if ((hdr.network - data) > (max_len - VLAN_HLEN))
			return max_len;

		protocol = hdr.vlan->h_vlan_encapsulated_proto;
		hdr.network += VLAN_HLEN;
	}

	/* handle L3 protocols */
	if (protocol == htons(ETH_P_IP)) {
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
	} else if (protocol == htons(ETH_P_IPV6)) {
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

static void rnp_set_rsc_gso_size(struct rnp_ring *ring, struct sk_buff *skb)
{
	u16 hdr_len = skb_headlen(skb);

	/* set gso_size to avoid messing up TCP MSS */
	skb_shinfo(skb)->gso_size =
		DIV_ROUND_UP((skb->len - hdr_len), RNP_CB(skb)->append_cnt);
	skb_shinfo(skb)->gso_type = SKB_GSO_TCPV4;
}

static void rnp_update_rsc_stats(struct rnp_ring *rx_ring, struct sk_buff *skb)
{
	/* if append_cnt is 0 then frame is not RSC */
	if (!RNP_CB(skb)->append_cnt)
		return;

	rx_ring->rx_stats.rsc_count += RNP_CB(skb)->append_cnt;
	rx_ring->rx_stats.rsc_flush++;

	rnp_set_rsc_gso_size(rx_ring, skb);

	/* gso_size is computed using append_cnt so always clear it last */
	RNP_CB(skb)->append_cnt = 0;
}

/**
 * rnp_process_skb_fields - Populate skb header fields from Rx descriptor
 * @rx_ring: rx descriptor ring packet is being transacted on
 * @rx_desc: pointer to the EOP Rx descriptor
 * @skb: pointer to current skb being populated
 *
 * This function checks the ring, descriptor, and packet information in
 * order to populate the hash, checksum, VLAN, timestamp, protocol, and
 * other fields within the skb.
 **/
static void rnp_process_skb_fields(struct rnp_ring *rx_ring,
								   union rnp_rx_desc *rx_desc,
								   struct sk_buff *skb)
{
	struct net_device *dev = rx_ring->netdev;

	// rnp_update_rsc_stats(rx_ring, skb);
	rnp_rx_hash(rx_ring, rx_desc, skb);

	rnp_rx_checksum(rx_ring, rx_desc, skb);
#ifdef NETIF_F_HW_VLAN_CTAG_RX
	if ((dev->features & NETIF_F_HW_VLAN_CTAG_RX) &&
#else
	if ((dev->features & NETIF_F_HW_VLAN_RX) &&
#endif
		rnp_test_staterr(rx_desc, RNP_RXD_STAT_VLAN_VALID) &&
		!ignore_veb_vlan(rx_ring->q_vector->adapter, rx_desc)) {
		u16 vid = le16_to_cpu(rx_desc->wb.vlan);

		__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), vid);
		rx_ring->rx_stats.vlan_remove++;
	}

	skb_record_rx_queue(skb, rx_ring->queue_index);

	skb->protocol = eth_type_trans(skb, dev);
}

static void rnp_rx_skb(struct rnp_q_vector *q_vector, struct sk_buff *skb)
{
	struct rnp_adapter *adapter = q_vector->adapter;

	if (!(adapter->flags & RNP_FLAG_IN_NETPOLL))
		napi_gro_receive(&q_vector->napi, skb);
	else
		netif_rx(skb);
}

#ifdef CONFIG_RNP_DISABLE_PACKET_SPLIT
/**
 * rnp_merge_active_tail - merge active tail into lro skb
 * @tail: pointer to active tail in frag_list
 *
 * This function merges the length and data of an active tail into the
 * skb containing the frag_list.  It resets the tail's pointer to the head,
 * but it leaves the heads pointer to tail intact.
 **/
static inline struct sk_buff *rnp_merge_active_tail(struct sk_buff *tail)
{
	struct sk_buff *head = RNP_CB(tail)->head;

	if (!head)
		return tail;

	head->len += tail->len;
	head->data_len += tail->len;
	head->truesize += tail->truesize;

	RNP_CB(tail)->head = NULL;

	return head;
}

/**
 * rnp_add_active_tail - adds an active tail into the skb frag_list
 * @head: pointer to the start of the skb
 * @tail: pointer to active tail to add to frag_list
 *
 * This function adds an active tail to the end of the frag list.  This tail
 * will still be receiving data so we cannot yet ad it's stats to the main
 * skb.  That is done via rnp_merge_active_tail.
 **/
static inline void rnp_add_active_tail(struct sk_buff *head,
									   struct sk_buff *tail)
{
	struct sk_buff *old_tail = RNP_CB(head)->tail;

	if (old_tail) {
		rnp_merge_active_tail(old_tail);
		old_tail->next = tail;
	} else {
		skb_shinfo(head)->frag_list = tail;
	}

	RNP_CB(tail)->head = head;
	RNP_CB(head)->tail = tail;
}

/**
 * rnp_close_active_frag_list - cleanup pointers on a frag_list skb
 * @head: pointer to head of an active frag list
 *
 * This function will clear the frag_tail_tracker pointer on an active
 * frag_list and returns true if the pointer was actually set
 **/
static inline bool rnp_close_active_frag_list(struct sk_buff *head)
{
	struct sk_buff *tail = RNP_CB(head)->tail;

	if (!tail)
		return false;

	rnp_merge_active_tail(tail);

	RNP_CB(head)->tail = NULL;

	return true;
}

#endif

/**
 * rnp_is_non_eop - process handling of non-EOP buffers
 * @rx_ring: Rx ring being processed
 * @rx_desc: Rx descriptor for current buffer
 * @skb: Current socket buffer containing buffer in progress
 *
 * This function updates next to clean.  If the buffer is an EOP buffer
 * this function exits returning false, otherwise it will place the
 * sk_buff in the next buffer to be chained and return true indicating
 * that this is in fact a non-EOP buffer.
 **/
static bool rnp_is_non_eop(struct rnp_ring *rx_ring,
						   union rnp_rx_desc *rx_desc,
						   struct sk_buff *skb)
{
	u32 ntc = rx_ring->next_to_clean + 1;
#ifdef CONFIG_RNP_DISABLE_PACKET_SPLIT
	struct sk_buff *next_skb;
#endif
	/* fetch, update, and store next to clean */
	ntc = (ntc < rx_ring->count) ? ntc : 0;
	rx_ring->next_to_clean = ntc;

	prefetch(RNP_RX_DESC(rx_ring, ntc));

	/* if we are the last buffer then there is nothing else to do */
	if (likely(rnp_test_staterr(rx_desc, RNP_RXD_STAT_EOP)))
		return false;
#ifdef CONFIG_RNP_RNP_DISABLE_PACKET_SPLIT
	next_skb = rx_ring->rx_buffer_info[ntc].skb;

	rnp_add_active_tail(skb, next_skb);
	RNP_CB(next_skb)->head = skb;
#else
	/* place skb in next buffer to be received */
	rx_ring->rx_buffer_info[ntc].skb = skb;
#endif
	rx_ring->rx_stats.non_eop_descs++;

	return true;
}

// try to add new method

#if (PAGE_SIZE < 8192)
#define RNP_MAX_2K_FRAME_BUILD_SKB (RNP_RXBUFFER_1536 - NET_IP_ALIGN)
#define RNP_2K_TOO_SMALL_WITH_PADDING \
	((NET_SKB_PAD + RNP_RXBUFFER_1536) > SKB_WITH_OVERHEAD(RNP_RXBUFFER_2K))

static inline int rnp_compute_pad(int rx_buf_len)
{
	int page_size, pad_size;

	page_size = ALIGN(rx_buf_len, PAGE_SIZE / 2);
	pad_size = SKB_WITH_OVERHEAD(page_size) - rx_buf_len;

	return pad_size;
}

static inline int rnp_skb_pad(void)
{
	int rx_buf_len;

	/* If a 2K buffer cannot handle a standard Ethernet frame then
	 * optimize padding for a 3K buffer instead of a 1.5K buffer.
	 *
	 * For a 3K buffer we need to add enough padding to allow for
	 * tailroom due to NET_IP_ALIGN possibly shifting us out of
	 * cache-line alignment.
	 */
	if (RNP_2K_TOO_SMALL_WITH_PADDING)
		rx_buf_len = RNP_RXBUFFER_3K + SKB_DATA_ALIGN(NET_IP_ALIGN);
	else
		rx_buf_len = RNP_RXBUFFER_1536;

	/* if needed make room for NET_IP_ALIGN */
	rx_buf_len -= NET_IP_ALIGN;
	return rnp_compute_pad(rx_buf_len);
}

#define RNP_SKB_PAD rnp_skb_pad()
#else
#define RNP_SKB_PAD (NET_SKB_PAD + NET_IP_ALIGN)
#endif

#ifndef CONFIG_RNP_DISABLE_PACKET_SPLIT

static inline unsigned int rnp_rx_offset(struct rnp_ring *rx_ring)
{
	return ring_uses_build_skb(rx_ring) ? RNP_SKB_PAD : 0;
}

static bool rnp_alloc_mapped_page(struct rnp_ring *rx_ring,
								  struct rnp_rx_buffer *bi)
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

	page = dev_alloc_pages(rnp_rx_pg_order(rx_ring));
	if (unlikely(!page)) {
		rx_ring->rx_stats.alloc_rx_page_failed++;
		return false;
	}

	/* map page for use */
	dma = dma_map_page_attrs(rx_ring->dev,
							 page,
							 0,
							 rnp_rx_pg_size(rx_ring),
							 DMA_FROM_DEVICE,
#if defined(HAVE_STRUCT_DMA_ATTRS) && defined(HAVE_SWIOTLB_SKIP_CPU_SYNC)
							 &attrs);
#else
							 RNP_RX_DMA_ATTR);
#endif

	/*
	 * if mapping failed free memory back to system since
	 * there isn't much point in holding memory we can't use
	 */
	if (dma_mapping_error(rx_ring->dev, dma)) {
		__free_pages(page, rnp_rx_pg_order(rx_ring));
		printk("map failed\n");

		rx_ring->rx_stats.alloc_rx_page_failed++;
		return false;
	}
	bi->dma = dma;
	bi->page = page;
	bi->page_offset = rnp_rx_offset(rx_ring);
#ifdef HAVE_PAGE_COUNT_BULK_UPDATE
	page_ref_add(page, USHRT_MAX - 1);
	bi->pagecnt_bias = USHRT_MAX;
	// printk("page ref_count is %x\n", page_ref_count(page));
#else
	bi->pagecnt_bias = 1;
#endif
	rx_ring->rx_stats.alloc_rx_page++;

	return true;
}
/**
 * rnp_pull_tail - rnp specific version of skb_pull_tail
 * @skb: pointer to current skb being adjusted
 *
 * This function is an rnp specific version of __pskb_pull_tail.  The
 * main difference between this version and the original function is that
 * this function can make several assumptions about the state of things
 * that allow for significant optimizations versus the standard function.
 * As a result we can do things like drop a frag and maintain an accurate
 * truesize for the skb.
 */
static void rnp_pull_tail(struct sk_buff *skb)
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
	// pull_len = rnp_get_headlen(va, RNP_RX_HDR_SIZE);
	pull_len = eth_get_headlen(skb->dev, va, RNP_RX_HDR_SIZE);

	/* align pull length to size of long to optimize memcpy performance */
	skb_copy_to_linear_data(skb, va, ALIGN(pull_len, sizeof(long)));

	/* update all of the pointers */
	skb_frag_size_sub(frag, pull_len);
	skb_frag_off_add(frag, pull_len);
	skb->data_len -= pull_len;
	skb->tail += pull_len;
}

/**
 * rnp_dma_sync_frag - perform DMA sync for first frag of SKB
 * @rx_ring: rx descriptor ring packet is being transacted on
 * @skb: pointer to current skb being updated
 *
 * This function provides a basic DMA sync up for the first fragment of an
 * skb.  The reason for doing this is that the first fragment cannot be
 * unmapped until we have reached the end of packet descriptor for a buffer
 * chain.
 */
static void rnp_dma_sync_frag(struct rnp_ring *rx_ring, struct sk_buff *skb)
{
#if defined(HAVE_STRUCT_DMA_ATTRS) && defined(HAVE_SWIOTLB_SKIP_CPU_SYNC)
	DEFINE_DMA_ATTRS(attrs);

	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
	dma_set_attr(DMA_ATTR_WEAK_ORDERING, &attrs);

#endif
	/* if the page was released unmap it, else just sync our portion */
	if (unlikely(RNP_CB(skb)->page_released)) {
		dma_unmap_page_attrs(rx_ring->dev,
							 RNP_CB(skb)->dma,
							 rnp_rx_pg_size(rx_ring),
							 DMA_FROM_DEVICE,
#if defined(HAVE_STRUCT_DMA_ATTRS) && defined(HAVE_SWIOTLB_SKIP_CPU_SYNC)
							 &attrs);
#else
							 RNP_RX_DMA_ATTR);
#endif
	} else if (ring_uses_build_skb(rx_ring)) {
		unsigned long offset = (unsigned long)(skb->data) & ~PAGE_MASK;

		dma_sync_single_range_for_cpu(rx_ring->dev,
									  RNP_CB(skb)->dma,
									  offset,
									  skb_headlen(skb),
									  DMA_FROM_DEVICE);
	} else {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[0];

		dma_sync_single_range_for_cpu(rx_ring->dev,
									  RNP_CB(skb)->dma,
									  skb_frag_off(frag),
									  skb_frag_size(frag),
									  DMA_FROM_DEVICE);
	}
}

/* drop this packets if error */
static bool rnp_check_csum_error(struct rnp_ring *rx_ring,
								 union rnp_rx_desc *rx_desc,
								 unsigned int size,
								 unsigned int *driver_drop_packets)
{
	bool err = false;

	struct net_device *netdev = rx_ring->netdev;

	if (netdev->features & NETIF_F_RXCSUM) {
		if (unlikely(rnp_test_staterr(rx_desc, RNP_RXD_STAT_ERR_MASK))) {
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
						rnp_test_staterr(rx_desc, RNP_RXD_STAT_SCTP_MASK))) {
					if (size > 60) {
						err = true;
						//	return true;
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
		struct rnp_rx_buffer *rx_buffer;
		u32 ntc = rx_ring->next_to_clean + 1;
#if (PAGE_SIZE < 8192)
		unsigned int truesize = rnp_rx_pg_size(rx_ring) / 2;
#else
		unsigned int truesize = ring_uses_build_skb(rx_ring)
									? SKB_DATA_ALIGN(RNP_SKB_PAD + size)
									: SKB_DATA_ALIGN(size);
#endif

		// if eop add drop_packets
		if (likely(rnp_test_staterr(rx_desc, RNP_RXD_STAT_EOP)))
			*driver_drop_packets = *driver_drop_packets + 1;

		/* we are reusing so sync this buffer for CPU use */
		rx_buffer = &rx_ring->rx_buffer_info[rx_ring->next_to_clean];
		dma_sync_single_range_for_cpu(rx_ring->dev,
									  rx_buffer->dma,
									  rx_buffer->page_offset,
									  size,
									  DMA_FROM_DEVICE);

		// no-need minis ,we don't send to os stack
		// rx_buffer->pagecnt_bias--;

#if (PAGE_SIZE < 8192)
		rx_buffer->page_offset ^= truesize;
#else
		rx_buffer->page_offset += truesize;
#endif
		rnp_put_rx_buffer(rx_ring, rx_buffer, NULL);

		// update to the next desc
		ntc = (ntc < rx_ring->count) ? ntc : 0;
		rx_ring->next_to_clean = ntc;
	}
	return err;
}

static bool rnp_check_src_mac(struct sk_buff *skb, struct net_device *netdev)
{
	char *data = (char *)skb->data;
	bool ret = false;
	struct netdev_hw_addr *ha;

	if (is_multicast_ether_addr(data)) {
		if (0 == memcmp(data + netdev->addr_len,
						netdev->dev_addr,
						netdev->addr_len)) {
			dev_kfree_skb_any(skb);
			ret = true;
		}
		// if src mac equal own mac
		netdev_for_each_uc_addr(ha, netdev)
		{
			if (0 ==
				memcmp(data + netdev->addr_len, ha->addr, netdev->addr_len)) {
				dev_kfree_skb_any(skb);
				// printk("drop own packets\n");
				ret = true;
			}
		}
	}
	return ret;
}

/**
 * rnp_cleanup_headers - Correct corrupted or empty headers
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
static bool rnp_cleanup_headers(struct rnp_ring __maybe_unused *rx_ring,
								union rnp_rx_desc *rx_desc,
								struct sk_buff *skb)
{
	struct net_device *netdev = rx_ring->netdev;
	struct rnp_adapter *adapter = netdev_priv(netdev);
	/* XDP packets use error pointer so abort at this point */
	if (IS_ERR(skb))
		return true;

	//	if (netdev->features & NETIF_F_RXCSUM) {
	//		if (unlikely(rnp_test_staterr(rx_desc, RNP_RXD_STAT_ERR_MASK))) {
	//			rx_debug_printk("rx error: VEB:%s mark:0x%x cmd:0x%x\n",
	//					(rx_ring->q_vector->adapter->flags &
	//					 RNP_FLAG_SRIOV_ENABLED) ?
	//						"On" :
	//						"Off",
	//					rx_desc->wb.mark, rx_desc->wb.cmd);
	//			/* push this packet to stack if in promisc mode */
	//			rx_ring->rx_stats.csum_err++;
	//
	//			if ((!(netdev->flags & IFF_PROMISC) &&
	//			     (!(netdev->features & NETIF_F_RXALL)))) {
	//				/* sctp less than 60 hw report err by mistake */
	//				if (unlikely(rnp_test_staterr(
	//					    rx_desc, RNP_RXD_STAT_SCTP_MASK))) {
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
		rnp_pull_tail(skb);
	/* if eth_skb_pad returns an error the skb was freed */
	if (eth_skb_pad(skb))
		return true;
	// check src mac if in sriov mode

	if (adapter->flags & RNP_FLAG_SRIOV_ENABLED)
		return rnp_check_src_mac(skb, rx_ring->netdev);
	else
		return false;
	// return false;
}

/**
 * rnp_reuse_rx_page - page flip buffer and store it back on the ring
 * @rx_ring: rx descriptor ring to store buffers on
 * @old_buff: donor buffer to have page reused
 *
 * Synchronizes page for reuse by the adapter
 **/
static void rnp_reuse_rx_page(struct rnp_ring *rx_ring,
							  struct rnp_rx_buffer *old_buff)
{
	struct rnp_rx_buffer *new_buff;
	u16 nta = rx_ring->next_to_alloc;

	new_buff = &rx_ring->rx_buffer_info[nta];

	/* update, and store next to alloc */
	nta++;
	rx_ring->next_to_alloc = (nta < rx_ring->count) ? nta : 0;

	/*
	 * Transfer page from old buffer to new buffer.
	 * Move each member individually to avoid possible store
	 * forwarding stalls and unnecessary copy of skb.
	 */
	new_buff->dma = old_buff->dma;
	new_buff->page = old_buff->page;
	new_buff->page_offset = old_buff->page_offset;
	new_buff->pagecnt_bias = old_buff->pagecnt_bias;
}

static inline bool rnp_page_is_reserved(struct page *page)
{
	return (page_to_nid(page) != numa_mem_id()) || page_is_pfmemalloc(page);
}

static bool rnp_can_reuse_rx_page(struct rnp_rx_buffer *rx_buffer)
{
	unsigned int pagecnt_bias = rx_buffer->pagecnt_bias;
	struct page *page = rx_buffer->page;

	/* avoid re-using remote pages */
	if (unlikely(rnp_page_is_reserved(page)))
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

		/*
		 * The last offset is a bit aggressive in that we assume the
		 * worst case of FCoE being enabled and using a 3K buffer.
		 * However this should have minimal impact as the 1K extra is
		 * still less than one buffer in size.
		 */
#define RNP_LAST_OFFSET (SKB_WITH_OVERHEAD(PAGE_SIZE) - RNP_RXBUFFER_2K)
	if (rx_buffer->page_offset > RNP_LAST_OFFSET)
		return false;
#endif

skip_check:

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
	/*
	 * Even if we own the page, we are not allowed to use atomic_set()
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
 * rnp_add_rx_frag - Add contents of Rx buffer to sk_buff
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
static void rnp_add_rx_frag(struct rnp_ring *rx_ring,
							struct rnp_rx_buffer *rx_buffer,
							struct sk_buff *skb,
							unsigned int size)
{
#if (PAGE_SIZE < 8192)
	unsigned int truesize = rnp_rx_pg_size(rx_ring) / 2;
#else
	unsigned int truesize = ring_uses_build_skb(rx_ring)
								? SKB_DATA_ALIGN(RNP_SKB_PAD + size)
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

static struct rnp_rx_buffer *rnp_get_rx_buffer(struct rnp_ring *rx_ring,
											   union rnp_rx_desc *rx_desc,
											   struct sk_buff **skb,
											   const unsigned int size)
{
	struct rnp_rx_buffer *rx_buffer;

	rx_buffer = &rx_ring->rx_buffer_info[rx_ring->next_to_clean];
	prefetchw(rx_buffer->page);
	*skb = rx_buffer->skb;

	rx_buf_dump("rx buf",
				page_address(rx_buffer->page) + rx_buffer->page_offset,
				rx_desc->wb.len);

	/* we are reusing so sync this buffer for CPU use */
	dma_sync_single_range_for_cpu(rx_ring->dev,
								  rx_buffer->dma,
								  rx_buffer->page_offset,
								  size,
								  DMA_FROM_DEVICE);
	/* skip_sync: */
	rx_buffer->pagecnt_bias--;

	return rx_buffer;
}

static void rnp_put_rx_buffer(struct rnp_ring *rx_ring,
							  struct rnp_rx_buffer *rx_buffer,
							  struct sk_buff *skb)
{
#if defined(HAVE_STRUCT_DMA_ATTRS) && defined(HAVE_SWIOTLB_SKIP_CPU_SYNC)
	DEFINE_DMA_ATTRS(attrs);

	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
	dma_set_attr(DMA_ATTR_WEAK_ORDERING, &attrs);

#endif
	if (rnp_can_reuse_rx_page(rx_buffer)) {
		/* hand second half of page back to the ring */
		rnp_reuse_rx_page(rx_ring, rx_buffer);
	} else {
		/* we are not reusing the buffer so unmap it */
		dma_unmap_page_attrs(rx_ring->dev,
							 rx_buffer->dma,
							 rnp_rx_pg_size(rx_ring),
							 DMA_FROM_DEVICE,
#if defined(HAVE_STRUCT_DMA_ATTRS) && defined(HAVE_SWIOTLB_SKIP_CPU_SYNC)
							 &attrs);
#else
							 RNP_RX_DMA_ATTR);
#endif
		__page_frag_cache_drain(rx_buffer->page, rx_buffer->pagecnt_bias);
	}

	/* clear contents of rx_buffer */
	rx_buffer->page = NULL;
	rx_buffer->skb = NULL;
}

static struct sk_buff *rnp_construct_skb(struct rnp_ring *rx_ring,
										 struct rnp_rx_buffer *rx_buffer,
										 struct xdp_buff *xdp,
										 union rnp_rx_desc *rx_desc)
{
	unsigned int size = xdp->data_end - xdp->data;
#if (PAGE_SIZE < 8192)
	unsigned int truesize = rnp_rx_pg_size(rx_ring) / 2;
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
	 * packets going to stack via rnp_build_skb(). The latter
	 * provides us currently with 192 bytes of headroom.
	 *
	 * For rnp_construct_skb() mode it means that the
	 * xdp->data_meta will always point to xdp->data, since
	 * the helper cannot expand the head. Should this ever
	 * change in future for legacy-rx mode on, then lets also
	 * add xdp->data_meta handling here.
	 */

	/* allocate a skb to store the frags */
	skb = napi_alloc_skb(&rx_ring->q_vector->napi, RNP_RX_HDR_SIZE);
	if (unlikely(!skb))
		return NULL;

	prefetchw(skb->data);

	if (size > RNP_RX_HDR_SIZE) {
		/*
		 * if (!rnp_test_staterr(rx_desc, RNP_RXD_STAT_EOP))
		 * RNP_CB(skb)->dma = rx_buffer->dma;
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
static struct sk_buff *rnp_build_skb(struct rnp_ring *rx_ring,
									 struct rnp_rx_buffer *rx_buffer,
									 struct xdp_buff *xdp,
									 union rnp_rx_desc *rx_desc)
{
#ifdef HAVE_XDP_BUFF_DATA_META
	unsigned int metasize = xdp->data - xdp->data_meta;
	void *va = xdp->data_meta;
#else
	void *va = xdp->data;
#endif
#if (PAGE_SIZE < 8192)
	unsigned int truesize = rnp_rx_pg_size(rx_ring) / 2;
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
		/* record DMA address if this is the start of a
		 * chain of buffers
		 */
		/* if (!rnp_test_staterr(rx_desc, RNP_RXD_STAT_EOP))
		 * RNP_CB(skb)->dma = rx_buffer->dma;
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

#define RNP_XDP_PASS	 0
#define RNP_XDP_CONSUMED 1
#define RNP_XDP_TX		 2

static void rnp_rx_buffer_flip(struct rnp_ring *rx_ring,
							   struct rnp_rx_buffer *rx_buffer,
							   unsigned int size)
{
#if (PAGE_SIZE < 8192)
	unsigned int truesize = rnp_rx_pg_size(rx_ring) / 2;

	rx_buffer->page_offset ^= truesize;
#else
	unsigned int truesize = ring_uses_build_skb(rx_ring)
								? SKB_DATA_ALIGN(RNP_SKB_PAD + size)
								: SKB_DATA_ALIGN(size);

	rx_buffer->page_offset += truesize;
#endif
}

/**
 * rnp_rx_ring_reinit - just reinit rx_ring with new count in ->reset_count
 * @rx_ring: rx descriptor ring to transact packets on
 */
int rnp_rx_ring_reinit(struct rnp_adapter *adapter, struct rnp_ring *rx_ring)
{
	struct rnp_ring temp_ring;
	int err = 0;
	struct rnp_hw *hw = &adapter->hw;

	if (rx_ring->count == rx_ring->reset_count)
		return 0;
	/* stop rx queue */

	rnp_disable_rx_queue(adapter, rx_ring);
	memset(&temp_ring, 0x00, sizeof(struct rnp_ring));
	/* reinit for this ring */
	memcpy(&temp_ring, rx_ring, sizeof(struct rnp_ring));
	/* setup new count */
	temp_ring.count = rx_ring->reset_count;
	err = rnp_setup_rx_resources(&temp_ring, adapter);
	if (err) {
		rnp_free_rx_resources(&temp_ring);
		goto err_setup;
	}
	rnp_free_rx_resources(rx_ring);
	memcpy(rx_ring, &temp_ring, sizeof(struct rnp_ring));
	rnp_configure_rx_ring(adapter, rx_ring);
err_setup:
	/* start rx */
	ring_wr32(rx_ring, RNP_DMA_RX_START, 1);
	return 0;
}

/**
 * rnp_clean_rx_irq - Clean completed descriptors from Rx ring - bounce buf
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

static int rnp_clean_rx_irq(struct rnp_q_vector *q_vector,
							struct rnp_ring *rx_ring,
							int budget)
{
	unsigned int total_rx_bytes = 0, total_rx_packets = 0;
	unsigned int driver_drop_packets = 0;
	struct rnp_adapter *adapter = q_vector->adapter;
	u16 cleaned_count = rnp_desc_unused_rx(rx_ring);
	bool xdp_xmit = false;
	struct xdp_buff xdp;

	xdp.data = NULL;
	xdp.data_end = NULL;

	/*
	 * #ifdef HAVE_XDP_BUFF_RXQ
	 * xdp.rxq = &rx_ring->xdp_rxq;
	 * #endif
	 */
	while (likely(total_rx_packets < budget)) {
		union rnp_rx_desc *rx_desc;
		struct rnp_rx_buffer *rx_buffer;
		struct sk_buff *skb;
		unsigned int size;

		/* return some buffers to hardware, one at a time is too slow */
		if (cleaned_count >= RNP_RX_BUFFER_WRITE) {
			rnp_alloc_rx_buffers(rx_ring, cleaned_count);
			cleaned_count = 0;
		}
		rx_desc = RNP_RX_DESC(rx_ring, rx_ring->next_to_clean);

		rx_buf_dump("rx-desc:", rx_desc, sizeof(*rx_desc));
		// buf_dump("rx-desc:", rx_desc, sizeof(*rx_desc));
		rx_debug_printk("  dd set: %s\n",
						(rx_desc->wb.cmd & RNP_RXD_STAT_DD) ? "Yes" : "No");

		if (!rnp_test_staterr(rx_desc, RNP_RXD_STAT_DD))
			break;

		rx_debug_printk(
			"queue:%d  rx-desc:%d has-data len:%d next_to_clean %d\n",
			rx_ring->rnp_queue_idx,
			rx_ring->next_to_clean,
			rx_desc->wb.len,
			rx_ring->next_to_clean);

		/* handle padding */
		if ((adapter->priv_flags & RNP_PRIV_FLAG_FT_PADDING) &&
			(!(adapter->priv_flags & RNP_PRIV_FLAG_PADDING_DEBUG))) {
			if (likely(rnp_test_staterr(rx_desc, RNP_RXD_STAT_EOP))) {
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

		/*
		 * should check csum err
		 * maybe one packet use mutiple descs
		 * no problems hw set all csum_err in mutiple descs
		 * maybe BUG if the last sctp desc less than 60
		 */
		if (rnp_check_csum_error(
				rx_ring, rx_desc, size, &driver_drop_packets)) {
			cleaned_count++;
			continue;
		}
		/* This memory barrier is needed to keep us from reading
		 * any other fields out of the rx_desc until we know the
		 * descriptor has been written back
		 */
		dma_rmb();

		rx_buffer = rnp_get_rx_buffer(rx_ring, rx_desc, &skb, size);

		if (!skb) {
			xdp.data = page_address(rx_buffer->page) + rx_buffer->page_offset;
#ifdef HAVE_XDP_BUFF_DATA_META
			xdp.data_meta = xdp.data;
#endif
			xdp.data_hard_start = xdp.data - rnp_rx_offset(rx_ring);
			xdp.data_end = xdp.data + size;
			/* call  xdp hook  use this to support xdp hook */
			// skb = rnp_run_xdp(adapter, rx_ring, &xdp);
		}

		if (IS_ERR(skb)) {
			if (PTR_ERR(skb) == -RNP_XDP_TX) {
				xdp_xmit = true;
				rnp_rx_buffer_flip(rx_ring, rx_buffer, size);
			} else {
				rx_buffer->pagecnt_bias++;
			}
			total_rx_packets++;
			total_rx_bytes += size;
		} else if (skb) {
			rnp_add_rx_frag(rx_ring, rx_buffer, skb, size);
#ifdef HAVE_SWIOTLB_SKIP_CPU_SYNC
		} else if (ring_uses_build_skb(rx_ring)) {
			skb = rnp_build_skb(rx_ring, rx_buffer, &xdp, rx_desc);
#endif
		} else {
			skb = rnp_construct_skb(rx_ring, rx_buffer, &xdp, rx_desc);
		}

		/* exit if we failed to retrieve a buffer */
		if (!skb) {
			rx_ring->rx_stats.alloc_rx_buff_failed++;
			rx_buffer->pagecnt_bias++;
			break;
		}
		if (module_enable_ptp && adapter->ptp_rx_en &&
			adapter->flags2 & RNP_FLAG2_PTP_ENABLED)
			rnp_ptp_get_rx_hwstamp(adapter, rx_desc, skb);

		rnp_put_rx_buffer(rx_ring, rx_buffer, skb);
		cleaned_count++;

		/* place incomplete frames back on ring for completion */
		if (rnp_is_non_eop(rx_ring, rx_desc, skb))
			continue;

		/* verify the packet layout is correct */
		if (rnp_cleanup_headers(rx_ring, rx_desc, skb)) {
			// skb = NULL;
			continue;
		}

		/* probably a little skewed due to removing CRC */
		total_rx_bytes += skb->len;

		/* populate checksum, timestamp, VLAN, and protocol */
		rnp_process_skb_fields(rx_ring, rx_desc, skb);

		// rx_buf_dump("rx-data:", skb->data, skb->len);

		rnp_rx_skb(q_vector, skb);

		/* update budget accounting */
		total_rx_packets++;
	}

	// if (xdp_xmit) {
	// struct rnp_ring *ring = adapter->xdp_ring[smp_processor_id()];
	//
	//  /* Force memory writes to complete before letting h/w
	//   * know there are new descriptors to fetch.
	//   */
	// wmb();
	// writel(ring->next_to_use, ring->tail);
	//
	// xdp_do_flush_map();
	// }

	u64_stats_update_begin(&rx_ring->syncp);
	rx_ring->stats.packets += total_rx_packets;
	rx_ring->stats.bytes += total_rx_bytes;
	rx_ring->rx_stats.driver_drop_packets += driver_drop_packets;
	rx_ring->rx_stats.rx_clean_count += total_rx_packets;
	rx_ring->rx_stats.rx_clean_times++;
	if (rx_ring->rx_stats.rx_clean_times > 10) {
		rx_ring->rx_stats.rx_clean_times = 0;
		rx_ring->rx_stats.rx_clean_count = 0;
	}
	u64_stats_update_end(&rx_ring->syncp);
	q_vector->rx.total_packets += total_rx_packets;
	q_vector->rx.total_bytes += total_rx_bytes;

	// printk("clean rx irq %d\n", total_rx_packets);
	if (total_rx_packets >= budget)
		rx_ring->rx_stats.poll_again_count++;
	return total_rx_packets;
}
#else /* CONFIG_RNP_DISABLE_PACKET_SPLIT */

static bool rnp_alloc_mapped_skb(struct rnp_ring *rx_ring,
								 struct rnp_rx_buffer *bi)
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
 * rnp_clean_rx_irq - Clean completed descriptors from Rx ring - legacy
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
static int rnp_clean_rx_irq(struct rnp_q_vector *q_vector,
							struct rnp_ring *rx_ring,
							int budget)
{
	unsigned int total_rx_bytes = 0, total_rx_packets = 0;
	struct net_device *netdev = rx_ring->netdev;
	u16 len = 0;
	u16 cleaned_count = rnp_desc_unused_rx(rx_ring);

	while (likely(total_rx_packets < budget)) {
		struct rnp_rx_buffer *rx_buffer;
		union rnp_rx_desc *rx_desc;
		struct sk_buff *skb;
		u16 ntc;

		/* return some buffers to hardware, one at a time is too slow */
		if (cleaned_count >= RNP_RX_BUFFER_WRITE) {
			rnp_alloc_rx_buffers(rx_ring, cleaned_count);
			cleaned_count = 0;
		}

		ntc = rx_ring->next_to_clean;
		rx_desc = RNP_RX_DESC(rx_ring, ntc);
		rx_buffer = &rx_ring->rx_buffer_info[ntc];

		if (!rnp_test_staterr(rx_desc, RNP_RXD_STAT_DD))
			break;
		/*
		 * if (!rx_desc->wb.upper.length)
		 *  break;
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
		if (!RNP_CB(skb)->head) {
			RNP_CB(skb)->dma = rx_buffer->dma;
		} else {
			skb = rnp_merge_active_tail(skb);
			dma_unmap_single(rx_ring->dev,
							 rx_buffer->dma,
							 rx_ring->rx_buf_len,
							 DMA_FROM_DEVICE);
		}

		/* clear skb reference in buffer info structure */
		rx_buffer->skb = NULL;
		rx_buffer->dma = 0;

		cleaned_count++;

		if (rnp_is_non_eop(rx_ring, rx_desc, skb))
			continue;

		/* unmap first */
		dma_unmap_single(rx_ring->dev,
						 RNP_CB(skb)->dma,
						 rx_ring->rx_buf_len,
						 DMA_FROM_DEVICE);

		RNP_CB(skb)->dma = 0;

		if (rnp_close_active_frag_list(skb) && !RNP_CB(skb)->append_cnt) {
			/* if we got here without RSC the packet is invalid */
			dev_kfree_skb_any(skb);
			continue;
		}

		/* ERR_MASK will only have valid bits if EOP set */
		if (unlikely(rnp_test_staterr(rx_desc, RNP_RXD_STAT_ERR_MASK) &&
					 !(netdev->features & NETIF_F_RXALL))) {
			dev_kfree_skb_any(skb);
			continue;
		}

		/* probably a little skewed due to removing CRC */
		total_rx_bytes += skb->len;

		/* populate checksum, timestamp, VLAN, and protocol */
		rnp_process_skb_fields(rx_ring, rx_desc, skb);

		rnp_rx_skb(q_vector, skb);

		/* update budget accounting */
		total_rx_packets++;
	}

	u64_stats_update_begin(&rx_ring->syncp);
	rx_ring->stats.packets += total_rx_packets;
	rx_ring->stats.bytes += total_rx_bytes;
	u64_stats_update_end(&rx_ring->syncp);
	q_vector->rx.total_packets += total_rx_packets;
	q_vector->rx.total_bytes += total_rx_bytes;

	/* maybe not good here */
	// if (cleaned_count)
	//	rnp_alloc_rx_buffers(rx_ring, cleaned_count);

	if (total_rx_packets >= budget)
		rx_ring->rx_stats.poll_again_count++;

	return total_rx_packets;
}

#endif /* CONFIG_RNP_DISABLE_PACKET_SPLIT */

/**
 * rnp_configure_msix - Configure MSI-X hardware
 * @adapter: board private structure
 *
 * rnp_configure_msix sets up the hardware to properly generate MSI-X
 * interrupts.
 **/
static void rnp_configure_msix(struct rnp_adapter *adapter)
{
	struct rnp_q_vector *q_vector;
	int i;
	u32 mask;

	/*
	 * configure ring-msix Registers table
	 */
	for (i = 0; i < adapter->num_q_vectors; i++) {
		struct rnp_ring *ring;

		q_vector = adapter->q_vector[i];
		rnp_for_each_ring(ring, q_vector->rx)
		{
			rnp_set_ring_vector(adapter, ring->rnp_queue_idx, q_vector->v_idx);
		}
	}
}

/**
 * rnp_update_itr - update the dynamic ITR value based on statistics
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
static void rnp_update_itr(struct rnp_q_vector *q_vector,
						   struct rnp_ring_container *ring_container)
{
	unsigned int itr = RNP_ITR_ADAPTIVE_MIN_USECS | RNP_ITR_ADAPTIVE_LATENCY;
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
		itr = (q_vector->itr >> 2) + RNP_ITR_ADAPTIVE_MIN_INC;
		if (itr > RNP_ITR_ADAPTIVE_MAX_USECS)
			itr = RNP_ITR_ADAPTIVE_MAX_USECS;
		itr += ring_container->itr & RNP_ITR_ADAPTIVE_LATENCY;
		goto clear_counts;
	}

	bytes = ring_container->total_bytes;

	/* If packets are less than 4 or bytes are less than 9000 assume
	 * insufficient data to use bulk rate limiting approach. We are
	 * likely latency driven.
	 */
	if (packets < 4 && bytes < 9000) {
		itr = RNP_ITR_ADAPTIVE_LATENCY;
		goto adjust_by_size;
	}

	/* Between 4 and 48 we can assume that our current interrupt delay
	 * is only slightly too low. As such we should increase it by a small
	 * fixed amount.
	 */
	if (packets < 48) {
		itr = (q_vector->itr >> 2) + RNP_ITR_ADAPTIVE_MIN_INC;
		if (itr > RNP_ITR_ADAPTIVE_MAX_USECS)
			itr = RNP_ITR_ADAPTIVE_MAX_USECS;
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
		if (itr < RNP_ITR_ADAPTIVE_MIN_USECS)
			itr = RNP_ITR_ADAPTIVE_MIN_USECS;
		goto clear_counts;
	}

	/* The paths below assume we are dealing with a bulk ITR since number
	 * of packets is 256 or greater. We are just going to have to compute
	 * a value and try to bring the count under control, though for smaller
	 * packet sizes there isn't much we can do as NAPI polling will likely
	 * be kicking in sooner rather than later.
	 */
	itr = RNP_ITR_ADAPTIVE_BULK;

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
	if (itr & RNP_ITR_ADAPTIVE_LATENCY)
		avg_wire_size >>= 1;

	/* Resultant value is 256 times larger than it needs to be. This
	 * gives us room to adjust the value as needed to either increase
	 * or decrease the value based on link speeds of 10G, 2.5G, 1G, etc.
	 *
	 * Use addition as we have already recorded the new latency flag
	 * for the ITR value.
	 */
	switch (q_vector->adapter->link_speed) {
		case RNP_LINK_SPEED_10GB_FULL:
		case RNP_LINK_SPEED_100_FULL:
		default:
			itr += DIV_ROUND_UP(avg_wire_size, RNP_ITR_ADAPTIVE_MIN_INC * 256) *
				   RNP_ITR_ADAPTIVE_MIN_INC;
			break;
		// case RNP_LINK_SPEED_2_5GB_FULL:
		case RNP_LINK_SPEED_1GB_FULL:
			// case RNP_LINK_SPEED_10_FULL:
			itr += DIV_ROUND_UP(avg_wire_size, RNP_ITR_ADAPTIVE_MIN_INC * 64) *
				   RNP_ITR_ADAPTIVE_MIN_INC;
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

/**
 * rnp_write_eitr - write EITR register in hardware specific way
 * @q_vector: structure containing interrupt and ring information
 *
 * This function is made to be called by ethtool and by the driver
 * when it needs to update EITR registers at runtime.  Hardware
 * specific quirks/differences are taken care of here.
 */
void rnp_write_eitr(struct rnp_q_vector *q_vector)
{
	struct rnp_adapter *adapter = q_vector->adapter;
	struct rnp_hw *hw = &adapter->hw;
	int v_idx = q_vector->v_idx;
	// u32 itr_reg = q_vector->itr & RNP_MAX_EITR;
	u32 itr_reg = q_vector->itr >> 2;
	struct rnp_ring *ring;

	itr_reg = itr_reg * hw->usecstocount; // 150M
	rnp_for_each_ring(ring, q_vector->rx)
	{
		ring_wr32(ring, RNP_DMA_REG_TX_INT_DELAY_TIMER, itr_reg);
	}
	rnp_for_each_ring(ring, q_vector->tx)
	{
		ring_wr32(ring, RNP_DMA_REG_RX_INT_DELAY_TIMER, itr_reg);
	}
}

static void rnp_set_itr(struct rnp_q_vector *q_vector)
{
	u32 new_itr;

	rnp_update_itr(q_vector, &q_vector->tx);
	rnp_update_itr(q_vector, &q_vector->rx);

	/* use the smallest value of new ITR delay calculations */
	new_itr = min(q_vector->rx.itr, q_vector->tx.itr);

	/* Clear latency flag if set, shift into correct position */
	new_itr &= ~RNP_ITR_ADAPTIVE_LATENCY;
	/* in 2us unit */
	new_itr <<= 2;

	if (new_itr != q_vector->itr) {
		/* save the algorithm value here */
		q_vector->itr = new_itr;
		rnp_write_eitr(q_vector);
	}
}
enum latency_range {
	lowest_latency = 0,
	low_latency = 1,
	bulk_latency = 2,
	latency_invalid = 255
};

static void rnp_check_sfp_event(struct rnp_adapter *adapter, u32 eicr)
{
	struct rnp_hw *hw = &adapter->hw;
}

static inline void rnp_irq_enable_queues(struct rnp_adapter *adapter,
										 struct rnp_q_vector *q_vector)
{
	struct rnp_ring *ring;
	struct rnp_hw *hw = &adapter->hw;

	rnp_for_each_ring(ring, q_vector->rx)
	{
#ifdef CONFIG_RNP_DISABLE_TX_IRQ
		rnp_wr_reg(ring->dma_int_mask, ~(RX_INT_MASK));
#else
		rnp_wr_reg(ring->dma_int_mask, ~(RX_INT_MASK | TX_INT_MASK));
#endif
	}
}

static inline void rnp_irq_disable_queues(struct rnp_q_vector *q_vector)
{
	struct rnp_ring *ring;

	rnp_for_each_ring(ring, q_vector->tx)
	{
		// update usecs
		if (q_vector->new_rx_count != q_vector->old_rx_count) {
			ring_wr32(
				ring, RNP_DMA_REG_RX_INT_DELAY_PKTCNT, q_vector->new_rx_count);
			q_vector->old_rx_count = q_vector->new_rx_count;
		}
		rnp_wr_reg(ring->dma_int_mask, (RX_INT_MASK | TX_INT_MASK));
	}
}
/**
 * rnp_irq_enable - Enable default interrupt generation settings
 * @adapter: board private structure
 **/
static inline void rnp_irq_enable(struct rnp_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_q_vectors; i++) {
		rnp_irq_enable_queues(adapter, adapter->q_vector[i]);
	}
}

static irqreturn_t rnp_msix_other(int irq, void *data)
{
	struct rnp_adapter *adapter = data;
	struct rnp_hw *hw = &adapter->hw;
	u32 eicr;

	rnp_msg_task(adapter);

#if 0
	/* re-enable the original interrupt state, no lsc, no queues */
	if (!test_bit(__RNP_DOWN, &adapter->state))
		rnp_irq_enable(adapter);
#endif

	return IRQ_HANDLED;
}

static void rnp_htimer_start(struct rnp_q_vector *q_vector)
{
	unsigned long ns = q_vector->irq_check_usecs * NSEC_PER_USEC / 2;

	// hrtimer_start_range_ns(&q_vector->irq_miss_check_timer, ns_to_ktime(ns),
	//			       ns, HRTIMER_MODE_REL);
	hrtimer_start_range_ns(&q_vector->irq_miss_check_timer,
						   ns_to_ktime(ns),
						   ns,
						   HRTIMER_MODE_REL_PINNED);
}

static void rnp_htimer_stop(struct rnp_q_vector *q_vector)
{
	hrtimer_cancel(&q_vector->irq_miss_check_timer);
}

static irqreturn_t rnp_msix_clean_rings(int irq, void *data)
{
	struct rnp_q_vector *q_vector = data;

	// disable the hrtimer
	if (q_vector->vector_flags & RNP_QVECTOR_FLAG_IRQ_MISS_CHECK)
		rnp_htimer_stop(q_vector);
	/*  disabled interrupts (on this vector) for us */
	rnp_irq_disable_queues(q_vector);

	if (q_vector->rx.ring || q_vector->tx.ring)
		napi_schedule_irqoff(&q_vector->napi);

	return IRQ_HANDLED;
}

void update_rx_count(int cleaned, struct rnp_q_vector *q_vector)
{
#if 0
        struct rnp_ring_container *ring_container = &q_vector->rx;
        unsigned long next_update = jiffies;
        unsigned int packets, bytes;

        if ((cleaned) && (cleaned < 10)) {
                q_vector->new_rx_count = 1;
                goto clear_counts;
        }

        if (time_after(next_update, ring_container->next_update))
                goto clear_counts;

        packets = ring_container->total_packets;
        bytes = ring_container->total_bytes;

        if (packets < 5) {
                q_vector->new_rx_count /= 2;

        } else if (packets < 20) {
                //q_vector->new_rx_count = 1;
                q_vector->new_rx_count *= 2;
                if (q_vector->new_rx_count > 64)
                        q_vector->new_rx_count = 128;

                if (q_vector->old_rx_count != q_vector->new_rx_count) {
                        printk("%d change large %x, packets %d\n", q_vector->v_idx, q_vector->new_rx_count, packets);
                        printk("old %d new %d\n", q_vector->old_rx_count, q_vector->new_rx_count);
                }
        } else if (packets < 40) {
                // 48 - 96
                // do nothing
        } else {
                //q_vector->small_times = 0;
                //q_vector->new_rx_count -= (1 << (q_vector->large_times++));
                q_vector->new_rx_count /= 2;
                if (q_vector->new_rx_count == 0)
                        q_vector->new_rx_count = 1;
                if (q_vector->old_rx_count != q_vector->new_rx_count)
                        printk("%d change small %x, packets %d\n", q_vector->v_idx, q_vector->new_rx_count, packets);

        }


clear_counts:
        /* write back value */
        //ring_container->itr = itr;

        /* next update should occur within next jiffy */
        ring_container->next_update = next_update + 1;
        ring_container->total_bytes = 0;
        ring_container->total_packets = 0;
#else
        if ((cleaned) && (cleaned != q_vector->new_rx_count)) {
                //q_vector->new_rx_count = cleaned;
                if (cleaned < 5) {
                        q_vector->small_times = 0;
                        q_vector->large_times = 0;
                        q_vector->too_small_times++;
                        //q_vector->middle_time = 0;
                        if (q_vector->too_small_times >= 2) {
                                q_vector->new_rx_count = 1;
                        } //else {
                                //printk("%d delay change to 1 %d\n", q_vector->v_idx, q_vector->too_small_times);
                        //}
                        //if (q_vector->old_rx_count != q_vector->new_rx_count) {
                        //      printk("%d change to 1 %d \n", q_vector->v_idx, cleaned);
                        //      printk("old %d new %d\n", q_vector->old_rx_count, q_vector->new_rx_count);
                        //}

                } else if (cleaned < 30) {
                        q_vector->too_small_times = 0;
                        q_vector->middle_time++;
                        // count is 10 - 40
                        // try to keep in this stage
                        //if (q_vector->middle_time >= 2) {
                                //q_vector->new_rx_count =  cleaned - 2;
                                if (cleaned < q_vector->new_rx_count) {
                                        //change small
                                        q_vector->small_times = 0;
                                        q_vector->new_rx_count -= (1 << (q_vector->large_times++));
                                        if (q_vector->new_rx_count < 0)
                                                q_vector->new_rx_count = 1;
                                        //printk("%d change small %d %d\n", q_vector->v_idx, cleaned, q_vector->new_rx_count);

                                } else {
                                        q_vector->large_times = 0;

                                        if (cleaned > 30) {
                                                if (q_vector->new_rx_count == (cleaned - 4)) {

                                                } else {
                                                        q_vector->new_rx_count += (1 << (q_vector->small_times++));
                                                }
                                                // should no more than q_vector
                                                if (q_vector->new_rx_count >= cleaned) {
                                                        q_vector->new_rx_count =  cleaned - 4;
                                                        q_vector->small_times = 0;
                                                }


                                        } else {
                                                if (q_vector->new_rx_count == (cleaned - 1)) {

                                                } else {
                                                        q_vector->new_rx_count += (1 << (q_vector->small_times++));
                                                }
                                                // should no more than q_vector
                                                if (q_vector->new_rx_count >= cleaned) {
                                                        q_vector->new_rx_count =  cleaned - 1;
                                                        q_vector->small_times = 0;
                                                }


                                        }
                                        //change small
                                        //printk("%d change large %d %d\n", q_vector->v_idx, cleaned, q_vector->new_rx_count);
                                }
                        //}
                } else {
                        //printk("%d change to 128 %d", q_vector->new_rx_count, cleaned);
                        //q_vector->middle_time = 0;
                        q_vector->too_small_times = 0;
                        q_vector->new_rx_count = 128;
                        q_vector->small_times = 0;
                        q_vector->large_times = 0;
                        // 40 - 64
                }
        }
        /*
           if ((cleaned) && (cleaned != q_vector->new_rx_count)) {
        //q_vector->new_rx_count = cleaned;
        if (cleaned < 10) {
        q_vector->new_rx_count = 1;
        q_vector->small_times = 0;
        q_vector->large_times = 0;

        } else if (cleaned < q_vector->new_rx_count) {
        // count is large
        q_vector->small_times = 0;
        q_vector->new_rx_count -= (1 << (q_vector->large_times++));

        if (q_vector->new_rx_count < 0)
        q_vector->new_rx_count = 1;
        printk("change small %x %x\n", cleaned, q_vector->new_rx_count);
        } else {
        q_vector->large_times = 0;
        q_vector->new_rx_count += (1 << (q_vector->small_times++));
        //if (q_vector->new_rx_count > 32)
        //      q_vector->new_rx_count = 128;
        //q_vector->new_rx_count += 4;
        printk("change large %x %x\n", cleaned, q_vector->new_rx_count);
        //printk("update to 128\n");
        }
        }
        */
#endif

}

/**
 * rnp_poll - NAPI Rx polling callback
 * @napi: structure for representing this polling device
 * @budget: how many packets driver is allowed to clean
 *
 * This function is used for legacy and MSI, NAPI mode
 **/
int rnp_poll(struct napi_struct *napi, int budget)
{
	struct rnp_q_vector *q_vector =
		container_of(napi, struct rnp_q_vector, napi);
	struct rnp_adapter *adapter = q_vector->adapter;
	struct rnp_hw *hw = &adapter->hw;
	struct rnp_ring *ring;
	int per_ring_budget, work_done = 0;
	bool clean_complete = true;
	int cleaned_total = 0;

#ifdef CONFIG_RNP_DCA
	if (adapter->flags & RNP_FLAG_DCA_ENABLED)
		rnp_update_dca(q_vector);
#endif

	rnp_for_each_ring(ring, q_vector->tx)
	{
		if (!rnp_clean_tx_irq(q_vector, ring, budget))
			clean_complete = false;
	}

	/* attempt to distribute budget to each queue fairly, but don't allow
	 * the budget to go below 1 because we'll exit polling
	 */
	if (q_vector->rx.count > 1)
		per_ring_budget = max(budget / q_vector->rx.count, 1);
	else
		per_ring_budget = budget;

	rnp_for_each_ring(ring, q_vector->rx)
	{
		int cleaned = 0;
		/* this ring is waitting to reset rx_len*/
		/* avoid to deal this ring until reset done */
		if (likely(!(ring->ring_flags & RNP_RING_FLAG_DO_RESET_RX_LEN)))
			cleaned = rnp_clean_rx_irq(q_vector, ring, per_ring_budget);
		/* check delay rx setup */
		if (unlikely(ring->ring_flags & RNP_RING_FLAG_DELAY_SETUP_RX_LEN)) {
			int head;

			head = ring_rd32(ring, RNP_DMA_REG_RX_DESC_BUF_HEAD);
			if (head < RNP_MIN_RXD) {
				/* it is time to delay set */
				/* stop rx */
				rnp_disable_rx_queue(adapter, ring);
				ring->ring_flags &= (~RNP_RING_FLAG_DELAY_SETUP_RX_LEN);
				ring->ring_flags |= RNP_RING_FLAG_DO_RESET_RX_LEN;
			}
		}
		work_done += cleaned;
		cleaned_total += cleaned;
		if (cleaned >= per_ring_budget)
			clean_complete = false;
		/*
		if ((cleaned < 10) && (cleaned != 0))
			q_vector->new_rx_count = 1;
		else
			q_vector->new_rx_count = adapter->rx_frames;
		*/
	}

	update_rx_count(cleaned_total, q_vector);

#ifndef HAVE_NETDEV_NAPI_LIST
	if (!netif_running(adapter->netdev))
		clean_complete = true;
#endif

	/* all work done, exit the polling mode */
	// test only
	// clean_complete = false;
	/* If all work not completed, return budget and keep polling */
	if (!clean_complete) {
		// irq affinity update here
#ifdef REDUCE_TX_IRQ_MISS
		rnp_for_each_ring(ring, q_vector->tx)
		{
			rnp_check_restart_tx(q_vector, ring);
                        if (q_vector->new_rx_count != q_vector->old_rx_count) {
                                ring_wr32(ring, RNP_DMA_REG_RX_INT_DELAY_PKTCNT, q_vector->new_rx_count);
                                //ring_wr32(ring, RNP_DMA_REG_RX_INT_DELAY_TIMER,
                                //              q_vector->new_usesc * 500);
                                q_vector->old_rx_count = q_vector->new_rx_count;
                        }
		}
#endif
		return budget;
	}

	// napi_complete_done(napi, work_done);

	/* try to do itr handle */
#if ITR_TEST
	if (adapter->rx_itr_setting == 1)
		rnp_set_itr(q_vector);
#endif
	if (likely(napi_complete_done(napi, work_done))) {
		if (!test_bit(__RNP_DOWN, &adapter->state))
			rnp_irq_enable_queues(adapter, q_vector);
			/* we need this to ensure irq start before tx start */
#ifdef REDUCE_TX_IRQ_MISS
		smp_mb();
		rnp_for_each_ring(ring, q_vector->tx)
		{
			rnp_check_restart_tx(q_vector, ring);
			// update rx count now?
			if (q_vector->new_rx_count != q_vector->old_rx_count) {
				ring_wr32(ring,
						  RNP_DMA_REG_RX_INT_DELAY_PKTCNT,
						  q_vector->new_rx_count);
				q_vector->old_rx_count = q_vector->new_rx_count;
			}
		}
#endif
	}
	// start a hrtimer to check irq miss for this q_vector
	// rnp_htimer_stop(q_vector);
	if (q_vector->vector_flags & RNP_QVECTOR_FLAG_IRQ_MISS_CHECK)
		if (!test_bit(__RNP_DOWN, &adapter->state))
		rnp_htimer_start(q_vector);

	return min(work_done, budget - 1);
}

#ifdef HAVE_IRQ_AFFINITY_NOTIFY
#ifdef SUPPORT_IRQ_AFFINITY_CHANGE
/**
 * i40e_irq_affinity_notify - Callback for affinity changes
 * @notify: context as to what irq was changed
 * @mask: the new affinity mask
 *
 * This is a callback function used by the irq_set_affinity_notifier function
 * so that we may register to receive changes to the irq affinity masks.
 **/
static void rnp_irq_affinity_notify(struct irq_affinity_notify *notify,
									const cpumask_t *mask)
{

	struct rnp_q_vector *q_vector =
		container_of(notify, struct rnp_q_vector, affinity_notify);

	cpumask_copy(&q_vector->affinity_mask, mask);
}

/**
 * i40e_irq_affinity_release - Callback for affinity notifier release
 * @ref: internal core kernel usage
 *
 * This is a callback function used by the irq_set_affinity_notifier function
 * to inform the current notification subscriber that they will no longer
 * receive notifications.
 **/
static void rnp_irq_affinity_release(struct kref *ref)
{
}
#endif
#endif /* HAVE_IRQ_AFFINITY_NOTIFY */

/**
 * rnp_request_msix_irqs - Initialize MSI-X interrupts
 * @adapter: board private structure
 *
 * rnp_request_msix_irqs allocates MSI-X vectors and requests
 * interrupts from the kernel.
 **/
static int rnp_request_msix_irqs(struct rnp_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	struct rnp_hw *hw = &adapter->hw;
	int err;
	int i = 0;
	DPRINTK(IFUP,
			INFO,
			"[%s] num_q_vectors:%d\n",
			__func__,
			adapter->num_q_vectors);

	/* for mbx:vector0 */
	/*
	if (adapter->num_other_vectors) {
		err = request_irq(adapter->msix_entries[0].vector,
						  rnp_msix_other,
						  0,
						  netdev->name,
						  adapter);
		if (err) {
			e_err(probe, "request_irq for msix_other failed: %d\n", err);
			goto disable_msix;
		}
		hw->mbx.ops.configure(hw, adapter->msix_entries[0].entry, true);
	}
	*/

	for (i = 0; i < adapter->num_q_vectors; i++) {
		struct rnp_q_vector *q_vector = adapter->q_vector[i];
		struct msix_entry *entry =
			&adapter->msix_entries[i + adapter->q_vector_off];

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
			entry->vector, &rnp_msix_clean_rings, 0, q_vector->name, q_vector);
		if (err) {
			e_err(probe,
				  "%s:request_irq failed for MSIX interrupt:%d "
				  "Error: %d\n",
				  netdev->name,
				  entry->vector,
				  err);
			goto free_queue_irqs;
		}
		//adapter->hw.mbx.other_irq_enabled = true;
#ifdef HAVE_IRQ_AFFINITY_NOTIFY
		/* register for affinity change notifications */
#ifdef SUPPORT_IRQ_AFFINITY_CHANGE
		q_vector->affinity_notify.notify = rnp_irq_affinity_notify;
		q_vector->affinity_notify.release = rnp_irq_affinity_release;
		irq_set_affinity_notifier(entry->vector, &q_vector->affinity_notify);
#endif /* SUPPORT_IRQ_AFFINITY_CHANGE */
#endif /* HAVE_IRQ_AFFINITY_NOTIFY */
#ifdef HAVE_IRQ_AFFINITY_HINT
		DPRINTK(IFUP,
				INFO,
				"[%s] set %s affinity_mask\n",
				__func__,
				q_vector->name);

		irq_set_affinity_hint(entry->vector, &q_vector->affinity_mask);
#endif
	}

	return 0;

free_queue_irqs:
	while (i) {
		i--;
		irq_set_affinity_hint(
			adapter->msix_entries[i + adapter->q_vector_off].vector, NULL);
		free_irq(adapter->msix_entries[i + adapter->q_vector_off].vector,
				 adapter->q_vector[i]);
#ifdef HAVE_IRQ_AFFINITY_NOTIFY
#ifdef SUPPORT_IRQ_AFFINITY_CHANGE
		irq_set_affinity_notifier(
			adapter->msix_entries[i + adapter->q_vector_off].vector, NULL);
#endif
#endif
#ifdef HAVE_IRQ_AFFINITY_HINT
		irq_set_affinity_hint(
			adapter->msix_entries[i + adapter->q_vector_off].vector, NULL);
#endif
	}
	// if(adapter->num_other_vectors){
	//	free_irq(adapter->msix_entries[0].vector, adapter);
	// }
	//	adapter->hw.mbx.other_irq_enabled = false;
	// fixme
	// disable_msix:
	//	pci_disable_msix(adapter->pdev);
	//	kfree(adapter->msix_entries);
	//	adapter->msix_entries = NULL;
	return err;
}

#ifdef DISABLE_RX_IRQ
int rx_poll_thread_handler(void *data)
{
	int i;
	struct rnp_adapter *adapter = data;

	dbg("%s  %s running...\n", __func__, adapter->name);

	do {
		for (i = 0; i < adapter->num_q_vectors; i++) {
			rnp_msix_clean_rings(0, adapter->q_vector[i]);
		}

		msleep(30);
	} while (!kthread_should_stop() && adapter->quit_poll_thread != true);

	dbg("%s  %s stoped\n", __func__, adapter->name);

	return 0;
}
#endif

/**
 * rnp_request_irq - initialize interrupts
 * @adapter: board private structure
 *
 * Attempts to configure interrupts using the best available
 * capabilities of the hardware and kernel.
 **/
static int rnp_request_irq(struct rnp_adapter *adapter)
{
	int err;

#ifdef DISABLE_RX_IRQ
	adapter->rx_poll_thread =
		kthread_run(rx_poll_thread_handler, adapter, adapter->name);
	if (!adapter->rx_poll_thread) {
		rnp_err("kthread_run faild!\n");
		return -EIO;
	}
	return 0;
#endif

	err = rnp_request_msix_irqs(adapter);
	if (err)
		e_err(probe, "request_irq failed, Error %d\n", err);

	return err;
}

static void rnp_free_irq(struct rnp_adapter *adapter)
{
	int i = 0;

#ifdef DISABLE_RX_IRQ
	return;
#endif

	// rnp_dbg("[%s] num_q_vectors:%d\n", __func__, adapter->num_q_vectors);

	/* mbx */
	/*
	if (adapter->num_other_vectors) {
		adapter->hw.mbx.ops.configure(
			&adapter->hw, adapter->msix_entries[0].entry, false);
		free_irq(adapter->msix_entries[0].vector, adapter);
	}*/

	for (i = 0; i < adapter->num_q_vectors; i++) {
		struct rnp_q_vector *q_vector = adapter->q_vector[i];
		struct msix_entry *entry =
			&adapter->msix_entries[i + adapter->q_vector_off];

		/* free only the irqs that were actually requested */
		if (!q_vector->rx.ring && !q_vector->tx.ring)
			continue;
#ifdef HAVE_IRQ_AFFINITY_NOTIFY
		/* clear the affinity notifier in the IRQ descriptor */
		irq_set_affinity_notifier(entry->vector, NULL);
#endif
#ifdef HAVE_IRQ_AFFINITY_HINT
		/* clear the affinity_mask in the IRQ descriptor */
		irq_set_affinity_hint(entry->vector, NULL);
#endif
		DPRINTK(IFDOWN, INFO, "free irq %s\n", q_vector->name);
		free_irq(entry->vector, q_vector);
	}
	// adapter->hw.mbx.other_irq_enabled = false;
}

/**
 * rnp_irq_disable - Mask off interrupt generation on the NIC
 * @adapter: board private structure
 **/
static inline void rnp_irq_disable(struct rnp_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_q_vectors; i++) {
		rnp_irq_disable_queues(adapter->q_vector[i]);
		synchronize_irq(
			adapter->msix_entries[i + adapter->q_vector_off].vector);
	}
}

int rnp_setup_tx_maxrate(struct rnp_ring *tx_ring,
						 u64 max_rate,
						 int samples_1sec)
{
	/* set hardware samping internal 1S */
	ring_wr32(tx_ring, RNP_DMA_REG_TX_FLOW_CTRL_TM, samples_1sec);
	ring_wr32(tx_ring, RNP_DMA_REG_TX_FLOW_CTRL_TH, max_rate);

	return 0;
}

/**
 * rnp_tx_maxrate_own - callback to set the maximum per-queue bitrate
 * @netdev: network interface device structure
 * @queue_index: Tx queue to set
 * @maxrate: desired maximum transmit bitrate Mbps
 **/
static int rnp_tx_maxrate_own(struct rnp_adapter *adapter, int queue_index)
{
	struct rnp_ring *tx_ring = adapter->tx_ring[queue_index];
	u64 real_rate = 0;
	u32 maxrate = adapter->max_rate[queue_index];

	if (!maxrate)
		return rnp_setup_tx_maxrate(
			tx_ring, 0, adapter->hw.usecstocount * 1000000);
	/* we need turn it to bytes/s */
	real_rate = ((u64)maxrate * 1024 * 1024) / 8;
	rnp_setup_tx_maxrate(
		tx_ring, real_rate, adapter->hw.usecstocount * 1000000);

	return 0;
}

/**
 * rnp_configure_tx_ring - Configure 8259x Tx ring after Reset
 * @adapter: board private structure
 * @ring: structure containing ring specific data
 *
 * Configure the Tx descriptor ring after a reset.
 **/
void rnp_configure_tx_ring(struct rnp_adapter *adapter, struct rnp_ring *ring)
{
	struct rnp_hw *hw = &adapter->hw;

	struct rnp_dma_info *dma = &hw->dma;
	u8 queue_idx = ring->rnp_queue_idx;

	/* disable queue to avoid issues while updating state */

	if (!(ring->ring_flags & RNP_RING_SKIP_TX_START))
		ring_wr32(ring, RNP_DMA_TX_START, 0);

	ring_wr32(ring, RNP_DMA_REG_TX_DESC_BUF_BASE_ADDR_LO, (u32)ring->dma);
	ring_wr32(ring,
			  RNP_DMA_REG_TX_DESC_BUF_BASE_ADDR_HI,
			  (u32)(((u64)ring->dma) >> 32) | (hw->pfvfnum << 24));
	ring_wr32(ring, RNP_DMA_REG_TX_DESC_BUF_LEN, ring->count);

	// tail <= head
	ring->next_to_clean = ring_rd32(ring, RNP_DMA_REG_TX_DESC_BUF_HEAD);
	ring->next_to_use = ring->next_to_clean;
	ring->tail = ring->ring_addr + RNP_DMA_REG_TX_DESC_BUF_TAIL;
	rnp_wr_reg(ring->tail, ring->next_to_use);

	// fixme
	if (adapter->flags & RNP_FLAG_SRIOV_ENABLED) {
		ring_wr32(ring,
				  RNP_DMA_REG_TX_DESC_FETCH_CTRL,
				  (8 << 0) /* max_water_flow */
					  | (8 << 16)
				  /* max-num_descs_peer_read */
		);

	} else {
		ring_wr32(ring,
				  RNP_DMA_REG_TX_DESC_FETCH_CTRL,
				  (64 << 0) /* max_water_flow */
					  | (TSRN10_TX_DEFAULT_BURST << 16)
				  /* max-num_descs_peer_read */
		);
	}
	ring_wr32(ring,
			  RNP_DMA_REG_TX_INT_DELAY_TIMER,
			  adapter->tx_usecs * hw->usecstocount);
	ring_wr32(ring, RNP_DMA_REG_TX_INT_DELAY_PKTCNT, adapter->tx_frames);

	rnp_tx_maxrate_own(adapter, ring->queue_index);
	/* flow control: bytes-peer-ctrl-tm-clk. 0:no-control */
	// wr32(hw, RNP_DMA_REG_TX_FLOW_CTRL_TH(queue_idx),
	//     0x0);
	/* reinitialize flowdirector state */
	if (adapter->flags & RNP_FLAG_FDIR_HASH_CAPABLE) {
		ring->atr_sample_rate = adapter->atr_sample_rate;
		ring->atr_count = 0;
		set_bit(__RNP_TX_FDIR_INIT_DONE, &ring->state);
	} else {
		ring->atr_sample_rate = 0;
	}
	/* initialize XPS */
	if (!test_and_set_bit(__RNP_TX_XPS_INIT_DONE, &ring->state)) {
		struct rnp_q_vector *q_vector = ring->q_vector;

		if (q_vector)
			netif_set_xps_queue(
				adapter->netdev, &q_vector->affinity_mask, ring->queue_index);
	}

	clear_bit(__RNP_HANG_CHECK_ARMED, &ring->state);

	// enable queue
	if (!(ring->ring_flags & RNP_RING_SKIP_TX_START)) {
		/* n500 should wait tx_ready before open tx start */
		int timeout = 0;
		u32 status = 0;

		do {
			status = ring_rd32(ring, RNP_DMA_TX_READY);
			usleep_range(100, 200);
			timeout++;
			rnp_dbg("wait %d tx ready to 1\n", ring->rnp_queue_idx);
		} while ((status != 1) && (timeout < 100));

		if (timeout >= 100)
			printk("wait tx ready timeout\n");
		ring_wr32(ring, RNP_DMA_TX_START, 1);
	}
}

static void rnp_setup_mtqc(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	u32 rttdcs, mtqc;
	u8 tcs = netdev_get_num_tc(adapter->netdev);

#if 0
	/* disable the arbiter while setting MTQC */
	rttdcs = RNP_READ_REG(hw, RNP_RTTDCS);
	rttdcs |= RNP_RTTDCS_ARBDIS;
	RNP_WRITE_REG(hw, RNP_RTTDCS, rttdcs);

	/* set transmit pool layout */
	if (adapter->flags & RNP_FLAG_SRIOV_ENABLED) {
		mtqc = RNP_MTQC_VT_ENA;
		if (tcs > 4)
			mtqc |= RNP_MTQC_RT_ENA | RNP_MTQC_8TC_8TQ;
		else if (tcs > 1)
			mtqc |= RNP_MTQC_RT_ENA | RNP_MTQC_4TC_4TQ;
		else if (adapter->ring_feature[RING_F_RSS].indices == 4)
			mtqc |= RNP_MTQC_32VF;
		else
			mtqc |= RNP_MTQC_64VF;
	} else {
		if (tcs > 4)
			mtqc = RNP_MTQC_RT_ENA | RNP_MTQC_8TC_8TQ;
		else if (tcs > 1)
			mtqc = RNP_MTQC_RT_ENA | RNP_MTQC_4TC_4TQ;
		else
			mtqc = RNP_MTQC_64Q_1PB;
	}

	RNP_WRITE_REG(hw, RNP_MTQC, mtqc);

	/* Enable Security TX Buffer IFG for multiple pb */
	if (tcs) {
		u32 sectx = RNP_READ_REG(hw, RNP_SECTXMINIFG);

		sectx |= RNP_SECTX_DCB;
		RNP_WRITE_REG(hw, RNP_SECTXMINIFG, sectx);
	}

	/* re-enable the arbiter */
	rttdcs &= ~RNP_RTTDCS_ARBDIS;
	RNP_WRITE_REG(hw, RNP_RTTDCS, rttdcs);
#endif
}

/**
 * rnp_configure_tx - Configure Transmit Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx unit of the MAC after a reset.
 **/
static void rnp_configure_tx(struct rnp_adapter *adapter)
{
	u32 i, dma_axi_ctl;
	struct rnp_hw *hw = &adapter->hw;
	struct rnp_dma_info *dma = &hw->dma;

	rnp_setup_mtqc(adapter);

	/* dma_axi_en.tx_en must be before Tx queues are enabled */
	// fixme with dma_wr32
	dma_axi_ctl = dma_rd32(dma, RNP_DMA_AXI_EN);
	dma_axi_ctl |= TX_AXI_RW_EN;
	dma_wr32(dma, RNP_DMA_AXI_EN, dma_axi_ctl);

	/* Setup the HW Tx Head and Tail descriptor pointers */
	for (i = 0; i < (adapter->num_tx_queues); i++)
		rnp_configure_tx_ring(adapter, adapter->tx_ring[i]);
}

void rnp_disable_rx_queue(struct rnp_adapter *adapter, struct rnp_ring *ring)
{
	struct rnp_hw *hw = &adapter->hw;

	ring_wr32(ring, RNP_DMA_RX_START, 0);
}

void rnp_configure_rx_ring(struct rnp_adapter *adapter, struct rnp_ring *ring)
{
	struct rnp_hw *hw = &adapter->hw;
	u64 desc_phy = ring->dma;
	u16 q_idx = ring->rnp_queue_idx;

	/* disable queue to avoid issues while updating state */
	rnp_disable_rx_queue(adapter, ring);

	/* set descripts registers*/
	ring_wr32(ring, RNP_DMA_REG_RX_DESC_BUF_BASE_ADDR_LO, (u32)desc_phy);
	ring_wr32(ring,
			  RNP_DMA_REG_RX_DESC_BUF_BASE_ADDR_HI,
			  ((u32)(desc_phy >> 32)) | (hw->pfvfnum << 24));
	ring_wr32(ring, RNP_DMA_REG_RX_DESC_BUF_LEN, ring->count);

	ring->tail = ring->ring_addr + RNP_DMA_REG_RX_DESC_BUF_TAIL;
	ring->next_to_clean = ring_rd32(ring, RNP_DMA_REG_RX_DESC_BUF_HEAD);
	ring->next_to_use = ring->next_to_clean;

	if (ring->ring_flags & RNP_RING_SCATER_SETUP)
		ring_wr32(ring, PCI_DMA_REG_RX_SCATTER_LENGH, 96);
	// fixme
	if (adapter->flags & RNP_FLAG_SRIOV_ENABLED) {
		ring_wr32(ring,
				  RNP_DMA_REG_RX_DESC_FETCH_CTRL,
				  0 | (TSRN10_RX_DEFAULT_LINE << 0) /* rx-desc-flow */
					  | (TSRN10_RX_DEFAULT_BURST << 16)
				  /* max-read-desc-cnt */
		);

	} else {
		ring_wr32(ring,
				  RNP_DMA_REG_RX_DESC_FETCH_CTRL,
				  0 | (TSRN10_RX_DEFAULT_LINE << 0) /* rx-desc-flow */
					  | (TSRN10_RX_DEFAULT_BURST << 16)
				  /* max-read-desc-cnt */
		);
	}
	ring_wr32(ring,
			  RNP_DMA_REG_RX_INT_DELAY_TIMER,
			  adapter->rx_usecs * hw->usecstocount);
	ring_wr32(ring, RNP_DMA_REG_RX_INT_DELAY_PKTCNT, adapter->rx_frames);

	rnp_alloc_rx_buffers(ring, rnp_desc_unused_rx(ring));

	/* enable receive descriptor ring */
	// wr32(hw, RNP_DMA_RX_START(q_idx), 1);
}

static void rnp_configure_virtualization(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	struct rnp_dma_info *dma = &hw->dma;
	u8 *mac;
	u32 maclow, machi, ring, vfnum;
	u8 port = 0; // FIXME

	if (!(adapter->flags & RNP_FLAG_SRIOV_ENABLED))
		return;

	/* Enable only the PF's pool for Tx/Rx */

	if (adapter->flags2 & RNP_FLAG2_BRIDGE_MODE_VEB) {
		dma_wr32(dma,
				 RNP_DMA_CONFIG,
				 dma_rd32(dma, RNP_DMA_CONFIG) & (~DMA_VEB_BYPASS));
		adapter->flags2 |= RNP_FLAG2_BRIDGE_MODE_VEB;
	}
#if 1
	ring = adapter->tx_ring[0]->rnp_queue_idx;

	hw->ops.set_sriov_status(hw, true);

	// enable find vf by dest-mac-address
	// wr32(hw, RNP_HOST_FILTER_EN, 1);
	// wr32(hw, RNP_REDIR_EN, 1);
	// wr32(hw, RNP_MRQC_IOV_EN, RNP_IOV_ENABLED);
	// wr32(hw, RNP_ETH_DMAC_FCTRL,
	// rd32(hw, RNP_ETH_DMAC_FCTRL) | RNP_FCTRL_BROADCASE_BYPASS);
	// wr32(hw, RNP_ETH_DMAC_MCSTCTRL, RNP_MCSTCTRL_DMAC_47);
	/* Map PF MAC address in RAR Entry 0 to first pool following VFs */
	// hw->mac.ops.set_vmdq(hw, 0, ring / 2);
#endif

	// set VEB table, so pf can receive broadcast. and can ping vf
	//	if (is_valid_ether_addr(adapter->netdev->dev_addr))
	//		mac = adapter->netdev->dev_addr;
	//	else
	//		mac = hw->mac.perm_addr;
	//	dbg("%s %x:%x:%x:%x:%x:%x\n", __func__, mac[0], mac[1], mac[2],
	//	       mac[3], mac[4], mac[5]);
	//	port = 0;
	//	if (rd32(hw, RNP_DMA_VERSION) >= 0x20201231) {
	//		for (port = 0; port < 4; port++) {
	//			// 4 veb table to same value in 2port mode
	//			vfnum = RNP_MAX_VF_CNT -
	//				1; // use last-vf's table entry. the last
	//			maclow = (mac[2] << 24) | (mac[3] << 16) |
	//				 (mac[4] << 8) | mac[5];
	//			machi = (mac[0] << 8) | mac[1];
	//			wr32(hw, RNP_DMA_PORT_VBE_MAC_LO_TBL(port, vfnum),
	//			     maclow);
	//			wr32(hw, RNP_DMA_PORT_VBE_MAC_HI_TBL(port, vfnum),
	//			     machi);
	//			ring |= ((0x80 | vfnum) << 8);
	//			wr32(hw, RNP_DMA_PORT_VEB_VF_RING_TBL(port, vfnum),
	//			     ring);
	//		}
	//	} else {
	//			// 4 veb table to same value in 2port mode
	//			vfnum = RNP_MAX_VF_CNT -
	//				1; // use last-vf's table entry. the last
	//		// use last-vf's table entry. the las
	//		maclow = (mac[2] << 24) | (mac[3] << 16) | (mac[4] << 8) |
	//			 mac[5];
	//		machi = (mac[0] << 8) | mac[1];
	//		wr32(hw, RNP_DMA_PORT_VBE_MAC_LO_TBL(port, vfnum), maclow);
	//		wr32(hw, RNP_DMA_PORT_VBE_MAC_HI_TBL(port, vfnum), machi);
	//		ring |= ((0x80 | vfnum) << 8);
	//		wr32(hw, RNP_DMA_PORT_VEB_VF_RING_TBL(port, vfnum), ring);
	//	}
	/* store vfnum */
	vfnum = hw->max_vfs - 1;
	hw->veb_ring = ring;
	hw->vfnum = vfnum;
	// use last-vf's table entry. the last
	adapter->vf_num_for_pf = 0x80 | vfnum;
}

static void rnp_set_rx_buffer_len(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	struct net_device *netdev = adapter->netdev;
	int max_frame = netdev->mtu + ETH_HLEN + ETH_FCS_LEN;
	struct rnp_ring *rx_ring;
	int i;
	u32 mhadd, hlreg0;
	// int max_frame = netdev->mtu + ETH_HLEN + ETH_FCS_LEN;

	if (max_frame < (ETH_FRAME_LEN + ETH_FCS_LEN))
		max_frame = (ETH_FRAME_LEN + ETH_FCS_LEN);

	for (i = 0; i < adapter->num_rx_queues; i++) {
		rx_ring = adapter->rx_ring[i];
#ifndef CONFIG_RNP_DISABLE_PACKET_SPLIT
		clear_bit(__RNP_RX_3K_BUFFER, &rx_ring->state);
		clear_bit(__RNP_RX_BUILD_SKB_ENABLED, &rx_ring->state);
#ifdef HAVE_SWIOTLB_SKIP_CPU_SYNC

		set_bit(__RNP_RX_BUILD_SKB_ENABLED, &rx_ring->state);
		hw_dbg(hw, "set build skb\n");

#if (PAGE_SIZE < 8192)
		if (RNP_2K_TOO_SMALL_WITH_PADDING ||
			(max_frame > (ETH_FRAME_LEN + ETH_FCS_LEN)))
			;
			//	set_bit(__RNP_RX_3K_BUFFER, &rx_ring->state);
#endif

#else /* !HAVE_SWIOTLB_SKIP_CPU_SYNC */
		/* fixed this */
		hw_dbg(hw, "set construct skb\n");

#endif /* HAVE_SWIOTLB_SKIP_CPU_SYNC */
#else
		rx_ring->rx_buf_len = RNP_RXBUFFER_2K;
#endif /* CONFIG_RNP_DISABLE_PACKET_SPLIT */
	}
}

/**
 * rnp_configure_rx - Configure 8259x Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Rx unit of the MAC after a reset.
 **/
static void rnp_configure_rx(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	struct rnp_dma_info *dma = &hw->dma;
	int i;
	u32 rxctrl = 0, dma_axi_ctl;

	/* disable receives while setting up the descriptors */
#if 0
	rnp_setup_psrtype(adapter);
	rnp_setup_rdrxctl(adapter);

	/* Program registers for the distribution of queues */
	rnp_setup_mrqc(adapter);
#endif

	/* set_rx_buffer_len must be called before ring initialization */
	rnp_set_rx_buffer_len(adapter);

	/*
	 * Setup the HW Rx Head and Tail Descriptor Pointers and
	 * the Base and Length of the Rx Descriptor Ring
	 */
	for (i = 0; i < adapter->num_rx_queues; i++)
		rnp_configure_rx_ring(adapter, adapter->rx_ring[i]);

	if (adapter->num_rx_queues > 0) {
		wr32(hw, RNP_ETH_DEFAULT_RX_RING, adapter->rx_ring[0]->rnp_queue_idx);
	}

	/* enable all receives */
	rxctrl |= 0;

	dma_axi_ctl = dma_rd32(dma, RNP_DMA_AXI_EN);
	dma_axi_ctl |= RX_AXI_RW_EN;
	dma_wr32(dma, RNP_DMA_AXI_EN, dma_axi_ctl);

	// hw->mac.ops.enable_rx_dma(hw, rxctrl);
}

#if defined(NETIF_F_HW_VLAN_TX) || defined(NETIF_F_HW_VLAN_CTAG_TX)

#ifdef HAVE_INT_NDO_VLAN_RX_ADD_VID
#ifdef NETIF_F_HW_VLAN_CTAG_TX
static int rnp_vlan_rx_add_vid(struct net_device *netdev,
							   __always_unused __be16 proto,
							   u16 vid)
#else  /* !NETIF_F_HW_VLAN_CTAG_TX */
static int rnp_vlan_rx_add_vid(struct net_device *netdev, u16 vid)
#endif /* NETIF_F_HW_VLAN_CTAG_TX */
#else  /* !HAVE_INT_NDO_VLAN_RX_ADD_VID */
static void rnp_vlan_rx_add_vid(struct net_device *netdev, u16 vid)
#endif /* HAVE_INT_NDO_VLAN_RX_ADD_VID */
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw *hw = &adapter->hw;
	int port = 0;

	bool sriov_flag = !!(adapter->flags & RNP_FLAG_SRIOV_ENABLED);

	if (sriov_flag) {
		/* in sriov mode */
		if ((vid) && (adapter->vf_vlan) && (vid != adapter->vf_vlan)) {
			dev_err(&adapter->pdev->dev, "only 1 vlan in sriov mode \n");
#ifdef HAVE_INT_NDO_VLAN_RX_ADD_VID
			return -EACCES;
#else
			return;
#endif
		}

		/* update this */
		if (vid)
			adapter->vf_vlan = vid;
	}

#ifndef HAVE_VLAN_RX_REGISTER
	// record vlan count
	if (vid) {
		if (!test_bit(vid, adapter->active_vlans)) {
			adapter->vlan_count++;
		}
	}

	if (vid < VLAN_N_VID)
		set_bit(vid, adapter->active_vlans);
#endif

	if (hw->ops.set_vlan_filter) {
		hw->ops.set_vlan_filter(hw, vid, true, sriov_flag);
	}

//	if (hw->mac.ops.set_vfta) {
//
//		if (adapter->flags & RNP_FLAG_SRIOV_ENABLED) {
//			/* in sriov mode */
//			if ((vid) && (adapter->vf_vlan) && ( vid != adapter->vf_vlan)) {
//				dev_err(&adapter->pdev->dev, "only 1 vlan in sriov mode \n");
//#ifdef HAVE_INT_NDO_VLAN_RX_ADD_VID
//				return -EACCES;
//#else
//				return;
//#endif
//			}
//
//		}
//
//
//#ifndef HAVE_VLAN_RX_REGISTER
//		// record vlan count
//		if (vid) {
//			if (!test_bit(vid, adapter->active_vlans)) {
//				adapter->vlan_count++;
//			}
//		}
//
//		if (vid < VLAN_N_VID)
//			set_bit(vid, adapter->active_vlans);
//#endif
//		/* config to hw */
//		if (adapter->flags & RNP_FLAG_SRIOV_ENABLED) {
//			u8 vfnum = RNP_MAX_VF_CNT - 1;
//
//			hw->mac.ops.set_vfta(&adapter->hw, vid, VMDQ_P(0), true);
//
//			if (vid) {
//				adapter->vf_vlan = vid;
//
//
//				if (rd32(hw, RNP_DMA_VERSION) >= 0x20201231) {
//					for (port = 0; port < 4; port++)
//						/* each vf can support only one vlan */
//						wr32(hw,
//								RNP_DMA_PORT_VEB_VID_TBL(port,
//									vfnum), vid);
//				} else {
//					wr32(hw,
//							RNP_DMA_PORT_VEB_VID_TBL(adapter->port,
//								vfnum), vid);
//				}
//			}
//		} else {
//			// maybe bug ?
//			// if 1 add vlan, 2 then open sriov 3 add pf vlan again ?
//			/* add VID to filter table */
//			hw->mac.ops.set_vfta(&adapter->hw, vid, VMDQ_P(0), true);
//
//		}
//	}
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
static int rnp_vlan_rx_kill_vid(struct net_device *netdev,
								__always_unused __be16 proto,
								u16 vid)
#else  /* !NETIF_F_HW_VLAN_CTAG_RX */
static int rnp_vlan_rx_kill_vid(struct net_device *netdev, u16 vid)
#endif /* NETIF_F_HW_VLAN_CTAG_RX */
#else
static void rnp_vlan_rx_kill_vid(struct net_device *netdev, u16 vid)
#endif
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw *hw = &adapter->hw;
	int i;
	bool sriov_flag = !!(adapter->flags & RNP_FLAG_SRIOV_ENABLED);

	if (!vid)
#ifdef HAVE_INT_NDO_VLAN_RX_ADD_VID
		return 0;
#else
		return;
#endif

#ifdef HAVE_VLAN_RX_REGISTER
	if (!test_bit(__RNP_DOWN, &adapter->state))
		rnp_irq_disable(adapter);

	vlan_group_set_device(adapter->vlgrp, vid, NULL);

	if (!test_bit(__RNP_DOWN, &adapter->state))
		rnp_irq_enable(adapter);

#endif /* HAVE_VLAN_RX_REGISTER */

	if (sriov_flag) {
		if (vid) {
			int true_remove = 1;
			// clean this
			adapter->vf_vlan = 0;
			for (i = 0; i < adapter->num_vfs; i++) {
				if (vid == adapter->vfinfo[i].vf_vlan)
					true_remove = 0;
			}
			if (true_remove) {

				hw->ops.set_vlan_filter(hw, vid, false, sriov_flag);
			}

			// remove veb
		}

	} else {
		hw->ops.set_vlan_filter(hw, vid, false, sriov_flag);
	}

//	if (hw->mac.ops.set_vfta) {
//		// clear_bit(vid, adapter->active_vlans);
//
//		if ((adapter->flags & RNP_FLAG_SRIOV_ENABLED)) {
//			u8 vfnum = RNP_MAX_VF_CNT - 1;
//
//			if (vid) {
//				int true_remove = 1;
//				// clean this
//				adapter->vf_vlan = 0;
//				for (i = 0; i < adapter->num_vfs; i++) {
//					if (vid == adapter->vfinfo[i].vf_vlan)
//						true_remove = 0;
//				}
//
//				if (true_remove) {
//					hw->mac.ops.set_vfta(&adapter->hw, vid, VMDQ_P(0), false);
//				}
//			}
//
//			if (rd32(hw, RNP_DMA_VERSION) >= 0x20201231) {
//				int port;
//
//				for (port = 0; port < 4; port++)
//					wr32(hw,
//					RNP_DMA_PORT_VEB_VID_TBL(
//					port, vfnum),
//					0);
//			} else {
//				wr32(hw,
//				RNP_DMA_PORT_VEB_VID_TBL(
//				adapter->port, vfnum),
//				0);
//			}
//		} else {
//			/* remove VID from filter table */
//			hw->mac.ops.set_vfta(&adapter->hw, vid, VMDQ_P(0), false);
//		}
//	}
#ifndef HAVE_VLAN_RX_REGISTER
	if (vid) {
		if (test_bit(vid, adapter->active_vlans)) {
			adapter->vlan_count--;
		}
	}
	clear_bit(vid, adapter->active_vlans);
#endif
#ifdef HAVE_INT_NDO_VLAN_RX_ADD_VID
	return 0;
#endif
}

#endif

/**
 * rnp_vlan_filter_disable - helper to disable hw vlan filtering
 * @adapter: driver data
 */
static void rnp_vlan_filter_disable(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	u32 vlnctrl;

	rnp_vlan_filter_off(hw);
}

/**
 * rnp_vlan_filter_enable - helper to enable hw vlan filtering
 * @adapter: driver data
 */
static void rnp_vlan_filter_enable(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	u32 vlnctrl;

	rnp_vlan_filter_on(hw);
}

/**
 * rnp_vlan_strip_disable - helper to disable hw vlan stripping
 * @adapter: driver data
 */
static void rnp_vlan_strip_disable(struct rnp_adapter *adapter)
{
	int i;
	struct rnp_ring *tx_ring;
	struct rnp_hw *hw = &adapter->hw;

	for (i = 0; i < adapter->num_rx_queues; i++) {
		tx_ring = adapter->rx_ring[i];
		// hw_queue_strip_rx_vlan(hw, tx_ring->rnp_queue_idx, false);
		hw->ops.set_vlan_strip(hw, tx_ring->rnp_queue_idx, false);
	}
}

/**
 * rnp_vlan_strip_enable - helper to enable hw vlan stripping
 * @adapter: driver data
 */
static void rnp_vlan_strip_enable(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	struct rnp_ring *tx_ring;
	int i, j;

	for (i = 0; i < adapter->num_rx_queues; i++) {
		tx_ring = adapter->rx_ring[i];

		hw->ops.set_vlan_strip(hw, tx_ring->rnp_queue_idx, true);
		// hw_queue_strip_rx_vlan(hw, tx_ring->rnp_queue_idx, true);
	}
}

static void rnp_restore_vlan(struct rnp_adapter *adapter)
{
	u16 vid;
	struct rnp_hw *hw = &adapter->hw;

#ifdef HAVE_INT_NDO_VLAN_RX_ADD_VID
#ifdef NETIF_F_HW_VLAN_CTAG_TX
	rnp_vlan_rx_add_vid(adapter->netdev, htons(ETH_P_8021Q), 0);
#else  /* !NETIF_F_HW_VLAN_CTAG_TX */
	rnp_vlan_rx_add_vid(adapter->netdev, 0);
#endif /* NETIF_F_HW_VLAN_CTAG_TX */
#else  /* !HAVE_INT_NDO_VLAN_RX_ADD_VID */
	rnp_vlan_rx_add_vid(adapter->netdev, 0);
#endif /* HAVE_INT_NDO_VLAN_RX_ADD_VID */

#ifndef HAVE_VLAN_RX_REGISTER
	for_each_set_bit(vid, adapter->active_vlans, VLAN_N_VID)
	{

#ifdef HAVE_INT_NDO_VLAN_RX_ADD_VID
#ifdef NETIF_F_HW_VLAN_CTAG_TX
		rnp_vlan_rx_add_vid(adapter->netdev, htons(ETH_P_8021Q), vid);
#else  /* !NETIF_F_HW_VLAN_CTAG_TX */
		rnp_vlan_rx_add_vid(adapter->netdev, vid);
#endif /* NETIF_F_HW_VLAN_CTAG_TX */
#else  /* !HAVE_INT_NDO_VLAN_RX_ADD_VID */
		rnp_vlan_rx_add_vid(adapter->netdev, vid);
#endif /* HAVE_INT_NDO_VLAN_RX_ADD_VID */
	}
#endif /* HAVE_VLAN_RX_REGISTER */
	   /* config vlan mode for mac */
	   // wr32(hw, RNP_MAC_TX_VLAN_MODE, 0x00100000);
}

/**
 * rnp_write_uc_addr_list - write unicast addresses to RAR table
 * @netdev: network interface device structure
 *
 * Writes unicast address list to the RAR table.
 * Returns: -ENOMEM on failure/insufficient address space
 *                0 on no addresses written
 *                X on writing X addresses to the RAR table
 **/
// static int rnp_write_uc_addr_list(struct net_device *netdev)
//{
//	struct rnp_adapter *adapter = netdev_priv(netdev);
//	struct rnp_hw *hw = &adapter->hw;
//	unsigned int rar_entries = hw->mac.num_rar_entries - 1;
//	int count = 0;
//
//	/* In SR-IOV mode significantly less RAR entries are available */
//	if (adapter->flags & RNP_FLAG_SRIOV_ENABLED)
//		rar_entries = RNP_MAX_PF_MACVLANS - 1;
//
//	/* return ENOMEM indicating insufficient memory for addresses */
//	if (netdev_uc_count(netdev) > rar_entries)
//		return -ENOMEM;
//
//	if (!netdev_uc_empty(netdev)) {
//		struct netdev_hw_addr *ha;
//
//		hw_dbg(hw, "%s: rar_entries:%d, uc_count:%d\n", __func__,
//		       hw->mac.num_rar_entries, netdev_uc_count(netdev));
//
//		/* return error if we do not support writing to RAR table */
//		if (!hw->mac.ops.set_rar)
//			return -ENOMEM;
//
//		netdev_for_each_uc_addr(ha, netdev) {
//			if (!rar_entries)
//				break;
//			/* VMDQ_P(0) is num_vfs pf use the last
//			 * vf in sriov mode
//			 */
//			/* that's ok */
//			hw->mac.ops.set_rar(hw, rar_entries--, ha->addr,
//					    VMDQ_P(0), RNP_RAH_AV);
//
//			count++;
//		}
//	}
//	/* write the addresses in reverse order to avoid write combining */
//
//	hw_dbg(hw, "%s: Clearing RAR[1 - %d]\n", __func__, rar_entries);
//	for (; rar_entries > 0; rar_entries--)
//		hw->mac.ops.clear_rar(hw, rar_entries);
//
//	return count;
// }

/**
 * rnp_set_rx_mode - Unicast, Multicast and Promiscuous mode set
 * @netdev: network interface device structure
 *
 * The set_rx_method entry point is called whenever the unicast/multicast
 * address list or the network interface flags are updated.  This routine is
 * responsible for configuring the hardware for proper unicast, multicast and
 * promiscuous mode.
 **/
void rnp_set_rx_mode(struct net_device *netdev)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw *hw = &adapter->hw;
	// u32 fctrl, mcstctrl;
	netdev_features_t features;
	// int count;
	bool sriov_flag = !!(adapter->flags & RNP_FLAG_SRIOV_ENABLED);

	hw->ops.set_rx_mode(hw, netdev, sriov_flag);

	if (sriov_flag) {
		// RNP_FLAG_VFIFO_USED
		if (!test_and_set_bit(__RNP_USE_VFINFI, &adapter->state)) {
			rnp_restore_vf_macvlans(adapter);
			clear_bit(__RNP_USE_VFINFI, &adapter->state);
		}
	}

	features = netdev->features;

#ifdef NETIF_F_HW_VLAN_CTAG_RX
	if (features & NETIF_F_HW_VLAN_CTAG_RX)
		rnp_vlan_strip_enable(adapter);
	else
		rnp_vlan_strip_disable(adapter);
#endif
}

static void rnp_napi_enable_all(struct rnp_adapter *adapter)
{
	int q_idx;

	for (q_idx = 0; q_idx < adapter->num_q_vectors; q_idx++)
		napi_enable(&adapter->q_vector[q_idx]->napi);
}

static void rnp_napi_disable_all(struct rnp_adapter *adapter)
{
	int q_idx;

	for (q_idx = 0; q_idx < adapter->num_q_vectors; q_idx++)
		napi_disable(&adapter->q_vector[q_idx]->napi);
}

#ifdef CONFIG_RNP_DCB
/**
 * rnp_configure_dcb - Configure DCB hardware
 * @adapter: rnp adapter struct
 *
 * This is called by the driver on open to configure the DCB hardware.
 * This is also called by the gennetlink interface when reconfiguring
 * the DCB state.
 */
static void rnp_configure_dcb(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	int max_frame = adapter->netdev->mtu + ETH_HLEN + ETH_FCS_LEN;

#if 0
	if (!(adapter->flags & RNP_FLAG_DCB_ENABLED)) {
		if (hw->mac.type == rnp_mac_82598EB)
			netif_set_gso_max_size(adapter->netdev, 65536);
		return;
	}

	if (hw->mac.type == rnp_mac_82598EB)
		netif_set_gso_max_size(adapter->netdev, 32768);

	/* reconfigure the hardware */
	if (adapter->dcbx_cap & DCB_CAP_DCBX_VER_CEE) {
		rnp_dcb_calculate_tc_credits(hw, &adapter->dcb_cfg, max_frame,
						DCB_TX_CONFIG);
		rnp_dcb_calculate_tc_credits(hw, &adapter->dcb_cfg, max_frame,
						DCB_RX_CONFIG);
		rnp_dcb_hw_config(hw, &adapter->dcb_cfg);
	} else if (adapter->rnp_ieee_ets && adapter->rnp_ieee_pfc) {
		rnp_dcb_hw_ets(&adapter->hw,
				 adapter->rnp_ieee_ets,
				 max_frame);
		rnp_dcb_hw_pfc_config(&adapter->hw,
					adapter->rnp_ieee_pfc->pfc_en,
					adapter->rnp_ieee_ets->prio_tc);
	}

	/* Enable RSS Hash per TC */
	if (hw->mac.type != rnp_mac_82598EB) {
		u32 msb = 0;
		u16 rss_i = adapter->ring_feature[RING_F_RSS].indices - 1;

		while (rss_i) {
			msb++;
			rss_i >>= 1;
		}

		/* write msb to all 8 TCs in one write */
		RNP_WRITE_REG(hw, RNP_RQTC, msb * 0x11111111);
	}
#endif
}
#endif

/* Additional bittime to account for RNP framing */
#define RNP_ETH_FRAMING 20

/**
 * rnp_hpbthresh - calculate high water mark for flow control
 *
 * @adapter: board private structure to calculate for
 * @pb: packet buffer to calculate
 */
static int rnp_hpbthresh(struct rnp_adapter *adapter, int pb)
{
	struct rnp_hw *hw = &adapter->hw;
	struct net_device *dev = adapter->netdev;
	int link, tc, kb, marker = 0;
	u32 dv_id, rx_pba;
#if 0

	/* Calculate max LAN frame size */
	tc = link = dev->mtu + ETH_HLEN + ETH_FCS_LEN + RNP_ETH_FRAMING;

	/* Calculate delay value for device */
	switch (hw->mac.type) {
	case rnp_mac_X540:
		dv_id = RNP_DV_X540(link, tc);
		break;
	default:
		dv_id = RNP_DV(link, tc);
		break;
	}

	/* Loopback switch introduces additional latency */
	if (adapter->flags & RNP_FLAG_SRIOV_ENABLED)
		dv_id += RNP_B2BT(tc);

	/* Delay value is calculated in bit times convert to KB */
	kb = RNP_BT2KB(dv_id);
	rx_pba = RNP_READ_REG(hw, RNP_RXPBSIZE(pb)) >> 10;

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
 * rnp_lpbthresh - calculate low water mark for for flow control
 *
 * @adapter: board private structure to calculate for
 * @pb: packet buffer to calculate
 */
static int rnp_lpbthresh(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	struct net_device *dev = adapter->netdev;
	int tc;
	u32 dv_id;
#if 0
	/* Calculate max LAN frame size */
	tc = dev->mtu + ETH_HLEN + ETH_FCS_LEN;

	/* Calculate delay value for device */
	switch (hw->mac.type) {
	case rnp_mac_X540:
		dv_id = RNP_LOW_DV_X540(tc);
		break;
	default:
		dv_id = RNP_LOW_DV(tc);
		break;
	}

	/* Delay value is calculated in bit times convert to KB */
	return RNP_BT2KB(dv_id);
#else
	return 0;
#endif
}

/*
 * rnp_pbthresh_setup - calculate and setup high low water marks
 */
static void rnp_pbthresh_setup(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	int num_tc = netdev_get_num_tc(adapter->netdev);
	int i;

#if 0
	if (!num_tc)
		num_tc = 1;

	hw->fc.low_water = rnp_lpbthresh(adapter);

	for (i = 0; i < num_tc; i++) {
		hw->fc.high_water[i] = rnp_hpbthresh(adapter, i);

		/* Low water marks must not be larger than high water marks */
		if (hw->fc.low_water > hw->fc.high_water[i])
			hw->fc.low_water = 0;
	}
#endif
}

static void rnp_configure_pb(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	int hdrm;
	u8 tc = netdev_get_num_tc(adapter->netdev);

#if 0
	if (adapter->flags & RNP_FLAG_FDIR_HASH_CAPABLE ||
	    adapter->flags & RNP_FLAG_FDIR_PERFECT_CAPABLE)
		hdrm = 32 << adapter->fdir_pballoc;
	else
		hdrm = 0;

	hw->mac.ops.set_rxpba(hw, tc, hdrm, PBA_STRATEGY_EQUAL);
	rnp_pbthresh_setup(adapter);
#endif
}

static void rnp_fdir_filter_restore(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	struct hlist_node *node2;
	struct rnp_fdir_filter *filter;

	spin_lock(&adapter->fdir_perfect_lock);

	// spin_unlock(&adapter->fdir_perfect_lock);
	/* enable tcam if set tcam mode */
	if (adapter->fdir_mode == fdir_mode_tcam) {
		wr32(hw, RNP_ETH_TCAM_EN, 1);
		wr32(hw, RNP_TOP_ETH_TCAM_CONFIG_ENABLE, 1);
		wr32(hw, RNP_TCAM_CACHE_ENABLE, 0);
	}

	/* setup ntuple */
	hlist_for_each_entry_safe(
		filter, node2, &adapter->fdir_filter_list, fdir_node)
	{
		rnp_fdir_write_perfect_filter(
			adapter->fdir_mode,
			hw,
			&filter->filter,
			filter->hw_idx,
			(filter->action == RNP_FDIR_DROP_QUEUE)
				? RNP_FDIR_DROP_QUEUE
				: adapter->rx_ring[filter->action]->rnp_queue_idx);
	}

	spin_unlock(&adapter->fdir_perfect_lock);
}

// static void rnp_configure_vlan(struct net_device *netdev,
//			       struct rnp_adapter *adapter)
//{
//	int i;
//	struct rnp_hw *hw = &adapter->hw;
//
//	/* open vlan filter */
//	if (adapter->netdev->features & (NETIF_F_HW_VLAN_CTAG_FILTER))
//		rnp_vlan_filter_on(hw);
//	else
//		rnp_vlan_filter_off(hw);
// }

static void rnp_configure_pause(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;

	hw->ops.set_pause_mode(hw);
	// hw->mac.ops.fc_enable(hw);
}

void rnp_vlan_stags_flag(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;

	/* stags is added */
	if (adapter->flags2 & RNP_FLAG2_VLAN_STAGS_ENABLED) {
		hw->ops.set_txvlan_mode(hw, false);
		// wr32(hw, RNP_MAC_TX_VLAN_TAG, 0xc600000);
		// wr32(hw, RNP_MAC_TX_VLAN_MODE, 0x180000);
		// wr32(hw, RNP_MAC_INNER_VLAN_INCL, 0x100000);
	} else {
		hw->ops.set_txvlan_mode(hw, true);
		// wr32(hw, RNP_MAC_TX_VLAN_TAG, 0x4000000);
		// wr32(hw, RNP_MAC_TX_VLAN_MODE, 0x100000);
		// wr32(hw, RNP_MAC_INNER_VLAN_INCL, 0x100000);
	}
}

void rnp_vxlan_setup(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	//#define VXLAN_PORT_NUM	(4789)
	//#define VXLAN_HW_ENABLE (1)

	// wr32(hw, RNP_ETH_VXLAN_PORT, VXLAN_PORT_NUM);
	// wr32(hw, RNP_ETH_TUNNEL_MOD, VXLAN_HW_ENABLE);
}

static void rnp_configure(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	bool sriov_flag = !!(adapter->flags & RNP_FLAG_SRIOV_ENABLED);

	rnp_configure_pb(adapter); // setup high low water

#ifdef CONFIG_RNP_DCB
	rnp_configure_dcb(adapter);
#endif

	/*
	 * We must restore virtualization before VLANs or else
	 * the VLVF registers will not be populated
	 */
	rnp_configure_virtualization(adapter);
	/* init setup pause */
	rnp_configure_pause(adapter);

	// Unicast, Multicast and Promiscuous mode set
	rnp_set_rx_mode(adapter->netdev);
	/* reconfigure hw */
	// hw->mac.ops.set_rar(hw, 0, hw->mac.addr, VMDQ_P(0), RNP_RAH_AV);
	hw->ops.set_mac(hw, hw->mac.addr, sriov_flag);

	/* in sriov mode vlan is not reset */
	rnp_restore_vlan(adapter);

	hw->ops.update_hw_info(hw);

	/* setup rss key and table */
	/* enable all eth filter */
	// wr32(hw, RNP_HOST_FILTER_EN, 1);
	/* open redir */
	// wr32(hw, RNP_REDIR_EN, 1);

	/* open sctp check en */
	/*if (hw->feature_flags & RNP_NET_FEATURE_TX_CHECKSUM)
		wr32(hw, RNP_ETH_SCTP_CHECKSUM_EN, 1);

	if (hw->dma_version >= 0x20210108) {
		// mark Multicast as broadcast
		wr32(hw, RNP_VEB_MAC_MASK_LO, 0xffffffff);
		wr32(hw, RNP_VEB_MAC_MASK_HI, 0xfeff);
	} */

	/* setup rx slic 1536 bytes */
	/* only in page 4k mode ? */
	// rnp_setup_dma_rx(adapter, 96);
	/* test this with stags */
	/* stags is stored in adapter->stags_vid */
	/*
	 * adapter->flags2 |= RNP_FLAG2_VLAN_STAGS_ENABLED;
	 * adapter->stags_vid = htons(0x04);
	 */

	rnp_vlan_stags_flag(adapter);

	// rnp_vxlan_setup(adapter);

	rnp_init_rss_key(adapter);
	rnp_init_rss_table(adapter);

	if (adapter->flags & RNP_FLAG_FDIR_HASH_CAPABLE) {
		// rnp_init_fdir_signature_n10(&adapter->hw,
		// adapter->fdir_pballoc);
	} else if (adapter->flags & RNP_FLAG_FDIR_PERFECT_CAPABLE)
		rnp_fdir_filter_restore(adapter);

	/* setup vxlan match mode */
	if (adapter->priv_flags & RNP_PRIV_FLAG_VXLAN_INNER_MATCH)
		hw->ops.set_vxlan_mode(hw, true);
	else
		hw->ops.set_vxlan_mode(hw, false);
	// wr32(hw, RNP_ETH_WRAP_FIELD_TYPE, 1);
	// else
	// wr32(hw, RNP_ETH_WRAP_FIELD_TYPE, 0);

	rnp_configure_tx(adapter);
	rnp_configure_rx(adapter);
}

static inline bool rnp_is_sfp(struct rnp_hw *hw)
{
	return true;
}

/**
 * rnp_sfp_link_config - set up SFP+ link
 * @adapter: pointer to private adapter struct
 **/
static void rnp_sfp_link_config(struct rnp_adapter *adapter)
{
	/*
	 * We are assuming the worst case scenario here, and that
	 * is that an SFP was inserted/removed after the reset
	 * but before SFP detection was enabled.  As such the best
	 * solution is to just start searching as soon as we start
	 */
	adapter->flags2 |= RNP_FLAG2_SFP_NEEDS_RESET;
}

/**
 * rnp_non_sfp_link_config - set up non-SFP+ link
 * @hw: pointer to private hardware struct
 *
 * Returns 0 on success, negative on failure
 **/
static int rnp_non_sfp_link_config(struct rnp_hw *hw)
{
	u32 speed;
	bool autoneg, link_up = false;
	u32 ret = RNP_ERR_LINK_SETUP;
#if 0
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

static void rnp_up_complete(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	int err;
	int i;
	u32 ctrl_ext;

	rnp_get_hw_control(adapter);

	rnp_configure_msix(adapter);

	/* enable the optics for n10 SFP+ fiber */
	if (hw->ops.enable_tx_laser)
		hw->ops.enable_tx_laser(hw);

	clear_bit(__RNP_DOWN, &adapter->state);
	rnp_napi_enable_all(adapter);

	if (rnp_is_sfp(hw)) {
		rnp_sfp_link_config(adapter);
	} else {
		err = rnp_non_sfp_link_config(hw);
		if (err)
			e_err(probe, "link_config FAILED %d\n", err);
	}
	/*clear any pending interrupts*/
	rnp_irq_enable(adapter);

	/* enable transmits */
	netif_tx_start_all_queues(adapter->netdev);

	/* enable rx transmit */
	for (i = 0; i < adapter->num_rx_queues; i++)
		/* setup rx scater */
		ring_wr32(adapter->rx_ring[i], RNP_DMA_RX_START, 1);

	/* bring the link up in the watchdog, this could race with our first
	 * link up interrupt but shouldn't be a problems
	 */
	adapter->flags |= RNP_FLAG_NEED_LINK_UPDATE;
	adapter->link_check_timeout = jiffies;
	mod_timer(&adapter->service_timer, jiffies);

	// should clean link status?
	/* Set PF Reset Done bit so PF/VF Mail Ops can work */
	/* maybe differ in n500 */
	// rnp_mbx_link_event_enable(&adapter->hw, 1);
	//hw->link = 0;
	hw->ops.set_mbx_link_event(hw, 1);
	hw->ops.set_mbx_ifup(hw, 1);
	hw->ops.set_mac_rx(hw, true);
}

void rnp_reinit_locked(struct rnp_adapter *adapter)
{
	WARN_ON(in_interrupt());
	/* put off any impending NetWatchDogTimeout */
	// adapter->netdev->trans_start = jiffies;

	while (test_and_set_bit(__RNP_RESETTING, &adapter->state))
		usleep_range(1000, 2000);
	rnp_down(adapter);
	/*
	 * If SR-IOV enabled then wait a bit before bringing the adapter
	 * back up to give the VFs time to respond to the reset.  The
	 * two second wait is based upon the watchdog timer cycle in
	 * the VF driver.
	 */
	if (adapter->flags & RNP_FLAG_SRIOV_ENABLED)
		msleep(2000);
	rnp_up(adapter);

	clear_bit(__RNP_RESETTING, &adapter->state);
}

void rnp_up(struct rnp_adapter *adapter)
{
	/* hardware has been reset, we need to reload some things */
	rnp_configure(adapter);

	rnp_up_complete(adapter);
}

void rnp_reset(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	int err;
	bool sriov_flag = !!(adapter->flags & RNP_FLAG_SRIOV_ENABLED);

	rnp_logd(LOG_ADPT_STAT, "%s\n", __func__);

	/* lock SFP init bit to prevent race conditions with the watchdog */
	while (test_and_set_bit(__RNP_IN_SFP_INIT, &adapter->state))
		usleep_range(1000, 2000);

	/* clear all SFP and link config related flags while holding SFP_INIT */
	adapter->flags2 &= ~(RNP_FLAG2_SEARCH_FOR_SFP | RNP_FLAG2_SFP_NEEDS_RESET);
	adapter->flags &= ~RNP_FLAG_NEED_LINK_CONFIG;

	// err = hw->mac.ops.init_hw(hw);
	err = hw->ops.init_hw(hw);
	if (err) {
		e_dev_err("init_hw: Hardware Error: err:%d. line:%d\n", err, __LINE__);
	}

	clear_bit(__RNP_IN_SFP_INIT, &adapter->state);

	/* reprogram the RAR[0] in case user changed it. */
	hw->ops.set_mac(hw, hw->mac.addr, sriov_flag);
	// hw->mac.ops.set_rar(hw, 0, hw->mac.addr, VMDQ_P(0), RNP_RAH_AV);

	if (module_enable_ptp) {
		if (adapter->flags2 & RNP_FLAG2_PTP_ENABLED &&
			(adapter->ptp_rx_en || adapter->ptp_tx_en))
			rnp_ptp_reset(adapter);
	}
}

/**
 * rnp_clean_rx_ring - Free Rx Buffers per Queue
 * @rx_ring: ring to free buffers from
 **/
static void rnp_clean_rx_ring(struct rnp_ring *rx_ring)
{
	u16 i = rx_ring->next_to_clean;
	struct rnp_rx_buffer *rx_buffer = &rx_ring->rx_buffer_info[i];
#if defined(HAVE_STRUCT_DMA_ATTRS) && defined(HAVE_SWIOTLB_SKIP_CPU_SYNC)
	DEFINE_DMA_ATTRS(attrs);
	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
	dma_set_attr(DMA_ATTR_WEAK_ORDERING, &attrs);
#endif

	/*
	#ifdef HAVE_AF_XDP_ZC_SUPPORT
		if (rx_ring->xsk_umem) {
			rnp_xsk_clean_rx_ring(rx_ring);
			goto skip_free;
		}

	#endif
	*/
	/* Free all the Rx ring sk_buffs */
#ifdef CONFIG_RNP_DISABLE_PACKET_SPLIT
	while (i != rx_ring->next_to_use) {
#else
	while (i != rx_ring->next_to_alloc) {
#endif
		if (rx_buffer->skb) {
			struct sk_buff *skb = rx_buffer->skb;

			//printk("from no null skb\n");
#ifndef CONFIG_RNP_DISABLE_PACKET_SPLIT
			/* no need this */
			if (RNP_CB(skb)->page_released)
				dma_unmap_page_attrs(rx_ring->dev,
									 RNP_CB(skb)->dma,
									 rnp_rx_pg_size(rx_ring),
									 DMA_FROM_DEVICE,
#if defined(HAVE_STRUCT_DMA_ATTRS) && defined(HAVE_SWIOTLB_SKIP_CPU_SYNC)
									 &attrs);
#else
									 RNP_RX_DMA_ATTR);
#endif
#else
			/* We need to clean up RSC frag lists */
			skb = rnp_merge_active_tail(skb);
			if (rnp_close_active_frag_list(skb))
				dma_unmap_single(rx_ring->dev,
								 RNP_CB(skb)->dma,
								 rx_ring->rx_buf_len,
								 DMA_FROM_DEVICE);
			RNP_CB(skb)->dma = 0;
#endif /* CONFIG_RNP_DISABLE_PACKET_SPLIT */
			dev_kfree_skb(skb);
			rx_buffer->skb = NULL;
		}

#ifndef CONFIG_RNP_DISABLE_PACKET_SPLIT
		/* Invalidate cache lines that may have been written to by
		 * device so that we avoid corrupting memory.
		 */
		dma_sync_single_range_for_cpu(rx_ring->dev,
									  rx_buffer->dma,
									  rx_buffer->page_offset,
									  rnp_rx_bufsz(rx_ring),
									  DMA_FROM_DEVICE);

		/* free resources associated with mapping */
		dma_unmap_page_attrs(rx_ring->dev,
							 rx_buffer->dma,
							 rnp_rx_pg_size(rx_ring),
							 DMA_FROM_DEVICE,
#if defined(HAVE_STRUCT_DMA_ATTRS) && defined(HAVE_SWIOTLB_SKIP_CPU_SYNC)
							 &attrs);
#else
							 RNP_RX_DMA_ATTR);
#endif

		__page_frag_cache_drain(rx_buffer->page, rx_buffer->pagecnt_bias);
#else  /* CONFIG_RNP_DISABLE_PACKET_SPLIT */
		if (rx_buffer->dma) {
			dma_unmap_single(rx_ring->dev,
							 rx_buffer->dma,
							 rx_ring->rx_buf_len,
							 DMA_FROM_DEVICE);
			rx_buffer->dma = 0;
		}
#endif /* CONFIG_RNP_DISABLE_PACKET_SPLIT */
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
skip_free:
#endif
#ifndef CONFIG_RNP_DISABLE_PACKET_SPLIT
	rx_ring->next_to_alloc = 0;
	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;
#endif
}

/**
 * rnp_clean_tx_ring - Free Tx Buffers
 * @tx_ring: ring to be cleaned
 **/
static void rnp_clean_tx_ring(struct rnp_ring *tx_ring)
{
	unsigned long size;
	u16 i = tx_ring->next_to_clean;
	struct rnp_tx_buffer *tx_buffer = &tx_ring->tx_buffer_info[i];

	BUG_ON(tx_ring == NULL);

	/* ring already cleared, nothing to do */
	if (!tx_ring->tx_buffer_info)
		return;

	while (i != tx_ring->next_to_use) {
		struct rnp_tx_desc *eop_desc, *tx_desc;

		dev_kfree_skb_any(tx_buffer->skb);
		/* unmap skb header data */
		dma_unmap_single(tx_ring->dev,
						 dma_unmap_addr(tx_buffer, dma),
						 dma_unmap_len(tx_buffer, len),
						 DMA_TO_DEVICE);

		eop_desc = tx_buffer->next_to_watch;
		tx_desc = RNP_TX_DESC(tx_ring, i);
		/* unmap remaining buffers */
		while (tx_desc != eop_desc) {
			tx_buffer++;
			tx_desc++;
			i++;
			if (unlikely(i == tx_ring->count)) {
				i = 0;
				tx_buffer = tx_ring->tx_buffer_info;
				tx_desc = RNP_TX_DESC(tx_ring, 0);
			}

			/* unmap any remaining paged data */
			if (dma_unmap_len(tx_buffer, len))
				dma_unmap_page(tx_ring->dev,
							   dma_unmap_addr(tx_buffer, dma),
							   dma_unmap_len(tx_buffer, len),
							   DMA_TO_DEVICE);
		}
		/* move us one more past the eop_desc for start of next pkt */
		tx_buffer++;
		i++;
		if (unlikely(i == tx_ring->count)) {
			i = 0;
			tx_buffer = tx_ring->tx_buffer_info;
		}
	}
	/* Free all the Tx ring sk_buffs */
	/*
	for (i = 0; i < tx_ring->count; i++) {
		tx_buffer_info = &tx_ring->tx_buffer_info[i];
		rnp_unmap_and_free_tx_resource(tx_ring, tx_buffer_info);
	}
	*/

	netdev_tx_reset_queue(txring_txq(tx_ring));

	size = sizeof(struct rnp_tx_buffer) * tx_ring->count;
	memset(tx_ring->tx_buffer_info, 0, size);

	/* Zero out the descriptor ring */
	memset(tx_ring->desc, 0, tx_ring->size);

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;
}

/**
 * rnp_clean_all_rx_rings - Free Rx Buffers for all queues
 * @adapter: board private structure
 **/
static void rnp_clean_all_rx_rings(struct rnp_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++)
		rnp_clean_rx_ring(adapter->rx_ring[i]);
}

/**
 * rnp_clean_all_tx_rings - Free Tx Buffers for all queues
 * @adapter: board private structure
 **/
static void rnp_clean_all_tx_rings(struct rnp_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++)
		rnp_clean_tx_ring(adapter->tx_ring[i]);
}

static void rnp_fdir_filter_exit(struct rnp_adapter *adapter)
{
	struct hlist_node *node2;
	struct rnp_fdir_filter *filter;
	struct rnp_hw *hw = &adapter->hw;

	spin_lock(&adapter->fdir_perfect_lock);

	hlist_for_each_entry_safe(
		filter, node2, &adapter->fdir_filter_list, fdir_node)
	{
		/* call earase to hw */
		rnp_fdir_erase_perfect_filter(
			adapter->fdir_mode, hw, &filter->filter, filter->hw_idx);

		hlist_del(&filter->fdir_node);
		kfree(filter);
	}
	adapter->fdir_filter_count = 0;
	if (hw->feature_flags & RNP_NET_FEATURE_TCAM) {
		if (hw->fdir_mode == fdir_mode_tcam) { 
			adapter->layer2_count = RNP_MAX_LAYER2_FILTERS - 1;
			adapter->tuple_5_count = RNP_MAX_TCAM_FILTERS - 1;
		} else {
			adapter->layer2_count = RNP_MAX_LAYER2_FILTERS - 1;
			adapter->tuple_5_count = RNP_MAX_TUPLE5_FILTERS - 1;
		}
	} else {
		adapter->layer2_count = RNP_MAX_LAYER2_FILTERS - 1;
		adapter->tuple_5_count = RNP_MAX_TUPLE5_FILTERS - 1;
	}
	

	spin_unlock(&adapter->fdir_perfect_lock);
}

int rnp_xmit_nop_frame_ring(struct rnp_adapter *adapter,
							struct rnp_ring *tx_ring)
{
	u16 i = tx_ring->next_to_use;
	struct rnp_tx_desc *tx_desc;

	tx_desc = RNP_TX_DESC(tx_ring, i);

	/* set length to 0 */
	tx_desc->blen_mac_ip_len = 0;
	tx_desc->vlan_cmd = cpu_to_le32(RNP_TXD_CMD_EOP | RNP_TXD_CMD_RS);
	/* update tail */
	rnp_wr_reg(tx_ring->tail, 0);
	return 0;
}

void rnp_down(struct rnp_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	struct rnp_hw *hw = &adapter->hw;
	u32 rxctrl;
	int i;


	/* signal that we are down to the interrupt handler */
	if (test_and_set_bit(__RNP_DOWN, &adapter->state))
		return;

	hw->ops.set_mac_rx(hw, false);

	hw->ops.set_mbx_link_event(hw, 0);
	hw->ops.set_mbx_ifup(hw, 0);

	// should clean link status if need
	if (hw->ops.clean_link)
		hw->ops.clean_link(hw);


	netif_tx_stop_all_queues(netdev);

	netif_carrier_off(netdev);

	rnp_irq_disable(adapter);

	// should wait irq stop
	usleep_range(10000, 20000);
	// should wait service stop
	// cause crash if link down to call rnp_reinit_locked
	/*while (test_bit(__RNP_SERVICE_SCHED, &adapter->state)) {
		usleep_range(10000, 20000);
	}*/

	netif_tx_disable(netdev);

	/* disable all enabled rx queues */
	for (i = 0; i < adapter->num_rx_queues; i++) {
		rnp_disable_rx_queue(adapter, adapter->rx_ring[i]);
		/* only handle when srio enable and change rx length setup */
		if ((adapter->flags & RNP_FLAG_SRIOV_ENABLED) &&
			(adapter->rx_ring[i]->ring_flags & RNP_RING_FLAG_CHANGE_RX_LEN)) {
			int head;
			struct rnp_ring *ring = adapter->rx_ring[i];

			head = ring_rd32(ring, RNP_DMA_REG_RX_DESC_BUF_HEAD);
			adapter->rx_ring[i]->ring_flags &= (~RNP_RING_FLAG_CHANGE_RX_LEN);
			/* we should delay setup rx length to
			 * wait rx head to 0
			 */
			if (head >= adapter->rx_ring[i]->reset_count) {
				adapter->rx_ring[i]->ring_flags |=
					RNP_RING_FLAG_DELAY_SETUP_RX_LEN;
				/* set sw count to head + 1*/
				adapter->rx_ring[i]->temp_count = head + 1;
			}
		}
		/* only down without rx_len change no need handle */
	}
	/* call carrier off first to avoid false dev_watchdog timeouts */

	rnp_napi_disable_all(adapter);

	adapter->flags2 &=
		~(RNP_FLAG2_FDIR_REQUIRES_REINIT | RNP_FLAG2_RESET_REQUESTED);
	adapter->flags &= ~RNP_FLAG_NEED_LINK_UPDATE;

	// del_timer_sync(&adapter->service_timer);
	//  maybe bug if call real_tx_hang handler
	// cancel_work_sync(&adapter->service_task);

	if (adapter->num_vfs) {
		/* Mark all the VFs as inactive */
		/* how to deal this is not sure */
		/*
		 * for (i = 0 ; i < adapter->num_vfs; i++)
		 * adapter->vfinfo[i].clear_to_send = false;
		 */
		/* ping all the active vfs to let them know we are going down */
		rnp_ping_all_vfs(adapter);

		/* Disable all VFTE/VFRE TX/RX */
		rnp_disable_tx_rx(adapter);
	}

	/* disable transmits in the hardware now that interrupts are off */
	for (i = 0; i < adapter->num_tx_queues; i++) {
		struct rnp_hw *hw = &adapter->hw;
		struct rnp_ring *tx_ring = adapter->tx_ring[i];
		int count = tx_ring->count;
		int head;
		int timeout = 0;
		/* 1. stop queue */
		if (!(tx_ring->ring_flags & RNP_RING_SKIP_TX_START))
			ring_wr32(tx_ring, RNP_DMA_TX_START, 0);
		// usleep_range(10000, 20000);
		/* 2. try to set tx head to 0 in sriov mode
		 * since we don't reset
		 */
		if ((adapter->flags & RNP_FLAG_SRIOV_ENABLED) &&
			(!(tx_ring->ring_flags & RNP_RING_SIZE_CHANGE_FIX))) {
			/* only do this if hw not support tx head to zero auto */

			/* n10 should wait tx_ready */
			int timeout = 0;
			u32 status = 0;

			do {
				status = ring_rd32(tx_ring, RNP_DMA_TX_READY);
				usleep_range(100, 200);
				timeout++;
				rnp_dbg("wait %d tx ready to 1\n", tx_ring->rnp_queue_idx);
			} while ((status != 1) && (timeout < 100));

			if (timeout >= 100)
				printk("wait tx ready timeout\n");

			head = ring_rd32(tx_ring, RNP_DMA_REG_TX_DESC_BUF_HEAD);
			if (head != 0) {
				u16 next_to_use = tx_ring->next_to_use;

				if (head != (count - 1)) {
					/* 3 set len head + 1 */
					ring_wr32(tx_ring, RNP_DMA_REG_TX_DESC_BUF_LEN, head + 1);
					// tx_ring->count = head + 1;
				}
				/* set to use head */
				tx_ring->next_to_use = head;
				/* 4 send a len zero packet */
				rnp_xmit_nop_frame_ring(adapter, tx_ring);
				if (!(tx_ring->ring_flags & RNP_RING_SKIP_TX_START))
					ring_wr32(tx_ring, RNP_DMA_TX_START, 1);
				/* 5 wait head to zero */
				while ((head != 0) && (timeout < 1000)) {
					head = ring_rd32(tx_ring, RNP_DMA_REG_TX_DESC_BUF_HEAD);
					usleep_range(10000, 20000);
					timeout++;
				}
				if (timeout >= 1000) {
					printk("[%s] Wait Tx-ring %d head to zero time out\n",
						   netdev->name,
						   tx_ring->rnp_queue_idx);
				}
				/* 6 stop queue again*/
				if (!(tx_ring->ring_flags & RNP_RING_SKIP_TX_START))
					ring_wr32(tx_ring, RNP_DMA_TX_START, 0);
				/* 7 write back next_to_use maybe hw hang */
				tx_ring->next_to_use = next_to_use;
			}
		}
	}

	if (!pci_channel_offline(adapter->pdev)) {
		if (!(adapter->flags & RNP_FLAG_SRIOV_ENABLED))
			rnp_reset(adapter);
	}

	/* power down the optics for n10 SFP+ fiber */
	if (hw->ops.disable_tx_laser)
		hw->ops.disable_tx_laser(hw);

	rnp_clean_all_tx_rings(adapter);
	rnp_clean_all_rx_rings(adapter);

#ifdef CONFIG_RNP_DCA
	/* since we reset the hardware DCA settings were cleared */
	rnp_setup_dca(adapter);
#endif
}

/**
 * rnp_tx_timeout - Respond to a Tx Hang
 * @netdev: network interface device structure
 **/
#ifdef HAVE_TX_TIMEOUT_TXQUEUE
static void rnp_tx_timeout(struct net_device *netdev, unsigned int txqueue)
#else
static void rnp_tx_timeout(struct net_device *netdev)
#endif
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	/* Do the reset outside of interrupt context */
	int i;
	bool real_tx_hang = false;

#define TX_TIMEO_LIMIT 16000
	for (i = 0; i < adapter->num_tx_queues; i++) {
		struct rnp_ring *tx_ring = adapter->tx_ring[i];

		if (check_for_tx_hang(tx_ring) && rnp_check_tx_hang(tx_ring))
			real_tx_hang = true;
	}

	if (real_tx_hang) {
		printk("hw read hang!!!!");
		/* Do the reset outside of interrupt context */
		rnp_tx_timeout_reset(adapter);
	} else {
		tx_dbg("Fake Tx hang detected with timeout of %d "
			   "seconds\n",
			   netdev->watchdog_timeo / HZ);

		/* fake Tx hang - increase the kernel timeout */
		if (netdev->watchdog_timeo < TX_TIMEO_LIMIT)
			netdev->watchdog_timeo *= 2;
	}
}

/**
 * rnp_sw_init - Initialize general software structures (struct rnp_adapter)
 * @adapter: board private structure to initialize
 *
 * rnp_sw_init initializes the Adapter private data structure.
 * Fields are initialized based on PCI device information and
 * OS network device settings (MTU size).
 **/
static int rnp_sw_init(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	struct pci_dev *pdev = adapter->pdev;
	unsigned int rss = 0, fdir;
	u32 fwsm;
	int i;
	int rss_limit = num_online_cpus();
#ifdef RNP_MAX_RINGS
	rss_limit = RNP_MAX_RINGS;
#endif

#ifdef CONFIG_RNP_DCB
	int j;
	struct tc_configuration *tc;
#endif

	hw->vendor_id = pdev->vendor;
	hw->device_id = pdev->device;
	hw->subsystem_vendor_id = pdev->subsystem_vendor;
	hw->subsystem_device_id = pdev->subsystem_device;

	/* if this hw can setup msix count */
	if (pf_msix_counts_set) {
		if (hw->ops.update_msix_count)
			hw->ops.update_msix_count(hw, pf_msix_counts_set);
	}

	/* Set common capability flags and settings */
	// if (hw->rss_type == rnp_rss_uv3p) {
	//	/* Makefile use RNP_MAX_RINGS to limit ring number */
	//	rss = min_t(int, adapter->max_ring_pair_counts, rss_limit);
	// } else {
	rss = min_t(int, adapter->max_ring_pair_counts, rss_limit);
	//}
	// should limit queue since cpu maybe large than vectors number
	// fixme
	rss =
		min_t(int, rss, hw->mac.max_msix_vectors - adapter->num_other_vectors);
	adapter->ring_feature[RING_F_RSS].limit =
		min_t(int, rss, adapter->max_ring_pair_counts);

	adapter->flags |= RNP_FLAG_VXLAN_OFFLOAD_CAPABLE;
	adapter->flags |= RNP_FLAG_VXLAN_OFFLOAD_ENABLE;

	adapter->flags2 |= RNP_FLAG2_RSC_CAPABLE;
	adapter->flags2 |= RNP_FLAG2_RSC_ENABLED;
	// todo maybe error with uv440
	//  we leave one for other
	adapter->max_q_vectors = hw->max_msix_vectors - 1;
	adapter->atr_sample_rate = 20;

	fdir = min_t(int, adapter->max_q_vectors, rss_limit);
	adapter->ring_feature[RING_F_FDIR].limit = fdir;

	if (hw->feature_flags & RNP_NET_FEATURE_RX_NTUPLE_FILTER) {
		// testme
		spin_lock_init(&adapter->fdir_perfect_lock);
		adapter->fdir_filter_count = 0;
		adapter->fdir_mode = hw->fdir_mode;
		/* fdir_pballoc not from zero, so add 2 */
		adapter->fdir_pballoc = 2 + hw->layer2_count + hw->tuple5_count;
		adapter->layer2_count = hw->layer2_count;
		adapter->tuple_5_count = hw->tuple5_count;
		/*
		if (hw->feature_flags & RNP_NET_FEATURE_TCAM) {
			adapter->fdir_mode = fdir_mode_tcam;
			adapter->fdir_pballoc = RNP_MAX_LAYER2_FILTERS +
		RNP_MAX_TCAM_FILTERS; adapter->layer2_count = RNP_MAX_LAYER2_FILTERS -
		1; adapter->tuple_5_count = RNP_MAX_TCAM_FILTERS - 1; } else {
			adapter->fdir_mode = fdir_mode_tuple5;
			adapter->fdir_pballoc = RNP_MAX_LAYER2_FILTERS +
		RNP_MAX_TUPLE5_FILTERS; adapter->layer2_count = RNP_MAX_LAYER2_FILTERS -
		1; adapter->tuple_5_count = RNP_MAX_TUPLE5_FILTERS - 1;
		} */
		// adapter->flags |= RNP_FLAG_FDIR_PERFECT_CAPABLE;
		//#endif
	}
	// adapter->fdir_pballoc = RNP_FDIR_PBALLOC_64K;
#ifdef CONFIG_RNP_DCA
	// we can't support dca
	adapter->flags |= RNP_FLAG_DCA_CAPABLE;
#endif

	/* itr sw setup here */
	adapter->sample_interval = 10;
	adapter->adaptive_rx_coal = 1;
	adapter->adaptive_tx_coal = 1;
	adapter->auto_rx_coal = 0;
	adapter->napi_budge = 64;
	/* set default work limits */
	adapter->tx_work_limit = RNP_DEFAULT_TX_WORK;
	adapter->rx_usecs = RNP_PKT_TIMEOUT;
	adapter->rx_frames = RNP_RX_PKT_POLL_BUDGET;
	// ft 2500 use 200 can speed netperf
	// adapter->tx_usecs = 200;
	// adapter->tx_frames = 0x30;
	adapter->tx_usecs = RNP_PKT_TIMEOUT_TX;
	adapter->tx_frames = RNP_TX_PKT_POLL_BUDGET;

	/* n-tuple support exists, always init our spinlock */
	/* init fdir count */

#if 0
#ifdef CONFIG_RNP_DCB
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
	hw->fc.requested_mode = rnp_fc_full;
	hw->fc.current_mode = rnp_fc_full;	/* init for ethtool output */
	rnp_pbthresh_setup(adapter);
	hw->fc.pause_time = RNP_DEFAULT_FCPAUSE;
	hw->fc.send_xon = true;
	hw->fc.disable_fc_autoneg =
		(rnp_device_supports_autoneg_fc(hw) == 0) ? false : true;
#endif
	/* enable itr by default in dynamic mode */
	// adapter->rx_itr_setting = 1;
	// adapter->tx_itr_setting = 1;

#ifdef CONFIG_PCI_IOV
	adapter->num_vfs = (max_vfs > (hw->max_vfs - 1)) ? 0 : max_vfs;
#endif

	/* set default ring sizes */
	adapter->tx_ring_item_count = RNP_DEFAULT_TXD;
	adapter->rx_ring_item_count = RNP_DEFAULT_RXD;

	/* initialize eeprom parameters */
	if (rnp_init_eeprom_params_generic(hw)) {
		e_dev_err("EEPROM initialization failed\n");
		return -EIO;
	}
	/*initialization default pause flow */
	hw->fc.requested_mode = rnp_fc_full;
	hw->fc.pause_time = RNP_DEFAULT_FCPAUSE;
	hw->fc.current_mode = rnp_fc_full;
	for (i = 0; i < RNP_MAX_TRAFFIC_CLASS; i++) {
		hw->fc.high_water[i] = RNP_DEFAULT_HIGH_WATER;
		hw->fc.low_water[i] = RNP_DEFAULT_LOW_WATER;
	}

	set_bit(__RNP_DOWN, &adapter->state);

	return 0;
}

/**
 * rnp_setup_tx_resources - allocate Tx resources (Descriptors)
 * @tx_ring:    tx descriptor ring (for a specific queue) to setup
 *
 * Return 0 on success, negative on failure
 **/

int rnp_setup_tx_resources(struct rnp_ring *tx_ring,
						   struct rnp_adapter *adapter)
{
	struct device *dev = tx_ring->dev;
	int orig_node = dev_to_node(dev);
	int numa_node = NUMA_NO_NODE;
	int size;

	size = sizeof(struct rnp_tx_buffer) * tx_ring->count;

#ifdef USE_NUMA_MEMORY
	if (tx_ring->q_vector)
		numa_node = tx_ring->q_vector->numa_node;
	// printk("dev node %d\n", orig_node);
	// printk("alloc %d memory node %d\n", tx_ring->queue_index, numa_node);
	tx_ring->tx_buffer_info = vzalloc_node(size, numa_node);
	if (!tx_ring->tx_buffer_info)
		tx_ring->tx_buffer_info = vzalloc(size);
	if (!tx_ring->tx_buffer_info)
		goto err;
#else
	tx_ring->tx_buffer_info = kzalloc(size, GFP_KERNEL);
#endif
	/* round up to nearest 4K */
	tx_ring->size = tx_ring->count * sizeof(struct rnp_tx_desc);
	tx_ring->size = ALIGN(tx_ring->size, 4096);

#ifdef USE_NUMA_MEMORY
	set_dev_node(dev, numa_node);
#endif
	tx_ring->desc =
		dma_alloc_coherent(dev, tx_ring->size, &tx_ring->dma, GFP_KERNEL);
#ifdef USE_NUMA_MEMORY
	set_dev_node(dev, orig_node);
#endif
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
			"TxRing:%d, vector:%d ItemCounts:%d "
			"desc:%p(0x%llx) node:%d\n",
			tx_ring->rnp_queue_idx,
			tx_ring->q_vector->v_idx,
			tx_ring->count,
			tx_ring->desc,
			tx_ring->dma,
			numa_node);
	return 0;

err:

#ifdef USE_NUMA_MEMORY
	vfree(tx_ring->tx_buffer_info);
#else
	kfree(tx_ring->tx_buffer_info);
#endif
	tx_ring->tx_buffer_info = NULL;
	dev_err(dev, "Unable to allocate memory for the Tx descriptor ring\n");
	return -ENOMEM;
}

/**
 * rnp_setup_all_tx_resources - allocate all queues Tx resources
 * @adapter: board private structure
 *
 * If this function returns with an error, then it's possible one or
 * more of the rings is populated (while the rest are not).  It is the
 * callers duty to clean those orphaned rings.
 *
 * Return 0 on success, negative on failure
 **/
static int rnp_setup_all_tx_resources(struct rnp_adapter *adapter)
{
	int i, err = 0;

	tx_dbg("adapter->num_tx_queues:%d, adapter->tx_ring[0]:%p\n",
		   adapter->num_tx_queues,
		   adapter->tx_ring[0]);

	for (i = 0; i < (adapter->num_tx_queues); i++) {
		BUG_ON(adapter->tx_ring[i] == NULL);
		err = rnp_setup_tx_resources(adapter->tx_ring[i], adapter);
		if (!err)
			continue;

		e_err(probe, "Allocation for Tx Queue %u failed\n", i);
		goto err_setup_tx;
	}

	return 0;
err_setup_tx:
	/* rewind the index freeing the rings as we go */
	while (i--)
		rnp_free_tx_resources(adapter->tx_ring[i]);
	return err;
}

/**
 * rnp_setup_rx_resources - allocate Rx resources (Descriptors)
 * @rx_ring:    rx descriptor ring (for a specific queue) to setup
 *
 * Returns 0 on success, negative on failure
 **/
int rnp_setup_rx_resources(struct rnp_ring *rx_ring,
						   struct rnp_adapter *adapter)
{
	struct device *dev = rx_ring->dev;
	int orig_node = dev_to_node(dev);
	int numa_node = NUMA_NO_NODE;
	int size;

	BUG_ON(rx_ring == NULL);

	size = sizeof(struct rnp_rx_buffer) * rx_ring->count;

#ifdef USE_NUMA_MEMORY
	if (rx_ring->q_vector)
		numa_node = rx_ring->q_vector->numa_node;

	rx_ring->rx_buffer_info = vzalloc_node(size, numa_node);
	if (!rx_ring->rx_buffer_info)
		rx_ring->rx_buffer_info = vzalloc(size);
	if (!rx_ring->rx_buffer_info)
		goto err;
#else
	rx_ring->rx_buffer_info = kzalloc(size, GFP_KERNEL);
#endif
	/* Round up to nearest 4K */
	rx_ring->size = rx_ring->count * sizeof(union rnp_rx_desc);
	rx_ring->size = ALIGN(rx_ring->size, 4096);

#ifdef USE_NUMA_MEMORY
	set_dev_node(dev, numa_node);
#endif
	rx_ring->desc =
		dma_alloc_coherent(dev, rx_ring->size, &rx_ring->dma, GFP_KERNEL);
#ifdef USE_NUMA_MEMORY
	set_dev_node(dev, orig_node);
#endif
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
			"RxRing:%d, vector:%d ItemCounts:%d "
			"desc:%p(0x%llx) node:%d\n",
			rx_ring->rnp_queue_idx,
			rx_ring->q_vector->v_idx,
			rx_ring->count,
			rx_ring->desc,
			rx_ring->dma,
			numa_node);

	return 0;
err:

#ifdef USE_NUMA_MEMORY
	vfree(rx_ring->rx_buffer_info);
#else
	kfree(rx_ring->rx_buffer_info);
#endif
	rx_ring->rx_buffer_info = NULL;
	dev_err(dev, "Unable to allocate memory for the Rx descriptor ring\n");
	return -ENOMEM;
}

/**
 * rnp_setup_all_rx_resources - allocate all queues Rx resources
 * @adapter: board private structure
 *
 * If this function returns with an error, then it's possible one or
 * more of the rings is populated (while the rest are not).  It is the
 * callers duty to clean those orphaned rings.
 *
 * Return 0 on success, negative on failure
 **/
static int rnp_setup_all_rx_resources(struct rnp_adapter *adapter)
{
	int i, err = 0;
	struct rnp_hw *hw = &adapter->hw;
	u32 head;

	for (i = 0; i < adapter->num_rx_queues; i++) {
		BUG_ON(adapter->rx_ring[i] == NULL);

		/* should check count and head */
		/* in sriov condition may head large than count */
		head = ring_rd32(adapter->rx_ring[i], RNP_DMA_REG_RX_DESC_BUF_HEAD);
		if (unlikely(head >= adapter->rx_ring[i]->count)) {
			dbg("[%s] Ring %d head large than count",
				adapter->netdev->name,
				adapter->rx_ring[i]->rnp_queue_idx);
			adapter->rx_ring[i]->ring_flags |= RNP_RING_FLAG_DELAY_SETUP_RX_LEN;
			adapter->rx_ring[i]->reset_count = adapter->rx_ring[i]->count;
			adapter->rx_ring[i]->count = head + 1;
		}
		err = rnp_setup_rx_resources(adapter->rx_ring[i], adapter);
		if (!err)
			continue;

		e_err(probe, "Allocation for Rx Queue %u failed\n", i);
		goto err_setup_rx;
	}

	return 0;
err_setup_rx:
	/* rewind the index freeing the rings as we go */
	while (i--)
		rnp_free_rx_resources(adapter->rx_ring[i]);
	return err;
}

/**
 * rnp_free_tx_resources - Free Tx Resources per Queue
 * @tx_ring: Tx descriptor ring for a specific queue
 *
 * Free all transmit software resources
 **/
void rnp_free_tx_resources(struct rnp_ring *tx_ring)
{
	BUG_ON(tx_ring == NULL);

	rnp_clean_tx_ring(tx_ring);
#ifdef USE_NUMA_MEMORY
	vfree(tx_ring->tx_buffer_info);
#else
	kfree(tx_ring->tx_buffer_info);
#endif
	tx_ring->tx_buffer_info = NULL;

	/* if not set, then don't free */
	if (!tx_ring->desc)
		return;

	dma_free_coherent(tx_ring->dev, tx_ring->size, tx_ring->desc, tx_ring->dma);

	tx_ring->desc = NULL;
}

/**
 * rnp_free_all_tx_resources - Free Tx Resources for All Queues
 * @adapter: board private structure
 *
 * Free all transmit software resources
 **/
static void rnp_free_all_tx_resources(struct rnp_adapter *adapter)
{
	int i;

	for (i = 0; i < (adapter->num_tx_queues); i++)
		rnp_free_tx_resources(adapter->tx_ring[i]);
}

/**
 * rnp_free_rx_resources - Free Rx Resources
 * @rx_ring: ring to clean the resources from
 *
 * Free all receive software resources
 **/
void rnp_free_rx_resources(struct rnp_ring *rx_ring)
{
	BUG_ON(rx_ring == NULL);

	rnp_clean_rx_ring(rx_ring);

#ifdef USE_NUMA_MEMORY
	vfree(rx_ring->rx_buffer_info);
#else
	kfree(rx_ring->rx_buffer_info);
#endif
	rx_ring->rx_buffer_info = NULL;

	/* if not set, then don't free */
	if (!rx_ring->desc)
		return;

	dma_free_coherent(rx_ring->dev, rx_ring->size, rx_ring->desc, rx_ring->dma);

	rx_ring->desc = NULL;
}

/**
 * rnp_free_all_rx_resources - Free Rx Resources for All Queues
 * @adapter: board private structure
 *
 * Free all receive software resources
 **/
static void rnp_free_all_rx_resources(struct rnp_adapter *adapter)
{
	int i;

	for (i = 0; i < (adapter->num_rx_queues); i++)
		if (adapter->rx_ring[i]->desc)
			rnp_free_rx_resources(adapter->rx_ring[i]);
}

/**
 * rnp_change_mtu - Change the Maximum Transfer Unit
 * @netdev: network interface device structure
 * @new_mtu: new value for maximum frame size
 *
 * Returns 0 on success, negative on failure
 **/
static int rnp_change_mtu(struct net_device *netdev, int new_mtu)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw *hw = &adapter->hw;
	int max_frame = new_mtu + ETH_HLEN + ETH_FCS_LEN * 2;

	/* MTU < 68 is an error and causes problems on some kernels */
	// todo set define to adapter
	if ((new_mtu < hw->min_length) || (max_frame > hw->max_length))
		return -EINVAL;

	e_info(probe, "changing MTU from %d to %d\n", netdev->mtu, new_mtu);

	if (netdev->mtu == new_mtu)
		return 0;

	/* must set new MTU before calling down or up */
	netdev->mtu = new_mtu;

	if (netif_running(netdev))
		rnp_reinit_locked(adapter);

	rnp_msg_post_status(adapter, PF_SET_MTU);

	// hw->set_mtu(hw, new_mtu);
	return 0;
}

/**
 * rnp_tx_maxrate - callback to set the maximum per-queue bitrate
 * @netdev: network interface device structure
 * @queue_index: Tx queue to set
 * @maxrate: desired maximum transmit bitrate Mbps
 **/
static int
rnp_tx_maxrate(struct net_device *netdev, int queue_index, u32 maxrate)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_ring *tx_ring = adapter->tx_ring[queue_index];
	u64 real_rate = 0;

	adapter->max_rate[queue_index] = maxrate;
	rnp_dbg("%s: queue:%d maxrate:%d\n", __func__, queue_index, maxrate);
	if (!maxrate)
		return rnp_setup_tx_maxrate(
			tx_ring, 0, adapter->hw.usecstocount * 1000000);
	/* we need turn it to bytes/s */
	real_rate = ((u64)maxrate * 1024 * 1024) / 8;
	rnp_setup_tx_maxrate(
		tx_ring, real_rate, adapter->hw.usecstocount * 1000000);

	return 0;
}

/**
 * rnp_open - Called when a network interface is made active
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
int rnp_open(struct net_device *netdev)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw *hw = &adapter->hw;
	int err;

	DPRINTK(IFUP, INFO, "ifup\n");

	// if(strcmp(netdev->name, "rnp00") == 0)
	//   return -EBUSY;

	/* disallow open during test */
	if (test_bit(__RNP_TESTING, &adapter->state))
		return -EBUSY;

	netif_carrier_off(netdev);

	/* allocate transmit descriptors */
	err = rnp_setup_all_tx_resources(adapter);
	if (err)
		goto err_setup_tx;

	/* allocate receive descriptors */
	err = rnp_setup_all_rx_resources(adapter);
	if (err)
		goto err_setup_rx;

	rnp_configure(adapter);

	err = rnp_request_irq(adapter);
	if (err)
		goto err_req_irq;

	/* Notify the stack of the actual queue counts. */
	err = netif_set_real_num_tx_queues(netdev, adapter->num_tx_queues);
	if (err)
		goto err_set_queues;

	err = netif_set_real_num_rx_queues(netdev, adapter->num_rx_queues);
	if (err)
		goto err_set_queues;

	if (module_enable_ptp)
		rnp_ptp_register(adapter);

	rnp_up_complete(adapter);

	return 0;

err_set_queues:
	rnp_free_irq(adapter);
err_req_irq:
	rnp_free_all_rx_resources(adapter);
err_setup_rx:
	rnp_free_all_tx_resources(adapter);
err_setup_tx:
	// rnp_mbx_ifup_down(&adapter->hw, 0);
	hw->ops.set_mbx_ifup(hw, 0);
	rnp_reset(adapter);

	return err;
}

/**
 * rnp_close - Disables a network interface
 * @netdev: network interface device structure
 *
 * Returns 0, this is not allowed to fail
 *
 * The close entry point is called when an interface is de-activated
 * by the OS.  The hardware is still under the drivers control, but
 * needs to be disabled.  A global MAC reset is issued to stop the
 * hardware, and all transmit and receive resources are freed.
 **/
int rnp_close(struct net_device *netdev)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw *hw = &adapter->hw;
	DPRINTK(IFDOWN, INFO, "ifdown\n");

#ifdef DISABLE_RX_IRQ
	adapter->quit_poll_thread = true;
#endif
	// rnp_mbx_ifup_down(&adapter->hw, 0);

	if (module_enable_ptp)
		rnp_ptp_unregister(adapter);

	rnp_down(adapter);

	rnp_free_irq(adapter);

	// rnp_fdir_filter_exit(adapter);

	rnp_free_all_tx_resources(adapter);
	rnp_free_all_rx_resources(adapter);

	// rnp_mbx_link_event_enable(&adapter->hw, 0);
	// rnp_mbx_ifup_down(&adapter->hw, 0);

	// if in sriov mode send link down to all vfs
	if (adapter->flags & RNP_FLAG_SRIOV_ENABLED) {
		// fixme in n500
		struct rnp_hw *hw = &adapter->hw;
		//
		adapter->link_up = 0;
		adapter->link_up_old = 0;
		// wr32(hw, RNP_ETH_EXCEPT_DROP_PROC, 0xf);
		rnp_msg_post_status(adapter, PF_SET_LINK_STATUS);
		/* wait all vf get this status */
		usleep_range(500, 1000);
	}
	// rnp_release_hw_control(adapter);

	return 0;
}

#ifdef CONFIG_PM
static int rnp_resume(struct pci_dev *pdev)
{
	struct rnp_adapter *adapter = pci_get_drvdata(pdev);
	struct net_device *netdev = adapter->netdev;
	u32 err;

	printk("call rnp_resume\n");
	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	/*
	 * pci_restore_state clears dev->state_saved so call
	 * pci_save_state to restore it.
	 */
	pci_save_state(pdev);

	err = pcim_enable_device(pdev);
	if (err) {
		e_dev_err("Cannot enable PCI device from suspend\n");
		return err;
	}
	pci_set_master(pdev);

	pci_wake_from_d3(pdev, false);

	rnp_reset(adapter);

	// RNP_WRITE_REG(&adapter->hw, RNP_WUS, ~0);

	rtnl_lock();

	err = rnp_init_interrupt_scheme(adapter);
	if (!err)
		err = register_mbx_irq(adapter);

	if (!err && netif_running(netdev))
		err = rnp_open(netdev);

	rtnl_unlock();

	if (err)
		return err;

	netif_device_attach(netdev);

	return 0;
}
#endif /* CONFIG_PM */

static int __rnp_shutdown(struct pci_dev *pdev, bool *enable_wake)
{
	struct rnp_adapter *adapter = pci_get_drvdata(pdev);
	struct net_device *netdev = adapter->netdev;
	struct rnp_hw *hw = &adapter->hw;
	u32 ctrl, fctrl;
	u32 wufc = adapter->wol;
#ifdef CONFIG_PM
	int retval = 0;
#endif

	netif_device_detach(netdev);

	rtnl_lock();
	if (netif_running(netdev)) {
		rnp_down(adapter);
		rnp_free_irq(adapter);
		rnp_free_all_tx_resources(adapter);
		rnp_free_all_rx_resources(adapter);
		/* should consider sriov mode ? */
	}
	rtnl_unlock();

	remove_mbx_irq(adapter);
	rnp_clear_interrupt_scheme(adapter);

#ifdef CONFIG_PM
	retval = pci_save_state(pdev);
	if (retval)
		return retval;

#endif
	if (wufc) {
		rnp_set_rx_mode(netdev);

		/* enable the optics for n10 SFP+ fiber as we can WoL */
		if (hw->ops.enable_tx_laser)
			hw->ops.enable_tx_laser(hw);

		/* turn on all-multi mode if wake on multicast is enabled */

	} else {
	}

	pci_wake_from_d3(pdev, !!wufc);
	*enable_wake = !!wufc;

	// rnp_release_hw_control(adapter);

	pci_disable_device(pdev);

	return 0;
}

#ifdef CONFIG_PM
static int rnp_suspend(struct pci_dev *pdev, pm_message_t state)
{
	int retval;
	bool wake;

	printk("call rnp_suspend\n");
	retval = __rnp_shutdown(pdev, &wake);
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

static void rnp_shutdown(struct pci_dev *pdev)
{
	bool wake;

	__rnp_shutdown(pdev, &wake);

	if (system_state == SYSTEM_POWER_OFF) {
		pci_wake_from_d3(pdev, wake);
		pci_set_power_state(pdev, PCI_D3hot);
	}
}
/**
 * rnp_update_stats - Update the board statistics counters.
 * @adapter: board private structure
 **/
void rnp_update_stats(struct rnp_adapter *adapter)
{
	struct net_device_stats *net_stats = &adapter->netdev->stats;
	struct rnp_hw *hw = &adapter->hw;
	struct rnp_hw_stats *hw_stats = &adapter->hw_stats;

	int i, port;
	struct rnp_ring *ring;
	// u64 packets = 0;
	// u64 bytes = 0;
	u64 hw_csum_rx_error = 0;
	u64 hw_csum_rx_good = 0;

	net_stats->tx_packets = 0;
	net_stats->tx_bytes = 0;
	// net_stats->tx_dropped = 0;
	// net_stats->tx_errors = 0;

	net_stats->rx_packets = 0;
	net_stats->rx_bytes = 0;
	net_stats->rx_dropped = 0;
	net_stats->rx_errors = 0;
	hw_stats->vlan_strip_cnt = 0;
	hw_stats->vlan_add_cnt = 0;

	if (test_bit(__RNP_DOWN, &adapter->state) ||
		test_bit(__RNP_RESETTING, &adapter->state))
		return;

	for (i = 0; i < adapter->num_q_vectors; i++) {
		rnp_for_each_ring(ring, adapter->q_vector[i]->rx)
		{
			hw_csum_rx_error += ring->rx_stats.csum_err;
			hw_csum_rx_good += ring->rx_stats.csum_good;
			hw_stats->vlan_strip_cnt += ring->rx_stats.vlan_remove;
			net_stats->rx_packets += ring->stats.packets;
			net_stats->rx_bytes += ring->stats.bytes;
			//			packets += ring->rx_stats.csum_err;
		}
		rnp_for_each_ring(ring, adapter->q_vector[i]->tx)
		{
			hw_stats->vlan_add_cnt += ring->tx_stats.vlan_add;
			net_stats->tx_packets += ring->stats.packets;
			net_stats->tx_bytes += ring->stats.bytes;
		}
	}
	// net_stats->rx_dropped = packets;
	//  update hardware status

	net_stats->rx_errors += hw_csum_rx_error;

	hw->ops.update_hw_status(hw, hw_stats, net_stats);

	// hw_stats->dma_to_dma = eth_rd32(eth,
	// RNP10_DMA_STATS_DMA_TO_MAC_CHANNEL_0) + 		       eth_rd32(eth,
	//RNP10_DMA_STATS_DMA_TO_MAC_CHANNEL_1) + 		       eth_rd32(eth,
	//RNP10_DMA_STATS_DMA_TO_MAC_CHANNEL_2) + 		       eth_rd32(eth,
	//RNP10_DMA_STATS_DMA_TO_MAC_CHANNEL_3);

	// hw_stats->dma_to_switch = rd32(hw, RNP_DMA_STATS_DMA_TO_SWITCH);
	////hw_stats->mac_to_mac = rd32(hw, RNP_DMA_STATS_MAC_TO_MAC);
	////hw_stats->switch_to_switch = rd32(hw, RNP_DMA_STATS_SWITCH_TO_SWITCH);
	// hw_stats->mac_to_dma = rd32(hw, RNP_DMA_STATS_MAC_TO_DMA);

	////=== rx
	////hw_stats->dma_to_host = rd32(hw, RNP_PCI_WR_TO_HOST) / 2;

	// for (port = 0; port < 4; port++) {
	//	/* we use Hardware stats? */
	//	//net_stats->rx_packets += rd32(hw, RNP_RXTRANS_RX_PKTS(port));
	//	//net_stats->rx_dropped += rd32(hw, RNP_RXTRANS_DROP_PKTS(port));
	//	net_stats->rx_crc_errors =
	//		rd32(hw, RNP_RXTRANS_CRC_ERR_PKTS(port));
	//	/*net_stats->rx_frame_errors +=
	//		rd32(hw, RNP_RXTRANS_SLEN_ERR_PKTS(port)) +
	//		rd32(hw, RNP_RXTRANS_GLEN_ERR_PKTS(port)) +
	//		rd32(hw, RNP_RXTRANS_IPH_ERR_PKTS(port)) +
	//		rd32(hw, RNP_RXTRANS_CSUM_ERR_PKTS(port)) +
	//		rd32(hw, RNP_RXTRANS_LEN_ERR_PKTS(port)) +
	//		rd32(hw, RNP_RXTRANS_CUT_ERR_PKTS(port));
	//	*/

	//	net_stats->rx_errors +=
	//		rd32(hw, RNP_RXTRANS_WDT_ERR_PKTS(port)) +
	//		rd32(hw, RNP_RXTRANS_CODE_ERR_PKTS(port)) +
	//		rd32(hw, RNP_RXTRANS_CRC_ERR_PKTS(port)) +
	//		rd32(hw, RNP_RXTRANS_SLEN_ERR_PKTS(port)) +
	//		rd32(hw, RNP_RXTRANS_GLEN_ERR_PKTS(port)) +
	//		rd32(hw, RNP_RXTRANS_IPH_ERR_PKTS(port)) +
	//		rd32(hw, RNP_RXTRANS_LEN_ERR_PKTS(port)) +
	//		rd32(hw, RNP_RXTRANS_CUT_ERR_PKTS(port));
	//	// tx
	//	/* use hardware reg status */
	//	//net_stats->tx_packets +=
	//	//	rd32(hw, RNP_ETH_EMAC_TX_TO_PHY_PKTS(port));
	//}
	////=== drop ===
	// hw_stats->invalid_droped_packets = rd32(hw, RNP_ETH_INVALID_DROP_PKTS);
	// hw_stats->filter_dropped_packets = rd32(hw, RNP_ETH_FILTER_DROP_PKTS);
	// hw_stats->host_l2_match_drop = rd32(hw, RNP_ETH_HOST_L2_DROP_PKTS);
	// hw_stats->redir_input_match_drop =
	//	rd32(hw, RNP_ETH_REDIR_INPUT_MATCH_DROP_PKTS);
	// hw_stats->redir_etype_match_drop = rd32(hw, RNP_ETH_ETYPE_DROP_PKTS);
	// hw_stats->redir_tcp_syn_match_drop =
	//	rd32(hw, RNP_ETH_TCP_SYN_DROP_PKTS);
	// hw_stats->redir_tuple5_match_drop =
	//	rd32(hw, RNP_ETH_REDIR_TUPLE5_DROP_PKTS);
	// hw_stats->redir_tcam_match_drop =
	//	rd32(hw, RNP_ETH_REDIR_TCAM_DROP_PKTS);
	// hw_stats->bmc_dropped_packets = rd32(hw, RNP_ETH_DECAP_BMC_DROP_NUM);
	// hw_stats->switch_dropped_packets =
	//	rd32(hw, RNP_ETH_DECAP_SWITCH_DROP_NUM);
	// hw_stats->mac_rx_broadcast = rd32(hw, RNP_MAC_STATS_BROADCAST_LOW);
	// hw_stats->mac_rx_broadcast +=
	//	((u64)rd32(hw, RNP_MAC_STATS_BROADCAST_HIGH) << 32);

	// hw_stats->mac_rx_multicast = rd32(hw, RNP_MAC_STATS_MULTICAST_LOW);
	// hw_stats->mac_rx_multicast +=
	//	((u64)rd32(hw, RNP_MAC_STATS_MULTICAST_HIGH) << 32);

	//=== emac 1to4 tx ==
	// hw_stats->in0_tx_pkts = rd32(hw, RNP_ETH_1TO4_INST0_IN_PKTS);
	// hw_stats->in1_tx_pkts = rd32(hw, RNP_ETH_1TO4_INST1_IN_PKTS);
	// hw_stats->in2_tx_pkts = rd32(hw, RNP_ETH_1TO4_INST2_IN_PKTS);
	// hw_stats->in3_tx_pkts = rd32(hw, RNP_ETH_1TO4_INST3_IN_PKTS);
	//=== phy tx ==
	// hw_stats->port0_to_phy_pkts = rd32(hw, RNP_ETH_EMAC_TX_TO_PHY_PKTS(0));
	// hw_stats->port1_to_phy_pkts = rd32(hw, RNP_ETH_EMAC_TX_TO_PHY_PKTS(1));
	// hw_stats->port2_to_phy_pkts = rd32(hw, RNP_ETH_EMAC_TX_TO_PHY_PKTS(2));
	// hw_stats->port3_to_phy_pkts = rd32(hw, RNP_ETH_EMAC_TX_TO_PHY_PKTS(3));

	adapter->hw_csum_rx_error = hw_csum_rx_error;
	adapter->hw_csum_rx_good = hw_csum_rx_good;
	net_stats->rx_errors = hw_csum_rx_error;
}

#if 0

/**
 * rnp_fdir_reinit_subtask - worker thread to reinit FDIR filter table
 * @adapter: pointer to the device adapter structure
 **/
static void rnp_fdir_reinit_subtask(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	int i;

	if (!(adapter->flags2 & RNP_FLAG2_FDIR_REQUIRES_REINIT))
		return;

	adapter->flags2 &= ~RNP_FLAG2_FDIR_REQUIRES_REINIT;

	/* if interface is down do nothing */
	if (test_bit(__RNP_DOWN, &adapter->state))
		return;

	/* do nothing if we are not using signature filters */
	if (!(adapter->flags & RNP_FLAG_FDIR_HASH_CAPABLE))
		return;

	adapter->fdir_overflow++;

	if (rnp_reinit_fdir_tables_n10(hw) == 0) {
		for (i = 0; i < adapter->num_tx_queues; i++)
			set_bit(__RNP_TX_FDIR_INIT_DONE,
			&(adapter->tx_ring[i]->state));
		/* re-enable flow director interrupts */
		RNP_WRITE_REG(hw, RNP_EIMS, RNP_EIMS_FLOW_DIR);
	} else {
		e_err(probe, "failed to finish FDIR re-initialization, "
		      "ignored adding FDIR ATR filters\n");
	}
}

#endif
/**
 * rnp_check_hang_subtask - check for hung queues and dropped interrupts
 * @adapter: pointer to the device adapter structure
 *
 * This function serves two purposes.  First it strobes the interrupt lines
 * in order to make certain interrupts are occurring.  Secondly it sets the
 * bits needed to check for TX hangs.  As a result we should immediately
 * determine if a hang has occurred.
 */
static void rnp_check_hang_subtask(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	u64 eics = 0;
	int i;
	struct rnp_ring *tx_ring;
	u64 tx_next_to_clean_old;
	u64 tx_next_to_clean;
	u64 tx_next_to_use;
	struct rnp_ring *rx_ring;
	u64 rx_next_to_clean_old;
	u64 rx_next_to_clean;
	u64 rx_next_to_use;
	union rnp_rx_desc *rx_desc;

	/* If we're down or resetting, just bail */
	if (test_bit(__RNP_DOWN, &adapter->state) ||
		test_bit(__RNP_RESETTING, &adapter->state))
		return;

	/* Force detection of hung controller */
	if (netif_carrier_ok(adapter->netdev)) {
		for (i = 0; i < adapter->num_tx_queues; i++)
			set_check_for_tx_hang(adapter->tx_ring[i]);
	}

	//	if (!(adapter->flags & RNP_FLAG_MSIX_ENABLED)) {
	//		/*
	//		 * for legacy and MSI interrupts don't set any bits
	//		 * that are enabled for EIAM, because this operation
	//		 * would set *both* EIMS and EICS for any bit in EIAM
	//		 */
	//		RNP_WRITE_REG(hw, RNP_EICS,
	//			(RNP_EICS_TCP_TIMER | RNP_EICS_OTHER));
	//	} else {
	//		/* get one bit for every active tx/rx interrupt vector */
	//		for (i = 0; i < adapter->num_q_vectors; i++) {
	//			struct rnp_q_vector *qv = adapter->q_vector[i];
	//
	//			if (qv->rx.ring || qv->tx.ring)
	//				eics |= ((u64)1 << i);
	//		}
	//	}
	//
	//	/* Cause software interrupt to ensure rings are cleaned */
	//	rnp_irq_rearm_queues(adapter, eics);

	// todo fix irq?
	// check if we lost tx irq ?
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
					struct rnp_q_vector *q_vector = tx_ring->q_vector;

					/* stats */
					// printk("maybe tx irq miss happen!!! \n");
					if (q_vector->rx.ring || q_vector->tx.ring)
						napi_schedule_irqoff(&q_vector->napi);

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
				rx_desc = RNP_RX_DESC(rx_ring, rx_ring->next_to_clean);
				if (rnp_test_staterr(rx_desc, RNP_RXD_STAT_DD)) {
					int size;
					struct rnp_q_vector *q_vector = rx_ring->q_vector;

					size = le16_to_cpu(rx_desc->wb.len);
					if (size) {
						rx_ring->rx_stats.rx_irq_miss++;
						if (q_vector->rx.ring || q_vector->tx.ring)
							napi_schedule_irqoff(&q_vector->napi);
					} else {
						adapter->flags2 |= RNP_FLAG2_RESET_REQUESTED;
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
 * rnp_watchdog_update_link - update the link status
 * @adapter: pointer to the device adapter structure
 * @link_speed: pointer to a u32 to store the link_speed
 **/
static void rnp_watchdog_update_link(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	u32 link_speed = adapter->link_speed;
	bool link_up = adapter->link_up;

	if (!(adapter->flags & RNP_FLAG_NEED_LINK_UPDATE))
		return;

	if (hw->ops.check_link) {
		hw->ops.check_link(hw, &link_speed, &link_up, false);
	} else {
		/* always assume link is up, if no check link function */
		link_speed = RNP_LINK_SPEED_10GB_FULL;
		link_up = true;
	}

	if (link_up ||
		time_after(jiffies,
				   (adapter->link_check_timeout + RNP_TRY_LINK_TIMEOUT))) {
		adapter->flags &= ~RNP_FLAG_NEED_LINK_UPDATE;
	}
	//if (adapter->flags2 & RNP_FLAG2_TEST_INFO)
	//	printk("in update link %x %x\n", link_up, link_speed);
	adapter->link_up = link_up;
	adapter->link_speed = link_speed;

	/*
	if ((adapter->link_up_old != link_up)
			|| (adapter->link_speed_old != link_speed)) {
		adapter->link_up_old = link_up;
		adapter->link_speed_old = link_speed;
		// if change send mbx to all vf
		rnp_msg_post_status(adapter, PF_SET_LINK_STATUS);

	}*/
}

static void rnp_update_default_up(struct rnp_adapter *adapter)
{
#ifdef CONFIG_RNP_DCB
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
 * rnp_watchdog_link_is_up - update netif_carrier status and
 *                             print link up message
 * @adapter: pointer to the device adapter structure
 **/
static void rnp_watchdog_link_is_up(struct rnp_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	struct rnp_hw *hw = &adapter->hw;
	u32 link_speed = adapter->link_speed;
	bool flow_rx = true, flow_tx = true;

	/* only continue if link was previously down */
	if (netif_carrier_ok(netdev))
		return;

	adapter->flags2 &= ~RNP_FLAG2_SEARCH_FOR_SFP;
	switch (hw->mac.type) {
		default:
			break;
	}

	e_info(drv,
		   "NIC Link is Up %s, Flow Control: %s\n",
		   (link_speed == RNP_LINK_SPEED_40GB_FULL
				? "40 Gbps"
				: (link_speed == RNP_LINK_SPEED_25GB_FULL
					   ? "25 Gbps"
					   : (link_speed == RNP_LINK_SPEED_10GB_FULL
							  ? "10 Gbps"
							  : (link_speed == RNP_LINK_SPEED_1GB_FULL
									 ? "1 Gbps"
									 : (link_speed == RNP_LINK_SPEED_100_FULL
											? "100 Mbps"
											: "unknown speed"))))),
		   ((flow_rx && flow_tx)
				? "RX/TX"
				: (flow_rx ? "RX" : (flow_tx ? "TX" : "None"))));

	netif_carrier_on(netdev);

	//rnp_link_stat_mark(hw, 1);

	netif_tx_wake_all_queues(netdev);
	// rnp_check_vf_rate_limit(adapter);

	/* update the default user priority for VFs */
	rnp_update_default_up(adapter);

	hw->ops.set_mac_rx(hw, true);

	/* ping all the active vfs to let them know link has changed */
	// rnp_ping_all_vfs(adapter);
}

/**
 * rnp_watchdog_link_is_down - update netif_carrier status and
 *                               print link down message
 * @adapter: pointer to the adapter structure
 **/
static void rnp_watchdog_link_is_down(struct rnp_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	struct rnp_hw *hw = &adapter->hw;

	adapter->link_up = false;
	adapter->link_speed = 0;
	//rnp_link_stat_mark(hw, 0);

	/* only continue if link was up previously */
	if (!netif_carrier_ok(netdev))
		return;

	/* poll for SFP+ cable when link is down */
	if (rnp_is_sfp(hw))
		adapter->flags2 |= RNP_FLAG2_SEARCH_FOR_SFP;

	e_info(drv, "NIC Link is Down\n");
	netif_carrier_off(netdev);

	netif_tx_stop_all_queues(netdev);

	hw->ops.set_mac_rx(hw, false);
	/* ping all the active vfs to let them know link has changed */
	// rnp_ping_all_vfs(adapter);
}

/**
 * rnp_watchdog_flush_tx - flush queues on link down
 * @adapter: pointer to the device adapter structure
 **/
static void rnp_watchdog_flush_tx(struct rnp_adapter *adapter)
{
	int i;
	int some_tx_pending = 0;

	if (!netif_carrier_ok(adapter->netdev)) {
		for (i = 0; i < adapter->num_tx_queues; i++) {
			struct rnp_ring *tx_ring = adapter->tx_ring[i];

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
			e_warn(drv, "initiating reset to clear Tx work after link loss\n");
			adapter->flags2 |= RNP_FLAG2_RESET_REQUESTED;
		}
	}
}

static void rnp_update_link_to_vf(struct rnp_adapter *adapter)
{
	if (!(adapter->flags & RNP_FLAG_SRIOV_INIT_DONE))
		return;

	if ((adapter->link_up_old != adapter->link_up) ||
		(adapter->link_speed_old != adapter->link_speed)) {
		// maybe delay if we are in other irq?
		adapter->link_up_old = adapter->link_up;
		adapter->link_speed_old = adapter->link_speed;
		// if change send mbx to all vf
		rnp_msg_post_status(adapter, PF_SET_LINK_STATUS);
	}
}

/**
 * rnp_watchdog_subtask - check and bring link up
 * @adapter: pointer to the device adapter structure
 **/
static void rnp_watchdog_subtask(struct rnp_adapter *adapter)
{
	/* if interface is down do nothing */
	/* should do link status if in sriov */
	if (test_bit(__RNP_DOWN, &adapter->state) ||
		test_bit(__RNP_RESETTING, &adapter->state))
		return;

	rnp_watchdog_update_link(adapter);

	if (adapter->link_up)
		rnp_watchdog_link_is_up(adapter);
	else
		rnp_watchdog_link_is_down(adapter);

	rnp_update_link_to_vf(adapter);

	rnp_update_stats(adapter);

	rnp_watchdog_flush_tx(adapter);
}

/**
 * rnp_sfp_detection_subtask - poll for SFP+ cable
 * @adapter: the rnp adapter structure
 **/
static void rnp_sfp_detection_subtask(struct rnp_adapter *adapter)
{
#if 0
	struct rnp_hw *hw = &adapter->hw;
	s32 err;

	/* not searching for SFP so there is nothing to do here */
	if (!(adapter->flags2 & RNP_FLAG2_SEARCH_FOR_SFP) &&
	    !(adapter->flags2 & RNP_FLAG2_SFP_NEEDS_RESET))
		return;

	/* concurent i2c reads are not supported */
	if (test_bit(__RNP_READ_I2C, &adapter->state))
		return;

	/* someone else is in init, wait until next service event */
	if (test_and_set_bit(__RNP_IN_SFP_INIT, &adapter->state))
		return;

	err = hw->phy.ops.identify_sfp(hw);
	if (err == RNP_ERR_SFP_NOT_SUPPORTED)
		goto sfp_out;

	if (err == RNP_ERR_SFP_NOT_PRESENT) {
		/* If no cable is present, then we need to reset
		 * the next time we find a good cable.
		 */
		adapter->flags2 |= RNP_FLAG2_SFP_NEEDS_RESET;
	}

	/* exit on error */
	if (err)
		goto sfp_out;

	/* exit if reset not needed */
	if (!(adapter->flags2 & RNP_FLAG2_SFP_NEEDS_RESET))
		goto sfp_out;

	adapter->flags2 &= ~RNP_FLAG2_SFP_NEEDS_RESET;

	/*
	 * A module may be identified correctly, but the EEPROM may not have
	 * support for that module.  setup_sfp() will fail in that case, so
	 * we should not allow that module to load.
	 */
	if (hw->mac.type == rnp_mac_82598EB)
		err = hw->phy.ops.reset(hw);
	else
		err = hw->mac.ops.setup_sfp(hw);

	if (err == RNP_ERR_SFP_NOT_SUPPORTED)
		goto sfp_out;

	adapter->flags |= RNP_FLAG_NEED_LINK_CONFIG;
	e_info(probe, "detected SFP+: %d\n", hw->phy.sfp_type);

sfp_out:
	clear_bit(__RNP_IN_SFP_INIT, &adapter->state);

	if ((err == RNP_ERR_SFP_NOT_SUPPORTED) &&
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
 * rnp_sfp_link_config_subtask - set up link SFP after module install
 * @adapter: the rnp adapter structure
 **/
static void rnp_sfp_link_config_subtask(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	u32 speed;
	bool autoneg = false;

	if (!(adapter->flags & RNP_FLAG_NEED_LINK_CONFIG))
		return;

#if 0
	/* someone else is in init, wait until next service event */
	if (test_and_set_bit(__RNP_IN_SFP_INIT, &adapter->state))
		return;

	adapter->flags &= ~RNP_FLAG_NEED_LINK_CONFIG;

	speed = hw->phy.autoneg_advertised;
	if ((!speed) && (hw->mac.ops.get_link_capabilities))
		hw->mac.ops.get_link_capabilities(hw, &speed, &autoneg);
	if (hw->mac.ops.setup_link)
		hw->mac.ops.setup_link(hw, speed, true);

	adapter->flags |= RNP_FLAG_NEED_LINK_UPDATE;
	adapter->link_check_timeout = jiffies;
	clear_bit(__RNP_IN_SFP_INIT, &adapter->state);
#endif
}

#ifdef CONFIG_PCI_IOV
static void rnp_check_for_bad_vf(struct rnp_adapter *adapter)
{
	int vf;
	struct rnp_hw *hw = &adapter->hw;
	struct net_device *netdev = adapter->netdev;
	u32 gpc;
	u32 ciaa, ciad;

#if 0
	gpc = RNP_READ_REG(hw, RNP_TXDGPC);
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
		RNP_WRITE_REG(hw, RNP_CIAA_n10, ciaa);
		ciad = RNP_READ_REG(hw, RNP_CIAD_n10);
		ciaa &= 0x7FFFFFFF;
		/* disable debug mode asap after reading data */
		RNP_WRITE_REG(hw, RNP_CIAA_n10, ciaa);
		/* Get the upper 16 bits which will be the PCI status reg */
		ciad >>= 16;
		if (ciad & PCI_STATUS_REC_MASTER_ABORT) {
			netdev_err(netdev, "VF %d Hung DMA\n", vf);
			/* Issue VFLR */
			ciaa = (vf << 16) | 0x80000000;
			ciaa |= 0xA8;
			RNP_WRITE_REG(hw, RNP_CIAA_n10, ciaa);
			ciad = 0x00008000;  /* VFLR */
			RNP_WRITE_REG(hw, RNP_CIAD_n10, ciad);
			ciaa &= 0x7FFFFFFF;
			RNP_WRITE_REG(hw, RNP_CIAA_n10, ciaa);
		}
	}
#endif
}

#endif
/**
 * rnp_service_timer - Timer Call-back
 * @data: pointer to adapter cast into an unsigned long
 **/
void rnp_service_timer(struct timer_list *t)
{
	struct rnp_adapter *adapter = from_timer(adapter, t, service_timer);
	unsigned long next_event_offset;
	bool ready = true;

	//if (adapter->flags2 & RNP_FLAG2_TEST_INFO)
	//	printk("call rnp_service timer");

	/* poll faster when waiting for link */
	if (adapter->flags & RNP_FLAG_NEED_LINK_UPDATE)
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
	    (adapter->flags & RNP_FLAG_NEED_LINK_UPDATE))
		goto normal_timer_service;

	/* If we have VFs allocated then we must check for DMA hangs */
	rnp_check_for_bad_vf(adapter);
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
	if (!test_bit(__RNP_REMOVE, &adapter->state))
		mod_timer(&adapter->service_timer, next_event_offset + jiffies);

	if (ready)
		rnp_service_event_schedule(adapter);
}

// call this task in sriov mode
static void rnp_reset_pf_subtask(struct rnp_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	u32 err;

	if (!(adapter->flags2 & RNP_FLAG2_RESET_PF))
		return;

	rtnl_lock();
	netif_device_detach(netdev);
	if (netif_running(netdev)) {
		rnp_down(adapter);
		rnp_free_irq(adapter);
		rnp_free_all_tx_resources(adapter);
		rnp_free_all_rx_resources(adapter);
	}
	rtnl_unlock();

	// send link down to all vfs
	adapter->link_up = 0;
	adapter->link_up_old = 0;
	// wr32(hw, RNP_ETH_EXCEPT_DROP_PROC, 0xf);
	rnp_msg_post_status(adapter, PF_SET_LINK_STATUS);
	/* wait all vf get this status */
	usleep_range(500, 1000);

	// reset pf first
	rnp_reset(adapter);

	remove_mbx_irq(adapter);
	rnp_clear_interrupt_scheme(adapter);

	rtnl_lock();
	err = rnp_init_interrupt_scheme(adapter);
	register_mbx_irq(adapter);
	if (!err && netif_running(netdev))
		err = rnp_open(netdev);

	rtnl_unlock();
	// ask all pf to reset
	rnp_msg_post_status(adapter, PF_SET_RESET);

	netif_device_attach(netdev);
	adapter->flags2 &= (~RNP_FLAG2_RESET_PF);
}

static void rnp_reset_subtask(struct rnp_adapter *adapter)
{
	if (!(adapter->flags2 & RNP_FLAG2_RESET_REQUESTED))
		return;

	adapter->flags2 &= ~RNP_FLAG2_RESET_REQUESTED;

	/* If we're already down or resetting, just bail */
	if (test_bit(__RNP_DOWN, &adapter->state) ||
		test_bit(__RNP_RESETTING, &adapter->state))
		return;

	// rnp_dump(adapter);
	netdev_err(adapter->netdev, "Reset adapter\n");
	adapter->tx_timeout_count++;
	rtnl_lock();
	rnp_reinit_locked(adapter);
	rtnl_unlock();
}

static void rnp_rx_len_reset_subtask(struct rnp_adapter *adapter)
{
	int i;
	struct rnp_ring *rx_ring;

	for (i = 0; i < adapter->num_tx_queues; i++) {
		rx_ring = adapter->rx_ring[i];
		if (unlikely(rx_ring->ring_flags & RNP_RING_FLAG_DO_RESET_RX_LEN)) {
			dbg("[%s] Rx-ring %d count reset\n",
				adapter->netdev->name,
				rx_ring->rnp_queue_idx);
			rnp_rx_ring_reinit(adapter, rx_ring);
			rx_ring->ring_flags &= (~RNP_RING_FLAG_DO_RESET_RX_LEN);
		}
	}
}

/* just modify rx itr */
static void rnp_auto_itr_moderation(struct rnp_adapter *adapter)
{
	int i;
	struct rnp_ring *rx_ring;
	u64 period = (u64)(jiffies - adapter->last_moder_jiffies);
	u32 pkt_rate_high, pkt_rate_low;
	struct rnp_hw *hw = &adapter->hw;
	u64 packets;
	u64 rate;
	u64 avg_pkt_size;
	u64 rx_packets;
	u64 rx_bytes;
	u64 rx_pkt_diff;
	u32 itr_reg;
	int moder_time;

	if (!adapter->auto_rx_coal)
		return;

	if (!adapter->adaptive_rx_coal || period < adapter->sample_interval * HZ) {
		return;
	}
	pkt_rate_low = READ_ONCE(adapter->pkt_rate_low);
	pkt_rate_high = READ_ONCE(adapter->pkt_rate_high);

	/* it is time to check moderation */
	for (i = 0; i < adapter->num_rx_queues; i++) {
		u64 x, y, result;
		rx_ring = adapter->rx_ring[i];
		rx_packets = READ_ONCE(rx_ring->stats.packets);
		rx_bytes = READ_ONCE(rx_ring->stats.bytes);
		rx_pkt_diff =
			rx_packets - adapter->last_moder_packets[rx_ring->queue_index];
		packets = rx_pkt_diff;

		x = packets * HZ;
		y = do_div(x, period);
		rate = x;

		// rate = packets * HZ / period;

		x = rx_bytes - adapter->last_moder_bytes[rx_ring->queue_index];
		y = do_div(x, packets);
		result = x;

		avg_pkt_size = packets ? result : 0;
		//(rx_bytes -
		// adapter->last_moder_bytes[rx_ring->queue_index]) /
		//		packets : 0;

		if (rate > (RNP_RX_RATE_THRESH / adapter->num_rx_queues) &&
			avg_pkt_size > RNP_AVG_PKT_SMALL) {
			if (rate <= pkt_rate_low)
				moder_time = adapter->rx_usecs_low;
			else if (rate >= pkt_rate_high)
				moder_time = adapter->rx_usecs_high;
			else {
				u64 x, y, result;

				x = (rate - pkt_rate_low) *
					(adapter->rx_usecs_high - adapter->rx_usecs_low);
				y = do_div(x, (pkt_rate_high - pkt_rate_low));
				result = x;

				moder_time = result + adapter->rx_usecs_low;
				/*
					(rate - pkt_rate_low) *
						(adapter->rx_usecs_high -
						 adapter->rx_usecs_low) /
						(pkt_rate_high - pkt_rate_low) +
					adapter->rx_usecs_low;
				*/
			}
		} else {
			moder_time = adapter->rx_usecs_low;
		}

		if (moder_time != adapter->last_moder_time[rx_ring->queue_index]) {
			itr_reg = moder_time * hw->usecstocount;
			/* setup time to hw */
			ring_wr32(rx_ring, RNP_DMA_REG_RX_INT_DELAY_TIMER, itr_reg);
			adapter->last_moder_time[rx_ring->queue_index] = moder_time;
		}
		/* write back new count */
		adapter->last_moder_packets[rx_ring->queue_index] = rx_packets;
		adapter->last_moder_bytes[rx_ring->queue_index] = rx_bytes;
	}
}

/**
 * rnp_service_task - manages and runs subtasks
 * @work: pointer to work_struct containing our data
 **/
void rnp_service_task(struct work_struct *work)
{
	struct rnp_adapter *adapter =
		container_of(work, struct rnp_adapter, service_task);

	//if (adapter->flags2 & RNP_FLAG2_TEST_INFO)
	//	printk("in rnp_service task");

#if defined(HAVE_UDP_ENC_RX_OFFLOAD) || defined(HAVE_VXLAN_RX_OFFLOAD)
#ifndef HAVE_UDP_TUNNEL_NIC_INFO
	if (adapter->flags2 & RNP_FLAG2_UDP_TUN_REREG_NEEDED) {
		rtnl_lock();
		adapter->flags2 &= ~RNP_FLAG2_UDP_TUN_REREG_NEEDED;
#ifdef HAVE_UDP_ENC_RX_OFFLOAD
		udp_tunnel_get_rx_info(adapter->netdev);
#else
		vxlan_get_rx_port(adapter->netdev);
#endif /* HAVE_UDP_ENC_RX_OFFLOAD */
		rtnl_unlock();
	}
#endif /* HAVE_UDP_TUNNEL_NIC_INFO */
#endif /* HAVE_UDP_ENC_RX_OFFLOAD || HAVE_VXLAN_RX_OFFLOAD */

	rnp_reset_subtask(adapter);
	rnp_reset_pf_subtask(adapter);
	rnp_sfp_detection_subtask(adapter);
	rnp_sfp_link_config_subtask(adapter);
	rnp_watchdog_subtask(adapter);
	rnp_rx_len_reset_subtask(adapter);
	// rnp_auto_itr_moderation(adapter);
	// rnp_fdir_reinit_subtask(adapter);
	rnp_check_hang_subtask(adapter);
#if 0
	/* TODO Support */
	if (adapter->flags2 & RNP_FLAG2_PTP_ENABLED) {
		rnp_ptp_overflow_check(adapter);
		rnp_ptp_rx_hang(adapter);
	}
#endif
	rnp_service_event_complete(adapter);
}

static int rnp_tso(struct rnp_ring *tx_ring,
				   struct rnp_tx_buffer *first,
				   u32 *mac_ip_len,
				   u8 *hdr_len,
				   u32 *tx_flags)
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

	/* initialize outer IP header fields */
	if (ip.v4->version == 4) {
		/* IP header will have to cancel out any data that
		 * is not a part of the outer IP header
		 */
		ip.v4->tot_len = 0;
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
		/* we should alayws do this */
		inner_mac = skb_inner_mac_header(skb);

		first->tunnel_hdr_len = (inner_mac - skb->data);

		if (skb_shinfo(skb)->gso_type &
			(SKB_GSO_UDP_TUNNEL | SKB_GSO_UDP_TUNNEL_CSUM)) {
			*tx_flags |= RNP_TXD_TUNNEL_VXLAN;
			l4.udp->check = 0;
			tx_dbg("set outer l4.udp to 0\n");
		} else {
			*tx_flags |= RNP_TXD_TUNNEL_NVGRE;
		}
		//*tx_flags |= RNP_TXD_TUNNEL_VXLAN;
		// l4.udp->check = 0;
		// tx_dbg("set outer l4.udp to 0\n");

		/* reset pointers to inner headers */
		ip.hdr = skb_inner_network_header(skb);
		l4.hdr = skb_inner_transport_header(skb);
	}

#endif /* HAVE_ENCAP_TSO_OFFLOAD */

	if (ip.v4->version == 4) {
		/* IP header will have to cancel out any data that
		 * is not a part of the outer IP header
		 */
		ip.v4->tot_len = 0;
		ip.v4->check = 0x0000;

	} else {
		ip.v6->payload_len = 0;
		/* set ipv6 type */
		*tx_flags |= RNP_TXD_FLAG_IPv6;
	}

	/* determine offset of inner transport header */
	l4_offset = l4.hdr - skb->data;

	paylen = skb->len - l4_offset;
	tx_dbg("before l4 checksum is %x\n", l4.tcp->check);

	csum_replace_by_diff(&l4.tcp->check, (__force __wsum)htonl(paylen));

	tx_dbg("l4 checksum is %x\n", l4.tcp->check);

	*mac_ip_len = (l4.hdr - ip.hdr) | ((ip.hdr - inner_mac) << 9);

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
	*tx_flags |= RNP_TXD_FLAG_TSO | RNP_TXD_IP_CSUM | RNP_TXD_L4_CSUM |
				 RNP_TXD_L4_TYPE_TCP;

	first->ctx_flag = true;
	return 1;
}

static int rnp_tx_csum(struct rnp_ring *tx_ring,
					   struct rnp_tx_buffer *first,
					   u32 *mac_ip_len,
					   u32 *tx_flags)
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

	if (skb->ip_summed != CHECKSUM_PARTIAL)
		return 0;

	ip.hdr = skb_network_header(skb);
	l4.hdr = skb_transport_header(skb);

	inner_mac = skb->data;

#ifdef HAVE_ENCAP_CSUM_OFFLOAD
	/* outer protocol */
	// if we can handle tunnel packet
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
				*tx_flags |= RNP_TXD_TUNNEL_VXLAN;
				break;
#ifdef HAVE_GRE_ENCAP_OFFLOAD
			case IPPROTO_GRE:
				*tx_flags |= RNP_TXD_TUNNEL_NVGRE;
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
	*mac_ip_len = (ip.hdr - inner_mac) << 9;
	tx_dbg("inner checksum needed %d", skb_checksum_start_offset(skb));
	tx_dbg("skb->encapsulation %d\n", skb->encapsulation);
	ip_len = (l4.hdr - ip.hdr);
	if (ip.v4->version == 4) {
		l4_proto = ip.v4->protocol;
	} else {
		exthdr = ip.hdr + sizeof(*ip.v6);
		l4_proto = ip.v6->nexthdr;
		if (l4.hdr != exthdr)
			ipv6_skip_exthdr(skb, exthdr - skb->data, &l4_proto, &frag_off);
		*tx_flags |= RNP_TXD_FLAG_IPv6;
	}
	/* Enable L4 checksum offloads */
	switch (l4_proto) {
		case IPPROTO_TCP:
			*tx_flags |= RNP_TXD_L4_TYPE_TCP | RNP_TXD_L4_CSUM;
			break;
		case IPPROTO_SCTP:
			tx_dbg("sctp checksum packet\n");
			*tx_flags |= RNP_TXD_L4_TYPE_SCTP | RNP_TXD_L4_CSUM;
			break;
		case IPPROTO_UDP:
			*tx_flags |= RNP_TXD_L4_TYPE_UDP | RNP_TXD_L4_CSUM;
			break;
		default:
			skb_checksum_help(skb);
			return 0;
	}
	if ((tx_ring->ring_flags & RNP_RING_NO_TUNNEL_SUPPORT) &&
		(first->ctx_flag)) {
		/* if not support tunnel */
		// clean tunnel type
		*tx_flags &= (~RNP_TXD_TUNNEL_MASK);
		// add tunnel_hdr_len to mac_len
		mac_len += first->tunnel_hdr_len;
		// clean ctx
		first->tunnel_hdr_len = 0;
		first->ctx_flag = false;
	}
	tx_dbg("mac length is %d\n", mac_len);
	tx_dbg("ip length is %d\n", ip_len);
	*mac_ip_len = (mac_len << 9) | ip_len;
	return 0;
}

static int __rnp_maybe_stop_tx(struct rnp_ring *tx_ring, u16 size)
{
	netif_stop_subqueue(tx_ring->netdev, tx_ring->queue_index);
	/* Herbert's original patch had:
	 *  smp_mb__after_netif_stop_queue();
	 * but since that doesn't exist yet, just open code it.
	 */
	smp_mb();

	/* We need to check again in a case another CPU has just
	 * made room available.
	 */
	if (likely(rnp_desc_unused(tx_ring) < size))
		return -EBUSY;

	/* A reprieve! - use start_queue because it doesn't call schedule */
	netif_start_subqueue(tx_ring->netdev, tx_ring->queue_index);
	++tx_ring->tx_stats.restart_queue;
	return 0;
}

static inline int rnp_maybe_stop_tx(struct rnp_ring *tx_ring, u16 size)
{
	if (likely(rnp_desc_unused(tx_ring) >= size))
		return 0;
	return __rnp_maybe_stop_tx(tx_ring, size);
}

static int rnp_tx_map(struct rnp_ring *tx_ring,
					  struct rnp_tx_buffer *first,
					  u32 mac_ip_len,
					  u32 tx_flags)
{
	struct sk_buff *skb = first->skb;
	struct rnp_tx_buffer *tx_buffer;
	struct rnp_tx_desc *tx_desc;
	skb_frag_t *frag;
	dma_addr_t dma;
	unsigned int data_len, size;
	u16 i = tx_ring->next_to_use;
	u64 fun_id = ((u64)(tx_ring->pfvfnum) << (56));

	tx_desc = RNP_TX_DESC(tx_ring, i);
	size = skb_headlen(skb);
	data_len = skb->data_len;

	dma = dma_map_single(tx_ring->dev, skb->data, size, DMA_TO_DEVICE);

	tx_buffer = first;

	for (frag = &skb_shinfo(skb)->frags[0];; frag++) {
		if (dma_mapping_error(tx_ring->dev, dma))
			goto dma_error;

		/* record length, and DMA address */
		dma_unmap_len_set(tx_buffer, len, size);
		dma_unmap_addr_set(tx_buffer, dma, dma);

		// 1st desc
		tx_desc->pkt_addr = cpu_to_le64(dma | fun_id);

		while (unlikely(size > RNP_MAX_DATA_PER_TXD)) {
			tx_desc->vlan_cmd_bsz =
				build_ctob(tx_flags, mac_ip_len, RNP_MAX_DATA_PER_TXD);
			//==== desc==
			buf_dump_line("tx0  ", __LINE__, tx_desc, sizeof(*tx_desc));
			i++;
			tx_desc++;
			if (i == tx_ring->count) {
				tx_desc = RNP_TX_DESC(tx_ring, 0);
				i = 0;
			}
			dma += RNP_MAX_DATA_PER_TXD;
			size -= RNP_MAX_DATA_PER_TXD;

			tx_desc->pkt_addr = cpu_to_le64(dma | fun_id);
		}

		buf_dump_line("tx1  ", __LINE__, tx_desc, sizeof(*tx_desc));
		if (likely(!data_len)) // if not sg break
			break;
		tx_desc->vlan_cmd_bsz = build_ctob(tx_flags, mac_ip_len, size);
		buf_dump_line("tx2  ", __LINE__, tx_desc, sizeof(*tx_desc));

		//==== frag==
		i++;
		tx_desc++;
		if (i == tx_ring->count) {
			tx_desc = RNP_TX_DESC(tx_ring, 0);
			i = 0;
		}
		// tx_desc->cmd = RNP_TXD_CMD_RS;
		// tx_desc->mac_ip_len = 0;

		size = skb_frag_size(frag);
		data_len -= size;
		dma = skb_frag_dma_map(tx_ring->dev, frag, 0, size, DMA_TO_DEVICE);
		tx_buffer = &tx_ring->tx_buffer_info[i];
	}

	/* write last descriptor with RS and EOP bits */
	tx_desc->vlan_cmd_bsz = build_ctob(
		tx_flags | RNP_TXD_CMD_EOP | RNP_TXD_CMD_RS, mac_ip_len, size);
	buf_dump_line("tx3  ", __LINE__, tx_desc, sizeof(*tx_desc));

	/* set the timestamp */
	first->time_stamp = jiffies;

	tx_ring->tx_stats.send_bytes += first->bytecount;
#ifdef NO_BQL_TEST
#else
	netdev_tx_sent_queue(txring_txq(tx_ring), first->bytecount);
#endif

	/*
	 * Force memory writes to complete before letting h/w know there
	 * are new descriptors to fetch.  (Only applicable for weak-ordered
	 * memory model archs, such as IA-64).
	 *
	 * We also need this memory barrier to make certain all of the
	 * status bits have been updated before next_to_watch is written.
	 */
	/* timestamp the skb as late as possible, just prior to notifying
	 *          * the MAC that it should transmit this packet
	 *                   */
	wmb();
	/* set next_to_watch value indicating a packet is present */
	first->next_to_watch = tx_desc;

	buf_dump_line("tx4  ", __LINE__, tx_desc, sizeof(*tx_desc));
	i++;
	if (i == tx_ring->count)
		i = 0;
	tx_ring->next_to_use = i;

	/* need this */
	rnp_maybe_stop_tx(tx_ring, DESC_NEEDED);

	skb_tx_timestamp(skb);
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

#ifdef HAVE_SKB_XMIT_MORE
	if (netif_xmit_stopped(txring_txq(tx_ring)) || !netdev_xmit_more()) {
		tx_ring->tx_stats.send_bytes_to_hw += first->bytecount;
		tx_ring->tx_stats.send_bytes_to_hw += tx_ring->tx_stats.todo_update;
		tx_ring->tx_stats.todo_update = 0;
		rnp_wr_reg(tx_ring->tail, i);
#ifndef SPIN_UNLOCK_IMPLIES_MMIOWB
		/* we need this if more than one processor can write to our tail
		 * at a time, it synchronizes IO on IA64/Altix systems
		 */
		mmiowb();
#endif
	} else {
		tx_ring->tx_stats.todo_update += first->bytecount;
	}
#else
	/* notify HW of packet */
	rnp_wr_reg(tx_ring->tail, i);

#ifndef SPIN_UNLOCK_IMPLIES_MMIOWB
	/* we need this if more than one processor can write to our tail
	 * at a time, it synchronizes IO on IA64/Altix systems
	 */
	mmiowb();
#endif
#endif /* HAVE_SKB_XMIT_MORE */

#endif
	return 0;
dma_error:
	dev_err(tx_ring->dev, "TX DMA map failed\n");

	/* clear dma mappings for failed tx_buffer_info map */
	for (;;) {
		tx_buffer = &tx_ring->tx_buffer_info[i];
		rnp_unmap_and_free_tx_resource(tx_ring, tx_buffer);
		if (tx_buffer == first)
			break;
		if (i == 0)
			i += tx_ring->count;
		i--;
	}
	dev_kfree_skb_any(first->skb);
	first->skb = NULL;
	tx_ring->next_to_use = i;

	return -1;
}

static void rnp_atr(struct rnp_ring *ring, struct rnp_tx_buffer *first)
{
	struct rnp_q_vector *q_vector = ring->q_vector;
#if 0
	union rnp_atr_hash_dword input = { .dword = 0 };
	union rnp_atr_hash_dword common = { .dword = 0 };
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
	if ((first->protocol != htons(ETH_P_IPV6) ||
	     hdr.ipv6->nexthdr != IPPROTO_TCP) &&
	    (first->protocol != htons(ETH_P_IP) ||
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

	vlan_id = htons(first->tx_flags >> RNP_TX_FLAGS_VLAN_SHIFT);

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
	if (first->tx_flags & (RNP_TX_FLAGS_SW_VLAN | RNP_TX_FLAGS_HW_VLAN))
		common.port.src ^= th->dest ^ htons(ETH_P_8021Q);
	else
		common.port.src ^= th->dest ^ first->protocol;
	common.port.dst ^= th->source;

	if (first->protocol == htons(ETH_P_IP)) {
		input.formatted.flow_type = RNP_ATR_FLOW_TYPE_TCPV4;
		common.ip ^= hdr.ipv4->saddr ^ hdr.ipv4->daddr;
	} else {
		input.formatted.flow_type = RNP_ATR_FLOW_TYPE_TCPV6;
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
	rnp_fdir_add_signature_filter_n10(&q_vector->adapter->hw,
					      input, common, ring->queue_index);
#endif
}

static void rnp_force_src_mac(struct sk_buff *skb, struct net_device *netdev)
{
	u8 *data = skb->data;
	bool ret = false;
	struct netdev_hw_addr *ha;
	// check the first u8
	// force all src mac to myself
	if (is_multicast_ether_addr(data)) {
		if (0 == memcmp(data + netdev->addr_len,
						netdev->dev_addr,
						netdev->addr_len)) {
			ret = true;
			goto DONE;
		}
		netdev_for_each_uc_addr(ha, netdev)
		{
			if (0 ==
				memcmp(data + netdev->addr_len, ha->addr, netdev->addr_len)) {
				// printk("drop own packets\n");
				ret = true;
				// if it is src mac, nothing todo
				goto DONE;
			}
		}
		/* if not src mac, force to src mac */
		if (!ret)
			memcpy(data + netdev->addr_len, netdev->dev_addr, netdev->addr_len);
	}
DONE:
	return;
}

netdev_tx_t rnp_xmit_frame_ring(struct sk_buff *skb,
								struct rnp_adapter *adapter,
								struct rnp_ring *tx_ring)
{
	struct rnp_tx_buffer *first;
	int tso;
	u32 tx_flags = 0;
	unsigned short f;
	u16 count = TXD_USE_COUNT(skb_headlen(skb));
	__be16 protocol = skb->protocol;
	u8 hdr_len = 0;
	int ignore_vlan = 0;
	/* default len should not 0 (hw request) */
	u32 mac_ip_len = 20;

	tx_dbg("=== begin ====\n");

	// rnp_skb_dump(skb, true);

	tx_dbg("rnp skb:%p, skb->len:%d  headlen:%d, data_len:%d\n",
		   skb,
		   skb->len,
		   skb_headlen(skb),
		   skb->data_len);
	tx_dbg("next_to_clean %d, next_to_use %d\n",
		   tx_ring->next_to_clean,
		   tx_ring->next_to_use);
	/*
	 * need: 1 descriptor per page * PAGE_SIZE/RNP_MAX_DATA_PER_TXD,
	 *       + 1 desc for skb_headlen/RNP_MAX_DATA_PER_TXD,
	 *       + 2 desc gap to keep tail from touching head,
	 *       + 1 desc for context descriptor,
	 * otherwise try next time
	 */
	for (f = 0; f < skb_shinfo(skb)->nr_frags; f++) {
		skb_frag_t *frag_temp = &skb_shinfo(skb)->frags[f];

		count += TXD_USE_COUNT(skb_frag_size(frag_temp));
		tx_dbg(" rnp #%d frag: size:%d\n", f, skb_frag_size(frag_temp));
	}

	if (rnp_maybe_stop_tx(tx_ring, count + 3)) {
		tx_ring->tx_stats.tx_busy++;
		return NETDEV_TX_BUSY;
	}

	if (adapter->flags & RNP_FLAG_SRIOV_ENABLED)
		rnp_force_src_mac(skb, tx_ring->netdev);

	/* record the location of the first descriptor for this packet */
	first = &tx_ring->tx_buffer_info[tx_ring->next_to_use];
	first->skb = skb;
	first->bytecount = skb->len;
	first->gso_segs = 1;

	first->mss_len_vf_num = 0;
	first->inner_vlan_tunnel_len = 0;

	first->ctx_flag = (adapter->flags & RNP_FLAG_SRIOV_ENABLED) ? true : false;

#ifdef FIX_MAC_PADDIN
        // for test only
        first->ctx_flag = true;
	first->gso_need_padding = false;
#endif


	/* if we have a HW VLAN tag being added default to the HW one */
	/* RNP_TXD_VLAN_VALID is used for veb */
	if (adapter->flags2 & RNP_FLAG2_VLAN_STAGS_ENABLED) {
		/* always add a stags for any packets out */
		tx_flags |= adapter->stags_vid;
		tx_flags |= RNP_TXD_VLAN_CTRL_INSERT_VLAN;
		if (skb_vlan_tag_present(skb)) {
			tx_flags |= RNP_TXD_VLAN_VALID;
			first->inner_vlan_tunnel_len |= (skb_vlan_tag_get(skb) << 8);
			first->ctx_flag = true;
			/* else if it is a SW VLAN check the next
			 * protocol and store the tag
			 */
		} else if (protocol == htons(ETH_P_8021Q)) {
			struct vlan_hdr *vhdr, _vhdr;

			vhdr = skb_header_pointer(skb, ETH_HLEN, sizeof(_vhdr), &_vhdr);
			if (!vhdr)
				goto out_drop;

			protocol = vhdr->h_vlan_encapsulated_proto;
			tx_flags |= RNP_TXD_VLAN_VALID;
		}
	} else {
		/* normal mode */
		if (skb_vlan_tag_present(skb)) {
			tx_flags |= skb_vlan_tag_get(skb);
			tx_flags |= RNP_TXD_VLAN_VALID | RNP_TXD_VLAN_CTRL_INSERT_VLAN;
			tx_ring->tx_stats.vlan_add++;

			/* else if it is a SW VLAN check the next
			 * protocol and store the tag
			 */
		} else if (protocol == htons(ETH_P_8021Q)) {
			struct vlan_hdr *vhdr, _vhdr;

			vhdr = skb_header_pointer(skb, ETH_HLEN, sizeof(_vhdr), &_vhdr);
			if (!vhdr)
				goto out_drop;

			protocol = vhdr->h_vlan_encapsulated_proto;
			tx_flags |= ntohs(vhdr->h_vlan_TCI);
			tx_flags |= RNP_TXD_VLAN_VALID;
			ignore_vlan = 1;
		}
	}
	protocol = vlan_get_protocol(skb);
#ifdef SKB_SHARED_TX_IS_UNION
	if (unlikely(skb_tx(skb)->hardware) &&
		adapter->flags2 & RNP_FLAG2_PTP_ENABLED && adapter->ptp_tx_en) {
		if (!test_and_set_bit_lock(__RNP_PTP_TX_IN_PROGRESS, &adapter->state)) {
			skb_tx(skb)->in_progress = 1;

#else
	if (unlikely(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
		adapter->flags2 & RNP_FLAG2_PTP_ENABLED && adapter->ptp_tx_en) {

		if (!test_and_set_bit_lock(__RNP_PTP_TX_IN_PROGRESS, &adapter->state)) {

			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
#endif
			tx_flags |= RNP_TXD_FLAG_PTP;
			adapter->ptp_tx_skb = skb_get(skb);
			adapter->tx_hwtstamp_start = jiffies;
			schedule_work(&adapter->tx_hwtstamp_work);
		} else {
			printk("ptp_tx_skb miss\n");
		}
	}
#if 0
	/* DCB maps skb priorities 0-7 onto 3 bit PCP of VLAN tag. */
	if ((adapter->flags & RNP_FLAG_DCB_ENABLED) &&
			((tx_flags & (RNP_TX_FLAGS_HW_VLAN |
				      RNP_TX_FLAGS_SW_VLAN)) ||
			 (skb->priority != TC_PRIO_CONTROL))) {
		tx_flags &= ~RNP_TX_FLAGS_VLAN_PRIO_MASK;
		tx_flags |= (skb->priority & 0x7) <<
			RNP_TX_FLAGS_VLAN_PRIO_SHIFT;
		if (tx_flags & RNP_TX_FLAGS_SW_VLAN) {
			struct vlan_ethhdr *vhdr;

			if (skb_header_cloned(skb) &&
					pskb_expand_head(skb, 0, 0, GFP_ATOMIC))
				goto out_drop;
			vhdr = (struct vlan_ethhdr *)skb->data;
			vhdr->h_vlan_TCI = htons(tx_flags >>
					RNP_TX_FLAGS_VLAN_SHIFT);
		} else {
			tx_flags |= RNP_TX_FLAGS_HW_VLAN;
		}
	}
#endif
	/* record initial flags and protocol */

	tso = rnp_tso(tx_ring, first, &mac_ip_len, &hdr_len, &tx_flags);
	if (tso < 0)
		goto out_drop;
	else if (!tso)
		rnp_tx_csum(tx_ring, first, &mac_ip_len, &tx_flags);
	/* check sriov mode */
	/* in this mode pf send msg should with vf_num */
	if (unlikely(adapter->flags & RNP_FLAG_SRIOV_ENABLED)) {
		first->ctx_flag = true;
		first->mss_len_vf_num |= (adapter->vf_num_for_pf << 16);
	}

	/* add control desc */
	rnp_maybe_tx_ctxtdesc(tx_ring, first, ignore_vlan);
	/* add the ATR filter if ATR is on */
	// if (test_bit(__RNP_TX_FDIR_INIT_DONE, &tx_ring->state))
	//	rnp_atr(tx_ring, first);

	if (rnp_tx_map(tx_ring, first, mac_ip_len, tx_flags)) {
		goto cleanup_tx_tstamp;
	}
#ifndef HAVE_TRANS_START_IN_QUEUE
	tx_ring->netdev->trans_start = jiffies;
#endif
	tx_dbg("=== end ====\n\n\n\n");
	return NETDEV_TX_OK;

out_drop:
	dev_kfree_skb_any(first->skb);
	first->skb = NULL;
cleanup_tx_tstamp:
	if (unlikely(tx_flags & RNP_TXD_FLAG_PTP)) {
		dev_kfree_skb_any(adapter->ptp_tx_skb);
		adapter->ptp_tx_skb = NULL;
		cancel_work_sync(&adapter->tx_hwtstamp_work);
		clear_bit_unlock(__RNP_PTP_TX_IN_PROGRESS, &adapter->state);
	}

	return NETDEV_TX_OK;
}

static netdev_tx_t rnp_xmit_frame(struct sk_buff *skb,
								  struct net_device *netdev)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_ring *tx_ring;

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
	tx_ring = adapter->tx_ring[skb->queue_mapping];

	return rnp_xmit_frame_ring(skb, adapter, tx_ring);
}

/**
 * rnp_set_mac - Change the Ethernet Address of the NIC
 * @netdev: network interface device structure
 * @p: pointer to an address structure
 *
 * Returns 0 on success, negative on failure
 **/
static int rnp_set_mac(struct net_device *netdev, void *p)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw *hw = &adapter->hw;
	struct sockaddr *addr = p;
	bool sriov_flag = !!(adapter->flags & RNP_FLAG_SRIOV_ENABLED);

	dbg("[%s] call set mac\n", netdev->name);

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	eth_hw_addr_set(netdev, addr->sa_data);
	// memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);
	memcpy(hw->mac.addr, addr->sa_data, netdev->addr_len);

	hw->ops.set_mac(hw, hw->mac.addr, sriov_flag);

	// hw->mac.ops.set_rar(hw, 0, hw->mac.addr, VMDQ_P(0), RNP_RAH_AV);
	/* reset veb table */
	rnp_configure_virtualization(adapter);
	return 0;
}

static int
rnp_mdio_read(struct net_device *netdev, int prtad, int devad, u16 addr)
{
	int rc = -EIO;
#if 0
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw *hw = &adapter->hw;
	u16 value;

	if (prtad != hw->phy.mdio.prtad)
		return -EINVAL;
	rc = hw->phy.ops.read_reg(hw, addr, devad, &value);
	if (!rc)
		rc = value;
#endif
	return rc;
}

static int rnp_mdio_write(
	struct net_device *netdev, int prtad, int devad, u16 addr, u16 value)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw *hw = &adapter->hw;

#if 0
	if (prtad != hw->phy.mdio.prtad)
		return -EINVAL;
	return hw->phy.ops.write_reg(hw, addr, devad, value);
#endif
	return -EINVAL;
}

static int rnp_mii_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	struct mii_ioctl_data *mii = (struct mii_ioctl_data *)&ifr->ifr_data;
	int prtad, devad, ret;

	prtad = (mii->phy_id & MDIO_PHY_ID_PRTAD) >> 5;
	devad = (mii->phy_id & MDIO_PHY_ID_DEVAD);

	if (cmd == SIOCGMIIREG) {
		ret = rnp_mdio_read(netdev, prtad, devad, mii->reg_num);
		if (ret < 0)
			return ret;
		mii->val_out = ret;
		return 0;
	} else {
		return rnp_mdio_write(netdev, prtad, devad, mii->reg_num, mii->val_in);
	}
}

static int rnp_ioctl(struct net_device *netdev, struct ifreq *req, int cmd)
{
#ifdef HAVE_PTP_1588_CLOCK
	struct rnp_adapter *adapter = netdev_priv(netdev);
#endif

	/* ptp 1588 used this */
	switch (cmd) {
#ifdef HAVE_PTP_1588_CLOCK
#ifdef SIOCGHWTSTAMP
		case SIOCGHWTSTAMP:
			if (module_enable_ptp)
				return rnp_ptp_get_ts_config(adapter, req);
			break;
#endif
		case SIOCSHWTSTAMP:
			if (module_enable_ptp)
				return rnp_ptp_set_ts_config(adapter, req);
			break;
#endif
		case SIOCGMIIREG:
		/*fall through */
		case SIOCSMIIREG:
			return rnp_mii_ioctl(netdev, req, cmd);
			break;
	}
	return -EINVAL;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 * Polling 'interrupt' - used by things like netconsole to send skbs
 * without having to re-enable interrupts. It's not called while
 * the interrupt routine is executing.
 */
static void rnp_netpoll(struct net_device *netdev)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	int i;

	/* if interface is down do nothing */
	if (test_bit(__RNP_DOWN, &adapter->state))
		return;

	adapter->flags |= RNP_FLAG_IN_NETPOLL;
	for (i = 0; i < adapter->num_q_vectors; i++)
		rnp_msix_clean_rings(0, adapter->q_vector[i]);
	adapter->flags &= ~RNP_FLAG_IN_NETPOLL;
}

#endif

#ifdef HAVE_NDO_GET_STATS64

#ifdef HAVE_VOID_NDO_GET_STATS64
static void rnp_get_stats64(struct net_device *netdev,
							struct rtnl_link_stats64 *stats)
#else
static struct rtnl_link_stats64 *
rnp_get_stats64(struct net_device *netdev, struct rtnl_link_stats64 *stats)
#endif
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	int i;

	rcu_read_lock();
	for (i = 0; i < adapter->num_rx_queues; i++) {
		struct rnp_ring *ring = READ_ONCE(adapter->rx_ring[i]);
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
		struct rnp_ring *ring = READ_ONCE(adapter->tx_ring[i]);
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
	/* following stats updated by rnp_watchdog_task() */
	stats->multicast = netdev->stats.multicast;
	stats->rx_errors = netdev->stats.rx_errors;
	stats->rx_length_errors = netdev->stats.rx_length_errors;
	stats->rx_crc_errors = netdev->stats.rx_crc_errors;
	stats->rx_missed_errors = netdev->stats.rx_missed_errors;

#ifndef HAVE_VOID_NDO_GET_STATS64
	return stats;
#endif
}
#else
/**
 * rnp_get_stats - Get System Network Statistics
 * @netdev: network interface device structure
 *
 * Returns the address of the device statistics structure.
 * The statistics are actually updated from the timer callback.
 **/
static struct net_device_stats *rnp_get_stats(struct net_device *netdev)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);

	/* update the stats data */
	rnp_update_stats(adapter);

#ifdef HAVE_NETDEV_STATS_IN_NETDEV
	/* only return the current stats */
	return &netdev->stats;
#else
	/* only return the current stats */
	return &adapter->net_stats;
#endif /* HAVE_NETDEV_STATS_IN_NETDEV */
}

#endif
#ifdef CONFIG_RNP_DCB
/**
 * rnp_validate_rtr - verify 802.1Qp to Rx packet buffer mapping is valid.
 * @adapter: pointer to rnp_adapter
 * @tc: number of traffic classes currently enabled
 *
 * Configure a valid 802.1Qp to Rx packet buffer mapping ie confirm
 * 802.1Q priority maps to a packet buffer that exists.
 */
static void rnp_validate_rtr(struct rnp_adapter *adapter, u8 tc)
{
	struct rnp_hw *hw = &adapter->hw;
	u32 reg, rsave;
	int i;
#if 0
	/* 82598 have a static priority to TC mapping that can not
	 * be changed so no validation is needed.
	 */
	if (hw->mac.type == rnp_mac_82598EB)
		return;

	reg = RNP_READ_REG(hw, RNP_RTRUP2TC);
	rsave = reg;

	for (i = 0; i < MAX_TRAFFIC_CLASS; i++) {
		u8 up2tc = reg >> (i * RNP_RTRUP2TC_UP_SHIFT);

		/* If up2tc is out of bounds default to zero */
		if (up2tc > tc)
			reg &= ~(0x7 << RNP_RTRUP2TC_UP_SHIFT);
	}

	if (reg != rsave)
		RNP_WRITE_REG(hw, RNP_RTRUP2TC, reg);
#endif
}

/**
 * rnp_set_prio_tc_map - Configure netdev prio tc map
 * @adapter: Pointer to adapter struct
 *
 * Populate the netdev user priority to tc map
 */
static void rnp_set_prio_tc_map(struct rnp_adapter *adapter)
{
	struct net_device *dev = adapter->netdev;

	// todo
#if 0
	struct rnp_dcb_config *dcb_cfg = &adapter->dcb_cfg;
	struct ieee_ets *ets = adapter->rnp_ieee_ets;
	u8 prio;

	for (prio = 0; prio < MAX_USER_PRIORITY; prio++) {
		u8 tc = 0;

		if (adapter->dcbx_cap & DCB_CAP_DCBX_VER_CEE)
			tc = rnp_dcb_get_tc_from_up(dcb_cfg, 0, prio);
		else if (ets)
			tc = ets->prio_tc[prio];

		netdev_set_prio_tc_map(dev, prio, tc);
	}
#endif
}

#endif /* CONFIG_RNP_DCB */
/**
 * rnp_setup_tc - configure net_device for multiple traffic classes
 *
 * @netdev: net device to configure
 * @tc: number of traffic classes to enable
 */
int rnp_setup_tc(struct net_device *dev, u8 tc)
{
	struct rnp_adapter *adapter = netdev_priv(dev);
	struct rnp_hw *hw = &adapter->hw;
	int ret = 0;
	/* Hardware supports up to 8 traffic classes */
	if ((tc > RNP_MAX_TCS_NUM) || (tc == 1))
		return -EINVAL;
	/* we canot support tc with sriov mode */
	if ((tc) && (adapter->flags & RNP_FLAG_SRIOV_ENABLED))
		return -EINVAL;

	/* Hardware has to reinitialize queues and interrupts to
	 * match packet buffer alignment. Unfortunately, the
	 * hardware is not flexible enough to do this dynamically.
	 */
	while (test_and_set_bit(__RNP_RESETTING, &adapter->state))
		usleep_range(1000, 2000);

	if (netif_running(dev))
		rnp_close(dev);

	rnp_fdir_filter_exit(adapter);

	remove_mbx_irq(adapter);
	rnp_clear_interrupt_scheme(adapter);

	adapter->num_tc = tc;

	//#ifdef CONFIG_RNP_DCB
	if (tc) {

		netdev_set_num_tc(dev, tc);

		// todo
		// rnp_set_prio_tc_map(adapter);

		adapter->flags |= RNP_FLAG_DCB_ENABLED;

		/*if (adapter->hw.mac.type == rnp_mac_82598EB) {
			adapter->last_lfc_mode = adapter->hw.fc.requested_mode;
			adapter->hw.fc.requested_mode = rnp_fc_none;
		}*/
	} else {
		netdev_reset_tc(dev);

		/*if (adapter->hw.mac.type == rnp_mac_82598EB)
			adapter->hw.fc.requested_mode = adapter->last_lfc_mode;
			*/

		adapter->flags &= ~RNP_FLAG_DCB_ENABLED;

		//	adapter->temp_dcb_cfg.pfc_mode_enable = false;
		//	adapter->dcb_cfg.pfc_mode_enable = false;
	}

	// todo
	// rnp_validate_rtr(adapter, tc);

	//#endif /* CONFIG_RNP_DCB */
	rnp_init_interrupt_scheme(adapter);

	register_mbx_irq(adapter);
	/* rss table must reset */
	adapter->rss_tbl_setup_flag = 0;

	if (netif_running(dev))
		ret = rnp_open(dev);
	// return rnp_open(dev);

	clear_bit(__RNP_RESETTING, &adapter->state);
	return ret;
}

#ifdef CONFIG_PCI_IOV
void rnp_sriov_reinit(struct rnp_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;

	rtnl_lock();
	rnp_setup_tc(netdev, netdev_get_num_tc(netdev));
	rtnl_unlock();

	// wait link status
	usleep_range(10000, 20000);
}
#endif

#ifdef NETIF_F_HW_TC
#ifdef HAVE_TC_SETUP_CLSU32
int rnp_delete_knode(struct net_device *dev, struct tc_cls_u32_offload *cls)
{
	/* 1. check weather filter rule is ingress root */
	struct rnp_adapter *adapter = netdev_priv(dev);
	u32 loc = cls->knode.handle & 0xfffff;
	u32 uhtid = TC_U32_USERHTID(cls->knode.handle);
	int ret;

	if ((uhtid != 0x800))
		return -EINVAL;

	spin_lock(&adapter->fdir_perfect_lock);
	ret = rnp_update_ethtool_fdir_entry(adapter, NULL, loc);
	spin_unlock(&adapter->fdir_perfect_lock);

	return ret;
}
#ifdef CONFIG_NET_CLS_ACT

static int rnp_action_parse(struct tcf_exts *exts, u64 *action, u8 *queue)
{
	const struct tc_action *a;
#if defined(HAVE_TCF_EXTS_TO_LIST)
	LIST_HEAD(actions);
#elif defined(HAVE_TCF_EXTS_FOR_EACH_ACTION)
	int j;
#endif

#ifdef HAVE_TCF_EXTS_HAS_ACTION
	if (!tcf_exts_has_actions(exts))

#else
	if (tc_no_actions(exts))
#endif
		return -EINVAL;
#if defined(HAVE_TCF_EXTS_TO_LIST)
	tcf_exts_to_list(exts, &actions);
	list_for_each_entry(a, &actions, list)
	{
#elif defined(HAVE_TCF_EXTS_FOR_EACH_ACTION)
	tcf_exts_for_each_action(j, a, exts)
	{
#else
	tc_for_each_action(a, exts)
	{
#endif
		/* Drop action */
		if (is_tcf_gact_shot(a)) {
			*action = RNP_FDIR_DROP_QUEUE;
			*queue = RNP_FDIR_DROP_QUEUE;
			return 0;
		}
#ifdef HAVE_TCF_MIRRED_REDIRECT
		/* Redirect to a VF or a offloaded macvlan */
#ifdef HAVE_TCF_MIRRED_EGRESS_REDIRECT
		if (is_tcf_mirred_egress_redirect(a)) {
#else

		if (is_tcf_mirred_redirect(a)) {
#endif

#ifdef HAVE_TCF_MIRRED_DEV
			struct net_device *dev = tcf_mirred_dev(a);

			if (!dev)
				return -EINVAL;
#else
			int ifindex = tcf_mirred_ifindex(a);
#endif /* HAVE_TCF_MIRRED_DEV */
		}
#endif /* HAVE_TCF_MIRRED_REDIRECT */

		return -EINVAL;
	}

	return 0;
}
#else
static int rnp_action_parse(struct tcf_exts *exts, u64 *action, u8 *queue)
{
	return -EINVAL;
}

#endif

static int rnp_clsu32_build_input(struct tc_cls_u32_offload *cls,
								  struct rnp_fdir_filter *input,
								  const struct rnp_match_parser *parsers)
{
	int i = 0, j = 0, err = -1;
	__be32 val, mask, off;
	bool found;

	for (i = 0; i < cls->knode.sel->nkeys; i++) {
		off = cls->knode.sel->keys[i].off;
		val = cls->knode.sel->keys[i].val;
		mask = cls->knode.sel->keys[i].mask;
		dbg("cls-key[%d] off %d val %d mask %d\n ", i, off, val, mask);
		found = false;
		for (j = 0; parsers[j].val; j++) {
			/* according the off select parser */
			if (off == parsers[j].off) {
				found = true;
				err = parsers[j].val(input, val, mask);
				if (err)
					return err;

				break;
			}
		}
		/* if the rule can't parse that we don't support the rule */
		if (!found)
			return -EINVAL;
	}

	return 0;
}

int rnp_config_knode(struct net_device *dev,
					 __be16 protocol,
					 struct tc_cls_u32_offload *cls)
{
	/*1. check ethernet hw-feature U32 can offload */
	/*2. check U32 protocol We just support IPV4 offloading For now*/
	/*3. check if this cls is a cls of root u32 or cls of class u32*/
	/*4. check if this cls has been added.
	 * the filter extry create but the match val and mask don't fill
	 * so we can use it.
	 * find a exist extry and the match val and mask is added before
	 * so we don't need add it again
	 */
	u32 uhtid, link_uhtid;
	int ret;
	struct rnp_adapter *adapter = netdev_priv(dev);
	u8 queue;
	struct rnp_fdir_filter *input;
	struct rnp_hw *hw = &adapter->hw;
	u32 loc = cls->knode.handle & 0xfffff;

	if (protocol != htons(ETH_P_IP))
		return -EOPNOTSUPP;

	uhtid = TC_U32_USERHTID(cls->knode.handle);
	link_uhtid = TC_U32_USERHTID(cls->knode.link_handle);

	netdev_info(dev,
				"uhtid %d link_uhtid %d protocol 0x%2x\n",
				uhtid,
				link_uhtid,
				ntohs(protocol));
	/* For now just support handle root ingress
	 * TODO more feature
	 */
	if (uhtid != 0x800)
		return -EINVAL;

	input = kzalloc(sizeof(*input), GFP_KERNEL);
	/*be carefull this input mem need to free */
	ret = rnp_clsu32_build_input(cls, input, rnp_ipv4_parser);
	if (ret) {
		netdev_warn(dev, "This Rules We Can't Support It\n");
		goto out;
	}
	ret = rnp_action_parse(cls->knode.exts, &input->action, &queue);
	if (ret)
		goto out;

	dbg("tc filter rule sw_location %d\n", loc);

	/* maybe bug here */
	input->hw_idx = adapter->tuple_5_count++;
	input->sw_idx = loc;
	spin_lock(&adapter->fdir_perfect_lock);

	/*ret = rnp_fdir_write_perfect_filter(
		adapter->fdir_mode, hw, &input->filter, input->hw_idx,
		(input->action == RNP_FDIR_DROP_QUEUE) ?
			RNP_FDIR_DROP_QUEUE :
			adapter->rx_ring[input->action]->rnp_queue_idx);
	if (ret)
		goto err_out_w_lock;
	*/
	rnp_update_ethtool_fdir_entry(adapter, input, input->sw_idx);
	spin_unlock(&adapter->fdir_perfect_lock);

	return 0;
err_out_w_lock:
	spin_unlock(&adapter->fdir_perfect_lock);
out:
	kfree(input);
	return -EOPNOTSUPP;
}

#ifdef HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV
int rnp_setup_tc_cls_u32(struct net_device *dev,
						 struct tc_cls_u32_offload *cls_u32)
{
	__be16 proto = cls_u32->common.protocol;
#else
int rnp_setup_tc_cls_u32(struct net_device *dev,
						 __be16 proto,
						 struct tc_cls_u32_offload *cls_u32)
{
#endif
#if 0
	if (!is_classid_clsact_ingress(cls_u32->common.classid) ||
			cls_u32->common.chain_index)
		return -EOPNOTSUPP;
#endif
	dbg("cls_u32->command is %d\n", cls_u32->command);
	switch (cls_u32->command) {
		case TC_CLSU32_NEW_KNODE:
		case TC_CLSU32_REPLACE_KNODE:
			return rnp_config_knode(dev, proto, cls_u32);
		case TC_CLSU32_DELETE_KNODE:
			return rnp_delete_knode(dev, cls_u32);
		default:
			return -EOPNOTSUPP;
	}
}

#endif /* HAVE_TC_SETUP_CLSU32 */

#ifdef HAVE_TCF_BLOCK
static int rnp_setup_tc_block_ingress_cb(enum tc_setup_type type,
										 void *type_data,
										 void *cb_priv)
{
	struct net_device *dev = cb_priv;
	struct rnp_adapter *adapter = netdev_priv(dev);

	if (test_bit(__RNP_DOWN, &adapter->state)) {
		netdev_err(adapter->netdev,
				   "Failed to setup tc on port %d. Link Down? 0x%.2lx\n",
				   adapter->port,
				   adapter->state);
		return -EINVAL;
	}
	if (!tc_cls_can_offload_and_chain0(dev, type_data))
		return -EOPNOTSUPP;

	switch (type) {
#ifdef HAVE_TC_SETUP_CLSU32
		case TC_SETUP_CLSU32:
			return rnp_setup_tc_cls_u32(dev, type_data);
#endif /* HAVE_TC_SETUP_CLSU32 */
		default:
			return -EOPNOTSUPP;
	}
}

static LIST_HEAD(rnp_block_cb_list);

#endif /*  HAVE_TCF_BLOCK */

#ifdef TC_MQPRIO_HW_OFFLOAD_MAX

static void rnp_setup_txr_prio(struct rnp_ring *tx_ring, int prio)
{
	ring_wr32(tx_ring, RNP_DMA_REG_TX_ARB_DEF_LVL, prio);
}
int rnp_setup_mqprio(struct net_device *netdev, void *type_data)
{
	struct tc_mqprio_qopt *mqprio = type_data;
	// struct tc_mqprio_qopt_offload *mqprio_qopt = type_data;

	rnp_dbg("call rnp_setup mqprio %d\n", mqprio->num_tc);
	mqprio->hw = TC_MQPRIO_HW_OFFLOAD_TCS;
	return rnp_setup_tc(netdev, mqprio->num_tc);

	//#ifdef __TC_MQPRIO_MODE_MAX
	//	struct tc_mqprio_qopt_offload *mqprio_qopt = type_data;
	//	struct rnp_adapter *adapter = netdev_priv(netdev);
	//	struct rnp_ring *tx_ring = NULL;
	//	u8 num_tc, hw;
	//	u16 mode = 0;
	//	int i = 0;
	//	u64 max_tx_rate = 0;
	//
	//	num_tc = mqprio_qopt->qopt.num_tc;
	//
	//
	//	hw = mqprio_qopt->qopt.hw;
	//	mode = mqprio_qopt->mode;
	//	netdev_info(netdev, "setup mqprio hw %d mode %d num_tx %d\n", hw, mode,
	//		    num_tc);
	//
	//	/* should check num_tc smaller than the adapter->num_tx_queues ? */
	//	if (!num_tc) {
	//		/* reset mqprio tc */
	//		netdev_reset_tc(netdev);
	//		netif_set_real_num_tx_queues(netdev, adapter->num_tx_queues);
	//		for (i = 0; i < adapter->num_tx_queues; i++) {
	//			/* reset all tx ring prio to zero */
	//			tx_ring = adapter->tx_ring[i];
	//			rnp_setup_txr_prio(tx_ring, 0);
	//			rnp_setup_tx_maxrate(tx_ring, 0,
	//					adapter->hw.usecstocount * 1000000);
	//		}
	//		return 0;
	//	}
	//	if (num_tc > RNP_MAX_TCS_NUM) {
	//		netdev_err(netdev, "Max %d traffic classes supported\n",
	//			   adapter->num_tx_queues);
	//		return -EINVAL;
	//	}
	//
	//	/* Reset the number of netdev queues based on the TC count */
	//	adapter->num_tc = num_tc;
	//	netdev_set_num_tc(netdev, num_tc);
	//
	//	/* For now we just set the prio for tx ring_index increase <=> prio */
	//	for (i = 0; i < RNP_MAX_USER_PRIO * 2; i++) {
	//		adapter->prio_tc_map[i] = mqprio_qopt->qopt.prio_tc_map[i];
	//		printk("%d prio_tc_map is %x\n", i, adapter->prio_tc_map[i]);
	//	}
	//	for (i = 0; i < RNP_MAX_USER_PRIO * 2; i++) {
	//		printk("netdev %d prio_tc_map is %x\n", i, netdev->prio_tc_map[i]);
	//	}
	//
	//	for (i = 0; i < num_tc; i++) {
	//		tx_ring = adapter->tx_ring[i];
	//		rnp_setup_txr_prio(tx_ring, i);
	//		max_tx_rate = mqprio_qopt->max_rate[i];
	//		netdev_info(netdev, "setup tc[%d] max_rate %llu\n", i,
	//			    max_tx_rate);
	//		if (max_tx_rate)
	//			rnp_setup_tx_maxrate(tx_ring, max_tx_rate,
	//					adapter->hw.usecstocount * 1000000);
	//	}
	//
	//	for (i = 0; i < num_tc; i++) {
	//		netdev_set_tc_queue(netdev, i, mqprio_qopt->qopt.count[i],
	//				    mqprio_qopt->qopt.offset[i]);
	//		printk(" %d count %d offset %d\n", i, mqprio_qopt->qopt.count[i],
	//				mqprio_qopt->qopt.offset[i]);
	//	}
	//
	//	return 0;
	//#else
	//	return -EOPNOTSUPP;
	//#endif
}
#endif /* TC_MQPRIO_HW_OFFLOAD_MAX */

#ifdef HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV
static int __rnp_setup_tc(struct net_device *netdev,
						  enum tc_setup_type type,
						  void *type_data)
#elif defined(HAVE_NDO_SETUP_TC_CHAIN_INDEX)
static int __rnp_setup_tc(struct net_device *netdev,
						  u32 handle,
						  u32 chain_index,
						  __be16 proto,
						  struct tc_to_netdev *tc)
#else
static int __rnp_setup_tc(struct net_device *netdev,
						  u32 handle,
						  __be16 proto,
						  struct tc_to_netdev *tc)
#endif /* HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV */
{

	struct rnp_adapter *adapter = netdev_priv(netdev);
#ifndef HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV
	unsigned int type = tc->type;
#ifdef HAVE_NDO_SETUP_TC_CHAIN_INDEX
	if (chain_index) {
		dbg("chain_index %d\n", chain_index);
		return -EOPNOTSUPP;
	}
#endif /* HAVE_NDO_SETUP_TC_CHAIN_INDEX */
	netdev_info(netdev,
				" TC_H_MAJ %x H_MAJ  %x\n",
				TC_H_MAJ(handle),
				TC_H_MAJ(TC_H_INGRESS));
#ifdef HAVE_TC_SETUP_CLSU32
	if (TC_H_MAJ(handle) == TC_H_MAJ(TC_H_INGRESS) && type == TC_SETUP_CLSU32) {
		netdev_info(netdev, "setup_tc type is %d\n", type);
		return rnp_setup_tc_cls_u32(netdev, proto, tc->cls_u32);
	}
#endif /* HAVE_TC_SETUP_CLSU32 */
#endif /* !HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV */
	switch (type) {
#ifdef HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV
#ifdef HAVE_TCF_BLOCK
		case TC_SETUP_BLOCK: {
			struct flow_block_offload *f =
				(struct flow_block_offload *)type_data;
			if (f->binder_type == FLOW_BLOCK_BINDER_TYPE_CLSACT_INGRESS)
				return flow_block_cb_setup_simple(type_data,
												  &rnp_block_cb_list,
												  rnp_setup_tc_block_ingress_cb,
												  adapter,
												  adapter,
												  true);
			else
				return -EOPNOTSUPP;
		}
#else
#ifdef HAVE_TC_SETUP_CLSU32
		case TC_SETUP_CLSU32:
			return rnp_setup_tc_cls_u32(netdev, type_data);
#endif /* HAVE_TC_SETUP_CLSU32 */
#endif /* HAVE_TCF_BLOCK */
#endif /* HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV */
		case TC_SETUP_QDISC_MQPRIO:
#if defined(HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV)
			return rnp_setup_mqprio(netdev, type_data);
#endif
		default:
			return -EOPNOTSUPP;
	}

	return 0;
}
#endif /*  NETIF_F_HW_TC */
void rnp_do_reset(struct net_device *netdev)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);

	if (netif_running(netdev))
		rnp_reinit_locked(adapter);
	else
		rnp_reset(adapter);
}

#ifdef HAVE_NDO_SET_FEATURES
#ifdef HAVE_RHEL6_NET_DEVICE_OPS_EXT
static u32 rnp_fix_features(struct net_device *netdev, u32 features)
#else
static netdev_features_t rnp_fix_features(struct net_device *netdev,
										  netdev_features_t features)
#endif
{
	struct rnp_adapter *adapter = netdev_priv(netdev);

	/* If Rx checksum is disabled, then RSC/LRO should also be disabled */
	if (!(features & NETIF_F_RXCSUM))
		features &= ~NETIF_F_LRO;

	/* close rx csum when rx fcs on */
	// maybe fixed in n500 ?
	if (features & NETIF_F_RXFCS)
		features &= (~NETIF_F_RXCSUM);
	/* Turn off LRO if not RSC capable */
	if (!(adapter->flags2 & RNP_FLAG2_RSC_CAPABLE))
		features &= ~NETIF_F_LRO;

	return features;
}

#ifdef HAVE_RHEL6_NET_DEVICE_OPS_EXT
static int rnp_set_features(struct net_device *netdev, u32 features)
#else
static int rnp_set_features(struct net_device *netdev,
							netdev_features_t features)
#endif
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	netdev_features_t changed = netdev->features ^ features;
	bool need_reset = false;
	struct rnp_hw *hw = &adapter->hw;
	int i;

#if 0
	/* Make sure RSC matches LRO, reset if change */
	/* we don't support rsc */
	if (!(features & NETIF_F_LRO)) {
		if (adapter->flags2 & RNP_FLAG2_RSC_ENABLED)
			need_reset = true;
		adapter->flags2 &= ~RNP_FLAG2_RSC_ENABLED;
	} else if ((adapter->flags2 & RNP_FLAG2_RSC_CAPABLE) &&
			!(adapter->flags2 & RNP_FLAG2_RSC_ENABLED)) {
		if (adapter->rx_itr_setting == 1 ||
				adapter->rx_itr_setting > RNP_MIN_RSC_ITR) {
			adapter->flags2 |= RNP_FLAG2_RSC_ENABLED;
			need_reset = true;
		} else if ((changed ^ features) & NETIF_F_LRO) {
			e_info(probe, "rx-usecs set too low, "
					"disabling RSC\n");
		}
	}
#endif
	netdev->features = features;

	// if changed ntuple should close all ?
	if (changed & NETIF_F_NTUPLE) {
		if (!(features & NETIF_F_NTUPLE)) {
			rnp_fdir_filter_exit(adapter);
		}
	}

	switch (features & NETIF_F_NTUPLE) {
		case NETIF_F_NTUPLE:
			/* turn off ATR, enable perfect filters and reset */
			if (!(adapter->flags & RNP_FLAG_FDIR_PERFECT_CAPABLE))
				need_reset = true;

			adapter->flags &= ~RNP_FLAG_FDIR_HASH_CAPABLE;
			adapter->flags |= RNP_FLAG_FDIR_PERFECT_CAPABLE;
			break;
		default:
			/* turn off perfect filters, enable ATR and reset */
			if (adapter->flags & RNP_FLAG_FDIR_PERFECT_CAPABLE)
				need_reset = true;

			adapter->flags &= ~RNP_FLAG_FDIR_PERFECT_CAPABLE;

			/* We cannot enable ATR if SR-IOV is enabled */
			if (adapter->flags & RNP_FLAG_SRIOV_ENABLED)
				break;

			/* We cannot enable ATR if we have 2 or more traffic classes */
			if (netdev_get_num_tc(netdev) > 1)
				break;

			/* We cannot enable ATR if RSS is disabled */
			// if (adapter->ring_feature[RING_F_RSS].limit <= 1)
			//    break;

			/* A sample rate of 0 indicates ATR disabled */
			if (!adapter->atr_sample_rate)
				break;

			adapter->flags |= RNP_FLAG_FDIR_HASH_CAPABLE;
			break;
	}

#ifdef NETIF_F_HW_VLAN_CTAG_FILTER
	/* vlan filter changed */
	if (changed & NETIF_F_HW_VLAN_CTAG_FILTER) {
		if (features & (NETIF_F_HW_VLAN_CTAG_FILTER)) {
			// rnp_vlan_filter_on(hw);
			hw->ops.set_vlan_filter_en(hw, true);
		} else {
			// rnp_vlan_filter_off(hw);
			hw->ops.set_vlan_filter_en(hw, false);
		}
		rnp_msg_post_status(adapter, PF_VLAN_FILTER_STATUS);
	}
#endif /* NETIF_F_HW_VLAN_CTAG_FILTER */

	/* rss hash changed */
	if (changed & (NETIF_F_RXHASH)) {
		// u32 iov_en = (adapter->flags & RNP_FLAG_SRIOV_ENABLED) ?
		//		     RNP_IOV_ENABLED :
		//		     0;
		bool iov_en = (adapter->flags & RNP_FLAG_SRIOV_ENABLED) ? true : false;

		if (netdev->features & (NETIF_F_RXHASH)) {
			hw->ops.set_rx_hash(hw, true, iov_en);
			// wr32(hw, RNP_ETH_RSS_CONTROL,
			//     RNP_ETH_ENABLE_RSS_ONLY | iov_en);
		} else {
			hw->ops.set_rx_hash(hw, false, iov_en);
			// wr32(hw, RNP_ETH_RSS_CONTROL,
			//     RNP_ETH_DISABLE_RSS | iov_en);
		}
	}

	/* rx fcs changed */
	/* in this mode rx l4/sctp checksum will get error */
	if (changed & NETIF_F_RXFCS) {
		// uint32_t rx_cfg = rd32(hw, RNP_MAC_RX_CFG);

		if (features & NETIF_F_RXFCS) {
			hw->ops.set_fcs_mode(hw, true);
			// wr32(hw, RNP_MAC_RX_CFG, 0x07d001c1);
			/* if in rx fcs mode ,hw rxcsum may error,
			 * close rxcusm
			 */
		} else {
			hw->ops.set_fcs_mode(hw, false);
			// wr32(hw, RNP_MAC_RX_CFG, 0x07d001c7);
		}
		rnp_msg_post_status(adapter, PF_FCS_STATUS);
	}

	if (changed & NETIF_F_RXALL)
		need_reset = true;
#ifdef NETIF_F_HW_VLAN_CTAG_RX
	if (features & NETIF_F_HW_VLAN_CTAG_RX)
		rnp_vlan_strip_enable(adapter);
	else
		rnp_vlan_strip_disable(adapter);
#endif

	if (need_reset)
		rnp_do_reset(netdev);

	return 0;
}
#endif /* HAVE_NDO_SET_FEATURES */

//#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 1, 0))
// static int rnp_ndo_fdb_add(struct ndmsg *ndm, struct nlattr *tb[],
//			   struct net_device *dev, const unsigned char *addr,
//			   u16 vid, u16 flags, struct netlink_ext_ack *extack)
//#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0))
// static int rnp_ndo_fdb_add(struct ndmsg *ndm, struct nlattr *tb[],
//			   struct net_device *dev, const unsigned char *addr,
//			   u16 flags)
//#else
// static int rnp_ndo_fdb_add(struct ndmsg *ndm, struct net_device *dev,
//			   unsigned char *addr, u16 flags)
//#endif
//{
//	struct rnp_adapter *adapter = netdev_priv(dev);
//	int err = 0;
//
//
//	if (!(adapter->flags & RNP_FLAG_SRIOV_ENABLED))
//#if defined(RHEL_RELEASE_CODE)
//		return ndo_dflt_fdb_add(ndm, tb, dev, addr, 0, flags);
//#else
//		return ndo_dflt_fdb_add(ndm, tb, dev, addr, flags);
//#endif
//
//
//	/* Hardware does not support aging addresses so if a
//	 * ndm_state is given only allow permanent addresses
//	 */
//	if (ndm->ndm_state && !(ndm->ndm_state & NUD_PERMANENT)) {
//		pr_info("%s: FDB only supports static addresses\n",
//			rnp_driver_name);
//		return -EINVAL;
//	}
//
//	if (is_unicast_ether_addr(addr) || is_link_local_ether_addr(addr)) {
//		u32 rar_uc_entries = RNP_MAX_PF_MACVLANS;
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
//#define HAVE_NDO_BRIDGE_SET_DEL_LINK_FLAGS
#ifdef HAVE_BRIDGE_ATTRIBS
#ifdef HAVE_NDO_BRIDGE_SETLINK_EXTACK
static int rnp_ndo_bridge_setlink(struct net_device *dev,
								  struct nlmsghdr *nlh,
								  __always_unused u16 flags,
								  struct netlink_ext_ack __always_unused *ext)
#elif defined(HAVE_NDO_BRIDGE_SET_DEL_LINK_FLAGS)
static int rnp_ndo_bridge_setlink(struct net_device *dev,
								  struct nlmsghdr *nlh,
								  __always_unused u16 flags)
#else
static int rnp_ndo_bridge_setlink(struct net_device *dev, struct nlmsghdr *nlh)
#endif /* HAVE_NDO_BRIDGE_SETLINK_EXTACK */

{
	struct rnp_adapter *adapter = netdev_priv(dev);
	struct rnp_hw *hw = &adapter->hw;
	struct nlattr *attr, *br_spec;
	int rem;

	if (!(adapter->flags & RNP_FLAG_SRIOV_ENABLED))
		return -EOPNOTSUPP;

	br_spec = nlmsg_find_attr(nlh, sizeof(struct ifinfomsg), IFLA_AF_SPEC);

	nla_for_each_nested(attr, br_spec, rem)
	{
		__u16 mode;
		u32 reg = 0;

		if (nla_type(attr) != IFLA_BRIDGE_MODE)
			continue;

		mode = nla_get_u16(attr);
		if (mode == BRIDGE_MODE_VEPA) {
			adapter->flags2 &= ~RNP_FLAG2_BRIDGE_MODE_VEB;
			wr32(hw, RNP_DMA_CONFIG, rd32(hw, RNP_DMA_CONFIG) | DMA_VEB_BYPASS);
		} else if (mode == BRIDGE_MODE_VEB) {
			adapter->flags2 |= RNP_FLAG2_BRIDGE_MODE_VEB;
			wr32(hw,
				 RNP_DMA_CONFIG,
				 rd32(hw, RNP_DMA_CONFIG) & (~DMA_VEB_BYPASS));

		} else
			return -EINVAL;

		e_info(drv,
			   "enabling bridge mode: %s\n",
			   mode == BRIDGE_MODE_VEPA ? "VEPA" : "VEB");
	}

	return 0;
}

#ifdef HAVE_NDO_BRIDGE_GETLINK_NLFLAGS
static int rnp_ndo_bridge_getlink(struct sk_buff *skb,
								  u32 pid,
								  u32 seq,
								  struct net_device *dev,
								  u32 __maybe_unused filter_mask,
								  int nlflags)
#elif defined(HAVE_BRIDGE_FILTER)
static int rnp_ndo_bridge_getlink(struct sk_buff *skb,
								  u32 pid,
								  u32 seq,
								  struct net_device *dev,
								  u32 __always_unused filter_mask)
#else
static int rnp_ndo_bridge_getlink(struct sk_buff *skb,
								  u32 pid,
								  u32 seq,
								  struct net_device *dev)
#endif /* HAVE_NDO_BRIDGE_GETLINK_NLFLAGS */
{
	struct rnp_adapter *adapter = netdev_priv(dev);
	u16 mode;

	if (!(adapter->flags & RNP_FLAG_SRIOV_ENABLED))
		return 0;

	if (adapter->flags2 & RNP_FLAG2_BRIDGE_MODE_VEB)
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

#ifdef HAVE_NDO_FEATURES_CHECK
#define RNP_MAX_TUNNEL_HDR_LEN 80
#ifdef NETIF_F_GSO_PARTIAL
#define RNP_MAX_MAC_HDR_LEN		127
#define RNP_MAX_NETWORK_HDR_LEN 511

static netdev_features_t rnp_features_check(struct sk_buff *skb,
											struct net_device *dev,
											netdev_features_t features)
{
	unsigned int network_hdr_len, mac_hdr_len;

	/* Make certain the headers can be described by a context descriptor */
	mac_hdr_len = skb_network_header(skb) - skb->data;
	if (unlikely(mac_hdr_len > RNP_MAX_MAC_HDR_LEN))
		return features &
			   ~(NETIF_F_HW_CSUM | NETIF_F_SCTP_CRC | NETIF_F_HW_VLAN_CTAG_TX |
				 NETIF_F_TSO | NETIF_F_TSO6);

	network_hdr_len = skb_checksum_start(skb) - skb_network_header(skb);
	if (unlikely(network_hdr_len > RNP_MAX_NETWORK_HDR_LEN))
		return features & ~(NETIF_F_HW_CSUM | NETIF_F_SCTP_CRC | NETIF_F_TSO |
							NETIF_F_TSO6);

	/* We can only support IPV4 TSO in tunnels if we can mangle the
	 * inner IP ID field, so strip TSO if MANGLEID is not supported.
	 */
	if (skb->encapsulation && !(features & NETIF_F_TSO_MANGLEID))
		features &= ~NETIF_F_TSO;

	return features;
}
#else
static netdev_features_t rnp_features_check(struct sk_buff *skb,
											struct net_device *dev,
											netdev_features_t features)
{
	if (!skb->encapsulation)
		return features;

	if (unlikely(skb_inner_mac_header(skb) - skb_transport_header(skb) >
				 RNP_MAX_TUNNEL_HDR_LEN))
		return features & ~NETIF_F_CSUM_MASK;

	return features;
}

#endif /* NETIF_F_GSO_PARTIAL */
#endif /* HAVE_NDO_FEATURES_CHECK */

#if defined(HAVE_UDP_ENC_RX_OFFLOAD) || defined(HAVE_VXLAN_RX_OFFLOAD)
void rnp_clear_udp_tunnel_port(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	u32 vxlanctrl;

	if (!(adapter->flags & (RNP_FLAG_VXLAN_OFFLOAD_CAPABLE)))
		return;

	// wr32(hw, RNP_ETH_VXLAN_PORT, 0);
	adapter->vxlan_port = 0;
	hw->ops.set_vxlan_port(hw, adapter->vxlan_port);
}
#endif /* HAVE_UDP_ENC_RX_OFFLOAD || HAVE_VXLAN_RX_OFFLOAD */

#ifdef HAVE_UDP_ENC_RX_OFFLOAD
/**
 * rnp_add_udp_tunnel_port - Get notifications about adding UDP tunnel ports
 * @dev: The port's netdev
 * @ti: Tunnel endpoint information
 **/
static void rnp_add_udp_tunnel_port(struct net_device *dev,
									struct udp_tunnel_info *ti)
{
	struct rnp_adapter *adapter = netdev_priv(dev);
	struct rnp_hw *hw = &adapter->hw;
	__be16 port = ti->port;
	// u32 port_shift = 0;
	// u32 reg;

	if (ti->sa_family != AF_INET)
		return;

	switch (ti->type) {
		case UDP_TUNNEL_TYPE_VXLAN:
			if (!(adapter->flags & RNP_FLAG_VXLAN_OFFLOAD_CAPABLE))
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
		default:
			return;
	}
	hw->ops.set_vxlan_port(hw, ntohs(adapter->vxlan_port));
	// wr32(hw, RNP_ETH_VXLAN_PORT, adapter->vxlan_port);
}

/**
 * rnp_del_udp_tunnel_port - Get notifications about removing UDP tunnel ports
 * @dev: The port's netdev
 * @ti: Tunnel endpoint information
 **/
static void rnp_del_udp_tunnel_port(struct net_device *dev,
									struct udp_tunnel_info *ti)
{
	struct rnp_adapter *adapter = netdev_priv(dev);

	if (ti->type != UDP_TUNNEL_TYPE_VXLAN)
		return;

	if (ti->sa_family != AF_INET)
		return;

	switch (ti->type) {
		case UDP_TUNNEL_TYPE_VXLAN:
			if (!(adapter->flags & RNP_FLAG_VXLAN_OFFLOAD_CAPABLE))
				return;

			if (adapter->vxlan_port != ti->port) {
				netdev_info(dev, "VXLAN port %d not found\n", ntohs(ti->port));
				return;
			}

			break;
		default:
			return;
	}

	rnp_clear_udp_tunnel_port(adapter);
	adapter->flags2 |= RNP_FLAG2_UDP_TUN_REREG_NEEDED;
}
#elif defined(HAVE_VXLAN_RX_OFFLOAD)
/**
 * rnp_add_vxlan_port - Get notifications about VXLAN ports that come up
 * @dev: The port's netdev
 * @sa_family: Socket Family that VXLAN is notifiying us about
 * @port: New UDP port number that VXLAN started listening to
 */
static void
rnp_add_vxlan_port(struct net_device *dev, sa_family_t sa_family, __be16 port)
{
	struct rnp_adapter *adapter = netdev_priv(dev);
	struct rnp_hw *hw = &adapter->hw;

	if (sa_family != AF_INET)
		return;

	if (!(adapter->flags & RNP_FLAG_VXLAN_OFFLOAD_ENABLE))
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
	hw->ops.set_vxlan_port(hw, adapter->vxlan_port);
	// wr32(hw, RNP_ETH_VXLAN_PORT, adapter->vxlan_port);
}

/**
 * rnp_del_vxlan_port - Get notifications about VXLAN ports that go away
 * @dev: The port's netdev
 * @sa_family: Socket Family that VXLAN is notifying us about
 * @port: UDP port number that VXLAN stopped listening to
 */
static void
rnp_del_vxlan_port(struct net_device *dev, sa_family_t sa_family, __be16 port)
{
	struct rnp_adapter *adapter = netdev_priv(dev);

	if (!(adapter->flags & RNP_FLAG_VXLAN_OFFLOAD_ENABLE))
		return;

	if (sa_family != AF_INET)
		return;

	if (adapter->vxlan_port != port) {
		netdev_info(dev, "Port %d was not found, not deleting\n", ntohs(port));
		return;
	}

	rnp_clear_udp_tunnel_port(adapter);
	adapter->flags2 |= RNP_FLAG2_UDP_TUN_REREG_NEEDED;
}
#endif /* HAVE_VXLAN_RX_OFFLOAD */

#ifdef HAVE_NET_DEVICE_OPS
const struct net_device_ops rnp10_netdev_ops = {
	.ndo_open = rnp_open,
	.ndo_stop = rnp_close,
	.ndo_start_xmit = rnp_xmit_frame,
	.ndo_set_rx_mode = rnp_set_rx_mode,
	.ndo_validate_addr = eth_validate_addr,

	.ndo_do_ioctl = rnp_ioctl,
#ifdef HAVE_RHEL7_NET_DEVICE_OPS_EXT
	/* RHEL7 requires this to be defined to enable extended ops.
	 * RHEL7 uses the function get_ndo_ext to retrieve offsets for
	 * extended fields from with the net_device_ops struct and
	 * ndo_size is checked to determine whether or not
	 * the offset is valid.
	 */
	.ndo_size = sizeof(const struct net_device_ops),
#endif
#ifdef HAVE_RHEL7_EXTENDED_MIN_MAX_MTU
	.extended.ndo_change_mtu = rnp_change_mtu,
#else
	.ndo_change_mtu = rnp_change_mtu,
#endif
#ifdef HAVE_NDO_GET_STATS64
	.ndo_get_stats64 = rnp_get_stats64,
#else
	.ndo_get_stats = rnp_get_stats,
#endif
	.ndo_tx_timeout = rnp_tx_timeout,
#ifdef HAVE_RHEL7_NETDEV_OPS_EXT_NDO_TX_MAXRATE
	.extended.ndo_set_tx_maxrate = rnp_tx_maxrate,
#else
#ifndef NO_TX_MAXRATE
	.ndo_set_tx_maxrate = rnp_tx_maxrate,
#endif
#endif

	//.ndo_set_features = rnp_set_features,
	//.ndo_fix_features = rnp_fix_features,
	.ndo_set_mac_address = rnp_set_mac,
#if defined(NETIF_F_HW_VLAN_TX) || defined(NETIF_F_HW_VLAN_CTAG_TX)
	.ndo_vlan_rx_add_vid = rnp_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid = rnp_vlan_rx_kill_vid,
#endif

#ifdef IFLA_VF_MAX
	.ndo_set_vf_mac = rnp_ndo_set_vf_mac,
//.ndo_set_vf_tx_rate	    = rnp_ndo_set_vf_ring_rate,
#ifdef HAVE_RHEL7_NETDEV_OPS_EXT_NDO_SET_VF_VLAN
	.extended.ndo_set_vf_vlan = rnp_ndo_set_vf_vlan,
#else
	.ndo_set_vf_vlan = rnp_ndo_set_vf_vlan,
#endif
#ifdef HAVE_NDO_SET_VF_MIN_MAX_TX_RATE
	.ndo_set_vf_rate = rnp_ndo_set_vf_bw,
#else
	.ndo_set_vf_tx_rate = rnp_ndo_set_vf_bw,
#endif /* HAVE_NDO_SET_VF_MIN_MAX_TX_RATE */
	.ndo_get_vf_config = rnp_ndo_get_vf_config,
#endif /* IFLA_VF_MAX */
#ifdef HAVE_SETUP_TC
#ifdef HAVE_RHEL7_NETDEV_OPS_EXT_NDO_SETUP_TC
	.extended.ndo_setup_tc_rh = __rnp_setup_tc,
#else
#ifdef NETIF_F_HW_TC
	.ndo_setup_tc = __rnp_setup_tc,
#else
	.ndo_setup_tc = rnp_setup_tc,
#endif /* NETIF_F_HW_TC */
#endif /* HAVE_RHEL7_NETDEV_OPS_EXT_NDO_SETUP_TC */
#endif /* HAVE_SETUP_TC */
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = rnp_netpoll,
#endif
#ifdef HAVE_FDB_OPS
//.ndo_fdb_add		= rnp_ndo_fdb_add,
#endif
#ifdef HAVE_BRIDGE_ATTRIBS
	.ndo_bridge_setlink = rnp_ndo_bridge_setlink,
	.ndo_bridge_getlink = rnp_ndo_bridge_getlink,
#endif
#ifdef HAVE_UDP_ENC_RX_OFFLOAD
#ifdef HAVE_RHEL7_NETDEV_OPS_EXT_NDO_UDP_TUNNEL
	.extended.ndo_udp_tunnel_add = rnp_add_udp_tunnel_port,
	.extended.ndo_udp_tunnel_del = rnp_del_udp_tunnel_port,
#else
#ifndef HAVE_UDP_TUNNEL_NIC_INFO
	.ndo_udp_tunnel_add = rnp_add_udp_tunnel_port,
	.ndo_udp_tunnel_del = rnp_del_udp_tunnel_port,
#endif
#endif
#elif defined(HAVE_VXLAN_RX_OFFLOAD)
	.ndo_add_vxlan_port = rnp_add_vxlan_port,
	.ndo_del_vxlan_port = rnp_del_vxlan_port,
#endif /* HAVE_UDP_ENC_RX_OFFLOAD */
#ifdef HAVE_NDO_FEATURES_CHECK
	.ndo_features_check = rnp_features_check,
#endif /* HAVE_NDO_FEATURES_CHECK */

#ifdef HAVE_RHEL6_NET_DEVICE_OPS_EXT
};

/* RHEL6 keeps these operations in a separate structure */
static const struct net_device_ops_ext rnp_netdev_ops_ext = {
	.size = sizeof(struct net_device_ops_ext),
#endif /* HAVE_RHEL6_NET_DEVICE_OPS_EXT */
#ifdef HAVE_NDO_SET_FEATURES
	.ndo_set_features = rnp_set_features,
	.ndo_fix_features = rnp_fix_features,
#endif /* HAVE_NDO_SET_FEATURES */
};
#endif /* HAVE_NET_DEVICE_OPS */

void rnp_assign_netdev_ops(struct net_device *dev)
{

	/* different hw can assign difference fun */
#ifdef HAVE_NET_DEVICE_OPS
	dev->netdev_ops = &rnp10_netdev_ops;
#ifdef HAVE_RHEL6_NET_DEVICE_OPS_EXT
	set_netdev_ops_ext(dev, &rnp_netdev_ops_ext);
#endif /* HAVE_RHEL6_NET_DEVICE_OPS_EXT */
#else  /* HAVE_NET_DEVICE_OPS */
	dev->open = &rnp_open;
	dev->stop = &rnp_close;
	dev->hard_start_xmit = &rnp_xmit_frame;
	// dev->get_stats = &rnp_get_stats;
#ifdef HAVE_SET_RX_MODE
	dev->set_rx_mode = &rnp_set_rx_mode;
#endif
	dev->set_multicast_list = &rnp_set_rx_mode;
	dev->set_mac_address = &rnp_set_mac;
	dev->change_mtu = &rnp_change_mtu;
	dev->do_ioctl = &rnp_ioctl;
#ifdef HAVE_TX_TIMEOUT
	dev->tx_timeout = &rnp_tx_timeout;
#endif
#if defined(NETIF_F_HW_VLAN_TX) || defined(NETIF_F_HW_VLAN_CTAG_TX)
	// dev->vlan_rx_register = &rnp_vlan_mode; //todo
	dev->vlan_rx_add_vid = &rnp_vlan_rx_add_vid;
	dev->vlan_rx_kill_vid = &rnp_vlan_rx_kill_vid;
#endif
#ifdef CONFIG_NET_POLL_CONTROLLER
	dev->poll_controller = &rnp_netpoll;
#endif
#ifdef HAVE_NETDEV_SELECT_QUEUE
	dev->select_queue = &__netdev_pick_tx;
#endif /* HAVE_NETDEV_SELECT_QUEUE */
#endif /* HAVE_NET_DEVICE_OPS */

#ifdef HAVE_RHEL6_NET_DEVICE_EXTENDED
#ifdef HAVE_NDO_BUSY_POLL
	// netdev_extended(dev)->ndo_busy_poll             = rnp_busy_poll_recv; //
	// todo
#endif /* HAVE_NDO_BUSY_POLL */
#endif /* HAVE_RHEL6_NET_DEVICE_EXTENDED */

	rnp_set_ethtool_ops(dev);
	dev->watchdog_timeo = 5 * HZ;
}

/**
 * rnp_wol_supported - Check whether device supports WoL
 * @hw: hw specific details
 * @device_id: the device ID
 * @subdev_id: the subsystem device ID
 *
 * This function is used by probe and ethtool to determine
 * which devices have WoL support
 *
 **/
int rnp_wol_supported(struct rnp_adapter *adapter,
					  u16 device_id,
					  u16 subdevice_id)
{
	int is_wol_supported = 0;

	struct rnp_hw *hw = &adapter->hw;
#if 0
	u16 wol_cap = adapter->eeprom_cap & RNP_DEVICE_CAPS_WOL_MASK;

	switch (device_id) {
	case RNP_DEV_ID_N10_SFP:
		/* Only these subdevices could supports WOL */
		switch (subdevice_id) {
		case RNP_SUBDEV_ID_n10_560FLR:
			/* only support first port */
			if (hw->bus.func != 0)
				break;
		case RNP_SUBDEV_ID_n10_SP_560FLR:
		case RNP_SUBDEV_ID_n10_SFP:
		case RNP_SUBDEV_ID_n10_RNDC:
		case RNP_SUBDEV_ID_n10_ECNA_DP:
		case RNP_SUBDEV_ID_n10_LOM_SFP:
			is_wol_supported = 1;
			break;
		}
		break;
	case RNP_DEV_ID_n10EN_SFP:
		/* Only this subdevice supports WOL */
		switch (subdevice_id) {
		case RNP_SUBDEV_ID_n10EN_SFP_OCP1:
			is_wol_supported = 1;
			break;
		}
		break;
	case RNP_DEV_ID_n10_COMBO_BACKPLANE:
		/* All except this subdevice support WOL */
		if (subdevice_id != RNP_SUBDEV_ID_n10_KX4_KR_MEZZ)
			is_wol_supported = 1;
		break;
	case RNP_DEV_ID_n10_KX4:
		is_wol_supported = 1;
		break;
	case RNP_DEV_ID_X540T:
	case RNP_DEV_ID_X540T1:
		/* check eeprom to see if enabled wol */
		if ((wol_cap == RNP_DEVICE_CAPS_WOL_PORT0_1) ||
			((wol_cap == RNP_DEVICE_CAPS_WOL_PORT0) &&
			(hw->bus.func == 0))) {
			is_wol_supported = 1;
		}
		break;
	}

#endif
	return is_wol_supported;
}

static inline unsigned long rnp_tso_features(struct rnp_hw *hw)
{
	unsigned long features = 0;

#ifdef NETIF_F_TSO
	if (hw->feature_flags & RNP_NET_FEATURE_TSO)
		features |= NETIF_F_TSO;
#endif /* NETIF_F_TSO */
#ifdef NETIF_F_TSO6
	if (hw->feature_flags & RNP_NET_FEATURE_TSO)
		features |= NETIF_F_TSO6;
#endif /* NETIF_F_TSO6 */
#ifdef NETIF_F_GSO_PARTIAL
	features |= NETIF_F_GSO_PARTIAL;
	if (hw->feature_flags & RNP_NET_FEATURE_TX_UDP_TUNNEL)
		features |= RNP_GSO_PARTIAL_FEATURES;
#endif

	return features;
}

static void remove_mbx_irq(struct rnp_adapter *adapter)
{
	/* mbx */
	if (adapter->num_other_vectors) {
		adapter->hw.mbx.ops.configure(
			&adapter->hw, adapter->msix_entries[0].entry, false);
		free_irq(adapter->msix_entries[0].vector, adapter);

		adapter->hw.mbx.other_irq_enabled = false;
	}
}
//
static int register_mbx_irq(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	struct net_device *netdev = adapter->netdev;
	int err = 0;

	/* for mbx:vector0 */
	if (adapter->num_other_vectors) {
		err = request_irq(adapter->msix_entries[0].vector,
						  rnp_msix_other,
						  0,
						  netdev->name,
						  adapter);
		if (err) {
			e_err(probe, "request_irq for msix_other failed: %d\n", err);
			goto err_mbx;
		}
		hw->mbx.ops.configure(hw, adapter->msix_entries[0].entry, true);
		adapter->hw.mbx.other_irq_enabled = true;
	}

err_mbx:
	return err;
}

static int rnp_rm_adpater(struct rnp_adapter *adapter)
{
	struct net_device *netdev;

	netdev = adapter->netdev;
	pr_info("= remove adapter:%s =\n", netdev->name);

	rnp_dbg_adapter_exit(adapter);

	netif_carrier_off(netdev);

	set_bit(__RNP_DOWN, &adapter->state);
	set_bit(__RNP_REMOVE, &adapter->state);
	if (module_enable_ptp) {
		// should wait ptp timeout
		while (test_bit(__RNP_PTP_TX_IN_PROGRESS, &adapter->state)) {
			usleep_range(10000, 20000);
		}
		cancel_work_sync(&adapter->tx_hwtstamp_work);
	}
	cancel_work_sync(&adapter->service_task);

	del_timer_sync(&adapter->service_timer);
#ifdef CONFIG_RNP_DCA
	if (adapter->flags & RNP_FLAG_DCA_ENABLED) {
		adapter->flags &= ~RNP_FLAG_DCA_ENABLED;
		dca_remove_requester(&pdev->dev);
		wr32(&adapter->hw + RNP_DCA_CTRL, 1);
	}
#endif
	rnp_sysfs_exit(adapter);

	// rm fdir only rm
	rnp_fdir_filter_exit(adapter);

	if (netdev->reg_state == NETREG_REGISTERED)
		unregister_netdev(netdev);

	adapter->netdev = NULL;

	remove_mbx_irq(adapter);

	rnp_clear_interrupt_scheme(adapter);

	iounmap(adapter->io_addr);

#ifdef FIX_VF_BUG
	if (adapter->io_addr_bar0)
		iounmap(adapter->io_addr_bar0);
#endif
	free_netdev(netdev);

	pr_info("remove complete\n");

	return 0;
}

#if 0
void fix_queue_number(struct rnp_hw *hw)
{
	struct rnp_adapter *adapter = hw->back;
	int count;

	count = rd32(hw, RNP_DMA_STATUS);
	count = (count & DMA_RING_NUM) >> 24;
	if (count != adapter->max_ring_pair_counts) {
		dbg("reset max_ring_pair_counts from %d to %d\n",
				adapter->max_ring_pair_counts, count);
		adapter->max_ring_pair_counts = count;
	}
}
#endif

static void rnp_fix_dma_tx_status(struct rnp_adapter *adapter)
{
	int i;
	struct rnp_hw *hw = &adapter->hw;
	struct rnp_dma_info *dma = &hw->dma;

	// update me
	// only n10 setup this
	if (hw->hw_type == rnp_hw_n10) {
		for (i = 0; i < dma->max_tx_queues; i++)
			dma_ring_wr32(dma, RING_OFFSET(i) + RNP_DMA_TX_START, 1);
	}
}

static int rnp_add_adpater(struct pci_dev *pdev,
						   struct rnp_info *ii,
						   struct rnp_adapter **padapter)
{
	int i, err = 0;
	struct rnp_adapter *adapter = NULL;
	struct net_device *netdev;
	struct rnp_hw *hw;
	u8 __iomem *hw_addr;
#ifdef FIX_VF_BUG
	u8 __iomem *hw_addr_bar0 = 0;
#endif
	u32 dma_version = 0;
	u32 nic_version = 0;
	u32 queues = ii->total_queue_pair_cnts;
	static int bd_number;
	netdev_features_t hw_enc_features = 0;
#ifndef NETIF_F_GSO_PARTIAL
#ifdef HAVE_NDO_SET_FEATURES
#ifndef HAVE_RHEL6_NET_DEVICE_OPS_EXT
	netdev_features_t hw_features;
#else
	u32 hw_features;
#endif
#endif
#endif /* NETIF_F_GSO_PARTIAL */

	pr_info("====  add adapter queues:%d ====", queues);
#ifdef HAVE_TX_MQ
	netdev = alloc_etherdev_mq(sizeof(struct rnp_adapter), queues);
#else
	queues = 1;
	netdev = alloc_etherdev(sizeof(struct rnp_adapter));
#endif
	if (!netdev)
		return -ENOMEM;

	if (!fix_eth_name)
		SET_NETDEV_DEV(netdev, &pdev->dev);

	adapter = netdev_priv(netdev);

	memset((char *)adapter, 0x00, sizeof(struct rnp_adapter));
	adapter->netdev = netdev;
	adapter->pdev = pdev;
#ifdef HAVE_TX_MQ
#ifndef HAVE_NETDEV_SELECT_QUEUE
	adapter->indices = queues;
#endif

#endif
	adapter->max_ring_pair_counts = queues;
	if (padapter)
		*padapter = adapter;

	adapter->bd_number = bd_number++;
	adapter->port = 0;
	// fixme in n500
	snprintf(adapter->name,
			 sizeof(netdev->name),
			 "%s%d%d",
			 rnp_driver_name,
			 1,
			 adapter->bd_number);
	pci_set_drvdata(pdev, adapter);

	hw = &adapter->hw;
	hw->back = adapter;
	/* first setup hw type */
	hw->rss_type = ii->rss_type;
	hw->hw_type = ii->hw_type;
	switch (hw->hw_type) {
		case rnp_hw_n10:
		case rnp_hw_n20:
		case rnp_hw_uv440:
#ifdef FIX_VF_BUG
			hw_addr_bar0 =
				ioremap(pci_resource_start(pdev, 0), pci_resource_len(pdev, 0));
			if (!hw_addr_bar0) {
				dev_err(&pdev->dev, "pcim_iomap bar%d faild!\n", 0);
				return -EIO;
			}
			rnp_wr_reg(
				hw_addr_bar0 + (0x7982fc & (pci_resource_len(pdev, 0) - 1)), 0);
#endif

/* n10 use bar4 */
#define RNP_NIC_BAR_N10 4
			hw_addr = ioremap(pci_resource_start(pdev, RNP_NIC_BAR_N10),
							  pci_resource_len(pdev, RNP_NIC_BAR_N10));
			if (!hw_addr) {
				dev_err(
					&pdev->dev, "pcim_iomap bar%d faild!\n", RNP_NIC_BAR_N10);
				return -EIO;
			}
			pr_info(
				"[bar%d]:%p %llx len=%d MB\n",
				RNP_NIC_BAR_N10,
				hw_addr,
				(unsigned long long)pci_resource_start(pdev, RNP_NIC_BAR_N10),
				(int)pci_resource_len(pdev, RNP_NIC_BAR_N10) / 1024 / 1024);
			/* get dma version */
			dma_version = rnp_rd_reg(hw_addr);
#ifdef FIX_VF_BUG
			if (rnp_is_pf1(pdev))
				hw->hw_addr = hw_addr + 0x100000;
			else
				hw->hw_addr = hw_addr;
#else
			hw->hw_addr = hw_addr;
#endif
				/* setup msix base */
#ifdef FIX_VF_BUG
			if (rnp_is_pf1(pdev))
				hw->ring_msix_base = hw->hw_addr + 0xa4000 + 0x200;
			else
				hw->ring_msix_base = hw->hw_addr + 0xa4000;
#else
			hw->ring_msix_base = hw->hw_addr + 0xa4000;
#endif
			if (rnp_is_pf1(pdev))
				hw->pfvfnum = PF_NUM(1);
			else
				hw->pfvfnum = PF_NUM(0);
			nic_version = rd32(hw, RNP_TOP_NIC_VERSION);

			break;
		case rnp_hw_n500:
			// todo
/* n500 use bar2 */
#define RNP_NIC_BAR_N500 2
			hw_addr = ioremap(pci_resource_start(pdev, RNP_NIC_BAR_N500),
							  pci_resource_len(pdev, RNP_NIC_BAR_N500));
			if (!hw_addr) {
				dev_err(
					&pdev->dev, "pcim_iomap bar%d faild!\n", RNP_NIC_BAR_N500);
				return -EIO;
			}
			pr_info(
				"[bar%d]:%p %llx len=%d MB\n",
				RNP_NIC_BAR_N10,
				hw_addr,
				(unsigned long long)pci_resource_start(pdev, RNP_NIC_BAR_N10),
				(int)pci_resource_len(pdev, RNP_NIC_BAR_N500) / 1024 / 1024);
			/* get dma version */
			dma_version = rnp_rd_reg(hw_addr);

			hw->hw_addr = hw_addr;
			/* setup msix base */
			hw->ring_msix_base = hw->hw_addr + 0x28700;
			// hw->ring_msix_base = hw->hw_addr + 0x2a000;

			// todo n500 no need this ?
			hw->pfvfnum = PF_NUM_N500(rnp_get_fuc(pdev));
			nic_version = rd32(hw, RNP500_TOP_NIC_VERSION);
			break;
		default:
#ifdef FIX_VF_BUG
			hw_addr_bar0 =
				ioremap(pci_resource_start(pdev, 0), pci_resource_len(pdev, 0));
#endif
			hw_addr =
				ioremap(pci_resource_start(pdev, 0), pci_resource_len(pdev, 0));
			goto err_free_net;
			break;
	}

	/* setup FT_PADDING */
	{
#ifdef FT_PADDING
		u32 data;

		data = rnp_rd_reg(hw->hw_addr + RNP_DMA_CONFIG);
		SET_BIT(8, data);
		rnp_wr_reg(hw->hw_addr + RNP_DMA_CONFIG, data);
		adapter->priv_flags |= RNP_PRIV_FLAG_FT_PADDING;
#endif
	}

	/* assign to adapter */
	adapter->io_addr = hw_addr;

#ifdef FIX_VF_BUG
	adapter->io_addr_bar0 = hw_addr_bar0;
#endif
	hw->pdev = pdev;
	hw->dma_version = dma_version;
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

	/* we have other irq */
	adapter->num_other_vectors = 1;

	spin_lock_init(&adapter->link_stat_lock);

	if (adapter->num_other_vectors) {
		/* Mailbox */
		rnp_init_mbx_params_pf(hw);
		memcpy(&hw->mbx.ops, ii->mbx_ops, sizeof(hw->mbx.ops));
		if (dma_version >= 0x20210111) {
#ifndef NO_MBX_VERSION
			rnp_dbg("try to get capability from firmware\n");
			rnp_mbx_link_event_enable(hw, 0);
			if (rnp_mbx_get_capability(hw, ii)) {
				dev_err(&pdev->dev, "rnp_mbx_get_capablity faild!\n");
				err = -EIO;
				goto err_free_net;
			}
			adapter->portid_of_card = hw->port_id[0];
#else
			// n500 use this temp
			rnp_dbg("no mbx to get capability\n");
			adapter->portid_of_card = hw->pfvfnum ? 1 : 0;
#endif
		}
	}
	hw->default_rx_queue = 0;
	pr_info("%s %s: dma versioin:0x%x, nic version:0x%x, pfvfnum:0x%x\n",
			adapter->name,
			pci_name(pdev),
			hw->dma_version,
			nic_version,
			hw->pfvfnum);

	/* Setup hw api */
	// memcpy(&hw->mac.ops, ii->mac_ops, sizeof(hw->mac.ops));
	hw->mac.type = ii->mac;
	/* EEPROM */
	if (ii->eeprom_ops)
		memcpy(&hw->eeprom.ops, ii->eeprom_ops, sizeof(hw->eeprom.ops));

	hw->phy.sfp_type = rnp_sfp_type_unknown;

#if 0
	/* rnp_identify_phy_generic will set prtad and mmds properly */
	hw->phy.mdio.prtad = MDIO_PRTAD_NONE;
	hw->phy.mdio.mmds = 0;
	hw->phy.mdio.mode_support = MDIO_SUPPORTS_C45 | MDIO_EMULATE_C22;
	hw->phy.mdio.dev = netdev;
	hw->phy.mdio.mdio_read = rnp_mdio_read;
	hw->phy.mdio.mdio_write = rnp_mdio_write;
#endif
	/* get software info */
	ii->get_invariants(hw);

	hw->ops.setup_ethtool(netdev);
	rnp_assign_netdev_ops(netdev);

	/* setup the private structure */
	/* this private is used only once
	 */
	err = rnp_sw_init(adapter);
	if (err)
		goto err_sw_init;

	/* Cache if MNG FW is up so we don't have to read the REG later */
	// if (hw->mac.ops.mng_fw_enabled)
	//	hw->mng_fw_enabled = hw->mac.ops.mng_fw_enabled(hw);

	/* Make it possible the adapter to be woken up via WOL */

	/* reset_hw fills in the perm_addr as well */
	// err = hw->mac.ops.reset_hw(hw);
	err = hw->ops.reset_hw(hw);
	hw->phy.reset_if_overtemp = false;
	if (err) {
		e_dev_err("HW Init failed: %d\n", err);
		goto err_sw_init;
	}

#if defined(CONFIG_PCI_IOV)
	if (adapter->num_other_vectors) {
		/* Mailbox */
		// rnp_init_mbx_params_pf(hw);
		// memcpy(&hw->mbx.ops, ii->mbx_ops, sizeof(hw->mbx.ops));
		rnp_enable_sriov(adapter);
		// pci_sriov_set_totalvfs(pdev, RNP_MAX_VF_FUNCTIONS - 1);
		pci_sriov_set_totalvfs(pdev, hw->max_vfs - 1);
	}
#endif

#ifdef HAVE_NETDEVICE_MIN_MAX_MTU
	/* MTU range: 68 - 9710 */
#ifdef HAVE_RHEL7_EXTENDED_MIN_MAX_MTU
	// netdev->extended->min_mtu = RNP_MIN_MTU;
	// netdev->extended->max_mtu = RNP_MAX_JUMBO_FRAME_SIZE -
	//                            (ETH_HLEN + 2 * ETH_FCS_LEN);
	netdev->extended->min_mtu = hw->min_length;
	netdev->extended->max_mtu = hw->max_length - (ETH_HLEN + 2 * ETH_FCS_LEN);
#else
	// netdev->min_mtu = RNP_MIN_MTU;
	// netdev->max_mtu = RNP_MAX_JUMBO_FRAME_SIZE - (ETH_HLEN + 2 *
	// ETH_FCS_LEN);
	netdev->min_mtu = hw->min_length;
	netdev->max_mtu = hw->max_length - (ETH_HLEN + 2 * ETH_FCS_LEN);
#endif
#endif

#ifdef NETIF_F_GSO_PARTIAL

	if (hw->feature_flags & RNP_NET_FEATURE_SG)
		netdev->features |= NETIF_F_SG;
	if (hw->feature_flags & RNP_NET_FEATURE_TSO)
		netdev->features |= NETIF_F_TSO | NETIF_F_TSO6;
	if (hw->feature_flags & RNP_NET_FEATURE_RX_HASH)
		netdev->features |= NETIF_F_RXHASH;
	if (hw->feature_flags & RNP_NET_FEATURE_RX_CHECKSUM)
		netdev->features |= NETIF_F_RXCSUM;
	if (hw->feature_flags & RNP_NET_FEATURE_TX_CHECKSUM) {
		netdev->features |= NETIF_F_HW_CSUM | NETIF_F_SCTP_CRC;
		// netdev->features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
		// NETIF_F_SCTP_CRC;
	}

	netdev->features |= NETIF_F_HIGHDMA;

	if (hw->feature_flags & RNP_NET_FEATURE_TX_UDP_TUNNEL) {
		netdev->gso_partial_features = RNP_GSO_PARTIAL_FEATURES;
		netdev->features |= NETIF_F_GSO_PARTIAL | RNP_GSO_PARTIAL_FEATURES;
	}

	netdev->hw_features |= netdev->features;

	if (hw->feature_flags & RNP_NET_FEATURE_VLAN_FILTER)
		netdev->hw_features |= NETIF_F_HW_VLAN_CTAG_FILTER;
	if (hw->feature_flags & RNP_NET_FEATURE_VLAN_OFFLOAD) {
		netdev->hw_features |=
			NETIF_F_HW_VLAN_CTAG_RX | NETIF_F_HW_VLAN_CTAG_TX;
	}
	netdev->hw_features |= NETIF_F_RXALL;
	if (hw->feature_flags & RNP_NET_FEATURE_RX_NTUPLE_FILTER)
		netdev->hw_features |= NETIF_F_NTUPLE;
	if (hw->feature_flags & RNP_NET_FEATURE_RX_FCS)
		netdev->hw_features |= NETIF_F_RXFCS;
#ifdef NETIF_F_HW_TC
	if (hw->feature_flags & RNP_NET_FEATURE_HW_TC)
		netdev->hw_features |= NETIF_F_HW_TC;
#endif

	netdev->vlan_features |= netdev->features | NETIF_F_TSO_MANGLEID;
	netdev->hw_enc_features |= netdev->vlan_features;
	netdev->mpls_features |= NETIF_F_HW_CSUM;

	if (hw->feature_flags & RNP_NET_FEATURE_VLAN_FILTER)
		netdev->features |= NETIF_F_HW_VLAN_CTAG_FILTER;
	if (hw->feature_flags & RNP_NET_FEATURE_VLAN_OFFLOAD) {
		netdev->features |= NETIF_F_HW_VLAN_CTAG_RX | NETIF_F_HW_VLAN_CTAG_TX;
	}

	netdev->priv_flags |= IFF_UNICAST_FLT;
	netdev->priv_flags |= IFF_SUPP_NOFCS;

	if (adapter->flags2 & RNP_FLAG2_RSC_CAPABLE)
		netdev->hw_features |= NETIF_F_LRO;

#else /* NETIF_F_GSO_PARTIAL */

	if (hw->feature_flags & RNP_NET_FEATURE_SG)
		netdev->features |= NETIF_F_SG;
	if (hw->feature_flags & RNP_NET_FEATURE_TX_CHECKSUM)
		netdev->features |= NETIF_F_IP_CSUM;

	netdev->features |= NETIF_F_HIGHDMA;

	// fixme
	netdev->features |= NETIF_F_GSO_UDP_TUNNEL | NETIF_F_GSO_UDP_TUNNEL_CSUM;

#ifdef NETIF_F_IPV6_CSUM
	if (hw->feature_flags & RNP_NET_FEATURE_TX_CHECKSUM)
		netdev->features |= NETIF_F_IPV6_CSUM;
#endif

#ifdef NETIF_F_HW_VLAN_CTAG_TX

	if (hw->feature_flags & RNP_NET_FEATURE_VLAN_FILTER)
		netdev->features |= NETIF_F_HW_VLAN_CTAG_FILTER;
	if (hw->feature_flags & RNP_NET_FEATURE_VLAN_OFFLOAD) {
		netdev->features |= NETIF_F_HW_VLAN_CTAG_RX | NETIF_F_HW_VLAN_CTAG_TX;
	}
#endif
	netdev->features |= rnp_tso_features(hw);

#ifdef NETIF_F_RXHASH
	if (hw->feature_flags & RNP_NET_FEATURE_RX_HASH)
		netdev->features |= NETIF_F_RXHASH;
#endif /* NETIF_F_RXHASH */

	if (hw->feature_flags & RNP_NET_FEATURE_RX_CHECKSUM)
		netdev->features |= NETIF_F_RXCSUM;

#ifdef HAVE_NDO_SET_FEATURES
		/* copy netdev features into list of user selectable features */
#ifndef HAVE_RHEL6_NET_DEVICE_OPS_EXT
	hw_features = netdev->hw_features;
#else
	hw_features = get_netdev_hw_features(netdev);
#endif
	hw_features |= netdev->features;

	/* give us the option of enabling RSC/LRO later */
	if (adapter->flags2 & RNP_FLAG2_RSC_CAPABLE)
		hw_features |= NETIF_F_LRO;
#else
#ifdef NETIF_F_GRO
	/* this is only needed on kernels prior to 2.6.39 */
	netdev->features |= NETIF_F_GRO;
#endif /* NETIF_F_GRO */
#endif /* HAVE_NDO_SET_FEATURES */

#ifdef HAVE_NDO_SET_FEATURES

	if (hw->feature_flags & RNP_NET_FEATURE_TX_CHECKSUM)
		hw_features |= NETIF_F_SCTP_CSUM;
	if (hw->feature_flags & RNP_NET_FEATURE_RX_NTUPLE_FILTER)
		hw_features |= NETIF_F_NTUPLE;
#ifdef NETIF_F_HW_TC
	if (hw->feature_flags & RNP_NET_FEATURE_HW_TC)
		hw_features |= NETIF_F_HW_TC;
#endif
	hw_features |= NETIF_F_RXALL;

	if (hw->feature_flags & RNP_NET_FEATURE_RX_FCS)
		hw_features |= NETIF_F_RXFCS;
#endif

#ifdef HAVE_NDO_SET_FEATURES
#ifdef HAVE_RHEL6_NET_DEVICE_OPS_EXT
	set_netdev_hw_features(netdev, hw_features);
#else
	netdev->hw_features = hw_features;
#endif
#endif

#ifdef HAVE_NETDEV_VLAN_FEATURES

	if (hw->feature_flags & RNP_NET_FEATURE_SG)
		netdev->vlan_features |= NETIF_F_SG;
	if (hw->feature_flags & RNP_NET_FEATURE_TX_CHECKSUM)
		netdev->features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;
	if (hw->feature_flags & RNP_NET_FEATURE_TSO)
		netdev->features |= NETIF_F_TSO | NETIF_F_TSO6;

#endif /* HAVE_NETDEV_VLAN_FEATURES */

#ifdef HAVE_ENCAP_CSUM_OFFLOAD
	if (hw->feature_flags & RNP_NET_FEATURE_SG)
		netdev->hw_enc_features |= NETIF_F_SG;
#endif /* HAVE_ENCAP_CSUM_OFFLOAD */

#ifdef HAVE_VXLAN_RX_OFFLOAD
	if (hw->feature_flags & RNP_NET_FEATURE_TX_CHECKSUM) {
		netdev->hw_enc_features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;
	}

#endif /* HAVE_VXLAN_RX_OFFLOAD */

#endif /* NETIF_F_GSO_PARTIAL */

#ifdef IFF_UNICAST_FLT
	netdev->priv_flags |= IFF_UNICAST_FLT;
#endif
#ifdef IFF_SUPP_NOFCS
	netdev->priv_flags |= IFF_SUPP_NOFCS;
#endif

#ifdef NET_FEATURE_DCB
#ifdef CONFIG_DCB
	rnp_dcb_init(netdev, adapter);
#endif
#endif

	if (adapter->flags2 & RNP_FLAG2_RSC_ENABLED)
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
	pr_info("dev mac:%pM \n", netdev->dev_addr);

	if (!is_valid_ether_addr(netdev->dev_addr)) {
		e_dev_err("invalid MAC address\n");
		err = -EIO;
		goto err_sw_init;
	}

	ether_addr_copy(hw->mac.addr, hw->mac.perm_addr);

	timer_setup(&adapter->service_timer, rnp_service_timer, 0);

	if (module_enable_ptp) {
		/* setup ptp_addr according to mac type */
		switch (adapter->hw.mac.mac_type) {
			case mac_dwc_xlg:
				adapter->ptp_addr = adapter->hw.mac.mac_addr + 0xd00;
				adapter->gmac4 = 1;
				break;
			case mac_dwc_g:
				adapter->ptp_addr = adapter->hw.mac.mac_addr + 0x700;
				adapter->gmac4 = 0;
				break;
		}
		adapter->flags2 |= RNP_FLAG2_PTP_ENABLED;
		if (adapter->flags2 & RNP_FLAG2_PTP_ENABLED) {
			adapter->tx_timeout_factor = 10;
			INIT_WORK(&adapter->tx_hwtstamp_work, rnp_tx_hwtstamp_work);
		}
	}

	INIT_WORK(&adapter->service_task, rnp_service_task);
	clear_bit(__RNP_SERVICE_SCHED, &adapter->state);

	if (fix_eth_name)
		strncpy(netdev->name, adapter->name, sizeof(netdev->name) - 1);
	else {
		strscpy(netdev->name, pci_name(pdev), sizeof(netdev->name));
	}

	err = rnp_init_interrupt_scheme(adapter);
	if (err)
		goto err_sw_init;

	// register other irq
	err = register_mbx_irq(adapter);
	if (err)
		goto err_register;

	/* WOL not supported for all devices */
	adapter->wol = 0;
#if 0
	hw->eeprom.ops.read(hw, 0x2c, &adapter->eeprom_cap);
	hw->wol_enabled = rnp_wol_supported(adapter, pdev->device,
			pdev->subsystem_device);
	if (hw->wol_enabled)
		adapter->wol = RNP_WUFC_MAG;

	device_set_wakeup_enable(&adapter->pdev->dev, adapter->wol);

	/* save off EEPROM version number */
	hw->eeprom.ops.read(hw, 0x2e, &adapter->eeprom_verh);
	hw->eeprom.ops.read(hw, 0x2d, &adapter->eeprom_verl);

	/* pick up the PCI bus settings for reporting later */
	hw->mac.ops.get_bus_info(hw);

	/* print bus type/speed/width info */
	e_dev_info("(PCI Express:%s:%s) %pM\n",
			(hw->bus.speed == rnp_bus_speed_8000 ? "8.0GT/s" :
			 hw->bus.speed == rnp_bus_speed_5000 ? "5.0GT/s" :
			 hw->bus.speed == rnp_bus_speed_2500 ? "2.5GT/s" :
			 "Unknown"),
			(hw->bus.width == rnp_bus_width_pcie_x8 ? "Width x8" :
			 hw->bus.width == rnp_bus_width_pcie_x4 ? "Width x4" :
			 hw->bus.width == rnp_bus_width_pcie_x1 ? "Width x1" :
			 "Unknown"),
			netdev->dev_addr);

	err = rnp_read_pba_string_generic(hw, part_str, RNP_PBANUM_LENGTH);
	if (err)
		strncpy(part_str, "Unknown", RNP_PBANUM_LENGTH);
	if (rnp_is_sfp(hw) && hw->phy.sfp_type != rnp_sfp_type_not_present)
		e_dev_info("MAC: %d, PHY: %d, SFP+: %d, PBA No: %s\n",
				hw->mac.type, hw->phy.type, hw->phy.sfp_type,
				part_str);
	else
		e_dev_info("MAC: %d, PHY: %d, PBA No: %s\n",
				hw->mac.type, hw->phy.type, part_str);

	if (hw->bus.width <= rnp_bus_width_pcie_x4) {
		e_dev_warn("PCI-Express bandwidth available for this card is "
				"not sufficient for optimal performance.\n");
		e_dev_warn("For optimal performance a x8 PCI-Express slot "
				"is required.\n");
	}

#endif
	/* reset the hardware with the new settings */
	// err = hw->mac.ops.start_hw(hw);
	err = hw->ops.start_hw(hw);
	rnp_fix_dma_tx_status(adapter);

	if (!fix_eth_name)
		strscpy(netdev->name, "eth%d", sizeof(netdev->name));
	err = register_netdev(netdev);
	if (err) {
		e_dev_err("register_netdev faild!\n");
		goto err_register;
	}

	/* power down the optics for n10 SFP+ fiber */
	if (hw->ops.disable_tx_laser)
		hw->ops.disable_tx_laser(hw);

	/* carrier off reporting is important to ethtool even BEFORE open */
	netif_carrier_off(netdev);

#ifdef CONFIG_RNP_DCA
	if (dca_add_requester(&pdev->dev) == 0) {
		adapter->flags |= RNP_FLAG_DCA_ENABLED;
		rnp_setup_dca(adapter);
	}
#endif

	if (adapter->flags & RNP_FLAG_SRIOV_ENABLED) {
		DPRINTK(PROBE, INFO, "IOV is enabled with %d VFs\n", adapter->num_vfs);
		for (i = 0; i < adapter->num_vfs; i++)
			rnp_vf_configuration(pdev, (i | 0x10000000));
	}

	if (rnp_sysfs_init(adapter))
		e_err(probe, "failed to allocate sysfs resources\n");

	rnp_dbg_adapter_init(adapter);

	// rnp_mbx_link_event_enable(&adapter->hw, 1);
	hw->ops.set_mbx_link_event(hw, 1);

	/* Need link setup for MNG FW, else wait for RNP_UP */
	// if (hw->mng_fw_enabled && hw->mac.ops.setup_link)
	//    hw->mac.ops.setup_link(hw, RNP_LINK_SPEED_10GB_FULL |
	//    RNP_LINK_SPEED_1GB_FULL, true);

	return 0;
err_unresiger_ndev:
	unregister_netdev(netdev);
err_register:
	remove_mbx_irq(adapter);
	rnp_clear_interrupt_scheme(adapter);
err_sw_init:
	rnp_disable_sriov(adapter);
	adapter->flags2 &= ~RNP_FLAG2_SEARCH_FOR_SFP;
err_free_net:
	free_netdev(netdev);
	return err;
}

/**
 * rnp_probe - Device Initialization Routine
 * @pdev: PCI device information struct
 * @ent: entry in rnp_pci_tbl
 *
 * Returns 0 on success, negative on failure
 *
 * rnp_probe initializes an adapter identified by a pci_dev structure.
 * The OS initialization, configuring of the adapter private structure,
 * and a hardware reset occur.
 **/
static int rnp_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct net_device *netdev;
	struct rnp_hw *hw;
	struct rnp_adapter *adapter;
	struct rnp_info *ii = rnp_info_tbl[id->driver_data];
	int i = 0, err;

	printk("adapter_cnt is %d\n", ii->adapter_cnt);
	/* Catch broken hardware that put the wrong VF device ID in
	 * the PCIe SR-IOV capability.
	 */
	if (pdev->is_virtfn) {
		WARN(1,
			 "%s (%hx:%hx) should not be a VF!\n",
			 pci_name(pdev),
			 pdev->vendor,
			 pdev->device);
		return -EINVAL;
	}

	err = pci_enable_device_mem(pdev);
	if (err)
		return err;

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

	err = pci_request_mem_regions(pdev, rnp_driver_name);
	if (err) {
		dev_err(&pdev->dev, "pci_request_selected_regions failed 0x%x\n", err);
		goto err_pci_reg;
	}

	pci_enable_pcie_error_reporting(pdev);

	pci_set_master(pdev);
	pci_save_state(pdev);

	err = rnp_add_adpater(pdev, ii, &adapter);
	if (err)
		goto err_regions;

	return 0;
err_regions:
	pci_release_mem_regions(pdev);
err_dma:
err_pci_reg:
	return err;
}

/**
 * rnp_remove - Device Removal Routine
 * @pdev: PCI device information struct
 *
 * rnp_remove is called by the PCI subsystem to alert the driver
 * that it should release a PCI device.  The could be caused by a
 * Hot-Plug event, or because the driver is going to be removed from
 * memory.
 **/
static void rnp_remove(struct pci_dev *pdev)
{
	struct rnp_adapter *adapter = pci_get_drvdata(pdev);
	int i;

#ifdef CONFIG_PCI_IOV
	/*
	 * Only disable SR-IOV on unload if the user specified the now
	 * deprecated max_vfs module parameter.
	 */
	// if (max_vfs)
	//  we always clean sriov if pf removed
	rnp_disable_sriov(adapter);
#endif

	rnp_rm_adpater(adapter);

	pci_release_mem_regions(pdev);
	pci_disable_pcie_error_reporting(pdev);
	pci_disable_device(pdev);
}

/**
 * rnp_io_error_detected - called when PCI error is detected
 * @pdev: Pointer to PCI device
 * @state: The current pci connection state
 *
 * This function is called after a PCI bus error affecting
 * this device has been detected.
 */
static pci_ers_result_t rnp_io_error_detected(struct pci_dev *pdev,
											  pci_channel_state_t state)
{
	struct rnp_adapter *adapter = pci_get_drvdata(pdev);
	struct net_device *netdev = adapter->netdev;

#if 0

#ifdef CONFIG_PCI_IOV
	struct pci_dev *bdev, *vfdev;
	u32 dw0, dw1, dw2, dw3;
	int vf, pos;
	u16 req_id, pf_func;

	if (adapter->hw.mac.type == rnp_mac_82598EB ||
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
		device_id = RNP_DEV_ID_N10_VF;

		/* Find the pci device of the offending VF */
		vfdev = pci_get_device(PCI_VENDOR_ID_MUCSE, device_id, NULL);
		while (vfdev) {
			if (vfdev->devfn == (req_id & 0xFF))
				break;
			vfdev = pci_get_device(PCI_VENDOR_ID_MUCSE,
					device_id, vfdev);
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
		rnp_down(adapter);
	pci_disable_device(pdev);
#endif
	/* Request a slot reset. */
	return PCI_ERS_RESULT_NEED_RESET;
}

/**
 * rnp_io_slot_reset - called after the pci bus has been reset.
 * @pdev: Pointer to PCI device
 *
 * Restart the card from scratch, as if from a cold-boot.
 */
static pci_ers_result_t rnp_io_slot_reset(struct pci_dev *pdev)
{
	pci_ers_result_t result = PCI_ERS_RESULT_NONE;

#if 0
	struct rnp_adapter *adapter = pci_get_drvdata(pdev);
	int err;

	if (pci_enable_device_mem(pdev)) {
		e_err(probe, "Cannot re-enable PCI device after reset.\n");
		result = PCI_ERS_RESULT_DISCONNECT;
	} else {
		pci_set_master(pdev);
		pci_restore_state(pdev);
		pci_save_state(pdev);

		pci_wake_from_d3(pdev, false);

		rnp_reset(adapter);
		//RNP_WRITE_REG(&adapter->hw, RNP_WUS, ~0);
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

/**
 * rnp_io_resume - called when traffic can start flowing again.
 * @pdev: Pointer to PCI device
 *
 * This callback is called when the error recovery driver tells us that
 * its OK to resume normal operation.
 */
static void rnp_io_resume(struct pci_dev *pdev)
{
	struct rnp_adapter *adapter = pci_get_drvdata(pdev);
	struct net_device *netdev = adapter->netdev;

#ifdef CONFIG_PCI_IOV
	if (adapter->vferr_refcount) {
		e_info(drv, "Resuming after VF err\n");
		adapter->vferr_refcount--;
		return;
	}

#endif
	if (netif_running(netdev))
		rnp_up(adapter);

	netif_device_attach(netdev);
}

static const struct pci_error_handlers rnp_err_handler = {
	.error_detected = rnp_io_error_detected,
	.slot_reset = rnp_io_slot_reset,
	.resume = rnp_io_resume,
};

static struct pci_driver rnp_driver = {
	.name = rnp_driver_name,
	.id_table = rnp_pci_tbl,
	.probe = rnp_probe,
	.remove = rnp_remove,
#ifdef CONFIG_PM
	.suspend = rnp_suspend,
	.resume = rnp_resume,
#endif
//.shutdown = rnp_shutdown,
#if defined(HAVE_SRIOV_CONFIGURE)
	.sriov_configure = rnp_pci_sriov_configure,
#endif /* HAVE_SRIOV_CONFIGURE */
	.err_handler = &rnp_err_handler,
};

static int __init rnp_init_module(void)
{
	int ret;

	pr_info("%s - version %s\n", rnp_driver_string, rnp_driver_version);
	pr_info("%s\n", rnp_copyright);
	rnp_wq = create_singlethread_workqueue(rnp_driver_name);

	if (!rnp_wq) {
		pr_err("%s: Failed to create workqueue\n", rnp_driver_name);
		return -ENOMEM;
	}

	rnp_dbg_init();

	ret = pci_register_driver(&rnp_driver);
	if (ret) {
		destroy_workqueue(rnp_wq);
		rnp_dbg_exit();
		return ret;
	}

	return 0;
}
module_init(rnp_init_module);

static void __exit rnp_exit_module(void)
{
	pci_unregister_driver(&rnp_driver);

	destroy_workqueue(rnp_wq);

	rnp_dbg_exit();

	rcu_barrier(); /* Wait for completion of call_rcu()'s */
}

module_exit(rnp_exit_module);
