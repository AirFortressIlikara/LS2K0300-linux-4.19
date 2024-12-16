#include "rnp.h"
#include "rnp_sriov.h"
#include "rnp_common.h"

//#define CPU_OFFSET_TEST
#ifdef CONFIG_RNP_DCB
/**
 * rnp_cache_ring_dcb_sriov - Descriptor ring to register mapping for SR-IOV
 * @adapter: board private structure to initialize
 *
 * Cache the descriptor ring offsets for SR-IOV to the assigned rings.  It
 * will also try to cache the proper offsets if RSS/FCoE are enabled along
 * with VMDq.
 *
 **/
static bool rnp_cache_ring_dcb_sriov(struct rnp_adapter *adapter)
{
	struct rnp_ring_feature *vmdq = &adapter->ring_feature[RING_F_VMDQ];
	int i;
	u8 tcs = netdev_get_num_tc(adapter->netdev);

	/* verify we have DCB queueing enabled before proceeding */
	if (tcs <= 1)
		return false;

	/* verify we have VMDq enabled before proceeding */
	if (!(adapter->flags & RNP_FLAG_SRIOV_ENABLED))
		return false;

	return true;
}
#endif

/* rnp_get_first_reg_idx - Return first register index associated with ring */
static void rnp_get_first_reg_idx(struct rnp_adapter *adapter, u8 tc,
				  unsigned int *tx, unsigned int *rx)
{
	struct net_device *dev = adapter->netdev;
	struct rnp_hw *hw = &adapter->hw;
	u8 num_tcs = netdev_get_num_tc(dev);

	*tx = 0;
	*rx = 0;

	//*tx = tc << 2; /* 0, 4,  8, 12, 16, 20, 24, 28 */
	//*rx = tc << 2; /* 0, 8, 16, 24, 32, 40, 48, 56 */
	//	switch (hw->mac.type) {
	//	case rnp_mac_n10:
	//		/* TxQs/TC: 4	RxQs/TC: 8 */
	//		*tx = tc << 2; /* 0, 4,  8, 12, 16, 20, 24, 28 */
	//		*rx = tc << 3; /* 0, 8, 16, 24, 32, 40, 48, 56 */
	//		break;
	//	case rnp_mac_n10EB:
	//	case rnp_mac_X540:
	//		if (num_tcs > 4) {
//			/*
//			 * TCs    : TC0/1 TC2/3 TC4-7
//			 * TxQs/TC:    32    16     8
//			 * RxQs/TC:    16    16    16
//			 */
//			*rx = tc << 4;
//			if (tc < 3)
//				*tx = tc << 5; /*   0,  32,  64 */
//			else if (tc < 5)
//				*tx = (tc + 2) << 4; /*  80,  96 */
//			else
//				*tx = (tc + 8) << 3; /* 104, 112, 120 */
//		} else {
//			/*
//			 * TCs    : TC0 TC1 TC2/3
//			 * TxQs/TC:  64  32    16
//			 * RxQs/TC:  32  32    32
//			 */
//			*rx = tc << 5;
//			if (tc < 2)
//				*tx = tc << 6; /*  0,  64 */
//			else
//				*tx = (tc + 4) << 4; /* 96, 112 */
//		}
//	default:
//		break;
//	}
}

/**
 * rnp_cache_ring_dcb - Descriptor ring to register mapping for DCB
 * @adapter: board private structure to initialize
 *
 * Cache the descriptor ring offsets for DCB to the assigned rings.
 *
 **/
static bool rnp_cache_ring_dcb(struct rnp_adapter *adapter)
{
	struct net_device *dev = adapter->netdev;
	unsigned int tx_idx, rx_idx;
	int tc, offset, rss_i, i, step;
	u8 num_tcs = netdev_get_num_tc(dev);
	struct rnp_ring *ring;
	struct rnp_hw *hw = &adapter->hw;
	struct rnp_dma_info *dma = &hw->dma;

	/* verify we have DCB queueing enabled before proceeding */
	if (num_tcs <= 1)
		return false;

	rss_i = adapter->ring_feature[RING_F_RSS].indices;

	step = 4;
	// reset ring 
	for (tc = 0, offset = 0; tc < num_tcs; tc++, offset += rss_i) {

		 /* 
		  * we from tc start
		  * tc0 0 4 8 c 
		  * tc1 1 5 9 d
		  * tc2 2 6 a e
		  * tc3 3 7 b f
		  */
		tx_idx = tc;
		rx_idx = tc;
		//rnp_get_first_reg_idx(adapter, tc, &tx_idx, &rx_idx);
		for (i = 0; i < rss_i; i++, tx_idx+=step, rx_idx+=step) {
			//rnp_dbg("ring %d idex is %d\n", offset + i,  tx_idx);
			//adapter->tx_ring[offset + i]->dcb_tc = tc;
			//adapter->rx_ring[offset + i]->dcb_tc = tc;

			ring = adapter->tx_ring[offset + i];

			ring->ring_addr = dma->dma_ring_addr + RING_OFFSET(tx_idx);
			ring->rnp_queue_idx = tx_idx;
			ring->dma_int_stat =
				ring->ring_addr + RNP_DMA_INT_STAT;
			ring->dma_int_mask = ring->ring_addr + RNP_DMA_INT_MASK;
			ring->dma_int_clr = ring->ring_addr + RNP_DMA_INT_CLR;

			ring = adapter->rx_ring[offset + i];
			ring->ring_addr = dma->dma_ring_addr + RING_OFFSET(rx_idx);
			ring->rnp_queue_idx = rx_idx;
			ring->dma_int_stat =
				ring->ring_addr + RNP_DMA_INT_STAT;
			ring->dma_int_mask = ring->ring_addr + RNP_DMA_INT_MASK;
			ring->dma_int_clr = ring->ring_addr + RNP_DMA_INT_CLR;
		}
	}

	return true;
}

/**
 * rnp_cache_ring_sriov - Descriptor ring to register mapping for sriov
 * @adapter: board private structure to initialize
 *
 * SR-IOV doesn't use any descriptor rings but changes the default if
 * no other mapping is used.
 *
 */
static bool rnp_cache_ring_sriov(struct rnp_adapter *adapter)
{
	struct rnp_ring_feature *vmdq = &adapter->ring_feature[RING_F_VMDQ];
	struct rnp_ring_feature *rss = &adapter->ring_feature[RING_F_RSS];
	int i;
	u16 reg_idx;

	/* only proceed if VMDq is enabled */
	if (!(adapter->flags & RNP_FLAG_VMDQ_ENABLED))
		return false;
#if 0
	/* start at VMDq register offset for SR-IOV enabled setups */
	reg_idx = vmdq->offset * __ALIGN_MASK(1, ~vmdq->mask);
	for (i = 0; i < adapter->num_rx_queues; i++, reg_idx++) {
		/* If we are greater than indices move to next pool */
		if ((reg_idx & ~vmdq->mask) >= rss->indices)
			reg_idx = __ALIGN_MASK(reg_idx, ~vmdq->mask);
		adapter->rx_ring[i]->reg_idx = reg_idx;
	}

	reg_idx = vmdq->offset * __ALIGN_MASK(1, ~vmdq->mask);
	for (i = 0; i < adapter->num_tx_queues; i++, reg_idx++) {
		/* If we are greater than indices move to next pool */
		if ((reg_idx & rss->mask) >= rss->indices)
			reg_idx = __ALIGN_MASK(reg_idx, ~vmdq->mask);
		adapter->tx_ring[i]->reg_idx = reg_idx;
	}
#endif

	return true;
}

/**
 * rnp_cache_ring_rss - Descriptor ring to register mapping for RSS
 * @adapter: board private structure to initialize
 *
 * Cache the descriptor ring offsets for RSS to the assigned rings.
 *
 **/
static bool rnp_cache_ring_rss(struct rnp_adapter *adapter)
{
	int i;
	/* setup here */

	struct rnp_ring *ring;
	struct rnp_hw *hw = &adapter->hw;
	struct rnp_dma_info *dma = &hw->dma;

	/* some ring alloc rules can be added here */
	for (i = 0; i < adapter->num_rx_queues; i++) {
		ring = adapter->tx_ring[i];
		ring->rnp_queue_idx = i;
		ring->ring_addr = dma->dma_ring_addr + RING_OFFSET(i);

		ring->dma_int_stat =
			ring->ring_addr + RNP_DMA_INT_STAT;
		ring->dma_int_mask = ring->ring_addr + RNP_DMA_INT_MASK;
		ring->dma_int_clr = ring->ring_addr + RNP_DMA_INT_CLR;
	}

	for (i = 0; i < adapter->num_tx_queues; i++) {
		ring = adapter->rx_ring[i];
		ring->rnp_queue_idx = i;
		ring->ring_addr = dma->dma_ring_addr + RING_OFFSET(i);
		ring->dma_int_stat =
			ring->ring_addr + RNP_DMA_INT_STAT;
		ring->dma_int_mask = ring->ring_addr + RNP_DMA_INT_MASK;
		ring->dma_int_clr = ring->ring_addr + RNP_DMA_INT_CLR;
	}

	return true;
}

/**
 * rnp_cache_ring_register - Descriptor ring to register mapping
 * @adapter: board private structure to initialize
 *
 * Once we know the feature-set enabled for the device, we'll cache
 * the register offset the descriptor ring is assigned to.
 *
 * Note, the order the various feature calls is important.  It must start with
 * the "most" features enabled at the same time, then trickle down to the
 * least amount of features turned on at once.
 **/
static void rnp_cache_ring_register(struct rnp_adapter *adapter)
{
	/* start with default case */

#ifdef CONFIG_RNP_DCB
	if (rnp_cache_ring_dcb_sriov(adapter))
		return;

#endif
	if (rnp_cache_ring_dcb(adapter))
		return;

	/* sriov ring alloc is added before, this maybe no use */
	if (rnp_cache_ring_sriov(adapter))
		return;

	rnp_cache_ring_rss(adapter);
}

#define RNP_RSS_128Q_MASK 0x7F
#define RNP_RSS_64Q_MASK 0x3F
#define RNP_RSS_16Q_MASK 0xF
#define RNP_RSS_32Q_MASK 0x1F
#define RNP_RSS_8Q_MASK 0x7
#define RNP_RSS_4Q_MASK 0x3
#define RNP_RSS_2Q_MASK 0x1
#define RNP_RSS_DISABLED_MASK 0x0

#ifdef CONFIG_RNP_DCB
/**
 * rnp_set_dcb_sriov_queues: Allocate queues for SR-IOV devices w/ DCB
 * @adapter: board private structure to initialize
 *
 * When SR-IOV (Single Root IO Virtualiztion) is enabled, allocate queues
 * and VM pools where appropriate.  Also assign queues based on DCB
 * priorities and map accordingly..
 *
 **/
static bool rnp_set_dcb_sriov_queues(struct rnp_adapter *adapter)
{
	int i;
	u16 vmdq_i = adapter->ring_feature[RING_F_VMDQ].limit;
	u16 vmdq_m = 0;
	u8 tcs = netdev_get_num_tc(adapter->netdev);

	/* verify we have DCB queueing enabled before proceeding */
	if (tcs <= 1)
		return false;

	/* verify we have VMDq enabled before proceeding */
	if (!(adapter->flags & RNP_FLAG_SRIOV_ENABLED))
		return false;

	/* Add starting offset to total pool count */
	vmdq_i += adapter->ring_feature[RING_F_VMDQ].offset;

	/* 16 pools w/ 8 TC per pool */
	if (tcs > 4) {
		vmdq_i = min_t(u16, vmdq_i, 16);
		vmdq_m = RNP_n10_VMDQ_8Q_MASK;
		/* 32 pools w/ 4 TC per pool */
	} else {
		vmdq_i = min_t(u16, vmdq_i, 32);
		vmdq_m = RNP_n10_VMDQ_4Q_MASK;
	}

	/* remove the starting offset from the pool count */
	vmdq_i -= adapter->ring_feature[RING_F_VMDQ].offset;

	/* save features for later use */
	adapter->ring_feature[RING_F_VMDQ].indices = vmdq_i;
	adapter->ring_feature[RING_F_VMDQ].mask = vmdq_m;

	/*
	 * We do not support DCB, VMDq, and RSS all simultaneously
	 * so we will disable RSS since it is the lowest priority
	 */
	adapter->ring_feature[RING_F_RSS].indices = 2;
	adapter->ring_feature[RING_F_RSS].mask = RNP_RSS_DISABLED_MASK;

	/* disable ATR as it is not supported when VMDq is enabled */
	adapter->flags &= ~RNP_FLAG_FDIR_HASH_CAPABLE;

	adapter->num_tx_queues = vmdq_i * tcs;
	adapter->num_rx_queues = vmdq_i * tcs;

	/* configure TC to queue mapping */
	for (i = 0; i < tcs; i++)
		netdev_set_tc_queue(adapter->netdev, i, 1, i);

	return true;
}
#endif

static bool rnp_set_dcb_queues(struct rnp_adapter *adapter)
{
	struct net_device *dev = adapter->netdev;
	struct rnp_ring_feature *f;
	int rss_i, rss_m, i;
	int tcs;

	/* Map queue offset and counts onto allocated tx queues */
	tcs = netdev_get_num_tc(dev);

	/* verify we have DCB queueing enabled before proceeding */
	if (tcs <= 1)
		return false;

	/* determine the upper limit for our current DCB mode */
	rss_i = dev->num_tx_queues / tcs;

	/* we only support 4 tc , rss_i max is 32 */

	/* 4 TC w/ 32 queues per TC */
	rss_i = min_t(u16, rss_i, 32);
	rss_m = RNP_RSS_32Q_MASK;

	/* set RSS mask and indices */
	/* f->limit is relative with cpu_vector */
	f = &adapter->ring_feature[RING_F_RSS];
	/* use f->limit to change rss */
	rss_i = min_t(int, rss_i, f->limit);
	f->indices = rss_i;
	f->mask = rss_m;

	/* disable ATR as it is not supported when multiple TCs are enabled */
	adapter->flags &= ~RNP_FLAG_FDIR_HASH_CAPABLE;

	/* setup queue tc num */
	for (i = 0; i < tcs; i++)
		netdev_set_tc_queue(dev, i, rss_i, rss_i * i);

	// set the true queues
	adapter->num_tx_queues = rss_i * tcs;
	adapter->num_rx_queues = rss_i * tcs;

	return true;
}

/**
 * rnp_set_sriov_queues - Allocate queues for SR-IOV devices
 * @adapter: board private structure to initialize
 *
 * When SR-IOV (Single Root IO Virtualiztion) is enabled, allocate queues
 * and VM pools where appropriate.  If RSS is available, then also try and
 * enable RSS and map accordingly.
 *
 **/
static bool rnp_set_sriov_queues(struct rnp_adapter *adapter)
{
	u16 vmdq_i = adapter->ring_feature[RING_F_VMDQ].limit;
	u16 vmdq_m = 0;
	u16 rss_i = adapter->ring_feature[RING_F_RSS].limit;
	u16 rss_m = RNP_RSS_DISABLED_MASK;
	struct rnp_hw *hw = &adapter->hw;

	/* only proceed if SR-IOV is enabled */
	if (!(adapter->flags & RNP_FLAG_SRIOV_ENABLED))
		return false;

	/* save features for later use */
	adapter->ring_feature[RING_F_VMDQ].indices =
		adapter->max_ring_pair_counts - 1;
	adapter->ring_feature[RING_F_VMDQ].mask = vmdq_m;

	/* limit RSS based on user input and save for later use */
	adapter->ring_feature[RING_F_RSS].indices = rss_i;
	adapter->ring_feature[RING_F_RSS].mask = rss_m;

	adapter->num_rx_queues = hw->sriov_ring_limit;
	adapter->num_tx_queues = hw->sriov_ring_limit;

	/* disable ATR as it is not supported when VMDq is enabled */
	adapter->flags &= ~RNP_FLAG_FDIR_HASH_CAPABLE;

	return true;
}

u32 rnp_rss_indir_tbl_entries(struct rnp_adapter *adapter)
{
	if (adapter->hw.rss_type == rnp_rss_uv3p)
		return 8;
	else if (adapter->hw.rss_type == rnp_rss_uv440)
		return 128;
	else if (adapter->hw.rss_type == rnp_rss_n10)
		return 128;
	else
		return 128;
}
/**
 * rnp_set_rss_queues - Allocate queues for RSS
 * @adapter: board private structure to initialize
 *
 * This is our "base" multiqueue mode.  RSS (Receive Side Scaling) will try
 * to allocate one Rx queue per CPU, and if available, one Tx queue per CPU.
 *
 **/
static bool rnp_set_rss_queues(struct rnp_adapter *adapter)
{
	struct rnp_ring_feature *f;
	u16 rss_i;

	f = &adapter->ring_feature[RING_F_RSS];
	/* use thid to change ring num */
	rss_i = f->limit;
	/* set limit -> indices */
	f->indices = rss_i;

	/* should init rss mask */
	switch(adapter->hw.rss_type) {
	case rnp_rss_uv3p:
		f->mask = RNP_RSS_8Q_MASK;
	break;
	case rnp_rss_uv440:
		f->mask = RNP_RSS_64Q_MASK;
	break;
	case rnp_rss_n10:
		/* maybe not good */
		f->mask = RNP_RSS_128Q_MASK;
	break;
		/* maybe not good */
	case rnp_rss_n500:
		f->mask = RNP_RSS_8Q_MASK;
	break;
	default:
		f->mask = 0;

	break;
	}

#if 0
	/* disable ATR by default, it will be configured below */
	adapter->flags &= ~RNP_FLAG_FDIR_HASH_CAPABLE;

	/*
	 * Use Flow Director in addition to RSS to ensure the best
	 * distribution of flows across cores, even when an FDIR flow
	 * isn't matched.
	 */
	if (rss_i > 1 && adapter->atr_sample_rate) {
		f = &adapter->ring_feature[RING_F_FDIR];

		rss_i = f->indices = f->limit;

		if (!(adapter->flags & RNP_FLAG_FDIR_PERFECT_CAPABLE))
			adapter->flags |= RNP_FLAG_FDIR_HASH_CAPABLE;
	}
#endif
	/* set rss_i -> adapter->num_tx_queues */
	// should not more than irq ?
	adapter->num_tx_queues =
		min_t(int, rss_i, adapter->max_ring_pair_counts);
	adapter->num_rx_queues = adapter->num_tx_queues;

	rnp_dbg("[%s] limit:%d indices:%d queues:%d\n", adapter->name, f->limit,
		f->indices, adapter->num_tx_queues);

	return true;
}

/**
 * rnp_set_num_queues - Allocate queues for device, feature dependent
 * @adapter: board private structure to initialize
 *
 * This is the top level queue allocation routine.  The order here is very
 * important, starting with the "most" number of features turned on at once,
 * and ending with the smallest set of features.  This way large combinations
 * can be allocated if they're turned on, and smaller combinations are the
 * fallthrough conditions.
 *
 **/
static void rnp_set_num_queues(struct rnp_adapter *adapter)
{
	/* Start with base case */
	adapter->num_tx_queues = 1;
	adapter->num_rx_queues = 1;

#ifdef CONFIG_RNP_DCB
	if (rnp_set_dcb_sriov_queues(adapter))
		return;

#endif
	if (rnp_set_dcb_queues(adapter))
		return;

	if (rnp_set_sriov_queues(adapter))
		return;
	/* at last we support rss */
	rnp_set_rss_queues(adapter);
}

int rnp_acquire_msix_vectors(struct rnp_adapter *adapter, int vectors)
{
	int err, vector;

#ifdef DISABLE_RX_IRQ
	vectors -= adapter->num_other_vectors;
	adapter->num_q_vectors = min(vectors, adapter->max_q_vectors);
	return 0;
#endif

	// todo fix can not get enough msix
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	err = pci_enable_msix_range(adapter->pdev, adapter->msix_entries,
				    vectors, vectors);
#else
	err = pci_enable_msix(adapter->pdev, adapter->msix_entries, vectors);
#endif
	if (err < 0) {
		rnp_err("pci_enable_msix faild: req:%d err:%d\n", vectors, err);
		kfree(adapter->msix_entries);
		adapter->msix_entries = NULL;
		return -EINVAL;
	}
	/*
	 * Adjust for only the vectors we'll use, which is minimum
	 * of max_msix_q_vectors + NON_Q_VECTORS, or the number of
	 * vectors we were allocated.
	 */
	vectors -= adapter->num_other_vectors;
	adapter->num_q_vectors = min(vectors, adapter->max_q_vectors);
	// in dcb we use max 32 q-vectors
	// each vectors for max 4 tcs
	if (adapter->flags & RNP_FLAG_DCB_ENABLED) {
		adapter->num_q_vectors = min(32, adapter->num_q_vectors);
		
	}

	return 0;
}

static void rnp_add_ring(struct rnp_ring *ring, struct rnp_ring_container *head)
{
	ring->next = head->ring;
	head->ring = ring;
	head->count++;
}

static inline void rnp_irq_disable_queues(struct rnp_q_vector *q_vector)
{
	struct rnp_ring *ring;

	rnp_for_each_ring(ring, q_vector->tx) {
		/*
		if (q_vector->new_usesc != q_vector->old_usesc) {

			ring_wr32(ring, RNP_DMA_REG_RX_INT_DELAY_TIMER,
					q_vector->new_usesc * 500);
			q_vector->old_usesc = q_vector->new_usesc;
		}*/
		rnp_wr_reg(ring->dma_int_mask, (RX_INT_MASK | TX_INT_MASK));
		// update usecs
	}
}


static enum hrtimer_restart irq_miss_check(struct hrtimer *hrtimer)
{
	struct rnp_q_vector *q_vector;
	struct rnp_ring *ring;
	struct rnp_tx_desc *eop_desc;
	struct rnp_adapter *adapter;
	
	int tx_next_to_clean;
	int tx_next_to_use;

	struct rnp_tx_buffer *tx_buffer;
	union rnp_rx_desc *rx_desc;

	q_vector = container_of(hrtimer, struct rnp_q_vector, irq_miss_check_timer);
	adapter = q_vector->adapter;
	/*
	rnp_for_each_ring(ring, q_vector->tx)
		rnp_wr_reg(ring->dma_int_mask, (RX_INT_MASK | TX_INT_MASK));
	*/
	if (test_bit(__RNP_DOWN, &adapter->state) ||
			test_bit(__RNP_RESETTING, &adapter->state))
		goto do_self_napi;
	// check tx irq miss
	rnp_for_each_ring(ring, q_vector->tx) {
		tx_next_to_clean = ring->next_to_clean;
		tx_next_to_use = ring->next_to_use;
		// have work to do
		if (tx_next_to_use != tx_next_to_clean) {
			tx_buffer = &ring->tx_buffer_info[tx_next_to_clean];
			eop_desc = tx_buffer->next_to_watch;
			// have tx done 
			// next_to_watch maybe null in some condition
			if (eop_desc) {
				if ((eop_desc->vlan_cmd & cpu_to_le32(RNP_TXD_STAT_DD))) {
					// close irq 
					// printk("call irq self\n");
					// close irq 
					rnp_irq_disable_queues(q_vector);
                                        if (q_vector->new_rx_count != q_vector->old_rx_count) {
                                                ring_wr32(ring, RNP_DMA_REG_RX_INT_DELAY_PKTCNT, q_vector->new_rx_count);
                                                //ring_wr32(ring, RNP_DMA_REG_RX_INT_DELAY_TIMER,
                                                //              q_vector->new_usesc * 500);
                                                q_vector->old_rx_count = q_vector->new_rx_count;
                                        }
					napi_schedule_irqoff(&q_vector->napi);
					goto do_self_napi;
				}
			}
		}
	}

	//check rx irq
	rnp_for_each_ring(ring, q_vector->rx) {
		rx_desc = RNP_RX_DESC(ring, ring->next_to_clean);
		if (rnp_test_staterr(rx_desc, RNP_RXD_STAT_DD)) {
			int size;
			
			size = le16_to_cpu(rx_desc->wb.len);

			if (size) {
				rnp_irq_disable_queues(q_vector);
				if (q_vector->new_rx_count != q_vector->old_rx_count) {
					ring_wr32(ring, RNP_DMA_REG_RX_INT_DELAY_PKTCNT, q_vector->new_rx_count);
					//ring_wr32(ring, RNP_DMA_REG_RX_INT_DELAY_TIMER,
					//              q_vector->new_usesc * 500);
					q_vector->old_rx_count = q_vector->new_rx_count;
				}
				napi_schedule_irqoff(&q_vector->napi);
			} else {
				// rx error, do reset
				// in sriov mode set reset pf flags
				if (adapter->flags & RNP_FLAG_SRIOV_ENABLED)
					adapter->flags2 |= RNP_FLAG2_RESET_PF;	
				else
					adapter->flags2 |= RNP_FLAG2_RESET_REQUESTED;	
			}
			goto do_self_napi;
		}
	}
do_self_napi:
	return HRTIMER_NORESTART;
}

/**
 * rnp_alloc_q_vector - Allocate memory for a single interrupt vector
 * @adapter: board private structure to initialize
 * @v_count: q_vectors allocated on adapter, used for ring interleaving
 * @v_idx: index of vector in adapter struct
 * @txr_count: total number of Tx rings to allocate
 * @txr_idx: index of first Tx ring to allocate
 * @rxr_count: total number of Rx rings to allocate
 * @rxr_idx: index of first Rx ring to allocate
 *
 * We allocate one q_vector.  If allocation fails we return -ENOMEM.
 **/
static int rnp_alloc_q_vector(struct rnp_adapter *adapter, int eth_queue_idx,
			      int v_idx, int r_idx, int r_count, int step)
{
	struct rnp_q_vector *q_vector;
	struct rnp_ring *ring;
	struct rnp_hw *hw = &adapter->hw;
	struct rnp_dma_info *dma = &hw->dma;
	int node = NUMA_NO_NODE;
	int cpu = -1;
	int ring_count, size;
	int txr_count, rxr_count, idx;
	int rxr_idx = r_idx, txr_idx = r_idx;
	u8 tcs = netdev_get_num_tc(adapter->netdev);
	struct device *dev = &adapter->pdev->dev;
	int cpu_offset = 0;
#ifdef CPU_OFFSET_TEST
	int i;
	int orig_node = dev_to_node(dev);
#endif

	DPRINTK(PROBE, INFO, 
			"eth_queue_idx:%d v_idx:%d(off:%d) ring:%d ring_cnt:%d, "
			"step:%d\n",
			eth_queue_idx,
			v_idx,
			adapter->q_vector_off,
			r_idx,
			r_count,
			step);

	txr_count = rxr_count = r_count;

	ring_count = txr_count + rxr_count;
	size = sizeof(struct rnp_q_vector) +
	       (sizeof(struct rnp_ring) * ring_count);

#ifdef CPU_OFFSET_TEST
	// get the first cpu in dev numa
	for (i = 0; i < num_online_cpus(); i++) {
		cpu = i;
		node = cpu_to_node(cpu);
		if (node == orig_node) {
			cpu_offset = cpu;
			break;
		}

	}
	
	// should consider larger than cpu number
	if (cpu_offset + v_idx - adapter->q_vector_off > num_online_cpus()) {
		cpu_offset = cpu_offset - num_online_cpus();
		rnp_dbg("start from zero cpu %d \n", num_online_cpus());
	}
#endif
	/* should minis adapter->q_vector_off */
	if (cpu_online(cpu_offset + v_idx - adapter->q_vector_off)) {
		/*
		 * test feiteng, assign Manually
		 * cpu 48 - 55
		 * node 6
		 */
#if 0
		cpu = 48 +  v_idx - adapter->q_vector_off;
		node = cpu_to_node(cpu);
#else
		/* cpu 1 - 7 */
		cpu = cpu_offset + v_idx - adapter->q_vector_off;
		node = cpu_to_node(cpu);

		//printk("bind cpu %d node %d %d\n", cpu, node, num_online_cpus());
#endif
	}

	/* allocate q_vector and rings */
	q_vector = kzalloc_node(size, GFP_KERNEL, node);
	if (!q_vector)
		q_vector = kzalloc(size, GFP_KERNEL);
	if (!q_vector)
		return -ENOMEM;

		/* setup affinity mask and node */
#ifdef HAVE_IRQ_AFFINITY_HINT
	if (cpu != -1) {
		cpumask_set_cpu(cpu, &q_vector->affinity_mask);
	}
#endif
	q_vector->numa_node = node;

#ifdef CONFIG_RNP_DCA
	/* initialize CPU for DCA */
	q_vector->cpu = -1;

#endif

	/* initialize nap */
	netif_napi_add(adapter->netdev, &q_vector->napi, rnp_poll,
		       adapter->napi_budge);

	/* tie q_vector and adapter together */
	adapter->q_vector[v_idx - adapter->q_vector_off] = q_vector;
	q_vector->adapter = adapter;
	q_vector->v_idx = v_idx;

	/* initialize work limits */
	q_vector->tx.work_limit = adapter->tx_work_limit;

	/* initialize pointer to rings */
	ring = q_vector->ring;

	for (idx = 0; idx < txr_count; idx++) {
		/* assign generic ring traits */
		ring->dev = pci_dev_to_dev(adapter->pdev);
		ring->netdev = adapter->netdev;

		/* configure backlink on ring */
		ring->q_vector = q_vector;

		/* update q_vector Tx values */
		rnp_add_ring(ring, &q_vector->tx);

		/* apply Tx specific ring traits */
		ring->count = adapter->tx_ring_item_count;
		if (adapter->flags & RNP_FLAG_DCB_ENABLED) {
			int rss_i;

			rss_i = adapter->ring_feature[RING_F_RSS].indices;
			/* in dcb mode should assign rss */
			ring->queue_index = eth_queue_idx + idx * rss_i;
			//printk("dcb : queue_index is %d\n", ring->queue_index);
		} else {
			ring->queue_index = eth_queue_idx + idx;
		}
		/* rnp_queue_idx can be changed after */
		/* it is used to location hw reg */
		ring->rnp_queue_idx = txr_idx;
		ring->ring_addr = dma->dma_ring_addr + RING_OFFSET(txr_idx);
		ring->dma_int_stat =
			ring->ring_addr + RNP_DMA_INT_STAT;
		ring->dma_int_mask = ring->ring_addr + RNP_DMA_INT_MASK;
		ring->dma_int_clr = ring->ring_addr + RNP_DMA_INT_CLR;
		ring->device_id = adapter->pdev->device;
		ring->pfvfnum = hw->pfvfnum;
		/* n10 should skip tx start control */
		if (hw->hw_type == rnp_hw_n10) {
			ring->ring_flags |= RNP_RING_SKIP_TX_START;	
		}
		if (hw->hw_type == rnp_hw_n500) {
			/* n500 not support tunnel */
			ring->ring_flags |= RNP_RING_NO_TUNNEL_SUPPORT;
			/* n500 fixed ring size change from large to small */
			ring->ring_flags |= RNP_RING_SIZE_CHANGE_FIX;

		}
		/* assign ring to adapter */
		adapter->tx_ring[ring->queue_index] = ring;

		/* update count and index */
		txr_idx += step;

		rnp_dbg("\t\t%s:vector[%d] <--RNP TxRing:%d, eth_queue:%d\n",
			adapter->name, v_idx, ring->rnp_queue_idx,
			ring->queue_index);

		/* push pointer to next ring */
		ring++;
	}

	for (idx = 0; idx < rxr_count; idx++) {
		/* assign generic ring traits */
		ring->dev = pci_dev_to_dev(adapter->pdev);
		ring->netdev = adapter->netdev;

		/* configure backlink on ring */
		ring->q_vector = q_vector;

		/* update q_vector Rx values */
		rnp_add_ring(ring, &q_vector->rx);

		/* apply Rx specific ring traits */
		ring->count = adapter->rx_ring_item_count;
		/* rnp_queue_idx can be changed after */
		/* it is used to location hw reg */
		if (adapter->flags & RNP_FLAG_DCB_ENABLED) {
			int rss_i;

			rss_i = adapter->ring_feature[RING_F_RSS].indices;
			/* in dcb mode should assign rss */
			ring->queue_index = eth_queue_idx + idx * rss_i;
			//printk("dcb : queue_index is %d\n", ring->queue_index);
		} else {
			ring->queue_index = eth_queue_idx + idx;
		}
		ring->rnp_queue_idx = rxr_idx;
		ring->ring_addr = dma->dma_ring_addr + RING_OFFSET(rxr_idx);
		ring->dma_int_stat =
			ring->ring_addr + RNP_DMA_INT_STAT;
		ring->dma_int_mask = ring->ring_addr + RNP_DMA_INT_MASK;
		ring->dma_int_clr = ring->ring_addr + RNP_DMA_INT_CLR;
		ring->device_id = adapter->pdev->device;
		ring->pfvfnum = hw->pfvfnum;
		if (hw->hw_type == rnp_hw_n10) {

		} else if (hw->hw_type == rnp_hw_n500) {
			/* n500 fixed ring size change from large to small */
			ring->ring_flags |= RNP_RING_SIZE_CHANGE_FIX;
			ring->ring_flags |= RNP_RING_SCATER_SETUP;
		}

		/* assign ring to adapter */
		adapter->rx_ring[ring->queue_index] = ring;
		rnp_dbg("\t\t%s:vector[%d] <--RNP RxRing:%d, eth_queue:%d\n",
			adapter->name, v_idx, ring->rnp_queue_idx,
			ring->queue_index);

		/* update count and index */
		rxr_idx += step;

		/* push pointer to next ring */
		ring++;
	}
	if (hw->hw_type == rnp_hw_n10) {
		q_vector->vector_flags |= RNP_QVECTOR_FLAG_IRQ_MISS_CHECK;
		q_vector->new_rx_count = adapter->rx_frames;
		q_vector->old_rx_count = adapter->rx_frames;
		/* initialize timer */
		q_vector->irq_check_usecs = 1000;
		//hrtimer_init(&q_vector->irq_miss_check_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		hrtimer_init(&q_vector->irq_miss_check_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
		q_vector->irq_miss_check_timer.function = irq_miss_check; /* initialize NAPI */

	}

	return 0;
}

/**
 * rnp_free_q_vector - Free memory allocated for specific interrupt vector
 * @adapter: board private structure to initialize
 * @v_idx: Index of vector to be freed
 *
 * This function frees the memory allocated to the q_vector.  In addition if
 * NAPI is enabled it will delete any references to the NAPI struct prior
 * to freeing the q_vector.
 **/
static void rnp_free_q_vector(struct rnp_adapter *adapter, int v_idx)
{
	struct rnp_q_vector *q_vector = adapter->q_vector[v_idx];
	struct rnp_ring *ring;

	rnp_dbg("v_idx:%d\n", v_idx);

	rnp_for_each_ring(ring, q_vector->tx)
		adapter->tx_ring[ring->queue_index] = NULL;

	rnp_for_each_ring(ring, q_vector->rx)
		adapter->rx_ring[ring->queue_index] = NULL;

	adapter->q_vector[v_idx] = NULL;
	netif_napi_del(&q_vector->napi);

	// must stop timer?
	if (q_vector->vector_flags & RNP_QVECTOR_FLAG_IRQ_MISS_CHECK)
		hrtimer_cancel(&q_vector->irq_miss_check_timer);
	
	/*
	 * rnp_get_stats64() might access the rings on this vector,
	 * we must wait a grace period before freeing it.
	 */
	kfree_rcu(q_vector, rcu);
}

/**
 * rnp_alloc_q_vectors - Allocate memory for interrupt vectors
 * @adapter: board private structure to initialize
 *
 * We allocate one q_vector per queue interrupt.  If allocation fails we
 * return -ENOMEM.
 **/
static int rnp_alloc_q_vectors(struct rnp_adapter *adapter)
{
	int v_idx = adapter->q_vector_off;
	int ring_idx = 0;
	int r_remaing =
		min_t(int, adapter->num_tx_queues, adapter->num_rx_queues);
	int ring_step = 1; //only 2port support
	int err, ring_cnt, v_remaing = adapter->num_q_vectors;
	int q_vector_nums = 0;
	struct rnp_hw *hw = &adapter->hw;

	if (adapter->flags & RNP_FLAG_SRIOV_ENABLED) {
		ring_idx = 0;
		// use 1ring  when vf  enabled
		/* only 2 rings when sriov enabled */
		/* from back */
#ifdef FIX_VF_BUG
		// this mode pf use vf 0 ring
		ring_idx = 0;
		r_remaing = hw->sriov_ring_limit;
		
#else
		ring_idx = adapter->max_ring_pair_counts -
			   ring_step * hw->sriov_ring_limit;
		r_remaing = hw->sriov_ring_limit;
#endif
	}

	adapter->eth_queue_idx = 0;
	BUG_ON(adapter->num_q_vectors == 0);

	if (adapter->flags & RNP_FLAG_DCB_ENABLED) {
		rnp_dbg("in dcb mode r_remaing %d, num_q_vectors %d\n", r_remaing,
			v_remaing);
	}

	rnp_dbg("r_remaing:%d, ring_step:%d num_q_vectors:%d\n", r_remaing,
		ring_step, v_remaing);

	/* can support muti rings in one q_vector */
	for (; r_remaing > 0 && v_remaing > 0; v_remaing--) {
		// todo fix in tcnum > 1 mode 
		// one q_vector assign tc0 ~ tc3
		// ring_cnt should no more than 4
		ring_cnt = DIV_ROUND_UP(r_remaing, v_remaing);
		if (adapter->flags & RNP_FLAG_DCB_ENABLED) {
			BUG_ON(ring_cnt != adapter->num_tc);
			//printk("dcb ring_cnt is %d\n", ring_cnt);
		}

		err = rnp_alloc_q_vector(adapter, adapter->eth_queue_idx, v_idx,
					 ring_idx, ring_cnt, ring_step);
		if (err)
			goto err_out;
		ring_idx += ring_step * ring_cnt;
		r_remaing -= ring_cnt;
		v_idx++;
		q_vector_nums++;
		/* dcb mode only add 1 */
		if (adapter->flags & RNP_FLAG_DCB_ENABLED)
			adapter->eth_queue_idx += 1;
		else
			adapter->eth_queue_idx += ring_cnt;
	}
	/* should fix the real used q_vectors_nums */
	adapter->num_q_vectors = q_vector_nums;

	return 0;

err_out:
	adapter->num_tx_queues = 0;
	adapter->num_rx_queues = 0;
	adapter->num_q_vectors = 0;

	while (v_idx--)
		rnp_free_q_vector(adapter, v_idx);

	return -ENOMEM;
}

/**
 * rnp_free_q_vectors - Free memory allocated for interrupt vectors
 * @adapter: board private structure to initialize
 *
 * This function frees the memory allocated to the q_vectors.  In addition if
 * NAPI is enabled it will delete any references to the NAPI struct prior
 * to freeing the q_vector.
 **/
static void rnp_free_q_vectors(struct rnp_adapter *adapter)
{
	int v_idx = adapter->num_q_vectors;

	adapter->num_rx_queues = 0;
	adapter->num_tx_queues = 0;
	adapter->num_q_vectors = 0;

	while (v_idx--)
		rnp_free_q_vector(adapter, v_idx);
}

static void rnp_reset_interrupt_capability(struct rnp_adapter *adapter)
{
	if (!adapter->dma2_in_1pf)
		pci_disable_msix(adapter->pdev);
	kfree(adapter->msix_entries);
	adapter->msix_entries = NULL;
	adapter->q_vector_off = 0;
}

/**
 * rnp_set_interrupt_capability - set MSI-X or MSI if supported
 * @adapter: board private structure to initialize
 *
 * Attempt to configure the interrupts using the best available
 * capabilities of the hardware and the kernel.
 **/
static int rnp_set_interrupt_capability(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	int vector, v_budget, err;
	v_budget = min_t(int, adapter->num_tx_queues, adapter->num_rx_queues);

	//v_budget = adapter->max_ring_pair_counts;
	/* in one ring mode should reset v_budget */
#ifdef RNP_MAX_RINGS
	v_budget = min_t(int, v_budget, RNP_MAX_RINGS);
#else
	v_budget = min_t(int, v_budget, num_online_cpus());
#endif
	v_budget += adapter->num_other_vectors;

	v_budget = min_t(int, v_budget, hw->mac.max_msix_vectors);

	adapter->msix_entries =
		kcalloc(v_budget, sizeof(struct msix_entry), GFP_KERNEL);

	if (!adapter->msix_entries) {
		rnp_err("alloc msix_entries faild!\n");
		return -EINVAL;
	}
	dbg("[%s] adapter:%p msix_entry:%p\n", __func__, adapter,
	    adapter->msix_entries);

	for (vector = 0; vector < v_budget; vector++)
		adapter->msix_entries[vector].entry = vector;

	err = rnp_acquire_msix_vectors(adapter, v_budget);
	if (err)
		goto out;

	adapter->q_vector_off = 0;
	if (adapter->num_other_vectors) // vector0 reversed for mbx
		adapter->q_vector_off = 1;

	rnp_dbg("adapter%d alloc vectors: cnt:%d [%d~%d] num_q_vectors:%d\n",
			adapter->bd_number,
			v_budget,
			adapter->q_vector_off,
			adapter->q_vector_off + v_budget - 1,
			adapter->num_q_vectors);
out:
	return err;
}


void rnp_print_ring_info(struct rnp_adapter *adapter)
{

	int i;
	struct rnp_ring *ring;
	struct rnp_q_vector *q_vector;

	rnp_dbg("tx_queue count %d\n", adapter->num_tx_queues);
	rnp_dbg("queue-mapping :\n");
	for (i = 0; i < adapter->num_tx_queues; i++) {
		ring = adapter->tx_ring[i];
		rnp_dbg(" queue %d , physical ring %d\n", i, ring->rnp_queue_idx);

	}
	rnp_dbg("rx_queue count %d\n", adapter->num_rx_queues);
	rnp_dbg("queue-mapping :\n");
	for (i = 0; i < adapter->num_rx_queues; i++) {
		ring = adapter->rx_ring[i];
		rnp_dbg(" queue %d , physical ring %d\n", i, ring->rnp_queue_idx);
	}
	rnp_dbg("q_vector count %d\n", adapter->num_q_vectors);
	rnp_dbg("vector-queue mapping:\n");
	for (i = 0; i < adapter->num_q_vectors; i++) {
		q_vector = adapter->q_vector[i];
		rnp_dbg("vector %d \n", i);
		rnp_for_each_ring(ring, q_vector->tx) {
			rnp_dbg(" tx physical ring %d\n",  ring->rnp_queue_idx);
		}
		rnp_for_each_ring(ring, q_vector->rx) {
			rnp_dbg(" rx physical ring %d\n",  ring->rnp_queue_idx);
		}
	}


}

/**
 * rnp_init_interrupt_scheme - Determine proper interrupt scheme
 * @adapter: board private structure to initialize
 *
 * We determine which interrupt scheme to use based on...
 * - Hardware queue count (num_*_queues)
 *   - defined by miscellaneous hardware support/features (RSS, etc.)
 **/
int rnp_init_interrupt_scheme(struct rnp_adapter *adapter)
{
	int err;

	/* Number of supported queues */
	rnp_set_num_queues(adapter);

	/* Set interrupt mode */
	err = rnp_set_interrupt_capability(adapter);
	if (err) {
		e_dev_err("Unable to get interrupt\n");
		goto err_set_interrupt;
	}

	err = rnp_alloc_q_vectors(adapter);
	if (err) {
		e_dev_err("Unable to allocate memory for queue vectors\n");
		goto err_alloc_q_vectors;
	}
	rnp_cache_ring_register(adapter);

	// printk now qvctor ring setup 
	DPRINTK(PROBE, INFO, 
		"Multiqueue %s: Rx Queue count = %u, Tx Queue count = %u\n\n",
		(adapter->num_rx_queues > 1) ? "Enabled" : "Disabled",
		adapter->num_rx_queues, adapter->num_tx_queues);
	rnp_print_ring_info(adapter);

	set_bit(__RNP_DOWN, &adapter->state);

	return 0;

err_alloc_q_vectors:
	rnp_reset_interrupt_capability(adapter);
err_set_interrupt:
	;
	return err;
}

/**
 * rnp_clear_interrupt_scheme - Clear the current interrupt scheme settings
 * @adapter: board private structure to clear interrupt scheme on
 *
 * We go through and clear interrupt specific resources and reset the structure
 * to pre-load conditions
 **/
void rnp_clear_interrupt_scheme(struct rnp_adapter *adapter)
{
	adapter->num_tx_queues = 0;
	adapter->num_rx_queues = 0;

	rnp_free_q_vectors(adapter);
	rnp_reset_interrupt_capability(adapter);
}

/**
 * rnp_tx_ctxtdesc - Send a control desc to hw
 * @tx_ring: target ring of this control desc
 * @mss_seg_len: mss length
 * @l4_hdr_len:  l4 length
 * @tunnel_hdr_len: tunnel_hdr_len
 * @inner_vlan_tag: inner_vlan_tag
 * @type_tucmd: cmd
 *
 **/

void rnp_tx_ctxtdesc(struct rnp_ring *tx_ring, u32 mss_len_vf_num,
		     u32 inner_vlan_tunnel_len, int ignore_vlan,
		     bool crc_pad)
{
	struct rnp_tx_ctx_desc *context_desc;
	u16 i = tx_ring->next_to_use;
	struct rnp_adapter *adapter = RING2ADAPT(tx_ring);
	u32 type_tucmd = 0;

	context_desc = RNP_TX_CTXTDESC(tx_ring, i);

	i++;
	tx_ring->next_to_use = (i < tx_ring->count) ? i : 0;

	/* set bits to identify this as an advanced context descriptor */
	//type_tucmd |= RNP_TXD_CMD_RS | RNP_TXD_CTX_CTRL_DESC;
	type_tucmd |= RNP_TXD_CTX_CTRL_DESC;

	// set mac padding
#ifdef FIX_MAC_PADDIN
	if (!crc_pad)
		type_tucmd |= RNP_TXD_MTI_CRC_PAD_CTRL;	// close mac padding
#endif	

	if (inner_vlan_tunnel_len & 0x00ffff00) {
		/* if a inner vlan */
		type_tucmd |= RNP_TXD_CMD_INNER_VLAN;
	}

	context_desc->mss_len_vf_num = cpu_to_le32(mss_len_vf_num);
	context_desc->inner_vlan_tunnel_len =
		cpu_to_le32(inner_vlan_tunnel_len);
	context_desc->resv_cmd = cpu_to_le32(type_tucmd);
	if (tx_ring->q_vector->adapter->flags & RNP_FLAG_SRIOV_ENABLED) {
		if (ignore_vlan)
			context_desc->inner_vlan_tunnel_len |= VF_VEB_IGNORE_VLAN;
	}
	buf_dump_line("ctx  ", __LINE__, context_desc, sizeof(*context_desc));
}
void rnp_maybe_tx_ctxtdesc(struct rnp_ring *tx_ring,
			   struct rnp_tx_buffer *first, u32 ignore_vlan)
{
	/* sriov mode pf use the last vf */
	if (first->ctx_flag) {
#ifdef FIX_MAC_PADDIN
		rnp_tx_ctxtdesc(tx_ring, first->mss_len_vf_num,
				first->inner_vlan_tunnel_len, ignore_vlan,
				first->gso_need_padding);
#else

		rnp_tx_ctxtdesc(tx_ring, first->mss_len_vf_num,
				first->inner_vlan_tunnel_len, ignore_vlan,
				false);
#endif

	}
}

void rnp_store_reta(struct rnp_adapter *adapter)
{
	u32 i, reta_entries = rnp_rss_indir_tbl_entries(adapter);
	struct rnp_hw *hw = &adapter->hw;
	u32 reta = 0;
	u8 *indir_tbl = adapter->rss_indir_tbl;
	/* relative with rss table */
	u32 port = adapter->hw.nr_lane;
	struct rnp_ring *rx_ring;
	printk("%s port:%d nr_lane:%d\n",
		   __func__,
		   adapter->bd_number,
		   adapter->hw.nr_lane);
	
	/* Write redirection table to HW */
	for (i = 0; i < reta_entries; i++) {
		if (adapter->flags & RNP_FLAG_SRIOV_ENABLED)
			reta = adapter->rss_indir_tbl[i];
		else {
			rx_ring = adapter->rx_ring[adapter->rss_indir_tbl[i]];
			reta = rx_ring->rnp_queue_idx;
		}
		/*
		if (hw->rss_type == rnp_rss_uv3p)
			wr32(hw, RNP_ETH_RSS_INDIR_TBL_UV3P(i), reta);
		else if (hw->rss_type == rnp_rss_uv440)
			wr32(hw, RNP_ETH_RSS_INDIR_TBL(port, i), reta);
		else if (hw->rss_type == rnp_rss_n10)
			wr32(hw, RNP_ETH_RSS_INDIR_TBL(port, i), reta);
		*/
		hw->rss_indir_tbl[i] = reta;
	}
	hw->ops.set_rss_table(hw);
}

void rnp_store_key(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	bool sriov_flag = !!(adapter->flags & RNP_FLAG_SRIOV_ENABLED);

	hw->ops.set_rss_key(hw, sriov_flag);
}

int rnp_init_rss_key(struct rnp_adapter *adapter)
{
	int i;
	struct rnp_hw *hw = &adapter->hw;
	//u32 iov_en =
		//(adapter->flags & RNP_FLAG_SRIOV_ENABLED) ? RNP_IOV_ENABLED : 0;
	bool sriov_flag = !!(adapter->flags & RNP_FLAG_SRIOV_ENABLED);

	/* only init rss key once */
	/* no change rss key if user input one */
	if (!adapter->rss_key_setup_flag) {
		netdev_rss_key_fill(adapter->rss_key, RNP_RSS_KEY_SIZE);
		adapter->rss_key_setup_flag = 1;
	}
	hw->ops.set_rss_key(hw, sriov_flag);
	//rnp_store_key(adapter);
	/* open rss if rx hash is open ? */
	//wr32(hw, RNP_ETH_RSS_CONTROL, RNP_ETH_ENABLE_RSS_ONLY | iov_en);

	return 0;
}

int rnp_init_rss_table(struct rnp_adapter *adapter)
{
	int rx_nums = adapter->num_rx_queues;
	int i, j;
	struct rnp_hw *hw = &adapter->hw;
	struct rnp_ring *rx_ring;
	u32 reta = 0;
	u32 reta_entries = rnp_rss_indir_tbl_entries(adapter);
	u32 port = adapter->hw.nr_lane;


	if (adapter->flags & RNP_FLAG_DCB_ENABLED) {
		rx_nums = rx_nums / adapter->num_tc;
		for (i = 0, j = 0; i < 8; i++) {
			
			//wr32(hw, RNP_ETH_TC_IPH_OFFSET_TABLE(i), j);
			adapter->rss_tc_tbl[i] = j;
			hw->rss_tc_tbl[i] = j;
			j = (j + 1) % adapter->num_tc;
		}
	} else {
		for (i = 0, j = 0; i < 8; i++) {
			//wr32(hw, RNP_ETH_TC_IPH_OFFSET_TABLE(i), 0);
			hw->rss_tc_tbl[i] = 0;
			adapter->rss_tc_tbl[i] = 0;
		}
	}

	/* adapter->num_q_vectors is not correct */
	for (i = 0, j = 0; i < reta_entries; i++) {
		/* init with default value */
		if (!adapter->rss_tbl_setup_flag)
			adapter->rss_indir_tbl[i] = j;

		if (adapter->flags & RNP_FLAG_SRIOV_ENABLED) {
			/* in sriov mode reta in [0, rx_nums] */
			reta = j;
		} else {
			/* in no sriov, reta is real ring number */
			rx_ring = adapter->rx_ring[adapter->rss_indir_tbl[i]];
			reta = rx_ring->rnp_queue_idx;
		}
		/* rss table should add ring_offset */
		/*
		if (hw->rss_type == rnp_rss_uv3p) {
			wr32(hw, RNP_ETH_RSS_INDIR_TBL_UV3P(i), reta);
			wr32(hw, RNP_ETH_RSS_MODE, 6);
		} else if (hw->rss_type == rnp_rss_uv440) {
			wr32(hw, RNP_ETH_RSS_INDIR_TBL(port, i), reta);
		} else if (hw->rss_type == rnp_rss_n10) {
			wr32(hw, RNP_ETH_RSS_INDIR_TBL(port, i), reta);
		} */
		/* store rss_indir_tbl */
		//adapter->rss_indir_tbl[i] = reta;
		hw->rss_indir_tbl[i] = reta;

		j = (j + 1) % rx_nums;
	}
	/* tbl only init once */
	adapter->rss_tbl_setup_flag = 1;

	hw->ops.set_rss_table(hw);
	/*
	for (i = 0, j = 0; i < reta_entries; i++) {
		dbg("indir %d table is %d\n", i, adapter->rss_indir_tbl[i]);
		if (hw->rss_type == rnp_rss_uv3p)
			dbg("reg %x is %d\n", RNP_ETH_RSS_INDIR_TBL_UV3P(i),
			    rd32(hw, RNP_ETH_RSS_INDIR_TBL_UV3P(i)));
		else
			dbg("reg %x is %d\n", RNP_ETH_RSS_INDIR_TBL(port, i),
			    rd32(hw, RNP_ETH_RSS_INDIR_TBL(port, i)));
	}*/
	return 0;
}

void rnp_setup_dma_rx(struct rnp_adapter *adapter, int count_in_dw)
{
	struct rnp_hw *hw = &adapter->hw;
	u32 data;

	data = rd32(hw, RNP_DMA_CONFIG);
	data &= (0x00000ffff);
	data |= (count_in_dw << 16);
	wr32(hw, RNP_DMA_CONFIG, data);
}

u16 rnp_setup_layer2_pritologic(u16 hw_id)
{
	//return (RNP_MAX_LAYER2_FILTERS - 1 - hw_id); 
	return hw_id;
}

u16 rnp_setup_tuple5_pritologic(u16 hw_id)
{
	//return (RNP_MAX_TUPLE5_FILTERS - 1 - hw_id);
	return hw_id;
}

u16 rnp_setup_tuple5_pritologic_tcam(u16 pri_id)
{
	/*
	int i;
	int hw_id = RNP_MAX_TCAM_FILTERS - 1;
	int step = 32;
	for (i = 0; i < pri_id; i++) {
		hw_id -= step;
		if (hw_id < 0) {
			hw_id = hw_id + RNP_MAX_TCAM_FILTERS - 1;
		}
	}
	*/
	int i;
	int hw_id = 0;
	int step = 32;
	for (i = 0; i < pri_id; i++) {
		hw_id += step;
		if (hw_id > RNP_MAX_TCAM_FILTERS) {
			hw_id = hw_id - RNP_MAX_TCAM_FILTERS + 1;
		}
	}

	return hw_id;
}
void rnp_setup_layer2_remapping(struct rnp_hw *hw, union rnp_atr_input *input,
				u16 pri_id, u8 queue)
{
	//printk("try to eable layer2 %x\n",input->layer2_formate.proto);
	/* enable l2 proto setup */
	//rnp_set_reg_bit(hw, RNP_ETH_VLAN_FILTER_ENABLE, 31);
	u16 hw_id;

	hw_id = rnp_setup_layer2_pritologic(pri_id);
	/* enable layer2 */
	wr32(hw, RNP_ETH_LAYER2_ETQF(hw_id),
	     (0x1 << 31) | (ntohs(input->layer2_formate.proto)));

	/* setup action */
	if (queue == RNP_FDIR_DROP_QUEUE) {
		wr32(hw, RNP_ETH_LAYER2_ETQS(hw_id), (0x1 << 31));
	} else {
		/* setup ring_number */
		wr32(hw, RNP_ETH_LAYER2_ETQS(hw_id),
		     (0x1 << 30) | (queue << 20));
	}
}

void rnp_setup_tuple5_remapping(struct rnp_hw *hw, union rnp_atr_input *input,
				u16 pri_id, u8 queue)
{
	u32 port = 0;
	u8 mask_temp = 0;
	u8 l4_proto_type = 0;
	u16 hw_id;

	//setup 

	hw_id = rnp_setup_tuple5_pritologic(pri_id);	
	dbg("try to eable tuple 5 %x\n", hw_id);
	if (input->formatted.src_ip[0] != 0) {
		wr32(hw, RNP_ETH_TUPLE5_SAQF(hw_id),
		     htonl(input->formatted.src_ip[0]));
	} else {
		mask_temp |= RNP_SRC_IP_MASK;
	}
	if (input->formatted.dst_ip[0] != 0) {
		wr32(hw, RNP_ETH_TUPLE5_DAQF(hw_id),
		     htonl(input->formatted.dst_ip[0]));
	} else
		mask_temp |= RNP_DST_IP_MASK;
	if (input->formatted.src_port != 0)
		port |= (htons(input->formatted.src_port));
	else
		mask_temp |= RNP_SRC_PORT_MASK;
	if (input->formatted.dst_port != 0)
		port |= (htons(input->formatted.dst_port) << 16);
	else
		mask_temp |= RNP_DST_PORT_MASK;

	if (port != 0)
		wr32(hw, RNP_ETH_TUPLE5_SDPQF(hw_id), port);

	switch (input->formatted.flow_type) {
	case RNP_ATR_FLOW_TYPE_TCPV4:
		l4_proto_type = IPPROTO_TCP;
		break;
	case RNP_ATR_FLOW_TYPE_UDPV4:
		l4_proto_type = IPPROTO_UDP;
		break;
	case RNP_ATR_FLOW_TYPE_SCTPV4:
		l4_proto_type = IPPROTO_SCTP;
		break;
	case RNP_ATR_FLOW_TYPE_IPV4:
		l4_proto_type = input->formatted.inner_mac[0];
		break;
	default:
		l4_proto_type = 0;
	}

	if (l4_proto_type == 0)
		mask_temp |= RNP_L4_PROTO_MASK;

	/* setup ftqf*/
	/* always set 0x3 */
	wr32(hw, RNP_ETH_TUPLE5_FTQF(hw_id),
	     (1 << 31) | (mask_temp << 25) | (l4_proto_type << 16) | 0x3);

	/* setup action */
	if (queue == RNP_FDIR_DROP_QUEUE) {
		wr32(hw, RNP_ETH_TUPLE5_POLICY(hw_id), (0x1 << 31));
	} else {
		/* setup ring_number */
		wr32(hw, RNP_ETH_TUPLE5_POLICY(hw_id),
		     ((0x1 << 30) | (queue << 20)));
	}
}

void rnp_setup_tuple5_remapping_tcam(struct rnp_hw *hw,
				     union rnp_atr_input *input, u16 pri_id,
				     u8 queue)
{
	u32 port = 0;
	u32 port_mask = 0;
	u8 l4_proto_type = 0;
	u8 l4_proto_mask = 0xff;
	u32 action = 0;
	u32 mark = 0;
	u16 hw_id;

	hw_id = rnp_setup_tuple5_pritologic_tcam(pri_id);
	wr32(hw, RNP_TCAM_MODE, 2);
	//wr32(hw, RNP_TCAM_CACHE_ENABLE, 0);
	dbg("try to eable tcam %d\n", hw_id);
	if (input->formatted.src_ip[0] != 0) {
		wr32(hw, RNP_TCAM_SAQF(hw_id),
		     htonl(input->formatted.src_ip[0]));
		wr32(hw, RNP_TCAM_SAQF_MASK(hw_id),
		     htonl(input->formatted.src_ip_mask[0]));

		dbg("tcam src ip 0%x ---> 0x%x\n", htonl(input->formatted.src_ip[0]),
				RNP_TCAM_SAQF(hw_id));
		dbg("tcam src ip mask 0%x ---> 0x%x\n", htonl(input->formatted.src_ip_mask[0]),
				RNP_TCAM_SAQF_MASK(hw_id));
	} else {
		wr32(hw, RNP_TCAM_SAQF(hw_id), 0);
		wr32(hw, RNP_TCAM_SAQF_MASK(hw_id), 0);
		dbg("tcam src ip 0%x ---> 0x%x\n", 0,
				RNP_TCAM_SAQF(hw_id));
		dbg("tcam src ip mask 0%x ---> 0x%x\n", 0,
				RNP_TCAM_SAQF_MASK(hw_id));
	}
	if (input->formatted.dst_ip[0] != 0) {
		wr32(hw, RNP_TCAM_DAQF(hw_id),
		     htonl(input->formatted.dst_ip[0]));
		wr32(hw, RNP_TCAM_DAQF_MASK(hw_id),
		     htonl(input->formatted.dst_ip_mask[0]));
		dbg("tcam dst ip 0%x ---> 0x%x\n", htonl(input->formatted.dst_ip[0]),
				RNP_TCAM_DAQF(hw_id));
		dbg("tcam dst ip mask 0%x ---> 0x%x\n", htonl(input->formatted.dst_ip_mask[0]),
				RNP_TCAM_DAQF_MASK(hw_id));
	} else {
		wr32(hw, RNP_TCAM_DAQF(hw_id), 0);
		wr32(hw, RNP_TCAM_DAQF_MASK(hw_id), 0);
		dbg("tcam dst ip 0%x ---> 0x%x\n", 0,
				RNP_TCAM_DAQF(hw_id));
		dbg("tcam dst ip mask 0%x ---> 0x%x\n", 0,
				RNP_TCAM_DAQF_MASK(hw_id));
	}
	if (input->formatted.src_port != 0) {
		port |= (htons(input->formatted.src_port) << 16);
		port_mask |= (htons(input->formatted.src_port_mask) << 16);

	} else {
	}
	if (input->formatted.dst_port != 0) {
		port |= (htons(input->formatted.dst_port));
		port_mask |= (htons(input->formatted.dst_port_mask));
	} else {
	}

	/* setup src & dst port */
	if (port != 0) {
		wr32(hw, RNP_TCAM_SDPQF(hw_id), port);
		wr32(hw, RNP_TCAM_SDPQF_MASK(hw_id), port_mask);

		dbg("tcam port 0%x ---> 0x%x\n", port,
				RNP_TCAM_SDPQF(hw_id));
		dbg("tcam port mask 0%x ---> 0x%x\n", port_mask,
				RNP_TCAM_SDPQF_MASK(hw_id));
	} else {
		wr32(hw, RNP_TCAM_SDPQF(hw_id), 0);
		wr32(hw, RNP_TCAM_SDPQF_MASK(hw_id), 0);
		dbg("tcam port 0%x ---> 0x%x\n", port,
				RNP_TCAM_SDPQF(hw_id));
		dbg("tcam port mask 0%x ---> 0x%x\n", port_mask,
				RNP_TCAM_SDPQF_MASK(hw_id));
	}

	switch (input->formatted.flow_type) {
	case RNP_ATR_FLOW_TYPE_TCPV4:
		l4_proto_type = IPPROTO_TCP;
		break;
	case RNP_ATR_FLOW_TYPE_UDPV4:
		l4_proto_type = IPPROTO_UDP;
		break;
	case RNP_ATR_FLOW_TYPE_SCTPV4:
		l4_proto_type = IPPROTO_SCTP;
		break;
	case RNP_ATR_FLOW_TYPE_IPV4:
		l4_proto_type = input->formatted.inner_mac[0];
		l4_proto_mask = input->formatted.inner_mac_mask[0];
		break;
	default:
		l4_proto_type = 0;
		l4_proto_mask = 0;
	}

	if (l4_proto_type != 0) {
		action |= l4_proto_type;
		mark |= l4_proto_mask;
	} else {
	}

	/* setup action */
	if (queue == RNP_FDIR_DROP_QUEUE) {
		wr32(hw, RNP_TCAM_APQF(hw_id), (0x1 << 31) | action);
		wr32(hw, RNP_TCAM_APQF_MASK(hw_id), mark);
		dbg("tcam action 0%x ---> 0x%x\n", (0x1 << 31) | action,
				RNP_TCAM_APQF(hw_id));
		dbg("tcam action mask 0%x ---> 0x%x\n", mark,
				RNP_TCAM_APQF_MASK(hw_id));
	} else {
		/* setup ring_number */
		wr32(hw, RNP_TCAM_APQF(hw_id),
		     ((0x1 << 30) | (queue << 16) | action));
		wr32(hw, RNP_TCAM_APQF_MASK(hw_id), mark);

		dbg("tcam action 0%x ---> 0x%x\n", (0x1 << 30) | (queue << 16) | action,
				RNP_TCAM_APQF(hw_id));
		dbg("tcam action mask 0%x ---> 0x%x\n", mark,
				RNP_TCAM_APQF_MASK(hw_id));
	}
	wr32(hw, RNP_TCAM_MODE, 1);
}

/* setup to the hw  */
s32 rnp_fdir_write_perfect_filter(int fdir_mode, struct rnp_hw *hw,
				  union rnp_atr_input *filter, u16 hw_id,
				  u8 queue)
{
	if (filter->formatted.flow_type == RNP_ATR_FLOW_TYPE_ETHER) {
		hw->ops.set_layer2_remapping(hw, filter, hw_id, queue);
		//rnp_setup_layer2_remapping(hw, filter, hw_id, queue);
	} else {
		hw->ops.set_tuple5_remapping(hw, filter, hw_id, queue);
		/*if (fdir_mode != fdir_mode_tcam) {
			rnp_setup_tuple5_remapping(hw, filter, hw_id, queue);
		} else {
			rnp_setup_tuple5_remapping_tcam(hw, filter, hw_id,
							queue);
		}*/
	}

	return 0;
}

s32 rnp_fdir_erase_perfect_filter(int fdir_mode, struct rnp_hw *hw,
				  union rnp_atr_input *input, u16 pri_id)
{
	//u16 hw_id;
	/* just diable filter */
	if (input->formatted.flow_type == RNP_ATR_FLOW_TYPE_ETHER) {
		hw->ops.clr_layer2_remapping(hw, pri_id); 
		//hw_id = rnp_setup_layer2_pritologic(pri_id);
		//wr32(hw, RNP_ETH_LAYER2_ETQF(hw_id), 0);
		dbg("disble layer2 %d\n", pri_id);
	} else {
		hw->ops.clr_tuple5_remapping(hw, pri_id);
		dbg("disble tuple5 %d\n", pri_id);
//		if (fdir_mode != fdir_mode_tcam) {
//			hw_id = rnp_setup_tuple5_pritologic(pri_id);	
//			wr32(hw, RNP_ETH_TUPLE5_FTQF(hw_id), 0);
//			dbg("disble tuple5 %d\n", hw_id);
//		} else {
//
//			hw_id = rnp_setup_tuple5_pritologic_tcam(pri_id);
//			dbg("disble tcam tuple5 %d\n", hw_id);
//			/* earase tcam */
//			wr32(hw, RNP_TCAM_MODE, 2);
//			//wr32(hw, RNP_TCAM_CACHE_ENABLE, 0);
//
//			wr32(hw, RNP_TCAM_SAQF(hw_id), 0);
//			wr32(hw, RNP_TCAM_SAQF_MASK(hw_id), 0);
//
//			wr32(hw, RNP_TCAM_DAQF(hw_id), 0);
//			wr32(hw, RNP_TCAM_DAQF_MASK(hw_id), 0);
//
//			wr32(hw, RNP_TCAM_SDPQF(hw_id), 0);
//			wr32(hw, RNP_TCAM_SDPQF_MASK(hw_id), 0);
//
//			wr32(hw, RNP_TCAM_APQF(hw_id), 0);
//			wr32(hw, RNP_TCAM_APQF_MASK(hw_id), 0);
//
//			wr32(hw, RNP_TCAM_MODE, 1);
//			/* update tcam cache */
//			//wr32(hw, RNP_TCAM_CACHE_ADDR_CLR, 0);
//			//wr32(hw, RNP_TCAM_CACHE_REQ_CLR, 0);
//		}
	}

	return 0;
}

u32 rnp_tx_desc_unused_sw(struct rnp_ring *tx_ring)
{
	u16 ntu = tx_ring->next_to_use;
	u16 ntc = tx_ring->next_to_clean;
	u16 count = tx_ring->count;

	return ((ntu >= ntc) ? (count - ntu + ntc) : (ntc - ntu));
}

u32 rnp_rx_desc_used_hw(struct rnp_hw *hw, struct rnp_ring *rx_ring)
{
	u32 head =
		ring_rd32(rx_ring, RNP_DMA_REG_RX_DESC_BUF_HEAD);
	u32 tail =
		ring_rd32(rx_ring, RNP_DMA_REG_RX_DESC_BUF_TAIL);
	u16 count = rx_ring->count;
	//printk("head %d tail %d count %d\n",head, tail, count);

	return ((tail >= head) ? (count - tail + head) : (head - tail));
}

u32 rnp_tx_desc_unused_hw(struct rnp_hw *hw, struct rnp_ring *tx_ring)
{
	u32 head =
		ring_rd32(tx_ring, RNP_DMA_REG_TX_DESC_BUF_HEAD);
	u32 tail =
		ring_rd32(tx_ring, RNP_DMA_REG_TX_DESC_BUF_TAIL);
	u16 count = tx_ring->count;

	return ((tail >= head) ? (count - tail + head) : (head - tail));
}

s32 rnp_disable_rxr_maxrate(struct net_device *netdev, u8 queue_index)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw *hw = &adapter->hw;
	struct rnp_ring *rx_ring = adapter->rx_ring[queue_index];
	u32 reg_idx = rx_ring->rnp_queue_idx;

	/* disable which dma ring in maxrate limit mode */
	wr32(hw, RNP_SELECT_RING_EN(reg_idx), 0);
	/* Clear Tx Ring maxrate */
	wr32(hw, RNP_RX_RING_MAXRATE(reg_idx), 0);

	return 0;
}

s32 rnp_enable_rxr_maxrate(struct net_device *netdev, u8 queue_index,
			   u32 maxrate)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw *hw = &adapter->hw;
	struct rnp_ring *rx_ring = adapter->rx_ring[queue_index];
	u32 reg_idx = rx_ring->rnp_queue_idx;
	u32 real_rate = maxrate / 16;

	if (!real_rate)
		return -EINVAL;

	wr32(hw, RNP_RING_FC_ENABLE, true);
	/* disable which dma ring in maxrate limit mode */
	wr32(hw, RNP_SELECT_RING_EN(reg_idx), true);
	/* Clear Tx Ring maxrate */
	wr32(hw, RNP_RX_RING_MAXRATE(reg_idx), real_rate);

	return 0;
}
