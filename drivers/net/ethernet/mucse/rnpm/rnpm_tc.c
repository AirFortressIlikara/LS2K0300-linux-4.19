#include <linux/netdevice.h>
#ifdef NETIF_F_HW_TC
#include <net/tc_act/tc_gact.h>
#include <net/tc_act/tc_mirred.h>
#include <net/pkt_cls.h>
#endif

#include "rnpm_tc_u32_parse.h"
#include "rnpm_tc.h"

static void __maybe_unused rnpm_setup_txr_prio(void __iomem *ioaddr,
											   struct rnpm_ring *tx_ring,
											   int prio)
{
	u16 dma_ring_idx = tx_ring->rnpm_queue_idx;

	rnpm_wr_reg(ioaddr + RNPM_DMA_REG_TX_ARB_DEF_LVL(dma_ring_idx), prio);
}

int rnpm_setup_tx_maxrate(void __iomem *ioaddr,
			  struct rnpm_ring *tx_ring, u64 max_rate,
			  int samples_1sec)
{
	u16 dma_ring_idx = tx_ring->rnpm_queue_idx;

	/* set hardware samping internal 1S */
	rnpm_wr_reg(ioaddr + RNPM_DMA_REG_TX_FLOW_CTRL_TM(dma_ring_idx),
		samples_1sec);
	rnpm_wr_reg(ioaddr + RNPM_DMA_REG_TX_FLOW_CTRL_TH(dma_ring_idx),
		max_rate);
	return 0;
}

//mutiple ports not support pfc
//int rnpm_setup_mqprio(struct net_device *netdev, void *type_data)
//{
//#ifdef __TC_MQPRIO_MODE_MAX
//	struct tc_mqprio_qopt_offload *mqprio_qopt = type_data;
//	struct rnpm_adapter *adapter = netdev_priv(netdev);
//	struct rnpm_ring  *tx_ring = NULL;
//	u8 enabled_tc = 0, num_tc, hw;
//	u16 mode = 0;
//	int ret = -EINVAL;
//	int i = 0;
//	u64 max_tx_rate = 0;
//
//	num_tc = mqprio_qopt->qopt.num_tc;
//	hw = mqprio_qopt->qopt.hw;
//	mode = mqprio_qopt->mode;
//	netdev_info(netdev, "setup mqprio hw %d mode %d num_tx %d\n",
//		hw, mode, num_tc);
//
//	/* should check num_tc smaller than the adapter->num_tx_queues ? */
//	if (!num_tc) {
//		/* reset mqprio tc */
//		netdev_reset_tc(netdev);
//		netif_set_real_num_tx_queues(netdev, adapter->num_tx_queues);
//		for (i = 0; i < adapter->num_tx_queues; i++) {
//		/* reset all tx ring prio to zero */
//			tx_ring = adapter->tx_ring[i];
//			rnpm_setup_txr_prio(adapter->hw.hw_addr, tx_ring, 0);
//			rnpm_setup_tx_maxrate(adapter->hw.hw_addr, tx_ring, 0 ,
//			adapter->hw.usecstocount * 1000000);
//		}
//		return 0;
//	}
//	if (num_tc > RNPM_MAX_TCS_NUM) {
//		netdev_err(netdev, "Max %d traffic classes supported\n",
//				adapter->num_tx_queues);
//		return -EINVAL;
//	}
//
//	/* Reset the number of netdev queues based on the TC count */
//	adapter->num_tc = num_tc;
//	netdev_set_num_tc(netdev, num_tc);
//
//	/* For now we just set the prio for tx ring_index increase <=> prio */
//	for (i = 0; i < RNPM_MAX_USER_PRIO; i++)
//		adapter->prio_tc_map[i] = mqprio_qopt->qopt.prio_tc_map[i];
//
//
//	for (i = 0; i < num_tc; i++) {
//		tx_ring = adapter->tx_ring[i];
//		rnpm_setup_txr_prio(adapter->hw.hw_addr, tx_ring, i);
//		max_tx_rate = mqprio_qopt->max_rate[i];
//		netdev_info(netdev, "setup tc[%d] max_rate %llu\n",
//			i, max_tx_rate);
//		if (max_tx_rate)
//			rnpm_setup_tx_maxrate(adapter->hw.hw_addr,
//				tx_ring, max_tx_rate, adapter->hw.usecstocount * 1000000);
//	}
//
//	for (i = 0; i < num_tc; i++)
//		netdev_set_tc_queue(netdev, i,
//			mqprio_qopt->qopt.count[i],
//			mqprio_qopt->qopt.offset[i]);
//
//	return 0;
//#else
//	return -EOPNOTSUPP;
//#endif
//}
//#if (defined HAVE_RHEL7_NETDEV_OPS_EXT_NDO_SETUP_TC) || (defined NETIF_F_HW_TC)
//static int rnpm_clsu32_build_input(struct tc_cls_u32_offload *cls,
//				  struct rnpm_fdir_filter *input,
//				  const struct rnpm_match_parser *parsers)
//{
//	int i = 0, j = 0, err = -1;
//	__be32 val, mask, off;
//	bool found;
//
//	for (i = 0; i < cls->knode.sel->nkeys; i++) {
//		off = cls->knode.sel->keys[i].off;
//		val = cls->knode.sel->keys[i].val;
//		mask = cls->knode.sel->keys[i].mask;
//		dbg("cls-key[%d] off %d val %d mask %d\n ", i, off, val, mask);
//		found = false;
//		for (j = 0; parsers[j].val; j++) {
//			/* according the off select parser */
//			if (off == parsers[j].off) {
//				found = true;
//				err = parsers[j].val(input, val, mask);
//				if (err)
//					return err;
//
//				break;
//			}
//		}
//		/* if the rule can't parse that we don't support the rule */
//		if (!found)
//			return -EINVAL;
//	}
//
//	return 0;
//}
//
//static int rnpm_action_parse(struct tcf_exts *exts, u64 *action, u8 *queue)
//{
//	const struct tc_action *a;
//#if defined(HAVE_TCF_EXTS_TO_LIST)
//	LIST_HEAD(actions);
//#elif defined(HAVE_TCF_EXTS_FOR_EACH_ACTION)
//	int j;
//#endif
//
//#ifdef HAVE_TCF_EXTS_HAS_ACTION
//	TRACE();
//	if (!tcf_exts_has_actions(exts))
//#else
//	TRACE();
//	if (tc_no_actions(exts))
//#endif
//		return -EINVAL;
//#if defined(HAVE_TCF_EXTS_TO_LIST)
//	tcf_exts_to_list(exts, &actions);
//	list_for_each_entry(a, &actions, list) {
//#elif defined(HAVE_TCF_EXTS_FOR_EACH_ACTION)
//	tcf_exts_for_each_action(j, a, exts) {
//#else
//	tc_for_each_action(a, exts) {
//#endif
//		/* Drop action */
//		if (is_tcf_gact_shot(a)) {
//			*action = RNPM_FDIR_DROP_QUEUE;
//			*queue = RNPM_FDIR_DROP_QUEUE;
//			return 0;
//		}
//#ifdef HAVE_TCF_MIRRED_REDIRECT
//		/* Redirect to a VF or a offloaded macvlan */
//#ifdef HAVE_TCF_MIRRED_EGRESS_REDIRECT
//		TRACE();
//		if (is_tcf_mirred_egress_redirect(a)) {
//#else
//		TRACE();
//		if (is_tcf_mirred_redirect(a)) {
//#endif
//
//#ifdef HAVE_TCF_MIRRED_DEV
//			struct net_device *dev = tcf_mirred_dev(a);
//
//			TRACE();
//			if (!dev)
//				return -EINVAL;
//			//return handle_redirect_action(adapter, dev->ifindex,
//			//	queue, action);
//#else
//			int ifindex = tcf_mirred_ifindex(a);
//
//    //                  return handle_redirect_action(adapter, ifindex,
//    //                                                  queue, action);
//#endif /* HAVE_TCF_MIRRED_DEV */
//		}
//#endif /* HAVE_TCF_MIRRED_REDIRECT */
//
//		return -EINVAL;
//	}
//
//	return 0;
//}
//int rnpm_config_knode(struct net_device *dev,
//		     __be16 protocol,
//		     struct tc_cls_u32_offload *cls)
//{
//	/*1. check ethernet hw-feature U32 can offload */
//	/*2. check U32 protocol We just support IPV4 offloading For now*/
//	/*3. check if this cls is a cls of root u32 or cls of class u32*/
//	/*4. check if this cls has been added.
//	 * the filter extry create but the match val and mask don't fill
//	 * so we can use it.
//	 * find a exist extry and the match val and mask is added before
//	 * so we don't need add it again
//	 */
//	u32 uhtid, link_uhtid;
//	int ret;
//	struct rnpm_adapter *adapter = netdev_priv(dev);
//	u8 queue;
//	struct rnpm_fdir_filter *input;
//	struct rnpm_hw *hw = &adapter->hw;
//	u32 loc = cls->knode.handle & 0xfffff;
//
//	if (protocol != htons(ETH_P_IP))
//		return -EOPNOTSUPP;
//
//	uhtid = TC_U32_USERHTID(cls->knode.handle);
//	link_uhtid = TC_U32_USERHTID(cls->knode.link_handle);
//
//	netdev_info(dev, "uhtid %d link_uhtid %d protocol 0x%2x\n",
//		uhtid, link_uhtid, ntohs(protocol));
//	/* For now just support handle root ingress
//	 * TODO more feature
//	 */
//	if (uhtid != 0x800)
//		return -EINVAL;
//
//	input = kzalloc(sizeof(*input), GFP_KERNEL);
//	/*be carefull this input mem need to free */
//	ret = rnpm_clsu32_build_input(cls, input, rnpm_ipv4_parser);
//	if (ret) {
//		netdev_warn(dev, "This Rules We Can't Support It\n");
//		goto out;
//	}
//	ret = rnpm_action_parse(cls->knode.exts, &input->action, &queue);
//	if (ret)
//		goto out;
//
//	dbg("tc filter rule sw_location %d\n", loc);
//
//	input->hw_idx = adapter->tuple_5_count;
//	input->sw_idx = loc;
//	spin_lock(&adapter->fdir_perfect_lock);
//
//	ret = rnpm_fdir_write_perfect_filter(adapter->fdir_mode, hw,
//			&input->filter, input->hw_idx,
//			(input->action == RNPM_FDIR_DROP_QUEUE) ?
//			RNPM_FDIR_DROP_QUEUE :
//			adapter->rx_ring[input->action]->rnpm_queue_idx
//			);
//	if (ret)
//		goto err_out_w_lock;
//
//	rnpm_update_ethtool_fdir_entry(adapter, input, input->sw_idx);
//	spin_unlock(&adapter->fdir_perfect_lock);
//
//	return 0;
//err_out_w_lock:
//	spin_unlock(&adapter->fdir_perfect_lock);
//out:
//	kfree(input);
//	return -EOPNOTSUPP;
//}
//
//int rnpm_delete_knode(struct net_device *dev, struct tc_cls_u32_offload *cls)
//{
//	/* 1. check weather filter rule is ingress root */
//	struct rnpm_adapter *adapter = netdev_priv(dev);
//	u32 loc = cls->knode.handle & 0xfffff;
//	u32 uhtid = TC_U32_USERHTID(cls->knode.handle);
//	int ret;
//
//	if ((uhtid != 0x800))
//		return -EINVAL;
//
//	spin_lock(&adapter->fdir_perfect_lock);
//	ret = rnpm_update_ethtool_fdir_entry(adapter, NULL, loc);
//	spin_unlock(&adapter->fdir_perfect_lock);
//
//	return ret;
//}
//#ifdef HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV
//int rnpm_setup_tc_cls_u32(struct net_device *dev,
//			 struct tc_cls_u32_offload *cls_u32)
//{
//	__be16 proto =  cls_u32->common.protocol;
//#else
//int rnpm_setup_tc_cls_u32(struct net_device *dev, __be16 proto,
//	struct tc_cls_u32_offload *cls_u32)
//{
//#endif
//#if 0
//	if (!is_classid_clsact_ingress(cls_u32->common.classid) ||
//		cls_u32->common.chain_index)
//		return -EOPNOTSUPP;
//#endif
//	TRACE();
//	dbg("cls_u32->command is %d\n", cls_u32->command);
//	switch (cls_u32->command) {
//	case TC_CLSU32_NEW_KNODE:
//	case TC_CLSU32_REPLACE_KNODE:
//		return rnpm_config_knode(dev, proto, cls_u32);
//	case TC_CLSU32_DELETE_KNODE:
//		return rnpm_delete_knode(dev, cls_u32);
//	default:
//		return -EOPNOTSUPP;
//	}
//}
//#if defined(HAVE_TCF_BLOCK)
//#if defined(HAVE_NDO_SETUP_TC_REMOVE_TC_TO_NETDEV)
//
//static int rnpm_setup_tc_block_ingress_cb(enum tc_setup_type type,
//		void *type_data, void *cb_priv)
//{
//	struct net_device *dev = cb_priv;
//	struct rnpm_adapter *adapter = netdev_priv(dev);
//
//	if (test_bit(__RNPM_DOWN, &adapter->state)) {
//		netdev_err(adapter->netdev,
//				"Failed to setup tc on port %d. Link Down? 0x%.2lx\n",
//				adapter->port, adapter->state);
//		return -EINVAL;
//	}
//	TRACE();
//	if (!tc_cls_can_offload_and_chain0(dev, type_data))
//		return -EOPNOTSUPP;
//
//	switch (type) {
//	case TC_SETUP_CLSU32:
//		return rnpm_setup_tc_cls_u32(dev, type_data);
//	default:
//		return -EOPNOTSUPP;
//	}
//}
//static LIST_HEAD(rnpm_block_cb_list);
//int rnpm_setup_tc_block(struct net_device *dev,
//	struct flow_block_offload *f)
//{
//	struct rnpm_adapter *adapter = netdev_priv(dev);
//#if (KERNEL_VERSION(5, 1, 0) > LINUX_VERSION_CODE)
//#if (!(RHEL_RELEASE_CODE && (RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(8,2))))
//        tc_setup_cb_t *cb;
//#else
//        flow_setup_cb_t *cb;
//#endif
//#else
//	flow_setup_cb_t *cb;
//#endif
//	bool ingress_only;
//
//	TRACE();
//	if (f->binder_type == FLOW_BLOCK_BINDER_TYPE_CLSACT_INGRESS) {
//		ingress_only = true;
//		cb = rnpm_setup_tc_block_ingress_cb;
//
//		return flow_block_cb_setup_simple(f, &rnpm_block_cb_list,
//				cb, adapter, dev, ingress_only);
//	}
//
//	return -EOPNOTSUPP;
//}
//#endif
//#endif
//#endif
