#ifndef __RNP_VF_H__
#define __RNP_VF_H__

#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/if_ether.h>
#include <linux/netdevice.h>

#include "defines.h"
#include "regs.h"
#include "mbx.h"

struct rnpvf_hw;

/* iterator type for walking multicast address lists */
typedef u8 *(*rnp_mc_addr_itr)(struct rnpvf_hw *hw, u8 **mc_addr_ptr,
			       u32 *vmdq);
struct rnp_mac_operations {
	s32 (*init_hw)(struct rnpvf_hw *);
	s32 (*reset_hw)(struct rnpvf_hw *);
	s32 (*start_hw)(struct rnpvf_hw *);
	s32 (*clear_hw_cntrs)(struct rnpvf_hw *);
	enum rnp_media_type (*get_media_type)(struct rnpvf_hw *);
	u32 (*get_supported_physical_layer)(struct rnpvf_hw *);
	s32 (*get_mac_addr)(struct rnpvf_hw *, u8 *);
	s32 (*get_queues)(struct rnpvf_hw *);
	s32 (*stop_adapter)(struct rnpvf_hw *);
	s32 (*get_bus_info)(struct rnpvf_hw *);
	int (*read_eth_reg)(struct rnpvf_hw *, int, u32 *);

	int (*get_mtu)(struct rnpvf_hw *);
	int (*set_mtu)(struct rnpvf_hw *, int);
	int (*req_reset_pf)(struct rnpvf_hw *);

	/* Link */
	s32 (*setup_link)(struct rnpvf_hw *, rnp_link_speed, bool, bool);
	s32 (*check_link)(struct rnpvf_hw *, rnp_link_speed *, bool *, bool);
	s32 (*get_link_capabilities)(struct rnpvf_hw *, rnp_link_speed *,
				     bool *);

	/* RAR, Multicast, VLAN */
	s32 (*set_rar)(struct rnpvf_hw *, u32, u8 *, u32);
	s32 (*set_uc_addr)(struct rnpvf_hw *, u32, u8 *);
	s32 (*init_rx_addrs)(struct rnpvf_hw *);
	s32 (*update_mc_addr_list)(struct rnpvf_hw *, struct net_device *);
	s32 (*enable_mc)(struct rnpvf_hw *);
	s32 (*disable_mc)(struct rnpvf_hw *);
	s32 (*clear_vfta)(struct rnpvf_hw *);
	s32 (*set_vfta)(struct rnpvf_hw *, u32, u32, bool);
	s32 (*set_vlan_strip)(struct rnpvf_hw *, bool);
};

enum rnp_mac_type {
	rnp_mac_unknown = 0,
	rnp_mac_2port_10G,
	rnp_mac_2port_40G,
	rnp_mac_4port_10G,
	rnp_mac_8port_10G,
	rnp_num_macs
};

struct rnp_mac_info {
	struct rnp_mac_operations ops;
	u8 addr[6];
	u8 perm_addr[6];

	enum rnp_mac_type type;

	s32 mc_filter_type;
	u32 dma_version;

	bool get_link_status;
	u32 max_tx_queues;
	u32 max_rx_queues;
	u32 max_msix_vectors;
};

#define RNP_MAX_TRAFFIC_CLASS 4
enum rnp_fc_mode {
	rnp_fc_none = 0,
	rnp_fc_rx_pause,
	rnp_fc_tx_pause,
	rnp_fc_full,
	rnp_fc_default
};
struct rnp_fc_info {
	u32 high_water[RNP_MAX_TRAFFIC_CLASS]; /* Flow Control High-water */
	u32 low_water[RNP_MAX_TRAFFIC_CLASS]; /* Flow Control Low-water */
	u16 pause_time; /* Flow Control Pause timer */
	bool send_xon; /* Flow control send XON */
	bool strict_ieee; /* Strict IEEE mode */
	bool disable_fc_autoneg; /* Do not autonegotiate FC */
	bool fc_was_autonegged; /* Is current_mode the result of autonegging? */
	enum rnp_fc_mode current_mode; /* FC mode in effect */
	enum rnp_fc_mode requested_mode; /* FC mode requested by caller */
};

struct rnp_mbx_operations {
	s32 (*init_params)(struct rnpvf_hw *hw);
	s32 (*read)(struct rnpvf_hw *, u32 *, u16, bool);
	s32 (*write)(struct rnpvf_hw *, u32 *, u16, bool);
	s32 (*read_posted)(struct rnpvf_hw *, u32 *, u16, bool);
	s32 (*write_posted)(struct rnpvf_hw *, u32 *, u16, bool);
	s32 (*check_for_msg)(struct rnpvf_hw *, bool);
	s32 (*check_for_ack)(struct rnpvf_hw *, bool);
	s32 (*check_for_rst)(struct rnpvf_hw *, bool);
	s32 (*configure)(struct rnpvf_hw *hw, int nr_vec, bool enable);
};

struct rnp_mbx_stats {
	u32 msgs_tx;
	u32 msgs_rx;

	u32 acks;
	u32 reqs;
	u32 rsts;
};

struct rnp_mbx_info {
	struct rnp_mbx_operations ops;
	struct rnp_mbx_stats stats;
	u32 timeout;
	u32 udelay;
	u32 v2p_mailbox;
	u16 size;

	u16 pf_req;
	u16 pf_ack;
	u16 cpu_req;
	u16 cpu_ack;
};

struct rnpvf_hw_stats_own {
	u64 vlan_add_cnt;
	u64 vlan_strip_cnt;
	u64 csum_err;
	u64 csum_good;
};

struct rnpvf_hw_stats {
	u64 base_vfgprc;
	u64 base_vfgptc;
	u64 base_vfgorc;
	u64 base_vfgotc;
	u64 base_vfmprc;

	u64 last_vfgprc;
	u64 last_vfgptc;
	u64 last_vfgorc;
	u64 last_vfgotc;
	u64 last_vfmprc;

	u64 vfgprc;
	u64 vfgptc;
	u64 vfgorc;
	u64 vfgotc;
	u64 vfmprc;

	u64 saved_reset_vfgprc;
	u64 saved_reset_vfgptc;
	u64 saved_reset_vfgorc;
	u64 saved_reset_vfgotc;
	u64 saved_reset_vfmprc;
};

struct rnpvf_info {
	enum rnp_mac_type mac;
	const struct rnp_mac_operations *mac_ops;
	s32 (*get_invariants)(struct rnpvf_hw *);
};

void rnpvf_rlpml_set_vf(struct rnpvf_hw *hw, u16 max_size);
//int rnpvf_negotiate_api_version(struct rnpvf_hw *hw, int api);
//int rnpvf_get_queues(struct rnpvf_hw *hw, unsigned int *num_tcs, unsigned int *default_tc);
#endif /* __RNP_VF_H__ */

