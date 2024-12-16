/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 2022 - 2023 Mucse Corporation. */

#ifndef _RNPGBE_SRIOV_H_
#define _RNPGBE_SRIOV_H_

int rnpgbe_setup_ring_maxrate(struct rnpgbe_adapter *adapter, int ring,
		u64 max_rate);
int rnpgbe_get_vf_ringnum(struct rnpgbe_hw *hw, int vf, int num);
void rnpgbe_restore_vf_macs(struct rnpgbe_adapter *adapter);
void rnpgbe_restore_vf_multicasts(struct rnpgbe_adapter *adapter);
void rnpgbe_restore_vf_macvlans(struct rnpgbe_adapter *adapter);
void rnpgbe_msg_task(struct rnpgbe_adapter *adapter);
int rnpgbe_vf_configuration(struct pci_dev *pdev, unsigned int event_mask);
void rnpgbe_ping_all_vfs(struct rnpgbe_adapter *adapter);
#ifdef HAVE_NDO_SET_VF_MIN_MAX_TX_RATE
int rnpgbe_ndo_set_vf_bw(struct net_device *netdev,
					  int vf,
					  int __always_unused min_tx_rate,
					  int max_tx_rate);
#else
int rnpgbe_ndo_set_vf_bw(struct net_device *netdev, int vf, int max_tx_rate);
#endif /* HAVE_NDO_SET_VF_MIN_MAX_TX_RATE */
int rnpgbe_ndo_set_vf_mac(struct net_device *netdev, int queue, u8 *mac);
int rnpgbe_msg_post_status(struct rnpgbe_adapter *adapter, enum PF_STATUS status);
#ifdef HAVE_NDO_SET_VF_MIN_MAX_TX_RATE
int rnpgbe_ndo_set_vf_bw(struct net_device *netdev, int vf,
		      int __always_unused min_tx_rate, int max_tx_rate);
#else
int rnpgbe_ndo_set_vf_bw(struct net_device *netdev, int vf, int max_tx_rate);
#endif /* HAVE_NDO_SET_VF_MIN_MAX_TX_RATE */
int rnpgbe_ndo_set_vf_spoofchk(struct net_device *netdev, int vf, bool setting);
int rnpgbe_ndo_get_vf_config(struct net_device *netdev, int vf,
			  struct ifla_vf_info *ivi);
void rnpgbe_check_vf_rate_limit(struct rnpgbe_adapter *adapter);
int rnpgbe_disable_sriov(struct rnpgbe_adapter *adapter);
#ifdef CONFIG_PCI_IOV
void rnpgbe_enable_sriov_true(struct rnpgbe_adapter *adapter);
void rnpgbe_enable_sriov(struct rnpgbe_adapter *adapter);
#endif
int rnpgbe_pci_sriov_configure(struct pci_dev *dev, int num_vfs);
#ifdef IFLA_VF_VLAN_INFO_MAX
int rnpgbe_ndo_set_vf_vlan(struct net_device *netdev, int vf, u16 vlan,
		u8 qos, __be16 vlan_proto);
#else
int rnpgbe_ndo_set_vf_vlan(struct net_device *netdev, int vf, u16 vlan, u8 qos);
#endif

static inline void rnpgbe_set_vmvir(struct rnpgbe_adapter *adapter, u16 vid, u16 qos,
				 u32 vf)
{
	//struct rnpgbe_hw *hw = &adapter->hw;
}
int rnpgbe_ndo_set_vf_link_state(struct net_device *netdev, int vf, int state);
#if IS_ENABLED(CONFIG_PCI_IOV)
int rnpgbe_ndo_set_vf_spoofchk(struct net_device *netdev, int vf, bool setting);
#endif
#ifdef HAVE_NDO_SET_VF_TRUST
int rnpgbe_ndo_set_vf_trust(struct net_device *netdev, int vf, bool setting);
#endif
#endif /* _RNP_SRIOV_H_ */

