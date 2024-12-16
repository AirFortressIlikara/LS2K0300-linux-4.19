#ifndef _RNP_SRIOV_H_
#define _RNP_SRIOV_H_

void rnp_restore_vf_multicasts(struct rnp_adapter *adapter);
void rnp_restore_vf_macvlans(struct rnp_adapter *adapter);
void rnp_msg_task(struct rnp_adapter *adapter);
int rnp_vf_configuration(struct pci_dev *pdev, unsigned int event_mask);
void rnp_disable_tx_rx(struct rnp_adapter *adapter);
void rnp_ping_all_vfs(struct rnp_adapter *adapter);
#ifdef HAVE_NDO_SET_VF_MIN_MAX_TX_RATE
int rnp_ndo_set_vf_bw(struct net_device *netdev,
					  int vf,
					  int __always_unused min_tx_rate,
					  int max_tx_rate);
#else
int rnp_ndo_set_vf_bw(struct net_device *netdev, int vf, int max_tx_rate);
#endif /* HAVE_NDO_SET_VF_MIN_MAX_TX_RATE */
int rnp_ndo_set_vf_mac(struct net_device *netdev, int queue, u8 *mac);
void rnp_msg_post_status(struct rnp_adapter *adapter, enum PF_STATUS status);
#ifdef HAVE_NDO_SET_VF_MIN_MAX_TX_RATE
int rnp_ndo_set_vf_bw(struct net_device *netdev, int vf,
		      int __always_unused min_tx_rate, int max_tx_rate);
#else
int rnp_ndo_set_vf_bw(struct net_device *netdev, int vf, int max_tx_rate);
#endif /* HAVE_NDO_SET_VF_MIN_MAX_TX_RATE */
int rnp_ndo_set_vf_spoofchk(struct net_device *netdev, int vf, bool setting);
int rnp_ndo_get_vf_config(struct net_device *netdev, int vf,
			  struct ifla_vf_info *ivi);
void rnp_check_vf_rate_limit(struct rnp_adapter *adapter);
int rnp_disable_sriov(struct rnp_adapter *adapter);
#ifdef CONFIG_PCI_IOV
void rnp_enable_sriov(struct rnp_adapter *adapter);
#endif
int rnp_pci_sriov_configure(struct pci_dev *dev, int num_vfs);
#ifdef IFLA_VF_VLAN_INFO_MAX
int rnp_ndo_set_vf_vlan(struct net_device *netdev, int vf, u16 vlan,
                          u8 qos, __be16 vlan_proto);
#else
int rnp_ndo_set_vf_vlan(struct net_device *netdev, int vf, u16 vlan, u8 qos);
#endif

static inline void rnp_set_vmvir(struct rnp_adapter *adapter, u16 vid, u16 qos,
				 u32 vf)
{
	struct rnp_hw *hw = &adapter->hw;
}

#endif /* _RNP_SRIOV_H_ */

