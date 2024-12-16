#include <linux/types.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/ipv6.h>
#ifdef NETIF_F_HW_VLAN_CTAG_TX
#include <linux/if_vlan.h>
#endif

#include "rnp.h"
#include "rnp_type.h"
#include "rnp_sriov.h"

void rnp_msg_post_status_signle(struct rnp_adapter *adapter,
								enum PF_STATUS status,
								int vf);
#ifdef CONFIG_PCI_IOV
static int __rnp_enable_sriov(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	int num_vf_macvlans, i;
	struct vf_macvlans *mv_list;
	u32 v;

	dbg("%s:%d flags:0x%x\n", __func__, __LINE__, adapter->flags);
	/* sriov and dcb cannot open together */
	/* reset numtc */
	adapter->flags &= (~RNP_FLAG_DCB_ENABLED);
	netdev_reset_tc(adapter->netdev);

	e_info(probe, "SR-IOV enabled with %d VFs\n", adapter->num_vfs);

	dbg("%s:%d flags:0x%x\n", __func__, __LINE__, adapter->flags);

	/* Enable VMDq flag so device will be set in VM mode */
	adapter->flags |= RNP_FLAG_VMDQ_ENABLED;
	if (!adapter->ring_feature[RING_F_VMDQ].limit)
		adapter->ring_feature[RING_F_VMDQ].limit = 1;
		// FIXME pf in sriov mode always use the last vf
#ifdef FIX_VF_BUG
	adapter->ring_feature[RING_F_VMDQ].offset = 0;
	// should close vf isolate
#else
	// adapter->ring_feature[RING_F_VMDQ].offset = RNP_MAX_VF_CNT - 1;
	adapter->ring_feature[RING_F_VMDQ].offset = hw->max_vfs - 1;
#endif

	// fix in n500?
	num_vf_macvlans =
		hw->num_rar_entries - (RNP_MAX_PF_MACVLANS + 1 + adapter->num_vfs);

	adapter->mv_list = mv_list =
		kcalloc(num_vf_macvlans, sizeof(struct vf_macvlans), GFP_KERNEL);
	if (mv_list) {
		/* Initialize list of VF macvlans */
		INIT_LIST_HEAD(&adapter->vf_mvs.l);
		for (i = 0; i < num_vf_macvlans; i++) {
			mv_list->vf = -1;
			mv_list->free = true;
			mv_list->rar_entry =
				hw->mac.num_rar_entries - (i + adapter->num_vfs + 1);
			list_add(&mv_list->l, &adapter->vf_mvs.l);
			mv_list++;
		}
	}

	adapter->flags2 |= RNP_FLAG2_BRIDGE_MODE_VEB;
	dbg("%s:%d flags:0x%x\n", __func__, __LINE__, adapter->flags);

	hw->ops.set_sriov_status(hw, true);

	adapter->vfinfo =
		kcalloc(adapter->num_vfs, sizeof(struct vf_data_storage), GFP_KERNEL);
	if (adapter->vfinfo) {
		/* limit trafffic classes based on VFs enabled */
		/* TODO analyze VF need support pfc or traffic classes */
		/* We do not support RSS w/ SR-IOV */
		adapter->ring_feature[RING_F_RSS].limit = hw->sriov_ring_limit;

		/* Disable RSC when in SR-IOV mode */
		adapter->flags2 &= ~(RNP_FLAG2_RSC_CAPABLE | RNP_FLAG2_RSC_ENABLED);

		adapter->flags |= RNP_FLAG_SRIOV_ENABLED;

		/* enable spoof checking for all VFs */
		// for (i = 0; i < adapter->num_vfs; i++)
		// adapter->vfinfo[i].spoofchk_enabled = true;
		return 0;
	}

	/* open flags at last to avoid null call adapter->vfinfo */
	dbg("%s:%d flags:0x%x\n", __func__, __LINE__, adapter->flags);
	return -ENOMEM;
}

/* Note this function is called when the user wants to enable SR-IOV
 * VFs using the now deprecated module parameter
 */
void rnp_enable_sriov(struct rnp_adapter *adapter)
{
	int pre_existing_vfs = 0;
	struct rnp_hw *hw = &adapter->hw;

	pre_existing_vfs = pci_num_vf(adapter->pdev);
	if (!pre_existing_vfs && !adapter->num_vfs)
		return;

	dbg("%s:%d flags:0x%x\n", __func__, __LINE__, adapter->flags);
	if (!pre_existing_vfs)
		dev_warn(&adapter->pdev->dev,
				 "Enabling SR-IOV VFs using the module parameter is deprecated "
				 "- please use the pci sysfs interface.\n");

	dbg("%s:%d flags:0x%x\n", __func__, __LINE__, adapter->flags);
	/* If there are pre-existing VFs then we have to force
	 * use of that many - over ride any module parameter value.
	 * This may result from the user unloading the PF driver
	 * while VFs were assigned to guest VMs or because the VFs
	 * have been created via the new PCI SR-IOV sysfs interface.
	 */
	if (pre_existing_vfs) {
		adapter->num_vfs = pre_existing_vfs;
		dev_warn(&adapter->pdev->dev,
				 "Virtual Functions already enabled for this device - Please "
				 "reload all VF drivers to avoid spoofed packet errors\n");
	} else {
		int err;
		/*
		 * The n10 supports up to 64 VFs per physical function
		 * but this implementation limits allocation to 126 so that
		 * basic networking resources are still available to the
		 * physical function.  If the user requests greater than
		 * 64 VFs then it is an error - reset to default of zero.
		 */
		adapter->num_vfs =
			min_t(unsigned int, adapter->num_vfs, hw->max_vfs - 1);

		err = pci_enable_sriov(adapter->pdev, adapter->num_vfs);
		if (err) {
			e_err(probe, "Failed to enable PCI sriov: %d\n", err);
			adapter->num_vfs = 0;
			return;
		}
		dbg("%s:%d flags:0x%x\n", __func__, __LINE__, adapter->flags);
	}

	dbg("%s:%d flags:0x%x\n", __func__, __LINE__, adapter->flags);
	if (!__rnp_enable_sriov(adapter))
		return;

	dbg("%s:%d flags:0x%x\n", __func__, __LINE__, adapter->flags);
	/* If we have gotten to this point then there is no memory available
	 * to manage the VF devices - print message and bail.
	 */
	e_err(probe,
		  "Unable to allocate memory for VF Data Storage - "
		  "SRIOV disabled\n");
	rnp_disable_sriov(adapter);
}

static bool rnp_vfs_are_assigned(struct rnp_adapter *adapter)
{
	struct pci_dev *pdev = adapter->pdev;
	struct pci_dev *vfdev;
	unsigned int dev_id = RNP_DEV_ID_N10_PF0_VF_N;
	unsigned int vendor_id = PCI_VENDOR_ID_MUCSE;

	// todo add in n500
	switch (adapter->pdev->device) {
		case RNP_DEV_ID_N10_PF0:
		case RNP_DEV_ID_N10_PF1:
			vendor_id = 0x1dab;
			if (rnp_is_pf1(pdev))
				dev_id = RNP_DEV_ID_N10_PF1_VF;
			else
				dev_id = RNP_DEV_ID_N10_PF0_VF;
			break;
		case PCI_DEVICE_ID_N10_PF0:
		case PCI_DEVICE_ID_N10_PF1:
			vendor_id = PCI_VENDOR_ID_MUCSE;
			if (rnp_is_pf1(pdev))
				dev_id = RNP_DEV_ID_N10_PF1_VF_N;
			else
				dev_id = RNP_DEV_ID_N10_PF0_VF_N;
	}

	/* loop through all the VFs to see if we own any that are assigned */
	vfdev = pci_get_device(vendor_id, dev_id, NULL);
	while (vfdev) {
		/* if we don't own it we don't care */
		if (vfdev->is_virtfn && vfdev->physfn == pdev) {
			/* if it is assigned we cannot release it */
			if (vfdev->dev_flags & PCI_DEV_FLAGS_ASSIGNED)
				return true;
		}

		vfdev = pci_get_device(vendor_id, dev_id, vfdev);
	}

	return false;
}

#endif /* #ifdef CONFIG_PCI_IOV */
int rnp_disable_sriov(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	u32 gpie;
	u32 vmdctl, v;
	int rss, i, port;
	int time = 0;


	if (!(adapter->flags & RNP_FLAG_SRIOV_ENABLED))
		return 0;

	adapter->num_vfs = 0;
	adapter->flags &= ~RNP_FLAG_SRIOV_ENABLED;
	adapter->flags &= ~RNP_FLAG_SRIOV_INIT_DONE;

	msleep(100);
	hw->ops.set_mac_rx(hw, false);

	hw->ops.set_sriov_status(hw, false);

	/* set num VFs to 0 to prevent access to vfinfo */
        while (test_and_set_bit(__RNP_USE_VFINFI, &adapter->state)) {
                msleep(100);
                time++;

                if (time > 100) {
                        printk("wait flags timeout\n");
                        break;
                }

        }
	if (time < 100)
		clear_bit(__RNP_USE_VFINFI, &adapter->state);


	/* free VF control structures */
	kfree(adapter->vfinfo);
	adapter->vfinfo = NULL;

	/* free macvlan list */
	kfree(adapter->mv_list);
	adapter->mv_list = NULL;

	dbg("%s:%d flags:0x%x\n", __func__, __LINE__, adapter->flags);
	/* if SR-IOV is already disabled then there is nothing to do */
	/*if (!(adapter->flags & RNP_FLAG_SRIOV_ENABLED)) {
		printk("%s: sriov is not enabled\n", __func__);
		return 0;
	}*/

	dbg("%s:%d flags:0x%x\n", __func__, __LINE__, adapter->flags);
#ifdef CONFIG_PCI_IOV
	/*
	 * If our VFs are assigned we cannot shut down SR-IOV
	 * without causing issues, so just leave the hardware
	 * available but disabled
	 */
	if (rnp_vfs_are_assigned(adapter)) {
		e_dev_warn("Unloading driver while VFs are assigned - VFs will not be "
				   "deallocated\n");
		return -EPERM;
	}
	/* disable iov and allow time for transactions to clear */
	pci_disable_sriov(adapter->pdev);
#endif
	dbg("%s:%d flags:0x%x\n", __func__, __LINE__, adapter->flags);

	/* set default pool back to 0 */

	/* Disable VMDq flag so device will be set in VM mode */
	if (adapter->ring_feature[RING_F_VMDQ].limit == 1)
		adapter->flags &= ~RNP_FLAG_VMDQ_ENABLED;
	adapter->ring_feature[RING_F_VMDQ].offset = 0;

	rss = min_t(int, adapter->max_ring_pair_counts, num_online_cpus());

	rss =
		min_t(int, rss, hw->mac.max_msix_vectors - adapter->num_other_vectors);

	adapter->ring_feature[RING_F_RSS].limit = rss;

	/* take a breather then clean up driver data */
	msleep(100);
	//adapter->flags &= ~RNP_FLAG_SRIOV_ENABLED;

	dbg("%s:%d flags:0x%x\n", __func__, __LINE__, adapter->flags);
	return 0;
}

static int rnp_pci_sriov_enable(struct pci_dev *dev, int num_vfs)
{
#ifdef CONFIG_PCI_IOV
	struct rnp_adapter *adapter = pci_get_drvdata(dev);
	struct rnp_hw *hw = &adapter->hw;
	int err = 0;
	int i;
	int pre_existing_vfs = pci_num_vf(dev);

	if (pre_existing_vfs && pre_existing_vfs != num_vfs)
		err = rnp_disable_sriov(adapter);
	else if (pre_existing_vfs && pre_existing_vfs == num_vfs)
		goto out;

	// check vlan setup before sriov enable
	// maybe bug, if add 1 vlan, then open sriov
	if (adapter->vlan_count > 1) {
		dev_err(&adapter->pdev->dev,
				"only 1 vlan in sriov mode, delete other vlans\n");
		dev_err(&adapter->pdev->dev, "please delete all vlans first\n");

		err = -EOPNOTSUPP;
		goto err_out;
	}
	if (err)
		goto err_out;

	/* While the SR-IOV capability structure reports total VFs to be
	 * 64 we limit the actual number that can be allocated to 63 so
	 * that some transmit/receive resources can be reserved to the
	 * PF.  The PCI bus driver already checks for other values out of
	 * range.
	 */
	if (num_vfs > (hw->max_vfs - 1)) {
		err = -EPERM;
		goto err_out;
	}

	adapter->num_vfs = num_vfs;

	err = __rnp_enable_sriov(adapter);
	if (err)
		goto err_out;

	for (i = 0; i < adapter->num_vfs; i++)
		rnp_vf_configuration(dev, (i | 0x10000000));
	// we should reinit pf first
	dbg("flags:0x%x\n", adapter->flags);
	rnp_sriov_reinit(adapter);

	err = pci_enable_sriov(dev, num_vfs);
	if (err) {
		e_dev_warn("Failed to enable PCI sriov: %d num %d\n", err, num_vfs);
		rnp_disable_sriov(adapter);	
		rnp_sriov_reinit(adapter);
		goto err_out;
	}
	adapter->flags |= RNP_FLAG_SRIOV_INIT_DONE;
	// open rx here

out:
	return num_vfs;

err_out:
	return err;
#endif
	return 0;
}

static int rnp_pci_sriov_disable(struct pci_dev *dev)
{
	struct rnp_adapter *adapter = pci_get_drvdata(dev);
	int err;
	u32 current_flags = adapter->flags;

	err = rnp_disable_sriov(adapter);

	/* Only reinit if no error and state changed */
	if (!err && current_flags != adapter->flags) {
		/* rnp_disable_sriov() doesn't clear VMDQ flag */
		adapter->flags &= ~RNP_FLAG_VMDQ_ENABLED;
#ifdef CONFIG_PCI_IOV
		rnp_sriov_reinit(adapter);
#endif
	}

	return err;
}

static int
rnp_set_vf_multicasts(struct rnp_adapter *adapter, u32 *msgbuf, u32 vf)
{
	int entries = (msgbuf[0] & RNP_VT_MSGINFO_MASK) >> RNP_VT_MSGINFO_SHIFT;
	u16 *hash_list = (u16 *)&msgbuf[1];
	struct vf_data_storage *vfinfo = &adapter->vfinfo[vf];
	struct rnp_hw *hw = &adapter->hw;
	int i;
	u32 vector_bit;
	u32 vector_reg;
	u32 mta_reg;

	/* only so many hash values supported */
	entries = min(entries, RNP_MAX_VF_MC_ENTRIES);

	/*
	 * salt away the number of multi cast addresses assigned
	 * to this VF for later use to restore when the PF multi cast
	 * list changes
	 */
	vfinfo->num_vf_mc_hashes = entries;

	/*
	 * VFs are limited to using the MTA hash table for their multicast
	 * addresses
	 */
	for (i = 0; i < entries; i++)
		vfinfo->vf_mc_hashes[i] = hash_list[i];

	for (i = 0; i < vfinfo->num_vf_mc_hashes; i++) {
		// fixed mode
		hw->ops.set_sriov_vf_mc(hw, vfinfo->vf_mc_hashes[i]);
	}

	return 0;
}

void rnp_restore_vf_macvlans(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	struct list_head *pos;
	struct vf_macvlans *entry;

	hw_dbg(hw, "%s Staring..\n", __func__);

	list_for_each(pos, &adapter->vf_mvs.l)
	{
		entry = list_entry(pos, struct vf_macvlans, l);
		if (!entry->free) {
			hw_dbg(hw,
				   "  vf:%d MACVLAN: RAR[%d] <= %pM\n",
				   entry->vf,
				   entry->rar_entry,
				   entry->vf_macvlan);

#ifdef FIX_VF_BUG
			hw->ops.set_rar_with_vf(
				hw, entry->vf_macvlan, entry->rar_entry, entry->vf + 1, true);
#else
			hw->ops.set_rar_with_vf(
				hw, entry->vf_macvlan, entry->rar_entry, entry->vf, true);
#endif
		}
	}
	hw_dbg(hw, "%s Done\n", __func__);
}

void rnp_restore_vf_multicasts(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	struct vf_data_storage *vfinfo;
	int i, j;
	u32 vector_bit;
	u32 vector_reg;
	u32 mta_reg;
	/* Restore any VF macvlans */
	rnp_restore_vf_macvlans(adapter);
}

static int
rnp_set_vf_vlan(struct rnp_adapter *adapter, int add, int vid, u32 vf)
{
	struct rnp_hw *hw = &adapter->hw;
	int true_handle = 1;
	int i;
	/* VLAN 0 is a special case, don't allow it to be removed */
	if (!vid && !add)
		return 0;

	// should check other vf
	if ((adapter->flags & RNP_FLAG_SRIOV_ENABLED)) {

		/* if other vf use this vlan, don't true remove */
		if (!add) {
			// check equal pf_vlan?
			if (vid == adapter->vf_vlan)
				true_handle = 0;
			if (!test_and_set_bit(__RNP_USE_VFINFI, &adapter->state)) {
				for (i = 0; i < adapter->num_vfs; i++) {
					/* check if other vf_vlan still valid */
					if ((i != vf) && (vid == adapter->vfinfo[i].vf_vlan))
						true_handle = 0;
					/* check if other pf_vlan still valid */
					if ((i != vf) && (vid == adapter->vfinfo[i].pf_vlan))
						true_handle = 0;
				}
				clear_bit(__RNP_USE_VFINFI, &adapter->state);
			}
		}
	}
	if (true_handle)
		hw->ops.set_vf_vlan_filter(hw, vid, vf, (bool)add, false);
	return 0;
}

static s32 rnp_set_vf_lpe(struct rnp_adapter *adapter, u32 *msgbuf, u32 vf)
{
	struct rnp_hw *hw = &adapter->hw;
	int max_frame = msgbuf[1];
	u32 max_frs;

#if 0
	/*
	 * For n10EB we have to keep all PFs and VFs operating with
	 * the same max_frame value in order to avoid sending an oversize
	 * frame to a VF.  In order to guarantee this is handled correctly
	 * for all cases we have several special exceptions to take into
	 * account before we can enable the VF for receive
	 */
	if (adapter->hw.mac.type == rnp_mac_n10EB) {
		struct net_device *dev = adapter->netdev;
		int pf_max_frame = dev->mtu + ETH_HLEN;
		u32 reg_offset, vf_shift, vfre;
		s32 err = 0;

		switch (adapter->vfinfo[vf].vf_api) {
		case rnp_mbox_api_11:
			/*
			 * Version 1.1 supports jumbo frames on VFs if PF has
			 * jumbo frames enabled which means legacy VFs are
			 * disabled
			 */
			if (pf_max_frame > ETH_FRAME_LEN)
				break;
		default:
			/*
			 * If the PF or VF are running w/ jumbo frames enabled
			 * we need to shut down the VF Rx path as we cannot
			 * support jumbo frames on legacy VFs
			 */
			if ((pf_max_frame > ETH_FRAME_LEN) ||
			    (max_frame > (ETH_FRAME_LEN + ETH_FCS_LEN)))
				err = -EINVAL;
			break;
		}

		/* determine VF receive enable location */
		vf_shift = vf % 32;
		reg_offset = vf / 32;

		/* enable or disable receive depending on error */
		vfre = RNP_READ_REG(hw, RNP_VFRE(reg_offset));
		if (err)
			vfre &= ~(1 << vf_shift);
		else
			vfre |= 1 << vf_shift;
		//RNP_WRITE_REG(hw, RNP_VFRE(reg_offset), vfre);

		if (err) {
			e_err(drv, "VF max_frame %d out of range\n", max_frame);
			return err;
		}
	}

	/* MTU < 68 is an error and causes problems on some kernels */
	if (max_frame > RNP_MAX_JUMBO_FRAME_SIZE) {
		e_err(drv, "VF max_frame %d out of range\n", max_frame);
		return -EINVAL;
	}

	/* pull current max frame size from hardware */
	max_frs = RNP_READ_REG(hw, RNP_MAXFRS);
	max_frs &= RNP_MHADD_MFS_MASK;
	max_frs >>= RNP_MHADD_MFS_SHIFT;

	if (max_frs < max_frame) {
		max_frs = max_frame << RNP_MHADD_MFS_SHIFT;
		//RNP_WRITE_REG(hw, RNP_MAXFRS, max_frs);
	}

	e_info(hw, "VF requests change max MTU to %d\n", max_frame);
#endif
	return 0;
}

static void rnp_set_vmolr(struct rnp_hw *hw, u32 vf, bool aupe)
{
#if 0
	u32 vmolr = RNP_READ_REG(hw, RNP_VMOLR(vf));

	vmolr |= (RNP_VMOLR_ROMPE |
		  RNP_VMOLR_BAM);
	if (aupe)
		vmolr |= RNP_VMOLR_AUPE;
	else
		vmolr &= ~RNP_VMOLR_AUPE;
	RNP_WRITE_REG(hw, RNP_VMOLR(vf), vmolr);
#endif
}

static void rnp_clear_vmvir(struct rnp_adapter *adapter, u32 vf)
{
	struct rnp_hw *hw = &adapter->hw;

	// RNP_WRITE_REG(hw, RNP_VMVIR(vf), 0);
}
static inline void rnp_vf_reset_event(struct rnp_adapter *adapter, u32 vf)
{
	struct rnp_hw *hw = &adapter->hw;
	struct vf_data_storage *vfinfo = &adapter->vfinfo[vf];
	int rar_entry = hw->mac.num_rar_entries - (vf + 1);
	u8 num_tcs = netdev_get_num_tc(adapter->netdev);
	int i;

	/* add PF assigned VLAN or VLAN 0 */
	//	rnp_set_vf_vlan(adapter, true, vfinfo->pf_vlan, vf);

	/* reset offloads to defaults */
	//	rnp_set_vmolr(hw, vf, !vfinfo->pf_vlan);

	/* set outgoing tags for VFs */
	/*
	if (!vfinfo->pf_vlan && !vfinfo->pf_qos && !num_tcs) {
		//		rnp_clear_vmvir(adapter, vf);
	} else {
		if (vfinfo->pf_qos || !num_tcs)
			rnp_set_vmvir(adapter, vfinfo->pf_vlan, vfinfo->pf_qos, vf);
		else
			rnp_set_vmvir(adapter, vfinfo->pf_vlan, adapter->default_up, vf);

		// if (vfinfo->spoofchk_enabled)
		//	hw->mac.ops.set_vlan_anti_spoofing(hw, true, vf);
	}*/

	/* reset multicast table array for vf */
	adapter->vfinfo[vf].num_vf_mc_hashes = 0;

	/* Flush and reset the mta with the new values */
	rnp_set_rx_mode(adapter->netdev);

	/* clear this rar_entry */
	// hw->mac.ops.clear_rar(hw, rar_entry);
	hw->ops.clr_rar(hw, rar_entry);

	/* reset VF api back to unknown */
	adapter->vfinfo[vf].vf_api = 0;
	// clear vf multicast
	for (i = 0; i < RNP_MAX_VF_MC_ENTRIES; i++)
		adapter->vfinfo[vf].vf_mc_hashes[i] = 0;
	// clear vf vlan setup
	adapter->vfinfo[vf].vf_vlan = 0;
	adapter->vfinfo[vf].vlan_count = 0;
}

static int
rnp_set_vf_mac(struct rnp_adapter *adapter, int vf, unsigned char *mac_addr)
{
	struct rnp_hw *hw = &adapter->hw;
	/* this rar_entry may be cofict with mac vlan with pf */
	int rar_entry = hw->mac.num_rar_entries - (vf + 1);

	memcpy(adapter->vfinfo[vf].vf_mac_addresses, mac_addr, 6);

	/* setup to the hw */
#ifdef FIX_VF_BUG
	hw->ops.set_rar_with_vf(hw, mac_addr, rar_entry, vf + 1, true);
#else
	hw->ops.set_rar_with_vf(hw, mac_addr, rar_entry, vf, true);
#endif

	return 0;
}

static int rnp_set_vf_macvlan(struct rnp_adapter *adapter,
							  int vf,
							  int index,
							  unsigned char *mac_addr)
{
	struct rnp_hw *hw = &adapter->hw;
	struct list_head *pos;
	struct vf_macvlans *entry;
	// index = 0 , only earase
	// index = 1 , earase and then set
	if (index <= 1) {
		list_for_each(pos, &adapter->vf_mvs.l)
		{
			entry = list_entry(pos, struct vf_macvlans, l);
			if (entry->vf == vf) {
				entry->vf = -1;
				entry->free = true;
				entry->is_macvlan = false;
				hw->ops.clr_rar(hw, entry->rar_entry);
				// hw->mac.ops.clear_rar(hw, entry->rar_entry);
			}
		}
	}

	/*
	 * If index was zero then we were asked to clear the uc list
	 * for the VF.  We're done.
	 */
	if (!index)
		return 0;

	entry = NULL;

	list_for_each(pos, &adapter->vf_mvs.l)
	{
		entry = list_entry(pos, struct vf_macvlans, l);
		if (entry->free)
			break;
	}

	/*
	 * If we traversed the entire list and didn't find a free entry
	 * then we're out of space on the RAR table.  Also entry may
	 * be NULL because the original memory allocation for the list
	 * failed, which is not fatal but does mean we can't support
	 * VF requests for MACVLAN because we couldn't allocate
	 * memory for the list management required.
	 */
	if (!entry || !entry->free)
		return -ENOSPC;

	entry->free = false;
	entry->is_macvlan = true;
	entry->vf = vf;
	memcpy(entry->vf_macvlan, mac_addr, ETH_ALEN);
#ifdef FIX_VF_BUG
	hw->ops.set_rar_with_vf(
		hw, entry->vf_macvlan, entry->rar_entry, entry->vf + 1, true);
#else
	hw->ops.set_rar_with_vf(
		hw, entry->vf_macvlan, entry->rar_entry, entry->vf, true);
#endif
	return 0;
}

int rnp_vf_configuration(struct pci_dev *pdev, unsigned int event_mask)
{
	unsigned char vf_mac_addr[6];
	struct rnp_adapter *adapter = pci_get_drvdata(pdev);
	unsigned int vfn = (event_mask & 0x3f);

	bool enable = ((event_mask & 0x10000000U) != 0);

	if (enable) {
		eth_zero_addr(vf_mac_addr);
		memcpy(vf_mac_addr, adapter->hw.mac.perm_addr, 6);
		vf_mac_addr[5] = vf_mac_addr[5] + (0x80 | vfn);
		vf_mac_addr[4] = vf_mac_addr[4] + (pdev->devfn); 

		memcpy(adapter->vfinfo[vfn].vf_mac_addresses, vf_mac_addr, 6);
	}

	return 0;
}

static int rnp_vf_reset_msg(struct rnp_adapter *adapter, u32 vf)
{
	struct rnp_hw *hw = &adapter->hw;
	unsigned char *vf_mac = adapter->vfinfo[vf].vf_mac_addresses;
	u32 reg, msgbuf[RNP_VF_PERMADDR_MSG_LEN];
	u32 reg_offset, vf_shift;
	u8 *addr = (u8 *)(&msgbuf[1]);

	/* reset the filters for the device */
	rnp_vf_reset_event(adapter, vf);

	/* set vf mac address */
	if (!is_zero_ether_addr(vf_mac))
		rnp_set_vf_mac(adapter, vf, vf_mac);

#if 0
	/* enable transmit for vf */

	/* enable receive for vf */
	/*
	 * The n10 cannot support a mix of jumbo and non-jumbo PF/VFs.
	 * For more info take a look at rnp_set_vf_lpe
	 */
	if (adapter->hw.mac.type == rnp_mac_n10EB) {
		struct net_device *dev = adapter->netdev;
		int pf_max_frame = dev->mtu + ETH_HLEN;

		if (pf_max_frame > ETH_FRAME_LEN)
			reg &= ~(1 << vf_shift);
	}
	RNP_WRITE_REG(hw, RNP_VFRE(reg_offset), reg);
#endif

	/* enable VF mailbox for further messages */
	adapter->vfinfo[vf].clear_to_send = true;

	/* Enable counting of spoofed packets in the SSVPC register */

	/* reply to reset with ack and vf mac address */
	msgbuf[0] = RNP_VF_RESET;
	if (!is_zero_ether_addr(vf_mac)) {
		msgbuf[0] |= RNP_VT_MSGTYPE_ACK;
		memcpy(addr, vf_mac, ETH_ALEN);
	} else {
		msgbuf[0] |= RNP_VT_MSGTYPE_NACK;
		// fixme
		dev_warn(&adapter->pdev->dev,
				 "VF %d has no MAC address assigned, you may have to assign "
				 "one manually\n",
				 vf);
	}

	/*
	 * Piggyback the multicast filter type so VF can compute the
	 * correct vectors
	 */
	msgbuf[RNP_VF_MC_TYPE_WORD] = 0;
	/* setup link status , pause mode, ft padding mode */

	/* link status */
	// to-do
	/* pause mode */
	msgbuf[RNP_VF_MC_TYPE_WORD] |= (0xff & hw->fc.current_mode) << 16;
	if (adapter->priv_flags & RNP_PRIV_FLAG_FT_PADDING)
		msgbuf[RNP_VF_MC_TYPE_WORD] |= (0x01 << 8);
	else
		msgbuf[RNP_VF_MC_TYPE_WORD] |= (0x00 << 8);
	/* mc_type */
	msgbuf[RNP_VF_MC_TYPE_WORD] |= rd32(hw, RNP_ETH_DMAC_MCSTCTRL) & 0x03;

	msgbuf[RNP_VF_DMA_VERSION_WORD] = rd32(hw, RNP_DMA_VERSION);

	msgbuf[RNP_VF_VLAN_WORD] = adapter->vfinfo[vf].pf_vlan;
	/* fixme tx fetch to be added here */
	msgbuf[RNP_VF_PHY_TYPE_WORD] = (hw->mac_type << 16) | hw->phy_type;

	msgbuf[RNP_VF_FW_VERSION_WORD] = (hw->fw_version);

	msgbuf[RNP_VF_LINK_STATUS_WORD] =
		(adapter->link_up ? RNP_PF_LINK_UP : 0) | adapter->link_speed;

	msgbuf[RNP_VF_AXI_MHZ] = hw->usecstocount;
#ifdef NETIF_F_HW_VLAN_CTAG_FILTER
	if (adapter->netdev->features & NETIF_F_HW_VLAN_CTAG_FILTER)
		msgbuf[RNP_VF_FEATURE] |= PF_FEATRURE_VLAN_FILTER;
#endif

	// msgbuf[RNP_VF_RING_PARAM] = (adapter->tx_ring_item_count << 16) |
	//	adapter->rx_ring_item_count;
	/* now vf maybe has no irq handler if it is the first reset*/
	rnp_write_mbx(hw, msgbuf, RNP_VF_PERMADDR_MSG_LEN, vf);

	return 0;
}

static int rnp_get_vf_mac_addr(struct rnp_adapter *adapter, u32 *msgbuf, u32 vf)
{
	u8 *mac = ((u8 *)(&msgbuf[1]));

	memcpy(mac, adapter->vfinfo[vf].vf_mac_addresses, 6);

	return 0;
}

/* vf call setup a new mac */
static int rnp_set_vf_mac_addr(struct rnp_adapter *adapter, u32 *msgbuf, u32 vf)
{
	u8 *new_mac = ((u8 *)(&msgbuf[1]));

	if (!is_valid_ether_addr(new_mac)) {
		e_warn(drv, "VF %d attempted to set invalid mac\n", vf);
		return -1;
	}

	if (adapter->vfinfo[vf].pf_set_mac &&
		memcmp(adapter->vfinfo[vf].vf_mac_addresses, new_mac, ETH_ALEN)) {
		e_warn(drv,
			   "VF %d attempted to override administratively set MAC address\n"
			   "Reload the VF driver to resume operations\n",
			   vf);
		return -1;
	}
	rnp_set_vf_mac(adapter, vf, new_mac);

	return 0;
}

static int rnp_set_vf_vlan_msg(struct rnp_adapter *adapter, u32 *msgbuf, u32 vf)
{
	struct rnp_hw *hw = &adapter->hw;
	int add = ((msgbuf[0] & RNP_VT_MSGINFO_MASK) >> RNP_VT_MSGINFO_SHIFT);
	int vid = (msgbuf[1] & RNP_VLVF_VLANID_MASK);
	int err;
	// u8 tcs = netdev_get_num_tc(adapter->netdev);

	if (adapter->vfinfo[vf].pf_vlan) {
		e_warn(drv,
			   "VF %d attempted to override administratively set VLAN "
			   "configuration\n"
			   "Reload the VF driver to resume operations\n",
			   vf);
		return -1;
	}
	// only allow 1 vlan for each vf
	if ((add) && (adapter->vfinfo[vf].vlan_count)) {
		e_warn(drv, "VF %d attempted to set more than 1 vlan", vf);
		e_warn(drv,
			   " vlan now %d, try to set %d\n",
			   adapter->vfinfo[vf].vf_vlan,
			   vid);
		return -1;
	}

	// vlan 0 has no work todo
	if (!vid)
		return 0;
	if (add) {
		adapter->vfinfo[vf].vlan_count++;
		// store vf vlan setup
		adapter->vfinfo[vf].vf_vlan = vid;
	} else if (adapter->vfinfo[vf].vlan_count) {
		adapter->vfinfo[vf].vf_vlan = 0;
		adapter->vfinfo[vf].vlan_count--;
	}

	err = rnp_set_vf_vlan(adapter, add, vid, vf);

	return err;
}

static int
rnp_set_vf_vlan_strip_msg(struct rnp_adapter *adapter, u32 *msgbuf, u32 vf)
{
	struct rnp_hw *hw = &adapter->hw;
	int vlan_strip_on = !!(msgbuf[1] >> 31);
	int queue_cnt = msgbuf[1] & 0xffff;
	int err = 0, i;

	vf_dbg("strip_on:%d queeu_cnt:%d, %d %d\n",
		   vlan_strip_on,
		   queue_cnt,
		   msgbuf[2],
		   msgbuf[3]);

	/* vf pf use different rings don't check this */
	/*
	if (vlan_strip_on &&
		!(adapter->netdev->features & NETIF_F_HW_VLAN_CTAG_RX)) {
		err = -1;
		return err;
	}*/

	for (i = 0; i < queue_cnt; i++) {
		if (vlan_strip_on)
			hw->ops.set_vlan_strip(hw, msgbuf[2 + i], true);
		else
			hw->ops.set_vlan_strip(hw, msgbuf[2 + i], false);
	}

	return err;
}

static int
rnp_set_vf_macvlan_msg(struct rnp_adapter *adapter, u32 *msgbuf, u32 vf)
{
	u8 *new_mac = ((u8 *)(&msgbuf[1]));
	int index = (msgbuf[0] & RNP_VT_MSGINFO_MASK) >> RNP_VT_MSGINFO_SHIFT;
	int err;

	if (adapter->vfinfo[vf].pf_set_mac && index > 0) {
		e_warn(
			drv,
			"VF %d requested MACVLAN filter but is administratively denied\n",
			vf);
		return -1;
	}

	/* An non-zero index indicates the VF is setting a filter */
	if (index) {
		if (!is_valid_ether_addr(new_mac)) {
			e_warn(drv, "VF %d attempted to set invalid mac\n", vf);
			return -1;
		}
	}

	err = rnp_set_vf_macvlan(adapter, vf, index, new_mac);
	if (err == -ENOSPC)
		e_warn(drv,
			   "VF %d has requested a MACVLAN filter but there is no space for "
			   "it\n",
			   vf);

	return err < 0;

	return 0;
}

static int
rnp_negotiate_vf_api(struct rnp_adapter *adapter, u32 *msgbuf, u32 vf)
{
	adapter->vfinfo[vf].vf_api = 0;

	return 0;
}

static int rnp_get_vf_reg(struct rnp_adapter *adapter, u32 *msgbuf, u32 vf)
{
	struct net_device *dev = adapter->netdev;
	u32 reg = msgbuf[1];

	msgbuf[1] = rd32(&adapter->hw, reg);

	return 0;
}

static int rnp_set_vf_mtu(struct rnp_adapter *adapter, u32 *msgbuf, u32 vf)
{
	struct net_device *netdev = adapter->netdev;
	if (msgbuf[1] > netdev->mtu) {
		e_dev_warn("vf %d try to change %d mtu to %d (large than pf limit)\n",
				   vf,
				   netdev->mtu,
				   msgbuf[1]);
		return -1;
	} else
		return 0;
}

static int rnp_get_vf_mtu(struct rnp_adapter *adapter, u32 *msgbuf, u32 vf)
{
	struct net_device *netdev = adapter->netdev;
	msgbuf[1] = netdev->mtu;
	return 0;
}

static int rnp_get_vf_fw(struct rnp_adapter *adapter, u32 *msgbuf, u32 vf)
{
	struct rnp_hw *hw = &adapter->hw;

	msgbuf[1] = hw->fw_version;

	return 0;
}

static int rnp_get_vf_link(struct rnp_adapter *adapter, u32 *msgbuf, u32 vf)
{
	struct rnp_hw *hw = &adapter->hw;

	msgbuf[1] = (adapter->link_up ? RNP_PF_LINK_UP : 0) | adapter->link_speed;

	return 0;
}

static int rnp_get_vf_queues(struct rnp_adapter *adapter, u32 *msgbuf, u32 vf)
{
	struct net_device *dev = adapter->netdev;
	struct rnp_ring_feature *vmdq = &adapter->ring_feature[RING_F_VMDQ];
	unsigned int default_tc = 0;
	struct rnp_hw *hw = &adapter->hw;

	msgbuf[RNP_VF_TX_QUEUES] = hw->sriov_ring_limit;
	msgbuf[RNP_VF_RX_QUEUES] = hw->sriov_ring_limit;
	msgbuf[RNP_VF_TRANS_VLAN] = adapter->vfinfo[vf].pf_vlan;
	msgbuf[RNP_VF_DEF_QUEUE] = 0;
#ifdef FIX_VF_BUG
	// maybe new variable should be added
	msgbuf[RNP_VF_QUEUE_START] = vf * 2 + 2;
#else
	msgbuf[RNP_VF_QUEUE_START] = vf * 2;
#endif
	msgbuf[RNP_VF_QUEUE_DEPTH] =
		(adapter->tx_ring_item_count << 16) | adapter->rx_ring_item_count;

	//	u8 num_tcs = netdev_get_num_tc(dev);
	//
	//	/* verify the PF is supporting the correct APIs */
	//
	//	/* only allow 1 Tx queue for bandwidth limiting */
	//	msgbuf[RNP_VF_TX_QUEUES] = 1;
	//	msgbuf[RNP_VF_RX_QUEUES] = 1;
	//
	//	/* if TCs > 1 determine which TC belongs to default user priority */
	//	if (num_tcs > 1)
	//		default_tc = netdev_get_prio_tc_map(dev, adapter->default_up);
	//
	//	/* notify VF of need for VLAN tag stripping, and correct queue */
	//	if (num_tcs)
	//		msgbuf[RNP_VF_TRANS_VLAN] = num_tcs;
	//	else if (adapter->vfinfo[vf].pf_vlan || adapter->vfinfo[vf].pf_qos)
	//		msgbuf[RNP_VF_TRANS_VLAN] = 1;
	//	else
	//		msgbuf[RNP_VF_TRANS_VLAN] = 0;
	//
	//	/* notify VF of default queue */
	//	msgbuf[RNP_VF_DEF_QUEUE] = default_tc;

	return 0;
}

static int rnp_rcv_msg_from_vf(struct rnp_adapter *adapter, u32 vf)
{
	u32 mbx_size = RNP_VFMAILBOX_SIZE;
	u32 msgbuf[RNP_VFMAILBOX_SIZE];
	struct rnp_hw *hw = &adapter->hw;
	s32 retval;

	vf_dbg("msg from vf:%d\n", vf);

	retval = rnp_read_mbx(hw, msgbuf, mbx_size, vf);
	if (retval) {
		pr_err("Error receiving message from VF\n");
		return retval;
	}
	vf_dbg("msg[0]=0x%08x\n", msgbuf[0]);

	/* this is a message we already processed, do nothing */
	if (msgbuf[0] & (RNP_VT_MSGTYPE_ACK | RNP_VT_MSGTYPE_NACK))
		return retval;

	/* flush the ack before we write any messages back */

	/* clear vf_num */
	msgbuf[0] &= (~RNP_VF_MASK);

	/* this is a vf reset irq */
	if ((msgbuf[0] & RNP_MAIL_CMD_MASK) == RNP_VF_RESET) {
		vf_dbg("vf %d up\n", vf);
		return rnp_vf_reset_msg(adapter, vf);
	}

	/*
	 * until the vf completes a virtual function reset it should not be
	 * allowed to start any configuration.
	 */
	if (!adapter->vfinfo[vf].clear_to_send) {
		vf_dbg("wait vf clear to send\n");
		msgbuf[0] |= RNP_VT_MSGTYPE_NACK;
		rnp_write_mbx(hw, msgbuf, 1, vf);
		return retval;
	}

	switch ((msgbuf[0] & RNP_MAIL_CMD_MASK)) {
		case RNP_VF_SET_MAC_ADDR:
			retval = rnp_set_vf_mac_addr(adapter, msgbuf, vf);
			break;
		case RNP_VF_SET_MULTICAST:
			retval = rnp_set_vf_multicasts(adapter, msgbuf, vf);
			break;
		case RNP_VF_SET_VLAN:
			retval = rnp_set_vf_vlan_msg(adapter, msgbuf, vf);
			break;
		case RNP_VF_SET_VLAN_STRIP:
			retval = rnp_set_vf_vlan_strip_msg(adapter, msgbuf, vf);
			break;
		case RNP_VF_SET_LPE: // no need
			retval = rnp_set_vf_lpe(adapter, msgbuf, vf);
			break;
		case RNP_VF_GET_MACADDR:
			retval = rnp_get_vf_mac_addr(adapter, msgbuf, vf);
			break;
		case RNP_VF_SET_MACVLAN:
			retval = rnp_set_vf_macvlan_msg(adapter, msgbuf, vf);
			break;
		case RNP_VF_API_NEGOTIATE:
			retval = rnp_negotiate_vf_api(adapter, msgbuf, vf);
			break;
		case RNP_VF_GET_QUEUES:
			retval = rnp_get_vf_queues(adapter, msgbuf, vf);
			break;
		case RNP_VF_REG_RD:
			retval = rnp_get_vf_reg(adapter, msgbuf, vf);
			break;
		case RNP_VF_GET_MTU:
			retval = rnp_get_vf_mtu(adapter, msgbuf, vf);
			break;
		case RNP_VF_SET_MTU:
			retval = rnp_set_vf_mtu(adapter, msgbuf, vf);
			break;
		case RNP_VF_GET_FW:
			retval = rnp_get_vf_fw(adapter, msgbuf, vf);
			break;
		case RNP_VF_GET_LINK:
			retval = rnp_get_vf_link(adapter, msgbuf, vf);
			break;
		case RNP_PF_REMOVE:
			vf_dbg("vf %d removed\n", vf);
			adapter->vfinfo[vf].clear_to_send = false;
			retval = 1;
			break;
		case RNP_VF_RESET_PF:
			adapter->flags2 |= RNP_FLAG2_RESET_PF;
			retval = 1;
			break;
		default:
			e_err(drv, "Unhandled Msg %8.8x\n", msgbuf[0]);
			retval = RNP_ERR_MBX;
			break;
	}

	/* notify the VF of the results of what it sent us */
	if (retval)
		msgbuf[0] |= RNP_VT_MSGTYPE_NACK;
	else
		msgbuf[0] |= RNP_VT_MSGTYPE_ACK;

	// dbg("msg buf is %x\n", msgbuf[0]);
	/* write vf_num */
	msgbuf[0] |= (vf << 21);

	msgbuf[0] |= RNP_VT_MSGTYPE_CTS;

	if ((msgbuf[0] & RNP_MAIL_CMD_MASK) != RNP_PF_REMOVE)
		rnp_write_mbx(hw, msgbuf, mbx_size, vf);

	return retval;
}

static void rnp_rcv_ack_from_vf(struct rnp_adapter *adapter, u32 vf)
{
	struct rnp_hw *hw = &adapter->hw;
	u32 msg = RNP_VT_MSGTYPE_NACK;

	/* if device isn't clear to send it shouldn't be reading either */
	if (!adapter->vfinfo[vf].clear_to_send)
		rnp_write_mbx(hw, &msg, 1, vf);
}

void rnp_msg_task(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	u32 vf;

	// printk("msg irq !!!!\n");
	rnp_fw_msg_handler(adapter);

	for (vf = 0; vf < adapter->num_vfs; vf++) {
		/* process any reset requests */
		// if (!rnp_check_for_rst(hw, vf))
		//	rnp_vf_reset_event(adapter, vf);

		/* process any messages pending */
		if (!rnp_check_for_msg(hw, vf))
			rnp_rcv_msg_from_vf(adapter, vf);

		/* process any acks */
		if (!rnp_check_for_ack(hw, vf))
			rnp_rcv_ack_from_vf(adapter, vf);
	}
}

void rnp_msg_post_status_signle(struct rnp_adapter *adapter,
								enum PF_STATUS status,
								int vf)
{
	u32 msgbuf[RNP_VFMAILBOX_SIZE];
	struct rnp_hw *hw = &adapter->hw;
	struct rnp_mbx_info *mbx = &hw->mbx;
	switch (status) {
		case PF_FCS_STATUS:
			msgbuf[0] = RNP_PF_SET_FCS | (vf << RNP_VNUM_OFFSET);
			if (adapter->netdev->features & NETIF_F_RXFCS)
				msgbuf[1] = 1;
			else
				msgbuf[1] = 0;
			break;
		case PF_PAUSE_STATUS:
			msgbuf[0] = RNP_PF_SET_PAUSE | (vf << RNP_VNUM_OFFSET);
			msgbuf[1] = hw->fc.requested_mode;
			break;
		case PF_FT_PADDING_STATUS:
			msgbuf[0] = RNP_PF_SET_FT_PADDING | (vf << RNP_VNUM_OFFSET);
			if (adapter->priv_flags & RNP_PRIV_FLAG_FT_PADDING) {
				msgbuf[1] = 1;
			} else {
				msgbuf[1] = 0;
			}

			break;
		case PF_VLAN_FILTER_STATUS:
			msgbuf[0] = RNP_PF_SET_VLAN_FILTER | (vf << RNP_VNUM_OFFSET);
#ifdef NETIF_F_HW_VLAN_CTAG_FILTER
			if (adapter->netdev->features & NETIF_F_HW_VLAN_CTAG_FILTER) {
				msgbuf[1] = 1;
			} else {
				msgbuf[1] = 0;
			}
#else
			msgbuf[1] = 0;
#endif

			break;
		case PF_SET_VLAN_STATUS:
			msgbuf[0] = RNP_PF_SET_VLAN | (vf << RNP_VNUM_OFFSET);

			msgbuf[1] = adapter->vfinfo[vf].pf_vlan;
			break;
		case PF_SET_LINK_STATUS:
			msgbuf[0] = RNP_PF_SET_LINK | (vf << RNP_VNUM_OFFSET);
			if (adapter->link_up) {
				msgbuf[1] = RNP_PF_LINK_UP | adapter->link_speed;
			} else {
				msgbuf[1] = 0;
			}
			break;
		case PF_SET_MTU:
			msgbuf[0] = RNP_PF_SET_MTU | (vf << RNP_VNUM_OFFSET);
			msgbuf[1] = adapter->netdev->mtu;
			break;
		case PF_SET_RESET:
			msgbuf[0] = RNP_PF_SET_RESET | (vf << RNP_VNUM_OFFSET);
			msgbuf[1] = 0;

			break;
	}
	// dbg("msg 0 is %x\n", msgbuf[0]);
	// dbg("msg 1 is %x\n", msgbuf[1]);
	//
	mbx->ops.write(hw, msgbuf, 2, vf);
}

/* try to send mailbox to all active vf */
void rnp_msg_post_status(struct rnp_adapter *adapter, enum PF_STATUS status)
{
	u32 vf;

	if (adapter->flags & RNP_FLAG_SRIOV_ENABLED) {
		/* broadcast */
		for (vf = 0; vf < adapter->num_vfs; vf++) {
			if (adapter->vfinfo[vf].clear_to_send) {
				rnp_msg_post_status_signle(adapter, status, vf);
			}
		}
	}
}

void rnp_disable_tx_rx(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;

	/* disable transmit and receive for all vfs */
}

void rnp_ping_all_vfs(struct rnp_adapter *adapter)
{
	struct rnp_hw *hw = &adapter->hw;
	u32 ping;
	int i;

	for (i = 0; i < adapter->num_vfs; i++) {
		ping = RNP_PF_CONTROL_PRING_MSG;
		/* only send to active vf */
		// if (adapter->vfinfo[i].clear_to_send) {
		ping |= RNP_VT_MSGTYPE_CTS;
		rnp_write_mbx(hw, &ping, 1, i);
		//	}
	}
}

int rnp_get_vf_ringnum(int vf, int num)
{
#ifdef FIX_VF_BUG
	return (vf * 2 + 2 + num);
#else
	return (vf * 2 + num);
#endif
}

int rnp_setup_ring_maxrate(struct rnp_adapter *adapter, int ring, u64 max_rate)
{
	struct rnp_hw *hw = &adapter->hw;
	struct rnp_dma_info *dma = &hw->dma;
	int samples_1sec = adapter->hw.usecstocount * 1000000;

	dma_ring_wr32(
		dma, RING_OFFSET(ring) + RNP_DMA_REG_TX_FLOW_CTRL_TM, samples_1sec);
	dma_ring_wr32(
		dma, RING_OFFSET(ring) + RNP_DMA_REG_TX_FLOW_CTRL_TH, max_rate);
	return 0;
}

static int rnp_disable_port_vlan(struct rnp_adapter *adapter, int vf)
{
	struct rnp_hw *hw = &adapter->hw;
	int err;
	int port;

	err = rnp_set_vf_vlan(adapter, false, adapter->vfinfo[vf].pf_vlan, vf);

	adapter->vfinfo[vf].pf_vlan = 0;
	adapter->vfinfo[vf].pf_qos = 0;
	/* clear veb */
	hw->ops.set_vf_vlan_filter(hw, 0, vf, false, true);

	return err;
}

static int
rnp_enable_port_vlan(struct rnp_adapter *adapter, int vf, u16 vlan, u8 qos)
{
	struct rnp_hw *hw = &adapter->hw;
	int err, port;

	err = rnp_set_vf_vlan(adapter, true, vlan, vf);
	if (err)
		goto out;

	adapter->vfinfo[vf].pf_vlan = vlan;
	adapter->vfinfo[vf].pf_qos = qos;
	dev_info(pci_dev_to_dev(adapter->pdev),
			 "Setting VLAN %d, QOS 0x%x on VF %d\n",
			 vlan,
			 qos,
			 vf);
	if (test_bit(__RNP_DOWN, &adapter->state)) {
		dev_warn(pci_dev_to_dev(adapter->pdev),
				 "The VF VLAN has been set, but the PF device is not up.\n");
		dev_warn(
			pci_dev_to_dev(adapter->pdev),
			"Bring the PF device up before attempting to use the VF device.\n");
	}
	// setup veb only
	hw->ops.set_vf_vlan_filter(hw, vlan, vf, true, true);

out:
	return err;
}

#ifdef IFLA_VF_VLAN_INFO_MAX
int rnp_ndo_set_vf_vlan(
	struct net_device *netdev, int vf, u16 vlan, u8 qos, __be16 vlan_proto)
#else
int rnp_ndo_set_vf_vlan(struct net_device *netdev, int vf, u16 vlan, u8 qos)
#endif
{
	int err = 0;
	struct rnp_adapter *adapter = netdev_priv(netdev);

	/* VLAN IDs accepted range 0-4094 */
	if (vf < 0 || vf >= adapter->num_vfs || vlan > VLAN_VID_MASK - 1 || qos > 7)
		return -EINVAL;

#ifdef IFLA_VF_VLAN_INFO_MAX
	if (vlan_proto != htons(ETH_P_8021Q))
		return -EPROTONOSUPPORT;
#endif
	if (vlan || qos) {
		/*
		 * Check if there is already a port VLAN set, if so
		 * we have to delete the old one first before we
		 * can set the new one.  The usage model had
		 * previously assumed the user would delete the
		 * old port VLAN before setting a new one but this
		 * is not necessarily the case.
		 */
		if (adapter->vfinfo[vf].vf_vlan) {
			dev_err(&adapter->pdev->dev,
					"vf set vlan before, delete it before add new\n");
			err = -EINVAL;
			goto out;
		}
		if (adapter->vfinfo[vf].pf_vlan)
			err = rnp_disable_port_vlan(adapter, vf);
		if (err)
			goto out;
		err = rnp_enable_port_vlan(adapter, vf, vlan, qos);

	} else {
		err = rnp_disable_port_vlan(adapter, vf);
	}
	// send mbx to vf
	rnp_msg_post_status_signle(adapter, PF_SET_VLAN_STATUS, vf);
out:
	return err;
}

#ifdef HAVE_NDO_SET_VF_MIN_MAX_TX_RATE
int rnp_ndo_set_vf_bw(struct net_device *netdev,
					  int vf,
					  int __always_unused min_tx_rate,
					  int max_tx_rate)
#else
int rnp_ndo_set_vf_bw(struct net_device *netdev, int vf, int max_tx_rate)
#endif /* HAVE_NDO_SET_VF_MIN_MAX_TX_RATE */
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	/* limit vf ring rate */
	int ring_max_rate;
	int vf_ring;
	int link_speed = 0;
	u64 real_rate = 0;

	if (vf >= RNP_MAX_VF_CNT - 1)
		return -EINVAL;

	// todo
	switch (adapter->link_speed) {
		case RNP_LINK_SPEED_40GB_FULL:
			link_speed = 40000;
			break;
		case RNP_LINK_SPEED_25GB_FULL:
			link_speed = 25000;
			break;
		case RNP_LINK_SPEED_10GB_FULL:
			link_speed = 10000;
			break;
		case RNP_LINK_SPEED_1GB_FULL:
			link_speed = 1000;
			break;
		case RNP_LINK_SPEED_100_FULL:
			link_speed = 100;
			break;
	}
	// link_speed = rnp_link_mbps(adapter);
	/* rate limit cannot be less than 10Mbs or greater than link speed */
	if (max_tx_rate && ((max_tx_rate <= 10) || (max_tx_rate > link_speed)))
		return -EINVAL;

	ring_max_rate = max_tx_rate / PF_RING_CNT_WHEN_IOV_ENABLED;

	real_rate = (ring_max_rate * 1024 * 1024) / 8;

	vf_ring = rnp_get_vf_ringnum(vf, 0);
	rnp_setup_ring_maxrate(adapter, vf_ring, real_rate);
	vf_ring = rnp_get_vf_ringnum(vf, 1);
	rnp_setup_ring_maxrate(adapter, vf_ring, real_rate);
	return 0;
}

int rnp_ndo_set_vf_mac(struct net_device *netdev, int vf, u8 *mac)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);

	if (!is_valid_ether_addr(mac) || (vf >= adapter->num_vfs))
		return -EINVAL;
	adapter->vfinfo[vf].pf_set_mac = true;
	dev_info(&adapter->pdev->dev, "setting MAC %pM on VF %d\n", mac, vf);
	dev_info(&adapter->pdev->dev,
			 "Reload the VF driver to make this"
			 " change effective.");
	if (test_bit(__RNP_DOWN, &adapter->state)) {
		dev_warn(&adapter->pdev->dev,
				 "The VF MAC address has been set,"
				 " but the PF device is not up.\n");
		dev_warn(&adapter->pdev->dev,
				 "Bring the PF device up before"
				 " attempting to use the VF device.\n");
	}
	rnp_set_vf_mac(adapter, vf, mac);
	// send reset to vf
	rnp_msg_post_status_signle(adapter, PF_SET_RESET, vf);

	return 0;
}
static int rnp_link_mbps(struct rnp_adapter *adapter)
{
	switch (adapter->link_speed) {
		case RNP_LINK_SPEED_100_FULL:
			return 100;
		case RNP_LINK_SPEED_1GB_FULL:
			return 1000;
		case RNP_LINK_SPEED_10GB_FULL:
			return 10000;
		default:
			return 0;
	}
}

#if 0
static void rnp_set_vf_rate_limit(struct rnp_adapter *adapter, int vf)
{
	struct rnp_ring_feature *vmdq = &adapter->ring_feature[RING_F_VMDQ];
	struct rnp_hw *hw = &adapter->hw;
	u32 bcnrc_val = 0;
	u16 queue, queues_per_pool;
	u16 tx_rate = adapter->vfinfo[vf].tx_rate;

	if (tx_rate) {
		/* start with base link speed value */
		bcnrc_val = adapter->vf_rate_link_speed;

		/* Calculate the rate factor values to set */
		bcnrc_val <<= RNP_RTTBCNRC_RF_INT_SHIFT;
		bcnrc_val /= tx_rate;

		/* clear everything but the rate factor */
		bcnrc_val &= RNP_RTTBCNRC_RF_INT_MASK |
			RNP_RTTBCNRC_RF_DEC_MASK;

		/* enable the rate scheduler */
		bcnrc_val |= RNP_RTTBCNRC_RS_ENA;
	}

	/*
	 * Set global transmit compensation time to the MMW_SIZE in RTTBCNRM
	 * register. Typically MMW_SIZE=0x014 if 9728-byte jumbo is supported
	 * and 0x004 otherwise.
	 */
	switch (hw->mac.type) {
	case rnp_mac_n10EB:
		RNP_WRITE_REG(hw, RNP_RTTBCNRM, 0x4);
		break;
	case rnp_mac_X540:
		RNP_WRITE_REG(hw, RNP_RTTBCNRM, 0x14);
		break;
	default:
		break;
	}

	/* determine how many queues per pool based on VMDq mask */
	queues_per_pool = __ALIGN_MASK(1, ~vmdq->mask);

	/* write value for all Tx queues belonging to VF */
	for (queue = 0; queue < queues_per_pool; queue++) {
		unsigned int reg_idx = (vf * queues_per_pool) + queue;

		RNP_WRITE_REG(hw, RNP_RTTDQSEL, reg_idx);
		RNP_WRITE_REG(hw, RNP_RTTBCNRC, bcnrc_val);
	}
}

void rnp_check_vf_rate_limit(struct rnp_adapter *adapter)
{
	int i;

	/* VF Tx rate limit was not set */
	if (!adapter->vf_rate_link_speed)
		return;

	if (rnp_link_mbps(adapter) != adapter->vf_rate_link_speed) {
		adapter->vf_rate_link_speed = 0;
		dev_info(&adapter->pdev->dev,
				"Link speed has been changed. VF Transmit rate is disabled\n");
	}

	for (i = 0; i < adapter->num_vfs; i++) {
		if (!adapter->vf_rate_link_speed)
			adapter->vfinfo[i].tx_rate = 0;

		rnp_set_vf_rate_limit(adapter, i);
	}
}

int rnp_ndo_set_vf_ring_rate(struct net_device *netdev, int vf, int tx_rate)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_adapter *ctrl = adapter->controler;
	int link_speed;

	/* verify VF is active */
	if (vf >= ctrl->num_vfs)
		return -EINVAL;

	/* verify link is up */
	if (!adapter->link_up)
		return -EINVAL;

	/* verify we are linked at 10Gbps */
	link_speed = rnp_link_mbps(adapter);
	if (link_speed != 10000)
		return -EINVAL;

	/* rate limit cannot be less than 10Mbs or greater than link speed */
	if (tx_rate && ((tx_rate <= 10) || (tx_rate > link_speed)))
		return -EINVAL;

	/* store values */
	adapter->vf_rate_link_speed = link_speed;
	adapter->vfinfo[vf].tx_rate = tx_rate;

	/* update hardware configuration */
	rnp_set_vf_rate_limit(adapter, vf);
	return 0;
}
#endif

int rnp_ndo_get_vf_config(struct net_device *netdev,
						  int vf,
						  struct ifla_vf_info *ivi)
{
	struct rnp_adapter *adapter = netdev_priv(netdev);

	if (vf >= adapter->num_vfs)
		return -EINVAL;
	ivi->vf = vf;
	memcpy(&ivi->mac, adapter->vfinfo[vf].vf_mac_addresses, ETH_ALEN);
	// ivi->tx_rate = adapter->vfinfo[vf].tx_rate;
	if (adapter->vfinfo[vf].pf_vlan)
		ivi->vlan = adapter->vfinfo[vf].pf_vlan;
	else
		ivi->vlan = adapter->vfinfo[vf].vf_vlan;

	ivi->qos = adapter->vfinfo[vf].pf_qos;
#ifdef HAVE_VF_SPOOFCHK_CONFIGURE
	ivi->spoofchk = adapter->vfinfo[vf].spoofchk_enabled;
#endif

	return 0;
}
int rnp_pci_sriov_configure(struct pci_dev *dev, int num_vfs)
{
	vf_dbg("\n\n !!!! %s:%d num_vfs:%d\n", __func__, __LINE__, num_vfs);
	if (num_vfs == 0)
		return rnp_pci_sriov_disable(dev);
	else
		return rnp_pci_sriov_enable(dev, num_vfs);
}
