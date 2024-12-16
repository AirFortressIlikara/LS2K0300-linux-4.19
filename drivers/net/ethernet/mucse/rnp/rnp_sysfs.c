#include <linux/module.h>
#include <linux/types.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/hwmon.h>

#include "rnp.h"
#include "rnp_common.h"
#include "rnp_type.h"

#include "rnp_mbx.h"
#include "rnp_mbx_fw.h"

// #define  TEST_PF_RESET
struct maintain_req {
	int magic;
#define MAINTAIN_MAGIC 0xa6a7a8a9

	int	 cmd;
	int	 arg0;
	int	 req_data_bytes;
	int	 reply_bytes;
	char data[0];
} __attribute__((packed));

static int print_desc(char *buf, void *data, int len)
{
	u8 *ptr = (u8 *)data;
	int ret = 0;
	int i	= 0;

	for (i = 0; i < len; i++) {
		ret += sprintf(buf + ret, "%02x ", *(ptr + i));
	}

	return ret;
}

#define to_net_device(n) container_of(n, struct net_device, dev)
#ifndef NO_BIT_ATTRS
static ssize_t maintain_read(struct file *filp, struct kobject *kobj, struct bin_attribute *attr,
							 char *buf, loff_t off, size_t count)
{
	struct device	   *dev		= kobj_to_dev(kobj);
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;
	int					rbytes	= count;

	if (adapter->maintain_buf == NULL) {
		return 0;
	}

#if 0
	printk("%s: off:%d cnt:%d total:%d\n",
		   __func__,
		   off,
		   count,
		   adapter->maintain_buf_len);
	buf_dump("rx2", adapter->maintain_buf + off, 100);
#endif
	if (off + count > adapter->maintain_buf_len) {
		rbytes = adapter->maintain_buf_len - off;
	}
	memcpy(buf, adapter->maintain_buf + off, rbytes);

	// end-of-buf
	if ((off + rbytes) >= adapter->maintain_buf_len) {
		kfree(adapter->maintain_buf);
		adapter->maintain_buf	  = NULL;
		adapter->maintain_buf_len = 0;
	}

	// printk("rbytes:%d\n", rbytes);

	return rbytes;
}

static ssize_t maintain_write(struct file *filp, struct kobject *kobj, struct bin_attribute *attr,
							  char *buf, loff_t off, size_t count)
{
	struct device		*dev = kobj_to_dev(kobj);
	int					 ret;
	int					 err	 = -EINVAL;
	struct net_device	*netdev	 = to_net_device(dev);
	struct rnp_adapter	*adapter = netdev_priv(netdev);
	struct rnp_hw		*hw		 = &adapter->hw;
	struct maintain_req *req;
	void				*dma_buf = NULL;
	dma_addr_t			 dma_phy;
	int					 bytes;

	if (off == 0) {
		if (count < sizeof(*req)) {
			return -EINVAL;
		}
		req = (struct maintain_req *)buf;
		if (req->magic != MAINTAIN_MAGIC) {
			return -EINVAL;
		}
		bytes = max_t(int, req->req_data_bytes, req->reply_bytes);
		bytes += sizeof(*req);

		// free no readed buf
		if (adapter->maintain_buf) {
			kfree(adapter->maintain_buf);
			adapter->maintain_buf	  = NULL;
			adapter->maintain_buf_len = 0;
		}

		// alloc buf
		// dma_buf = pci_alloc_consistent(hw->pdev, bytes, &dma_phy);
		dma_buf = dma_alloc_coherent(&hw->pdev->dev, bytes, &dma_phy, GFP_ATOMIC);
		if (!dma_buf) {
			netdev_err(netdev, "%s: no memory:%d!", __func__, bytes);
			return -ENOMEM;
		}

		adapter->maintain_dma_buf  = dma_buf;
		adapter->maintain_dma_phy  = dma_phy;
		adapter->maintain_dma_size = bytes;
		adapter->maintain_in_bytes = req->req_data_bytes + sizeof(*req);

		memcpy(dma_buf + off, buf, count);

		if (count < adapter->maintain_in_bytes)
			return count;
	}

	dma_buf = adapter->maintain_dma_buf;
	dma_phy = adapter->maintain_dma_phy;
	req		= (struct maintain_req *)dma_buf;

	memcpy(dma_buf + off, buf, count);

	// all data got, send req
	if ((off + count) >= adapter->maintain_in_bytes) {
		int reply_bytes = req->reply_bytes;
		// send req
		err = rnp_maintain_req(hw, req->cmd, req->arg0, req->req_data_bytes, req->reply_bytes,
							   dma_phy);
		if (err != 0) {
			goto err_quit;
		}
		// req can't be acces, a
		// copy data for read
		if (reply_bytes > 0) {
			adapter->maintain_buf_len = reply_bytes;
			adapter->maintain_buf	  = kmalloc(adapter->maintain_buf_len, GFP_KERNEL);
			if (!adapter->maintain_buf) {
				netdev_err(netdev, "No Memory for maintain buf:%d\n", adapter->maintain_buf_len);
				err = -ENOMEM;

				goto err_quit;
			}
			memcpy(adapter->maintain_buf, dma_buf, reply_bytes);
			// buf_dump("rx", adapter->maintain_buf, 100);
		}

		if (dma_buf) {
			// pci_free_consistent(
			dma_free_coherent(&hw->pdev->dev, adapter->maintain_dma_size, dma_buf, dma_phy);
		}
		adapter->maintain_dma_buf = NULL;
	}

	return count;
err_quit:
	if (dma_buf) {
		// pci_free_consistent(
		dma_free_coherent(&hw->pdev->dev, adapter->maintain_dma_size, dma_buf, dma_phy);
		adapter->maintain_dma_buf = NULL;
	}
	return err;
}

static BIN_ATTR(maintain, (S_IWUSR | S_IRUGO), maintain_read, maintain_write, 1 * 1024 * 1024);
#endif
// static BIN_ATTR_RW(maintain, 1 * 1024 * 1024);
#ifdef TEST_PF_RESET
static ssize_t show_test_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	int					ret		= 0;
	/*
	if (adapter->flags2 & RNP_FLAG2_RESET_PF)
		ret += sprintf(buf + ret, "set pf reset");
	else
		ret += sprintf(buf + ret, "not set pf reset");
	// print next to watch desc
	*/
	if (netif_carrier_ok(netdev))
		ret += sprintf(buf + ret, "dev carrier ok");
	else
		ret += sprintf(buf + ret, "dev carrier not ok");

	ret += sprintf(buf + ret, "adapter->state is %lx\n", adapter->state);

	ret += sprintf(buf + ret, "\n");

	return ret;
}

static ssize_t store_test_info(struct device *dev, struct device_attribute *attr, const char *buf,
							   size_t count)
{
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	int					ret		= count;

	u32 flags;

	if (0 != kstrtou32(buf, 0, &flags))
		return -EINVAL;
	// should check tx_ring_num is valid
	if (flags == 1) {
		// printk("set reset_pf\n");
		// adapter->flags2 |= RNP_FLAG2_RESET_PF;
		adapter->flags2 |= RNP_FLAG2_TEST_INFO;
	} else
		adapter->flags2 &= (~RNP_FLAG2_TEST_INFO);

	return ret;
}
#endif
static ssize_t show_rx_desc_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device  *netdev		= to_net_device(dev);
	struct rnp_adapter *adapter		= netdev_priv(netdev);
	u32					rx_ring_num = adapter->sysfs_rx_ring_num;
	u32					rx_desc_num = adapter->sysfs_rx_desc_num;
	struct rnp_ring	   *ring		= adapter->rx_ring[rx_ring_num];
	int					ret			= 0;
	union rnp_rx_desc  *desc;

	desc = RNP_RX_DESC(ring, rx_desc_num);
	ret += sprintf(buf + ret, "rx ring %d desc %d:\n", rx_ring_num, rx_desc_num);
	// print next to watch desc
	ret += print_desc(buf + ret, desc, sizeof(*desc));
	ret += sprintf(buf + ret, "\n");

	return ret;
}

static ssize_t store_rx_desc_info(struct device *dev, struct device_attribute *attr,
								  const char *buf, size_t count)
{
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	int					ret		= count;

	u32 rx_desc_num = adapter->sysfs_rx_desc_num;
	u32 rx_ring_num = adapter->sysfs_rx_ring_num;

	struct rnp_ring *ring = adapter->rx_ring[rx_ring_num];

	if (0 != kstrtou32(buf, 0, &rx_desc_num))
		return -EINVAL;
	// should check tx_ring_num is valid
	if (rx_desc_num < ring->count) {
		adapter->sysfs_rx_desc_num = rx_desc_num;
	} else
		ret = -EINVAL;

	return ret;
}

static ssize_t show_tx_desc_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device  *netdev		= to_net_device(dev);
	struct rnp_adapter *adapter		= netdev_priv(netdev);
	u32					tx_ring_num = adapter->sysfs_tx_ring_num;
	u32					tx_desc_num = adapter->sysfs_tx_desc_num;
	struct rnp_ring	   *ring		= adapter->tx_ring[tx_ring_num];
	int					ret			= 0;
	struct rnp_tx_desc *desc;

	desc = RNP_TX_DESC(ring, tx_desc_num);
	ret += sprintf(buf + ret, "tx ring %d desc %d:\n", tx_ring_num, tx_desc_num);
	// print next to watch desc
	ret += print_desc(buf + ret, desc, sizeof(*desc));
	ret += sprintf(buf + ret, "\n");

	return ret;
}

static ssize_t store_tx_desc_info(struct device *dev, struct device_attribute *attr,
								  const char *buf, size_t count)
{
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	int					ret		= count;

	u32 tx_desc_num = adapter->sysfs_tx_desc_num;
	u32 tx_ring_num = adapter->sysfs_tx_ring_num;

	struct rnp_ring *ring = adapter->tx_ring[tx_ring_num];

	if (0 != kstrtou32(buf, 0, &tx_desc_num))
		return -EINVAL;
	// should check tx_ring_num is valid
	if (tx_desc_num < ring->count) {
		adapter->sysfs_tx_desc_num = tx_desc_num;
	} else
		ret = -EINVAL;

	return ret;
}

static ssize_t show_rx_ring_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device  *netdev		= to_net_device(dev);
	struct rnp_adapter *adapter		= netdev_priv(netdev);
	u32					rx_ring_num = adapter->sysfs_rx_ring_num;
	struct rnp_ring	   *ring		= adapter->rx_ring[rx_ring_num];
	int					ret			= 0;
	int					i;
	union rnp_rx_desc  *rx_desc;

	// print all tx_ring_num info
	ret += sprintf(buf + ret, "queue %d info:\n", rx_ring_num);

	ret += sprintf(buf + ret, "next_to_use %d\n", ring->next_to_use);
	ret += sprintf(buf + ret, "next_to_clean %d\n", ring->next_to_clean);

	rx_desc = RNP_RX_DESC(ring, ring->next_to_clean);
	ret += sprintf(buf + ret, "next_to_clean desc: ");
	ret += print_desc(buf + ret, rx_desc, sizeof(*rx_desc));
	ret += sprintf(buf + ret, "\n");

	return ret;
}

static ssize_t store_rx_ring_info(struct device *dev, struct device_attribute *attr,
								  const char *buf, size_t count)
{
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	int					ret		= count;

	u32 rx_ring_num = adapter->sysfs_rx_ring_num;

	if (0 != kstrtou32(buf, 0, &rx_ring_num))
		return -EINVAL;
	// should check tx_ring_num is valid
	if (rx_ring_num < adapter->num_rx_queues) {
		adapter->sysfs_rx_ring_num = rx_ring_num;
	} else
		ret = -EINVAL;

	return ret;
}

static ssize_t show_tx_ring_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device	 *netdev	  = to_net_device(dev);
	struct rnp_adapter	 *adapter	  = netdev_priv(netdev);
	u32					  tx_ring_num = adapter->sysfs_tx_ring_num;
	struct rnp_ring		 *ring		  = adapter->tx_ring[tx_ring_num];
	int					  ret		  = 0;
	int					  i;
	struct rnp_tx_buffer *tx_buffer;
	// struct rnp_tx_desc *tx_desc;
	struct rnp_tx_desc *eop_desc;

	// print all tx_ring_num info
	ret += sprintf(buf + ret, "queue %d info:\n", tx_ring_num);

	ret += sprintf(buf + ret, "next_to_use %d\n", ring->next_to_use);
	ret += sprintf(buf + ret, "next_to_clean %d\n", ring->next_to_clean);

	tx_buffer = &ring->tx_buffer_info[ring->next_to_clean];
	eop_desc  = tx_buffer->next_to_watch;
	/* if have watch desc */
	if (eop_desc) {
		ret += sprintf(buf + ret, "next_to_watch:\n");
		// print next to watch desc
		ret += print_desc(buf + ret, eop_desc, sizeof(*eop_desc));
		ret += sprintf(buf + ret, "\n");
	} else {
		ret += sprintf(buf + ret, "no next_to_watch data\n");
	}

	// print all desc
	/* for (i = 0; i < ring->count; i++) {
		ret += sprintf(buf + ret, "desc %d:", i);
		tx_desc = RNP_TX_DESC(ring, i);
		ret += print_desc(buf + ret, tx_desc, sizeof(*tx_desc));
		ret += sprintf(buf + ret, "\n");
		// print desc
	} */

	return ret;
}

static ssize_t store_tx_ring_info(struct device *dev, struct device_attribute *attr,
								  const char *buf, size_t count)
{
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	int					ret		= count;

	u32 tx_ring_num = adapter->sysfs_tx_ring_num;

	if (0 != kstrtou32(buf, 0, &tx_ring_num))
		return -EINVAL;
	// should check tx_ring_num is valid
	if (tx_ring_num < adapter->num_tx_queues) {
		adapter->sysfs_tx_ring_num = tx_ring_num;
	} else
		ret = -EINVAL;

	return ret;
}

static ssize_t show_queue_mapping(struct device *dev, struct device_attribute *attr, char *buf)
{
	int					 ret = 0;
	int					 i;
	struct net_device	*netdev	 = to_net_device(dev);
	struct rnp_adapter	*adapter = netdev_priv(netdev);
	struct rnp_ring		*ring;
	struct rnp_q_vector *q_vector;

	ret += sprintf(buf + ret, "tx_queue count %d\n", adapter->num_tx_queues);
	ret += sprintf(buf + ret, "queue-mapping :\n");
	for (i = 0; i < adapter->num_tx_queues; i++) {
		ring = adapter->tx_ring[i];
		ret += sprintf(buf + ret, "tx queue %d <---> ring %d\n", i, ring->rnp_queue_idx);
	}
	ret += sprintf(buf + ret, "rx_queue count %d\n", adapter->num_rx_queues);
	ret += sprintf(buf + ret, "queue-mapping :\n");
	for (i = 0; i < adapter->num_rx_queues; i++) {
		ring = adapter->rx_ring[i];
		ret += sprintf(buf + ret, "rx queue %d <---> ring %d\n", i, ring->rnp_queue_idx);
	}
	ret += sprintf(buf + ret, "vector-queue mapping:\n");
	for (i = 0; i < adapter->num_q_vectors; i++) {
		q_vector = adapter->q_vector[i];
		ret += sprintf(buf + ret, "---vector %d--- \n", i);
		rnp_for_each_ring(ring, q_vector->tx)
		{
			ret += sprintf(buf + ret, "tx ring %d\n", ring->rnp_queue_idx);
		}
		rnp_for_each_ring(ring, q_vector->rx)
		{
			ret += sprintf(buf + ret, "rx ring %d\n", ring->rnp_queue_idx);
		}
	}

	return ret;
}

static ssize_t store_queue_mapping(struct device *dev, struct device_attribute *attr,
								   const char *buf, size_t count)
{
	return count;
}

static ssize_t show_active_vid(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16					vid, current_vid = 0;
	int					ret		= 0;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;
	u8					vfnum	= RNP_MAX_VF_CNT - 1;  // use last-vf's table entry. the las

	if ((adapter->flags & RNP_FLAG_SRIOV_ENABLED)) {
		current_vid = rd32(hw, RNP_DMA_PORT_VEB_VID_TBL(adapter->port, vfnum));
	}

#ifndef HAVE_VLAN_RX_REGISTER
	for_each_set_bit(vid, adapter->active_vlans, VLAN_N_VID)
	{
		ret += sprintf(buf + ret, "%u%s ", vid, (current_vid == vid ? "*" : ""));
	}
#endif
	ret += sprintf(buf + ret, "\n");
	return ret;
}

static ssize_t store_active_vid(struct device *dev, struct device_attribute *attr, const char *buf,
								size_t count)
{
	u16					vid;
	int					err		= -EINVAL;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;
	u8					vfnum	= RNP_MAX_VF_CNT - 1;  // use last-vf's table entry. the las
	int					port	= 0;

	if (!(adapter->flags & RNP_FLAG_SRIOV_ENABLED))
		return -EIO;

	if (0 != kstrtou16(buf, 0, &vid))
		return -EINVAL;

#ifndef HAVE_VLAN_RX_REGISTER
	if ((vid < 4096) && test_bit(vid, adapter->active_vlans)) {
		if (rd32(hw, RNP_DMA_VERSION) >= 0x20201231) {
			for (port = 0; port < 4; port++) wr32(hw, RNP_DMA_PORT_VEB_VID_TBL(port, vfnum), vid);
		} else {
			wr32(hw, RNP_DMA_PORT_VEB_VID_TBL(adapter->port, vfnum), vid);
		}
		err = 0;
	}
#endif

	return err ? err : count;
}

static ssize_t show_port_idx(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16					vid, current_vid = 0;
	int					ret		= 0;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;

	ret += sprintf(buf, "%d\n", adapter->portid_of_card);
	return ret;
}
static DEVICE_ATTR(port_idx, S_IRUGO | S_IRUSR, show_port_idx, NULL);

static ssize_t show_debug_linkstat(struct device *dev, struct device_attribute *attr, char *buf)
{
	int					ret		= 0;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;

	ret += sprintf(buf, "%d %d dumy:0x%x up-flag:%d carry:%d\n", adapter->link_up, adapter->hw.link,
				   rd32(hw, 0xc), adapter->flags & RNP_FLAG_NEED_LINK_UPDATE,
				   netif_carrier_ok(netdev));
	return ret;
}
static DEVICE_ATTR(debug_linkstat, S_IRUGO | S_IRUSR, show_debug_linkstat, NULL);

static ssize_t show_sfp(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16					vid, current_vid = 0;
	int					ret		= 0;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;

	if (rnp_mbx_get_lane_stat(hw) != 0) {
		ret += sprintf(buf, " IO Error\n");
	} else {
		ret += sprintf(buf, "mod-abs:%d\ntx-fault:%d\ntx-dis:%d\nrx-los:%d\n", adapter->sfp.mod_abs,
					   adapter->sfp.fault, adapter->sfp.tx_dis, adapter->sfp.los);
	}

	return ret;
}
static DEVICE_ATTR(sfp, S_IRUGO | S_IRUSR, show_sfp, NULL);

static ssize_t store_pci(struct device *dev, struct device_attribute *attr, const char *buf,
						 size_t count)
{
	int					err		= -EINVAL;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;
	int					gen = 3, lanes = 8;

	if (count > 30) {
		return -EINVAL;
	}

	if (sscanf(buf, "gen%dx%d", &gen, &lanes) != 2) {
		printk("Error: invalid input. example: gen3x8\n");
		return -EINVAL;
	}
	if (gen > 3 || lanes > 8) {
		return -EINVAL;
	}

	err = rnp_set_lane_fun(hw, LANE_FUN_PCI_LANE, gen, lanes, 0, 0);

	return err ? err : count;
}

static ssize_t show_pci(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16					vid, current_vid = 0;
	int					ret		= 0;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;

	if (rnp_mbx_get_lane_stat(hw) != 0) {
		ret += sprintf(buf, " IO Error\n");
	} else {
		ret += sprintf(buf, "gen%dx%d\n", hw->pci_gen, hw->pci_lanes);
	}

	return ret;
}
static DEVICE_ATTR(pci, S_IRUGO | S_IWUSR | S_IRUSR, show_pci, store_pci);

static ssize_t store_sfp_tx_disable(struct device *dev, struct device_attribute *attr,
									const char *buf, size_t count)
{
	int					err		= -EINVAL;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;
	long				enable	= 0;

	if (kstrtol(buf, 10, &enable)) {
		return -EINVAL;
	}

	err = rnp_set_lane_fun(hw, LANE_FUN_SFP_TX_DISABLE, !!enable, 0, 0, 0);

	return err ? err : count;
}

static ssize_t show_sfp_tx_disable(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16					vid, current_vid = 0;
	int					ret		= 0;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;

	if (rnp_mbx_get_lane_stat(hw) != 0) {
		ret += sprintf(buf, " IO Error\n");
	} else {
		ret += sprintf(buf, "%d\n", adapter->sfp.tx_dis);
	}

	return ret;
}
static DEVICE_ATTR(sfp_tx_disable, S_IRUGO | S_IWUSR | S_IRUSR, show_sfp_tx_disable,
				   store_sfp_tx_disable);

static ssize_t store_link_traing(struct device *dev, struct device_attribute *attr, const char *buf,
								 size_t count)
{
	int					err		= -EINVAL;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;
	long				enable	= 0;

	if (kstrtol(buf, 10, &enable)) {
		return -EINVAL;
	}

	err = rnp_set_lane_fun(hw, LANE_FUN_LINK_TRAING, !!enable, 0, 0, 0);

	return err ? err : count;
}

static ssize_t show_link_traing(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16					vid, current_vid = 0;
	int					ret		= 0;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;

	if (rnp_mbx_get_lane_stat(hw) != 0) {
		ret += sprintf(buf, " IO Error\n");
	} else {
		ret += sprintf(buf, "%d\n", adapter->link_traing);
	}

	return ret;
}
static DEVICE_ATTR(link_traing, S_IRUGO | S_IWUSR | S_IRUSR, show_link_traing, store_link_traing);

static ssize_t store_fec(struct device *dev, struct device_attribute *attr, const char *buf,
						 size_t count)
{
	int					err		= -EINVAL;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;
	long				enable	= 0;

	if (kstrtol(buf, 10, &enable)) {
		return -EINVAL;
	}

	err = rnp_set_lane_fun(hw, LANE_FUN_FEC, !!enable, 0, 0, 0);

	return err ? err : count;
}

static ssize_t show_fec(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16					vid, current_vid = 0;
	int					ret		= 0;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;

	if (rnp_mbx_get_lane_stat(hw) != 0) {
		ret += sprintf(buf, " IO Error\n");
	} else {
		ret += sprintf(buf, "%d\n", adapter->fec);
	}

	return ret;
}
static DEVICE_ATTR(fec, S_IRUGO | S_IWUSR | S_IRUSR, show_fec, store_fec);

static ssize_t store_prbs(struct device *dev, struct device_attribute *attr, const char *buf,
						  size_t count)
{
	int					err		= -EINVAL;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;
	long				prbs	= 0;

	if (kstrtol(buf, 10, &prbs)) {
		return -EINVAL;
	}

	err = rnp_set_lane_fun(hw, LANE_FUN_PRBS, prbs, 0, 0, 0);

	return err ? err : count;
}
static DEVICE_ATTR(prbs, S_IRUGO | S_IWUSR | S_IRUSR, NULL, store_prbs);

static ssize_t store_autoneg(struct device *dev, struct device_attribute *attr, const char *buf,
							 size_t count)
{
	int					err		= -EINVAL;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;
	long				enable	= 0;

	if (kstrtol(buf, 10, &enable)) {
		return -EINVAL;
	}

	err = rnp_set_lane_fun(hw, LANE_FUN_AN, !!enable, 0, 0, 0);

	return err ? err : count;
}

static ssize_t show_autoneg(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16					vid, current_vid = 0;
	int					ret		= 0;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;

	if (rnp_mbx_get_lane_stat(hw) != 0) {
		ret += sprintf(buf, " IO Error\n");
	} else {
		ret += sprintf(buf, "%d\n", adapter->an);
	}

	return ret;
}
static DEVICE_ATTR(autoneg, S_IRUGO | S_IWUSR | S_IRUSR, show_autoneg, store_autoneg);

static ssize_t store_lane_si(struct device *dev, struct device_attribute *attr, const char *buf,
							 size_t count)
{
	int					err		= -EINVAL;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;
	int					si_main = -1, si_pre = -1, si_post = -1, si_txboost = -1;
	int					cnt;

	if (rnp_mbx_get_lane_stat(hw) != 0) {
		printk("Error: rnp_mbx_get_lane_stat faild\n");
		return -EIO;
	}
	if (count > 100) {
		printk("Error: Input size >100: too large\n");
		return -EINVAL;
	}

	if (hw->supported_link & (RNP_LINK_SPEED_40GB_FULL | RNP_LINK_SPEED_25GB_FULL)) {
		u32 lane0_main, lane0_pre, lane0_post, lane0_boost;
		u32 lane1_main, lane1_pre, lane1_post, lane1_boost;
		u32 lane2_main, lane2_pre, lane2_post, lane2_boost;
		u32 lane3_main, lane3_pre, lane3_post, lane3_boost;

		cnt = sscanf(buf, "%u %u %u %u,%u %u %u %u,%u %u %u %u,%u %u %u %u", &lane0_main,
					 &lane0_pre, &lane0_post, &lane0_boost, &lane1_main, &lane1_pre, &lane1_post,
					 &lane1_boost, &lane2_main, &lane2_pre, &lane2_post, &lane2_boost, &lane3_main,
					 &lane3_pre, &lane3_post, &lane3_boost);
		if (cnt != 16) {
			printk("Error: Invalid Input.\n"
				   "  <lane0_si>,<lane1_si>,<lane2_si>,<lane3_si>\n"
				   "  laneX_si: <main> <pre> <post> <boost>\n\n"
				   "   ie: 21 0 11 11,22 0 12 12,23 0 13 13,24 0 14 14 \n");

			return -EINVAL;
		}

		si_main = ((lane0_main & 0xff) << 0) | ((lane1_main & 0xff) << 8) |
				  ((lane2_main & 0xff) << 16) | ((lane3_main & 0xff) << 24);
		si_pre = ((lane0_pre & 0xff) << 0) | ((lane1_pre & 0xff) << 8) |
				 ((lane2_pre & 0xff) << 16) | ((lane3_pre & 0xff) << 24);
		si_post = ((lane0_post & 0xff) << 0) | ((lane1_post & 0xff) << 8) |
				  ((lane2_post & 0xff) << 16) | ((lane3_post & 0xff) << 24);
		si_txboost = ((lane0_boost & 0xf) << 0) | ((lane1_boost & 0xf) << 4) |
					 ((lane2_boost & 0xf) << 8) | ((lane3_boost & 0xf) << 12);
		printk("%s: main:0x%x pre:0x%x post:0x%x boost:0x%x\n", adapter->name, si_main, si_pre,
			   si_post, si_txboost);
	} else {
		cnt = sscanf(buf, "%u %u %u %u", &si_main, &si_pre, &si_post, &si_txboost);
		if (cnt != 4) {
			printk("Error: Invalid Input: <main> <pre> <post> <tx_boost>\n");
			return -EINVAL;
		}
		if (si_main > 63 || si_pre > 63 || si_post > 63) {
			printk("Error: Invalid value. should in 0~63\n");
			return -EINVAL;
		}
		if (si_txboost > 16) {
			printk("Error: Invalid txboost. should in 0~15\n");
			return -EINVAL;
		}
	}
	err = rnp_set_lane_fun(hw, LANE_FUN_SI, si_main, si_pre, si_post, si_txboost);

	return err ? err : count;
}

static ssize_t show_lane_si(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16					vid, current_vid = 0;
	int					ret		= 0, i;
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;

	if (rnp_mbx_get_lane_stat(hw) != 0) {
		ret += sprintf(buf, " IO Error\n");
	} else {
		if (hw->supported_link & (RNP_LINK_SPEED_40GB_FULL | RNP_LINK_SPEED_25GB_FULL)) {
			ret +=
				sprintf(buf + ret, "main:0x%08x pre:0x%08x post:0x%08x tx_boost:0x%04x\n\n",
						adapter->si.main, adapter->si.pre, adapter->si.post, adapter->si.tx_boost);
			for (i = 0; i < 4; i++) {
				ret += sprintf(
					buf + ret, " lane%d main:%u pre:%u post:%u tx_boost:%u\n", i,
					(adapter->si.main >> (i * 8)) & 0xff, (adapter->si.pre >> (i * 8)) & 0xff,
					(adapter->si.post >> (i * 8)) & 0xff, (adapter->si.tx_boost >> (i * 4)) & 0xf);
			}
		} else {
			ret += sprintf(buf + ret, "lane:%d main:%u pre:%u post:%u tx_boost:%u\n", hw->nr_lane,
						   adapter->si.main, adapter->si.pre, adapter->si.post,
						   adapter->si.tx_boost & 0xf);
		}
	}

	return ret;
}
static DEVICE_ATTR(si, S_IRUGO | S_IWUSR | S_IRUSR, show_lane_si, store_lane_si);

static ssize_t show_temperature(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device  *netdev	= to_net_device(dev);
	struct rnp_adapter *adapter = netdev_priv(netdev);
	struct rnp_hw	   *hw		= &adapter->hw;
	int					ret = 0, temp = 0, voltage = 0;

	temp = rnp_mbx_get_temp(hw, &voltage);

	ret += sprintf(buf, "temp:%d oC  volatage:%d mV\n", temp, voltage);
	return ret;
}

static struct pci_dev *pcie_find_root_port_old(struct pci_dev *dev)
{
	while (1) {
		if (!pci_is_pcie(dev))
			break;
		if (pci_pcie_type(dev) == PCI_EXP_TYPE_ROOT_PORT)
			return dev;
		if (!dev->bus->self)
			break;
		dev = dev->bus->self;
	}
	return NULL;
}

static ssize_t show_root_slot_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device  *netdev	  = to_net_device(dev);
	struct rnp_adapter *adapter	  = netdev_priv(netdev);
	int					ret		  = 0;
	struct pci_dev	   *root_pdev = pcie_find_root_port_old(adapter->pdev);

	if (root_pdev) {
		ret += sprintf(buf + ret, "%02x:%02x.%x\n", root_pdev->bus->number,
					   PCI_SLOT(root_pdev->devfn), PCI_FUNC(root_pdev->devfn));
	}
	// pci_find_slot
	return ret;
}

static int do_switch_loopback_set(struct rnp_adapter *adapter, int en, int sport_lane,
								  int dport_lane)
{
	int			   v;
	struct rnp_hw *hw = &adapter->hw;

	printk("%s: %s %d -> %d en:%d\n", __func__, netdev_name(adapter->netdev), sport_lane,
		   dport_lane, en);

	if (en) {
		adapter->flags |= RNP_FLAG_SWITCH_LOOPBACK_EN;
	} else {
		adapter->flags &= ~RNP_FLAG_SWITCH_LOOPBACK_EN;
	}

	// redir pkgs to peer
	wr32(hw, RNP_ETH_INPORT_POLICY_REG(sport_lane), BIT(29) | (dport_lane << 16));

	// enable/disable policy
	v = rd32(hw, RNP_ETH_INPORT_POLICY_VAL);
	if (en) {
		v |= BIT(sport_lane);  // enable this-port-policy
	} else {
		v &= ~BIT(sport_lane);
	}
	wr32(hw, RNP_ETH_INPORT_POLICY_VAL, v);

	// mac promisc
	v = mac_rd32(&hw->mac, RNP10_MAC_PKT_FLT);
	if (en) {
		v |= (RNP_RX_ALL | RNP_RX_ALL_MUL);
	} else {
		v &= ~(RNP_RX_ALL | RNP_RX_ALL_MUL);
	}
	mac_wr32(&hw->mac, RNP10_MAC_PKT_FLT, v);

	// disable unicase-table
	eth_wr32(&hw->eth, RNP10_ETH_DMAC_MCSTCTRL, 0x0);

	return 0;
}

static ssize_t _switch_loopback(struct rnp_adapter *adapter, const char *peer_eth, int en)
{
	struct net_device  *peer_netdev	 = NULL;
	struct rnp_adapter *peer_adapter = NULL;
	char				name[100];

	strncpy(name, peer_eth, sizeof(name));
	strim(name);

	printk("%s: nr_lane:%d peer_lane:%s en:%d\n", __func__, 0, peer_eth, en);

	peer_netdev = dev_get_by_name(&init_net, name);
	if (!peer_netdev) {
		printk("canot' find %s\n", name);
		return -EINVAL;
	}
	peer_adapter = netdev_priv(peer_netdev);

	if (PCI_SLOT(peer_adapter->pdev->devfn) != PCI_SLOT(adapter->pdev->devfn)) {
		printk("%s %s not in same slot\n", netdev_name(adapter->netdev),
			   netdev_name(peer_adapter->netdev));
		dev_put(peer_netdev);
		return -EINVAL;
	}

	printk("%s: %s(%d)<->%s(%d)\n", __func__, netdev_name(adapter->netdev), 0,
		   netdev_name(peer_adapter->netdev), 0);

	do_switch_loopback_set(adapter, en, 0, rnp_is_pf1(peer_adapter->pdev) ? 4 : 0);
	do_switch_loopback_set(peer_adapter, en, 0, rnp_is_pf1(adapter->pdev) ? 4 : 0);

	if (peer_netdev) {
		dev_put(peer_netdev);
	}

	return 0;
}
static ssize_t store_switch_loopback_on(struct device *dev, struct device_attribute *attr,
										const char *buf, size_t count)
{
	struct rnp_adapter *adapter = netdev_priv(to_net_device(dev));

	return _switch_loopback(adapter, buf, 1) == 0 ? count : -EINVAL;
}
static DEVICE_ATTR(switch_loopback_on, 0664, NULL, store_switch_loopback_on);

static ssize_t store_switch_loopback_off(struct device *dev, struct device_attribute *attr,
										 const char *buf, size_t count)
{
	struct rnp_adapter *adapter = netdev_priv(to_net_device(dev));

	return _switch_loopback(adapter, buf, 0) == 0 ? count : -EINVAL;
}
static DEVICE_ATTR(switch_loopback_off, 0664, NULL, store_switch_loopback_off);

static DEVICE_ATTR(root_slot_info, 0644, show_root_slot_info, NULL);

// static DEVICE_ATTR(ptp_info, 0644, show_root_slot_info,
//                    NULL);

static DEVICE_ATTR(temperature, S_IRUGO | S_IRUSR, show_temperature, NULL);

static DEVICE_ATTR(active_vid, 0644, show_active_vid, store_active_vid);

static DEVICE_ATTR(queue_mapping, 0644, show_queue_mapping, store_queue_mapping);

static DEVICE_ATTR(tx_ring_info, 0644, show_tx_ring_info, store_tx_ring_info);

static DEVICE_ATTR(rx_ring_info, 0644, show_rx_ring_info, store_rx_ring_info);

static DEVICE_ATTR(tx_desc_info, 0644, show_tx_desc_info, store_tx_desc_info);

static DEVICE_ATTR(rx_desc_info, 0644, show_rx_desc_info, store_rx_desc_info);

#ifdef TEST_PF_RESET
static DEVICE_ATTR(test_info, 0644, show_test_info, store_test_info);
#endif

static struct attribute *dev_attrs[] = {
//	&dev_attr_ptp_info.attr,
#ifdef TEST_PF_RESET
	&dev_attr_test_info.attr,
#endif
	&dev_attr_root_slot_info.attr,
	&dev_attr_active_vid.attr,
	&dev_attr_queue_mapping.attr,
	&dev_attr_tx_ring_info.attr,
	&dev_attr_rx_ring_info.attr,
	&dev_attr_tx_desc_info.attr,
	&dev_attr_rx_desc_info.attr,
	&dev_attr_port_idx.attr,
	&dev_attr_temperature.attr,
	&dev_attr_si.attr,
	&dev_attr_sfp.attr,
	&dev_attr_autoneg.attr,
	&dev_attr_sfp_tx_disable.attr,
	&dev_attr_fec.attr,
	&dev_attr_link_traing.attr,
	&dev_attr_pci.attr,
	&dev_attr_prbs.attr,
	&dev_attr_debug_linkstat.attr,
	&dev_attr_switch_loopback_off.attr,
	&dev_attr_switch_loopback_on.attr,
	NULL,
};
#ifndef NO_BIT_ATTRS
static struct bin_attribute *dev_bin_attrs[] = {
	&bin_attr_maintain,
	NULL,
};
#endif
static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
#ifndef NO_BIT_ATTRS
	.bin_attrs = dev_bin_attrs,
#endif
};

/* called from rnp_main.c */
void rnp_sysfs_exit(struct rnp_adapter *adapter)
{
	sysfs_remove_group(&adapter->netdev->dev.kobj, &dev_attr_grp);
}

/* called from rnp_main.c */
int rnp_sysfs_init(struct rnp_adapter *adapter)
{
	int err;

	err = sysfs_create_group(&adapter->netdev->dev.kobj, &dev_attr_grp);
	if (err != 0) {
		dev_err(&adapter->netdev->dev, "sysfs_create_group faild:err:%d\n", err);
		return err;
	}
	return 0;
}
