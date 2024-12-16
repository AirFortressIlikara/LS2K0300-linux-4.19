// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2022 - 2023 Mucse Corporation. */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/hwmon.h>
#include <linux/ctype.h>

#include "rnpgbe.h"
#include "rnpgbe_common.h"
#include "rnpgbe_type.h"

#include "rnpgbe_mbx.h"
#include "rnpgbe_mbx_fw.h"
//#define TEST_PF_RESET

struct maintain_req {
	int magic;
#define MAINTAIN_MAGIC 0xa6a7a8a9

	int cmd;
	int arg0;
	int req_data_bytes;
	int reply_bytes;
	char data[0];
} __attribute__((packed));

struct maintain_reply {
	int magic;
#define MAINTAIN_REPLY_MAGIC 0xB6B7B8B9
	int cmd;
	int arg0;
	int data_bytes;
	int rev;
	int data[0];
} __attribute__((packed));

struct ucfg_mac_sn {
	unsigned char macaddr[64];
	unsigned char sn[32];
	int magic;
#define MAC_SN_MAGIC 0x87654321
	char rev[52];
	unsigned char pn[32];
} __attribute__((packed, aligned(4)));

static int print_desc(char *buf, void *data, int len)
{
	u8 *ptr = (u8 *)data;
	int ret = 0;
	int i = 0;

	for (i = 0; i < len; i++)
		ret += sprintf(buf + ret, "%02x ", *(ptr + i));

	return ret;
}
#ifdef RNP_HWMON
static ssize_t rnpgbe_hwmon_show_location(struct device __always_unused *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct hwmon_attr *rnpgbe_attr =
		container_of(attr, struct hwmon_attr, dev_attr);

	return snprintf(buf, PAGE_SIZE, "loc%u\n",
			rnpgbe_attr->sensor->location);
}

static ssize_t rnpgbe_hwmon_show_name(struct device __always_unused *dev,
				      struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "rnp\n");
}

static ssize_t rnpgbe_hwmon_show_temp(struct device __always_unused *dev,
				      struct device_attribute *attr, char *buf)
{
	struct hwmon_attr *rnpgbe_attr =
		container_of(attr, struct hwmon_attr, dev_attr);
	unsigned int value;

	/* reset the temp field */
	rnpgbe_attr->hw->ops.get_thermal_sensor_data(rnpgbe_attr->hw);

	value = rnpgbe_attr->sensor->temp;
	/* display millidegree */
	value *= 1000;

	return snprintf(buf, PAGE_SIZE, "%u\n", value);
}

static ssize_t
rnpgbe_hwmon_show_cautionthresh(struct device __always_unused *dev,
				struct device_attribute *attr, char *buf)
{
	struct hwmon_attr *rnpgbe_attr =
		container_of(attr, struct hwmon_attr, dev_attr);
	unsigned int value = rnpgbe_attr->sensor->caution_thresh;
	/* display millidegree */
	value *= 1000;

	return snprintf(buf, PAGE_SIZE, "%u\n", value);
}

static ssize_t rnpgbe_hwmon_show_maxopthresh(struct device __always_unused *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct hwmon_attr *rnpgbe_attr =
		container_of(attr, struct hwmon_attr, dev_attr);
	unsigned int value = rnpgbe_attr->sensor->max_op_thresh;

	/* display millidegree */
	value *= 1000;

	return snprintf(buf, PAGE_SIZE, "%u\n", value);
}
/**
 * rnpgbe_add_hwmon_attr - Create hwmon attr table for a hwmon sysfs file.
 * @adapter: pointer to the adapter structure
 * @offset: offset in the eeprom sensor data table
 * @type: type of sensor data to display
 *
 * For each file we want in hwmon's sysfs interface we need a device_attribute
 * This is included in our hwmon_attr struct that contains the references to
 * the data structures we need to get the data to display.
 */
static int rnpgbe_add_hwmon_attr(struct rnpgbe_adapter *adapter,
				 unsigned int offset, int type)
{
	unsigned int n_attr;
	struct hwmon_attr *rnpgbe_attr;
#ifdef HAVE_HWMON_DEVICE_REGISTER_WITH_GROUPS

	n_attr = adapter->rnpgbe_hwmon_buff->n_hwmon;
	rnpgbe_attr = &adapter->rnpgbe_hwmon_buff->hwmon_list[n_attr];
#else
	int rc;

	n_attr = adapter->rnpgbe_hwmon_buff.n_hwmon;
	rnpgbe_attr = &adapter->rnpgbe_hwmon_buff.hwmon_list[n_attr];
#endif /* HAVE_HWMON_DEVICE_REGISTER_WITH_GROUPS */

	switch (type) {
	case RNP_HWMON_TYPE_LOC:
		rnpgbe_attr->dev_attr.show = rnpgbe_hwmon_show_location;
		snprintf(rnpgbe_attr->name, sizeof(rnpgbe_attr->name),
			 "temp%u_label", offset + 1);
		break;
	case RNP_HWMON_TYPE_NAME:
		rnpgbe_attr->dev_attr.show = rnpgbe_hwmon_show_name;
		snprintf(rnpgbe_attr->name, sizeof(rnpgbe_attr->name), "name");
		break;
	case RNP_HWMON_TYPE_TEMP:
		rnpgbe_attr->dev_attr.show = rnpgbe_hwmon_show_temp;
		snprintf(rnpgbe_attr->name, sizeof(rnpgbe_attr->name),
			 "temp%u_input", offset + 1);
		break;
	case RNP_HWMON_TYPE_CAUTION:
		rnpgbe_attr->dev_attr.show = rnpgbe_hwmon_show_cautionthresh;
		snprintf(rnpgbe_attr->name, sizeof(rnpgbe_attr->name),
			 "temp%u_max", offset + 1);
		break;
	case RNP_HWMON_TYPE_MAX:
		rnpgbe_attr->dev_attr.show = rnpgbe_hwmon_show_maxopthresh;
		snprintf(rnpgbe_attr->name, sizeof(rnpgbe_attr->name),
			 "temp%u_crit", offset + 1);
		break;
	default:
		return -EPERM;
	}

	/* These always the same regardless of type */
	rnpgbe_attr->sensor = &adapter->hw.thermal_sensor_data.sensor[offset];
	rnpgbe_attr->hw = &adapter->hw;
	rnpgbe_attr->dev_attr.store = NULL;
	rnpgbe_attr->dev_attr.attr.mode = 0444;
	rnpgbe_attr->dev_attr.attr.name = rnpgbe_attr->name;

#ifdef HAVE_HWMON_DEVICE_REGISTER_WITH_GROUPS
	sysfs_attr_init(&rnpgbe_attr->dev_attr.attr);

	adapter->rnpgbe_hwmon_buff->attrs[n_attr] = &rnpgbe_attr->dev_attr.attr;

	++adapter->rnpgbe_hwmon_buff->n_hwmon;

	return 0;
#else
	rc = device_create_file(pci_dev_to_dev(adapter->pdev),
				&rnpgbe_attr->dev_attr);

	if (rc == 0)
		++adapter->rnpgbe_hwmon_buff.n_hwmon;

	return rc;
#endif /* HAVE_HWMON_DEVICE_REGISTER_WITH_GROUPS */
}
#endif /* RNP_HWMON */

#define to_net_device(n) container_of(n, struct net_device, dev)
#ifndef NO_BIT_ATTRS
static ssize_t maintain_read(struct file *filp, struct kobject *kobj,
			     struct bin_attribute *attr, char *buf, loff_t off,
			     size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	int rbytes = count;

	if (adapter->maintain_buf == NULL)
		return 0;

	if (off + count > adapter->maintain_buf_len)
		rbytes = adapter->maintain_buf_len - off;
	memcpy(buf, adapter->maintain_buf + off, rbytes);

	if ((off + rbytes) >= adapter->maintain_buf_len) {
		kfree(adapter->maintain_buf);
		adapter->maintain_buf = NULL;
		adapter->maintain_buf_len = 0;
	}
	return rbytes;
}

void n500_exchange_share_ram(struct rnpgbe_hw *hw, u32 *buf, int flag, int len)
{
	int i;
	struct rnpgbe_mbx_info *mbx = &hw->mbx;

	if (len > mbx->share_size)
		return;
	// write
	if (flag) {
		for (i = 0; i < len; i = i + 4)
			rnpgbe_wr_reg(hw->hw_addr + mbx->cpu_vf_share_ram + i,
				      *(buf + i / 4));
	} else {
		// read
		for (i = 0; i < len; i = i + 4)
			*(buf + i / 4) = rnpgbe_rd_reg(
				hw->hw_addr + mbx->cpu_vf_share_ram + i);
	}
}

static ssize_t maintain_write(struct file *filp, struct kobject *kobj,
			      struct bin_attribute *attr, char *buf, loff_t off,
			      size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	int err = -EINVAL;
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	struct rnpgbe_hw *hw = &adapter->hw;
	struct maintain_req *req;
	void *dma_buf = NULL;
	dma_addr_t dma_phy;
	int bytes;

	if (off == 0) {
		if (count < sizeof(*req))
			return -EINVAL;
		req = (struct maintain_req *)buf;
		if (req->magic != MAINTAIN_MAGIC)
			return -EINVAL;
		bytes = max_t(int, req->req_data_bytes, req->reply_bytes);
		bytes += sizeof(*req);

		if (adapter->maintain_buf) {
			kfree(adapter->maintain_buf);
			adapter->maintain_buf = NULL;
			adapter->maintain_buf_len = 0;
		}

		dma_buf = dma_alloc_coherent(&hw->pdev->dev, bytes, &dma_phy,
					     GFP_ATOMIC);
		if (!dma_buf) {
			netdev_err(netdev, "%s: no memory:%d!", __func__,
				   bytes);
			return -ENOMEM;
		}

		adapter->maintain_dma_buf = dma_buf;
		adapter->maintain_dma_phy = dma_phy;
		adapter->maintain_dma_size = bytes;
		adapter->maintain_in_bytes = req->req_data_bytes + sizeof(*req);

		memcpy(dma_buf + off, buf, count);

		if (count < adapter->maintain_in_bytes)
			return count;
	}

	dma_buf = adapter->maintain_dma_buf;
	dma_phy = adapter->maintain_dma_phy;
	req = (struct maintain_req *)dma_buf;

	memcpy(dma_buf + off, buf, count);

	/* all data got, send req */
	if ((off + count) >= adapter->maintain_in_bytes) {
		int reply_bytes = req->reply_bytes;
		int offset;
		struct rnpgbe_mbx_info *mbx = &hw->mbx;

		switch (hw->hw_type) {
		case rnpgbe_hw_n10:
			err = rnpgbe_maintain_req(hw, req->cmd, req->arg0,
						  req->req_data_bytes,
						  req->reply_bytes, dma_phy);
			if (err != 0)
				goto err_quit;
			if (reply_bytes > 0) {
				adapter->maintain_buf_len = reply_bytes;
				adapter->maintain_buf = kmalloc(
					adapter->maintain_buf_len, GFP_KERNEL);
				if (!adapter->maintain_buf) {
					netdev_err(
						netdev,
						"No Memory for maintain buf:%d\n",
						adapter->maintain_buf_len);
					err = -ENOMEM;

					goto err_quit;
				}
				memcpy(adapter->maintain_buf, dma_buf,
				       reply_bytes);
			}

			if (dma_buf) {
				dma_free_coherent(&hw->pdev->dev,
						  adapter->maintain_dma_size,
						  dma_buf, dma_phy);
			}
			adapter->maintain_dma_buf = NULL;

			break;
		case rnpgbe_hw_n500:
		case rnpgbe_hw_n210:
			if (req->cmd) {
				int data_len;
				int ram_size = mbx->share_size;

				offset = 0;
				while (offset < req->req_data_bytes) {
					data_len =
						(req->req_data_bytes - offset) >
								ram_size ?
							ram_size :
							(req->req_data_bytes -
							 offset);
					// copy to ram
					n500_exchange_share_ram(
						hw,
						(u32 *)(dma_buf + offset +
							sizeof(*req)),
						1, data_len);
					err = rnpgbe_maintain_req(hw, req->cmd,
								  req->arg0,
								  offset, 0, 0);
					if (err != 0)
						goto err_quit;

					offset += data_len;
				}
			} else {
				int data_len;
				int ram_size = mbx->share_size;
				struct maintain_reply reply;
				// it is a read
				adapter->maintain_buf_len = reply_bytes;
				adapter->maintain_buf = kmalloc(
					adapter->maintain_buf_len, GFP_KERNEL);
				if (!adapter->maintain_buf) {
					netdev_err(
						netdev,
						"No Memory for maintain buf:%d\n",
						adapter->maintain_buf_len);
					err = -ENOMEM;

					goto err_quit;
				}
				// first setup reply
				reply.magic = MAINTAIN_REPLY_MAGIC;
				reply.cmd = req->cmd;
				reply.arg0 = req->arg0;
				reply.data_bytes = req->reply_bytes;
				memcpy(adapter->maintain_buf, &reply,
				       sizeof(struct maintain_reply));
				/* copy req first */
				offset = 0;
				while (offset < reply_bytes) {
					data_len =
						(reply_bytes - offset) >
								ram_size ?
							ram_size :
							(reply_bytes - offset);
					err = rnpgbe_maintain_req(hw, req->cmd,
								  req->arg0, 0,
								  offset, 0);
					if (err != 0)
						goto err_quit;
					// copy to ram
					n500_exchange_share_ram(
						hw,
						(u32 *)(adapter->maintain_buf +
							offset + sizeof(*req)),
						0, data_len);
					offset += data_len;
				}
			}
			if (dma_buf) {
				dma_free_coherent(&hw->pdev->dev,
						  adapter->maintain_dma_size,
						  dma_buf, dma_phy);
			}
			adapter->maintain_dma_buf = NULL;

			break;
		default:

			break;
		}
	}

	return count;
err_quit:
	if (dma_buf) {
		dma_free_coherent(&hw->pdev->dev, adapter->maintain_dma_size,
				  dma_buf, dma_phy);
		adapter->maintain_dma_buf = NULL;
	}
	return err;
}

static BIN_ATTR(maintain, 0644, maintain_read, maintain_write,
		1 * 1024 * 1024);
#endif
#ifdef TEST_PF_RESET
static ssize_t test_info_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);

	int ret = 0;
	int i;
	struct rnpgbe_q_vector *q_vector;

	for (i = 0; i < adapter->num_tx_queues; i++) {
		q_vector = adapter->q_vector[i];
		ret += sprintf(buf + ret, "q_vector %d itr  %d\n",
			       q_vector->v_idx, q_vector->itr_rx >> 2);
	}

	return ret;
}

static ssize_t test_info_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	int ret = count;

	return ret;
}
#endif
static ssize_t rx_desc_info_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	u32 rx_ring_num = adapter->sysfs_rx_ring_num;
	u32 rx_desc_num = adapter->sysfs_rx_desc_num;
	struct rnpgbe_ring *ring = adapter->rx_ring[rx_ring_num];
	int ret = 0;
	union rnpgbe_rx_desc *desc;

	desc = RNP_RX_DESC(ring, rx_desc_num);
	ret += sprintf(buf + ret, "rx ring %d desc %d:\n", rx_ring_num,
		       rx_desc_num);
	ret += print_desc(buf + ret, desc, sizeof(*desc));
	ret += sprintf(buf + ret, "\n");

	return ret;
}

static ssize_t rx_desc_info_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	int ret = count;

	u32 rx_desc_num = adapter->sysfs_rx_desc_num;
	u32 rx_ring_num = adapter->sysfs_rx_ring_num;

	struct rnpgbe_ring *ring = adapter->rx_ring[rx_ring_num];

	if (kstrtou32(buf, 0, &rx_desc_num) != 0)
		return -EINVAL;
	if (rx_desc_num < ring->count)
		adapter->sysfs_rx_desc_num = rx_desc_num;
	else
		ret = -EINVAL;

	return ret;
}

static ssize_t tcp_sync_info_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	int ret = 0;

	if (adapter->priv_flags & RNP_PRIV_FLAG_TCP_SYNC)
		ret += sprintf(
			buf + ret, "tcp sync remap on queue %d prio %s\n",
			adapter->tcp_sync_queue,
			(adapter->priv_flags & RNP_PRIV_FLAG_TCP_SYNC_PRIO) ?
				"NO" :
				"OFF");
	else
		ret += sprintf(buf + ret, "tcp sync remap off\n");

	return ret;
}

static ssize_t tcp_sync_info_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	struct rnpgbe_hw *hw = &adapter->hw;
	int ret = count;
	u32 tcp_sync_queue;

	if (kstrtou32(buf, 0, &tcp_sync_queue) != 0)
		return -EINVAL;

	if (tcp_sync_queue < adapter->num_rx_queues) {
		adapter->tcp_sync_queue = tcp_sync_queue;
		adapter->priv_flags |= RNP_PRIV_FLAG_TCP_SYNC;

		if (adapter->priv_flags & RNP_PRIV_FLAG_TCP_SYNC_PRIO)
			hw->ops.set_tcp_sync_remapping(
				hw, adapter->tcp_sync_queue, true, true);
		else
			hw->ops.set_tcp_sync_remapping(
				hw, adapter->tcp_sync_queue, true, false);

	} else {
		adapter->priv_flags &= ~RNP_PRIV_FLAG_TCP_SYNC;

		hw->ops.set_tcp_sync_remapping(hw, adapter->tcp_sync_queue,
					       false, false);
	}

	return ret;
}

static ssize_t rx_skip_info_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	int ret = 0;

	if (adapter->priv_flags & RNP_PRIV_FLAG_RX_SKIP_EN)
		ret += sprintf(buf + ret, "rx skip bytes: %d\n",
			       16 * (adapter->priv_skip_count + 1));
	else
		ret += sprintf(buf + ret, "rx skip off\n");

	return ret;
}

__maybe_unused static ssize_t store_rx_skip_info(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t count)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	struct rnpgbe_hw *hw = &adapter->hw;
	int ret = count;
	u32 rx_skip_count;

	if (kstrtou32(buf, 0, &rx_skip_count) != 0)
		return -EINVAL;

	if ((rx_skip_count > 0) && (rx_skip_count < 17)) {
		adapter->priv_skip_count = rx_skip_count - 1;
		adapter->priv_flags |= RNP_PRIV_FLAG_RX_SKIP_EN;
		hw->ops.set_rx_skip(hw, adapter->priv_skip_count, true);

	} else {
		adapter->priv_flags &= ~RNP_PRIV_FLAG_RX_SKIP_EN;

		hw->ops.set_rx_skip(hw, adapter->priv_skip_count, false);

		return -EINVAL;
	}

	return ret;
}

static ssize_t rx_drop_info_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	int ret = 0;

	ret += sprintf(buf + ret, "rx_drop_status %llx\n",
		       adapter->rx_drop_status);

	return ret;
}

static ssize_t rx_drop_info_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	struct rnpgbe_hw *hw = &adapter->hw;
	int ret = count;
	u64 rx_drop_status;

	if (kstrtou64(buf, 0, &rx_drop_status) != 0)
		return -EINVAL;

	adapter->rx_drop_status = rx_drop_status;

	hw->ops.update_rx_drop(hw);

	return ret;
}

static ssize_t outer_vlan_info_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	int ret = 0;

	if (adapter->priv_flags & RNP_PRIV_FLAG_DOUBLE_VLAN)
		ret += sprintf(buf + ret, "double vlan on\n");
	else
		ret += sprintf(buf + ret, "double vlan off\n");

	switch (adapter->outer_vlan_type) {
	case outer_vlan_type_88a8:
		ret += sprintf(buf + ret, "outer vlan 0x88a8\n");

		break;
#ifdef ETH_P_QINQ1
	case outer_vlan_type_9100:
		ret += sprintf(buf + ret, "outer vlan 0x9100\n");

		break;
#endif
#ifdef ETH_P_QINQ2
	case outer_vlan_type_9200:
		ret += sprintf(buf + ret, "outer vlan 0x9200\n");

		break;
#endif
	default:
		ret += sprintf(buf + ret, "outer vlan error\n");
		break;
	}
	return ret;
}

static ssize_t outer_vlan_info_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	struct rnpgbe_hw *hw = &adapter->hw;
	int ret = count;
	u32 outer_vlan_type;

	if (kstrtou32(buf, 0, &outer_vlan_type) != 0)
		return -EINVAL;
	if (outer_vlan_type < outer_vlan_type_max)
		adapter->outer_vlan_type = outer_vlan_type;
	else
		ret = -EINVAL;
	if (hw->ops.set_outer_vlan_type)
		hw->ops.set_outer_vlan_type(hw, outer_vlan_type);

	return ret;
}

static ssize_t tx_stags_info_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	int ret = 0;

	if (adapter->flags2 & RNP_FLAG2_VLAN_STAGS_ENABLED)
		ret += sprintf(buf + ret, "tx stags on\n");
	else
		ret += sprintf(buf + ret, "tx stags off\n");

	ret += sprintf(buf + ret, "vid 0x%x\n", adapter->stags_vid);

	return ret;
}

static ssize_t tx_stags_info_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	struct rnpgbe_hw *hw = &adapter->hw;

	struct rnpgbe_eth_info *eth = &hw->eth;
	int ret = count;
	u16 tx_stags;

	if (kstrtou16(buf, 0, &tx_stags) != 0)
		return -EINVAL;
	if (tx_stags < VLAN_N_VID)
		adapter->stags_vid = tx_stags;
	else
		ret = -EINVAL;

	eth->ops.set_vfta(eth, adapter->stags_vid, true);

	return ret;
}

static ssize_t gephy_test_info_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	int ret = 0;

	if (adapter->gephy_test_mode)
		ret += sprintf(buf + ret, "gephy_test on: %d\n",
			       adapter->gephy_test_mode);
	else
		ret += sprintf(buf + ret, "gephy_test off\n");

	return ret;
}

static ssize_t gephy_test_info_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	struct rnpgbe_hw *hw = &adapter->hw;

	//struct rnpgbe_eth_info *eth = &hw->eth;
	int ret = count;
	u32 test_mode;

#define MAX_MODE (5)
	if (kstrtou32(buf, 0, &test_mode) != 0)
		return -EINVAL;
	if (test_mode < 5)
		adapter->gephy_test_mode = test_mode;
	else
		ret = -EINVAL;

	rnpgbe_mbx_gephy_test_set(hw, test_mode);

	return ret;
}

static ssize_t tx_desc_info_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	u32 tx_ring_num = adapter->sysfs_tx_ring_num;
	u32 tx_desc_num = adapter->sysfs_tx_desc_num;
	struct rnpgbe_ring *ring = adapter->tx_ring[tx_ring_num];
	int ret = 0;
	struct rnpgbe_tx_desc *desc;

	desc = RNP_TX_DESC(ring, tx_desc_num);
	ret += sprintf(buf + ret, "tx ring %d desc %d:\n", tx_ring_num,
		       tx_desc_num);
	ret += print_desc(buf + ret, desc, sizeof(*desc));
	ret += sprintf(buf + ret, "\n");

	return ret;
}

static ssize_t tx_desc_info_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	int ret = count;

	u32 tx_desc_num = adapter->sysfs_tx_desc_num;
	u32 tx_ring_num = adapter->sysfs_tx_ring_num;

	struct rnpgbe_ring *ring = adapter->tx_ring[tx_ring_num];

	if (kstrtou32(buf, 0, &tx_desc_num) != 0)
		return -EINVAL;
	if (tx_desc_num < ring->count)
		adapter->sysfs_tx_desc_num = tx_desc_num;
	else
		ret = -EINVAL;

	return ret;
}

static ssize_t rx_ring_info_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	u32 rx_ring_num = adapter->sysfs_rx_ring_num;
	struct rnpgbe_ring *ring = adapter->rx_ring[rx_ring_num];
	int ret = 0;
	union rnpgbe_rx_desc *rx_desc;

	ret += sprintf(buf + ret, "queue %d info:\n", rx_ring_num);

	ret += sprintf(buf + ret, "next_to_use %d\n", ring->next_to_use);
	ret += sprintf(buf + ret, "next_to_clean %d\n", ring->next_to_clean);

	rx_desc = RNP_RX_DESC(ring, ring->next_to_clean);
	ret += sprintf(buf + ret, "next_to_clean desc: ");
	ret += print_desc(buf + ret, rx_desc, sizeof(*rx_desc));
	ret += sprintf(buf + ret, "\n");

	return ret;
}

static ssize_t rx_ring_info_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	int ret = count;

	u32 rx_ring_num = adapter->sysfs_rx_ring_num;

	if (kstrtou32(buf, 0, &rx_ring_num) != 0)
		return -EINVAL;
	if (rx_ring_num < adapter->num_rx_queues)
		adapter->sysfs_rx_ring_num = rx_ring_num;
	else
		ret = -EINVAL;

	return ret;
}

static ssize_t mii_reg_info_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	int ret = count;

	u32 reg_num;

	if (kstrtou32(buf, 0, &reg_num) != 0)
		return -EINVAL;
	adapter->sysfs_mii_reg = reg_num;

	return ret;
}

static ssize_t mii_control_info_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	int ret = count;

	u32 reg_num;

	if (kstrtou32(buf, 0, &reg_num) != 0)
		return -EINVAL;
	adapter->sysfs_mii_control = reg_num;

	return ret;
}

static ssize_t mii_value_info_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	int ret = count;

	u32 reg_value;

	if (kstrtou32(buf, 0, &reg_value) != 0)
		return -EINVAL;
	adapter->sysfs_mii_value = reg_value;

	return ret;
}

static int rnpgbe_mdio_read(struct net_device *netdev, int prtad, int devad,
			    u32 addr, u32 *phy_value)
{
	int rc = -EIO;
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	struct rnpgbe_hw *hw = &adapter->hw;
	u16 value;

	rc = hw->ops.phy_read_reg(hw, addr, 0, &value);
	*phy_value = value;

	return rc;
}

static int rnpgbe_mdio_write(struct net_device *netdev, int prtad, int devad,
			     u16 addr, u16 value)
{
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	struct rnpgbe_hw *hw = &adapter->hw;

	return hw->ops.phy_write_reg(hw, addr, 0, value);
}

static ssize_t mii_info_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	u32 reg_num = adapter->sysfs_mii_reg;
	u32 reg_value = adapter->sysfs_mii_value;
	int ret = 0;
	u32 value;

	if (adapter->sysfs_mii_control) {
		//write
		rnpgbe_mdio_write(netdev, 0, 0, reg_num, reg_value);
		ret += sprintf(buf + ret, "write reg %x : %x\n", reg_num,
			       reg_value);

	} else {
		rnpgbe_mdio_read(netdev, 0, 0, reg_num, &value);
		ret += sprintf(buf + ret, "read reg %x : %x\n", reg_num, value);
		//read
	}

	return ret;
}

static ssize_t tx_ring_info_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	u32 tx_ring_num = adapter->sysfs_tx_ring_num;
	struct rnpgbe_ring *ring = adapter->tx_ring[tx_ring_num];
	int ret = 0;
	struct rnpgbe_tx_buffer *tx_buffer;
	//struct rnpgbe_tx_desc *tx_desc;
	struct rnpgbe_tx_desc *eop_desc;

	// print all tx_ring_num info
	ret += sprintf(buf + ret, "queue %d info:\n", tx_ring_num);

	ret += sprintf(buf + ret, "next_to_use %d\n", ring->next_to_use);
	ret += sprintf(buf + ret, "next_to_clean %d\n", ring->next_to_clean);

	tx_buffer = &ring->tx_buffer_info[ring->next_to_clean];
	eop_desc = tx_buffer->next_to_watch;
	/* if have watch desc */
	if (eop_desc) {
		ret += sprintf(buf + ret, "next_to_watch:\n");
		ret += print_desc(buf + ret, eop_desc, sizeof(*eop_desc));
		ret += sprintf(buf + ret, "\n");
	} else {
		ret += sprintf(buf + ret, "no next_to_watch data\n");
	}

	return ret;
}

static ssize_t tx_ring_info_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	int ret = count;

	u32 tx_ring_num = adapter->sysfs_tx_ring_num;

	if (kstrtou32(buf, 0, &tx_ring_num) != 0)
		return -EINVAL;

	if (tx_ring_num < adapter->num_tx_queues)
		adapter->sysfs_tx_ring_num = tx_ring_num;
	else
		ret = -EINVAL;

	return ret;
}

static ssize_t queue_mapping_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int i;
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	struct rnpgbe_ring *ring;
	struct rnpgbe_q_vector *q_vector;

	ret += sprintf(buf + ret, "tx_queue count %d\n",
		       adapter->num_tx_queues);
	ret += sprintf(buf + ret, "queue-mapping :\n");
	for (i = 0; i < adapter->num_tx_queues; i++) {
		ring = adapter->tx_ring[i];
		ret += sprintf(buf + ret, "tx queue %d <---> ring %d\n", i,
			       ring->rnpgbe_queue_idx);
	}
	ret += sprintf(buf + ret, "rx_queue count %d\n",
		       adapter->num_rx_queues);
	ret += sprintf(buf + ret, "queue-mapping :\n");
	for (i = 0; i < adapter->num_rx_queues; i++) {
		ring = adapter->rx_ring[i];
		ret += sprintf(buf + ret, "rx queue %d <---> ring %d\n", i,
			       ring->rnpgbe_queue_idx);
	}
	ret += sprintf(buf + ret, "vector-queue mapping:\n");
	for (i = 0; i < adapter->num_q_vectors; i++) {
		q_vector = adapter->q_vector[i];
		ret += sprintf(buf + ret, "---vector %d---\n", i);
		rnpgbe_for_each_ring(ring, q_vector->tx) {
			ret += sprintf(buf + ret, "tx ring %d\n",
				       ring->rnpgbe_queue_idx);
		}
		rnpgbe_for_each_ring(ring, q_vector->rx) {
			ret += sprintf(buf + ret, "rx ring %d\n",
				       ring->rnpgbe_queue_idx);
		}
	}

	return ret;
}

static ssize_t queue_mapping_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	return count;
}

static ssize_t tx_counts_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	u32 val = 0;
	int ret = 0;
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	struct rnpgbe_hw *hw = &adapter->hw;

	ret += sprintf(buf + ret, "tx counters\n");
	ret += sprintf(buf + ret, "ring0-tx:\n");

	val = rd32(hw, RNP_DMA_REG_TX_DESC_BUF_LEN);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "len:", RNP_DMA_REG_TX_DESC_BUF_LEN, val);

	val = rd32(hw, RNP_DMA_REG_TX_DESC_BUF_HEAD);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "head:", RNP_DMA_REG_TX_DESC_BUF_HEAD, val);

	val = rd32(hw, RNP_DMA_REG_TX_DESC_BUF_TAIL);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "tail:", RNP_DMA_REG_TX_DESC_BUF_TAIL, val);

	ret += sprintf(buf + ret, "to_1to4_p1:\n");

	val = rd32(hw, RNP_ETH_1TO4_INST0_IN_PKTS);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "emac_in:", RNP_ETH_1TO4_INST0_IN_PKTS, val);

	val = rd32(hw, RNP_ETH_IN_0_TX_PKT_NUM(0));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "emac_send:", RNP_ETH_IN_0_TX_PKT_NUM(0), val);

	ret += sprintf(buf + ret, "to_1to4_p2:\n");

	val = rd32(hw, RNP_ETH_IN_1_TX_PKT_NUM(0));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "sop_pkt:", RNP_ETH_IN_1_TX_PKT_NUM(0), val);

	val = rd32(hw, RNP_ETH_IN_2_TX_PKT_NUM(0));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "eop_pkt:", RNP_ETH_IN_2_TX_PKT_NUM(0), val);

	val = rd32(hw, RNP_ETH_IN_3_TX_PKT_NUM(0));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "send_terr:", RNP_ETH_IN_3_TX_PKT_NUM(0), val);

	ret += sprintf(buf + ret, "to_tx_trans(phy):\n");

	val = rd32(hw, RNP_ETH_EMAC_TX_TO_PHY_PKTS(0));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "in:", RNP_ETH_EMAC_TX_TO_PHY_PKTS(0), val);

	val = rd32(hw, RNP_ETH_TXTRANS_PTP_PKT_NUM(0));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "out:", RNP_ETH_TXTRANS_PTP_PKT_NUM(0), val);

	ret += sprintf(buf + ret, "mac:\n");

	val = rd32(hw, 0x1081c);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n", "tx:", 0x1081c, val);

	val = rd32(hw, 0x1087c);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "underflow_err:", 0x1087c, val);

	val = rd32(hw, RNP_ETH_TX_DEBUG(0));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "port0_txtrans_sop:", RNP_ETH_TX_DEBUG(0), val);

	val = rd32(hw, RNP_ETH_TX_DEBUG(4));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "port0_txtrans_eop:", RNP_ETH_TX_DEBUG(4), val);

	val = rd32(hw, RNP_ETH_TX_DEBUG(13));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "tx_empty:", RNP_ETH_TX_DEBUG(13), val);

	val = rd32(hw, RNP_ETH_TX_DEBUG(14));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: 0x%x\n",
		       "tx_prog_full:", RNP_ETH_TX_DEBUG(14), val);

	val = rd32(hw, RNP_ETH_TX_DEBUG(15));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: 0x%x\n",
		       "tx_full:", RNP_ETH_TX_DEBUG(15), val);

	return ret;
}

static DEVICE_ATTR(tx_counts, 0644, tx_counts_show, NULL);

static ssize_t rx_count_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	u32 val = 0, port = 0;
	int ret = 0;
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	struct rnpgbe_hw *hw = &adapter->hw;

	ret += sprintf(buf + ret, "rx counters\n");
	for (port = 0; port < 4; port++) {
		ret += sprintf(buf + ret, "emac_rx_trans (port:%d):\n", port);

		val = rd32(hw, RNP_RXTRANS_RX_PKTS(port));
		ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
			       "pkts:", RNP_RXTRANS_RX_PKTS(port), val);

		val = rd32(hw, RNP_RXTRANS_DROP_PKTS(port));
		ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
			       "drop:", RNP_RXTRANS_DROP_PKTS(port), val);

		val = rd32(hw, RNP_RXTRANS_WDT_ERR_PKTS(port));
		ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
			       "wdt_err:", RNP_RXTRANS_WDT_ERR_PKTS(port), val);

		val = rd32(hw, RNP_RXTRANS_CODE_ERR_PKTS(port));
		ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
			       "code_err:", RNP_RXTRANS_CODE_ERR_PKTS(port),
			       val);

		val = rd32(hw, RNP_RXTRANS_CRC_ERR_PKTS(port));
		ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
			       "crc_err:", RNP_RXTRANS_CRC_ERR_PKTS(port), val);

		val = rd32(hw, RNP_RXTRANS_SLEN_ERR_PKTS(port));
		ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
			       "slen_err:", RNP_RXTRANS_SLEN_ERR_PKTS(port),
			       val);

		val = rd32(hw, RNP_RXTRANS_GLEN_ERR_PKTS(port));
		ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
			       "glen_err:", RNP_RXTRANS_GLEN_ERR_PKTS(port),
			       val);

		val = rd32(hw, RNP_RXTRANS_IPH_ERR_PKTS(port));
		ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
			       "iph_err:", RNP_RXTRANS_IPH_ERR_PKTS(port), val);

		val = rd32(hw, RNP_RXTRANS_CSUM_ERR_PKTS(port));
		ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
			       "csum_err:", RNP_RXTRANS_CSUM_ERR_PKTS(port),
			       val);

		val = rd32(hw, RNP_RXTRANS_LEN_ERR_PKTS(port));
		ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
			       "len_err:", RNP_RXTRANS_LEN_ERR_PKTS(port), val);

		val = rd32(hw, RNP_RXTRANS_CUT_ERR_PKTS(port));
		ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
			       "trans_cut_err:", RNP_RXTRANS_CUT_ERR_PKTS(port),
			       val);

		val = rd32(hw, RNP_RXTRANS_EXCEPT_BYTES(port));
		ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
			       "expt_byte_err:", RNP_RXTRANS_EXCEPT_BYTES(port),
			       val);

		val = rd32(hw, RNP_RXTRANS_G1600_BYTES_PKTS(port));
		ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
			       ">1600Byte:", RNP_RXTRANS_G1600_BYTES_PKTS(port),
			       val);
	}

	ret += sprintf(buf + ret, "gather:\n");
	val = rd32(hw, RNP_ETH_TOTAL_GAT_RX_PKT_NUM);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "total_in_pkts:", RNP_ETH_TOTAL_GAT_RX_PKT_NUM, val);

	port = 0;
	val = rd32(hw, RNP_ETH_RX_PKT_NUM(port));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "to_nxt_mdodule:", RNP_ETH_RX_PKT_NUM(port), val);

	for (port = 0; port < 4; port++) {
		char pname[16] = { 0 };

		val = rd32(hw, RNP_ETH_RX_PKT_NUM(port));
		sprintf(pname, "p%d-rx:", port);
		ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n", pname,
			       RNP_ETH_RX_PKT_NUM(port), val);
	}

	for (port = 0; port < 4; port++) {
		char pname[16] = { 0 };

		val = rd32(hw, RNP_ETH_RX_DROP_PKT_NUM(port));
		sprintf(pname, "p%d-drop:", port);
		ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n", pname,
			       RNP_ETH_RX_DROP_PKT_NUM(port), val);
	}

	ret += sprintf(buf + ret, "ip-parse:\n");

	val = rd32(hw, RNP_ETH_PKT_EGRESS_NUM);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "pkg_egree:", RNP_ETH_PKT_EGRESS_NUM, val);

	val = rd32(hw, RNP_ETH_PKT_IP_HDR_LEN_ERR_NUM);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "L3_len_err:", RNP_ETH_PKT_IP_HDR_LEN_ERR_NUM, val);

	val = rd32(hw, RNP_ETH_PKT_IP_PKT_LEN_ERR_NUM);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "ip_hdr_err:", RNP_ETH_PKT_IP_PKT_LEN_ERR_NUM, val);

	val = rd32(hw, RNP_ETH_PKT_L3_HDR_CHK_ERR_NUM);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "l3-csum-err:", RNP_ETH_PKT_L3_HDR_CHK_ERR_NUM, val);

	val = rd32(hw, RNP_ETH_PKT_L4_HDR_CHK_ERR_NUM);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "l4-csum-err:", RNP_ETH_PKT_L4_HDR_CHK_ERR_NUM, val);

	val = rd32(hw, RNP_ETH_PKT_SCTP_CHK_ERR_NUM);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "sctp-err:", RNP_ETH_PKT_SCTP_CHK_ERR_NUM, val);

	val = rd32(hw, RNP_ETH_PKT_VLAN_ERR_NUM);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "vlan-err:", RNP_ETH_PKT_VLAN_ERR_NUM, val);

	val = rd32(hw, RNP_ETH_PKT_EXCEPT_SHORT_NUM);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "except_short_num:", RNP_ETH_PKT_EXCEPT_SHORT_NUM, val);

	val = rd32(hw, RNP_ETH_PKT_PTP_NUM);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "ptp:", RNP_ETH_PKT_PTP_NUM, val);

	ret += sprintf(buf + ret, "to-indecap:\n");

	val = rd32(hw, RNP_ETH_DECAP_PKT_IN_NUM);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "*in engin*:", RNP_ETH_DECAP_PKT_IN_NUM, val);

	val = rd32(hw, RNP_ETH_DECAP_PKT_OUT_NUM);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "*out engin*:", RNP_ETH_DECAP_PKT_OUT_NUM, val);

	val = rd32(hw, RNP_ETH_DECAP_DMAC_OUT_NUM);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "to-dma/host:", RNP_ETH_DECAP_DMAC_OUT_NUM, val);

	val = rd32(hw, RNP_ETH_DECAP_BMC_OUT_NUM);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "to-bmc:", RNP_ETH_DECAP_BMC_OUT_NUM, val);

	val = rd32(hw, RNP_ETH_DECAP_SW_OUT_NUM);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "to-switch:", RNP_ETH_DECAP_SW_OUT_NUM, val);

	val = rd32(hw, RNP_ETH_DECAP_MIRROR_OUT_NUM);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "bmc+host:", RNP_ETH_DECAP_MIRROR_OUT_NUM, val);

	val = rd32(hw, RNP_ETH_DECAP_PKT_DROP_NUM(0x0));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "err_drop:", RNP_ETH_DECAP_PKT_DROP_NUM(0x0), val);

	val = rd32(hw, RNP_ETH_DECAP_PKT_DROP_NUM(1));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "plicy_drop:", RNP_ETH_DECAP_PKT_DROP_NUM(1), val);

	val = rd32(hw, RNP_ETH_DECAP_PKT_DROP_NUM(2));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "dmac_drop:", RNP_ETH_DECAP_PKT_DROP_NUM(2), val);

	val = rd32(hw, RNP_ETH_DECAP_PKT_DROP_NUM(3));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "bmc_drop:", RNP_ETH_DECAP_PKT_DROP_NUM(3), val);

	val = rd32(hw, RNP_ETH_DECAP_PKT_DROP_NUM(4));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "sw_drop:", RNP_ETH_DECAP_PKT_DROP_NUM(4), val);

	val = rd32(hw, RNP_ETH_DECAP_PKT_DROP_NUM(5));
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "rm_vlane_num:", RNP_ETH_DECAP_PKT_DROP_NUM(5), val);

	ret += sprintf(buf + ret, "dma-2-host:\n");

	val = rd32(hw, 0x264);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n", "fifo equ:", 0x264,
		       val);

	val = rd32(hw, 0x268);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n", "fifo deq:", 0x268,
		       val);

	val = rd32(hw, 0x114);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n",
		       "unexpt_abtring:", 0x114, val);

	val = rd32(hw, 0x288);
	ret += sprintf(buf + ret, "\t %16s 0x%08x: %d\n", "pci2host:", 0x288,
		       val);

	for (port = 0; port < 4; port++) {
		ret += sprintf(buf + ret, "rx-ring%d:\n", port);

		val = rd32(hw, RNP_DMA_REG_RX_DESC_BUF_HEAD);
		ret += sprintf(buf + ret, "\t %16s 0x%08x: 0x%x\n",
			       "head:", RNP_DMA_REG_RX_DESC_BUF_HEAD, val);

		val = rd32(hw, RNP_DMA_REG_RX_DESC_BUF_TAIL);
		ret += sprintf(buf + ret, "\t %16s 0x%08x: 0x%x\n",
			       "tail:", RNP_DMA_REG_RX_DESC_BUF_TAIL, val);

		val = rd32(hw, RNP_DMA_REG_RX_DESC_BUF_LEN);
		ret += sprintf(buf + ret, "\t %16s 0x%08x: 0x%x\n",
			       "len:", RNP_DMA_REG_RX_DESC_BUF_LEN, val);
	}

	return ret;
}

static DEVICE_ATTR(rx_count, 0644, rx_count_show, NULL);

static ssize_t active_vid_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
#ifndef HAVE_VLAN_RX_REGISTER
	u16 vid;
#endif
	u16 current_vid = 0;
	int ret = 0;
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	struct rnpgbe_hw *hw = &adapter->hw;
	u8 vfnum = hw->max_vfs - 1; //use last-vf's table entry. the las

	if ((adapter->flags & RNP_FLAG_SRIOV_ENABLED)) {
		current_vid = rd32(hw, RNP_DMA_PORT_VEB_VID_TBL(adapter->port,
								vfnum));
	}

#ifndef HAVE_VLAN_RX_REGISTER
	for_each_set_bit(vid, adapter->active_vlans, VLAN_N_VID) {
		ret += sprintf(buf + ret, "%u%s ", vid,
			       (current_vid == vid ? "*" : ""));
	}
#endif
	ret += sprintf(buf + ret, "\n");
	return ret;
}

static ssize_t active_vid_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	u16 vid;
	int err = -EINVAL;
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
#ifndef HAVE_VLAN_RX_REGISTER
	struct rnpgbe_hw *hw = &adapter->hw;
	u8 vfnum = hw->max_vfs - 1; //use last-vf's table entry. the las
	int port = 0;
#endif

	if (!(adapter->flags & RNP_FLAG_SRIOV_ENABLED))
		return -EIO;

	if (kstrtou16(buf, 0, &vid) != 0)
		return -EINVAL;

#ifndef HAVE_VLAN_RX_REGISTER
	if ((vid < 4096) && test_bit(vid, adapter->active_vlans)) {
		if (rd32(hw, RNP_DMA_VERSION) >= 0x20201231) {
			for (port = 0; port < 4; port++)
				wr32(hw, RNP_DMA_PORT_VEB_VID_TBL(port, vfnum),
				     vid);
		} else {
			wr32(hw, RNP_DMA_PORT_VEB_VID_TBL(adapter->port, vfnum),
			     vid);
		}
		err = 0;
	}
#endif

	return err ? err : count;
}

static inline int pn_sn_dlen(unsigned char *v, int v_len)
{
	int i, len = 0;

	for (i = 0; i < v_len; i++) {
		if (isascii(v[i]))
			len++;
		else
			break;
	}
	return len;
}

int rnpgbe_mbx_get_pn_sn(struct rnpgbe_hw *hw, char pn[33], char sn[33])
{
	struct maintain_req *req;
	void *dma_buf = NULL;
	dma_addr_t dma_phy;
	struct ucfg_mac_sn *cfg;

	int err = 0, bytes = sizeof(sizeof(*req) + sizeof(struct ucfg_mac_sn));

	memset(pn, 0, 33);
	memset(sn, 0, 33);

	dma_buf =
		dma_alloc_coherent(&hw->pdev->dev, bytes, &dma_phy, GFP_ATOMIC);
	if (!dma_buf) {
		printk(KERN_DEBUG "%s: no memory:%d!", __func__, bytes);
		return -ENOMEM;
	}

	req = (struct maintain_req *)dma_buf;
	memset(dma_buf, 0, bytes);
	cfg = (struct ucfg_mac_sn *)(req + 1);
	req->magic = MAINTAIN_MAGIC;
	req->cmd = 0; // READ
	req->arg0 = 3; // PARTION 3
	req->req_data_bytes = 0;
	req->reply_bytes = bytes - sizeof(*req);

	err = rnpgbe_maintain_req(hw, req->cmd, req->arg0, req->req_data_bytes,
				  req->reply_bytes, dma_phy);
	if (err != 0)
		goto err_quit;

	if (cfg->magic == MAC_SN_MAGIC) {
		int sz = pn_sn_dlen(cfg->pn, 32);

		if (sz) {
			memcpy(pn, cfg->pn, sz);
			pn[sz] = 0;
		}
		sz = pn_sn_dlen(cfg->sn, 32);
		if (sz) {
			memcpy(sn, cfg->sn, sz);
			sn[sz] = 0;
		}
	}

err_quit:
	if (dma_buf)
		dma_free_coherent(&hw->pdev->dev, bytes, dma_buf, dma_phy);

	return 0;
}


static ssize_t port_idx_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	int ret = 0;
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);

	ret += sprintf(buf, "%d\n", adapter->portid_of_card);
	return ret;
}

static DEVICE_ATTR(port_idx, 0644, port_idx_show, NULL);

static ssize_t debug_link_stat_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	struct rnpgbe_hw *hw = &adapter->hw;

	ret += sprintf(buf, "%d %d dumy:0x%x up-flag:%d carry:%d\n",
		       adapter->link_up, adapter->hw.link, rd32(hw, 0xc),
		       adapter->flags & RNP_FLAG_NEED_LINK_UPDATE,
		       netif_carrier_ok(netdev));
	return ret;
}

static DEVICE_ATTR(debug_link_stat, 0644, debug_link_stat_show,
		   NULL);


static ssize_t pci_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int err = -EINVAL;
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	struct rnpgbe_hw *hw = &adapter->hw;
	int gen = 3, lanes = 8;

	if (count > 30)
		return -EINVAL;

	if (sscanf(buf, "gen%dx%d", &gen, &lanes) != 2) {
		printk(KERN_DEBUG "Error: invalid input. example: gen3x8\n");
		return -EINVAL;
	}
	if (gen > 3 || lanes > 8)
		return -EINVAL;

	err = rnpgbe_set_lane_fun(hw, LANE_FUN_PCI_LANE, gen, lanes, 0, 0);

	return err ? err : count;
}

static ssize_t pci_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int ret = 0;
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	struct rnpgbe_hw *hw = &adapter->hw;

	if (rnpgbe_mbx_get_lane_stat(hw) != 0)
		ret += sprintf(buf, " IO Error\n");
	else
		ret += sprintf(buf, "gen%dx%d\n", hw->pci_gen, hw->pci_lanes);

	return ret;
}
static DEVICE_ATTR(pci, 0644, pci_show, pci_store);

static ssize_t temperature_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	struct rnpgbe_hw *hw = &adapter->hw;
	int ret = 0, temp = 0, voltage = 0;

	temp = rnpgbe_mbx_get_temp(hw, &voltage);

	ret += sprintf(buf, "temp:%d oC \n", temp);
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

static ssize_t root_slot_info_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct net_device *netdev = to_net_device(dev);
	struct rnpgbe_adapter *adapter = netdev_priv(netdev);
	int ret = 0;
	struct pci_dev *root_pdev = pcie_find_root_port_old(adapter->pdev);

	if (root_pdev) {
		ret += sprintf(buf + ret, "%02x:%02x.%x\n",
			       root_pdev->bus->number,
			       PCI_SLOT(root_pdev->devfn),
			       PCI_FUNC(root_pdev->devfn));
	}
	return ret;
}


static DEVICE_ATTR(root_slot_info, 0644, root_slot_info_show, NULL);
static DEVICE_ATTR(temperature, 0644, temperature_show, NULL);
static DEVICE_ATTR(active_vid, 0644, active_vid_show, active_vid_store);
static DEVICE_ATTR(queue_mapping, 0644, queue_mapping_show,
		   queue_mapping_store);
static DEVICE_ATTR(tx_ring_info, 0644, tx_ring_info_show, tx_ring_info_store);
static DEVICE_ATTR(mii_info, 0644, mii_info_show, NULL);
static DEVICE_ATTR(mii_reg_info, 0644, NULL, mii_reg_info_store);
static DEVICE_ATTR(mii_control_info, 0644, NULL, mii_control_info_store);
static DEVICE_ATTR(mii_value_info, 0644, NULL, mii_value_info_store);
static DEVICE_ATTR(rx_ring_info, 0644, rx_ring_info_show, rx_ring_info_store);
static DEVICE_ATTR(tx_desc_info, 0644, tx_desc_info_show, tx_desc_info_store);
static DEVICE_ATTR(rx_desc_info, 0644, rx_desc_info_show, rx_desc_info_store);
static DEVICE_ATTR(rx_drop_info, 0644, rx_drop_info_show, rx_drop_info_store);
static DEVICE_ATTR(outer_vlan_info, 0644, outer_vlan_info_show,
		   outer_vlan_info_store);
static DEVICE_ATTR(tcp_sync_info, 0644, tcp_sync_info_show,
		   tcp_sync_info_store);
static DEVICE_ATTR(rx_skip_info, 0644, rx_skip_info_show, NULL);
static DEVICE_ATTR(tx_stags_info, 0644, tx_stags_info_show,
		   tx_stags_info_store);
static DEVICE_ATTR(gephy_test_info, 0644, gephy_test_info_show,
		   gephy_test_info_store);
#ifdef TEST_PF_RESET
static DEVICE_ATTR(test_info, 0644, test_info_show, test_info_store);
#endif

static struct attribute *dev_attrs[] = {
	&dev_attr_tx_stags_info.attr,
	&dev_attr_gephy_test_info.attr,
#ifdef TEST_PF_RESET
	&dev_attr_test_info.attr,
#endif
	&dev_attr_root_slot_info.attr,
	&dev_attr_active_vid.attr,
	&dev_attr_queue_mapping.attr,
	&dev_attr_rx_drop_info.attr,
	&dev_attr_outer_vlan_info.attr,
	&dev_attr_tcp_sync_info.attr,
	&dev_attr_rx_skip_info.attr,
	&dev_attr_tx_ring_info.attr,
	&dev_attr_mii_info.attr,
	&dev_attr_mii_control_info.attr,
	&dev_attr_mii_reg_info.attr,
	&dev_attr_mii_value_info.attr,
	&dev_attr_rx_ring_info.attr,
	&dev_attr_tx_desc_info.attr,
	&dev_attr_rx_desc_info.attr,
	&dev_attr_tx_counts.attr,
	&dev_attr_rx_count.attr,
	//&dev_attr_vpd.attr,
	&dev_attr_port_idx.attr,
	&dev_attr_temperature.attr,
	//&dev_attr_si.attr,
	//&dev_attr_sfp.attr,
	//&dev_attr_autoneg.attr,
	//&dev_attr_sfp_tx_disable.attr,
	//&dev_attr_fec.attr,
	//&dev_attr_link_traing.attr,
	&dev_attr_pci.attr,
	//&dev_attr_prbs.attr,
	&dev_attr_debug_link_stat.attr,
	//&dev_attr_switch_loopback_off.attr,
	//&dev_attr_switch_loopback_on.attr,
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

static void
rnpgbe_sysfs_del_adapter(struct rnpgbe_adapter __maybe_unused *adapter)
{
#ifdef RNP_HWMON
#ifndef HAVE_HWMON_DEVICE_REGISTER_WITH_GROUPS
	int i;

	if (adapter == NULL)
		return;

	for (i = 0; i < adapter->rnpgbe_hwmon_buff.n_hwmon; i++) {
		device_remove_file(
			pci_dev_to_dev(adapter->pdev),
			&adapter->rnpgbe_hwmon_buff.hwmon_list[i].dev_attr);
	}

	kfree(adapter->rnpgbe_hwmon_buff.hwmon_list);

	if (adapter->rnpgbe_hwmon_buff.device)
		hwmon_device_unregister(adapter->rnpgbe_hwmon_buff.device);
#endif /* HAVE_HWMON_DEVICE_REGISTER_WITH_GROUPS */
#endif /* RNP_HWMON */
}

/* called from rnpgbe_main.c */
void rnpgbe_sysfs_exit(struct rnpgbe_adapter *adapter)
{
	rnpgbe_sysfs_del_adapter(adapter);
	sysfs_remove_group(&adapter->netdev->dev.kobj, &dev_attr_grp);
	if (adapter->maintain_buf) {
		kfree(adapter->maintain_buf);
		adapter->maintain_buf = NULL;
		adapter->maintain_buf_len = 0;
	}
}

/* called from rnpgbe_main.c */
int rnpgbe_sysfs_init(struct rnpgbe_adapter *adapter)
{
	int rc = 0;
	int flag;
#ifdef RNP_HWMON
#ifdef HAVE_HWMON_DEVICE_REGISTER_WITH_GROUPS
	struct hwmon_buff *rnpgbe_hwmon;
	struct device *hwmon_dev;
#else
	struct hwmon_buff *rnpgbe_hwmon = &adapter->rnpgbe_hwmon_buff;
	int n_attrs;
#endif /* HAVE_HWMON_DEVICE_REGISTER_WITH_GROUPS */
	unsigned int i;
#endif /* RNP_HWMON */

	flag = sysfs_create_group(&adapter->netdev->dev.kobj, &dev_attr_grp);
	if (flag != 0) {
		dev_err(&adapter->netdev->dev,
			"sysfs_create_group faild:flag:%d\n", flag);
		return flag;
	}
#ifdef RNP_HWMON
	/* If this method isn't defined we don't support thermals */
	if (adapter->hw.ops.init_thermal_sensor_thresh == NULL)
		goto no_thermal;

	/* Don't create thermal hwmon interface if no sensors present */
	if (adapter->hw.ops.init_thermal_sensor_thresh(&adapter->hw))
		goto no_thermal;

#ifdef HAVE_HWMON_DEVICE_REGISTER_WITH_GROUPS
	rnpgbe_hwmon = devm_kzalloc(&adapter->pdev->dev, sizeof(*rnpgbe_hwmon),
				    GFP_KERNEL);

	if (!rnpgbe_hwmon) {
		rc = -ENOMEM;
		goto exit;
	}

	adapter->rnpgbe_hwmon_buff = rnpgbe_hwmon;
#else
	/*
	 * Allocation space for max attributs
	 * max num sensors * values (loc, temp, max, caution)
	 */
	n_attrs = RNP_MAX_SENSORS * 4;
	rnpgbe_hwmon->hwmon_list =
		kcalloc(n_attrs, sizeof(struct hwmon_attr), GFP_KERNEL);

	if (!rnpgbe_hwmon->hwmon_list) {
		rc = -ENOMEM;
		goto err;
	}
#endif /* HAVE_HWMON_DEVICE_REGISTER_WITH_GROUPS */

	for (i = 0; i < RNP_MAX_SENSORS; i++) {
		/*
		 * Only create hwmon sysfs entries for sensors that have
		 * meaningful data for.
		 */
		if (adapter->hw.thermal_sensor_data.sensor[i].location == 0)
			continue;

		/* Bail if any hwmon attr struct fails to initialize */
		rc = rnpgbe_add_hwmon_attr(adapter, i, RNP_HWMON_TYPE_CAUTION);
		if (rc)
			goto err;
		rc = rnpgbe_add_hwmon_attr(adapter, i, RNP_HWMON_TYPE_LOC);
		if (rc)
			goto err;
		rc = rnpgbe_add_hwmon_attr(adapter, i, RNP_HWMON_TYPE_TEMP);
		if (rc)
			goto err;
		rc = rnpgbe_add_hwmon_attr(adapter, i, RNP_HWMON_TYPE_MAX);
		if (rc)
			goto err;
	}

#ifdef HAVE_HWMON_DEVICE_REGISTER_WITH_GROUPS
	rnpgbe_hwmon->groups[0] = &rnpgbe_hwmon->group;
	rnpgbe_hwmon->group.attrs = rnpgbe_hwmon->attrs;

	hwmon_dev = devm_hwmon_device_register_with_groups(
		&adapter->pdev->dev, "rnp", rnpgbe_hwmon, rnpgbe_hwmon->groups);

	if (IS_ERR(hwmon_dev)) {
		rc = PTR_ERR(hwmon_dev);
		goto exit;
	}

#else
	rnpgbe_hwmon->device =
		hwmon_device_register(pci_dev_to_dev(adapter->pdev));

	if (IS_ERR(rnpgbe_hwmon->device)) {
		rc = PTR_ERR(rnpgbe_hwmon->device);
		goto err;
	}

#endif /* HAVE_HWMON_DEVICE_REGISTER_WITH_GROUPS */
no_thermal:
#endif /* RNP_HWMON */
	goto exit;

err:
	rnpgbe_sysfs_exit(adapter);
exit:
	return rc;
}
