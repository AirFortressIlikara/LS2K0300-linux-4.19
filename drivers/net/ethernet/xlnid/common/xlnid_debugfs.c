// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2008 - 2022 Xel Technology.  */

#include "xlnid.h"

#ifdef HAVE_XLNID_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/module.h>

static struct dentry *xlnid_dbg_root;

static char xlnid_dbg_reg_ops_buf[256] = "";

/**
 * xlnid_dbg_reg_ops_read - read for reg_ops datum
 * @filp: the opened file
 * @buffer: where to write the data for the user to read
 * @count: the size of the user's buffer
 * @ppos: file position offset
 **/
static ssize_t xlnid_dbg_reg_ops_read(struct file *filp, char __user *buffer,
				    size_t count, loff_t *ppos)
{
	struct xlnid_adapter *adapter = filp->private_data;
	char *buf;
	int len;

	/* don't allow partial reads */
	if (*ppos != 0)
		return 0;

	buf = kasprintf(GFP_KERNEL, "%s: %s\n",
			adapter->netdev->name,
			xlnid_dbg_reg_ops_buf);
	if (!buf)
		return -ENOMEM;

	if (count < strlen(buf)) {
		kfree(buf);
		return -ENOSPC;
	}

	len = simple_read_from_buffer(buffer, count, ppos, buf, strlen(buf));

	kfree(buf);
	return len;
}

/**
 * xlnid_dbg_reg_ops_write - write into reg_ops datum
 * @filp: the opened file
 * @buffer: where to find the user's data
 * @count: the length of the user's data
 * @ppos: file position offset
 **/
static ssize_t xlnid_dbg_reg_ops_write(struct file *filp,
				     const char __user *buffer,
				     size_t count, loff_t *ppos)
{
	struct xlnid_adapter *adapter = filp->private_data;
	int len;

	/* don't allow partial writes */
	if (*ppos != 0)
		return 0;
	if (count >= sizeof(xlnid_dbg_reg_ops_buf))
		return -ENOSPC;

	len = simple_write_to_buffer(xlnid_dbg_reg_ops_buf,
				     sizeof(xlnid_dbg_reg_ops_buf)-1,
				     ppos,
				     buffer,
				     count);
	if (len < 0)
		return len;

	xlnid_dbg_reg_ops_buf[len] = '\0';

	if (strncmp(xlnid_dbg_reg_ops_buf, "write", 5) == 0) {
		u32 reg, value;
		int cnt;
		cnt = sscanf(&xlnid_dbg_reg_ops_buf[5], "%x %x", &reg, &value);
		/* check format and bounds check register access */
		if (cnt == 2 ) {
			xlnid_write_reg(&adapter->hw, reg, value, true);
			value = xlnid_read_reg(&adapter->hw, reg, false, true);
			e_dev_info("write: 0x%08x = 0x%08x\n", reg, value);
		} else {
			e_dev_info("write <reg> <value>\n");
		}
	} else if (strncmp(xlnid_dbg_reg_ops_buf, "read", 4) == 0) {
		u32 reg, value;
		int cnt;
		cnt = sscanf(&xlnid_dbg_reg_ops_buf[4], "%x", &reg);
		/* check format and bounds check register access */
		if (cnt == 1 ) {
			value = xlnid_read_reg(&adapter->hw, reg, false, true);
			e_dev_info("read 0x%08x = 0x%08x\n", reg, value);
		} else {
			e_dev_info("read <reg>\n");
		}
	} else {
		e_dev_info("Unknown command %s\n", xlnid_dbg_reg_ops_buf);
		e_dev_info("Available commands:\n");
		e_dev_info("   read <reg>\n");
		e_dev_info("   write <reg> <value>\n");
	}
	return count;
}

static const struct file_operations xlnid_dbg_reg_ops_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read =  xlnid_dbg_reg_ops_read,
	.write = xlnid_dbg_reg_ops_write,
};

static char xlnid_dbg_netdev_ops_buf[256] = "";

/**
 * xlnid_dbg_netdev_ops_read - read for netdev_ops datum
 * @filp: the opened file
 * @buffer: where to write the data for the user to read
 * @count: the size of the user's buffer
 * @ppos: file position offset
 **/
static ssize_t xlnid_dbg_netdev_ops_read(struct file *filp,
					 char __user *buffer,
					 size_t count, loff_t *ppos)
{
	struct xlnid_adapter *adapter = filp->private_data;
	char *buf;
	int len;

	/* don't allow partial reads */
	if (*ppos != 0)
		return 0;

	buf = kasprintf(GFP_KERNEL, "%s: %s\n",
			adapter->netdev->name,
			xlnid_dbg_netdev_ops_buf);
	if (!buf)
		return -ENOMEM;

	if (count < strlen(buf)) {
		kfree(buf);
		return -ENOSPC;
	}

	len = simple_read_from_buffer(buffer, count, ppos, buf, strlen(buf));

	kfree(buf);
	return len;
}

/**
 * xlnid_dbg_netdev_ops_write - write into netdev_ops datum
 * @filp: the opened file
 * @buffer: where to find the user's data
 * @count: the length of the user's data
 * @ppos: file position offset
 **/
static ssize_t xlnid_dbg_netdev_ops_write(struct file *filp,
					  const char __user *buffer,
					  size_t count, loff_t *ppos)
{
	struct xlnid_adapter *adapter = filp->private_data;
	int len;

	/* don't allow partial writes */
	if (*ppos != 0)
		return 0;
	if (count >= sizeof(xlnid_dbg_netdev_ops_buf))
		return -ENOSPC;

	len = simple_write_to_buffer(xlnid_dbg_netdev_ops_buf,
				     sizeof(xlnid_dbg_netdev_ops_buf)-1,
				     ppos,
				     buffer,
				     count);
	if (len < 0)
		return len;

	xlnid_dbg_netdev_ops_buf[len] = '\0';

	if (strncmp(xlnid_dbg_netdev_ops_buf, "tx_timeout", 10) == 0) {
#ifdef HAVE_NET_DEVICE_OPS
#ifdef HAVE_TX_TIMEOUT_TXQUEUE
        adapter->netdev->netdev_ops->ndo_tx_timeout(adapter->netdev,
                                        UINT_MAX);
#else
		adapter->netdev->netdev_ops->ndo_tx_timeout(adapter->netdev);
#endif
#else
		adapter->netdev->tx_timeout(adapter->netdev);
#endif /* HAVE_NET_DEVICE_OPS */
		e_dev_info("tx_timeout called\n");
	} else {
		e_dev_info("Unknown command: %s\n", xlnid_dbg_netdev_ops_buf);
		e_dev_info("Available commands:\n");
		e_dev_info("    tx_timeout\n");
	}
	return count;
}

static struct file_operations xlnid_dbg_netdev_ops_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = xlnid_dbg_netdev_ops_read,
	.write = xlnid_dbg_netdev_ops_write,
};

/**
 * xlnid_dbg_adapter_init - setup the debugfs directory for the adapter
 * @adapter: the adapter that is starting up
 **/
void xlnid_dbg_adapter_init(struct xlnid_adapter *adapter)
{
	const char *name = pci_name(adapter->pdev);
	struct dentry *pfile;
	adapter->xlnid_dbg_adapter = debugfs_create_dir(name, xlnid_dbg_root);
	if (adapter->xlnid_dbg_adapter) {
		pfile = debugfs_create_file("reg_ops", 0600,
					    adapter->xlnid_dbg_adapter, adapter,
					    &xlnid_dbg_reg_ops_fops);
		if (!pfile)
			e_dev_err("debugfs reg_ops for %s failed\n", name);
		pfile = debugfs_create_file("netdev_ops", 0600,
					    adapter->xlnid_dbg_adapter, adapter,
					    &xlnid_dbg_netdev_ops_fops);
		if (!pfile)
			e_dev_err("debugfs netdev_ops for %s failed\n", name);
	} else {
		e_dev_err("debugfs entry for %s failed\n", name);
	}
}

/**
 * xlnid_dbg_adapter_exit - clear out the adapter's debugfs entries
 * @adapter: board private structure
 **/
void xlnid_dbg_adapter_exit(struct xlnid_adapter *adapter)
{
	if (adapter->xlnid_dbg_adapter)
		debugfs_remove_recursive(adapter->xlnid_dbg_adapter);
	adapter->xlnid_dbg_adapter = NULL;
}

/**
 * xlnid_dbg_init - start up debugfs for the driver
 **/
void xlnid_dbg_init(const char *driver_name)
{
	xlnid_dbg_root = debugfs_create_dir(driver_name, NULL);
	if (xlnid_dbg_root == NULL)
		pr_err("init of debugfs failed\n");
}

/**
 * xlnid_dbg_exit - clean out the driver's debugfs entries
 **/
void xlnid_dbg_exit(void)
{
	debugfs_remove_recursive(xlnid_dbg_root);
}

#endif /* HAVE_XLNID_DEBUG_FS */
