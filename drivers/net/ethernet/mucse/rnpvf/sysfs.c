#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/sysfs.h>

#include "rnpvf_compat.h"

#include "rnpvf.h"

//#define	to_net_device(n) container_of(n, struct net_device, dev)

/*
static ssize_t show_active_vid(struct device *dev, struct device_attribute *attr, char *buf)
{
    u16 vid, current_vid;
    int ret = 0;
    struct net_device *netdev = to_net_device(dev);
	struct rnpvf_adapter *adapter = netdev_priv(netdev);
    struct rnpvf_hw* hw = &adapter->hw;

    current_vid = rd32(hw, RNP_DMA_PORT_VEB_VID_TBL(adapter->port,VFNUM(hw->vfnum)));

    for_each_set_bit(vid, adapter->active_vlans, VLAN_N_VID){
        ret += sprintf(buf + ret, "%u%s ", vid, (current_vid==vid?"*":""));
    }
    ret += sprintf(buf + ret, "\n");
    return ret;
}

static ssize_t store_active_vid(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    u16 vid;
    int err = -EINVAL;
    struct net_device *netdev = to_net_device(dev);
	struct rnpvf_adapter *adapter = netdev_priv(netdev);
    struct rnpvf_hw* hw = &adapter->hw;

    if (0 != kstrtou16(buf, 0, &vid))
        return -EINVAL;

    if((vid < 4096) && test_bit(vid, adapter->active_vlans)){
        wr32(hw, RNP_DMA_PORT_VEB_VID_TBL(adapter->port,VFNUM(hw->vfnum)), vid);
        err = 0;
    }

    return err ? err : count;
}

static DEVICE_ATTR(active_vid, S_IRUGO | S_IWUSR, show_active_vid, store_active_vid);

static struct attribute *dev_attrs[] = {
   // &dev_attr_active_vid.attr,
    NULL,
};

static struct attribute_group dev_attr_grp = {
    .attrs = dev_attrs,
};
*/

int rnpvf_sysfs_init(struct net_device *ndev)
{
//    int err;
//
//    err = sysfs_create_group(&ndev->dev.kobj, &dev_attr_grp);
//    if (err != 0){
//        dev_err(&ndev->dev, "sysfs_create_group faild:err:%d\n", err);
//        return err;
//    }
    return 0;
}

void rnpvf_sysfs_exit(struct net_device *ndev)
{
//    sysfs_remove_group(&ndev->dev.kobj, &dev_attr_grp);
}
