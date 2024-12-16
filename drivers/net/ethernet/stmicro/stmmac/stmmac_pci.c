/*******************************************************************************
  This contains the functions to handle the pci driver.

  Copyright (C) 2011-2012  Vayavya Labs Pvt Ltd

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Author: Rayagond Kokatanur <rayagond@vayavyalabs.com>
  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#include <linux/pci.h>
#include <linux/dmi.h>

#include "stmmac.h"
#ifdef CONFIG_FIXED_PHY
#include "dwmac1000.h"
#include <linux/phy_fixed.h>
static struct fixed_phy_status fixed_phy_status __initdata = {
	.link		= 1,
	.speed		= 100,
	.duplex		= 1,
};

static int link_update(struct net_device *ndev, struct fixed_phy_status *fp)
{
	if (ndev && ndev->phydev && fp) {
		struct stmmac_priv *priv = netdev_priv(ndev);
		int link, duplex, speed, speed_value;
		u32 status;

		status = readl(priv->ioaddr + GMAC_RGSMIIIS);
		link = !!(status & GMAC_RGSMIIIS_LNKSTS);
		duplex = (status & GMAC_RGSMIIIS_LNKMODE);
		speed_value =
			(status & GMAC_RGSMIIIS_SPEED) >> GMAC_RGSMIIIS_SPEED_SHIFT;

		if (speed_value == GMAC_RGSMIIIS_SPEED_125)
			speed = SPEED_1000;
		else if (speed_value == GMAC_RGSMIIIS_SPEED_25)
			speed = SPEED_100;
		else
			speed = SPEED_10;
		if(!link){
			speed = SPEED_1000;
			duplex = DUPLEX_FULL;
		}
		fp->link = link;
		fp->duplex = duplex;
		fp->speed = speed;
	}
	return 0;
}
#endif

/*
 * This struct is used to associate PCI Function of MAC controller on a board,
 * discovered via DMI, with the address of PHY connected to the MAC. The
 * negative value of the address means that MAC controller is not connected
 * with PHY.
 */
struct stmmac_pci_func_data {
	unsigned int func;
	int phy_addr;
};

struct stmmac_pci_dmi_data {
	const struct stmmac_pci_func_data *func;
	size_t nfuncs;
};

struct stmmac_pci_info {
	int (*setup)(struct pci_dev *pdev, struct plat_stmmacenet_data *plat);
};

static int stmmac_pci_find_phy_addr(struct pci_dev *pdev,
				    const struct dmi_system_id *dmi_list)
{
	const struct stmmac_pci_func_data *func_data;
	const struct stmmac_pci_dmi_data *dmi_data;
	const struct dmi_system_id *dmi_id;
	int func = PCI_FUNC(pdev->devfn);
	size_t n;

	dmi_id = dmi_first_match(dmi_list);
	if (!dmi_id)
		return -ENODEV;

	dmi_data = dmi_id->driver_data;
	func_data = dmi_data->func;

	for (n = 0; n < dmi_data->nfuncs; n++, func_data++)
		if (func_data->func == func)
			return func_data->phy_addr;

	return -ENODEV;
}

static void common_default_data(struct plat_stmmacenet_data *plat)
{
	plat->clk_csr = 2;	/* clk_csr_i = 20-35MHz & MDC = clk_csr_i/16 */
	plat->has_gmac = 1;
	plat->force_sf_dma_mode = 1;

	plat->mdio_bus_data->phy_reset = NULL;
	plat->mdio_bus_data->phy_mask = 0;

	/* Set default value for multicast hash bins */
	plat->multicast_filter_bins = HASH_TABLE_SIZE;

	/* Set default value for unicast filter entries */
	plat->unicast_filter_entries = 1;

	/* Set the maxmtu to a default of JUMBO_LEN */
	plat->maxmtu = JUMBO_LEN;

	/* Set default number of RX and TX queues to use */
	plat->tx_queues_to_use = 1;
	plat->rx_queues_to_use = 1;

	/* Disable Priority config by default */
	plat->tx_queues_cfg[0].use_prio = false;
	plat->rx_queues_cfg[0].use_prio = false;

	/* Disable RX queues routing by default */
	plat->rx_queues_cfg[0].pkt_route = 0x0;
}

static int stmmac_default_data(struct pci_dev *pdev,
			       struct plat_stmmacenet_data *plat)
{
	/* Set common default data first */
	common_default_data(plat);

	plat->bus_id = 1;
	plat->phy_addr = 0;
	plat->interface = PHY_INTERFACE_MODE_GMII;

	plat->dma_cfg->pbl = 32;
	plat->dma_cfg->pblx8 = true;
	/* TODO: AXI */

	return 0;
}

static const struct stmmac_pci_info stmmac_pci_info = {
	.setup = stmmac_default_data,
};

static const struct stmmac_pci_func_data galileo_stmmac_func_data[] = {
	{
		.func = 6,
		.phy_addr = 1,
	},
};

static const struct stmmac_pci_dmi_data galileo_stmmac_dmi_data = {
	.func = galileo_stmmac_func_data,
	.nfuncs = ARRAY_SIZE(galileo_stmmac_func_data),
};

static const struct stmmac_pci_func_data iot2040_stmmac_func_data[] = {
	{
		.func = 6,
		.phy_addr = 1,
	},
	{
		.func = 7,
		.phy_addr = 1,
	},
};

static const struct stmmac_pci_dmi_data iot2040_stmmac_dmi_data = {
	.func = iot2040_stmmac_func_data,
	.nfuncs = ARRAY_SIZE(iot2040_stmmac_func_data),
};

static const struct dmi_system_id quark_pci_dmi[] = {
	{
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "Galileo"),
		},
		.driver_data = (void *)&galileo_stmmac_dmi_data,
	},
	{
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "GalileoGen2"),
		},
		.driver_data = (void *)&galileo_stmmac_dmi_data,
	},
	/*
	 * There are 2 types of SIMATIC IOT2000: IOT20202 and IOT2040.
	 * The asset tag "6ES7647-0AA00-0YA2" is only for IOT2020 which
	 * has only one pci network device while other asset tags are
	 * for IOT2040 which has two.
	 */
	{
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "SIMATIC IOT2000"),
			DMI_EXACT_MATCH(DMI_BOARD_ASSET_TAG,
					"6ES7647-0AA00-0YA2"),
		},
		.driver_data = (void *)&galileo_stmmac_dmi_data,
	},
	{
		.matches = {
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "SIMATIC IOT2000"),
		},
		.driver_data = (void *)&iot2040_stmmac_dmi_data,
	},
	{}
};

static int quark_default_data(struct pci_dev *pdev,
			      struct plat_stmmacenet_data *plat)
{
	int ret;

	/* Set common default data first */
	common_default_data(plat);

	/*
	 * Refuse to load the driver and register net device if MAC controller
	 * does not connect to any PHY interface.
	 */
	ret = stmmac_pci_find_phy_addr(pdev, quark_pci_dmi);
	if (ret < 0) {
		/* Return error to the caller on DMI enabled boards. */
		if (dmi_get_system_info(DMI_BOARD_NAME))
			return ret;

		/*
		 * Galileo boards with old firmware don't support DMI. We always
		 * use 1 here as PHY address, so at least the first found MAC
		 * controller would be probed.
		 */
		ret = 1;
	}

	plat->bus_id = PCI_DEVID(pdev->bus->number, pdev->devfn);
	plat->phy_addr = ret;
	plat->interface = PHY_INTERFACE_MODE_RMII;

	plat->dma_cfg->pbl = 16;
	plat->dma_cfg->pblx8 = true;
	plat->dma_cfg->fixed_burst = 1;
	/* AXI (TODO) */

	return 0;
}

static const struct stmmac_pci_info quark_pci_info = {
	.setup = quark_default_data,
};

static void loongson_alloc_msi_irq(struct pci_dev *pdev,
			struct plat_stmmacenet_data *plat, struct stmmac_resources *res)
{
	u32 version;
	int ch_cnt, vecs, i;

	version = readl(res->addr + GMAC_VERSION) & 0xff;

	switch (version) {
	case DWLGMAC_CORE_1_00:
		ch_cnt = 8;
		break;

	case DWMAC_CORE_3_50:
	case DWMAC_CORE_3_70:
	default:
		res->msi_vecs = 0;
		return;
	}

	plat->rx_queues_to_use = ch_cnt;
	plat->tx_queues_to_use = ch_cnt;

	pci_disable_msi(pdev);

	vecs = roundup_pow_of_two(ch_cnt * 2 + 1);
	if (pci_alloc_irq_vectors(pdev, vecs, vecs, PCI_IRQ_MSI) < 0) {
		dev_info(&pdev->dev,
			"MSI enable failed, Fallback to line interrupt\n");
		res->msi_vecs = 0;
		res->irq = pdev->irq;
		res->wol_irq = res->irq;
		return;
	}

	res->msi_vecs = vecs;
	res->irq = pci_irq_vector(pdev, 0);
	res->wol_irq = res->irq;

	/*
	 * INT NAME | MAC | CH7 rx | CH7 tx | ... | CH0 rx | CH0 tx |
	 * --------- ----- -------- --------  ...  -------- --------
	 * IRQ NUM  |  0  |   1    |   2    | ... |   15   |   16   |
	 */
	for (i = 0; i < ch_cnt; i++) {
		res->rx_irq[ch_cnt - 1 - i] = pci_irq_vector(pdev, 1 + i * 2);
		res->tx_irq[ch_cnt - 1 - i] = pci_irq_vector(pdev, 2 + i * 2);
	}
}

static int loongson_default_data(struct pci_dev *pdev,
				struct plat_stmmacenet_data *plat)
{
	struct device_node *np;
	/* Set common default data first */
	common_default_data(plat);

	plat->bus_id = (pci_domain_nr(pdev->bus) << 16) | PCI_DEVID(pdev->bus->number, pdev->devfn);
	plat->phy_addr = -1;
	plat->interface = PHY_INTERFACE_MODE_RGMII_ID;

	plat->dma_cfg->pbl = 32;
	plat->dma_cfg->pblx8 = true;
	/*ls2h,ls2k,ls7a mcast filter register is 256bit.*/
	plat->multicast_filter_bins = 256;
	/* AXI (TODO) */
	plat->clk_ref_rate = 125000000;
	plat->clk_ptp_rate = 125000000;
	plat->has_lgmac = 1;
	np = dev_of_node(&pdev->dev);
	if (np)
		plat->phy_node = of_parse_phandle(np, "phy-handle", 0);
	if (np && plat->phy_node) {
#ifdef CONFIG_FIXED_PHY
		if (plat->phy_node == np) {
			struct phy_device *phydev;
			of_property_read_u32(np, "speed",
					(u32 *)&fixed_phy_status.speed);
			phydev =
				fixed_phy_register(PHY_POLL, &fixed_phy_status, -1,
						plat->phy_node);
			if (phydev) {
				plat->phy_addr = phydev->mdio.addr;
				fixed_phy_set_link_update(phydev, link_update);
				phydev->is_pseudo_fixed_link = false;
			}
		}
#endif
		/* PHYLINK automatically parses the phy-handle property */
		plat->mdio_bus_data = NULL;
	}

	return 0;
}

static struct stmmac_pci_info loongson_pci_info = {
	.setup = loongson_default_data,
};

static void ls_gnet_fix_speed(void *priv, unsigned int speed)
{
	struct net_device *ndev = (struct net_device *)(*(unsigned long *)priv);
	struct stmmac_priv *ptr = netdev_priv(ndev);

	if (speed == SPEED_1000) {
		if (readl(ptr->ioaddr + MAC_CTRL_REG) & (1 << 15) /* PS */) {
			/* reset phy */
			phy_set_bits(ndev->phydev, 0 /*MII_BMCR*/, 0x200 /*BMCR_ANRESTART*/);
		}
	}
}

static int loongson_gnet_data(struct pci_dev *pdev,
				struct plat_stmmacenet_data *plat)
{
	/* Set common default data first */
	common_default_data(plat);

	plat->bus_id = (pci_domain_nr(pdev->bus) << 16) | PCI_DEVID(pdev->bus->number, pdev->devfn);

	/* 7A2000 GNET internal PHY address is fixed */
	plat->mdio_bus_data->phy_mask = 0xfffffffb;
	plat->phy_addr = 2;

	/* 7A2000 new gmac should change PHY_INTERFACE_MODE_GMII */
	plat->interface = PHY_INTERFACE_MODE_GMII;

	plat->dma_cfg->pbl = 32;
	plat->dma_cfg->pblx8 = true;
	/* 7A2000 mcast filter register is 256bit. */
	plat->multicast_filter_bins = 256;

	/* gnet 1000M speed need workaround */
	plat->fix_mac_speed = ls_gnet_fix_speed;
	/* used to get netdev pointer address */
	plat->bsp_priv = &(pdev->dev.driver_data);
	plat->clk_ref_rate = 125000000;
	plat->clk_ptp_rate = 125000000;

	plat->disable_half_duplex = true;
	plat->has_lgmac = 1;

	return 0;
}

static struct stmmac_pci_info loongson_gnet_pci_info = {
	.setup = loongson_gnet_data,
};

/**
 * stmmac_pci_probe
 *
 * @pdev: pci device pointer
 * @id: pointer to table of device id/id's.
 *
 * Description: This probing function gets called for all PCI devices which
 * match the ID table and are not "owned" by other driver yet. This function
 * gets passed a "struct pci_dev *" for each device whose entry in the ID table
 * matches the device. The probe functions returns zero when the driver choose
 * to take "ownership" of the device or an error code(-ve no) otherwise.
 */
static int stmmac_pci_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	struct stmmac_pci_info *info = (struct stmmac_pci_info *)id->driver_data;
	struct plat_stmmacenet_data *plat;
	struct stmmac_resources res = {0};
	int i;
	int ret;

	plat = devm_kzalloc(&pdev->dev, sizeof(*plat), GFP_KERNEL);
	if (!plat)
		return -ENOMEM;

	plat->mdio_bus_data = devm_kzalloc(&pdev->dev,
					   sizeof(*plat->mdio_bus_data),
					   GFP_KERNEL);
	if (!plat->mdio_bus_data)
		return -ENOMEM;

	plat->dma_cfg = devm_kzalloc(&pdev->dev, sizeof(*plat->dma_cfg),
				     GFP_KERNEL);
	if (!plat->dma_cfg)
		return -ENOMEM;

	/* Enable pci device */
	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "%s: ERROR: failed to enable device\n",
			__func__);
		return ret;
	}

	/* Get the base address of device */
	for (i = 0; i <= PCI_STD_RESOURCE_END; i++) {
		if (pci_resource_len(pdev, i) == 0)
			continue;
		ret = pcim_iomap_regions(pdev, BIT(i), pci_name(pdev));
		if (ret)
			return ret;
		break;
	}

	pci_set_master(pdev);

	ret = info->setup(pdev, plat);
	if (ret)
		return ret;

	pci_enable_msi(pdev);

	memset(&res, 0, sizeof(res));
	res.addr = pcim_iomap_table(pdev)[i];
	res.wol_irq = pdev->irq;
	res.irq = pdev->irq;

	if (plat->has_lgmac)
		loongson_alloc_msi_irq(pdev, plat, &res);

	return stmmac_dvr_probe(&pdev->dev, plat, &res);
}

/**
 * stmmac_pci_remove
 *
 * @pdev: platform device pointer
 * Description: this function calls the main to free the net resources
 * and releases the PCI resources.
 */
static void stmmac_pci_remove(struct pci_dev *pdev)
{
	int i;

	stmmac_dvr_remove(&pdev->dev);

	for (i = 0; i <= PCI_STD_RESOURCE_END; i++) {
		if (pci_resource_len(pdev, i) == 0)
			continue;
		pcim_iounmap_regions(pdev, BIT(i));
		break;
	}

	pci_free_irq_vectors(pdev);
	pci_disable_device(pdev);
}

static int __maybe_unused stmmac_pci_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	int ret;

	ret = stmmac_suspend(dev);
	if (ret)
		return ret;

	ret = pci_save_state(pdev);
	if (ret)
		return ret;

	pci_disable_device(pdev);
	pci_wake_from_d3(pdev, true);
	return 0;
}

static int __maybe_unused stmmac_pci_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	int ret;

	pci_restore_state(pdev);
	pci_set_power_state(pdev, PCI_D0);

	ret = pci_enable_device(pdev);
	if (ret)
		return ret;

	pci_set_master(pdev);

	return stmmac_resume(dev);
}

static SIMPLE_DEV_PM_OPS(stmmac_pm_ops, stmmac_pci_suspend, stmmac_pci_resume);

/* synthetic ID, no official vendor */
#define PCI_VENDOR_ID_STMMAC 0x700

#define STMMAC_QUARK_ID  0x0937
#define STMMAC_DEVICE_ID 0x1108

#define STMMAC_DEVICE(vendor_id, dev_id, info)	{	\
	PCI_VDEVICE(vendor_id, dev_id),			\
	.driver_data = (kernel_ulong_t)&info		\
	}

static const struct pci_device_id stmmac_id_table[] = {
	STMMAC_DEVICE(STMMAC, STMMAC_DEVICE_ID, stmmac_pci_info),
	STMMAC_DEVICE(STMICRO, PCI_DEVICE_ID_STMICRO_MAC, stmmac_pci_info),
	STMMAC_DEVICE(INTEL, STMMAC_QUARK_ID, quark_pci_info),
	STMMAC_DEVICE(LOONGSON, PCI_DEVICE_ID_LOONGSON_GMAC, loongson_pci_info),
	STMMAC_DEVICE(LOONGSON, PCI_DEVICE_ID_LOONGSON_GNET, loongson_gnet_pci_info),
	{}
};

MODULE_DEVICE_TABLE(pci, stmmac_id_table);

static struct pci_driver stmmac_pci_driver = {
	.name = STMMAC_RESOURCE_NAME,
	.id_table = stmmac_id_table,
	.probe = stmmac_pci_probe,
	.remove = stmmac_pci_remove,
	.driver         = {
		.pm     = &stmmac_pm_ops,
	},
};

module_pci_driver(stmmac_pci_driver);

MODULE_DESCRIPTION("STMMAC 10/100/1000 Ethernet PCI driver");
MODULE_AUTHOR("Rayagond Kokatanur <rayagond.kokatanur@vayavyalabs.com>");
MODULE_AUTHOR("Giuseppe Cavallaro <peppe.cavallaro@st.com>");
MODULE_LICENSE("GPL");
