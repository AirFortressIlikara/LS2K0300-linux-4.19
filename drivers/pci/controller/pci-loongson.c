// SPDX-License-Identifier: GPL-2.0
/*
 * Loongson PCI Host Controller Driver
 *
 * Copyright (C) 2020 Jiaxun Yang <jiaxun.yang@flygoat.com>
 * 			Jianmin Lv <lvjianmin@loongson.cn>
 */

#include <linux/of_device.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/pci-acpi.h>
#include <linux/pci-ecam.h>

#include "../pci.h"

#define FLAG_CFG0	BIT(0)
#define FLAG_CFG1	BIT(1)
#define FLAG_DEV_FIX	BIT(2)
#define FLAG_LS2K1000	BIT(3)

struct pcie_controller_data {
	u64 flags;
	u32 cfg0_type1_offset;
	u32 cfg1_type1_offset;
	struct pci_ops *ops;
};

struct loongson_pci {
	void __iomem *cfg0_base;
	void __iomem *cfg1_base;
	struct platform_device *pdev;
	struct pcie_controller_data *data;
};

static struct loongson_pci *pci_bus_to_loongson_pci(struct pci_bus *bus)
{
	struct pci_config_window *cfg;

	if (acpi_disabled)
		return (struct loongson_pci *)(bus->sysdata);

	cfg = bus->sysdata;
	return (struct loongson_pci *)(cfg->priv);
}
static void __iomem *cfg1_map(struct loongson_pci *priv, int bus,
				unsigned int devfn, int where, int cfg_type1_offset)
{
	unsigned long addroff = 0x0;

	if (bus != 0)
		addroff |= BIT(cfg_type1_offset); /* Type 1 Access */
	addroff |= (where & 0xff) | ((where & 0xf00) << 16);
	addroff |= (bus << 16) | (devfn << 8);
	return priv->cfg1_base + addroff;
}

static void __iomem *cfg0_map(struct loongson_pci *priv, int bus,
				unsigned int devfn, int where, int cfg_type1_offset)
{
	unsigned long addroff = 0x0;

	if (bus != 0)
		addroff |= BIT(cfg_type1_offset); /* Type 1 Access */
	addroff |= (bus << 16) | (devfn << 8) | where;
	return priv->cfg0_base + addroff;
}

static void __iomem *pci_loongson_map_bus(struct pci_bus *bus, unsigned int devfn,
			       int where)
{
	unsigned char busnum = bus->number;
	struct loongson_pci *priv = pci_bus_to_loongson_pci(bus);
	int device = PCI_SLOT(devfn);
	int function = PCI_FUNC(devfn);

	if (pci_is_root_bus(bus))
		busnum = 0;
	/*
	 * Do not read more than one device on the bus other than
	 * the host bus. For our hardware the root bus is always bus 0.
	 */
	if (priv->data->flags & FLAG_DEV_FIX && busnum != 0
			&& device > 0 && bus->self != NULL)
		return NULL;

	if (priv->data->flags & FLAG_LS2K1000) {
		/* ls2k1000 only scan pcie */
		if ((busnum == 0) && (device < 9 || device > 14))
			return NULL;
	} else {
		if ((busnum == 0) && (device >= 9 && device <= 20 && function == 1))
			return NULL;
	}

	if (priv->cfg1_base && where < PCI_CFG_SPACE_EXP_SIZE) {
		return cfg1_map(priv, busnum, devfn, where, priv->data->cfg1_type1_offset);
	} else if (priv->cfg0_base && where < PCI_CFG_SPACE_SIZE) {
		return cfg0_map(priv, busnum, devfn, where, priv->data->cfg0_type1_offset);
	}
	return NULL;
}

static int pci_loongson_config_read(struct pci_bus *bus, unsigned int devfn,
			    int where, int size, u32 *val)
{
	void __iomem *addr;

	addr = bus->ops->map_bus(bus, devfn, where);
	if (!addr) {
		*val = ~0;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (size == 1)
		*val = readb(addr);
	else if (size == 2)
		*val = readw(addr);
	else
		*val = readl(addr);
	/*
	 * fix some pcie card not scanning properly when bus number is
	 * inconsistent during firmware and kernel scan phases.
	 */
	if (*val == 0x0 && where == PCI_VENDOR_ID) {
		writel(*val, addr);
		*val = readl(addr);
	}


	return PCIBIOS_SUCCESSFUL;
}

#if defined(CONFIG_OF)
static int loongson_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	int irq;
	u8 val;

	irq = of_irq_parse_and_map_pci(dev, slot, pin);
	if (irq > 0)
		return irq;

	/* Care i8259 legacy systems */
	pci_read_config_byte(dev, PCI_INTERRUPT_LINE, &val);
	/* i8259 only have 15 IRQs */
	if (val > 15)
		return 0;

	return val;
}

/* H/w only accept 32-bit PCI operations */
static struct pci_ops loongson_pci_ops32 = {
	.map_bus = pci_loongson_map_bus,
	.read	= pci_generic_config_read32,
	.write	= pci_generic_config_write32,
};

static struct pci_ops loongson_pci_ops = {
	.map_bus = pci_loongson_map_bus,
	.read	= pci_loongson_config_read,
	.write	= pci_generic_config_write,
};

static const struct pcie_controller_data ls2k1000_pcie_data = {
	.flags = FLAG_CFG1 | FLAG_LS2K1000,
	.cfg1_type1_offset = 28,
	.ops = &loongson_pci_ops,
};

static const struct pcie_controller_data ls2k_pcie_data = {
	.flags = FLAG_CFG1,
	.cfg1_type1_offset = 28,
	.ops = &loongson_pci_ops,
};

static const struct pcie_controller_data ls7a_pcie_data = {
	.flags = FLAG_CFG1 | FLAG_DEV_FIX,
	.cfg1_type1_offset = 28,
	.ops = &loongson_pci_ops,
};

static const struct pcie_controller_data rs780e_pcie_data = {
	.flags = FLAG_CFG0 | FLAG_DEV_FIX,
	.cfg0_type1_offset = 24,
	.ops = &loongson_pci_ops32,
};

static const struct of_device_id loongson_pci_of_match[] = {
	{ .compatible = "loongson,ls2k1000-pci",
		.data = (void *)&ls2k1000_pcie_data, },
	{ .compatible = "loongson,ls2k-pci",
		.data = (void *)&ls2k_pcie_data, },
	{ .compatible = "loongson,ls7a-pci",
		.data = (void *)&ls7a_pcie_data, },
	{ .compatible = "loongson,rs780e-pci",
		.data = (void *)&rs780e_pcie_data, },
	{}
};

static int loongson_pci_probe(struct platform_device *pdev)
{
	struct loongson_pci *priv;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct pci_host_bridge *bridge;
	struct resource *regs;
	unsigned int num = 0;

	LIST_HEAD(pci_res);
	if (!node)
		return -ENODEV;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*priv));
	if (!bridge)
		return -ENODEV;

	priv = pci_host_bridge_priv(bridge);
	priv->pdev = pdev;
	priv->data = (struct pcie_controller_data *)of_device_get_match_data(dev);

	if (priv->data->flags & FLAG_CFG0) {
		regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!regs)
			dev_info(dev, "missing mem resources for cfg0\n");
		else {
			priv->cfg0_base = devm_pci_remap_cfg_resource(dev, regs);
			if (IS_ERR(priv->cfg0_base))
				priv->cfg0_base = NULL;
		}
	}

	if (priv->data->flags & FLAG_CFG1) {
		if (priv->cfg0_base)
			num = 1;
		regs = platform_get_resource(pdev, IORESOURCE_MEM, num);
		if (!regs)
			dev_info(dev, "missing mem resource for cfg1\n");
		else {
			priv->cfg1_base = devm_pci_remap_cfg_resource(dev, regs);
			if (IS_ERR(priv->cfg1_base))
				priv->cfg1_base = NULL;
		}
	}

	if (pci_parse_request_of_pci_ranges(&pdev->dev, &pci_res, NULL))
		return -ENODEV;

	list_splice_init(&pci_res, &bridge->windows);
	bridge->dev.parent = dev;
	bridge->dev.of_node = node;
	bridge->sysdata = priv;
	bridge->ops = priv->data->ops;
	bridge->map_irq = loongson_map_irq;

	return pci_host_probe(bridge);
}

static struct platform_driver loongson_pci_driver = {
	.driver = {
		.name = "loongson-pci",
		.of_match_table = loongson_pci_of_match,
	},
	.probe = loongson_pci_probe,
};
builtin_platform_driver(loongson_pci_driver);
#endif

#if defined(CONFIG_ACPI) && defined(CONFIG_PCI_QUIRKS)
static int loongson_pci_ecam_init(struct pci_config_window *cfg)
{
	struct device *dev = cfg->parent;
	struct loongson_pci *priv;
	struct pcie_controller_data *data;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->flags = FLAG_CFG1,
	data->cfg1_type1_offset = 28;
	priv->data = data;
	priv->cfg1_base = cfg->win - (cfg->busr.start << 16);

	cfg->priv = priv;
	return 0;
}

struct pci_ecam_ops loongson_pci_ecam_ops = {
	.bus_shift	= 16,
	.init		= loongson_pci_ecam_init,
	.pci_ops	= {
		.map_bus	= pci_loongson_map_bus,
		.read		= pci_loongson_config_read,
		.write		= pci_generic_config_write,
	}
};
#endif
