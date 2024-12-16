// SPDX-License-Identifier: GPL-2.0
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_irq.h>

#define LS2K_PCICFG_TYPE0_BASE	0xfe00000000
#define LS2K_PCICFG_TYPE1_BASE	0xfe10000000

extern int devm_of_pci_get_host_bridge_resources(struct device *dev,
			unsigned char busno, unsigned char bus_max,
			struct list_head *resources, resource_size_t *io_base);

struct ls2k_pcie_port {
	struct resource *busn;
	resource_size_t iobase;
	int irq;
};

static inline u32 pci_cfg_read_32bit(struct pci_bus *bus, unsigned int devfn,
	int where)
{
	u32 *cfgaddr;
	unsigned long addr;
	u32 data;

	addr = (bus->number << 16) + (PCI_SLOT(devfn) << 11) + (PCI_FUNC(devfn) << 8);

	where &= ~3; /* word align */
	where = (where & 0xff) + ((where >> 8) << 24);

	if (bus->number == 0)
		cfgaddr = (void *)TO_UNCAC(LS2K_PCICFG_TYPE0_BASE + addr + where);
	else
		cfgaddr = (void *)TO_UNCAC(LS2K_PCICFG_TYPE1_BASE + addr + where);

	data = readl(cfgaddr);
	return data;
}

static inline void pci_cfg_write_32bit(struct pci_bus *bus, unsigned int devfn,
	int where, u32 data)
{
	u32 *cfgaddr;
	unsigned long addr;

	addr = (bus->number << 16) + (PCI_SLOT(devfn) << 11) + (PCI_FUNC(devfn) << 8);

	where &= ~3; /* word align */
	where = (where & 0xff) + ((where >> 8) << 24);

	if (bus->number == 0)
		cfgaddr = (void *)TO_UNCAC(LS2K_PCICFG_TYPE0_BASE + addr + where);
	else
		cfgaddr = (void *)TO_UNCAC(LS2K_PCICFG_TYPE1_BASE + addr + where);

	writel(data, cfgaddr);
}

static int ls2k_pcibios_read(struct pci_bus *bus, unsigned int devfn,
	int where, int size, u32 *val)
{
	u32 data = 0xffffffff;

	/*
	 * we created a virtual pci host bridge on each pcie port,
	 * the real host bridge not scanned by us(scanned in BIOS),
	 * so only scan device 0 on virtual root bus
	 */
	if (!bus->parent && PCI_SLOT(devfn) > 0)
		return PCIBIOS_DEVICE_NOT_FOUND;
	/* only support 4K configration space */
	if (where > 4095)
		return PCIBIOS_BAD_REGISTER_NUMBER;
	if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	data = pci_cfg_read_32bit(bus, devfn, where);

	if (bus->number == 0 && where == PCI_CLASS_REVISION && (data>>16) == PCI_CLASS_PROCESSOR_MIPS)
		data = (PCI_CLASS_BRIDGE_PCI<<16) | (data & 0xffff);

	data >>= (where & 3)*8;
	if (size == 1)
		*val = data & 0xff;
	else if (size == 2)
		*val = data & 0xffff;
	else
		*val = data;

	return PCIBIOS_SUCCESSFUL;
}

static int ls2k_pcibios_write(struct pci_bus *bus, unsigned int devfn,
		int where, int size, u32 val)
{
	u32 data = 0xffffffff;

	/*
	 * we created a virtual pci host bridge on each pcie port,
	 * the real host bridge not scanned by us(scanned in BIOS),
	 * so only scan device 0 on virtual root bus
	 */
	if (!bus->parent && PCI_SLOT(devfn) > 0)
		return PCIBIOS_DEVICE_NOT_FOUND;
	/* only support 4K configration space */
	if (where > 4095)
		return PCIBIOS_BAD_REGISTER_NUMBER;
	if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	data = pci_cfg_read_32bit(bus, devfn, where);

	if (size == 1)
		data = (data & ~(0xff << ((where & 3) << 3))) |
			(val << ((where & 3) << 3));
	else if (size == 2)
		data = (data & ~(0xffff << ((where & 3) << 3))) |
			(val << ((where & 3) << 3));
	else
		data = val;

	pci_cfg_write_32bit(bus, devfn, where, data);

	return PCIBIOS_SUCCESSFUL;
}

int pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	struct pci_host_bridge *bridge = pci_find_host_bridge(dev->bus);
	struct ls2k_pcie_port *pp = pci_host_bridge_priv(bridge);

	return pp->irq;
}

/* Do platform specific device initialization at pci_enable_device() time */
int pcibios_plat_dev_init(struct pci_dev *dev)
{

	int pos;
	u16 max_payload_spt, cur_payload_spt, control;

	/**
	 * fixup settings of MPS & MRRS during fixing irq
	 * check whether MPSSPT is smaller than parents',
	 * keep the smaller MPSSPT in the child's register
	 */
	if (!(dev->bus->parent)) {
		pos = pci_find_capability(dev, PCI_CAP_ID_EXP);
		if (!pos)
			return 0;
		pci_read_config_word(dev, pos + PCI_EXP_DEVCAP,
				     &max_payload_spt);
		max_payload_spt &= PCI_EXP_DEVCAP_PAYLOAD;
	} else {
		pos = pci_find_capability(dev->bus->self, PCI_CAP_ID_EXP);
		if (!pos)
			return 0;
		pci_read_config_word(dev->bus->self, pos + PCI_EXP_DEVCAP,
				     &max_payload_spt);
		max_payload_spt &= PCI_EXP_DEVCAP_PAYLOAD;

		pos = pci_find_capability(dev, PCI_CAP_ID_EXP);
		pci_read_config_word(dev, pos + PCI_EXP_DEVCAP,
				     &cur_payload_spt);
		cur_payload_spt &= PCI_EXP_DEVCAP_PAYLOAD;

		if (max_payload_spt > cur_payload_spt)
			max_payload_spt = cur_payload_spt;
	}

	if (max_payload_spt > 1)
		max_payload_spt = 1;

	pci_read_config_word(dev, pos + PCI_EXP_DEVCTL, &control);
	control &= (~PCI_EXP_DEVCTL_PAYLOAD & ~PCI_EXP_DEVCTL_READRQ);
	control |= ((max_payload_spt << 5) | (max_payload_spt << 12));
	pci_write_config_word(dev, pos + PCI_EXP_DEVCTL, control);
	pr_info("pci %s: set Max_Payload_Size & Max_Read_Request_Size to %03x\n",
	       pci_name(dev), max_payload_spt);

	return 0;
}

unsigned long pci_address_to_pio(phys_addr_t address)
{
	return address & IO_SPACE_LIMIT;
}

struct pci_ops ls2k_pcie_ops = {
	.read = ls2k_pcibios_read,
	.write = ls2k_pcibios_write,
};

int ls2k_pcie_probe(struct platform_device *pdev)
{
	struct pci_host_bridge *bridge;
	struct device *dev = &pdev->dev;
	struct pci_bus *child;
	struct ls2k_pcie_port *pp;
	struct resource_entry *win, *tmp;
	struct pci_controller *controller;

	dev_dbg(dev, "%s : start\n", __func__);

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(struct ls2k_pcie_port));
	if (!bridge)
		return -ENOMEM;
	controller = devm_kzalloc(dev, sizeof(struct pci_controller), GFP_KERNEL);
	if (!controller)
		return -ENOMEM;

	pp = pci_host_bridge_priv(bridge);

	/* get io & mem & bus resource */
	devm_of_pci_get_host_bridge_resources(dev, 0, 0xff, &bridge->windows, &pp->iobase);

	/* request io & mem resource */
	devm_request_pci_bus_resources(dev, &bridge->windows);

	resource_list_for_each_entry_safe(win, tmp, &bridge->windows) {
		switch (resource_type(win->res)) {
		case IORESOURCE_BUS:
			pp->busn = win->res;
			break;
		case IORESOURCE_IO:
			controller->io_resource = win->res;
			controller->io_map_base = (unsigned long)ioremap_nocache(pp->iobase &
								(~IO_SPACE_LIMIT),
								resource_size(win->res));
			break;
		case IORESOURCE_MEM:
			controller->mem_resource = win->res;
			break;
		}
	}

	/* get irq resource */
	pp->irq = of_irq_get(dev->of_node, 0);
	if (pp->irq < 0)
		pr_err("%s: get irq error ret = %d\n", __func__, pp->irq);

	controller->pci_ops = &ls2k_pcie_ops;

	bridge->swizzle_irq = pci_common_swizzle;
	bridge->map_irq = pcibios_map_irq;
	bridge->ops = &ls2k_pcie_ops;
	bridge->busnr = pp->busn->start;
	bridge->sysdata = controller;

	/* scan */
	pci_scan_root_bus_bridge(bridge);

	/* size bridge */
	pci_bus_size_bridges(bridge->bus);

	/* assign resource */
	pci_bus_assign_resources(bridge->bus);

	/* configure MPS,MRRS,etc */
	list_for_each_entry(child, &bridge->bus->children, node)
		pcie_bus_configure_settings(child);

	pci_bus_add_devices(bridge->bus);

	dev_dbg(dev, "%s : end\n", __func__);
	return 0;
}

static const struct of_device_id ls2k_pcie_of_match[] = {
	{ .compatible = "loongson,ls-pcie", },
	{},
};

static struct platform_driver ls2k_pcie_driver = {
	.driver = {
		.name = "ls2k-pcie",
		.of_match_table = ls2k_pcie_of_match,
	},
	.probe = ls2k_pcie_probe,
};
builtin_platform_driver(ls2k_pcie_driver);
