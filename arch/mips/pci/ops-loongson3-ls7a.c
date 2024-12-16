#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/suspend.h>

#include <asm/mips-boards/bonito64.h>

#include <loongson.h>

#define PCI_ACCESS_READ  0
#define PCI_ACCESS_WRITE 1

#define PCI_byte_BAD 0
#define PCI_word_BAD (pos & 1)
#define PCI_dword_BAD (pos & 3)
#define PCI_FIND_CAP_TTL        48

#define PCI_OP_READ(size,type,len) \
static int pci_bus_read_config_##size##_nolock \
	(struct pci_bus *bus, unsigned int devfn, int pos, type *value) \
{                                                                       \
	int res;                                                        \
	u32 data = 0;                                                   \
	if (PCI_##size##_BAD) return PCIBIOS_BAD_REGISTER_NUMBER;       \
	res = bus->ops->read(bus, devfn, pos, len, &data);              \
	*value = (type)data;                                            \
	return res;                                                     \
}

PCI_OP_READ(byte, u8, 1)
PCI_OP_READ(word, u16, 2)
PCI_OP_READ(dword, u32, 4)

static int __pci_find_next_cap_ttl_nolock(struct pci_bus *bus, unsigned int devfn,
					  u8 pos, int cap, int *ttl)
{
	u8 id;
	u16 ent;

	pci_bus_read_config_byte_nolock(bus, devfn, pos, &pos);

	while ((*ttl)--) {
		if (pos < 0x40)
			break;
		pos &= ~3;
		pci_bus_read_config_word_nolock(bus, devfn, pos, &ent);

		id = ent & 0xff;
		if (id == 0xff)
			break;
		if (id == cap)
			return pos;
		pos = (ent >> 8);
	}
	return 0;
}

static int __pci_find_next_cap_nolock(struct pci_bus *bus, unsigned int devfn,
					u8 pos, int cap)
{
	int ttl = PCI_FIND_CAP_TTL;

	return __pci_find_next_cap_ttl_nolock(bus, devfn, pos, cap, &ttl);
}

static int __pci_bus_find_cap_start_nolock(struct pci_bus *bus,
					   unsigned int devfn, u8 hdr_type)
{
	u16 status;

	pci_bus_read_config_word_nolock(bus, devfn, PCI_STATUS, &status);
	if (!(status & PCI_STATUS_CAP_LIST))
		return 0;

	switch (hdr_type) {
	case PCI_HEADER_TYPE_NORMAL:
	case PCI_HEADER_TYPE_BRIDGE:
		return PCI_CAPABILITY_LIST;
	case PCI_HEADER_TYPE_CARDBUS:
		return PCI_CB_CAPABILITY_LIST;
	default:
		return 0;
	}

	return 0;
}

static int pci_bus_find_capability_nolock(struct pci_bus *bus, unsigned int devfn, int cap)
{
	int pos;
	u8 hdr_type;

	pci_bus_read_config_byte_nolock(bus, devfn, PCI_HEADER_TYPE, &hdr_type);

	pos = __pci_bus_find_cap_start_nolock(bus, devfn, hdr_type & 0x7f);
	if (pos)
		pos = __pci_find_next_cap_nolock(bus, devfn, pos, cap);

	return pos;
}

static int ls7a_pci_config_access(unsigned char access_type,
		struct pci_bus *bus, unsigned int devfn,
		int where, u32 *data)
{
	u_int64_t addr;
	void *addrp;
	unsigned char busnum = bus->number;
	unsigned int val;
	int device = PCI_SLOT(devfn);
	int function = PCI_FUNC(devfn);
	int reg = where & ~3;
	int pos;

	if (busnum == 0) {
		/** Filter out non-supported devices.
		 *  device 2: misc, device 21: confbus
		 */
		if (device > 23 || (device >= 9 && device <= 20 && function == 1))
			return PCIBIOS_DEVICE_NOT_FOUND;
		if (reg < 0x100) {
			addr = (device << 11) | (function << 8) | reg;
			addrp = (void *)TO_UNCAC(HT1LO_PCICFG_BASE | addr);
		} else {
			addr = (device << 11) | (function << 8) | (reg & 0xff) | ((reg & 0xf00) << 16);
			addrp = (void *)(TO_UNCAC(HT1LO_EXT_PCICFG_BASE) | addr);
		}
	} else {
		if (reg < 0x100) {
			addr = (busnum << 16) | (device << 11) | (function << 8) | reg;
			addrp = (void *)TO_UNCAC(HT1LO_PCICFG_BASE_TP1 | addr);
		} else {
			addr = (busnum << 16) | (device << 11) | (function << 8) | (reg & 0xff) | ((reg & 0xf00) << 16);
			addrp = (void *)(TO_UNCAC(HT1LO_EXT_PCICFG_BASE_TP1) | addr);
		}
	}

	if (access_type == PCI_ACCESS_WRITE) {
		pos = pci_bus_find_capability_nolock(bus, devfn, PCI_CAP_ID_EXP);

#ifdef CONFIG_PM_SLEEP
		/* Devctl should be restored during resume. */
		if (pm_suspend_target_state == PM_SUSPEND_ON &&
				pos != 0 && (pos + PCI_EXP_DEVCTL) == reg)
#else
		if (pos != 0 && (pos + PCI_EXP_DEVCTL) == reg)
#endif
		{
			/* need pmon/uefi configure mrrs to pcie slot max mrrs capability */
			val = *(volatile unsigned int *)addrp;
			if ((*data & PCI_EXP_DEVCTL_READRQ) > (val & PCI_EXP_DEVCTL_READRQ)) {
				printk(KERN_ERR "MAX READ REQUEST SIZE shuould not greater than the slot max capability");
				*data = *data & ~PCI_EXP_DEVCTL_READRQ;
				*data = *data | (val & PCI_EXP_DEVCTL_READRQ);
			}
		}

		*(volatile unsigned int *)addrp = cpu_to_le32(*data);
	} else {
		*data = le32_to_cpu(*(volatile unsigned int *)addrp);
		if (busnum == 0 && reg == PCI_CLASS_REVISION && *data == 0x06000001)
			*data = (PCI_CLASS_BRIDGE_PCI << 16) | (*data & 0xffff);

		/*
		 * fix some pcie card not scanning properly when bus number is
		 * inconsistent during firmware and kernel scan phases.
		 */
		if (*data == 0x0 && where == PCI_VENDOR_ID) {
			*(volatile unsigned int *)addrp = cpu_to_le32(*data);
			*data = le32_to_cpu(*(volatile unsigned int *)addrp);
		}

		if (*data == 0xffffffff && (where == PCI_VENDOR_ID)) {
			*data = -1;
			return PCIBIOS_DEVICE_NOT_FOUND;
		}
	}
	return PCIBIOS_SUCCESSFUL;
}

static int ls7a_pci_pcibios_read(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 * val)
{
	u32 data = 0;
	int ret = ls7a_pci_config_access(PCI_ACCESS_READ,
			bus, devfn, where, &data);

	if (ret != PCIBIOS_SUCCESSFUL)
		return ret;

	if (size == 1)
		*val = (data >> ((where & 3) << 3)) & 0xff;
	else if (size == 2)
		*val = (data >> ((where & 3) << 3)) & 0xffff;
	else
		*val = data;

	return PCIBIOS_SUCCESSFUL;
}

static int ls7a_pci_pcibios_write(struct pci_bus *bus, unsigned int devfn,
				  int where, int size, u32 val)
{
	u32 data = 0;
	int ret;

	if (size == 4)
		data = val;
	else {
		ret = ls7a_pci_config_access(PCI_ACCESS_READ,
				bus, devfn, where, &data);
		if (ret != PCIBIOS_SUCCESSFUL)
			return ret;

		if (size == 1)
			data = (data & ~(0xff << ((where & 3) << 3))) |
			    (val << ((where & 3) << 3));
		else if (size == 2)
			data = (data & ~(0xffff << ((where & 3) << 3))) |
			    (val << ((where & 3) << 3));
	}

	ret = ls7a_pci_config_access(PCI_ACCESS_WRITE,
			bus, devfn, where, &data);
	if (ret != PCIBIOS_SUCCESSFUL)
		return ret;

	return PCIBIOS_SUCCESSFUL;
}

struct pci_ops ls7a_pci_ops = {
	.read = ls7a_pci_pcibios_read,
	.write = ls7a_pci_pcibios_write
};
