#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/msi.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <loongson-pch.h>
#include <asm/mach-loongson64/loongson.h>
#include <irq.h>
#include <boot_param.h>

static bool msix_enable = 0;
core_param(msix, msix_enable, bool, 0664);

struct irq_chip msi_ht_irq_chip;
struct irq_chip msi_ext_irq_chip;
#ifdef CONFIG_LOONGSON_PCH_MSI
extern struct fwnode_handle *msi_irqdomain_handle(int id);
extern int msi_ext_ioi(int id);
static int setup_msidomain_irq(struct pci_dev *pdev, int nvec)
{
	int ret;
	struct irq_domain *msi_domain;
	struct msi_desc *entry;
	msi_domain = irq_find_matching_fwnode(msi_irqdomain_handle(0), DOMAIN_BUS_PCI_MSI);
	if (msi_domain == NULL)
		return -ENOSYS;

	ret = msi_domain_alloc_irqs(msi_domain, &pdev->dev, nvec);
	if (!ret) {
		if (!msi_ext_ioi(0)) {
			spin_lock(&bitmap_lock);
			for_each_pci_msi_entry(entry, pdev) {
				create_ipi_dirq(entry->irq);
			}
			spin_unlock(&bitmap_lock);
		}
	}
	return ret;
}
static int teardown_msidomain_irq(unsigned int irq)
{
	struct irq_domain *msi_domain;
	msi_domain = irq_find_matching_fwnode(msi_irqdomain_handle(0), DOMAIN_BUS_PCI_MSI);
	if (msi_domain)
		irq_domain_free_irqs(irq, 1);

	if (!msi_ext_ioi(0)) {
		spin_lock(&bitmap_lock);
		destroy_ipi_dirq(irq);
		spin_unlock(&bitmap_lock);
	}
	return 0;
}
#else
static int setup_msidomain_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	return -1;
}
static int teardown_msidomain_irq(unsigned int irq)
{
	return -1;
}
#endif
int setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	int irq = irq_alloc_desc_from(loongson_sysconf.msi_base_irq, 0);
	struct msi_msg msg;

	if (irq < 0)
		return irq;

	if (irq >= loongson_sysconf.msi_last_irq) {
		irq_free_desc(irq);
		return -ENOSPC;
	}

	spin_lock(&bitmap_lock);
	create_ipi_dirq(irq);
	spin_unlock(&bitmap_lock);
	irq_set_chip_and_handler(irq, &msi_ht_irq_chip, handle_edge_irq);
	irq_set_msi_desc(irq, desc);

	msg.data = irq - loongson_sysconf.io_base_irq;
	msg.address_hi = loongson_sysconf.msi_address_hi;
	msg.address_lo = loongson_sysconf.msi_address_lo;

	write_msi_msg(irq, &msg);
	return 0;
}

void teardown_msi_irq(unsigned int irq)
{
	irq_free_desc(irq);
	spin_lock(&bitmap_lock);
	destroy_ipi_dirq(irq);
	spin_unlock(&bitmap_lock);
}
int arch_setup_msi_irqs(struct pci_dev *dev, int nvec, int type)
{
	struct msi_desc *entry;
	int ret;

	if (!pci_msi_enabled())
		return -ENOSPC;

	if (type == PCI_CAP_ID_MSIX && !msix_enable)
		return -ENOSPC;

	ret = setup_msidomain_irq(dev, nvec);
	if (ret) {
		for_each_pci_msi_entry(entry, dev) {
			ret = setup_msi_irq(dev, entry);
			if (ret < 0)
				return ret;
		}
	}
	return 0;
}

int arch_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	int ret;

	if (!pci_msi_enabled())
		return -ENOSPC;

	if (desc->msi_attrib.is_msix && !msix_enable)
		return -ENOSPC;

	ret = setup_msidomain_irq(pdev, 1);
	if (!ret)
		return ret;

	return setup_msi_irq(pdev, desc);
}

void arch_teardown_msi_irq(unsigned int irq)
{
	if (teardown_msidomain_irq(irq))
		teardown_msi_irq(irq);
}

static void msi_nop(struct irq_data *data) { }

struct irq_chip msi_ht_irq_chip = {
	.name = "PCI-MSI-HT",
	.irq_ack = msi_nop,
	.irq_enable = unmask_msi_irq,
	.irq_disable = mask_msi_irq,
	.irq_mask = mask_msi_irq,
	.irq_unmask = unmask_msi_irq,
	.irq_set_affinity = def_set_irq_affinity,
};

static int __init lspci_msi_init(void)
{
	if (loongson_cpu_has_msi256)
		msix_enable = 1;

	return 0;
}

arch_initcall(lspci_msi_init);
