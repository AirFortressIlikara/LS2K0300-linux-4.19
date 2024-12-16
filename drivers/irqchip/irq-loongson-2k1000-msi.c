#include <linux/irq.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>
#include <linux/msi.h>
#include <loongson.h>
#include <loongson-2k.h>

extern raw_spinlock_t ls2k_irq_lock;

#define LS2K_INT_MSI_ADDR_0		0x1fe114b0
#define LS2K_INT_MSI_ADDR_1		0x1fe114f0
#define LS2K_INTEDGE_REG(i)		TO_UNCAC(0x1fe11434 | (((i) > 31) ? 0x40 : 0))
#define LS2K_MSI_TRIGGER_EN_REG(i)	TO_UNCAC(0x1fe114b4 | (((i) > 31) ? 0x40 : 0))
#define LS2K_INTENCLR_REG(i)		TO_UNCAC(0x1fe1142c | (((i) > 31) ? 0x40 : 0))
#define LS2K_INTENSET_REG(i)		TO_UNCAC(0x1fe11428 | (((i) > 31) ? 0x40 : 0))
#define LS2K_INTEN_REG(i)		TO_UNCAC(0x1fe11424 | (((i) > 31) ? 0x40 : 0))

struct ls2k_msi {
	u64  mask;
	u64  used;
	struct irq_domain *domain;
};

static struct ls2k_msi *msi_controller;

void ls2k_msi_handler(struct irq_desc *desc)
{
	unsigned int hwirq = desc->irq_data.irq - LS2K_IRQ_BASE;

	do_IRQ(hwirq + LS2K_MSI_IRQ_BASE);
}

int arch_setup_msi_irq(struct pci_dev *dev, struct msi_desc *desc)
{
	struct msi_msg msg;
	int virq, hwirq, ret;
	u16 val;
	u64 free;
	u32 reg_val;
	unsigned long flags;

	pci_read_config_word(dev, dev->msi_cap + PCI_MSI_FLAGS, &val);
	if ((val & PCI_MSI_FLAGS_ENABLE) == 0) {
		val |= PCI_MSI_FLAGS_ENABLE;
		pci_write_config_word(dev, dev->msi_cap + PCI_MSI_FLAGS, val);
	}

	free = msi_controller->mask ^ msi_controller->used;
	hwirq = fls64(free);
	if (hwirq <= 0)
		return 1;

	hwirq -= 1;
	msi_controller->used |= (1UL << hwirq);

	raw_spin_lock_irqsave(&ls2k_irq_lock, flags);
	reg_val = ls64_conf_read32((void *)LS2K_INTEDGE_REG(hwirq));
	reg_val |= (1 << (hwirq%32));
	ls64_conf_write32(reg_val, (void *)LS2K_INTEDGE_REG(hwirq));
	raw_spin_unlock_irqrestore(&ls2k_irq_lock, flags);

	irq_set_chained_handler(hwirq + LS2K_IRQ_BASE, ls2k_msi_handler);

	msg.address_hi = 0;
	if (hwirq <= 31)
		msg.address_lo = LS2K_INT_MSI_ADDR_0;
	else
		msg.address_lo = LS2K_INT_MSI_ADDR_1;

	virq = hwirq + LS2K_MSI_IRQ_BASE;

	msg.data = hwirq%32;

	ret = irq_set_msi_desc(virq, desc);
	if (ret < 0)
		return ret;
	pci_write_msi_msg(virq, &msg);
	return 0;
}

void arch_teardown_msi_irq(unsigned int virq)
{
	unsigned int hwirq = virq - LS2K_MSI_IRQ_BASE;

	msi_controller->used &= ~(1UL << hwirq);
	irq_set_chained_handler(hwirq + LS2K_IRQ_BASE, NULL);
}

int arch_setup_msi_irqs(struct pci_dev *dev, int nvec, int type)
{
	struct msi_desc *entry;
	int ret;

	if (msi_controller == NULL)
		return -ENOSPC;
	/*
	 * MSI-X is not supported.
	 */
	if (type == PCI_CAP_ID_MSIX)
		return -EINVAL;

	/*
	 * If an architecture wants to support multiple MSI, it needs to
	 * override arch_setup_msi_irqs()
	 */
	if (type == PCI_CAP_ID_MSI && nvec > 1)
		return 1;

	for_each_pci_msi_entry(entry, dev) {
		ret = arch_setup_msi_irq(dev, entry);
		if (ret < 0)
			return ret;
		if (ret > 0)
			return -ENOSPC;
	}
	return 0;
}

void ls2k_msi_mask(struct irq_data *data)
{
	unsigned long flags;
	unsigned long hwirq = data->hwirq - LS2K_IOINTC_HWIRQ_BASE;
	u32 reg_val;

	raw_spin_lock_irqsave(&ls2k_irq_lock, flags);
	reg_val = ls64_conf_read32((void *)LS2K_MSI_TRIGGER_EN_REG(hwirq));
	reg_val &= ~(1 << (hwirq%32));
	ls64_conf_write32(reg_val, (void *)LS2K_MSI_TRIGGER_EN_REG(hwirq));
	raw_spin_unlock_irqrestore(&ls2k_irq_lock, flags);

	pci_msi_mask_irq(data);
}

void ls2k_msi_unmask(struct irq_data *data)
{
	unsigned long flags;
	unsigned long hwirq = data->hwirq - LS2K_IOINTC_HWIRQ_BASE;
	u32 reg_val;

	raw_spin_lock_irqsave(&ls2k_irq_lock, flags);
	reg_val = ls64_conf_read32((void *)LS2K_MSI_TRIGGER_EN_REG(hwirq));
	reg_val |= 1 << (hwirq%32);
	ls64_conf_write32(reg_val, (void *)LS2K_MSI_TRIGGER_EN_REG(hwirq));
	raw_spin_unlock_irqrestore(&ls2k_irq_lock, flags);

	pci_msi_unmask_irq(data);
}

void ls2k_msi_ack(struct irq_data *data)
{
	unsigned long hwirq = data->hwirq - LS2K_IOINTC_HWIRQ_BASE;
	unsigned long reg;
	unsigned long flags;
	unsigned int reg_val;

	raw_spin_lock_irqsave(&ls2k_irq_lock, flags);

	reg = LS2K_INTEN_REG(hwirq);
	reg_val = ls64_conf_read32((void *)reg);

	reg = LS2K_INTENCLR_REG(hwirq);
	ls64_conf_write32(1<<(hwirq%32), (void *)reg);

	if (reg_val & (1 << (hwirq%32)))
		reg = LS2K_INTENSET_REG(hwirq);
	else
		reg = LS2K_INTENCLR_REG(hwirq);
	ls64_conf_write32(1<<(hwirq%32), (void *)reg);
	raw_spin_unlock_irqrestore(&ls2k_irq_lock, flags);
}

extern int ls2k_iointc_set_affinity(struct irq_data *d, const struct cpumask *affinity,
		bool force);

struct irq_chip ls2k_msi_chip = {
		.name = "2k-msi",
		.irq_mask = ls2k_msi_mask,
		.irq_unmask = ls2k_msi_unmask,
		.irq_ack = ls2k_msi_ack,
		.irq_set_affinity = ls2k_iointc_set_affinity,
};

int ls2k_msi_irq_domain_map(struct irq_domain *d, unsigned int virq, irq_hw_number_t hw)
{
	irq_set_chip_and_handler(virq, &ls2k_msi_chip, handle_edge_irq);
	return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
	.map	= ls2k_msi_irq_domain_map,
};

static int ls2k_msi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;

	/* disable all msi interrupts */
	ls64_conf_write64(0, (void *)LS2K_MSI_TRIGGER_EN_REG(0));
	ls64_conf_write64(0, (void *)LS2K_MSI_TRIGGER_EN_REG(32));

	msi_controller = devm_kzalloc(&pdev->dev, sizeof(struct ls2k_msi), GFP_KERNEL);
	if (!msi_controller)
		return -ENOMEM;

	if (of_property_read_u64(np, "msi-mask", &msi_controller->mask)) {
		dev_err(&pdev->dev, "failed to parse msi-mask\n");
		return -EINVAL;
	}

	/* ls2k has upto 64 msi interrupts */
	irq_alloc_descs(LS2K_MSI_IRQ_BASE, LS2K_MSI_IRQ_BASE, 64, LS2K_IOINTC_HWIRQ_BASE);
	msi_controller->domain =  irq_domain_add_legacy(np, 64, LS2K_MSI_IRQ_BASE,
					LS2K_IOINTC_HWIRQ_BASE, &msi_domain_ops, msi_controller);

	return 0;
}

static const struct of_device_id ls2k_msi_of_match[] = {
	{ .compatible = "loongson,2k-pci-msi", NULL },
	{ .compatible = "loongson,2k1000-pci-msi", NULL },
	{ },
};

static struct platform_driver ls2k_msi_driver = {
	.driver = {
		.name = "2k-msi",
		.of_match_table = ls2k_msi_of_match,
	},
	.probe = ls2k_msi_probe,
};

static int __init ls2k_msi_init(void)
{
	return platform_driver_register(&ls2k_msi_driver);
}
subsys_initcall(ls2k_msi_init);
