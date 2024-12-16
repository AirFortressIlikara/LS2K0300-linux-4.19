// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2020, Jiaxun Yang <jiaxun.yang@flygoat.com>
 *			Jianmin Lv <lvjianmin@loongson.cn>
 *  Loongson PCH PIC support
 */

#define pr_fmt(fmt) "pch-pic: " fmt

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/syscore_ops.h>
#include <loongson.h>

/* Registers */
#define PCH_PIC_MASK		0x20
#define PCH_PIC_HTMSI_EN	0x40
#define PCH_PIC_EDGE		0x60
#define PCH_PIC_CLR		0x80
#define PCH_PIC_AUTO0		0xc0
#define PCH_PIC_AUTO1		0xe0
#define PCH_INT_ROUTE(irq)	(0x100 + irq)
#define PCH_INT_HTVEC(irq)	(0x200 + irq)
#define PCH_PIC_POL		0x3e0
#define PCH_PIC_STS		0x3a0

#define LIOINTC_PIN_SYSINT0	0
#define PIC_COUNT_PER_REG	64
#define PIC_REG_COUNT		1
#define PIC_COUNT		(PIC_COUNT_PER_REG * PIC_REG_COUNT)
#define GUEST_PIC_COUNT		32
#define PIC_REG_IDX(irq_id)	((irq_id) / PIC_COUNT_PER_REG)
#define PIC_REG_BIT(irq_id)	((irq_id) % PIC_COUNT_PER_REG)
#define PCH_PIC_GSI_BASE	64
#define LS7A_VZ_PCH_REG_BASE		0xe0010000000UL

int nr_pch_pic;
static struct pch_pic {
	void __iomem		*base;
	struct fwnode_handle	*domain_handle;
	struct irq_domain	*pic_domain;
	u32			ht_vec_base;
	u64			saved_vec_en[PIC_REG_COUNT];
	u64			saved_vec_edge[PIC_REG_COUNT];
	u64			saved_vec_pol[PIC_REG_COUNT];
	int			model;
	int			gsi_end;
	int			gsi_base;
	raw_spinlock_t		pic_lock;
} *pch_pic_priv[MAX_PCH_PICS];

u32 pchintc_gsi_base(int id)
{
	return pch_pic_priv[id] ? pch_pic_priv[id]->gsi_base : 0;
}

struct fwnode_handle *pch_pic_get_fwnode(int id)
{
	return pch_pic_priv[id] ? pch_pic_priv[id]->domain_handle : NULL;
}

static void pch_pic_bitset(struct pch_pic *priv, int offset, int bit)
{
	u64 reg;
	unsigned long flags;
	void __iomem *addr = priv->base + offset + PIC_REG_IDX(bit) * 8;

	raw_spin_lock_irqsave(&priv->pic_lock, flags);
	reg = readq(addr);
	reg |= BIT(PIC_REG_BIT(bit));
	writeq(reg, addr);
	raw_spin_unlock_irqrestore(&priv->pic_lock, flags);
}

static void pch_pic_bitclr(struct pch_pic *priv, int offset, int bit)
{
	u64 reg;
	unsigned long flags;
	void __iomem *addr = priv->base + offset + PIC_REG_IDX(bit) * 8;

	raw_spin_lock_irqsave(&priv->pic_lock, flags);
	reg = readq(addr);
	reg &= ~BIT(PIC_REG_BIT(bit));
	writeq(reg, addr);
	raw_spin_unlock_irqrestore(&priv->pic_lock, flags);
}

static void pch_pic_mask_irq(struct irq_data *d)
{
	struct pch_pic *priv = irq_data_get_irq_chip_data(d);

	pch_pic_bitset(priv, PCH_PIC_MASK, d->hwirq);
	irq_chip_mask_parent(d);
}

static void pch_pic_unmask_irq(struct irq_data *d)
{
	struct pch_pic *priv = irq_data_get_irq_chip_data(d);
	u32 idx = PIC_REG_IDX(d->hwirq);

	irq_chip_unmask_parent(d);
	/*
	 * do not remove it, or put PCH_PIC_CLR in ack function
	 * since level-triggered intx for pass-through vm need
	 * EIO from interrupt controller
	 */
	writeq(BIT(PIC_REG_BIT(d->hwirq)),
			priv->base + PCH_PIC_CLR + idx * 8);
	pch_pic_bitclr(priv, PCH_PIC_MASK, d->hwirq);
}

static void pch_line_mask_irq(struct irq_data *d)
{
	struct pch_pic *priv = irq_data_get_irq_chip_data(d);

	pch_pic_bitset(priv, PCH_PIC_MASK, d->hwirq);
}

static void pch_line_unmask_irq(struct irq_data *d)
{
	struct pch_pic *priv = irq_data_get_irq_chip_data(d);

	pch_pic_bitclr(priv, PCH_PIC_MASK, d->hwirq);
}
static int pch_pic_set_type(struct irq_data *d, unsigned int type)
{
	struct pch_pic *priv = irq_data_get_irq_chip_data(d);
	int ret = 0;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		pch_pic_bitset(priv, PCH_PIC_EDGE, d->hwirq);
		pch_pic_bitclr(priv, PCH_PIC_POL, d->hwirq);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		pch_pic_bitset(priv, PCH_PIC_EDGE, d->hwirq);
		pch_pic_bitset(priv, PCH_PIC_POL, d->hwirq);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		pch_pic_bitclr(priv, PCH_PIC_EDGE, d->hwirq);
		pch_pic_bitclr(priv, PCH_PIC_POL, d->hwirq);
		irq_set_handler_locked(d, handle_level_irq);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		pch_pic_bitclr(priv, PCH_PIC_EDGE, d->hwirq);
		pch_pic_bitset(priv, PCH_PIC_POL, d->hwirq);
		irq_set_handler_locked(d, handle_level_irq);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static unsigned int pch_line_startup(struct irq_data *d)
{
	create_ipi_dirq(d->irq);
	pch_line_unmask_irq(d);
	return 0;
}

static void pch_line_shutdown(struct irq_data *d)
{
	pch_line_mask_irq(d);
	destroy_ipi_dirq(d->irq);
}
static unsigned int pch_pic_startup(struct irq_data *d)
{
	create_ipi_dirq(d->irq);
	pch_pic_unmask_irq(d);
	return 0;
}

static void pch_pic_shutdown(struct irq_data *d)
{
	pch_pic_mask_irq(d);
	destroy_ipi_dirq(d->irq);
}

static void pch_pic_ack_irq(struct irq_data *d)
{
	u64 reg;
	struct pch_pic *priv = irq_data_get_irq_chip_data(d);
	void __iomem *addr =
			priv->base + PCH_PIC_EDGE + PIC_REG_IDX(d->hwirq) * 8;

	/* only clear edge-trigger IRQs ?? */
	reg = readq(addr);
	if (reg & BIT_ULL(d->hwirq)) {
		writeq(BIT(PIC_REG_BIT(d->hwirq)),
			priv->base + PCH_PIC_CLR + PIC_REG_IDX(d->hwirq) * 8);
	}
	irq_chip_ack_parent(d);
}

static struct irq_chip pch_line_irq_chip = {
	.name			= "PCH-PIC-LINE",
	.irq_mask		= pch_line_mask_irq,
	.irq_unmask		= pch_line_unmask_irq,
	.irq_set_type		= pch_pic_set_type,
	.irq_startup		= pch_line_startup,
	.irq_shutdown		= pch_line_shutdown,
	.irq_set_affinity	= def_set_irq_affinity,
	.flags			= IRQCHIP_SKIP_SET_WAKE,
};

static struct irq_chip pch_pic_irq_chip = {
	.name			= "PCH-PIC-HT",
	.irq_mask		= pch_pic_mask_irq,
	.irq_unmask		= pch_pic_unmask_irq,
	.irq_ack		= pch_pic_ack_irq,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.irq_set_type		= pch_pic_set_type,
	.irq_startup		= pch_pic_startup,
	.irq_shutdown		= pch_pic_shutdown,
	.flags			= IRQCHIP_SKIP_SET_WAKE,
};

static void pch_handle_irq(struct irq_desc *desc)
{
	struct pch_pic *priv = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	int cpu = smp_processor_id();
	u64 pending;

	chained_irq_enter(chip, desc);

	pending = readq(priv->base + PCH_PIC_STS);

	if (!pending)
		spurious_interrupt();

	while (pending) {
		int bit = __ffs(pending);
		int virq = irq_linear_revmap(priv->pic_domain, bit);
		if (virq > 0) handle_virq(virq, cpu);
		pending &= ~BIT(bit);
	}
	chained_irq_exit(chip, desc);
}

static int pch_pic_domain_translate(struct irq_domain *d,
					struct irq_fwspec *fwspec,
					unsigned long *hwirq,
					unsigned int *type)
{
	struct pch_pic *priv = d->host_data;
	struct device_node *of_node = to_of_node(fwspec->fwnode);

	if (fwspec->param_count < 1)
		return -EINVAL;

	if (of_device_is_compatible(of_node, "loongson,pch-pic-1.0")) {
		if (fwspec->param_count < 2)
			return -EINVAL;

		*hwirq = fwspec->param[0];
	} else {
		*hwirq = fwspec->param[0] - priv->gsi_base;
	}

	if (fwspec->param_count > 1)
		*type = fwspec->param[1] & IRQ_TYPE_SENSE_MASK;
	else
		*type = IRQ_TYPE_NONE;

	return 0;
}

static int pch_line_alloc(struct irq_domain *domain, unsigned int virq,
				unsigned int nr_irqs, void *arg)
{
	unsigned int type, i;
	unsigned long hwirq = 0;
	struct htvec *priv = domain->host_data;

	pch_pic_domain_translate(domain, arg, &hwirq, &type);
	for (i = 0; i < nr_irqs; i++) {
		irq_domain_set_info(domain, virq + i, hwirq + i, &pch_line_irq_chip,
					priv, handle_level_irq, NULL, NULL);
	}

	irq_set_noprobe(virq);
	return 0;
}

static int pch_pic_alloc(struct irq_domain *domain, unsigned int virq,
				unsigned int nr_irqs, void *arg)
{
	int err;
	unsigned int type;
	struct irq_fwspec fwspec;
	struct pch_pic *priv = domain->host_data;
	unsigned long hwirq = 0;

	pch_pic_domain_translate(domain, arg, &hwirq, &type);

	fwspec.fwnode = domain->parent->fwnode;
	fwspec.param_count = 1;
	fwspec.param[0] = hwirq + priv->ht_vec_base;
	err = irq_domain_alloc_irqs_parent(domain, virq, 1, &fwspec);
	if (err)
		return err;

	irq_domain_set_info(domain, virq, hwirq,
				&pch_pic_irq_chip, priv,
				handle_level_irq, NULL, NULL);
	irq_set_noprobe(virq);

	return 0;
}
static const struct irq_domain_ops pch_line_domain_ops = {
	.translate	= pch_pic_domain_translate,
	.alloc		= pch_line_alloc,
	.free		= irq_domain_free_irqs_parent,
};
static const struct irq_domain_ops pch_pic_domain_ops = {
	.translate	= pch_pic_domain_translate,
	.alloc		= pch_pic_alloc,
	.free		= irq_domain_free_irqs_parent,
};

static void pch_pic_reset(struct pch_pic *priv)
{
	int i;

	for (i = 0; i < PIC_COUNT; i++) {
		if (priv->model != PCH_IRQ_ROUTE_LINE) {
			/* Write vector ID */
			writeb(priv->ht_vec_base + i, priv->base + PCH_INT_HTVEC(i));
		}
		/* Hardcode route to HT0 Lo */
		writeb(1, priv->base + PCH_INT_ROUTE(i));
	}

	for (i = 0; i < PIC_REG_COUNT; i++) {
		/* Clear IRQ cause registers, mask all interrupts */
		writeq_relaxed((u64)-1, priv->base + PCH_PIC_MASK + 8 * i);
		writeq_relaxed((u64)-1, priv->base + PCH_PIC_CLR + 8 * i);
		/* Clear auto bounce, we don't need that */
		writeq_relaxed(0, priv->base + PCH_PIC_AUTO0 + 8 * i);
		writeq_relaxed(0, priv->base + PCH_PIC_AUTO1 + 8 * i);
		if (priv->model != PCH_IRQ_ROUTE_LINE) {
			/* Enable HTMSI transformer */
			writeq_relaxed((u64)-1, priv->base + PCH_PIC_HTMSI_EN + 8 * i);
		}
	}
}

int find_pch_pic(u32 gsi)
{
	int i;

	if (nr_pch_pic == 0)
		return -1;

	/* Find the PCH_PIC that manages this GSI. */
	for (i = 0; i < nr_pch_pic; i++) {
		if (gsi >= pch_pic_priv[i]->gsi_base && gsi <= pch_pic_priv[i]->gsi_end)
			return i;
	}

	printk(KERN_ERR "ERROR: Unable to locate PCH_PIC for GSI %d\n", gsi);
	return -1;
}
int pch_pic_init(struct fwnode_handle *irq_handle,
		unsigned long addr,
		unsigned int size,
		int model,
		int gsi_base)
{
	struct pch_pic *priv;
	int err;
	struct fwnode_handle *parent;
	int count;
	struct device_node *of_node = to_of_node(irq_handle);

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	raw_spin_lock_init(&priv->pic_lock);
	priv->base = ioremap(addr, size);

	if (!priv->base) {
		err = -ENOMEM;
		goto free_priv;
	}

	if (of_device_is_compatible(of_node, "loongson,pch-pic-1.0")) {
		int vec_base;

		if (of_property_read_u32(of_node, "loongson,pic-base-vec", &vec_base)) {
			pr_err("Failed to determine pic-base-vec\n");
			goto free_priv;
		}
		priv->ht_vec_base = vec_base;
	} else {
		priv->ht_vec_base = 0;
	}
	count = (((unsigned long)readq(priv->base) >> 48) & 0xff) + 1;
	priv->domain_handle = irq_handle;
	priv->model = model;
	priv->gsi_base = gsi_base;
	priv->gsi_end = gsi_base + count - 1;
	if (priv->model == PCH_IRQ_ROUTE_LINE) {
		parent = liointc_get_fwnode();
		priv->pic_domain = irq_domain_create_linear(priv->domain_handle,
							count,
							&pch_line_domain_ops,
							priv);
	} else if (priv->model == PCH_IRQ_ROUTE_HT ||
			priv->model == PCH_IRQ_ROUTE_EXT) {
		struct irq_domain *parent_domain;
		if (priv->model == PCH_IRQ_ROUTE_EXT) {
			parent = eiointc_get_fwnode(nr_pch_pic);
			pch_pic_irq_chip.name = "PCH-PIC-EXT";
		} else
			parent = htvec_get_fwnode();
		parent_domain = irq_find_matching_fwnode(
							parent, DOMAIN_BUS_ANY);
		if (!parent_domain) {
			pr_err("Failed to find the parent domain\n");
			err = -ENODEV;
			goto iounmap_base;
		}

		priv->pic_domain = irq_domain_create_hierarchy(parent_domain, 0,
							count,
							priv->domain_handle,
							&pch_pic_domain_ops,
							priv);
	} else if (priv->model == PCH_IRQ_ROUTE_EXT_GUEST) {
		struct irq_domain *parent_domain;
		parent = eiointc_get_fwnode(0);
		parent_domain = irq_find_matching_fwnode(
						parent, DOMAIN_BUS_ANY);
		if (!parent_domain) {
			pr_err("Failed to find the parent domain\n");
			err = -ENODEV;
			goto iounmap_base;
		}
		pch_pic_irq_chip.name = "PCH-PIC-EXT";

		priv->pic_domain = irq_domain_create_hierarchy(parent_domain, 0,
							count,
							priv->domain_handle,
							&pch_pic_domain_ops,
							priv);

	}

	if (!priv->pic_domain) {
		pr_err("Failed to create IRQ domain\n");
		err = -ENOMEM;
		goto iounmap_base;
	}

#ifdef CONFIG_LOONGARCH
	if (!nr_pch_pic && priv->model != PCH_IRQ_ROUTE_EXT_SOC)
#endif
#ifdef CONFIG_MIPS
	if (!nr_pch_pic)
#endif
		irq_set_default_host(priv->pic_domain);
	if (priv->model == PCH_IRQ_ROUTE_LINE) {
		struct irq_fwspec fwspec;
		int parent_irq;
		fwspec.fwnode = parent;
		fwspec.param[0] = LIOINTC_PIN_SYSINT0;
		fwspec.param_count = 1;
		parent_irq = irq_create_fwspec_mapping(&fwspec);
		irq_set_chained_handler_and_data(parent_irq,
						 pch_handle_irq, priv);
	}
	pch_pic_reset(priv);
	pch_pic_priv[nr_pch_pic++] = priv;
	return 0;

iounmap_base:
	iounmap(priv->base);
free_priv:
	kfree(priv);

	return err;
}
static int pch_pic_of_init(struct device_node *of_node,
				struct device_node *parent)
{
	struct resource res;
	u64 res_array[2];
	struct irq_domain *parent_domain;

	of_property_read_u64_array(of_node, "reg", res_array, 2);
	res.start = res_array[0];
	res.end = res_array[0] + res_array[1];

	if (of_device_is_compatible(of_node, "loongson,pch-pic-1.0")) {
		parent_domain = irq_find_host(parent);
		if (!parent_domain) {
			pr_err("Failed to find the parent domain\n");
			return -ENXIO;
		}

		if ((parent_domain->fwnode != eiointc_get_fwnode(nr_pch_pic)) &&
				(parent_domain->fwnode != htvec_get_fwnode())) {
			pr_err("Wrong parent domain\n");
			return -EINVAL;
		}
	}

	pch_pic_init(of_node_to_fwnode(of_node),
			res.start,
			res_array[1],
			get_irq_route_model(),
			64);
	return 0;
}

#ifdef CONFIG_PM
static void pch_pic_resume(void)
{
	int i, j;
	for (j = 0; j < nr_pch_pic; j++) {
		pch_pic_reset(pch_pic_priv[j]);
		for (i = 0; i < PIC_REG_COUNT; i++) {
			writeq(pch_pic_priv[j]->saved_vec_en[i],
					pch_pic_priv[j]->base + PCH_PIC_MASK + 8 * i);
			writeq(pch_pic_priv[j]->saved_vec_edge[i],
					pch_pic_priv[j]->base + PCH_PIC_EDGE + 8 * i);
			writeq(pch_pic_priv[j]->saved_vec_pol[i],
					pch_pic_priv[j]->base + PCH_PIC_POL + 8 * i);
		}
	}
}

static int pch_pic_suspend(void)
{
	int i, j;
	for (j = 0; j < nr_pch_pic; j++)
		for (i = 0; i < PIC_REG_COUNT; i++) {
			pch_pic_priv[j]->saved_vec_en[i] =
				readq(pch_pic_priv[j]->base + PCH_PIC_MASK + 8 * i);
			pch_pic_priv[j]->saved_vec_edge[i] =
				readq(pch_pic_priv[j]->base + PCH_PIC_EDGE + 8 * i);
			pch_pic_priv[j]->saved_vec_pol[i] =
				readq(pch_pic_priv[j]->base + PCH_PIC_POL + 8 * i);
		}
	return 0;
}

#else
#define pch_pic_suspend NULL
#define pch_pic_resume NULL
#endif

static struct syscore_ops pch_pic_syscore_ops = {
	.suspend =  pch_pic_suspend,
	.resume =  pch_pic_resume,
};

static int __init pch_pic_init_syscore_ops(void)
{
	if (nr_pch_pic > 0)
		register_syscore_ops(&pch_pic_syscore_ops);
	return 0;
}
device_initcall(pch_pic_init_syscore_ops);
IRQCHIP_DECLARE(pch_pic, "loongson,ls7a-interrupt-controller", pch_pic_of_init);
IRQCHIP_DECLARE(pch_pic_v1, "loongson,pch-pic-1.0", pch_pic_of_init);
#ifdef CONFIG_ACPI
static int __init pch_pic_acpi_init_v1(struct acpi_subtable_header *header,
				   const unsigned long end)
{
	struct acpi_madt_bio_pic *pch_pic_entry;
	struct fwnode_handle *irq_handle;
	pch_pic_entry = (struct acpi_madt_bio_pic *)header;

	irq_handle = irq_domain_alloc_named_id_fwnode("pch_pic", pch_pic_entry->id);
	if (!irq_handle) {
		pr_err("Unable to allocate domain handle\n");
		return -ENOMEM;
	}

	pch_pic_init(irq_handle,
			pch_pic_entry->address,
			pch_pic_entry->size,
			get_irq_route_model(),
			pch_pic_entry->gsi_base);
	return 0;
}
IRQCHIP_ACPI_DECLARE(pch_pic_v1, ACPI_MADT_TYPE_BIO_PIC,
		NULL, ACPI_MADT_BIO_PIC_VERSION_V1,
		pch_pic_acpi_init_v1);
#endif
