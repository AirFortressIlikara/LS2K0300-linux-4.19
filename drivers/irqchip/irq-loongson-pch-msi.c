// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2020, Jiaxun Yang <jiaxun.yang@flygoat.com>
 *			lvjianmin <lvjianmin@loongson.cn>
 *  Loongson PCH MSI support
 */

#define pr_fmt(fmt) "pch-msi: " fmt

#include <linux/irqchip.h>
#include <linux/msi.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <boot_param.h>

struct pch_msi_data {
	struct mutex	msi_map_lock;
	u32		irq_first;	/* The vector number that MSIs starts */
	u32		num_irqs;	/* The number of vectors for MSIs */
	unsigned long	*msi_map;
	struct fwnode_handle *domain_handle;
	int		ext;
	u64	msg_address;
} *pch_msi_priv[MAX_MSI_PICS];

int nr_msi;
static void pch_msi_mask_msi_irq(struct irq_data *d)
{
	pci_msi_mask_irq(d);
	irq_chip_mask_parent(d);
}

static void pch_msi_unmask_msi_irq(struct irq_data *d)
{
	irq_chip_unmask_parent(d);
	pci_msi_unmask_irq(d);
}

static void pch_msi_compose_msi_msg(struct irq_data *data,
					struct msi_msg *msg)
{
	struct pch_msi_data *priv;
	struct msi_domain_info *info = (struct msi_domain_info *)data->domain->host_data;
	priv = (struct pch_msi_data *)info->data;

	msg->address_hi = priv->msg_address >> 32;
	msg->address_lo = priv->msg_address;
	msg->data = data->hwirq;
}

static struct irq_chip pch_msi_irq_chip = {
	.name			= "PCH-MSI-HT",
	.irq_mask		= pch_msi_mask_msi_irq,
	.irq_unmask		= pch_msi_unmask_msi_irq,
	.irq_ack		= irq_chip_ack_parent,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.irq_compose_msi_msg	= pch_msi_compose_msi_msg,
};

static int pch_msi_allocate_hwirq(unsigned int num_req, struct pch_msi_data *priv)
{
	int first;

	mutex_lock(&priv->msi_map_lock);

	first = bitmap_find_free_region(priv->msi_map,
					priv->num_irqs,
					get_count_order(num_req));
	if (first < 0) {
		mutex_unlock(&priv->msi_map_lock);
		return -ENOSPC;
	}

	mutex_unlock(&priv->msi_map_lock);
	return priv->irq_first + first;
}

static irq_hw_number_t msi_domain_ops_get_hwirq(struct msi_domain_info *info,
						msi_alloc_info_t *arg)
{
	return arg->param[0];
}

static void msi_domain_ops_free(struct irq_domain *domain,
				struct msi_domain_info *info, unsigned int virq)
{
	struct irq_data *irq_data_cur = irq_get_irq_data(virq);
	struct pch_msi_data *priv = info->data;
	int first = irq_data_cur->hwirq - priv->irq_first;

	mutex_lock(&priv->msi_map_lock);
	bitmap_release_region(priv->msi_map, first, 0);
	mutex_unlock(&priv->msi_map_lock);
}

static int msi_domain_ops_prepare(struct irq_domain *domain, struct device *dev,
				  int nvec, msi_alloc_info_t *arg)
{
	memset(arg, 0, sizeof(*arg));
	*((u64 *)&arg->param[IRQ_DOMAIN_IRQ_SPEC_PARAMS - 2]) = (u64)domain->host_data;
	return 0;
}

static void pci_msi_domain_set_desc(msi_alloc_info_t *arg,
				    struct msi_desc *desc)
{
	struct pch_msi_data *priv;
	struct msi_domain_info *info = (struct msi_domain_info *)(*(u64 *)&arg->param[IRQ_DOMAIN_IRQ_SPEC_PARAMS - 2]);
	priv = (struct pch_msi_data *)info->data;

	arg->param_count = 1;
	arg->param[0] = pch_msi_allocate_hwirq(desc->nvec_used, priv);
}

static struct msi_domain_ops pch_msi_domain_ops = {
	.get_hwirq	= msi_domain_ops_get_hwirq,
	.msi_free	= msi_domain_ops_free,
	.msi_prepare	= msi_domain_ops_prepare,
	.set_desc	= pci_msi_domain_set_desc,
};

static struct msi_domain_info pch_msi_domain_info[MAX_MSI_PICS];

struct fwnode_handle *msi_irqdomain_handle(int id)
{
	return pch_msi_priv[id] ? pch_msi_priv[id]->domain_handle : NULL;
}

int msi_ext_ioi(int id)
{
	return pch_msi_priv[id] ? pch_msi_priv[id]->ext : 0;
}

static int pch_msi_init_domain(struct fwnode_handle *parent_handle,
				bool ext, struct pch_msi_data *priv, int id)
{
	struct irq_domain *parent, *msi_domain;

	parent = irq_find_matching_fwnode(parent_handle, DOMAIN_BUS_ANY);
	if (!parent)
		return -EINVAL;
	pch_msi_domain_info[id].chip = &pch_msi_irq_chip;
	if (ext)
		pch_msi_domain_info[id].chip->name = "PCH-MSI-EXT";
	pch_msi_domain_info[id].data = (void *)priv;
	pch_msi_domain_info[id].flags = MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		  MSI_FLAG_MULTI_PCI_MSI | MSI_FLAG_PCI_MSIX;
	pch_msi_domain_info[id].ops = &pch_msi_domain_ops;

	msi_domain = pci_msi_create_irq_domain(priv->domain_handle,
						&pch_msi_domain_info[id],
						parent);
	if (!msi_domain) {
		pr_err("Failed to create PCI MSI domain\n");
		return -ENOMEM;
	}

	return 0;
}

int pch_msi_init(struct fwnode_handle *irq_handle,
		struct fwnode_handle *parent_handle,
		u64 msg_address,
		bool ext, int start, int count)
{
	int ret;
	pch_msi_priv[nr_msi] = kzalloc(sizeof(struct pch_msi_data), GFP_KERNEL);
	if (!pch_msi_priv[nr_msi])
		return -ENOMEM;

	mutex_init(&pch_msi_priv[nr_msi]->msi_map_lock);
	pch_msi_priv[nr_msi]->domain_handle = irq_handle;
	pch_msi_priv[nr_msi]->irq_first = start;
	pch_msi_priv[nr_msi]->num_irqs = count;
	pch_msi_priv[nr_msi]->ext = ext;
	pch_msi_priv[nr_msi]->msg_address = msg_address;
	pch_msi_priv[nr_msi]->msi_map = bitmap_zalloc(pch_msi_priv[nr_msi]->num_irqs,
					     GFP_KERNEL);
	if (!pch_msi_priv[nr_msi]->msi_map) {
		ret = -ENOMEM;
		goto err_priv;
	}

	pr_debug("Registering %d MSIs, starting at %d\n", pch_msi_priv[nr_msi]->num_irqs, pch_msi_priv[nr_msi]->irq_first);

	ret = pch_msi_init_domain(parent_handle, ext, pch_msi_priv[nr_msi], nr_msi);
	if (ret)
		goto err_map;

	nr_msi++;
	return 0;

err_map:
	kfree(pch_msi_priv[nr_msi]->msi_map);
err_priv:
	kfree(pch_msi_priv[nr_msi]);
	return ret;
}

static int  __init pch_msi_of_init(struct device_node *of_node,
				struct device_node *parent)
{
	struct fwnode_handle *irq_handle, *parent_handle;
	struct resource res;
	u32 msi_start, msi_count;
	u64 msg_address;
	int ret;

	irq_handle = of_node_to_fwnode(of_node);
	if (!irq_handle) {
		pr_err("Failed to find the irq_handle\n");
		return -ENXIO;
	}

	if (get_irq_route_model() == PCH_IRQ_ROUTE_EXT)
		parent_handle = eiointc_get_fwnode(nr_msi);
	else
		parent_handle = htvec_get_fwnode();

	ret =	of_property_read_u32(of_node, "loongson,msi-base-vec", &msi_start);
	if (ret) {
		pr_err("Unable to parse MSI vec base\n");
		return -EINVAL;
	}

	ret =	of_property_read_u32(of_node, "loongson,msi-num-vecs", &msi_count);
	if (ret) {
		pr_err("Unable to parse MSI vec number\n");
		return -EINVAL;
	}

	ret =	of_address_to_resource(of_node, 0, &res);
	if (ret) {
		pr_err("Unable to parse MSI vec address\n");
		return -EINVAL;
	}
	msg_address = res.start;

	return pch_msi_init(irq_handle, parent_handle,
			msg_address,
			get_irq_route_model() == PCH_IRQ_ROUTE_EXT,
			msi_start,
			msi_count);
}
IRQCHIP_DECLARE(pch_pic, "loongson,pch-msi-1.0", pch_msi_of_init);

#ifdef CONFIG_ACPI
static int __init pch_msi_acpi_init_v1(struct acpi_subtable_header *header,
				   const unsigned long end)
{
	struct acpi_madt_msi_pic *pch_msi_entry;
	struct fwnode_handle *irq_handle, *parent_handle;
	pch_msi_entry = (struct acpi_madt_msi_pic *)header;

	irq_handle = irq_domain_alloc_named_id_fwnode("msintc", nr_msi);
	if (!irq_handle) {
		pr_err("Unable to allocate domain handle\n");
		return -ENOMEM;
	}

	if (get_irq_route_model() == PCH_IRQ_ROUTE_EXT) {
		parent_handle = eiointc_get_fwnode(nr_msi);
	} else {
		parent_handle = htvec_get_fwnode();
	}

	pch_msi_init(irq_handle, parent_handle,
			pch_msi_entry->msg_address,
			get_irq_route_model() == PCH_IRQ_ROUTE_EXT,
			pch_msi_entry->start,
			pch_msi_entry->count);
	return 0;
}
IRQCHIP_ACPI_DECLARE(pch_msi_v1, ACPI_MADT_TYPE_MSI_PIC,
		NULL, ACPI_MADT_MSI_PIC_VERSION_V1,
		pch_msi_acpi_init_v1);
#endif
