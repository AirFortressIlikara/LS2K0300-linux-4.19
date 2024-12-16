// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2020, Jiaxun Yang <jiaxun.yang@flygoat.com>
 *			Jianmin Lv <lvjianmin@loongson.cn>
 *  Loongson Local IO Interrupt Controller support
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/irqchip.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <loongson.h>
#include <linux/irqchip/chained_irq.h>

#include <boot_param.h>
#include <asm/irq_cpu.h>

#define LIOINTC_CHIP_IRQ	32
#define LIOINTC_NUM_CORES	4
#define LIOINTC_NUM_PARENT	4

#define LIOINTC_INTC_CHIP_START	0x20
#define LIOINTC_REG_INTC_STATUS(cpuid)	(LIOINTC_INTC_CHIP_START + 0x20 + (cpuid) * 8)
#define LIOINTC_REG_INTC_EN_STATUS	(LIOINTC_INTC_CHIP_START + 0x04)
#define LIOINTC_REG_INTC_ENABLE		(LIOINTC_INTC_CHIP_START + 0x08)
#define LIOINTC_REG_INTC_DISABLE	(LIOINTC_INTC_CHIP_START + 0x0c)
#define LIOINTC_REG_INTC_POL		(LIOINTC_INTC_CHIP_START + 0x10)
#define LIOINTC_REG_INTC_EDGE		(LIOINTC_INTC_CHIP_START + 0x14)

#define LIOINTC_SHIFT_INTx	4
#define LIOINTC_VECS_TO_IP2	0x00FFFFFE /* others */
#define LIOINTC_VECS_TO_IP3_V1	0xFF000000 /* HT1 0-7 */
#define LIOINTC_VECS_TO_IP3_V0 0x0F000001 /* HT1 0-3 and sys int 0 */

#ifdef CONFIG_LOONGARCH
#define loongson_cpu_has_msi256 1
#endif

#if defined(CONFIG_MIPS)
#define liointc_core_id		get_ebase_cpunum()
#else
#define liointc_core_id		get_csr_cpuid()
#endif

struct fwnode_handle *liointc_handle;
struct liointc_handler_data {
	struct liointc_priv	*priv;
	u32			parent_int_map;
};

struct liointc_priv {
	struct irq_chip_generic		*gc;
	struct liointc_handler_data	handler[LIOINTC_NUM_PARENT];
	void __iomem			*core_isr[LIOINTC_NUM_CORES];
	u8				map_cache[LIOINTC_CHIP_IRQ];
	u32				int_pol;
	u32				int_edge;
};

struct fwnode_handle *liointc_get_fwnode(void)
{
	return liointc_handle;
}
static void liointc_chained_handle_irq(struct irq_desc *desc)
{
	struct liointc_handler_data *handler = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct irq_chip_generic *gc = handler->priv->gc;
	int core = liointc_core_id % LIOINTC_NUM_CORES;
	u32 pending;

	chained_irq_enter(chip, desc);

	pending = readl(handler->priv->core_isr[core]);

	while (pending) {
		int bit = __ffs(pending);
		generic_handle_irq(irq_find_mapping(gc->domain, bit));
		pending &= ~BIT(bit);
	}

	chained_irq_exit(chip, desc);
}

static void liointc_suspend(struct irq_chip_generic *gc)
{
	struct liointc_priv *priv = gc->private;

	priv->int_pol = readl(gc->reg_base + LIOINTC_REG_INTC_POL);
	priv->int_edge = readl(gc->reg_base + LIOINTC_REG_INTC_EDGE);
}

static void liointc_resume(struct irq_chip_generic *gc)
{
	struct liointc_priv *priv = gc->private;
	int i;

	/* Disable all at first */
	writel(0xffffffff, gc->reg_base + LIOINTC_REG_INTC_DISABLE);
	/* Revert map cache */
	for (i = 0; i < LIOINTC_CHIP_IRQ; i++) {
		writeb(priv->map_cache[i], gc->reg_base + i);
	}

	writel(priv->int_pol, gc->reg_base + LIOINTC_REG_INTC_POL);
	writel(priv->int_edge, gc->reg_base + LIOINTC_REG_INTC_EDGE);

	/* Revert mask cache */
	writel(*gc->chip_types[0].mask_cache, gc->reg_base + LIOINTC_REG_INTC_ENABLE);
}

static void liointc_set_bit(struct irq_chip_generic *gc,
				unsigned int offset,
				u32 mask, bool set)
{
	if (set)
		writel(readl(gc->reg_base + offset) | mask,
				gc->reg_base + offset);
	else
		writel(readl(gc->reg_base + offset) & ~mask,
				gc->reg_base + offset);
}

static int liointc_set_type(struct irq_data *data, unsigned int type)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(data);
	u32 mask = data->mask;
	unsigned long flags;

	irq_gc_lock_irqsave(gc, flags);
	switch (type) {
	case IRQ_TYPE_LEVEL_HIGH:
		liointc_set_bit(gc, LIOINTC_REG_INTC_EDGE, mask, false);
		liointc_set_bit(gc, LIOINTC_REG_INTC_POL, mask, false);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		liointc_set_bit(gc, LIOINTC_REG_INTC_EDGE, mask, false);
		liointc_set_bit(gc, LIOINTC_REG_INTC_POL, mask, true);
		break;
	case IRQ_TYPE_EDGE_RISING:
		liointc_set_bit(gc, LIOINTC_REG_INTC_EDGE, mask, true);
		liointc_set_bit(gc, LIOINTC_REG_INTC_POL, mask, false);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		liointc_set_bit(gc, LIOINTC_REG_INTC_EDGE, mask, true);
		liointc_set_bit(gc, LIOINTC_REG_INTC_POL, mask, true);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		pr_warn("Not support both_edge interrupt, but requester can simulate it\n");
		break;
	default:
		irq_gc_unlock_irqrestore(gc, flags);
		return -EINVAL;
	}
	irq_gc_unlock_irqrestore(gc, flags);

	irqd_set_trigger_type(data, type);
	return 0;
}

static const char *const parent_names[] = {"int0", "int1", "int2", "int3"};
static const char *const core_reg_names[] = {"isr0", "isr1", "isr2", "isr3"};

static void __iomem *liointc_get_reg_byname(struct device_node *node,
						const char *name)
{
	int index = of_property_match_string(node, "reg-names", name);

	if (index < 0)
		return NULL;

	return of_iomap(node, index);
}

/*
 * Route */
int __init liointc_init(struct resource *res,
			int parent_irq_num,
			u32 *parent_irq,
			u32 *parent_int_map,
			struct fwnode_handle *domain_handle, int model)
{
	struct irq_chip_generic *gc;
	struct irq_domain *domain;
	struct irq_chip_type *ct;
	struct liointc_priv *priv;
	struct device_node *of_node;
	const char *chip_name;
	void __iomem *base;
	int i, err = 0;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		pr_err("%s: Unable to allocate memory\n", __func__);
		return -ENOMEM;
	}

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		pr_err("%s: ioremap error\n", __func__);
		err = -EINVAL;
		goto out_free_priv;
	}

	for (i = 0; i < LIOINTC_NUM_CORES; i++) {
		of_node = to_of_node(domain_handle);
		if (of_node) {
			priv->core_isr[i] = liointc_get_reg_byname(of_node,
							core_reg_names[i]);
			chip_name = of_node->full_name;
		} else {
			priv->core_isr[i] = base + LIOINTC_REG_INTC_STATUS(i);
			chip_name = "LIOINTC";
		}
	}

	if (!priv->core_isr[0]) {
		err = -ENODEV;
		goto out_iounmap;
	}

	for (i = 0; i < parent_irq_num; i++)
		priv->handler[i].parent_int_map = parent_int_map[i];

	domain = irq_domain_create_linear(domain_handle, LIOINTC_CHIP_IRQ,
					&irq_generic_chip_ops, priv);
	if (!domain) {
		pr_err("loongson-liointc: cannot add IRQ domain\n");
		err = -EINVAL;
		goto out_iounmap_isr;
	}

	err = irq_alloc_domain_generic_chips(domain, LIOINTC_CHIP_IRQ, 1,
					chip_name, handle_level_irq,
					0, IRQ_NOPROBE, 0);
	if (err) {
		pr_err("loongson-liointc: unable to register IRQ domain\n");
		err = -ENOMEM;
		goto out_free_domain;
	}

	/* Disable all IRQs */
	writel(0xffffffff, base + LIOINTC_REG_INTC_DISABLE);
	/* Set to level triggered */
	writel(0x0, base + LIOINTC_REG_INTC_EDGE);

	/* Generate parent INT part of map cache */
	for (i = 0; i < parent_irq_num; i++) {
		u32 pending = priv->handler[i].parent_int_map;

		while (pending) {
			int bit = __ffs(pending);

			priv->map_cache[bit] = BIT(i) << LIOINTC_SHIFT_INTx;
			pending &= ~BIT(bit);
		}
	}

	for (i = 0; i < LIOINTC_CHIP_IRQ; i++) {
		/* Generate core part of map cache */
		priv->map_cache[i] |= BIT(cpu_logical_map(0));
		writeb(priv->map_cache[i], base + i);
	}

	gc = irq_get_domain_generic_chip(domain, 0);
	gc->private = priv;
	gc->reg_base = base;
	gc->domain = domain;
	gc->suspend = liointc_suspend;
	gc->resume = liointc_resume;

	ct = gc->chip_types;
	ct->regs.enable = LIOINTC_REG_INTC_ENABLE;
	ct->regs.disable = LIOINTC_REG_INTC_DISABLE;
	ct->chip.irq_unmask = irq_gc_unmask_enable_reg;
	ct->chip.irq_mask = irq_gc_mask_disable_reg;
	ct->chip.irq_mask_ack = irq_gc_mask_disable_reg;
	ct->chip.irq_set_type = liointc_set_type;
	ct->chip.flags |= IRQCHIP_SKIP_SET_WAKE;

	gc->mask_cache = 0;
	priv->gc = gc;

	for (i = 0; i < parent_irq_num; i++) {
		if (parent_irq[i] <= 0)
			continue;
		if (parent_irq[i] == LOONGSON_BRIDGE_IRQ &&
				model == PCH_IRQ_ROUTE_EXT)
			continue;
		priv->handler[i].priv = priv;
		irq_set_chained_handler_and_data(parent_irq[i],
				liointc_chained_handle_irq, &priv->handler[i]);
	}

	liointc_handle = domain_handle;
	return 0;
out_free_domain:
	irq_domain_remove(domain);
out_iounmap_isr:
	for (i = 0; i < LIOINTC_NUM_CORES; i++) {
		if (!priv->core_isr[i])
			continue;
		iounmap(priv->core_isr[i]);
	}
out_iounmap:
	iounmap(base);
out_free_priv:
	kfree(priv);

	return err;
}

static int __init liointc_of_init(struct device_node *node,
				  struct device_node *parent)
{
	bool have_parent = FALSE;
	int sz, i, err = 0;
	struct resource res;
	u32 parent_irq[LIOINTC_NUM_PARENT];
	u32 parent_int_map[LIOINTC_NUM_PARENT];

	if (of_address_to_resource(node, 0, &res))
		return -EINVAL;

	for (i = 0; i < LIOINTC_NUM_PARENT; i++) {
		parent_irq[i] = of_irq_get_byname(node, parent_names[i]);
		if (parent_irq[i] > 0)
			have_parent = TRUE;
	}
	if (!have_parent)
		err = -ENODEV;

	sz = of_property_read_variable_u32_array(node,
						"loongson,parent_int_map",
						&parent_int_map[0],
						LIOINTC_NUM_PARENT,
						LIOINTC_NUM_PARENT);
	if (sz < 4) {
		pr_err("loongson-liointc: No parent_int_map\n");
		err = -ENODEV;
	}

	err = liointc_init(&res, LIOINTC_NUM_PARENT,
			parent_irq, parent_int_map, of_node_to_fwnode(node),
			get_irq_route_model());

	return err;
}

IRQCHIP_DECLARE(loongson_liointc_2_0, "loongson,liointc-2.0", liointc_of_init);

#ifdef CONFIG_ACPI
static int __init liointc_acpi_init_v1(struct acpi_subtable_header *header,
				   const unsigned long end)
{
	struct acpi_madt_lio_pic *lio_pic_entry;
	struct fwnode_handle *irq_handle;
	struct irq_fwspec fwspec;
	u32 parent_int_map[2];
	u32 parent_irq[2];
	int parent_irq_num;
	int i;
	struct resource res;
	lio_pic_entry = (struct acpi_madt_lio_pic *)header;

	irq_handle = irq_domain_alloc_named_fwnode("lio_pic");
	if (!irq_handle) {
		pr_err("Unable to allocate domain handle\n");
		return -ENOMEM;
	}

	parent_irq_num = 0;
	for (i = 0; i < 2; i++) {
		if (lio_pic_entry->cascade[i] > 0) {
			fwspec.fwnode = coreintc_get_fwnode();
			fwspec.param[0] = lio_pic_entry->cascade[i];
			fwspec.param_count = 1;
			parent_irq[i] = irq_create_fwspec_mapping(&fwspec);
			parent_int_map[i] = lio_pic_entry->cascade_map[i];
			parent_irq_num++;
		}
	}
	if (parent_irq_num == 0) {
		return -EINVAL;
	}
	res.start = lio_pic_entry->address;
	res.end = lio_pic_entry->address + lio_pic_entry->size;
	liointc_init(&res, parent_irq_num,
			parent_irq,
			parent_int_map,
			irq_handle, get_irq_route_model());
	return 0;
}
IRQCHIP_ACPI_DECLARE(liointc_v1, ACPI_MADT_TYPE_LIO_PIC,
		NULL, ACPI_MADT_LIO_PIC_VERSION_V1,
		liointc_acpi_init_v1);
#endif
