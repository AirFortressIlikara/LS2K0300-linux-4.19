/*
 *	Support irq for Loongson chipsets.
 *
 *	lvjianmin <lvjianmin@loongson.cn>
 *
 */

#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/compiler.h>
#include <linux/stddef.h>
#include <linux/acpi.h>
#include <loongson.h>
#include <irq.h>
#include <linux/module.h>
#include <linux/syscore_ops.h>
#include <asm/irq_cpu.h>
#include <asm/mipsregs.h>
#include <loongson.h>
#include <loongson-pch.h>
#include <linux/irqdomain.h>
#include <linux/irqchip.h>

#define	for_each_pch_pic(idx)		\
	for ((idx) = 0; (idx) < nr_pch_pics; (idx)++)
#define	for_each_pin(idx, pin)		\
	for ((pin) = 0; (pin) < pch_pics[(idx)].nr_registers; (pin)++)
#define	for_each_pch_pic_pin(idx, pin)	\
	for_each_pch_pic((idx))		\
		for_each_pin((idx), (pin))
#define MSI_IRQ_NR_64		64
#define MSI_IRQ_NR_192		192
#define MSI_IRQ_NR_224		224

#define LIOINTC_MEM_SIZE	0x80
#define LIOINTC_VECS_TO_IP2	0x00FFFFFE /* others */
#define LIOINTC_VECS_TO_IP3	0xFF000000 /* HT1 0-7 */
#define PCH_PIC_SIZE		0x400
extern const struct plat_smp_ops *mp_ops;
extern u64 acpi_liointc_addr;
extern void (*do_cascade)(void);
extern void do_pch(void);
/* The one past the highest gsi number used */
u32 gsi_top;
static int nr_pch_pics;
static int msi_irqbase;
static enum pch_irq_route_model_id pch_irq_route_model = PCH_IRQ_ROUTE_EXT;
enum pch_irq_route_model_id get_irq_route_model(void)
{
	return pch_irq_route_model;
}

static struct pch_pic {
	/*
	 * # of IRQ routing registers
	 */
	int nr_registers;

	/* pch pic config */
	struct pch_pic_config config;
	/* pch pic gsi routing info */
	struct pch_pic_gsi  gsi_config;
} pch_pics[MAX_PCH_PICS];

static int __init pch_pic_get_redir_entries(int pch_pic)
{
	if (cpu_guestmode)
		return 32;
	return 64;
}
struct pch_pic_gsi *pch_pic_gsi_routing(int pch_pic_idx)
{
	return &pch_pics[pch_pic_idx].gsi_config;
}

int pch_pic_id(int pch_pic_idx)
{
	return pch_pics[pch_pic_idx].config.pch_pic_id;
}

unsigned long pch_pic_addr(int pch_pic_idx)
{
	return pch_pics[pch_pic_idx].config.pch_pic_addr;
}

static __init int bad_pch_pic(unsigned long address)
{
	if (nr_pch_pics >= MAX_PCH_PICS) {
		pr_warn("WARNING: Max # of I/O PCH_PICs (%d) exceeded (found %d), skipping\n",
			MAX_PCH_PICS, nr_pch_pics);
		return 1;
	}
	if (!address) {
		pr_warn("WARNING: Bogus (zero) I/O PCH_PIC address found in table, skipping!\n");
		return 1;
	}
	return 0;
}
#define pch_pic_ver(pch_pic_idx)	pch_pics[pch_pic_idx].config.pch_pic_ver
void __init register_pch_pic(int id, u64 address, u32 gsi_base)
{
	int idx = 0;
	int entries;
	struct pch_pic_gsi *gsi_cfg;

	if (bad_pch_pic(address))
		return;

	idx = nr_pch_pics;
	if (cpu_guestmode)
		pch_pics[idx].config.pch_pic_addr =
				LS7A_VZ_PCH_REG_BASE | address;
	else
		pch_pics[idx].config.pch_pic_addr = address;

	pch_pics[idx].config.pch_pic_id = id;
	pch_pics[idx].config.pch_pic_ver = 0;

	/*
	 * Build basic GSI lookup table to facilitate lookups
	 * and to prevent reprogramming of PCH_PIC pins (PCI GSIs).
	 */
	entries = pch_pic_get_redir_entries(idx);
	gsi_cfg = pch_pic_gsi_routing(idx);
	gsi_cfg->gsi_base = gsi_base;
	gsi_cfg->gsi_end = gsi_base + entries - 1;
	/*
	 * The number of PCH_PIC IRQ registers (== #pins):
	 */
	pch_pics[idx].nr_registers = entries;

	if (gsi_cfg->gsi_end >= gsi_top)
		gsi_top = gsi_cfg->gsi_end + 1;

	pr_info("PCH_PIC[%d]: pch_pic_id %d, version %d, address 0x%lx, GSI %d-%d\n",
		idx, pch_pic_id(idx),
		pch_pic_ver(idx), pch_pic_addr(idx),
		gsi_cfg->gsi_base, gsi_cfg->gsi_end);

	nr_pch_pics++;
	msi_irqbase += entries;
}

void handle_virq(unsigned int irq, unsigned int cpu)
{
	struct irq_data *irqd;
	struct cpumask affinity;

	/* handled by local core */
	if (ipi_irq2pos[irq] == -1) {
		generic_handle_irq(irq);
		return;
	}

	irqd = irq_get_irq_data(irq);
	cpumask_and(&affinity, irqd->common->affinity, cpu_active_mask);
	if (cpumask_empty(&affinity)) {
		generic_handle_irq(irq);
		return;
	}

	irq_cpu[irq] = cpumask_next(irq_cpu[irq], &affinity);
	if (irq_cpu[irq] >= nr_cpu_ids)
		irq_cpu[irq] = cpumask_first(&affinity);

	if (irq_cpu[irq] == cpu) {
		generic_handle_irq(irq);
		return;
	}

	/* balanced by other cores */
	mp_ops->send_ipi_single(irq_cpu[irq], (0x1 << (ipi_irq2pos[irq])) << IPI_IRQ_OFFSET);
}

void __init of_setup_pch_irqs(int model)
{
	unsigned int pch_pic, pin;
	struct device_node *of_node;
	struct pch_pic_gsi *gsi_cfg = pch_pic_gsi_routing(0);
	if (!gsi_cfg)
		return;

	of_node = of_find_compatible_node(NULL, NULL, "loongson,ls7a-interrupt-controller");
	if (of_node) {
		pch_pic_init(of_node_to_fwnode(of_node),
				pch_pic_addr(0),
				PCH_PIC_SIZE,
				model,
				gsi_cfg->gsi_base);
	}

	for_each_pch_pic_pin(pch_pic, pin) {
		struct irq_fwspec fwspec;
		fwspec.fwnode = NULL;
		fwspec.param[0] = pin + gsi_cfg->gsi_base;
		fwspec.param_count = 1;
		irq_create_fwspec_mapping(&fwspec);
	}
}
void __init setup_pch_irqs(int model)
{
	unsigned int pch_pic, pin;
	struct fwnode_handle *irq_handle;
	int count;

	for_each_pch_pic(pch_pic) {
		struct pch_pic_gsi *gsi_cfg = pch_pic_gsi_routing(pch_pic);
		if (!gsi_cfg)
			continue;
#ifdef CONFIG_LOONGSON_PCH_PIC
		irq_handle = irq_domain_alloc_fwnode((void *)acpi_liointc_addr);
		if (!irq_handle) {
			panic("Unable to allocate domain handle for liointc irqdomain.\n");
		}
		count = gsi_cfg->gsi_end - gsi_cfg->gsi_base + 1;
		pch_pic_init(irq_handle,
				pch_pic_addr(pch_pic),
				PCH_PIC_SIZE,
				model,
				gsi_cfg->gsi_base);
#endif
	}
	for_each_pch_pic_pin(pch_pic, pin) {
		struct pch_pic_gsi *gsi_cfg = pch_pic_gsi_routing(pch_pic);
		acpi_register_gsi(NULL, pin + gsi_cfg->gsi_base, -1, -1);
	}
}
extern struct system_loongson *esys;
static bool fw_support_fdt(void)
{
	return !!(esys && esys->vers >= 2 && esys->of_dtb_addr);
}

static void pch_pic_lpc_init(void)
{
	struct device_node *np;

	if (fw_support_fdt()) {
		np = of_find_compatible_node(NULL, NULL, "simple-bus");
		if (np) {
			if (of_property_read_bool(np, "enable-lpc-irq")) {
				if (of_property_read_bool(np, "lpc-irq-low")) {
					writel(0, LS7A_LPC_INT_POL);
				} else {
					writel(-1, LS7A_LPC_INT_POL);
				}

			}
		}
	}
}
extern void pci_no_msi(void);
static enum pch_irq_route_model_id init_pch_irq_model(void)
{
	if ((loongson_cpu_has_msi128 || loongson_cpu_has_msi256)) {
		if (loongson_cpu_has_extioi) {
			pch_irq_route_model = PCH_IRQ_ROUTE_EXT;
		} else {
			pch_irq_route_model = PCH_IRQ_ROUTE_HT;
		}
	} else {
		pci_no_msi();
		pch_irq_route_model = PCH_IRQ_ROUTE_LINE;
	}
	return pch_irq_route_model;
}

void pch_msi_domain_init(int start, int count)
{
	struct fwnode_handle *irq_handle, *parent_handle;
	u64 msg_address;

	msg_address = loongson_sysconf.msi_address_lo;
	msg_address |= ((u64)loongson_sysconf.msi_address_hi << 32);
	irq_handle = irq_domain_alloc_fwnode((void *)msg_address);
	if (!irq_handle) {
		panic("Unable to allocate domain handle  for pch_msi irqdomain.\n");
	}

	if (get_irq_route_model() == PCH_IRQ_ROUTE_EXT) {
		parent_handle = eiointc_get_fwnode(0);
	} else {
		parent_handle = htvec_get_fwnode();
	}

	pch_msi_init(irq_handle, parent_handle,
			msg_address,
			get_irq_route_model() == PCH_IRQ_ROUTE_EXT,
			start,
			count);
}

void pch_lpc_domain_init(void)
{
#ifdef CONFIG_LOONGSON_PCH_LPC
	struct fwnode_handle *irq_handle;
	struct device_node *np;
	struct irq_fwspec fwspec;
	int parent_irq;

	fwspec.fwnode = NULL;
	fwspec.param[0] = LS7A_LPC_CASCADE_IRQ;
	fwspec.param_count = 1;
	parent_irq = irq_create_fwspec_mapping(&fwspec);

	irq_handle = irq_domain_alloc_fwnode((void *)((u64)LS7A_LPC_INT_BASE));
	if (!irq_handle) {
		panic("Unable to allocate domain handle for pch_lpc irqdomain.\n");
	}

	np = of_find_compatible_node(NULL, NULL, "simple-bus");
	if (np) {
		if (of_property_read_bool(np, "enable-lpc-irq"))
			pch_lpc_init(LS7A_LPC_INT_BASE,
				LS7A_LPC_INT_SIZE,
				parent_irq,
				irq_handle);
	} else {
		pch_lpc_init(LS7A_LPC_INT_BASE,
			LS7A_LPC_INT_SIZE,
			parent_irq,
			irq_handle);
	}
#endif
}
void __init setup_PCH_PIC(void)
{
	int i, start, count;
	enum pch_irq_route_model_id model = PCH_IRQ_ROUTE_END;
	struct device_node *np;
	struct resource res;
	u64 res_array[2];
	int num_parents;
	struct irq_fwspec fwspec;
	struct fwnode_handle *irq_fwnode;
	u32 parent_int_map[2] = {LIOINTC_VECS_TO_IP2, LIOINTC_VECS_TO_IP3};
	u32 parent_irq[2] = {LOONGSON_LINTC_IRQ, LOONGSON_BRIDGE_IRQ};
	u32 ht_parent_irq[8];;

	model = init_pch_irq_model();
	irq_alloc_descs(-1, MIPS_CPU_IRQ_BASE, 8, 0);
	for (i = MIPS_CPU_IRQ_BASE; i < MIPS_CPU_IRQ_BASE + 8; i++)
		irq_set_noprobe(i);
	do_cascade = do_pch;
	mips_cpu_irq_init();
	start = msi_irqbase;
	if (loongson_cpu_has_msi256) {
		count = 256 - start;
		num_parents = 8;
	} else {
		count = 128 - start;
		num_parents = 4;
	}
#ifdef CONFIG_LOONGSON_LIOINTC
	np = of_find_compatible_node(NULL, NULL, "loongson,liointc");
	if (np) {
		of_property_read_u64_array(np, "reg", res_array, 2);
		res.start = res_array[0];
		res.end = res_array[0] + res_array[1];
		liointc_handle = of_fwnode_handle(np);
		liointc_init(&res,
			2,
			parent_irq,
			parent_int_map,
			of_node_to_fwnode(np), model);
	} else {
		irq_fwnode = irq_domain_alloc_fwnode((void *)acpi_liointc_addr);
		if (!irq_fwnode) {
			panic("Unable to allocate domain handle for liointc irqdomain.\n");
		}
		res.start = acpi_liointc_addr;
		res.end = acpi_liointc_addr + LIOINTC_MEM_SIZE;
		liointc_handle = irq_fwnode;
		liointc_init(&res,
			2,
			parent_irq,
			parent_int_map,
			irq_fwnode, model);
	}
#endif
	switch (model) {
	case PCH_IRQ_ROUTE_EXT:
		pr_info("Support EXT interrupt.\n");
#ifdef CONFIG_LOONGSON_EXTIOI
		irq_fwnode = irq_domain_alloc_fwnode((void *)((u64)LOONGSON_CSR_EXTIOI_ISR_BASE));
		if (!irq_fwnode) {
			panic("Unable to allocate domain handle for eiointc irqdomain.\n");
		}
		extioi_vec_init(irq_fwnode, LOONGSON_BRIDGE_IRQ, CSR_EXTIOI_VECTOR_NUM, 0, 0, 0, 0);
#endif
		if (acpi_disabled)
			of_setup_pch_irqs(model);
		else
			setup_pch_irqs(model);
		pch_msi_domain_init(start, count);
		break;
	case PCH_IRQ_ROUTE_HT:
		pr_info("Support HT interrupt.\n");
#ifdef CONFIG_LOONGSON_HTVEC
		for (i = 0; i < num_parents; i++) {
			fwspec.fwnode = liointc_get_fwnode();
			fwspec.param[0] = LOONGSON_CPU_HT1_VEC + i;
			fwspec.param_count = 1;
			ht_parent_irq[i] = irq_create_fwspec_mapping(&fwspec);
		}
		irq_fwnode = irq_domain_alloc_named_fwnode("htintc");
		if (!irq_fwnode) {
			panic("Unable to allocate domain handle for eiointc irqdomain.\n");
		}
		htvec_init(LOONGSON_HT1_INT_VECTOR_BASE,
				num_parents,
				ht_parent_irq,
				irq_fwnode);
#endif
		if (acpi_disabled)
			of_setup_pch_irqs(model);
		else
			setup_pch_irqs(model);
		pch_msi_domain_init(start, count);
		break;
	case PCH_IRQ_ROUTE_LINE:
		pr_info("Support LINE interrupt.\n");
		if (acpi_disabled)
			of_setup_pch_irqs(model);
		else
			setup_pch_irqs(model);
		break;
	default:
		BUG_ON(true);
	}
	pch_lpc_domain_init();
	pch_pic_lpc_init();
}
