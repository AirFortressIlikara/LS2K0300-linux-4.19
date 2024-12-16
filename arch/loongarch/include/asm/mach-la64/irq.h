/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_MACH_LOONGSON64_IRQ_H_
#define __ASM_MACH_LOONGSON64_IRQ_H_

#include <asm/mach-la64/boot_param.h>

#define MAX_PCH_PICS			16
#define MAX_MSI_PICS			16
#define MAX_EIO_PICS			16
#define LOONGSON_CPU_UART0_VEC		10 /* CPU UART0 */
#define LOONGSON_CPU_THSENS_VEC		14 /* CPU Thsens */
#define LOONGSON_CPU_HT0_VEC		16 /* CPU HT0 irq vector base number */
#define NR_IRQS				(64 + (256 * MAX_EIO_PICS))

#define LIOINTC_MEM_SIZE		0x80
#define LIOINTC_VECS_TO_IP2		0x00FFFFFE /* others */
#define LIOINTC_VECS_TO_IP3		0xFF000000 /* HT1 0-7 */

#define PCH_PIC_SIZE			0x400

#define MSI_ADDRESS_LO			0x2FF00000
#define MSI_ADDRESS_HI			0
#define MSI_IRQ_NR_64			64
#define MSI_IRQ_NR_192			192
#define MSI_IRQ_NR_224			224


/* IRQ number definitions */
/*lpc*/
#define LOONGSON_LPC_IRQ_BASE		0
#define LOONGSON_LPC_IRQ_SIZE		16
#define LOONGSON_LPC_IRQ_END		(LOONGSON_LPC_IRQ_BASE + LOONGSON_LPC_IRQ_SIZE - 1)

/*legacy IO*/
#define LOONGSON_LIO_IRQ_BASE		16
#define LOONGSON_LIO_IRQ_SIZE		32
#define LOONGSON_LIO_IRQ_END		(LOONGSON_LIO_IRQ_BASE + LOONGSON_LIO_IRQ_SIZE - 1)

/*cpu core*/
#define LOONGARCH_CPU_IRQ_BASE		50
#define LOONGARCH_CPU_IRQ_SIZE		14
#define LOONGSON_LINTC_IRQ		(LOONGARCH_CPU_IRQ_BASE + 2) /* IP2 for CPU legacy I/O interrupt controller */
#define LOONGSON_BRIDGE_IRQ		(LOONGARCH_CPU_IRQ_BASE + 3) /* IP3 for bridge */
#define LOONGSON_TIMER_IRQ		(LOONGARCH_CPU_IRQ_BASE + 11) /* IP11 CPU Timer */
#define LOONGARCH_CPU_IRQ_END		(LOONGARCH_CPU_IRQ_BASE + LOONGARCH_CPU_IRQ_SIZE - 1)

/*chipset*/
#define LOONGSON_PCH_IRQ_BASE		64
#define LOONGSON_PCH_IRQ_SIZE		64
#define LOONGSON_PCH_IRQ_END		(LOONGSON_PCH_IRQ_BASE + LOONGSON_PCH_IRQ_SIZE - 1)


#define LOONGSON_HT1_CFG_BASE		0x80000EFDFB000000
#define LOONGSON_HT1_INT_VECTOR_BASE	(LOONGSON_HT1_CFG_BASE + 0x80)

#define GSI_MIN_PCH_IRQ			LOONGSON_PCH_IRQ_BASE
#define GSI_MIN_CPU_IRQ			16
#define GSI_MAX_CPU_IRQ			47

/* Both flat mode and compatible mode treat 4 cores as 1 extioi node */
#define CORES_PER_EXTIOI_NODE 4

/* One node in guest mode may contain MAX_CPUS cores */
#define MAX_CORES_PER_EIO_NODE	NR_CPUS

/*
 * Define pch I/O interrupt route model
 *
 **/
enum pch_irq_route_model_id {
	PCH_IRQ_ROUTE_LINE,	/* route to interrupt line on CPU legacy I/O interrupt controller*/
	PCH_IRQ_ROUTE_HT,	/* route to ht controller */
	PCH_IRQ_ROUTE_EXT,	/* route to CPU ext I/O interrupt controller through ht controller*/
	PCH_IRQ_ROUTE_EXT_GUEST,/* guest ext I/O interrupt controller*/
	PCH_IRQ_ROUTE_EXT_SOC,	/* soc ext I/O interrupt controller*/
	PCH_IRQ_ROUTE_END
};

static inline int cpu_to_eio_node_core(int cpu)
{
	int cores = cpu_has_hypervisor ? MAX_CORES_PER_EIO_NODE : CORES_PER_EXTIOI_NODE;

	return cpu_logical_map(cpu) % cores;
}

static inline int cpu_to_eio_node(int cpu)
{
	int cores = cpu_has_hypervisor ? MAX_CORES_PER_EIO_NODE : CORES_PER_EXTIOI_NODE;

	return cpu_logical_map(cpu) / cores;
}

static inline int eio_node_to_cpu(int node)
{
	int cores = cpu_has_hypervisor ? MAX_CORES_PER_EIO_NODE : CORES_PER_EXTIOI_NODE;

	return node * cores;
}

static inline int create_ipi_dirq(unsigned int irq)
{
	return 0;
}
static inline void destroy_ipi_dirq(unsigned int irq)
{

}
static inline int def_set_irq_affinity(struct irq_data *d, const struct cpumask *affinity, bool force)
{
	return 0;
}

struct pch_pic_irq {
	u32 irq_base;
	u32 irq_end;
};

extern u32 irq_top;
struct pch_pic_config {
	unsigned char pch_pic_id;
	unsigned char pch_pic_ver;
	unsigned long pch_pic_addr;
};

/*
 * # of PCH_PICs and # of IRQ routing registers
 */
extern int nr_pch_pics;
extern int pch_pic_id(int pch_pic);
extern int find_pch_pic(u32 gsi);
extern unsigned long pch_pic_addr(int pch_pic);
extern struct pch_pic_irq *pch_pic_irq_routing(int pch_pic);
extern void __init register_pch_pic(int id, u32 address, u32 irq_base);
extern int __init liointc_init(struct resource *res,
			int parent_irq_num,
			u32 *parent_irq,
			u32 *parent_int_map,
			struct fwnode_handle *domain_handle, int model);
extern int pch_msi_init(struct fwnode_handle *irq_handle,
		struct fwnode_handle *parent_handle,
		u64 msg_address,
		bool ext, int start, int count);
extern int htvec_init(unsigned long addr,
		unsigned int num_parents,
		unsigned int *parent_irq,
		struct fwnode_handle *irq_handle);
extern int extioi_vec_init(struct fwnode_handle *fwnode, int cascade_irq,
				u32 vec_count, u32 misc_func, u32 eio_en_off, u64 node_map, u32 node);
extern int pch_pic_init(struct fwnode_handle *irq_handle, unsigned long addr,
		unsigned int size,
		int model, int gsi_base);
extern void fixup_irqs(void);
extern void extioi_init(void);
extern void loongson3_ipi_interrupt(int irq);
extern void pmu_handle_irq(int irq);
extern int pch_lpc_init(u64 address, u16 size,
		int parent_irq,
		struct fwnode_handle *irq_handle);
extern void handle_virq(unsigned int irq, unsigned int cpu);
extern u32 pchintc_gsi_base(int id);
extern int eiointc_get_node(int id);
extern struct fwnode_handle *coreintc_get_fwnode(void);
extern struct fwnode_handle *eiointc_get_fwnode(int id);
extern struct fwnode_handle *pch_pic_get_fwnode(int id);
extern struct fwnode_handle *liointc_get_fwnode(void);
extern struct fwnode_handle *htvec_get_fwnode(void);
extern struct fwnode_handle *pch_lpc_get_fwnode(void);
extern enum pch_irq_route_model_id get_irq_route_model(void);

#endif /* __ASM_MACH_LOONGSON64_IRQ_H_ */
