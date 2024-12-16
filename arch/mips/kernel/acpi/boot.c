/*
 *  boot.c - Architecture-Specific Low-Level ACPI Boot Support
 *
 *  Lvjianmin <lvjianmin@loongson.cn>
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/init.h>
#include <linux/acpi.h>
#include <linux/acpi_pmtmr.h>
#include <linux/efi.h>
#include <linux/stddef.h>
#include <linux/cpumask.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/slab.h>
#include <linux/bootmem.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/memblock.h>

#include <asm/pgtable.h>
#include <asm/io.h>
#include <asm/smp.h>
#include <asm/numa.h>
#include <loongson.h>
#include <loongson-pch.h>

static unsigned int num_processors;
int acpi_disabled = 0;
EXPORT_SYMBOL(acpi_disabled);
int acpi_noirq;				/* skip ACPI IRQ initialization */
int acpi_pci_disabled;		/* skip ACPI PCI scan and IRQ initialization */
EXPORT_SYMBOL(acpi_pci_disabled);
int acpi_strict;
extern void loongson_acpi_wakeup_start(void);
extern struct list_head acpi_wakeup_device_list;
u64 arch_acpi_wakeup_start;
u64 acpi_saved_sp_s3;
#define BAD_MADT_ENTRY(entry, end) (					    \
		(!entry) || (unsigned long)entry + sizeof(*entry) > end ||  \
		((struct acpi_subtable_header *)entry)->length < sizeof(*entry))

# define MAX_LOCAL_APIC 256

u64 acpi_liointc_addr __initdata = LIOINTC_DEFAULT_PHYS_BASE;

static inline void __iomem *loongson_acpi_os_ioremap(acpi_physical_address phys,
					    acpi_size size)
{
	return (void __iomem *)phys;
}
#define acpi_os_ioremap loonsonacpi_os_ioremap

#define PREFIX			"ACPI: "
/*
 * The default interrupt routing model is PIC (8259).  This gets
 * overridden if IOAPICs are enumerated (below).
 */
enum acpi_irq_model_id acpi_irq_model = ACPI_IRQ_MODEL_PIC;
void __iomem *__init __acpi_map_table(unsigned long phys, unsigned long size)
{

	if (!phys || !size)
		return NULL;

	return early_ioremap(phys, size);
}
void __init __acpi_unmap_table(void __iomem *map, unsigned long size)
{
	if (!map || !size)
		return;

	early_iounmap(map, size);
}
/*
 * Following __acpi_xx functions should be implemented for sepecific cpu. */
int acpi_gsi_to_irq(u32 gsi, unsigned int *irqp)
{
	if (irqp != NULL) *irqp = acpi_register_gsi(NULL, gsi, -1, -1);
	return 0;
}
EXPORT_SYMBOL_GPL(acpi_gsi_to_irq);

int acpi_isa_irq_to_gsi(unsigned isa_irq, u32 *gsi)
{
	if (gsi) *gsi = isa_irq;
	return 0;
}

#ifdef CONFIG_SUSPEND
static void fix_enable_wakeup_gpes(void)
{
	struct list_head *node, *next;
	u32 data = 0;

	list_for_each_safe(node, next, &acpi_wakeup_device_list) {
		struct acpi_device *dev =
			container_of(node, struct acpi_device, wakeup_list);

		if (!dev->wakeup.flags.valid
		    || ACPI_STATE_S3 > (u32) dev->wakeup.sleep_state
		    || !(device_may_wakeup(&dev->dev)
		        || dev->wakeup.prepare_count))
			continue;

		data |= (1 << dev->wakeup.gpe_number);
	}
	writel(data, LS7A_GPE0_ENA_REG);
}

int mips_acpi_suspend_lowlevel(void)
{
	mach_common_suspend();
	/* processor specific suspend */
	fix_enable_wakeup_gpes();
	loongson_suspend_enter();
	mach_common_resume();
	return 0;
}
#else
int mips_acpi_suspend_lowlevel(void)
{
	return 0;
}
#endif
#ifdef CONFIG_ACPI_SLEEP
int (*acpi_suspend_lowlevel)(void) = mips_acpi_suspend_lowlevel;
#else
int (*acpi_suspend_lowlevel)(void);
#endif
/*
 * success: return IRQ number (>=0)
 * failure: return < 0
 */
int acpi_register_gsi(struct device *dev, u32 gsi, int trigger, int polarity)
{

	if ((gsi >= GSI_MIN_LPC_IRQ &&
				gsi <= GSI_MAX_LPC_IRQ) ||
				(gsi >= GSI_MIN_MIPS_IRQ &&
				 gsi <= GSI_MAX_MIPS_IRQ)) {
		return gsi;
	} else if (gsi >= GSI_MIN_CPU_IRQ && gsi <= GSI_MAX_CPU_IRQ) {
		struct irq_fwspec fwspec;
		fwspec.fwnode = liointc_handle;
		fwspec.param[0] = gsi - GSI_MIN_CPU_IRQ;
		fwspec.param_count = 1;
		return irq_create_fwspec_mapping(&fwspec);
	} else if (gsi >= GSI_MIN_PCH_IRQ) {
#ifdef CONFIG_LOONGSON_PCH_PIC
		int id;
		struct fwnode_handle * handle;
		id = find_pch_pic(gsi);
		if (id < 0)
			return -1;
		handle = pch_pic_get_fwnode(id);
		if (handle) {
			struct pch_pic_gsi * gsic;
			struct irq_fwspec fwspec;
			gsic = pch_pic_gsi_routing(id);
			if (!gsic)
				return -1;
			fwspec.fwnode = handle;
			fwspec.param[0] = gsi;
			fwspec.param_count = 1;
			return irq_create_fwspec_mapping(&fwspec);
		} else
#endif
			return gsi;
	}
	return -1;
}
EXPORT_SYMBOL_GPL(acpi_register_gsi);

void acpi_unregister_gsi(u32 gsi)
{

}
EXPORT_SYMBOL_GPL(acpi_unregister_gsi);

/*
 *  ACPI based hotplug support for CPU
 */
#ifdef CONFIG_ACPI_HOTPLUG_CPU
#include <acpi/processor.h>

/* wrapper to silence section mismatch warning */
int __ref acpi_map_lsapic(acpi_handle handle, int physid, int *pcpu)
{
	return 0;
}
EXPORT_SYMBOL(acpi_map_lsapic);

int acpi_unmap_lsapic(int cpu)
{
	return 0;
}

EXPORT_SYMBOL(acpi_unmap_lsapic);
#endif				/* CONFIG_ACPI_HOTPLUG_CPU */

void __init acpi_boot_table_init(void)
{
	/*
	 * If acpi_disabled, bail out
	 */
	if (acpi_disabled)
		return;

	/*
	 * Initialize the ACPI boot-time table parser.
	 */
	if (acpi_table_init()) {
		disable_acpi();
		return;
	}
}

static int set_processor_mask(struct acpi_madt_local_apic *processor)
{

	int cpu;
	int cpuid = processor->id;

	if (!cpu_guestmode && (!(processor->lapic_flags & ACPI_MADT_ENABLED))) {
		return 0;
	}

	if (num_processors >= nr_cpu_ids) {
		pr_warning(
			"acpi: nr_cpus/possible_cpus limit of %i reached."
			"  processor 0x%x ignored.\n", nr_cpu_ids, cpuid);

	}
	if (cpuid == loongson_sysconf.boot_cpu_id) {
		cpu = 0;
	} else
		cpu = cpumask_next_zero(-1, cpu_present_mask);

	__cpu_number_map[cpuid] = cpu;
	__cpu_logical_map[cpu] = cpuid;
	set_cpu_possible(cpu, true);
	if (processor->lapic_flags & ACPI_MADT_ENABLED) {
		set_cpu_present(cpu, true);
		num_processors++;
	}
	loongson_sysconf.reserved_cpus_mask &= (~(1 << cpuid));
	return cpu;
}


static int __init
acpi_parse_lapic(struct acpi_subtable_header * header, const unsigned long end)
{
	struct acpi_madt_local_apic *processor = NULL;
	processor = (struct acpi_madt_local_apic *)header;

	if (BAD_MADT_ENTRY(processor, end))
		return -EINVAL;

	acpi_table_print_madt_entry(header);

	set_processor_mask(processor);
	return 0;
}

static int __init acpi_parse_madt_lapic_entries(void)
{
	int ret;
	struct acpi_subtable_proc madt_proc[1];

	memset(madt_proc, 0, sizeof(madt_proc));
	madt_proc[0].id = ACPI_MADT_TYPE_LOCAL_APIC;
	madt_proc[0].handler = acpi_parse_lapic;
	ret = acpi_table_parse_entries_array(ACPI_SIG_MADT,
				sizeof(struct acpi_table_madt),
				madt_proc, ARRAY_SIZE(madt_proc), MAX_LOCAL_APIC);
	if (ret < 0) {
		printk(KERN_ERR PREFIX
					"Error parsing LAPIC entries\n");
		return ret;
	}

	return 0;
}

static int __init acpi_parse_madt(struct acpi_table_header *table)
{
	struct acpi_table_madt *madt = NULL;

	madt = (struct acpi_table_madt *)table;
	if (!madt) {
		printk(KERN_WARNING PREFIX "Unable to map MADT\n");
		return -ENODEV;
	}

	if (madt->address) {
		acpi_liointc_addr = (u64) madt->address;
		printk(KERN_DEBUG PREFIX "Local I/O int controller address 0x%08x\n", madt->address);
	}

	return 0;
}


static int __init
acpi_parse_pch_pic(struct acpi_subtable_header * header, const unsigned long end)
{
	struct acpi_madt_io_apic *pch_pic = NULL;

	pch_pic = (struct acpi_madt_io_apic *)header;

	if (BAD_MADT_ENTRY(pch_pic, end))
		return -EINVAL;

	acpi_table_print_madt_entry(header);

	register_pch_pic(pch_pic->id, pch_pic->address, pch_pic->global_irq_base);

	return 0;
}
/*
 * Parse PCH_PIC related entries in MADT
 * returns 0 on success, < 0 on error
 */
static int __init acpi_parse_madt_pch_pic_entries(void)
{
	int count;

	/*
	 * ACPI interpreter is required to complete interrupt setup,
	 * so if it is off, don't enumerate the io-apics with ACPI.
	 * If MPS is present, it will handle them,
	 * otherwise the system will stay in PIC mode
	 */
	if (acpi_disabled || acpi_noirq)
		return -ENODEV;

	count = acpi_table_parse_madt(ACPI_MADT_TYPE_IO_APIC, acpi_parse_pch_pic, MAX_PCH_PICS);
	if (!count) {
		printk(KERN_ERR PREFIX "No PCH_PIC entries present\n");
		return -ENODEV;
	} else if (count < 0) {
		printk(KERN_ERR PREFIX "Error parsing PCH_PIC entry\n");
		return count;
	}

	return 0;
}
static void __init acpi_process_madt(void)
{
	int i, error;
	for (i = 0; i < NR_CPUS; i++) {
		__cpu_number_map[i] = -1;
		__cpu_logical_map[i] = -1;
	}
	loongson_sysconf.reserved_cpus_mask = 0xFFFF;

	if (acpi_table_parse(ACPI_SIG_MADT, acpi_parse_madt))
		return;
	/*
	 * Parse MADT LAPIC entries
	 */
	error = acpi_parse_madt_lapic_entries();
	if (!error) {
		/*
		 * Parse MADT IO-APIC entries
		 */

		acpi_parse_madt_pch_pic_entries();
		acpi_irq_model = ACPI_IRQ_MODEL_IOAPIC;
	}
	if (error == -EINVAL) {
		printk(KERN_ERR PREFIX
			       "Invalid BIOS MADT, disabling ACPI\n");
		disable_acpi();
	}

	loongson_sysconf.nr_cpus = num_processors;
	return;
}

int __init acpi_boot_init(void)
{
	/*
	 * If acpi_disabled, bail out
	 */
	if (acpi_disabled)
		return 1;

	loongson_sysconf.boot_cpu_id = read_c0_ebase() & 0x3ff;
#ifdef CONFIG_SUSPEND
	arch_acpi_wakeup_start = (u64)loongson_acpi_wakeup_start;
#endif
	/*
	 * Process the Multiple APIC Description Table (MADT), if present
	 */
	acpi_process_madt();
	return 0;
}

static int __init parse_acpi(char *arg)
{
	if (!arg)
		return -EINVAL;

	/* "acpi=off" disables both ACPI table parsing and interpreter */
	if (strcmp(arg, "off") == 0) {
		disable_acpi();
	} else {
		/* Core will printk when we return error. */
		return -EINVAL;
	}
	return 0;
}
early_param("acpi", parse_acpi);

/*
 * MIPS has no SMM, and bios will not acquire
 * the global lock to do something when kernel running, so
 * don't have to implement global lock.
 * __acpi_acquire_global_lock
 * will always return -1 indicating owning the lock.
 *
 * __acpi_release_global_lock will always return 0 indicating
 * no acquring request pending.
 *
 * */
int __acpi_acquire_global_lock(unsigned int *lock)
{
	return -1;
}

int __acpi_release_global_lock(unsigned int *lock)
{
	return 0;
}

unsigned long used_mem_item;
unsigned long used_mem_size;
extern struct loongsonlist_mem_map *loongson_mem_map;
void __init arch_reserve_mem_area(acpi_physical_address addr, size_t size)
{
	u64 mem_start, mem_end;
	u8 i, map_count = loongson_mem_map->map_count;

	for (i = 0; i < map_count; i++) {
		mem_start = loongson_mem_map->map[i].mem_start;
		mem_end = loongson_mem_map->map[i].mem_start + loongson_mem_map->map[i].mem_size;
		if (((addr >= mem_start) && (addr + size) < mem_end)) {
			loongson_mem_map->map[i].mem_size -= size;
			used_mem_size += size;
			used_mem_item = i;
			break;
		}
	}
	loongson_mem_map->map[map_count].mem_type = SYSTEM_RAM_RESERVED;
	loongson_mem_map->map[map_count].mem_start = addr;
	loongson_mem_map->map[map_count].mem_size = size;
	loongson_mem_map->map_count++;
}

#ifdef CONFIG_ACPI_NUMA

static __init int setup_node(int pxm)
{
	return acpi_map_pxm_to_node(pxm);
}


/*
 * Callback for SLIT parsing.  pxm_to_node() returns NUMA_NO_NODE for
 * I/O localities since SRAT does not list them.  I/O localities are
 * not supported at this point.
 */
extern unsigned char __node_distances[MAX_NUMNODES][MAX_NUMNODES];
unsigned int numa_distance_cnt;

static inline unsigned int get_numa_distances_cnt(struct acpi_table_slit *slit)
{
	return slit->locality_count;
}

void __init numa_set_distance(int from, int to, int distance)
{
	unsigned char *numa_distance = (unsigned char *)__node_distances;
	printk("run ACPI SLIT\n");
	if ((u8)distance != distance ||
			(from == to && distance != LOCAL_DISTANCE)) {
		pr_warn_once("Warning: invalid distance parameter, from=%d to=%d distance=%d\n",
				from, to, distance);
		return;
	}

	numa_distance[from * numa_distance_cnt + to] = distance;
}

void __init acpi_numa_slit_init(struct acpi_table_slit *slit)
{
	int i, j;

	numa_distance_cnt = get_numa_distances_cnt(slit);

	for (i = 0; i < slit->locality_count; i++) {
		const int from_node = pxm_to_node(i);

		if (from_node == NUMA_NO_NODE)
			continue;

		for (j = 0; j < slit->locality_count; j++) {
			const int to_node = pxm_to_node(j);

			if (to_node == NUMA_NO_NODE)
				continue;

			numa_set_distance(from_node, to_node,
					slit->entry[slit->locality_count * i + j]);
		}
	}
}

extern cpumask_t possible_cpu_per_node;
/* Callback for Proximity Domain -> CPUID mapping */
void __init
acpi_numa_processor_affinity_init(struct acpi_srat_cpu_affinity *pa)
{
	int pxm, node;

	if (srat_disabled())
		return;
	if (pa->header.length != sizeof(struct acpi_srat_cpu_affinity)) {
		bad_srat();
		return;
	}
	if ((pa->flags & ACPI_SRAT_CPU_ENABLED) == 0)
		return;
	pxm = pa->proximity_domain_lo;
	if (acpi_srat_revision >= 2) {
		pxm |= (pa->proximity_domain_hi[0] << 8);
		pxm |= (pa->proximity_domain_hi[1] << 16);
		pxm |= (pa->proximity_domain_hi[2] << 24);
	}
	node = setup_node(pxm);
	if (node < 0) {
		printk(KERN_ERR "SRAT: Too many proximity domains %x\n", pxm);
		bad_srat();
		return;
	}

	if (pa->apic_id >= CONFIG_NR_CPUS) {
		printk(KERN_INFO "SRAT: PXM %u -> CPU 0x%02x -> Node %u skipped apicid that is too big\n", pxm, pa->apic_id, node);
		return;
	}

	if (!cpu_guestmode)
		numa_add_cpu(__cpu_number_map[pa->apic_id], node);
	else
		numa_add_cpu(pa->apic_id, node);

	set_cpuid_to_node(pa->apic_id, node);
	node_set(node, numa_nodes_parsed);
	acpi_numa = 1;
	printk(KERN_INFO "SRAT: PXM %u -> CPU 0x%02x -> Node %u\n",
		pxm, pa->apic_id, node);
}

#ifdef CONFIG_MEMORY_HOTPLUG
static inline int save_add_info(void) {return 1;}
#else
static inline int save_add_info(void) {return 0;}
#endif

/* Callback for parsing of the Proximity Domain <-> Memory Area mappings */
int __init
acpi_numa_memory_affinity_init(struct acpi_srat_mem_affinity *ma)
{
	u64 start, end;
	u32 hotpluggable;
	int node, pxm;

	if (srat_disabled())
		goto out_err;
	if (ma->header.length != sizeof(struct acpi_srat_mem_affinity))
		goto out_err_bad_srat;
	if ((ma->flags & ACPI_SRAT_MEM_ENABLED) == 0)
		goto out_err;
	hotpluggable = ma->flags & ACPI_SRAT_MEM_HOT_PLUGGABLE;
	if (hotpluggable && !save_add_info())
		goto out_err;

	start = ma->base_address;
	end = start + ma->length;
	pxm = ma->proximity_domain;
	if (acpi_srat_revision <= 1)
		pxm &= 0xff;

	node = setup_node(pxm);
	if (node < 0) {
		printk(KERN_ERR "SRAT: Too many proximity domains.\n");
		goto out_err_bad_srat;
	}
	if (numa_add_memblk(node, start, end) < 0)
		goto out_err_bad_srat;

	node_set(node, numa_nodes_parsed);

	pr_info("SRAT: Node %u PXM %u [mem %#010Lx-%#010Lx]%s%s\n",
		node, pxm,
		(unsigned long long) start, (unsigned long long) end - 1,
		hotpluggable ? " hotplug" : "",
		ma->flags & ACPI_SRAT_MEM_NON_VOLATILE ? " non-volatile" : "");

	/* Mark hotplug range in memblock. */
	if (hotpluggable && memblock_mark_hotplug(start, ma->length))
		pr_warn("SRAT: Failed to mark hotplug range [mem %#010Lx-%#010Lx] in memblock\n",
			(unsigned long long)start, (unsigned long long)end - 1);

	max_possible_pfn = max(max_possible_pfn, PFN_UP(end - 1));

	return 0;
out_err_bad_srat:
	bad_srat();
out_err:
	return -1;
}

void __init acpi_numa_arch_fixup(void) {}
#endif

#ifdef CONFIG_ACPI_HOTPLUG_CPU
#include <acpi/processor.h>
static int acpi_map_cpu2node(acpi_handle handle, int cpu, int physid)
{
#ifdef CONFIG_ACPI_NUMA
	int nid;

	nid = acpi_get_node(handle);
	if (nid != NUMA_NO_NODE) {
		set_cpuid_to_node(cpu, nid);
		node_set(nid, numa_nodes_parsed);
	}
#endif
	return 0;
}

int acpi_map_cpu(acpi_handle handle, phys_cpuid_t physid, u32 acpi_id,
		 int *pcpu)
{
	int cpu;

	struct acpi_madt_local_apic *processor = kzalloc(sizeof(struct acpi_madt_local_apic), GFP_KERNEL);;

	processor->id = physid;
	processor->processor_id = acpi_id;
	processor->lapic_flags = ACPI_MADT_ENABLED;

	cpu = set_processor_mask(processor);
	if (cpu < 0) {
		pr_info(PREFIX "Unable to map lapic to logical cpu number\n");
		return cpu;
	}

	acpi_map_cpu2node(handle, cpu, physid);

	*pcpu = cpu;
	return 0;
}
EXPORT_SYMBOL(acpi_map_cpu);

int acpi_unmap_cpu(int cpu)
{
#ifdef CONFIG_ACPI_NUMA
	set_cpuid_to_node(cpu, NUMA_NO_NODE);
#endif
	set_cpu_present(cpu, false);
	num_processors--;

	pr_info("cpu%d hot remove!\n",cpu);

	return (0);
}
EXPORT_SYMBOL(acpi_unmap_cpu);
#endif				/* CONFIG_ACPI_HOTPLUG_CPU */

