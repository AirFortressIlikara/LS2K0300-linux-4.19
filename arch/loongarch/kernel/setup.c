// SPDX-License-Identifier: GPL-2.0
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/export.h>
#include <linux/screen_info.h>
#include <linux/memblock.h>
#include <linux/bootmem.h>
#include <linux/initrd.h>
#include <linux/root_dev.h>
#include <linux/highmem.h>
#include <linux/console.h>
#include <linux/pfn.h>
#include <linux/debugfs.h>
#include <linux/kexec.h>
#include <linux/sizes.h>
#include <linux/device.h>
#include <linux/dma-contiguous.h>
#include <linux/decompress/generic.h>
#include <linux/of_fdt.h>
#include <linux/crash_dump.h>
#include <linux/suspend.h>
#include <asm/addrspace.h>
#include <asm/bootinfo.h>
#include <asm/cache.h>
#include <asm/cpu.h>
#include <asm/debug.h>
#include <asm/dma.h>
#include <asm/numa.h>
#include <asm/pgalloc.h>
#include <asm/sections.h>
#include <asm/setup.h>
#include <asm/smp.h>
#include <asm/unwind.h>

struct cpuinfo_loongarch cpu_data[NR_CPUS] __read_mostly;

EXPORT_SYMBOL(cpu_data);

#ifdef CONFIG_VT
struct screen_info screen_info;
#endif

static phys_addr_t crashmem_start, crashmem_size;

/*
 * Setup information
 *
 * These are initialized so they are in the .data section
 */
static char __initdata command_line[COMMAND_LINE_SIZE];
char __initdata arcs_cmdline[COMMAND_LINE_SIZE];

#ifdef CONFIG_CMDLINE_BOOL
static char __initdata builtin_cmdline[COMMAND_LINE_SIZE] = CONFIG_CMDLINE;
#endif

static struct resource code_resource = { .name = "Kernel code", };
static struct resource data_resource = { .name = "Kernel data", };
static struct resource bss_resource = { .name = "Kernel bss", };

/*
 * Manage initrd
 */
#ifdef CONFIG_BLK_DEV_INITRD
static void __init reserve_initrd_mem(void)
{
	phys_addr_t phys_initrd_start;
	phys_addr_t start;
	unsigned long initrd_size, size;


	/* The small fdt method should be skipped directly to avoid two reserved operations. */
	if (fw_arg2 == 0)
		return;

	initrd_size = initrd_end - initrd_start;

	if (!initrd_size)
		return;

	phys_initrd_start = __pa(initrd_start);

	/*
	 * Round the memory region to page boundaries as per free_initrd_mem()
	 * This allows us to detect whether the pages overlapping the initrd
	 * are in use, but more importantly, reserves the entire set of pages
	 * as we don't want these pages allocated for other purposes.
	 */
	start = round_down(phys_initrd_start, PAGE_SIZE);
	size = initrd_size + (phys_initrd_start - start);
	size = round_up(size, PAGE_SIZE);

	if (!memblock_is_region_memory(start, size)) {
		pr_err("INITRD: 0x%08llx+0x%08lx is not a memory region",
		       (u64)start, size);
		goto disable;
	}

	if (memblock_is_region_reserved(start, size)) {
		pr_err("INITRD: 0x%08llx+0x%08lx overlaps in-use memory region\n",
		       (u64)start, size);
		goto disable;
	}

	memblock_reserve(start, size);
	initrd_below_start_ok = 1;

	return;
disable:
	pr_cont(" - disabling initrd\n");
	initrd_start = 0;
	initrd_end = 0;
}

static int __init early_initrd(char *p)
{
	unsigned long start, size;
	char *endp;

	start = memparse(p, &endp);
	if (*endp == ',') {
		size = memparse(endp + 1, NULL);

		if (start + size > PFN_PHYS(max_low_pfn)) {
			pr_err(KERN_INFO "Initrd physical address is out of memory!");
			return 0;
		}

		initrd_start = (unsigned long)__va(start);
		initrd_end = initrd_start + size;
	}

	return 0;
}
early_param("initrd", early_initrd);

static int __init rd_start_early(char *p)
{
	initrd_start = memparse(p, &p);

	return 0;
}
early_param("rd_start", rd_start_early);

static int __init rd_size_early(char *p)
{
	unsigned long size;

	size = memparse(p, &p);
	initrd_end = initrd_start + size;

	return 0;
}
early_param("rd_size", rd_size_early);

#else  /* !CONFIG_BLK_DEV_INITRD */
static unsigned long __init init_initrd(void)
{
	return 0;
}
#endif
static int loongson_syscall = true;
static int __init loongson_syscall_disable(char *str)
{
	loongson_syscall = false;
	return 1;
}
early_param("loongson_syscall_disable", loongson_syscall_disable);

#ifdef CONFIG_ARCH_WRITECOMBINE
pgprot_t pgprot_wc = PAGE_KERNEL_WUC;
#else
pgprot_t pgprot_wc = PAGE_KERNEL_SUC;
#endif

EXPORT_SYMBOL(pgprot_wc);

static int __init setup_writecombine(char *p)
{
	if (!strcmp(p, "on"))
		pgprot_wc = PAGE_KERNEL_WUC;
	else if (!strcmp(p, "off"))
		pgprot_wc = PAGE_KERNEL_SUC;
	else
		pr_warn("Unknown writecombine setting \"%s\".\n", p);

	return 0;
}
early_param("writecombine", setup_writecombine);

/*
 * arch_mem_init - initialize memory management subsystem
 */
static int usermem __initdata;

static int __init early_parse_mem(char *p)
{
	phys_addr_t start, size;

	if (!p) {
		pr_err("mem parameter is empty, do nothing\n");
		return -EINVAL;
	}

	/*
	 * If a user specifies memory size, we
	 * blow away any automatically generated
	 * size.
	 */
	if (usermem == 0) {
		usermem = 1;
		if (!strstr(boot_command_line, "elfcorehdr"))
			memblock_remove(memblock_start_of_DRAM(),
				memblock_end_of_DRAM() - memblock_start_of_DRAM());
	}
	start = 0;
	size = memparse(p, &p);
	if (*p == '@')
		start = memparse(p + 1, &p);

	if (IS_ENABLED(CONFIG_NUMA))
		memblock_add_node(start, size, pa_to_nid(start));
	else
		memblock_add(start, size);

	if (strstr(boot_command_line, "elfcorehdr") && start && size) {
		crashmem_start = start;
		crashmem_size = size;
	}

	return 0;
}
early_param("mem", early_parse_mem);

static int __init early_parse_memmap(char *p)
{
	char *oldp;
	u64 start_at, mem_size;

	if (!p)
		return -EINVAL;

	if (!strncmp(p, "exactmap", 8)) {
		pr_err("\"memmap=exactmap\" invalid on LoongArch\n");
		return 0;
	}

	oldp = p;
	mem_size = memparse(p, &p);
	if (p == oldp)
		return -EINVAL;

	if (*p == '@') {
		start_at = memparse(p+1, &p);
		memblock_add(start_at, mem_size);
	} else if (*p == '#') {
		pr_err("\"memmap=nn#ss\" (force ACPI data) invalid on LoongArch\n");
		return -EINVAL;
	} else if (*p == '$') {
		start_at = memparse(p+1, &p);
		memblock_reserve(start_at, mem_size);
	} else {
		pr_err("\"memmap\" invalid format!\n");
		return -EINVAL;
	}

	if (*p == '\0') {
		usermem = 1;
		return 0;
	} else
		return -EINVAL;
}
early_param("memmap", early_parse_memmap);

static void __init loongarch_reserve_vmcore(void)
{
#ifdef CONFIG_PROC_VMCORE
	struct memblock_region *mem;

	if (!is_kdump_kernel())
		return;

	if (!elfcorehdr_size) {
		for_each_memblock(memory, mem) {
			unsigned long start = mem->base;
			unsigned long end = mem->base + mem->size;

			if (elfcorehdr_addr >= start && elfcorehdr_addr < end) {
				/*
				 * Reserve from the elf core header to the end of
				 * the memory segment, that should all be kdump
				 * reserved memory.
				 */
				elfcorehdr_size = end - elfcorehdr_addr;
				break;
			}
		}
	}

	pr_info("Reserving %ldKB of memory at %ldKB for kdump\n",
		(unsigned long)elfcorehdr_size >> 10, (unsigned long)elfcorehdr_addr >> 10);

	memblock_reserve(elfcorehdr_addr, elfcorehdr_size);
#endif
}

#ifdef CONFIG_KEXEC
/* 16M alignment for crash kernel regions */
#define CRASH_ALIGN     SZ_16M
#define CRASH_ADDR_MAX	SZ_4G

static inline unsigned long long get_total_mem(void)
{
	unsigned long long total;

	total = max_pfn - min_low_pfn;
	return total << PAGE_SHIFT;
}

static void __init loongarch_parse_crashkernel(void)
{
	unsigned long long total_mem;
	unsigned long long crash_size, crash_base;
	int ret;

	total_mem = get_total_mem();
	ret = parse_crashkernel(boot_command_line, total_mem,
				&crash_size, &crash_base);
	if (ret != 0 || crash_size <= 0)
		return;

	if (crash_base <= 0) {
		crash_base = memblock_find_in_range(CRASH_ALIGN, CRASH_ADDR_MAX,
				crash_size, CRASH_ALIGN);
		if (!crash_base) {
			pr_warn("crashkernel reservation failed - No suitable area found.\n");
			return;
		}
	} else if (!memblock_find_in_range(crash_base, crash_base + crash_size, crash_size, 1)) {
		pr_warn("Invalid memory region reserved for crash kernel\n");
		return;
	}

	crashk_res.start = crash_base;
	crashk_res.end	 = crash_base + crash_size - 1;
}

static void __init request_crashkernel(struct resource *res)
{
	int ret;

	if (crashk_res.start == crashk_res.end)
		return;

	ret = request_resource(res, &crashk_res);
	if (!ret)
		pr_info("Reserving %ldMB of memory at %ldMB for crashkernel\n",
			(unsigned long)((crashk_res.end -
					 crashk_res.start + 1) >> 20),
			(unsigned long)(crashk_res.start  >> 20));
}
#else /* !defined(CONFIG_KEXEC)		*/
static void __init loongarch_parse_crashkernel(void)
{
}

static void __init request_crashkernel(struct resource *res)
{
}
#endif /* !defined(CONFIG_KEXEC)  */

/* Traditionally, LoongArch's contiguous low memory is 256M, so crashkernel=X@Y is
 * unable to be large enough in some cases. Thus, if the total memory of a node
 * is more than 1GB, we reserve the top 256MB for the capture kernel */
static void reserve_crashm_region(int node, unsigned long s0, unsigned long e0)
{
#ifdef CONFIG_KEXEC
	if (crashk_res.start == crashk_res.end)
		return;

	if ((e0 - s0) <= (SZ_1G >> PAGE_SHIFT))
		return;

	s0 = e0 - (SZ_256M >> PAGE_SHIFT);

	memblock_reserve(PFN_PHYS(s0), (e0 - s0) << PAGE_SHIFT);
#endif
}

/*
 * After the kdump operation is performed to enter the capture kernel, the
 * memory area used by the previous production kernel should be reserved to
 * avoid destroy to the captured data.
 */
static void reserve_oldmem_region(int node, unsigned long s0, unsigned long e0)
{
#ifdef CONFIG_CRASH_DUMP
	unsigned long s1, e1;

	if (!is_kdump_kernel())
		return;

	if ((e0 - s0) > (SZ_1G >> PAGE_SHIFT))
		e0 = e0 - (SZ_256M >> PAGE_SHIFT);

	/* crashmem_start is crashk_res reserved by primary production kernel */
	s1 = PFN_UP(crashmem_start);
	e1 = PFN_DOWN(crashmem_start + crashmem_size);

	if (s1 == 0)
		return;

	if (node == 0) {
		memblock_reserve(PFN_PHYS(s0), (s1 - s0) << PAGE_SHIFT);
		memblock_reserve(PFN_PHYS(e1), (e0 - e1) << PAGE_SHIFT);
	} else {
		memblock_reserve(PFN_PHYS(s0), (e0 - s0) << PAGE_SHIFT);
	}
#endif
}

static void __init reserve_nosave_region(void)
{
	unsigned long start_pfn = PFN_DOWN(__pa_symbol(&__nosave_begin));
	unsigned long end_pfn = PFN_UP(__pa_symbol(&__nosave_end));
	register_nosave_region(start_pfn, end_pfn);
}

static void __init arch_mem_init(void)
{
	unsigned int node;
	unsigned long start_pfn, end_pfn;
	struct memblock_region *reg;
	extern void plat_mem_setup(void);
#ifdef CONFIG_MACH_LOONGSON64
	bool enable;
#endif

	/* call board setup routine */
	plat_mem_setup();
	memblock_set_bottom_up(true);

	/*
	 * Prevent memblock from allocating high memory.
	 * This cannot be done before max_low_pfn is detected, so up
	 * to this point is possible to only reserve physical memory
	 * with memblock_reserve; memblock_virt_alloc* can be used
	 * only after this point
	 */
	memblock_set_current_limit(PFN_PHYS(max_low_pfn));

	loongarch_reserve_vmcore();

	loongarch_parse_crashkernel();
#ifdef CONFIG_KEXEC
	if (crashk_res.start != crashk_res.end)
		memblock_reserve(crashk_res.start,
				 crashk_res.end - crashk_res.start + 1);
#endif
	for_each_online_node(node) {
		get_pfn_range_for_nid(node, &start_pfn, &end_pfn);
		reserve_crashm_region(node, start_pfn, end_pfn);
		reserve_oldmem_region(node, start_pfn, end_pfn);
	}

#ifdef CONFIG_QUICKLIST
	quicklist_init();
#endif
#ifdef CONFIG_MACH_LOONGSON64
	enable = memblock_bottom_up();
	memblock_set_bottom_up(false);
#endif
	sparse_memory_present_with_active_regions(MAX_NUMNODES);
	sparse_init();
#ifdef CONFIG_MACH_LOONGSON64
	memblock_set_bottom_up(enable);
#endif
	plat_swiotlb_setup();

	dma_contiguous_reserve(PFN_PHYS(max_low_pfn));
	/* Tell bootmem about cma reserved memblock section */
	for_each_memblock(reserved, reg)
		if (reg->size != 0)
			memblock_reserve(reg->base, reg->size);
	reserve_nosave_region();
}

static int num_standard_resources;
static struct resource *standard_resources;
static void __init resource_init(void)
{
	struct memblock_region *region;
	struct resource *res;
	unsigned long i = 0;
	int ret = 0;
	code_resource.start = __pa_symbol(&_text);
	code_resource.end = __pa_symbol(&_etext) - 1;
	data_resource.start = __pa_symbol(&_etext);
	data_resource.end = __pa_symbol(&_edata) - 1;
	bss_resource.start = __pa_symbol(&__bss_start);
	bss_resource.end = __pa_symbol(&__bss_stop) - 1;

	num_standard_resources = memblock.memory.cnt;
	standard_resources = alloc_bootmem(num_standard_resources *
					       sizeof(*standard_resources));

	for_each_memblock(memory, region) {
		res = &standard_resources[i++];
		if (memblock_is_nomap(region)) {
			res->name  = "reserved";
			res->flags = IORESOURCE_MEM;
		res->start = __pfn_to_phys(memblock_region_reserved_base_pfn(region));
		res->end = __pfn_to_phys(memblock_region_reserved_end_pfn(region)) - 1;
		} else {
			res->name  = "System RAM";
			res->flags = IORESOURCE_SYSTEM_RAM | IORESOURCE_BUSY;
		res->start = __pfn_to_phys(memblock_region_memory_base_pfn(region));
		res->end = __pfn_to_phys(memblock_region_memory_end_pfn(region)) - 1;
		}

		ret = request_resource(&iomem_resource, res);


		/*
		 *  We don't know which RAM region contains kernel data,
		 *  so we try it repeatedly and let the resource manager
		 *  test it.
		 */
		request_resource(res, &code_resource);
		request_resource(res, &data_resource);
		request_resource(res, &bss_resource);
		request_crashkernel(res);
	}
}

static int __init reserve_memblock_reserved_regions(void)
{
	u64 i, j;

	for (i = 0; i < num_standard_resources; ++i) {
		struct resource *mem = &standard_resources[i];
		phys_addr_t r_start, r_end, mem_size = resource_size(mem);

		if (!strcmp("reserved", mem->name) || !memblock_is_region_reserved(mem->start, mem_size))
			continue;

		for_each_reserved_mem_region(j, &r_start, &r_end) {
			resource_size_t start, end;

			start = max(PFN_PHYS(PFN_DOWN(r_start)), mem->start);
			end = min(PFN_PHYS(PFN_UP(r_end)) - 1, mem->end);

			if (start > mem->end || end < mem->start)
				continue;

			reserve_region_with_split(mem, start, end, "reserved");
		}
	}

	return 0;
}
arch_initcall(reserve_memblock_reserved_regions);

#ifdef CONFIG_SMP
static void __init prefill_possible_map(void)
{
	int i, possible;

	possible = num_processors + disabled_cpus;
	if (!possible)
		return;

	if (possible > nr_cpu_ids)
		possible = nr_cpu_ids;

	printk(KERN_INFO "SMP: Allowing %d CPUs, %d hotplug CPUs\n",
			possible, max((possible - num_processors), 0));
	for (i = 0; i < possible; i++)
		set_cpu_possible(i, true);
	for (; i < NR_CPUS; i++)
		set_cpu_possible(i, false);

	nr_cpu_ids = possible;
}
#else
static inline void prefill_possible_map(void) {}
#endif

static void parse_cmdline(char **cmdline_p)
{
	phys_addr_t size;

	strlcat(boot_command_line, " ", COMMAND_LINE_SIZE);

	/*
	 * Make sure all kernel memory is in the maps.  The "UP" and
	 * "DOWN" are opposite for initdata since if it crosses over
	 * into another memory section you don't want that to be
	 * freed when the initdata is freed.
	 */
	size = (PFN_UP(__pa_symbol(&_edata)) << PAGE_SHIFT) -
	    (PFN_DOWN(__pa_symbol(&_text)) << PAGE_SHIFT);
	memblock_add(PFN_DOWN(__pa_symbol(&_text)) << PAGE_SHIFT, size);

	size = (PFN_DOWN(__pa_symbol(&__init_end)) << PAGE_SHIFT) -
	    (PFN_UP(__pa_symbol(&__init_begin)) << PAGE_SHIFT);
	memblock_reserve(PFN_UP(__pa_symbol(&__init_begin)) << PAGE_SHIFT, size);

	strlcat(boot_command_line, arcs_cmdline, COMMAND_LINE_SIZE);

	if (builtin_cmdline[0]) {
		if (boot_command_line[0])
			strlcat(boot_command_line, " ", COMMAND_LINE_SIZE);
		strlcat(boot_command_line, builtin_cmdline, COMMAND_LINE_SIZE);
	}

	strlcpy(command_line, boot_command_line, COMMAND_LINE_SIZE);

	*cmdline_p = command_line;

	parse_early_param();

	if (usermem) {
		pr_info("User-defined physical RAM map:\n");
		__memblock_dump_all();
	}
}
extern void early_init(void);
extern void loongarch_syscall_init(void);
void __init setup_arch(char **cmdline_p)
{
	unwind_init();
	cpu_probe();
	*cmdline_p = boot_command_line;
	early_init();
	parse_cmdline(cmdline_p);
	reserve_initrd_mem();
	platform_init();

#if defined(CONFIG_VT)
#if defined(CONFIG_VGA_CONSOLE)
	conswitchp = &vga_con;
#elif defined(CONFIG_DUMMY_CONSOLE)
	conswitchp = &dummy_con;
#endif
#endif
	arch_mem_init();

	resource_init();
	plat_smp_setup();
	prefill_possible_map();

	paging_init();

#if defined(CONFIG_KASAN)
	kasan_init();
#endif
	if (loongson_syscall)
		loongarch_syscall_init();
}

DEFINE_PER_CPU(unsigned long, kernelsp);
unsigned long fw_arg0, fw_arg1, fw_arg2;
