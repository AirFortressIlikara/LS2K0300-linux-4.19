/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/device.h>
#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/thermal.h>
#include <linux/dmi.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <loongson.h>

/*Rate control delay*/
#define PKG_TEMP_THERMAL_NOTIFY_DELAY 5000
static int notify_delay_ms = PKG_TEMP_THERMAL_NOTIFY_DELAY;
module_param(notify_delay_ms, int, 0644);
MODULE_PARM_DESC(notify_delay_ms,
	"User space notification delay in milli seconds.");

/*
* Number of trip points in thermal zone. Currently it can't be
* more than 7. Loongson-3 support 8 thresholds now, 0-3 is low
* temp threshold, 4-7 is high temp thredhold, but 7 is reserved.
*/
#define MAX_NUMBER_OF_TRIPS 7
#define LS3_MIN_TEMP -40000
#define LS3_MAX_TEMP 125000

#ifdef CONFIG_MIPS
#define LS3_TEMP_INT_REG_BASE		0x3ff01460
#else /* CONFIG_MIPS */
#define LS3_TEMP_INT_REG_BASE		0x1fe01460
#endif
#define LS3_TEMP_INT_HI_OFFSET		0
#define LS3_TEMP_INT_LO_OFFSET		8
#define LS3_TEMP_INT_STA_OFFSET		16
#define LS3_TEMP_INT_UP_OFFSET		24

static DEFINE_PER_CPU(unsigned int, loongson3_cpufreq_thermal_pctg);

static unsigned int loongson3_thermal_cpufreq_is_init = 0;

#define loongson3_reduction_pctg(cpu) \
	per_cpu(loongson3_cpufreq_thermal_pctg, phys_package_first_cpu(cpu))

static int loongson3_def_trip_temp[MAX_NUMBER_OF_TRIPS] = {
	-30000, -32000, -34000, -36000,
	114000, 116000, 118000
};

static int loongson3_adj_freq_trip_temp[MAX_NUMBER_OF_TRIPS] = {
	-30000, -32000, -34000, -36000,
	60000, 65000, 70000
};

struct pkg_device {
	bool work_scheduled;
	int cpu;
	int package_id;
	int irq;
	u64 temp_int_hi_reg;
	u64 temp_int_lo_reg;
	struct delayed_work work;
	struct thermal_zone_device *tzone;
	struct cpumask cpumask;
};

struct loongson3_thsens_reg {
	u64 mask;
	u64 shift;
	u64 intr;
	u64 sel;
	u64 up_mask;
	u64 up_shift;
};

static struct thermal_zone_params pkg_temp_tz_params = {
	.governor_name = "user_space",
	.no_hwmon = true,
};

static int nr_packages;
static struct loongson3_thsens_reg *thsens_reg;
/* Array of package pointers */
static struct pkg_device **packages;
/* Serializes interrupt notification, work and hotplug */
static DEFINE_SPINLOCK(pkg_temp_lock);
/* Protects zone operation in the work function against hotplug removal */
static DEFINE_MUTEX(thermal_zone_mutex);
/* The dynamically assigned cpu hotplug state for module_exit() */
static enum cpuhp_state pkg_thermal_hp_state;
static int irq_registered;
static bool adjust_frequency;
static u64 thermal_irq;
static u64 loongson_tempinthi[MAX_PACKAGES];
static u64 loongson_tempintlo[MAX_PACKAGES];
static u64 loongson_tempintsta[MAX_PACKAGES];
static u64 loongson_tempintup[MAX_PACKAGES];

#define DEFINE_LOONGSON_TEMP_GET(__name)			\
static inline u64 loongson_##__name##_get(int id)		\
{								\
	return readq((void __iomem *)loongson_##__name[id]);	\
}

#ifdef CONFIG_LOONGARCH
static inline void xconf_writeq(u64 val64, volatile void __iomem *addr)
{
	asm volatile (
	"	st.d	%[v], %[hw], 0	\n"
	"	ld.b	$r0, %[hw], 0	\n"
	:
	: [hw] "r" (addr),  [v] "r" (val64)
	);
}
#endif

#ifdef CONFIG_MIPS
#define DEFINE_LOONGSON_TEMP_SET(__name)				\
static inline void loongson_##__name##_set(u64 val, int id)		\
{									\
	ls64_conf_write64(val, (void __iomem *)loongson_##__name[id]);	\
}
#else /* CONFIG_MIPS */
#define DEFINE_LOONGSON_TEMP_SET(__name)				\
static inline void loongson_##__name##_set(u64 val, int id)		\
{									\
	xconf_writeq(val, (void __iomem *)loongson_##__name[id]);	\
}
#endif

DEFINE_LOONGSON_TEMP_GET(tempinthi)
DEFINE_LOONGSON_TEMP_SET(tempinthi)
DEFINE_LOONGSON_TEMP_GET(tempintlo)
DEFINE_LOONGSON_TEMP_SET(tempintlo)
DEFINE_LOONGSON_TEMP_GET(tempintsta)
DEFINE_LOONGSON_TEMP_SET(tempintsta)
DEFINE_LOONGSON_TEMP_GET(tempintup)
DEFINE_LOONGSON_TEMP_SET(tempintup)

/*Temperature*/
#define TEMP_SENSOR0_SEL	0ULL
#define TEMP_SENSOR1_SEL	1ULL

#define TEMP_HIGH_INT_ENABLE(id)    (1ULL << (8 + (id) * 16))
#define TEMP_HIGH_SHIFT(id)         (0 + (id) * 16)
#define TEMP_HIGH_MASK(id)          (0xffULL << TEMP_HIGH_SHIFT(id))
#define TEMP_HIGH_SHIFT_SEL(id)     (10 + (id) * 16)

#define TEMP_LOW_INT_ENABLE(id)     (1ULL << (8 + (id) * 16))
#define TEMP_LOW_SHIFT(id)          (0 + (id) * 16)
#define TEMP_LOW_MASK(id)           (0xffULL << TEMP_LOW_SHIFT(id))
#define TEMP_LOW_SHIFT_SEL(id)      (10 + (id) * 16)

#define TEMP_INT_STATUS_HI          1ULL
#define TEMP_INT_STATUS_LO          (1ULL << 1)

#define TEMP_HIGH_GATE_SHIFT(id)    (0 + (id) * 8)
#define TEMP_HIGH_GATE_MASK(id)     (0xffULL << TEMP_HIGH_GATE_SHIFT(id))
#define TEMP_LOW_GATE_SHIFT(id)     (32 + (id) * 8)
#define TEMP_LOW_GATE_MASK(id)      (0xffULL << TEMP_LOW_GATE_SHIFT(id))

static int phys_package_first_cpu(int cpu)
{
	int i;
	int id = topology_physical_package_id(cpu);

	for_each_online_cpu(i)
		if (topology_physical_package_id(i) == id)
			return i;
	return 0;
}

static int cpu_has_cpufreq(unsigned int cpu)
{
	struct cpufreq_policy policy;
	if (!loongson3_thermal_cpufreq_is_init || cpufreq_get_policy(&policy, cpu))
		return 0;
	return 1;
}

static struct pkg_device *pkg_temp_thermal_get_dev(unsigned int cpu)
{
	int pkgid = topology_physical_package_id(cpu);

	if (pkgid >= 0 && pkgid < nr_packages){
		return packages[pkgid];
	}

	return NULL;
}

static int loongson3_thermal_cpufreq_notifier(struct notifier_block *nb,
					 unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned long max_freq = 0;

	if (event != CPUFREQ_ADJUST)
		goto out;

	max_freq = (
	    policy->cpuinfo.max_freq *
	    (100 - loongson3_reduction_pctg(policy->cpu) * 20)
	) / 100;

	cpufreq_verify_within_limits(policy, 0, max_freq);

out:
	return 0;
}

static struct notifier_block loongson3_thermal_cpufreq_notifier_block = {
	.notifier_call = loongson3_thermal_cpufreq_notifier,
};

static int
loongson3_thsens_reg_info(int trip, struct loongson3_thsens_reg *th_reg)
{
	if(trip < 4) {
		th_reg->mask = TEMP_LOW_MASK(trip);
		th_reg->shift = TEMP_LOW_SHIFT(trip);
		th_reg->intr = TEMP_LOW_INT_ENABLE(trip);
		th_reg->sel = (TEMP_SENSOR0_SEL << TEMP_LOW_SHIFT_SEL(trip));
		th_reg->up_mask = TEMP_LOW_GATE_MASK(trip);
		th_reg->up_shift = TEMP_LOW_GATE_SHIFT(trip);
	} else {
		th_reg->mask = TEMP_HIGH_MASK(trip - 4);
		th_reg->shift = TEMP_HIGH_SHIFT(trip - 4);
		th_reg->intr = TEMP_HIGH_INT_ENABLE(trip - 4);
		th_reg->sel = (TEMP_SENSOR0_SEL << TEMP_HIGH_SHIFT_SEL(trip - 4));
		th_reg->up_mask = TEMP_HIGH_GATE_MASK(trip - 4);
		th_reg->up_shift = TEMP_HIGH_GATE_SHIFT(trip - 4);
	}

	return 0;
}

static int
loongson3_set_trip_temp(
	int trip, int temp, struct loongson3_thsens_reg *th_reg, int pkg_id)
{
	u64 reg_hi, reg_lo, reg_up, trans_temp;

	reg_lo = loongson_tempintlo_get(pkg_id);
	reg_hi = loongson_tempinthi_get(pkg_id);
	reg_up = loongson_tempintup_get(pkg_id);

	loongson3_thsens_reg_info(trip, th_reg);

	trans_temp = ((temp / 1000 + 273) * 0x4000 / 731) & 0xffff;
	reg_up &= ~th_reg->up_mask;
	reg_up |= ((trans_temp >> 8) << th_reg->up_shift);
	loongson_tempintup_set(reg_up, pkg_id);

	if (trip < 4) {
		reg_lo &= ~th_reg->mask;
		reg_lo |= ((trans_temp & 0xff) << th_reg->shift);
		reg_lo |= th_reg->intr;
		reg_lo |= th_reg->sel;
		loongson_tempintlo_set(reg_lo, pkg_id);
	} else {
		reg_hi &= ~th_reg->mask;
		reg_hi |= ((trans_temp & 0xff) << th_reg->shift);
		reg_hi |= th_reg->intr;
		reg_hi |= th_reg->sel;
		loongson_tempinthi_set(reg_hi, pkg_id);
	}

	return 0;
}

static int is_available_cpu_type(void)
{
	u32 prid_rev;
	int cpu_type;

#ifdef CONFIG_LOONGARCH
	cpu_type = current_cpu_data.cputype;
	if (cpu_type == CPU_LOONGSON64)
#endif
#ifdef CONFIG_MIPS
	cpu_type = boot_cpu_type();
	if (cpu_type == CPU_LOONGSON3_COMP)
#endif
	{
		return 0;
#ifdef __mips__
	}else if(cpu_type == CPU_LOONGSON3){
		prid_rev = read_c0_prid() & PRID_REV_MASK;
		switch (prid_rev) {
		case PRID_REV_LOONGSON3A_R1:
		case PRID_REV_LOONGSON3B_R1:
		case PRID_REV_LOONGSON3B_R2:
		case PRID_REV_LOONGSON3A_R2_0:
		case PRID_REV_LOONGSON3A_R2_1:
			pr_err("cpu type error\n");
			return -EINVAL;
		}
		return 0;
#endif
	}else{
		pr_err("cpu type error\n");
		return -EINVAL;
	}
}

static int loongson3_init_trip_temp(int pkg_id)
{
	int trip, temp;
	struct loongson3_thsens_reg *th_reg = thsens_reg;

	if(is_available_cpu_type() != 0)
		return -EINVAL;

	for(trip = 0; trip < MAX_NUMBER_OF_TRIPS; trip++){
		if (adjust_frequency == true)
			temp = loongson3_adj_freq_trip_temp[trip];
		else
			temp = loongson3_def_trip_temp[trip];

		loongson3_set_trip_temp(trip, temp, th_reg, pkg_id);
	}
	return 0;
}

/*
 * Loongson-3 series cpu has two sensors inside,
 * each of them from 0 to 255,
 * if more than 127, that is dangerous.
 * here only provide sensor1 data, because it always hot than sensor0
 */
static int get_curr_temp(int pkg_id)
{
	int curr_temp;
	u32 reg, prid_rev;

	reg = LOONGSON_CHIPTEMP(pkg_id);
#ifdef CONFIG_LOONGARCH
	switch (cpu_data[0].cputype) {
#else
	switch(boot_cpu_type()) {
#endif
#ifdef __mips__
		case CPU_LOONGSON3:
			prid_rev = read_c0_prid() & PRID_REV_MASK;
			switch (prid_rev) {
			case PRID_REV_LOONGSON3A_R1:
				reg = (reg >> 8) & 0xff;
				break;
			case PRID_REV_LOONGSON3B_R1:
			case PRID_REV_LOONGSON3B_R2:
			case PRID_REV_LOONGSON3A_R2_0:
			case PRID_REV_LOONGSON3A_R2_1:
				reg = ((reg >> 8) & 0xff) - 100;
				break;
			case PRID_REV_LOONGSON3A_R3_0:
			case PRID_REV_LOONGSON3A_R3_1:
			case PRID_REV_LOONGSON3A_R4_0:
				reg = (reg & 0xffff) * 731 / 0x4000 - 273;
				break;
			}
			break;
#endif
#ifdef CONFIG_LOONGARCH
		case CPU_LOONGSON64:
#endif
#ifdef CONFIG_MIPS
		case CPU_LOONGSON3_COMP:
#endif
			reg = (reg & 0xffff) * 731 / 0x4000 - 273;
			break;

	}

	curr_temp = (int)reg * 1000;

	return curr_temp;
}

static int sys_get_curr_temp(struct thermal_zone_device *tzd, int *temp)
{
	struct pkg_device *pkgdev = tzd->devdata;

	*temp = get_curr_temp(pkgdev->package_id);

	return 0;
}

static int
sys_get_trip_temp(struct thermal_zone_device *tzd, int trip, int *temp)
{
	u16 thres_reg_value;
	u64 reg_down, reg_up;
	struct pkg_device *pkgdev = tzd->devdata;
	struct loongson3_thsens_reg *th_reg = thsens_reg;

	if (trip >= MAX_NUMBER_OF_TRIPS)
		return -EINVAL;

	if (!pkgdev)
		return -EINVAL;

	if(is_available_cpu_type() != 0)
		return -EINVAL;

	loongson3_thsens_reg_info(trip, th_reg);

	if(trip < 4){
		reg_down = loongson_tempintlo_get(pkgdev->package_id);
	}else{
		reg_down = loongson_tempinthi_get(pkgdev->package_id);
	}
	reg_up = loongson_tempintup_get(pkgdev->package_id);

	thres_reg_value = (((reg_up & th_reg->up_mask) >> th_reg->up_shift) << 8)
				 | ((reg_down & th_reg->mask) >> th_reg->shift);
	if (thres_reg_value)
		*temp = (thres_reg_value * 731 / 0x4000 - 273) * 1000 ;
	else
		*temp = 0;

	return 0;
}

static int
sys_set_trip_temp(struct thermal_zone_device *tzd, int trip, int temp)
{
	struct pkg_device *pkgdev = tzd->devdata;
	struct loongson3_thsens_reg *th_reg = thsens_reg;

	if (trip >= MAX_NUMBER_OF_TRIPS || temp > LS3_MAX_TEMP
		 || temp < LS3_MIN_TEMP)
		return -EINVAL;

	if (!pkgdev)
		return -EINVAL;

	if (adjust_frequency == true)
		loongson3_adj_freq_trip_temp[trip] = temp;
	else
		loongson3_def_trip_temp[trip] = temp;


	if(is_available_cpu_type() != 0)
		return -EINVAL;

	loongson3_set_trip_temp(trip, temp, th_reg, pkgdev->package_id);

	return 0;
}

static int sys_get_trip_type(struct thermal_zone_device *thermal,
		 	int trip, enum thermal_trip_type *type)
{
	*type = THERMAL_TRIP_PASSIVE;
	return 0;
}

/* Thermal zone callback registry */
static struct thermal_zone_device_ops tzone_ops = {
	.get_temp = sys_get_curr_temp,
	.get_trip_temp = sys_get_trip_temp,
	.get_trip_type = sys_get_trip_type,
	.set_trip_temp = sys_set_trip_temp,
};

/* Enable threshold interrupt on local package/cpu */
static inline void loongson3_enable_pkg_thres_interrupt(int pkg_id)
{
	u64 reg_lo, reg_hi;

	reg_lo = loongson_tempintlo_get(pkg_id);
	reg_hi = loongson_tempinthi_get(pkg_id);

	reg_lo |= (TEMP_LOW_INT_ENABLE(0) | TEMP_LOW_INT_ENABLE(1) |
			TEMP_LOW_INT_ENABLE(2) | TEMP_LOW_INT_ENABLE(3));

	reg_hi |= (TEMP_HIGH_INT_ENABLE(0) | TEMP_HIGH_INT_ENABLE(1) |
			TEMP_HIGH_INT_ENABLE(2));

	loongson_tempintlo_set(reg_lo, pkg_id);
	loongson_tempinthi_set(reg_hi, pkg_id);
}

/* Disable threshold interrupt on local package/cpu */
static inline void loongson3_disable_pkg_thres_interrupt(int pkg_id)
{
	u64 reg_lo, reg_hi;

	reg_lo = loongson_tempintlo_get(pkg_id);
	reg_hi = loongson_tempinthi_get(pkg_id);

	reg_lo &= ~(TEMP_LOW_INT_ENABLE(0) | TEMP_LOW_INT_ENABLE(1) |
			TEMP_LOW_INT_ENABLE(2) | TEMP_LOW_INT_ENABLE(3));

	reg_hi &= ~(TEMP_HIGH_INT_ENABLE(0) | TEMP_HIGH_INT_ENABLE(1) |
			TEMP_HIGH_INT_ENABLE(2));

	loongson_tempintlo_set(reg_lo, pkg_id);
	loongson_tempinthi_set(reg_hi, pkg_id);
}

/* Clear threshold interrupt on local package/cpu */
static inline void loongson3_clear_pkg_thres_interrupt(int pkg_id)
{
	u64 reg_sta, wr_val;

	reg_sta = loongson_tempintsta_get(pkg_id);
	wr_val = reg_sta & (TEMP_INT_STATUS_HI | TEMP_INT_STATUS_LO);
	loongson_tempintsta_set(wr_val, pkg_id);

}

static void pkg_temp_thermal_threshold_work_fn(struct work_struct *work)
{
	struct thermal_zone_device *tzone = NULL;
	int cpu = smp_processor_id();
	struct pkg_device *pkgdev;
	int i, cur_temp, thermal_state;

	mutex_lock(&thermal_zone_mutex);
	spin_lock_irq(&pkg_temp_lock);

	pkgdev = pkg_temp_thermal_get_dev(cpu);
	if (!pkgdev) {
		spin_unlock_irq(&pkg_temp_lock);
		mutex_unlock(&thermal_zone_mutex);
		return;
	}
	pkgdev->work_scheduled = false;

	tzone = pkgdev->tzone;

	loongson3_enable_pkg_thres_interrupt(pkgdev->package_id);
	spin_unlock_irq(&pkg_temp_lock);

	/*
	 * If tzone is not NULL, then thermal_zone_mutex will prevent the
	 * concurrent removal in the cpu offline callback.
	 */
	if (tzone) {
		thermal_zone_device_update(tzone, THERMAL_EVENT_UNSPECIFIED);
	}

	if (adjust_frequency == true) {
		if (!cpu_has_cpufreq(cpu)) {
			mutex_unlock(&thermal_zone_mutex);
			return;
		}

		cur_temp = get_curr_temp(pkgdev->package_id);
		if (cur_temp > loongson3_adj_freq_trip_temp[6])
			thermal_state = 2;
		else if (cur_temp > loongson3_adj_freq_trip_temp[5])
			thermal_state = 1;
		else
			thermal_state = 0;

		loongson3_reduction_pctg(cpu) = thermal_state;

		for_each_online_cpu(i) {
			if (topology_physical_package_id(i) == topology_physical_package_id(cpu)){
				cpufreq_update_policy(i);
			}
		}
	}

	mutex_unlock(&thermal_zone_mutex);
}

static void pkg_thermal_schedule_work(int cpu, struct delayed_work *work)
{
	unsigned long ms = msecs_to_jiffies(notify_delay_ms);

	schedule_delayed_work_on(cpu, work, ms);
}

static irqreturn_t loongson3_temp_irq_handler(int irq, void *data)
{
	int cpu = smp_processor_id();
	struct pkg_device *pkgdev;
	unsigned long flags;

	spin_lock_irqsave(&pkg_temp_lock, flags);

	pkgdev = pkg_temp_thermal_get_dev(cpu);
	if (pkgdev) {
		loongson3_disable_pkg_thres_interrupt(pkgdev->package_id);
		loongson3_clear_pkg_thres_interrupt(pkgdev->package_id);
	}

	/* Work is per package, so scheduling it once is enough. */
	if (pkgdev && !pkgdev->work_scheduled) {
		pkgdev->work_scheduled = true;
		pkg_thermal_schedule_work(pkgdev->cpu, &pkgdev->work);
	}

	spin_unlock_irqrestore(&pkg_temp_lock, flags);
	return IRQ_HANDLED;
}

static struct irqaction loongson3_temp_irqaction = {
	.name = "thsens",
	.flags = IRQF_ONESHOT,
	.handler = loongson3_temp_irq_handler,
};

static int pkg_temp_thermal_device_add(unsigned int cpu)
{
	int pkgid = topology_physical_package_id(cpu);
	struct pkg_device *pkgdev;
	int thres_count, err;
	struct fwnode_handle *domain_handle;
#if defined(__mips__)
	int irq = LOONGSON_THSENS_IRQ;
#else
	int irq = 0;
#endif
	if (pkgid >= nr_packages)
		return -ENODEV;

	thres_count = MAX_NUMBER_OF_TRIPS;
	if (!thres_count)
		return -ENODEV;

	thres_count = clamp_val(thres_count, 0, MAX_NUMBER_OF_TRIPS);

	pkgdev = kzalloc(sizeof(*pkgdev), GFP_KERNEL);
	if (!pkgdev)
		return -ENOMEM;

	INIT_DELAYED_WORK(&pkgdev->work, pkg_temp_thermal_threshold_work_fn);
	pkgdev->cpu = cpu;
	pkgdev->package_id = pkgid;
	pkgdev->irq = irq;

	/* Store reg value for temp interrupt, to restore at exit */
	pkgdev->temp_int_lo_reg = loongson_tempintlo_get(pkgid);
	pkgdev->temp_int_hi_reg = loongson_tempinthi_get(pkgid);

	cpumask_set_cpu(cpu, &pkgdev->cpumask);
	spin_lock_irq(&pkg_temp_lock);
	packages[pkgid] = pkgdev;
	spin_unlock_irq(&pkg_temp_lock);

	if (thermal_irq) {
		pkgdev->irq = thermal_irq;
	} else {
		domain_handle = liointc_get_fwnode();
		if (domain_handle) {
			struct irq_fwspec fwspec;

			fwspec.fwnode = domain_handle;
			fwspec.param[0] = LOONGSON_CPU_THSENS_VEC;
			fwspec.param_count = 1;
			pkgdev->irq = irq_create_fwspec_mapping(&fwspec);
		}
	}

	if (!irq_registered) {
		if (pkgdev->irq) {
			setup_irq(pkgdev->irq, &loongson3_temp_irqaction);
			irq_registered = 1;
		} else {
			printk("Unable to map irq for thermal sensor!!!\n");
			return -ENODEV;
		}
	}

	pkgdev->tzone = thermal_zone_device_register("loongson3_pkg_temp",
			thres_count, (1 << thres_count) - 1,
			pkgdev, &tzone_ops, &pkg_temp_tz_params, 0, 0);
	if (IS_ERR(pkgdev->tzone)) {
		err = PTR_ERR(pkgdev->tzone);
		kfree(pkgdev);
		return err;
	}
	loongson3_init_trip_temp(pkgid);

	return 0;
}

static int dmi_check_cb(const struct dmi_system_id *dmi)
{
	printk("laptop model '%s'\n", dmi->ident);
	adjust_frequency = true;

	return 1;
}

static const struct dmi_system_id loongson_device_table[] __initconst = {
	{
		.ident = "loongson laptop",
		.matches = {
			DMI_MATCH(DMI_CHASSIS_TYPE, "9"),
		},
		.callback = dmi_check_cb,
	},
	{}
};

MODULE_DEVICE_TABLE(dmi, loongson_device_table);

static int pkg_thermal_cpu_online(unsigned int cpu)
{
	struct pkg_device *pkgdev = pkg_temp_thermal_get_dev(cpu);

	/* If the package exists, nothing to do */
	if (pkgdev) {
		cpumask_set_cpu(cpu, &pkgdev->cpumask);
		return 0;
	}
	return pkg_temp_thermal_device_add(cpu);
}

static int pkg_thermal_cpu_offline(unsigned int cpu)
{
	struct pkg_device *pkgdev = pkg_temp_thermal_get_dev(cpu);
	bool lastcpu, was_target;
	int target;

	if (!pkgdev)
		return 0;

	target = cpumask_any_but(&pkgdev->cpumask, cpu);
	cpumask_clear_cpu(cpu, &pkgdev->cpumask);
	lastcpu = target >= nr_cpu_ids;
	/*
	 * Remove the sysfs files, if this is the last cpu in the package
	 * before doing further cleanups.
	 */
	if (lastcpu) {
		struct thermal_zone_device *tzone = pkgdev->tzone;

		/*
		 * We must protect against a work function calling
		 * thermal_zone_update, after/while unregister. We null out
		 * the pointer under the zone mutex, so the worker function
		 * won't try to call.
		 */
		mutex_lock(&thermal_zone_mutex);
		pkgdev->tzone = NULL;
		mutex_unlock(&thermal_zone_mutex);

		thermal_zone_device_unregister(tzone);
		remove_irq(pkgdev->irq, &loongson3_temp_irqaction);
	}

	/* Protect against work and interrupts */
	spin_lock_irq(&pkg_temp_lock);

	/*
	 * Check whether this cpu was the current target and store the new one.
	 */
	was_target = pkgdev->cpu == cpu;
	pkgdev->cpu = target;

	/*
	 * If this is the last CPU in the package remove the package
	 * reference from the array and restore the interrupt.
	 */
	if (lastcpu) {
		packages[topology_physical_package_id(cpu)] = NULL;
		loongson_tempintlo_set(pkgdev->temp_int_lo_reg,
					topology_physical_package_id(cpu));
		loongson_tempinthi_set(pkgdev->temp_int_hi_reg,
					topology_physical_package_id(cpu));
	}

	/*
	 * Check whether there is work scheduled and whether the work is
	 * targeted at the outgoing CPU.
	 */
	if (pkgdev->work_scheduled && was_target) {
		/*
		 * To cancel the work we need to drop the lock, otherwise
		 * we might deadlock if the work needs to be flushed.
		 */
		spin_unlock_irq(&pkg_temp_lock);
		cancel_delayed_work_sync(&pkgdev->work);
		spin_lock_irq(&pkg_temp_lock);
		/*
		 * If this is not the last cpu in the package and the work
		 * did not run after we dropped the lock above, then we
		 * need to reschedule the work, otherwise the interrupt
		 * stays disabled forever.
		 */
		if (!lastcpu && pkgdev->work_scheduled)
			pkg_thermal_schedule_work(target, &pkgdev->work);
	}

	spin_unlock_irq(&pkg_temp_lock);

	/* Final cleanup if this is the last cpu */
	if (lastcpu)
		kfree(pkgdev);

	return 0;
}

static void loongson_thermal_set_pkg_regs(u64 reg_base)
{
	int i;
	u64 pkg_base;

	for (i = 0; i < MAX_PACKAGES; i++) {
		pkg_base = LOONGSON3_NODE_BASE(i) | reg_base;
		loongson_tempinthi[i] = pkg_base + LS3_TEMP_INT_HI_OFFSET;
		loongson_tempintlo[i] = pkg_base + LS3_TEMP_INT_LO_OFFSET;
		loongson_tempintsta[i] = pkg_base + LS3_TEMP_INT_STA_OFFSET;
		loongson_tempintup[i] = pkg_base + LS3_TEMP_INT_UP_OFFSET;
	}
}

static int loongson3_thermal_probe(struct platform_device *pdev)
{
	u64 reg_base;
	struct resource *regs;
	struct device *dev = &pdev->dev;
	int ret;

	if (acpi_disabled) {
		if (!dmi_check_system(loongson_device_table))
			adjust_frequency = false;
		loongson_thermal_set_pkg_regs(LS3_TEMP_INT_REG_BASE);
	} else {
		regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!regs) {
			dev_err(dev, "no regs resource!\n");
			return -ENODEV;
		}
		reg_base = (u64)(regs->start);
		loongson_thermal_set_pkg_regs(reg_base);
		thermal_irq = platform_get_irq(pdev, 0);
		adjust_frequency = device_property_read_bool(dev,
							"adjust-frequency");
	}

	if (adjust_frequency == true) {
		ret = cpufreq_register_notifier(
				&loongson3_thermal_cpufreq_notifier_block,
				CPUFREQ_POLICY_NOTIFIER);
		if (!ret)
			loongson3_thermal_cpufreq_is_init = 1;
	}

	nr_packages = loongson_sysconf.nr_cpus /
		loongson_sysconf.cores_per_package;

	thsens_reg = kzalloc(sizeof(struct loongson3_thsens_reg), GFP_KERNEL);
	if (!thsens_reg)
		return -ENOMEM;

	packages = kcalloc(nr_packages, sizeof(struct pkg_device *), GFP_KERNEL);
	if (!packages)
		return -ENOMEM;

	ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "thermal/loongson3_pkg:online",
			pkg_thermal_cpu_online, pkg_thermal_cpu_offline);
	if (ret < 0) {
		pr_err("Loongson temp thermal init err...\n");
		goto err;
	}

	/* Store the state for module exit */
	pkg_thermal_hp_state = ret;

	return 0;

err:
	kfree(thsens_reg);
	kfree(packages);
	return ret;
}

static int loongson3_thermal_remove(struct platform_device *pdev)
{
	if (adjust_frequency == true) {
		if (loongson3_thermal_cpufreq_is_init)
			cpufreq_unregister_notifier(
				&loongson3_thermal_cpufreq_notifier_block,
				CPUFREQ_POLICY_NOTIFIER);

		loongson3_thermal_cpufreq_is_init = 0;
	}

	cpuhp_remove_state(pkg_thermal_hp_state);
	kfree(thsens_reg);
	kfree(packages);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int loongson3_thermal_suspend(struct device *dev)
{
	return 0;
}

static int loongson3_thermal_resume(struct device *dev)
{
	int pkg_id;

	for (pkg_id = 0; pkg_id < nr_packages; pkg_id++)
		loongson3_init_trip_temp(pkg_id);

	return 0;
}

static SIMPLE_DEV_PM_OPS(loongson3_thermal_pm_ops, loongson3_thermal_suspend,
			 loongson3_thermal_resume);
#define LOONGSON_THERMAL_PM	(&loongson3_thermal_pm_ops)
#else
#define LOONGSON_THERMAL_PM	NULL
#endif

static const struct acpi_device_id loongson3_thermal_acpi_match[] = {
	{"LOON0008"},
	{}
};
MODULE_DEVICE_TABLE(acpi, loongson3_thermal_acpi_match);

static struct platform_driver loongson3_thermal_driver = {
	.driver = {
		.name = "loongson3_thermal",
		.pm = LOONGSON_THERMAL_PM,
		.acpi_match_table = ACPI_PTR(loongson3_thermal_acpi_match),
	},
	.probe = loongson3_thermal_probe,
	.remove = loongson3_thermal_remove,
};

static int __init loongson3_temp_thermal_init(void)
{
	int ret;

#ifdef CONFIG_LOONGARCH
	if (cpu_has_hypervisor)
#endif
#ifdef CONFIG_MIPS
	if (cpu_guestmode)
#endif
		return 0;

	ret = platform_driver_register(&loongson3_thermal_driver);

	return ret;
}

static void __exit loongson3_temp_thermal_exit(void)
{
#ifdef CONFIG_LOONGARCH
	if (cpu_has_hypervisor)
#endif
#ifdef CONFIG_MIPS
	if (cpu_guestmode)
#endif
		return;

	platform_driver_unregister(&loongson3_thermal_driver);
}

module_init(loongson3_temp_thermal_init);
module_exit(loongson3_temp_thermal_exit);

MODULE_DESCRIPTION("Loongson3 package temperature thermal driver");
MODULE_AUTHOR("yangqiming <yangqiming@loongson.cn>");
MODULE_LICENSE("GPL");
