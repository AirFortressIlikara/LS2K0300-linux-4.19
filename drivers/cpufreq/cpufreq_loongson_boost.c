/*
 *  drivers/cpufreq/cpufreq_loongson_boost.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *            (C)  2019 Yi Jun <yijun@loongson.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpu.h>
#include <linux/percpu-defs.h>
#include <linux/slab.h>
#include <linux/tick.h>
#include <loongson.h>
#include <linux/cpufreq.h>
#include "cpufreq_governor.h"
#include "loongson_boost.h"

/* loongson boost governor macros */
#define DEF_FREQUENCY_UP_THRESHOLD		(80)
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#define MAX_SAMPLING_DOWN_FACTOR		(100000)
#define MICRO_FREQUENCY_UP_THRESHOLD		(95)
#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(10000)
#define MIN_FREQUENCY_UP_THRESHOLD		(11)
#define MAX_FREQUENCY_UP_THRESHOLD		(100)

#define RECORD_LENGTH				(32)
#define BOOST_DELTA				(0xe6666665)

/* factor ** 8 = 0.5 */
#define FACTOR					(0xeac0c6e8)
#define MAX_LOAD				(1193)
#define BOOST_THRESHOLD 		(900)

static unsigned int default_powersave_bias;

static struct mutex boost_mutex[MAX_PACKAGES];

static u64 load_record[MAX_PACKAGES][NR_CPUS/MAX_PACKAGES] = {{0,}, };
static int prev_boost[MAX_PACKAGES] = {0,};

static int curr_boost_core_num[MAX_PACKAGES] = {0, };
static int real_boosted_core_num[MAX_PACKAGES] = {0, };

static int prev_boosted_cores[MAX_PACKAGES][NR_CPUS/MAX_PACKAGES] = {{0, }, };
static int real_boosted_cores[MAX_PACKAGES][NR_CPUS/MAX_PACKAGES] = {{0, }, };

extern struct clocksource csrc_hpet;

static struct temp_policy policy_table = {
	.table[0] = {
		.low_temp = -40,
		.high_temp = 65,
		.fix_weight = 0,
	},
	.table[1] = {
		.low_temp = 65,
		.high_temp = 70,
		.fix_weight = -15,
	},
	.table[2] = {
	.low_temp = 70,
	.high_temp = 75,
	.fix_weight = 5,
	},
	.table[3] = {
		.low_temp = 75,
		.high_temp = 80,
		.fix_weight = 10,
	},
	.table[4] = {
		.low_temp = 80,
		.high_temp = 85,
		.fix_weight = 20,
	},
	.table[5] = {
		.low_temp = 85,
		.high_temp = 120,
		.fix_weight = 15,
	},
};

static int readtemp(void)
{
	int temp;

	temp = (READTEMPREG & 0xffff);
	temp = (temp*731)/0x4000;
	temp -= 273;

	return temp;
}

static int get_fixload(int temp)
{
	int fixload = 0,i = 0;

	for(i = 0; i < MAX_TABLE; i++){
		if((temp >= policy_table.table[i].low_temp) && (temp < policy_table.table[i].high_temp)){
			fixload = 100 - ( temp + policy_table.table[i].fix_weight );
			break;
		}
	}
	return fixload;
}
/*
 * Not all CPUs want IO time to be accounted as busy; this depends on how
 * efficient idling at a higher frequency/voltage is.
 * Pavel Machek says this is not so for various generations of AMD and old
 * Intel systems.
 * Mike Chan (android.com) claims this is also not true for ARM.
 * Because of this, whitelist specific known (series) of CPUs by default, and
 * leave all others up to the user.
 */
static int should_io_be_busy(void)
{
	return 1;
}

unsigned int find_freq_index(struct cpufreq_frequency_table *freq_table, unsigned int target_freq)
{
	struct cpufreq_frequency_table *pos;
	unsigned int freq;
	unsigned int index = ls_upper_index;

	cpufreq_for_each_valid_entry(pos, freq_table) {
		freq = pos->frequency;
		index = pos - freq_table;

		if (freq == target_freq) {
			break;
		}
	}

	return index;
}

/*
 * normal mode: loads of all cores great than upper threshold
 * boost mode: load of one core great than upper threshold and loads
 * of others cores less than lower
 * boost == 0, normal mode, boost == 1, boost mode
 */

static int boost_mode(int package_id, int cpu, int *boosted_core_num, int *high_load_cores, int load)
{
	int i;
	int curr_boost = 0;
	int core_id = cpu_logical_map(cpu) % loongson_sysconf.cores_per_package;

	if (ls_boost_supported) {
		load_record[package_id][core_id] = (u64) load + ((load_record[package_id][core_id] * FACTOR) >> 32);

		for (i = 0; i < loongson_sysconf.cores_per_package; i++) {
			if (load_record[package_id][i] > BOOST_THRESHOLD) {
				high_load_cores[i] = 1;
				(*boosted_core_num)++;
			} else {
				high_load_cores[i] = 0;
			}
		}

		/* boost mode */
		if (*boosted_core_num > 0 && *boosted_core_num <= ls_boost_cores) {
			curr_boost = 1;
		}
	}

	return curr_boost;
}

/*
 * Every sampling_rate, we check, if current idle time is less than 20%
 * (default), then we try to increase frequency. Else, we adjust the frequency
 * proportional to load.
 */
static void boost_update(struct cpufreq_policy *current_policy)
{
	unsigned int load = dbs_update(current_policy);
	int cpu = current_policy->cpu;
	struct cpufreq_policy *policy;
	struct policy_dbs_info *policy_dbs = current_policy->governor_data;
	struct boost_dbs_tuners *tuners = policy_dbs->dbs_data->tuners;

	int curr_boost = 0;
	int boosted_core_num = 0;

	unsigned int freq_next, min_f, max_f;

	int i;
	int high_load_cores[NR_CPUS/MAX_PACKAGES] = {0, };
	int retval = -EINVAL;

	int package_id = cpu_data[cpu].package;
	int index;
	struct cpufreq_freqs freqs;
	unsigned int freq_level = 0;
	unsigned int boost_level = 0;
	unsigned int cpu_offset = cpu % loongson_sysconf.cores_per_package;
	int temp = 0,fix_load = 0;

	mutex_lock(&boost_mutex[package_id]);

	curr_boost = boost_mode(package_id, cpu, &boosted_core_num, high_load_cores, load);

	if(tuners->boost_htrf){
		temp = readtemp();
		if(temp >= TEMP_UP_LIMIT){
			fix_load = get_fixload(temp);
			pr_debug("(func:%s) (temperature%d)(load:%d)(fix_load:%d)\n",__func__,temp,load,fix_load);
			if(fix_load < 0)
				fix_load = 0;
			load = fix_load;
		}
	}
	if (curr_boost == 1) {
		ls3a4000_freq_table_switch(ls3a4000_boost_table);

		for (i = package_id * loongson_sysconf.cores_per_package;
				i < (package_id + 1) * loongson_sysconf.cores_per_package; i++) {
			policy = cpufreq_cpu_get(i);
			if (policy) {
				cpufreq_frequency_table_cpuinfo(policy, ls3a4000_boost_table);
			}
			cpufreq_cpu_put(policy);
		}
	} else {
		ls3a4000_freq_table_switch(ls3a4000_normal_table);

		for (i = package_id * loongson_sysconf.cores_per_package;
				i < (package_id + 1) * loongson_sysconf.cores_per_package; i++) {

			policy = cpufreq_cpu_get(i);
			if (policy) {
				cpufreq_frequency_table_cpuinfo(policy, ls3a4000_normal_table);
			}
			cpufreq_cpu_put(policy);
		}
	}

	/* boost to normal */
	if (prev_boost[package_id] == 1 && curr_boost == 0) {

		/* notification begin*/
		for (i = package_id * loongson_sysconf.cores_per_package;
			i < (package_id + 1) * loongson_sysconf.cores_per_package; i++) {

			policy = cpufreq_cpu_get_raw(i);

			freqs.old = policy->cur;
			freqs.flags = 0;

			index = find_freq_index(ls3a4000_boost_table, policy->cur);

			freqs.new = ls3a4000_normal_table[index].frequency;

			cpufreq_freq_transition_begin(policy, &freqs);
		}

		retval = ls3a4000_set_boost(NORMAL_MODE, 0);

		for (i = package_id * loongson_sysconf.cores_per_package;
			i < (package_id + 1) * loongson_sysconf.cores_per_package; i++) {
			policy = cpufreq_cpu_get_raw(i);

			freqs.old = policy->cur;
			freqs.flags = 0;

			index = find_freq_index(ls3a4000_boost_table, policy->cur);

			freqs.new = ls3a4000_normal_table[index].frequency;

			cpufreq_freq_transition_end(policy, &freqs, retval);
		}

		real_boosted_core_num[package_id] = 0;
		for (i = 0; i < loongson_sysconf.cores_per_package; i++) {
			real_boosted_cores[package_id][i] = 0;
			prev_boosted_cores[package_id][i] = 0;
		}
	}

	/* normal to boost */
	if (prev_boost[package_id] == 0 && curr_boost == 1) {
		for (i = package_id * loongson_sysconf.cores_per_package;
				i < (package_id + 1) * loongson_sysconf.cores_per_package; i++) {
			policy = cpufreq_cpu_get_raw(i);

			min_f = policy->cpuinfo.max_freq / FREQ_DIV;
			max_f = policy->cpuinfo.max_freq;

			freqs.old = policy->cur;
			freqs.flags = 0;

			boost_level = cpufreq_frequency_table_target(policy, max_f, CPUFREQ_RELATION_C);

			/* boost the last core first */
			if (high_load_cores[i % loongson_sysconf.cores_per_package]) {
				freqs.new = max_f;
				real_boosted_cores[package_id][i]  = 1;
				real_boosted_core_num[package_id]++;

				freq_level = freq_level | (boost_level << ((i % loongson_sysconf.cores_per_package) * 4));
			} else {
				freqs.new = max_f * (ls_upper_index - RESERVED_FREQ) / FREQ_DIV;
				freq_level = freq_level | (ls_upper_index << ((i % loongson_sysconf.cores_per_package) * 4));
			}

			pr_debug("%s: cpu: %d, oldfreq: %u, new freq: %u\n",
				__func__, policy->cpu, freqs.old, freqs.new);
			cpufreq_freq_transition_begin(policy, &freqs);
		}

		retval = ls3a4000_set_boost(BOOST_MODE, freq_level);

		for (i = package_id * loongson_sysconf.cores_per_package;
				i < (package_id + 1) * loongson_sysconf.cores_per_package; i++) {
			policy = cpufreq_cpu_get_raw(i);

			min_f = policy->cpuinfo.max_freq / FREQ_DIV;
			max_f = policy->cpuinfo.max_freq;

			freqs.old = policy->cur;
			freqs.flags = 0;

			if (high_load_cores[i % loongson_sysconf.cores_per_package]) {
				freqs.new = max_f;
			} else {
				freqs.new = max_f * (ls_upper_index - RESERVED_FREQ) / FREQ_DIV;
			}

			pr_debug("%s: cpu: %d, oldfreq: %u, new freq: %u\n",
				__func__, policy->cpu, freqs.old, freqs.new);
			cpufreq_freq_transition_end(policy, &freqs, retval);
		}

		for (i = 0; i < loongson_sysconf.cores_per_package; i++) {
			prev_boosted_cores[package_id][i] = real_boosted_cores[package_id][i];
		}
	}

	/* normal mode */
	if (prev_boost[package_id] == 0 && curr_boost == 0) {
		min_f = current_policy->cpuinfo.max_freq / FREQ_DIV;
		max_f = current_policy->cpuinfo.max_freq;

		freq_next = min_f + load * (max_f - min_f) / 100;

		__cpufreq_driver_target(current_policy, freq_next, CPUFREQ_RELATION_C);
	}

	/* boost mode */
	if (prev_boost[package_id] == 1 && curr_boost == 1) {

		min_f = current_policy->cpuinfo.max_freq / FREQ_DIV;
		max_f = current_policy->cpuinfo.max_freq;

		if (real_boosted_core_num[package_id] < boosted_core_num) {
			if (prev_boosted_cores[package_id][cpu_offset] == 0
					&& high_load_cores[cpu_offset] == 1) {

				__cpufreq_driver_target(current_policy, max_f, CPUFREQ_RELATION_C);
				real_boosted_cores[package_id][cpu_offset] = 1;
				real_boosted_core_num[package_id]++;

			}

			if (prev_boosted_cores[package_id][cpu_offset] == 1
					&& high_load_cores[cpu_offset] == 0) {
				freq_next = min_f + load * (max_f - min_f) / 100;
				freq_next = (freq_next > max_f * (ls_upper_index - RESERVED_FREQ) / FREQ_DIV)
						? max_f * (ls_upper_index - RESERVED_FREQ) / FREQ_DIV : freq_next;
				__cpufreq_driver_target(current_policy, freq_next, CPUFREQ_RELATION_C);

				real_boosted_cores[package_id][cpu_offset] = 0;
				real_boosted_core_num[package_id]--;
			}

			if (prev_boosted_cores[package_id][cpu_offset] == 0
					&& high_load_cores[cpu_offset] == 0) {
				freq_next = min_f + load * (max_f - min_f) / 100;
				freq_next = (freq_next > max_f * (ls_upper_index - RESERVED_FREQ) / FREQ_DIV)
						? max_f * (ls_upper_index - RESERVED_FREQ) / FREQ_DIV : freq_next;
				__cpufreq_driver_target(current_policy, freq_next, CPUFREQ_RELATION_C);
			}
		} else {
			if (prev_boosted_cores[package_id][cpu_offset] == 1
					&& high_load_cores[cpu_offset] == 0) {
				freq_next = min_f + load * (max_f - min_f) / 100;
				freq_next = (freq_next > max_f * (ls_upper_index - RESERVED_FREQ) / FREQ_DIV)
						? max_f * (ls_upper_index - RESERVED_FREQ) / FREQ_DIV : freq_next;
				__cpufreq_driver_target(current_policy, freq_next, CPUFREQ_RELATION_C);

				real_boosted_cores[package_id][cpu_offset] = 0;
				real_boosted_core_num[package_id]--;
			}

			if (prev_boosted_cores[package_id][cpu_offset] == 0
					&& high_load_cores[cpu_offset] == 0) {
				freq_next = min_f + load * (max_f - min_f) / 100;
				freq_next = (freq_next > max_f * (ls_upper_index - RESERVED_FREQ) / FREQ_DIV)
						? max_f * (ls_upper_index - RESERVED_FREQ) / FREQ_DIV : freq_next;
				__cpufreq_driver_target(current_policy, freq_next, CPUFREQ_RELATION_C);
			}

			if (prev_boosted_cores[package_id][cpu_offset] == 0
					&& high_load_cores[cpu_offset] == 1) {
				freq_next = min_f + load * (max_f - min_f) / 100;
				freq_next = (freq_next > max_f * (ls_upper_index - RESERVED_FREQ) / FREQ_DIV)
						? max_f * (ls_upper_index - RESERVED_FREQ) / FREQ_DIV : freq_next;
				__cpufreq_driver_target(current_policy, freq_next, CPUFREQ_RELATION_C);
			}
		}

		for (i = 0; i < loongson_sysconf.cores_per_package; i++) {
			prev_boosted_cores[package_id][i] = real_boosted_cores[package_id][i];
		}
	}

	prev_boost[package_id] = curr_boost;
	mutex_unlock(&boost_mutex[package_id]);
}

static unsigned int boost_dbs_update(struct cpufreq_policy *policy)
{
	struct policy_dbs_info *policy_dbs = policy->governor_data;
	struct dbs_data *dbs_data = policy_dbs->dbs_data;
	struct boost_policy_dbs_info *dbs_info = to_dbs_info(policy_dbs);
	int sample_type = dbs_info->sample_type;

	/* Common NORMAL_SAMPLE setup */
	dbs_info->sample_type = BOOST_NORMAL_SAMPLE;
	/*
	 * OD_SUB_SAMPLE doesn't make sense if sample_delay_ns is 0, so ignore
	 * it then.
	 */
	if (sample_type == BOOST_SUB_SAMPLE && policy_dbs->sample_delay_ns > 0) {
		__cpufreq_driver_target(policy, dbs_info->freq_lo,
					CPUFREQ_RELATION_H);
		return dbs_info->freq_lo_delay_us;
	}

	boost_update(policy);

	if (dbs_info->freq_lo) {
		/* Setup SUB_SAMPLE */
		dbs_info->sample_type = BOOST_SUB_SAMPLE;
		return dbs_info->freq_hi_delay_us;
	}

	return dbs_data->sampling_rate * policy_dbs->rate_mult;
}

/************************** sysfs interface ************************/
static struct dbs_governor boost_dbs_gov;

static ssize_t store_io_is_busy(struct gov_attr_set *attr_set, const char *buf,
				size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_data->io_is_busy = !!input;

	/* we need to re-evaluate prev_cpu_idle */
	gov_update_cpu_data(dbs_data);

	return count;
}

static ssize_t store_up_threshold(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}

	dbs_data->up_threshold = input;
	return count;
}

static ssize_t store_sampling_down_factor(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	struct policy_dbs_info *policy_dbs;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;

	dbs_data->sampling_down_factor = input;

	/* Reset down sampling multiplier in case it was active */
	list_for_each_entry(policy_dbs, &attr_set->policy_list, list) {
		/*
		 * Doing this without locking might lead to using different
		 * rate_mult values in boost_update() and boost_dbs_update().
		 */
		mutex_lock(&policy_dbs->update_mutex);
		policy_dbs->rate_mult = 1;
		mutex_unlock(&policy_dbs->update_mutex);
	}

	return count;
}

static ssize_t store_ignore_nice_load(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == dbs_data->ignore_nice_load) { /* nothing to do */
		return count;
	}
	dbs_data->ignore_nice_load = input;

	/* we need to re-evaluate prev_cpu_idle */
	gov_update_cpu_data(dbs_data);

	return count;
}

static ssize_t store_boost_htrf(struct gov_attr_set *attr_set,const char *buf, size_t count)
{
	struct dbs_data *dbs_data = to_dbs_data(attr_set);
	struct boost_dbs_tuners *boost_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	boost_tuners->boost_htrf = input;
	gov_update_cpu_data(dbs_data);

	return count;
}

gov_show_one_common(sampling_rate);
gov_show_one_common(io_is_busy);
gov_show_one_common(up_threshold);
gov_show_one_common(sampling_down_factor);
gov_show_one_common(ignore_nice_load);
gov_show_one(boost, boost_htrf);

gov_attr_rw(sampling_rate);
gov_attr_rw(io_is_busy);
gov_attr_rw(up_threshold);
gov_attr_rw(sampling_down_factor);
gov_attr_rw(ignore_nice_load);
gov_attr_rw(boost_htrf);

static struct attribute *boost_attributes[] = {
	&sampling_rate.attr,
	&up_threshold.attr,
	&sampling_down_factor.attr,
	&ignore_nice_load.attr,
	&boost_htrf.attr,
	&io_is_busy.attr,
	NULL
};

/************************** sysfs end ************************/

static struct policy_dbs_info *boost_alloc(void)
{
	struct boost_policy_dbs_info *dbs_info;

	dbs_info = kzalloc(sizeof(*dbs_info), GFP_KERNEL);
	return dbs_info ? &dbs_info->policy_dbs : NULL;
}

static void boost_free(struct policy_dbs_info *policy_dbs)
{
	kfree(to_dbs_info(policy_dbs));
}

static int boost_init(struct dbs_data *dbs_data)
{
	struct boost_dbs_tuners *tuners;
	u64 idle_time;
	int cpu;

	tuners = kzalloc(sizeof(*tuners), GFP_KERNEL);
	if (!tuners) {
		return -ENOMEM;
	}

	cpu = get_cpu();
	idle_time = get_cpu_idle_time_us(cpu, NULL);
	put_cpu();
	if (idle_time != -1ULL) {
		/* Idle micro accounting is supported. Use finer thresholds */
		dbs_data->up_threshold = MICRO_FREQUENCY_UP_THRESHOLD;
	} else {
		dbs_data->up_threshold = DEF_FREQUENCY_UP_THRESHOLD;
	}

	dbs_data->sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR;
	dbs_data->ignore_nice_load = 0;
	tuners->powersave_bias = default_powersave_bias;
	tuners->boost_htrf = 0;
	dbs_data->io_is_busy = should_io_be_busy();

	dbs_data->tuners = tuners;

	return 0;
}

static void boost_exit(struct dbs_data *dbs_data)
{
	struct cpufreq_policy *policy;
	int i;

	ls3a4000_freq_table_switch(ls3a4000_normal_table);

	for (i = 0; i < nr_cpu_ids; i++) {
		policy = cpufreq_cpu_get_raw(i);

		cpufreq_frequency_table_cpuinfo(policy, ls3a4000_normal_table);
		load_record[i / MAX_PACKAGES][i % MAX_PACKAGES] = 0;
		prev_boosted_cores[i / MAX_PACKAGES][i % MAX_PACKAGES] = 0;
		real_boosted_cores[i / MAX_PACKAGES][i % MAX_PACKAGES] = 0;
	}


	for (i = 0; i < MAX_PACKAGES; i++) {
		curr_boost_core_num[i] = 0;
		real_boosted_core_num[i] = 0;
	}

	kfree(dbs_data->tuners);
}


static void boost_start(struct cpufreq_policy *policy)
{
	struct boost_policy_dbs_info *dbs_info = to_dbs_info(policy->governor_data);

	dbs_info->sample_type = BOOST_NORMAL_SAMPLE;
	dbs_info->freq_lo = 0;
}

static struct dbs_governor boost_dbs_gov = {
	.gov = CPUFREQ_DBS_GOVERNOR_INITIALIZER("loongson_boost"),
	.kobj_type = { .default_attrs = boost_attributes },
	.gov_dbs_update = boost_dbs_update,
	.alloc = boost_alloc,
	.free = boost_free,
	.init = boost_init,
	.exit = boost_exit,
	.start = boost_start,
};

#define CPU_FREQ_GOV_LOONGSON_BOOST	(&boost_dbs_gov.gov)

static int __init cpufreq_gov_dbs_init(void)
{
	int i;

	if (current_cpu_type() == CPU_LOONGSON3_COMP && dvfs_enabled) {
		for (i = 0; i < MAX_PACKAGES; i++) {
			mutex_init(&boost_mutex[i]);
		}

		return cpufreq_register_governor(CPU_FREQ_GOV_LOONGSON_BOOST);
	} else {
		return -EINVAL;
	}
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	struct cpufreq_policy *policy;
	int i;

	ls3a4000_freq_table_switch(ls3a4000_normal_table);

	for (i = 0; i < nr_cpu_ids; i++) {
		policy =  cpufreq_cpu_get_raw(i);

		cpufreq_frequency_table_cpuinfo(policy, ls3a4000_normal_table);
		load_record[i / MAX_PACKAGES][i % MAX_PACKAGES] = 0;
		prev_boosted_cores[i / MAX_PACKAGES][i % MAX_PACKAGES] = 0;
		real_boosted_cores[i / MAX_PACKAGES][i % MAX_PACKAGES] = 0;
	}

	for (i = 0; i < MAX_PACKAGES; i++) {
		curr_boost_core_num[i] = 0;
		real_boosted_core_num[i] = 0;
	}

	cpufreq_unregister_governor(CPU_FREQ_GOV_LOONGSON_BOOST);
}

MODULE_AUTHOR(" Yijun <yijun@loongson.cn>");
MODULE_DESCRIPTION("'cpufreq_loongson_boost' - A dynamic cpufreq governor for "
	"Loongson boost mode governor");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_LOONGSON_BOOST
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return CPU_FREQ_GOV_LOONGSON_BOOST;
}
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
