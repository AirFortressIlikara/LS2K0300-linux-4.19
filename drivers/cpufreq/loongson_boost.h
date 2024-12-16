#include "cpufreq_governor.h"

#define FREQ_DIV	(8)
/* Minimal freq level when coprocessor disabled */
#define MIN_LEVEL	4
#define MAX_TABLE          (6)
#define TEMP_UP_LIMIT      (65)
#define CPU_TEMP_REG_ADDR  (0x900000001fe0019c)
#define READTEMPREG (*(volatile unsigned int *)(CPU_TEMP_REG_ADDR))

extern int ls_boost_freq;
extern int ls_stable_base_freq;
extern int ls_boost_supported;
extern int ls_boost_cores;
extern int ls_upper_index;
extern int dvfs_enabled;
extern const int RESERVED_FREQ;

/* Ondemand Sampling types */
enum {BOOST_NORMAL_SAMPLE, BOOST_SUB_SAMPLE};

/* 3a4000 frequency */
enum freq {
	FREQ_LEV0,
	FREQ_LEV1,
	FREQ_LEV2,
	FREQ_LEV3,

	FREQ_LEV4,
	FREQ_LEV5,
	FREQ_LEV6,
	FREQ_LEV7,

	FREQ_LEV8,
	FREQ_LEV9,
	FREQ_LEV10,
	FREQ_LEV11,

	FREQ_LEV12,
	FREQ_LEV13,
	FREQ_LEV14,
	FREQ_LEV15,
};

struct boost_ops {
	unsigned int (*powersave_bias_target)(struct cpufreq_policy *policy,
			unsigned int freq_next, unsigned int relation);
};

struct boost_policy_dbs_info {
	struct policy_dbs_info policy_dbs;
	unsigned int freq_lo;
	unsigned int freq_lo_delay_us;
	unsigned int freq_hi_delay_us;
	unsigned int sample_type:1;
};

static inline struct boost_policy_dbs_info *to_dbs_info(struct policy_dbs_info *policy_dbs)
{
	return container_of(policy_dbs, struct boost_policy_dbs_info, policy_dbs);
}

struct boost_dbs_tuners {
	unsigned int powersave_bias;
	unsigned int boost_htrf;
};

struct temperature_table
{
	int low_temp;
	int high_temp;
	int fix_weight;
};

struct temp_policy
{
	struct temperature_table table[MAX_TABLE];
 };
