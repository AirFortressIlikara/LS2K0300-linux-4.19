#include <linux/init.h>
#include <asm/time.h>
#include <loongson.h>
#include <asm/cevt-r4k.h>

#define NODE_COUNTER_FREQ cpu_clock_freq
#define NODE_COUNTER_PTR 0x900000003FF00408UL
#define LOONGSON_CSR_NODE_CONTER    0x408

static u64 node_counter_read(struct clocksource *cs)
{
	u64 count;
	unsigned long mask, delta, tmp;
	volatile unsigned long *counter = (unsigned long *)NODE_COUNTER_PTR;

	asm volatile (
		"ld	%[count], %[counter] \n\t"
		"andi	%[tmp], %[count], 0xff \n\t"
		"sltiu	%[delta], %[tmp], 0xf9 \n\t"
		"li	%[mask], -1 \n\t"
		"bnez	%[delta], 1f \n\t"
		"addiu	%[tmp], -0xf8 \n\t"
		"sll	%[tmp], 3 \n\t"
		"dsrl	%[mask], %[tmp] \n\t"
		"daddiu %[delta], %[mask], 1 \n\t"
		"dins	%[mask], $0, 0, 8 \n\t"
		"dsubu	%[delta], %[count], %[delta] \n\t"
		"and	%[tmp], %[count], %[mask] \n\t"
		"dsubu	%[tmp], %[mask] \n\t"
		"movz	%[count], %[delta], %[tmp] \n\t"
		"1:	\n\t"
		:[count]"=&r"(count), [mask]"=&r"(mask),
		 [delta]"=&r"(delta), [tmp]"=&r"(tmp)
		:[counter]"m"(*counter)
	);

	return count;
}

static u64 node_counter_csr_read(struct clocksource *cs)
{
    return csr_readq(LOONGSON_CSR_NODE_CONTER);
}

static void node_counter_suspend(struct clocksource *cs)
{
}

static void node_counter_resume(struct clocksource *cs)
{
}

static struct clocksource csrc_node_counter = {
	.name = "node_counter",
	/* mips clocksource rating is less than 300, so node_counter is better. */
	.rating = 360,
	.read = node_counter_read,
	.mask = CLOCKSOURCE_MASK(64),
	/* oneshot mode work normal with this flag */
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
	.suspend = node_counter_suspend,
	.resume = node_counter_resume,
	.mult = 0,
	.shift = 10,
};

extern void update_clocksource_for_loongson(struct clocksource *cs);
extern unsigned long loops_per_jiffy;

/* Used for adjust clocksource and clockevent for guest
 * when migrate to a diffrent cpu freq
 */
void loongson_nodecounter_adjust(void)
{
	unsigned int cpu;
	struct clock_event_device *cd;
	struct clocksource *cs = &csrc_node_counter;
	u32 cpu_freq;

	cpu_freq = LOONGSON_FREQCTRL(0);

	for_each_online_cpu(cpu){
		cd = &per_cpu(mips_clockevent_device, cpu);
		clockevents_update_freq(cd, cpu_freq / 2);
		cpu_data[cpu].udelay_val = cpufreq_scale(loops_per_jiffy,
					cpu_clock_freq / 1000, cpu_freq / 1000);
	}

	__clocksource_update_freq_scale(cs, 1, cpu_freq);
	update_clocksource_for_loongson(cs);
}

int __init init_node_counter_clocksource(void)
{
	int res;

	if (!NODE_COUNTER_FREQ)
		return -ENXIO;

	/* Loongson3A2000 and Loongson3A3000 has node counter */
	switch(current_cpu_type()){
	case CPU_LOONGSON3:
		switch (read_c0_prid() & 0xF) {
		case PRID_REV_LOONGSON3A_R2_0:
		case PRID_REV_LOONGSON3A_R2_1:
		case PRID_REV_LOONGSON3A_R3_0:
		case PRID_REV_LOONGSON3A_R3_1:
			break;
		case PRID_REV_LOONGSON3A_R1:
		case PRID_REV_LOONGSON3B_R1:
		case PRID_REV_LOONGSON3B_R2:
		default:
			return 0;
		}
		break;
	case CPU_LOONGSON3_COMP:
        if(csr_readq(LOONGSON_CSR_FEATURES) & LOONGSON_CSRF_NODECNT){
            csrc_node_counter.read = node_counter_csr_read;
            break;
        } else
		    return 0;
	default:
		break;
	}
	csrc_node_counter.mult =
		clocksource_hz2mult(NODE_COUNTER_FREQ, csrc_node_counter.shift);
	res = clocksource_register_hz(&csrc_node_counter, NODE_COUNTER_FREQ);
	printk(KERN_INFO "node counter clock source device register\n");

	return res;
}

arch_initcall(init_node_counter_clocksource);
