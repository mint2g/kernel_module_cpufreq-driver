/*
 * Copyright (C) 2016 Psych Half, <psych.half@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 */


#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/cpu.h>
#include <linux/regulator/consumer.h>
#include <asm/system.h>
#include <trace/events/power.h>

#include <mach/hardware.h>
#include <mach/regulator.h>
#include <mach/adi.h>


/* 
 * Cortex-A5 has a single clock source. AHB and AXI are derived from the MCU. 
 * But access to MCU in sc8810 is disabled by Spreadtrum. 
 * There is no register to enable it.
 *
 * So we just get hold of the MPLL and change it directly.
 * However this means we cannot change the AHB and AXI clocks.
 * The AHB and AXI get clock dividers are 4 and  2 respectively.
 *
 * So going anything below 600Mhz, will slow the whole system and memory bus.
 * In other words, lags like hell.
 *
 * We do not know other side effects of changing the MPLL directly, yet.
 *
 */ 

/* voltage constraints */
#define ARMVOLT_MAX (1400 * 1000)
#define ARMVOLT_MIN  (650 * 1000)


#define FREQ_TABLE_SIZE 	(11)
/* overly estimated value of 10ms */
#define TRANSITION_LATENCY	(10 * 1000 * 1000)

/* 
 * WAIT_BOOT_TIME: (jiffies)
 * 	We do not change frequency at early boot up and let the clock stabilize first. 
 * WAIT_TRANS_TIME: (msecs)
 * 	Changing frequency is a very costly operation due to extensive locking. 
 * So we limit actual transistions regardless of the transistion  latency.
 */ 
#define WAIT_BOOT_TIME         (60 * HZ)
#define WAIT_TRANS_TIME		(100)

static DEFINE_MUTEX(freq_lock);
struct cpufreq_freqs global_freqs; 
unsigned long boot_time,trans_time;

/* just for initalization, these will be calculated from the frequency table */
unsigned int freq_min_limit = 600000;
unsigned int freq_max_limit = 1000000;

struct cpufreq_conf {
	struct clk		*clk;
	struct regulator 	*regulator;
	struct cpufreq_frequency_table		*freq_tbl;
	unsigned long		*vdduv_tbl;
	
};

struct cpufreq_table_data {
	struct cpufreq_frequency_table 		freq_tbl[FREQ_TABLE_SIZE];
	unsigned long				vdduv_tbl[FREQ_TABLE_SIZE];
};

enum clocking_levels {
	OC5,OC4,OC3,OC2,OC1,	/* over clock */
	NOC, UC0=NOC, OC0=NOC,	/* no over or under clock */
	UC1, UC2, UC3, UC4,	/* under clock */
	MAX_OC=OC5,MAX_UC=UC4,
	EC, 			/* end of clocking */
};

static struct cpufreq_table_data sc8810_cpufreq_table_data = {
	/* multiplier should be a multiple of 4 to allow efficient scaling */
	.freq_tbl = {		/* M*25 */
		{OC5, 1500000}, /*  60  */
		{OC4, 1400000}, /*  56  */
		{OC3, 1300000},	/*  52  */
		{OC2, 1200000},	/*  48  */
		{OC1, 1100000},	/*  44  */
		{NOC, 1000000},	/*  40  */
		{UC1, 900000},  /*  36  */
		{UC2, 800000},  /*  32  */
		{UC3, 700000},  /*  28  */
		{UC4, 600000},  /*  24  */
		{EC, CPUFREQ_TABLE_END},
	},
	/* 50mV steps */
	.vdduv_tbl = {
	[OC5] =	1100000,
	[OC4] = 1050000,
	[OC3] =	1000000,
	[OC2] =	950000,
	[OC1] =	900000,
	[NOC] =	850000,
	[UC1] =	800000,
	[UC2] =	750000,
	[UC3] =	700000,
	[UC4] =	650000,
	[EC] =	1100000,
	},
};

struct cpufreq_conf sc8810_cpufreq_conf = {
	.clk = NULL,
	.regulator = NULL,
	.freq_tbl = NULL,
	.vdduv_tbl = NULL ,	
};

struct cpufreq_conf *sprd_cpufreq_conf = NULL;




/* clk and regulator api works, so no need to mess with registers */
static inline unsigned long sprd_raw_getfreq(void) {
	return (clk_get_rate(sprd_cpufreq_conf->clk)  / 1000 );
}

static inline unsigned long sprd_raw_getvolt(void) {
	return ( regulator_get_voltage(sprd_cpufreq_conf->regulator));
}

/* do  not call the these functions directly.*/
static inline int sprd_raw_setfreq(unsigned int freq_khz) {	
	return clk_set_rate(sprd_cpufreq_conf->clk, freq_khz*1000 );
}

static inline int sprd_raw_setvolt(unsigned long vdd_uv) {
	return regulator_set_voltage(sprd_cpufreq_conf->regulator, vdd_uv, vdd_uv);
}


/* function to find index when cpufreq core gives us wrong index */
static inline void sprd_find_freqtbl_index (unsigned long freq, unsigned int *index) {
	 int i = 0; 
/* fallback frequency  */
     *index = NOC;

/* ignore the whole target relation crap and use integer division */
/* this should give a frequency pretty close to target */

    // TODO: optimize this loop with MAX_OC, MAX_UC, and NOC
	while ( i < FREQ_TABLE_SIZE) {
		if ((sprd_cpufreq_conf->freq_tbl[i].frequency / 100000 ) == (freq / 100000 ))
			*index = i;
             i++;
	}
}

/* generic cpufreq functions  */
static int sprd_cpufreq_verify_speed(struct cpufreq_policy *policy)
{
	if (policy->cpu) {
		pr_err("cpufreq_verify:  no such cpu id %d\n", policy->cpu);
		return -EINVAL;
	}

	return cpufreq_frequency_table_verify(policy, sprd_cpufreq_conf->freq_tbl);
}

static int sprd_cpufreq_target(struct cpufreq_policy *policy,
		unsigned int target_freq, unsigned int relation )
{
	unsigned int index = NOC;
	unsigned long new_freq, new_volt;

	unsigned int flags = 0;
	unsigned long cur_freq, cur_volt;

	if(time_before(jiffies, boot_time)){
		pr_debug("cpufreq_target: skipping request to scale frequency at early boot %lu %lu", jiffies, boot_time);
		return 0;
	}
	if(time_before(jiffies, trans_time )){
		pr_debug("waiting %ums before changing frequency" , 
				jiffies_to_msecs(trans_time-jiffies));
		return 0;
	}

	/* bail early if requested frequency is above limits */	
	if((target_freq < freq_min_limit ) || (target_freq > freq_max_limit ))
	{
		pr_err("cpufreq_target: invalid target_freq: %u min_limit %u max_limit %d\n", target_freq,
				freq_max_limit, freq_min_limit );
		return -EINVAL;
	}

	cpufreq_frequency_table_target(policy,sprd_cpufreq_conf->freq_tbl, target_freq, relation,&index);

	pr_debug("cpufreq_target: CPU%d target %d policy min,max (%d-%d)",
			policy->cpu, target_freq,
			policy->min, policy->max);

	new_freq = target_freq;

	mutex_lock(&freq_lock);

	if (target_freq == global_freqs.old) {
		mutex_unlock(&freq_lock);
		return 0;
	}

	/* check if we got the right index in the frequency table */
	if(target_freq != sprd_cpufreq_conf->freq_tbl[index].frequency) {
 	sprd_find_freqtbl_index(target_freq,&index);
     new_freq = sprd_cpufreq_conf->freq_tbl[index].frequency;
	}

    global_freqs.new = new_freq;
	new_volt =  sprd_cpufreq_conf->vdduv_tbl[index];

	pr_info("check_setfreq: perpare to set %lu khz , %luuV for cpu%u\n",
			new_freq, new_volt, policy->cpu);

		cpufreq_notify_transition(&global_freqs, CPUFREQ_PRECHANGE);

	/* The below locking part is very critical, a spinlock won't suffice here. 
	 * We need to disable the second OS from taking the CPU.
	 * So we must use the VLX specific hw_local_irq lock.
	 * Which is a very heavvy lock, so we  try to hold it for a very short time.
	 * */
	flags = hw_local_irq_save();
	cur_freq =  sprd_raw_getfreq();
	if (new_freq != cur_freq) {
	if (new_freq > cur_freq)
	sprd_raw_setvolt(new_volt );
	sprd_raw_setfreq(new_freq);
	if (new_freq < cur_freq)
	sprd_raw_setvolt(new_volt);
	}
	cur_freq = sprd_raw_getfreq();
	cur_volt = sprd_raw_getvolt();
	hw_local_irq_restore(flags);

	if (new_freq == cur_freq && new_volt == cur_volt)   {
		new_volt=cur_volt;
		new_freq=cur_freq;
		pr_info("setting suceessful new values: freq %luMHz volt: %lumV \n", new_freq/1000, new_volt/1000);	
	} else {
		new_volt=cur_volt;
		new_freq=cur_freq;
		pr_info("setting fail current values: freq %luMHz volt: %lumV \n", new_freq/1000, new_volt/1000);	
}

	cpufreq_notify_transition(&global_freqs, CPUFREQ_POSTCHANGE);

	global_freqs.old = global_freqs.new;     trans_time=jiffies+msecs_to_jiffies(WAIT_TRANS_TIME);

	mutex_unlock(&freq_lock);

	return 0;
}

static unsigned int sprd_cpufreq_getspeed(unsigned int cpu)
{
	if (cpu)
		return -EINVAL;

	return sprd_raw_getfreq();
}

static void sprd_gen_freq_table(void)
{
	int i;

	/* initalize frequency table */
	sprd_cpufreq_conf->freq_tbl = sc8810_cpufreq_table_data.freq_tbl;
	sprd_cpufreq_conf->vdduv_tbl = sc8810_cpufreq_table_data.vdduv_tbl;
	/* calculate min and max frequency */
	freq_max_limit = sprd_cpufreq_conf->freq_tbl[0].frequency;

	for (i = 1; i < FREQ_TABLE_SIZE; i++) {
		if (sprd_cpufreq_conf->freq_tbl[i].frequency == CPUFREQ_TABLE_END)
			break;
	}
	freq_min_limit = sprd_cpufreq_conf->freq_tbl[i-1].frequency;

	pr_info("gen_freq_table:  min limit=%dKhz, max limit=%dKhz \n" , freq_min_limit, freq_max_limit);

	return;
}

static int sprd_cpufreq_init(struct cpufreq_policy *policy)
{
	int ret;

	/* get the actual frequency, first */
	policy->cur = sprd_raw_getfreq();

	policy->cpuinfo.transition_latency = TRANSITION_LATENCY;

	ret=cpufreq_frequency_table_cpuinfo(policy, sprd_cpufreq_conf->freq_tbl);


	if (ret) {
		pr_err("cpufreq_init: failed to config freq table: %d\n", ret);
		return ret;
	}

	/* do not switch frequencies unless explicitly asked us to */
	policy->max = sprd_cpufreq_conf->freq_tbl[NOC].frequency ;
	policy->min = sprd_cpufreq_conf->freq_tbl[NOC].frequency ;
	cpufreq_frequency_table_get_attr(sprd_cpufreq_conf->freq_tbl, policy->cpu);

	pr_info("cpufreq_init: policy: cpu=%d, cur=%u, min=%u, max=%u, ret=%d\n",
			policy->cpu, policy->cur, policy->min,policy->max, ret);

	return ret;
}

static int sprd_cpufreq_exit(struct cpufreq_policy *policy)
{
	/* empty for now  */
	return 0;
}

static struct freq_attr *sprd_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

ssize_t sprd_vdd_get(char *buf) {
	int i, len = 0;
	for (i = 0; i <= MAX_UC; i++) {
		len += sprintf(buf + len, "%umhz: %lu mV\n", sprd_cpufreq_conf->freq_tbl[i].frequency / 1000, sprd_cpufreq_conf->vdduv_tbl[i] / 1000);
	}
	return len;
}

void sprd_vdd_set(const char *buf) {
	int ret = -EINVAL;
	int i = 0;
	int j = 0;
	int u[MAX_UC + 1];
	while (j < MAX_UC + 1) {
		int consumed;
		int val;
		ret = sscanf(buf, "%d%n", &val, &consumed);
		if (ret > 0) {
			buf += consumed;
			u[j++] = val;
		}
		else {
			break;
		}
	}

	for (i = 0; i < j; i++) {
		if (u[i] > ARMVOLT_MAX / 1000) {
			u[i] = ARMVOLT_MAX / 1000;
		}
         if( u[i] % 25 == 0 ) {
		 sprd_cpufreq_conf->vdduv_tbl[i] = u[i] * 1000; }
	}
   return;
}

static struct vdd_levels_control sprd_vdd_control = {
      .get = sprd_vdd_get,
      .set = sprd_vdd_set,
};

static struct cpufreq_driver sprd_cpufreq_driver = {
	.verify		= sprd_cpufreq_verify_speed,
	.target		= sprd_cpufreq_target,
	.get		= sprd_cpufreq_getspeed,
	.init		= sprd_cpufreq_init,
	.exit		= sprd_cpufreq_exit,
	.name		= "cpufreq_sc8810",
	.attr		= sprd_cpufreq_attr,
	.volt_control = &sprd_vdd_control ,
};


static int sprd_cpufreq_policy_notifier(
		struct notifier_block *nb, unsigned long event, void *data)
{
	/* empty for now */
	return NOTIFY_OK;
}

static struct notifier_block sprd_cpufreq_policy_nb = {
	.notifier_call = sprd_cpufreq_policy_notifier,
};

static int __init sprd_cpufreq_modinit(void)
{
	int ret;  
	unsigned int reg_success = 0x1;

	pr_info("cpufreq driver module for sc8810 initializing.. \n"); 
	pr_info("number of cpus %d \n", NR_CPUS);


	/* skip checks for if  we are actually running an sc8810 chip for now */
	sprd_cpufreq_conf = &sc8810_cpufreq_conf;
	sprd_gen_freq_table();

	sprd_cpufreq_conf->clk = clk_get_sys(NULL, "mpll_ck");
	if (IS_ERR(sprd_cpufreq_conf->clk)) {
		pr_err("modinit: unable to get clk_mcu %ld\n",
				PTR_ERR(sprd_cpufreq_conf->clk) );
		return 0;
	} else {
		pr_info("modinit: got clk_mcu" );
	}


	sprd_cpufreq_conf->regulator = regulator_get(NULL, "VDDARM");
	if (IS_ERR(sprd_cpufreq_conf->regulator)) { 
		pr_err("modinit: unable to get regulator %ld\n",
				PTR_ERR(sprd_cpufreq_conf->regulator ) );
		return 0;
	} else {
		pr_info("modinit: got regulator" );
	}

	boot_time=WAIT_BOOT_TIME;
	trans_time=jiffies - msecs_to_jiffies(WAIT_BOOT_TIME*10);
	global_freqs.old = sprd_raw_getfreq();

	pr_info("modinit: old_frequency: %dKhz, old_volt: %lu mV",  global_freqs.old, sprd_raw_getvolt() / 1000 );

	ret = cpufreq_register_driver(&sprd_cpufreq_driver); 

	if (!ret){
		pr_info("sucessufully registered cpufreq driver \n");
	} else { 
		pr_err("unable to register cpufreq driver %d\n", ret);
		reg_success = 0; 
		return 0;
	}

	ret = cpufreq_register_notifier(
			&sprd_cpufreq_policy_nb, CPUFREQ_POLICY_NOTIFIER);

	if (!ret){
		pr_info("sucessufully registered cpufreq notifier \n");
	} else { 
		pr_err("unable to register cpufreq notifier %d\n",ret);
		reg_success = 0; 
	}

	if(!reg_success) {
		regulator_put(sprd_cpufreq_conf->regulator);
	} 
	return 0;
}

static void __exit sprd_cpufreq_modexit(void)
{
	if (!IS_ERR_OR_NULL(sprd_cpufreq_conf->regulator))
		regulator_put(sprd_cpufreq_conf->regulator);

	pr_info("unregistering  driver \n"); cpufreq_unregister_driver(&sprd_cpufreq_driver);

	pr_info("unregistering notifier \n");
	cpufreq_unregister_notifier(
			&sprd_cpufreq_policy_nb, CPUFREQ_POLICY_NOTIFIER);

}

module_init(sprd_cpufreq_modinit);
module_exit(sprd_cpufreq_modexit);

MODULE_AUTHOR("Psych Half, <psych.half@gmail.com>");
MODULE_DESCRIPTION("cpufreq driver for sc8810");
MODULE_LICENSE("GPL");
