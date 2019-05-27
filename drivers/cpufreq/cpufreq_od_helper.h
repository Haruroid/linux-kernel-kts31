#ifndef __CPUFREQ_OD_HELPER_H_
#define __CPUFREQ_OD_HELPER_H_


struct cpufreq_od_helper_data {
    struct list_head list;
    int enabled;
    int multiplier;    
};

#ifdef CONFIG_CPUFREQ_OD_HELPER

int cpufreq_od_helper_get_multiplier(void);
int cpufreq_od_helper_register(struct cpufreq_od_helper_data *d);
void cpufreq_od_helper_unregister(struct cpufreq_od_helper_data *d);

extern struct mutex cpufreq_od_helper_lock;


#else

static inline int cpufreq_od_helper_get_multiplier(void) 
{
    return 100;    
}

static inline int cpufreq_od_helper_register(struct cpufreq_od_helper_data *d)
{
    return 0;    
}

static inline void cpufreq_od_helper_unregister(struct cpufreq_od_helper_data *d)
{
}

static DEFINE_MUTEX(cpufreq_od_helper_lock);

#endif /* CONFIG_CPUFREQ_OD_HELPER */

#endif
