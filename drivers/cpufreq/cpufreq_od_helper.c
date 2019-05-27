#include <linux/mutex.h>
#include "cpufreq_od_helper.h"

static DEFINE_MUTEX(cpufreq_od_helper_list_lock);

DEFINE_MUTEX(cpufreq_od_helper_lock);

static LIST_HEAD(cpufreq_od_helper_list);

int cpufreq_od_helper_get_multiplier(void)
{
    struct list_head * it = NULL;
    int    mult = 100;

    mutex_lock(&cpufreq_od_helper_list_lock);
    mutex_lock(&cpufreq_od_helper_lock);    
    list_for_each(it, &cpufreq_od_helper_list) {
        struct cpufreq_od_helper_data *d = list_entry(it, struct cpufreq_od_helper_data, list);
        if (d->enabled) {
            mult = mult * d->multiplier / 100;
        }
    }
    mutex_unlock(&cpufreq_od_helper_lock);
    mutex_unlock(&cpufreq_od_helper_list_lock);
    return mult;
}

int cpufreq_od_helper_register(struct cpufreq_od_helper_data *d)
{
    mutex_lock(&cpufreq_od_helper_list_lock);
    list_add(&d->list, &cpufreq_od_helper_list);
    mutex_unlock(&cpufreq_od_helper_list_lock);    
    return 0;
}

void cpufreq_od_helper_unregister(struct cpufreq_od_helper_data *d)
{
    mutex_lock(&cpufreq_od_helper_list_lock);
    list_del(&d->list);
    mutex_unlock(&cpufreq_od_helper_list_lock);
}



