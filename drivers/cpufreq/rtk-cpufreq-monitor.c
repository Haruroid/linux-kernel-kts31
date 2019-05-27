#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/reset.h>
#include <linux/reset-helper.h> 
#include <linux/power-control.h>
#include <linux/workqueue.h>
#include <linux/genhd.h>
#include <linux/blkdev.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/core_control.h>
#include "cpufreq_od_helper.h"
#include <linux/cpufreq.h>

/********************************************************************************
 * Generic
 ********************************************************************************/

#define GPU_SAMPLING_RATE (HZ / 3)

#define SECOND_TO_COUNT(_sec, _config) (_sec) * HZ / (_config)->sampling_rate + 1

static struct dentry *root = NULL;

#define DEBUGFS_ADD_U32_FILE(_v, _d) \
    debugfs_create_u32(#_v, 0444, _d, &(_v));

static struct workqueue_struct *rtk_cpufreq_monitor_queue;

struct rtk_cpufreq_monitor_config {
    struct delayed_work dwork;
    int    sampling_rate;
    struct cpufreq_od_helper_data od_helper;
};


/********************************************************************************
 * Core Control Monitor on CPU Loading 
 ********************************************************************************/
struct __core_priv {
    u64 prev_idle;
    u64 prev_time;
};

static DEFINE_PER_CPU(struct __core_priv, core_priv);

struct load_config {
    struct delayed_work dwork;
    int off_cnt;
    int on_cnt;
    int sampling_rate;
    struct core_controller * cc;
};

struct load_config __config = {
    .sampling_rate = 1 * HZ,    
};

static void monitor_load(struct work_struct *work)
{
    struct load_config *config = 
        container_of(work, struct load_config, dwork.work);
    int cpu;
    int load, load_sum = 0, n_cpus = 0;
    int ret;
    u64 time, idle, d_idle, d_time;

    for_each_online_cpu(cpu) {
        struct __core_priv *priv = &per_cpu(core_priv, cpu);
        idle = get_cpu_idle_time(cpu, &time, 1);

        d_idle = idle - priv->prev_idle;
        d_time = time - priv->prev_time;

        if (d_time <= d_idle)
            load = 0;
        else
            load = div64_u64(100 * (d_time - d_idle), d_time);

        load_sum += load;
        n_cpus += 1;

        priv->prev_idle = idle;
        priv->prev_time = time;        
    }

    load = load_sum / n_cpus;

    if (load <= 4) {
        config->off_cnt ++;
        config->on_cnt = 0;
    } else if (load > 95) {
        config->off_cnt = 0;
        config->on_cnt += 5;
    } else if (load > 70) {
        config->off_cnt = 0;        
        config->on_cnt ++;        
    }

    if (config->off_cnt >= 10) {
        ret = core_control_set_any_cpu_offline(config->cc);
        if (!ret)
            printk(KERN_INFO "CPU Loading monitor set a CPU offline\n");
        config->off_cnt = 0;
    }

    if (config->on_cnt >= 5) {
        ret = core_control_set_any_cpu_online(config->cc);
        if(!ret)
            printk(KERN_INFO "CPU Loading monitor set a CPU online\n");
        config->on_cnt = 0;
    }

    if (core_control_is_enabled())
        mod_delayed_work(rtk_cpufreq_monitor_queue, &config->dwork, config->sampling_rate);
}

static int monitor_load_notifier(struct notifier_block *nb,
    unsigned long event, void *data)
{
    if (event == CORE_CONTROL_ENABLE) {
        struct load_config *config = &__config;
        mod_delayed_work(rtk_cpufreq_monitor_queue, &config->dwork, config->sampling_rate);
        return NOTIFY_OK;
    }
    return NOTIFY_DONE;
}

static struct notifier_block monitor_load_notifier_block = {
    .notifier_call = monitor_load_notifier,
};

static int __init init_monitor_load(void) 
{
    struct load_config *config = &__config;

    INIT_DELAYED_WORK(&config->dwork, monitor_load);
    
    config->cc = core_control_register_controller("cpufreq");
    if (config->cc)
        core_control_set_token_owner(config->cc);
    register_core_control_notifier(&monitor_load_notifier_block);    

    return 0;
}

/********************************************************************************
 *  Monitor GPU
 ********************************************************************************/
struct __gpu_config {
    struct rtk_cpufreq_monitor_config config;
    struct clk *clk_gpu;
    struct power_control *pctrl_gpu;
    int    power_on_score;
};

static struct __gpu_config gpu_priv = {
    .config = {
        .sampling_rate = GPU_SAMPLING_RATE,
        .od_helper = {
            .enabled = 0,
            .multiplier = 301,
        },
    },
    .power_on_score = 0,
};

static void monitor_gpu(struct work_struct *work)
{
    struct rtk_cpufreq_monitor_config *config =
        container_of(work, struct rtk_cpufreq_monitor_config, dwork.work);
    struct __gpu_config *priv =
        container_of(config, struct __gpu_config, config);

    mutex_lock(&cpufreq_od_helper_lock);

    priv->power_on_score = priv->power_on_score * 4 / 10;
    if (power_control_is_powered_on(priv->pctrl_gpu) == 1) {
        priv->power_on_score += 10;
        if (priv->power_on_score >= 16) {
            config->od_helper.enabled = SECOND_TO_COUNT(10, config);
        }
    }
    if (config->od_helper.enabled) {
        config->od_helper.enabled --;
    }

    mutex_unlock(&cpufreq_od_helper_lock);

    mod_delayed_work(rtk_cpufreq_monitor_queue, &config->dwork, config->sampling_rate);
}

static inline __init int init_monitor_gpu(void)
{
    struct rtk_cpufreq_monitor_config *config = &gpu_priv.config;

    gpu_priv.clk_gpu = clk_get(NULL, "clk_gpu");
    gpu_priv.pctrl_gpu = power_control_get("pctrl_gpu");
    if (IS_ERR_OR_NULL(gpu_priv.clk_gpu)
        || IS_ERR_OR_NULL(gpu_priv.pctrl_gpu)) {
        pr_err("%s: Failed to monitor_gpu\n", __func__);
        return -EINVAL;
    }

#ifdef CONFIG_DEBUG_FS
    do {
        struct dentry *dir = debugfs_create_dir("gpu", root);
        debugfs_create_u32("sampling_rate", 0444, dir, &config->sampling_rate);
        debugfs_create_u32("od_helper.enabled", 0444, dir, &config->od_helper.enabled);
        debugfs_create_u32("od_helper.multiplier", 0444, dir, &config->od_helper.multiplier);
        debugfs_create_u32("priv.power_on_score", 0444, dir, &gpu_priv.power_on_score);
    } while (0);
#endif

    cpufreq_od_helper_register(&config->od_helper);

    INIT_DELAYED_WORK(&config->dwork, monitor_gpu);
    queue_delayed_work(rtk_cpufreq_monitor_queue, &config->dwork, 30 * HZ);
    return 0;
}

/********************************************************************************
 * Init
 ********************************************************************************/
static int __init rtk_cpufreq_monitor_init(void)
{

    rtk_cpufreq_monitor_queue = create_workqueue("cpufreq_monitor");

    root = debugfs_create_dir("rtk_cpufreq_monitor", NULL);

    init_monitor_gpu();

    init_monitor_load();

    return 0;    
}
late_initcall(rtk_cpufreq_monitor_init);

