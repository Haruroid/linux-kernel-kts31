#define pr_fmt(fmt) "cpufreq: " fmt

#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <soc/realtek/rtd129x_cpu.h>

static void devm_table_release(struct device *dev, void *res)
{
    kfree(*(void**)res);
}

static int devm_add_table(struct device *dev, void *table)
{
    void **ptr;
    ptr = devres_alloc(devm_table_release, sizeof(*ptr), GFP_KERNEL);
    if (!ptr)
        return -ENOMEM;
    
    *ptr = table;
    devres_add(dev, ptr);
    return 0;    
}

enum dvfs_voltage_state {
    VOLT_UNCHECK = 0, 
    VOLT_VALID, 
    VOLT_INVALID,
};

struct voltage_table {
    int state;
    unsigned int vdd_min;
    unsigned int vdd_max;
};

static struct clk *cpu_clk;
static struct cpufreq_frequency_table *freq_table;
static unsigned int cpu_transition_latency = 0;
static int max_index = 0;

static struct regulator *cpu_regulator;
static struct voltage_table *volt_table;
static const unsigned long cpu_regulator_latency = 100000;
static struct regulator *l2_regulator;
static struct voltage_table *l2_volt_table;    
static DEFINE_MUTEX(cpufreq_lock);
static bool virtual_freq_en = false;
static unsigned int virtual_freq_threshold = -1;
static unsigned int virtual_freq_stor = 0;

#define SET_REGULATOR_VOLTAGE(_regulator, _volt) \
    if (_regulator) { \
        int ret = regulator_set_voltage((_regulator), \
            (_volt)->vdd_min, (_volt)->vdd_max); \
        if (ret) { \
            pr_err("Failed to set %s's voltage to %d uV, to: %d\n", \
                #_regulator, (_volt)->vdd_min, ret); \
            return ret; \
        } \
    }

#define ABS(x) ((x) < 0 ? -(x) : (x))
#define ROUND_RANGE 5000

/**
 *  realtek_129x_cpufreq_get_speed - get CPU speed
 *  @cpu: the target cpu to get speed
 *
 *  make sure the booting frequency in freq_table to prevent Kernel
 *  Panic when using cpufreq_stat
 *
 *  Return the rounded clock rate from pll.
 */
static unsigned int realtek_129x_cpufreq_get_speed(unsigned int cpu)
{
    int clk_rate; 
    const struct cpufreq_frequency_table *freq;

    if (virtual_freq_en && virtual_freq_stor > virtual_freq_threshold)
        return virtual_freq_stor;
    
    clk_rate = clk_get_rate(cpu_clk) / 1000;

    freq = freq_table;
    while (freq->frequency != CPUFREQ_TABLE_END) {
        
        if (ABS((int)freq->frequency - clk_rate) < ROUND_RANGE) {
            clk_rate = freq->frequency;
            break;
        }
        freq++;
    }

    pr_debug("%s: %d KHz", __func__, clk_rate);

    return clk_rate;
}

/**
 * __realtek_129x_cpufreq_set_target_index - target_index callback function without lock
 * @policy
 * @inx
 *
 *  This callback function is not used directly with the following reason(s)
 *   1. free frequency value for userspace GOV
 *   2. virtual frequency
 *
 *  Return 0 on success.
 */
static int __realtek_129x_cpufreq_set_target_index(struct cpufreq_policy *policy, 
    unsigned int idx)
{
    int ret;
    unsigned int freq_old = policy->cur;
    unsigned int freq_new = freq_table[idx].frequency;
    int volt_idx = freq_table[idx].driver_data;

    if (freq_new > freq_old) {
        SET_REGULATOR_VOLTAGE(cpu_regulator, &volt_table[volt_idx]);
        SET_REGULATOR_VOLTAGE(l2_regulator, &l2_volt_table[volt_idx]);        
        udelay(50);
    }

    ret = clk_set_rate(cpu_clk, freq_new * 1000);
    if (ret) {
        pr_err("Failed to set clk to %d MHz: %d\n", freq_new / 1000, ret);
        return ret;
    }

    policy->cur = freq_new;

    if (freq_new < freq_old) {
        udelay(50);
        SET_REGULATOR_VOLTAGE(cpu_regulator, &volt_table[volt_idx]);
        SET_REGULATOR_VOLTAGE(l2_regulator, &l2_volt_table[volt_idx]);
    }   

    return 0;
}

static int realtek_129x_cpufreq_set_target(struct cpufreq_policy *policy,
        unsigned int target_freq, unsigned int relation)
{
    int ret;
    unsigned int idx;
    struct cpufreq_freqs freqs;

    if (!strcmp(policy->governor->name, "userspace")) {

        pr_debug("[USERSPACE GOV] set frequency to %d MHz\n",
            target_freq / 1000);

        mutex_lock(&cpufreq_lock);
        ret = clk_set_rate(cpu_clk, target_freq * 1000);
        policy->cur = target_freq;
        mutex_unlock(&cpufreq_lock);

        return ret;
    }

    virtual_freq_stor = target_freq;
    if (virtual_freq_en && target_freq >= virtual_freq_threshold) {
        pr_debug("[VF] target_freq = %d MHz\n", virtual_freq_threshold/1000);
        target_freq = virtual_freq_threshold;
    }

    ret = cpufreq_frequency_table_target(policy, freq_table,
        target_freq, relation, &idx);
    if (ret) {
        pr_err("Failed to get index from freq_table %d\n", ret);
        return ret;
    }

    freqs.old = policy->cur;
    freqs.new = freq_table[idx].frequency;

    if (freqs.old == freqs.new) 
        return 0;
    
    cpufreq_freq_transition_begin(policy, &freqs);

    mutex_lock(&cpufreq_lock);
    ret = __realtek_129x_cpufreq_set_target_index(policy, idx);
    mutex_unlock(&cpufreq_lock);

    cpufreq_freq_transition_end(policy, &freqs, ret);

    return ret;
}

static inline int __init check_voltage(struct regulator *regulator,
    struct voltage_table *item)
{
    if (item->state == VOLT_INVALID)
        return -EINVAL;
    if (item->state == VOLT_VALID)
        return 0;

    item->state = VOLT_INVALID;
    if (regulator_is_supported_voltage(regulator, 
        item->vdd_min, item->vdd_max)) {
        item->state = VOLT_VALID;
        return 0;
    }

    return -EINVAL;
}

static void __init realtek_129x_check_frequency_voltage(void)
{
    int count, ret;
    struct cpufreq_frequency_table *freq;

    if (!cpu_regulator) 
        return;

    count = regulator_count_voltages(cpu_regulator);
    if (count <= 0) {
        pr_err("Failed to check supported voltages: %d\n", count);
        goto done;
    }

    for (freq = freq_table;
        freq->frequency != CPUFREQ_TABLE_END;
        freq++) {
        if (freq->frequency == CPUFREQ_ENTRY_INVALID) 
            continue;

        /* check the voltage is in the cpu regulator */
        ret = check_voltage(cpu_regulator, &volt_table[freq->driver_data]);
        if (ret) {
            pr_err("Unspport CPU voltage for %d MHz: %d\n", 
                freq->frequency/1000, ret);
            freq->frequency = CPUFREQ_ENTRY_INVALID;
        } else {
            max_index = (freq - freq_table);
        }

        /* check the l2 volatage table */
        if (!l2_regulator)
            continue;

        ret = check_voltage(l2_regulator, &l2_volt_table[freq->driver_data]);
        if (ret) {
            pr_warn("Unspport L2 voltage for %d MHz: %d\n",
                 freq->frequency/1000, ret);
        }      
    }
done:
    return;
}

static int realtek_129x_cpufreq_driver_init(struct cpufreq_policy *policy)
{
    unsigned int transition_latency;

    /* Datasheet says PLL stabalisation time (if we were to use
     * the PLLs, which we don't currently) is ~300us worst case,
     * but add some fudge.
     *
     * NOTE: transition_latency is used as the sampling rate in
     * the ondemand governor, which defined as following
     *    
     *  sampling_rate(us) =
     *      transition_latency(ns) / 1000 * LATENCY_MULTIPLIER
     *
     * Default value of LATENCY_MULTIPLIER is 1000. Please see
     * the macro defined in cpufreq.h to get current value.
     */
    transition_latency = cpu_transition_latency + cpu_regulator_latency;

    realtek_129x_check_frequency_voltage();

    policy->suspend_freq = freq_table[max_index].frequency;

    return cpufreq_generic_init(policy, freq_table, transition_latency);
}

int realtek_129x_cpufreq_voltage_reset(void)
{
    int volt_idx = freq_table[max_index].driver_data;
    SET_REGULATOR_VOLTAGE(cpu_regulator, &volt_table[volt_idx]);
    SET_REGULATOR_VOLTAGE(l2_regulator, &l2_volt_table[volt_idx]);
    return 0;
}
EXPORT_SYMBOL(realtek_129x_cpufreq_voltage_reset);

static struct cpufreq_driver realtek_129x_cpufreq_driver = {
    .name   = "realtek",
    .flags  = 0/*CPUFREQ_CONST_LOOPS*/,
    .verify = cpufreq_generic_frequency_table_verify,
    .target = realtek_129x_cpufreq_set_target,
    .get    = realtek_129x_cpufreq_get_speed,
    .init   = realtek_129x_cpufreq_driver_init,
#ifdef CONFIG_PM
    .suspend = cpufreq_generic_suspend,
#endif
};

static void rtk_dvfs_shutdown(struct platform_device *pdev)
{
    realtek_129x_cpufreq_voltage_reset();
}

static int of_read_frequency_table(struct device_node *np, const char *table_name,
    struct cpufreq_frequency_table **out)
{
#define FREQUENCY_COLUMN 3
    int size, i;
    struct cpufreq_frequency_table *table;
    const u32 *prop = of_get_property(np, table_name, &size);

    size /= sizeof(u32);
    if (prop && (size % FREQUENCY_COLUMN) == 0) {
        int count = size / FREQUENCY_COLUMN;
        table = kcalloc(count + 1, sizeof(*table), GFP_KERNEL);
        if (!table)
            return -ENOMEM;

        pr_info("freq_table: %s:\n", table_name);
        
        for (i = 0; i < count; i++) {
            table[i].flags       = of_read_number(prop++, 1);
            table[i].driver_data = of_read_number(prop++, 1);
            table[i].frequency   = of_read_number(prop++, 1);
            pr_info("%2d .volidx = %d .freq = %7d \n", 
                i, table[i].driver_data, table[i].frequency);
        }
        
        table[i].flags       = 0;
        table[i].driver_data = 0;
        table[i].frequency   = CPUFREQ_TABLE_END;

        *out = table;
        return 0;
    }

    return -EINVAL;
}

static int devm_of_read_frequency_table(struct device *dev, struct device_node *np,
    const char *table_name, struct cpufreq_frequency_table **out)
{
    struct cpufreq_frequency_table *ft;
    int ret;

    ret = of_read_frequency_table(np, table_name, &ft);
    if (ret)
        return ret;

    ret = devm_add_table(dev, ft);
    if (ret) {
        kfree(ft);
        return ret;
    }

    *out = ft;
    return 0;
}


static int of_read_voltage_table(struct device_node *np, const char *table_name, 
    struct voltage_table **out)
{
#define VOLTAGE_COLUMN 3
    int size, i;
    struct voltage_table *table;
    const u32 *prop = of_get_property(np, table_name, &size);

    size /= sizeof(u32);
    if (prop && (size % VOLTAGE_COLUMN) == 0) {
        int count = size / VOLTAGE_COLUMN;
        table = kcalloc(count, sizeof(*table), GFP_KERNEL);
        if (!table)
            return -ENOMEM;

        pr_info("volt_table: %s:\n", table_name);

        for (i = 0; i < count; i++) {
            int index = of_read_number(prop++, 1);
            table[index].state = VOLT_UNCHECK;
            table[index].vdd_min = of_read_number(prop++, 1);
            table[index].vdd_max = of_read_number(prop++, 1);
            
            pr_info("%d .vdd_min = %7d .vdd_max = %7d \n",
                index, table[index].vdd_min, table[index].vdd_max);
        }

        *out = table;
        return 0;
    }
    return -EINVAL;
}

static int devm_of_read_voltage_table(struct device *dev, struct device_node *np, 
    const char *table_name, struct voltage_table **out) 
{
    struct voltage_table *vt;
    int ret;

    ret = of_read_voltage_table(np, table_name, &vt);
    if (ret)
        return ret;

    ret = devm_add_table(dev, vt);
    if (ret) {
        kfree(vt);
        return ret;
    }

    *out = vt;
    return 0;
}

static int rtk_dvfs_probe(struct platform_device *pdev)
{
    const char *prop_name;
    struct device_node *np = pdev->dev.of_node;
    int ret = 0;
    int rev = get_rtd129x_cpu_revision() >> 16;
    char ft_name[30];

    dev_dbg(&pdev->dev, "%s", __func__);

    cpu_clk = devm_clk_get(&pdev->dev, "spll");
    if (IS_ERR(cpu_clk)) {
        ret = PTR_ERR(cpu_clk);
        dev_err(&pdev->dev, "Failed to get CPU clk: %d\n", ret);
        return ret;
    }

#ifdef CONFIG_REGULATOR

#if defined(CONFIG_REGULATOR_RTKGPIO) && defined(CONFIG_REGULATOR_G2227)
    #warning "Multiple regulator is configured."
    dev_alert(&pdev->dev, "Multiple regulator is configured.\n");  
#endif

#ifdef CONFIG_REGULATOR_RTKGPIO
    dev_dbg(&pdev->dev, "regulator: rtkgpio_regulator\n");
    cpu_regulator = devm_regulator_get(&pdev->dev, "rtkgpio_regulator");
    l2_regulator  = NULL;
#elif defined(CONFIG_REGULATOR_G2227)
    dev_dbg(&pdev->dev, "regulator: g2227\n");
    cpu_regulator = devm_regulator_get(&pdev->dev, "dcdc2");
    l2_regulator  = devm_regulator_get(&pdev->dev, "ldo3");
#endif

#else /* !defined(CONFIG_REGULATOR) */
    dev_dbg(&pdev->dev, "regulator: none\n");
    cpu_regulator = NULL;
    l2_regulator  = NULL;
#endif

    if (IS_ERR(cpu_regulator)) {
        ret = PTR_ERR(cpu_regulator);
        dev_err(&pdev->dev, "Failed to get CPU regulator: %d\n", ret);
        return ret;
    }

    if (IS_ERR(l2_regulator)) {
        ret = PTR_ERR(l2_regulator);
        dev_warn(&pdev->dev, "Failed to get L2 regulator: %d\n", ret);
        l2_regulator = NULL;
    }

    prop_name = "transition_latency";
    ret = of_property_read_u32(np, prop_name, &cpu_transition_latency);
    if (ret) {
        dev_err(&pdev->dev, "Failed to read %s: %d\n", prop_name, ret);
        return ret;
    }
    
    sprintf(ft_name, "frequency-table,rev%d", rev);
    prop_name = of_find_property(np, ft_name, NULL) ? ft_name : "frequency-table";
    ret = devm_of_read_frequency_table(&pdev->dev, np, prop_name, &freq_table);
    if (ret) {
        dev_err(&pdev->dev, "Failed to read %s: %d\n", prop_name, ret);
        return ret;
    }

    if (cpu_regulator) {
        prop_name = "voltage-table";
        ret = devm_of_read_voltage_table(&pdev->dev, np, prop_name, &volt_table);
        if (ret) {
            dev_err(&pdev->dev, "Failed to read %s: %d\n", prop_name, ret);
            return ret;
        }
    }

    if (l2_regulator) {
        prop_name = "l2-voltage-table";    
        ret = devm_of_read_voltage_table(&pdev->dev, np, prop_name, &l2_volt_table);
        if (ret) {
            dev_err(&pdev->dev, "Failed to read %s: %d\n", prop_name, ret);
            return ret;
        }        
    }

    virtual_freq_en = false;
    prop_name = "virtual-freq-threshold";
    if (!of_property_read_u32(np, prop_name, &virtual_freq_threshold)) {
        virtual_freq_en = true;

        dev_info(&pdev->dev, "[VF] virtual frequncy: enabled\n");
        dev_info(&pdev->dev, "[VF] %s = %u KHz\n", prop_name, virtual_freq_threshold);
    }

    ret = cpufreq_register_driver(&realtek_129x_cpufreq_driver);
    if (ret) {
        dev_err(&pdev->dev, "Failed to register cpufreq driver: %d\n", ret);
        return ret;
    }

    dev_dbg(&pdev->dev, "%s done", __func__);

    return 0;
}

static struct of_device_id rtk_dvfs_ids[] = {
#ifdef CONFIG_REGULATOR_RTKGPIO
    {.compatible = "Realtek,rtk129x-gpio-dvfs" },
#else /* defined(CONFIG_REGULATOR_G2227) || !defined(CONFIG_REGULATOR) */
    {.compatible = "Realtek,rtk129x-dvfs" },
#endif
    { /* Sentinel */ },
};

static struct platform_driver realtek_cpufreq_platdrv = {
    .driver = {
        .name   = "rtk129x-cpufreq",
        .of_match_table = rtk_dvfs_ids,
    },
    .probe      = rtk_dvfs_probe,
    .shutdown   = rtk_dvfs_shutdown,
};

module_platform_driver_probe(realtek_cpufreq_platdrv, rtk_dvfs_probe);

