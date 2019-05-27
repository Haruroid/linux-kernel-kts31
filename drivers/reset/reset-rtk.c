/*
 * reset-rtk.c - Realtek reset controller & reset control
 *
 * Copyright (C) 2016, Realtek Semiconductor Corporation
 *  Cheng-Yu Lee <cylee12@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) "reset-rtk: " fmt

#include <linux/reset-controller.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/reset.h>
#include <linux/reset-helper.h>
#include <asm-generic/io.h>
#include <linux/cpu_pm.h>
#include <linux/suspend.h>
#include <soc/realtek/rtd129x_sb2_sem.h>

static DEFINE_SPINLOCK(rtk_rstn_lock);
static struct sb2_sem *crt_lock = NULL;

static inline void rstn_lock(unsigned long *flags)
{
	spin_lock_irqsave(&rtk_rstn_lock, *flags);

	if (crt_lock)
		sb2_sem_lock(crt_lock, SB2_SEM_ID_SCPU_RESET);

}

static inline void rstn_unlock(unsigned long *flags)
{
	if (crt_lock)
		sb2_sem_unlock(crt_lock);

	spin_unlock_irqrestore(&rtk_rstn_lock, *flags);
}



#define USB_APPLY_NUM 0x100

enum { RESET_GENERAL = 0, RESET_GROUPED, RESET_USB };

struct reset_priv {
    void __iomem *reg;
    struct reset_controller_dev rcdev;
    u32 type;
    u32 data[2];

#ifdef CONFIG_PM
    struct list_head pm_list;
    u32 value;
#endif
    const char *name;
};

#define to_reset_priv(_p) container_of((_p), struct reset_priv, rcdev)

#ifdef CONFIG_PM

static LIST_HEAD(pm_list_head);
extern int rtk_reset_log_enable;
#define PR_INFO_BOOT(...)                 \
    if (unlikely(rtk_reset_log_enable)) { \
        pr_info(__VA_ARGS__);             \
    }


static inline int rtk_rstn_pm_config(struct reset_priv *priv)
{
    list_add(&priv->pm_list, &pm_list_head);
    return 0;
}

static inline int rtk_rstn_pm_save(void)
{
    struct list_head * it;

    if (RTK_PM_STATE == PM_SUSPEND_STANDBY)
        return 0;

    list_for_each(it, &pm_list_head) {
        struct reset_priv *priv = list_entry(it, struct reset_priv, pm_list);
        priv->value = readl(priv->reg);

        pr_debug("%s: save 0x%08x\n", priv->name, priv->value);
    }

    return 0;
}

static inline int rtk_rstn_pm_restore(void)
{
    struct list_head * it;

    if (RTK_PM_STATE == PM_SUSPEND_STANDBY)
        return 0;

    list_for_each_prev(it, &pm_list_head) {
        struct reset_priv *priv = list_entry(it, struct reset_priv, pm_list);
        u32 val = readl(priv->reg);
        u32 val_o = val;
        writel(val, priv->reg);
        
        pr_debug("%s: restore [0x%08x -> 0x%08x]\n", priv->name, val_o, priv->value);
    }
    return 0;
}

static int rtk_rstn_pm_notify(struct notifier_block *notify_block, unsigned long mode, void *unused)
{
    switch (mode) {
    case CPU_CLUSTER_PM_ENTER:
        printk(KERN_INFO "[RESET] Enter %s, act = CPU_CLUSTER_PM_ENTER\n", __func__);
        rtk_rstn_pm_save();
        printk(KERN_INFO "[RESET] Exit %s,  act = CPU_CLUSTER_PM_ENTER\n", __func__);
        break;

    case CPU_CLUSTER_PM_EXIT:
        printk(KERN_INFO "[RESET] Enter %s, act = CPU_CLUSTER_PM_EXIT\n", __func__);
        rtk_rstn_pm_restore();
        printk(KERN_INFO "[RESET] Exit %s,  act = CPU_CLUSTER_PM_EXIT\n", __func__);
        break;

    default:
        break;
    }
    return NOTIFY_OK;
}
static struct notifier_block pm_notifier_block = {
    .notifier_call = rtk_rstn_pm_notify,
};

#endif /* CONFIG_PM */

static int rtk_reset_assert(struct reset_controller_dev *rcdev,
        unsigned long id)
{
    struct reset_priv *priv = to_reset_priv(rcdev);
    u32 val, val_old, mask;
    void __iomem *reg = priv->reg;
    unsigned long flags;

    switch (priv->type) {
    case RESET_USB:
        if (id == USB_APPLY_NUM)
            return -EINVAL;

        if (id >= 32) {
            reg += (id / 32) * 4;
            id %= 32;
        }
    case RESET_GENERAL:
    case RESET_GROUPED: 
        mask = priv->type == RESET_GROUPED ? id : BIT(id);

        rstn_lock(&flags);
        val_old = val = readl(reg);
        val &= ~mask;
        writel(val, reg);
        rstn_unlock(&flags);
        
        PR_INFO_BOOT("%s: ASSERT id=%lu, val=[old: 0x%08x new: 0x%08x]\n", priv->name, id, val_old, val);
        break;
    } /* switch (priv->type) */
    
    return 0;
}

static int rtk_reset_deassert(struct reset_controller_dev *rcdev,
        unsigned long id)
{
    struct reset_priv *priv = to_reset_priv(rcdev);
    u32 val, val_old, mask;
    void __iomem *reg = priv->reg;
    unsigned long flags;

    switch (priv->type) {
    case RESET_GENERAL:
    case RESET_GROUPED:
        mask = priv->type == RESET_GROUPED ? id : BIT(id);

        rstn_lock(&flags);
        val_old = val = readl(reg);
        val |= mask;
        writel(val, reg);  
        rstn_unlock(&flags);

        PR_INFO_BOOT("%s: DEASSERT id=%lu, val=[old: 0x%08x new: 0x%08x]\n", priv->name, id, val_old, val);
        break;

    case RESET_USB:
        rstn_lock(&flags);
        
        /* when id is 0xffffffff, write to memory */
        if (id == USB_APPLY_NUM) {
            int i;
            for (i = 0; i < 2; i++) {
                mask = priv->data[i];

                if (mask) {
                    val_old = val = readl(reg);
                    val |= mask;
                    writel(val, reg);

                    PR_INFO_BOOT("%s: DEASSERT id=%lu, val=[old: 0x%08x new: 0x%08x]\n", 
                        priv->name, id, val_old, val);
                }

                reg += 4;
            }

            priv->data[0] = priv->data[1] = 0;
        } else {
            priv->data[id / 32] |= BIT(id % 32);
        }

        rstn_unlock(&flags);
        break;
    } /* switch (priv->type) */

    return 0;
}

static int rtk_reset_reset(struct reset_controller_dev *rcdev,
        unsigned long id)
{
    int ret;

    ret = rtk_reset_assert(rcdev, id);
    if (ret)
        return ret;

    return rtk_reset_deassert(rcdev, id);
}

static int rtk_reset_status(struct reset_controller_dev *rcdev,
        unsigned long id)
{
    struct reset_priv *priv = to_reset_priv(rcdev);
    u32 val;
    void __iomem * reg = priv->reg;
    unsigned long flags;

    if (id >= 64) 
        return -EINVAL;

    if (id >= 32) {
        id -= 32;
        reg += 4;
    }

    rstn_lock(&flags);
    val = readl(reg);
    rstn_unlock(&flags);


    if (priv->type == RESET_GROUPED) {
        if ((val & id) == id)
            return 0;
        else if ((val & id) == 0)
            return 1;
        else
            return -EINVAL;
    }
    else
        return !(val & BIT(id));
}

static struct reset_control_ops rtk_reset_ops = {
    .assert     = rtk_reset_assert,
    .deassert   = rtk_reset_deassert,
    .reset      = rtk_reset_reset, 
    .status     = rtk_reset_status,
};

static int rtl_reset_of_xlate(struct reset_controller_dev *rcdev,
        const struct of_phandle_args *reset_spec)
{
    struct reset_priv __maybe_unused *priv = to_reset_priv(rcdev);
    int id;
    
    if (reset_spec->args_count != 1)
        return -EINVAL;

    id = (int)reset_spec->args[0];    
    return id;
}


static int __init init_rtk_reset_controller(struct device_node *np)
{
    struct reset_priv * priv;
    const char *name;

    priv = kzalloc(sizeof(struct reset_priv), GFP_KERNEL);
    if (!priv) 
        return -ENOMEM;

    priv->reg = of_iomap(np, 0);
    priv->rcdev.owner = THIS_MODULE;
    priv->rcdev.ops   = &rtk_reset_ops;
    priv->rcdev.of_node = np;
    priv->rcdev.of_reset_n_cells = 2;
    priv->rcdev.of_xlate = rtl_reset_of_xlate;
    
    name = strrchr(np->full_name, '@');
    if (name == NULL)
        name = np->name;
    else
        name ++;
    priv->name = name;

    if (of_find_property(np, "is-grouped", NULL))
        priv->type = RESET_GROUPED;
    else if (of_find_property(np, "is-usb", NULL))
        priv->type = RESET_USB;
    else
        priv->type = RESET_GENERAL;

    reset_controller_register(&priv->rcdev);

#ifdef CONFIG_PM
    if (!of_find_property(np, "ignore-in-pm", NULL))
        rtk_rstn_pm_config(priv);
#endif

    return 0;
}

static int __init init_rtk_reset_control(struct device_node *node)
{
    int num_of_names = of_property_count_strings(node, "reset-names");
    int i;

    for (i = 0; i < num_of_names; ++i) {
        const char * reset_name;
        struct reset_control *rstc;

        if (of_property_read_string_index(node, "reset-names", i, &reset_name))
            continue;

        if (reset_name[0] == '*')
            continue;

        rstc = of_reset_control_get(node, reset_name);
        if (!IS_ERR(rstc)) {
            int ret = rstc_add(rstc, reset_name);
            if (ret)
                reset_control_put(rstc);
        }
    }

    return 0; 
}

static const struct of_device_id rtk_reset_match[] = {
    {.compatible = "realtek,129x-soft-reset",},
    {}
};

static const struct of_device_id rtk_rtsc_init_match[] = {
    {.compatible = "realtek,129x-rstc-init",},
    {}
};

static int __init rtk_init_reset(void)
{
    struct device_node *np;

	crt_lock = sb2_sem_get(0);
	if (IS_ERR(crt_lock)) {
		pr_crit("Fauled to get CRT reg lock\n");
		crt_lock = NULL;
	}

    for_each_matching_node(np, rtk_reset_match) {
        init_rtk_reset_controller(np);
    }

    for_each_matching_node(np, rtk_rtsc_init_match) {   
        init_rtk_reset_control(np);
    }

#ifdef CONFIG_PM
    cpu_pm_register_notifier(&pm_notifier_block);
#endif

    return 0;
}
early_initcall(rtk_init_reset);

static int __init rtk_init_reset_late(void)
{
   rtk_reset_log_enable = 0;
   return 0;
}
late_initcall(rtk_init_reset_late);
