/*
 * Realtek Userspace Control
 */
#define pr_fmt(fmt) "uctrl: " fmt

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/reset.h>
#include <linux/reset-helper.h>
#include <linux/power-control.h>
#include <asm/uaccess.h>

struct rtk_miscdevice_priv {
    struct miscdevice mdev;
    struct clk *clk_en_tp;
    struct reset_control *rstn_tp;
};

#define RTK_UCTRL_IOCTL_MAGIC 0xEB
#define RTK_UCTRL_IOCTL_NONE             _IO(RTK_UCTRL_IOCTL_MAGIC, 0)
#define RTK_UCTRL_IOCTL_CLK_EN_TP_SET    _IO(RTK_UCTRL_IOCTL_MAGIC, 1)
#define RTK_UCTRL_IOCTL_CLK_EN_TP_CLEAR  _IO(RTK_UCTRL_IOCTL_MAGIC, 2)
#define RTK_UCTRL_IOCTL_CLK_EN_TP_STATE  _IOR(RTK_UCTRL_IOCTL_MAGIC, 3, unsigned int)
#define RTK_UCTRL_IOCTL_RSTN_TP_SET      _IO(RTK_UCTRL_IOCTL_MAGIC, 4)
#define RTK_UCTRL_IOCTL_RSTN_TP_CLEAR    _IO(RTK_UCTRL_IOCTL_MAGIC, 5)
#define RTK_UCTRL_IOCTL_RSTN_TP_STATE    _IOR(RTK_UCTRL_IOCTL_MAGIC, 6, unsigned int)

static struct rtk_miscdevice_priv priv;
static DEFINE_SEMAPHORE(rtk_uctrl_sem);

static int rtk_uctrl_open(struct inode *inode, struct file *filp)
{
    return 0;
}

static inline int rtk_uctrl_clk_enable(struct clk *clk)
{
    if (!clk)
        return -ENODEV;

    if (!__clk_is_prepared(clk))
        clk_prepare(clk);

    clk_enable(clk);

    return 0;
}

static inline int rtk_uctrl_clk_disable(struct clk *clk)
{
    if (!clk)
        return -ENODEV;

    if (!__clk_is_prepared(clk)) 
        clk_prepare(clk);
    
    clk_disable_unprepare(clk);

    return 0;
}

static inline int rtk_uctrl_clk_deassert(struct reset_control *rstc)
{
    if (!rstc)
        return -ENODEV;

    reset_control_deassert(rstc);
    
    return 0;
}

static inline int rtk_uctrl_clk_assert(struct reset_control *rstc)
{
    if (!rstc)
        return -ENODEV;

    reset_control_assert(rstc);

    return 0;
}

static long rtk_uctrl_ioctl(struct file *filp, unsigned int cmd, 
    unsigned long arg)
{
    int ret = 0;
    unsigned int st;

    down(&rtk_uctrl_sem);

    switch (cmd) {
    case RTK_UCTRL_IOCTL_CLK_EN_TP_SET:
        ret = rtk_uctrl_clk_enable(priv.clk_en_tp);
        break;
    
    case RTK_UCTRL_IOCTL_CLK_EN_TP_CLEAR:
        ret = rtk_uctrl_clk_disable(priv.clk_en_tp);
        break;

    case RTK_UCTRL_IOCTL_CLK_EN_TP_STATE:
        st = __clk_is_enabled(priv.clk_en_tp);
        ret = copy_to_user((unsigned int __user *)arg, 
            &st, sizeof(unsigned int));
        break;

    case RTK_UCTRL_IOCTL_RSTN_TP_SET:
        ret = rtk_uctrl_clk_deassert(priv.rstn_tp);
        break;

    case RTK_UCTRL_IOCTL_RSTN_TP_CLEAR:
        ret = rtk_uctrl_clk_assert(priv.rstn_tp);
        break;

    case RTK_UCTRL_IOCTL_RSTN_TP_STATE:
        st = !reset_control_status(priv.rstn_tp);
        ret = copy_to_user((unsigned int __user *)arg,
            &st, sizeof(unsigned int));
        break;

    default:
        ret = -EFAULT;
        break;
    }

    up(&rtk_uctrl_sem);

    return ret;    
}

static int rtk_uctrl_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static struct file_operations rtk_uctrl_fops = {
    .owner = THIS_MODULE,
    .open = rtk_uctrl_open,
    .unlocked_ioctl = rtk_uctrl_ioctl,
    .release = rtk_uctrl_release,
};

static int __init rtk_uctrl_init(void)
{
    int ret;

    priv.mdev.minor  = MISC_DYNAMIC_MINOR;
    priv.mdev.name   = "uctrl";
    priv.mdev.fops   = &rtk_uctrl_fops;
    priv.mdev.parent = NULL;

    ret = misc_register(&priv.mdev);
    if (ret) {
        pr_err("Failed to register misc device: %d\n", ret);
        return ret;
    }

    priv.clk_en_tp = clk_get(NULL, "clk_en_tp");
    if (IS_ERR(priv.clk_en_tp)) {
        priv.clk_en_tp = NULL;
        pr_err("Failed to get clk_en_tp\n");
    }

    priv.rstn_tp = rstc_get("rstn_tp");
    if (IS_ERR(priv.rstn_tp)) {
         priv.rstn_tp = NULL;
        pr_err("Failed to get rstn_tp\n");
    }

    return 0;
}
late_initcall(rtk_uctrl_init);
