/*
 * drivers/pwm/pwm-rtd1295.c
 *
 * rtd1295 pulse-width-modulation controller driver
 *
 * Copyright (c) 2014, Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pwm.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

// RTK define here
#define RTK_DEF_PWM_IDX     (0)     // default use PWM idx set to 0

#define RTK_ADDR_PWM_OCD    (0x0)
#define RTK_ADDR_PWM_CD     (0x4)
#define RTK_ADDR_PWM_CSD    (0x8)

#define RTK_PWM_OCD_SHIFT       (8)
#define RTK_PWM_CD_SHIFT        (8)
#define RTK_PWM_CSD_SHIFT       (4)

#define RTK_PWM_OCD_MASK       (0xff)
#define RTK_PWM_CD_MASK        (0xff)
#define RTK_PWM_CSD_MASK       (0xf)

#define RTK_PWM_OCD_DEFAULT      	(0xff)
#define RTK_PWM_CD_DEFAULT       	(0x1)
#define RTK_PWM_CSD_DEFAULT     	(0x1)

#define NUM_PWM 4


struct RTK119X_pwm_map {
	int duty_rate;
	int ocd_data;
	int cd_data;
};

static const struct RTK119X_pwm_map rtk129x_maps[] = {
	{100, 1, 1}, // 1
	{80, 4, 3},  // 4/5
	{75, 3, 2},  // 3/4
	{66, 2, 1},  // 2/3
	{60, 4, 2},  // 3/5
	{50, 3, 1},  // 1/2
	{40, 4, 1},  // 2/5
	{33, 5, 1},  // 1/3
	{25, 3, 0},  // 1/4
	{20, 3, 0},  // 1/5
	{0, 0, 0},   // 0
};

struct rtd1295_pwm_chip {
	struct pwm_chip		chip;
	struct device		*dev;
	void __iomem		*mmio_pwm_reg_base;
	int			proc_id;
	int			pwm_idx;

	int			base_freq;
	int			out_freq; //Hz
	int			duty_rate;
	int			enable;
	int			clksrc_div;
	int			clkout_div;
	int			clk_duty;
};

ssize_t pwm_show_dutyRate(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t pwm_store_dutyRate(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);

ssize_t pwm_show_enable(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t pwm_store_enable(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);

ssize_t pwm_show_clksrcDiv(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t pwm_store_clksrcDiv(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);

ssize_t pwm_show_clkoutDiv(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t pwm_store_clkoutDiv(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);

ssize_t pwm_show_out_freq(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t pwm_store_out_freq(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);

static DEVICE_ATTR(dutyRate, ((S_IRUGO | S_IWUGO) & ~S_IWOTH), pwm_show_dutyRate, pwm_store_dutyRate);
static DEVICE_ATTR(enable, ((S_IRUGO | S_IWUGO) & ~S_IWOTH), pwm_show_enable, pwm_store_enable);
static DEVICE_ATTR(clksrcDiv, ((S_IRUGO | S_IWUGO) & ~S_IWOTH), pwm_show_clksrcDiv, pwm_store_clksrcDiv);
static DEVICE_ATTR(clkoutDiv, ((S_IRUGO | S_IWUGO) & ~S_IWOTH), pwm_show_clkoutDiv, pwm_store_clkoutDiv);
static DEVICE_ATTR(out_freq, ((S_IRUGO | S_IWUGO) & ~S_IWOTH), pwm_show_out_freq, pwm_store_out_freq);

static struct attribute *pwm_dev_attrs[] = {
	&dev_attr_dutyRate.attr,
	&dev_attr_enable.attr,
	&dev_attr_clksrcDiv.attr,
	&dev_attr_clkoutDiv.attr,
	&dev_attr_out_freq.attr,
	NULL,
};

static struct attribute_group pwm_dev_attr_group = {
	.attrs		= pwm_dev_attrs,
};

static inline struct rtd1295_pwm_chip *to_rtd1295_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct rtd1295_pwm_chip, chip);
}

#define to_pwm_device(d) return container_of(d, struct rtd1295_pwm_chip, chip);

static int pwm_map_idx_lookup(int duty_rate) {
	int idx=0;
	int duty=duty_rate;
	int found = 0;
	int mapSize=ARRAY_SIZE(rtk129x_maps);

	if( duty >= 100) {
		duty = 100;
		idx=0;
		pr_info("--- debug : pwm_map_idx_lookup, duty_rate as (%d) is set to 100, idx=%d \n", duty, idx);
		return idx;
	}
	else if (duty <= 0 ) {
		duty = 0;
		idx=mapSize-1;
		pr_info("--- debug : pwm_map_idx_lookup, duty_rate as (%d) is set to 0, idx=%d \n", duty, idx);
		return idx;
	}

	idx=0;
	while(!found) {
		if( idx==(mapSize-1) )
			break;
		if( duty <= rtk129x_maps[idx].duty_rate && duty > rtk129x_maps[idx+1].duty_rate ) {
			found = 1;
		}
		else {
			idx++;
		}
	}
	pr_info("--- debug : pwm_map_idx_lookup, duty_rate as (%d) , found idx=%d \n", duty, idx);
	return idx;
}

int set_real_freq_by_target_freq(struct rtd1295_pwm_chip *pc, int target_freq) {
	int base_freq = 27000000;
	int real_freq;
	int ocd, csd, div, opt_div;
	div = base_freq / target_freq;

	{
		// give a div to get max ocd and min csd
		int min_ocd = 0;
		int max_ocd = 255;
		int min_csd = 0;
		int max_csd = 15;
		int i;
		// find max bit
		for (i = 0; i < 32; i++) {
			if ((div << i) & BIT(31)) {
				break;
			}
		}
		csd = (32 - (i + 8)) - 1;
		ocd = (div >> (csd + 1)) - 1;
		if (csd > max_csd) csd = max_csd;
		else if (csd < min_csd) csd = min_csd;

		if (ocd > max_ocd) ocd = max_ocd;
		else if (ocd < min_ocd) ocd = min_ocd;

		opt_div = BIT(csd + 1)*(ocd + 1);
		pr_info("--- debug : target_div=%d, real_div=%d, osd=%d, ocd=%d\n", div, opt_div, csd, ocd);
	}

	real_freq = base_freq / opt_div;
	pc->clkout_div = ocd;
	pc->clksrc_div = csd;

	pc->out_freq = real_freq;

	pr_info("--- %s debug : target_freq=%d, real_freq=%d, osd=%d, ocd=%d\n",
			__func__, target_freq, pc->out_freq, pc->clksrc_div, pc->clkout_div);

	return real_freq;
}

int set_real_period_by_target_period(struct rtd1295_pwm_chip *pc, int target_period_ns) {
	int base_ns = 1000000000;
	int real_period_ns;
	int target_freq, real_freq;

	target_freq = base_ns / target_period_ns;
	real_freq = set_real_freq_by_target_freq(pc, target_freq);
	real_period_ns = base_ns / real_freq;

	pr_info("--- %s debug : target_period_ns=%d (freq=%d), real_period_ns=%d (real_freq=%d), osd=%d, ocd=%d\n",
			__func__, target_period_ns, target_freq, real_period_ns, real_freq,
			pc->clksrc_div, pc->clkout_div);

	return real_period_ns;
}

int set_real_freq_by_target_div(struct rtd1295_pwm_chip *pc, int clksrc_div, int clkout_div) {
	int base_freq = 27000000;
	int real_freq, div;

	div = BIT(clksrc_div + 1) * (clkout_div + 1);
	real_freq = base_freq / div;

	pc->clkout_div = clkout_div;
	pc->clksrc_div = clksrc_div;

	pc->out_freq = real_freq;

	pr_info("--- %s debug : real_freq=%d, osd=%d, ocd=%d\n",
			__func__, pc->out_freq, pc->clksrc_div, pc->clkout_div);

	return real_freq;
}

int set_clk_duty(struct rtd1295_pwm_chip *pc, int duty_rate) {
	int clkout_div = pc->clkout_div;
	if (duty_rate < 0) duty_rate = 0;
	if (duty_rate > 100) duty_rate = 100;

	pc->duty_rate = duty_rate;
	pc->clk_duty = (duty_rate * (clkout_div + 1) / 100) - 1;

	if (pc->clk_duty < 0) pc->clk_duty = 0;
	if (pc->clk_duty > clkout_div) pc->clk_duty = clkout_div;

	return 0;
}

static void pwm_set_register(struct rtd1295_pwm_chip *pc)
{
	u32 value;
	int clkout_div = 0;
	int clk_duty = 0;
	int clksrc_div = 0;
	if(pc==NULL) {
		pr_err("--- debug : pwm_set_register -- parameter error!-- \n");
		return;
	}

	if (pc->enable) {
		clkout_div = pc->clkout_div;
		clk_duty = pc->clk_duty;
		clksrc_div = pc->clksrc_div;
	}

	pr_info("--- debug : pwm_set_register -- set OCD=%d,CD=%d,CSD=%d\n",
			clkout_div, clk_duty, clksrc_div);
	value = readl(pc->mmio_pwm_reg_base + RTK_ADDR_PWM_OCD);
	value &= ~(RTK_PWM_OCD_MASK << (pc->pwm_idx*RTK_PWM_OCD_SHIFT));
	value |= clkout_div << (pc->pwm_idx*RTK_PWM_OCD_SHIFT);
	writel(value, pc->mmio_pwm_reg_base + RTK_ADDR_PWM_OCD);

	value = readl(pc->mmio_pwm_reg_base + RTK_ADDR_PWM_CD);
	value &= ~(RTK_PWM_CD_MASK << (pc->pwm_idx*RTK_PWM_CD_SHIFT));
	value |= clk_duty << (pc->pwm_idx*RTK_PWM_CD_SHIFT);
	writel(value, pc->mmio_pwm_reg_base + RTK_ADDR_PWM_CD);

	value = readl(pc->mmio_pwm_reg_base + RTK_ADDR_PWM_CSD);
	value &= ~(RTK_PWM_CSD_MASK << (pc->pwm_idx*RTK_PWM_CSD_SHIFT));
	value |= clksrc_div << (pc->pwm_idx*RTK_PWM_CSD_SHIFT);
	writel(value, pc->mmio_pwm_reg_base + RTK_ADDR_PWM_CSD);

	pr_info("--- debug :  ---- pwm_set_register --- done! \n");
}

static int rtd1295_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			    int duty_ns, int period_ns)
{
	struct rtd1295_pwm_chip *pc = to_rtd1295_pwm_chip(chip);
	int real_period_ns, duty_rate;

	pr_info("--- debug : rtd1295_pwm_config : enable=%d duty_ns=%d period_ns=%d\n",
			pc->enable, duty_ns, period_ns);
	real_period_ns = set_real_period_by_target_period(pc, period_ns);
	duty_rate = duty_ns *100 / period_ns;

	set_clk_duty(pc, duty_rate);

	pwm_set_register(pc);

	return 0;
}

static int rtd1295_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct rtd1295_pwm_chip *pc = to_rtd1295_pwm_chip(chip);
	pr_info("--- debug : rtd1295_pwm_enable ---- duty_rate=%d \n", pc->enable);
	pc->enable = 1;
	pwm_set_register(pc);

	return 0;
}

static void rtd1295_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct rtd1295_pwm_chip *pc = to_rtd1295_pwm_chip(chip);
	pr_info("--- debug :  rtd1295_pwm_disable ---- disable PWM now !\n");
	pc->enable = 0;
	pwm_set_register(pc);
}

static const struct pwm_ops rtd1295_pwm_ops = {
	.config = rtd1295_pwm_config,
	.enable = rtd1295_pwm_enable,
	.disable = rtd1295_pwm_disable,
	.owner = THIS_MODULE,
};

/** define show/store API for each file here **/
ssize_t pwm_show_dutyRate(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtd1295_pwm_chip *pc = platform_get_drvdata(pdev);

	pr_info("--- debug : pwm_show_dutyRate called : dutyRate=> pc->dutyRate=[%d]--- \n", pc->duty_rate);

	return sprintf(buf, "%d\%\n", pc->duty_rate);
}

ssize_t pwm_store_dutyRate(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value=0;
	int ret=-1;
	struct platform_device *pdev = to_platform_device(dev);
	struct rtd1295_pwm_chip *pc = platform_get_drvdata(pdev);

	pr_info("--- debug : pwm_store_dutyRate called : count:%d ",count);

	if( count < 1 ) {
		pr_err("--- debug : pwm_store_dutyRate , count is too small, return \n");
		return count;
	}
	ret = kstrtoint(buf, 10, &value);
	if ( ret != 0 ) {
		pr_err("--- debug : pwm_store_dutyRate , parse buf error! ret=%d \n", ret);
		return count;
	}
	pr_info(" value:[%d] \n",value);
	if( value < 0 || value > 100 ) {
		pr_err("--- debug : pwm_store_dutyRate  ====  input should betweek 0 ~ 100 \n");
		return count;
	}
	if( pc->duty_rate == value ) {
		pr_err(" debug : pwm_store_dutyRate -- dutyRate value is not change, return! \n");
		return count;
	}
	else {
		pr_err(" debug : pwm_store_dutyRate -- assign [%d] to duty_rate now! \n", value);

		set_clk_duty(pc, value);
		pwm_set_register(pc);
	}

	return count;
}

ssize_t pwm_show_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtd1295_pwm_chip *pc = platform_get_drvdata(pdev);

	pr_info("--- debug : pwm_show_enable called : enable=%d---- \n", pc->enable);

	return sprintf(buf, "%d\n", pc->enable);
}

ssize_t pwm_store_enable(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtd1295_pwm_chip *pc = platform_get_drvdata(pdev);
	int value=0;

	if(buf ==NULL) {
		pr_err("--- debug : rtd1295_pwm_enable	====  buffer is null, return \n");
		return count;
	}
	sscanf(buf, "%d", &value);

	if( pc->enable == value ) {
		pr_err("--- debug : rtd1295_pwm_enable	====  the same, do nothing (value=%d) \n", value);
		return count;
	}

	pc->enable = value;
	pwm_set_register(pc);

	pr_info("--- debug : rtd1295_pwm_enable  ====  done \n");
	return count;
}

ssize_t pwm_show_clksrcDiv(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtd1295_pwm_chip *pc = platform_get_drvdata(pdev);

	pr_info("--- debug : pwm_show_clksrcDiv called :clksrc_div=%d---- \n", pc->clksrc_div);

	return sprintf(buf, "%d\n", pc->clksrc_div);
}

ssize_t pwm_store_clksrcDiv(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtd1295_pwm_chip *pc = platform_get_drvdata(pdev);

	int value=0;
	if(buf ==NULL) {
		pr_err("--- debug : pwm_store_clksrcDiv	====  buffer is null, return \n");
		return count;
	}
	sscanf(buf, "%d", &value);

	if(value < 0 || value > 15) {
		pr_err("--- debug : pwm_store_clksrcDiv  ====  input should betweek 0 ~ 15 \n");
		return count;
	}

	if(pc->clksrc_div == value) {
		pr_err("--- debug : pwm_store_clksrcDiv  ====  input is the same=%d, do nothing! \n", value);
		return count;
	}

	set_real_freq_by_target_div(pc, value, pc->clkout_div);

	pwm_set_register(pc);

	pr_info("--- debug : pwm_store_clksrcDiv  ====  done (value=%d) \n", value);
	return count;
}

ssize_t pwm_show_clkoutDiv(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtd1295_pwm_chip *pc = platform_get_drvdata(pdev);

	pr_info("--- debug : pwm_show_clkoutDiv called :clkout_div=%d---- \n", pc->clkout_div);

	return sprintf(buf, "%d\n", pc->clkout_div);
}

ssize_t pwm_store_clkoutDiv(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtd1295_pwm_chip *pc = platform_get_drvdata(pdev);

	int value=0;
	if(buf ==NULL) {
		pr_err("--- debug : pwm_store_clkoutDiv	====  buffer is null, return \n");
		return count;
	}
	sscanf(buf, "%d", &value);

	if(value < 0 || value > 255) {
		pr_err("--- debug : pwm_store_clkoutDiv  ====  input should betweek 0 ~ 255 \n");
		return count;
	}

	if(pc->clkout_div == value) {
		pr_err("--- debug : pwm_store_clkoutDiv  ====  input is the same=%d, do nothing! \n", value);
		return count;
	}

	set_real_freq_by_target_div(pc, pc->clksrc_div, value);

	pwm_set_register(pc);

	pr_info("--- debug : pwm_store_clkoutDiv  ====  done (value=%d) \n", value);
	return count;
}

ssize_t pwm_show_out_freq(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtd1295_pwm_chip *pc = platform_get_drvdata(pdev);

	pr_info("--- debug : pwm_show_out_freq called :out_freq=%dHz---- \n", pc->out_freq);

	return sprintf(buf, "%dHz\n", pc->out_freq);
}

ssize_t pwm_store_out_freq(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct rtd1295_pwm_chip *pc = platform_get_drvdata(pdev);

	int value=0;
	if(buf ==NULL) {
		pr_err("--- debug : pwm_store_out_freq	====  buffer is null, return \n");
		return count;
	}
	sscanf(buf, "%d", &value);

	if(value < 0 || value > 27000000) {
		pr_err("--- debug : pwm_store_out_freq  ====  input should betweek 0 ~ 27MHz \n");
		return count;
	}

	if(pc->out_freq == value) {
		pr_err("--- debug : pwm_store_out_freq  ====  input is the same=%d, do nothing! \n", value);
		return count;
	}

	set_real_freq_by_target_freq(pc, value);

	pwm_set_register(pc);

	pr_info("--- debug : pwm_store_out_freq  ====  done (value=%d) \n", value);
	return count;
}

static int rtd1295_pwm_probe(struct platform_device *pdev)
{
	struct rtd1295_pwm_chip *pwm;
	struct device_node *node = pdev->dev.of_node;
	int ret=0;
	u32 val=0;
	pr_info("--- debug : rtd1295_pwm_probe ---- \n");

	pwm = devm_kzalloc(&pdev->dev, sizeof(*pwm), GFP_KERNEL);
	if (!pwm) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}
	pwm->proc_id = pdev->id;
	pwm->dev = &pdev->dev;

	platform_set_drvdata(pdev, pwm);

	pwm->mmio_pwm_reg_base = of_iomap(node, 0);

	pwm->base_freq = 27000000;
	pwm->out_freq = 0;
	pwm->enable = 0;
	pwm->pwm_idx = 0;
	pwm->clkout_div = RTK_PWM_OCD_MASK;
	pwm->clksrc_div = RTK_PWM_CD_DEFAULT;
	pwm->clk_duty = RTK_PWM_CSD_DEFAULT;
	pwm->duty_rate = 0;

	if (!of_property_read_u32(pdev->dev.of_node, "clkout_div", &val))
		pwm->clkout_div = val;

	if (!of_property_read_u32(pdev->dev.of_node, "clksrc_div", &val))
		pwm->clksrc_div = val;

	set_real_freq_by_target_div(pwm, pwm->clksrc_div, pwm->clkout_div);

	if (!of_property_read_u32(pdev->dev.of_node, "enable", &val))
		pwm->enable = val;

	if(!of_property_read_u32(pdev->dev.of_node, "pwm_idx", &val))
		pwm->pwm_idx = val;

	if(!of_property_read_u32(pdev->dev.of_node, "duty_rate", &val))
		set_clk_duty(pwm, val);

	pr_info("--- debug : rtd1295_pwm_probe - enable=(%d) duty_rate=(%d) clksrc_div=(%d) clkout_div=(%d)---  \n",
			pwm->enable, pwm->duty_rate, pwm->clksrc_div, pwm->clkout_div);
	pr_info("--- debug : rtd1295_pwm_probe - defualt output frequence = %dHz ---  \n",
			pwm->out_freq);

	pwm->chip.dev = &pdev->dev;
	pwm->chip.ops = &rtd1295_pwm_ops;
	pwm->chip.base = 0; // -1;
	pwm->chip.npwm = NUM_PWM;
	pwm->chip.can_sleep = true;

	ret = sysfs_create_group(&pdev->dev.kobj, &pwm_dev_attr_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "sysfs_create_group() failed: %d\n", ret);
		return ret;
	}

	ret = pwmchip_add(&pwm->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
		sysfs_remove_group(&pdev->dev.kobj, &pwm_dev_attr_group);
		return ret;
	}

	pwm_set_register(pwm);

	pr_info("--- debug : rtd1295_pwm_probe --  done! !\n");

	return 0;
}

static int rtd1295_pwm_remove(struct platform_device *pdev)
{
	struct rtd1295_pwm_chip *pc = platform_get_drvdata(pdev);

	pc->enable = 0;
	pwm_set_register(pc);
	sysfs_remove_group(&pdev->dev.kobj, &pwm_dev_attr_group);

	pr_info("--- debug : rtd1295_pwm_remove ---- \n");

	return pwmchip_remove(&pc->chip);
}

static const struct of_device_id rtd1295_pwm_of_match[] = {
	{ .compatible = "realtek,rtd1295-pwm" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtd1295_pwm_of_match);

static struct platform_driver rtd1295_pwm_platform_driver = {
	.driver = {
		.name = "pwm",
		.of_match_table = rtd1295_pwm_of_match,
	},
	.probe = rtd1295_pwm_probe,
	.remove = rtd1295_pwm_remove,
};
module_platform_driver(rtd1295_pwm_platform_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("REALTEK Corporation");
MODULE_ALIAS("platform:rtd1295-pwm");
