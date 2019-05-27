#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/printk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/system_misc.h>

static void __iomem * wdt_base;

#define WDT_CLR			4
#define WDT_CTL			0
#define WDT_OVERFLOW	0xC
#define WDT_NMI         8

void rtk_machine_restart(char mode, const char *cmd)
{
	writel(BIT(0), wdt_base + WDT_CLR);
	writel(0x00800000, wdt_base + WDT_OVERFLOW);
	writel(0x000000FF, wdt_base + WDT_CTL);

	while (1)
		mdelay(100);
}

static struct of_device_id rtk_restart_ids[] = {
	{.compatible = "Realtek,rtk-watchdog", .data = rtk_machine_restart},
	{}
};

static int rtk_setup_restart(void)
{
	const struct of_device_id *of_id;
	struct device_node *np;

	np = of_find_matching_node(NULL, rtk_restart_ids);
	if (WARN(!np, "Unable to setup watchdog restart"))
		return 1;

	wdt_base = of_iomap(np, 0);
	WARN(!wdt_base, "failed to map IO base for watchdog");

	of_id = of_match_node(rtk_restart_ids, np);
	WARN(!of_id, "restart not available");

	arm_pm_restart = of_id->data;

	return 0;
}

arch_initcall(rtk_setup_restart);
