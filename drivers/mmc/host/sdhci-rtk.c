#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/host.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/delay.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include "sdhci-pltfm.h"

//#define RTK_DEBUG
#ifdef RTK_DEBUG
#define RTK_debug(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define RTK_debug(fmt, ...)
#endif

#define DRIVER_NAME    "rtk-sdio"
#define BANNER    "Realtek SDIO Host Driver"
#define SDIO_CLKEN_REGOFF    0x0C
#define SDIO_CLKEN_REGBIT    30

static void __iomem *sdio_membase;
static void __iomem *crt_membase;
u32 sdio_gpio_23 = 0;

struct sdhci_rtk_sdio_data {
    const struct sdhci_pltfm_data *pdata;
};

struct sdhci_rtk {
    const struct sdhci_rtk_sdio_data *soc_data;
    int power_gpio;
};

static void rtk_sdhci_buswidth(struct sdhci_host *host, int bus_width)
{
    u32 ctrl = 0;

    ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
    if((host->mmc->caps & MMC_CAP_8_BIT_DATA) &&
        (bus_width == MMC_BUS_WIDTH_8)){
        ctrl &= ~SDHCI_CTRL_4BITBUS;
        ctrl |= SDHCI_CTRL_8BITBUS;
    }else{
        ctrl &= ~SDHCI_CTRL_8BITBUS;
        if (bus_width == MMC_BUS_WIDTH_4)
            ctrl |= SDHCI_CTRL_4BITBUS;
        else
            ctrl &= ~SDHCI_CTRL_4BITBUS;
    }
    sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
}

static void rtk_sdhci_reset(struct sdhci_host *host, u8 mask)
{
    unsigned long timeout = 0;

    sdhci_writeb(host, mask, SDHCI_SOFTWARE_RESET);
    if(mask & SDHCI_RESET_ALL)
        host->clock = 0;

    /* Wait max 100 ms */
    timeout = 100;

    /* hw clears the bit when it's done */
    while(sdhci_readb(host, SDHCI_SOFTWARE_RESET) & mask){
        if(timeout == 0){
            pr_err("%s: Reset 0x%x never completed.\n",
                mmc_hostname(host->mmc), (int)mask);
            return;
        }
        timeout--;
        mdelay(1);
    }
}

static void rtk_sdhci_uhs_signaling(struct sdhci_host *host, unsigned timing){

    u16 ctrl_2 = 0;

    sdhci_writew(host, sdhci_readw(host, SDHCI_CLOCK_CONTROL) | SDHCI_CLOCK_CARD_EN, SDHCI_CLOCK_CONTROL);

    ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);
    /* Select Bus Speed Mode for host */
    ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
    if ((timing == MMC_TIMING_MMC_HS200) ||
        (timing == MMC_TIMING_UHS_SDR104))
        ctrl_2 |= SDHCI_CTRL_UHS_SDR104;
    else if (timing == MMC_TIMING_UHS_SDR12)
        ctrl_2 |= SDHCI_CTRL_UHS_SDR12;
    else if (timing == MMC_TIMING_UHS_SDR25)
        ctrl_2 |= SDHCI_CTRL_UHS_SDR25;
    else if (timing == MMC_TIMING_UHS_SDR50)
        ctrl_2 |= SDHCI_CTRL_UHS_SDR50;
    else if ((timing == MMC_TIMING_UHS_DDR50) ||
         (timing == MMC_TIMING_MMC_DDR52))
        ctrl_2 |= SDHCI_CTRL_UHS_DDR50;
    else if (timing == MMC_TIMING_MMC_HS400)
        ctrl_2 |= SDHCI_CTRL_HS400; /* Non-standard */

    sdhci_writew(host, ctrl_2 | SDHCI_CTRL_VDD_180, SDHCI_HOST_CONTROL2);
}

static u16 rtk_sdhci_get_preset_value(struct sdhci_host *host)
{
    u16 preset = 0;

    switch(host->timing){
    case MMC_TIMING_UHS_SDR12:
        preset = sdhci_readw(host, SDHCI_PRESET_FOR_SDR12);
        break;
    case MMC_TIMING_UHS_SDR25:
        preset = sdhci_readw(host, SDHCI_PRESET_FOR_SDR25);
        break;
    case MMC_TIMING_UHS_SDR50:
        preset = sdhci_readw(host, SDHCI_PRESET_FOR_SDR50);
        break;
    case MMC_TIMING_UHS_SDR104:
    case MMC_TIMING_MMC_HS200:
        preset = sdhci_readw(host, SDHCI_PRESET_FOR_SDR104);
        break;
    case MMC_TIMING_UHS_DDR50:
        preset = sdhci_readw(host, SDHCI_PRESET_FOR_DDR50);
        break;
    case MMC_TIMING_MMC_HS400:
        preset = sdhci_readw(host, SDHCI_PRESET_FOR_HS400);
        break;
    default:
        pr_warn("%s: Invalid UHS-I mode selected\n",
            mmc_hostname(host->mmc));
        preset = sdhci_readw(host, SDHCI_PRESET_FOR_SDR12);
        break;
    }
    return preset;
}

static void rtk_sdhci_set_clock(struct sdhci_host *host, unsigned int clock)
{
    int div = 0; /* Initialized for compiler warning */
    int real_div = div, clk_mul = 1;
    u16 clk = 0;
    u16 pre_val = 0;
    unsigned long timeout = 0;

    if(clock && clock == host->clock)
        return;

    host->mmc->actual_clock = 0;

    sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

    if(clock == 0)
        goto out;

    if(host->version >= SDHCI_SPEC_300){
        if(sdhci_readw(host, SDHCI_HOST_CONTROL2) & SDHCI_CTRL_PRESET_VAL_ENABLE){
            clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
            pre_val = rtk_sdhci_get_preset_value(host);
            div = (pre_val & SDHCI_PRESET_SDCLK_FREQ_MASK)
                >> SDHCI_PRESET_SDCLK_FREQ_SHIFT;
            if(host->clk_mul && (pre_val & SDHCI_PRESET_CLKGEN_SEL_MASK)){
                clk = SDHCI_PROG_CLOCK_MODE;
                real_div = div + 1;
                clk_mul = host->clk_mul;
            }else{
                real_div = max_t(int, 1, div << 1);
            }
            goto clock_set;
        }

        /*
         * Check if the Host Controller supports Programmable Clock
         * Mode.
         */
        if(host->clk_mul){
            for(div = 1 ; div <= 1024 ; div++){
                if ((host->max_clk * host->clk_mul / div)
                    <= clock)
                    break;
            }
            /*
             * Set Programmable Clock Mode in the Clock
             * Control register.
             */
            clk = SDHCI_PROG_CLOCK_MODE;
            real_div = div;
            clk_mul = host->clk_mul;
            div--;
        }else{
            /* Version 3.00 divisors must be a multiple of 2. */
            if (host->max_clk <= clock)
                div = 1;
            else{
                for(div = 2; div < SDHCI_MAX_DIV_SPEC_300;
                     div += 2) {
                    if((host->max_clk / div) <= clock)
                        break;
                }
            }
            real_div = div;
            div >>= 1;
        }
    }else{
        /* Version 2.00 divisors must be a power of 2. */
        for(div = 1 ; div < SDHCI_MAX_DIV_SPEC_200 ; div *= 2){
            if((host->max_clk / div) <= clock)
                break;
        }
        real_div = div;
        div >>= 1;
    }

clock_set:
    if(real_div)
        host->mmc->actual_clock = (host->max_clk * clk_mul) / real_div;

    clk |= (div & SDHCI_DIV_MASK) << SDHCI_DIVIDER_SHIFT;
    clk |= ((div & SDHCI_DIV_HI_MASK) >> SDHCI_DIV_MASK_LEN) << SDHCI_DIVIDER_HI_SHIFT;
    clk |= SDHCI_CLOCK_INT_EN;
    sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

    /* Wait max 20 ms */
    timeout = 20;
    while(!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL)) & SDHCI_CLOCK_INT_STABLE)){
        if(timeout == 0){
            pr_err("%s: Internal clock never " "stabilised.\n", mmc_hostname(host->mmc));
            return;
        }
        timeout--;
        mdelay(1);
    }

    clk |= SDHCI_CLOCK_CARD_EN;
    sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
printk(KERN_ERR "[SDIO] rtk_sdhci_set_clock end real_div=%x, div=%x, c3c=%x, PLL=%x, CLK=%x\n",real_div, div, readl(host->ioaddr + 0x3c) ,readl(crt_membase + 0x01A8),readl(host->ioaddr + SDHCI_CLOCK_CONTROL));
   
out:
    host->clock = clock;
}

static void rtk_sdhci_platform_init(struct sdhci_host *host)
{
    gpio_direction_output(sdio_gpio_23, 0);
    mdelay(50);
    gpio_direction_output(sdio_gpio_23, 1);
    mdelay(10);

#ifdef CONFIG_MMC_SDHCI_RTK_SDIO30 //SDIO 3.0 support
	if(readl(crt_membase+0x1a204)==0x00) {
		printk(KERN_ERR "SDIO 3.0 A00 version\n");
    		writel(0x00000003, crt_membase + 0x01A0);
    		writel(0x00000006, crt_membase + 0x01AC);
    		writel(0x04517893, crt_membase + 0x01A4);
    		writel(0x00b64388, crt_membase + 0x01A8); //SDIO 3.0, 200MHz
    		mdelay(2);
    		writel(0x00000007, crt_membase + 0x01AC);
    		writel(readl(crt_membase + 0x04) | (1 << 12), crt_membase + 0x04);
    		writel(readl(crt_membase + 0x0C) | (1 << 30), crt_membase + 0x0C);
    		writel(readl(crt_membase + 0x0C) | (1 << 26), crt_membase + 0x0C);
    		writel(0x00000000, crt_membase + 0x010A40);
    		writel(0x00000013, crt_membase + 0x010A34);
	} else {
		printk(KERN_ERR "SDIO 3.0 A01 version\n");
		writel(readl(crt_membase + 0x04) | (1 << 12), crt_membase + 0x04);
                writel(readl(crt_membase + 0x0C) | (1 << 30), crt_membase + 0x0C);
                writel(readl(crt_membase + 0x0C) | (1 << 26), crt_membase + 0x0C);

		writel(0x00000003, crt_membase + 0x01A0);

		writel(0x40000000, crt_membase+ 0x10a58);
                writel(0x00000006, crt_membase + 0x01AC);
                writel(0x04517893, crt_membase + 0x01A4);
                writel(0x00b64388, crt_membase + 0x01A8); //SDIO 3.0, 200MHz
                mdelay(2);
                writel(0x00000007, crt_membase + 0x01AC);
		writel(0x00000000, crt_membase+ 0x10a58);

		writel(0x00000000, crt_membase + 0x010A40);
                writel(0x00000013, crt_membase + 0x010A34);
	}
#else // SDIO 2.0 support
	if(readl(crt_membase+0x1a204)==0x00) {
		printk(KERN_ERR "SDIO 2.0 A00 version\n");
    		writel(0x00000003, crt_membase + 0x01A0);
    		writel(0x00000006, crt_membase + 0x01AC);
    		writel(0x04517893, crt_membase + 0x01A4);
    		writel(0x00564388, crt_membase + 0x01A8); //SDIO 2.0, 100MHz
    		mdelay(2);
    		writel(0x00000007, crt_membase + 0x01AC);
    		writel(readl(crt_membase + 0x04) | (1 << 12), crt_membase + 0x04);
    		writel(readl(crt_membase + 0x0C) | (1 << 30), crt_membase + 0x0C);
    		writel(readl(crt_membase + 0x0C) | (1 << 26), crt_membase + 0x0C);
    		writel(0x00000013, crt_membase + 0x010A34);
	}else {
		printk(KERN_ERR "SDIO 2.0 A01 version\n");
		writel(readl(crt_membase + 0x04) | (1 << 12), crt_membase + 0x04);
                writel(readl(crt_membase + 0x0C) | (1 << 30), crt_membase + 0x0C);
                writel(readl(crt_membase + 0x0C) | (1 << 26), crt_membase + 0x0C);

		writel(0x00000003, crt_membase + 0x01A0);
		
		writel(0x40000000, crt_membase+ 0x10a58);
                writel(0x00000006, crt_membase + 0x01AC);
                writel(0x04517893, crt_membase + 0x01A4);
                writel(0x00564388, crt_membase + 0x01A8); //SDIO 2.0, 100MHz
                mdelay(2);
                writel(0x00000007, crt_membase + 0x01AC);
		writel(0x00000000, crt_membase+ 0x10a58);

                //writel(readl(crt_membase + 0x04) | (1 << 12), crt_membase + 0x04);
                //writel(readl(crt_membase + 0x0C) | (1 << 30), crt_membase + 0x0C);
                //writel(readl(crt_membase + 0x0C) | (1 << 26), crt_membase + 0x0C);
                writel(0x00000013, crt_membase + 0x010A34);
	}
#endif
    writel(0x00000003, crt_membase + 0x010A10);
}

static int rtk_sdhci_execute_tuning(struct sdhci_host *host, u32 opcode)
{

    printk(KERN_INFO "%s : Execute Clock Phase Tuning\n", __func__);

    /* [ToDo] SDIO30 clock phase tuning, jamestai20151229 */

    return 0;
}

static const struct sdhci_ops rtk_sdhci_ops = {
    .reset = rtk_sdhci_reset,
    .set_bus_width = rtk_sdhci_buswidth,
    .set_uhs_signaling = rtk_sdhci_uhs_signaling,
    .set_clock = rtk_sdhci_set_clock,
    .platform_init = rtk_sdhci_platform_init,
    .platform_execute_tuning = rtk_sdhci_execute_tuning,
};

static const struct sdhci_pltfm_data sdhci_rtk_sdio_pdata = {
    .quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
              SDHCI_QUIRK_SINGLE_POWER_WRITE |
              SDHCI_QUIRK_NO_HISPD_BIT |
              SDHCI_QUIRK_BROKEN_CARD_DETECTION|
              SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC,
    .quirks2 = SDHCI_QUIRK2_BROKEN_DDR50|
               SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
    .ops  = &rtk_sdhci_ops,
};

static struct sdhci_rtk_sdio_data sdhci_rtk_sdio_data = {
    .pdata = &sdhci_rtk_sdio_pdata,
};

static const struct of_device_id sdhci_rtk_dt_match[] = {
    { .compatible = "Realtek,rtk1295-sdio", .data = &sdhci_rtk_sdio_data },
    {}
};
MODULE_DEVICE_TABLE(of, sdhci_rtk_dt_match);

static int sdhci_rtk_probe(struct platform_device *pdev)
{
    const struct of_device_id *match;
    const struct sdhci_rtk_sdio_data *soc_data;
    struct sdhci_host *host;
    struct sdhci_pltfm_host *pltfm_host;
    struct sdhci_rtk *rtk_host;
    int rc = 0;

    struct device_node *node = pdev->dev.of_node;

    printk(KERN_INFO "%s: build at : %s \n", DRIVER_NAME, utsname()->version);

#ifdef CONFIG_MMC_SDHCI_RTK_SDIO30 //SDIO 3.0 support
    printk(KERN_INFO "%s: Support SDIO 3.0 mode\n", DRIVER_NAME);
#else
    printk(KERN_INFO "%s: Support SDIO 2.0 mode\n", DRIVER_NAME);
#endif

    sdio_membase = of_iomap(node, 0);
    if(!sdio_membase)
        return -ENOMEM;

    crt_membase = of_iomap(node, 1);
    if(!crt_membase)
        return -ENOMEM;

    sdio_gpio_23 = of_get_gpio_flags(pdev->dev.of_node, 0, NULL);
    if(gpio_is_valid(sdio_gpio_23)){
        rc = gpio_request(sdio_gpio_23, "sdio_gpio(23)");
        if (rc < 0)
            printk(KERN_ERR "%s: can't request gpio %d\n", __func__, sdio_gpio_23);
    }else
        printk(KERN_ERR "%s: gpio %d is not valid\n", __func__, sdio_gpio_23);

    match = of_match_device(sdhci_rtk_dt_match, &pdev->dev);
    if(!match)
        return -EINVAL;
    soc_data = match->data;

    host = sdhci_pltfm_init(pdev, soc_data->pdata, 0);
    if(IS_ERR(host))
        return PTR_ERR(host);
    pltfm_host = sdhci_priv(host);

    rtk_host = devm_kzalloc(&pdev->dev, sizeof(*rtk_host), GFP_KERNEL);
    if(!rtk_host) {
        dev_err(mmc_dev(host->mmc), "failed to allocate rtk_host\n");
        rc = -ENOMEM;
        goto err_alloc_rtk_host;
    }

    rtk_host->soc_data = soc_data;
    pltfm_host->priv = rtk_host;

    host->mmc->caps = MMC_CAP_4_BIT_DATA |
                      MMC_CAP_SD_HIGHSPEED |
                      MMC_CAP_MMC_HIGHSPEED |
                      MMC_CAP_UHS_SDR12 |
                      MMC_CAP_UHS_SDR25 |
                      MMC_CAP_UHS_SDR50 |
                      //MMC_CAP_UHS_DDR50 |
                      MMC_CAP_UHS_SDR104 |
                      MMC_CAP_NONREMOVABLE ;

    host->mmc->pm_caps = MMC_PM_KEEP_POWER | MMC_PM_WAKE_SDIO_IRQ;

    rc = sdhci_add_host(host);

    if(rc)
        goto err_add_host;

    return 0;

err_add_host:
err_alloc_rtk_host:
    sdhci_pltfm_free(pdev);
    return rc;
}

static int sdhci_rtk_remove(struct platform_device *pdev)
{
    struct sdhci_host *host = platform_get_drvdata(pdev);
    struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
    struct sdhci_rtk *rtk_host = pltfm_host->priv;
    int dead = (readl(host->ioaddr + SDHCI_INT_STATUS) == 0xFFFFFFFF);

    sdhci_remove_host(host, dead);

    if (gpio_is_valid(rtk_host->power_gpio)) {
        gpio_free(rtk_host->power_gpio);
    }
    sdhci_pltfm_free(pdev);

    return 0;
}

static int sdhci_rtk_suspend(struct device *dev){

    struct sdhci_host *host = dev_get_drvdata(dev);
    printk(KERN_ERR "[SDIO] sdhci_rtk_suspend start\n");
    sdhci_suspend_host(host);
    printk(KERN_ERR "[SDIO] sdhci_rtk_suspend OK\n");
    return 0;
}

static int sdhci_rtk_resume(struct device *dev){

    struct sdhci_host *host = dev_get_drvdata(dev);
    //void __iomem *crt_membase;
    void __iomem *addr;
    unsigned int val;

    //struct device_node *node = dev->of_node;

    host->clock = 0;

    /*sdio_membase = of_iomap(node, 0);
    if(!sdio_membase)
        return -ENOMEM;

    crt_membase = of_iomap(node, 1);
    if (!crt_membase)
        return -ENOMEM;*/
    printk(KERN_ERR "[SDIO] sdhci_rtk_resume start\n");

    addr = crt_membase + SDIO_CLKEN_REGOFF;

#ifdef CONFIG_MMC_SDHCI_RTK_SDIO30 //SDIO 3.0 support
	if(readl(crt_membase+0x1a204)==0x00) {
    		writel(0x00000003, crt_membase + 0x01A0);

    		writel(0x00000006, crt_membase + 0x01AC);
   		writel(0x04517893, crt_membase + 0x01A4);
    		writel(0x00b74388, crt_membase + 0x01A8); //SDIO 3.0, 200MHz
    		mdelay(2);
    		writel(0x00000007, crt_membase + 0x01AC);

    		writel(0x00000006, crt_membase + 0x01AC);
    		writel(0x04517893, crt_membase + 0x01A4);
    		writel(0x00b64388, crt_membase + 0x01A8); //SDIO 3.0, 200MHz
    		mdelay(2);
    		writel(0x00000007, crt_membase + 0x01AC);
    		writel(readl(crt_membase + 0x04) | (1 << 12), crt_membase + 0x04);
    		writel(readl(crt_membase + 0x0C) | (1 << 30), crt_membase + 0x0C);
    		writel(readl(crt_membase + 0x0C) | (1 << 26), crt_membase + 0x0C);
    		writel(0x00000000, crt_membase + 0x010A40);
    		writel(0x00000013, crt_membase + 0x010A34);
	}
	else {
		writel(readl(crt_membase + 0x04) | (1 << 12), crt_membase + 0x04);
                writel(readl(crt_membase + 0x0C) | (1 << 30), crt_membase + 0x0C);
                writel(readl(crt_membase + 0x0C) | (1 << 26), crt_membase + 0x0C);
		
		writel(0x40000000, crt_membase+ 0x10a58);
		writel(0x00000006, crt_membase + 0x01AC);
                writel(0x04517893, crt_membase + 0x01A4);
                writel(0x00b64388, crt_membase + 0x01A8); //SDIO 3.0, 200MHz
                mdelay(2);
                writel(0x00000007, crt_membase + 0x01AC);
		writel(0x00000000, crt_membase+ 0x10a58);

		writel(0x00000000, crt_membase + 0x010A40);
                writel(0x00000013, crt_membase + 0x010A34);
	}
#else // SDIO 2.0 support
    /*writel(0x00000003, crt_membase + 0x01A0);
    writel(0x00000006, crt_membase + 0x01AC);
    writel(0x04517893, crt_membase + 0x01A4);
    writel(0x00574388, crt_membase + 0x01A8); //SDIO 2.0, 100MHz
    mdelay(2);
    writel(0x00000007, crt_membase + 0x01AC);
    */
	if(readl(crt_membase+0x1a204)==0x00) {
    		writel(0x00000001, crt_membase + 0x0018);
    
    		writel(0x00000006, crt_membase + 0x01AC);
    		writel(0x04517893, crt_membase + 0x01A4);
    		writel(0x00564388, crt_membase + 0x01A8); //SDIO 2.0, 100MHz
    		mdelay(2);
    		writel(0x00000007, crt_membase + 0x01AC);
    
    		writel(0x00000000, crt_membase + 0x0018);

    		writel(readl(crt_membase + 0x04) | (1 << 12), crt_membase + 0x04);
    		writel(readl(crt_membase + 0x0C) | (1 << 30), crt_membase + 0x0C);
    		writel(readl(crt_membase + 0x0C) | (1 << 26), crt_membase + 0x0C);
    		writel(0x00000013, crt_membase + 0x010A34);
	}else {
		writel(readl(crt_membase + 0x04) | (1 << 12), crt_membase + 0x04);
                writel(readl(crt_membase + 0x0C) | (1 << 30), crt_membase + 0x0C);
                writel(readl(crt_membase + 0x0C) | (1 << 26), crt_membase + 0x0C);
//		writel(0x00000001, crt_membase + 0x0018);
		writel(0x40000000, crt_membase+ 0x10a58);
                writel(0x00000006, crt_membase + 0x01AC);
                writel(0x04517893, crt_membase + 0x01A4);
                writel(0x00564388, crt_membase + 0x01A8); //SDIO 2.0, 100MHz
                mdelay(2);
                writel(0x00000007, crt_membase + 0x01AC);
		writel(0x00000000, crt_membase+ 0x10a58);
//              writel(0x00000000, crt_membase + 0x0018);
		writel(0x00000013, crt_membase + 0x010A34);
	}
#endif
    writel(0x00000003, crt_membase + 0x010A10);
    //printk(KERN_INFO "crt_membase + SDIO_CLKEN_REGOFF = %llx\n", (u64)(crt_membase + SDIO_CLKEN_REGOFF));
    val = readl(addr);
    val  |=  1 << (SDIO_CLKEN_REGBIT);
    writel(val, addr);

    val = readl(host->ioaddr + 0x28);
//    printk(KERN_ERR "read(host->ioaddr + 0x28) = %x\n", val);
    val  |=  0x0f << 8;
    writel(val, host->ioaddr + 0x28);
    val = readl(host->ioaddr + 0x28);
//    printk(KERN_ERR "read(host->ioaddr + 0x28) = %x\n", val);
    
    sdhci_resume_host(host);
    printk(KERN_ERR "[SDIO] sdhci_rtk_suspend OK\n");  

    return 0;
}

const struct dev_pm_ops sdhci_rtk_pmops = {
    .suspend = sdhci_rtk_suspend,
    .resume = sdhci_rtk_resume,
};

static struct platform_driver sdhci_rtk_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = sdhci_rtk_dt_match,
        //.pm    = SDHCI_PLTFM_PMOPS,
        .pm = &sdhci_rtk_pmops,
    },
    .probe = sdhci_rtk_probe,
    .remove = sdhci_rtk_remove,
};

module_platform_driver(sdhci_rtk_driver);

MODULE_DESCRIPTION("SDHCI host driver for Realtek");
