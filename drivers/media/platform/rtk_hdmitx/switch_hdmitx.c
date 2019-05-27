#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/switch.h>
#include <linux/device.h>

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>

#include "hdmitx.h"
#include "hdmitx_dev.h"
#include "hdmitx_api.h"
#include "rtk_edid.h"
#include "hdmitx_scdc.h"

#define HDMI_SWITCH_NAME "hdmi"

extern struct edid_info hdmitx_edid_info;

#ifdef CONFIG_RTK_HDMIRX
extern void HdmiRx_save_tx_physical_addr(unsigned char byteMSB, unsigned char byteLSB);
#endif

static struct hdmitx_switch_data s_data;
static struct switch_dev *sdev = NULL;

#if 0
static ssize_t hdmitx_show(struct device *device, struct device_attribute *attr, char *buffer)
{
	
	HDMI_DEBUG("hdmi_show_interface");
	return sprintf(buffer, "%d", s_data.state);
	
}

static ssize_t hdmitx_store(struct device *device, struct device_attribute *attr, const char *buffer, ssize_t count)
{
        int state;
		
        state = simple_strtol(buffer, NULL, 10);
        s_data.state = (state) ? 1 : 0;
        schedule_work(&s_data.work);
		HDMI_DEBUG("hdmi_store_interface");
        return count;
}

static DEVICE_ATTR(hdmitx, 0777, hdmitx_show, hdmitx_store);
#endif
static ssize_t hdmitx_switch_print_state(struct switch_dev *sdev, char *buffer)
{
	HDMI_DEBUG("hdmitx_switch_print_state");
	return sprintf(buffer, "%d", s_data.state);
}

int hdmitx_switch_get_state(void)
{
	return s_data.state;
}

static void hdmitx_switch_work_func(struct work_struct *work) 
{
	int state =0,sink_changed=0;		
	hdmitx_device_t *pdev = container_of(sdev, hdmitx_device_t,sdev);
	asoc_hdmi_t *drvdata = (asoc_hdmi_t*)hdmitx_get_drvdata(pdev);

	state = gpio_get_value(s_data.pin);	
	s_data.state = state;

	HDMI_INFO("%s:start", state?"plugged in":"pulled out");
	
	if(state == 1)
		sink_changed = hdmitx_get_sink_capability(drvdata);
	else
    {       
		hdmitx_turn_off_tmds(HDMI_MODE_HDMI);     
		hdmitx_reset_sink_capability(drvdata);
    } 

	if(sink_changed)	
	{		
		state = gpio_get_value(s_data.pin);
		s_data.state = state;
		
		hdmitx_dump_error_code();		
	}
	
	HDMI_INFO("%s:done", state?"plugged in":"pulled out");

	if(s_data.state!= switch_get_state(sdev))
	{
		switch_set_state(sdev, s_data.state);
		HDMI_INFO("Switch state to %u",s_data.state);

		if(s_data.state==1)
		{
#ifdef CONFIG_RTK_HDMIRX// HDMI 1.4 CTS 9-5, Physical Address Handling
			HdmiRx_save_tx_physical_addr(drvdata->sink_cap.cec_phy_addr[0], drvdata->sink_cap.cec_phy_addr[1]);
#endif
			if(hdmitx_edid_info.scdc_capable&SCDC_RR_CAPABLE)
				enable_hdmitx_scdcrr(1);
		}
		else
		{
			enable_hdmitx_scdcrr(0);
		}
	}


}

static irqreturn_t hdmitx_switch_isr(int irq, void *data)
{    		
	schedule_work(&s_data.work);	
	HDMI_DEBUG("hdmitx_switch_isr");
	
	return IRQ_HANDLED;
}

int register_hdmitx_switchdev(hdmitx_device_t * device)
{
	int ret;
			
	HDMI_DEBUG("register_hdmitx_switch");	

	if (&device->sdev==NULL)
		return -ENOMEM;
		
	sdev = &device->sdev;		
 
	INIT_WORK(&s_data.work, hdmitx_switch_work_func);
	
	sdev->name = HDMI_SWITCH_NAME;
	sdev->print_state = hdmitx_switch_print_state;

	ret = switch_dev_register(sdev);
	if (ret < 0) {
		HDMI_ERROR("err_register_switch");
		goto err_register_switch;
	}

	// Get hotplug pin state
	s_data.pin  = device->hpd_gpio;
	gpio_direction_input(s_data.pin);
	gpio_set_debounce(s_data.pin,30*1000); //30ms
	s_data.state = gpio_get_value(s_data.pin);

	if(s_data.state)
		schedule_work(&s_data.work);

	s_data.irq = device->hpd_irq;
		
	irq_set_irq_type(s_data.irq, IRQ_TYPE_EDGE_BOTH);
	if(request_irq(s_data.irq, hdmitx_switch_isr,IRQF_SHARED,"switch_hdmitx",&device->dev)) {
		HDMI_ERROR("cannot register IRQ %d", s_data.irq);
	}
	
	/*ret = device_create_file(sdev->dev, &dev_attr_hdmitx);

	if (ret < 0) {
		goto err_create_sysfs_file;
	}*/	

	goto end;
		
//err_create_sysfs_file:
//	switch_dev_unregister(sdev);
err_register_switch:	
	HDMI_ERROR("register_hdmitx_switch failed");
end:	
	return ret;
}
EXPORT_SYMBOL(register_hdmitx_switchdev);


void deregister_hdmitx_switchdev(hdmitx_device_t* device)
{
    return switch_dev_unregister(&device-> sdev);
}
EXPORT_SYMBOL(deregister_hdmitx_switchdev);


int show_hpd_status(bool real_time)
{		
	if(real_time)
		return gpio_get_value(s_data.pin);	
	else
		return s_data.state;
}
EXPORT_SYMBOL(show_hpd_status);


int rtk_hdmitx_switch_suspend(void)
{	
	int state=0;	
	hdmitx_device_t *pdev = container_of(sdev, hdmitx_device_t,sdev);

	free_irq(s_data.irq, &pdev->dev);
	HDMI_DEBUG("%s free irq=%x ",__FUNCTION__,s_data.irq);

	cancel_work_sync(&s_data.work);//Cancel work and wait for it to finish

	s_data.state = state;		
	hdmitx_reset_sink_capability((asoc_hdmi_t*)hdmitx_get_drvdata(pdev));	
	switch_set_state(sdev, state);	

	HDMI_INFO("Switch state to %u",s_data.state);
			
	return 0;
}
EXPORT_SYMBOL(rtk_hdmitx_switch_suspend);

int rtk_hdmitx_switch_resume(void)
{	
	hdmitx_device_t *pdev = container_of(sdev, hdmitx_device_t,sdev);

	gpio_set_debounce(s_data.pin,30*1000); //30ms

	irq_set_irq_type(s_data.irq, IRQ_TYPE_EDGE_BOTH);
	
	if(request_irq(s_data.irq, hdmitx_switch_isr,IRQF_SHARED,"switch_hdmitx",&pdev->dev)) {
		HDMI_ERROR("cannot register IRQ %d", s_data.irq);
	}

	schedule_work(&s_data.work);

	return 0;
}
EXPORT_SYMBOL(rtk_hdmitx_switch_resume);

