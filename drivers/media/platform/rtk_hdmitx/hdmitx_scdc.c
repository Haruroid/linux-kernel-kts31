#include <linux/i2c.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>

#include "hdmitx_scdc.h"
#include "hdmitx.h"
#include "hdmitx_dev.h"
#include "hdmitx_rpc.h"
#include "rtk_edid.h"

extern struct edid_info hdmitx_edid_info;

static struct hdmitx_scdc_data tx_scdc; 

void hdmitx_scdc_work_func(struct work_struct *work);

int hdmitx_write_scdc_port(unsigned char offset, unsigned char value)
{
	struct i2c_adapter *p_adap;
	unsigned char bus_id =1;
	unsigned char data[2];

	data[0]= offset;
	data[1]= value;

	struct i2c_msg msgs[] = {
		{
			.addr	= SCDC_I2C_ADDR,
			.flags	= 0,
			.len	= 2,
			.buf	= data,
		}
	};

	if ((p_adap = i2c_get_adapter(bus_id))==NULL){
		HDMI_ERROR("[%s] get i2c adapter %d failed\n", __FUNCTION__,bus_id);
		return SCDC_I2C_FAIL;
	}

	if (i2c_transfer(p_adap, msgs, 1) != 1) {
		HDMI_ERROR("Write SCDC port fail, offset(0x%02x) value(0x%02x)",offset,value);
		return SCDC_I2C_FAIL;
	} else {
		HDMI_DEBUG("Write SCDC port, offset(0x%02x) value(0x%02x)",offset,value);
	}

	return SCDC_I2C_OK;
}

int hdmitx_read_scdc_port(unsigned char offset, unsigned char *buf, unsigned char len)
{
	struct i2c_adapter *p_adap;
	unsigned char bus_id =1;
	unsigned char data;

	data = offset;

	struct i2c_msg msgs[] = {
		{
			.addr	= SCDC_I2C_ADDR,
			.flags	= 0,
			.len	= 1,
			.buf	= &data,
		}, {
			.addr	= SCDC_I2C_ADDR,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		}
	};


	if ((p_adap = i2c_get_adapter(bus_id))==NULL){
		HDMI_ERROR("[%s] get i2c adapter %d failed\n", __FUNCTION__,bus_id);
		return SCDC_I2C_FAIL;
	}

	if (i2c_transfer(p_adap, msgs, 2) != 2) {
		HDMI_ERROR("Read SCDC port fail, offset(0x%02x) len(%u)",offset,len);
		return SCDC_I2C_FAIL;
	} else {
		HDMI_DEBUG("Read SCDC port, offset(0x%02x) len(%u)",offset,len);
	}

	return SCDC_I2C_OK;
}

unsigned char hdmitx_send_scdc_TmdsConfig(unsigned int standard, unsigned int dataInt0)
{
	unsigned char config_data;
	unsigned char deep_color,depp_depth;

	deep_color = (dataInt0>>1)&0x1;
	depp_depth = (dataInt0>>2)&0xF;

	if((depp_depth!=5)&&(depp_depth!=6))
		deep_color = 0;

	switch(standard)
	{
		//Scramble, 1/40 data rate
		case VO_STANDARD_HDTV_2160P_50:
		case VO_STANDARD_HDTV_2160P_60:
		case VO_STANDARD_HDTV_4096_2160P_50:
		case VO_STANDARD_HDTV_4096_2160P_60:
		{
			config_data = 0x3;
			break;
		}

		//Scramble, 1/40 data rate
		case VO_STANDARD_HDTV_2160P_23:
		case VO_STANDARD_HDTV_2160P_24:
		case VO_STANDARD_HDTV_2160P_25:
		case VO_STANDARD_HDTV_2160P_29:
		case VO_STANDARD_HDTV_2160P_30:
		case VO_STANDARD_HDTV_4096_2160P_24:
		case VO_STANDARD_HDTV_4096_2160P_25:
		case VO_STANDARD_HDTV_4096_2160P_30:
		{
			if(deep_color)
				config_data = 0x3;
			else
				config_data = 0x0;

			break;
		}

		//Depends on 340M_SCRAMBLE support
		case VO_STANDARD_HDTV_2160P_50_420:
		case VO_STANDARD_HDTV_2160P_60_420:
		case VO_STANDARD_HDTV_4096_2160P_50_420:
		case VO_STANDARD_HDTV_4096_2160P_60_420:
		{
			if(deep_color)
				config_data = 0x3;
			else if(hdmitx_edid_info.scdc_capable&SCDC_340M_SCRAMBLE)
				config_data = 0x1;// Scramble, 1/10
			else
				config_data = 0x0;

			break;
		}
		//No Scramble, 1/10 data rate
		default:
			config_data = 0x0;
	}

	if(hdmitx_edid_info.scdc_capable&SCDC_PRESENT)
	{
		hdmitx_write_scdc_port(SCDCS_TMDS_Config, config_data);
		HDMI_INFO("Send SCDC TMDS_Config(0x%02x) standard(%u) deep_color(%u)",config_data,standard,deep_color);
	}
	else if(config_data!=0)
	{
		HDMI_ERROR("Sink not support SCDC_PRESENT, but need scramble");
		hdmitx_write_scdc_port(SCDCS_TMDS_Config, config_data);
		HDMI_INFO("Send SCDC TMDS_Config(0x%02x) standard(%u) deep_color(%u)",config_data,standard,deep_color);
	}

	return config_data;
}

void hdmitx_scdc_work_func(struct work_struct *work)
{
	unsigned char update_flags[2],status_flags[2],err_det[7];
	unsigned int i,sum=0;

	if(hdmitx_read_scdc_port(SCDCS_Update_0, update_flags, 2))
		return;

	HDMI_DEBUG("SCDC: Update(0x%02x)", update_flags[0]);

	if(update_flags[0]&0x1)//Status_Update
	{
		hdmitx_read_scdc_port(SCDCS_Status_Flag_0, status_flags, 2);
		HDMI_DEBUG("SCDC: Status(0x%02x)", status_flags[0]);
	}

	if(update_flags[0]&0x2)//CED_Update
	{
		hdmitx_read_scdc_port(SCDCS_Err_Det_0_L, err_det, 7);

		for(i=0;i<6;i++)
			sum += err_det[i];

		sum = (0x100-(sum&0xFF))&0xFF;

		if(sum != err_det[6])
			HDMI_ERROR("Wrong CED checksum");
		else
			HDMI_DEBUG("SCDC: CED checksum ok");
	}

	hdmitx_write_scdc_port(SCDCS_Update_0, update_flags[0]);// Write clear
}


static irqreturn_t hdmitx_scdc_isr(int irq, void *dev_id)
{
	HDMI_DEBUG("Get SCDC read request");
	schedule_work(&tx_scdc.rr_work);
	return IRQ_HANDLED;
}

void enable_hdmitx_scdcrr(unsigned char enable)
{
	if(!tx_scdc.enable_rr)
		return;

	HDMI_INFO("%s SCDC read request",enable?"Enable":"Disable");

	if((!tx_scdc.i2c1_req_reg)||(!tx_scdc.dev_nd))
	{
		HDMI_ERROR("[%s] No control reg or no device node\n", __FUNCTION__);
		return;
	}

	if(enable)
	{
		wr_reg(tx_scdc.i2c1_req_reg, 0x3);
		hdmitx_write_scdc_port(SCDCS_Config_0, 0x1);// Send RR_Enable
	}
	else
		wr_reg(tx_scdc.i2c1_req_reg, 0x0);
}

void register_hdmitx_scdcrr(struct device_node *dev)
{
	HDMI_INFO("[%s]",__FUNCTION__);

	memset(&tx_scdc,0x0,sizeof(tx_scdc));

	tx_scdc.i2c1_req_reg = of_iomap(dev, 1);

	if(tx_scdc.i2c1_req_reg==NULL)
	{
		HDMI_ERROR("[%s] Unable to map I2C1_REQ registers\n", __FUNCTION__);
		return;
	}

	//Default disable SCDC read request function for CTS 8-9
	wr_reg(tx_scdc.i2c1_req_reg, 0x0);

	// Get SCDC read request device node
	tx_scdc.dev_nd = of_get_child_by_name(dev, "scdc_rr");
	if (!tx_scdc.dev_nd)
	{
		HDMI_ERROR("[%s] Get SCDC read request node fail!\n", __FUNCTION__);
		return;
	}

	if(of_property_read_u32(tx_scdc.dev_nd, "enable-scdc-rr", &tx_scdc.enable_rr))
		tx_scdc.enable_rr = 0;

	if(!tx_scdc.enable_rr)
	{
		HDMI_INFO("Skip SCDC RR init");
		return;
	}

	// Init interrupt work function
	INIT_WORK(&tx_scdc.rr_work, hdmitx_scdc_work_func);

	// Register read request interrupt
	tx_scdc.i2c1_irq = irq_of_parse_and_map(tx_scdc.dev_nd, 0);
	if(request_irq(tx_scdc.i2c1_irq, hdmitx_scdc_isr, IRQF_SHARED, "hdmitx_scdc", tx_scdc.dev_nd))
		HDMI_ERROR("[%s] Request irq fail\n", __FUNCTION__);

}

void hdmitx_scdcrr_suspend(void)
{
	if(!tx_scdc.enable_rr)
		return;

	enable_hdmitx_scdcrr(0);
	free_irq(tx_scdc.i2c1_irq, tx_scdc.dev_nd);
	cancel_work_sync(&tx_scdc.rr_work);
}

void hdmitx_scdcrr_resume(void)
{
	if(!tx_scdc.enable_rr)
		return;

	// Register read request interrupt
	if(request_irq(tx_scdc.i2c1_irq, hdmitx_scdc_isr, IRQF_SHARED, "hdmitx_scdc", tx_scdc.dev_nd))
		HDMI_ERROR("[%s] Request irq fail\n", __FUNCTION__);

}

