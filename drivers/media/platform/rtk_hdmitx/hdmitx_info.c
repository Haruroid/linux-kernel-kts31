#include <drm/drm_edid.h>
#include <drm/drm_crtc.h>

#include "hdmitx.h"
#include "hdmitx_api.h"
#include "hdmitx_rpc.h"
#include "rtk_edid.h"

extern struct edid_info hdmitx_edid_info;

// EISA ID is based upon compressed ASCII
const unsigned char eisa_id[] = { ' ','A','B','C','D','E','F','G','H','I','J','K','L','M','N','O',
								  'P','Q','R','S','T','U','V','W','X','Y','Z',' ',' ',' ',' ',' '};

ssize_t show_hdmitx_info(struct device *cd, struct device_attribute *attr, char *buf)
{
	unsigned int ret_count;
	unsigned char color_index;
	struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM hdmitx_tv_system;

	char *mode[] = { "DVI", "HDMI", "OFF", "MHL", "OFF" };
	char *pixel_format[] = { "RGB", "YUV422", "YUV444", "YUV420", "Reserved", "Reserved", "Reserved", "IDO-Defined"};
	char *depth[] = { "8 Bits", "10 Bits", "12 Bits"};
	char *colorimetry[] = { "No Data", "SMPTE 170M", "ITU 709", "Extend" };
	char *extended_colorimetry[] = { "xvYCC601", "xvYCC709", "sYCC601", "AdobeYCC601", "AdobeRGB", "BT2020YcCbcCrx", "BT2020RGB", "Reserved"};

	RPC_ToAgent_QueryConfigTvSystem(&hdmitx_tv_system);

	ret_count = sprintf(buf, "------ HDMI Info -----\n");

	if(hdmitx_tv_system.hdmiInfo.hdmiMode<=4)
		ret_count += sprintf(buf+ret_count, "Mode:%s\n", mode[hdmitx_tv_system.hdmiInfo.hdmiMode]);

	ret_count += sprintf(buf+ret_count, "VIC:%u\n",hdmitx_tv_system.hdmiInfo.dataByte4&0x7F);

	ret_count += sprintf(buf+ret_count, "Pixel format:%s\n", pixel_format[hdmitx_tv_system.hdmiInfo.dataByte1>>5]);

	ret_count += sprintf(buf+ret_count, "Color depth:%s\n", depth[0]);

	color_index = (hdmitx_tv_system.hdmiInfo.dataByte2>>6)&0x3;//Colorimetry C1:C0
	if(color_index!=0x3)
	{
		ret_count += sprintf(buf+ret_count, "Colorimetry:%s\n", colorimetry[color_index]);
	}
	else
	{
		color_index = (hdmitx_tv_system.hdmiInfo.dataByte3>>4)&0x7;//Extended Colorimetry EC2:EC1:EC0
		ret_count += sprintf(buf+ret_count, "Colorimetry:%s\n", extended_colorimetry[color_index]);
	}

	return ret_count;
}

ssize_t show_edid_info(struct device *cd, struct device_attribute *attr, char *buf)
{
	unsigned int ret_count;
	unsigned int vendor_id;
	unsigned char id_index[3];

	ret_count = sprintf(buf, "------ EDID Info -----\n");

	vendor_id = hdmitx_edid_info.mfg_id[0];
	vendor_id = (vendor_id<<8) | hdmitx_edid_info.mfg_id[1];
	ret_count += sprintf(buf+ret_count, "Vendor ID:%04x\n",vendor_id);

	if(hdmitx_edid_info.hdmi_id==HDMI_2P0_IDENTIFIER)
		ret_count += sprintf(buf+ret_count, "HDMI version:2.0\n");
	else
		ret_count += sprintf(buf+ret_count, "HDMI version:1.x\n");

	// EISA ID for manufacturer name, using five-bit codes
	id_index[0] = (vendor_id>>10)&0x1F;
	id_index[1] = (vendor_id>>5)&0x1F;
	id_index[2] = vendor_id&0x1F;
	ret_count += sprintf(buf+ret_count, "TV name:%c%c%c\n",eisa_id[id_index[0]],eisa_id[id_index[1]],eisa_id[id_index[2]]);

	ret_count += sprintf(buf+ret_count, "ProductCode:%02x%02x\n",hdmitx_edid_info.prod_code[1],hdmitx_edid_info.prod_code[0]);

	ret_count += sprintf(buf+ret_count, "SerialNumber:%08x\n",hdmitx_edid_info.serial);

	ret_count += sprintf(buf+ret_count, "ManufactureYear:%u\n",1990+hdmitx_edid_info.mfg_year);

	ret_count += sprintf(buf+ret_count, "ManufactureWeek:%u\n",hdmitx_edid_info.mfg_week);

	if(hdmitx_edid_info.hdmi_id==HDMI_2P0_IDENTIFIER)
	{
		ret_count += sprintf(buf+ret_count, "Max TMDS character rate:%u\n",hdmitx_edid_info.max_tmds_char_rate);

		ret_count += sprintf(buf+ret_count, "Deep Color 420:0x%x\n",hdmitx_edid_info.dc_420);
	}
	return ret_count;
}

