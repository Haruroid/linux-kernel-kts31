#include "hdmirx_video_dev.h"

#include "v4l2_hdmi_dev.h"
#include "hdmirx_reg.h"
#include "mipi_wrapper.h"

#include "rx_drv/hdmiInternal.h"

#if HDMI_REPEATER_SUPPORT
void __iomem *pll_hdmi_i2_vaddr,*pll_hdmi_vaddr, *disp_pll_vaddr;
int hdmi_repeater_mode=0;// 0:regular mode, 1:repeater mode

/*=================== extern Variable/Function ===================*/
extern MIPI_TOP_INFO mipi_top;
extern HDMI_INFO_T hdmi;

extern void Hdmi_SetHPD(char high);
/*======================================================*/

void Set_Hdmitx_source(unsigned char repeater)
{
	unsigned int reg_val;

	if(repeater)
	{
		reg_val = readl(pll_hdmi_vaddr)&0xFCFFFFFF;//Bit25:Bit24
		writel(reg_val|(1<<25), pll_hdmi_vaddr);//Set TX clock input from RX

		writel((1<<26)|(1<<25), disp_pll_vaddr);//Set TX data input from RX

		hdmi_rx_reg_write32(HDMIRPT, 0x1, HDMI_RX_PHY);//Enable HDMI repeater 1.4 mode
	}
	else
	{
		writel(readl(pll_hdmi_vaddr)&0xFCFFFFFF, pll_hdmi_vaddr);//Set TX clock input from TX
		writel((1<<26), disp_pll_vaddr);//Set TX data input from TX
		hdmi_rx_reg_write32(HDMIRPT, 0x0, HDMI_RX_PHY);//Disable HDMI repeater mode
	}
}

void Set_Hdmitx_PLL(void)
{
	unsigned int tmds,reg_val;
	unsigned char m2,m1;

	if(!hdmi_repeater_mode)
		return;

	Set_Hdmitx_source(1);

	tmds = hdmi.b*27/256;
	if(TMDS_CPS_get_pll_div2_en(hdmi_rx_reg_read32(TMDS_CPS,HDMI_RX_MAC)))
		tmds = tmds*2;
	
	if(tmds>150)//297MHz/594MHz
	{
		m2 = 0;
		m1 = 0;
	}
	else if(tmds>145)//148.5MHz
	{
		m2 = 0;
		m1 = 1;
	}
	else if(tmds>70)//74.25MHz
	{
		m2 = 0;
		m1 = 2;
	}
	else//27MHz
	{
		m2 = 0;
		m1 = 3;
	}
	reg_val = readl(pll_hdmi_i2_vaddr)&0xFFFFFFF0;
	writel(reg_val|(m2<<2)|m1, pll_hdmi_i2_vaddr);//Set TX PLL

}

void Set_Hdmi_repeater_mode(int enable)
{

	if(hdmi_repeater_mode == enable)
		return;

	if(enable)
	{
		HDMIRX_INFO("Enable HDMI repeater mode");
		pll_hdmi_i2_vaddr = ioremap(0x98000194, 0x4);
		pll_hdmi_vaddr = ioremap(0x98000238, 0x4);
		disp_pll_vaddr = ioremap(0x98000024, 0x4);

		if(mipi_top.hdmi_rx_init)
		{
			Hdmi_SetHPD(0);
			usleep_range(500000, 600000);
			Hdmi_SetHPD(1);
		}

	}
	else
	{
		HDMIRX_INFO("Disable HDMI repeater mode");
		Set_Hdmitx_source(0);

		iounmap(pll_hdmi_i2_vaddr);
		iounmap(pll_hdmi_vaddr);
		iounmap(disp_pll_vaddr);
	}
	hdmi_repeater_mode = enable;
}
#endif
