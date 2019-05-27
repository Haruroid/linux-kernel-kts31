#include "hdmiInternal.h"
#include "mipi_wrapper.h"

#define	HDCP_10_DVI 	0x91
#define	HDCP_11_HDMI 	0x93

/*=================== extern Variable/Function ===================*/
extern MIPI_TOP_INFO mipi_top;
extern HDMIRX_IOCTL_STRUCT_T hdmi_ioctl_struct;

extern void Hdmi_SetHPD(char high);
/*======================================================*/


#ifdef CONFIG_RTK_HDCPRX_2P2
extern void Hdmi_HDCP_2_2_Init(void);
#endif

static HDCP_KEY_T rx_key;
static unsigned char hdcpkey_enable=false;

/**
 * Hdmi_HdcpPortCWrite
 * HDCP DDC port setting download function with auto-address increase
 * where HDCP port ddc address for HOST side is 0x74/0x75
 *
 * @param <addr>			{ hdcp register address }
 * @param <data>			{ data pointer }
 * @param <num>			{ length data }
 * @return 				{ void }
 */
void inline Hdmi_HdcpPortCWrite(unsigned char addr ,HDMI_CONST unsigned char* value ,unsigned char num )
{
	hdmi_rx_reg_mask32(HDCP_PCR, ~_BIT0, 0, HDMI_RX_MAC);
	hdmi_rx_reg_write32(HDCP_AP, addr,HDMI_RX_MAC);

	while(num--)
		hdmi_rx_reg_write32(HDCP_DP, *(value++), HDMI_RX_MAC);

	hdmi_rx_reg_mask32(HDCP_PCR, ~_BIT0, _BIT0, HDMI_RX_MAC);
}

/**
 * Hdmi_HdcpPortWrite
 * HDCP DDC port setting download function
 * where HDCP port ddc address for HOST side is 0x74/0x75
 *
 * @param <addr>			{ hdcp register address }
 * @param <data>			{ data }
 * @return 				{ void }
 */
void inline Hdmi_HdcpPortWrite(unsigned char addr ,unsigned char value)
{
	hdmi_rx_reg_write32(HDCP_AP, addr, HDMI_RX_MAC);
	hdmi_rx_reg_write32(HDCP_DP, value, HDMI_RX_MAC);
}

/**
 * Hdmi_HdcpPortRead
 * HDCP DDC port setting read function
 * where HDCP port ddc address for HOST side is 0x74/0x75
 *
 * @param <addr>			{ hdcp register address }
 * @return 				{ read data }
 */
unsigned char  Hdmi_HdcpPortRead(unsigned char addr)
{
	hdmi_rx_reg_write32(HDCP_AP, addr, HDMI_RX_MAC);
	return hdmi_rx_reg_read32(HDCP_DP, HDMI_RX_MAC);
}

void Hdmi_HdcpFSM(void)
{
	UINT32 Aksv_flag;

	Aksv_flag = HDCP_FLAG1_get_wr_aksv_flag(hdmi_rx_reg_read32(HDCP_FLAG1, HDMI_RX_MAC));

	if (Aksv_flag)//Tx write Aksv
	{
#ifdef CONFIG_RTK_HDCPRX_2P2
		// Disable HDCP 2.2
		if(HDCP_2P2_CR_hdcp_2p2_en(hdmi_rx_reg_read32(HDCP_2P2_CR,HDMI_RX_MAC)))
		{
			hdmi_rx_reg_mask32(HDCP_2P2_CR, ~HDCP_2P2_CR_hdcp_2p2_en_mask, 0, HDMI_RX_MAC);
			hdmi_ioctl_struct.hdcp_state = 1;// HDCP 1.4
			HDMIRX_INFO("[HDCP] Discover 1.4, disable 2.2");
		}
#endif
		if(hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC) & _BIT0)//HDMI mode
		{
			hdmi_rx_reg_write32(HDCP_FLAG1, HDCP_FLAG1_wr_aksv_flag_mask, HDMI_RX_MAC);//clear Aksv flag
		}
	}

}

/**
 * Hdmi_HdcpInit
 * HDCP initial function
 *
 * @param 				{ void }
 * @return 				{ void }
 */
void Hdmi_HdcpInit(void)
{
	int i;

	if(!hdcpkey_enable)
		return;

	HDMIRX_INFO("[HDCP] Hdmi_HdcpInit");

	// Disable HDCP and clear HDCP address
	hdmi_rx_reg_write32(HDCP_CR,0x06,HDMI_RX_MAC);

	hdmi_rx_reg_mask32(HDCP_PCR ,~(HDCP_PCR_km_clk_sel_mask|HDCP_PCR_enc_tog_mask),0, HDMI_RX_MAC);

	// write BKsv into DDC channel
	Hdmi_HdcpPortCWrite(0x00 , rx_key.BKsv, 5);	//set KSV 40 bits
	#if 1 //must set in hdcp cts1.4
	Hdmi_HdcpPortWrite(0x40,HDCP_11_HDMI);				//set OESS for DVI
	#else
	Hdmi_HdcpPortWrite(0x40,HDCP_10_DVI);				//set OESS for DVI
	#endif

#ifdef CONFIG_RTK_HDCPRX_2P2
	Hdmi_HDCP_2_2_Init();
	HDMI_PRINTF("HDCP2Vision = %x",Hdmi_HdcpPortRead(0x50));
#endif

	// Clear Data Port
	hdmi_rx_reg_mask32(HDCP_PCR, ~_BIT0, 0, HDMI_RX_MAC);
	hdmi_rx_reg_write32(HDCP_AP, 0x00, HDMI_RX_MAC);
	for (i = 0; i < 640; i++) {
		hdmi_rx_reg_write32(HDCP_AP, 0x43 + (i<<8), HDMI_RX_MAC);
		hdmi_rx_reg_write32(HDCP_DP, 0x00, HDMI_RX_MAC);
	}

	hdmi_rx_reg_write32(HDCP_AP, 0x00, HDMI_RX_MAC);
	hdmi_rx_reg_mask32(HDCP_PCR, ~_BIT0, _BIT0, HDMI_RX_MAC);

	// Write device private key
	for(i=0;i<320;i++) {
		hdmi_rx_reg_write32(HDCP_DKAP,rx_key.Key[i], HDMI_RX_MAC);
	}

	hdmi_rx_reg_write32(HDCP_CR,0x00, HDMI_RX_MAC);
 
	// Enable HDCP function for all
	hdmi_rx_reg_mask32(HDCP_CR,~(_BIT7|_BIT0),(_BIT7|_BIT0), HDMI_RX_MAC);//for New_AC_Mode Enable,fix simplay bug

}

void HdmiRx_enable_hdcp(unsigned char *bksv, unsigned char *device_key)
{
	if(hdcpkey_enable)//Already save the key, return
		return;

	memset(&rx_key, 0, sizeof(rx_key));
	memcpy(&rx_key.BKsv,bksv,5);
	memcpy(&rx_key.Key,device_key,320);

	hdcpkey_enable = true;
	HDMIRX_INFO("HdmiRx_enable_hdcp");

	if(mipi_top.hdmi_rx_init)
	{
		Hdmi_SetHPD(0);
		Hdmi_HdcpInit();
		HDMI_DELAYMS(500);
		Hdmi_SetHPD(1);
	}

	return;
}

