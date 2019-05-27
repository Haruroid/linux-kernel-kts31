#include "v4l2_hdmi_dev.h"
#include "hdmiInternal.h"


unsigned char lc128[16]={ 0xB5, 0xD8, 0xE9, 0xAB, 0x5F, 0x8A ,0xFE, 0xCA ,0x38, 0x55, 0xB1, 0xA5, 0x1E, 0xC9 ,0xBC, 0x0F};

static unsigned char hdcp2p2key_enable=false;
static unsigned char CTS_key[864];

/*=================== extern Variable/Function ===================*/
extern HDMIRX_IOCTL_STRUCT_T hdmi_ioctl_struct;

extern void hdcp2_save_keyset(unsigned char *pKeySet);
extern int h2MessageParse(unsigned char *message, unsigned int length);
extern int h2MessagePoll(unsigned char *message, unsigned int length);
extern void Hdmi_HdcpPortWrite(unsigned char addr ,unsigned char value);
/*======================================================*/


//***********************************************************************************//
//                                        HDCP2-2  global variable
//********************************************************************************** //
unsigned char g_bmsg_id ,g_RX_state , g_Rx_Send_State;

unsigned char TX_msg_id[864] ;
unsigned char bLen;

unsigned int b_Tx_Len;

//For HDCP2_2  TEST 20140815
#define HDCP_RXstatus_addr    0x70
#define IDEL  					0   //TX=>RX
#define AKE_INIT 				2   //TX=>RX
#define AKE_Send_Cert 			3   //RX=>TX
#define AKE_NO_Stored_Km  		4   //TX=>RX
#define AKE_Stored_Km  			5   //TX=>RX
#define AKE_Send_H_prime  		7   //RX=>TX
#define AKE_Send_Pariring_Info  8   //RX=>TX
#define LC_Init               	9   //TX=>RX
#define LC_Send_L_prime         10  //RX=>TX
#define SKE_Send_Eks            11  //TX=>RX

//RX send FSM
#define  RX_FSM_SEND_START		1
#define  RX_FSM_WAIT			2
#define  RX_FSM_FINISH			3

//HDCP 2.2 OFFSET
#define Write_Message_offset	0x60
#define Rxstatus_offset			0x70
#define Read_Message_offset		0x80

struct work_struct hdcp2p2_msg_work;

void Hdmi_HDCP_2_2_msg_work_func(struct work_struct *work);


char Get_Hdmi_hdcp_2_2_Write_Message(void)
{
	if (HDCP_2P2_SR0_get_irq_wr_msg_done(hdmi_rx_reg_read32(HDCP_2P2_SR0, HDMI_RX_MAC)) )
		return 1;
	else
		return 0;
}

char Get_Hdmi_hdcp_2_2_Read_Message(void)
{
	if (HDCP_2P2_SR0_get_irq_rd_msg_done(hdmi_rx_reg_read32(HDCP_2P2_SR0, HDMI_RX_MAC)) )
		return 1;
	else
		return 0;
}

void Clear_Hdmi_hdcp_2_2_Read_Status(void)
{
      hdmi_rx_reg_write32(HDCP_2P2_SR0, HDCP_2P2_SR0_irq_rd_msg_done_mask, HDMI_RX_MAC);
}

void HW_Cipher_Setting(void)
{
	hdmi_rx_reg_mask32(HDCP_2P2_CR, ~(HDCP_2P2_CR_fw_mode_riv_mask), HDCP_2P2_CR_fw_mode_riv_mask, HDMI_RX_MAC);	

	//lc 128
	hdmi_rx_reg_mask32(HDCP_2P2_LC0, ~HDCP_2P2_LC0_lc_mask, (lc128[15]|(lc128[14]<<8)|(lc128[13]<<16)|(lc128[12]<<24)), HDMI_RX_MAC);	
	hdmi_rx_reg_mask32(HDCP_2P2_LC1, ~HDCP_2P2_LC1_lc_mask, (lc128[11]|(lc128[10]<<8)|(lc128[9]<<16)|(lc128[8]<<24)), HDMI_RX_MAC);	
	hdmi_rx_reg_mask32(HDCP_2P2_LC2, ~HDCP_2P2_LC2_lc_mask, (lc128[7]|(lc128[6]<<8)|(lc128[5]<<16)|(lc128[4]<<24)), HDMI_RX_MAC);
	hdmi_rx_reg_mask32(HDCP_2P2_LC3, ~HDCP_2P2_LC3_lc_mask, (lc128[3]|(lc128[2]<<8)|(lc128[1]<<16)|(lc128[0]<<24)), HDMI_RX_MAC);
}

void KS_Setting(unsigned char* bKs)
{
	hdmi_rx_reg_mask32(HDCP_2P2_CR, ~(HDCP_2P2_CR_fw_mode_riv_mask), HDCP_2P2_CR_fw_mode_riv_mask, HDMI_RX_MAC);	
	//initial ks value 
	hdmi_rx_reg_mask32(HDCP_2P2_KS0, ~HDCP_2P2_KS0_ks_mask, ((*(bKs+15))|((*(bKs+14))<<8)|((*(bKs+13))<<16)|((*(bKs+12))<<24)), HDMI_RX_MAC);	
	hdmi_rx_reg_mask32(HDCP_2P2_KS1, ~HDCP_2P2_KS1_ks_mask, ((*(bKs+11))|((*(bKs+10))<<8)|((*(bKs+9))<<16)|((*(bKs+8))<<24)), HDMI_RX_MAC);	
	hdmi_rx_reg_mask32(HDCP_2P2_KS2, ~HDCP_2P2_KS2_ks_mask, ((*(bKs+7))|((*(bKs+6))<<8)|((*(bKs+5))<<16)|((*(bKs+4))<<24)), HDMI_RX_MAC);
	hdmi_rx_reg_mask32(HDCP_2P2_KS3, ~HDCP_2P2_KS3_ks_mask, ((*(bKs+3))|((*(bKs+2))<<8)|((*(bKs+1))<<16)|((*bKs)<<24)), HDMI_RX_MAC);

	hdmi_rx_reg_mask32(HDCP_2P2_CR, ~HDCP_2P2_CR_ks_done_mask, HDCP_2P2_CR_ks_done_mask, HDMI_RX_MAC);		
	HDMIRX_INFO("[HDCP2.2]KS_Setting bKs  = 0x%x , 0x%x ,0x%x ,0x%x ,0x%x ,0x%x ,0x%x ,0x%x",(*bKs),*(bKs+1),*(bKs+2),*(bKs+3),*(bKs+4),*(bKs+5),*(bKs+6),*(bKs+7));
}

void Riv_Setting(unsigned char* bRiv)
{
	hdmi_rx_reg_mask32(HDCP_2P2_RIV0, ~HDCP_2P2_RIV0_riv_mask, (*(bRiv+7)|(*(bRiv+6)<<8)|(*(bRiv+5)<<16)|(*(bRiv+4)<<24)), HDMI_RX_MAC);	
	hdmi_rx_reg_mask32(HDCP_2P2_RIV1, ~HDCP_2P2_RIV1_riv_mask, (*(bRiv+3)|(*(bRiv+2)<<8)|(*(bRiv+1)<<16)|((*bRiv)<<24)), HDMI_RX_MAC);

	HDMIRX_INFO("[HDCP2.2]Riv_Setting bRiv = 0x%x , 0x%x ,0x%x ,0x%x ,0x%x ,0x%x ,0x%x ,0x%x",(*bRiv),*(bRiv+1),*(bRiv+2),*(bRiv+3),*(bRiv+4),*(bRiv+5),*(bRiv+6),*(bRiv+7));
}

void HdmiRx_enable_hdcp2p2(unsigned char *key)
{
	if(hdcp2p2key_enable)//Already save the key, return
		return;

	memset(&CTS_key, 0, sizeof(CTS_key));
	memcpy(&CTS_key,key,sizeof(CTS_key));

	HDMIRX_INFO("[HDCP2.2] Device ID = 0x%02x%02x%02x%02x%02x",CTS_key[20],CTS_key[19],CTS_key[18],CTS_key[17],CTS_key[16]);
	hdcp2p2key_enable = true;

	return;
}

void Hdmi_HDCP_2_2_Init(void)
{
	struct reset_control *reset_tp,*reset_cp;
	struct clk *clk_tp,*clk_cp;
	static unsigned char clk_init_flag=0;

	HDMIRX_INFO("[HDCP2.2] Hdmi_HDCP_2_2_Init");

	if(!hdcp2p2key_enable)
	{
		HDMIRX_INFO("[HDCP2.2] key not ready");
		return;
	}

	//Enable TP&CP clock
	if(!clk_init_flag)
	{
		reset_tp = rstc_get("rstn_tp");
		reset_cp = rstc_get("rstn_cp");

		clk_tp = clk_get(NULL, "clk_en_tp");
		clk_cp = clk_get(NULL, "clk_en_cp");

		reset_control_deassert(reset_tp);
		reset_control_deassert(reset_cp);
		clk_prepare_enable(clk_tp);
		clk_prepare_enable(clk_cp);
		clk_init_flag=1;

		INIT_WORK(&hdcp2p2_msg_work, Hdmi_HDCP_2_2_msg_work_func);
	}

	hdmi_rx_reg_mask32(HDCP_2P2_CR, ~HDCP_2P2_CR_hdcp_2p2_en_mask, 0, HDMI_RX_MAC);	
	hdmi_rx_reg_mask32(HDCP_2P2_SR1, ~HDCP_2P2_SR1_irq_wr_msg_done_en_mask, HDCP_2P2_SR1_irq_wr_msg_done_en_mask, HDMI_RX_MAC);

	// Load hdcp key
	hdcp2_save_keyset(&CTS_key[0]);

	// SET Lc  
	HW_Cipher_Setting();

	// enable hdcp 2.2 HW
	hdmi_rx_reg_mask32(HDCP_2P2_CR,~HDCP_2P2_CR_hdcp_2p2_en_mask, HDCP_2P2_CR_hdcp_2p2_en_mask, HDMI_RX_MAC);
	Hdmi_HdcpPortWrite(0x50,0x04);//support hdcp2.2
	hdmi_rx_reg_mask32(HDCP_2P2_SR1,~HDCP_2P2_SR1_irq_wr_msg_done_en_mask, HDCP_2P2_SR1_irq_wr_msg_done_en_mask, HDMI_RX_MAC);
}

char  Hdmi_HDCP2_2_Write_Data_to_TX(unsigned char* bSendData,unsigned short wLen)
{
	unsigned int i;

	if(bSendData ==NULL)     return FALSE;
	HDMI_PRINTF("[HDCP2.2] bSendData(%d)\n",bSendData[0]);
	for(i=0;i<wLen;i++)
	{
		hdmi_rx_reg_write32(HDCP_MSAP, i, HDMI_RX_MAC);
		hdmi_rx_reg_write32(HDCP_MSDP, *(bSendData+i), HDMI_RX_MAC);
	}

	hdmi_rx_reg_write32(HDCP_AP, HDCP_RXstatus_addr, HDMI_RX_MAC);
	hdmi_rx_reg_write32(HDCP_DP, (wLen&0xFF), HDMI_RX_MAC); 	
	hdmi_rx_reg_write32(HDCP_AP, HDCP_RXstatus_addr+1, HDMI_RX_MAC);
	hdmi_rx_reg_write32(HDCP_DP, ((wLen&0x3FF)>>8), HDMI_RX_MAC);

	return TRUE;
}
#if 0
void driv_Wait_TX_read_finish(void)
{
	while(!Get_Hdmi_hdcp_2_2_Read_Message());// TODO: Add 100ms timeout
    
	hdmi_rx_reg_write32(HDCP_2P2_SR0, HDCP_2P2_SR0_irq_rd_msg_done_mask, HDMI_RX_MAC);

	HDMI_PRINTF("[HDCP2.2] TX read finish !!!!!     \n" );
}
#endif
void Hdmi_HDCP_2_2_Force_Cipher(void)
{
	hdmi_rx_reg_mask32(HDCP_2P2_CR,~(HDCP_2P2_CR_switch_state_mask), HDCP_2P2_CR_switch_state(0x10), HDMI_RX_MAC);// set authenticated
	hdmi_rx_reg_mask32(HDCP_2P2_CR,~(HDCP_2P2_CR_apply_state_mask|HDCP_2P2_CR_fw_mode_riv_mask),
									(HDCP_2P2_CR_apply_state_mask|HDCP_2P2_CR_fw_mode_riv_mask), HDMI_RX_MAC);

}

void Get_Msg_Command(void)
{
	//hdcp2.2 sw parse the message for test gpo
	h2MessageParse(TX_msg_id, bLen);

	if (g_bmsg_id ==SKE_Send_Eks)
	{
		hdmi_rx_reg_mask32(HDCP_2P2_CR, ~HDCP_2P2_CR_switch_state_mask, HDCP_2P2_CR_switch_state(0x10), HDMI_RX_MAC);
		hdmi_rx_reg_mask32(HDCP_2P2_CR, ~HDCP_2P2_CR_apply_state_mask, HDCP_2P2_CR_apply_state_mask, HDMI_RX_MAC);
	}

}

void Send_Msg_Command(void)
{
	if (g_Rx_Send_State ==  RX_FSM_SEND_START )
	{
		g_Rx_Send_State = RX_FSM_WAIT ;
	}

	if ((g_RX_state == AKE_Send_H_prime)&&(g_bmsg_id ==  AKE_NO_Stored_Km ))
	{
		HDMI_PRINTF("[HDCP2.2] Send pairing\n" );
		h2MessagePoll(NULL, 0);
		g_Rx_Send_State = RX_FSM_SEND_START;
		g_RX_state = AKE_Send_Pariring_Info;
	}
	else
	{
		g_Rx_Send_State = RX_FSM_FINISH ;
	}
	hdmi_rx_reg_write32(HDCP_2P2_SR0, HDCP_2P2_SR0_irq_rd_msg_done_mask, HDMI_RX_MAC);
	HDMI_PRINTF("[HDCP2.2] Tx Read finish !!!!!\n" );

}

void Hdmi_HDCP_2_2_msg_work_func(struct work_struct *work)
{
	Get_Msg_Command();
	Send_Msg_Command();
}


void  Hdmi_HDCP_2_2_msg_hander(void)
{
	unsigned char i;
	if (Get_Hdmi_hdcp_2_2_Write_Message())  //GET TX mesage 
 	{
 		if(HDCP_CR_get_hdcp_en(hdmi_rx_reg_read32(HDCP_CR,HDMI_RX_MAC)))// Must close hdcp1.4
 		{
			hdmi_rx_reg_mask32(HDCP_CR, ~HDCP_CR_hdcp_en_mask,0, HDMI_RX_MAC);
			HDMIRX_INFO("[HDCP2.2] Discover 2.2, disable 1.4");
		}
		hdmi_rx_reg_mask32(HDCP_2P2_CR, ~HDCP_2P2_CR_hdcp_2p2_en_mask, HDCP_2P2_CR_hdcp_2p2_en_mask, HDMI_RX_MAC);//open hdcp2p2
		hdmi_rx_reg_write32(HDCP_MSAP, 0, HDMI_RX_MAC);
		g_bmsg_id = hdmi_rx_reg_read32(HDCP_MSDP, HDMI_RX_MAC);
		TX_msg_id[0]= g_bmsg_id;
		HDMI_PRINTF("[HDCP2.2] bmsg_id  =%x\n",g_bmsg_id);

		g_Rx_Send_State =  RX_FSM_SEND_START;
		switch(g_bmsg_id)
		{
			case AKE_INIT:
				HDMIRX_INFO("[HDCP2.2] AKE_INIT");
				hdmi_ioctl_struct.hdcp_state = 2;// HDCP 2.2
				g_RX_state =  AKE_Send_Cert ;//AKE_Send_Cert ;
				bLen =11;
				break;
			case AKE_NO_Stored_Km:
				HDMIRX_INFO("[HDCP2.2] AKE_NO_Stored_Km");
				g_RX_state =  AKE_Send_H_prime;//AKE_Send_H_prime ;
				bLen =128;
				break;
			case AKE_Stored_Km:
				HDMIRX_INFO("[HDCP2.2] AKE_Stored_Km");
				g_RX_state =  AKE_Send_H_prime;//AKE_Send_H_prime ;
				bLen =32;
				break;
			case LC_Init:
				//HDMIRX_INFO("[HDCP2.2] LC_Init");  // Need reply in 20ms
				g_RX_state =  LC_Send_L_prime;//LC_Send_L_prime ;
				bLen =8;
				break;
			case SKE_Send_Eks:
				HDMIRX_INFO("[HDCP2.2] SKE_Send_Eks");
				hdmi_rx_reg_mask32(HDCP_2P2_CR, ~HDCP_2P2_CR_switch_state_mask, HDCP_2P2_CR_switch_state(0x01), HDMI_RX_MAC);	
				hdmi_rx_reg_mask32(HDCP_2P2_CR,~HDCP_2P2_CR_apply_state_mask, HDCP_2P2_CR_apply_state_mask, HDMI_RX_MAC);
				bLen =24; 
				break;
			default:
				bLen =0;
				HDMI_PRINTF("[HDCP2.2] Unknown message\n");
				break;
		}

		for(i=1;i<=bLen;i++)
		{
			TX_msg_id[i] = hdmi_rx_reg_read32(HDCP_MSDP, HDMI_RX_MAC);
		}

		// for HW Cipher setting
		if(g_bmsg_id ==SKE_Send_Eks)
			Riv_Setting((&(TX_msg_id[17])));// set Riv address

		//Clear interrupt
		hdmi_rx_reg_write32(HDCP_2P2_SR0, HDCP_2P2_SR0_irq_wr_msg_done_mask, HDMI_RX_MAC);
		hdmi_rx_reg_write32(HDMI_INTCR,0x80,HDMI_RX_MAC);

		schedule_work(&hdcp2p2_msg_work);
	}
		
}

char drvif_Hdmi_HDCP2_2_Write_Data_to_TX(unsigned char* bSendData,unsigned short wLen)
{
	return  (Hdmi_HDCP2_2_Write_Data_to_TX(bSendData,wLen));
}

void drvif_KS_Setting(unsigned char* bKs)
{
	return  (KS_Setting(bKs));
}

