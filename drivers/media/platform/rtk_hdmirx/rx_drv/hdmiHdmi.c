#include <linux/gpio.h>

#include "hdmiInternal.h"

#include "mipi_wrapper.h"

/*===================== HDMI 2.0 flag =======================*/
#if HDMI2p0
u_int8_t   bHDMI_6G_flag,b6G_detect_cnt;
u_int8_t   bHDMI_420_Space;// For YUV420

//SCDC Reg mapping
u_int8_t bScdc_reset_flag;
#define SCDC_TMDS_Config	 0x20

extern UINT8 bSCDC_Enable_flag ;
#endif
/*======================================================*/

#define HDMI_MEMSURE_RETRY	2

#define AVI_Data_BYTE1	   (4)
#define AVI_Data_BYTE3	   (6)
#define AVI_Data_BYTE4	   (7)
#define AVI_Data_BYTE5	   (8)
#define AVI_Y1Y0_mask		0x60
#define AVI_YQ1YQ0_mask		0xC0
#define AVI_Q_Range_mask	0x0c
#define RGB_Default			0
#define RGB_Limite_Range	1
#define RGB_Full_Range		2
#define Not_Read			3
#define YUV_Limite_Range	0
#define YUV_Full_Range		1


HDMI_CONST HDMI_AUDIO_PLL_COEFF_T audio_pll_coeff[] = {
	{ 32000, _AUDIO_256_TIMES, 0},
	{ 44100, _AUDIO_256_TIMES, 0},
	{ 48000, _AUDIO_256_TIMES, 0},
	{ 88200, _AUDIO_256_TIMES, 1},
	{ 96000, _AUDIO_256_TIMES, 1},
	{ 176400, _AUDIO_128_TIMES, 2},
	{ 192000, _AUDIO_128_TIMES, 3}
};

HDMI_CONST HDMI_AUDIO_PLL_PARAM_T hdmi_audiopll_param[] = {
	{ 2,   20,	2,	8,	1,	0x1D, 0xDC,  "32K"	   },//0x1E, 0xB0
	{ 2,   20,	2,	6,	1,	0xFE, 0x7A,  "44.1K"  },//0xFC, 0x68
	{ 2,   22,	2,	6,	1,	0x07, 0x40,  "48K"	   },
	{ 2,   20,	1,	6,	1,	0xFE, 0x7A,  "88.2K"  },
	{ 2,   22,	1,	6,	1,	0x07, 0x40,  "96K"	   },
	{ 2,   20,	1,	6,	0,	0xFE, 0x7A,  "176.4K" },
	{ 2,   22,	1,	6,	0,	0x07, 0x40,  "192K"    }
};

HDMI_CONST unsigned int AUDIO_CHANNEL_STATUS[] = {
	44100,
	   0,				   // 000 indicate standard no support
	48000,
	32000,
	22000,
	   0,
	24000,
	   0,
	88200,
	   0,
	96000,
	   0,
	176000,
	   0,
	192000,
	   0,
};

HDMI_CONST HDMI_COLOR_SPACE_T ColorMap[] = {
	COLOR_RGB,		// 0
	COLOR_YUV422,	// 01
	COLOR_YUV444,	// 10
	COLOR_YUV420,	// 11
};

static unsigned int ACR_N=0;
HDMI_INFO_T hdmi;
unsigned char HDMI_Audio_Conut = 0;

/*=================== extern Variable/Function ===================*/
extern HDMIRX_IOCTL_STRUCT_T hdmi_ioctl_struct;

extern void Hdmi_PhyInit(void);
extern int rtd_hdmiRx_cmd(u_int8_t cmd,  void * arg);
extern void hdmi_z0_set(unsigned char port, unsigned char lane, unsigned char enable);
extern bool HDMI_Bit_Err_One_Time_Detect(u_int8_t period, u_int16_t thd);
extern bool Hdmi_MeasureTiming(HDMI_TIMING_T *timing, int b);
extern void set_hdmirx_wrapper_control_0(int fifo_stage,int fw_dma_en,int polarity_set_en,int polarity_set,int yuv_fmt,int hdmirx_en);
extern unsigned char hdmirx_wrapper_convert_color_fmt(HDMI_COLOR_SPACE_T color);
extern void set_hdmirx_wrapper_interrupt_en(int ver_err_en,int hor_err_en,int polarity_detect_en);
/*======================================================*/


void HdmiSetAPStatus(HDMIRX_APSTATUS_T apstatus)
{
	unsigned long value = apstatus;
	rtd_hdmiRx_cmd(IOCTRL_HDMI_SET_APSTATUS, (void *)value);
}

void HdmiGetStruct(HDMIRX_IOCTL_STRUCT_T* ptr)
{
	rtd_hdmiRx_cmd(IOCTRL_HDMI_GET_STRUCT, ptr);
}

HDMI_DVI_MODE_T IsHDMI(void)
{
#if HDMI2p0
	if(bHDMI_6G_flag)
		return MODE_HDMI; // HDMI mode
#endif

	if(HDMI_SR_get_mode(hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC)))
	{
		//HDMIRX_DEBUG("[%s] HDMI mode",__FUNCTION__);
		return MODE_HDMI;
	}
	else
	{
		//HDMIRX_DEBUG("[%s] DVI mode",__FUNCTION__);
		return MODE_DVI;
	}
}

int Cbus_GetRx5v(void)
{
	return ISO_CBUS_TX_PHY_CTRL5_get_REGO_5V_DETECT(hdmi_rx_reg_read32(ISO_CBUS_TX_PHY_CTRL5, HDMI_TX_CBUS));
}

void Hdmi_SetHPD(char high)
{
	HDMIRX_INFO("Set HPD(%u)",high);

	if (high)
	{
		gpio_direction_output(hdmi.gpio_hpd_ctrl,0);
	}
	else
	{
		gpio_direction_output(hdmi.gpio_hpd_ctrl,1);
	}
}

void Hdmi_Power(char enable)
{
	if(enable)
	{
		hdmi_rx_reg_mask32(TMDS_OUTCTL, ~(TMDS_OUTCTL_tbcoe_mask | TMDS_OUTCTL_tgcoe_mask | TMDS_OUTCTL_trcoe_mask | TMDS_OUTCTL_ocke_mask),
							(TMDS_OUTCTL_tbcoe(1) | TMDS_OUTCTL_tgcoe(1) | TMDS_OUTCTL_trcoe(1) | TMDS_OUTCTL_ocke(1)), HDMI_RX_MAC);//enable PLL TMDS, RGB clock output

#if 1//Changed by Ken2016/1/18
		hdmi_rx_reg_mask32(LDO, ~(LDO_reg_p0_ldo_pow_mask), 0, HDMI_RX_PHY);
		hdmi_rx_reg_mask32(MBIAS, ~(0x1), 0x1, HDMI_RX_PHY);
#else
		//LDO power on port0
		hdmi_rx_reg_mask32(LDO, ~(LDO_reg_p0_ldo_pow_mask|LDO_reg_p1_ldo_pow_mask|LDO_reg_p2_ldo_pow_mask), LDO_reg_p0_ldo_pow_mask, HDMI_RX_PHY);
#endif
		hdmi_z0_set(0 /* Port0 */, LN_ALL, 1);
	}
	else
	{
		hdmi_rx_reg_mask32(TMDS_OUTCTL, ~(TMDS_OUTCTL_tbcoe_mask | TMDS_OUTCTL_tgcoe_mask | TMDS_OUTCTL_trcoe_mask | TMDS_OUTCTL_ocke_mask),
							(TMDS_OUTCTL_tbcoe(0) | TMDS_OUTCTL_tgcoe(0) | TMDS_OUTCTL_trcoe(0) | TMDS_OUTCTL_ocke(0)), HDMI_RX_MAC);//disable PLL TMDS, RGB clock output

#if 1//Changed by Ken2016/1/18
		hdmi_rx_reg_mask32(LDO,
			~(LDO_reg_p0_ldo_pow_mask|LDO_reg_p1_ldo_pow_mask|LDO_reg_p2_ldo_pow_mask),
			(LDO_reg_p0_ldo_pow_mask|LDO_reg_p1_ldo_pow_mask|LDO_reg_p2_ldo_pow_mask), HDMI_RX_PHY);
#else
		//LDO power OFF
		hdmi_rx_reg_mask32(LDO, ~(LDO_reg_p0_ldo_pow_mask|LDO_reg_p1_ldo_pow_mask|LDO_reg_p2_ldo_pow_mask), 0, HDMI_RX_PHY);
#endif
		hdmi_z0_set(0 /* Port0 */, LN_ALL, 0);
	}

}

void Hdmi_TmdsInit(void)
{
	//initial HDMI
	hdmi_rx_reg_mask32(MHL_DEMUX_CTRL, ~(MHL_DEMUX_CTRL_mhl_pp_en_mask|MHL_DEMUX_CTRL_mhl_en_mask), 0, HDMI_RX_MHL);
	hdmi_rx_reg_mask32(MHL_HDMI_MAC_CTRL,~(MHL_HDMI_MAC_CTRL_pp_mode_output_mask|MHL_HDMI_MAC_CTRL_packet_mhl_en_mask|
											MHL_HDMI_MAC_CTRL_xor_pixel_sel_mask|MHL_HDMI_MAC_CTRL_ch_dec_pp_mode_en_mask), 0, HDMI_RX_MAC);

	//for BCH & debounce
	/*hdmiport_mask(HDMI_TMDS_ERRC_reg,~HDMI_TMDS_ERRC_nl_mask,HDMI_TMDS_ERRC_nl(4));*/ 	// 1+8 cycle debounce + de masking transition of vs/hs + masking first 8-line de
	hdmi_rx_reg_mask32(HDMI_BCHCR, ~(HDMI_BCHCR_enrwe_mask|HDMI_BCHCR_bche_mask), (HDMI_BCHCR_enrwe(1)|HDMI_BCHCR_bche(1)), HDMI_RX_MAC);// BCH function enable

	//hdmiport_mask(TMDS_PWDCTL_reg,~(TMDS_PWDCTL_ebip_mask|TMDS_PWDCTL_egip_mask|TMDS_PWDCTL_erip_mask|TMDS_PWDCTL_ecc_mask),(TMDS_PWDCTL_ebip(1)|TMDS_PWDCTL_egip(1)|TMDS_PWDCTL_erip(1)|TMDS_PWDCTL_ecc(1)));				//enable TMDS input
	hdmi_rx_reg_mask32(TMDS_PWDCTL, ~(TMDS_PWDCTL_ebip_mask|TMDS_PWDCTL_egip_mask|TMDS_PWDCTL_erip_mask|TMDS_PWDCTL_ecc_mask), 0, HDMI_RX_MAC);//enable TMDS input

	//hdmiport_mask(TMDS_Z0CC_reg,~TMDS_Z0CC_hde_mask,TMDS_Z0CC_hde(1));					//HDMI&DVI function enable
	hdmi_rx_reg_mask32(TMDS_Z0CC, ~TMDS_Z0CC_hde_mask, 0, HDMI_RX_MAC);//HDMI&DVI function disable

	hdmi_rx_reg_mask32(HDMI_VCR, ~HDMI_VCR_iclk_sel_mask, HDMI_VCR_iclk_sel(0), HDMI_RX_MAC);
	hdmi_rx_reg_mask32(TMDS_CPS, ~TMDS_CPS_clkv_meas_sel_mask, TMDS_CPS_clkv_meas_sel(3), HDMI_RX_MAC);//measure input clock source sel

	hdmi_rx_reg_mask32(HDMI_AVMCR, ~HDMI_AVMCR_ve_mask, HDMI_AVMCR_ve(1), HDMI_RX_MAC);//need to enable video before enable TMDS clock
	hdmi_rx_reg_mask32(TMDS_OUTCTL, ~(TMDS_OUTCTL_ocke_mask|TMDS_OUTCTL_tbcoe_mask|TMDS_OUTCTL_tgcoe_mask|TMDS_OUTCTL_trcoe_mask),
									(TMDS_OUTCTL_ocke(1)|TMDS_OUTCTL_tbcoe(1)|TMDS_OUTCTL_tgcoe(1)|TMDS_OUTCTL_trcoe(1)), HDMI_RX_MAC);//enable TMDS output
	hdmi_rx_reg_mask32(TMDS_DPC_SET0, ~TMDS_DPC_SET0_dpc_bypass_dis_mask, TMDS_DPC_SET0_dpc_en(1), HDMI_RX_MAC);// video function block enable
}

void Hdmi_MacInit(void)
{
	hdmi_rx_reg_write32(HDMI_SCR, 0x123, HDMI_RX_MAC);//DVI/HDMI condition(A,B) select  /* hdcp cts1.4 : Bstatus (MSB): 0x00*/
	hdmi_rx_reg_mask32(HDMI_AFCR, ~(HDMI_AFCR_audio_test_enable_mask|HDMI_AFCR_afifowe_mask|HDMI_AFCR_afifore_mask),
									HDMI_AFCR_audio_test_enable(0)|HDMI_AFCR_afifowe(1)|HDMI_AFCR_afifore(2), HDMI_RX_MAC);//Enable Audio FIFO
	hdmi_rx_reg_write32(HDMI_AVMCR, 0x48, HDMI_RX_MAC);//enable video & audio output
	hdmi_rx_reg_write32(HDMI_WDCR0, 0x00, HDMI_RX_MAC);//disable watch dog
	//hdmiport_out(HDMI_HDMI_BCHCR_reg , 0x19); // Enable BCH Function
	hdmi_rx_reg_mask32(HDMI_BCHCR, (HDMI_BCHCR_pe_mask|HDMI_BCHCR_bche_mask|HDMI_BCHCR_enrwe_mask|HDMI_BCHCR_spcss_mask),
								(HDMI_BCHCR_pe_mask|HDMI_BCHCR_bche_mask|HDMI_BCHCR_enrwe_mask|HDMI_BCHCR_spcss_mask), HDMI_RX_MAC);// Enable BCH Function
	//hdmi_rx_reg_write32(HDMI_PVGCR0, 0x09, HDMI_RX_MAC);//For HDMI Packet

#if HDMI2p0   // for hdmi 2.0 spec use
	hdmi_rx_reg_mask32(SCDC_CR, (~SCDC_CR_scdc_en_mask), SCDC_CR_scdc_en_mask, HDMI_RX_MAC);
	//hdmiport_mask(HDMI_SCDC_PCR_reg,(~HDMI_SCDC_PCR_i2c_free_num_mask),HDMI_SCDC_PCR_i2c_free_num(0x7f));
	hdmi_rx_reg_mask32(SCDC_PCR, (~SCDC_PCR_i2c_free_num_mask), SCDC_PCR_i2c_free_num(0x0a), HDMI_RX_MAC);
	bScdc_reset_flag =1;
	bHDMI_6G_flag = 0 ;  //detect 6 G or not
	bSCDC_Enable_flag = 0;
#endif
#if 1 //fix panasonic issue((dvd mode + hdcp) fail)
	hdmi_rx_reg_mask32(HDCP_PCR,
		~(HDCP_PCR_km_clk_sel_mask|HDCP_PCR_hdcp_vs_sel_mask),
		(HDCP_PCR_km_clk_sel_mask|HDCP_PCR_hdcp_vs_sel_mask), HDMI_RX_MAC);
#endif
	hdmi_rx_reg_mask32(HDMI_VCR, ~(HDMI_VCR_csam_mask | HDMI_VCR_prdsam_mask), HDMI_VCR_csam(1) | HDMI_VCR_prdsam(1), HDMI_RX_MAC);
//	hdmiport_mask(HDMI_HDMI_AOCR_VADDR,(unsigned char)~(0xff),0x00);//Disable SPDIF/I2S Output
//	hdmiport_mask(HDMI_HDMI_AOCR_VADDR,~_BIT10,_BIT10);//Hold avmute value outside v-sync region
	hdmi_rx_reg_write32(HDMI_MAGCR0, 0xE000/*0x14*/, HDMI_RX_MAC);
	hdmi_rx_reg_write32(HDMI_PTRSV1, 0x82, HDMI_RX_MAC);
	// clear HDMI interrupt control register
	hdmi_rx_reg_write32(HDMI_INTCR, 0, HDMI_RX_MAC);
	hdmi_rx_reg_mask32(HDMI_VCR, ~(_BIT7) , 0, HDMI_RX_MAC);// not inverse EVEN/ODD

	hdmi_rx_reg_mask32(HDMI_PAMICR , ~_BIT0, 0x00, HDMI_RX_MAC);//Disable packet variation Watch Dog
	//hdmiport_mask(HDMI_HDMI_AOCR_VADDR,0x00, 0x00);//Disable SPDIF/I2S Output
	hdmi_rx_reg_mask32(HDMI_AOCR, (~0x0ff), 0xFF, HDMI_RX_MAC);//Enable SPDIF/I2S Output

	//Magellan_clock set
	/*
	hdmi_out(HDMI_CLKDET_GDI_TMDSCLK_SETTING_00_VADDR,0x10010); //Set TC
	hdmi_out(HDMI_CLKDET_GDI_TMDSCLK_SETTING_01_VADDR,0x7800047); //clk detect disable
	*/
	/* cycle debounce*/
	//hdmiport_mask (HDMI_TMDS_ERRC_reg, ~HDMI_TMDS_ERRC_nl_mask, HDMI_TMDS_ERRC_nl(4));
	hdmi_rx_reg_mask32(TMDS_ERRC, ~TMDS_ERRC_nl_mask, TMDS_ERRC_nl(3), HDMI_RX_MAC);
	//hdmi_rx_reg_mask32(MHL3_CTRL, ~MHL3_CTRL_errc_sel_mask, MHL3_CTRL_errc_sel(4), HDMI_RX_MHL);

	hdmi_rx_reg_mask32(HDMI_BCHCR, ~(HDMI_BCHCR_enrwe_mask|HDMI_BCHCR_bche_mask), (HDMI_BCHCR_enrwe(1)|HDMI_BCHCR_bche(1)), HDMI_RX_MAC);// BCH function enable
}

void Audio_CTS_Bound(void)
{
	unsigned long cts_up, cts_low,b;
	HDMIRX_IOCTL_STRUCT_T isr_info;
	HdmiGetStruct(&isr_info);
	b = isr_info.b;

	if (b>5600)	//6G timing
	{
		cts_up = 0xfffff ;//990000;
		cts_low = 445500 *7/8 ;
	}
	else if (b >2750) //	 3G timing
	{
		cts_up = 421875 * 9/8 ; //990000;
		cts_low = 222750 *7/8 ;
	}
	else if (b >1380)//148M
	{
		cts_up = 421875 * 9/8  ;//990000;
		cts_low = 140625 *7/8 ;
	}
	else if (b>660)
	{
		cts_up = 234375 * 9/8  ;//990000;
		cts_low = 74250 *7/8 ;
	}
	else
	{
		cts_up = 60060 * 9/8  ;//990000;
		cts_low = 25200 *7/8 ;

	}

	hdmi_rx_reg_mask32(AUDIO_CTS_UP_BOUND, ~(AUDIO_CTS_UP_BOUND_cts_up_bound_mask), AUDIO_CTS_UP_BOUND_cts_up_bound(cts_up), HDMI_RX_MAC);
	hdmi_rx_reg_mask32(AUDIO_CTS_LOW_BOUND, ~AUDIO_CTS_LOW_BOUND_cts_low_bound_mask, AUDIO_CTS_LOW_BOUND_cts_low_bound(cts_low), HDMI_RX_MAC);//CTS low boundary set 20000
	HDMI_PRINTF( "Audio_CTS_Bound H = %ld , L =%ld \n",cts_up,cts_low);
}

void Audio_N_Bound(int freq)
{
	int N_up, N_low;

	if (freq>190000)	 //192k
	{
		N_up = 46592 *9/8 ;//;
		N_low = 20480 *7/8 ;
	}
	else if (freq >170000) //   176.4
	{
		N_up = 71344 * 9/8 ; //990000;
		N_low = 17836 *7/8 ;
	}
	else if (freq >80000)//   96 k 88.2k
	{
		N_up = 35672 * 9/8	;//990000;
		N_low = 8918 *7/8 ;
	}
	else//32k	44.1k  48k
	{
		N_up = 17836 * 9/8	;//990000;
		N_low = 3072 *7/8 ;
	}

	hdmi_rx_reg_mask32(AUDIO_N_UP_BOUND, ~(AUDIO_N_UP_BOUND_n_up_bound_mask), AUDIO_N_UP_BOUND_n_up_bound(N_up), HDMI_RX_MAC);
	hdmi_rx_reg_mask32(AUDIO_N_LOW_BOUND, ~AUDIO_N_LOW_BOUND_n_low_bound_mask, AUDIO_N_LOW_BOUND_n_low_bound(N_low), HDMI_RX_MAC);//CTS low boundary set 20000
	HDMI_PRINTF( " freq = %d Audio_N_Bound H = %d , L =%d \n",freq,N_up,N_low);
}

#define FCVO_MIN	250
#define FCVO_MAX	500
HDMI_CONST VIDEO_DPLL_RATIO_T dpll_ratio[] = {
	{	15, 15, 	1, 1	},	// 24 bits
	{	12, 15, 	4, 5	},	// 30 bits
	{	10, 15, 	2, 3	},	// 36 bits
	{	15, 30, 	1, 2	},	// 48 bits
};
HDMI_bool Hdmi_VideoPLLSetting(int b, int cd, int Enable2X)
{
//	Fin =	Fxtal * 1024 / b
//	Target vco = ( Fin * m / n )	 , TagretVco_HB = 400 ,  TagretVco_LB=200
//	Fin * M / N / 2^o / 2 * s = Fout = Fin * [24/30, 24/36, 24/48] ,  [10bits, 12bits,16bits]
//	200 <  ( Fin * m / n )	< 400  -->	200 <	Fin * 2^o * s * [ 8/5 , 4/3, 1 ]   < 400
//	Scode_LB =	200 * 15 * b  / ( Fxtal *1024 * [24,20,15] )
//	Scode_HB =	400 * 15 * b  / ( Fxtal *1024 * [24,20,15] )
//	Smean = (Scode_LB +Scode_HB ) /2
//	M/N = [8/5 , 4/3 , 1 ] * S

	unsigned int large_ratio, Smean,Stest, m, n, o, fraction1, fraction2, fvco;
	unsigned long pixel_clockx1024;

	if (b <= 0)
		return FALSE;

	if (cd >= 4) cd = 0;

	hdmi_rx_reg_mask32(HDMI_VPLLCR1, ~(HDMI_VPLLCR1_dpll_pow_mask|HDMI_VPLLCR1_dpll_vcorstb_mask |HDMI_VPLLCR1_dpll_freeze_mask|HDMI_VPLLCR1_dpll_stoppsw_mask),
									HDMI_VPLLCR1_dpll_freeze_mask, HDMI_RX_MAC);// Disable PLL

	pixel_clockx1024 =((unsigned long)hdmi.b * 27 * dpll_ratio[cd].SM * 1024) / (dpll_ratio[cd].SN * 256);

	HDMI_PRINTF("pixel_clock = %ld\n", pixel_clockx1024);

	if ((pixel_clockx1024 < (160 * 1024) && Enable2X == 0)||(HDMI_VCR_get_csc_r(hdmi_rx_reg_read32(HDMI_VCR, HDMI_RX_MAC))==3)) {
		Enable2X = 1;// if pixel_clock is under 160MHz then enable 2X clock maybe for DI
	}

	// interlace must 2 x	, MHL not
#if HDMI2p0
	if(bHDMI_420_Space == COLOR_YUV420)
	{
		large_ratio = 2;
		hdmi_rx_reg_mask32(TMDS_CPS, ~TMDS_CPS_pll_div2_en_mask, TMDS_CPS_pll_div2_en(1), HDMI_RX_MAC);
	}
	else
#endif
	if ((Enable2X)&&(((hdmi.tx_timing.progressive) == 0)))
	{
		large_ratio = 2;
		hdmi_rx_reg_mask32(TMDS_CPS, ~TMDS_CPS_pll_div2_en_mask, TMDS_CPS_pll_div2_en(1), HDMI_RX_MAC);
	} else {
		large_ratio = 1;
		hdmi_rx_reg_mask32(TMDS_CPS, ~TMDS_CPS_pll_div2_en_mask, TMDS_CPS_pll_div2_en(0), HDMI_RX_MAC);
	}

#if HDMI2p0
	if(bHDMI_420_Space == COLOR_YUV420)
	{
		hdmi_rx_reg_mask32(TMDS_DPC_SET0, ~TMDS_DPC_SET0_dpc_bypass_dis_mask, TMDS_DPC_SET0_dpc_bypass_dis(1), HDMI_RX_MAC);
	}
	else
#endif
	if	((cd || Enable2X)&&(((hdmi.tx_timing.progressive) == 0)))
	{
		hdmi_rx_reg_mask32(TMDS_DPC_SET0, ~TMDS_DPC_SET0_dpc_bypass_dis_mask, TMDS_DPC_SET0_dpc_bypass_dis(1), HDMI_RX_MAC);
	} else {
		hdmi_rx_reg_mask32(TMDS_DPC_SET0, ~TMDS_DPC_SET0_dpc_bypass_dis_mask, TMDS_DPC_SET0_dpc_bypass_dis(0), HDMI_RX_MAC);
	}

	if (large_ratio != 1)
	{
		pixel_clockx1024 =(hdmi.b * 27 * dpll_ratio[cd].SM * 1024 * 2) / (dpll_ratio[cd].SN * 256);
		HDMI_PRINTF("large_ratio=%d, 2X pixel clock PLL = %ld\n", large_ratio, pixel_clockx1024);
	}

	if (pixel_clockx1024 == 0)
	{
		HDMI_PRINTF("pixel_clockx1024 is zero\n");
		return FALSE;
	}

	o = 1;
	Smean = 0;
	Stest = 1;
	while(pixel_clockx1024 <= (200*1024))
	{
		if (Smean == 0)
			Stest = 1;
		else
			Stest = Smean * 2;
		// 2---> 2^o
		if (((pixel_clockx1024*2)*Stest)>= (FCVO_MIN*1024))
			break;
		Smean ++;
	};

	if (pixel_clockx1024 > (200*1024))// >200MHz
	{
		o = 0;
		Smean = 0;
	}
	HDMI_PRINTF( "Smean =  %d\n", Smean);

	n = 0;
	do {
		n += dpll_ratio[cd].RatioN;
		m =((dpll_ratio[cd].RatioM * Stest * n * large_ratio)<<o) / dpll_ratio[cd].RatioN;
		HDMI_PRINTF( "%d %d\n", m, n);
	}while(n < 2);

	fvco = (hdmi.b * 27 * m) / (256 * n);

//	Icp x (N/M) x (27M/Fin) = 0.95uA
//	Icp x (N/M) x (27M/(27Mx1024/b) = 0.95
//	Icp x (N/M) x (b/1024) = 0.95
//	Icp x (N/M) x (b/1024) x 100 = 95
//	Icp = (Mx1024x95)/(Nxbx100)
	fraction1 = ((unsigned long)m *95 *4 *b /(n *256 *100)) ;	 // 2bit fractional part
	fraction2 = 0x00;
	HDMI_PRINTF("***************fraction1=%d\n",fraction1);

	if (fraction1 >=10)
	   fraction1 -= 10;

	if(fraction1 >= 40) 	   // 2bit fractional part
	{
	   fraction1 -= 40;
	   fraction2 |= 0x04;
	}

	if(fraction1 >= 20) 	   // 2bit fractional part
	{
	   fraction1 -= 20;
	   fraction2 |= 0x02;
	}

	if(fraction1 >= 10) 	   // 2bit fractional part
	{
	   fraction1 -= 10;
	   fraction2 |= 0x01;
	}
	HDMI_PRINTF("***************fraction2=%d\n",fraction2);
	//fraction2 |= 0x18;

	if (((m-2)==0))//mag2 PLL bug	m min must > 1 , or PLL will crash
	{
		n=n*2;
		m = m* 2;
	}

	HDMIRX_INFO("PLL Setting b(%d) TMDS(%dMHz) P(%u) 2X(%d) 6G_flag(%u)",hdmi.b,hdmi.b*27/256,hdmi.tx_timing.progressive,Enable2X,bHDMI_6G_flag);
	HDMI_PRINTF("***************TMDS=%d MHz\n",hdmi.b*27/256);
	HDMI_PRINTF("***************cd=%d\n",cd);
	HDMI_PRINTF("***************m=%d\n",m);
	HDMI_PRINTF("***************n=%d\n",n);
	HDMI_PRINTF("***************o=%d\n",o);
	HDMI_PRINTF("***************s=%d\n",Smean);
	//HDMI_PRINTF("***************fraction1=%d\n",fraction1);
	HDMI_PRINTF("***************fraction2=%d\n",fraction2);
	HDMI_PRINTF("***************pixel_clockx1024=%ld\n",pixel_clockx1024);
	HDMI_PRINTF("***************fvco=%d MHz\n",fvco);
	HDMI_PRINTF("***************larget=%d\n",large_ratio);

	hdmi_rx_reg_write32(HDMI_VPLLCR0, HDMI_VPLLCR0_dpll_m(m-2)|HDMI_VPLLCR0_dpll_o(o)|
									HDMI_VPLLCR0_dpll_n(n-2)|HDMI_VPLLCR0_dpll_rs(3)|
									HDMI_VPLLCR0_dpll_ip(fraction2),
									HDMI_RX_MAC);// | fraction2); //set video PLL parameter

	hdmi_rx_reg_write32(MN_SCLKG_DIVS, Smean, HDMI_RX_MAC); //set video PLL output divider
	//hdmiport_mask(HDMI_VPLLCR1_reg, (~(_BIT0 | _BIT1 | _BIT2)),_BIT1|_BIT3);	// Enable PLL
	hdmi_rx_reg_mask32(MN_SCLKG_CTRL, ~_BIT4, _BIT4, HDMI_RX_MAC); //video PLL double buffer load
	usleep_range(5, 6);
	//hdmiport_mask(HDMI_VPLLCR1_reg, (~(_BIT0|_BIT1 |_BIT2|_BIT3)),_BIT1|_BIT3);// Enable PLL
	//video PLL power enable
	hdmi_rx_reg_mask32(HDMI_VPLLCR1, ~(HDMI_VPLLCR1_dpll_ldo_pow_mask), HDMI_VPLLCR1_dpll_ldo_pow_mask, HDMI_RX_MAC);
	usleep_range(5, 6);
	hdmi_rx_reg_mask32(HDMI_VPLLCR1, ~(HDMI_VPLLCR1_dpll_pow_mask|HDMI_VPLLCR1_dpll_stoppsw_mask),
									HDMI_VPLLCR1_dpll_pow_mask|HDMI_VPLLCR1_dpll_stoppsw_mask, HDMI_RX_MAC); //20140922
	usleep_range(5, 6);
	hdmi_rx_reg_mask32(HDMI_VPLLCR1, ~HDMI_VPLLCR1_dpll_vcorstb_mask, HDMI_VPLLCR1_dpll_vcorstb_mask, HDMI_RX_MAC);
	usleep_range(10, 15);
	hdmi_rx_reg_mask32(HDMI_VPLLCR1, ~HDMI_VPLLCR1_dpll_freeze_mask, 0, HDMI_RX_MAC);
	HDMI_PRINTF("Hdmi_VideoPLLSetting d404 = %x\n", hdmi_rx_reg_read32(HDMI_VPLLCR1, HDMI_RX_MAC));

	return TRUE;
}

unsigned char Hdmi_AudioPLLSetting(int freq, HDMI_AUDIO_TRACK_MODE track_mode)
{
	u_int8_t coeff = 0;
	u_int8_t i;
	int timeout = 10;
	u_int32_t tmp1;
	u_int32_t I_Code=0,S=0;

	for (i=0; i < 7; i++) {
		if (audio_pll_coeff[i].freq == freq)  {
			coeff = audio_pll_coeff[i].coeff;
			/*rate = audio_pll_coeff[i].rate;*/
			goto PLL_SETTING;
		}
	}
	HDMI_PRINTF( "Unsupport audio freq = %d\n", freq);
	return FALSE;

	PLL_SETTING:

	/*(2)audio output enable = auto mode*/
	hdmi_rx_reg_mask32(HDMI_AFCR, ~_BIT6 ,_BIT6, HDMI_RX_MAC);
	/*(3)Disable trend and boundary tracking*/
	hdmi_rx_reg_mask32(HDMI_WDCR0, ~HDMI_WDCR0_bt_track_en_mask, 0x0, HDMI_RX_MAC);//<2>Disable trend and boundary tracking
	hdmi_rx_reg_mask32(HDMI_PSCR, ~(_BIT3 |_BIT2), 0x00, HDMI_RX_MAC);//<1>Disable trend and boundary tracking
	hdmi_rx_reg_mask32(HDMI_CMCR, ~_BIT4 , _BIT4, HDMI_RX_MAC);//Update Double Buffer
	/*(4)FSM back to Pre-mode*/
	hdmi_rx_reg_mask32(HDMI_AVMCR, ~_BIT5, 0x00, HDMI_RX_MAC);
	/*(5)Disable N/CTS tracking*/
	hdmi_rx_reg_mask32(AUDIO_CTS_UP_BOUND, ~_BIT20, 0x0, HDMI_RX_MAC);//CTS has glitch not to tracking disable
	hdmi_rx_reg_mask32(AUDIO_N_UP_BOUND, ~_BIT20, 0x0, HDMI_RX_MAC);//N has glitch not to tracking disable
	hdmi_rx_reg_mask32(HDMI_PSCR, ~_BIT4, 0x00, HDMI_RX_MAC);
	hdmi_rx_reg_mask32(HDMI_CMCR, ~_BIT4, _BIT4, HDMI_RX_MAC);//Update Double Buffer
	/*(6)Disable SDM*/
	hdmi_rx_reg_mask32(HDMI_AAPNR, ~_BIT1, 0x0, HDMI_RX_MAC);
	/*(7)Disable PLL*/
	//hdmiport_mask(HDMI_APLLCR1_reg,  ~( _BIT2 | _BIT0| _BIT13| _BIT12| _BIT11), (_BIT2 | _BIT0)); // Disable PLL
	// cloud modify magellan remove bit 16 ~bit10  , need check
	hdmi_rx_reg_mask32(HDMI_APLLCR1, ~(HDMI_APLLCR1_dpll_freeze_mask|HDMI_APLLCR1_dpll_vcorstb_mask|HDMI_APLLCR1_dpll_pow_mask),
									(HDMI_APLLCR1_dpll_freeze_mask ), HDMI_RX_MAC);// Disable PLL //20140922
	/*(8)resetS &S1 code to avoid dead lock*/
	hdmi_rx_reg_mask32(HDMI_CMCR, ~( _BIT6|_BIT5), 0x0, HDMI_RX_MAC); // PLL output clk sel from crystal
	hdmi_rx_reg_write32(HDMI_SCAPR, 0x00, HDMI_RX_MAC); //S1 & S2 code clear to 0 , to avoid dead lock
	hdmi_rx_reg_mask32(HDMI_CMCR, ~_BIT4, _BIT4, HDMI_RX_MAC);//Update Double Buffer
	HDMI_DELAYMS(1);
	hdmi_rx_reg_mask32(HDMI_CMCR,  ~( _BIT6|_BIT5), (_BIT6), HDMI_RX_MAC);// PLL output clk sel from VCO
	hdmi_rx_reg_write32(HDMI_CMCR, 0x50,HDMI_RX_MAC);//Enable Double Buffer
	/*(9)D code*/
	hdmi_rx_reg_write32(HDMI_DCAPR0, (hdmi_audiopll_param[i].D_HighByte << 8) | hdmi_audiopll_param[i].D_LowByte, HDMI_RX_MAC);
	hdmi_rx_reg_mask32(HDMI_CMCR, ~(HDMI_CMCR_dbdcb_mask), HDMI_CMCR_dbdcb(1), HDMI_RX_MAC);//Enable Double Buffer for K/M/S/D/O

	/*(10)Initial PLL*/
	hdmi_rx_reg_mask32(HDMI_APLLCR0, ~(HDMI_APLLCR0_dpll_m_mask |HDMI_APLLCR0_dpll_o_mask |HDMI_APLLCR0_dpll_n_mask),
									HDMI_APLLCR0_dpll_m(hdmi_audiopll_param[i].M - 2) | HDMI_APLLCR0_dpll_o(hdmi_audiopll_param[i].O), HDMI_RX_MAC);
	hdmi_rx_reg_mask32(HDMI_CMCR, ~(HDMI_CMCR_dbdcb_mask), HDMI_CMCR_dbdcb(1), HDMI_RX_MAC);//Enable Double Buffer for K/M/S/D/O

	if (hdmi_audiopll_param[i].N < 2) {
		hdmi_rx_reg_mask32(HDMI_APLLCR0, ~(HDMI_APLLCR0_dpll_bpsin_mask | HDMI_APLLCR0_dpll_n_mask),
										HDMI_APLLCR0_dpll_bpsin(1) | HDMI_APLLCR0_dpll_n(0), HDMI_RX_MAC);//set audio PLL N code
	} else {
		hdmi_rx_reg_mask32(HDMI_APLLCR0, ~(HDMI_APLLCR0_dpll_bpsin_mask | HDMI_APLLCR0_dpll_n_mask),
										HDMI_APLLCR0_dpll_bpsin(0) | HDMI_APLLCR0_dpll_n(hdmi_audiopll_param[i].N-2), HDMI_RX_MAC);//set audio PLL N code
	}
	hdmi_rx_reg_mask32(HDMI_CMCR, ~(HDMI_CMCR_dbdcb_mask), HDMI_CMCR_dbdcb(1), HDMI_RX_MAC);//Enable Double Buffer for K/M/S/D/O
	hdmi_rx_reg_write32(HDMI_SCAPR,  (hdmi_audiopll_param[i].S1) ? ((hdmi_audiopll_param[i].S / 2) | 0x80) : (hdmi_audiopll_param[i].S / 2), HDMI_RX_MAC);
	hdmi_rx_reg_write32(PRESET_S_CODE1, 0xf800, HDMI_RX_MAC);//S1 code
	hdmi_rx_reg_mask32(HDMI_APLLCR0,~(_BIT2|_BIT1|_BIT0), 0x3, HDMI_RX_MAC);//set Icp
	hdmi_rx_reg_mask32(HDMI_CMCR, ~_BIT4, _BIT4, HDMI_RX_MAC);//Enable Double Buffer for K/M/S/D/O
	hdmi_rx_reg_mask32(HDMI_APLLCR0, ~(_BIT5|_BIT4|_BIT3), (_BIT4| _BIT3), HDMI_RX_MAC);//set RS=13k
	hdmi_rx_reg_mask32(HDMI_CMCR, ~_BIT4, _BIT4, HDMI_RX_MAC);//Enable Double Buffer for K/M/S/D/O

	//hdmiport_mask(HDMI_APLLCR1_reg,~(_BIT18|_BIT17),(_BIT18|_BIT17) );//set CS=42pf
	hdmi_rx_reg_mask32(HDMI_APLLCR1, ~(HDMI_APLLCR1_dpll_CS_MASK), (HDMI_APLLCR1_dpll_CS_35P), HDMI_RX_MAC);//set CS=42pf
	//hdmiport_mask(HDMI_HDMI_APLLCR1_ADDR,~_BIT1,_BIT1);// Enable divider K and enable VCOSTART
	hdmi_rx_reg_mask32(HDMI_CMCR, ~_BIT4, _BIT4, HDMI_RX_MAC);//Enable Double Buffer for K/M/S/D/O
	//hdmiport_mask(HDMI_APLLCR1_reg,~(_BIT3 | _BIT1 |_BIT0), (_BIT3 | _BIT1));//Enable PLL
	hdmi_rx_reg_mask32(HDMI_APLLCR1, ~(HDMI_APLLCR1_dpll_stoppsw_mask|HDMI_APLLCR1_dpll_pow_mask),
									(HDMI_APLLCR1_dpll_pow_mask|HDMI_APLLCR1_dpll_stoppsw_mask), HDMI_RX_MAC);//Enable PLL //20140922
	hdmi_rx_reg_mask32(HDMI_APLLCR1, ~(HDMI_APLLCR1_dpll_vcorstb_mask), (HDMI_APLLCR1_dpll_vcorstb_mask), HDMI_RX_MAC);//RST //20140922

	HDMI_PRINTF(" m = %x\n o = %x\n s = %x\n ", hdmi_audiopll_param[i].M, hdmi_audiopll_param[i].O, hdmi_audiopll_param[i].S);
	//Wait PLL Stable
	HDMI_DELAYMS(1);
	//PLL un-freeze
	//hdmiport_mask(HDMI_APLLCR1_reg,~_BIT2, 0x0); //DPLL normal->clk really output to fifo for read
	hdmi_rx_reg_mask32(HDMI_APLLCR1, ~HDMI_APLLCR1_dpll_freeze_mask, 0x0, HDMI_RX_MAC);//
	/*(11)Enable SDM*/
	//hdmiport_mask(HDMI_AAPNR_reg, ~_BIT1,_BIT1);
	hdmi_rx_reg_mask32(HDMI_AAPNR, ~HDMI_AAPNR_esdm_mask, HDMI_AAPNR_esdm_mask, HDMI_RX_MAC);
	HDMI_DELAYMS(1);

	if(track_mode == HDMI_AUDIO_N_CTS_TREND_BOUND)
	{
		HDMI_PRINTF("\n *****N/CTS Trend& Boundary Tracking*****\n");
		/*(12)Enable N/CTS tracking*/
		/*Modify N/CTS tracking parameter  USER:kistlin DATE:2011/08/04*/
		//for phase error count source Fpec = Fdds = Fvco/4
		//PEpec x Tpec = delta(Tvco)xNxSxPLLO = Tvco(1/8)(D[15:0]/2^16)xNxSxPLLO
		//D[15:0] = PEpec x Tpec /[Tvco(1/8)(1//2^16)xNxSxPLLO]
		//and D[15:0] = PEpec x (1/8)Icode
		//Icode calculate I code =2^24/(N*S*PLLO)

		//for phase error count source = fvco/4,fdds
		//Icode calculate I code =2*2048*2^10/(N*S*PLLO)-->x2 �~���O 20110701 kist
		if (hdmi_audiopll_param[i].S1)
			S = hdmi_audiopll_param[i].S*2;
		else
			S = hdmi_audiopll_param[i].S;
		HDMI_PRINTF("S = %d , ACR_N=%d ,	hdmi_audiopll_param[i].O = %d\n",S, ACR_N, hdmi_audiopll_param[i].O );
		if (ACR_N)
		{
			I_Code =16*1024*1024/(ACR_N*S*(hdmi_audiopll_param[i].O<<1));
			I_Code = I_Code;
		}
		else
			I_Code = 0x02;

		//calculate 4*N*(1/128fa) or 4*CTS*Tv, 4x for delay (HDMI_DELAYMS(1) ����300us)
		tmp1 = 4*ACR_N*1000/(128*freq);
		if (tmp1 < 5)
			tmp1 = tmp1;//CTS*Tv < tmp1/4 > 2* CTS*Tv,	CTS*Tv(0.67ms~3.3ms)
		else if (tmp1 < 9)
			tmp1 = tmp1+2;
		else
			tmp1 = tmp1+3;

		//HDMI_PRINTF( "I Code = %d\n",I_Code);
		//HDMI_PRINTF( "tmp1 = %d\n",tmp1);

		hdmi_rx_reg_write32(HDMI_ICPSNCR0, I_Code, HDMI_RX_MAC);//Set I code of Ncts[15:8]
		hdmi_rx_reg_write32(HDMI_PCPSNCR0, 0x0000, HDMI_RX_MAC);//Set P code of Ncts [15:8]
		//	hdmiport_mask(HDMI_NPECR_reg,~_BIT30,_BIT30);	//N_CTS tracking re-enable toggle function enable
		hdmi_rx_reg_mask32(HDMI_NPECR, ~HDMI_NPECR_ncts_re_enable_off_en_mask, HDMI_NPECR_ncts_re_enable_off_en_mask, HDMI_RX_MAC);
		hdmi_rx_reg_write32(HDMI_PSCR, 0x92, HDMI_RX_MAC);//Enable N_CTS tracking & set FIFO depth
		hdmi_rx_reg_mask32(HDMI_CMCR,~_BIT4, _BIT4, HDMI_RX_MAC);//Update Double Buffer

		hdmi_rx_reg_write32(HDMI_PETR, 0x1e, HDMI_RX_MAC);//phase error threshold
		for (timeout = 0; timeout < 25; timeout++) {
			hdmi_rx_reg_mask32(HDMI_SR, ~_BIT3, _BIT3, HDMI_RX_MAC);
			hdmi_rx_reg_write32(HDMI_NCPER, 0xff, HDMI_RX_MAC);
			HDMI_DELAYMS(20);
			if ((hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC) & (_BIT3)) == 0)
				break;
		}

		if (timeout == 25)
			HDMI_PRINTF( "PLL 1st check not lock = %x\n",hdmi_rx_reg_read32(HDMI_NCPER, HDMI_RX_MAC));
		else
			HDMI_PRINTF( "PLL 1st check lock count = %d\n",timeout);

		//hdmiport_mask(HDMI_PSCR_reg,~_BIT4,0);//disable N_CTS tracking
		hdmi_rx_reg_mask32(HDMI_PSCR, ~HDMI_PSCR_etcn_mask, 0, HDMI_RX_MAC);
		//hdmiport_mask(HDMI_CMCR_reg,~_BIT4 , _BIT4);//Update Double Buffer
		hdmi_rx_reg_mask32(HDMI_CMCR, ~HDMI_CMCR_dbdcb_mask, HDMI_CMCR_dbdcb_mask, HDMI_RX_MAC);//Update Double Buffer
		//hdmiport_mask(HDMI_NPECR_reg,~_BIT30,0);	//N_CTS tracking re-enable toggle function disable
		hdmi_rx_reg_mask32(HDMI_NPECR, ~HDMI_NPECR_ncts_re_enable_off_en_mask, 0, HDMI_RX_MAC);
		//hdmi_out(HDMI_ICPSNCR0_reg,0x0002); //Set I code of Ncts[15:8]
		hdmi_rx_reg_write32(HDMI_ICPSNCR0, (HDMI_ICPSNCR0_icl(0x02)|HDMI_ICPSNCR0_ich(0)), HDMI_RX_MAC);//Set I code of Ncts[15:8]
		//hdmi_out(HDMI_PCPSNCR0_reg,0x2000);//Set P code of Ncts [15:8]
		hdmi_rx_reg_write32(HDMI_PCPSNCR0, (HDMI_PCPSNCR0_pcl(0x0)|HDMI_PCPSNCR0_pch(0x20)), HDMI_RX_MAC);//Set P code of Ncts [15:8]
		//hdmiport_mask(HDMI_PSCR_reg,~_BIT4,_BIT4);//enable N_CTS tracking
		hdmi_rx_reg_mask32(HDMI_PSCR, ~HDMI_PSCR_etcn_mask, HDMI_PSCR_etcn_mask, HDMI_RX_MAC);
		//hdmiport_mask(HDMI_CMCR_reg,~_BIT4 , _BIT4);//Update Double Buffer
		hdmi_rx_reg_mask32(HDMI_CMCR, ~HDMI_CMCR_dbdcb_mask, HDMI_CMCR_dbdcb_mask, HDMI_RX_MAC);
		//N&CTS boundary set
		//hdmiport_mask(AUDIO_CTS_UP_BOUND_reg,~0xfffff,0x6ddd0);//CTS up boundary set 450000
		//hdmiport_mask(HDMI_AUDIO_CTS_UP_BOUND_reg,~(HDMI_AUDIO_CTS_UP_BOUND_cts_up_bound_mask),HDMI_AUDIO_CTS_UP_BOUND_cts_up_bound(450000)); 	//CTS up boundary set 450000
		//hdmiport_mask(HDMI_AUDIO_CTS_LOW_BOUND_reg,~0xfffff,0x4e20);//CTS low boundary set 20000
		Audio_CTS_Bound();
		//hdmiport_mask(HDMI_AUDIO_N_UP_BOUND_reg,~0xfffff,0x13880);//N up boundary set 80000
		//hdmiport_mask(HDMI_AUDIO_N_LOW_BOUND_reg,~0xfffff,0x7d0);//N low boundary set 2000

		Audio_N_Bound(freq);
		hdmi_rx_reg_mask32(AUDIO_CTS_UP_BOUND, ~_BIT20, _BIT20, HDMI_RX_MAC);//CTS has glitch not to tracking enable
		hdmi_rx_reg_mask32(AUDIO_N_UP_BOUND, ~_BIT20, _BIT20, HDMI_RX_MAC);//N has glitch not to tracking enable

		/*(13)Wait PLL lock by N&CTS tracking*/
		hdmi_rx_reg_write32(HDMI_PETR, 0x1e, HDMI_RX_MAC);//phase error threshold
		for (timeout = 0; timeout < 25; timeout++) {
			hdmi_rx_reg_mask32(HDMI_SR, ~_BIT3, _BIT3, HDMI_RX_MAC);
			hdmi_rx_reg_write32(HDMI_NCPER, 0xff, HDMI_RX_MAC);
			HDMI_DELAYMS(20);
			if ((hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC) & (_BIT3)) == 0)
				break;
		}

		if (timeout == 25)
			HDMI_PRINTF( "PLL not lock = %x\n",hdmi_rx_reg_read32(HDMI_NCPER, HDMI_RX_MAC));
		else
			HDMI_PRINTF( "PLL lock count = %d\n",timeout);

		/*(14)FSM Initial*/
		//hdmiport_out(HDMI_HDMI_FBR_VADDR,0x77);//Target FIFO depth = 14 ,Boundary address distance = 7
		hdmi_rx_reg_write32(HDMI_FBR,0x74, HDMI_RX_MAC);
		//hdmiport_out(HDMI_HDMI_FTR_VADDR,0x03);//target times for summation of one trend to decide the trend
		hdmi_rx_reg_mask32(HDMI_AVMCR, ~_BIT6, _BIT6, HDMI_RX_MAC);//FSM entry Pre-mode (AOC=1)
		hdmi_rx_reg_mask32(HDMI_AVMCR, ~_BIT5, _BIT5, HDMI_RX_MAC);//FSM entry next step (AOMC=1)
		HDMI_DELAYMS(1);//wait fifo to target fifo level

		/*(15)Enable trend and boundary tracking*/
		hdmi_rx_reg_write32(HDMI_ICTPSR0, 0x0005, HDMI_RX_MAC);//Set I code  of trend [15:8]
		hdmi_rx_reg_write32(HDMI_PCTPSR0, 0x02FF, HDMI_RX_MAC);//Set P code of trend [15:8]
		hdmi_rx_reg_write32(HDMI_ICBPSR0, 0x0005, HDMI_RX_MAC);//Set I code of bnd [15:8]
		hdmi_rx_reg_write32(HDMI_PCBPSR0, 0x02FF, HDMI_RX_MAC);//Set P code of bnd [15:8]
		hdmi_rx_reg_write32(HDMI_STBPR, 0x01, HDMI_RX_MAC);//Set Boundary Tracking Update Response Time
		hdmi_rx_reg_mask32(HDMI_PSCR , ~(_BIT3|_BIT2), (_BIT3|_BIT2), HDMI_RX_MAC);//<1>Enable trend and boundary tracking
		hdmi_rx_reg_mask32(HDMI_CMCR,~_BIT4 , _BIT4, HDMI_RX_MAC);//Update Double Buffer
		HDMI_DELAYMS(20);
		hdmi_rx_reg_mask32(HDMI_WDCR0, ~HDMI_WDCR0_bt_track_en_mask, HDMI_WDCR0_bt_track_en_mask, HDMI_RX_MAC);//<2>Enable trend and boundary tracking
	}
	else if(track_mode == HDMI_AUDIO_TREND_BOUND)
	{
		HDMI_PRINTF("\n ***** TREND_BOUND Tracking*****\n");
		/*(14)FSM Initial*/
		//hdmiport_out(HDMI_HDMI_FBR_VADDR,0x77);//Target FIFO depth = 14 ,Boundary address distance = 7
		hdmi_rx_reg_write32(HDMI_FBR, 0x74, HDMI_RX_MAC);
		//hdmiport_out(HDMI_HDMI_FTR_VADDR,0x03);//target times for summation of one trend to decide the trend
		hdmi_rx_reg_mask32(HDMI_AVMCR, ~_BIT6, _BIT6, HDMI_RX_MAC);//FSM entry Pre-mode (AOC=1)
		hdmi_rx_reg_mask32(HDMI_AVMCR, ~_BIT5, _BIT5, HDMI_RX_MAC);//FSM entry next step (AOMC=1)
		HDMI_DELAYMS(1);//wait fifo to target fifo level

		/*(15)Enable trend and boundary tracking*/
		hdmi_rx_reg_write32(HDMI_ICTPSR0, 0x0005, HDMI_RX_MAC);//Set I code  of trend [15:8]
		hdmi_rx_reg_write32(HDMI_PCTPSR0, 0x02FF, HDMI_RX_MAC);//Set P code of trend [15:8]
		hdmi_rx_reg_write32(HDMI_ICBPSR0, 0x0001, HDMI_RX_MAC);//Set I code of bnd [15:8]
		hdmi_rx_reg_write32(HDMI_PCBPSR0, 0x02FF, HDMI_RX_MAC);//Set P code of bnd [15:8]
		hdmi_rx_reg_write32(HDMI_STBPR, 0x01, HDMI_RX_MAC);//Set Boundary Tracking Update Response Time
		hdmi_rx_reg_write32(HDMI_PSCR, (HDMI_PSCR_fdint(4)|HDMI_PSCR_etcn(0)|HDMI_PSCR_etfd(1)|HDMI_PSCR_etfbc(1)|HDMI_PSCR_pecs(2)), HDMI_RX_MAC);// FIFO depth tracking
		hdmi_rx_reg_mask32(HDMI_PSCR , ~(_BIT3|_BIT2), (_BIT3|_BIT2), HDMI_RX_MAC);//<1>Enable trend and boundary tracking
		hdmi_rx_reg_mask32(HDMI_CMCR, ~_BIT4 , _BIT4, HDMI_RX_MAC);//Update Double Buffer
		HDMI_DELAYMS(20);
		hdmi_rx_reg_mask32(HDMI_WDCR0, ~HDMI_WDCR0_bt_track_en_mask, HDMI_WDCR0_bt_track_en_mask, HDMI_RX_MAC);//<2>Enable trend and boundary tracking

	}else{//  H/W N/CTS Tracking
		HDMI_PRINTF("\n ***** N/CTS Tracking*****\n");
		/*(12)Enable N/CTS tracking*/
		if (hdmi_audiopll_param[i].S1)
			S = hdmi_audiopll_param[i].S*2;
		else
			S = hdmi_audiopll_param[i].S;
		HDMI_PRINTF("S = %d , ACR_N=%d ,	hdmi_audiopll_param[i].O = %d\n",S, ACR_N, hdmi_audiopll_param[i].O );
		if (ACR_N)
		{
			I_Code =16*1024*1024/(ACR_N*S*(hdmi_audiopll_param[i].O<<1));
			I_Code = I_Code;
		}
		else
			I_Code = 0x02;

		//calculate 4*N*(1/128fa) or 4*CTS*Tv, 4x for delay (HDMI_DELAYMS(1) ����300us)
		tmp1 = 4*ACR_N*1000/(128*freq);
		if (tmp1 < 5)
			tmp1 = tmp1;//CTS*Tv < tmp1/4 > 2* CTS*Tv,	CTS*Tv(0.67ms~3.3ms)
		else if (tmp1 < 9)
			tmp1 = tmp1+2;
		else
			tmp1 = tmp1+3;

		//HDMI_PRINTF( "I Code = %d\n",I_Code);
		//HDMI_PRINTF( "tmp1 = %d\n",tmp1);

		hdmi_rx_reg_write32(HDMI_ICPSNCR0, I_Code, HDMI_RX_MAC);//Set I code of Ncts[15:8]
		hdmi_rx_reg_write32(HDMI_PCPSNCR0, 0x0000, HDMI_RX_MAC);	//Set P code of Ncts [15:8]
		//hdmiport_mask(HDMI_NPECR_reg,~_BIT30,_BIT30); //N_CTS tracking re-enable toggle function enable
		hdmi_rx_reg_mask32(HDMI_NPECR, ~HDMI_NPECR_ncts_re_enable_off_en_mask, HDMI_NPECR_ncts_re_enable_off_en_mask, HDMI_RX_MAC);
		hdmi_rx_reg_write32(HDMI_PSCR, 0x92, HDMI_RX_MAC);//Enable N_CTS tracking & set FIFO depth
		hdmi_rx_reg_mask32(HDMI_CMCR, ~_BIT4, _BIT4, HDMI_RX_MAC);//Update Double Buffer

		hdmi_rx_reg_write32(HDMI_PETR, 0x1e, HDMI_RX_MAC);//phase error threshold
		for (timeout = 0; timeout < 25; timeout++) {
			hdmi_rx_reg_mask32(HDMI_SR, ~_BIT3, _BIT3, HDMI_RX_MAC);
			hdmi_rx_reg_write32(HDMI_NCPER, 0xff, HDMI_RX_MAC);
			HDMI_DELAYMS(20);
			if ((hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC) & (_BIT3)) == 0)
				break;
		}

		if (timeout == 25)
			HDMI_PRINTF( "PLL 1st check not lock = %x\n",hdmi_rx_reg_read32(HDMI_NCPER, HDMI_RX_MAC));
		else
			HDMI_PRINTF( "PLL 1st check lock count = %d\n",timeout);


		hdmi_rx_reg_mask32(HDMI_PSCR, ~_BIT4, 0, HDMI_RX_MAC);//disable N_CTS tracking
		hdmi_rx_reg_mask32(HDMI_CMCR, ~_BIT4 , _BIT4, HDMI_RX_MAC);//Update Double Buffer
		//hdmiport_mask(HDMI_NPECR_reg,~_BIT30,0);	//N_CTS tracking re-enable toggle function disable
		hdmi_rx_reg_mask32(HDMI_NPECR, ~HDMI_NPECR_ncts_re_enable_off_en_mask, 0, HDMI_RX_MAC);//N_CTS tracking re-enable toggle function disable
		hdmi_rx_reg_write32(HDMI_ICPSNCR0, 0x0002, HDMI_RX_MAC);//Set I code of Ncts[15:8]
		hdmi_rx_reg_write32(HDMI_PCPSNCR0, 0x2000, HDMI_RX_MAC);//Set P code of Ncts [15:8]
		hdmi_rx_reg_mask32(HDMI_PSCR, ~_BIT4, _BIT4, HDMI_RX_MAC);//enable N_CTS tracking
		hdmi_rx_reg_mask32(HDMI_CMCR, ~_BIT4, _BIT4, HDMI_RX_MAC);//Update Double Buffer

		//N&CTS boundary set
		//hdmiport_mask(HDMI_AUDIO_CTS_UP_BOUND_reg,~0xfffff,0x6ddd0);		//CTS up boundary set 450000
		//hdmiport_mask(HDMI_AUDIO_CTS_LOW_BOUND_reg,~0xfffff,0x4e20);		//CTS low boundary set 20000
		Audio_CTS_Bound();
		//hdmiport_mask(HDMI_AUDIO_N_UP_BOUND_reg,~0xfffff,0x13880);		//N up boundary set 80000
		//hdmiport_mask(HDMI_AUDIO_N_LOW_BOUND_reg,~0xfffff,0x7d0); 	//N low boundary set 2000
		Audio_N_Bound(freq);
		hdmi_rx_reg_mask32(AUDIO_CTS_UP_BOUND, ~_BIT20, _BIT20, HDMI_RX_MAC);//CTS has glitch not to tracking enable
		hdmi_rx_reg_mask32(AUDIO_N_UP_BOUND, ~_BIT20, _BIT20, HDMI_RX_MAC);//N has glitch not to tracking enable

		/*(13)Wait PLL lock by N&CTS tracking*/
		hdmi_rx_reg_write32(HDMI_PETR, 0x1e, HDMI_RX_MAC);//phase error threshold
		for (timeout = 0; timeout < 25; timeout++) {
			hdmi_rx_reg_mask32(HDMI_SR,~_BIT3,_BIT3, HDMI_RX_MAC);
			hdmi_rx_reg_write32(HDMI_NCPER, 0xff, HDMI_RX_MAC);
			HDMI_DELAYMS(20);
			if ((hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC) & (_BIT3)) == 0)
				break;
		}

		if (timeout == 25)
			HDMI_PRINTF( "PLL not lock = %x\n",hdmi_rx_reg_read32(HDMI_NCPER, HDMI_RX_MAC));
		else
			HDMI_PRINTF( "PLL lock count = %d\n",timeout);

		/*(14)FSM Initial*/
		hdmi_rx_reg_write32(HDMI_FBR, 0x77, HDMI_RX_MAC);//Target FIFO depth = 14 ,Boundary address distance = 7
		hdmi_rx_reg_mask32(HDMI_AVMCR, ~_BIT6, _BIT6, HDMI_RX_MAC);//FSM entry Pre-mode (AOC=1)
		hdmi_rx_reg_mask32(HDMI_AVMCR, ~_BIT5, _BIT5, HDMI_RX_MAC);//FSM entry next step (AOMC=1)
		HDMI_DELAYMS(1);//wait fifo to target fifo level
	}

	/*(16)Wait FIFO stable*/
	for (timeout = 0; timeout < 5; timeout++) {
		hdmi_rx_reg_write32(HDMI_NCPER, 0xff, HDMI_RX_MAC);
		HDMI_DELAYMS(20);
		if ((hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC) & (_BIT3|_BIT2|_BIT1))==0)
			break;
		hdmi_rx_reg_mask32(HDMI_SR,~(_BIT3|_BIT2|_BIT1),(_BIT3|_BIT2|_BIT1), HDMI_RX_MAC);
	}

	if (timeout == 5)
		HDMI_PRINTF( "FIFO Unstable  = %x \n",hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC));
	else
		HDMI_PRINTF( "FIFO timeout count = %d\n",timeout);

	HDMI_PRINTF( "HDMI_HDMI_AVMCR_reg = %x\n", hdmi_rx_reg_read32(HDMI_AVMCR, HDMI_RX_MAC));

	return TRUE;

}

void Hdmi_AudioOutputDisable(void)
{
	hdmi_rx_reg_mask32(HDMI_AVMCR, ~(_BIT6 | _BIT5), _BIT6, HDMI_RX_MAC);
	hdmi_rx_reg_write32(HDMI_CMCR, 0x50, HDMI_RX_MAC);	 //K code =2
}

void Hdmi_AudioOutputEnable(void)
{
	hdmi_rx_reg_mask32(HDMI_AVMCR, ~(_BIT6|_BIT5), (_BIT6|_BIT5), HDMI_RX_MAC);
}

unsigned char Hdmi_WaitAudioSample(void)
{
	char   timeout_cnt = 6;
	/*
		looking for ACR info using RSV2
	*/
	hdmi_rx_reg_mask32(HDMI_PTRSV1, ~0x00FF00, 0x02<< 8, HDMI_RX_MAC);// Wait for ACR : Packet Type = 0x01
	hdmi_rx_reg_write32(HDMI_GPVS, _BIT6, HDMI_RX_MAC);// Clear RSV2 indicator

	for (;timeout_cnt>0;timeout_cnt--) { // Wait 30ms max. to wait ACR
		HDMI_DELAYMS(10);
		if (hdmi_rx_reg_read32(HDMI_GPVS, HDMI_RX_MAC) & _BIT6) {
			hdmi_rx_reg_write32(HDMI_PSAP, 110, HDMI_RX_MAC);

			if (hdmi_rx_reg_read32(HDMI_PSDP, HDMI_RX_MAC) & 0x10) {
				SET_HDMI_AUDIO_LAYOUT(1);
			} else {
				SET_HDMI_AUDIO_LAYOUT(0);
			}
//			HDMI_PRINTF("Layout = %d\n", GET_HDMI_AUDIO_LAYOUT());
			hdmi_rx_reg_write32(HIGH_BIT_RATE_AUDIO_PACKET, 0x04, HDMI_RX_MAC);

			return TRUE;
		}
	}

	HDMI_PRINTF("Audio Sample miss\n");
	return FALSE;
}

int Hdmi_AudioFreqCorrect(unsigned int freq, unsigned long b, HDMI_AUDIO_TRACK_MODE *track_mode)
{
	/*
		TO-DO : use ABS function
	*/
	unsigned int b_ratio=1000;

	*track_mode = HDMI_AUDIO_N_CTS_TREND_BOUND;

	freq *= 10;
	if((freq >= (31700*b_ratio/100)) && (freq <= (32300*b_ratio/100)))
			freq = 32000;
	else if((freq >= (43500*b_ratio/100)) && (freq <= (44600*b_ratio/100)))
			freq = 44100;
	else if((freq >= 47500*b_ratio/100) && (freq <= (48500*b_ratio/100)))
			freq = 48000;
	else if((freq >= (87700*b_ratio/100)) && (freq <= (88700*b_ratio/100)))
			freq = 88200;
	else if((freq >= (95500*b_ratio/100)) && (freq <= (96500*b_ratio/100)))
			freq = 96000;
	else if((freq >= (175400*b_ratio/100)) && (freq <= (177400*b_ratio/100)))
			freq = 176400;
	else if((freq >= (191000*b_ratio/100)) && (freq <= (193000*b_ratio/100)))
			freq = 192000;
	else
		 freq = 0;

	return freq;
}

void Hdmi_GetAudioFreq(HDMI_AUDIO_FREQ_T *freq, HDMI_AUDIO_TRACK_MODE *track_mode)
{
	unsigned long cts, n , b;
	unsigned char count=0;
	HDMIRX_IOCTL_STRUCT_T isr_info;

	freq->ACR_freq = 0;
	freq->AudioInfo_freq = 0;
	freq->SPDIF_freq = 0;

	/*
		Set trigger to get CTS&N and LPCM Channel Status Info
	*/

	//Start Pop up N_CTS value
	hdmi_rx_reg_mask32(HDMI_ACRCR, ~(_BIT1|_BIT0), _BIT1|_BIT0, HDMI_RX_MAC);

	// Restart measure b
	hdmi_rx_reg_mask32(HDMI_NTX1024TR0, ~_BIT3, _BIT3, HDMI_RX_MAC);

	// Clear Info Frame update indicator
	hdmi_rx_reg_write32(HDMI_ASR0, 0x07, HDMI_RX_MAC);

	for (count = 0; count < 15; count++) {
		if (((hdmi_rx_reg_read32(HDMI_ACRCR, HDMI_RX_MAC) & (_BIT1|_BIT0)) == 0) && ((hdmi_rx_reg_read32(HDMI_ASR0, HDMI_RX_MAC) & 0x01) == 0x01))
			break;
		HDMI_DELAYMS(10);
	}
	if (count >=15)
	{
		HDMI_PRINTF("POP UP TIME OUT %x  %x \n",hdmi_rx_reg_read32(HDMI_ACRCR, HDMI_RX_MAC),hdmi_rx_reg_read32(HDMI_ASR0, HDMI_RX_MAC));
	}
	HDMI_PRINTF("POP UP TIME %d \n",count);

	/*
		Get Audio Frequency from CTS&N
	*/
	cts= HDMI_ACRSR0_get_cts(hdmi_rx_reg_read32(HDMI_ACRSR0, HDMI_RX_MAC));
	n =  HDMI_ACRSR1_get_n(hdmi_rx_reg_read32(HDMI_ACRSR1, HDMI_RX_MAC));

	ACR_N = n;
	HdmiGetStruct(&isr_info);
	b = isr_info.b;

	HDMI_PRINTF("cts=%ld\nn=%ld\nb=%ld\n",cts, n, b);

	if(cts==0 || n==0 || b == 0)
		goto METHOD_AUDIO_INFO;

	// 128fs = 1024/b * fx * N / CTS  =>  fs = (1024 * fx *N)/(128 * b * CTS) = (8 * fx *N)/(b*CTS)
	// calculate freq in 0.1kHz unit
	// freq = (unsigned long)8 * 2 * 10000 * HDMI_RTD_XTAL/ cts * n / ((unsigned long)b * 1000);

	freq->ACR_freq = ((((270000 * b)/256)/128) * n) / (cts);
	freq->ACR_freq *= 100;

	freq->ACR_freq = Hdmi_AudioFreqCorrect(freq->ACR_freq, b, track_mode);

	/*
		Get Audio Frequency from Audio Info Frame
	*/
	METHOD_AUDIO_INFO:

	if ((hdmi_rx_reg_read32(HDMI_ASR0, HDMI_RX_MAC) & 0x01) == 0x01)  {
		freq->SPDIF_freq = AUDIO_CHANNEL_STATUS[hdmi_rx_reg_read32(HDMI_ASR1, HDMI_RX_MAC)&0xf];
	}

	HDMI_PRINTF("\n *************** SPDIF freq=%ld\n", freq->SPDIF_freq);
}


HDMI_bool Hdmi_AudioModeDetect(void)
{
	unsigned char result = FALSE;
	unsigned int d_code;
	int i;
	HDMI_AUDIO_FREQ_T t, t2;
	HDMI_AUDIO_TRACK_MODE track_mode;
	static unsigned int spdif_freq;

	if (hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC) & _BIT6){
		HDMI_PRINTF("Audio Detect AV Mute\n");
		Hdmi_AudioOutputDisable();
		SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);
		return FALSE;
	}

	switch(GET_HDMI_AUDIO_FSM()) {
		case AUDIO_FSM_AUDIO_START:
		{
			HDMI_PRINTF("AUDIO_FSM_AUDIO_START\n");

			hdmi_ioctl_struct.audio_detect_done = 0;

			SET_HDMI_AUDIO_TYPE(HDMI_AUDIO_PCM);

			SET_HDMI_HBR_MODE(0);
			// clear Overflow,Underflow,phase_non_lock, auto load double buffter
			hdmi_rx_reg_write32(HDMI_DBCR , 0x00, HDMI_RX_MAC);
			/*Disable audio watch dog*/
			hdmi_rx_reg_mask32(HDMI_WDCR0, (~_BIT15), 0x00, HDMI_RX_MAC);	//disable  tmds clock audio watch dog
			hdmi_rx_reg_mask32(HDMI_WDCR0, ~(_BIT1 |_BIT2|_BIT3 | _BIT4 ),0x00, HDMI_RX_MAC);
			Hdmi_AudioOutputDisable();

			/*Disable FIFO trend tracking*/
			hdmi_rx_reg_write32(HDMI_PSCR, 0xE2, HDMI_RX_MAC);
			/*Update Double Buffer*/
			hdmi_rx_reg_write32(HDMI_CMCR, 0x50, HDMI_RX_MAC);//K code =2
// mag2 need check
//			hdmiport_mask(HDMI_BCHCR_reg, ~HDMI_BCHCR_spcss_mask, HDMI_BCHCR_spcss(1)); // 20150109 non-pcm's cs is unstable
			if(Hdmi_WaitAudioSample() == FALSE)
				break;

			HDMI_Audio_Conut = 0; //20141016 reset state if only pattern change without timing change
			SET_HDMI_AUDIO_FSM(AUDIO_FSM_FREQ_DETECT);
		}
//		break;
		case AUDIO_FSM_FREQ_DETECT:
		{
			HDMI_PRINTF("AUDIO_FSM_FREQ_DETECT\n");
			if (HDMI_AUDIO_IS_LPCM() == 0) {
				HDMI_PRINTF("HDMI NON-PCM Audio\n");
				SET_HDMI_AUDIO_TYPE(HDMI_AUDIO_NPCM);
			}
			if (HDMI_AUDIO_IS_LPCM() || HDMI_AUDIO_SUPPORT_NON_PCM()) {
				Hdmi_GetAudioFreq(&t, &track_mode);
				//detect HDMI audio freq twice for stable freq
				Hdmi_GetAudioFreq(&t2, &track_mode);
				HDMI_PRINTF("Hdmi_GetAudioFreq t=%ld\n",t.ACR_freq);
				HDMI_PRINTF("Hdmi_GetAudioFreq t2=%ld\n",t2.ACR_freq);
				if ((t.ACR_freq != 0 )&& (t.ACR_freq==t2.ACR_freq)) {
					if (HDMI_Audio_Conut == 0){
						HDMI_Audio_Conut = 1;
						if (Hdmi_AudioPLLSetting(t.ACR_freq, track_mode) == TRUE) {
							SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_WAIT_PLL_READY);
							spdif_freq = t.SPDIF_freq;
							break;
						}
					}
					else{//HDMI_Audio_Conut  = 1, force to trend_boundary tracking
						HDMI_Audio_Conut = 0;
						if (Hdmi_AudioPLLSetting(t.ACR_freq, HDMI_AUDIO_TREND_BOUND) == TRUE) {
							SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_WAIT_PLL_READY);
							spdif_freq = t.SPDIF_freq;
							break;
						}
					}
				} else {
					if ((t.ACR_freq == 0)||(t2.ACR_freq == 0)){//cts = 0,use t.SPDIF_freq and force to trend_boundary tracking
						if (Hdmi_AudioPLLSetting(t.SPDIF_freq, HDMI_AUDIO_TREND_BOUND) == TRUE) {
							if (t.SPDIF_freq == t2.SPDIF_freq){
								SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_WAIT_PLL_READY);
								spdif_freq = t.SPDIF_freq;
								break;
							}
						}
					}
				}
			}
			//SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);
			break;
		}
		case AUDIO_FSM_AUDIO_WAIT_PLL_READY:
			HDMI_PRINTF("AUDIO_FSM_AUDIO_WAIT_PLL_READY\n");

			for (i = 0; i < 5; i++) {
				hdmi_rx_reg_write32(HDMI_NCPER, 0xff, HDMI_RX_MAC);
				HDMI_DELAYMS(20);
				if ((hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC) & (_BIT3|_BIT2|_BIT1))==0)
					break;
				hdmi_rx_reg_mask32(HDMI_SR, ~(_BIT3|_BIT2|_BIT1), (_BIT3|_BIT2|_BIT1), HDMI_RX_MAC);
			}

			HDMI_PRINTF( "FIFO timeout count2= %d\n",i);
			if (hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC) & (_BIT1|_BIT2|_BIT3)){
				HDMI_PRINTF("Audio PLL not ready = %x\n",hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC));
				SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);
				return FALSE;
			}
			SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START_OUT);

		//break;
		// Play audio here
		case AUDIO_FSM_AUDIO_START_OUT:
		{
			HDMI_PRINTF("AUDIO_FSM_AUDIO_START_OUT\n");
			hdmi_rx_reg_mask32(HDMI_DBCR , 0xF0, 0x0F, HDMI_RX_MAC);
// FIXME:
			d_code = hdmi_rx_reg_read32(HDMI_APLLCR3, HDMI_RX_MAC);

			for (i=0; i<5; i++) {
				if (d_code == hdmi_rx_reg_read32(HDMI_APLLCR3, HDMI_RX_MAC)) break;
			}

			hdmi_rx_reg_write32(HDMI_DCAPR0, d_code, HDMI_RX_MAC);//pre-set D code


			hdmi_rx_reg_write32(HDMI_PSCR, 0xE2, HDMI_RX_MAC);//pre-disable N/CTS tracking & FIFO depth

			//Enable audio Overflow & Underflow watch dog but not Audio type wdg
			hdmi_rx_reg_mask32(HDMI_WDCR0,(~(_BIT1|_BIT2|_BIT3 | _BIT4)), _BIT1|_BIT2|_BIT3 | _BIT4, HDMI_RX_MAC);
			hdmi_rx_reg_mask32(HDMI_WDCR0, (~_BIT15), _BIT15, HDMI_RX_MAC);//Enable audio tmds clock  watch dog

			Hdmi_AudioOutputEnable();

			hdmi_rx_reg_mask32(HDMI_AOCR, (unsigned char)(~0x0ff), 0xFF, HDMI_RX_MAC); //Enable SPDIF/I2S Output
			SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_CHECK);
			result = TRUE;

			hdmi.audio_freq = spdif_freq;
			hdmi.spdif_type = HDMI_SR_get_spdiftype(hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC));
			hdmi_ioctl_struct.audio_detect_done = 1;
		}
		break;

		case AUDIO_FSM_AUDIO_CHECK:
		{
//			HDMI_LOG("AUDIO_FSM_AUDIO_CHECK\n");

			// if FIFO overflow then restart Audio process
			if (hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC) & (_BIT3)) {
				HDMI_PRINTF( "Audio Output Disable cause by pll unlock :%x\n",hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC));
				Hdmi_AudioOutputDisable();
				SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);
			}
			if(hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC) & (_BIT1|_BIT2)) {
				HDMI_PRINTF( "Audio Output Disable cause by over_underflow :%x\n",hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC));
				Hdmi_AudioOutputDisable();
				SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);
			}

			if ((hdmi_rx_reg_read32(HDMI_AVMCR, HDMI_RX_MAC) & (_BIT5)) == 0) {
				HDMI_PRINTF( "Audio Output Disable cause by AVMCR output disable:%x\n",hdmi_rx_reg_read32(HDMI_AVMCR, HDMI_RX_MAC));
				Hdmi_AudioOutputDisable();
				SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);
			}

			if ((HDMI_AUDIO_IS_LPCM() == 0) && (HDMI_AUDIO_SUPPORT_NON_PCM()==0)) { 	// if TX change audio mode to non-LPCM
				Hdmi_AudioOutputDisable();
				HDMI_PRINTF( "Audio Output Disable cause non-Linear PCM\n");
				SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);
			}
			if ( ((HDMI_AUDIO_IS_LPCM()) && GET_HDMI_AUDIO_TYPE()) || ((HDMI_AUDIO_IS_LPCM() == 0) && (GET_HDMI_AUDIO_TYPE()==0))) {
				Hdmi_AudioOutputDisable();
				HDMI_PRINTF( "Audio Type change \n");
				SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);
			}

		}
		break;

		default:
		{
			SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);
		}
		break;

	}

	return result;
}

void Hdmi_VideoOutputDisable(void)
{
	hdmi_rx_reg_mask32(HDMI_AVMCR, ~_BIT3, 0x00, HDMI_RX_MAC);
}

void Hdmi_VideoOutputEnable(void)
{
	hdmi_rx_reg_mask32(HDMI_AVMCR, ~_BIT3, _BIT3, HDMI_RX_MAC);
}

void hdmi_clock_port_select(int port, char enable)
{
	switch ( port )
	{
	case HDMI_CHANNEL0:
		// port 0 set CKAFE power ,port bias ,input reference no hysteresis amp
		hdmi_rx_reg_mask32(P0_CK1, ~(P0_CK1_reg_p0_ck_1_mask|p0_ck_2_CMU_CKIN_SEL),
										(p0_ck_1_CKAFE_POW|p0_ck_1_port_bias|_BIT13|_BIT14|_BIT15), HDMI_RX_PHY);

		// port 1 set CKAFE power ,port bias ,input reference no hysteresis amp
		hdmi_rx_reg_mask32(P1_CK1, ~(P1_CK1_reg_p1_ck_1_mask|p1_ck_2_CMU_CKIN_SEL), 0, HDMI_RX_PHY);

		// port 2 set CKAFE power ,port bias ,input reference no hysteresis amp
		hdmi_rx_reg_mask32(P2_CK1, ~(P2_CK1_reg_p2_ck_1_mask|p2_ck_2_CMU_CKIN_SEL), 0, HDMI_RX_PHY);
		break;
	default:
		HDMIRX_INFO("[%s] Unknow port",__FUNCTION__);
		break;
	}
}

void hdmi_phy_port_select(int port, char enable)
{
	hdmi_z0_set(port, LN_ALL, 1);

	// CK_TX select from which port
	hdmi_rx_reg_mask32(TOP_IN, ~TOP_IN_reg_top_in_1_mask, TOP_IN_reg_top_in_1(1<<port), HDMI_RX_PHY);
	hdmi_clock_port_select(port,enable);
	// Digital part sel port
	hdmi_rx_reg_mask32(PHY_FIFO_CR, ~PHY_FIFO_CR_port_sel_mask, PHY_FIFO_CR_port_sel(port), HDMI_RX_MAC);
	// clock detect
	hdmi_rx_reg_mask32(MD, ~MD_reg_ck_ckdet_mask, MD_reg_ck_ckdet(1<<(2-port)), HDMI_RX_PHY);

	switch ( port )
	{
		case HDMI_CHANNEL0:
			//mode detect clock reset
			hdmi_rx_reg_mask32(REGD42, ~(REGD42_p0_ck_md_rstb_mask), (REGD42_p0_ck_md_rstb_mask), HDMI_RX_PHY);

			//HDMI mode & MHL 2.0 buffer power off
			hdmi_rx_reg_mask32(MHL, ~(MHL_reg_p0_mhl_hdmi_cksel_mask|MHL_reg_p0_mhl_hdmi_datasel_mask|MHL_reg_p0_mhl_hdmi_cksel_mask),
									(MHL_reg_p0_mhl_hdmi_datasel_mask|MHL_reg_p0_mhl_hdmi_cksel_mask), HDMI_RX_PHY);

			//B lane PR power on & EQ power
			hdmi_rx_reg_mask32(P0_B2, ~(P0_b_8_POW_PR|P0_b_5_EQ_POW), (P0_b_8_POW_PR|P0_b_5_EQ_POW), HDMI_RX_PHY);
			//G lane PR power on  & EQ power
			hdmi_rx_reg_mask32(P0_G2, ~(P0_g_8_POW_PR|P0_g_5_EQ_POW), (P0_g_8_POW_PR|P0_g_5_EQ_POW), HDMI_RX_PHY);

			//R lane PR power on  & EQ power
			hdmi_rx_reg_mask32(P0_R2, ~(P0_r_8_POW_PR|P0_r_5_EQ_POW), (P0_r_8_POW_PR|P0_r_5_EQ_POW), HDMI_RX_PHY);

			//port bias  & CK AFE power
			hdmi_rx_reg_mask32(P0_CK1, ~(p0_ck_1_port_bias|p0_ck_1_CKAFE_POW), (p0_ck_1_port_bias|p0_ck_1_CKAFE_POW), HDMI_RX_PHY);

			//LDO enable
			hdmi_rx_reg_mask32(P0_CK2, ~(P0_ck_8_LDO_EN), (P0_ck_8_LDO_EN), HDMI_RX_PHY);
			// PLL enable  need to check close other port or not ???
			hdmi_rx_reg_mask32(ENABLE, ~(ENABLE_reg_p0_en_cmu_mask), (ENABLE_reg_p0_en_cmu_mask), HDMI_RX_PHY);
			//Asynchronous FIFO control  invert
			hdmi_rx_reg_mask32(PHY_FIFO_CR, ~(PHY_FIFO_CR_port0_bclk_inv_mask|PHY_FIFO_CR_port0_gclk_inv_mask|PHY_FIFO_CR_port0_rclk_inv_mask),
											(PHY_FIFO_CR_port0_bclk_inv_mask|PHY_FIFO_CR_port0_gclk_inv_mask|PHY_FIFO_CR_port0_rclk_inv_mask), HDMI_RX_MAC);

		break;

		default:
			HDMIRX_INFO("[%s] Unknow port",__FUNCTION__);
		break;
	}

}

HDMI_ERR_T Hdmi_CheckConditionChange(void)
{
	HDMIRX_IOCTL_STRUCT_T isr_info;
	HdmiGetStruct(&isr_info);

	if (isr_info.b < 10)
	{
		//HDMIRX_DEBUG("b < 10 change = %d  " ,isr_info.b);
		return HDMI_EER_GENERIC;
	}

	if (HDMI_ABS(hdmi.b, isr_info.b) > 4)
	{
		//HDMIRX_DEBUG(" abs b > 4  change");
		return HDMI_EER_GENERIC;
	}

	if ((unsigned int)GET_HDMI_CD() != (hdmi_rx_reg_read32(TMDS_DPC0, HDMI_RX_MAC) & 0xf))
	{
		//HDMIRX_DEBUG("color depth");
		return HDMI_EER_GENERIC;
	}

	if (isr_info.b_change)
	{
		//HDMIRX_DEBUG("b change = %d",isr_info.b);
		return HDMI_EER_GENERIC;
	}

#if 0// TODO: remove?
	if (GET_HDMI_ISINTERLACE()!= Hdmi_GetInterlace(HDMI_MS_MODE_ONESHOT))
	{
		//HDMIRX_DEBUG("interlace change");
		return HDMI_EER_GENERIC;
	}
#endif

	if (GET_ISHDMI() != IsHDMI()) {
		//HDMIRX_DEBUG("HDMI/DVI change");
		return HDMI_EER_GENERIC;
	}

	return HDMI_ERR_NO;
}

HDMI_COLOR_SPACE_T Hdmi_GetColorSpace(void)
{
	UINT8 bColor_temp;
	HDMIRX_IOCTL_STRUCT_T isr_info;
	HdmiGetStruct(&isr_info);

	bColor_temp = (HDMI_COLOR_SPACE_T)HDMI_VCR_get_csc_r(hdmi_rx_reg_read32(HDMI_VCR, HDMI_RX_MAC));

	if (bColor_temp == COLOR_YUV420 )
	{
		bColor_temp = COLOR_YUV444;
		hdmi_rx_reg_mask32(YUV420_CR, ~YUV420_CR_en_mask, YUV420_CR_en_mask, HDMI_RX_MAC);//// Enable YUV420to444
	}
	else
	{
		hdmi_rx_reg_mask32(YUV420_CR, ~YUV420_CR_en_mask, 0, HDMI_RX_MAC);// Disable YUV420to444
	}

	hdmi.gen_timing.color= (HDMI_COLOR_SPACE_T)bColor_temp;

	return ColorMap[hdmi.gen_timing.color];
}


HDMI_bool Hdmi_Measure(void)
{
	int retry;
	HDMI_TIMING_T temp;

	if ( FALSE==Hdmi_MeasureTiming(&hdmi.tx_timing, hdmi.b) )
	{
#if HDMI2p0
		TMDS_6G_Recovery();
#endif
		HDMI_PRINTF("Hdmi_MeasureTiming Error\n");
		return MEASURE_FAIL;
	}

	if (hdmi.tx_timing.v_act_len > hdmi.tx_timing.v_total) return MEASURE_FAIL;

	if (hdmi.tx_timing.h_act_len > hdmi.tx_timing.h_total) return MEASURE_FAIL;

	if (hdmi.tx_timing.h_act_len < 120) return MEASURE_FAIL;

	if (hdmi.tx_timing.v_act_len < 240) return MEASURE_FAIL;


	for (retry = 0; retry < HDMI_MEMSURE_RETRY; retry++)
	{
		if ( FALSE==Hdmi_MeasureTiming(&temp, hdmi.b) )
		{
			HDMI_PRINTF("Hdmi_MeasureTiming Error\n");
			return MEASURE_FAIL;
		}

		if (ABS(temp.h_total, hdmi.tx_timing.h_total) != 0) return MEASURE_FAIL;
		if (ABS(temp.v_total, hdmi.tx_timing.v_total) > 5) return MEASURE_FAIL;

		if (ABS(temp.h_act_len, hdmi.tx_timing.h_act_len) != 0) return MEASURE_FAIL;
		if (ABS(temp.v_act_len, hdmi.tx_timing.v_act_len) != 0) return MEASURE_FAIL;

		if (ABS(temp.h_freq, hdmi.tx_timing.h_freq) > 5) return MEASURE_FAIL;
		if (ABS(temp.v_freq,  hdmi.tx_timing.v_freq) > 5) return MEASURE_FAIL;
	}

#if HDMI2p0
	b6G_detect_cnt = 0;
#endif

	return _MODE_SUCCESS;
}

void Hdmi_DumpState(void)
{
#if HDMI_INTERNAL_DEBUG
	char *colormetry_name[]  = {
		"HDMI_COLORIMETRY_NOSPECIFIED",
		"HDMI_COLORIMETRY_601",
		"HDMI_COLORIMETRY_709",
		"HDMI_COLORIMETRY_XYYCC601",
		"HDMI_COLORIMETRY_XYYCC709",
		"HDMI_COLORIMETRY_SYCC601",
		"HDMI_COLORIMETRY_ADOBE_YCC601",
		"HDMI_COLORIMETRY_ADOBE_RGB",
	};
	char *depth_name[] = {
		"HDMI_COLOR_DEPTH_8B",
		"HDMI_COLOR_DEPTH_10B",
		"HDMI_COLOR_DEPTH_12B",
		"HDMI_COLOR_DEPTH_16B",
	};
	char *colorspace_name[] = {
		"COLOR_RGB",
		"COLOR_YUV422",
		"COLOR_YUV444",
		"COLOR_YUV420",
		"COLOR_UNKNOW"
	};
	char *hdmi_3d_name[] = {
		"HDMI3D_FRAME_PACKING",
		"HDMI3D_FIELD_ALTERNATIVE",
		"HDMI3D_LINE_ALTERNATIVE",
		"HDMI3D_SIDE_BY_SIDE_FULL",
		"HDMI3D_L_DEPTH",
		"HDMI3D_L_DEPTH_GPX",
		"HDMI3D_TOP_AND_BOTTOM",
		"HDMI3D_RSV0",
		"HDMI3D_SIDE_BY_SIDE_HALF",
		"HDMI3D_RSV1",
		"HDMI3D_2D_ONLY",
	};
#endif
	HDMI_PRINTF( "bHDMIColorSpace = %s\n", hdmi.tx_timing.color < (sizeof(colorspace_name)/4)  ? colorspace_name[hdmi.tx_timing.color] : "UNDEFINED");
	HDMI_PRINTF( "IsInterlaced = %d\n", GET_HDMI_ISINTERLACE());
	HDMI_PRINTF( "bIsHDMIDVI = %d\n", GET_ISHDMI());
	HDMI_PRINTF( "VedioFSMState = %d\n", GET_HDMI_VIDEO_FSM());
	HDMI_PRINTF( "AudioFSMState = %d\n", GET_HDMI_AUDIO_FSM());
	HDMI_PRINTF( "ColorDepth = %s\n", hdmi.tx_timing.depth < (sizeof(depth_name)/4)  ? depth_name[hdmi.tx_timing.depth] : "UNDEFINED");
	HDMI_PRINTF("ColorMetry = %s\n", hdmi.tx_timing.colorimetry < (sizeof(colormetry_name)/4)  ? colormetry_name[hdmi.tx_timing.colorimetry]: "UNDEFINED");
	HDMI_PRINTF( "3D Format  = %s\n", hdmi.tx_timing.hdmi_3dformat < (sizeof(hdmi_3d_name)/4) ? hdmi_3d_name[hdmi.tx_timing.hdmi_3dformat] : "UNDEFINED");
	HDMI_PRINTF( "FW 3D Format	= %s\n", hdmi.gen_timing.hdmi_3dformat < (sizeof(hdmi_3d_name)/4) ? hdmi_3d_name[hdmi.gen_timing.hdmi_3dformat] : "UNDEFINED");

	if (hdmi.gen_timing.hdmi_3dformat < HDMI3D_2D_ONLY) {
		HDMI_PRINTF( "FW ColorDepth = %s\n", hdmi.gen_timing.depth < (sizeof(depth_name)/4)  ? depth_name[hdmi.gen_timing.depth] : "UNDEFINED");
		HDMI_PRINTF(" FW ColorMetry = %s\n", hdmi.gen_timing.colorimetry < (sizeof(colormetry_name)/4)	? colormetry_name[hdmi.gen_timing.colorimetry]: "UNDEFINED");
	}
//	HDMI_PRINTF( "Is422 = %d\n", GET_SCALER_IS422());
}


#if HDMI2p0
unsigned char drvif_Hdmi2p0_Scdc_Read(unsigned char addr)
{
	hdmi_rx_reg_write32(SCDC_AP, addr, HDMI_RX_MAC);
	return hdmi_rx_reg_read32(SCDC_DP, HDMI_RX_MAC);
}

void drvif_Hdmi2p0_Scdc_Write(unsigned char addr,unsigned char value)
{
	hdmi_rx_reg_write32(SCDC_AP, addr, HDMI_RX_MAC);
	hdmi_rx_reg_write32(SCDC_DP, value, HDMI_RX_MAC);
}

void drvif_Hdmi2p0_DetectMode(void)
{
	UINT8 bTMDSStatus;

	// support HDCP2p2
	if (hdmi_rx_reg_read32(HDMI_AFCR, HDMI_RX_MAC)&HDMI_6G_TEST)
	{
		bHDMI_6G_flag = 1;//For CTS test
		HDMI_PRINTF("[HDMI2.0] TMDS mode clock 40X  CTS mode \n");
	}
	else
	{
		bTMDSStatus = drvif_Hdmi2p0_Scdc_Read(SCDC_TMDS_Config);
		if(((bTMDSStatus&_BIT1) == _BIT1 ))
		{
			bHDMI_6G_flag = 1;
			HDMI_PRINTF("[HDMI2.0] TMDS mode clock 40X \n");
		}
		else
		{
			if (b6G_detect_cnt >2)// For 6G recovery setting
			{
				bHDMI_6G_flag = 1;
				HDMI_PRINTF("[HDMI2.0] TMDS mode clock 40X \n");
			}
			else
			{
				bHDMI_6G_flag = 0;
				HDMI_PRINTF("[HDMI2.0] TMDS mode clock 10X \n");
			}
		}
	}

	//alwyas open hdmi2.0 setting
	hdmi_rx_reg_mask32(HDMI_2P0_CR, ~(HDMI_2P0_CR_hdmi_2p0_en_mask), HDMI_2P0_CR_hdmi_2p0_en_mask, HDMI_RX_MAC);

	hdmi_rx_reg_mask32(SCR_CR, ~(SCR_CR_scr_auto_mask), SCR_CR_scr_auto_mask, HDMI_RX_MAC);
	if( bHDMI_6G_flag ==1 )
	{
		//For RGB bug
		hdmi_rx_reg_mask32(HDMI_SCR, ~(HDMI_SCR_dvi_reset_ds_cm_en_mask), 0, HDMI_RX_MAC);
	}
	else
	{
		hdmi_rx_reg_mask32(HDMI_SCR, ~(HDMI_SCR_dvi_reset_ds_cm_en_mask), HDMI_SCR_dvi_reset_ds_cm_en_mask, HDMI_RX_MAC);
		//For 340M>TMDS
	}


}

void drvif_Hdmi2p0_Error_Count(void)
{
	u_int32_t tout=5;
	//HW reset
	//clock reset
	hdmi_rx_reg_mask32(CERCR, ~(CERCR_ch_locked_reset_mask), CERCR_ch_locked_reset_mask, HDMI_RX_MAC);
	hdmi_rx_reg_mask32(CERCR, ~(CERCR_valid_reset_mask|CERCR_reset_err_det_mask|CERCR_keep_err_det_mask|CERCR_refer_implem_mask|CERCR_reset_mask),
							(CERCR_valid_reset_mask|CERCR_reset_err_det_mask|CERCR_keep_err_det_mask|CERCR_refer_implem_mask|CERCR_reset_mask), HDMI_RX_MAC);
	//set period & mode
	hdmi_rx_reg_mask32(CERCR, ~(CERCR_period_mask|CERCR_mode_mask), CERCR_period_mask, HDMI_RX_MAC);
	//reset release
	hdmi_rx_reg_mask32(CERCR, ~(CERCR_ch_locked_reset_mask), 0, HDMI_RX_MAC);
	hdmi_rx_reg_mask32(CERCR, ~(CERCR_valid_reset_mask|CERCR_reset_err_det_mask|CERCR_keep_err_det_mask|CERCR_refer_implem_mask|CERCR_reset_mask),0,HDMI_RX_MAC);
	//set enable
	hdmi_rx_reg_mask32(CERCR, ~(CERCR_en_mask), CERCR_en_mask, HDMI_RX_MAC);

	while( CERCR_get_en(hdmi_rx_reg_read32(CERCR,HDMI_RX_MAC)) && tout--)
	{
		HDMI_DELAYMS(50);
	}

	HDMI_PRINTF("[HDMI][BER] [video]   cnt0  bit_err_cnt=%d ,cnt1 bit_err_cnt=%d  ,cnt2 bit_err_cnt=%d \n",
					CERSR0_get_err_cnt0_video(hdmi_rx_reg_read32(CERSR0, HDMI_RX_MAC)),
					CERSR0_get_err_cnt1_video(hdmi_rx_reg_read32(CERSR0, HDMI_RX_MAC)),
					CERSR1_get_err_cnt2_video(hdmi_rx_reg_read32(CERSR1, HDMI_RX_MAC)));
	HDMI_PRINTF("[HDMI][BER] [packet]  cnt0  bit_err_cnt=%d ,cnt1 bit_err_cnt=%d  ,cnt2 bit_err_cnt=%d \n",
					CERSR1_get_err_cnt0_pkt(hdmi_rx_reg_read32(CERSR1, HDMI_RX_MAC)),
					CERSR2_get_err_cnt1_pkt(hdmi_rx_reg_read32(CERSR2, HDMI_RX_MAC)),
					CERSR2_get_err_cnt2_pkt(hdmi_rx_reg_read32(CERSR2, HDMI_RX_MAC)));
	HDMI_PRINTF("[HDMI][BER] [control]  cnt0  bit_err_cnt=%d ,cnt1 bit_err_cnt=%d  ,cnt2 bit_err_cnt=%d \n",
					CERSR3_get_err_cnt0_ctr(hdmi_rx_reg_read32(CERSR3, HDMI_RX_MAC)),
					CERSR3_get_err_cnt1_ctr(hdmi_rx_reg_read32(CERSR3, HDMI_RX_MAC)),
					CERSR4_get_err_cnt2_ctr(hdmi_rx_reg_read32(CERSR4, HDMI_RX_MAC)));
}

void TMDS_6G_Recovery(void)
{// IF SCDC not right , toggle 6G flag setting
	if (hdmi_rx_reg_read32(HDMI_AFCR, HDMI_RX_MAC)&HDMI_6G_TEST)
	{
		HDMI_PRINTF("[HDMI2.0] CTS mode  \n");
	}
	else
	{
		if((drvif_Hdmi2p0_Scdc_Read(SCDC_TMDS_Config)&_BIT1) != _BIT1 )
		{
			b6G_detect_cnt++;
			if (b6G_detect_cnt >2)
			{
				bHDMI_6G_flag?( bHDMI_6G_flag= 0):(bHDMI_6G_flag= 1)  ;  //detect 6 G or not
				b6G_detect_cnt = 0;
			}
		}
	}
}

void HDMI_YUV420_Setting(void)
{
	bHDMI_420_Space = HDMI_VCR_get_csc_r(hdmi_rx_reg_read32(HDMI_VCR, HDMI_RX_MAC) );
	if(bHDMI_420_Space == COLOR_YUV420)
	{
		hdmi_rx_reg_mask32(YUV420_CR, ~YUV420_CR_en_mask, YUV420_CR_en_mask, HDMI_RX_MAC);
		HDMI_PRINTF("[HDMI2.0] HDMI420 mode  \n");
	}
	HDMI_PRINTF("[HDMI2.0] color space  mode =%d   \n",HDMI_VCR_get_csc_r(hdmi_rx_reg_read32(HDMI_VCR, HDMI_RX_MAC) ));
	if ((IsHDMI() ==MODE_DVI)||(bHDMI_420_Space!= COLOR_YUV420))
	{
		hdmi_rx_reg_mask32(YUV420_CR, ~YUV420_CR_en_mask, 0, HDMI_RX_MAC);
	}
}
#endif//End HDMI2p0


unsigned char drvif_Hdmi_GetColorDepth(void)
{
	if (GET_HDMI_CD() >= 4)
		return (GET_HDMI_CD()-4);
	else
		return 0;
}

unsigned char drvif_Hdmi_AVI_VIC(void)
{
	unsigned char reg_val;
	hdmi_rx_reg_write32(HDMI_PSAP, AVI_Data_BYTE4, HDMI_RX_MAC);

	reg_val = (hdmi_rx_reg_read32(HDMI_PSDP, HDMI_RX_MAC)) & 0x7F;

	if(reg_val <= 107)
		return reg_val;
	else
		return 0;
}

unsigned char drvif_Hdmi_AVI_RGB_Range(void)
{
	unsigned char Temp,RGB_Temp,YUV_Temp;
	/*
		looking for ACR info using RSV2
	*/
	HDMI_PRINTF("drvif_Hdmi_AVI_RGB_Range \n");
	hdmi_rx_reg_write32(HDMI_PSAP, AVI_Data_BYTE1, HDMI_RX_MAC); //read AVI infor REG direct

	Temp =	(hdmi_rx_reg_read32(HDMI_PSDP, HDMI_RX_MAC) & AVI_Y1Y0_mask)>>5 ;

	if(Temp == 0)
	{
		hdmi_rx_reg_write32(HDMI_PSAP, AVI_Data_BYTE3, HDMI_RX_MAC); //read AVI infor Data Byte 3
		RGB_Temp = (hdmi_rx_reg_read32(HDMI_PSDP, HDMI_RX_MAC) & AVI_Q_Range_mask)>>2 ;
		switch(RGB_Temp)
		{
			case RGB_Default :
				SET_HDMI_RGB_Q_RANGE(RGB_Default);
				HDMI_PRINTF("drvif_Hdmi_AVI_RGB_Range test0  = %d\n", GET_HDMI_RGB_Q_RANGE());
			break;
			case RGB_Limite_Range:
				SET_HDMI_RGB_Q_RANGE(RGB_Limite_Range);
				HDMI_PRINTF("drvif_Hdmi_AVI_RGB_Range test1 = %d\n", GET_HDMI_RGB_Q_RANGE());
			break;
			case RGB_Full_Range:
				SET_HDMI_RGB_Q_RANGE(RGB_Full_Range);
				HDMI_PRINTF("drvif_Hdmi_AVI_RGB_Range test2  = %d\n", GET_HDMI_RGB_Q_RANGE());
			break;
		}
		HDMI_PRINTF("drvif_Hdmi_AVI_RGB_Range = %d\n", RGB_Temp);
	}
	else if ((Temp == 0x01 )||(Temp == 0x02 ))
	{
		hdmi_rx_reg_write32(HDMI_PSAP, AVI_Data_BYTE5, HDMI_RX_MAC); //read AVI infor Data Byte 3
		YUV_Temp = (hdmi_rx_reg_read32(HDMI_PSDP, HDMI_RX_MAC) & AVI_YQ1YQ0_mask)>>6 ;
		switch(YUV_Temp)
		{
			case YUV_Limite_Range:
				SET_HDMI_RGB_Q_RANGE(RGB_Limite_Range);
				HDMI_PRINTF("drvif_Hdmi_AVI_YUV_Range test1 = %d\n", GET_HDMI_RGB_Q_RANGE());
			break;
			case YUV_Full_Range:
				SET_HDMI_RGB_Q_RANGE(RGB_Full_Range);
				HDMI_PRINTF("drvif_Hdmi_AVI_YUV_Range test2  = %d\n", GET_HDMI_RGB_Q_RANGE());
			break;
		}
		HDMI_PRINTF("drvif_Hdmi_AVI_YUV_Range = %d\n", YUV_Temp);
	}

	return Temp;
}


HDMI_bool drvif_Hdmi_DetectMode(void)
{
	unsigned char result = _MODE_DETECT;
	HDMIRX_IOCTL_STRUCT_T isr_info;
	unsigned char yuv_fmt;
	unsigned int retry_times = 0;
retry:

	if (Hdmi_CheckConditionChange() != HDMI_ERR_NO)
	{
		SET_HDMI_VIDEO_FSM(MAIN_FSM_HDMI_SETUP_VEDIO_PLL);
		SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);
	}

	switch(GET_HDMI_VIDEO_FSM())
	{

		case MAIN_FSM_HDMI_SETUP_VEDIO_PLL:
			// HDMI_PRINTF("MAIN_FSM_HDMI_SETUP_VEDIO_PLL\n");

			HDMI_Audio_Conut = 0;

			hdmi_rx_reg_mask32(HDMI_WDCR0, ~0xFFDF9E, 0x00, HDMI_RX_MAC); // Clear Audio Watch Dog and Set X: 15
			hdmi_rx_reg_mask32(HDMI_VCR, (~_BIT7) , 0, HDMI_RX_MAC); // not inverse EVEN/ODD

			// enable auto detection of colorspace and pixel repeat
			hdmi_rx_reg_mask32(HDMI_VCR, ~(HDMI_VCR_csam_mask|HDMI_VCR_prdsam_mask), HDMI_VCR_csam(1)|HDMI_VCR_prdsam(1), HDMI_RX_MAC);

			hdmi_rx_reg_mask32(MHL_DEMUX_CTRL, ~MHL_DEMUX_CTRL_dvi_ch_sync_mask, 0, HDMI_RX_MHL);
			hdmi_rx_reg_mask32(HDMI_VCR, ~(HDMI_VCR_se_mask|HDMI_VCR_eot_mask|HDMI_VCR_int_pro_chg_flag_mask), HDMI_VCR_eot_mask, HDMI_RX_MAC); //clear status, prepare to check interlace/progressive

			memset(&hdmi.gen_timing, 0, sizeof(hdmi.gen_timing));

			// get TMDS b value for ISR/high priority task
			HdmiGetStruct(&isr_info);
			hdmi.b = isr_info.b;
			//hdmi_out(HDMI_ONMS_WATCHDOG_EN_reg, 0);


			if (hdmi.b < 116)
			{
				return _MODE_NOSIGNAL;
			}

#if HDMI2p0
			drvif_Hdmi2p0_DetectMode();
#endif

			// measure interlace/progressive
			hdmi.tx_timing.progressive = (Hdmi_GetInterlace(HDMI_MS_MODE_ONESHOT) == 0);
			SET_HDMI_ISINTERLACE((hdmi.tx_timing.progressive == 0));
			// measure color depth
			SET_HDMI_CD(hdmi_rx_reg_read32(TMDS_DPC0, HDMI_RX_MAC) & (0x0f));
			// tx_timing
			hdmi.tx_timing.depth = (HDMI_COLOR_DEPTH_T) drvif_Hdmi_GetColorDepth();

#if HDMI2p0
			HDMI_YUV420_Setting();

			// setup Video PLL
			if(bHDMI_420_Space == COLOR_YUV420)
			{
				if (!Hdmi_VideoPLLSetting(hdmi.b, (int)hdmi.tx_timing.depth, 1))
					return _MODE_NOSIGNAL;
			}
			else
#endif
			if((hdmi.tx_timing.progressive) == 0)
			{
				if (!Hdmi_VideoPLLSetting(hdmi.b, (int)hdmi.tx_timing.depth, 1))
					return _MODE_NOSIGNAL;
				//rtd_maskl(TMDS_DPC_SET0_reg, ~TMDS_DPC_SET0_dpc_bypass_dis_mask, TMDS_DPC_SET0_dpc_bypass_dis(1));
			}
			else
			{
				if (!Hdmi_VideoPLLSetting(hdmi.b, (int)hdmi.tx_timing.depth, 0))
					return _MODE_NOSIGNAL;
				//rtd_maskl(TMDS_DPC_SET0_reg, ~TMDS_DPC_SET0_dpc_bypass_dis_mask, TMDS_DPC_SET0_dpc_bypass_dis(0));
			}

			SET_ISHDMI(IsHDMI());

			if( (GET_ISHDMI() == MODE_DVI))
			{
				//Disable Auto color space detect,Auto pixel reapeat down sample
				HDMI_PRINTF("DVI mode setting\n");
				hdmi_rx_reg_mask32(HDMI_VCR, ~(HDMI_VCR_csam_mask | HDMI_VCR_prdsam_mask | HDMI_VCR_dsc_mask),
											HDMI_VCR_csam(0) | HDMI_VCR_prdsam(0) | HDMI_VCR_dsc(0), HDMI_RX_MAC);
				HDMI_DELAYMS(20);//Set down sampling  =1 can't set if no this delay
				// Set down sampling = 1
				hdmi_rx_reg_mask32(HDMI_VCR, ~(HDMI_VCR_csam_mask | HDMI_VCR_prdsam_mask | HDMI_VCR_dsc_mask | HDMI_VCR_csc_mask),
											HDMI_VCR_csam(0) | HDMI_VCR_prdsam(0) | HDMI_VCR_dsc(0) | HDMI_VCR_csc(0), HDMI_RX_MAC);
				// TODO: may don't need to set twice
			}

			Hdmi_Get3DInfo(HDMI_MS_MODE_ONESHOT_INIT);
			//Hdmi_GetInterlace(HDMI_MS_MODE_CONTINUOUS_INIT);
			SET_HDMI_VIDEO_FSM(MAIN_FSM_HDMI_MEASURE);
			//goto retry;
			//break;
		case MAIN_FSM_HDMI_MEASURE:
			HDMI_PRINTF("MAIN_FSM_HDMI_MEASURE\n");

			Hdmi_VideoOutputEnable();
			if (hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC)&HDMI_SR_avmute_mask){
				HDMI_PRINTF("AV Mute\n");
				HdmiSetAPStatus(HDMIRX_DETECT_AVMUTE);
			} else {
				HdmiSetAPStatus(HDMIRX_DETECT_FAIL);
			}

			if(GET_ISHDMI() == MODE_HDMI) {
				SET_HDMI_COLOR_SPACE(Hdmi_GetColorSpace());
			}else {
				// Determine Color Space
				SET_HDMI_COLOR_SPACE(COLOR_RGB);
			}

			if (GET_HDMI_ISINTERLACE() != Hdmi_GetInterlace(HDMI_MS_MODE_ONESHOT)) {
				HDMI_PRINTF("interlace change in measure mode\n");
				SET_HDMI_VIDEO_FSM(MAIN_FSM_HDMI_SETUP_VEDIO_PLL);	// reset Check mode state to initial
				return _MODE_DETECT;
			}

#if HDMI2p0
			if (bHDMI_420_Space !=HDMI_VCR_get_csc_r(hdmi_rx_reg_read32(HDMI_VCR, HDMI_RX_MAC))) {
				HDMI_PRINTF("color space change in measure mode\n");
				SET_HDMI_VIDEO_FSM(MAIN_FSM_HDMI_SETUP_VEDIO_PLL);	// reset Check mode state to initial
				return _MODE_DETECT;
			}
#endif
			if ((result = Hdmi_Measure()) != _MODE_SUCCESS) {
				SET_HDMI_VIDEO_FSM(MAIN_FSM_HDMI_SETUP_VEDIO_PLL);	// reset Check mode state to initial
				HDMI_Bit_Err_One_Time_Detect(100,20);
				if (hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC) & _BIT6) return _MODE_DETECT;
				return result;
			}

			if (GET_HDMI_ISINTERLACE() != Hdmi_GetInterlace(HDMI_MS_MODE_ONESHOT)) {
				HDMI_PRINTF("interlace change in measure mode\n");
				SET_HDMI_VIDEO_FSM(MAIN_FSM_HDMI_SETUP_VEDIO_PLL);	// reset Check mode state to initial
				return _MODE_DETECT;
			}

			HdmiSetAPStatus(HDMIRX_DETECT_SUCCESS);
			//hdmi_onms_measure(&hdmi.tx_timing, HDMI_MS_MODE_CONTINUOUS_INIT);
			
			hdmi.tx_timing.hdmi_3dformat = Hdmi_Get3DInfo(HDMI_MS_MODE_ONESHOT_GET_RESULT);
			SET_HDMI_VIDEO_FSM(MAIN_FSM_HDMI_DISPLAY_ON);

			if(retry_times<3)//Prevent stuck in this function
			{
				retry_times++;
				goto retry;
			}

			break;

		case MAIN_FSM_HDMI_DISPLAY_ON:
			HDMI_PRINTF("MAIN_FSM_HDMI_DISPLAY_ON\n");
			if ((hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC) & HDMI_SR_avmute_mask) != 0) {
				HDMI_PRINTF("#################### AV mute ####################\n");
				//cloud clear AV mute flag 20141119
				hdmi_rx_reg_mask32(HDMI_AVMCR, ~(HDMI_AVMCR_avmute_flag_mask), HDMI_AVMCR_avmute_flag_mask, HDMI_RX_MAC);
				hdmi_rx_reg_mask32(HDMI_AVMCR, ~(HDMI_AVMCR_avmute_flag_mask), 0, HDMI_RX_MAC);
				SET_HDMI_VIDEO_FSM(MAIN_FSM_HDMI_SETUP_VEDIO_PLL);	// reset Check mode state to initial
				return _MODE_DETECT;
			}
			// Determine DVI/HDMI mode
			if(GET_ISHDMI() == MODE_HDMI) {
				// Determine Color Space
				SET_HDMI_COLOR_SPACE(Hdmi_GetColorSpace());
			}else {
				// Determine Color Space
				SET_HDMI_COLOR_SPACE(COLOR_RGB);
			}
			drvif_Hdmi_AVI_RGB_Range();//cloud test
			if (GET_HDMI_ISINTERLACE() != Hdmi_GetInterlace(HDMI_MS_MODE_ONESHOT)) {
				HDMI_PRINTF("interlace change in measure mode\n");
				SET_HDMI_VIDEO_FSM(MAIN_FSM_HDMI_SETUP_VEDIO_PLL);	// reset Check mode state to initial
				return _MODE_DETECT;
			}
			// To gather detail information from TX
			hdmi.tx_timing.colorimetry = Hdmi_GetColorimetry();
			hdmi.tx_timing.color = GET_HDMI_COLOR_SPACE();
			hdmi.tx_timing.depth =(HDMI_COLOR_DEPTH_T) drvif_Hdmi_GetColorDepth();
			HDMIRX_INFO("FSM_HDMI_DISPLAY_ON colorimetry(%u) color(%u) progressive(%u) depth(%u)",
								hdmi.tx_timing.colorimetry,hdmi.tx_timing.color,hdmi.tx_timing.progressive,hdmi.tx_timing.depth);

			Hdmi_Get3DGenTiming(&hdmi.tx_timing, &hdmi.gen_timing);

			//hdmi.gen_timing.depth = HDMI_COLOR_DEPTH_8B;

			Hdmi_DumpState();
			//if (hdmi_in(HDMI_HDMI_SR_VADDR) & _BIT6) { // if AVMute Set --> wait	AVMute Off
			if (hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC) & HDMI_SR_avmute_mask) {
				HDMI_PRINTF("#################### AV mute ####################\n");
				//cloud clear AV mute flag
				hdmi_rx_reg_mask32(HDMI_AVMCR, ~(HDMI_AVMCR_avmute_flag_mask), HDMI_AVMCR_avmute_flag_mask, HDMI_RX_MAC);
				hdmi_rx_reg_mask32(HDMI_AVMCR, ~(HDMI_AVMCR_avmute_flag_mask), 0, HDMI_RX_MAC);
				SET_HDMI_VIDEO_FSM(MAIN_FSM_HDMI_SETUP_VEDIO_PLL);	// reset Check mode state to initial
				return _MODE_DETECT;
			}
			hdmi_rx_reg_write32(HDMI_SR, hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC), HDMI_RX_MAC);	// this line is strange ,cloud need check

			#if 1//HDMI_FIX_GREEN_LINE
			hdmi_rx_reg_mask32(TMDS_OUT_CTRL, ~(TMDS_OUT_CTRL_tmds_bypass_mask), TMDS_OUT_CTRL_tmds_bypass(1), HDMI_RX_MAC);
			hdmi_rx_reg_write32(TMDS_ROUT, 0, HDMI_RX_MAC);
			hdmi_rx_reg_write32(TMDS_GOUT, 0, HDMI_RX_MAC);
			hdmi_rx_reg_write32(TMDS_BOUT, 0, HDMI_RX_MAC);
			if (GET_HDMI_COLOR_SPACE() != COLOR_RGB) {
				hdmi_rx_reg_mask32(TMDS_OUT_CTRL, ~(TMDS_OUT_CTRL_tmds_bypass_mask) ,TMDS_OUT_CTRL_tmds_bypass(0), HDMI_RX_MAC);
				hdmi_rx_reg_write32(TMDS_ROUT, 0x8000, HDMI_RX_MAC);
				hdmi_rx_reg_write32(TMDS_GOUT, 0x1000, HDMI_RX_MAC);
				hdmi_rx_reg_write32(TMDS_BOUT, 0x8000, HDMI_RX_MAC);
			}
			#endif

			if (hdmi.hw_hdmi_dma == HDMI_3D_OPMODE_DISABLE || hdmi.hw_hdmi_dma == HDMI_3D_OPMODE_HW_NODMA) {
				//mac3- hdmi_out(GET_VGIP_CHNx_DELAY_VADDR(), 0);
				SET_HDMI_VIDEO_FSM(MAIN_FSM_HDMI_WAIT_READY);  // reset Check mode state to initial
				//LCHECK					SET_HDMI_VIDEO_FSM(MAIN_FSM_HDMI_SETUP_VEDIO_PLL);	// reset Check mode state to initial
				//LCHECK					HDMI_DELAYMS(30);

			}else {
				SET_HDMI_VIDEO_FSM(MAIN_FSM_HDMI_WAIT_READY);  // reset Check mode state to initial
			}

			//HDMI_DELAYMS(100000);
			SET_IS_FIRST_CHECK_MODE(1);
			SET_DVI_AUDIO_PATH_SETUP(0);
			//LCHECK				hdmi_onms_measure(&hdmi.tx_timing, HDMI_MS_MODE_CONTINUOUS_INIT);
			drvif_Hdmi_AVI_RGB_Range();//cloud test

			//Enable rx wrapper detect
			yuv_fmt=hdmirx_wrapper_convert_color_fmt(hdmi.tx_timing.color);
			set_hdmirx_wrapper_control_0(-1,-1,-1,-1, yuv_fmt,-1);
			hdmi_ioctl_struct.detect_done = 1;
			set_hdmirx_wrapper_interrupt_en(0,0,1);
			return _MODE_SUCCESS;
		break;
		case MAIN_FSM_HDMI_MEASURE_ACTIVE_SPACE:
		break;
		case MAIN_FSM_HDMI_WAIT_READY:
		{
			hdmi.tx_timing.color = Hdmi_GetColorSpace();
			if ( hdmi.tx_timing.color != GET_HDMI_COLOR_SPACE() ) {
				SET_HDMI_COLOR_SPACE(hdmi.tx_timing.color);
				HDMI_PRINTF("Color Space Change = %d\n", hdmi.tx_timing.color);
				//return FALSE;
			};

			if (hdmi.resume) {
				SET_HDMI_VIDEO_FSM(MAIN_FSM_HDMI_MEASURE);	// reset Check mode state to initial
				return _MODE_DETECT;
			}

			if (hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC) & _BIT6)
				return _MODE_DETECT;
			//LCHECK			SET_HDMI_VIDEO_FSM(MAIN_FSM_HDMI_SETUP_VEDIO_PLL);	// reset Check mode state to initial
			return _MODE_SUCCESS;
		}
		break;

		default:
			SET_HDMI_VIDEO_FSM(MAIN_FSM_HDMI_SETUP_VEDIO_PLL);	// reset Check mode state to initial
		break;


	}
	return result;
}

HDMI_bool drvif_Hdmi_CheckMode(void)
{
	HDMIRX_IOCTL_STRUCT_T isr_info;

	HdmiSetAPStatus(HDMIRX_CHECK_MODE);

#if HDMI_INTERNAL_DEBUG
	Hdmi_CRC_check();
#endif

	if(GET_ISHDMI() == MODE_HDMI) {//HDMI mode
		// Check Vedio Format change
		if (Hdmi_GetColorSpace() != GET_HDMI_COLOR_SPACE()) {
			HDMI_PRINTF("Color Space Change\n");
			SET_HDMI_COLOR_SPACE(Hdmi_GetColorSpace());
			return FALSE;
		};

		Hdmi_AudioModeDetect();

		if (hdmi_rx_reg_read32(HDMI_SR, HDMI_RX_MAC) & _BIT6) {
			Hdmi_VideoOutputDisable();
			Hdmi_AudioOutputDisable();
			HDMI_PRINTF("AVMute ON----> Force re-detect signal\n");
			SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);
			SET_IS_FIRST_DETECT_MODE(1);
			return FALSE;
		}
	}
#if 1// TODO: Remove?
	if (GET_HDMI_ISINTERLACE() != Hdmi_GetInterlace(HDMI_MS_MODE_ONESHOT)) {   // if interlace mode change
		Hdmi_VideoOutputDisable();
		Hdmi_AudioOutputDisable();
		HDMI_PRINTF("IsInterlaced change\n");
		SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);
		SET_IS_FIRST_DETECT_MODE(1);
		return FALSE;
	}
#endif
	if (GET_ISHDMI()!= IsHDMI()) {
		Hdmi_VideoOutputDisable();
		Hdmi_AudioOutputDisable();
		HDMI_PRINTF("bIsHDMIDVI change\n");
		SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);
		SET_IS_FIRST_DETECT_MODE(1);
		return FALSE;
	}

	if ((unsigned int)GET_HDMI_CD()!= (hdmi_rx_reg_read32(TMDS_DPC0, HDMI_RX_MAC) & 0xf)) {
		Hdmi_VideoOutputDisable();
		Hdmi_AudioOutputDisable();
		HDMI_PRINTF("bIsHDMIDVI change\n");
		SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);
		SET_IS_FIRST_DETECT_MODE(1);
		return FALSE;
	}

	HdmiGetStruct(&isr_info);
	if (ABS(hdmi.b, isr_info.b) > 4) {
		Hdmi_VideoOutputDisable();
		Hdmi_AudioOutputDisable();
		HDMI_PRINTF("isr_info.b change\n");
		SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);
		SET_IS_FIRST_DETECT_MODE(1);
		return FALSE;
	};

	if (isr_info.b_change) {
		Hdmi_VideoOutputDisable();
		Hdmi_AudioOutputDisable();
		HDMI_PRINTF("b change = %d \n",isr_info.b);
		SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);
		SET_IS_FIRST_DETECT_MODE(1);
		return FALSE;
	}

	if (hdmi.resume) {
		hdmi.resume = 0;
		HDMI_PRINTF("hdmi user resume\n");
		Hdmi_VideoOutputDisable();
		Hdmi_AudioOutputDisable();
		SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);
		SET_IS_FIRST_DETECT_MODE(1);
		return FALSE;
	}
	return TRUE;
}


void drvif_Hdmi_InitSrc(unsigned char channel)
{
	HDMIRX_INFO("[%s] ",__FUNCTION__);

	SET_HDMI_CHANNEL(channel); /* before hdmi_power() */

	hdmi_phy_port_select(HDMI_CHANNEL0, 1);

	Hdmi_Power(1);// HDMI power on , exit power saving mode

	Hdmi_TmdsInit();

	Hdmi_MacInit();	

	// setup Audio PLL
	Hdmi_AudioOutputDisable();

	// reset hdmi struct
	SET_HDMI_BOOT_FIRST_RUN(0);
	SET_IS_FIRST_DETECT_MODE(1);
	SET_HDMI_COLOR_SPACE(COLOR_RGB);
	SET_HDMI_ISINTERLACE(0);
	SET_ISHDMI(MODE_DVI);
	SET_HDMI_CD(0);
	SET_HDMI_CHANNEL(HDMI_CHANNEL0);
	// reset FSM
	SET_HDMI_VIDEO_FSM(MAIN_FSM_HDMI_SETUP_VEDIO_PLL);
	SET_HDMI_AUDIO_FSM(AUDIO_FSM_AUDIO_START);

	hdmi.b = 0;
	hdmi.tx_timing.hdmi_3dformat = HDMI3D_2D_ONLY;
	hdmi.vsync_cnt = 0;

}


void drvif_Hdmi_Release(void)
{
	HDMIRX_DEBUG("[%s] ",__FUNCTION__);

	SET_IS_FIRST_DETECT_MODE(1);
	HdmiSetAPStatus(HDMIRX_RELEASED);

	Hdmi_AudioOutputDisable();
	Hdmi_VideoOutputDisable();

	Hdmi_Power(0);

#if HDMI2p0
	hdmi_rx_reg_mask32(SCDC_CR, (~SCDC_CR_scdc_reset_mask), SCDC_CR_scdc_reset_mask, HDMI_RX_MAC);
	hdmi_rx_reg_mask32(SCDC_CR, (~SCDC_CR_scdc_reset_mask), 0, HDMI_RX_MAC);
#endif

}


void drvif_Hdmi_Init(void)
{
	HDMIRX_INFO("[%s] ",__FUNCTION__);

	hdmi.hw_dither12x10_enable = 0;
	hdmi.hw_hdmi_dma = HDMI_3D_OPMODE_HW;

	hdmi.fast_boot_source = -1;
	hdmi.force_all_3d_disable = 1;
	hdmi.force_3dto2d_enable = 0;
	hdmi.force_2dto3d_enable = 0;
	hdmi.force_2dto3d_mode = HDMI3D_2D_ONLY;
	hdmi.hdmi3d_capability = 0;
	hdmi.enable_3ddma = 1;

	hdmi.path_policy = HDMI_PATH_POLICY_2D_ONLY;

	Hdmi_PhyInit();
	Hdmi_HdcpInit();
}


