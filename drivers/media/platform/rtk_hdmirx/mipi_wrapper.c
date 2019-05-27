#include "v4l2_hdmi_dev.h"
#include "mipi_wrapper.h"
#include "hdmirx_reg.h"

#include "rx_drv/hdmiInternal.h"

MIPI_TOP_INFO mipi_top;
int hdmi_isr_count = 0;

/*=================== extern Variable/Function ===================*/
extern HDMI_INFO_T hdmi;
extern HDMI_SW_BUF_CTL hdmi_sw_buf_ctl;
extern int hdmi_stream_on;

extern HDMI_ERR_T Hdmi_CheckConditionChange(void);
extern void hdmirx_wrapper_isr(void);
extern void set_hdmirx_wrapper_control_0(int fifo_stage,int fw_dma_en,int polarity_set_en,int polarity_set,int yuv_fmt,int hdmirx_en);
extern void restartHdmiRxWrapperDetection(void);
#ifdef CONFIG_RTK_HDCPRX_2P2
extern void  Hdmi_HDCP_2_2_msg_hander(void);
#endif
/*======================================================*/

void mipi_reset_work_func(struct work_struct *work)
{
	struct reset_control *reset_mipi;
	reset_mipi = rstc_get("rstn_mipi");
	reset_control_assert(reset_mipi);
	reset_control_deassert(reset_mipi);
}

void stop_mipi_process(void)
{
	set_enable_mipi(0);
	set_hdmirx_wrapper_control_0(-1, 0,-1,-1,-1,-1);//Stop DMA
	schedule_work(&mipi_top.mipi_reset_work);//Reset mipi clock

	atomic_set(&hdmi_sw_buf_ctl.read_index, 0);
    atomic_set(&hdmi_sw_buf_ctl.write_index, 0);
    atomic_set(&hdmi_sw_buf_ctl.fill_index, 0);
    hdmi_sw_buf_ctl.use_v4l2_buffer = 0;
    hdmi_stream_on = 0;
    mipi_top.mipi_init = 0;
}

void set_mipi_env(void)
{
	HDMIRX_DEBUG("[%s]",__FUNCTION__);

	// on 1195
	hdmi_rx_reg_write32(MIPI_APHY_REG15, MIPI_APHY_REG15_SEL_DIV_RG(DEMUX16), HDMI_RX_MIPI_PHY);//SEL_DIV_RG
	hdmi_rx_reg_write32(MIPI_DPHY_REG4, MIPI_DPHY_REG4_SL_RG(fourlane) | MIPI_DPHY_REG4_Sw_rst(disable), HDMI_RX_MIPI_PHY);//Lane mux sel & soft reset

	// 4 lane
	hdmi_rx_reg_write32(MIPI_DPHY_PWDB, MIPI_DPHY_PWDB_L3_pwdb(enable) | MIPI_DPHY_PWDB_L2_pwdb(enable) | 
										MIPI_DPHY_PWDB_L1_pwdb(enable) | MIPI_DPHY_PWDB_L0_pwdb(enable), 
										HDMI_RX_MIPI_PHY);//Lane1 & Lane2 power on

	hdmi_rx_reg_write32(MIPI_DPHY_REG0, MIPI_DPHY_REG0_Div_sel(div_bit_16) | MIPI_DPHY_REG0_Lane3_en(enable) | 
										MIPI_DPHY_REG0_Lane2_en(enable) | MIPI_DPHY_REG0_Lane1_en(enable) | 
										MIPI_DPHY_REG0_Lane0_en(enable), HDMI_RX_MIPI_PHY);//16bit data width, Lane1 & Lane2 enable

	hdmi_rx_reg_write32(MIPI_DPHY_REG9, MIPI_DPHY_REG9_Lane3_clk_edge(posedge) | MIPI_DPHY_REG9_Lane2_clk_edge(posedge) |
										MIPI_DPHY_REG9_Lane1_clk_edge(posedge) | MIPI_DPHY_REG9_Lane0_clk_edge(posedge), HDMI_RX_MIPI_PHY);//Lane CLK edge

	hdmi_rx_reg_write32(MIPI_DPHY_REG1, MIPI_DPHY_REG1_yuv_src_sel(SEL_UYVY) | MIPI_DPHY_REG1_dec_format(YUV422_dec_format), HDMI_RX_MIPI_PHY);// SEL_UYVY //SEL_YUYV & Dec data format for YUV422

	hdmi_rx_reg_write32(MIPI_DPHY_REG2, MIPI_DPHY_REG2_D_type(DTYPE_YUY8), HDMI_RX_MIPI_PHY);//MIPI data type YUV422 8bit

	hdmi_rx_reg_write32(MIPI_DPHY_REG6	, MIPI_DPHY_REG6_Dvld_lane1(0x6) | MIPI_DPHY_REG6_Dvld_lane0(0x6), HDMI_RX_MIPI_PHY); 	   // 0x6
	hdmi_rx_reg_write32(MIPI_DPHY_REG7	, MIPI_DPHY_REG7_Dvld_lane3(0x6) | MIPI_DPHY_REG7_Dvld_lane2(0x6), HDMI_RX_MIPI_PHY);

}

unsigned int rx_pitch_measurement(unsigned int output_h, MIPI_OUT_COLOR_SPACE_T output_color)
{
	unsigned int pitch, pitch_factor;
	HDMIRX_INFO("rx_pitch_measurement,output_color=%x\n",output_color);

	switch(output_color)
	{
		case OUT_8BIT_YUV422:
		case OUT_8BIT_YUV420:
			pitch_factor = 1;
			break;
		default: //all other RGB case
			pitch_factor = 4;
			break;
	}

	pitch = output_h * pitch_factor;
	// pitch has to be n*16 byte
	if(output_color < OUT_ARGB)
		pitch = roundup16(pitch);
	return pitch;
}

void set_video_DDR_start_addr(struct v4l2_hdmi_dev *dev)
{
	unsigned int addr1y, addr1uv, addr2y, addr2uv;
	unsigned int offset;
	unsigned long flags = 0;
	struct hdmi_dmaqueue *hdmidq = &dev->hdmidq;

	offset = roundup16(mipi_top.pitch) * roundup16(mipi_top.v_output_len);

	spin_lock_irqsave(&dev->slock, flags);
	hdmidq->hwbuf[0] = list_entry(hdmidq->active.next, struct hdmi_buffer, list);
	list_del(&hdmidq->hwbuf[0]->list);
	//pr_info("%s: del buf[%d] from queue\n", __func__, hdmidq->hwbuf[0]->vb.v4l2_buf.index);
	hdmidq->hwbuf[1] = list_entry(hdmidq->active.next, struct hdmi_buffer, list);
	list_del(&hdmidq->hwbuf[1]->list);
	//pr_info("%s: del buf[%d] from queue\n", __func__, hdmidq->hwbuf[1]->vb.v4l2_buf.index);
	atomic_sub(2, &hdmidq->qcnt);

	addr1y = hdmidq->hwbuf[0]->phys;
	addr1uv = hdmidq->hwbuf[0]->phys + offset;
	addr2y = hdmidq->hwbuf[1]->phys;
	addr2uv = hdmidq->hwbuf[1]->phys + offset;
	set_video_dest_addr(addr1y, addr1uv, addr2y, addr2uv);
	spin_unlock_irqrestore(&dev->slock, flags);

	HDMIRX_INFO("[%s] addr1y(0x%08x) addr1uv(0x%08x) addr2y(0x%08x) addr2uv(0x%08x)",
				__FUNCTION__,addr1y,addr1uv,addr2y,addr2uv);
}

void set_mipi_type(unsigned int type)
{
	unsigned int reg_val;

	HDMIRX_DEBUG("[%s] type(%u)",__FUNCTION__,type);

	reg_val = hdmi_rx_reg_read32(MIPI_TYPE, HDMI_RX_MIPI);

	reg_val = reg_val & (~MIPI_TYPE_mipi_type_mask);

	hdmi_rx_reg_write32(MIPI_TYPE, reg_val | MIPI_TYPE_mipi_type(type), HDMI_RX_MIPI);
}

void set_pic_dest_addr(int addry, int addruv)
{
	if(addry >= 0)
	{
		hdmi_rx_reg_write32(MIPI_SA2, addry, HDMI_RX_MIPI);
	}
	if(addruv >= 0)
	{
		hdmi_rx_reg_write32(MIPI_SA2_UV, addruv, HDMI_RX_MIPI);
	}
}

void set_pic_dest_size(unsigned int dst_width, unsigned int pitch)
{

	hdmi_rx_reg_write32(MIPI_SIZE1, MIPI_SIZE1_dst_width_pic(dst_width) | MIPI_SIZE1_pitch_pic(pitch), HDMI_RX_MIPI);
}

void set_video_dest_addr(int addr1y, int addr1uv, int addr2y, int addr2uv)
{
	if(addr1y >= 0)
		hdmi_rx_reg_write32(MIPI_SA0, addr1y, HDMI_RX_MIPI);

	if(addr1uv >= 0)
		hdmi_rx_reg_write32(MIPI_SA0_UV, addr1uv, HDMI_RX_MIPI);

	if(addr2y >= 0)
		hdmi_rx_reg_write32(MIPI_SA1, addr2y, HDMI_RX_MIPI);

	if(addr2uv >= 0)
		hdmi_rx_reg_write32(MIPI_SA1_UV, addr2uv, HDMI_RX_MIPI);
}

void set_video_dest_size(unsigned int dst_width, unsigned int pitch)
{

	hdmi_rx_reg_write32(MIPI_SIZE0, MIPI_SIZE0_dst_width_video(dst_width) | MIPI_SIZE0_pitch_video(pitch), HDMI_RX_MIPI);
}

void set_video_src_size(unsigned int src_width)
{
	unsigned int reg_val;

	reg_val = hdmi_rx_reg_read32(MIPI_SIZE2, HDMI_RX_MIPI);
	reg_val = reg_val & (~MIPI_SIZE2_src_width_video_mask);
	hdmi_rx_reg_write32(MIPI_SIZE2, reg_val | MIPI_SIZE2_src_width_video(src_width), HDMI_RX_MIPI);
}

void set_init_mipi_value(MIPI_REG *mipi_reg)
{
	unsigned int reg_val = 0;

	reg_val = (mipi_reg->dst_fmt << 27) |
					(mipi_reg->src_fmt << 25) |
					(mipi_reg->src_sel << 24) |
					(mipi_reg->yuv420_uv_seq << 23) |
					(mipi_reg->vs << 22) |
					(mipi_reg->vs_near << 21) |
					(mipi_reg->vs_yodd << 20) |
					(mipi_reg->vs_codd << 19) |
					(mipi_reg->hs << 18) |
					(mipi_reg->hs_yodd << 17) |
					(mipi_reg->hs_codd << 16) |
					(mipi_reg->yuv420_fmt << 15) |
					(mipi_reg->ccs_data_format << 13) |
					(mipi_reg->yuv_to_rgb << 12) |
					(mipi_reg->chroma_ds_mode << 11) |
					(mipi_reg->chroma_ds_en << 10) |
					(mipi_reg->chroma_us_mode << 9) |
					(mipi_reg->chroma_us_en << 8) |
					(mipi_reg->hdmirx_interlace_en << 7) |
					(mipi_reg->hdmirx_interlace_polarity << 6) |
					(mipi_reg->int_en4 << 5) |
					(mipi_reg->int_en3 << 4) |
					(mipi_reg->int_en2 << 3) |
					(mipi_reg->int_en1 << 2) |
					(mipi_reg->int_en0 << 1) |
					(mipi_reg->en);

	hdmi_rx_reg_write32(MIPI, reg_val, HDMI_RX_MIPI);
	HDMIRX_INFO("Set MIPI=0x%08x",reg_val);
	HDMIRX_INFO("Read MIPI=0x%08x",hdmi_rx_reg_read32(MIPI, HDMI_RX_MIPI));
}

void set_enable_mipi(unsigned char enable)
{
	hdmi_rx_reg_mask32(MIPI, ~(MIPI_int_en0_mask|MIPI_int_en1_mask|MIPI_int_en2_mask|MIPI_int_en3_mask|MIPI_int_en4_mask|MIPI_en_mask),
							MIPI_int_en0(enable)|MIPI_int_en1(enable)|MIPI_int_en2(enable)|MIPI_int_en3(enable)|MIPI_int_en4(enable)|MIPI_en(enable),
							HDMI_RX_MIPI);

}

void set_hs_scaler(unsigned int hsi_offset, unsigned int hsi_phase, unsigned int hsd_out, unsigned int hsd_delta)
{
	hdmi_rx_reg_write32(SCALER_HSI, SCALER_HSI_hsi_offset(hsi_offset) | 
									SCALER_HSI_hsi_phase(hsi_phase), HDMI_RX_MIPI);

	hdmi_rx_reg_write32(SCALER_HSD, SCALER_HSD_hsd_out(hsd_out) |
									SCALER_HSD_hsd_delta(hsd_delta), HDMI_RX_MIPI);

	HDMIRX_INFO("hsd_out=0x%x,hsd_delta=0x%x",hsd_out,hsd_delta);
}

void set_hs_coeff(void)
{
	unsigned int reg_val;

	//for Y
	reg_val = (0x400<<16) | 0x400 ;
	hdmi_rx_reg_write32(SCALER_HSYNC0, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_HSYNC1, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_HSYNC2, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_HSYNC3, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_HSYNC4, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_HSYNC5, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_HSYNC6, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_HSYNC7, reg_val, HDMI_RX_MIPI);

	//for U,V
	hdmi_rx_reg_write32(SCALER_HSCC0, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_HSCC1, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_HSCC2, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_HSCC3, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_HSCC4, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_HSCC5, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_HSCC6, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_HSCC7, reg_val, HDMI_RX_MIPI);
}

void set_vs_scaler(unsigned int vsi_offset, unsigned int vsi_phase, unsigned int vsd_out, unsigned int vsd_delta)
{
	hdmi_rx_reg_write32(SCALER_VSI, SCALER_VSI_vsi_offset(vsi_offset) | 
									SCALER_VSI_vsi_phase(vsi_phase), HDMI_RX_MIPI);

	hdmi_rx_reg_write32(SCALER_VSD, SCALER_VSD_vsd_out(vsd_out) |
									SCALER_VSD_vsd_delta(vsd_delta), HDMI_RX_MIPI);

	HDMIRX_INFO("vsd_out=0x%x,vsd_delta=0x%x\n",vsd_out,vsd_delta);
}

void set_vs_coeff(void)
{
	unsigned int reg_val;

	reg_val = (0x800<<16) | 0x800 ;
	hdmi_rx_reg_write32(SCALER_VSYC0, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_VSCC0, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_VSYC1, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_VSCC1, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_VSYC2, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_VSCC2, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_VSYC3, reg_val, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(SCALER_VSCC3, reg_val, HDMI_RX_MIPI);
}

void set_alpha(unsigned int alpha)
{

	hdmi_rx_reg_write32(CONSTANT_ALPHA, CONSTANT_ALPHA_alpha(alpha), HDMI_RX_MIPI);
}

void set_YUV2RGB_coeff(void)
{
	hdmi_rx_reg_write32(CS_TRANS0,  0x04a80, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(CS_TRANS1,  0x00000, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(CS_TRANS2,  0x072c0, HDMI_RX_MIPI);

	hdmi_rx_reg_write32(CS_TRANS3,  0x04a80, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(CS_TRANS4,  0x1f260, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(CS_TRANS5,  0x1ddd0, HDMI_RX_MIPI);

	hdmi_rx_reg_write32(CS_TRANS6,  0x04a80, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(CS_TRANS7,  0x08760, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(CS_TRANS8,  0x00000, HDMI_RX_MIPI);

	hdmi_rx_reg_write32(CS_TRANS9,  0x0fc20, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(CS_TRANS10, 0x00134, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(CS_TRANS11, 0x0fb7c, HDMI_RX_MIPI);
}

void set_RGB2YUV_coeff(void)
{
	//C1~C3
	hdmi_rx_reg_write32(CS_TRANS0,	0x00BB8, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(CS_TRANS1,	0x02748, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(CS_TRANS2,	0x003F8, HDMI_RX_MIPI);
	//C4~C6
	hdmi_rx_reg_write32(CS_TRANS3,	0x1F988, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(CS_TRANS4,	0x1EA60, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(CS_TRANS5,	0x01C18, HDMI_RX_MIPI);
	//C7~C9
	hdmi_rx_reg_write32(CS_TRANS6,	0x01C18, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(CS_TRANS7,	0x1E6E9, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(CS_TRANS8,	0x1FD7B, HDMI_RX_MIPI);
	//K1~K3
	hdmi_rx_reg_write32(CS_TRANS9,	0x00100, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(CS_TRANS10, 0x00800, HDMI_RX_MIPI);
	hdmi_rx_reg_write32(CS_TRANS11, 0x00800, HDMI_RX_MIPI);
}

void mipi_scale_down(unsigned int src_width,unsigned int src_height,unsigned int dst_width,unsigned int dst_height)
{
	unsigned int delta_num, delta_den, offset, phase;

	if( HDMI_VCR_get_eot(hdmi_rx_reg_read32(HDMI_VCR, HDMI_RX_MAC)) ) // interlace mode
	   dst_height = (dst_height>>1);

	if(src_width > dst_width)
	{
		//set hs_scaler
		offset = 0;
		phase = 0;
		delta_num = (src_width / dst_width) << 14;
		delta_den = ((src_width % dst_width)*0x4000) / dst_width ;
		set_hs_scaler(offset,phase,dst_width, (delta_num | delta_den));//offset,phase,out,delta
		set_hs_coeff();
	}
	if(src_height > dst_height)
	{
		//set vs_scaler
		offset = 0;
		phase = 0;
		delta_num = (src_height / dst_height) << 14;
		delta_den = ((src_height % dst_height)*0x4000) / dst_height ;
		set_vs_scaler(offset,phase,dst_height, (delta_num | delta_den));//offset,phase,out,delta
		set_vs_coeff();
	}
}

void setup_mipi(void)
{
	MIPI_REG mipi_reg;
	HDMIRX_INFO("[%s]",__FUNCTION__);

	set_mipi_env();
	set_mipi_type(0);//0:video, 1:pic

	memset(&mipi_reg, 0, sizeof(MIPI_REG));

	mipi_reg.dst_fmt = mipi_top.output_color;
	mipi_reg.src_fmt = mipi_top.input_color;
	mipi_reg.src_sel = mipi_top.src_sel;
	mipi_reg.yuv420_uv_seq = mipi_top.uv_seq;

	if(mipi_top.v_input_len > mipi_top.v_output_len)
		mipi_reg.vs = 1;

	if(mipi_top.h_input_len > mipi_top.h_output_len)
		mipi_reg.hs = 1;

	//mipi_reg.yuv420_fmt
	//mipi_reg.ccs_data_format

	if((mipi_top.input_color!=IN_RGB888)&&(mipi_top.output_color>=OUT_ARGB))//YUV -> RGB
	{
		mipi_reg.yuv_to_rgb = 1;
		set_alpha(0xff);
		set_YUV2RGB_coeff();
	}
	else if((mipi_top.input_color==IN_RGB888)&&(mipi_top.output_color<=OUT_10BIT_YUV420))//RGB -> YUV
	{
		mipi_reg.chroma_ds_mode = 0;//0:drop 1:avg
		mipi_reg.chroma_ds_en = 1;
		mipi_reg.yuv_to_rgb = 1;
		set_RGB2YUV_coeff();
	}

	if((mipi_top.input_color == IN_YUV444)&&(mipi_top.output_color <= OUT_10BIT_YUV420))
	{
		mipi_reg.chroma_ds_mode = 0;//0:drop 1:avg
		mipi_reg.chroma_ds_en = 1;
	}

	if((mipi_top.input_color == IN_YUV422)&&(mipi_reg.vs || mipi_reg.hs))
	{
		mipi_reg.chroma_us_mode = 0;//0:repeat 1:avg
		mipi_reg.chroma_us_en = 1;
		if(mipi_top.output_color <= OUT_10BIT_YUV420)
		{
			mipi_reg.chroma_ds_mode = 0;//0:drop 1:avg
			mipi_reg.chroma_ds_en = 1;
		}
	}

	mipi_reg.hdmirx_interlace_en = (!hdmi.tx_timing.progressive);
	mipi_reg.hdmirx_interlace_polarity = 0;

	mipi_reg.int_en4 = 1;
	mipi_reg.int_en3 = 1;
	mipi_reg.int_en2 = 1;
	mipi_reg.int_en1 = 1;
	mipi_reg.int_en0 = 1;

	mipi_reg.en = 1;

	set_video_dest_size(mipi_top.h_output_len,mipi_top.pitch);
	set_video_src_size(mipi_top.h_input_len);
	mipi_scale_down(mipi_top.h_input_len,mipi_top.v_input_len,mipi_top.h_output_len,mipi_top.v_output_len);

	hdmi_rx_reg_write32(MIPI_DPHY_REG11, MIPI_DPHY_REG11_Esc_lane3_int_flg(1)|
										MIPI_DPHY_REG11_Esc_lane2_int_flg(1)|
										MIPI_DPHY_REG11_Esc_lane1_int_flg(1)|
           								MIPI_DPHY_REG11_Esc_lane0_int_flg(1)|
           								MIPI_DPHY_REG11_Clk_ulps_int_flg(1)|
           								MIPI_DPHY_REG11_Crc16_err_int_flg(1)|
           								MIPI_DPHY_REG11_Ecc_crt_int_flg(1)|
           								MIPI_DPHY_REG11_Ecc_err_int_flg(1), HDMI_RX_MIPI_PHY);

	set_init_mipi_value(&mipi_reg);
	set_hdmirx_wrapper_control_0(-1, 1, mipi_reg.hdmirx_interlace_en, mipi_reg.hdmirx_interlace_polarity,-1,-1);
	mipi_top.mipi_init = 1;
}

void update_mipi_hw_buffer(int src_index, struct v4l2_hdmi_dev *dev)
{
	unsigned int addry, addruv, offset;
    unsigned long flags = 0;
    struct hdmi_dmaqueue *hdmidq = &dev->hdmidq;

	offset = roundup16(mipi_top.pitch) * roundup16(mipi_top.v_output_len);

    spin_lock_irqsave(&dev->slock, flags);

    vb2_buffer_done(&hdmidq->hwbuf[src_index]->vb,VB2_BUF_STATE_DONE);
    atomic_inc(&hdmidq->rcnt);

    hdmidq->hwbuf[src_index] = list_entry(hdmidq->active.next, struct hdmi_buffer, list);
    list_del(&hdmidq->hwbuf[src_index]->list);
    atomic_dec(&hdmidq->qcnt);

    spin_unlock_irqrestore(&dev->slock, flags);

    addry = hdmidq->hwbuf[src_index]->phys;
    addruv = hdmidq->hwbuf[src_index]->phys + offset;

    if(src_index == 0)
        set_video_dest_addr(addry, addruv, -1, -1);
    else
        set_video_dest_addr(-1, -1, addry, addruv);
}

void hdmi_hw_buf_update(int src_index, struct v4l2_hdmi_dev *dev)
{
	static int skip_num = 0;
    static int total_num = 0;
    static unsigned long prev_jif = 0;
    int empty = 0;
    unsigned long flags = 0;
    struct hdmi_dmaqueue *hdmidq = &dev->hdmidq;

    if(hdmi_sw_buf_ctl.pre_frame_done == -1)
          hdmi_sw_buf_ctl.pre_frame_done = src_index;

    hdmi_isr_count++;
    if((mipi_top.src_sel == 1) && (Hdmi_CheckConditionChange() != HDMI_ERR_NO))
    {
        HDMIRX_INFO("HDMI Condition Change. Drop frame]");
        restartHdmiRxWrapperDetection();
        return;
    }

    if ((hdmi_sw_buf_ctl.use_v4l2_buffer == 0)
        ||(mipi_top.mipi_init == 0))
    {
        //pr_info("[HW UPDATE R]\n");
        return;
    }

    if(unlikely((jiffies - prev_jif) >= (10 * HZ))){
        HDMIRX_INFO("skip %d/%d in %lu jiffies\n", skip_num, total_num, jiffies - prev_jif);
        prev_jif = jiffies;
        total_num = 0;
        skip_num = 0;
    }
    total_num++;

    spin_lock_irqsave(&dev->slock, flags);
    empty = list_empty(&hdmidq->active);
    spin_unlock_irqrestore(&dev->slock, flags);
    if(empty){
        //pr_notice_ratelimited("No active queue to serve ?????? %d-2-%d\n", atomic_read(&hdmidq->rcnt), atomic_read(&hdmidq->qcnt));
        skip_num++;
        return;
    }
    update_mipi_hw_buffer(src_index, dev);
}

irqreturn_t hdmirx_mipi_isr(int irq, void* dev_id)
{
	unsigned int st_reg_val, reg_val;
	struct v4l2_hdmi_dev *dev = dev_id;

	hdmirx_wrapper_isr();

#ifdef CONFIG_RTK_HDCPRX_2P2
	Hdmi_HDCP_2_2_msg_hander();

	// Return isr if dma not enabled
	if( !HDMIRX_WRAPPER_CONTROL_0_get_fw_dma_en(hdmi_rx_reg_read32(HDMIRX_WRAPPER_CONTROL_0, HDMI_RX_HDMI_WRAPPER)) )
		return IRQ_HANDLED;
#endif

	st_reg_val = hdmi_rx_reg_read32(MIPI_INT_ST, HDMI_RX_MIPI);

    if(st_reg_val & 0x30) // Buffer is overflow or one frame is dropped
    {
        //clear interrupt status
        hdmi_rx_reg_write32(MIPI_INT_ST, st_reg_val, HDMI_RX_MIPI);

		HDMIRX_INFO("MIPI frame dropped");

        return IRQ_HANDLED;
    }

    reg_val = hdmi_rx_reg_read32(MIPI_TYPE, HDMI_RX_MIPI);
    if(reg_val & 0x01) // Picture mode
    {
        if(st_reg_val & 0x08) // fm_done2:One image is written to mipi_sa2
        {
            //Clear interrupt status
            hdmi_rx_reg_write32(MIPI_INT_ST, st_reg_val, HDMI_RX_MIPI);
        }
    }
    else // Preview or video mode
    {
        //Clear interrupt status
        hdmi_rx_reg_write32(MIPI_INT_ST, st_reg_val, HDMI_RX_MIPI);
        if(st_reg_val & 0x02)// fm_done0:One image is written to mipi_index0/mipi_sa0
        {
            hdmi_hw_buf_update(0,dev);
        }
        else if((st_reg_val & 0x04))// fm_done1:One image is written to mipi_index1/mipi_sa1
        {
            hdmi_hw_buf_update(1,dev);
        }
    }

	return IRQ_HANDLED;
}

