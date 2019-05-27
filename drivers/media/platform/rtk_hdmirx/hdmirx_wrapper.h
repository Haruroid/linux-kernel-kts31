#ifndef __HDMIRX_WRAPPER_H__
#define __HDMIRX_WRAPPER_H__

#define HDMI_MIN_SUPPORT_H_PIXEL    0x40

typedef struct  {
    unsigned int width;
    unsigned int height;
    unsigned int fps;
	unsigned int interlace;
}HDMI_VIC_TABLE_T;

void set_hdmirx_wrapper_interrupt_en(int ver_err_en,int hor_err_en,int polarity_detect_en);
void set_hdmirx_wrapper_active_pixels(unsigned v_pixel, unsigned h_pixel);
void set_hdmirx_wrapper_hor_threshold(unsigned int h_threshold);
unsigned int hdmirx_wrapper_get_active_line(void);
unsigned int hdmirx_wrapper_get_active_pixel(void);
void set_hdmirx_wrapper_control_0(int fifo_stage,int fw_dma_en,int polarity_set_en,int polarity_set,int yuv_fmt,int hdmirx_en);
void restartHdmiRxWrapperDetection(void);
void hdmirx_wrapper_isr(void);
void hdmi_related_wrapper_init(void);
#endif//__HDMIRX_WRAPPER_H__