#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/ioctl.h> /* needed for the _IOW etc stuff used later */
#include <linux/syscalls.h> /* needed for the _IOW etc stuff used later */
#include <linux/mpage.h>
#include <linux/dcache.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <sound/asound.h>
#include <asm/cacheflush.h>
#include <linux/slab.h>

#include <linux/power-control.h>
#include <linux/reset-helper.h> // rstc_get
#include <linux/reset.h>
#include <linux/clkdev.h>  // clk_get
#include <linux/clk-provider.h>

#include <soc/realtek/kernel-rpc.h>
#include <linux/dma-mapping.h>
#include "hdmitx_rpc.h"
#include <dc2vo/dc_rpc.h>
#include "hdmitx.h"
#include "hdmitx_dev.h"
#include "hdmitx_reg.h"
#include "rtk_edid.h"
#include "crt_reg.h"
#include "hdmitx_scdc.h"

#ifdef CONFIG_RTK_MHLTX
#include "mhltx_dev.h"
#include "mhltx_cbus.h"
#endif

#ifdef USE_ION_AUDIO_HEAP
#include "uapi/ion.h"
#include "ion/ion.h"
#include "uapi/rtk_phoenix_ion.h"

struct ion_client *rpc_ion_client;
extern struct ion_device *rtk_phoenix_ion_device;
#endif

extern struct edid_info hdmitx_edid_info;

enum HDMI_AVMUTE{
	HDMI_CLRAVM =0,
	HDMI_SETAVM 
};

#define CONVERT_FOR_AVCPU(x)		((unsigned int)(x) | 0xA0000000)

#if 0
#define VO_RESOLUTION (0x18070000 + 0x1100 ) // VO_RESOLUTION_INFO

typedef struct{
    unsigned int magic;  //0x11223344 means valid data
    unsigned int width;  //vo resolution: width
    unsigned int height; //vo resolution: height
} VO_RESOLUTION_INFO ;

static  VO_RESOLUTION_INFO * vo_res_info = NULL;
#endif

static int hdmi_error;
unsigned int tmds_en;

void hdmitx_print_sink_info(asoc_hdmi_t *p_this);

u32 check_hdmi_mhl_mode(void)
{	
	void __iomem * vaddr;
	u32 ret;
	vaddr = ioremap(SYS_DISP_PLL_DIV2_reg, 0x4);

	//0: HDMI mode , 1:  MHL mode
	ret = SYS_DISP_PLL_DIV2_get_sel_pllhdmi_mhl(rd_reg(vaddr));
	HDMI_INFO("Mode = %s",ret?"MHL":"HDMI");

	iounmap(vaddr);
	return ret;
}

int hdmitx_check_rx_sense(void __iomem *reg_base)
{
	u32 val ;

	val = HDMI_INTST_get_riupdated(rd_reg(reg_base + HDMI_INTST));

	HDMI_INFO("rxupdated=%u",val);
				
	if(val)
		return HDMI_RX_SENSE_ON;
	else
		return HDMI_RX_SENSE_OFF;
}

int hdmitx_check_tmds_src(void __iomem *reg_base)
{
	u32 val,data;

    if(tmds_en ==1)
	{
		val = rd_reg(reg_base + HDMI_CR);
		HDMI_INFO("CR=%x",val);
		
		if( val == 0x15)
			return TMDS_HDMI_ENABLED;
		else if (val == 0x0)
			return TMDS_HDMI_DISABLED;
		else if (val == 0x11)
			return TMDS_MHL_ENABLED;
		else
			return TMDS_MODE_UNKNOW;
	}
	else
	{
	#if 0//TODO
		val = rd_reg(reg_base + TMDS_SCR2);
		data = TMDS_SCR2_REG_TX_PU_src(val);
		HDMI_INFO("TMDS_SCR2=%x",val);
		
		if( data == 0xF)
			return TMDS_HDMI_ENABLED;
		else if (data == 0x0)
			return TMDS_HDMI_DISABLED;
		else if (data == 0x2)
			return TMDS_MHL_ENABLED;
		else
			return TMDS_MODE_UNKNOW;
	#else
		return TMDS_HDMI_ENABLED;
	#endif
	
	}
		
}

void hdmitx_turn_off_tmds(int vo_mode)
{
	void __iomem * vaddr;
	
	if(!tmds_en)
	{
		HDMI_INFO("skip %s",__FUNCTION__);
		return;
	}

	vaddr = ioremap(HDMI_CR_reg, 0x4);

	#if 0//TODO
	if(vo_mode == HDMI_MODE_MHL)
		CLEARBITS(base, TMDS_SCR3,TMDS_SCR3_REG_EN_MHL_mask);
	#endif

	wr_reg(vaddr, 0x2A);
	HDMI_INFO("CR OFF = %x",rd_reg(vaddr));
	iounmap(vaddr);
}

int hdmitx_send_AVmute(void __iomem *reg_base, int flag)
{
	void __iomem * vaddr;
	unsigned char pll_hdmi;
	struct clk *clk_hdmitx;
	struct reset_control *reset_hdmitx;

	// Skip AV mute if HDMI clock not enable
	vaddr = ioremap(0x98000190, 0x4);//PLL_HDMI
	pll_hdmi = rd_reg(vaddr)&0xBF;
	iounmap(vaddr);
	clk_hdmitx = clk_get(NULL, "clk_en_hdmi");
	reset_hdmitx = rstc_get("rstn_hdmi");

	if((pll_hdmi != 0xBF)||(!__clk_is_enabled(clk_hdmitx))||(reset_control_status(reset_hdmitx)))
	{
		HDMI_DEBUG("Skip av mute, HDMI clock not enable");
		return 1;
	}

	if (flag == HDMI_SETAVM){
		
		HDMI_DEBUG("set av mute");
		
		wr_reg((reg_base+ HDMI_GCPCR), HDMI_GCPCR_enablegcp(1) |
								  HDMI_GCPCR_gcp_clearavmute(1) |
								  HDMI_GCPCR_gcp_setavmute(1) |
								  HDMI_GCPCR_write_data(0));
		wr_reg((reg_base+ HDMI_GCPCR), HDMI_GCPCR_enablegcp(1) |
								  HDMI_GCPCR_gcp_clearavmute(0) |
								  HDMI_GCPCR_gcp_setavmute(1) |
								  HDMI_GCPCR_write_data(1));
			
		return 0;			 
	}		
	else if (flag == HDMI_CLRAVM){
		
		HDMI_DEBUG("clear av mute");
		
		wr_reg((reg_base + HDMI_GCPCR), HDMI_GCPCR_enablegcp(1) |
								   HDMI_GCPCR_gcp_clearavmute(1) |
					               HDMI_GCPCR_gcp_setavmute(1) |
								   HDMI_GCPCR_write_data(0));
		wr_reg((reg_base + HDMI_GCPCR), HDMI_GCPCR_enablegcp(1) |
								   HDMI_GCPCR_gcp_clearavmute(1) |
								   HDMI_GCPCR_gcp_setavmute(0) |
								   HDMI_GCPCR_write_data(1));
		return 0;
					 
	}else{
	
		HDMI_DEBUG("unknown av mute parameter");
		return 1;
	}	
}


// Need ION_AUDIO_HEAP
void register_ion_client(const char *name)
{
    rpc_ion_client = ion_client_create(rtk_phoenix_ion_device, name);
	
}

void deregister_ion_client(const char *name)
{
    if (rpc_ion_client != NULL)
    {
        HDMI_INFO("%s :%s", __FUNCTION__,name);
        ion_client_destroy(rpc_ion_client);
    }
}

#if __RTK_HDMI_GENERIC_DEBUG__
void hdmitx_dump_VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM(struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *arg, char *str) 
{	
	HDMI_DEBUG("%s", __func__);
    HDMI_INFO( "interfaceType 		=0x%x", arg->interfaceType);

	HDMI_INFO( "videoInfo.standard  =0x%x", arg->videoInfo.standard);
	HDMI_INFO( "videoInfo.enProg 	=0x%x", arg->videoInfo.enProg);
	HDMI_INFO( "videoInfo.enDIF 	=0x%x", arg->videoInfo.enDIF);
	HDMI_INFO( "videoInfo.enCompRGB =0x%x", arg->videoInfo.enCompRGB);
	HDMI_INFO( "videoInfo.pedType   =0x%x", arg->videoInfo.pedType);
	HDMI_INFO( "videoInfo.dataInt0  =0x%x", arg->videoInfo.dataInt0);
	HDMI_INFO( "videoInfo.dataInt1  =0x%x", arg->videoInfo.dataInt1);

	HDMI_INFO( "hdmiInfo.hdmiMode 			=0x%x", arg->hdmiInfo.hdmiMode);
    HDMI_INFO( "hdmiInfo.audioSampleFreq 	=0x%x", arg->hdmiInfo.audioSampleFreq);
	HDMI_INFO( "hdmiInfo.audioChannelCount  =0x%x", arg->hdmiInfo.audioChannelCount);
	HDMI_INFO( "hdmiInfo.dataByte1 			=0x%x", arg->hdmiInfo.dataByte1);
	HDMI_INFO( "hdmiInfo.dataByte2 			=0x%x", arg->hdmiInfo.dataByte2);
	HDMI_INFO( "hdmiInfo.dataByte3 			=0x%x", arg->hdmiInfo.dataByte3);
	HDMI_INFO( "hdmiInfo.dataByte4 			=0x%x", arg->hdmiInfo.dataByte4);
	HDMI_INFO( "hdmiInfo.dataByte5 			=0x%x", arg->hdmiInfo.dataByte5);
	HDMI_INFO( "hdmiInfo.dataInt0 			=0x%x", arg->hdmiInfo.dataInt0);
	HDMI_INFO( "hdmiInfo.reserved1 			=0x%x",arg->hdmiInfo.reserved1);
	HDMI_INFO( "hdmiInfo.reserved2 			=0x%x",arg->hdmiInfo.reserved2);
	HDMI_INFO( "hdmiInfo.reserved3 			=0x%x",arg->hdmiInfo.reserved3);
	HDMI_INFO( "hdmiInfo.reserved4 			=0x%x",arg->hdmiInfo.reserved4);

}

void hdmitx_dump_AUDIO_HDMI_OUT_EDID_DATA2(struct AUDIO_HDMI_OUT_EDID_DATA2 *arg, char *str) 
{
	HDMI_DEBUG("%s", __func__);
    HDMI_INFO( "Version 		 	=0x%x", arg->Version);
	HDMI_INFO( "HDMI_output_enable  =0x%x", arg->HDMI_output_enable);
	HDMI_INFO( "EDID_DATA_addr 		=0x%x", arg->EDID_DATA_addr);

}
#endif

int RPC_TOAGENT_HDMI_Config_TV_System(struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *arg) 
{
	struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *rpc = NULL;
    int ret = -1;
	u32 RPC_ret;
	struct ion_handle *handle = NULL;
    ion_phys_addr_t dat;
    size_t len;
    
    handle = ion_alloc(rpc_ion_client, 4096, 1024, RTK_PHOENIX_ION_HEAP_AUDIO_MASK, ION_FLAG_NONCACHED |
																					ION_FLAG_SCPUACC | 
																					ION_FLAG_ACPUACC);
    
	if (IS_ERR(handle)) {
        HDMI_ERROR("[%s %d ion_alloc fail]", __FUNCTION__, __LINE__);
        goto exit;
    }

    if(ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
        HDMI_ERROR("[%s %d fail]", __FUNCTION__, __LINE__);
        goto exit;
    }

    rpc = ion_map_kernel(rpc_ion_client, handle);
	

	memcpy(rpc,arg,sizeof(struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM));  

	rpc->interfaceType = htonl(arg-> interfaceType);
	
	rpc->videoInfo.standard = htonl(arg->videoInfo.standard);
	rpc->videoInfo.pedType  = htonl(arg->videoInfo.pedType);
    rpc->videoInfo.dataInt0 = htonl(arg->videoInfo.dataInt0);
	rpc->videoInfo.dataInt1 = htonl(arg->videoInfo.dataInt1);
	
	rpc->hdmiInfo.hdmiMode  = htonl(arg->hdmiInfo.hdmiMode);
    rpc->hdmiInfo.audioSampleFreq = htonl(arg->hdmiInfo.audioSampleFreq);
	rpc->hdmiInfo.dataInt0  = htonl(arg->hdmiInfo.dataInt0);
    rpc->hdmiInfo.hdmi2p0_feature = htonl(arg->hdmiInfo.hdmi2p0_feature);
    rpc->hdmiInfo.reserved2 = htonl(arg->hdmiInfo.reserved2);
    rpc->hdmiInfo.reserved3 = htonl(arg->hdmiInfo.reserved3);
    rpc->hdmiInfo.reserved4 = htonl(arg->hdmiInfo.reserved4);

    if (send_rpc_command(RPC_AUDIO, 
        ENUM_VIDEO_KERNEL_RPC_CONFIG_TV_SYSTEM,
        CONVERT_FOR_AVCPU(dat),
        CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM))),
        &RPC_ret)) 
    {
        HDMI_ERROR("[%s %d RPC fail]",__FUNCTION__, __LINE__);
        goto exit;
    }

    ret = 0;
exit:
    if(handle != NULL) {
        ion_unmap_kernel(rpc_ion_client, handle);
        ion_free(rpc_ion_client, handle);
    }
    return ret;
}

int RPC_TOAGENT_HDMI_Config_AVI_Info(struct VIDEO_RPC_VOUT_CONFIG_HDMI_INFO_FRAME *arg) 
{
	struct VIDEO_RPC_VOUT_CONFIG_HDMI_INFO_FRAME *rpc = NULL;
    int ret = -1;
    u32 RPC_ret;
    struct ion_handle *handle = NULL;
    ion_phys_addr_t dat;
    size_t len;
    
	handle = ion_alloc(rpc_ion_client, 4096, 1024, RTK_PHOENIX_ION_HEAP_AUDIO_MASK, ION_FLAG_NONCACHED |
																					ION_FLAG_SCPUACC | 
																					ION_FLAG_ACPUACC);
	
	if (IS_ERR(handle)) {
	    HDMI_ERROR("[%s %d ion_alloc fail]", __FUNCTION__, __LINE__);
	    goto exit;
	}
	
	if(ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
	    HDMI_ERROR("[%s %d fail]", __FUNCTION__, __LINE__);
	    goto exit;
	}
	
	rpc = ion_map_kernel(rpc_ion_client, handle);
	
	memcpy(rpc,arg,sizeof(struct VIDEO_RPC_VOUT_CONFIG_HDMI_INFO_FRAME)); 

	rpc->hdmiMode  = htonl(arg->hdmiMode);
    rpc->audioSampleFreq = htonl(arg->audioSampleFreq);
    rpc->dataInt0 =  htonl(arg->dataInt0);
    rpc->hdmi2p0_feature = htonl(arg->hdmi2p0_feature);
    rpc->reserved2 = htonl(arg->reserved2);
    rpc->reserved3 = htonl(arg->reserved3);
    rpc->reserved4 = htonl(arg->reserved4);
	
	 if (send_rpc_command(RPC_AUDIO, 
        ENUM_VIDEO_KERNEL_RPC_CONFIG_HDMI_INFO_FRAME,
        CONVERT_FOR_AVCPU(dat),
        CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(struct VIDEO_RPC_VOUT_CONFIG_HDMI_INFO_FRAME))),
        &RPC_ret)) 
    {
        HDMI_ERROR("[%s %d RPC fail]",__FUNCTION__, __LINE__);
        goto exit;
    }

    ret = 0;
exit:
	if(handle != NULL) {
        ion_unmap_kernel(rpc_ion_client, handle);
        ion_free(rpc_ion_client, handle);
    }
    return ret;
}

int RPC_TOAGENT_HDMI_Set(struct AUDIO_HDMI_SET *arg) 
{
	struct AUDIO_HDMI_SET *rpc = NULL;
    int ret = -1;
    u32 RPC_ret;
    struct ion_handle *handle = NULL;
    ion_phys_addr_t dat;
    size_t len;
   
	
	handle = ion_alloc(rpc_ion_client, 4096, 1024, RTK_PHOENIX_ION_HEAP_AUDIO_MASK, ION_FLAG_NONCACHED |
																					ION_FLAG_SCPUACC | 
																					ION_FLAG_ACPUACC);
	
	if (IS_ERR(handle)) {
	    HDMI_ERROR("[%s %d ion_alloc fail]", __FUNCTION__, __LINE__);
	    goto exit;
	}
	
	if(ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
	    HDMI_ERROR("[%s %d fail]", __FUNCTION__, __LINE__);
	    goto exit;
	}
	
	rpc = ion_map_kernel(rpc_ion_client, handle);
	
   	
    pli_ipcCopyMemory((unsigned char*)rpc,(unsigned char*)arg,sizeof(struct AUDIO_HDMI_SET));   //set frequency

    if (send_rpc_command(RPC_AUDIO, 
        ENUM_KERNEL_RPC_HDMI_SET,
        CONVERT_FOR_AVCPU(dat),
        CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(struct AUDIO_HDMI_SET))),
        &RPC_ret)) 
    {
        HDMI_ERROR("[%s %d RPC fail]",__FUNCTION__, __LINE__);
        goto exit;
    }

    ret = 0;
exit:
	if(handle != NULL) {
        ion_unmap_kernel(rpc_ion_client, handle);
        ion_free(rpc_ion_client, handle);
    }
    return ret;
}


int RPC_TOAGENT_HDMI_Mute(struct AUDIO_HDMI_MUTE_INFO *arg) 
{
	struct AUDIO_HDMI_MUTE_INFO *rpc = NULL;
    int ret = -1;
    u32 RPC_ret;
   	struct ion_handle *handle = NULL;
    ion_phys_addr_t dat;
    size_t len;
    
    handle = ion_alloc(rpc_ion_client, 4096, 1024, RTK_PHOENIX_ION_HEAP_AUDIO_MASK, ION_FLAG_NONCACHED |
																					ION_FLAG_SCPUACC | 
																					ION_FLAG_ACPUACC);
	
	if (IS_ERR(handle)) {
	    HDMI_ERROR("[%s %d ion_alloc fail]", __FUNCTION__, __LINE__);
	    goto exit;
	}
	
	if(ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
	    HDMI_ERROR("[%s %d fail]", __FUNCTION__, __LINE__);
	    goto exit;
	}
	
	rpc = ion_map_kernel(rpc_ion_client, handle);
	
	rpc-> instanceID = htonl(arg->instanceID);
    rpc-> hdmi_mute = arg-> hdmi_mute;
	   	
    if (send_rpc_command(RPC_AUDIO, 
        ENUM_KERNEL_RPC_HDMI_MUTE,
        dat,
        dat + get_rpc_alignment_offset(sizeof(struct AUDIO_HDMI_MUTE_INFO)),
        &RPC_ret)) 
    {
        HDMI_ERROR("[%s %d RPC fail]",__FUNCTION__, __LINE__);
        goto exit;
    }

    ret = 0;
exit:
	if(handle != NULL) {
        ion_unmap_kernel(rpc_ion_client, handle);
        ion_free(rpc_ion_client, handle);
    }
    return ret;
}

int RPC_TOAGENT_HDMI_OUT_VSDB(struct AUDIO_HDMI_OUT_VSDB_DATA *arg) 
{	
	struct AUDIO_HDMI_OUT_VSDB_DATA *rpc = NULL;
    int ret = -1;
    u32 RPC_ret;
    struct ion_handle *handle = NULL;
    ion_phys_addr_t dat;
    size_t len;
    
	handle = ion_alloc(rpc_ion_client, 4096, 1024, RTK_PHOENIX_ION_HEAP_AUDIO_MASK, ION_FLAG_NONCACHED |
																					ION_FLAG_SCPUACC | 
																					ION_FLAG_ACPUACC);
	
	if (IS_ERR(handle)) {
	    HDMI_ERROR("[%s %d ion_alloc fail]", __FUNCTION__, __LINE__);
	    goto exit;
	}
	
	if(ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
	    HDMI_ERROR("[%s %d fail]", __FUNCTION__, __LINE__);
	    goto exit;
	}
	
	rpc = ion_map_kernel(rpc_ion_client, handle);
	
   	pli_ipcCopyMemory((unsigned char*)rpc,(unsigned char*)arg,sizeof(struct AUDIO_HDMI_OUT_VSDB_DATA)); // latency[] - latency[] or fixed value.
	
    if (send_rpc_command(RPC_AUDIO, 
        ENUM_KERNEL_RPC_HDMI_OUT_VSDB,
        CONVERT_FOR_AVCPU(dat),
        CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(struct AUDIO_HDMI_OUT_VSDB_DATA))),
        &RPC_ret)) 
    {
        HDMI_ERROR("[%s %d RPC fail]",__FUNCTION__, __LINE__);
        goto exit;
    }

    ret = 0;
exit:
	if(handle != NULL) {
        ion_unmap_kernel(rpc_ion_client, handle);
        ion_free(rpc_ion_client, handle);
    }
    return ret;
}

int RPC_ToAgent_HDMI_OUT_EDID_0(struct AUDIO_HDMI_OUT_EDID_DATA2 *arg) 
{
	struct AUDIO_HDMI_OUT_EDID_DATA2 *rpc = NULL;
    int ret = -1;
    u32 RPC_ret;
    struct ion_handle *handle = NULL;
    ion_phys_addr_t dat;
    size_t len;
    
    handle = ion_alloc(rpc_ion_client, 4096, 1024, RTK_PHOENIX_ION_HEAP_AUDIO_MASK, ION_FLAG_NONCACHED |
																					ION_FLAG_SCPUACC | 
																					ION_FLAG_ACPUACC);
	
	if (IS_ERR(handle)) {
	    HDMI_ERROR("[%s %d ion_alloc fail]", __FUNCTION__, __LINE__);
	    goto exit;
	}
	
	if(ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
	    HDMI_ERROR("[%s %d fail]", __FUNCTION__, __LINE__);
	    goto exit;
	}
	
	rpc = ion_map_kernel(rpc_ion_client, handle);
	
   	pli_ipcCopyMemory((unsigned char*)rpc,(unsigned char*)arg,sizeof(struct AUDIO_HDMI_OUT_EDID_DATA2)); 

    if (send_rpc_command(RPC_AUDIO, 
        ENUM_KERNEL_RPC_HDMI_OUT_EDID2,
        CONVERT_FOR_AVCPU(dat),
        CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(struct AUDIO_HDMI_OUT_EDID_DATA2))),
        &RPC_ret)) 
    {
        HDMI_ERROR("[%s %d RPC fail]",__FUNCTION__, __LINE__);
        goto exit;
    }

    ret = 0;
exit:
	if(handle != NULL) {
        ion_unmap_kernel(rpc_ion_client, handle);
        ion_free(rpc_ion_client, handle);
    }
    return ret;
}

int RPC_ToAgent_QueryDisplayWin_0(struct VIDEO_RPC_VOUT_QUERY_DISP_WIN_OUT *arg) 
{	
	struct VIDEO_RPC_VOUT_QUERY_DISP_WIN_IN *i_rpc= NULL;
	struct VIDEO_RPC_VOUT_QUERY_DISP_WIN_OUT *o_rpc = NULL;
    int ret = -1;
    u32 RPC_ret;
    struct ion_handle *handle = NULL;
    ion_phys_addr_t dat;
    size_t len;
    unsigned int offset;
	
    handle = ion_alloc(rpc_ion_client, 4096, 1024, RTK_PHOENIX_ION_HEAP_AUDIO_MASK, ION_FLAG_NONCACHED |
																					ION_FLAG_SCPUACC | 
																					ION_FLAG_ACPUACC);
	
	if (IS_ERR(handle)) {
	    HDMI_ERROR("[%s %d ion_alloc fail]", __FUNCTION__, __LINE__);
	    goto exit;
	}
	
	if(ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
	    HDMI_ERROR("[%s %d fail]", __FUNCTION__, __LINE__);
	    goto exit;
	}
	
	i_rpc = ion_map_kernel(rpc_ion_client, handle);	
	offset = get_rpc_alignment_offset(sizeof(struct VIDEO_RPC_VOUT_QUERY_DISP_WIN_IN));
	o_rpc = (struct VIDEO_RPC_VOUT_QUERY_DISP_WIN_OUT *)((unsigned long)i_rpc + offset);

	HDMI_DEBUG("i_rpc=%p ,o_rpc=%p\n",i_rpc,o_rpc);
	
    if (send_rpc_command(RPC_AUDIO, 
        ENUM_VIDEO_KERNEL_RPC_QUERY_DISPLAY_WIN,
        CONVERT_FOR_AVCPU(dat),
        CONVERT_FOR_AVCPU(dat + offset),
        &RPC_ret)) 
    {
        HDMI_ERROR("[%s %d RPC fail]",__FUNCTION__, __LINE__);
        goto exit;
    }
	
	
	 if( RPC_ret != S_OK) {
       HDMI_ERROR("[%s %d RPC fail]",__FUNCTION__, __LINE__);
       goto exit;
   }
	
	arg->standard= htonl(o_rpc->standard);
	HDMI_DEBUG("%s: standard=%d\n",__FUNCTION__,arg->standard);

    ret = 0;
exit:
	if(handle != NULL) {
        ion_unmap_kernel(rpc_ion_client, handle);
        ion_free(rpc_ion_client, handle);
    }
    return ret;
}

int RPC_ToAgent_Vout_EDIDdata(struct VIDEO_RPC_VOUT_EDID_DATA *arg)
{
	struct VIDEO_RPC_VOUT_EDID_DATA *rpc = NULL;
	int ret = -1;
	u32 RPC_ret;
	struct ion_handle *handle = NULL;
	ion_phys_addr_t dat;
	size_t len;

	handle = ion_alloc(rpc_ion_client, 4096, 1024, RTK_PHOENIX_ION_HEAP_AUDIO_MASK, ION_FLAG_NONCACHED |
																					ION_FLAG_SCPUACC | 
																					ION_FLAG_ACPUACC);

	if (IS_ERR(handle)) {
		HDMI_ERROR("[%s %d ion_alloc fail]", __FUNCTION__, __LINE__);
		goto exit;
	}

	if(ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
		HDMI_ERROR("[%s %d fail]", __FUNCTION__, __LINE__);
		goto exit;
	}

	rpc = ion_map_kernel(rpc_ion_client, handle);

	memcpy(rpc,arg,sizeof(struct VIDEO_RPC_VOUT_EDID_DATA));

	if (send_rpc_command(RPC_AUDIO,
		ENUM_VIDEO_KERNEL_RPC_VOUT_EDID_DATA,
		CONVERT_FOR_AVCPU(dat),
		CONVERT_FOR_AVCPU(dat + get_rpc_alignment_offset(sizeof(struct VIDEO_RPC_VOUT_EDID_DATA))),
		&RPC_ret))
	{
		HDMI_ERROR("[%s %d RPC fail]",__FUNCTION__, __LINE__);
		goto exit;
	}

	ret = 0;
exit:
	if(handle != NULL) {
		ion_unmap_kernel(rpc_ion_client, handle);
		ion_free(rpc_ion_client, handle);
	}
	return ret;
}

int RPC_ToAgent_QueryConfigTvSystem(struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *arg)
{
	struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *i_rpc = NULL;
	struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *o_rpc = NULL;
	int ret = -1;
	u32 RPC_ret;
	struct ion_handle *handle = NULL;
	ion_phys_addr_t dat;
	size_t len;
	unsigned int offset;

	handle = ion_alloc(rpc_ion_client, 4096, 1024, RTK_PHOENIX_ION_HEAP_AUDIO_MASK, ION_FLAG_NONCACHED |
																					ION_FLAG_SCPUACC | 
																					ION_FLAG_ACPUACC);

	if (IS_ERR(handle)) {
		HDMI_ERROR("[%s %d ion_alloc fail]", __FUNCTION__, __LINE__);
		goto exit;
	}

	if(ion_phys(rpc_ion_client, handle, &dat, &len) != 0) {
	    HDMI_ERROR("[%s %d fail]", __FUNCTION__, __LINE__);
	    goto exit;
	}

	i_rpc = ion_map_kernel(rpc_ion_client, handle);
	offset = get_rpc_alignment_offset(sizeof(struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM));
	o_rpc = (struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM *)((unsigned long)i_rpc + offset);

	if (send_rpc_command(RPC_AUDIO,
        ENUM_VIDEO_KERNEL_RPC_QUERY_CONFIG_TV_SYSTEM,
        CONVERT_FOR_AVCPU(dat),
        CONVERT_FOR_AVCPU(dat + offset),
        &RPC_ret))
    {
        HDMI_ERROR("[%s %d RPC fail]",__FUNCTION__, __LINE__);
        goto exit;
    }

	memcpy(arg,o_rpc,sizeof(struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM));

	arg->interfaceType				= htonl(arg->interfaceType);
	arg->videoInfo.standard			= htonl(arg->videoInfo.standard);
	arg->videoInfo.pedType			= htonl(arg->videoInfo.pedType);
	arg->videoInfo.dataInt0			= htonl(arg->videoInfo.dataInt0);
	arg->videoInfo.dataInt1			= htonl(arg->videoInfo.dataInt1);
	arg->hdmiInfo.hdmiMode          = htonl(arg->hdmiInfo.hdmiMode);
    arg->hdmiInfo.audioSampleFreq   = htonl(arg->hdmiInfo.audioSampleFreq);

	ret = 0;
exit:
	if(handle != NULL) {
		ion_unmap_kernel(rpc_ion_client, handle);
		ion_free(rpc_ion_client, handle);
	}
	return ret;
}


int Force_RPC_TOAGENT_HDMI_Config_TV_System(int tv_system, int vo_mode) 
{	
	struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM arg;
	
	HDMI_DEBUG("%s",__FUNCTION__);
		
	arg.interfaceType = VO_ANALOG_AND_DIGITAL;	
	arg.videoInfo.standard  = tv_system;
	arg.videoInfo.enProg    =  1;
	arg.videoInfo.enDIF     =  1;
	arg.videoInfo.enCompRGB =  0;
	arg.videoInfo.pedType   = 1;
	arg.videoInfo.dataInt0  = 4;
	arg.videoInfo.dataInt1  = 0;
	
	arg.hdmiInfo.hdmiMode          = vo_mode;
	arg.hdmiInfo.audioSampleFreq   = 3;
	arg.hdmiInfo.audioChannelCount = 1; 
	arg.hdmiInfo.dataByte1         = 64;
	arg.hdmiInfo.dataByte2         = 168;
	arg.hdmiInfo.dataByte3         = 0;
	arg.hdmiInfo.dataByte4         = 0;
	arg.hdmiInfo.dataByte5         = 0;
	arg.hdmiInfo.dataInt0          = 1;
		
	return RPC_TOAGENT_HDMI_Config_TV_System(&arg);
}

int hdmitx_reset_sink_capability(asoc_hdmi_t* p_this)
{	
	p_this-> sink_cap_available = false;
	memset(&p_this->sink_cap,0x0,sizeof(p_this->sink_cap));
	memset(&p_this->sink_cap.cec_phy_addr,0xff,sizeof(p_this->sink_cap.cec_phy_addr));	
	memset(&hdmitx_edid_info,0x0,sizeof(hdmitx_edid_info));

    if(p_this->edid_ptr != NULL)
    {
        kfree(p_this->edid_ptr);
        p_this->edid_ptr = NULL;
    }

	return 0;
}

#ifdef CONFIG_RTK_MHLTX
void hdmitx_synchronize_with_device_capability(asoc_hdmi_t *p_this)
{
	if(!check_hdmi_mhl_mode())
		return ;
		
	struct cbus_adapter *p_adap =cbus_get_adapter();
		
	if(!p_adap -> rtk_mhltx_config.devcap.SUPP_PackedPixel)
	{		
		HDMI_INFO("%s",__FUNCTION__);		
		p_this->sink_cap.vic &= ~(1UL << (Vid_1920x1080p_60Hz-1));
		p_this->sink_cap.vic &= ~(1UL << (Vid_1920x1080p_50Hz-1));
	}
}
#endif

void hdmitx_set_error_code(int error_code)
{	
	hdmi_error = error_code;
	return ;
}

void hdmitx_dump_error_code(void)
{
	switch (hdmi_error) {
	case HDMI_ERROR_NO_MEMORY:
		HDMI_ERROR("get EDID failed, no memory");
		break;
	case HDMI_ERROR_I2C_ERROR:
		HDMI_ERROR("get EDID failed, I2C error");
		break;
	case HDMI_ERROR_HPD:
		HDMI_ERROR("get EDID failed, cable pulled out");
		break;
	case HDMI_ERROR_INVALID_EDID:
		HDMI_ERROR("get EDID failed, invalid EDID");
		break;	
	default:
		break;
	}
}

int hdmitx_get_sink_capability(asoc_hdmi_t *p_this)
{	
	struct edid *edid = NULL;
	
	if(p_this-> sink_cap_available== true)
		return 0;
	
    hdmitx_set_error_code(HDMI_ERROR_RESERVED);

	if((edid = rtk_get_edid(p_this))==NULL)	
		return -1;
			
	p_this-> sink_cap_available= true;
		
	if(rtk_detect_hdmi_monitor(edid))
		p_this-> sink_cap.hdmi_mode=HDMI_MODE_HDMI;		
	else
		p_this-> sink_cap.hdmi_mode=HDMI_MODE_DVI;

    if(check_hdmi_mhl_mode())
		p_this-> sink_cap.hdmi_mode=HDMI_MODE_MHL;
	
	rtk_add_edid_modes(edid, &p_this-> sink_cap);	
	rtk_edid_to_eld(edid, &p_this-> sink_cap);
		
#ifdef CONFIG_RTK_MHLTX
	hdmitx_synchronize_with_device_capability(p_this);
#endif
	hdmitx_print_sink_info(p_this);
	
	return 0;
}

#if __RTK_HDMI_GENERIC_DEBUG__
void hdmitx_print_sink_capability(asoc_hdmi_t *p_this)
{	
	HDMI_INFO("\n=================== sink_capabilities ========================");
	HDMI_INFO("hdmi_mode=%u ",p_this->sink_cap.hdmi_mode);
	HDMI_INFO("est_modes=%u ",p_this->sink_cap.est_modes);
	HDMI_INFO("\n");
	//VSBD
	HDMI_INFO("cec_phy_addr=0x%02x 0x%02x ",p_this->sink_cap.cec_phy_addr[0],p_this->sink_cap.cec_phy_addr[1]);
	HDMI_INFO("support_AI=%d ",p_this->sink_cap.support_AI);
	HDMI_INFO("DC_Y444=%d ",p_this->sink_cap.DC_Y444);
	HDMI_INFO("color_space=%u ",p_this->sink_cap.color_space);
	HDMI_INFO("dvi_dual=%u ",p_this->sink_cap.dvi_dual);
	HDMI_INFO("max_tmds_clock=%d ",p_this->sink_cap.max_tmds_clock);
	HDMI_INFO("latency_present=%d %d ",p_this->sink_cap.latency_present[0],p_this->sink_cap.latency_present[1]);
	HDMI_INFO("video_latency=%u %u ",p_this->sink_cap.video_latency[0],p_this->sink_cap.video_latency[1]);
	HDMI_INFO("audio_latency=%u %u ",p_this->sink_cap.audio_latency[0],p_this->sink_cap.audio_latency[1]);
	HDMI_INFO("\n");

	//printk("structure_all 0x%x\n",p_this-> sink_cap.structure_all);
	//printk("_3D_vic 0x%x\n",p_this-> sink_cap._3D_vic);
		
	//Video
	HDMI_INFO("display_info.width_mm=%u ", p_this->sink_cap.display_info.width_mm);
	HDMI_INFO("display_info.height_mm=%u ", p_this->sink_cap.display_info.height_mm);
	HDMI_INFO("display_info.min_vfreq=%u ", p_this->sink_cap.display_info.min_vfreq);
	HDMI_INFO("display_info.max_vfreq=%u ", p_this->sink_cap.display_info.max_vfreq);
	HDMI_INFO("display_info.min_hfreq=%u ", p_this->sink_cap.display_info.min_hfreq);
	HDMI_INFO("display_info.max_hfreq=%u ", p_this->sink_cap.display_info.max_hfreq);
	HDMI_INFO("display_info.pixel_clock=%u ", p_this->sink_cap.display_info.pixel_clock);
	HDMI_INFO("display_info.bpc=%u ", p_this->sink_cap.display_info.bpc);
	HDMI_INFO("display_info.color_formats=%u ", p_this->sink_cap.display_info.color_formats);
	HDMI_INFO("display_info.edid_hdmi_dc_modes=%u ", p_this->sink_cap.display_info.edid_hdmi_dc_modes);
	HDMI_INFO("display_info.cea_rev=%u ", p_this->sink_cap.display_info.cea_rev);
	HDMI_INFO("vic=0x%llx ",p_this->sink_cap.vic);
	HDMI_INFO("extended_vic=%u ",p_this->sink_cap.extended_vic);
	HDMI_INFO("vic2=0x%llx ",p_this->sink_cap.vic2);
	HDMI_INFO("vic2_420=0x%llx ",p_this->sink_cap.vic2_420);
	HDMI_INFO("HDR metadata: et(0x%02x) sm(0x%02x) max_luminace(0x%02x) max_frame_avg(0x%02x) min_luminace(0x%02x) ",
		p_this->sink_cap.vout_edid_data.et, p_this->sink_cap.vout_edid_data.sm,
		p_this->sink_cap.vout_edid_data.max_luminace, p_this->sink_cap.vout_edid_data.max_frame_avg,
		p_this->sink_cap.vout_edid_data.min_luminace);
	HDMI_INFO("Color characteristics: red_green_lo(0x%02x) black_white_lo(0x%02x) ",
		p_this->sink_cap.vout_edid_data.red_green_lo, p_this->sink_cap.vout_edid_data.black_white_lo);
	HDMI_INFO("Color characteristics: red_x(0x%02x) red_y(0x%02x) ",
		p_this->sink_cap.vout_edid_data.red_x, p_this->sink_cap.vout_edid_data.red_y);
	HDMI_INFO("Color characteristics: green_x(0x%02x) green_y(0x%02x) ",
		p_this->sink_cap.vout_edid_data.green_x, p_this->sink_cap.vout_edid_data.green_y);
	HDMI_INFO("Color characteristics: blue_x(0x%02x) blue_y(0x%02x) ",
		p_this->sink_cap.vout_edid_data.blue_x, p_this->sink_cap.vout_edid_data.blue_y);
	HDMI_INFO("Color characteristics: white_x(0x%02x) white_y(0x%02x) ",
		p_this->sink_cap.vout_edid_data.white_x, p_this->sink_cap.vout_edid_data.white_y);

	HDMI_INFO("=================================================================\n");
	return;
}
#endif

void hdmitx_print_sink_info(asoc_hdmi_t *p_this)
{	
	HDMI_INFO("Dump EDID \n");
	
	print_color_formats(p_this->sink_cap.display_info.color_formats);

	if(p_this->sink_cap.hdmi_mode==HDMI_MODE_HDMI)
		HDMI_INFO("DEVICE MODE: HDMI Mode ");
	else if(p_this->sink_cap.hdmi_mode==HDMI_MODE_DVI)
		HDMI_INFO("DEVICE MODE: DVI Mode ");
	else if(p_this->sink_cap.hdmi_mode==HDMI_MODE_MHL)
		HDMI_INFO("DEVICE MODE: MHL Mode ");

	print_cea_modes(p_this->sink_cap.vic, p_this->sink_cap.vic2, p_this->sink_cap.vic2_420);
	
	HDMI_INFO("CEC Address: 0x%x%02x ",p_this->sink_cap.cec_phy_addr[0],p_this->sink_cap.cec_phy_addr[1]);

	print_deep_color(p_this->sink_cap.display_info.bpc);
	
	if(p_this->sink_cap.DC_Y444)
			HDMI_INFO("Support Deep Color in YCbCr444 ");
			
	if(p_this->sink_cap.max_tmds_clock)
		HDMI_INFO("Max TMDS clock: %d",p_this->sink_cap.max_tmds_clock);

	HDMI_INFO("Video Latency: %d %d ", p_this->sink_cap.video_latency[0],p_this->sink_cap.video_latency[1]);
	HDMI_INFO("Audio Latency: %d %d ", p_this->sink_cap.audio_latency[0],p_this->sink_cap.audio_latency[1]);

	print_color_space(p_this->sink_cap.color_space);

    hdmi_print_raw_edid(p_this->edid_ptr);

	
#if 0	
	hdmi_print_edid(&p_this-> edid_basic);
	
	//print_established_modes(p_this-> sink_cap.est_modes);	
	
	print_color_formats(p_this-> sink_cap.display_info.color_formats);
	
	if(p_this-> sink_cap.hdmi_mode!=0)		
		printk("\n%12s : %s","DEVICE MODE","HDMI Mode");
		
	printk("\n");	
					
	//Video Data Block
	print_cea_modes(p_this->sink_cap.vic, p_this->sink_cap.vic2, p_this->sink_cap.vic2_420);
	
	//Audio Data Block
	//printk("\n%12s : Sampling Rate=%d","AUDIO DATA",p_this-> sink_cap.sampling_rate_cap);
	//printk("\n%12s   Max Channel=%d"," ",p_this-> sink_cap.max_channel_cap);
	
	printk("\n");	
	//Vendor Specific Data Block
	printk("\n%12s : CEC Address=0x%x%02x","VENDOR SPEC"
								,p_this-> sink_cap.cec_phy_addr[0],p_this-> sink_cap.cec_phy_addr[1]);
	
	print_deep_color(p_this-> sink_cap.display_info.bpc);
	
	if(p_this-> sink_cap.DC_Y444)
			printk("\n%12s   Support Deep Color in YCbCr444"," ");
			
	//if(p_this-> sink_cap.dvi_dual)
		//	printk("Support DVI dual\n ");
		
	if(p_this-> sink_cap.max_tmds_clock)
		printk("\n%12s   Max TMDS clock=%d"," ",p_this-> sink_cap.max_tmds_clock);

//	if(p_this-> sink_cap.latency_present[0]||p_this-> sink_cap.latency_present[1])
	{	printk("\n%12s   Video Latency= %d %d "," ", p_this-> sink_cap.video_latency[0],
										p_this-> sink_cap.video_latency[1]);
										
		printk("\n%12s   Audio Latency= %d %d "," ", p_this-> sink_cap.audio_latency[0],
											p_this-> sink_cap.audio_latency[1]);
	}
	
	//printk("structure_all 0x%x\n",p_this-> sink_cap.structure_all);
	//printk("_3D_vic 0x%x\n",p_this-> sink_cap._3D_vic);
	
	//Extended-Colorimetry Data Block
	printk("\n");	
	print_color_space(p_this-> sink_cap.color_space);
	
	printk("\n");	
	hdmi_print_raw_edid(p_this-> edid_ptr);	

#endif
	return ;
}

int ops_config_tv_system(void __user *arg)
{
	struct VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM tv_system;

	HDMI_DEBUG("%s", __func__);		
	
	if (copy_from_user(&tv_system, arg, sizeof(tv_system))) {
		HDMI_ERROR("%s:failed to copy from user !", __func__);
		return -EFAULT;
	}

	if(tv_system.hdmiInfo.hdmiMode==VO_HDMI_ON)
	{
		if(hdmitx_edid_info.hdmi_id==HDMI_2P0_IDENTIFIER)
			tv_system.hdmiInfo.hdmi2p0_feature |= 0x1;//[Bit0] HDMI2.0

		if(hdmitx_send_scdc_TmdsConfig(tv_system.videoInfo.standard, tv_system.hdmiInfo.dataInt0)!=0)
			tv_system.hdmiInfo.hdmi2p0_feature |= 0x2;//[Bit1]Scrabmle
	}

#if __RTK_HDMI_GENERIC_DEBUG__
	hdmitx_dump_VIDEO_RPC_VOUT_CONFIG_TV_SYSTEM(&tv_system, NULL);
#endif
	return RPC_TOAGENT_HDMI_Config_TV_System(&tv_system);	
}

int ops_config_avi_info(void __user *arg)
{
	struct VIDEO_RPC_VOUT_CONFIG_HDMI_INFO_FRAME info_frame;
	
	HDMI_DEBUG("%s", __func__);
	
	if (copy_from_user(&info_frame , arg, sizeof(info_frame))) {
		HDMI_ERROR("%s:failed to copy from user !", __func__);
		return -EFAULT;
	}

	return RPC_TOAGENT_HDMI_Config_AVI_Info(&info_frame);
}

int ops_set_frequency(void __user *arg)
{
	struct AUDIO_HDMI_SET set;
	
	HDMI_DEBUG("%s", __func__);

	if (copy_from_user(&set, arg, sizeof(set))) {
		HDMI_ERROR("%s:failed to copy from user !", __func__);
		return -EFAULT;
	}	
	
	return RPC_TOAGENT_HDMI_Set(&set);	

}

int ops_set_audio_mute(void __user *arg)
{
	struct AUDIO_HDMI_MUTE_INFO mute_info;

	HDMI_DEBUG("%s", __func__);
	
	if (copy_from_user(&mute_info, arg, sizeof(mute_info))) {
		HDMI_ERROR("%s:failed to copy from user !", __func__);
		return -EFAULT;
	}
	
	return RPC_TOAGENT_HDMI_Mute(&mute_info);
}

int ops_send_audio_vsdb_data(void __user *arg)
{	
	struct AUDIO_HDMI_OUT_VSDB_DATA vsdb_data;
	
	HDMI_DEBUG("%s", __func__);	
	
	if (copy_from_user(&vsdb_data,arg,sizeof(vsdb_data))) {		
		HDMI_ERROR("%s:failed to copy from user ! ", __func__);
		return -EFAULT;
	}
	
	return RPC_TOAGENT_HDMI_OUT_VSDB(&vsdb_data);	
}

int ops_send_audio_edid2(void __user *arg)
{
	struct AUDIO_HDMI_OUT_EDID_DATA2 edid2;
	
	HDMI_DEBUG("%s", __func__);	
	
	if (copy_from_user(&edid2,arg,sizeof(edid2))) {		
		HDMI_ERROR("%s:failed to copy from user !", __func__);
		return -EFAULT;
	}
	
	return RPC_ToAgent_HDMI_OUT_EDID_0(&edid2);	
}

int ops_query_display_standard(void __user *arg)
{
	int ret= 0;	
	int standard =0; 
	struct VIDEO_RPC_VOUT_QUERY_DISP_WIN_OUT display_win;
	
	HDMI_DEBUG("%s", __func__);

	ret = RPC_ToAgent_QueryDisplayWin_0(&display_win);
	standard = (int)display_win.standard;
		
	if (copy_to_user(arg, &standard ,sizeof(standard))) {
		HDMI_ERROR("%s:failed to copy to user !", __func__);
		return -EFAULT;
	}
				
	return ret;	
}

int ops_get_sink_cap(void __user *arg, asoc_hdmi_t *data)
{
	int ret= 0;
	
	HDMI_DEBUG("%s", __func__);
	
	if (!(data-> sink_cap_available)) {
		HDMI_ERROR("[%s]sink cap is not available", __func__);
		return -ENOMSG;
	}

	if (copy_to_user(arg, &data->sink_cap, sizeof(data->sink_cap))) {
		HDMI_ERROR("%s:failed to copy to user !", __func__);
		return -EFAULT;
	}

#if __RTK_HDMI_GENERIC_DEBUG__
	hdmitx_print_sink_capability(data);
#endif
	return ret;	
}

int ops_get_raw_edid(void __user *arg, asoc_hdmi_t *data)
{
	int ret= 0;
	
	HDMI_DEBUG("%s", __func__);
	
	if (!(data->sink_cap_available)) {
		HDMI_ERROR("[%s]sink cap is not available", __func__);
		return -ENOMSG;
	}
					
	if (copy_to_user(arg, data->edid_ptr ,sizeof(struct raw_edid))) {
		HDMI_ERROR("%s:failed to copy to user !", __func__);
		return -EFAULT;
	}	
		
	return ret;	
}
int ops_get_link_status(void __user *arg)
{
	int ret= 0;
	int status =0; 
	
	HDMI_DEBUG("%s", __func__);

#ifdef CONFIG_RTK_MHLTX		
	if(check_hdmi_mhl_mode())
		status = mhltx_switchdev_show_HPD_status();
	else
#endif	
		status = show_hpd_status(false);
		
	if (copy_to_user(arg, &status ,sizeof(status))) {
		HDMI_ERROR("%s:failed to copy to user !", __func__);
		return -EFAULT;
	}
				
	return ret;	
}

int ops_get_video_config(void __user *arg, asoc_hdmi_t *data)
{
	int ret= 0;
	
	HDMI_DEBUG("%s", __func__);	
	
	if (copy_to_user( arg, & data->sink_cap.vic ,sizeof(data->sink_cap.vic))) {
		HDMI_ERROR("%s:failed to copy to user !", __func__);
		return -EFAULT;
	}
				
	return ret;	
}

int ops_send_AVmute(void __user *arg, void __iomem *base)
{
	int flag;
	
	HDMI_DEBUG("%s", __func__);	
	
	if (copy_from_user(&flag, arg, sizeof(flag))) {
		HDMI_ERROR("%s:failed to copy from user !", __func__);
		return -EFAULT;
	}
		
	return hdmitx_send_AVmute(base,flag);
}

int ops_check_tmds_src(void __user *arg, void __iomem *base)
{	
	int tmds_mode = hdmitx_check_tmds_src(base);
	int ret = 0;

	HDMI_DEBUG("%s", __func__);		
	
	if (copy_to_user(arg, &tmds_mode, sizeof(tmds_mode))) {	
		HDMI_ERROR("%s:failed to copy to user !", __func__);
		return -EFAULT;
	}
	
	return ret;	
}

int ops_check_rx_sense(void __user *arg, void __iomem *base)
{	
	int rx_sense = hdmitx_check_rx_sense(base);
	int ret = 0;
	
	HDMI_DEBUG("%s", __func__);	
	
	if (copy_to_user(arg, &rx_sense, sizeof(rx_sense))) {	
		HDMI_ERROR("%s:failed to copy to user !", __func__);
		return -EFAULT;
	}
	
	return ret;	
}

int ops_get_extension_blk_count(void __user *arg, asoc_hdmi_t *data)
{
	int ret= 0;
	int count= 0;
	
	HDMI_DEBUG("%s", __func__);
	
	if (!(data->sink_cap_available)) {
		HDMI_ERROR("[%s]sink cap is not available", __func__);
		return -ENOMSG;
	}
	
	count= (int) data -> edid_ptr[0x7e];	
		
	if (copy_to_user(arg, &count ,sizeof(int))) {
		HDMI_ERROR("%s:failed to copy to user !", __func__);
		return -EFAULT;
	}
			
	return ret;	
}


int ops_get_extended_edid(void __user *arg, asoc_hdmi_t *data)
{
	int ret= 0;
	struct ext_edid ext={0};
	
	HDMI_DEBUG("%s", __func__);
	
	if (!(data->sink_cap_available)) {
		HDMI_ERROR("[%s]sink cap is not available", __func__);
		return -ENOMSG;
	}
		
	if (copy_from_user(&ext,arg,sizeof(struct ext_edid))) {
		HDMI_ERROR("%s:failed to copy from user !", __func__);
		return -EFAULT;
	}
	
	if (ext.current_blk >ext.extension) {
		HDMI_ERROR("[%s] out of range !", __func__);
		return -EFAULT;
	}
	
	HDMI_DEBUG("ext.extension=%d ext.current_blk=%d",ext.extension,ext.current_blk);						

	if(ext.current_blk%2==0)
		memcpy(ext.data,data->edid_ptr+ EDID_LENGTH*(ext.current_blk), sizeof(ext.data));
	else	
		memcpy(ext.data,data->edid_ptr+ EDID_LENGTH*ext.current_blk, EDID_LENGTH*sizeof(unsigned char));	
		
	if (copy_to_user(arg,&ext,sizeof(struct ext_edid))) {
		HDMI_ERROR("%s:failed to copy to user ! ", __func__);
		return -EFAULT;
	}
			
	return ret;	
}


int ops_send_vout_edid_data(void __user *arg)
{
	struct VIDEO_RPC_VOUT_EDID_DATA vout_edid_data;

	HDMI_DEBUG("%s", __func__);

	if (copy_from_user(&vout_edid_data,arg,sizeof(vout_edid_data))) {
		HDMI_ERROR("%s:failed to copy from user !", __func__);
		return -EFAULT;
	}
	return RPC_ToAgent_Vout_EDIDdata(&vout_edid_data);
}

