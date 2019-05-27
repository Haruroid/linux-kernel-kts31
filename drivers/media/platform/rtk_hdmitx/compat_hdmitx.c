#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/compat.h>


#include "compat_hdmitx.h"


/*------------------------------------------------------------------
 * Func : compat_hdmitx_ioctl
 *
 * Desc : ioctl function of hdmitx miscdev
 *
 * Parm : file	: context of file
 *		  cmd	: control command
 *		  arg	: arguments
 *		   
 * Retn : 0 : success, others fail	
 *------------------------------------------------------------------*/
long compat_hdmitx_ioctl(struct file* file,unsigned int cmd, unsigned long arg)
{

	if (!file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {	
	case HDMI_GET_SINK_CAPABILITY:
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));

	case HDMI_GET_RAW_EDID: 
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));	

	case HDMI_CHECK_LINK_STATUS:
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));

	case HDMI_GET_VIDEO_CONFIG:
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));

	case HDMI_SEND_AVMUTE:
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));

	case HDMI_CONFIG_TV_SYSTEM:
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));

	case HDMI_CONFIG_AVI_INFO:
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));	

	case HDMI_SET_FREQUNCY:
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));

	case HDMI_SEND_AUDIO_MUTE:		
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));	

	case HDMI_SEND_AUDIO_VSDB_DATA: 
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));

	case HDMI_SEND_AUDIO_EDID2: 
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));	

	case HDMI_CHECK_TMDS_SRC:	
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));

	case HDMI_CHECK_Rx_Sense:	
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg)); 

	case HDMI_GET_EXT_BLK_COUNT:
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));

	case HDMI_GET_EXTENDED_EDID:	
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));

	case HDMI_QUERY_DISPLAY_STANDARD:	
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));

	case HDMI_SEND_VOUT_EDID_DATA:
		return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));

	default:		 
		return -EFAULT; 		 
	}		
}

