/* Copyright (C) 2007-2014 Realtek Semiconductor Corporation. */

#ifndef _HDCP_TOP_H_
#define _HDCP_TOP_H_

#include <linux/interrupt.h>

#if IS_ENABLED(CONFIG_COMPAT)
long compat_hdcp_ioctl(struct file* file,unsigned int cmd, unsigned long arg);
#else
#define compat_hdcp_ioctl  NULL
#endif

irqreturn_t HDCP_interrupt_handler(int irq, void *dev_id);

#endif /* _HDCP_TOP_H_ */
