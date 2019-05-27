#ifndef _LINUX_COMPAT_HDMITX_H
#define _LINUX_COMPAT_HDMITX_H

#if IS_ENABLED(CONFIG_COMPAT)

#include "hdmitx.h"

long compat_hdmitx_ioctl(struct file* file,unsigned int cmd, unsigned long arg);

#else

#define compat_hdmitx_ioctl  NULL

#endif /* CONFIG_COMPAT */
#endif /* _LINUX_COMPAT_HDMITX_H */
