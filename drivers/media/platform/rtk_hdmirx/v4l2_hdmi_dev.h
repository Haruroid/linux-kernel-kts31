#ifndef V4L2_HDMI_DEV_H
#define V4L2_HDMI_DEV_H

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/font.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/dma-contiguous.h>
#include <linux/device.h>
#include <linux/fb.h>
#include <linux/switch.h>

#include <linux/power-control.h>
#include <linux/reset-helper.h> // rstc_get
#include <linux/reset.h>
#include <linux/clkdev.h>  // clk_get
#include <linux/clk-provider.h>

#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-core.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>
#include <media/v4l2-common.h>
#include <media/videobuf2-memops.h>

#include <asm/page.h>
#include <asm/memory.h>



#define __RTK_HDMI_RX_DEBUG__	0

#if __RTK_HDMI_RX_DEBUG__
#define HDMIRX_DEBUG(format, ...) pr_err("[HDMI RX DBG]" format "\n", ## __VA_ARGS__)
#else
#define HDMIRX_DEBUG(format, ...) 
#endif

#define HDMIRX_ERROR(format, ...) printk(KERN_ERR "[HDMI RX ERR]" format "\n", ## __VA_ARGS__)
#define HDMIRX_INFO(format, ...) printk(KERN_WARNING "[HDMI RX]" format "\n", ## __VA_ARGS__)


#define HDMI_SW_BUF_NUM 20

#define roundup16(x)	roundup(x, 16)

typedef enum {
	CLK_HDMIRX	= 0x1,
	CLK_RXWRAP	= 0x2,
	CLK_MIPI	= 0x4,
	CLK_CBUS	= 0x8,
	CLK_ALL		= 0xF
} HDMI_CLK_TYPE;

typedef enum {
	CTL_DISABLE	= 0,
	CTL_ENABLE	= 1
} HDMI_CLK_CTL;

typedef struct
{
	atomic_t read_index;
	atomic_t write_index;
	atomic_t fill_index;
	int use_v4l2_buffer;
	int pre_frame_done;
	int ISR_FLIP;
}HDMI_SW_BUF_CTL;


/* buffer for one video frame */
struct hdmi_buffer {
	/* common v4l buffer stuff -- must be first */
	struct vb2_buffer	vb;
	//unsigned long		phys;
	// TODO: type
	u32		phys;
	struct list_head	list;
};

struct hdmi_dmaqueue {
	atomic_t			rcnt;	//number of buffers already processed by hw
	atomic_t			qcnt;	//number of buffers waiting for hw process
	struct list_head	active;
	struct hdmi_buffer	*hwbuf[2];
};

struct v4l2_hdmi_dev {
	struct v4l2_device		v4l2_dev;
	struct video_device		vdev;
	struct switch_dev		sdev;
	struct switch_dev		asdev;// Audio switch device

	spinlock_t		slock;
	struct mutex	mutex; /* Protects queue */

	struct hdmi_dmaqueue	   hdmidq;

	/* video capture */
	unsigned int		   width;
	unsigned int		   height;
	unsigned int		   outfmt;
	unsigned int		   bpp;
	struct vb2_queue	   vb_hdmidq;
};

int hdmi_queue_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], void *alloc_ctxs[]);
int hdmi_buffer_prepare(struct vb2_buffer *vb);
void hdmi_buffer_finish(struct vb2_buffer *vb);
void hdmi_buffer_queue(struct vb2_buffer *vb);
int hdmi_start_streaming(struct vb2_queue *vq, unsigned int count);
void hdmi_stop_streaming(struct vb2_queue *vq);
void hdmi_unlock(struct vb2_queue *vq);
void hdmi_lock(struct vb2_queue *vq);

static const struct vb2_ops hdmi_qops = {
	.queue_setup		= hdmi_queue_setup,
	.buf_prepare		= hdmi_buffer_prepare,
	.buf_finish 		= hdmi_buffer_finish,
	.buf_queue			= hdmi_buffer_queue,
	.start_streaming	= hdmi_start_streaming,
	.stop_streaming 	= hdmi_stop_streaming,
	.wait_prepare		= hdmi_unlock,
	.wait_finish		= hdmi_lock,
};

struct mipi_vb2_vmalloc_buf {
	   void 						   *vaddr;
	   struct page					   **pages;
	   struct vm_area_struct		   *vma;
	   int							   write;
	   //unsigned long				   size;
	   // TODO: type
	   u32				   size;
	   unsigned int 				   n_pages;
	   atomic_t 					   refcount;
	   struct vb2_vmarea_handler	   handler;
	   struct dma_buf				   *dbuf;
};

int hdmirx_queue_init(struct v4l2_hdmi_dev *dev, struct vb2_queue *q, enum v4l2_memory memory);
int hdmirx_rtk_drv_probe(struct platform_device *pdev);
int hdmirx_rtk_drv_remove(struct platform_device *pdev);
int hdmirx_rtk_drv_suspend(struct platform_device *pdev, pm_message_t state);
int hdmirx_rtk_drv_resume(struct platform_device *);

#endif // V4L2_HDMI_DEV_H
