#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/printk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>


#define  SB1_DEVICE_NAME         "sb1"
#define  SB3_DEVICE_NAME         "sb3"
#define  SB4_DEVICE_NAME         "sb4"
#define  SB0_DEVICE_NAME         "sb0"


static struct device*   	sb1_device;
static struct cdev      sb1_dev;
static struct device*   	sb3_device;
static struct cdev      sb3_dev;
static struct device*   	sb4_device;
static struct cdev      sb4_dev;
static struct device*   	sb0_device;
static struct cdev      sb0_dev;

static dev_t            sb1_devno;
static dev_t            sb3_devno;
static dev_t            sb4_devno;
static dev_t            sb0_devno;



static struct class 		*sbx_dev_class = NULL;

enum {
	GET_SBX_PRIORITY1,
	SET_SBX_PRIORITY1,
	GET_SBX_PRIORITY2,
	SET_SBX_PRIORITY2,
        GET_CHANNEL_HIGH_PRIORITY,
	SET_CHANNEL_HIGH_PRIORITY,
        GET_CHANNEL_REQ_MASK,
	SET_CHANNEL_REQ_MASK,
        GET_CHANNEL_REQ_BUSY,
        GET_MISC_CTRL,
	SET_MISC_CTRL,
	
};


static long sbx_dev_ioctl(struct file *file,
	unsigned int cmd, 
	unsigned long arg) {

        void __user *argp = (void __user *)arg;
	struct device *dev = (struct device *)file->private_data;
	void __iomem 	*reg_base = (void __iomem *)dev_get_drvdata(dev);
	const char *devname = dev_name(dev);
	u32 value;
	u32 offset;
	bool in;

	pr_info("%s %s cmd=%d\n", __func__, devname, cmd); 
        switch (cmd) {
        	case GET_SBX_PRIORITY1:
			offset = 0;
			in = true;
			break;	
		case SET_SBX_PRIORITY1:
			offset = 0;
			in = false;
			break;
		case GET_SBX_PRIORITY2:
			if (!strcmp("sb0", devname))		
				return -EFAULT;

			offset = 4;
			in = true;
			break;
		case SET_SBX_PRIORITY2:
			if (!strcmp("sb0", devname))		
				return -EFAULT;

                	offset = 4;
			in = false;
			break;
        	case GET_CHANNEL_HIGH_PRIORITY:
			if (!strcmp("sb0", devname))		
				return -EFAULT;

			offset = 8;
			in = true;
			break;
		
		case SET_CHANNEL_HIGH_PRIORITY:
			if (!strcmp("sb0", devname))		
				return -EFAULT;

			offset = 8;
			in = false;
			break;


        	case GET_CHANNEL_REQ_MASK:
			offset = 0xc;
		 	if (!strcmp("sb0", devname))		
				offset = 0x4;
			in = true;
			break;


		case SET_CHANNEL_REQ_MASK:
			offset = 0xc;
			if (!strcmp("sb0", devname))		
				offset = 0x4;

			in = false;
			break;


        	case GET_CHANNEL_REQ_BUSY:
			offset = 0x10;
			if (!strcmp("sb0", devname))		
				offset = 0x8;

			in = true;
			break;

        	case GET_MISC_CTRL:
			offset = 0x14;
			if (!strcmp("sb0", devname))		
				offset = 0xc;
	
			in = true;
			break;

		case SET_MISC_CTRL:
			offset = 0x14;
			if (!strcmp("sb0", devname))		
				offset = 0xc;

			in = false;
			break;

        	default:
                	return -EFAULT;

	}



	if (in) {
		value = readl(reg_base + offset); 
		pr_info("read reg_base= 0x%p offset=0x%x value=0x%x \n", reg_base, offset, value);	
		if (copy_to_user(argp, &value , sizeof(u32))) {
                        return -EFAULT;
                }
	}
	else {
	
		if (copy_from_user(&value, argp, sizeof(u32))) {
            
                        return -EFAULT;
                }
		pr_info("write reg_base= 0x%p offset=0x%x value=0x%x \n", reg_base, offset, value);	

		writel(value, reg_base + offset);
	}

	return 0;

}




static long compat_sbx_dev_ioctl(
    	struct file*            file,
    	unsigned int            cmd,
    	unsigned long           arg) {
	
	 if (!file->f_op->unlocked_ioctl)
                return -EFAULT;
        else 
                return file->f_op->unlocked_ioctl(file, cmd,(unsigned long)compat_ptr(arg));
}
	



static
int sbx_dev_open(struct inode *inode, struct file *file)
{
	dev_t devno = inode->i_cdev->dev;

    	pr_info("sbx open\n");
	if (sb0_devno == devno)
		file->private_data = (void *)sb0_device;
	else if (sb1_devno == devno)
		file->private_data = (void *)sb1_device;
	else if (sb3_devno == devno)
		file->private_data = (void *)sb3_device;
	else if (sb4_devno == devno)
		file->private_data = (void *)sb4_device;
	else 
		return -EFAULT;

        return 0;
    
}

static struct file_operations sbx_ops =
{
    	.owner		= THIS_MODULE,
    	.unlocked_ioctl = sbx_dev_ioctl,
    	.compat_ioctl 	= compat_sbx_dev_ioctl,
    	.open       	= sbx_dev_open,
};

static ssize_t sbx_priority_1_show(struct device *dev,
                struct device_attribute *attr, char *buffer) {
	void __iomem 	*reg_base = (void __iomem *)dev_get_drvdata(dev);
	dev_dbg(dev, "reg_base =0x%p \n", reg_base);
        return sprintf(buffer, "0x%x\n",  readl(reg_base + 0) );
}

static ssize_t sbx_priority_1_store(struct device *dev, struct device_attribute *attr,
                const char *buffer, size_t size) {
        int value;
	void __iomem 	*reg_base = (void __iomem *)dev_get_drvdata(dev);

	dev_dbg(dev, "%s" , buffer);
	value = simple_strtol(buffer, 0, 16);
	dev_dbg(dev, "0x%x ", value);
       	writel(value, reg_base + 0);

        return size;
}

static ssize_t sbx_priority_2_show(struct device *dev,
                struct device_attribute *attr, char *buffer) {
	void __iomem 	*reg_base = (void __iomem *)dev_get_drvdata(dev);
        return sprintf(buffer, "0x%x\n", readl(reg_base + 4) );

}

static ssize_t sbx_priority_2_store(struct device *dev, struct device_attribute *attr,
                const char *buffer, size_t size) {
        int value;
	void __iomem 	*reg_base = (void __iomem *)dev_get_drvdata(dev);

	dev_dbg(dev, "%s" , buffer);
	value = simple_strtol(buffer, 0, 16);
	dev_dbg(dev, "0x%x ", value);
       	writel(value, reg_base + 4);

        return size;
}

static ssize_t sbx_channel_high_prioriy_show(struct device *dev,
                struct device_attribute *attr, char *buffer) {
	void __iomem 	*reg_base = (void __iomem *)dev_get_drvdata(dev);
        return sprintf(buffer, "0x%x\n", readl(reg_base + 8) );

}

static ssize_t sbx_channel_high_prioriy_store(struct device *dev, struct device_attribute *attr,
                const char *buffer, size_t size) {
        int value;
	void __iomem 	*reg_base = (void __iomem *)dev_get_drvdata(dev);

	dev_dbg(dev, "%s" , buffer);
	value = simple_strtol(buffer, 0, 16);
	dev_dbg(dev, "0x%x ", value);
       	writel(value, reg_base + 8);

        return size;
}

static ssize_t sbx_channel_req_mask_show(struct device *dev,
                struct device_attribute *attr, char *buffer) {
	void __iomem 	*reg_base = (void __iomem *)dev_get_drvdata(dev);

	if (!strcmp("sb0", dev_name(dev)))
		return sprintf(buffer, "sb0 0x%x\n", readl(reg_base + 0x4) );

        return sprintf(buffer, "0x%x\n", readl(reg_base + 0xc) );

}

static ssize_t sbx_channel_req_mask_store(struct device *dev, struct device_attribute *attr,
                const char *buffer, size_t size) {
        int value;
	void __iomem 	*reg_base = (void __iomem *)dev_get_drvdata(dev);

	dev_dbg(dev, "%s" , buffer);
	value = simple_strtol(buffer, 0, 16);
	dev_dbg(dev, "0x%x ", value);
	if (!strcmp("sb0", dev_name(dev)))
		writel(value, reg_base + 0x4);
	else
       		writel(value, reg_base + 0xc);

        return size;
}

static ssize_t channel_req_busy_show(struct device *dev,
                struct device_attribute *attr, char *buffer) {
	void __iomem 	*reg_base = (void __iomem *)dev_get_drvdata(dev);
	if (!strcmp("sb0", dev_name(dev)))
		return sprintf(buffer, "sb0 0x%x\n", readl(reg_base + 0x8) );

        return sprintf(buffer, "0x%x\n", readl(reg_base + 0x10) );


}

static ssize_t sbx_misc_ctrl_show(struct device *dev,
                struct device_attribute *attr, char *buffer) {
	void __iomem 	*reg_base = (void __iomem *)dev_get_drvdata(dev);
	if (!strcmp("sb0", dev_name(dev)))
		return sprintf(buffer, "sb0 0x%x\n", readl(reg_base + 0xc) );

        return sprintf(buffer, "0x%x\n", readl(reg_base + 0x14) );

}

static ssize_t sbx_misc_ctrl_store(struct device *dev, struct device_attribute *attr,
                const char *buffer, size_t size) {
        int value;
	void __iomem 	*reg_base = (void __iomem *)dev_get_drvdata(dev);

	dev_dbg(dev, "%s" , buffer);
	value = simple_strtol(buffer, 0, 16);
	dev_dbg(dev, "0x%x ", value);
	if (!strcmp("sb0", dev_name(dev)))
		writel(value, reg_base + 0xc);
	else
       		writel(value, reg_base + 0x14);

        return size;
}




    
static DEVICE_ATTR(priority_1, 0644, sbx_priority_1_show, sbx_priority_1_store);
static DEVICE_ATTR(priority_2, 0644, sbx_priority_2_show, sbx_priority_2_store);
static DEVICE_ATTR(channel_high_prioriy, 0644, sbx_channel_high_prioriy_show, sbx_channel_high_prioriy_store);
static DEVICE_ATTR(channel_req_mask, 0644, sbx_channel_req_mask_show, sbx_channel_req_mask_store);
static DEVICE_ATTR_RO(channel_req_busy);
static DEVICE_ATTR(misc_ctrl, 0644, sbx_misc_ctrl_show, sbx_misc_ctrl_store);


static const struct attribute *rtk_sbx_attrs[] = {
        &dev_attr_priority_1.attr,
        &dev_attr_priority_2.attr,
        &dev_attr_channel_high_prioriy.attr,
        &dev_attr_channel_req_mask.attr,
        &dev_attr_channel_req_busy.attr,
        &dev_attr_misc_ctrl.attr,
        NULL
};

static const struct attribute *rtk_sb0_attrs[] = {
        &dev_attr_priority_1.attr,
        &dev_attr_channel_req_mask.attr,
        &dev_attr_channel_req_busy.attr,
        &dev_attr_misc_ctrl.attr,
        NULL
};


static int sbx_init(struct platform_device *pdev)
{
	struct device_node *np;
	void __iomem 		*sb1_reg_base;
	void __iomem 		*sb3_reg_base;
	void __iomem 		*sb0_reg_base;
	void __iomem 		*sb4_reg_base;
	dev_t	devno;
	int retval = 0;

	if (WARN_ON(!(pdev->dev.of_node)))
	{
		pr_err("[SBX DBG] Error: No node\n");
		return -ENODEV;
	}

	np = pdev->dev.of_node;
	sb1_reg_base = of_iomap(np, 0);
	sb3_reg_base = of_iomap(np, 1);
	sb0_reg_base = of_iomap(np, 2);
	sb4_reg_base = of_iomap(np, 3);
	
	/* character device , /dev */
	cdev_init(&sb1_dev, &sbx_ops);

    	if (alloc_chrdev_region(&devno, 0, 1, SB1_DEVICE_NAME)!=0)
    	{
        	cdev_del(&sb1_dev);
        	printk("%s %s %d",__FILE__, __func__,__LINE__);
        	return -EFAULT;
    	}
	pr_info("sb1 devno=%d \n", devno);
	sb1_devno = devno;
    	if (cdev_add(&sb1_dev, devno, 1)<0)
        	return -EFAULT;

	/* device , /sysfs */
    	sbx_dev_class = class_create(THIS_MODULE, "sbx");       // create a new class for sbx

    	sb1_device = device_create(
        		sbx_dev_class,      	// class
                    	NULL,               	// parent
                    	devno,              	// dev number
                    	sb1_reg_base,   	// driver data
                    	"sb1");        

	retval = sysfs_create_files(&sb1_device->kobj,
                        (const struct attribute **)rtk_sbx_attrs);


	cdev_init(&sb3_dev, &sbx_ops);

    	if (alloc_chrdev_region(&devno, 0, 1, SB3_DEVICE_NAME)!=0)
    	{
        	cdev_del(&sb3_dev);
        	printk("%s %s %d",__FILE__, __func__,__LINE__);
        	return -EFAULT;
    	}
	pr_info("sb3 devno=%d \n", devno);
	sb3_devno = devno;

    	if (cdev_add(&sb3_dev, devno, 1)<0)
        	return -EFAULT;


    	sb3_device = device_create(
        		sbx_dev_class,      	// class
                    	NULL,               	// parent
                    	devno,              	// dev number
                    	sb3_reg_base,   	// driver data
                    	"sb3");        

	retval = sysfs_create_files(&sb3_device->kobj,
                        (const struct attribute **)rtk_sbx_attrs);



	cdev_init(&sb4_dev, &sbx_ops);

    	if (alloc_chrdev_region(&devno, 0, 1, SB4_DEVICE_NAME)!=0)
    	{
        	cdev_del(&sb4_dev);
        	printk("%s %s %d",__FILE__, __func__,__LINE__);
        	return -EFAULT;
    	}
	pr_info("sb4 devno=%d \n", devno);
	sb4_devno = devno;

    	if (cdev_add(&sb4_dev, devno, 1)<0)
        	return -EFAULT;


    	sb4_device = device_create(
        		sbx_dev_class,      	// class
                    	NULL,               	// parent
                    	devno,              	// dev number
                    	sb4_reg_base,   	// driver data
                    	"sb4");        

	retval = sysfs_create_files(&sb4_device->kobj,
                        (const struct attribute **)rtk_sbx_attrs);


	cdev_init(&sb0_dev, &sbx_ops);

    	if (alloc_chrdev_region(&devno, 0, 1, SB0_DEVICE_NAME)!=0)
    	{
        	cdev_del(&sb0_dev);
        	printk("%s %s %d",__FILE__, __func__,__LINE__);
        	return -EFAULT;
    	}
	pr_info("sb0 devno=%d \n", devno);
	sb0_devno = devno;

    	if (cdev_add(&sb0_dev, devno, 1)<0)
        	return -EFAULT;


    	sb0_device = device_create(
        		sbx_dev_class,      	// class
                    	NULL,               	// parent
                    	devno,              	// dev number
                    	sb0_reg_base,   	// driver data
                    	"sb0");        

	retval = sysfs_create_files(&sb0_device->kobj,
                        (const struct attribute **)rtk_sb0_attrs);



	return retval;
}

static int sbx_exit(struct platform_device *pdev)
{

	return 0;
}

int sbx_suspend(struct device *dev)
{
	return 0;
}

int sbx_resume(struct device *dev)
{
	return 0;
}


static struct dev_pm_ops sbx_pm_ops = {
	.suspend_noirq = sbx_suspend,
	.resume_noirq = sbx_resume,
};

static const struct of_device_id rtk_sbx_match[] = {
	{.compatible = "Realtek,rtk-sbx"},
	{},
};

static struct platform_driver rtk129x_sbx_driver = {
	.probe = sbx_init,
	.remove = sbx_exit,
	.driver = {
		.name = "rtk129x-sbx",
		.owner = THIS_MODULE,
		.pm = &sbx_pm_ops,
		.of_match_table = of_match_ptr(rtk_sbx_match),
	},
};
module_platform_driver(rtk129x_sbx_driver);

MODULE_DESCRIPTION("Realtek SBx driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtk119x-sbx");

