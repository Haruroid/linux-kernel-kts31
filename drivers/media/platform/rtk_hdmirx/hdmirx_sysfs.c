#include "v4l2_hdmi_dev.h"
#include "hdmiEDID.h"

static DEFINE_MUTEX(hdmirx_sysfs_lock);


static ssize_t hdmirx_edid_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret_count=0;

	mutex_lock(&hdmirx_sysfs_lock);
	ret_count = sprintf(buf, "%s\n",HdmiRx_GetEDID_version()?"2.0":"1.4");
	mutex_unlock(&hdmirx_sysfs_lock);

	return ret_count;
}

static ssize_t hdmirx_edid_version_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	mutex_lock(&hdmirx_sysfs_lock);
	if (sysfs_streq(buf, "1.4"))
		HdmiRx_SetEDID_version(0);
	else if (sysfs_streq(buf, "2.0"))
		HdmiRx_SetEDID_version(1);
	else
		HDMIRX_ERROR("Set EDID version fail, unknown version\n");
	mutex_unlock(&hdmirx_sysfs_lock);
	return size;
}

/* /sys/devices/platform/98037000.hdmirx/edid_version */
static DEVICE_ATTR(edid_version, 0644, hdmirx_edid_version_show, hdmirx_edid_version_store);

int register_hdmirx_sysfs(struct platform_device *pdev)
{
	device_create_file(&pdev->dev, &dev_attr_edid_version);
}

