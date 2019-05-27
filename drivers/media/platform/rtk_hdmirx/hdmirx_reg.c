
#include "asm/io.h"
#include "hdmirx_reg.h"

void __iomem *hdmi_rx_base[HDMI_RX_REG_BLOCK_NUM];


unsigned int hdmi_rx_reg_read32 (unsigned int addr_offset, HDMI_RX_REG_BASE_TYPE type)
{
	return (readl(hdmi_rx_base[type] + addr_offset));
}

void hdmi_rx_reg_write32 (unsigned int addr_offset, unsigned int value, HDMI_RX_REG_BASE_TYPE type)
{
#if 0
	if( type==HDMI_RX_MIPI_PHY)
		pr_err("[MIPI_PHY Write] Reg(0x%08x)=0x%08x\n", 0x98004000+addr_offset,value);
	else if(type==HDMI_RX_MIPI)
		pr_err("[MIPI Write] Reg(0x%08x)=0x%08x\n",0x98004100+addr_offset,value);
#endif
	writel(value, hdmi_rx_base[type] + addr_offset);
}

void hdmi_rx_reg_mask32(unsigned int addr_offset, unsigned int andMask, unsigned int orMask, HDMI_RX_REG_BASE_TYPE type)
{
	hdmi_rx_reg_write32(addr_offset, ((hdmi_rx_reg_read32(addr_offset, type) & (andMask)) | (orMask)), type);
}


