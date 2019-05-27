#ifndef _HDMIEDID_H_
#define _HDMIEDID_H_


typedef struct
{
   unsigned char EDID[256];
   unsigned char tx_phy_addr[2];// TX Physical Address
   unsigned char valid;
} HDMIRX_DTS_EDID_TBL_T;

void HdmiRx_Save_DTS_EDID_Table(HDMIRX_DTS_EDID_TBL_T * dts_edid);
void HdmiRx_Save_DTS_EDID2p0_Table(HDMIRX_DTS_EDID_TBL_T * dts_edid);
void HdmiRx_SetEDID_version(int version);
int HdmiRx_GetEDID_version(void);
void HdmiRx_ChangeCurrentEDID(unsigned char *edid);
unsigned char HdmiRx_EnableEDID(void);
#endif//_HDMIEDID_H_