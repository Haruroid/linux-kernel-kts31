#ifndef _RTK_EDID_H_
#define _RTK_EDID_H_

#include "hdmitx.h"

#define HDMI_1PX_IDENTIFIER 0x000C03
#define HDMI_2P0_IDENTIFIER 0xC45DD8

struct edid_info {
	// ----- Base Block 128 Bytes Begin -----
	u8 header[8];
	/* Vendor & product info */
	u8 mfg_id[2];
	u8 prod_code[2];
	u32 serial; /* FIXME: byte order */
	u8 mfg_week;
	u8 mfg_year;
	/* EDID version */
	u8 version;
	u8 revision;
	/* Display info: */
	u8 input;
	u8 width_cm;
	u8 height_cm;
	u8 gamma;
	u8 features;
	/* Color characteristics */
	u8 red_green_lo;
	u8 black_white_lo;
	u8 red_x;
	u8 red_y;
	u8 green_x;
	u8 green_y;
	u8 blue_x;
	u8 blue_y;
	u8 white_x;
	u8 white_y;
	/* Est. timings and mfg rsvd timings*/
	struct est_timings established_timings;
	/* Standard timings 1-8*/
	struct std_timing standard_timings[8];
	/* Detailing timings 1-4 */
	struct detailed_timing detailed_timings[4];
	/* Number of 128 byte ext. blocks */
	u8 extensions;
	/* Checksum */
	u8 checksum;
	// ----- Base Block 128 Bytes End ----

	u32 hdmi_id;
	u32 max_tmds_char_rate;
	u8 scdc_capable;
	u8 dc_420;//Deep Color420, [Bit2]DC_48bit_420;[Bit1]DC_36bit_420;[Bit0]:DC_30bit_420
} __attribute__((packed));

// HDMI Forum Vendor Specific Data Block Byte6
enum SCDC_CAPABLES {
	SCDC_PRESENT = 0x80, /* Bit7 */
	SCDC_RR_CAPABLE = 0x40, /* Bit6*/
	SCDC_340M_SCRAMBLE = 0x08, /* Bit3 */
};

int rtk_edid_header_is_valid(const u8 *raw_edid);
bool rtk_edid_block_valid(u8 *raw_edid, int block);
bool rtk_detect_hdmi_monitor(struct edid *edid);
struct edid *rtk_get_edid(asoc_hdmi_t *hdmi);
int rtk_do_probe_ddc_edid(unsigned char *buf,int block,int len);
bool rtk_edid_is_valid(struct edid *edid);
int rtk_add_edid_modes(struct edid *edid, struct sink_capabilities_t *sink_cap);
void rtk_edid_to_eld(struct edid *edid, struct sink_capabilities_t *sink_cap);

void hdmi_print_edid(struct edid *edid);
void print_established_modes(u32 est_format);
void print_cea_modes(u64 cea_format, u64 cea_format2, u64 cea_format2_420);
void print_deep_color(u32 var);
void print_color_formats(u32 var);
void print_color_space(u8 var);
void hdmi_print_raw_edid(unsigned char *edid);

#endif /* _RTK_EDID_H_*/
