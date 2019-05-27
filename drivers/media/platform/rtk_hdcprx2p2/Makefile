#
# Makefile for the Realtek HDCP2.2 rx driver
#

ccflags-y += -I$(srctree)/drivers/media/platform/rtk_hdcprx2p2/include

obj-$(CONFIG_RTK_HDCPRX_2P2) := core/crypto_spu.o core/secure_flash.o core/hdmi.o \
  core/bigdigits.o core/crypto.o core/hmac.o core/rcp_api.o core/sha2.o \
  rx/hdcp2_interface.o rx/hdcp2_messages.o rx/hdcp2_session.o rx/spu_main.o \
  tx/HDCPTX22.o tx/HDMI_HDCP_API.o
