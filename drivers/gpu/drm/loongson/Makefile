#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI)
#

ccflags-y := -Iinclude/drm
loongson-y := loongson_drv.o \
	loongson_irq.o \
	loongson_encoder.o \
	loongson_crtc.o \
	loongson_connector.o \
	loongson_fb.o \
	loongson_cursor.o \
	loongson_vbios.o \
	loongson_ttm.o \
	loongson_debugfs.o \
	loongson_i2c.o \
	loongson_backlight.o \
	loongson_prime.o

loongson-y += bridge/bridge_phy.o
loongson-y += bridge/it66121_drv.o
loongson-y += bridge/lt8618_drv.o
loongson-y += bridge/ms7210_drv.o
loongson-y += bridge/ncs8805_drv.o
loongson-y += bridge/lt8619_drv.o
loongson-y += bridge/lt9721_drv.o

obj-$(CONFIG_DRM_LOONGSON_VGA) += loongson.o
