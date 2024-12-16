# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Xel network device drivers.
#

obj-$(CONFIG_XLNID) += sdk/

subdir-ccflags-y += -I$(src)/include/

ifeq ($(CONFIG_XLNID_AUTO_MEDIA), y)
subdir-ccflags-y += -DXLNID_AUTO_MEDIA_SUPPORT
else
subdir-ccflags-y += -DXLNID_GEPHY_SUPPORT
endif
