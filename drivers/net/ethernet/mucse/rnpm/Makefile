obj-$(CONFIG_RNPM) := rnpm.o
rnpm-objs :=   \
	rnpm_common.o \
	rnpm_ethtool.o \
	rnpm_lib.o \
	rnpm_main.o \
	rnpm_mbx.o \
	rnpm_vu440.o \
	rnpm_n10.o \
	rnpm_pcs.o \
	rnpm_mbx_fw.o\
	rnpm_phy.o \
	rnpm_sriov.o \
	rnpm_compat.o \
	rnpm_tc.o \
	rnpm_ptp.o

EXTRA_CFLAGS += -DN10
EXTRA_CFLAGS += -DFIX_VF_BUG

rnp-${CONFIG_RNPM_DCB} += rnpm_dcb.o
rnpm-${CONFIG_DEBUG_FS} += rnpm_debugfs.o
rnpm-${CONFIG_SYSFS} += rnpm_sysfs.o
