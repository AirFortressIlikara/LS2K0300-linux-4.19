obj-$(CONFIG_RNP) += rnp.o
rnp-objs :=   \
		rnp_main.o \
		rnp_common.o \
		rnp_ethtool.o \
		rnp_lib.o \
		rnp_mbx.o \
		rnp_pcs.o \
		rnp_n10.o \
		rnp_n500.o \
		rnp_phy.o \
		rnp_vu440.o \
		rnp_mbx_fw.o\
		rnp_sriov.o \
		rnp_compat.o \
		rnp_ptp.o

EXTRA_CFLAGS += -DN10
EXTRA_CFLAGS += -DFIX_VF_BUG

rnp-${CONFIG_RNP_DCB} += rnp_dcb.o
rnp-${CONFIG_DEBUG_FS} += rnp_debugfs.o
rnp-${CONFIG_SYSFS} += rnp_sysfs.o
