obj-$(CONFIG_RNPVF) := rnpvf.o
rnpvf-objs :=   \
				vf.o \
                mbx.o \
                ethtool.o \
				rnpvf_compat.o\
                rnpvf_main.o

EXTRA_CFLAGS += -DN10
EXTRA_CFLAGS += -DFIX_VF_BUG
rnpvf-${CONFIG_SYSFS} += sysfs.o
