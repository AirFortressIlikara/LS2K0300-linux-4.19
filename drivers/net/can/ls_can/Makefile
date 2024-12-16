# CTU CAN FD ip core Linux driver Makefile

# Invoked make without parameters for native build for current running kernel
# Set KERNEL_VERSION variable to compile for specific installed kernel version
# Exact location of Linux kernel build tree can be chosen by KDIR variable
# If you build for different CPU architecture than native and you do not
# use GNUmakefile trick to set CROSS_COMPILE and ARCH then set these to
# matching values else you do not screw only CTU CAN FD build but even
# base kernel build

obj-$(CONFIG_CAN_LSCAN) := lscan.o
lscan-y := lscan_olddma.o
#lscanfd-y := lscanfd_dma.o
obj-$(CONFIG_CAN_LSCAN_PLATFORM) += lscan_platform.o
drivers/net/can/ls_can/lscan_platform.o:
	cp drivers/net/can/ls_can/lscan_platform.elf drivers/net/can/ls_can/lscan_platform.o
drivers/net/can/ls_can/lscan_olddma.o:
	cp drivers/net/can/ls_can/lscan_olddma.elf drivers/net/can/ls_can/lscan_olddma.o
