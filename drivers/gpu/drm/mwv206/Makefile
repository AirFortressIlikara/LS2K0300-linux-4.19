
# mips64       => _MWV206_LOONGSON_
# loongarch64  => _MWV206_LOONGSON_
# aarch64      => _MWV206_AARCH64_
# x86_64       =>
# sw_64        => _MWV206_SW_64_
CPU=$(shell LANG=en uname -m)
ifeq ($(CPU),mips64)
        EXTRA_CFLAGS+=-D__PCIE_CONFIGSPACE_MAXREG__=256 #default is 0x1000
else ifeq ($(CPU),loongarch64)
        EXTRA_CFLAGS+=-D__PCIE_CONFIGSPACE_MAXREG__=256 #default is 0x1000
endif


#ID=kylin
#ID="neokylin"
#ID=uos
#ID=nfsdesktop
#ID=kylinSecOS
#ID=ubuntu
OS=$(shell cat /etc/os-release | sed -n '/^ID=/p' | sed -e 's/"//g' -e 's/ID=//g' )
ifeq ($(OS), uos)
        EXTRA_CFLAGS+=-D_MWV206_SKIP_VT_SWITCH_
else ifeq ($(OS), UOS)
        EXTRA_CFLAGS+=-D_MWV206_SKIP_VT_SWITCH_
else
	KCFLAGS+=-Wno-missing-field-initializers
endif

#CC=gcc
KCFLAGS+=-O2
#KCFLAGS+=-fstack-protector-strong
KCFLAGS+=-Wl,-z,relro -z,now -Wl,-z,-Wstramplines,noexecstack -fPIC -D_FORTIFY_SOURCE=2

KCFLAGS+=-fno-strict-aliasing -Wall -W -Wextra -Wno-unused-label -Wno-unused-function -Wno-pointer-sign -Wno-unused-parameter -Wno-missing-braces \
		-Wno-deprecated-declarations -Werror -Wno-sign-compare

CC+=$(KCFLAGS)

EXTRA_CFLAGS+=-Wno-sizeof-pointer-memaccess
EXTRA_CFLAGS+=-D__MWV206_SYSLIBPATH__="\"$(SYSTEMLIBPATH)\""

AR=ar
ARFLAGS=-cr

INSTALL=install

LDFLAGS=

ifeq ($(KVERS),)
    KVERS = /lib/modules/$(shell uname -r)
endif

# CONFIG_SOUND = y,m
# CONFIG_SND_PCM = y,m
mwv206-config-$(CONFIG_SND_PCM) += -DSUPPORT_SND_PCM

SRCDIR=$(abspath $(dir $(word $(words $(MAKEFILE_LIST)), $(MAKEFILE_LIST))))
EXTRA_CFLAGS+=-I$(SRCDIR) -I$(SRCDIR)/include -I$(SRCDIR)/mwv206linuxdev -I$(SRCDIR)/mwv206hal $(mwv206-config-m) $(mwv206-config-y)

# Kernel modules
obj-m += mwv206.o

mwv206-objs := mwv206hal/gljos_kernel.o \
	mwv206hal/jmgpio.o \
	mwv206hal/jmgraphoutput.o \
	mwv206hal/jmiic.o \
	mwv206hal/jmirq.o \
	mwv206hal/jmpwm.o \
	mwv206hal/jmspi.o \
	mwv206hal/jmspi_flash.o \
	mwv206hal/mwv206ddr_chip.o \
	mwv206hal/mwv206dec.o \
	mwv206hal/mwv206dec_priv.o \
	mwv206hal/mwv206dev.o \
	mwv206hal/mwv206devconfigparser.o \
	mwv206hal/mwv206devconfigreader.o \
	mwv206hal/mwv206ioctl.o \
	mwv206hal/mwv206ioctl_cmd.o \
	mwv206hal/mwv206ioctl_config.o \
	mwv206hal/mwv206ioctl_dev.o \
	mwv206hal/mwv206ioctl_display.o \
	mwv206hal/mwv206ioctl_intr.o \
	mwv206hal/mwv206ioctl_mem.o \
	mwv206hal/mwv206kdma.o \
	mwv206hal/mwv206kg2d.o \
	mwv206hal/mwv206kg3d.o \
	mwv206hal/mwv206memmgr.o \
	mwv206hal/mwv206reg.o \
	mwv206hal/mwv206sdm.o \
	mwv206hal/mwv206ver_kernel.o \
	mwv206hal/oshal_linux_kernel.o \
	mwv206linuxdev/copy.o \
	mwv206linuxdev/fill.o \
	mwv206linuxdev/gljos.o \
	mwv206linuxdev/mwv206_backlight.o \
	mwv206linuxdev/mwv206_edid.o \
	mwv206linuxdev/mwv206_fb_interpret_edid.o \
	mwv206linuxdev/mwv206_fb.o \
	mwv206linuxdev/mwv206_hdmiaudio.o \
	mwv206linuxdev/mwv206_pci.o \
	mwv206linuxdev/mwv206_pm.o \
	mwv206linuxdev/mwv206_proc.o

# Specify flags for the module compilation.
#EXTRA_CFLAGS=-g -O0

default: kernel_modules
build: kernel_modules

kernel_modules:
ifeq ($(shell if [ -d $(KVERS)/build ]; then echo "exist"; fi), exist)
	make -C $(KVERS)/build M=$(CURDIR) modules
else
	@echo "$(KVERS)/build : Not exist"
endif

clean:
ifeq ($(shell if [ -d $(KVERS)/build ]; then echo "exist"; fi), exist)
	make -C $(KVERS)/build M=$(CURDIR) clean
else
	@echo "$(KVERS)/build : Not exist"
endif

distclean: clean

install:
ifeq ($(shell if [ -e mwv206.ko ]; then echo "exist"; fi), exist)
	install -D mwv206.ko $(DESTDIR)/opt/mwv206/glj206-kernel/$(KVERS)/extra/mwv206.ko
else
	@echo "mwv206.ko : Not exist"
endif

.PHONY: clean install kernel_modules build default distclean
