#
# Common code for all Loongson based systems, except Loongson2k soc.
#
ifneq ($(CONFIG_LOONGSON_MACH2K),y)
obj-$(CONFIG_MACH_LOONGSON64) += common/
endif

#
# Lemote Fuloong mini-PC (Loongson 2E-based)
#

obj-$(CONFIG_LEMOTE_FULOONG2E)	+= fuloong-2e/

#
# Lemote loongson2f family machines
#

obj-$(CONFIG_LEMOTE_MACH2F)  += lemote-2f/

#
# All Loongson-3 family machines
#

obj-$(CONFIG_CPU_LOONGSON3)  += loongson-3/

#
# All Loongson-2K family machines
#

obj-$(CONFIG_CPU_LOONGSON2K)  += loongson-2k/
