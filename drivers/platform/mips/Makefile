obj-$(CONFIG_CPU_HWMON) += cpu_hwmon.o
obj-$(CONFIG_CPU_HWMON) += ls7a_fan.o
obj-$(CONFIG_I2C_PIIX4) += emc1412.o tmp75.o
obj-$(CONFIG_CPU_HWMON) += wpce_fan.o sbx00_fan.o
obj-$(CONFIG_LEMOTE3A_LAPTOP) += lemote3a-laptop.o
obj-$(CONFIG_LOONGSON_CZC_LAPTOP) += loongson-czc-laptop.o
obj-$(CONFIG_LOONGSON_EA_LAPTOP) += loongson-ea-laptop.o
obj-$(CONFIG_LOONGSON_BITLAND_PM_HOTKEY) += loongson-bitland-pm-hotkey.o
obj-$(CONFIG_LOONGSON_3NOD_PM_HOTKEY) += loongson-3nod-pm-hotkey.o
obj-$(CONFIG_LOONGSON_GENERIC_LAPTOP) += loongson_generic_laptop.o

ifndef CONFIG_CPU_LOONGSON2K
obj-m += ec_rom.o at24c04.o
endif

ifdef CONFIG_MTD
obj-m += pmon_flash.o
endif
