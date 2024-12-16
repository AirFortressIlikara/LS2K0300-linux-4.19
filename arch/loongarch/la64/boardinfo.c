// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Loongson Technology Co., Ltd.
 */
#include <linux/efi.h>
#include <linux/init.h>
#include <linux/kobject.h>

#include <boot_param.h>

static ssize_t boardinfo_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
			"BIOS Information\n"
			"Vendor\t\t\t: %s\n"
			"Version\t\t\t: %s\n"
			"ROM Size\t\t: %d KB\n"
			"Release Date\t\t: %s\n\n"
			"Board Information\n"
			"Manufacturer\t\t: %s\n"
			"Board Name\t\t: %s\n"
			"Family\t\t\t: LOONGSON3\n\n",
			b_info.bios_vendor, b_info.bios_version,
			b_info.bios_size, b_info.bios_release_date,
			b_info.board_vendor, b_info.board_name);
}

static struct kobj_attribute boardinfo_attr = __ATTR(boardinfo, 0444,
						boardinfo_show, NULL);

static int __init boardinfo_init(void)
{
	if (!efi_kobj)
		return -EINVAL;

	return sysfs_create_file(efi_kobj, &boardinfo_attr.attr);
}

late_initcall(boardinfo_init);
