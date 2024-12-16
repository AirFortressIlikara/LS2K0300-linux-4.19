#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/module.h>

#include <boot_param.h>

char *bios_vendor;
char *bios_release_date;
char *board_manufacturer;
char _bios_info[64];
char _board_info[64];
extern struct interface_info *einter;
extern struct board_devices *eboard;

static int show_boardinfo(struct seq_file *m, void *v)
{
	seq_printf(m, "BIOS Information\n");
	seq_printf(m, "Vendor\t\t\t: %s\n", bios_vendor);
	seq_printf(m, "Version\t\t\t: %s\n", einter->description);
	seq_printf(m, "BIOS ROMSIZE\t\t: %d\n", einter->size);
	seq_printf(m, "Release date\t\t: %s\n", bios_release_date);
	seq_printf(m, "\n");

	seq_printf(m, "Base Board Information\t\t\n");
	seq_printf(m, "Manufacturer\t\t: %s\n", board_manufacturer);
	seq_printf(m, "Board name\t\t: %s\n", eboard->name);
	seq_printf(m, "Family\t\t\t: LOONGSON3\n");
	seq_printf(m, "\n");

	return 0;
}

static void *bd_start(struct seq_file *m, loff_t *pos)
{
	unsigned long i = *pos;

	return i ? NULL : (void *)0x1;
}

static void *bd_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return bd_start(m, pos);
}

static void bd_stop(struct seq_file *m, void *v)
{
}

const struct seq_operations boardinfo_op = {
	.start	= bd_start,
	.next	= bd_next,
	.stop	= bd_stop,
	.show	= show_boardinfo,
};

static int boardinfo_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &boardinfo_op);
}

static const struct file_operations proc_boardinfo_operations = {
	.open		= boardinfo_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int __init proc_boardinfo_init(void)
{
	proc_create("boardinfo", 0, NULL, &proc_boardinfo_operations);
	return 0;
}

module_init(proc_boardinfo_init);
