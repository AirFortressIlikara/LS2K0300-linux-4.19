/*
 *  register /proc/kgdb, tools to help access address directly from gdb,
 *  more convenient than /proc/kcore.
 *  Copyright (C) 2020 Loongson Technology Corporation Limited
 *  This file is licensed under the terms of the GNU General Public License
 *  version 2. This program is licensed "as is" without any warranty of any
 *  kind, whether express or implied.
 *  Modified from gdbstub.c.
 * Author: Chong Qiao <qiaochong@loongson.cn>
 */

#include <linux/mm.h>
#include <linux/proc_fs.h>
#include <linux/kcore.h>
#include <linux/user.h>
#include <linux/capability.h>
#include <linux/elf.h>
#include <linux/elfcore.h>
#include <linux/notifier.h>
#include <linux/vmalloc.h>
#include <linux/highmem.h>
#include <linux/printk.h>
#include <linux/bootmem.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/list.h>
#include <linux/ioport.h>
#include <linux/memory.h>
#include <asm/sections.h>
#include <linux/module.h>
#include <linux/kallsyms.h>
#include <linux/unaligned/access_ok.h>

#define BUFMAX			2048
#ifdef __loongarch__
#define DBG_MAX_REG_NUM		33
#else
#define DBG_MAX_REG_NUM		72
#endif
#define NUMREGBYTES		(DBG_MAX_REG_NUM * __SIZEOF_LONG__)

#define CIRCBUF_XMIT_SIZE 4096
struct uart_circ_buf {
	char buf[CIRCBUF_XMIT_SIZE];
	int head;
	int tail;
};

#include "circbuf.h"

static struct proc_dir_entry *proc_root_kcore;

static void memcpy_align(void *dst, const void *src, size_t size)
{
	if ((size == 1 || size == 2 || size == 4 || size == 8) && ((long)src & (size - 1)) == 0
		&& ((long)dst & (size - 1)) == 0) {
		switch (size) {
		case 1:
			*(char *)dst = *(char *)src;
			break;
		case 2:
			*(short *)dst = *(short *)src;
			break;
		case 4:
			*(int *)dst = *(int *)src;
			break;
		case 8:
			*(long long *)dst = *(long long *)src;
			break;
		}
	} else
		memcpy(dst, src, size);
}


long probe_kernel_read(void *dst, const void *src, size_t size)
{
	if (src == 0 || (long)src < 0 && (long)src + 16 >= 0)
		return -EFAULT;
	memcpy_align(dst, src, size);

	return 0;
}

long probe_kernel_write(void *dst, const void *src, size_t size)
{
	if (dst == 0 || (long)dst < 0 && (long)dst + 16 >= 0)
		return -EFAULT;
	memcpy_align(dst, src, size);

	return 0;
}


/*
 * While we find nice hex chars, build a long_val.
 * Return number of chars processed.
 */
int kgdb_hex2long(char **ptr, unsigned long *long_val)
{
	int hex_val;
	int num = 0;
	int negate = 0;

	*long_val = 0;

	if (**ptr == '-') {
		negate = 1;
		(*ptr)++;
	}
	while (**ptr) {
		hex_val = hex_to_bin(**ptr);
		if (hex_val < 0)
			break;

		*long_val = (*long_val << 4) | hex_val;
		num++;
		(*ptr)++;
	}

	if (negate)
		*long_val = -*long_val;

	return num;
}

int kgdb_hex2longaddr(char **ptr, unsigned long *long_val)
{
	char *ptr0 = *ptr;
	int ret;
	ret = kgdb_hex2long(ptr, long_val);
	if (*ptr - ptr0 < 16 && (*long_val & (1<<31)))
		*long_val = (int)*long_val;
	return ret;
}

/*
 * Convert the hex array pointed to by buf into binary to be placed in
 * mem.  Return a pointer to the character AFTER the last byte
 * written.  May return an error.
 */
int kgdb_hex2mem(char *buf, char *mem, int count)
{
	char *tmp_raw;
	char *tmp_hex;

	/*
	 * We use the upper half of buf as an intermediate buffer for the
	 * raw memory that is converted from hex.
	 */
	tmp_raw = buf + count * 2;

	tmp_hex = tmp_raw - 1;
	while (tmp_hex >= buf) {
		tmp_raw--;
		*tmp_raw = hex_to_bin(*tmp_hex--);
		*tmp_raw |= hex_to_bin(*tmp_hex--) << 4;
	}

	return probe_kernel_write(mem, tmp_raw, count);
}

/*
 * Convert the memory pointed to by mem into hex, placing result in
 * buf.  Return a pointer to the last char put in buf (null). May
 * return an error.
 */
char *kgdb_mem2hex(char *mem, char *buf, int count)
{
	char *tmp;
	int err;

	/*
	 * We use the upper half of buf as an intermediate buffer for the
	 * raw memory copy.  Hex conversion will work against this one.
	 */
	tmp = buf + count;

	err = probe_kernel_read(tmp, mem, count);
	if (err)
		return NULL;
	while (count > 0) {
		buf = hex_byte_pack(buf, *tmp);
		tmp++;
		count--;
	}
	*buf = 0;

	return buf;
}


struct uart_circ_buf out_buffer;
static char			*remcom_in_buffer;
static char			remcom_out_buffer[BUFMAX];
static char			user_in_buffer[BUFMAX];
static char *in_buffer;
static int in_count;

/*
 * GDB remote protocol parser:
 */


static void circ_putchar(char c)
{
	struct uart_circ_buf *circ = &out_buffer;
	if (circ_chars_free(circ)) {
		circ->buf[circ->head] = c;
		circ->head = (circ->head + 1) & (CIRCBUF_XMIT_SIZE - 1);
	}
}

/*
 * Send the packet in buffer.
 * Check for gdb connection if asked for.
 */
static void put_packet(char *buffer)
{
	unsigned char checksum;
	int count;
	char ch;

	/*
	 * $<packet info>#<checksum>.
	 */
		circ_putchar('+');
		circ_putchar('$');
		checksum = 0;
		count = 0;

		while ((ch = buffer[count])) {
			circ_putchar(ch);
			checksum += ch;
			count++;
		}

		circ_putchar('#');
		circ_putchar(hex_asc_hi(checksum));
		circ_putchar(hex_asc_lo(checksum));

}

char *gdb_detect_and_trap(char ch)
{
	static unsigned char checksum;
	static unsigned char xmitcsum;
	static int count;
	static int kgdb_connected;
	static char buffer[BUFMAX];

	if (kgdb_connected == 0) {
		if (ch  == '$') {
			kgdb_connected = 1;
			checksum = 0;
			xmitcsum = -1;
			count = 0;
		}
		return 0;
	} else if (kgdb_connected == 1) {
		/*
		 * now, read until a # or end of buffer is found:
		 */
		if (ch == '#') {
			kgdb_connected = 2;
			return 0;
		}
		if (count < (BUFMAX - 1)) {
			checksum = checksum + ch;
			buffer[count] = ch;
			count = count + 1;
			return 0;
		} else {
			kgdb_connected = 0;
			return 0;
		}

	} else if (kgdb_connected == 2) {
		xmitcsum = hex_to_bin(ch) << 4;
		kgdb_connected = 3;
		return 0;
	} else if (kgdb_connected == 3) {
		xmitcsum += hex_to_bin(ch);
		buffer[count] = 0;
		kgdb_connected = 0;

		if (checksum != xmitcsum)
			return 0;
		else {
			return buffer;
		}
	} else {
		kgdb_connected = 0;
		return 0;
	}
}

static char *get_packet(void)
{
	char *ret;
	char ch;
	while (in_count) {
		ch = *in_buffer;
		in_count--;
		in_buffer++;
		ret = gdb_detect_and_trap(ch);
		if (ret) {
			return ret;
		}
	}
	return NULL;
}


/*
 * Copy the binary array pointed to by buf into mem.  Fix $, #, and
 * 0x7d escaped with 0x7d. Return -EFAULT on failure or 0 on success.
 * The input buf is overwitten with the result to write to mem.
 */
static int kgdb_ebin2mem(char *buf, char *mem, int count)
{
	int size = 0;
	char *c = buf;

	while (count-- > 0) {
		c[size] = *buf++;
		if (c[size] == 0x7d)
			c[size] = *buf++ ^ 0x20;
		size++;
	}

	return probe_kernel_write(mem, c, size);
}

/* Write memory due to an 'M' or 'X' packet. */
static int write_mem_msg(int binary)
{
	char *ptr = &remcom_in_buffer[1];
	unsigned long addr;
	unsigned long length;
	int err;

	if (kgdb_hex2longaddr(&ptr, &addr) > 0 && *(ptr++) == ',' &&
	    kgdb_hex2long(&ptr, &length) > 0 && *(ptr++) == ':') {
		if (binary)
			err = kgdb_ebin2mem(ptr, (char *)addr, length);
		else
			err = kgdb_hex2mem(ptr, (char *)addr, length);
		if (err)
			return err;
		return 0;
	}

	return -EINVAL;
}

static void error_packet(char *pkt, int error)
{
	error = -error;
	pkt[0] = 'E';
	pkt[1] = hex_asc[(error / 10)];
	pkt[2] = hex_asc[(error % 10)];
	pkt[3] = '\0';
}


/*
 * All the functions that start with gdb_cmd are the various
 * operations to implement the handlers for the gdbserial protocol
 * where KGDB is communicating with an external debugger
 */

/* Handle the '?' status packets */
static void gdb_cmd_status(void)
{
	int signo = 2;
	/*
	 * We know that this packet is only sent
	 * during initial connect.  So to be safe,
	 * we clear out our breakpoints now in case
	 * GDB is reconnecting.
	 */

	remcom_out_buffer[0] = 'S';
	hex_byte_pack(&remcom_out_buffer[1], signo);
}

/* Handle the 'g' get registers request */
static void gdb_cmd_getregs(void)
{
	memset(remcom_out_buffer, '0', NUMREGBYTES * 2);
	remcom_out_buffer[NUMREGBYTES*2] = 0;
}

/* Handle the 'G' set registers request */
static void gdb_cmd_setregs(void)
{
		strcpy(remcom_out_buffer, "OK");
}

/* Handle the 'm' memory read bytes */
static void gdb_cmd_memread(void)
{
	char *ptr = &remcom_in_buffer[1];
	unsigned long length;
	unsigned long addr;
	char *err;

	if (kgdb_hex2longaddr(&ptr, &addr) > 0 && *(ptr++) == ',' &&
					kgdb_hex2long(&ptr, &length) > 0) {
		err = kgdb_mem2hex((char *)addr, remcom_out_buffer, length);
		if (!err)
			error_packet(remcom_out_buffer, -EINVAL);
	} else {
		error_packet(remcom_out_buffer, -EINVAL);
	}
}

/* Handle the 'M' memory write bytes */
static void gdb_cmd_memwrite(void)
{
	int err = write_mem_msg(0);

	if (err)
		error_packet(remcom_out_buffer, err);
	else
		strcpy(remcom_out_buffer, "OK");
}


/* Handle the 'X' memory binary write bytes */
static void gdb_cmd_binwrite(void)
{
	int err = write_mem_msg(1);

	if (err)
		error_packet(remcom_out_buffer, err);
	else
		strcpy(remcom_out_buffer, "OK");
}

/* Handle the 'D' or 'k', detach or kill packets */
static void gdb_cmd_detachkill(void)
{
	/* The detach case */
	if (remcom_in_buffer[0] == 'D') {
		strcpy(remcom_out_buffer, "OK");
		put_packet(remcom_out_buffer);
	}
}



/*
 * This function performs all gdbserial command procesing
 */
int gdb_serial_stub(void)
{
	while (1) {


		remcom_in_buffer = get_packet();
		if (!remcom_in_buffer)
			return 0;
		/* Clear the out buffer. */
		memset(remcom_out_buffer, 0, sizeof(remcom_out_buffer));

		switch (remcom_in_buffer[0]) {
		case '?': /* gdbserial status */
			gdb_cmd_status();
			break;
		case 'm': /* mAA..AA,LLLL  Read LLLL bytes at address AA..AA */
			gdb_cmd_memread();
			break;
		case 'M': /* MAA..AA,LLLL: Write LLLL bytes at address AA..AA */
			gdb_cmd_memwrite();
			break;
		case 'g': /* return the value of the CPU registers */
			gdb_cmd_getregs();
			break;
		case 'G': /* set the value of the CPU registers - return OK */
			gdb_cmd_setregs();
			break;
		case 'X': /* XAA..AA,LLLL: Write LLLL bytes at address AA..AA */
			gdb_cmd_binwrite();
			break;
			/* kill or detach. KGDB should treat this like a
			 * continue.
			 */
		case 'D': /* Debugger detach */
		case 'k': /* Debugger detach via kill */
			gdb_cmd_detachkill();
		default:
			break;

		}

		/* reply to the request */
		put_packet(remcom_out_buffer);
	}

	return 0;
}

/*****************************************************************************/
/*
 * read from the ELF header and then kernel memory
 */
static ssize_t
read_kcore(struct file *file, char __user *buffer, size_t buflen, loff_t *fpos)
{

	struct uart_circ_buf *circ = &out_buffer;
	int ret = 0;
	int c;
	while (1) {
		c = CIRC_CNT_TO_END(circ->head, circ->tail, CIRCBUF_XMIT_SIZE);
		if (buflen < c)
			c = buflen;
		if (c <= 0)
			break;
		memcpy(buffer, circ->buf + circ->tail, c);
		circ->tail = (circ->tail + c) & (CIRCBUF_XMIT_SIZE - 1);
		buffer += c;
		buflen -= c;
		ret += c;
	}

	return ret;
}

static ssize_t
write_kcore(struct file *file, const char __user *buffer, size_t buflen, loff_t *fpos)
{
	memcpy(user_in_buffer, buffer, buflen);
	in_buffer = user_in_buffer;
	in_count = buflen;

	gdb_serial_stub();
	return buflen;
}

static int open_kcore(struct inode *inode, struct file *filp)
{
	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;

	return 0;
}


static const struct file_operations proc_kcore_operations = {
	.read		= read_kcore,
	.write		= write_kcore,
	.open		= open_kcore,
};


static int __init proc_kcore_init(void)
{

	proc_root_kcore = proc_create("kgdb", S_IRUSR, NULL,
				      &proc_kcore_operations);
	if (!proc_root_kcore) {
		pr_err("couldn't create /proc/kcore\n");
		return 0; /* Always returns 0. */
	}

	return 0;
}
module_init(proc_kcore_init);
MODULE_LICENSE("GPL");
