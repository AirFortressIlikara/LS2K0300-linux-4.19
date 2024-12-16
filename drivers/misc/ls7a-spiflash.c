/*
 * Copyright (c) 2018 Loongson Technology Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/err.h>

#define PAGE_PROGRAM	0x02
#define READ_DATA		0x03
#define WR_DIS			0x04
#define RD_STATUS		0x05
#define WR_EN			0x06
#define SEC_ERASE		0x20
#define DEV_BUSY		0x1
#define SECTOR_SIZE 	0x1000
#define SPI_PAGE_SIZE 	0x100

static struct mutex spiflash_lock;
static struct spi_device *spi_dev;

static void spidev_complete(void *arg)
{
	complete(arg);
}

static int ls_spiflash_write_en(unsigned char wr_en)
{
	u8 cmd[1];
	struct spi_message msg;

	if (spi_dev == NULL) {
		return -ENODEV;
	}

	DECLARE_COMPLETION_ONSTACK(done);

	struct spi_transfer instruction = {
			.tx_buf = cmd,
			.rx_buf = NULL,
			.len = 1,
	};

	cmd[0] = wr_en;
	spi_message_init(&msg);
	spi_message_add_tail(&instruction, &msg);

	msg.complete = spidev_complete;
	msg.context = &done;
	msg.spi = spi_dev;

	spi_dev->master->transfer(spi_dev,&msg);
	wait_for_completion(&done);

	return 0;
}

unsigned char ls_spiflash_read_status(void)
{
	u8 cmd[1],ret;
	struct spi_message msg;

	if (spi_dev == NULL) {
		return -ENODEV;
	}

	DECLARE_COMPLETION_ONSTACK(done);

	struct spi_transfer instruction = {
			.tx_buf = cmd,
			.rx_buf = NULL,
			.len = 1,
	};

	struct spi_transfer status = {
			.tx_buf = NULL,
			.rx_buf = &ret,
			.len = 1,
	};

	cmd[0] = RD_STATUS;
	spi_message_init(&msg);
	spi_message_add_tail(&instruction, &msg);
	spi_message_add_tail(&status, &msg);

	msg.complete = spidev_complete;
	msg.context = &done;
	msg.spi = spi_dev;

	spi_dev->master->transfer(spi_dev,&msg);
	wait_for_completion(&done);

	return ret;
}
EXPORT_SYMBOL(ls_spiflash_read_status);

static int ls_spiflash_sec_erase(int addr)
{
	u8 cmd[4];
	struct spi_message msg;

	if (spi_dev == NULL) {
		return -ENODEV;
	}

	DECLARE_COMPLETION_ONSTACK(done);

	struct spi_transfer instruction = {
			.tx_buf = cmd,
			.rx_buf = NULL,
			.len = 4,
	};

	addr &= ~(SECTOR_SIZE - 1);
	cmd[0] = SEC_ERASE;
	cmd[1] = (addr >> 16) & 0xff;
	cmd[2] = (addr >> 8) & 0xff;
	cmd[3] = addr & 0xff;

	spi_message_init(&msg);
	spi_message_add_tail(&instruction, &msg);

	msg.complete = spidev_complete;
	msg.context = &done;
	msg.spi = spi_dev;

	spi_dev->master->transfer(spi_dev,&msg);
	wait_for_completion(&done);

	return 0;
}

int ls_spiflash_read(int addr, unsigned char *buf,int data_len)
{
	u8 cmd[4];
	struct spi_message msg;

	if (spi_dev == NULL) {
		printk("SPI driver is not registered !\n");
		return -ENODEV;
	}

	DECLARE_COMPLETION_ONSTACK(done);

	struct spi_transfer instruction = {
			.tx_buf = cmd,
			.rx_buf = NULL,
			.len = 4,
	};

	struct spi_transfer mac_addr = {
			.tx_buf = NULL,
			.rx_buf = buf,
			.len = data_len,
	};

	cmd[0] = READ_DATA;
	cmd[1] = (addr >> 16) & 0xff;
	cmd[2] = (addr >> 8) & 0xff;
	cmd[3] = addr & 0xff;

	spi_message_init(&msg);
	spi_message_add_tail(&instruction, &msg);
	spi_message_add_tail(&mac_addr, &msg);

	msg.complete = spidev_complete;
	msg.context = &done;
	msg.spi = spi_dev;

	spi_dev->master->transfer(spi_dev,&msg);
	wait_for_completion(&done);

	return 0;
}
EXPORT_SYMBOL(ls_spiflash_read);

static int spi_flash_page_program(int addr, unsigned char *buf,int data_len)
{
	u8 cmd[4];
	struct spi_message msg;

	if (spi_dev == NULL) {
		return -ENODEV;
	}

	DECLARE_COMPLETION_ONSTACK(done);

	struct spi_transfer instruction = {
			.tx_buf = cmd,
			.rx_buf = NULL,
			.len = 4,
	};

	struct spi_transfer mac_addr = {
			.rx_buf = NULL,
			.tx_buf = buf,
			.len = data_len,
	};

	cmd[0] = PAGE_PROGRAM;
	cmd[1] = (addr >> 16) & 0xff;
	cmd[2] = (addr >> 8) & 0xff;
	cmd[3] = addr & 0xff;

	spi_message_init(&msg);
	spi_message_add_tail(&instruction, &msg);
	spi_message_add_tail(&mac_addr, &msg);

	msg.complete = spidev_complete;
	msg.context = &done;
	msg.spi = spi_dev;

	spi_dev->master->transfer(spi_dev,&msg);
	wait_for_completion(&done);

	return 0;
}

int ls_spiflash_write(int addr, unsigned char *buf,int data_len)
{
	int start_addr;
	/*first read status to detect external spi flash*/
	if (ls_spiflash_read_status() == 0xff)
		return -ENODEV;

	for(start_addr = addr; addr < start_addr + data_len;addr += SPI_PAGE_SIZE,buf += SPI_PAGE_SIZE){
		mutex_lock(&spiflash_lock);
		ls_spiflash_write_en(WR_EN);
		if(((addr % SPI_PAGE_SIZE) + data_len) <= SPI_PAGE_SIZE)
			spi_flash_page_program(addr, buf,data_len);
		else
			spi_flash_page_program(addr, buf,SPI_PAGE_SIZE);
		while ((ls_spiflash_read_status() & DEV_BUSY) == DEV_BUSY);
		mutex_unlock(&spiflash_lock);
	}

	return 0;
}
EXPORT_SYMBOL(ls_spiflash_write);

int ls_spiflash_sectors_erase(int addr, int data_len)
{
	int start_addr;
	/*first read status to detect external spi flash*/
	if (ls_spiflash_read_status() == 0xff)
		return -ENODEV;

	for(start_addr = addr;addr < start_addr + data_len;addr += SECTOR_SIZE){
		mutex_lock(&spiflash_lock);
		ls_spiflash_write_en(WR_EN);
		ls_spiflash_sec_erase(addr);
		while ((ls_spiflash_read_status() & DEV_BUSY) == DEV_BUSY);
		mutex_unlock(&spiflash_lock);
	}

	return 0;
}

static int ls_spiflash_probe(struct spi_device *spi)
{
	mutex_init(&spiflash_lock);
	spi_dev = spi;

	return 0;
}

static int ls_spiflash_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver ls_spiflash_driver = {
	.driver = {
		.name		= "w25q16dvssig",
		.owner		= THIS_MODULE,
	},

	.probe		= ls_spiflash_probe,
	.remove		= ls_spiflash_remove,
};

static struct spi_board_info ls_spi_dev __initdata = {
	/* spi flash */
	.modalias	= "w25q16dvssig",
	.chip_select	= 0,
	.mode		= 0,
};

extern struct spi_master *get_ls7a_spi_master(void);

int __init ls_spiflash_init(void)
{
	struct spi_master *ls7a_spi_master = get_ls7a_spi_master();

	if (spi_dev)
		return 0;

	if (ls7a_spi_master)
		ls_spi_dev.bus_num = ls7a_spi_master->bus_num;

	spi_register_board_info(&ls_spi_dev, 1);

	if (spi_register_driver(&ls_spiflash_driver)) {
		pr_err("No spi flash device register!");
		return -ENODEV;
	}
	return 0;
}
EXPORT_SYMBOL(ls_spiflash_init);

MODULE_AUTHOR("Xuefeng Li <lixuefeng@loongson.cn>");
MODULE_DESCRIPTION("LS7A SPI FLASH driver");
MODULE_LICENSE("GPL");
