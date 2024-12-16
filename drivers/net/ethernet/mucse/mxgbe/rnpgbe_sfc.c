// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2022 - 2023 Mucse Corporation. */

#include "rnpgbe_sfc.h"
#include "rnpgbe.h"

#ifndef NO_CM3_MBX
static inline void rsp_hal_sfc_command(u8 __iomem *hw_addr, u32 cmd)
{
	iowrite32(cmd, (hw_addr + 0x8));
	iowrite32(1, (hw_addr + 0x0));
	while (ioread32(hw_addr) != 0)
		;
}

static inline void rsp_hal_sfc_flash_write_disable(u8 __iomem *hw_addr)
{
	iowrite32(CMD_CYCLE(8), (hw_addr + 0x10));
	iowrite32(WR_DATA_CYCLE(0), (hw_addr + 0x14));

	rsp_hal_sfc_command(hw_addr, CMD_WRITE_DISABLE);
}

static int32_t rsp_hal_sfc_flash_wait_idle(u8 __iomem *hw_addr)
{
	iowrite32(CMD_CYCLE(8), (hw_addr + 0x10));
	iowrite32(RD_DATA_CYCLE(8), (hw_addr + 0x14));

	while (1) {
		rsp_hal_sfc_command(hw_addr, CMD_READ_STATUS);
		if ((ioread32(hw_addr + 0x4) & 0x1) == 0)
			break;
	}
	return HAL_OK;
}

static inline void rsp_hal_sfc_flash_write_enable(u8 __iomem *hw_addr)
{
	iowrite32(CMD_CYCLE(8), (hw_addr + 0x10));
	iowrite32(0x1f, (hw_addr + 0x18));
	iowrite32(0x100000, (hw_addr + 0x14));

	rsp_hal_sfc_command(hw_addr, CMD_WRITE_ENABLE);
}

static int rsp_hal_sfc_flash_erase_block_internal(u8 __iomem *hw_addr,
						   u32 address)
{
	if (address >= RSP_FLASH_HIGH_16M_OFFSET)
		return HAL_EINVAL;

	if (address % 0x10000)
		return HAL_EINVAL;

	rsp_hal_sfc_flash_write_enable(hw_addr);

	iowrite32((CMD_CYCLE(8)|ADDR_CYCLE(24)), (hw_addr + 0x10));
	iowrite32((RD_DATA_CYCLE(0)|WR_DATA_CYCLE(0)), (hw_addr + 0x14));
	iowrite32(SFCADDR(address), (hw_addr + 0xc));
	rsp_hal_sfc_command(hw_addr, CMD_BLOCK_ERASE_64K);
	rsp_hal_sfc_flash_wait_idle(hw_addr);
	rsp_hal_sfc_flash_write_disable(hw_addr);

	return HAL_OK;
}

static int rsp_hal_sfc_flash_erase_sector_internal(u8 __iomem *hw_addr,
						   u32 address)
{
	if (address >= RSP_FLASH_HIGH_16M_OFFSET)
		return HAL_EINVAL;

	if (address % 4096)
		return HAL_EINVAL;

	rsp_hal_sfc_flash_write_enable(hw_addr);

	iowrite32((CMD_CYCLE(8)|ADDR_CYCLE(24)), (hw_addr + 0x10));
	iowrite32((RD_DATA_CYCLE(0)|WR_DATA_CYCLE(0)), (hw_addr + 0x14));
	iowrite32(SFCADDR(address), (hw_addr + 0xc));
	rsp_hal_sfc_command(hw_addr, CMD_SECTOR_ERASE);
	rsp_hal_sfc_flash_wait_idle(hw_addr);
	rsp_hal_sfc_flash_write_disable(hw_addr);

	return HAL_OK;
}

static int rsp_hal_sfc_flash_erase_internal(u8 __iomem *hw_addr,
					    u32 address, u32 size)
{
	u32 i;

	for (i = 0; i < size / 0x10000; i++) {
		rsp_hal_sfc_flash_erase_block_internal(hw_addr,
				(address + i * 0x10000));
	}

	address = address + (0x10000 * (size / 0x10000));
	size = size % 0x10000;
	if (size != 0) {
		for (i = 0; i < (size / 0x1000); i++) {
			rsp_hal_sfc_flash_erase_sector_internal(hw_addr,
					(address + i * 0x1000));
		}
	}

	return HAL_OK;
}

int rsp_hal_sfc_flash_erase(struct rnpgbe_hw *hw, u32 size)
{
	u32 addr = SFC_MEM_BASE;

	if (addr < SFC_MEM_BASE)
		return HAL_EINVAL;

	addr = addr - SFC_MEM_BASE;

	if (size == 0)
		return HAL_EINVAL;

	if ((addr + size) > RSP_FLASH_HIGH_16M_OFFSET)
		return HAL_EINVAL;

	if (addr % 4096)
		return HAL_EINVAL;

	if (size % 4096)
		return HAL_EINVAL;


	rsp_hal_sfc_flash_erase_internal(hw->hw_addr, addr, size);

	return HAL_OK;
}

#endif /*NO_CM3_MBX*/
