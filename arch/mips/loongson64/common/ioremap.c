/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/io.h>

void __init __iomem *early_ioremap(u64 phys_addr, unsigned long size)
{
	if (phys_addr == 0xF0000)
		phys_addr = 0xfffe000;

	return ((void *)TO_CAC(phys_addr));
}

void __init early_iounmap(void __iomem *addr, unsigned long size)
{

}
