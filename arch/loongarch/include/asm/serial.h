/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM__SERIAL_H
#define __ASM__SERIAL_H

/* Set it to 0 to prevent 8250_early using it. */
#define BASE_BAUD 0

#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST)

#endif /* __ASM__SERIAL_H */
