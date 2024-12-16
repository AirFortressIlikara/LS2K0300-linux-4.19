/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_MODULE_SYMBOL_H
#define _LINUX_MODULE_SYMBOL_H

#include <stdbool.h>

/* This ignores the intensely annoying "mapping symbols" found in ELF files. */
static inline bool is_mapping_symbol(const char *str)
{
	if (str[0] == '.' && str[1] == 'L')
		return true;
	if (str[0] == 'L' && str[1] == '0')
		return true;
	return str[0] == '$' &&
	       (str[1] == 'a' || str[1] == 'd' || str[1] == 't' || str[1] == 'x')
	       && (str[2] == '\0' || str[2] == '.');
}

#endif /* _LINUX_MODULE_SYMBOL_H */
