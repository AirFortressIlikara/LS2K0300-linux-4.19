// SPDX-License-Identifier: GPL-2.0
#include <elf.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

__attribute__((noreturn))
static void die(const char *msg)
{
	fputs(msg, stderr);
	exit(EXIT_FAILURE);
}

int main(int argc, const char *argv[])
{
	uint64_t entry;
	size_t nread;
	FILE *file;
	union {
		Elf32_Ehdr ehdr32;
		Elf64_Ehdr ehdr64;
	} hdr;

	if (argc != 2)
		die("Usage: elf-entry <elf-file>\n");

	file = fopen(argv[1], "r");
	if (!file) {
		perror("Unable to open input file");
		return EXIT_FAILURE;
	}

	nread = fread(&hdr, 1, sizeof(hdr), file);
	if (nread != sizeof(hdr)) {
		fclose(file);
		perror("Unable to read input file");
		return EXIT_FAILURE;
	}

	if (memcmp(hdr.ehdr32.e_ident, ELFMAG, SELFMAG)) {
		fclose(file);
		die("Input is not an ELF\n");
	}

	switch (hdr.ehdr32.e_ident[EI_CLASS]) {
	case ELFCLASS32:
		/* Sign extend to form a canonical address */
		entry = (int64_t)(int32_t)hdr.ehdr32.e_entry;
		break;

	case ELFCLASS64:
		entry = hdr.ehdr64.e_entry;
		break;

	default:
		fclose(file);
		die("Invalid ELF class\n");
	}

	fclose(file);
	printf("0x%016" PRIx64 "\n", entry);

	return EXIT_SUCCESS;
}
