// SPDX-License-Identifier: GPL-2.0
#include <stdio.h>
#include "bpf_load.h"

int main(int argc, char **argv)
{
	if (load_bpf_file("hello_kern.o") != 0) {
		printf("Failed to load BPF program\n");
		return -1;
	}

	read_trace_pipe();
	return 0;
}
