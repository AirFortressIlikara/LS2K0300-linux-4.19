// SPDX-License-Identifier: GPL-2.0
#include <linux/bpf.h>
#include <linux/version.h>
#include "bpf_helpers.h"

SEC("tracepoint/syscalls/sys_enter_execve")
int tracepoint__sys_enter_execve(void *ctx)
{
	char msg[] = "Hello BPF tracepoint!\n";

	bpf_trace_printk(msg, sizeof(msg));
	return 0;
}

SEC("kprobe/sys_execve")
int kprobe__sys_execve(void *ctx)
{
	char msg[] = "Hello BPF kprobe!\n";

	bpf_trace_printk(msg, sizeof(msg));
	return 0;
}

char _license[] SEC("license") = "GPL";
u32 _version SEC("version") = LINUX_VERSION_CODE;
