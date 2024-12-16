# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Linux/LoongArch kernel.
#

OBJECT_FILES_NON_STANDARD_head.o		:=y
OBJECT_FILES_NON_STANDARD_relocate_kernel.o	:=y

extra-y		:= head.o vmlinux.lds

obj-y		+= cmpxchg.o cpu-probe.o elf.o entry.o genex.o idle.o irq.o \
		   process.o prom.o ptrace.o reset.o setup.o signal.o \
		   syscall.o time.o topology.o traps.o unaligned.o \
		   vdso.o cacheinfo.o io.o inst.o alternative.o bugs.o cmdline.o switch.o

obj-$(CONFIG_ACPI)	+= acpi.o
obj-$(CONFIG_EFI)	+= efi.o

ifdef CONFIG_FUNCTION_TRACER
  ifdef CONFIG_DYNAMIC_FTRACE
    CFLAGS_REMOVE_ftrace_dyn.o = $(CC_FLAGS_FTRACE)
    obj-y += mcount_dyn.o ftrace_dyn.o
  else
    CFLAGS_REMOVE_ftrace.o = $(CC_FLAGS_FTRACE)
    obj-y += mcount.o ftrace.o
  endif
  CFLAGS_REMOVE_time.o = $(CC_FLAGS_FTRACE)
  CFLAGS_REMOVE_perf_event.o = $(CC_FLAGS_FTRACE)
  CFLAGS_REMOVE_inst.o = $(CC_FLAGS_FTRACE)
endif

KASAN_SANITIZE_vdso.o := n

obj-$(CONFIG_STACKTRACE)	+= stacktrace.o
obj-$(CONFIG_MODULES)		+= module.o module-sections.o
obj-$(CONFIG_JUMP_LABEL)	+= jump_label.o

obj-$(CONFIG_CPU_HAS_FPU)	+= fpu.o kfpu.o

obj-$(CONFIG_SMP)		+= smp.o
obj-$(CONFIG_SMP_UP)		+= smp-up.o

obj-$(CONFIG_KPROBES)		+= kprobes.o
obj-$(CONFIG_64BIT)		+= scall64-64.o
obj-$(CONFIG_CPU_HAS_LBT)	+= scall-trans-mips64.o trans_mips_syscalls.o scall-trans-i386.o trans_i386_syscalls.o

obj-$(CONFIG_KGDB)		+= kgdb.o
obj-$(CONFIG_PROC_FS)		+= proc.o
obj-$(CONFIG_MAGIC_SYSRQ)	+= sysrq.o

obj-$(CONFIG_RELOCATABLE)	+= relocate.o

obj-$(CONFIG_KEXEC)		+= machine_kexec.o relocate_kernel.o
obj-$(CONFIG_CRASH_DUMP)	+= crash_dump.o
obj-$(CONFIG_SPINLOCK_TEST)	+= spinlock_test.o

obj-$(CONFIG_HAVE_STD_PC_SERIAL_PORT)	+= 8250-platform.o

obj-$(CONFIG_PERF_EVENTS)	+= perf_event.o perf_regs.o
obj-$(CONFIG_HARDWARE_WATCHPOINTS)     += watch.o

obj-$(CONFIG_UPROBES)		+= uprobes.o

obj-$(CONFIG_CPU_PM)		+= pm.o

obj-$(CONFIG_UNWINDER_GUESS)	+= unwind_guess.o
obj-$(CONFIG_UNWINDER_PROLOGUE)	+= unwind_prologue.o
obj-$(CONFIG_UNWINDER_ORC)		+= unwind_orc.o

obj-$(CONFIG_PARAVIRT)        += paravirt.o
obj-$(CONFIG_PARAVIRT_SPINLOCKS) += paravirt-spinlocks.o

CPPFLAGS_vmlinux.lds		:= $(KBUILD_CFLAGS)
