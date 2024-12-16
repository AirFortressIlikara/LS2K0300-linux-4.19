#
# This file is subject to the terms and conditions of the GNU General Public
# License.
#
# Copyright (C) 2020 Loongson Technology
# Author: Huacai Chen <chenhuacai@loongson.cn>
#

OBJECT_FILES_NON_STANDARD	:= y

include $(srctree)/arch/loongarch/Kbuild.platforms

# set the default size of the mallocing area for decompressing
BOOT_HEAP_SIZE := 0x400000

ifdef CONFIG_KASAN
KASAN_SANITIZE := n
endif

# Disable Function Tracer
KBUILD_CFLAGS := $(filter-out $(CC_FLAGS_FTRACE), $(KBUILD_CFLAGS))

KBUILD_CFLAGS := $(filter-out -fstack-protector, $(KBUILD_CFLAGS))

KBUILD_CFLAGS += -fpic

KBUILD_CFLAGS := $(KBUILD_CFLAGS) -D__KERNEL__ \
	-DBOOT_HEAP_SIZE=$(BOOT_HEAP_SIZE) \
	-D"VMLINUX_LOAD_ADDRESS_ULL=$(VMLINUX_LOAD_ADDRESS)ull" \
	-D"KERNEL_ENTRY=$(VMLINUX_ENTRY_ADDRESS)ull" \

KBUILD_AFLAGS := $(KBUILD_AFLAGS) -D__ASSEMBLY__ \
	-DBOOT_HEAP_SIZE=$(BOOT_HEAP_SIZE) \
	-DKERNEL_ENTRY=$(VMLINUX_ENTRY_ADDRESS)

# decompressor objects (linked with vmlinuz)
vmlinuzobjs-y := $(obj)/head.o $(obj)/decompress.o $(obj)/string.o

ifdef CONFIG_DEBUG_ZBOOT
vmlinuzobjs-$(CONFIG_DEBUG_ZBOOT)	+= $(obj)/dbg.o
vmlinuzobjs-$(CONFIG_DEBUG_ZBOOT)	+= $(obj)/uart-16550.o
endif

vmlinuzobjs-$(CONFIG_KERNEL_XZ) += $(obj)/ashldi3.o

extra-y += ashldi3.c
$(obj)/ashldi3.c: $(obj)/%.c: $(srctree)/lib/%.c FORCE
	$(call if_changed,shipped)

targets := $(notdir $(vmlinuzobjs-y))

targets += vmlinux.bin
OBJCOPYFLAGS_vmlinux.bin := $(OBJCOPYFLAGS) -O binary -R .comment -S
$(obj)/vmlinux.bin: vmlinux FORCE
	$(call if_changed,objcopy)

tool_$(CONFIG_KERNEL_GZIP)    = gzip
tool_$(CONFIG_KERNEL_BZIP2)   = bzip2
tool_$(CONFIG_KERNEL_LZ4)     = lz4
tool_$(CONFIG_KERNEL_LZMA)    = lzma
tool_$(CONFIG_KERNEL_LZO)     = lzo
tool_$(CONFIG_KERNEL_XZ)      = xzkern

targets += vmlinux.bin.z
$(obj)/vmlinux.bin.z: $(obj)/vmlinux.bin FORCE
	$(call if_changed,$(tool_y))

targets += piggy.o dummy.o
OBJCOPYFLAGS_piggy.o := --add-section=.image=$(obj)/vmlinux.bin.z \
			--set-section-flags=.image=contents,alloc,load,readonly,data
$(obj)/piggy.o: $(obj)/dummy.o $(obj)/vmlinux.bin.z FORCE
	$(call if_changed,objcopy)

HOSTCFLAGS_calc_vmlinuz_load_addr.o += $(LINUXINCLUDE)

# Calculate the load address of the compressed kernel image
hostprogs-y := calc_vmlinuz_load_addr

ifneq ($(zload-y),)
VMLINUZ_LOAD_ADDRESS := $(zload-y)
else
VMLINUZ_LOAD_ADDRESS = $(shell $(obj)/calc_vmlinuz_load_addr \
		$(obj)/vmlinux.bin $(VMLINUX_LOAD_ADDRESS))
endif
UIMAGE_LOADADDR = $(VMLINUZ_LOAD_ADDRESS)

vmlinuzobjs-y += $(obj)/piggy.o

quiet_cmd_zld = LD      $@
      cmd_zld = $(LD) $(KBUILD_LDFLAGS) -Ttext $(VMLINUZ_LOAD_ADDRESS) -T $< $(vmlinuzobjs-y) -o $@
quiet_cmd_strip = STRIP	  $@
      cmd_strip = $(STRIP) -s $@
vmlinuz: $(src)/ld.script $(vmlinuzobjs-y) $(obj)/calc_vmlinuz_load_addr
	$(call cmd,zld)
	$(call cmd,strip)

efiobjs-y := $(obj)/efi-header.o $(obj)/decompress.o $(obj)/piggy.o \
		$(obj)/string.o $(objtree)/drivers/firmware/efi/libstub/lib.a

efiobjs-$(CONFIG_DEBUG_ZBOOT)	+= $(obj)/dbg.o
efiobjs-$(CONFIG_DEBUG_ZBOOT)	+= $(obj)/uart-16550.o

targets += $(notdir $(efiobjs-y))

quiet_cmd_efild = LD      $@
      cmd_efild = $(LD) $(KBUILD_LDFLAGS) -T $< $(efiobjs-y) -o $@

quiet_cmd_eficopy = OBJCOPY $@
cmd_eficopy = $(OBJCOPY) $(OBJCOPYFLAGS) -O binary -R .comment -S $@ $@

vmlinuz.efi: $(src)/ld.script $(efiobjs-y)
	$(call cmd,efild)
	$(call cmd,eficopy)

clean-files += $(objtree)/vmlinuz
clean-files += $(objtree)/vmlinuz.efi
clean-files += $(objtree)/vmlinuz.ecoff
clean-files += $(objtree)/vmlinuz.bin
clean-files += $(objtree)/vmlinuz.srec
