# SPDX-License-Identifier: GPL-2.0
# Objects to go into the VDSO.
OBJECT_FILES_NON_STANDARD		:= y

ifdef CONFIG_KASAN
KASAN_SANITIZE := n
endif

obj-vdso-y := elf.o gettimeofday.o sigreturn.o

# Common compiler flags between ABIs.
ccflags-vdso := \
	$(filter -I%,$(KBUILD_CFLAGS)) \
	$(filter -E%,$(KBUILD_CFLAGS)) \
	$(filter -march=%,$(KBUILD_CFLAGS)) \
	$(filter -m%-float,$(KBUILD_CFLAGS)) \
	-D__VDSO__

ifeq ($(cc-name),clang)
ccflags-vdso += $(filter --target=%,$(KBUILD_CFLAGS))
endif

cflags-vdso := $(ccflags-vdso) \
	$(filter -W%,$(filter-out -Wa$(comma)%,$(KBUILD_CFLAGS))) \
	-O2 -g -fno-strict-aliasing -fno-common -fno-builtin -G0 \
	-DDISABLE_BRANCH_PROFILING \
	$(call cc-option, -fno-asynchronous-unwind-tables) \
	$(call cc-option, -fno-stack-protector)
aflags-vdso := $(ccflags-vdso) \
	-D__ASSEMBLY__ -Wa,-gdwarf-2

#
# For the pre-R6 code in arch/loongarch/vdso/vdso.h for locating
# the base address of VDSO, the linker will emit a R_LOONGARCH_PC32
# relocation in binutils > 2.25 but it will fail with older versions
# because that relocation is not supported for that symbol. As a result
# of which we are forced to disable the VDSO symbols when building
# with < 2.25 binutils on pre-R6 kernels. For more references on why we
# can't use other methods to get the base address of VDSO please refer to
# the comments on that file.
#
ifeq ($(call ld-ifversion, -lt, 225000000, y),y)
  $(warning LOONGARCH VDSO requires binutils >= 2.25)
  obj-vdso-y := $(filter-out gettimeofday.o, $(obj-vdso-y))
  ccflags-vdso += -DDISABLE_LOONGARCH_VDSO
endif

# VDSO linker flags.
VDSO_LDFLAGS := \
	-Wl,-Bsymbolic -Wl,--no-undefined -Wl,-soname=linux-vdso.so.1 \
	$(addprefix -Wl$(comma),$(filter -E%,$(KBUILD_CFLAGS))) \
	-nostdlib -shared \
	$(call cc-ldoption, -Wl$(comma)--hash-style=sysv) \
	$(call cc-ldoption, -Wl$(comma)--build-id)

GCOV_PROFILE := n

#
# Shared build commands.
#

quiet_cmd_vdsold = VDSO    $@
      cmd_vdsold = $(CC) $(c_flags) $(VDSO_LDFLAGS) \
                   -Wl,-T $(filter %.lds,$^) $(filter %.o,$^) -o $@

quiet_cmd_vdsoas_o_S = AS       $@
      cmd_vdsoas_o_S = $(CC) $(a_flags) -c -o $@ $<

# Strip rule for the raw .so files
$(obj)/%.so.raw: OBJCOPYFLAGS := -S
$(obj)/%.so.raw: $(obj)/%.so.dbg.raw FORCE
	$(call if_changed,objcopy)

hostprogs-y := genvdso

quiet_cmd_genvdso = GENVDSO $@
define cmd_genvdso
	$(foreach file,$(filter %.raw,$^),cp $(file) $(file:%.raw=%) &&) \
	$(obj)/genvdso $(<:%.raw=%) $(<:%.dbg.raw=%) $@ $(VDSO_NAME)
endef

#
# Build native VDSO.
#

native-abi := $(filter -mabi=%,$(KBUILD_CFLAGS))

targets += $(obj-vdso-y)
targets += vdso.lds
targets += vdso.so.dbg.raw vdso.so.raw
targets += vdso.so.dbg vdso.so
targets += vdso-image.c

obj-vdso := $(obj-vdso-y:%.o=$(obj)/%.o)

$(obj-vdso): KBUILD_CFLAGS := $(cflags-vdso) $(native-abi)
$(obj-vdso): KBUILD_AFLAGS := $(aflags-vdso) $(native-abi)

$(obj)/vdso.lds: KBUILD_CPPFLAGS := $(ccflags-vdso) $(native-abi)

$(obj)/vdso.so.dbg.raw: $(obj)/vdso.lds $(obj-vdso) FORCE
	$(call if_changed,vdsold)

$(obj)/vdso-image.c: $(obj)/vdso.so.dbg.raw $(obj)/vdso.so.raw \
                     $(obj)/genvdso FORCE
	$(call if_changed,genvdso)

obj-y += vdso-image.o



# FIXME: Need install rule for debug.
# Needs to deal with dependency for generation of dbg by cmd_genvdso...
