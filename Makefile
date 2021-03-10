# version 0 (alpha)
# 
# This make file for stm32duino core is possibly *dangerous*
# i.e. it compiles the sources (in src, Arduino_Core_STM32 and CMSIS_5) directory 
# and overwrite all the stuff in $(out_dir)
# $(out_dir) is the binary directory where the object files are dumped there
#
# make clean *deletes* the *$(out_dir)* (coded here as bin )
#
# this is a relative path makefile
# the directory structure needs to be
# Root
#   +-- src (your sources e.g. the sketch
#   |        the files has to be cpp, c or h, no ino)
#   |
#   +-- Arduino_Core_STM32 (copy from the stm32duino core)
#   |
#   +-- CMSIS_5
#   |
#   +-- Makefile (this makefile)
# 
# the make needs to be run from the root of this file structure
# the make is relative path from this Root 
# generated object files and binaries (elf, bin) are placed into the
# $(out_dir) - bin
#
# make clean  - *deletes* $(out_dir)
#
# make all  - starts the build
# 
# This software released into the public domain.
#
# Anyone is free to copy, modify, publish, use, compile, sell, or
# distribute this software, either in source code form or as a compiled
# binary, for any purpose, commercial or non-commercial, and by any
# means.
#
# In jurisdictions that recognize copyright laws, the author or authors
# of this software dedicate any and all copyright interest in the
# software to the public domain. 
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
# OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.
# 

# this folder is deleted with make clean
out_dir := build

# this is the name of the subdirectory under Arduino_Core_STM32/variants
# it needs to match, this caters to a single variant
variant := DISCO_F407VG
build_series := STM32F4xx
build_board := DISCO_F407VG
product_line := STM32F407xx

build_flash_offset := 0
upload_maximum_size := 1048576
upload_maximum_data_size := 131072

# these are the defines passed to gcc, g++ and the assembler
# use += to add more definitions
defines := ARDUINO=10813 
defines += $(build_series)
defines += ARDUINO_$(build_board) 
# -DARDUINO_ARCH_{build.arch} 
defines += BOARD_NAME=\"$(build_board)\"
defines += $(product_line)
#defines += VECT_TAB_SRAM
#defines += VECT_TAB_OFFSET=0
defines += CONFIG_MAPLE_MINI_NO_DISABLE_DEBUG
defines += DEBUG_LEVEL=DEBUG_NONE
defines += F_CPU=168000000L
defines += CRYSTAL_FREQ=8
# defines += HAL_UART_MODULE_ENABLED
defines += HAL_PCD_MODULE_ENABLED
defines += USBCON
# defines += USBD_VID=0x0483
# defines += USBD_PID=0x0004
# defines += USBD_VID=0
# defines += USBD_PID=0
# defines += USB_MANUFACTURER=\"Unknown\"
# defines += USB_PRODUCT=\"$(build_board)\"

# usb serial
defines += USBD_USE_CDC

# HID keyboard and mouse 
#defines += USBD_USE_HID_COMPOSITE
#defines += LED_BUILTIN=PC13

# points to the root folder of stm32duino core
# this should be 1 level above STM32F1 folder
# use of this is not encouraged, it is safer to copy the Arduino_Core_STM32 and CMSIS_5 
# folder into the current directory
# omit the final slash after the directory
core_root := Arduino_Core_STM32
build_core_path = $(core_root)/cores/arduino
build_system_path = $(core_root)/system
CMSIS_path := CMSIS/5.5.1
# This is for the CMSIS math lib
CMSIS_LD_PATH := $(CMSIS_path)/CMSIS/DSP/Lib/GCC
cmsis_lib_gcc := arm_cortexM4lf_math

# these are the source directories for the libmaple core
# and variant, normally they stay unchanged
# if you want to place the libmaple core directory somewhere else, 
# define core_root above
coredir := $(core_root)/cores/arduino
coredir += $(core_root)/libraries/SrcWrapper/src
#coredir += $(build_system_path)/Drivers/$(build_series)_HAL_Driver/Src
#coredir += $(build_system_path)/$(build_series)
#coredir += $(build_system_path)/Middlewares/ST/STM32_USB_Device_Library/Core/Src
variantdir := $(core_root)/variants/$(variant)
coredir += $(variantdir)


# source directories
# these are the initial directories to search for sources
# relative to this build (root) directory
# if you use libraries either put them in a sub-directory in src
srcdir := src
# or add it here
#srcdir += library1


#this is the ld script in STM32F1/variants/$(variant)/ld to use
#ldscript := bootloader_20.ld
ldscript := ldscript.ld

#the includes i.e. the -Ipath needs to be exlicitly stated
#no automatic recursive searching
#if you use libraries you may want to add it here
includedir := $(srcdir)
includedir += $(variantdir)
includedir += $(CMSIS_path)/CMSIS/Core/Include
includedir += $(build_system_path)/Drivers/CMSIS/Device/ST/$(build_series)/Include/
#includedir += $(build_system_path)/Drivers/CMSIS/Device/ST/$(build_series)/Source/Templates/gcc/ 
includedir += $(CMSIS_path)/CMSIS/DSP/Include
includedir += $(build_core_path)
includedir += $(build_core_path)/avr
includedir += $(build_core_path)/stm32
includedir += $(build_core_path)/stm32/LL 
includedir += $(build_core_path)/stm32/usb 
includedir += $(build_core_path)/stm32/usb/hid 
includedir += $(build_core_path)/stm32/usb/cdc 
includedir += $(build_system_path)/Drivers/$(build_series)_HAL_Driver/Inc 
includedir += $(build_system_path)/Drivers/$(build_series)_HAL_Driver/Src 
includedir += $(build_system_path)/$(build_series)
includedir += $(build_system_path)/Middlewares/ST/STM32_USB_Device_Library/Core/Inc 
includedir += $(build_system_path)/Middlewares/ST/STM32_USB_Device_Library/Core/Src


#if you use core_root, you would need to add that as a prefix
#includedir := $(addprefix $(core_root)/,$(includedir))

# update this to match 
# this should be the install base location of ARM_NONE_EABI_GCC toolchain
#ARM_NONE_EABI_PATH := /mingw64
ARM_NONE_EABI_PATH :=D:\Dev-Tools\gcc-arm-8
# this should be the location of the arm standard libraries c & c++
# the arm-none-eabi/lib select the folder matching the arch
# compile options e.g. thumb and v7-m
LD_TOOLCHAIN_PATH := $(ARM_NONE_EABI_PATH)/arm-none-eabi/lib/thumb/v7e-m+fp


# recursive wildcard function, call with params:
#  - start directory (finished with /) or empty string for current dir
#  - glob pattern
# (taken from http://blog.jgc.org/2011/07/gnu-make-recursive-wildcard-function.html)
rwildcard=$(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2) $(filter $(subst *,%,$2),$d))

cfiles := $(strip $(foreach s, $(srcdir), $(call rwildcard,$(s),*.c)))
cxxfiles := $(strip $(foreach s, $(srcdir), $(call rwildcard,$(s),*.cpp)))
asfiles := $(strip $(foreach s, $(srcdir), $(call rwildcard,$(s),*.s)))

core_cfiles += $(subst $(core_root)/,,$(strip $(foreach s, $(coredir), $(call rwildcard,$(s),*.c))))
core_cxxfiles += $(subst $(core_root)/,,$(strip $(foreach s, $(coredir), $(call rwildcard,$(s),*.cpp))))
core_asfiles += $(subst $(core_root)/,,$(strip $(foreach s, $(coredir), $(call rwildcard,$(s),*.s))))

#exclude due to repeat system init
core_cfiles := $(filter-out libraries/SrcWrapper/src/stm32/system_stm32yyxx.c,$(core_cfiles))
#$(info $(core_cfiles))

src_files := $(cfiles) $(cxxfiles) $(asfiles) 
core_files := $(core_cfiles) $(core_cxxfiles) $(core_asfiles)
files := $(src_files) $(core_files)
sdirs := $(sort $(dir $(files)))

#hfiles := $(foreach s, $(includedir), $(call rwildcard,$(s),*.h))
#hfiles += $(foreach s, $(srcdir), $(call rwildcard,$(s),*.h))
#incdirs = $(sort $(dir $(hfiles)))

TOOLPREFIX := arm-none-eabi-
CC      = $(ARM_NONE_EABI_PATH)/bin/$(TOOLPREFIX)gcc
CXX     = $(ARM_NONE_EABI_PATH)/bin/$(TOOLPREFIX)g++
AS      = $(ARM_NONE_EABI_PATH)/bin/$(TOOLPREFIX)as
OBJCOPY = $(ARM_NONE_EABI_PATH)/bin/$(TOOLPREFIX)objcopy
OBJDUMP = $(ARM_NONE_EABI_PATH)/bin/$(TOOLPREFIX)objdump
AR      = $(ARM_NONE_EABI_PATH)/bin/$(TOOLPREFIX)ar
SIZE    = $(ARM_NONE_EABI_PATH)/bin/$(TOOLPREFIX)size
NM      = $(ARM_NONE_EABI_PATH)/bin/$(TOOLPREFIX)nm
LD      = $(ARM_NONE_EABI_PATH)/bin/$(TOOLPREFIX)ld
DFUUTIL = dfu-util


RM      = /usr/bin/rm
MKDIR   = /usr/bin/mkdir -p
TEST	= /usr/bin/test

DEFINES := $(addprefix -D,$(defines))
INCLUDES := $(addprefix -I,$(includedir))

#optimise
# -O0 - none
# -Os - optimise size
# -O1 - optimise
# -O2 - optimise more
# -O3 - optimise most
# -Og - optimise debug
OFLAG := -Os

#debug
# default none
# -g - debug
# -g1 - minimal
# -g3 - maximal
DFLAG = -g1

COMMON_OFLAGS := -Wl,--gc-sections $(OFLAG) $(DFLAG) \
                 -ffunction-sections -fdata-sections \
                 -fmessage-length=0 -fsigned-char \
                 -ffreestanding -fno-move-loop-invariants \
                 --specs=nano.specs -Wall -Wextra

FPU_FLAGS := -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant
#FPU_FLAGS := -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -fsingle-precision-constant


TARGET_FLAGS += -mcpu=cortex-m4 -march=armv7e-m+fp -mthumb \
                 $(FPU_FLAGS) \
                 $(INCLUDES) $(DEFINES)
                 
GLOBAL_CFLAGS := $(COMMON_OFLAGS) $(TARGET_FLAGS)
TARGET_CFLAGS := 
GLOBAL_CXXFLAGS := -fno-rtti -fno-exceptions \
                   -fno-use-cxa-atexit -fno-threadsafe-statics \
                   $(COMMON_OFLAGS) \
                   $(TARGET_FLAGS)
TARGET_CXXFLAGS :=     
GLOBAL_ASFLAGS  := $(TARGET_FLAGS)
#TARGET_ASFLAGS  := -Wl,--gc-sections $(OFLAG) $(DFLAG) -Xassembler -Wall
TARGET_ASFLAGS  := -Wl,--gc-sections $(OFLAG) $(DFLAG)
LD_SCRIPT_PATH := $(variantdir)/$(ldscript)

#				   -nostdlib 
#                  -nodefaultlibs
#                  -nostartfiles  
#                  -Wl,--gc-sections
GLOBAL_LDFLAGS := --specs=nano.specs \
                  -Xlinker --gc-sections

TARGET_LDFLAGS := $(TARGET_FLAGS) \
                  -Xlinker -T$(LD_SCRIPT_PATH) \
                  -L $(variantdir) \
                  

CFLAGS   = $(GLOBAL_CFLAGS) $(TARGET_CFLAGS)
CXXFLAGS = $(GLOBAL_CXXFLAGS) $(TARGET_CXXFLAGS)
CPPFLAGS =
ASFLAGS  = $(GLOBAL_ASFLAGS) $(TARGET_ASFLAGS)

# Add toolchain directory to LD search path
TOOLCHAIN_LDFLAGS := -L $(LD_TOOLCHAIN_PATH) -L $(CMSIS_LD_PATH) -l $(cmsis_lib_gcc)
LDFLAGS = $(GLOBAL_LDFLAGS) $(TARGET_LDFLAGS) $(TOOLCHAIN_LDFLAGS)

#build lists of object files relative to $(out_dir)
COBJS = $(addprefix $(out_dir)/,$(patsubst %.c,%.o,$(cfiles)))
CXXOBJS = $(addprefix $(out_dir)/,$(patsubst %.cpp,%.o,$(cxxfiles)))
ASOBJS =  $(addprefix $(out_dir)/,$(patsubst %.s,%.o,$(asfiles)))

CORE_COBJS = $(addprefix $(out_dir)/,$(patsubst %.c,%.o,$(core_cfiles)))
CORE_CXXOBJS = $(addprefix $(out_dir)/,$(patsubst %.cpp,%.o,$(core_cxxfiles)))
CORE_ASOBJS =  $(addprefix $(out_dir)/,$(patsubst %.s,%.o,$(core_asfiles)))


variant.ELF = $(out_dir)/$(variant).elf
variant.BIN = $(out_dir)/$(variant).bin
variant.HEX = $(out_dir)/$(variant).hex

.PHONY: all clean mkdir flash
all: mkdir $(variant.BIN) $(variant.HEX)
	@echo
	@$(SIZE) $(variant.ELF)
	@echo
	@ls -l $(variant.BIN)
	@echo
	@ls -l $(variant.HEX)
	@echo
	@$(OBJDUMP) --section-headers $(variant.ELF)
	@echo
	@echo Source dirs
	@echo $(srcdir) $(coredir) | sed 's/ /\n/g'
	@echo
	@echo $(sort $(dir $(src_files))) | sed 's/ /\n/g'
	@echo $(addprefix $(core_root)/,$(sort $(dir $(core_files)))) | sed 's/ /\n/g'
	@echo
	@echo Includes
	@echo $(INCLUDES) | sed 's/ /\n/g'
	@echo
	@echo Defines
	@echo $(DEFINES) | sed 's/ /\n/g'
	    		

install: all
	$(DFUUTIL) -d 0483:df11 -a 0 -s 0x8000000 -D $(variant.BIN)
	
$(variant.BIN): $(variant.ELF)
	$(OBJCOPY) -v -Obinary $(variant.ELF) $@ 

$(variant.HEX): $(variant.ELF)
	$(OBJCOPY) -v -O ihex $(variant.ELF) $@ 
	
$(variant.ELF): $(ASOBJS) $(COBJS) $(CXXOBJS) \
                $(CORE_COBJS) $(CORE_CXXOBJS) $(CORE_ASOBJS)
	$(CXX) $(LDFLAGS) -o $@ -Wl,-Map,$(out_dir)/$(variant).map $+

# General directory independent build rules, generate dependency information
$(COBJS): $(out_dir)/%.o: %.c
	$(CC) $(CFLAGS) -MMD -MP -MF $(@:%.o=%.d) -MT $@ -o $@ -c $<

$(CXXOBJS): $(out_dir)/%.o: %.cpp
	$(CXX) $(CXXFLAGS) -MMD -MP -MF $(@:%.o=%.d) -MT $@ -o $@ -c $<

$(ASOBJS): $(out_dir)/%.o: %.s
	@echo $(ASOBJS)
	$(CC) $(ASFLAGS) -MMD -MP -MF $(@:%.o=%.d) -MT $@ -o $@ -c $<

$(CORE_COBJS): $(out_dir)/%.o: $(core_root)/%.c
	$(CC) $(CFLAGS) -MMD -MP -MF $(@:%.o=%.d) -MT $@ -o $@ -c $<

$(CORE_CXXOBJS): $(out_dir)/%.o: $(core_root)/%.cpp
	$(CXX) $(CXXFLAGS) -MMD -MP -MF $(@:%.o=%.d) -MT $@ -o $@ -c $<

$(CORE_ASOBJS): $(out_dir)/%.o: $(core_root)/%.s
	$(CC) $(ASFLAGS) -MMD -MP -MF $(@:%.o=%.d) -MT $@ -o $@ -c $<


# create the build directories
tgtdirs := $(addsuffix .dir,$(addprefix $(out_dir)/,$(sdirs)))

mkdir: $(tgtdirs)

# the .dir file is a marker file that the directory is created
$(tgtdirs) : $(out_dir)/.dir
	$(MKDIR) $(dir $@)
	@touch $@

$(out_dir)/.dir:
	$(MKDIR) $(dir $@)
	@touch $@
	
clean:
	@echo clean
	$(RM) -r $(out_dir)
	
DEPENDS := $(COBJS:%.o=%.d) $(CXXOBJS:%.o=%.d) $(ASOBJS:%.o=%.d)
DEPENDS += $(CORE_COBJS:%.o=%.d) $(CORE_CXXOBJS:%.o=%.d) $(CORE_ASOBJS:%.o=%.d)

-include $(DEPENDS)

flash: $(variant.ELF) $(variant.HEX)
	st-flash --reset --format ihex write $(variant.HEX)