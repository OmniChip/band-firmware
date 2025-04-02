#!/usr/bin/make -f
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2019-2020, OmniChip sp. z o.o.
# Author: Michał Mirosław

BAND_P2 := 1
#BAND_P3 := 1

#USE_HID := 1
#ACCEL_TEST := 1

override PROJECT_NAME := devbrd_band
override SDK_ROOT := SDK
override PROJ_DIR := .
override SDK_CONFIG_FILE := sdk_config.h
override OPT := -O3 -ggdb -DDEBUG -DACCEL_TEST=$(if $(ACCEL_TEST),1,0) # -save-temps -Wp,-dD

GIT_REV_MAJ := $(shell git rev-list origin/master --count 2>/dev/null)
GIT_REV_MIN := $(shell git rev-list origin/master..HEAD --count 2>/dev/null)
GIT_DIRTY := $(shell git diff-index --quiet HEAD 2>/dev/null || echo +)
VERSION := $(GIT_REV_MAJ).$(GIT_REV_MIN)$(GIT_DIRTY)
VER_FILE = $(OUTPUT_DIRECTORY)/fw-version.h

SRC_FILES += \
	$(if $(USE_HID),$(PROJ_DIR)/hids.c) \
	$(PROJ_DIR)/ble.c \
	$(PROJ_DIR)/basics.c \
	$(PROJ_DIR)/bands.c \
	$(PROJ_DIR)/sensor.c \
	$(PROJ_DIR)/vsense.c \
	$(PROJ_DIR)/motor.c \
	$(PROJ_DIR)/pstore.c \
	$(PROJ_DIR)/timebase.c \
	$(PROJ_DIR)/timesync.c \
	$(SDK_ROOT)/components/ble/ble_services/nrf_ble_bms/nrf_ble_bms.c \
	$(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_ppi.c \
	$(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_pwm.c \
	$(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_rtc.c \
	$(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_twim.c \
	$(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_spim.c \
	$(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_swi.c \
	$(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_timer.c \

INC_FOLDERS += . \
	$(SDK_ROOT)/components/ble/ble_services/nrf_ble_bms \

include Makefile.sdk

ifneq ($(BAND_P3),)
CFLAGS := $(filter-out -DBOARD_PCA10040,$(CFLAGS)) -DCUSTOM_BOARD_INC=board-band-p3
ASMFLAGS := $(filter-out -DBOARD_PCA10040,$(ASMFLAGS)) -DCUSTOM_BOARD_INC=board-band-p3
else
ifneq ($(BAND_P2),)
CFLAGS := $(filter-out -DBOARD_PCA10040,$(CFLAGS)) -DCUSTOM_BOARD_INC=board-band-p2
ASMFLAGS := $(filter-out -DBOARD_PCA10040,$(ASMFLAGS)) -DCUSTOM_BOARD_INC=board-band-p2
else
ifneq ($(BAND_P1),)
CFLAGS := $(filter-out -DBOARD_PCA10040,$(CFLAGS)) -DCUSTOM_BOARD_INC=board-band-p1
ASMFLAGS := $(filter-out -DBOARD_PCA10040,$(ASMFLAGS)) -DCUSTOM_BOARD_INC=board-band-p1
endif
endif
endif

OBJCOPY ?= arm-none-eabi-objcopy
OBJCOPY_INPUT_BIN := -I binary -B arm
OBJCOPY_OUTPUT_RO := -O elf32-littlearm #--rename-section .data=.rodata,alloc,load,readonly,data,contents

$(OUTPUT_DIRECTORY)/nrf52832_xxaa.out: \
  LINKER_SCRIPT  := devb_nrf52_s132.ld

empty :=
space := $(empty) $(empty)
objcopy_cvt = $(subst /,_,$(subst .,_,$(subst $(space),_,$(1))))
objcopy_bin_rename = $(foreach sfx, start end size, --redefine-sym _binary_$(1)_$(sfx)=_binary_$(2)_$(sfx))

$(OUTPUT_DIRECTORY)/band.hid.o: $(OUTPUT_DIRECTORY)/band.hid.bin
	$(OBJCOPY) $(OBJCOPY_INPUT_BIN) $(OBJCOPY_OUTPUT_RO) \
		$(call objcopy_bin_rename,$(call objcopy_cvt,$<),band_hid_bin) \
		$< $@

.PHONY: FORCE
FORCE:

VER_CHANGED := $(shell grep -Fq '"$(VERSION)"' $(VER_FILE) 2>/dev/null || echo 1)

$(VER_FILE): Makefile $(if $(VER_CHANGED),FORCE)
	echo '#define FW_VERSION "$(VERSION)"' > $@

$(OUTPUT_DIRECTORY)/band.hid.bin: band.hid.xml
	hidrd-convert -i xml $< $@

$(OUTPUT_DIRECTORY)/band.hid.txt: $(OUTPUT_DIRECTORY)/band.hid.bin
	(hexdump -ve '/1 "%02x "' $< && echo) > $@

$(OUTPUT_DIRECTORY)/band.hid.cdata: band.hid.xml
	hidrd-convert -i xml -o code $< $@

$(OUTPUT_DIRECTORY)/band.hex: $(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex
	mergehex -o $@ -m $< $(SDK_ROOT)/components/softdevice/s132/hex/s132_nrf52_6.1.1_softdevice.hex

.PHONY: combined
combined: $(OUTPUT_DIRECTORY)/band.hex

$(foreach target, $(TARGETS), $(eval $(OUTPUT_DIRECTORY)/$(strip $(target))/hids.c.o: $(OUTPUT_DIRECTORY)/band.hid.cdata))
$(foreach target, $(TARGETS), $(eval $(OUTPUT_DIRECTORY)/$(strip $(target))/basics.c.o: $(VER_FILE)))
