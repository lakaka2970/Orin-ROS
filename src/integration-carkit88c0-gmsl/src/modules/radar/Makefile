# Subdirectory that contains board-specific files
BOARD_SPECIFICS_DIR := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))/board_specifics

# Auto-detect available board subdirectories
AVAILABLE_BOARDS := $(shell find $(BOARD_SPECIFICS_DIR) -maxdepth 1 -type d ! -path $(BOARD_SPECIFICS_DIR) -exec basename {} \;)

# Check if any board directories are present
ifeq ($(AVAILABLE_BOARDS),)
    $(error No board directories found)
endif

ifeq ($(words $(AVAILABLE_BOARDS)),1)
    ifndef BOARD_TYPE
        BOARD_TYPE := $(AVAILABLE_BOARDS)
    endif
else
    ifndef BOARD_TYPE
        $(error Multiple board directories found: $(AVAILABLE_BOARDS). Please specify: make BOARD_TYPE=<board>)
    endif

    ifeq ($(filter $(BOARD_TYPE),$(AVAILABLE_BOARDS)),)
        $(error Invalid BOARD_TYPE '$(BOARD_TYPE)'. Available options are: $(AVAILABLE_BOARDS))
    endif
endif

obj-m += radar.o

# Add board-specific include path
ccflags-y += -I$(BOARD_SPECIFICS_DIR)/$(BOARD_TYPE)

# Disable outdated warning 
ccflags-y += -Wno-declaration-after-statement

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
BUILD_DIR := $(PWD)/build

all: $(BUILD_DIR)
	@echo "Building radar module for board $(BOARD_TYPE)"
	$(MAKE) -C $(KDIR) M=$(PWD) modules
	@mkdir -p $(BUILD_DIR)
	@mv -f *.ko $(BUILD_DIR)/ 2>/dev/null || true
	@mv -f *.mod $(BUILD_DIR)/ 2>/dev/null || true
	@mv -f *.mod.c $(BUILD_DIR)/ 2>/dev/null || true
	@mv -f *.mod.o $(BUILD_DIR)/ 2>/dev/null || true
	@mv -f *.o $(BUILD_DIR)/ 2>/dev/null || true
	@mv -f modules.order $(BUILD_DIR)/ 2>/dev/null || true
	@mv -f Module.symvers $(BUILD_DIR)/ 2>/dev/null || true

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -rf $(BUILD_DIR)

install: all
	sudo insmod $(BUILD_DIR)/radar.ko

.PHONY: all clean install