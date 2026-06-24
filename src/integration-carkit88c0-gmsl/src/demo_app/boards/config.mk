MKDIR        := mkdir -p
RMDIR        := rm -rf

CC           := gcc

# Get the directory where this config.mk file is located
CURR_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
# Remove trailing slash if desired
CURR_DIR := $(patsubst %/,%,$(CURR_DIR))
ROOT_DIR := $(CURR_DIR)/..
PLATFORM     := jetson

# Debug configuration
DEBUG ?= 0
ifeq ($(DEBUG), 1)
    CFLAGS += -g -O0 -DDEBUG
    CXXFLAGS += -g -O0 -DDEBUG
    BUILD_TYPE := debug
else
    CFLAGS += -O2 -DNDEBUG
    CXXFLAGS += -O2 -DNDEBUG
    BUILD_TYPE := release
endif

include $(ROOT_DIR)/submodules/iRFE/makelist.mke

BIN_DIR     := ./bin
OBJ_DIR     := ./obj
SUBMOD_DIR  := ../submodules
SRC_DIR     := $(ROOT_DIR)/src
WRP_DIR     := $(ROOT_DIR)/wrappers
PTFRM_DIR   := $(ROOT_DIR)/submodules/platform-jetson/src
PMIC_DIR    := $(ROOT_DIR)/pmic_power
SERDES_ADPT_DIR := $(ROOT_DIR)/serdes_adapters

APP_DIR     := $(subst $(realpath $(ROOT_DIR))/,,$(realpath .))
APP_SRCS    := $(wildcard ./*.c)
APP_OBJS    := $(patsubst ./%,$(OBJ_DIR)/$(APP_DIR)/%.o,$(APP_SRCS))

PTFRM_SRCS  := $(wildcard $(PTFRM_DIR)/*.c)
SRC_SRCS    := $(wildcard $(SRC_DIR)/*.c)
WRP_SRCS    := $(wildcard $(WRP_DIR)/*.c)
SERDES_ADPT_SCRS := $(wildcard $(SERDES_ADPT_DIR)/*.c)

FW_INC_DIR  := $(wildcard $(ROOT_DIR)/ctrx_firmware/$(CTRX_FW_DIR)*)
FW_SRC      := $(wildcard $(FW_INC_DIR)/*.c)

PMIC_SRCS   := $(wildcard $(PMIC_DIR)/*.c)

SRCS        := $(SRC_SRCS) $(FW_SRC) $(WRP_SRCS) $(SERDES_ADPT_SCRS) $(PTFRM_SRCS) $(PMIC_SRCS) $(C_FILES)
OBJS        := $(patsubst $(ROOT_DIR)/%,$(OBJ_DIR)/%.o,$(SRCS))

BIN         := $(BIN_DIR)/$(APP_NAME)

DEFINES      += $(CTRX_DEF)
DEFINE_FLAGS := $(shell for DEF in $(DEFINES); do echo -D$$DEF; done)

CFLAGS += -Wall $(DEFINE_FLAGS)

IRFE_INCLUDES := $(addprefix -I, $(INCLUDE_DIRS))

IFLAGS  := -I. -I$(SERDES_ADPT_DIR)/ -I$(PTFRM_DIR) -I$(SRC_DIR)/ -I$(ROOT_DIR)/include/  $(IRFE_INCLUDES) -I$(WRP_DIR)/ -I$(FW_INC_DIR)/ -I$(ROOT_DIR)/tests/unit_tests/support/ -I$(PMIC_DIR)

LDFLAGS := -lm -lgpiod -li2c

.PHONY: all info clean

all: $(BIN)

$(BIN): $(APP_OBJS) $(OBJS)
	@$(MKDIR) $(@D)
	$(CC) $(C_STANDARD) $^ $(LDFLAGS) -o $@

$(OBJ_DIR)/%.c.o: $(ROOT_DIR)/%.c
	@$(MKDIR) $(@D)
	$(CC) -c $(C_STANDARD) $(IFLAGS) $(CFLAGS) -o $@ $<

info:
	@echo "======= MAIN ======="
	@echo $(APP_DIR)
	@echo $(APP_SRCS)
	@echo $(APP_OBJS)
	@echo "======= SOURCES ======="
	@echo $(SRCS)
	@echo "======= OBJECTS ======="
	@echo $(OBJS)
	@echo "========== CTRX_DEF ==========="
	@echo $(CTRX_DEF)
	@echo "========== CTRX_INFO ==========="
	@echo $(CHIP_TYPE)
	@echo $(CTRX_FW_DIR)
	@echo $(CTRX_DEF)
	@echo $(FW_INC_DIR)
	@echo $(FW_SRC)
	@echo "====================="

# Debug target
debug:
	$(MAKE) DEBUG=1

# Release target
release:
	$(MAKE) DEBUG=0

# Clean build
clean:
	$(RMDIR) $(BIN_DIR) $(OBJ_DIR) $(SUBMOD_DIR)

# Clean debug builds
clean-debug:
	$(MAKE) clean DEBUG=1

.PHONY: debug release clean-debug