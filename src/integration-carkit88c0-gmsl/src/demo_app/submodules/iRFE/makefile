rwildcard=$(wildcard $1/$2) $(foreach d,$(wildcard $1/*/.),$(call rwildcard,$(d:/.=),$2))

# 1. Determine the current directory
CURRENT_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
UNITTESTDIR = tests/unit_tests

export BUILD_DIR    := build_$(CHIP_TYPE)
BUILD_DIR       	:= $(strip $(BUILD_DIR))

export CHIP_TYPE ?=  # 8181_ES == 8181 B-STEP, 8191_A11, 8191_B11, 8188

# 5. unit tests
unittests: $(UNITTESTDIR)
.PHONY: unittests

$(UNITTESTDIR):
	$(MAKE) -C $@
.PHONY: $(UNITTESTDIR)

unittests_clean:
	$(MAKE) -C $(UNITTESTDIR) clean
