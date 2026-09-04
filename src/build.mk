################################################################################
#
# build.mk -- Makefile fragment for including this NC-SI (DSP0222) responder
# stack into another project's build.
#
# New code written for this port (not from bcm5719-fw).
#
# Usage, from another project's Makefile:
#
#   NCSI_FW_ROOT := path/to/ncsi_fw
#   include $(NCSI_FW_ROOT)/src/build.mk
#
#   SRCS      += $(NCSI_SRCS)
#   CFLAGS    += $(NCSI_CFLAGS)
#
# That alone gets you src/ncsi/ncsi.c (the portable protocol engine) and its
# include path -- nothing else. It deliberately does NOT pull in
# ncsi_board_config.c or a HAL implementation automatically: those are
# board-specific by design (see README.md), and auto-including this repo's
# own already-filled-in ncsi_board_config.c (OCP NIC 3.0 / GPIO20 channel ID)
# into an unrelated project would silently give it the wrong hardware
# config. Add your own board config + HAL source files to your project's own
# SRCS list alongside $(NCSI_SRCS) -- see NCSI_BOARD_CONFIG_TEMPLATE and
# NCSI_HAL_TEMPLATE below if you want to start from this repo's versions.
#
# This file locates its own directory via MAKEFILE_LIST, so it works
# regardless of where the including project keeps/references this repo
# (git submodule, subtree, plain copy, ...) and regardless of the including
# Makefile's own working directory.
#
################################################################################

NCSI_FW_SRC_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
NCSI_FW_NCSI_DIR := $(NCSI_FW_SRC_DIR)/ncsi

# --- Include path ------------------------------------------------------------
NCSI_INC_DIRS := $(NCSI_FW_NCSI_DIR)/include
NCSI_CFLAGS   := $(addprefix -I,$(NCSI_INC_DIRS))

# --- Sources always safe to build as-is --------------------------------------
# ncsi.c has no board-specific content -- it's the DSP0222 command engine,
# portable to any target via the Network.h / ncsi_board_config.h HAL
# contracts. This is the only thing NCSI_SRCS pulls in automatically.
NCSI_SRCS := $(NCSI_FW_NCSI_DIR)/ncsi.c

# --- Board-specific sources: NOT added to NCSI_SRCS, reference only ----------
# Point at this repo's own filled-in board config (GPIO20 = channel ID,
# OCP NIC 3.0) and unfilled HAL template. Use these paths directly only if
# you are genuinely building for the exact same hardware; otherwise copy
# them into your own project and fill in your own board's values --
# see README.md's "Quick start" steps 1-4.
NCSI_BOARD_CONFIG_TEMPLATE := $(NCSI_FW_NCSI_DIR)/ncsi_board_config.c
NCSI_HAL_TEMPLATE          := $(NCSI_FW_NCSI_DIR)/ncsi_hal_template.c

# --- Sanity checks -------------------------------------------------------------
ifeq ($(wildcard $(NCSI_SRCS)),)
$(error build.mk: NCSI_SRCS points at "$(NCSI_SRCS)", which doesn't exist -- \
        check NCSI_FW_ROOT / where this repo actually is)
endif
