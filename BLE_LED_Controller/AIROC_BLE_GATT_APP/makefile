ifeq ($(WHICHFILE),true)
$(info Processing $(lastword $(MAKEFILE_LIST)))
endif

#
# Basic Configuration
#
APPNAME=BLE_LED_CONTROLLER
TOOLCHAIN=GCC_ARM
CONFIG=Debug
VERBOSE=

# default target
TARGET=CYW920835M2EVB-01

SUPPORTED_TARGETS = \
  CYW920819EVB-02 \
  CYW920819M2EVB-01 \
  CYW920820EVB-02 \
  CYW989820EVB-01 \
  CYW989820M2EVB-01 \
  CYW920719B2Q40EVB-01 \
  CYW920835M2EVB-01 \
  CYBLE-343072-EVAL-M2B \
  CYBLE-333074-EVAL-M2B \
  CYW943012BTEVK-01 \
  CYW955572BTEVK-01 \
  CYW920721M2EVB-03 \
  CYW920820M2EVB-01

#
# Advanced Configuration
#
SOURCES=
INCLUDES=
DEFINES=
VFP_SELECT=
CFLAGS=
CXXFLAGS=
ASFLAGS=
LDFLAGS=
LDLIBS=
LINKER_SCRIPT=
PREBUILD=
POSTBUILD=
FEATURES=

#
# App features/defaults
#
OTA_FW_UPGRADE?=0
BT_DEVICE_ADDRESS?=default
UART?=AUTO
XIP?=xip
TRANSPORT?=UART
ENABLE_DEBUG=0
APP_NAME=BLE_LED_CONTROLLER

# wait for SWD attach
ifeq ($(ENABLE_DEBUG),1)
CY_APP_DEFINES+=-DENABLE_DEBUG=1
endif

CY_APP_DEFINES+=-DWICED_BT_TRACE_ENABLE
CY_APP_DEFINES+=-DDEV_NAME='"$(APP_NAME)"'

# Enable Below Macro to get HCI Traces
# CY_APP_DEFINES+=-DENABLE_HCI_TRACE

#
# Components (middleware libraries)
#
COMPONENTS +=bsp_design_modus

#
# list of any TARGET M2 base boards that are Rev 6 or Rev 4, or BASE2 Rev 2
#
M2_BASE_BOARD_REV4:= CYW920835M2EVB-01 CYW943012BTEVK-01 CYW955572BTEVK-01 CYW920721M2EVB-03
M2_BASE_BOARD_REV6:= CYW920820M2EVB-01
M2_BASE_BOARD2_REV2:= CYW920819M2EVB-01 CYW989820M2EVB-01


# choose appropriate i2c device interface component for kit
ifeq ($(filter $(M2_BASE_BOARD_REV4),$(TARGET)),$(TARGET))
  COMPONENTS +=ambient_light_sensor_lib
  COMPONENTS +=ambient_light_sensor_max44009
  CY_APP_DEFINES+=-DUSE_LIGHT_SENSOR_MAX44009
  $(info USE_LIGHT_SENSOR_MAX44009)
else
  ifeq ($(filter $(M2_BASE_BOARD_REV6) $(M2_BASE_BOARD2_REV2),$(TARGET)),$(TARGET))
    COMPONENTS +=ambient_light_sensor_opt3002_lib
    COMPONENTS +=ambient_light_sensor_opt3002
    CY_APP_DEFINES+=-DUSE_LIGHT_SENSOR_OPT3002
    $(info USE_LIGHT_SENSOR_OPT3002)
  else
    COMPONENTS +=nine_axis_motion_sensor
    CY_APP_DEFINES+=-DUSE_MOTION_SENSOR
    $(info USE_MOTION_SENSOR)
  endif
endif

# nvram_emulation library is provided to support nvram emulation in RAM for designs without flash
# Use library-manager to add the 'btsdk-drivers' library to app if the TARGET device supports nvram emulation
# disabled by default, set to 1 or override on command line to enable
# note that some configurations are adjusted to support demonstrating the nvram_emulation
# these include transport_cfg, max_number_of_buffer_pools, and rpa_refresh_timeout
NVRAM_EMULATION=0
# optionally use HCI communication with a Host MCU as backup storage if NVRAM_EMULATION_HCI is set to 1
NVRAM_EMULATION_HCI=0

ifeq ($(NVRAM_EMULATION),1)
COMPONENTS +=nvram_emulation
ifeq ($(NVRAM_EMULATION_HCI),1)
CY_APP_DEFINES+=-DNVRAM_EMULATION_HCI=1
endif
endif

################################################################################
# Paths
################################################################################

# Path (absolute or relative) to the project
CY_APP_PATH=.

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI><COMMIT><LOCATION>. If the <LOCATION> field
# begins with $$ASSET_REPO$$, then the repo is deposited in the path specified by
# the CY_GETLIBS_SHARED_PATH variable. The default location is one directory level
# above the current app directory.
# This is used with CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

# Absolute path to the compiler (Default: GCC in the tools)
CY_COMPILER_PATH=

# Locate ModusToolbox IDE helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_* \
    $(CY_IDE_TOOLS_DIR))

# If you install ModusToolbox IDE in a custom location, add the path to its
# "tools_X.Y" folder (where X and Y are the version number of the tools
# folder).
CY_TOOLS_PATHS+=

# Default to the newest installed tools folder, or the users override (if it's
# found).
CY_TOOLS_DIR=C:\Infineon\Tools\ModusToolbox\tools_3.6

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS))
endif

# tools that can be launched with "make open CY_OPEN_TYPE=<tool>
CY_BT_APP_TOOLS+=BTSpy ClientControl hello_client

-include internal.mk
ifeq ($(filter $(TARGET),$(SUPPORTED_TARGETS)),)
$(error TARGET $(TARGET) not supported for this application. Edit SUPPORTED_TARGETS in the code example makefile to add new BSPs)
endif
include $(CY_TOOLS_DIR)/make/start.mk
