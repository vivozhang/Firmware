#
# Board-specific definitions for the raspilotio-beta
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM3
CONFIG_BOARD			 = RASPILOTIO_BETA

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
