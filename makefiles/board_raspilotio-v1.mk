#
# Board-specific definitions for the raspilotio-v1
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM3
CONFIG_BOARD			 = RASPILOTIO_V1

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
