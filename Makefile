TARGET = main
C_SRCS += main.c fe310_spi.c fe310_gpio.c mrf24j40.c
CFLAGS += -O2 -fno-builtin-printf -DNO_INIT

BSP_BASE = ../../bsp
include $(BSP_BASE)/env/common.mk
