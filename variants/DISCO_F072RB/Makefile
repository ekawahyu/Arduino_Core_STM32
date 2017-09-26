ARM_NONE_EABI_BIN=/Users/nodino/gcc-arm-none-eabi/bin
OPENOCD_HOME=/opt/local
COPY=cp

# put your *.o targets here, make should handle the rest!
SRCS = board.c hooks.c itoa.c pins_arduino.c syscalls_stm32.c
SRCS += wiring_analog.c wiring_digital.c wiring_shift.c wiring_time.c
SRCS += analog.c clock.c core_callback.c digital_io.c hal_uart_emul.c hw_config.c interrupt.c
SRCS += PinConfigured.c pinmap.c PortNames.c spi_com.c stm32_eeprom.c system_stm32yyxx.c timer.c twi.c 
SRCS += uart_emul.c uart.c usb_device_core.c usb_device_ctlreq.c usb_device_ioreq.c usb_interface.c
SRCS += usbd_hid_composite.c usb_device_cdc.c
SRCS += usbd_cdc_if.c usbd_conf.c usbd_desc.c
SRCS += PeripheralPins.c
CXXSRCS = abi.cpp HardwareSerial.cpp IPAddress.cpp main.cpp new.cpp Print.cpp
CXXSRCS += RingBuffer.cpp Stream.cpp Tone.cpp wiring_pulse.cpp WMath.cpp WString.cpp
CXXSRCS += variant.cpp
INOSRCS = Blink.ino

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)
PROJ_NAME=main

# Location of the STM32F0xx HAL Drivers
SYSTEM=../../system
DRIVER=../../system/Drivers

# Location of the linker scripts
LDSCRIPT_INC=./
LINKER=ldscript.ld

# location of OpenOCD Board .cfg files (only used with 'make program')
OPENOCD_BOARD_DIR=$(OPENOCD_HOME)/scripts/board
OPENOCD_INTERFACE_DIR=$(OPENOCD_HOME)/share/openocd/scripts/interface
OPENOCD_TARGET_DIR=$(OPENOCD_HOME)/share/openocd/scripts/target

# Configuration (cfg) file containing programming directives for OpenOCD
OPENOCD_PROC_FILE=scripts/stm32f0-openocd.cfg

# that's it, no need to change anything below this line!

###################################################

CC=$(ARM_NONE_EABI_BIN)/arm-none-eabi-gcc
CXX=$(ARM_NONE_EABI_BIN)/arm-none-eabi-g++
OBJCOPY=$(ARM_NONE_EABI_BIN)/arm-none-eabi-objcopy
OBJDUMP=$(ARM_NONE_EABI_BIN)/arm-none-eabi-objdump
SIZE=$(ARM_NONE_EABI_BIN)/arm-none-eabi-size

CFLAGS  = -Wall -g -Os
CFLAGS += -mlittle-endian -mcpu=cortex-m0  -march=armv6-m -mthumb
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wl,--gc-sections -Wl,-Map=$(PROJ_NAME).map
CFLAGS += -specs=nano.specs
#CFLAGS += -u _printf_float -u _scanf_float

CFLAGS += -DSTM32F0xx -DSTM32F072xB -DUSBCON -DUSBD_USE_CDC -DUSE_USB_FS ############ need to move/define this somewhere else

CXXFLAGS = -fno-rtti -fno-exceptions -std=gnu++11

###################################################

vpath %.c $(DRIVER)/CMSIS/Device/ST/STM32F0xx/Source/Templates
vpath %.c $(SYSTEM) ../../cores/arduino ../../cores/arduino/stm32
vpath %.c ./usb
vpath %.cpp ../../cores/arduino
vpath %.s $(DRIVER)/CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc
vpath %.a $(DRIVER)
vpath %.ino ../../../examples/Blink

ROOT=$(shell pwd)

CFLAGS += -I. -I./usb -I$(DRIVER) -I$(DRIVER)/CMSIS/Device/ST/STM32F0xx/Include
CFLAGS += -I$(DRIVER)/CMSIS/Device/ST/STM32F0xx/Source/Templates
CFLAGS += -I$(DRIVER)/CMSIS/Include -I$(DRIVER)/STM32F0xx_HAL_Driver/Inc
CFLAGS += -I$(SYSTEM)/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
CFLAGS += -I$(SYSTEM)/Middlewares/ST/STM32_USB_Device_Library/Core/Src
CFLAGS += -I$(SYSTEM)/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
CFLAGS += -I$(SYSTEM)/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src
CFLAGS += -I../../cores/arduino -I../../cores/arduino/stm32

ASMSRCS += startup_stm32f072xb.s ############ need to move/define this somewhere else

OBJS = $(SRCS:.c=.o) $(ASMSRCS:.s=.o) $(CXXSRCS:.cpp=.o) $(INOSRCS:.ino=.o)

###################################################

.PHONY: lib proj

all: lib proj

lib:
	$(MAKE) -C $(DRIVER)

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ -L$(DRIVER) -lstm32f0 -L$(LDSCRIPT_INC) -T$(LINKER)
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	$(OBJDUMP) -St $(PROJ_NAME).elf >$(PROJ_NAME).lst
	$(SIZE) $(PROJ_NAME).elf
	$(COPY) $(PROJ_NAME).elf ../..
	$(COPY) $(PROJ_NAME).bin ../..

%.o: %.ino
	$(COPY) $< $<.cpp
	$(CXX) $(CFLAGS) $(CXXFLAGS) -c $<.cpp -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.cpp
	$(CXX) $(CFLAGS) $(CXXFLAGS) -c $< -o $@

flash-stlink: all $(PROJ_NAME).bin
	$(OPENOCD_HOME)/bin/openocd -f $(OPENOCD_INTERFACE_DIR)/stlink-v2.cfg -f $(OPENOCD_TARGET_DIR)/stm32f0x.cfg -f $(OPENOCD_PROC_FILE) -c "stm_flash `pwd`/$(PROJ_NAME).bin" -c shutdown

flash-jlink: all $(PROJ_NAME).bin
	$(OPENOCD_HOME)/bin/openocd -f $(OPENOCD_INTERFACE_DIR)/jlink.cfg -c "transport select swd" -f $(OPENOCD_TARGET_DIR)/stm32f0x.cfg -f $(OPENOCD_PROC_FILE) -c "stm_flash `pwd`/$(PROJ_NAME).bin" -c shutdown

clean:
	find ./ -name '*~' | xargs rm -f
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
	rm -f $(PROJ_NAME).map
	rm -f $(PROJ_NAME).lst
	rm -f ../../$(PROJ_NAME).elf
	rm -f ../../$(PROJ_NAME).bin

distclean: clean
	$(MAKE) -C $(DRIVER) clean
