#==============================================================================#
# STM8S103F3P6 HC-SR04 + 2-Digit 7-Seg LED - Makefile for SDCC
#==============================================================================#

# Toolchain
CC       = sdcc
PROGRAMMER = stm8flash

# Target MCU
MCU      = stm8s103f3

# Project name
PROJECT  = project5

# Source files
SRCS     = main.c hcsr04.c segled.c uart.c beep.c

# Compiler flags
CFLAGS   = -mstm8 --std-c99 -I./

# Output files
IHX      = $(PROJECT).ihx
HEX      = $(PROJECT).hex
LST      = $(PROJECT).lst
MAP      = $(PROJECT).map
CDB      = $(PROJECT).cdb

#==============================================================================#
# Build targets
#==============================================================================#

.PHONY: all clean flash size

all: $(IHX)

$(IHX): $(SRCS)
	$(CC) $(CFLAGS) -c main.c
	$(CC) $(CFLAGS) -c hcsr04.c
	$(CC) $(CFLAGS) -c segled.c
	$(CC) $(CFLAGS) -c uart.c
	$(CC) $(CFLAGS) -c beep.c
	$(CC) $(CFLAGS) main.rel hcsr04.rel segled.rel uart.rel beep.rel -o $@

# Generate Intel HEX from IHX
hex: $(IHX)
	packihx $(IHX) > $(HEX)

# Show memory usage
size: $(IHX)
	@echo "----------------------------------------"
	@echo "  Memory usage:"
	@echo "  Flash:  up to 8 KB (0x8000 - 0x9FFF)"
	@echo "  RAM:    up to 1 KB"
	@echo "----------------------------------------"

# Flash the chip via ST-LINK/V2
flash: $(IHX)
	$(PROGRAMMER) -c stlinkv2 -p $(MCU) -w $<

# Flash via serial bootloader (UART1)
flash_uart: $(IHX)
	stm8gal -p /dev/ttyUSB0 -w $< -R 3

# Clean build artifacts
clean:
	rm -f *.ihx *.hex *.lk *.lst *.map *.sym *.cdb *.asm *.rel *.rst

# Show help
help:
	@echo "Available targets:"
	@echo "  all         Build the project (default)"
	@echo "  hex         Generate Intel HEX file"
	@echo "  size        Show memory usage info"
	@echo "  flash       Flash via ST-LINK/V2 (stm8flash)"
	@echo "  flash_uart  Flash via UART bootloader (stm8gal)"
	@echo "  clean       Remove all build artifacts"
