
MCU = atmega328p
CLOCK = 16000000UL
PORT = /dev/ttyACM0

SRC_DIR = src
INC_DIR = inc
OBJ_DIR = obj

PROGRAMMER = -c arduino -p $(MCU) -P $(PORT) -b 115200

AVRDUDE = avrdude $(PROGRAMMER)
COMPILE = avr-gcc -Wall -Os -g -DF_CPU=$(CLOCK) -mmcu=$(MCU) -I$(INC_DIR)
LINK = avr-gcc -mmcu=$(MCU) -Wl,-u,vfprintf -lprintf_flt -lm

SOURCES := $(shell find $(SRC_DIR) -name "*.c")
OBJECTS := $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SOURCES))

all: clean flash

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(COMPILE) -c $< -o $@

main.elf: $(OBJECTS)
	$(LINK) -o main.elf $(OBJECTS)

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex

flash: main.hex
	$(AVRDUDE) -U flash:w:main.hex:i

clean:
	rm -f main.hex main.elf -R $(OBJ_DIR)