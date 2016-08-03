
DEVICE = attiny45
CLOCK = 8000000
PROGRAMMER = avrispmkII

CC = avr-gcc
CFLAGS = -Wall -std=gnu99 -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)

SOURCES = $(wildcard *.c)
OBJECTS = $(SOURCES:.c=.o)

FILENAME = main
TARGET = $(FILENAME).elf
HEX = $(FILENAME).hex

all: build size

build: $(SOURCES) $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -c $(FILENAME).c -o $@

size:
	avr-size --format=avr --mcu=$(DEVICE) $(TARGET)

flash: $(HEX)
	avrdude -v -p $(DEVICE) -c $(PROGRAMMER) -U flash:w:$(HEX):i

$(HEX): $(TARGET)
	avr-objcopy -j .text -j .data -O ihex $(TARGET) $(HEX)

clean:
	rm $(OBJECTS) $(TARGET) $(HEX)
