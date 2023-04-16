MICROCONTROLLER=atxmega256a3u
PACKS=/home/caspar/local/share/atmel_packs_xmegaa/1.2.141
PROGRAMMER=avrispmkII
CC=avr-gcc
OBJ2HEX=avr-objcopy
AVRDUDE= avrdude -c $(PROGRAMMER) -p $(MICROCONTROLLER)
CFLAGS=-funsigned-char -funsigned-bitfields -Og -Wall -mmcu=atxmega256a3u -c -std=gnu99
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt
LDFLAGS=-mmcu=atxmega256a3u -B $(PACKS)/gcc/dev/$(MICROCONTROLLER) $(PRINTF_LIB_FLOAT)
TARGET=uart

MAIN_FOLDER = /Users/lucawarm/CLionProjects/Ontwerpen3
SRC = $(MAIN_FOLDER)/src/central_node
BUILD_DIR = $(SRC)

SRC_LIBS = $(MAIN_FOLDER)/lib
OBJS = $(patsubst $(SRC)/%.c,$(BUILD_DIR)/%.o,$(wildcard $(SRC)/*.c))
OBJS_LIBS = $(patsubst $(SRC_LIBS)/%.c, $(BUILD_DIR)/%.o, $(wildcard $(SRC_LIBS)/*.c))

.PHONY: build clean flash

# Build
build: $(BUILD_DIR) $(TARGET).elf

# Program
flash: $(TARGET).elf
	$(AVRDUDE) -e -U flash:w:$<

# Link
$(TARGET).elf: $(OBJS) $(BUILD_DIR)/libutils.a
	$(CC) -o $@ $^ $(LDFLAGS) -L$(SRC) -lutils

# Compile
%.o: $(SRC)/%.c
	$(CC) -c $(CFLAGS) -o $@ $<

-include $(OBJS:.o=.d)


$(BUILD_DIR)/%.o: $(SRC_LIBS)/%.c
	$(CC) -c $(CFLAGS) -o $@ $<

-include $(OBJS_LIBS:.o=.d)

$(BUILD_DIR)/libutils.a: $(OBJS_LIBS)
	ar crs $@ $^

# Clean
clean:
	@rm -f *.obj *.o *.elf *.map *.d *.a
