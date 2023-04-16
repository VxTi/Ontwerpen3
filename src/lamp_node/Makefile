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

OBJS = $(patsubst %.c,%.o,$(wildcard *.c))

.PHONY: clean program target

program: $(TARGET).elf
	$(AVRDUDE) -e -U flash:w:$<

%.elf: $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) -x c -c $(CFLAGS) $<

clean:
	@rm -f *.obj *.o *.elf *.map *.d
