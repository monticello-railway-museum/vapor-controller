CFLAGS=-Os -g -Wall -mmcu=atmega328p -DF_CPU=16000000

%.o: %.c
	avr-gcc $(CFLAGS) -c $<

%.elf: %.o
	avr-gcc $(CFLAGS) -o $@ $<

%.hex: %.elf
	avr-objcopy -O ihex -R .eeprom $< $@
