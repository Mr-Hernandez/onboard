CC=arm-none-eabi-gcc
MACH=cortex-m4
MAIN=main
CFLAGS= -g -c -mcpu=$(MACH) -mthumb -std=gnu11 -Wall -O0 -IDrivers/Inc -I.
LDFLAGS= -nostdlib -T stm32_ls.ld -Wl,-Map=final.map


all:$(MAIN).o stm32_startup.o final.elf

main.o:main.c
	$(CC) $(CFLAGS) $^ -o $@

stm32_startup.o:stm32_startup.c
	$(CC) $(CFLAGS) $^ -o $@

final.elf: main.o stm32_startup.o
	$(CC) $(LDFLAGS) $^ -o $@

clean:
	rm -rf *.o *.elf *.map

load:
	openocd -f /usr/share/openocd/scripts/board/stm32f4discovery.cfg
