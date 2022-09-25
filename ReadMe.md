THIS IS AN OLD README AND UNTIL THIS LINE IS REMOVED JUST IGNORE IT.
I MIGHT USE SOME OF IT LATER SO I'M LEAVING IT UP FOR NOW.

This project is from the "Baremetal" section of the Fastbit Embedded Brain 
Academy course found on Udemy and youtube.

This implementation uses a different board than the course.
Board Used: STM32F411VE Discovery board.

Mostly, this means the stm32_startup.c file is different, because the vector 
table for the F411 is different.

To run this you'll need
Make
OpenOCD
GNU Project Debugger (GDB)

The steps to run this are as follows. 
1. plug in board
2. Open terminal to folder containing the files
3. write to the terminal
```
    make
    make load
```    
4. open another terminal
5. write to the new terminal
```
    arm-none-eabi-gdb.exe
    target remote localhost:3333
    monitor reset init
    monitor flash write_image erase final.elf
```
6. Now you can choose to run any of the following
```
    monitor reset
    monitor reset halt
```
7. You can resume or halt the board by using
```
    monitor resume
    monitor halt
```
8. To load symbol table, first the main.o (and maybe main.c) should be run with gcc
	and the -g tag. Then to load the symbol table run the command from gdb.
```
	file main.o
```
	where main.o can be your .o file with whatever you named it. Now you can use gdb 
	commands like "where" and "list" to see code and such.
	
The writer.ccp is just a script I used to generate a lot of the stm32_startup.c
file.
