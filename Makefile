# Modify as appropriate
STELLARISWARE=/home/phil/tetraquark/StellarisWare

CC=arm-none-eabi-gcc -Wall -Os -g -march=armv7-m -mcpu=cortex-m3 -mthumb -mfix-cortex-m3-ldrd -Wl,--gc-sections
	
main_linked_list.elf: kernel_linked_list.c
	${CC} -o $@ -I${STELLARISWARE} -L${STELLARISWARE}/driverlib/gcc -Tlinkscript.x -Wl,-Map,kernel.map -Wl,--entry,ResetISR kernel_linked_list.c create.s main.c startup_gcc.c syscalls.c rit128x96x4.c thread_list.c-ldriver

.PHONY: clean
clean:
	rm -f *.elf *.map

# vim: noexpandtab  
