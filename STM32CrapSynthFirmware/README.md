Startup file and linker script were modified. The correct size of CCMRAM (16KiB instead of 8KiB) was specified. CCMRAM sections for code and data were added in linker script. Startup file was modified to include CCMRAM copy of vector table, and copying routine was added to copy the part of the firmware from Flash to CCMRAM.

The main program copies interrupt vector table to CCMRAM and writes its new address to VTOR register of SCB.