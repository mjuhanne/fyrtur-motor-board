#include "main.h"

// Sets the 'jump_to_bootloader_magic' variable and resets CPU
void reset_to_bootloader();

// Called by SystemInit() during bootup. Checks 'jump_to_bootloader_magic' variable to see if bootloader should be entered
void check_bootloader();

