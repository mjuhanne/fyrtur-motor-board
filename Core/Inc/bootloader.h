#include "main.h"

#define SYSMEM_RESET_VECTOR            0x1FFFEC00
#define JUMP_TO_BOOTLOADER_MAGIC_CODE 0xDEADBEEF

extern uint32_t jump_to_bootloader_magic;

void check_bootloader();
void reset_to_bootloader();
