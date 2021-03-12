#include "bootloader.h"

__attribute__((section(".noinit"))) uint32_t jump_to_bootloader_magic;

void check_bootloader() {
    if (jump_to_bootloader_magic == JUMP_TO_BOOTLOADER_MAGIC_CODE) {
        void (*bootloader)(void) = (void (*)(void)) (*((uint32_t *) (SYSMEM_RESET_VECTOR+4)));
        jump_to_bootloader_magic = 0;
        __set_MSP(*((uint32_t *)(SYSMEM_RESET_VECTOR + 0)));
        bootloader();
    }
}

void reset_to_bootloader() {
    jump_to_bootloader_magic = 0xDEADBEEF;

    

    // Warning! If RESET PIN is connected with pull-up, hardware reset is needed because NVIC_SystemReset() will hang!
    NVIC_SystemReset();	
}
