/*
 * STM32F103
 *
 * Blink LED
 *
 */


#include "regs.h"

int main(void){
    // Emable GPIO C
    (*(volatile uint32_t *)(0x40021018)) |= (1 << 4);

    // Set GPIO C13 as output
    (*(volatile uint32_t *)(0x40011004)) |= (0x02 << ((13 - 8) * 4));
 
    while(1) {
        for(int i = 0; i < 50000; i++) __asm__("nop");
        (*(volatile uint32_t *)(0x4001100C)) |= (0x01 << 13);
        for(int i = 0; i < 50000; i++) __asm__("nop");
        (*(volatile uint32_t *)(0x4001100C)) &= ~(0x01 << 13);
    }
}


