/*
 * STM32F103
 *
 * Blink LED
 *
 */


#include "regs.h"

volatile TIM2_Type* timer2 = (volatile TIM2_Type*)TIM2_BASE;
volatile GPIOA_Type* gpioc = (volatile GPIOA_Type*)GPIOC_BASE;
volatile RCC_Type* rcc = (volatile RCC_Type*)RCC_BASE;



int main(void){

    // Enable GPIO Port C:
    rcc->APB2ENR |= (0x01 << 4);

    // Configure Pin PC13 as OUTPUT:
    gpioc->CRH |= (0x02 << 20);

    // Enable TIMER2:
    rcc->APB1ENR |= (0x01 << 0);

    // Configure TIMER2: 
    timer2->CR1 |= (0x01 << 0);
    timer2->PSC |= 0x1C20;         // 7200 Prescalar
    timer2->ARR |= 0x2710;         // 10000 count
    timer2->DIER |= (0x01 << 6);   // Enable Interrupt
    

    while(1) {
        for(int i = 0; i < 50000; i++) __asm__("nop");
        gpioc->ODR |= (0x01 << 13);
        for(int i = 0; i < 50000; i++) __asm__("nop");
        gpioc->ODR &= ~(0x01 << 13);
    }
}


