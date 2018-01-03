#include "system_util.h"

/* To set chip frequency clock to 72m */
void
on_72m() {
    FLASH->ACR |= 0x32;
    RCC->CFGR |= 0x001d0402;
    RCC->CR |= 0x01010000;
}

/* To set GPIOA and GPIOB all output 50M */
void
on_gpio_pp() {
    RCC->APB2ENR |= 0x0000000c;
    GPIOA->CRL = 0x33333333;
    GPIOA->CRH = 0x33333333;
    GPIOB->CRL = 0x33333333;
    GPIOB->CRH = 0x33333333;
}

void
delay_us(unsigned int time) {
    unsigned int i = 0;
    while (time--) {
        i = 12;  // calibration
        while (i--)
            ;
    }
}

void
delay_ms(unsigned int time) {
    unsigned int i = 0;
    while (time--) {
        i = 12000;  // calibration
        while (i--)
            ;
    }
}

void
delay() {
    if (RCC->CFGR) {
        delay_us(480 + 80);
    } else {
        delay_us(60 + 10);
    }
}
