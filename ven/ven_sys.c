#include "./ven_sys.h"

/* To set chip frequency clock to 72m */
void
ven_sys_72m() {
    FLASH->ACR |= 0x32;
    RCC->CFGR |= 0x001d0402;
    RCC->CR |= 0x01010000;
}

/* To set GPIOA and GPIOB all output 50M */
void
ven_sys_gpio_pp() {
    RCC->APB2ENR |= 0x0000000c;
    GPIOA->CRL = 0x33333333;
    GPIOA->CRH = 0x33333333;
    GPIOB->CRL = 0x33333333;
    GPIOB->CRH = 0x33333333;
}
