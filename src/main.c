#include "main.h"
#include "../ven/device_8digi_test.h"
#include "../ven/system_util.h"
int numberToShow = 0;
unsigned char uartBuffer;
unsigned int *rx = B1_in;
unsigned int *tx = B0_out;
int uartCount = 0;
int main()
{
    on_72m();

    Device_8digi d8;
    d8.number_system = 16;
    d8.type_digital = 1;
    // RCC->APB2ENR |= 0x00000008;
    RCC->APB2ENR |= 0x1 << 3;
    GPIOB->CRH &= 0xf000ffff;
    GPIOB->CRH |= 0x03330000;
    d8.chip_74hc595.serialInput = B12_out;
    d8.chip_74hc595.resetClock = B13_out;
    d8.chip_74hc595.shiftClock = B14_out;

    numberToShow = 0x0;

    // 外部中断启用
    NVIC->ISER[0] |= 0x1 << 7;
    RCC->APB2ENR |= 0x1 << 3;
    RCC->APB2ENR |= 0x1 << 0;
    GPIOB->CRH &= 0xffffff00;
    GPIOB->CRH |= 0x00000083;
    AFIO->EXTICR[0] |= 0x1 << 4;
    EXTI->IMR |= 0x1 << 1;
    EXTI->FTSR |= 0x1 << 1;

    //配置一个定时器
    NVIC->ISER[0] |= 0x10000000;
    RCC->APB1ENR |= 0x00000001;
    TIM2->PSC = 71; //RCC->CFGR == 0x001d040a ? 7199 : 799; // 799 7199
    TIM2->ARR = 103;
    TIM2->DIER = 0x1;
    TIM2->CR1 |= 0x1 << 7;
    TIM2->CR1 |= 0x1 << 4;

    while (1)
    {
        // device_8digi_show(d8, ven_tim.acc);
        device_8digi_show(d8, numberToShow);
    }
}

void TIM2_IRQHandler()
{
    uartCount++;
    if (uartCount == 1)
    {
        while (*rx != 0)
            ;
    }
    if (2 <= uartCount && uartCount <= 9)
    {
        uartBuffer >>= 1;
        uartBuffer |= *rx << 7;
    }
    if (uartCount == 10)
    {
        while (*rx != 1)
            ;
        numberToShow = numberToShow << 8 | uartBuffer;
        EXTI->IMR |= 0x1 << 1;
        TIM2->CR1 &= 0x0 << 0;
    }
    //
    TIM2->SR &= ~(1 << 0);
}

void EXTI1_IRQHandler()
{
    uartCount = 0;
    EXTI->IMR &= 0x0 << 1;
    TIM2->CR1 |= 0x1 << 0;
    EXTI->PR |= 0x1 << 1;
}
