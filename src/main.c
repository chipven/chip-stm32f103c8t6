#include "main.h"
#include "../ven/device_8digi_test.h"
#include "../ven/system_util.h"
int idle = 1;
int acc = 0;
int main()
{
    on_72m();
    // initial timer
    // initial interrupt setting
    NVIC->ISER[0] |= 0x10000000;
    RCC->APB1ENR |= 0x00000001;
    TIM2->PSC = 7199; //RCC->CFGR == 0x001d040a ? 7199 : 799; // 799 7199
    TIM2->ARR = 9;
    TIM2->DIER = 0x1;
    TIM2->CR1 |= 0x91;
    // in idle

    Device_8digi d8;
    d8.number_system = 10;
    d8.type_digital = 1;
    RCC->APB2ENR |= 0x00000008;
    GPIOB->CRH &= 0xe000eeee;
    GPIOB->CRH |= 0x03330000;
    d8.chip_74hc595.serialInput = B12_out;
    d8.chip_74hc595.resetClock = B13_out;
    d8.chip_74hc595.shiftClock = B14_out;
    while (idle)
    {
        // device_8digi_show(d8, ven_tim.acc);
        device_8digi_show(d8, acc);
    }
}

#ifndef TIM2_IRQHandler_Rewrite
#define TIM2_IRQHandler_Rewrite
void TIM2_IRQHandler()
{
    TIM2->SR &= ~(1 << 0);
    acc++;
}
#endif