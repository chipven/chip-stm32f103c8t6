#include "ven_tim.h"

#ifndef TIM2_IRQHandler_Rewrite
void
TIM2_IRQHandler() {
    ven_tim.idle = 0;
    TIM2->SR &= 0xfffffffe;
}
#endif  // TIM2_IRQHandler_Rewrite

#ifndef TIM3_IRQHandler_Rewrite
void
TIM3_IRQHandler() {
    ven_tim.idle = 0;
    TIM2->SR &= 0xfffffffe;
}
#endif  // TIM2_IRQHandler_Rewrite

#ifndef TIM4_IRQHandler_Rewrite
void
TIM4_IRQHandler() {
    ven_tim.idle = 0;
    TIM2->SR &= 0xfffffffe;
}
#endif  // TIM2_IRQHandler_Rewrite

#ifndef TIM5_IRQHandler_Rewrite
void
TIM5_IRQHandler() {
    ven_tim.idle = 0;
    TIM2->SR &= 0xfffffffe;
}
#endif  // TIM2_IRQHandler_Rewrite

void
ven_delay_tim2(u32 ms_5000_max) {
    // initial timer
    ven_tim.idle = 1;

    // initial interrupt setting
    NVIC->ISER[0] |= 0x10000000;
    RCC->APB1ENR |= 0x00000001;
    TIM2->PSC = RCC->CFGR == 0x001d040a ? 7199 : 7;  // 799 7199
    TIM2->ARR = 1 * ms_5000_max;
    TIM2->DIER = 0x1;
    TIM2->CR1 = 0x9;

    // in idle
    while (ven_tim.idle) {
        // ven_8digi_DEC(ven_tim.acc);
    }
}

void
ven_TIM2_init() {
    NVIC->ISER[0] = 0x10000000;
    RCC->APB1ENR |= 0x00000001;
    TIM2->PSC = RCC->CFGR == 0x001d040a ? 7199 : 799;  // 799 7199
    TIM2->ARR = 10000;
    TIM2->CR1 = 0x1;
    TIM2->DIER = 0x1;
}
