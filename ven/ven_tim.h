#ifndef VE_TIM_H
#define VE_TIM_H

#include "../make/stm32f103c8t6.h"
#include "ven_8digi.h"

typedef struct {
    u32 acc;
    u32 idle;
} ven_tim_def;

ven_tim_def ven_tim;

void
TIM2_IRQHandler();
void
TIM3_IRQHandler();
void
TIM4_IRQHandler();
void
TIM5_IRQHandler();
void
ven_delay(u32 ms_5000_max);
void
ven_TIM2_init();

#endif /* VE_TIM_H */
