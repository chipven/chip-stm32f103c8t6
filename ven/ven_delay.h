#ifndef VE_DELAY_H
#define VE_DELAY_H

#include "../make/stm32f103c8t6.h"

void
ven_delay();
void
ven_delay_us(u32 time);
void
ven_delay_ms(u32 time);

#endif /* VE_DELAY_H */
