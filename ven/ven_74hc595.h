#ifndef VEN_74HC595_H
#define VEN_74HC595_H

#include "../make/stm32f103c8t6.h"

struct ven_74hc595 {
    u32 *serialInput;  // serial input GPIO pointer
    u32 *resetClock;   // reset clock GPIO pointer
    u32 *shiftClock;   // shift clock GPIO pointer
} ven_74hc595;

void
ven_74hc595_sendBytes(u32 howManyBytes,u32 data);

#endif /* VEN_74HC595_H */
