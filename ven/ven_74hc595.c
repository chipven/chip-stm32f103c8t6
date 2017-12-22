#include "ven_74hc595.h"

void
ven_74hc595_sendBytes(u32 howManyBytes,u32 data) {
    for (u32 i = 0; i < howManyBytes * 8; i++) {
        *ven_74hc595.serialInput = data >> (howManyBytes * 8 - 1);
        data <<= 1;
        *ven_74hc595.shiftClock = 0;
        *ven_74hc595.shiftClock = 1;
    }
    *ven_74hc595.resetClock = 1;
    *ven_74hc595.resetClock = 0;
}
