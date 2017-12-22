#include "ven_delay.h"

void
ven_delay() {
    if (RCC->CFGR) {
        ven_delay_us(480 + 80);
    } else {
        ven_delay_us(60 + 10);
    }
}

void
ven_delay_us(u32 time) {
    u32 i = 0;
    while (time--) {
        i = 12;  // calibration
        while (i--)
            ;
    }
}

void
ven_delay_ms(u32 time) {
    u32 i = 0;
    while (time--) {
        i = 12000;  // calibration
        while (i--)
            ;
    }
}
