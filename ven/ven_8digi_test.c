#include "ven_8digi.h"

void
ven_8digi_show_test() {
    RCC->APB2ENR |= 0x00000008;
    GPIOB->CRH &= 0xe000eeee;
    GPIOB->CRH |= 0x03330000;

    ven_74hc595.serialInput = (u32 *)(0x42000000 + 0x10c0c * 0x20 + 12 * 4);
	ven_74hc595.resetClock = (u32 *)(0x42000000 + 0x10c0c * 0x20 + 13 * 4);
    ven_74hc595.shiftClock = (u32 *)(0x42000000 + 0x10c0c * 0x20 + 14 * 4);
	

    while (1) {
        ven_8digi_show(1,16,0xabcdef17);
    }
}
