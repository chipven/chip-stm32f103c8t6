#include "ven_8digi.h"

void
ven_8digi_show(u32 typeDigital, u32 numberSystem,
              u32 numberToShow) {
    u32 _ven_8digi_tab[2][16] = {
        {0xfc, 0x60, 0xda, 0xf2, 0x66, 0xb6, 0xbe, 0xe0, 0xfe, 0xf6, 0xee, 0x3e,
         0x1a, 0x7a, 0x9e, 0x8e},
        {0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, 0x80, 0x90, 0x88, 0x83,
         0xa7, 0xa1, 0x86, 0x8e}};

    u32 po[] = {
        1,
        numberSystem,
        numberSystem * numberSystem,
        numberSystem * numberSystem * numberSystem,
        numberSystem * numberSystem * numberSystem * numberSystem,
        numberSystem * numberSystem * numberSystem * numberSystem *
            numberSystem,
        numberSystem * numberSystem * numberSystem * numberSystem *
            numberSystem * numberSystem,
        numberSystem * numberSystem * numberSystem * numberSystem *
            numberSystem * numberSystem * numberSystem,
    };

    u32 data;
    for (u32 i = 0; i < 8; i++) {
        if (typeDigital == 0) {
            data =
                (7 - i) << 8 |
                _ven_8digi_tab[typeDigital][(numberToShow / po[i] % numberSystem)];
        }
        if (typeDigital == 1) {
            data =
                (1 << i) |
                _ven_8digi_tab[typeDigital][(numberToShow / po[i] % numberSystem)]
                    << 8;
        }
        ven_74hc595_sendBytes(2,data);
    }
}
