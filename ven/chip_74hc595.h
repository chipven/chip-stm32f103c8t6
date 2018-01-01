#ifndef VEN_74HC595_H
#define VEN_74HC595_H
#include "../make/stm32f103c8t6.h"
struct chip_74hc595 {
    unsigned int *serialInput;  // serial input GPIO pointer
    unsigned int *resetClock;   // reset clock GPIO pointer
    unsigned int *shiftClock;   // shift clock GPIO pointer
};
void
chip_74hc595_sendBytes(struct chip_74hc595 in, unsigned int how_many_bytes,
                       unsigned int data);
#endif /* VEN_74HC595_H */
