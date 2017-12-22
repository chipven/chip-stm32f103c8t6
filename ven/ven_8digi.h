#ifndef VEN_8DIGI_H
#define VEN_8DIGI_H

#include "../make/stm32f103c8t6.h"
#include "ven_74hc595.h"

/*
 * struct ven_8digi {
 *     u32 *serialInput;  // serial input GPIO pointer
 *     u32 *resetClock;   // reset clock GPIO pointer
 *     u32 *shiftClock;   // shift clock GPIO pointer
 *     u32 typeDigital;   // choose 1/2[> common cathode or anode
 *     u32 numberSystem;  // numeric for binary decimal hexadecimal
 *     u32 showNumber;    // what's the number want to show on screen
 * };
 */

extern struct ven_74hc595 ven_74hc595;

void
ven_8digi_show(u32 typeDigital,u32 numberSystem,u32 showNumber);

#endif /* VEN_8DIGI_H */
