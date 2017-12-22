#ifndef VE_24C256_H
#define VE_24C256_H

#include "../make/stm32f103c8t6.h"
#include "./ven_i2c.h"

extern struct ven_i2c ven_i2c;
void
ven_24c256_writeByte(u8 addr, u8 data);
u8
ven_24c256_readByte(u8 addr);

#endif /* VE_24C256_H */
