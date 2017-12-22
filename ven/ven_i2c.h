#ifndef VE_I2C_H
#define VE_I2C_H

#include "../make/stm32f103c8t6.h"
#include "./ven_delay.h"

/*
 * Example:
 *
 * ven_i2c.scl = (u32 *)(0x42000000 + 0x10c0c * 0x20 + 9 * 4);
 * ven_i2c.sda = (u32 *)(0x42000000 + 0x10c0c * 0x20 + 8 * 4);
 * ven_i2c.scl_read = (u32 *)(0x42000000 + 0x10c08 * 0x20 + 9 * 4);
 * ven_i2c.sda_read = (u32 *)(0x42000000 + 0x10c08 * 0x20 + 8 * 4);
 *
 */

struct ven_i2c {
    u32 *scl;      /* GPIO pointer clock */
    u32 *sda;      /* GPIO pointer data */
    u32 *scl_read; /* GPIO pointer read clock */
    u32 *sda_read; /* GPIO pointer data clock */
} ven_i2c;

void
ven_i2c_start();
void
ven_i2c_stop();
void
ven_i2c_sendByte(u32 dataTx);
u32
ven_i2c_receiveByte();

#endif /* VE_I2C_H */
