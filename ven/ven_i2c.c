#include "ven_i2c.h"

void
ven_i2c_start() {
	*ven_i2c.scl = 1;
    ven_delay();
    *ven_i2c.sda = 1;
    ven_delay();
    // 1 ven_delay();
    // if (!ven_iic_SDA_read) {
    // //SDA线为低电平则总线忙,退出
    //     return 0;
    // }
    *ven_i2c.sda = 0;
    ven_delay();
    // ven_delay();
    // if (ven_iic_SDA_read) {
    // //SDA线为高电平则总线出错,退出
    //     return 0;
    // }
    *ven_i2c.scl = 0;
    ven_delay();
}

void
ven_i2c_stop() {
    *ven_i2c.scl = 0;
    ven_delay();
    *ven_i2c.sda = 0;
    ven_delay();
    *ven_i2c.scl = 1;
    ven_delay();
    *ven_i2c.sda = 1;
    ven_delay();
}

void
_ven_i2c_putAck(u32 ack) {
    *ven_i2c.scl = 0;
    ven_delay();
    if (ack) {
        *ven_i2c.sda = 0;  // ack
    } else {
        *ven_i2c.sda = 1;  // no ack
    }
    ven_delay();
    *ven_i2c.scl = 1;
    ven_delay();
    *ven_i2c.scl = 0;
    ven_delay();
}

int
_ven_i2c_waitAck() {
    *ven_i2c.scl = 0;
    ven_delay();
    *ven_i2c.sda = 1;
    ven_delay();
    *ven_i2c.scl = 1;
    ven_delay();
    if (*ven_i2c.sda_read) {
        *ven_i2c.scl = 0;
        return 0;
    }
    *ven_i2c.scl = 0;
    return 1;
}

void
ven_i2c_sendByte(u32 dataTx) {
    u8 i = 8;
    while (i--) {
        /* low serial clock */
        *ven_i2c.scl = 0;
        ven_delay();
        /* send from high bit */
        if (dataTx & 0x80) {
            *ven_i2c.sda = 1;
        } else {
            *ven_i2c.sda = 0;
        }
        /* move left one bit */
        dataTx <<= 1;
        ven_delay();
        /* high serial clock */
        *ven_i2c.scl = 1;
        ven_delay();
    }
    /* low serial clock */
    *ven_i2c.scl = 0;
    ven_delay();
    /* wait ackownledge */
    _ven_i2c_waitAck();
}

u32
ven_i2c_receiveByte() {
    /* from higher bit to lower */
    u8 i = 8;
    u32 receiveByte = 0;

    *ven_i2c.scl = 0;
    ven_delay();

    *ven_i2c.sda = 1;

    while (i--) {  //
        *ven_i2c.scl = 1;
        ven_delay();
        receiveByte <<= 1;
        if (*ven_i2c.sda_read) {
            receiveByte |= 0x01;
        }
        *ven_i2c.scl = 0;
        ven_delay();
    }  //
    // ven_i2c_SCL = 0;  //   SCL_L;
    _ven_i2c_putAck(1);
    return receiveByte;
}
