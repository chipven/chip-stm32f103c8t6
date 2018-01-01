#include "protocol_i2c.h"
void
protocol_i2c_start(struct protocol_i2c in) {
    *in.scl = 1;
    system_util_delay();
    *in.sda = 1;
    system_util_delay();
    // 1 system_util_delay();
    // if (!ven_iic_SDA_read) {
    // //SDA线为低电平则总线忙,退出
    //     return 0;
    // }
    *in.sda = 0;
    system_util_delay();
    // system_util_delay();
    // if (ven_iic_SDA_read) {
    // //SDA线为高电平则总线出错,退出
    //     return 0;
    // }
    *in.scl = 0;
    system_util_delay();
}
void
protocol_i2c_stop(struct protocol_i2c in) {
    *in.scl = 0;
    system_util_delay();
    *in.sda = 0;
    system_util_delay();
    *in.scl = 1;
    system_util_delay();
    *in.sda = 1;
    system_util_delay();
}
void
__protocol_i2c_putAck(struct protocol_i2c in, unsigned int ack) {
    *in.scl = 0;
    system_util_delay();
    if (ack) {
        *in.sda = 0;  // ack
    } else {
        *in.sda = 1;  // no ack
    }
    system_util_delay();
    *in.scl = 1;
    system_util_delay();
    *in.scl = 0;
    system_util_delay();
}
int
__protocol_i2c_waitAck(struct protocol_i2c in) {
    *in.scl = 0;
    system_util_delay();
    *in.sda = 1;
    system_util_delay();
    *in.scl = 1;
    system_util_delay();
    if (*in.sda_read) {
        *in.scl = 0;
        return 0;
    }
    *in.scl = 0;
    return 1;
}
void
protocol_i2c_sendByte(struct protocol_i2c in, unsigned int dataTx) {
    unsigned char i = 8;
    while (i--) {
        /* low serial clock */
        *in.scl = 0;
        system_util_delay();
        /* send from high bit */
        if (dataTx & 0x80) {
            *in.sda = 1;
        } else {
            *in.sda = 0;
        }
        /* move left one bit */
        dataTx <<= 1;
        system_util_delay();
        /* high serial clock */
        *in.scl = 1;
        system_util_delay();
    }
    /* low serial clock */
    *in.scl = 0;
    system_util_delay();
    /* wait ackownledge */
    __protocol_i2c_waitAck(in);
}
unsigned int
protocol_i2c_receiveByte(struct protocol_i2c in) {
    /* from higher bit to lower */
    unsigned char i = 8;
    unsigned int receiveByte = 0;
    *in.scl = 0;
    system_util_delay();
    *in.sda = 1;
    while (i--) {  //
        *in.scl = 1;
        system_util_delay();
        receiveByte <<= 1;
        if (*in.sda_read) {
            receiveByte |= 0x01;
        }
        *in.scl = 0;
        system_util_delay();
    }  //
    // in_SCL = 0;  //   SCL_L;
    __protocol_i2c_putAck(in, 1);
    return receiveByte;
}
