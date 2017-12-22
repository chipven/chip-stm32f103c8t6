#include "./ven_24c256.h"

void
ven_24c256_writeByte(u8 address, u8 data) {
    /* 1 start */
    ven_i2c_start();
    /* 2 device addressess | write (write 0 / read 1) */
    ven_i2c_sendByte(0xa0);
    /* 3 data addressess 16 bit  */
    ven_i2c_sendByte((u8)(address >> 8));
    ven_i2c_sendByte((u8)(address & 0x00ff));
    /* 4 write 1 Byte */
    ven_i2c_sendByte(data);
    /* 5 stop */
    ven_i2c_stop();
}

u8
ven_24c256_readByte(u8 address) {
    /* 1 start */
    ven_i2c_start();
    /* 2 device addressess | write (write 0 / read 1) */
    ven_i2c_sendByte(0xa0);
    /* 3 data addressess 16 bit */
    ven_i2c_sendByte((u8)(address >> 8));
    ven_i2c_sendByte((u8)(address & 0x00ff));
    /* 4 restart */
    ven_i2c_start();
    /* 5 device addressess | read (write 0 / read 1) */
    ven_i2c_sendByte(0xa1);
    /* 6 read one byte */
	u8 data;
    data = ven_i2c_receiveByte();
    /* 7 stop */
    ven_i2c_stop();
	/* return */
	return data;
}
