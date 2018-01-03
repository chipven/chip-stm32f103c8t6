#include "device_sd.h"

u8
device_sd_write(Device_sd sd, u8 Tx) {
    u8 Rx = 0;
    *sd.cs = 0;
    *sd.sclk = 1;
    for (u8 i = 0; i < 8; i++) {
        if (Tx & 0x80) {
            *sd.mosi = 1;
        } else {
            *sd.mosi = 0;
        }
        Tx <<= 1;
        *sd.sclk = 0;
        Rx <<= 1;
        if (*sd.miso == 1) {
            Rx |= 0x01;
        }
        *sd.sclk = 1;
    }
    return Rx;
}

u8
device_sd_74_clock(Device_sd sd) {
    *sd.cs = 1;
    for (u8 i = 0; i < 0xff; i++) {
        *sd.sclk = 0;
        *sd.sclk = 1;
    }
    return 0;
}

u8
device_sd_CMD0(Device_sd sd) {
    u32 read;
    device_sd_write(sd, 0x40);
    device_sd_write(sd, 0x00);
    device_sd_write(sd, 0x00);
    device_sd_write(sd, 0x00);
    device_sd_write(sd, 0x00);
    device_sd_write(sd, 0x95);
    for (u8 i = 0; i < 0xf; i++) {
        read = device_sd_write(sd,0xff);
        if (read == 1) return read;
    }
    return read;
}

u8
device_sd_init(Device_sd sd) {
    device_sd_74_clock(sd);
    return device_sd_CMD0(sd);
}

u8
device_sd_learn(Device_sd sd){
	return device_sd_init(sd);
}
