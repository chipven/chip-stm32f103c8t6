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

void
device_sd_cs1_74(Device_sd sd) {
    *sd.cs = 1;
    for (u8 i = 0; i < 80; i++) {
        *sd.sclk = 0;
        *sd.sclk = 1;
    }
}

void
device_sd_cs1_8(Device_sd sd) {
    *sd.cs = 1;
    for (u8 i = 0; i < 8; i++) {
        *sd.sclk = 0;
        *sd.sclk = 1;
    }
}

void
device_sd_cs0_100(Device_sd sd) {
    *sd.cs = 0;
    for (u8 i = 0; i < 8; i++) {
        *sd.sclk = 0;
        *sd.sclk = 1;
    }
}

u32
device_sd_CMD0_reset(Device_sd sd) {
    u32 read;
    for (u32 i = 0; i < 0xf; i++) {
        device_sd_cs1_74(sd);
        device_sd_write(sd, 0x40);
        device_sd_write(sd, 0x00);
        device_sd_write(sd, 0x00);
        device_sd_write(sd, 0x00);
        device_sd_write(sd, 0x00);
        device_sd_write(sd, 0x95);
        for (u32 i = 0; i < 0xf; i++) {
            read = device_sd_write(sd, 0xff);
            if (read == 1) break;
        }
    }
    device_sd_cs1_8(sd);
    return read;
}

u32
device_sd_CMD1_init(Device_sd sd) {
    u32 read;
    device_sd_write(sd, 0x41);
    device_sd_write(sd, 0x00);
    device_sd_write(sd, 0x00);
    device_sd_write(sd, 0x00);
    device_sd_write(sd, 0x00);
    device_sd_write(sd, 0xff);
    for (u32 i = 0; i < 0xf; i++) {
        read = device_sd_write(sd, 0xff);
        if (read != 0xFF) break;
    }
    device_sd_cs1_8(sd);
    return read;
}

u32
device_sd_CMD24_write(Device_sd sd, u32 address, u8** data512) {
    u32 read;
    device_sd_write(sd, 0x58);
    device_sd_write(sd, (u8)(address >> 24));
    device_sd_write(sd, (u8)(address >> 16));
    device_sd_write(sd, (u8)(address >> 8));
    device_sd_write(sd, (u8)(address));
    device_sd_write(sd, 0xff);
    for (u32 i = 0; i < 0xf; i++) {
        read = device_sd_write(sd, 0xff);
        if (read != 0xFF) break;
    }
    if (read != 0) return 0xee;
    device_sd_cs0_100(sd);
    device_sd_write(sd, 0xfe);
    // @Todo: data contents
    // @Todo: CRC
    return 0;
}

u32
device_sd_learn(Device_sd sd) {
    u32 cmd0 = device_sd_CMD0_reset(sd);
    u8* data[1024];
    device_sd_CMD24_write(sd, 0x56743, data);
    return cmd0;
}
