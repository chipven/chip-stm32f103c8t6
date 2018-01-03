#include "device_sd_test.h"
#include "device_8digi.h"

void
device_sd_test() {
    RCC->APB2ENR |= 0x00000008;
    GPIOB->CRH = 0x03330083;
    GPIOB->CRL = 0x33000000;

    Device_sd sd;
    sd.cs = B6_out;
    sd.sclk = B7_out;
    sd.mosi = B8_out;
    sd.miso = B9_in;

    u8 response = device_sd_learn(sd);

    Device_8digi d8;
    d8.chip_74hc595.serialInput = B12_out;
    d8.chip_74hc595.resetClock = B13_out;
    d8.chip_74hc595.shiftClock = B14_out;
    d8.number_system = 16;
    d8.type_digital = 1;

    while (1) {
        device_8digi_show(d8, response);
    }
}
