#include "ven_24c256.h"
#include "../make/stm32f103c8t6.h"
#include "ven_8digi.h"
#include "ven_i2c.h"
#include "ven_sys.h"

void
ven_24c256_test() {
    /* enable 72m rate for stm32 */
    ven_sys_72m();
    /* enable GPIOB in RCC->APB2ENR register */
    RCC->APB2ENR = 0x00000008;
    /* enable GPIOB pin 8 & 9 to floating mode */
    GPIOB->CRH &= 0xffffff00;
    GPIOB->CRH |= 0x00000077;
    /* enable GPIOB pin 12 13 14 to out mode */
    GPIOB->CRH &= 0xf000ffff;
    GPIOB->CRH |= 0x03330000;
    /* prepare the structure for 24c256 */
    ven_i2c.scl = (u32 *)(0x42000000 + 0x10c0c * 0x20 + 9 * 4);
    ven_i2c.sda = (u32 *)(0x42000000 + 0x10c0c * 0x20 + 8 * 4);
    ven_i2c.scl_read = (u32 *)(0x42000000 + 0x10c08 * 0x20 + 9 * 4);
    ven_i2c.sda_read = (u32 *)(0x42000000 + 0x10c08 * 0x20 + 8 * 4);
    /* write some to 24c256 */
    ven_24c256_writeByte(0x0005, 0x05);
	/* prepare the structure for 8digi tube */
    ven_74hc595.serialInput = (u32 *)(0x42000000 + 0x10c0c * 0x20 + 12 * 4);
    ven_74hc595.resetClock = (u32 *)(0x42000000 + 0x10c0c * 0x20 + 13 * 4);
    ven_74hc595.shiftClock = (u32 *)(0x42000000 + 0x10c0c * 0x20 + 14 * 4);
    /* read some from 24c256 */
	u8 data = ven_24c256_readByte(0x0007);
	/* display loop */
    while (1) {
        ven_8digi_show(1,16,data);
    }
}
