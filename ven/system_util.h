#ifndef VE_SYS_H
#define VE_SYS_H
#include "../make/stm32f103c8t6.h"
void on_72m();
void on_gpio_pp();
void delay();
void openUart1();
void uart1SendByte(unsigned char data);
void uart1SendString(char *cmd);
void open72m();
#endif /* VE_SYS_H */
