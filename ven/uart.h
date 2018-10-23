#ifndef __uart_h__
#define __uart_h__
void openUart1();
void uart1SendByte(unsigned char data);
void uart1SendString(char *cmd);
void open72m();
#endif