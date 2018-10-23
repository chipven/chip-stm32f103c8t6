#include "../make/stm32f103c8t6.h"
#include "system_util.h"
#include "uart.h"
// int index;
// char buffer[50];
int main() {
        openUart1();
        uart1SendByte(0x00);
        while (1)
                ;
}

void USART1_IRQHandler() {
        if (USART1->DR == 0x61) uart1SendString("This is a");
        if (USART1->DR == 0x62) uart1SendString("This is b");
        if (USART1->DR == 0x63) uart1SendString("This is c");
}