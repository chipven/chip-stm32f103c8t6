#include "../make/stm32f103c8t6.h"
#include "system_util.h"
#include "uart.h"
int main() {
        openUart1();
        while (1)
                ;
}

void USART1_IRQHandler() {
        char *cmd = "Hello World !!!\n";
        if (USART1->SR & (1 << 5)) {
                if (USART1->DR == 0x61) uart1SendString(cmd);
                if (USART1->DR == 0x62) uart1SendString("This is b");
        }
}