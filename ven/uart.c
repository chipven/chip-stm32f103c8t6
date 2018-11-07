#include "../make/stm32f103c8t6.h"
#include "string.h"
void open72m() {
        FLASH->ACR |= 0x1 << 5;  // PRFTBS=1:预取缓冲区启用标记位
        FLASH->ACR |= 0x1 << 4;  // PRFTBE=1:启用预取缓冲区
        FLASH->ACR |= 0x2 << 0;  // LATENCY=010:flash需要两个等待状态

        RCC->CFGR |= 0x7 << 18;  // PLLMUL=0111:PLL提供9倍输出
        RCC->CFGR |= 0x1 << 16;  // PLLSRC=1:HSE做为PLL输入时钟
        RCC->CFGR |= 0x4 << 8;   // PPRE1=100:HCLK提供2分频
        RCC->CFGR |= 0x2 << 0;   // SW=10:PLL输出做为系统时钟

        RCC->CR |= 0x1 << 24;  // PLLON=1:PLL使能
        RCC->CR |= 0x1 << 16;  // HSEON=1:外部高速时钟使能
}
void openUart1() {
        open72m();
        NVIC->ISER[1] |= 1 << 5;  //设置usart1的中断向量37-32=5 写在ISER[1]
        RCC->APB2ENR |= 1 << 14;        // USART1EN=1:开启usart1的时钟
        RCC->APB2ENR |= 1 << 2;         // IOPAEN=1:开启GPIOA的时钟
        RCC->APB2ENR |= 1 << 3;         // IOPBEN=1:开启GPIOB的时钟
        RCC->APB2ENR |= 1 << 4;         // IOPCEN=1:开启GPIOC的时钟
        GPIOA->CRH = 0x444444b4;        //设置PA9为复用推挽输出
        GPIOB->CRL = 0x44444444;        //设置PB6为复用推挽输出
        USART1->BRR = (468 << 4) | 12;  //设置BBR=468.75 16*0.75=12
        USART1->CR1 |= 1 << 13;         // UE=1:USART使能
        USART1->CR1 |= 1 << 8;  // PEIE=1:校验错误中断使能（还没判断）
        USART1->CR1 |= 1 << 5;  // RXNEIE=1:接收缓冲区非空中断使能
        USART1->CR1 |= 1 << 3;  // TE=1:发送使能
        USART1->CR1 |= 1 << 2;  // RE=1:接收使能
}
void uart1SendByte(unsigned char data) {
        USART1->DR = data;  //发送接收寄存器写入数据
        //判断usart状态寄存器TC发送完成标记，
        //如果为零一直等，如果为一放行
        while (0 == (USART1->SR & (1 << 6)))
                ;
}
void uart1SendString(char *cmd) {
        int len = strlen(cmd);
        for (int i = 0; i < len; i++) {
                uart1SendByte(cmd[i]);
        }
}

// void USART1_IRQHandler(void)
// {
//判断usart状态寄存器SR的RXNE读取寄存器非空
// if(USART1->SR&(1<<5))
// {
// rs232_send_byte(USART1->DR);
// }
// }