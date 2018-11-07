#include "../make/stm32f103c8t6.h"
#include "stdlib.h"
#include "string.h"
#include "system_util.h"
#include "uart.h"
unsigned int *b3 = (unsigned int *)(0x42000000 + 0x10c0c * 32 + 12 * 4);
unsigned int *b4 = (unsigned int *)(0x42000000 + 0x10c0c * 32 + 13 * 4);
unsigned int *b5 = (unsigned int *)(0x42000000 + 0x10c0c * 32 + 14 * 4);
unsigned int *b6 = (unsigned int *)(0x42000000 + 0x10c0c * 32 + 15 * 4);
unsigned int *c13 = (unsigned int *)(0x42000000 + 0x1100c * 32 + 13 * 4);
int motorCount = 0;
int speed = 0;
int main() {
        openUart1();
        uart1SendByte(0x00);
        RCC->APB2ENR |= 1 << 3;  // gpioben=1
        RCC->APB2ENR |= 1 << 4;  // gpiocen=1
        GPIOB->CRL = 0x43333444;
        GPIOB->CRH = 0x33334444;
        GPIOC->CRH = 0x44344444;
        *c13 = 0;

        //配置一个RX定时器
        NVIC->ISER[0] |= 0x1 << 28;  //开启TIM2中断向量28
        RCC->APB1ENR |= 0x1 << 0;    // TIM2EN=1,开启APB1的TIM2EN
        TIM2->PSC = 35;              //预分频
        TIM2->ARR = 1;               //预装载
        TIM2->DIER |= 0x1 << 0;  // DMA中断使能寄存器UIE允许更新中断
        TIM2->CR1 |= 0x1 << 7;   // ARPE=1允许自动装载
        TIM2->CR1 |= 0x1 << 4;   //向下计数
        TIM2->CR1 |= 0x1 << 0;

        // //配置一个TX定时器
        // NVIC->ISER[0] |= 0x1 << 29;  //开启TIM3中断向量29
        // RCC->APB1ENR |= 0x1 << 1;    // TIM3EN=1
        // TIM3->PSC = 7199;            //预分频
        // TIM3->ARR = 999;             //预装载
        // TIM3->DIER |= 0x1 << 0;      // UIE=1 允许更新中断
        // TIM3->CR1 |= 0x1 << 7;       //允许ARR载入
        // TIM3->CR1 |= 0x1 << 4;       //向下计数

        while (1) {
        }
}
char buffer[50];
unsigned int bufferCount;
void USART1_IRQHandler() {
        if (USART1->SR & (1 << 5)) {
                buffer[bufferCount] = USART1->DR;
                bufferCount++;
                if (USART1->DR == 0x29 || USART1->DR == 0x0a ||
                    USART1->DR == 0x13) {
                        // 0x29=")"
                        char *cmd;
                        char *arg;

                        if (strcmp(buffer, ")") != 0) {
                                cmd = strtok(buffer, "()");
                                arg = strtok(NULL, "()");
                                uart1SendString("set ");
                                uart1SendString(cmd);
                                uart1SendString(" is ");
                                uart1SendString(arg);

                                if (strcmp(cmd, "speed") == 0) {
                                        int num = atoi(arg);
                                        speed = num;
                                }
                        }

                        bufferCount = 0;
                        memset(buffer, 0, 50);
                        memset(cmd, 0, 20);
                        memset(arg, 0, 20);
                }
        }
        // int set = USART1->DR;
}

void TIM2_IRQHandler() {
        motorCount++;
        if (motorCount >= 1 && motorCount <= speed) {
                *b3 = 1;
        }
        if (motorCount > speed && motorCount < 0xff) {
                *b3 = 0;
        }
        if (motorCount == 0xff) {
                motorCount = 0;
        }
        TIM2->SR &= ~(0x1 << 0);
}
// void TIM3_IRQHandler() {
//         *b3 = 0;
//         TIM3->SR &= ~(0x1 << 0);
// }
