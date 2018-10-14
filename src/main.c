#include "main.h"
#include "../ven/device_8digi_test.h"
#include "../ven/system_util.h"
int numberToShow = 0;
unsigned char uartBuffer;
unsigned char txBuffer;
unsigned int *rx = B1_in;
unsigned int *tx = B0_out;

int main()
{
    on_72m();

    Device_8digi d8;
    d8.number_system = 16;
    d8.type_digital = 1;
    // RCC->APB2ENR |= 0x00000008;
    RCC->APB2ENR |= 0x1 << 3;
    GPIOB->CRH &= 0xf000ffff;
    GPIOB->CRH |= 0x03330000;
    d8.chip_74hc595.serialInput = B12_out;
    d8.chip_74hc595.resetClock = B13_out;
    d8.chip_74hc595.shiftClock = B14_out;

    numberToShow = 0x0;

    // 外部中断启用
    NVIC->ISER[0] |= 0x1 << 7;
    RCC->APB2ENR |= 0x1 << 3;
    RCC->APB2ENR |= 0x1 << 0;
    GPIOB->CRH &= 0xffffff00;
    GPIOB->CRH |= 0x00000083;
    AFIO->EXTICR[0] |= 0x1 << 4;
    EXTI->IMR |= 0x1 << 1;
    EXTI->FTSR |= 0x1 << 1;

    //配置一个RX定时器
    NVIC->ISER[0] |= 0x10000000;
    RCC->APB1ENR |= 0x00000001;
    TIM2->PSC = 71; //RCC->CFGR == 0x001d040a ? 7199 : 799; // 799 7199
    TIM2->ARR = 103;
    TIM2->DIER = 0x1;
    TIM2->CR1 |= 0x1 << 7;
    TIM2->CR1 |= 0x1 << 4;
    //配置一个TX定时器
    NVIC->ISER[0] |= 0x1 << 29; //TIM3中断向量29
    RCC->APB1ENR |= 0x1 << 1;   //TIM3EN=1
    TIM3->PSC = 71;
    TIM3->ARR = 103;
    TIM3->DIER |= 0x1 << 0; //UIE=1 允许更新中断
    TIM3->CR1 |= 0x1 << 7;  //允许ARR载入
    TIM3->CR1 |= 0x1 << 4;  //向下计数

    *tx = 1;
    while (1)
    {

        // device_8digi_show(d8, ven_tim.acc);
        device_8digi_show(d8, numberToShow);
        if (numberToShow == 0x20118866)
        {
            *tx = 0;
            // send(0x6c);
        }
    }
}

//uart rx 专用计数器
int uartCount = 0;
void TIM2_IRQHandler()
{
    //计数器加1
    uartCount++;
    //如果计数器为1 等待*rx的下拉信号
    if (uartCount == 1)
    {
        while (*rx != 0)
            ;
    }
    //如果是第2至9位,收取8位rx的信号;
    if (2 <= uartCount && uartCount <= 9)
    {
        uartBuffer >>= 1;
        uartBuffer |= *rx << 7;
    }
    //如果是第十位,等待停止位;
    if (uartCount == 10)
    {
        while (*rx != 1)
            ;
        //要显示的内容向左移8位;
        numberToShow <<= 8;
        //把buffer的内容或进最后8位;
        numberToShow |= uartBuffer;
        //关闭TIM2使能
        TIM2->CR1 &= 0x0 << 0;
        //开启EXTI1使能
        EXTI->IMR |= 0x1 << 1;
    }
    //改写TIM2更新标志
    TIM2->SR &= ~(1 << 0);
}

void EXTI1_IRQHandler()
{
    //打开TIM2定时器
    TIM2->CR1 |= 0x1 << 0;
    //uart计数器归零
    uartCount = 0;
    //关闭EXTI1使能
    EXTI->IMR &= 0x0 << 1;
    //取反外部中断挂起位
    EXTI->PR |= 0x1 << 1;
}

int txCount = 0;
void send(int data)
{
    txCount = 0;
    txBuffer = data;
    TIM3->CR1 |= 0x1 << 0; //开始计数
}

void TIM3_IRQHandler()
{
    txCount++;
    if (txCount == 0)
    {
        *tx = 0;
    }
    if (txCount >= 1 && txCount <= 9)
    {
        *tx = (txBuffer << 7) >> 7;
        txBuffer >>= 1;
    }
    if (txCount == 10)
    {
        *tx = 1;
        TIM3->CR1 &= ~(0x1 << 0);
        txCount = 0;
    }
    TIM3->SR &= ~(0x1 << 0);
}