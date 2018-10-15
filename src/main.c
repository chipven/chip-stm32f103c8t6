#include "main.h"
#include "../ven/device_8digi_test.h"
#include "../ven/system_util.h"
int numberToShow = 0;
unsigned char RxBuffer;
unsigned int TxBuffer;
unsigned int *rx = B1_in;
unsigned int *tx = B0_out;
int txCount = 0;

void send(int data)
{
    txCount = 0;
    TxBuffer = data;
    TIM3->CR1 |= 0x1 << 0; //开始计数
}

int main()
{
    on_72m();

    //配置led管的针脚
    Device_8digi d8;
    d8.number_system = 16;
    d8.type_digital = 1;
    //IOPBEN=1
    RCC->APB2ENR |= 0x1 << 3;
    //GPIOB 12 13 14 为推挽输出
    GPIOB->CRH &= 0xf000ffff;
    GPIOB->CRH |= 0x03330000;
    //给数码管设置引脚
    d8.chip_74hc595.serialInput = B12_out;
    d8.chip_74hc595.resetClock = B13_out;
    d8.chip_74hc595.shiftClock = B14_out;

    //要显示的数据
    numberToShow = 0xff;

    // 外部中断启用
    NVIC->ISER[0] |= 0x1 << 7;   //开启EXTI1,中断向量为7
    RCC->APB2ENR |= 0x1 << 3;    //IOPBEN=1
    RCC->APB2ENR |= 0x1 << 0;    //AFIOEN=1
    GPIOB->CRL &= 0xffffff00;    //与零
    GPIOB->CRL |= 0x00000083;    //b0 推挽输出, b1 推挽输入
    AFIO->EXTICR[0] |= 0x1 << 4; //EXTI1的关联GPIO为GPIOB
    EXTI->IMR |= 0x1 << 1;       //EXTI1的中断屏蔽打开
    EXTI->FTSR |= 0x1 << 1;      //EXTI1开启下降沿检测

    //配置一个RX定时器
    NVIC->ISER[0] |= 0x1 << 28; //开启TIM2中断向量28
    RCC->APB1ENR |= 0x1 << 0;   //TIM2EN=1,开启APB1的TIM2EN
    TIM2->PSC = 71;             //预分频
    TIM2->ARR = 103;            //预装载
    TIM2->DIER = 0x1;           //DMA中断使能寄存器UIE允许更新中断
    TIM2->CR1 |= 0x1 << 7;      //ARPE=1允许自动装载
    TIM2->CR1 |= 0x1 << 4;      //向下计数

    //配置一个TX定时器
    NVIC->ISER[0] |= 0x1 << 29; //开启TIM3中断向量29
    RCC->APB1ENR |= 0x1 << 1;   //TIM3EN=1
    TIM3->PSC = 71;             //预分频
    TIM3->ARR = 103;            //预装载
    TIM3->DIER |= 0x1 << 0;     //UIE=1 允许更新中断
    TIM3->CR1 |= 0x1 << 7;      //允许ARR载入
    TIM3->CR1 |= 0x1 << 4;      //向下计数

    *tx = 1;

    while (1)
    {

        // device_8digi_show(d8, ven_tim.acc);
        device_8digi_show(d8, numberToShow);
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
        RxBuffer >>= 1;
        RxBuffer |= *rx << 7;
    }
    //如果是第十位,等待停止位;
    if (uartCount == 10)
    {
        while (*rx != 1)
            ;
        //要显示的内容向左移8位;
        numberToShow <<= 8;
        //把buffer的内容或进最后8位;
        numberToShow |= RxBuffer;
        //关闭TIM2使能
        TIM2->CR1 &= 0x0 << 0;
        //开启EXTI1使能
        EXTI->IMR |= 0x1 << 1;
        send(numberToShow);
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
//tx timer
void TIM3_IRQHandler()
{
    txCount++;
    if (txCount == 1)
    {
        *tx = 0;
    }
    if (txCount >= 2 && txCount <= 9)
    {
        *tx = (TxBuffer << 7) >> 7;
        TxBuffer >>= 1;
    }
    if (txCount == 10)
    {
        *tx = 1;
        TIM3->CR1 &= ~(0x1 << 0);
        txCount = 0;
    }
    TIM3->SR &= ~(0x1 << 0);
}