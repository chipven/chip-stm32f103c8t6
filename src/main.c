#include "main.h"
int main()
{
    RCC->APB2ENR |= 0x00000008;
    GPIOB->CRH &= 0xeee0eeee;
    GPIOB->CRH |= 0x00030000;

    // unsigned int *b12 = (unsigned int *)(0x42000000 + 0x10c0c * 0x20 + 12 * 4);
    unsigned int *b12 = B12_out;
    while (1)
    {
        *b12 = 0;
        for (int i = 0; i < 1000000; i++)
        {
        }
        *b12 = 1;
        for (int i = 0; i < 1000000; i++)
        {
        }
    }
}
