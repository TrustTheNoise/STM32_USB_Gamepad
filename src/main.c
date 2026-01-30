#include "device_mcu_includes.h"

int main(void)
{
    GPIOB->CRH |= GPIO_CRx_CNF_In_Analog;

    while (1)
    {

    }
}