// File that contains target includes for a given MCU
#ifndef DEVICE_MCU_INCLUDES_H_
#define DEVICE_MCU_INCLUDES_H_

#include "stm32f1xx.h" // Target MCU

#ifdef USE_STM32_SDK
    #include "_cmsis_enhancement.h" // CMSIS additions we need to write a better code
#endif

#endif /* DEVICE_MCU_INCLUDES_H_ */
