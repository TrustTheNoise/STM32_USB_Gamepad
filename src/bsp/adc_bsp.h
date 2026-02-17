#ifndef ADC_BSP_H_
#define ADC_BSP_H_

#include "device_mcu_includes.h"

void setup_adc(void);
void adc_wait_until_injected_sampling_is_finished( void );
uint16_t* adc_get_dma_buffer(uint8_t half);

#endif