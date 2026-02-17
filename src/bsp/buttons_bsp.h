#ifndef BUTTONS_BSP_H_
#define BUTTONS_BSP_H_

#include <stdint.h>

void setup_buttons_gpio(void);
uint32_t buttons_read(void);
uint8_t d_pad_read(void);

#endif /* BUTTONS_BSP_H_ */ 