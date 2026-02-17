/**
 * @file gpio_bsp.c
 * 
 * @details
 * GPIO mapping:
 *  
 *  PA0 - D-Pad Down
 *  PC15 - D-Pad Right
 *  PC14 - D-Pad Left
 *  PC13 - D-Pad Up
 *  
 *  PB0 - A Button
 *  PB1 - B Button
 *  PB10 - X Button
 *  PB11 - Y Button
 * 
 *  PB12 - LB Button
 *  PB13 - LT Button
 *  PB9 - RB Button
 *  PB8 - RT Button
 * 
 *  PA7 - Right Stick Button
 *  PA1 - Left Stick Button
 */
#include "buttons_bsp.h"
#include "device_mcu_includes.h"
#include "tusb.h"

#define DEBOUNCE_STABLE     5

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t hid_bit;
    uint8_t stable_count;    
    uint8_t state;      
} button_gpio_t;

typedef enum {
    BUTTON_A_pos = 0,
    BUTTON_B_pos,
    BUTTON_X_pos, 
    BUTTON_Y_pos, 
    BUTTON_LB_pos, 
    BUTTON_LT_pos, 
    BUTTON_RB_pos, 
    BUTTON_RT_pos, 
    BUTTON_LEFT_STICK_pos, 
    BUTTON_RIGHT_STICK_pos,
    BUTTON_COUNT, 
} button_pos_t;

static button_gpio_t buttons[] = {
    [BUTTON_A_pos]          = {GPIOB, GPIO_IDR_IDR0, GAMEPAD_BUTTON_A},
    [BUTTON_B_pos]          = {GPIOB, GPIO_IDR_IDR1, GAMEPAD_BUTTON_B},
    [BUTTON_X_pos]          = {GPIOB, GPIO_IDR_IDR10, GAMEPAD_BUTTON_X},
    [BUTTON_Y_pos]          = {GPIOB, GPIO_IDR_IDR11, GAMEPAD_BUTTON_Y},
    [BUTTON_LB_pos]         = {GPIOB, GPIO_IDR_IDR12, GAMEPAD_BUTTON_TL},
    [BUTTON_LT_pos]         = {GPIOB, GPIO_IDR_IDR13, GAMEPAD_BUTTON_TL2},
    [BUTTON_RB_pos]         = {GPIOB, GPIO_IDR_IDR9, GAMEPAD_BUTTON_TR},
    [BUTTON_RT_pos]         = {GPIOB, GPIO_IDR_IDR8, GAMEPAD_BUTTON_TR2},
    [BUTTON_LEFT_STICK_pos] = {GPIOA, GPIO_IDR_IDR1, GAMEPAD_BUTTON_THUMBL},
    [BUTTON_RIGHT_STICK_pos]= {GPIOA, GPIO_IDR_IDR7, GAMEPAD_BUTTON_THUMBR},
};

typedef enum {
    DPAD_UP_pos = 0,
    DPAD_RIGHT_pos,
    DPAD_DOWN_pos,
    DPAD_LEFT_pos,
    DPAD_COUNT
} dpad_pos_t;

static button_gpio_t d_pad[] = {
    [DPAD_UP_pos]           = {GPIOC, GPIO_IDR_IDR13},
    [DPAD_RIGHT_pos]        = {GPIOC, GPIO_IDR_IDR15},
    [DPAD_DOWN_pos]         = {GPIOA, GPIO_IDR_IDR0},
    [DPAD_LEFT_pos]         = {GPIOC, GPIO_IDR_IDR14},
};

#define DPAD_UP    (1 << DPAD_UP_pos)
#define DPAD_RIGHT (1 << DPAD_RIGHT_pos)
#define DPAD_DOWN  (1 << DPAD_DOWN_pos)
#define DPAD_LEFT  (1 << DPAD_LEFT_pos)
#define DPAD_COMBOS (1 << 4)  // 16 combinations of 4 buttons press

static const uint8_t d_pad_lookup[DPAD_COMBOS] = {
    [0b0000 ... (DPAD_COMBOS-1)] = GAMEPAD_HAT_CENTERED,
    [DPAD_UP] = GAMEPAD_HAT_UP,  
    [DPAD_RIGHT] = GAMEPAD_HAT_RIGHT,  
    [DPAD_DOWN] = GAMEPAD_HAT_DOWN,  
    [DPAD_LEFT] = GAMEPAD_HAT_LEFT,  
    [DPAD_UP | DPAD_RIGHT] = GAMEPAD_HAT_UP_RIGHT,
    [DPAD_UP | DPAD_LEFT] = GAMEPAD_HAT_UP_LEFT,
    [DPAD_DOWN | DPAD_RIGHT] = GAMEPAD_HAT_DOWN_RIGHT,
    [DPAD_DOWN | DPAD_LEFT] = GAMEPAD_HAT_DOWN_LEFT,
};

/**************************************************************************************************/
/*                                                                                                */
/*                                 Static functions declarations                                  */
/*                                                                                                */
/**************************************************************************************************/



/**************************************************************************************************/
/*                                                                                                */
/*                                Global functions implementation                                 */
/*                                                                                                */
/**************************************************************************************************/

void setup_buttons_gpio(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;

    /* GPIOA setup */
    const uint32_t gpio_a_crl_clear_msk = ~(GPIO_CRL_CNF0_Msk | GPIO_CRL_CNF1_Msk | GPIO_CRL_CNF7_Msk);
    const uint32_t gpio_a_crl_set_msk = GPIO_CRx_CNF_In_PullUpDown << GPIO_CRL_CNF0_Pos
                | GPIO_CRx_CNF_In_PullUpDown << GPIO_CRL_CNF1_Pos
                | GPIO_CRx_CNF_In_PullUpDown << GPIO_CRL_CNF7_Pos;

    GPIOA->CRL = (GPIOA->CRL & gpio_a_crl_clear_msk) | gpio_a_crl_set_msk;

    GPIOA->ODR |= GPIO_ODR_ODR0 | GPIO_ODR_ODR1 | GPIO_ODR_ODR7;

    /* GPIOB setup */
    const uint32_t gpio_b_crl_clear_msk = ~(GPIO_CRL_CNF0_Msk | GPIO_CRL_CNF1_Msk);
    const uint32_t gpio_b_crl_set_msk = GPIO_CRx_CNF_In_PullUpDown << GPIO_CRL_CNF0_Pos
                                    | GPIO_CRx_CNF_In_PullUpDown << GPIO_CRL_CNF1_Pos;
    GPIOB->CRL = (GPIOB->CRL & gpio_b_crl_clear_msk) | gpio_b_crl_set_msk;

    const uint32_t gpio_b_crh_clear_msk = ~(GPIO_CRH_CNF8_Msk | GPIO_CRH_CNF9_Msk
                                    | GPIO_CRH_CNF10_Msk | GPIO_CRH_CNF11_Msk
                                    | GPIO_CRH_CNF12_Msk | GPIO_CRH_CNF13_Msk);
    const uint32_t gpio_b_crh_set_msk = GPIO_CRx_CNF_In_PullUpDown << GPIO_CRH_CNF8_Pos
                                    | GPIO_CRx_CNF_In_PullUpDown << GPIO_CRH_CNF9_Pos
                                    | GPIO_CRx_CNF_In_PullUpDown << GPIO_CRH_CNF10_Pos
                                    | GPIO_CRx_CNF_In_PullUpDown << GPIO_CRH_CNF11_Pos
                                    | GPIO_CRx_CNF_In_PullUpDown << GPIO_CRH_CNF12_Pos
                                    | GPIO_CRx_CNF_In_PullUpDown << GPIO_CRH_CNF13_Pos;
    GPIOB->CRH = (GPIOB->CRH & gpio_b_crh_clear_msk) | gpio_b_crh_set_msk;

    GPIOB->ODR |= GPIO_ODR_ODR0 | GPIO_ODR_ODR1 | GPIO_ODR_ODR8 | GPIO_ODR_ODR9
                | GPIO_ODR_ODR10 | GPIO_ODR_ODR11 | GPIO_ODR_ODR12 | GPIO_ODR_ODR13;

    /* GPIOC setup */
    const uint32_t gpio_c_crh_clear_msk = ~(GPIO_CRH_CNF13_Msk | GPIO_CRH_CNF14_Msk
                                    | GPIO_CRH_CNF15_Msk);
    const uint32_t gpio_c_crh_set_msk = GPIO_CRx_CNF_In_PullUpDown << GPIO_CRH_CNF13_Pos
                                    | GPIO_CRx_CNF_In_PullUpDown << GPIO_CRH_CNF14_Pos
                                    | GPIO_CRx_CNF_In_PullUpDown << GPIO_CRH_CNF15_Pos;
    
    GPIOC->CRH = (GPIOC->CRH & gpio_c_crh_clear_msk) | gpio_c_crh_set_msk;

    GPIOC->ODR |= GPIO_ODR_ODR13 | GPIO_ODR_ODR14 | GPIO_ODR_ODR15;
}


uint32_t buttons_read(void)
{
    uint32_t button_mask = 0;

    for(uint8_t i = 0; i < BUTTON_COUNT; i+=1)
    {
        uint8_t raw = (buttons[i].port->IDR & buttons[i].pin) ? 0 : 1;

        if(raw == buttons[i].state) {
            if(buttons[i].stable_count < DEBOUNCE_STABLE)
                buttons[i].stable_count+=1;
        } else {
            buttons[i].stable_count = 1;
            buttons[i].state = raw;
        }

        if(buttons[i].stable_count >= DEBOUNCE_STABLE && buttons[i].state)
            button_mask |= buttons[i].hid_bit;
    }

    return button_mask;
}


uint8_t d_pad_read(void)
{
    uint8_t d_pad_mask = 0;

    for(uint8_t i = 0; i < DPAD_COUNT; i+=1)
    {
        uint8_t raw = (d_pad[i].port->IDR & d_pad[i].pin) ? 0 : 1;

        if(raw == d_pad[i].state) {
            if(d_pad[i].stable_count < DEBOUNCE_STABLE)
                d_pad[i].stable_count+=1;
        } else {
            d_pad[i].stable_count = 1;
            d_pad[i].state = raw;
        }

        if(d_pad[i].stable_count >= DEBOUNCE_STABLE && d_pad[i].state)
            d_pad_mask |= (1U << i);
    }

    return d_pad_lookup[d_pad_mask];
}

/**************************************************************************************************/
/*                                                                                                */
/*                                Static functions implementation                                 */
/*                                                                                                */
/**************************************************************************************************/