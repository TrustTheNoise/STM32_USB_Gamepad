/**
 * @file main.c
 * @author Aliaksei Sarkisau(sark.leha05@gmail.com)
 * @brief USB HID Gamepad with stm32f103 blue pill
 * @version 0.1
 * @date 2026-01-30
 * 
 * @defgroup bsp
 * @defgroup driver
 * 
 */


#include "device_mcu_includes.h"
#include "adc_bsp.h"
#include "buttons_bsp.h"
#include "tusb.h"
#include "usb_device_driver.h"
#include "stm32f1xx_hal.h"

#define PERIOD_MS       10

uint32_t buttons_mask;
uint8_t d_pad_mask;

static struct {
    volatile int8_t x;
    volatile int8_t y;
    volatile int8_t rx;
    volatile int8_t ry;
}sticks_report;

static void setup_pll_hse_clk(void);

int main(void)
{
    setup_pll_hse_clk();
    SystemCoreClockUpdate();
    HAL_Init();

    setup_buttons_gpio();
    setup_adc();

    setup_usb_device_hid();
    uint32_t previous_ms = HAL_GetTick();
    while(1) {
        tud_task();
        uint32_t current_ms = HAL_GetTick();
        if(previous_ms - current_ms >= PERIOD_MS )
        {
            previous_ms = current_ms;
            tud_hid_gamepad_report(0, 
                sticks_report.x,
                sticks_report.y,
                0,
                0,
                sticks_report.rx,
                sticks_report.ry,
                d_pad_mask,
                buttons_mask
            );
        }
    }
}

/**************************************************************************************************/
/*                                                                                                */
/*                                       Interrupt handlers                                       */
/*                                                                                                */
/**************************************************************************************************/
void SysTick_Handler(void)
{
    HAL_IncTick();

    buttons_mask = buttons_read();
    d_pad_mask = d_pad_read();
}

void USB_HP_CAN_TX_IRQHandler(void)
{
    tusb_int_handler(0, true);
}

void USB_LP_CAN_RX0_IRQHandler(void)
{
    tusb_int_handler(0, true);
}

static inline int8_t adc12_to_int8_inverted(uint16_t adc, uint8_t inverted)
{
    int32_t v = (int32_t)adc - 2048;   
    v = -1 * v;
    v = v * 127 / 2048;                

    if (v > 127)  v = 127;
    if (v < -127) v = -127;

    return (int8_t)v;
}

void DMA1_Channel1_IRQHandler(void)
{   
    uint16_t* adc_result = (void*)0;
    if(DMA1->ISR & DMA_ISR_HTIF1)
    {
        DMA1->IFCR |= DMA_IFCR_CHTIF1;
        adc_result = adc_get_dma_buffer(0);
    }
    if(DMA1->ISR & DMA_ISR_TCIF1)
    {
        DMA1->IFCR |= DMA_IFCR_CTCIF1;
        adc_result = adc_get_dma_buffer(1);
    }
    sticks_report.x = adc12_to_int8_inverted(adc_result[0], 1);
    sticks_report.y = adc12_to_int8_inverted(adc_result[1], 1);
    sticks_report.rx = adc12_to_int8_inverted(adc_result[2], 0);
    sticks_report.ry = adc12_to_int8_inverted(adc_result[3], 0);
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

/**************************************************************************************************/
/*                                                                                                */
/*                                Static functions implementation                                 */
/*                                                                                                */
/**************************************************************************************************/

static void setup_pll_hse_clk(void)
{
    RCC->CR |= RCC_CR_HSEON;

    while( (RCC->CR & RCC_CR_HSERDY) == 0){}

    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;

    RCC->CFGR |= RCC_CFGR_PLLSRC // Sets up HSE as PLL clock source
            | RCC_CFGR_PLLMULL9
            | RCC_CFGR_ADCPRE_6 << RCC_CFGR_ADCPRE_Pos
            | RCC_CFGR_PPRE1_DIV2;

    RCC->CR |= RCC_CR_PLLON;

    while( (RCC->CR & RCC_CR_PLLRDY) == 0){}

    RCC->CFGR |= RCC_CFGR_SW_PLL;

    while((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL) {}

    return;
}