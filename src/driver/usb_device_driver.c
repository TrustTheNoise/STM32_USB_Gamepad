/**
 * @file usb_device_driver.c
 * 
 * @details
 * GPIO mapping:
 *  PA11 - USBDM
 *  PA12 - USBDP 
 */

#include "usb_device_driver.h"
#include "device_mcu_includes.h"
#include "tusb.h"


/**************************************************************************************************/
/*                                                                                                */
/*                              USB Peripheral Configuration                               */
/*                                                                                                */
/**************************************************************************************************/

static const PCD_InitTypeDef s_pcd_init = {
        .dev_endpoints = 4,
        .dma_enable = DISABLE,    // OTG_FS in stm32f411xe doesn't support DMA
        .speed = PCD_SPEED_FULL,    // stm32f103 supports only FS
        .phy_itface = PCD_PHY_EMBEDDED,
        .ep0_mps = 64,
        // other fields will be with DISABLE=0 value
};

static const GPIO_InitTypeDef s_gpioa_usb_init = {
        .Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_9,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_LOW,
};

static PCD_HandleTypeDef hpcd;


/**************************************************************************************************/
/*                                                                                                */
/*                                 Static functions declarations                                  */
/*                                                                                                */
/**************************************************************************************************/

static inline void setup_usb_gpio(void);
static inline void setup_usb_peripheral(void);

/**************************************************************************************************/
/*                                                                                                */
/*                                Global functions implementation                                 */
/*                                                                                                */
/**************************************************************************************************/

void setup_usb_device_hid(void)
{
    setup_usb_gpio();
    setup_usb_peripheral();

    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,
    };

    tusb_init(BOARD_TUD_RHPORT, &dev_init);
}


/**************************************************************************************************/
/*                                                                                                */
/*                                       TinyUSB Callbacks                                        */
/*                                                                                                */
/**************************************************************************************************/


// Invoked when received SET_REPORT control request or
// received data on OUT endpoint (Report ID = 0, Type = OUTPUT)
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{

}


// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{

}


/**************************************************************************************************/
/*                                                                                                */
/*                                Static functions implementation                                 */
/*                                                                                                */
/**************************************************************************************************/

static inline void setup_usb_gpio(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_GPIO_Init(GPIOA, (GPIO_InitTypeDef*)&s_gpioa_usb_init);
}

static inline void setup_usb_peripheral(void)
{
    __HAL_RCC_USB_CLK_ENABLE();

    hpcd.Instance = USB;
    hpcd.Init = s_pcd_init;
    
    HAL_PCD_Init(&hpcd);

    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}