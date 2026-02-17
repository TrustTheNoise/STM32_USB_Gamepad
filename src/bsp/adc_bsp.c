/**@file adc_bsp.c
 * 
 * @ingroup bsp
 * 
 * @brief : ADC configuration
 * 
 * @details
 * GPIO mapping:
 *  PA2 - ADC1 IN2 - Left Stick Y
 *  PA3 - ADC1 IN3 - Left Stick X
 *  PA5 - ADC1 IN5 - Right Stick X
 *  PA6 - ADC1 IN6 - Right Stick Y
 */
#include "adc_bsp.h"
#include "utils_pif.h"

uint16_t adc_dma_buffer[4*2];

#define ADC_DR_DMA_CH      DMA1_Channel1

/**************************************************************************************************/
/*                                                                                                */
/*                                 Static functions declarations                                  */
/*                                                                                                */
/**************************************************************************************************/

static inline void setup_adc_gpio( void );
static inline void setup_adc_peripheral( void );
static inline void adc_wake_up_from_power_down( void );
static inline void setup_adc_dma(void);

/**************************************************************************************************/
/*                                                                                                */
/*                                Global functions implementation                                 */
/*                                                                                                */
/**************************************************************************************************/

void setup_adc(void)
{
    setup_adc_gpio();
    setup_adc_peripheral();
    setup_adc_dma();
}

void adc_wait_until_injected_sampling_is_finished( void )
{
    while(ADC1->CR2 & ADC_CR2_JSWSTART) {};
}

/*!
 * This functions return the address of JDR1, allowing us to use a pointer to JDR1
 * as an array, so that jdr_pointer[0] = JDR1, ..., jdr_pointer[3] = JDR4
 * Of course, in this way we have access to all the memory. But if we use a pointer
 * to constant data, then we won't be able to write anything to the registers,
 * so it will work for our read-only purposes.
 */
uint32_t* adc_get_injected_measurements()
{
    return (uint32_t*)(&ADC1->JDR1);
}

uint16_t* adc_get_dma_buffer(uint8_t half)
{
    if(half)
    {
        return &adc_dma_buffer[4];
    }else
    {
        return &adc_dma_buffer[0];
    }
}

/**************************************************************************************************/
/*                                                                                                */
/*                                Static functions implementation                                 */
/*                                                                                                */
/**************************************************************************************************/
static inline void adc_wake_up_from_power_down( void )
{
    ADC1->CR2 |= ADC_CR2_ADON;
    dummy_delay_us(20); // 20 us for ADC to wake up
}


static inline void setup_adc_gpio( void )
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    const uint32_t gpio_a_crl_clear_msk = ~(GPIO_CRL_CNF2_Msk | GPIO_CRL_CNF3_Msk
                                    | GPIO_CRL_CNF5_Msk | GPIO_CRL_CNF6_Msk);  

    const uint32_t gpio_a_crl_set_msk = GPIO_CRx_MODE_In << GPIO_CRL_MODE2_Pos
                                    | GPIO_CRx_CNF_In_Analog << GPIO_CRL_CNF2_Pos
                                    | GPIO_CRx_MODE_In << GPIO_CRL_MODE3_Pos
                                    | GPIO_CRx_CNF_In_Analog << GPIO_CRL_CNF3_Pos
                                    | GPIO_CRx_MODE_In << GPIO_CRL_MODE5_Pos
                                    | GPIO_CRx_CNF_In_Analog << GPIO_CRL_CNF5_Pos
                                    | GPIO_CRx_MODE_In << GPIO_CRL_MODE6_Pos
                                    | GPIO_CRx_CNF_In_Analog << GPIO_CRL_CNF6_Pos;

    GPIOA->CRL = (GPIOA->CRL & gpio_a_crl_clear_msk) | gpio_a_crl_set_msk;
}


static inline void setup_adc_peripheral( )
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    ADC1->CR1 |= ADC_CR1_SCAN;

    adc_wake_up_from_power_down();

    // ** Calibration of ADC ** //
    ADC1->CR2 |= ADC_CR2_CAL; // Calibrate single ended channels
    while(ADC1->CR2 & ADC_CR2_CAL){}; // Wait for the end of calibration

    // ** Set up ADC ** //
    
    // TODO: calculate the number of free clock cycles during the reading of all buttons
    ADC1->SMPR2 |= ADC_SMPR_SMP_13p5_CC << ADC_SMPR2_SMP2_Pos
                | ADC_SMPR_SMP_13p5_CC << ADC_SMPR2_SMP3_Pos
                | ADC_SMPR_SMP_13p5_CC << ADC_SMPR2_SMP5_Pos
                | ADC_SMPR_SMP_13p5_CC << ADC_SMPR2_SMP6_Pos;

    ADC1->CR2 |= ADC_CR2_EXTSEL_ADC12_SWSTART << ADC_CR2_EXTSEL_Pos |
        ADC_CR2_EXTTRIG;

        ADC1->CR2 |= ADC_CR2_EXTTRIG;

    ADC1->SQR3 = 3 << ADC_SQR3_SQ1_Pos | 2 << ADC_SQR3_SQ2_Pos 
        | 5 << ADC_SQR3_SQ3_Pos | 6 << ADC_SQR3_SQ4_Pos;

    ADC1->SQR1 = ADC_SQR1_L_4_CONVERSIONS << ADC_SQR1_L_Pos;

}


static inline void setup_adc_dma(void)
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
        // ** Set up DMA ** //
    ADC1->CR2 |= ADC_CR2_DMA; // Enable DMA in ADC

    // DMA channel 1 setup
    ADC_DR_DMA_CH->CPAR = (uint32_t)&(ADC1->DR);
    ADC_DR_DMA_CH->CMAR = (uint32_t)adc_dma_buffer;  // Memory address to write to.

    ADC_DR_DMA_CH->CNDTR = 8;           // Number of transfers
    ADC_DR_DMA_CH->CCR |= DMA_CCR_MSIZE_16_BITS << DMA_CCR_MSIZE_Pos
                        | DMA_CCR_PSIZE_16_BITS << DMA_CCR_PSIZE_Pos
                        | DMA_CCR_MINC | DMA_CCR_CIRC
                        | DMA_CCR_HTIE | DMA_CCR_TCIE;        // 16 bit in and out, circular mode, increment in memmory

    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    ADC_DR_DMA_CH->CCR |= DMA_CCR_EN;       // enable DMA

    ADC1->CR2 |= ADC_CR2_SWSTART;
}