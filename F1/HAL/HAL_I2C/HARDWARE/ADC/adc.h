/**
 * ADC library for STM32
 
 * |----------------------------------------------------------------------

 * |----------------------------------------------------------------------
 *	
 * 02, 2014
 *	- Added support for measure Vbat pin with ADC
 *	
 * Pinout
 *	CHANNEL			ADC1	ADC2	ADC3
 *	0							PA0		PA0		PA0
 *	1							PA1		PA1		PA1
 *	2							PA2		PA2		PA2
 *	3							PA3		PA3		PA3
 *	4							PA4		PA4		PF6
 *	5							PA5		PA5		PF7
 *	6							PA6		PA6		PF8
 *	7							PA7		PA7		PF9
 *	8							PB0		PB0		PF10
 *	9							PB1		PB1		PF3
 *	10						PC0		PC0		PC0
 *	11						PC1		PC1		PC1
 *	12						PC2		PC2		PC2
 *	13						PC3		PC3		PC3
 *	14						PC4		PC4		PF4
 *	15						PC5		PC5		PF5
 */
#ifndef ADC_H
#define ADC_H 110
/**
 * Library dependencies
 * - STM32F4xx
 * - STM32F4xx RCC
 * - STM32F4xx GPIO
 * - STM32F4xx ADC
 * - defines.h
 */
/**
 * Includes
 */
 #include "sys.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_adc.h"


/**
 * This values can be overwritten in defines.h file
 */
#ifndef ADC1_RESOLUTION
#define ADC1_RESOLUTION		ADC_Resolution_12b
#endif

#ifndef ADC2_RESOLUTION
#define ADC2_RESOLUTION		ADC_Resolution_12b
#endif

#ifndef ADC3_RESOLUTION
#define ADC3_RESOLUTION		ADC_Resolution_12b
#endif

/* Supply voltage is 3.3V */
#ifndef ADC_SUPPLY_VOLTAGE
#define ADC_SUPPLY_VOLTAGE		3300
#endif

/* Get multipliers for vbat, vref measurements */
#if defined (STM32F40_41xxx)
#define ADC_VBAT_MULTI			2
#endif
#if defined (STM32F427_437xx) || defined (STM32F429_439xx) || defined (STM32F401xx) || defined (STM32F411xE)
#define ADC_VBAT_MULTI			4
#endif

/**
 * Initialize ADCx peripheral
 *
 * Parameters:
 * 	- ADC_TypeDef* ADCx: ADCx
 * 		ADC1, ADC2, ADC3
 */
extern void ADC_InitADC(ADC_TypeDef* ADCx);

/**
 * Initialize ADCx with ADCx channel
 *
 * Parameters:
 * 	- ADC_TypeDef* ADCx: ADCx
 * 		ADC1, ADC2, ADC3
 * 	- uint8_t channel: channel (pin) for ADCx
 * 		- ADC_Channel_0, ADC_Channel_1, ..., ADC_Channel_15
 */
extern void Adc_Init(ADC_TypeDef* ADCx, uint8_t channel);

/**
 * Read from ADCx channel
 *
 * Parameters:
 * 	- ADC_TypeDef* ADCx: ADCx
 * 		ADC1, ADC2, ADC3
 * 	- uint8_t channel: channel (pin) for ADCx
 * 		- ADC_Channel_0, ADC_Channel_1, ..., ADC_Channel_15
 */
extern uint16_t ADC_Read(ADC_TypeDef* ADCx, uint8_t channel);
extern uint16_t Get_Adc_Average(ADC_TypeDef* ADCx, uint8_t channel,uint8_t times);
/**
 * Enable Vbat channel for ADC
 *
 * No return
 */
extern void ADC_EnableVbat(void);

/**
 * Disable Vbat channel for ADC
 *
 * No return
 */
extern void ADC_DisableVbat(void);

/**
 * Read vbat pin voltage
 *
 * Parameters:
 * 	- ADC_TypeDef* ADCx: ADCx
 * 		ADC1, ADC2, ADC3
 *
 * Result is in mV
 */
extern uint16_t ADC_ReadVbat(ADC_TypeDef* ADCx);

#endif
