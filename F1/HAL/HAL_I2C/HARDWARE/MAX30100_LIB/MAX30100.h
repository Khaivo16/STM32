/*
 * MAX30100.h
 *
 *  Created on: Nov 6, 2022
 *      Author: hp
 */

#ifndef MAX30100_LIB_INC_MAX30100_H_
#define MAX30100_LIB_INC_MAX30100_H_

#include "MAX30100_config.h"
#include "MAX30100_Registers.h"
#include "ring_buffer.h"

typedef enum {
	BML_OK = 0x00U, BML_ERROR = 0x01U, BML_BUSY = 0x02U, BML_TIMEOUT = 0x03U
} BML_StatusTypeDef;



#define		I2C_Delay(x)							HAL_Delay(x);
#define		ADC_Delay(x)							HAL_Delay(x);
#define		USART_Delay(x)							HAL_Delay(x);
#define		RCC_Delay(x)							HAL_Delay(x);
#define 	Delay(x)								HAL_Delay(x);
//=================| User Include Section |===================//
//#include "BML_DEF.h"

#include "myiic.h"
//============================================================//

#define SCL_PORT	PORTB
#define SCL_PIN		8
#define SDA_PORT	PORTB
#define SDA_PIN		9
#define	MAX30100_DELAY(x)	I2C_Delay(x)

#define DEFAULT_MODE                MAX30100_MODE_HRONLY
#define DEFAULT_SAMPLING_RATE       MAX30100_SAMPRATE_100HZ
#define DEFAULT_PULSE_WIDTH         MAX30100_SPC_PW_1600US_16BITS
#define DEFAULT_RED_LED_CURRENT     MAX30100_LED_CURR_50MA
#define DEFAULT_IR_LED_CURRENT      MAX30100_LED_CURR_50MA
#define EXPECTED_PART_ID            0x11
#define RINGBUFFER_SIZE             16

BML_StatusTypeDef user_i2cConfig(void);
BML_StatusTypeDef MAX30100_begin(void);
uint8_t MAX30100_readRegister(uint8_t address);
BML_StatusTypeDef MAX30100_writeRegister(uint8_t address, uint8_t data);
BML_StatusTypeDef MAX30100_burstRead(uint8_t baseAddress, uint8_t *buffer, uint8_t length);
uint8_t MAX30100_getPartId(void);
BML_StatusTypeDef MAX30100_setMode(Mode mode);
BML_StatusTypeDef MAX30100_setLedsPulseWidth(LEDPulseWidth ledPulseWidth);
BML_StatusTypeDef MAX30100_setSamplingRate(SamplingRate samplingRate);
BML_StatusTypeDef MAX30100_setLedsCurrent(LEDCurrent irLedCurrent, LEDCurrent redLedCurrent);
BML_StatusTypeDef MAX30100_setHighresModeEnabled(bool enabled);
BML_StatusTypeDef MAX30100_update(void);
bool MAX30100_getRawValues(uint16_t *ir, uint16_t *red);
BML_StatusTypeDef MAX30100_resetFifo(void);
BML_StatusTypeDef MAX30100_readFifoData(void);
BML_StatusTypeDef MAX30100_startTemperatureSampling(void);
bool MAX30100_isTemperatureReady(void);
float MAX30100_retrieveTemperature(void);
BML_StatusTypeDef MAX30100_shutdown(void);
BML_StatusTypeDef MAX30100_resume(void);


#endif /* MAX30100_LIB_INC_MAX30100_H_ */
