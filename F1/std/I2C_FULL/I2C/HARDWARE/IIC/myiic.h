#ifndef __MYIIC_H
#define __MYIIC_H
//#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
/*
 *	Pinout
 *	
 *			
 *	I2CX	|SCL	SDA		|SCL	SDA		|SCL	SDA
 *			|				|				|
 *	I2C1	|PB6	PB7		|PB8	PB9		|PB6	PB9
 *	I2C2	|PB10	PB11	|PF1	PF0		|PH4	PH5
 *	I2C3	|PA8	PC9		|PH7	PH8		|-		-
 *
////////////////////////////////////////////////////////////////////////////////////// 
						 SCL:SDA
					Pin_PB6PB7,	////I2C1
					Pin_PB8PB9,	////I2C1
					Pin_PB6PB9,	////1
					Pin_PB10PB11,	////I2C2
					Pin_PF1PF0,	////I2C2
					Pin_PA8PC9,	////I2C3
					Pin_PH7PH8	////I2C3

 */								  
////////////////////////////////////////////////////////////////////////////////// 	
   	   		   

/**
 * Includes
 */
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"


// Timeout
#ifndef I2C_TIMEOUT
#define I2C_TIMEOUT					20000
#endif

//I2C1 settings, change defines
#ifndef I2C1_ACKNOWLEDGED_ADDRESS
#define I2C1_ACKNOWLEDGED_ADDRESS	I2C_AcknowledgedAddress_7bit
#endif
#ifndef I2C1_MODE
#define I2C1_MODE					I2C_Mode_I2C
#endif
#ifndef I2C1_OWN_ADDRESS
#define I2C1_OWN_ADDRESS				0x00
#endif
#ifndef I2C1_ACK
#define I2C1_ACK						I2C_Ack_Disable
#endif
#ifndef I2C1_DUTY_CYCLE
#define I2C1_DUTY_CYCLE				I2C_DutyCycle_2
#endif

//I2C2 settings, change defines
#ifndef I2C2_ACKNOWLEDGED_ADDRESS
#define I2C2_ACKNOWLEDGED_ADDRESS	I2C_AcknowledgedAddress_7bit
#endif
#ifndef I2C2_MODE
#define I2C2_MODE					I2C_Mode_I2C
#endif
#ifndef I2C2_OWN_ADDRESS
#define I2C2_OWN_ADDRESS				0x00
#endif
#ifndef I2C2_ACK
#define I2C2_ACK						I2C_Ack_Disable
#endif
#ifndef I2C2_DUTY_CYCLE
#define I2C2_DUTY_CYCLE				I2C_DutyCycle_2
#endif



//I2C speed modes
#define I2C_CLOCK_STANDARD			100000
#define I2C_CLOCK_FAST_MODE			400000
#define I2C_CLOCK_FAST_MODE_PLUS		1000000
#define I2C_CLOCK_HIGH_SPEED			3400000

/**
 * Pins used
 */
typedef enum {//SCL:SDA
	Pin_PB6PB7,	////I2C1
	Pin_PB8PB9,	////I2C1
	Pin_PB6PB9,	////1
	Pin_PB10PB11,	////I2C2
	Pin_PF1PF0,	////I2C2
	Pin_PA8PC9,	////I2C3
	Pin_PH7PH8	////I2C3
} I2C_PinsPack_t;

/**
 * Initialize I2C
 */
extern void IIC_Init(I2C_TypeDef* I2Cx, I2C_PinsPack_t pinspack, uint32_t clockSpeed);

/**
 * Read single byte from slave
 */
extern uint8_t I2C_Read(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg);

/**
 * Read single byte from slave
 */
extern void I2C_Write(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t data);

/**
 * Read multi bytes from slave
 */
extern u8 I2C_ReadMulti(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t *data, uint16_t count);

/**
 * Read multi bytes from slave without setting register from where to start read
 */
extern void I2C_ReadMultiNoRegister(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* data, uint16_t count);

/**
 * Write multi bytes from slave
 */
extern u8 I2C_WriteMulti(I2C_TypeDef* I2Cx, uint8_t address, uint8_t reg, uint8_t *data, uint16_t count);

/**
 * Checks if device is connected to I2C bus
 * Returns 1 if device is connected, or 0 if not
 */
extern uint8_t I2C_IsDeviceConnected(I2C_TypeDef* I2Cx, uint8_t address);

/**
 * I2C Start 
 */
extern int16_t I2C_Start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction, uint8_t ack);

/**
 * Stop I2c
 */
extern uint8_t I2C_Stop(I2C_TypeDef* I2Cx);

/**
 * Read byte without ack
 */
extern uint8_t I2C_ReadNack(I2C_TypeDef* I2Cx);

/**
 * Read byte with ack
 */
extern uint8_t I2C_ReadAck(I2C_TypeDef* I2Cx);

/**
 * Write to slave
 */
extern void I2C_WriteData(I2C_TypeDef* I2Cx, uint8_t data);  
#endif
















