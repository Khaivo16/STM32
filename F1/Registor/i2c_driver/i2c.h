#ifndef __I2C_H_
#define __I2C_H_

#include "gpio.h"



typedef enum {
	Pin_PB6PB7,	//i2c1
	Pin_PB8PB9,	////remap i2c1
	Pin_PB10PB11,//i2c2
} Pins_I2C;



void I2Cx_Init(I2C_TypeDef* I2Cx,Pins_I2C pin,uint32_t Speed);

void I2Cx_WriteMulti(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint8_t reg, uint8_t * data, uint8_t size);

void I2Cx_ReadMulti(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint8_t reg, uint8_t * data, uint8_t size);

void i2c_write(I2C_TypeDef *I2Cx, uint16_t DevAddress,char data[]);
void i2c_start(I2C_TypeDef *I2Cx);
void i2c_add(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint8_t RW);
void i2c_data(I2C_TypeDef *I2Cx,char data);
void i2c_stop(I2C_TypeDef *I2Cx);



#endif









