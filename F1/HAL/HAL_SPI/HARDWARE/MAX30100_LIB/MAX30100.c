/*
 * MAX30100.c
 *
 *  Created on: Nov 6, 2022
 *      Author: hp
 */

#include "MAX30100.h"

BML_StatusTypeDef user_i2cConfig(void) {
//	gpio_config(SCL_PORT, SCL_PIN, ALT_MODE, HIGH_SPEED, EN_PU, OD);
//	gpio_config(SDA_PORT, SDA_PIN, ALT_MODE, HIGH_SPEED, EN_PU, OD);

//	gpio_altfn(SCL_PORT, SCL_PIN, I2C1_SCL_PB8);
//	gpio_altfn(SDA_PORT, SDA_PIN, I2C1_SDA_PB9);

//	I2C_INIT(_MAX30100_I2C, _MAX30100_I2C_SPEED);

//	return I2C_DeviceReady(I2C1, _MAX30100_I2C_ADDRESS, 1000);
	IIC_Init(I2C1, Pin_PB8PB9, 100000);
	
	return BML_OK;
	
}

BML_StatusTypeDef MAX30100_begin(void) {
	if (user_i2cConfig() != BML_OK) {
		return BML_ERROR;
	}//printf("%d \n",MAX30100_getPartId());

	if (MAX30100_getPartId() != EXPECTED_PART_ID) {
		return BML_ERROR;
	}

	if (MAX30100_setMode(DEFAULT_MODE) != BML_OK) {
		return BML_ERROR;
	}
	if (MAX30100_setLedsPulseWidth(DEFAULT_PULSE_WIDTH) != BML_OK) {
		return BML_ERROR;
	}
	if (MAX30100_setSamplingRate(DEFAULT_SAMPLING_RATE) != BML_OK) {
		return BML_ERROR;
	}
	if (MAX30100_setLedsCurrent(DEFAULT_IR_LED_CURRENT, DEFAULT_RED_LED_CURRENT)
			!= BML_OK) {
		return BML_ERROR;
	}
	if (MAX30100_setHighresModeEnabled(1) != BML_OK) {
		return BML_ERROR;
	}
	return BML_OK;
}

uint8_t MAX30100_readRegister(uint8_t address) {
//	uint8_t rdata = I2C_Read(_MAX30100_I2C, _MAX30100_I2C_ADDRESS, address);
//	if ( rdata == BML_OK) {
//		return 255;
//	}
	return I2C_Read(_MAX30100_I2C, _MAX30100_I2C_ADDRESS, address);
}

BML_StatusTypeDef MAX30100_writeRegister(uint8_t address, uint8_t data) {
	I2C_Write(_MAX30100_I2C, _MAX30100_I2C_ADDRESS, address,data);
	return BML_OK;
}

BML_StatusTypeDef MAX30100_burstRead(uint8_t baseAddress, uint8_t *buffer,
		uint8_t length) {
	return I2C_ReadMulti(_MAX30100_I2C, _MAX30100_I2C_ADDRESS, baseAddress, buffer, length);
}

uint8_t MAX30100_getPartId() {
	return MAX30100_readRegister(0xff);
}

BML_StatusTypeDef MAX30100_setMode(Mode mode) {
	return MAX30100_writeRegister(MAX30100_REG_MODE_CONFIGURATION, mode);
}

BML_StatusTypeDef MAX30100_setLedsPulseWidth(LEDPulseWidth ledPulseWidth) {
	uint8_t previous = MAX30100_readRegister(MAX30100_REG_SPO2_CONFIGURATION);
	return MAX30100_writeRegister(MAX30100_REG_SPO2_CONFIGURATION,
			(previous & 0xfc) | ledPulseWidth);
}

BML_StatusTypeDef MAX30100_setSamplingRate(SamplingRate samplingRate) {
	uint8_t previous = MAX30100_readRegister(MAX30100_REG_SPO2_CONFIGURATION);
	return MAX30100_writeRegister(MAX30100_REG_SPO2_CONFIGURATION,
			(previous & 0xe3) | (samplingRate << 2));
}

BML_StatusTypeDef MAX30100_setLedsCurrent(LEDCurrent irLedCurrent,
		LEDCurrent redLedCurrent) {
	return MAX30100_writeRegister(MAX30100_REG_LED_CONFIGURATION,
			redLedCurrent << 4 | irLedCurrent);
}

BML_StatusTypeDef MAX30100_setHighresModeEnabled(bool enabled) {
	uint8_t previous = MAX30100_readRegister(MAX30100_REG_SPO2_CONFIGURATION);
	if (enabled) {
		return MAX30100_writeRegister(MAX30100_REG_SPO2_CONFIGURATION,
				previous | MAX30100_SPC_SPO2_HI_RES_EN);
	} else {
		return MAX30100_writeRegister(MAX30100_REG_SPO2_CONFIGURATION,
				previous & ~MAX30100_SPC_SPO2_HI_RES_EN);
	}
}

BML_StatusTypeDef MAX30100_update() {
	return MAX30100_readFifoData();
}

bool MAX30100_getRawValues(uint16_t *ir, uint16_t *red) {
	bool result = 0;
	rb_data_type data;

	if (rb_size()) {
		data = rb_shift();
		*ir = data.ir;
		*red = data.red;
		result = 1;
	}
	return result;
}

BML_StatusTypeDef MAX30100_resetFifo() {
	if (MAX30100_writeRegister(MAX30100_REG_FIFO_WRITE_POINTER, 0) != BML_OK) {
		return BML_ERROR;
	}
	if (MAX30100_writeRegister(MAX30100_REG_FIFO_READ_POINTER, 0) != BML_OK) {
		return BML_ERROR;
	}
	if (MAX30100_writeRegister(MAX30100_REG_FIFO_OVERFLOW_COUNTER, 0)
			!= BML_OK) {
		return BML_ERROR;
	}
	return BML_OK;
}

BML_StatusTypeDef MAX30100_readFifoData() {
	uint8_t buffer[MAX30100_FIFO_DEPTH * 4];
	uint8_t toRead;
	rb_data_type value;

	toRead = (MAX30100_readRegister(MAX30100_REG_FIFO_WRITE_POINTER)- MAX30100_readRegister(MAX30100_REG_FIFO_READ_POINTER))& (MAX30100_FIFO_DEPTH - 1);

	if (toRead) {
		if (MAX30100_burstRead(MAX30100_REG_FIFO_DATA, buffer, 4 * toRead)
				!= BML_OK) {
			return BML_ERROR;
		}

		for (uint8_t i = 0; i < toRead; ++i) {
			// Warning: the values are always left-aligned
			value.ir = (uint16_t) ((buffer[i * 4] << 8) | buffer[i * 4 + 1]);
			value.red =(uint16_t) ((buffer[i * 4 + 2] << 8) | buffer[i * 4 + 3]);
			rb_push(value);
		}
		return BML_OK;
	}
	return BML_TIMEOUT;
}

BML_StatusTypeDef MAX30100_startTemperatureSampling() {
	uint8_t modeConfig = MAX30100_readRegister(MAX30100_REG_MODE_CONFIGURATION);
	modeConfig |= MAX30100_MC_TEMP_EN;

	if (MAX30100_writeRegister(MAX30100_REG_MODE_CONFIGURATION, modeConfig)
			!= BML_OK) {
		return BML_ERROR;
	}
	return BML_OK;
}

bool MAX30100_isTemperatureReady() {
	return !(MAX30100_readRegister(MAX30100_REG_MODE_CONFIGURATION)
			& MAX30100_MC_TEMP_EN);
}

float MAX30100_retrieveTemperature() {
	int8_t tempInteger = MAX30100_readRegister(
	MAX30100_REG_TEMPERATURE_DATA_INT);
	float tempFrac = MAX30100_readRegister(MAX30100_REG_TEMPERATURE_DATA_FRAC);

	return tempFrac * 0.0625 + tempInteger;
}

BML_StatusTypeDef MAX30100_shutdown() {
	uint8_t modeConfig = MAX30100_readRegister(MAX30100_REG_MODE_CONFIGURATION);
	modeConfig |= MAX30100_MC_SHDN;

	if (MAX30100_writeRegister(MAX30100_REG_MODE_CONFIGURATION, modeConfig)
			!= BML_OK) {
		return BML_ERROR;
	}
	return BML_OK;
}

BML_StatusTypeDef MAX30100_resume() {
	uint8_t modeConfig = MAX30100_readRegister(MAX30100_REG_MODE_CONFIGURATION);
	modeConfig &= ~MAX30100_MC_SHDN;

	if (MAX30100_writeRegister(MAX30100_REG_MODE_CONFIGURATION, modeConfig)
			!= BML_OK) {
		return BML_ERROR;
	}
	return BML_OK;
}
