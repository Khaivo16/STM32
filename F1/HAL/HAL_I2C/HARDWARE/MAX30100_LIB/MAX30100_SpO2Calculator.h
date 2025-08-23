/*
 * MAX30100_SpO2Calculator.h
 *
 *  Created on: 08-Nov-2022
 *      Author: hp
 */

#ifndef MAX30100_LIB_INC_MAX30100_SPO2CALCULATOR_H_
#define MAX30100_LIB_INC_MAX30100_SPO2CALCULATOR_H_

#include "MAX30100.h"

#define CALCULATE_EVERY_N_BEATS         3

static const uint8_t spO2LUT[43];
extern float irACValueSqSum;
extern float redACValueSqSum;
extern uint8_t beatsDetectedNum;
extern uint32_t samplesRecorded;
extern uint8_t spO2;

//    SpO2Calculator();

void SpO2Calculator_update(float irACValue, float redACValue, bool beatDetected);
void SpO2Calculator_reset(void);
uint8_t SpO2Calculator_getSpO2(void);

#endif /* MAX30100_LIB_INC_MAX30100_SPO2CALCULATOR_H_ */
