/*
 * MAX30100_SpO2Calculator.c
 *
 *  Created on: Nov 7, 2022
 *      Author: hp
 */
#include "MAX30100_SpO2Calculator.h"

float irACValueSqSum = 0;
float redACValueSqSum = 0;
uint8_t beatsDetectedNum = 0;
uint32_t samplesRecorded = 0;
uint8_t spO2 = 0;

// SaO2 Look-up Table
// http://www.ti.com/lit/an/slaa274b/slaa274b.pdf
const uint8_t SpO2Calculator_spO2LUT[43] = {100,100,100,100,99,99,99,99,99,99,98,98,98,98,
                                             98,97,97,97,97,97,97,96,96,96,96,96,96,95,95,
                                             95,95,95,95,94,94,94,94,94,93,93,93,93,93};

void SpO2Calculator_update(float irACValue, float redACValue, bool beatDetected)
{
    irACValueSqSum += irACValue * irACValue;
    redACValueSqSum += redACValue * redACValue;
    ++samplesRecorded;

    if (beatDetected) {
        ++beatsDetectedNum;
        if (beatsDetectedNum == CALCULATE_EVERY_N_BEATS) {
            float acSqRatio = 100.0 * log(redACValueSqSum/samplesRecorded) / log(irACValueSqSum/samplesRecorded);
            uint8_t index = 0;

            if (acSqRatio > 66) {
                index = (uint8_t)acSqRatio - 66;
            } else if (acSqRatio > 50) {
                index = (uint8_t)acSqRatio - 50;
            }
            SpO2Calculator_reset();

            spO2 = spO2LUT[index];
        }
    }
}

void SpO2Calculator_reset()
{
    samplesRecorded = 0;
    redACValueSqSum = 0;
    irACValueSqSum = 0;
    beatsDetectedNum = 0;
    spO2 = 0;
}

uint8_t SpO2Calculator_getSpO2()
{
    return spO2;
}
