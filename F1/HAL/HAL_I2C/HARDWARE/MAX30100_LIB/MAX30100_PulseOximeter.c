/*
 * MAX30100_PulseOximeter.c
 *
 *  Created on: Nov 7, 2022
 *      Author: hp
 */
#include "MAX30100_PulseOximeter.h"

PulseOximeterState pstate = PULSEOXIMETER_STATE_INIT;
PulseOximeterDebuggingMode debuggingMode;
uint32_t tsFirstBeatDetected = 0;
uint32_t tsLastBeatDetected = 0;
uint32_t tsLastBiasCheck = 0;
uint32_t tsLastCurrentAdjustment = 0;
uint8_t redLedCurrentIndex = (uint8_t)RED_LED_CURRENT_START;
LEDCurrent irLedCurrent = DEFAULT_IR_LED_CURRENT;
DCRemover irDCRemover;
DCRemover redDCRemover;
FilterBuLp1 lpf;
void *onBeatDetected = NULL;

bool PulseOximeter_begin(PulseOximeterDebuggingMode debuggingMode_)
{
    debuggingMode = debuggingMode_;

    BML_StatusTypeDef ready = MAX30100_begin();

    if (ready !=  BML_OK) {
        if (debuggingMode != PULSEOXIMETER_DEBUGGINGMODE_NONE) {
            printf("Failed to initialize the HRM sensor");
        }
        return 0;
    }

//    MAX30100_setMode(MAX30100_MODE_SPO2_HR);
//    MAX30100_setLedsCurrent(irLedCurrent, (LEDCurrent)redLedCurrentIndex);
	MAX30100_setMode(MAX30100_MODE_SPO2_HR);
	MAX30100_setLedsCurrent(MAX30100_LED_CURR_50MA, MAX30100_LED_CURR_27_1MA);
	MAX30100_setSamplingRate(MAX30100_SAMPRATE_100HZ);
	MAX30100_setHighresModeEnabled(1);

    irDCRemover.alpha = DC_REMOVER_ALPHA;
    redDCRemover.alpha = DC_REMOVER_ALPHA;

    pstate = PULSEOXIMETER_STATE_IDLE;

    lpf.v[0] = 0;

    return 1;
}

BML_StatusTypeDef PulseOximeter_update()
{
    if(MAX30100_update() != BML_OK){
    	return BML_ERROR;
    }

    PulseOximeter_checkSample();
    PulseOximeter_checkCurrentBias();

    return BML_OK;
}

float PulseOximeter_getHeartRate()
{
    return BeatDetector_getRate();
}

uint8_t PulseOximeter_getSpO2()
{
    return SpO2Calculator_getSpO2();
}

uint8_t PulseOximeter_getRedLedCurrentBias()
{
    return redLedCurrentIndex;
}

void PulseOximeter_setOnBeatDetectedCallback(void (*cb)())
{
    onBeatDetected = cb;
}

void PulseOximeter_setIRLedCurrent(LEDCurrent irLedNewCurrent)
{
    irLedCurrent = irLedNewCurrent;
    MAX30100_setLedsCurrent(irLedCurrent, (LEDCurrent)redLedCurrentIndex);
}

void PulseOximeter_shutdown()
{
    MAX30100_shutdown();
}

void PulseOximeter_resume()
{
    MAX30100_resume();
}

void PulseOximeter_checkSample()
{
    uint16_t rawIRValue, rawRedValue;
//    uint8_t str[100] = { 0 };
    // Dequeue all available samples, they're properly timed by the HRM
    while (MAX30100_getRawValues(&rawIRValue, &rawRedValue)) {
        float irACValue = stepDCRemover(&irDCRemover, rawIRValue);
        float redACValue = stepDCRemover(&redDCRemover, rawRedValue);
        // The signal fed to the beat detector is mirrored since the cleanest monotonic spike is below zero
        float filteredPulseValue = stepFilterBuLp1(&lpf, -irACValue);
        bool beatDetected = BeatDetector_addSample(filteredPulseValue);

        if (BeatDetector_getRate() > 0) {
        	pstate = PULSEOXIMETER_STATE_DETECTING;
            SpO2Calculator_update(irACValue, redACValue, beatDetected);
        } else if (pstate == PULSEOXIMETER_STATE_DETECTING) {
        	pstate = PULSEOXIMETER_STATE_IDLE;
            SpO2Calculator_reset();
        }

        switch (debuggingMode) {
            case PULSEOXIMETER_DEBUGGINGMODE_RAW_VALUES:
            	printf("R: %d, %d \n",rawIRValue, rawRedValue);
                break;

            case PULSEOXIMETER_DEBUGGINGMODE_AC_VALUES:
                printf("R: %f, %f \n",irACValue, redACValue);
                break;

            case PULSEOXIMETER_DEBUGGINGMODE_PULSEDETECT:
            	printf("R: %f, %f \n",filteredPulseValue, BeatDetector_getCurrentThreshold());
                break;

            default:
                break;
        }

        if (beatDetected && onBeatDetected) {
            void *onBeatDetected = NULL;
        }
    }
}

void PulseOximeter_checkCurrentBias()
{
    // Follower that adjusts the red led current in order to have comparable DC baselines between
    // red and IR leds. The numbers are really magic: the less possible to avoid oscillations
    if (HAL_GetTick() - tsLastBiasCheck > CURRENT_ADJUSTMENT_PERIOD_MS) {
        bool changed = 0;
        if (getDCW(&irDCRemover) - getDCW(&redDCRemover) > 70000 && redLedCurrentIndex < MAX30100_LED_CURR_50MA) {
            ++redLedCurrentIndex;
            changed = 1;
        } else if (getDCW(&redDCRemover) - getDCW(&irDCRemover) > 70000 && redLedCurrentIndex > 0) {
            --redLedCurrentIndex;
            changed = 1;
        }

        if (changed) {
            MAX30100_setLedsCurrent(irLedCurrent, (LEDCurrent)redLedCurrentIndex);
            tsLastCurrentAdjustment = HAL_GetTick();

            if (debuggingMode != PULSEOXIMETER_DEBUGGINGMODE_NONE) {
                printf("I: %d\n", redLedCurrentIndex);
            }
        }

        tsLastBiasCheck = HAL_GetTick();
    }
}

