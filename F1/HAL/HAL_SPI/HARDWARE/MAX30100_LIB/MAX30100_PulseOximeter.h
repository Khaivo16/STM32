/*
 * MAX30100_PulseOximeter.h
 *
 *  Created on: 08-Nov-2022
 *      Author: hp
 */

#ifndef MAX30100_LIB_INC_MAX30100_PULSEOXIMETER_H_
#define MAX30100_LIB_INC_MAX30100_PULSEOXIMETER_H_


#include "usart.h"

#include "MAX30100.h"
#include "MAX30100_BeatDetector.h"
#include "MAX30100_Filters.h"
#include "MAX30100_SpO2Calculator.h"

#define SAMPLING_FREQUENCY                  100
#define CURRENT_ADJUSTMENT_PERIOD_MS        500
#define DEFAULT_IR_LED_CURRENT              MAX30100_LED_CURR_50MA
#define RED_LED_CURRENT_START               MAX30100_LED_CURR_27_1MA
#define DC_REMOVER_ALPHA                    0.95

typedef enum PulseOximeterState {
	PULSEOXIMETER_STATE_INIT,
	PULSEOXIMETER_STATE_IDLE,
	PULSEOXIMETER_STATE_DETECTING
} PulseOximeterState;

typedef enum PulseOximeterDebuggingMode {
	PULSEOXIMETER_DEBUGGINGMODE_NONE,
	PULSEOXIMETER_DEBUGGINGMODE_RAW_VALUES,
	PULSEOXIMETER_DEBUGGINGMODE_AC_VALUES,
	PULSEOXIMETER_DEBUGGINGMODE_PULSEDETECT
} PulseOximeterDebuggingMode;

extern PulseOximeterState pstate;
extern PulseOximeterDebuggingMode debuggingMode;
extern uint32_t tsFirstBeatDetected;
extern uint32_t tsLastBeatDetected;
extern uint32_t tsLastBiasCheck;
extern uint32_t tsLastCurrentAdjustment;
extern uint8_t redLedCurrentIndex;
extern LEDCurrent irLedCurrent;
extern DCRemover irDCRemover;
extern DCRemover redDCRemover;
extern FilterBuLp1 lpf;

extern void *onBeatDetected;

bool PulseOximeter_begin(PulseOximeterDebuggingMode debuggingMode_);
BML_StatusTypeDef PulseOximeter_update(void);
float PulseOximeter_getHeartRate(void);
uint8_t PulseOximeter_getSpO2(void);
uint8_t PulseOximeter_getRedLedCurrentBias(void);
void PulseOximeter_setOnBeatDetectedCallback(void (*cb)());
void PulseOximeter_setIRLedCurrent(LEDCurrent irLedCurrent);
void PulseOximeter_shutdown(void);
void PulseOximeter_resume(void);

void PulseOximeter_checkSample(void);
void PulseOximeter_checkCurrentBias(void);


#endif /* MAX30100_LIB_INC_MAX30100_PULSEOXIMETER_H_ */
