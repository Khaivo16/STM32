/*
 * MAX30100_BeatDetector.c
 *
 *  Created on: Nov 7, 2022
 *      Author: hp
 */
#include "MAX30100_BeatDetector.h"

//#ifndef min
//#define min(a,b) \
//   ({ __typeof__ (a) _a = (a); \
//       __typeof__ (b) _b = (b); \
//     _a < _b ? _a : _b; })
//#endif

#define min(a, b)	a<b?a:b

BeatDetectorState state = BEATDETECTOR_STATE_INIT;
float threshold = BEATDETECTOR_MIN_THRESHOLD;
float beatPeriod = 0;
float lastMaxValue = 0;
uint32_t tsLastBeat = 0;

bool BeatDetector_addSample(float sample)
{
    return BeatDetector_checkForBeat(sample);
}

float BeatDetector_getRate()
{
    if (beatPeriod != 0) {
        return 1 / beatPeriod * 1000 * 60;
    } else {
        return 0;
    }
}

float BeatDetector_getCurrentThreshold()
{
    return threshold;
}

bool BeatDetector_checkForBeat(float sample)
{
    bool beatDetected = 0;

    switch (state) {
    printf("In check for beat\n");
        case BEATDETECTOR_STATE_INIT:
            if (HAL_GetTick() > BEATDETECTOR_INIT_HOLDOFF) {
                state = BEATDETECTOR_STATE_WAITING;
            }
            break;

        case BEATDETECTOR_STATE_WAITING:
            if (sample > threshold) {
                threshold = min(sample, BEATDETECTOR_MAX_THRESHOLD);
                state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
            }

            // Tracking lost, resetting
            if (HAL_GetTick() - tsLastBeat > BEATDETECTOR_INVALID_READOUT_DELAY) {
                beatPeriod = 0;
                lastMaxValue = 0;
            }

            BeatDetector_decreaseThreshold();
            break;

        case BEATDETECTOR_STATE_FOLLOWING_SLOPE:
            if (sample < threshold) {
                state = BEATDETECTOR_STATE_MAYBE_DETECTED;
            } else {
                threshold = min(sample, BEATDETECTOR_MAX_THRESHOLD);
            }
            break;

        case BEATDETECTOR_STATE_MAYBE_DETECTED:
            if (sample + BEATDETECTOR_STEP_RESILIENCY < threshold) {
                // Found a beat
                beatDetected = 1;
                lastMaxValue = sample;
                state = BEATDETECTOR_STATE_MASKING;
                float delta = HAL_GetTick() - tsLastBeat;
                if (delta) {
                    beatPeriod = BEATDETECTOR_BPFILTER_ALPHA * delta +
                            (1 - BEATDETECTOR_BPFILTER_ALPHA) * beatPeriod;
                }

                tsLastBeat = HAL_GetTick();
            } else {
                state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
            }
            break;

        case BEATDETECTOR_STATE_MASKING:
            if (HAL_GetTick() - tsLastBeat > BEATDETECTOR_MASKING_HOLDOFF) {
                state = BEATDETECTOR_STATE_WAITING;
            }
            BeatDetector_decreaseThreshold();
            break;
    }

    return beatDetected;
}

void BeatDetector_decreaseThreshold()
{
    // When a valid beat rate readout is present, target the
    if (lastMaxValue > 0 && beatPeriod > 0) {
        threshold -= lastMaxValue * (1 - BEATDETECTOR_THRESHOLD_FALLOFF_TARGET) /
                (beatPeriod / BEATDETECTOR_SAMPLES_PERIOD);
    } else {
        // Asymptotic decay
        threshold *= BEATDETECTOR_THRESHOLD_DECAY_FACTOR;
    }

    if (threshold < BEATDETECTOR_MIN_THRESHOLD) {
        threshold = BEATDETECTOR_MIN_THRESHOLD;
    }
}

