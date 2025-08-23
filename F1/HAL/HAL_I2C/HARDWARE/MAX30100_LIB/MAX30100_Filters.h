/*
 * MAX30100_Fliter.h
 *
 *  Created on: 08-Nov-2022
 *      Author: hp
 */

#ifndef MAX30100_LIB_INC_MAX30100_FILTERS_H_
#define MAX30100_LIB_INC_MAX30100_FILTERS_H_


// http://www.schwietering.com/jayduino/filtuino/
// Low pass butterworth filter order=1 alpha1=0.1
// Fs=100Hz, Fc=6Hz
#include "MAX30100.h"

typedef struct{
	float v[2];
}FilterBuLp1;

// http://sam-koblenski.blogspot.de/2015/11/everyday-dsp-for-programmers-dc-and.html

typedef struct {
	float alpha;
	float dcw;
} DCRemover;

#define MEAN_FILTER_SIZE        15

typedef struct
{
  float values[MEAN_FILTER_SIZE];
  uint8_t index;
  float sum;
  uint8_t count;
} meanDiffFilter_t;

float stepDCRemover(DCRemover* dcr, float x);
float getDCW(DCRemover* dcr);
float stepFilterBuLp1(FilterBuLp1* fbp, float x);
float meanDiff(float M, meanDiffFilter_t *filterValues);

#endif /* MAX30100_LIB_INC_MAX30100_FILTERS_H_ */
