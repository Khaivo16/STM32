/*
 * MAX30100_Filters.c
 *
 *  Created on: Nov 8, 2022
 *      Author: hp
 */
#include "MAX30100_Filters.h"

float stepDCRemover(DCRemover* dcr, float x) {
	float olddcw = dcr->dcw;
	dcr->dcw = (float) x + dcr->alpha * dcr->dcw;
	return (dcr->dcw - olddcw);
}

float getDCW(DCRemover* dcr) {
	return dcr->dcw;
}

float stepFilterBuLp1(FilterBuLp1* fbp, float x) //class II
{
	fbp->v[0] = fbp->v[1];
	fbp->v[1] = (0.2452372752527856026 * x) + (0.50952544949442879485 * fbp->v[0]);
	return (fbp->v[0] + fbp->v[1]);
}

float meanDiff(float M, meanDiffFilter_t* filterValues)
{
  float avg = 0;
  filterValues->sum -= filterValues->values[filterValues->index];
  filterValues->values[filterValues->index] = M;
  filterValues->sum += filterValues->values[filterValues->index];

  filterValues->index++;
  filterValues->index = filterValues->index % MEAN_FILTER_SIZE;

  if(filterValues->count < MEAN_FILTER_SIZE)
    filterValues->count++;

  avg = filterValues->sum / filterValues->count;
  return avg - M;
}
