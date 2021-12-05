/*
 * kalman.h
 *
 *  Created on: 2 Mar 2020
 *      Author: candan
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#define KALMAN_FILTER_ACTIVATION_LEVEL 10
#define KALMAN_FILTER_DEFAULT_MIN_CRETERIA 250

typedef struct {

	int iKalmanLen;
	int iBufferCunter;
	int iKalmanMinCriteria;
	float fPrevData;
	float fP;
	float fQ;
	float fR;
	float fKGain;
	double ary_dDataSet[100];

}typedef_kalman;

extern typedef_kalman mKalmanFilter;

//function prototypes
double dCalculateKalmanDataSet(double inData);
void vInitKalman(int kalman_len, int initial_data, int min_criteria);

#endif /* KALMAN_H_ */
