/*
 * kalman.c
 *
 *  Created on: 2 Mar 2020
 *      Author: candan
 */

#include "kalman.h"


typedef_kalman mKalmanFilter;


double dCalculateKalmanDataSet(double inData) {
	 int i = 0;

	  if(mKalmanFilter.iBufferCunter == 0){
	    if (inData < mKalmanFilter.iKalmanMinCriteria)
	      return inData;
	    for(i=0;i<(mKalmanFilter.iKalmanLen - 1);i++)
	    	mKalmanFilter.ary_dDataSet[i] = inData;
	    mKalmanFilter.iBufferCunter = mKalmanFilter.iKalmanLen - 1;
	    return inData;
	  }

	  float prevData=0;
	  float p=10, q=0.0001, r=0.05, kGain=0;

	  //add item last
	  mKalmanFilter.ary_dDataSet[mKalmanFilter.iKalmanLen - 1] = inData;

	  for(i=0;i<mKalmanFilter.iKalmanLen;i++){
	    inData = mKalmanFilter.ary_dDataSet[i];
	    //Kalman filter function start*******************************
	      p = p+q;
	      kGain = p/(p+r);

	      inData = prevData+(kGain*(inData-prevData));
	      p = (1-kGain)*p;
	    prevData = inData;
	      //Kalman filter function stop********************************
	  }

	  //shif buffer left 1
	  for(i=0;i<(mKalmanFilter.iKalmanLen - 1); i++){
		  mKalmanFilter.ary_dDataSet[i] = mKalmanFilter.ary_dDataSet[i+1];
	  }

	  return inData;
}

void vInitKalman(int kalman_len, int initial_data, int min_criteria) {

	mKalmanFilter.iKalmanLen=kalman_len;
	mKalmanFilter.iKalmanMinCriteria=min_criteria;

	int i = 0;
	for(i=0;i<kalman_len;i++)
		mKalmanFilter.ary_dDataSet[i] = initial_data;

	mKalmanFilter.fP=10;
	mKalmanFilter.fQ=0.0001;
	mKalmanFilter.fR=0.03;
	mKalmanFilter.fKGain=0;
}
