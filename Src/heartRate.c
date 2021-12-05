#include "heartRate.h"

// local functions
static int16_t susAverageDCEstimator(int32_t *p, uint16_t x);
static int16_t susLowPassFIRFilter(int16_t din);
//static int32_t suiMul16(int16_t x, int16_t y);
static int32_t siMul16(int16_t x, int16_t y);
// local declerations
uint8_t offset = 0;

int16_t IR_AC_Max = 20;
int16_t IR_AC_Min = -20;
int16_t IR_AC_Signal_Current = 0;
int16_t IR_AC_Signal_Previous;
int16_t IR_AC_Signal_min = 0;
int16_t IR_AC_Signal_max = 0;
int16_t IR_Average_Estimated;
int16_t positiveEdge = 0;
int16_t negativeEdge = 0;
int16_t cbuf[32];

int32_t ir_avg_reg = 0;

static const uint16_t FIRCoeffs[12] = {172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012, 4096};
// global functions
uint8_t ucCheckForBeat(int32_t sample){
	uint8_t beatDetected = FALSE;

  IR_AC_Signal_Previous = IR_AC_Signal_Current;
    //  Process next data sample
  IR_Average_Estimated = susAverageDCEstimator(&ir_avg_reg, sample);
  IR_AC_Signal_Current = susLowPassFIRFilter(sample - IR_Average_Estimated);
  //  Detect positive zero crossing (rising edge)
  if ((IR_AC_Signal_Previous < 0) & (IR_AC_Signal_Current >= 0)) {
    IR_AC_Max = IR_AC_Signal_max; //Adjust our AC max and min
    IR_AC_Min = IR_AC_Signal_min;
    positiveEdge = 1;
    negativeEdge = 0;
    IR_AC_Signal_max = 0;
    //if ((IR_AC_Max - IR_AC_Min) > 20 & (IR_AC_Max - IR_AC_Min) < 1000)
		//if ((IR_AC_Max - IR_AC_Min) > 100 & (IR_AC_Max - IR_AC_Min) < 1000)
    if ((IR_AC_Max - IR_AC_Min) > 50 & (IR_AC_Max - IR_AC_Min) < 1000){
      //Heart beat!!!
      beatDetected = TRUE;
    }
  }
  //  Detect negative zero crossing (falling edge)
  if ((IR_AC_Signal_Previous > 0) & (IR_AC_Signal_Current <= 0)){
    positiveEdge = 0;
    negativeEdge = 1;
    IR_AC_Signal_min = 0;
  }
  //  Find Maximum value in positive cycle
  if (positiveEdge & (IR_AC_Signal_Current > IR_AC_Signal_Previous)){
    IR_AC_Signal_max = IR_AC_Signal_Current;
  }
  //  Find Minimum value in negative cycle
  if (negativeEdge & (IR_AC_Signal_Current < IR_AC_Signal_Previous)) {
    IR_AC_Signal_min = IR_AC_Signal_Current;
  }
  return(beatDetected);
}



// local functions
//  Average DC Estimator
int16_t susAverageDCEstimator(int32_t *p, uint16_t x) {
  *p += ((((long) x << 15) - *p) >> 4);
  return (*p >> 15);
}

//  Low Pass FIR Filter
int16_t susLowPassFIRFilter(int16_t din) {
  cbuf[offset] = din;
  int32_t z = siMul16(FIRCoeffs[11], cbuf[(offset - 11) & 0x1F]);
  for (uint8_t i = 0 ; i < 11 ; i++) {
    z += siMul16(FIRCoeffs[i], cbuf[(offset - i) & 0x1F] + cbuf[(offset - 22 + i) & 0x1F]);
  }
  offset++;
  offset %= 32; //Wrap condition
  return(z >> 15);
}

//  Integer multiplier
static int32_t siMul16(int16_t x, int16_t y) {
  return((long)x * (long)y);
}
