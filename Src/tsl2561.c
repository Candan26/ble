/*
 * tsl2561.c
 *
 *  Created on: 11 Þub 2020
 *      Author: candan
 */

#include "tsl2561.h"

#define hi2ctsl2561 hi2c3

typedefTsl2561 mTsl2561Sensor;
//function prototypes
uint32_t tTsl2561CalculateLux(uint16_t ch0, uint16_t ch1);
tsl2561Error_t tTsl2561WriteCmd(uint8_t cmd);
tsl2561Error_t tTsl2561Write8(uint8_t reg, uint32_t value);
tsl2561Error_t tTsl2561Read16(uint8_t reg, uint16_t *value);
tsl2561Error_t tTsl2561Enable(void);
tsl2561Error_t tTsl2561Disable(void);

//static defs
static tsl2561IntegrationTime_t _tsl2561IntegrationTime = TSL2561_INTEGRATIONTIME_13MS;
static tsl2561Gain_t _tsl2561Gain = TSL2561_GAIN_0X;

void vTsl2561ProcessLuminity(){
	if(mTsl2561Sensor.ucSeqCounter>INTEGRATIONTIME_SEQ_COUNTER_13MS){
		mTsl2561Sensor.uiLuxValue=tTsl2561CalculateLux(mTsl2561Sensor.usBroadband,mTsl2561Sensor.usIr);
		mTsl2561Sensor.ucSeqCounter=0;
	}
	tTsl2561GetLuminosity(&mTsl2561Sensor.usBroadband,&mTsl2561Sensor.usIr);
}

tsl2561Error_t tTsl2561WriteCmd(uint8_t cmd) {
	unsigned char ucI2cAddr = TSL2561_ADDRESS;
	unsigned char ucaSend[1] = { cmd };
	if (HAL_I2C_Master_Transmit(&hi2ctsl2561, ucI2cAddr, ucaSend, 1, 300)
			== HAL_OK)
		return TSL2561_ERROR_OK;
	return TSL2561_ERROR_LAST;
}

tsl2561Error_t tTsl2561Write8(uint8_t reg, uint32_t value) {
	unsigned char ucI2cAddr = TSL2561_ADDRESS;
	unsigned char ucaSend[2] = { reg, (value & 0xFF) };
	if (HAL_I2C_Master_Transmit(&hi2ctsl2561, ucI2cAddr, ucaSend, 2, 100)
			== HAL_OK)
		return TSL2561_ERROR_OK;
	return TSL2561_ERROR_LAST;
}

tsl2561Error_t tTsl2561Read16(uint8_t reg, uint16_t *value) {
	unsigned char ucI2cAddr = TSL2561_ADDRESS;
	unsigned char ucaSend[1] = { reg };
	unsigned char ucaResponse[2];

	if (HAL_I2C_Master_Transmit(&hi2ctsl2561, ucI2cAddr, ucaSend, 1, 100)!= HAL_OK)
		return TSL2561_ERROR_LAST;
	if (HAL_I2C_Master_Receive(&hi2ctsl2561, ucI2cAddr, ucaResponse, 2, 100)!= HAL_OK)
		return TSL2561_ERROR_LAST;
	// Shift values to create properly formed integer (low byte first)
	*value = (ucaResponse[0] | ucaResponse[1] << 8);
	return TSL2561_ERROR_OK;
}

tsl2561Error_t tTsl2561Enable(void) {

	// Enable the device by setting the control bit to 0x03
	return tTsl2561Write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL,
	TSL2561_CONTROL_POWERON);
}

tsl2561Error_t tTsl2561Disable(void) {

	// Turn the device off to save power
	return tTsl2561Write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL,
	TSL2561_CONTROL_POWEROFF);
}

tsl2561Error_t tTsl2561Init(void) {
	mTsl2561Sensor.ucSeqCounter=0;
	// Set default integration time and gain
	tTsl2561SetTiming(_tsl2561IntegrationTime, _tsl2561Gain);
	// Note: by default, the device is in power down mode on bootup
	return TSL2561_ERROR_OK;
}

tsl2561Error_t tTsl2561SetTiming(tsl2561IntegrationTime_t integration, tsl2561Gain_t gain) {

	tsl2561Error_t error = TSL2561_ERROR_OK;
	// Enable the device by setting the control bit to 0x03
	error = tTsl2561Enable();
	if (error)
		return error;

	// Turn the device off to save power
	error = tTsl2561Write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING,
			integration | gain);
	if (error)
		return error;

	// Update value placeholders
	_tsl2561IntegrationTime = integration;
	_tsl2561Gain = gain;

	// Turn the device off to save power
	error = tTsl2561Disable();
	if (error)
		return error;

	return error;
}

tsl2561Error_t tTsl2561GetLuminosity(uint16_t *broadband, uint16_t *ir) {


	tsl2561Error_t error = TSL2561_ERROR_OK;

	// Enable the device by setting the control bit to 0x03
	if(mTsl2561Sensor.ucSeqCounter==0)
		error = tTsl2561Enable();
	if (error)
		return error;
	mTsl2561Sensor.ucSeqCounter++;
	// Wait x ms for ADC to complete
	switch (_tsl2561IntegrationTime) {
	case TSL2561_INTEGRATIONTIME_13MS:
		if(mTsl2561Sensor.ucSeqCounter<=INTEGRATIONTIME_SEQ_COUNTER_13MS)
			return TSL2561_ERROR_LAST;
		break;
	case TSL2561_INTEGRATIONTIME_101MS:
		if(mTsl2561Sensor.ucSeqCounter<=INTEGRATIONTIME_SEQ_COUNTER_101MS)
			return TSL2561_ERROR_LAST;
		break;
	default:
		if(mTsl2561Sensor.ucSeqCounter<=INTEGRATIONTIME_SEQ_COUNTER_DEFAULT)
			return TSL2561_ERROR_LAST;
		break;
	}

	// Reads two byte value from channel 0 (visible + infrared)
	error = tTsl2561Read16(
			TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW,
			broadband);
	if (error)
		return error;

	// Reads two byte value from channel 1 (infrared)
	error = tTsl2561Read16(
			TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW,
			ir);
	if (error)
		return error;

	// Turn the device off to save power
	error = tTsl2561Disable();
	if (error)
		return error;
	return error;
}

uint32_t tTsl2561CalculateLux(uint16_t ch0, uint16_t ch1) {

	unsigned long chScale;
	unsigned long channel1;
	unsigned long channel0;

	switch (_tsl2561IntegrationTime) {
	case TSL2561_INTEGRATIONTIME_13MS:
		chScale = TSL2561_LUX_CHSCALE_TINT0;
		break;
	case TSL2561_INTEGRATIONTIME_101MS:
		chScale = TSL2561_LUX_CHSCALE_TINT1;
		break;
	default: // No scaling ... integration time = 402ms
		chScale = (1 << TSL2561_LUX_CHSCALE);
		break;
	}

	// Scale for gain (1x or 16x)
	if (!_tsl2561Gain)
		chScale = chScale << 4;

	// scale the channel values
	channel0 = (ch0 * chScale) >> TSL2561_LUX_CHSCALE;
	channel1 = (ch1 * chScale) >> TSL2561_LUX_CHSCALE;

	// find the ratio of the channel values (Channel1/Channel0)
	unsigned long ratio1 = 0;
	if (channel0 != 0)
		ratio1 = (channel1 << (TSL2561_LUX_RATIOSCALE + 1)) / channel0;

	// round the ratio value
	unsigned long ratio = (ratio1 + 1) >> 1;

	unsigned int b, m;

#ifdef TSL2561_PACKAGE_CS
	if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1C))
	{	b=TSL2561_LUX_B1C; m=TSL2561_LUX_M1C;}
	else if (ratio <= TSL2561_LUX_K2C)
	{	b=TSL2561_LUX_B2C; m=TSL2561_LUX_M2C;}
	else if (ratio <= TSL2561_LUX_K3C)
	{	b=TSL2561_LUX_B3C; m=TSL2561_LUX_M3C;}
	else if (ratio <= TSL2561_LUX_K4C)
	{	b=TSL2561_LUX_B4C; m=TSL2561_LUX_M4C;}
	else if (ratio <= TSL2561_LUX_K5C)
	{	b=TSL2561_LUX_B5C; m=TSL2561_LUX_M5C;}
	else if (ratio <= TSL2561_LUX_K6C)
	{	b=TSL2561_LUX_B6C; m=TSL2561_LUX_M6C;}
	else if (ratio <= TSL2561_LUX_K7C)
	{	b=TSL2561_LUX_B7C; m=TSL2561_LUX_M7C;}
	else if (ratio > TSL2561_LUX_K8C)
	{	b=TSL2561_LUX_B8C; m=TSL2561_LUX_M8C;}
#else
	if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1T)) {
		b = TSL2561_LUX_B1T;
		m = TSL2561_LUX_M1T;
	} else if (ratio <= TSL2561_LUX_K2T) {
		b = TSL2561_LUX_B2T;
		m = TSL2561_LUX_M2T;
	} else if (ratio <= TSL2561_LUX_K3T) {
		b = TSL2561_LUX_B3T;
		m = TSL2561_LUX_M3T;
	} else if (ratio <= TSL2561_LUX_K4T) {
		b = TSL2561_LUX_B4T;
		m = TSL2561_LUX_M4T;
	} else if (ratio <= TSL2561_LUX_K5T) {
		b = TSL2561_LUX_B5T;
		m = TSL2561_LUX_M5T;
	} else if (ratio <= TSL2561_LUX_K6T) {
		b = TSL2561_LUX_B6T;
		m = TSL2561_LUX_M6T;
	} else if (ratio <= TSL2561_LUX_K7T) {
		b = TSL2561_LUX_B7T;
		m = TSL2561_LUX_M7T;
	} else if (ratio > TSL2561_LUX_K8T) {
		b = TSL2561_LUX_B8T;
		m = TSL2561_LUX_M8T;
	}
#endif

	unsigned long temp;
	temp = ((channel0 * b) - (channel1 * m));

	// do not allow negative lux value
	if (temp < 0)
		temp = 0;

	// round lsb (2^(LUX_SCALE-1))
	temp += (1 << (TSL2561_LUX_LUXSCALE - 1));

	// strip off fractional portion
	uint32_t lux = temp >> TSL2561_LUX_LUXSCALE;

	// Signal I2C had no errors
	return lux;
}
