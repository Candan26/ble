/*
 * si7021.c
 *
 *  Created on: 9 Þub 2020
 *      Author: candan
 */
#include "si7021.h"
#include <string.h>

#define hi2csi7021 hi2c3
typedef_si7021 mSi7021Sensor;
//private function definitions
void vReadSerialNumbers() {
	unsigned char ucaSend[2];
	unsigned char ucaResponse[4];

	//read serial number 1
	ucaSend[0] = SI7021_ID1_CMD_MS;
	ucaSend[1] = SI7021_ID1_CMD_LS;
	HAL_I2C_Master_Transmit(&hi2csi7021, mSi7021Sensor.ucI2cAddr, ucaSend, 2, 300);
	HAL_I2C_Master_Receive(&hi2csi7021, mSi7021Sensor.ucI2cAddr, ucaResponse, 4, 300);
	memcpy(&mSi7021Sensor.uiSernum_a, ucaResponse, 4);

	// read serial number 2
	memset(ucaResponse, 0, 4 * sizeof(unsigned char));
	memset(ucaSend, 0, 2 * sizeof(unsigned char));
	ucaSend[0] = SI7021_ID2_CMD_MS;
	ucaSend[1] = SI7021_ID2_CMD_LS;
	HAL_I2C_Master_Transmit(&hi2csi7021, mSi7021Sensor.ucI2cAddr, ucaSend, 2, 300);
	HAL_I2C_Master_Receive(&hi2csi7021, mSi7021Sensor.ucI2cAddr, ucaResponse, 4, 300);
	memcpy(&mSi7021Sensor.uiSernum_b, ucaResponse, 4);
}
void vReadFirmwareRevision() {
	unsigned char ucaSend[2];
	unsigned char ucaResponse[1];

	ucaSend[0] = SI7021_FIRMVERS_CMD_MS;
	ucaSend[1] = SI7021_FIRMVERS_CMD_LS;

	HAL_I2C_Master_Transmit(&hi2csi7021, mSi7021Sensor.ucI2cAddr, ucaSend, 2, 300);
	HAL_I2C_Master_Receive(&hi2csi7021, mSi7021Sensor.ucI2cAddr, ucaResponse, 1, 300);
	mSi7021Sensor.ucFirmwareRevision = ucaResponse[0];
}
void vReset(){
	unsigned char ucaSend[1];
	ucaSend[0]=SI7021_RESET_CMD;
	HAL_I2C_Master_Transmit(&hi2csi7021, mSi7021Sensor.ucI2cAddr, ucaSend, 1, 300);
}
void vWriteRegister(unsigned char reg, unsigned char value){
	unsigned char ucaSend[2];
	ucaSend[0]=reg;
	ucaSend[1]=value;
	HAL_I2C_Master_Transmit(&hi2csi7021, mSi7021Sensor.ucI2cAddr, ucaSend, 2, 300);
}
void vReadRegister(unsigned char reg){
	unsigned char ucaSend[1];
	unsigned char ucaResponse[1];
	ucaSend[0]=reg;
	HAL_I2C_Master_Transmit(&hi2csi7021, mSi7021Sensor.ucI2cAddr, ucaSend, 2, 300);
	HAL_I2C_Master_Receive(&hi2csi7021, mSi7021Sensor.ucI2cAddr, ucaResponse, 1, 300);
	mSi7021Sensor.ucRegisterVal=ucaResponse[0];
}
void vReadHumidity() {
	unsigned char ucaSend[1];
	unsigned char ucaResponse[3];

	ucaSend[0] = SI7021_MEASRH_NOHOLD_CMD;
	HAL_I2C_Master_Transmit(&hi2csi7021, mSi7021Sensor.ucI2cAddr, ucaSend, 1, 300);
	HAL_Delay(2);
	HAL_I2C_Master_Receive(&hi2csi7021, mSi7021Sensor.ucI2cAddr, ucaResponse, 3, 300);
	uint16_t hum = ucaResponse[0] << 8 | ucaResponse[1];
	mSi7021Sensor.ucChecksumHum = ucaResponse[2];

	mSi7021Sensor.fHumidty = hum;
	mSi7021Sensor.fHumidty *= 125;
	mSi7021Sensor.fHumidty /= 0xFFFF;
	mSi7021Sensor.fHumidty -= 6;

	mSi7021Sensor.fHumidty = mSi7021Sensor.fHumidty > 100.0 ? 100.0 : mSi7021Sensor.fHumidty;
}
void vReadTemparature(){
	unsigned char ucaSend[1];
	unsigned char ucaResponse[3];

	ucaSend[0] = SI7021_MEASTEMP_NOHOLD_CMD;
	HAL_I2C_Master_Transmit(&hi2csi7021, mSi7021Sensor.ucI2cAddr, ucaSend, 1, 300);
	HAL_Delay(2);
	HAL_I2C_Master_Receive(&hi2csi7021, mSi7021Sensor.ucI2cAddr, ucaResponse, 3, 300);
	uint16_t temp = ucaResponse[0] << 8 | ucaResponse[1];
	mSi7021Sensor.ucChecksumTemp = ucaResponse[2];

	mSi7021Sensor.fTemperature=temp;
	mSi7021Sensor.fTemperature *= 175.72;
	mSi7021Sensor.fTemperature /= 0xFFFF;
	mSi7021Sensor.fTemperature -= 46.85;
}

// global function definitions
unsigned char vInitsi7021() {
	unsigned char ucaSend[1];
	unsigned char ucaResponse[1];
	mSi7021Sensor.ucI2cAddr = ADDRESS_OF_SI7021;

	ucaSend[0] = SI7021_READRHT_REG_CMD;
	HAL_I2C_Master_Transmit(&hi2csi7021, mSi7021Sensor.ucI2cAddr, ucaSend, 	1, 300);
	HAL_I2C_Master_Receive(&hi2csi7021, mSi7021Sensor.ucI2cAddr, ucaResponse, 1, 300);
	// assing function pointers
	if (ucaResponse[0] != 0x3A)
		return 0;
	vReadSerialNumbers();
	vReadFirmwareRevision();
	return 1;
}

void vSi7021ProcessHumidity() {
	vReadHumidity();
}

void vSi7021ProcessTemperature() {
	vReadTemparature();
}
