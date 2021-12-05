/*
 * max30102.c
 *
 *  Created on: 25 Tem 2020
 *      Author: candan
 */

#include "max30102.h"


//local function prototypes

#define max1002I2c hi2c3

volatile typedef_max30102 mMax30102Sensor;
// local variables
#define FILTER_LEVEL 8 /*????*/
#define BUFF_SIZE 50
SAMPLE sampleBuff[BUFF_SIZE];

uint16_t redAC = 0;
uint32_t redDC = 0;
uint16_t iRedAC = 0;
uint32_t iRedDC = 0;

uint8_t unreadSampleCount = 0;
SAMPLE sampleBuffTemp[5];

uint8_t wr = 0, rd = 0;
uint8_t dataInit =0;
// local functions
uint8_t max30102_getStatus(void);
HAL_StatusTypeDef stI2c;

void i2c_read(uint8_t address, uint8_t config_data, uint8_t *data,
		uint32_t size) {
	if( HAL_I2C_GetState(&max1002I2c) != HAL_I2C_STATE_BUSY){
		HAL_I2C_Master_Transmit(&max1002I2c, MAX30102_ADDR_WRITE, &config_data, 1, 300);
		stI2c = HAL_I2C_Master_Receive(&max1002I2c, address, data, size, 300);
	}
	if(stI2c == HAL_ERROR){
		HAL_I2C_DeInit(&max1002I2c);
		HAL_I2C_Init(&max1002I2c);
	}

}

void i2c_write(uint8_t address, uint8_t config_data, uint8_t *data,
		uint32_t size) {
	uint8_t ucaBuffer[2];
	ucaBuffer[0] = config_data;
	ucaBuffer[1] = data[0];
	if( HAL_I2C_GetState(&max1002I2c) != HAL_I2C_STATE_BUSY){
		HAL_I2C_Master_Transmit(&max1002I2c, address, ucaBuffer, 2, 300);
	}

}

void filter(SAMPLE *s) {
	uint8_t i;
	uint32_t red = 0;
	uint32_t ired = 0;
	for (i = 0; i < FILTER_LEVEL - 1; i++) {
		red += sampleBuff[i].red;
		ired += sampleBuff[i].iRed;
	}
	s->red = (red + s->red) / FILTER_LEVEL;
	s->iRed = (ired + s->iRed) / FILTER_LEVEL;
}

void buffInsert(SAMPLE s) {
	uint8_t i;
	for (i = BUFF_SIZE - 1; i > 0; i--) {
		sampleBuff[i].red = sampleBuff[i - 1].red;
		sampleBuff[i].iRed = sampleBuff[i - 1].iRed;
	}
	sampleBuff[0].red = s.red;
	sampleBuff[0].iRed = s.iRed;
}

void calAcDc(uint16_t *rac, uint32_t *rdc, uint16_t *iac, uint32_t *idc) {
	uint32_t rMax = sampleBuff[0].red;
	uint32_t rMin = sampleBuff[0].red;
	uint32_t iMax = sampleBuff[0].iRed;
	uint32_t iMin = sampleBuff[0].iRed;

	uint8_t i;
	for (i = 0; i < BUFF_SIZE; i++) {
		if (sampleBuff[i].red > rMax)
			rMax = sampleBuff[i].red;
		if (sampleBuff[i].red < rMin)
			rMin = sampleBuff[i].red;
		if (sampleBuff[i].iRed > iMax)
			iMax = sampleBuff[i].iRed;
		if (sampleBuff[i].iRed < iMin)
			iMin = sampleBuff[i].iRed;
	}
	*rac = rMax - rMin;
	*rdc = (rMax + rMin) / 2;
	*iac = iMax - iMin;
	*idc = (iMax + iMin) / 2;
}

void max30102_getFIFO(SAMPLE *data, uint8_t sampleCount) {
	uint8_t dataTemp[5 * 6];
	if (sampleCount > 5)
		sampleCount = 5;
	i2c_read(MAX30102_ADDR_READ,RES_FIFO_DATA_REGISTER,dataTemp, 6 * sampleCount);

	uint8_t i;
	for (i = 0; i < sampleCount; i++) {
		data[i].red = (((uint32_t) dataTemp[i * 6]) << 16
				| ((uint32_t) dataTemp[i * 6 + 1]) << 8 | dataTemp[i * 6 + 2])
				& 0x3ffff;
		data[i].iRed = (((uint32_t) dataTemp[i * 6 + 3]) << 16
				| ((uint32_t) dataTemp[i * 6 + 4]) << 8 | dataTemp[i * 6 + 5])
				& 0x3ffff;
	}
}

uint8_t max30102_getStatus(){
    uint8_t data = 0, dataTemp = 0;
		i2c_read(MAX30102_ADDR_READ,RES_INTERRUPT_STATUS_1,&dataTemp, 1);
    //HAL_I2C_Mem_Read(&max1002I2c, MAX30102_ADDR_READ, RES_INTERRUPT_STATUS_1, I2C_MEMADD_SIZE_8BIT, &dataTemp, 1, 10);
    data = dataTemp;
    i2c_read(MAX30102_ADDR_READ,RES_INTERRUPT_STATUS_2,&dataTemp, 1);
		//HAL_I2C_Mem_Read(&max1002I2c, MAX30102_ADDR_READ, RES_INTERRUPT_STATUS_2, I2C_MEMADD_SIZE_8BIT, &dataTemp, 1, 10);
    return data | dataTemp;
}

uint8_t max30102_getUnreadSampleCount() {
	i2c_read(MAX30102_ADDR_READ,RES_FIFO_WRITE_POINTER,&wr,1);
	i2c_read(MAX30102_ADDR_READ,RES_FIFO_READ_POINTER,&rd,1);
	if ((wr - rd) < 0)
		return (wr - rd) + 32;
	else
		return (wr - rd);
}

// Global Function Definitions
uint8_t data = 0;
void vMax30102Init(void) {
	  /*reset*/
	    data = 0x47;//pre 0x40
			i2c_write(MAX30102_ADDR_WRITE,RES_MODE_CONFIGURATION,&data,1);
	    //HAL_I2C_Mem_Write(&max1002I2c, MAX30102_ADDR_WRITE, RES_MODE_CONFIGURATION, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
	    /*?????*/
		/*
		do
	    {
	        i2c_read(MAX30102_ADDR_READ, RES_MODE_CONFIGURATION, &data, 1);
					i2c_read(MAX30102_ADDR_WRITE, RES_MODE_CONFIGURATION, &data, 1);
	    } while (data & 0x40);
	    */
	    data = 0x40;
			i2c_write(MAX30102_ADDR_WRITE,RES_INTERRUPT_ENABLE_1,&data,1); //0x02
	    //HAL_I2C_Mem_Write(&max1002I2c, MAX30102_ADDR_WRITE, RES_INTERRUPT_ENABLE_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
	    data = 0x63;
			i2c_write(MAX30102_ADDR_WRITE,RES_SPO2_CONFIGURATION,&data,1);	//0x0a
			//HAL_I2C_Mem_Write(&max1002I2c, MAX30102_ADDR_WRITE, RES_SPO2_CONFIGURATION, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
	    /*????*/
	    data = 0xFF;//pre 0x47
			i2c_write(MAX30102_ADDR_WRITE,RES_LED_PLUSE_AMPLITUDE_1,&data,1);	//0x0c
			i2c_write(MAX30102_ADDR_WRITE,RES_LED_PLUSE_AMPLITUDE_2,&data,1);	//0x0d
			i2c_write(MAX30102_ADDR_WRITE,RES_PROXIMITY_MODE_LED_PLUSE_AMPLITUDE,&data,1);	//0x10
			//HAL_I2C_Mem_Write(&max1002I2c, MAX30102_ADDR_WRITE, RES_LED_PLUSE_AMPLITUDE_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
	    //HAL_I2C_Mem_Write(&max1002I2c, MAX30102_ADDR_WRITE, RES_LED_PLUSE_AMPLITUDE_2, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
	    //HAL_I2C_Mem_Write(&max1002I2c, MAX30102_ADDR_WRITE, RES_PROXIMITY_MODE_LED_PLUSE_AMPLITUDE, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
	    /*FIFO clear*/
	    data = 0;
			i2c_write(MAX30102_ADDR_WRITE,RES_FIFO_WRITE_POINTER,&data,1);	//0x04
			i2c_write(MAX30102_ADDR_WRITE,RES_OVERFLOW_COUNTER,&data,1);	//0x05
			i2c_write(MAX30102_ADDR_WRITE,RES_FIFO_READ_POINTER,&data,1);	//0x06
	    //HAL_I2C_Mem_Write(&max1002I2c, MAX30102_ADDR_WRITE, RES_FIFO_WRITE_POINTER, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
	    //HAL_I2C_Mem_Write(&max1002I2c, MAX30102_ADDR_WRITE, RES_OVERFLOW_COUNTER, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
	    //HAL_I2C_Mem_Write(&max1002I2c, MAX30102_ADDR_WRITE, RES_FIFO_READ_POINTER, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
	    /*interrupt status clear*/
	    data = max30102_getStatus();
	    data = 0x03;
			i2c_write(MAX30102_ADDR_WRITE,RES_MODE_CONFIGURATION,&data,1);	//0x09
	    HAL_I2C_Mem_Write(&max1002I2c, MAX30102_ADDR_WRITE, RES_MODE_CONFIGURATION, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
}

void vMax30102Shutdown(void) {
    uint8_t data = 0;
		i2c_read(MAX30102_ADDR_READ,RES_MODE_CONFIGURATION,&data, 1);
    //HAL_I2C_Mem_Read(&max1002I2c, MAX30102_ADDR_READ, RES_MODE_CONFIGURATION, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
    data |= 0x80;
		i2c_write(MAX30102_ADDR_WRITE,RES_MODE_CONFIGURATION,&data,1);
    //HAL_I2C_Mem_Write(&max1002I2c, MAX30102_ADDR_WRITE, RES_MODE_CONFIGURATION, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
}

void  vMax30102StartUp(void) {
    uint8_t data = 0;
		i2c_read(MAX30102_ADDR_READ,RES_MODE_CONFIGURATION,&data, 1);
    //HAL_I2C_Mem_Read(&max1002I2c, MAX30102_ADDR_READ, RES_MODE_CONFIGURATION, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
    data &= ~(0x80);
		i2c_write(MAX30102_ADDR_WRITE,RES_MODE_CONFIGURATION,&data,1);
    //HAL_I2C_Mem_Write(&max1002I2c, MAX30102_ADDR_WRITE, RES_MODE_CONFIGURATION, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
}
long lastBeat = 0; //Time at which the last beat occurred
void vMax30102ReadData(void) {

	if (HAL_GPIO_ReadPin(MAX30102_INT_GPIO_Port, MAX30102_INT_Pin)	== GPIO_PIN_RESET) {

		unreadSampleCount = max30102_getUnreadSampleCount();
		if(unreadSampleCount == 0 )
			return;
		max30102_getFIFO(sampleBuffTemp, unreadSampleCount);

		static uint8_t eachBeatSampleCount = 0;    //????????????
		static uint8_t lastTenBeatSampleCount[10]; //?????????????
		static uint32_t last_iRed = 0;             //???????,????
		uint8_t i, ii;
		for (i = 0; i < unreadSampleCount; i++) {
			if (sampleBuffTemp[i].iRed < 40000) //??????,??
					{
				mMax30102Sensor.ucHR = 0;
				mMax30102Sensor.ucSPO2 = 0;
				mMax30102Sensor.usDiff = 0;
				mMax30102Sensor.uiIRed = 0;
				mMax30102Sensor.uiRed = 0;
				continue;
			}
			mMax30102Sensor.uiIRed = sampleBuffTemp[i].iRed;
			mMax30102Sensor.uiRed = sampleBuffTemp[i].red;
			buffInsert(sampleBuffTemp[i]);
			calAcDc(&redAC, &redDC, &iRedAC, &iRedDC);
			filter(&sampleBuffTemp[i]);
			//??spo2
			float R = (((float) (redAC)) / ((float) (redDC)))
					/ (((float) (iRedAC)) / ((float) (iRedDC)));
			if (R >= 0.36 && R < 0.66)
				mMax30102Sensor.ucSPO2 = (uint8_t) (107 - 20 * R);
			else if (R >= 0.66 && R < 1)
				mMax30102Sensor.ucSPO2 = (uint8_t) (129.64 - 54 * R);
			//????,30-250ppm  count:200-12
			mMax30102Sensor.usDiff = last_iRed - sampleBuffTemp[i].iRed;
			// bpm temp
			/*
			if (ucCheckForBeat(sampleBuffTemp[i].iRed)){
				  long delta = HAL_GetTick() - lastBeat;                   //Measure duration between two beats
				    lastBeat = HAL_GetTick();
				mMax30102Sensor.ucHR  = 60 / (delta / 1000.0);
			}
			*/

			// bpm temp
			if (mMax30102Sensor.usDiff > 50 && eachBeatSampleCount > 12) {
				for (ii = 9; ii > 0; ii--)
					lastTenBeatSampleCount[i] = lastTenBeatSampleCount[i - 1];
				lastTenBeatSampleCount[0] = eachBeatSampleCount;
				uint32_t totalTime = 0;
				for (ii = 0; ii < 10; ii++)
					totalTime += lastTenBeatSampleCount[i];
				mMax30102Sensor.ucHR = (uint8_t) (60.0 * 10 / 0.02	/ ((float) totalTime));
				eachBeatSampleCount = 0;
			}
			last_iRed = sampleBuffTemp[i].iRed;
			eachBeatSampleCount++;
		}

	}
}

unsigned char ucGetMax30102HR() {
	return mMax30102Sensor.ucHR;
}

unsigned char ucGetMax30102SPO2() {
	return mMax30102Sensor.ucSPO2;
}

unsigned short usGetMax30102Diff() {
	return mMax30102Sensor.usDiff;
}

unsigned int uiGetMax30102PulseCounter() {
	return mMax30102Sensor.usPulseCounter;
}

uint32_t uiGetMax30102Red(){
	return mMax30102Sensor.uiRed;
}
uint32_t uiGetMax30102IRed(){
	return mMax30102Sensor.uiIRed;
}
