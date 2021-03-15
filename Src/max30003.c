/*
 * max30003.c
 *
 *  Created on: 25 Tem 2020
 *      Author: candan
 */

#include "max30003.h"
#include <string.h>

#define MAX3003_INIT_DELAY_TIME 30
//TODO define hspi value here
#define max3003Spi hspi1
#define MAX3003_CS_GPIO_PORT_Pin SPI1_CS_Pin
#define MAX3003_CS_GPIO_Port SPI1_CS_GPIO_Port

typedef_max3003 mMax3003Sensor;
//local function prototypes
void static vsMax30003SoftwareReset();
void static vsMax30003RegWrite(uint8_t addrs, uint64_t data);
void static vsMax30003Synch();
void static vsMax30003RegRead(uint8_t addrs, uint8_t *data_buffer);
// attribute definitions

uint8_t ucDataLen = 8;

int32_t i = 0;

int64_t slEcgData;

uint64_t ulData;
uint64_t ulECGRaw = 0;

// Local Function Definitions

void static vsMax30003SoftwareReset() {
	vsMax30003RegWrite(SW_RST, 0x000000);
}

void static vsMax30003Synch() {
	vsMax30003RegWrite(SYNCH, 0x000000);
}

void static vsMax30003RegWrite(uint8_t addrs, uint64_t data) {
	// now combine the register address and the command into one byte:
	uint8_t ucDataToSend = (addrs << 1) | WREG;
	uint8_t ucaData[4];
	// take the chip select low to select the device:
	 HAL_GPIO_WritePin(MAX3003_CS_GPIO_Port,MAX3003_CS_GPIO_PORT_Pin,GPIO_PIN_RESET);
	 HAL_Delay(2);
	 ucaData[0] = ucDataToSend;
	 ucaData[1] = data>>16;
	 ucaData[2] = data>>8;
	 ucaData[3] = data;
	 HAL_SPI_Transmit(&max3003Spi,ucaData,4,300);
	 HAL_Delay(2);
	 HAL_GPIO_WritePin(MAX3003_CS_GPIO_Port,MAX3003_CS_GPIO_PORT_Pin,GPIO_PIN_SET);
}

void static vsMax30003RegRead(uint8_t addrs, uint8_t *data_buffer) {
	HAL_GPIO_WritePin(MAX3003_CS_GPIO_Port,MAX3003_CS_GPIO_PORT_Pin,GPIO_PIN_RESET);
	addrs = (addrs << 1)| RREG;
	HAL_SPI_Transmit(&max3003Spi,&addrs,1,300);
	HAL_SPI_Receive(&max3003Spi,data_buffer,3,300);
	HAL_GPIO_WritePin(MAX3003_CS_GPIO_Port,MAX3003_CS_GPIO_PORT_Pin,GPIO_PIN_SET);
}

// Global Function Definitions
void vMax30003Init(void) {
	vsMax30003SoftwareReset();
	HAL_Delay(MAX3003_INIT_DELAY_TIME);

	vsMax30003RegWrite(CNFG_GEN, 0x081007);
	HAL_Delay(MAX3003_INIT_DELAY_TIME);

	vsMax30003RegWrite(CNFG_CAL, 0x720000);  // 0x700000
	HAL_Delay(MAX3003_INIT_DELAY_TIME);

	vsMax30003RegWrite(CNFG_EMUX, 0x0B0000);
	HAL_Delay(MAX3003_INIT_DELAY_TIME);

	vsMax30003RegWrite(CNFG_ECG, 0x805000); // d23 - d22 : 10 for 250sps , 00:500 sps
	HAL_Delay(MAX3003_INIT_DELAY_TIME);

	vsMax30003RegWrite(CNFG_RTOR1, 0x3fc600);

	vsMax30003Synch();
	HAL_Delay(MAX3003_INIT_DELAY_TIME);
}

void vMax30003ReadData(void) {
	uint8_t ucatmpData[3] = { 0, 0, 0 };
	memset(ucatmpData,0,3);
	vsMax30003RegRead(ECG_FIFO, ucatmpData);
	mMax3003Sensor.ulData = (uint64_t) (ucatmpData[0] <<24 | ucatmpData[1]<<16 	| ucatmpData[2]<<8);
	mMax3003Sensor.lEcgData = (int64_t) (mMax3003Sensor.ulData);
	memset(ucatmpData,0,3);
	vsMax30003RegRead(RTOR, ucatmpData);
	uint64_t RTOR_msb = (uint64_t) (ucatmpData[0]);
	// RTOR_msb = RTOR_msb <<8;
	uint8_t RTOR_lsb = (uint8_t) (ucatmpData[1]);

	mMax3003Sensor.ulRtor = (RTOR_msb << 8 | RTOR_lsb);
	mMax3003Sensor.ulRtor = ((mMax3003Sensor.ulRtor >> 2) & 0x3fff);

	mMax3003Sensor.fHR = 60 / ((float) mMax3003Sensor.ulRtor * 0.008);
	mMax3003Sensor.uiHR = (uint32_t) mMax3003Sensor.fHR;  // type cast to int
	mMax3003Sensor.uiRR = (uint32_t) mMax3003Sensor.ulRtor * 8;
	//;
	mMax3003Sensor.ucaDataPacketHeader[0] = 0x0A;
	mMax3003Sensor.ucaDataPacketHeader[1] = 0xFA;
	mMax3003Sensor.ucaDataPacketHeader[2] = 0x0C;
	mMax3003Sensor.ucaDataPacketHeader[3] = 0;
	mMax3003Sensor.ucaDataPacketHeader[4] = 0x02;

	mMax3003Sensor.ucaDataPacketHeader[5] = mMax3003Sensor.lEcgData;
	mMax3003Sensor.ucaDataPacketHeader[6] = mMax3003Sensor.lEcgData >> 8;
	mMax3003Sensor.ucaDataPacketHeader[7] = mMax3003Sensor.lEcgData >> 16;
	mMax3003Sensor.ucaDataPacketHeader[8] = mMax3003Sensor.lEcgData >> 24;

	mMax3003Sensor.ucaDataPacketHeader[9] = mMax3003Sensor.uiRR;
	mMax3003Sensor.ucaDataPacketHeader[10] = mMax3003Sensor.uiRR >> 8;
	mMax3003Sensor.ucaDataPacketHeader[11] = 0x00;
	mMax3003Sensor.ucaDataPacketHeader[12] = 0x00;

	mMax3003Sensor.ucaDataPacketHeader[13] = mMax3003Sensor.uiHR;
	mMax3003Sensor.ucaDataPacketHeader[14] = mMax3003Sensor.uiHR >> 8;
	mMax3003Sensor.ucaDataPacketHeader[15] = 0x00;
	mMax3003Sensor.ucaDataPacketHeader[16] = 0x00;

	mMax3003Sensor.ucaDataPacketHeader[17] = 0x00;
	mMax3003Sensor.ucaDataPacketHeader[18] = 0x0b;
}

unsigned int uiGetMax3003ECG(){
	return mMax3003Sensor.lEcgData;
}
unsigned int uiGetMax3003RR(){
	return mMax3003Sensor.uiRR;
}
