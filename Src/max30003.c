/*
 * max30003.c
 *
 *  Created on: 25 Tem 2020
 *      Author: candan
 */

#include "max30003.h"
#include <string.h>

#define MAX3003_INIT_DELAY_TIME 10
//TODO define hspi value here
#define max3003Spi hspi1
#define MAX3003_CS_GPIO_PORT_Pin SPI1_CS_Pin
#define MAX3003_CS_GPIO_Port SPI1_CS_GPIO_Port

volatile typedef_max3003 mMax3003Sensor;
//local function prototypes
void vMax30003SoftwareReset();
void vMax30003RegWrite(uint8_t addrs, uint64_t data);
void vMax30003Synch();
void vMax30003RegRead(uint8_t addrs, uint8_t *data_buffer);
// attribute definitions
uint8_t ucDataLen = 8;
int32_t i = 0;
int64_t slEcgData;
uint64_t ulData;
uint64_t ulECGRaw = 0;
uint32_t uicounterOfReset = 0;

uint32_t ecgFIFO, readECGSamples, idx, ETAG[32], status;
int16_t ecgSample[32];
// Local Function Definitions
void vMax30003SoftwareReset() {
	vMax30003RegWrite(SW_RST, 0x000000);
}

void vMax30003Synch() {
	vMax30003RegWrite(SYNCH, 0x000000);
}

void vMax30003RegWrite(uint8_t addrs, uint64_t data) {
	// now combine the register address and the command into one byte:
	uint8_t ucDataToSend = (addrs << 1) | WREG;
	uint8_t ucaData[4];
	// take the chip select low to select the device:
	HAL_GPIO_WritePin(MAX3003_CS_GPIO_Port, MAX3003_CS_GPIO_PORT_Pin,
			GPIO_PIN_RESET);
	//HAL_Delay(1);
	ucaData[0] = ucDataToSend;
	ucaData[1] = data >> 16;
	ucaData[2] = data >> 8;
	ucaData[3] = data;
	HAL_SPI_Transmit(&max3003Spi, ucaData, 4, 300);
	//HAL_Delay(1);
	HAL_GPIO_WritePin(MAX3003_CS_GPIO_Port, MAX3003_CS_GPIO_PORT_Pin,
			GPIO_PIN_SET);
}

void vMax30003RegRead(uint8_t addrs, uint8_t *data_buffer) {
	HAL_GPIO_WritePin(MAX3003_CS_GPIO_Port, MAX3003_CS_GPIO_PORT_Pin,
			GPIO_PIN_RESET);
	addrs = (addrs << 1) | RREG;
	HAL_SPI_Transmit(&max3003Spi, &addrs, 1, 30);
	HAL_SPI_Receive(&max3003Spi, data_buffer, 3, 30);
	HAL_GPIO_WritePin(MAX3003_CS_GPIO_Port, MAX3003_CS_GPIO_PORT_Pin,
			GPIO_PIN_SET);
}

void vMax3003InitFormer() {

	vMax30003SoftwareReset();
	HAL_Delay(MAX3003_INIT_DELAY_TIME);

	vMax30003RegWrite(CNFG_GEN, 0x081007);
	HAL_Delay(MAX3003_INIT_DELAY_TIME);

	//vMax30003RegWrite(CNFG_CAL, 0x720000);  // 0x700000
	HAL_Delay(MAX3003_INIT_DELAY_TIME);

	vMax30003RegWrite(CNFG_EMUX, 0x0B0000);
	HAL_Delay(MAX3003_INIT_DELAY_TIME);

	vMax30003RegWrite(CNFG_ECG, 0x805000); // d23 - d22 : 10 for 250sps , 00:500 sps
	HAL_Delay(MAX3003_INIT_DELAY_TIME);

	vMax30003RegWrite(CNFG_RTOR1, 0x3fc600);

	vMax30003Synch();
	HAL_Delay(MAX3003_INIT_DELAY_TIME);
}

void vMax3003ReadDataFormer() {
	uint8_t ucatmpData[3] = { 0, 0, 0 };
	memset(ucatmpData, 0, 3);
	vMax30003RegRead(ECG_FIFO, ucatmpData);

	unsigned long data0 = (uint64_t) (ucatmpData[0]);
	data0 = data0 << 24;
	unsigned long data1 = (uint64_t) (ucatmpData[1]);
	data1 = data1 << 16;
	unsigned long data2 = (uint64_t) (ucatmpData[2]);
	data2 = data2 >> 6;
	data2 = data2 & 0x03;
	mMax3003Sensor.ulData = (uint64_t) (data0 | data1 | data2);
	mMax3003Sensor.lEcgData = (int64_t) (mMax3003Sensor.ulData);
	memset(ucatmpData, 0, 3);
	vMax30003RegRead(RTOR, ucatmpData);
	uint64_t RTOR_msb = (uint64_t) (ucatmpData[0]);
	// RTOR_msb = RTOR_msb <<8;
	uint8_t RTOR_lsb = (uint8_t) (ucatmpData[1]);

	mMax3003Sensor.ulRtor = (RTOR_msb << 8 | RTOR_lsb);
	mMax3003Sensor.ulRtor = ((mMax3003Sensor.ulRtor >> 2) & 0x3fff);

	mMax3003Sensor.fHR = 60 / ((float) mMax3003Sensor.ulRtor * 0.0098125);
	mMax3003Sensor.uiHR = (uint32_t) mMax3003Sensor.fHR;  // type cast to int
	mMax3003Sensor.uiRR = (uint32_t) mMax3003Sensor.ulRtor * (9.8125);
	//;
	/*
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
	 */
}

uint32_t max30003ReadRegister(uint8_t addrs) {
	uint32_t data = 0;
	uint8_t data_buffer[3];
	HAL_GPIO_WritePin(MAX3003_CS_GPIO_Port, MAX3003_CS_GPIO_PORT_Pin,
			GPIO_PIN_RESET);
	addrs = (addrs << 1) | 1;
	HAL_SPI_Transmit(&max3003Spi, &addrs, 1, 300);
	HAL_SPI_Receive(&max3003Spi, data_buffer, 3, 300);
	data |= (data_buffer[0] << 16);
	data |= (data_buffer[1] << 8);
	data |= data_buffer[2];
	HAL_GPIO_WritePin(MAX3003_CS_GPIO_Port, MAX3003_CS_GPIO_PORT_Pin,
			GPIO_PIN_SET);
	return data;
}

void max30003WriteRegister(const enum Registers_e reg, const uint32_t data) {
	// now combine the register address and the command into one byte:
	uint8_t ucDataToSend = (reg << 1);
	uint8_t ucaData[4];

	// take the chip select low to select the device:

	HAL_GPIO_WritePin(MAX3003_CS_GPIO_Port, MAX3003_CS_GPIO_PORT_Pin,
			GPIO_PIN_RESET);
	HAL_Delay(2);
	ucaData[0] = ucDataToSend;
	ucaData[1] = ((0x00FF0000 & data) >> 16);
	ucaData[2] = ((0x0000FF00 & data) >> 8);
	ucaData[3] = (0x000000FF & data);
	HAL_SPI_Transmit(&max3003Spi, ucaData, 4, 300);
	HAL_Delay(2);
	HAL_GPIO_WritePin(MAX3003_CS_GPIO_Port, MAX3003_CS_GPIO_PORT_Pin,
			GPIO_PIN_SET);
}

TypeDefGeneralConfiguration CNFG_GEN_r;
TypeDefECGConfiguration CNFG_ECG_r;
TypeDefRtoR1Configuration CNFG_RTOR_r;
TypeDefManageInterrupts MNG_INT_r;
TypeDefEnableInterrupts EN_INT_r;
TypeDefManageDynamicModes MNG_DYN_r;
TypeDefMuxConfiguration CNFG_MUX_r;


uint16_t usaSquareWave[]= { 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff,
							0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff,
							0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
							0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
							0xfff, 0xfff, 0xfff, 0xfff,
							0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff,
							0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff,
							0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff,
							0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff, 0xfff,
							0x0, 0x0, 0x0, 0x0,
							0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
							0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
							0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
							0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0
};
int d=0;
uint16_t usaSinWave[]= {
			0x7ff, 0x86a, 0x8d5, 0x93f, 0x9a9, 0xa11, 0xa78, 0xadd, 0xb40, 0xba1,
		    0xbff, 0xc5a, 0xcb2, 0xd08, 0xd59, 0xda7, 0xdf1, 0xe36, 0xe77, 0xeb4,
		    0xeec, 0xf1f, 0xf4d, 0xf77, 0xf9a, 0xfb9, 0xfd2, 0xfe5, 0xff3, 0xffc,
		    0xfff, 0xffc, 0xff3, 0xfe5, 0xfd2, 0xfb9, 0xf9a, 0xf77, 0xf4d, 0xf1f,
		    0xeec, 0xeb4, 0xe77, 0xe36, 0xdf1, 0xda7, 0xd59, 0xd08, 0xcb2, 0xc5a,
		    0xbff, 0xba1, 0xb40, 0xadd, 0xa78, 0xa11, 0x9a9, 0x93f, 0x8d5, 0x86a,
		    0x7ff, 0x794, 0x729, 0x6bf, 0x655, 0x5ed, 0x586, 0x521, 0x4be, 0x45d,
		    0x3ff, 0x3a4, 0x34c, 0x2f6, 0x2a5, 0x257, 0x20d, 0x1c8, 0x187, 0x14a,
		    0x112, 0xdf, 0xb1, 0x87, 0x64, 0x45, 0x2c, 0x19, 0xb, 0x2,
		    0x0, 0x2, 0xb, 0x19, 0x2c, 0x45, 0x64, 0x87, 0xb1, 0xdf,
		    0x112, 0x14a, 0x187, 0x1c8, 0x20d, 0x257, 0x2a5, 0x2f6, 0x34c, 0x3a4,
		    0x3ff, 0x45d, 0x4be, 0x521, 0x586, 0x5ed, 0x655, 0x6bf, 0x729, 0x794
};


// Global Function Definitions
void vMax30003Init(void) {

	// Reset ECG to clear registers
	max30003WriteRegister(SW_RST, 0);

	// General config register setting

	CNFG_GEN_r.bits.en_ecg = 1;     // Enable ECG channel
	CNFG_GEN_r.bits.rbiasn = 1;     // Enable resistive bias on negative input
	CNFG_GEN_r.bits.rbiasp = 1;     // Enable resistive bias on positive input
	CNFG_GEN_r.bits.en_rbias = 1;   // Enable resistive bias
	CNFG_GEN_r.bits.imag = 2;       // Current magnitude = 10nA
	CNFG_GEN_r.bits.en_dcloff = 1;  // Enable DC lead-off detection
	max30003WriteRegister(CNFG_GEN, CNFG_GEN_r.all);

	// ECG Config register setting

	CNFG_ECG_r.bits.dlpf = 1;       // Digital LPF cutoff = 40Hz
	CNFG_ECG_r.bits.dhpf = 1;       // Digital HPF cutoff = 0.5Hz
	CNFG_ECG_r.bits.gain = 3;       // ECG gain = 160V/V
	CNFG_ECG_r.bits.rate = 2;       // Sample rate = 128 sps
	max30003WriteRegister(CNFG_ECG, CNFG_ECG_r.all);

	//R-to-R configuration
    CNFG_RTOR_r.bits.wndw = 0b0011;         // WNDW = 96ms
	CNFG_RTOR_r.bits.rgain = 0b1111;        // Auto-scale gain
	CNFG_RTOR_r.bits.pavg = 0b11;           // 16-average
	CNFG_RTOR_r.bits.ptsf = 0b0011;         // PTSF = 4/16
	CNFG_RTOR_r.bits.en_rtor = 1;           // Enable R-to-R detection
	max30003WriteRegister(CNFG_RTOR1, CNFG_RTOR_r.all);

	//Manage interrupts register setting

	MNG_INT_r.bits.efit = 3;          // Assert EINT w/ 4 unread samples
	MNG_INT_r.bits.clr_rrint = 1;        // Clear R-to-R on RTOR reg. read back
	max30003WriteRegister(MNGR_INT, MNG_INT_r.all);

	//Enable interrupts register setting

	EN_INT_r.all = 0;
	EN_INT_r.bits.en_eint = 1;              // Enable EINT interrupt
	EN_INT_r.bits.en_rrint = 1;             // Enable  R-to-R interrupt
	EN_INT_r.bits.intb_type = 3;         // Open-drain NMOS with internal pullup
	max30003WriteRegister(EN_INT, EN_INT_r.all);

	//Dyanmic modes config

	MNG_DYN_r.bits.fast = 0;                // Fast recovery mode disabled
	max30003WriteRegister(MNGR_DYN, MNG_DYN_r.all);

	// MUX Config

	CNFG_MUX_r.bits.openn = 0;          // Connect ECGN to AFE channel
	CNFG_MUX_r.bits.openp = 0;          // Connect ECGP to AFE channel
	max30003WriteRegister(CNFG_EMUX, CNFG_MUX_r.all);

	max30003WriteRegister(SYNCH  , 0);
}

void vMax30003ReadData(void) {
	uint8_t ucatmpData[3] = { 0, 0, 0 };

	// Constants
	const int EINT_STATUS_MASK = 1 << 23;
    const int RTOR_STATUS =  1 << 10;
    const int RTOR_REG_OFFSET = 10;
    const float RTOR_LSB_RES = 0.0078125f;
	const int FIFO_OVF_MASK = 0x7;
	const int FIFO_VALID_SAMPLE_MASK = 0x0;
	const int FIFO_FAST_SAMPLE_MASK = 0x1;
	const int ETAG_BITS_MASK = 0x7;


	if(uicounterOfReset > 150){
		ecgFIFOIntFlag=1;
		uicounterOfReset=0;
	}
	// Read back ECG samples from the FIFO
	if (ecgFIFOIntFlag) {
		//reset data
		ecgFIFOIntFlag = 0;
		//uicounterOfReset = 0;
		status = max30003ReadRegister(STATUS);      // Read the STATUS register
		// Check if EINT interrupt asserted

		if ((status & RTOR_STATUS) == RTOR_STATUS) {
			uint32_t tempRtor=0;
			tempRtor = max30003ReadRegister(RTOR )>>  RTOR_REG_OFFSET;
			//mMax3003Sensor.ulRtor = (RTOR_msb << 8 | RTOR_lsb);
			//mMax3003Sensor.ulRtor = ((mMax3003Sensor.ulRtor >> 2) & 0x3fff);
			mMax3003Sensor.uiRR = tempRtor;
		}

		if ((status & EINT_STATUS_MASK) == EINT_STATUS_MASK) {
			readECGSamples = 0;                        // Reset sample counter
			memset(ecgSample, 0, 32);
			do {
				ecgFIFO = max30003ReadRegister(ECG_FIFO);       // Read FIFO
				ecgSample[readECGSamples] = ecgFIFO >> 8; // Isolate voltage data
				ETAG[readECGSamples] = (ecgFIFO >> 3) & ETAG_BITS_MASK; // Isolate ETAG
				readECGSamples++;                    // Increment sample counter
				// Check that sample is not last sample in FIFO
			} while (ETAG[readECGSamples - 1] == FIFO_VALID_SAMPLE_MASK
					|| ETAG[readECGSamples - 1] == FIFO_FAST_SAMPLE_MASK);
			// Check if FIFO has overflowed
			if (ETAG[readECGSamples - 1] == FIFO_OVF_MASK) {
				max30003WriteRegister(FIFO_RST,0); // Reset FIFO
			}
			// Print results
			for (idx = 0; idx < readECGSamples; idx++) {
				//mMax3003Sensor.usaDataPacketHeader[idx]= usaSinWave[idx + d*15];
				mMax3003Sensor.usaDataPacketHeader[idx]=ecgSample[idx];
			}
			//d++;
			d= d%3;
		}
	}else{
		uicounterOfReset++;
	}
}

void vMax30003ReadDataTemp(void) {
	uint8_t ucatmpData[3] = { 0, 0, 0 };
	// Constants
	const int EINT_STATUS_MASK = 1 << 23;
	const int FIFO_OVF_MASK = 0x7;
	const int FIFO_VALID_SAMPLE_MASK = 0x0;
	const int FIFO_FAST_SAMPLE_MASK = 0x1;
	const int ETAG_BITS_MASK = 0x7;
	// Read back ECG samples from the FIFO
	if (ecgFIFOIntFlag) {
		//reset data
		for (idx = 0; idx < 50; idx++)
			mMax3003Sensor.usaDataPacketHeader[idx];
		ecgFIFOIntFlag = 0;
		status = max30003ReadRegister(0x01);      // Read the STATUS register
		// Check if EINT interrupt asserted
		if ((status & EINT_STATUS_MASK) == EINT_STATUS_MASK) {
			readECGSamples = 0;                        // Reset sample counter
			memset(ecgSample, 0, 32);
			do {
				ecgFIFO = max30003ReadRegister(0x21);       // Read FIFO
				ecgSample[readECGSamples] = ecgFIFO >> 8; // Isolate voltage data
				ETAG[readECGSamples] = (ecgFIFO >> 3) & ETAG_BITS_MASK; // Isolate ETAG
				readECGSamples++;                    // Increment sample counter
				// Check that sample is not last sample in FIFO
			} while (ETAG[readECGSamples - 1] == FIFO_VALID_SAMPLE_MASK
					|| ETAG[readECGSamples - 1] == FIFO_FAST_SAMPLE_MASK);
			// Check if FIFO has overflowed
			if (ETAG[readECGSamples - 1] == FIFO_OVF_MASK) {
				max30003ReadRegister(0x0A); // Reset FIFO
			}
			// Print results
			for (idx = 0; idx < readECGSamples; idx++) {
				mMax3003Sensor.usaDataPacketHeader[idx]=ecgSample[idx];
			}

		}
		memset(ucatmpData, 0, 3);
		vMax30003RegRead(RTOR, ucatmpData);
		uint64_t RTOR_msb = (uint64_t) (ucatmpData[0]);
		// RTOR_msb = RTOR_msb <<8;
		uint8_t RTOR_lsb = (uint8_t) (ucatmpData[1]);
		mMax3003Sensor.ulRtor = (RTOR_msb << 8 | RTOR_lsb);
		mMax3003Sensor.ulRtor = ((mMax3003Sensor.ulRtor >> 2) & 0x3fff);
		mMax3003Sensor.uiRR = (uint32_t) mMax3003Sensor.ulRtor * (9.8125);
	}
}


//
unsigned int uiGetMax3003ECG() {
	return mMax3003Sensor.lEcgData;
}
unsigned int uiGetMax3003RR() {
	return mMax3003Sensor.uiRR;
}
unsigned int uiGetMax3003HR() {
	return mMax3003Sensor.uiHR;
}

