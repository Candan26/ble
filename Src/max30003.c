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
		uicounterOfReset = 0;
		status = max30003ReadRegister(STATUS);      // Read the STATUS register
		// Check if EINT interrupt asserted

		if ((status & RTOR_STATUS) == RTOR_STATUS) {
			uint32_t tempRtor=0;
			tempRtor = max30003ReadRegister(RTOR )>>  RTOR_REG_OFFSET;
			mMax3003Sensor.faBpm[0] = 1.0f / ( tempRtor * RTOR_LSB_RES / 60.0f );
			mMax3003Sensor.ucBpmCounter++;
			if(mMax3003Sensor.ucBpmCounter>=5){
				mMax3003Sensor.ucBpmCounter=0;
			}

			mMax3003Sensor.uiaRorVal[0] = tempRtor;
			mMax3003Sensor.ucRorCounter++;
			if(mMax3003Sensor.ucRorCounter>=5){
				mMax3003Sensor.ucRorCounter=0;
			}

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

				//mMax3003Sensor.usaDataPacketHeader[idx]= usaSinWave[idx + d*15];
				mMax3003Sensor.usaEcgVal[mMax3003Sensor.usEcgCounter]=ecgSample[idx];
				mMax3003Sensor.usEcgCounter++;
#ifdef DEBUG_ECG
				vPrintSensorData(ecgSample[idx]);
#endif

				if(mMax3003Sensor.usEcgCounter>=160){
					mMax3003Sensor.usEcgCounter=0;
				}

			}
		}
	else{
		uicounterOfReset++;
	}
}
//
unsigned int uiGetMax3003ECG() {
	return mMax3003Sensor.lEcgData;
}

