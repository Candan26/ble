/*
 * max30102.c
 *
 *  Created on: 25 Tem 2020
 *      Author: candan
 */

#include "max30102.h"
#include "heartRate.h"

//local function prototypes

#define max1002I2c hi2c3
#define max30102uart huart1
#define PULSE_COUNTER_UP_LIMIT 300

typedef_max30102 mMax30102Sensor;
// local variables
int tick = 0;
long lastBeat = 0; //Time at which the last beat occurred
long delta = 0;

// local functions
void static vsSetLEDs(pulseWidth pw, ledCurrent red, ledCurrent ir); // Sets the LED state
void static vsSetSPO2(sampleRate sr, high_resolution hi_res); // Setup the SPO2 sensor, disabled by default
void static vsGetAvarageBeat(); // get avarage beat based on hr values;
void static vsReadSensor(void);       // Updates the values
void static vsShutdown(void);   // Instructs device to power-save
void static vsReset(void);      // Resets the device
void static vsStartup(void);    // Leaves power-save
char static csGetRevID(void);   // Gets revision ID
char static csGetPartID(void);  // Gets part ID
int static isGetNumSamp(void);       // Get number of samples
uint32_t static uisMillis();
// Longest pulseWidth , Highest current, 2nd lowest sampleRate pw = pw411
void static vsBegin(pulseWidth pw, ledCurrent ir, sampleRate sr);

void static vsInit(pulseWidth pw, sampleRate sr, high_resolution hi_res,
		ledCurrent red, ledCurrent ir);
void static vsSetTemp(void);
int static isReadTemp(void);
void static vsSetSPO2mode(void);
void static vsSetInterruptSPO2(void);
void static vsPrintRegisters(void); // Dumps contents of registers for debug

void i2c_read(uint8_t address, uint8_t config_data, uint8_t *data,
		uint32_t size) {
	HAL_I2C_Master_Transmit(&max1002I2c, address, &config_data, 1, 300);
	HAL_I2C_Master_Receive(&max1002I2c, address, data, size, 300);
}

void i2c_write(uint8_t address, uint8_t config_data, uint8_t *data,
		uint32_t size) {
	uint8_t ucaBuffer[2];
	ucaBuffer[0] = config_data;
	ucaBuffer[1] = data[0];
	HAL_I2C_Master_Transmit(&max1002I2c, address, ucaBuffer, 2, 300);
}
// Local Function Definitions

// Sets the LED state
void static vsSetLEDs(pulseWidth pw, ledCurrent red, ledCurrent ir) {
	uint8_t data_read[1] = { 0 };
	uint8_t data_write[1] = { 0 };
	i2c_read(MAX30102_ADDRESS, MAX30102_SPO2_CONFIG, data_read, 1);
	data_read[0] = data_read[0] & 0xFD;                      // Set LED_PW to 01
	data_write[0] = data_read[0] | pw;
	i2c_write(MAX30102_ADDRESS, MAX30102_SPO2_CONFIG, data_write, 1); // Mask LED_PW
	data_write[0] = (red);
	i2c_write(MAX30102_ADDRESS, MAX30102_LED_CONFIG_1, data_write, 1); // write LED1 configs
	data_write[0] = (ir);
	i2c_write(MAX30102_ADDRESS, MAX30102_LED_CONFIG_2, data_write, 1); // write LED2 configs
}

void static vsSetSPO2(sampleRate sr, high_resolution hi_res) {
	uint8_t data_read[1] = { 0 };
	uint8_t data_write[1] = { 0 };
	i2c_read(MAX30102_ADDRESS, MAX30102_SPO2_CONFIG, data_read, 1);
	data_read[0] = data_read[0] & 0x85; // Set ADC_rge to 00, SPO2_SR to 001 = sr100 and LEDpw to 01 = 118
	data_write[0] = data_read[0] | (sr << 2) | (hi_res << 6);
	i2c_write(MAX30102_ADDRESS, MAX30102_SPO2_CONFIG, data_write, 1); // Mask SPO2_SR
	i2c_read(MAX30102_ADDRESS, MAX30102_CONFIG, data_read, 1);
	data_write[0] = data_read[0] & 0xF8; // Set Mode to 000
	i2c_write(MAX30102_ADDRESS, MAX30102_CONFIG, data_write, 1);    // Mask MODE
}

void static vsSetInterruptSPO2(void) {
	uint8_t data_read[1] = { 0 };
	uint8_t data_write[1] = { 0 };
	i2c_read(MAX30102_ADDRESS, MAX30102_INT_ENABLE, data_read, 1);
	data_write[0] = data_read[0] & 0x00; // Set Interrupt enable for SPO2 | 0x10   // New: disable prox! & ~0x10
	i2c_write(MAX30102_ADDRESS, MAX30102_INT_ENABLE, data_write, 1); // Mask ENB_SPO2_RDY
}

int static isGetNumSamp(void) {
	uint8_t data_read[1] = { 0 };
	i2c_read(MAX30102_ADDRESS, MAX30102_FIFO_W_POINTER, data_read, 1);
	char wrPtr = data_read[0];
	i2c_read(MAX30102_ADDRESS, MAX30102_FIFO_R_POINTER, data_read, 1);
	char rdPtr = data_read[0];
	return ((int) wrPtr - (int) rdPtr);                          // New counting
}

void static vsSetTemp(void) {
	uint8_t data_read[1] = { 0 };
	uint8_t data_write[1] = { 0 };
	i2c_read(MAX30102_ADDRESS, MAX30102_TEMP_CONFIG, data_read, 1);
	data_write[0] = data_read[0] | 0x01;    // Enable temperature
	i2c_write(MAX30102_ADDRESS, MAX30102_TEMP_CONFIG, data_write, 1); // Mask MODE
	i2c_read(MAX30102_ADDRESS, MAX30102_TEMP_CONFIG, data_read, 1);
}

void static vsSetSPO2mode(void) {
	uint8_t data_read[1] = { 0 };
	uint8_t data_write[1] = { 0 };
	i2c_read(MAX30102_ADDRESS, MAX30102_CONFIG, data_read, 1);
	data_write[0] = data_read[0] | 0x03;    // Set SPO2 Mode
	i2c_write(MAX30102_ADDRESS, MAX30102_CONFIG, data_write, 1);
}

int static isReadTemp(void) {
	uint8_t data_read[1] = { 0 };
	uint8_t temp_int, temp_fract;
	int32_t temp_measured;
	i2c_read(MAX30102_ADDRESS, MAX30102_TEMP_INTEGER, data_read, 1);
	temp_int = data_read[0];
	i2c_read(MAX30102_ADDRESS, MAX30102_TEMP_FRACTION, data_read, 1);
	temp_fract = data_read[0] & 0x0F;
	temp_measured = ((int) temp_int) + (((int) temp_fract) >> 4);
	return temp_measured;
}

void static vsReadSensor(void) {
	uint8_t data_read[6] = { 0 };
	mMax30102Sensor.uiHR = 0;
	mMax30102Sensor.uiSPO2 = 0;
	i2c_read(MAX30102_ADDRESS, MAX30102_FIFO_DATA_REG, data_read, 6); // Read six times from the FIFO
	mMax30102Sensor.uiHR = (data_read[0] << 16) | (data_read[1] << 8)
			| data_read[2];         // Combine values to get the actual number
	mMax30102Sensor.uiHR = mMax30102Sensor.uiHR >> 2;
	mMax30102Sensor.uiSPO2 = (data_read[3] << 16) | (data_read[4] << 8)
			| data_read[5];       // Combine values to get the actual number
	mMax30102Sensor.uiSPO2 = mMax30102Sensor.uiSPO2 >> 2;
}

void static vsShutdown(void) {
	uint8_t data_read[1] = { 0 };
	uint8_t data_write[1] = { 0 };
	i2c_read(MAX30102_ADDRESS, MAX30102_CONFIG, data_read, 1); // Get the current register
	data_write[0] = data_read[0] | 0x80;
	i2c_write(MAX30102_ADDRESS, MAX30102_CONFIG, data_write, 1); // mask the SHDN bit
}

void static vsReset(void) {
	uint8_t data_read[1] = { 0 };
	uint8_t data_write[1] = { 0 };
	i2c_read(MAX30102_ADDRESS, MAX30102_CONFIG, data_read, 1); // Get the current register
	data_write[0] = data_read[0] | 0x40;
	i2c_write(MAX30102_ADDRESS, MAX30102_CONFIG, data_write, 1); // mask the RESET bit
}

void static vsStartup(void) {
	uint8_t data_read[1] = { 0 };
	uint8_t data_write[1] = { 0 };
	i2c_read(MAX30102_ADDRESS, MAX30102_CONFIG, data_read, 1); // Get the current register
	data_write[0] = data_read[0] & 0x7F;
	i2c_write(MAX30102_ADDRESS, MAX30102_CONFIG, data_write, 1); // mask the SHDN bit
}

char static csGetRevID(void) {
	uint8_t data_read[1] = { 0 };
	i2c_read(MAX30102_ADDRESS, MAX30102_REVISION_ID, data_read, 1);
	return data_read[0];
}

char static csGetPartID(void) {
	uint8_t data_read[1] = { 0 };
	i2c_read(MAX30102_ADDRESS, MAX30102_PART_ID, data_read, 1);
	return data_read[0];
}
// pw = pw411 ir = i50 sr = sr100
void static vsBegin(pulseWidth pw, ledCurrent ir, sampleRate sr) {
	uint8_t data_write[1] = { 0 };
	data_write[0] = 0x02;
	i2c_write(MAX30102_ADDRESS, MAX30102_CONFIG, data_write, 1); // Heart rate only
	data_write[0] = ir;
	i2c_write(MAX30102_ADDRESS, MAX30102_LED_CONFIG_1, data_write, 1);
	data_write[0] = ((sr << 2) | pw);
	i2c_write(MAX30102_ADDRESS, MAX30102_SPO2_CONFIG, data_write, 1);
}

void static vsInit(pulseWidth pw, sampleRate sr, high_resolution hi_res,
		ledCurrent red, ledCurrent ir) {
	uint8_t data_write[1] = { 0 };
	vsSetLEDs(pw, red, ir);
	vsSetSPO2(sr, hi_res);
	data_write[0] = 0x10;
	i2c_write(MAX30102_ADDRESS, MAX30102_FIFO_CONFIG, data_write, 1);
}

void static vsPrintRegisters(void) {
	uint8_t data_read[1] = { 0 };
	i2c_read(MAX30102_ADDRESS, MAX30102_INT_STATUS, data_read, 1);
	mMax30102Sensor.ucaPrintRegister[INT_STATUS] = data_read[0];

	i2c_read(MAX30102_ADDRESS, MAX30102_INT_ENABLE, data_read, 1);
	mMax30102Sensor.ucaPrintRegister[INT_ENABLE] = data_read[0];

	i2c_read(MAX30102_ADDRESS, MAX30102_FIFO_W_POINTER, data_read, 1);
	mMax30102Sensor.ucaPrintRegister[FIFO_W_POINTER] = data_read[0];

	i2c_read(MAX30102_ADDRESS, MAX30102_OVR_COUNTER, data_read, 1);
	mMax30102Sensor.ucaPrintRegister[OVR_COUNTER] = data_read[0];

	i2c_read(MAX30102_ADDRESS, MAX30102_FIFO_R_POINTER, data_read, 1);
	mMax30102Sensor.ucaPrintRegister[FIFO_R_POINTER] = data_read[0];

	i2c_read(MAX30102_ADDRESS, MAX30102_FIFO_DATA_REG, data_read, 1);
	mMax30102Sensor.ucaPrintRegister[FIFO_DATA_REG] = data_read[0];

	i2c_read(MAX30102_ADDRESS, MAX30102_CONFIG, data_read, 1);
	mMax30102Sensor.ucaPrintRegister[CONFIG] = data_read[0];

	i2c_read(MAX30102_ADDRESS, MAX30102_SPO2_CONFIG, data_read, 1);
	mMax30102Sensor.ucaPrintRegister[SPO2_CONFIG] = data_read[0];

	i2c_read(MAX30102_ADDRESS, MAX30102_LED_CONFIG_2, data_read, 1);
	mMax30102Sensor.ucaPrintRegister[LED_CONFIG] = data_read[0];

	i2c_read(MAX30102_ADDRESS, MAX30102_TEMP_INTEGER, data_read, 1);
	mMax30102Sensor.ucaPrintRegister[TEMP_INTEGER] = data_read[0];

	i2c_read(MAX30102_ADDRESS, MAX30102_TEMP_FRACTION, data_read, 1);
	mMax30102Sensor.ucaPrintRegister[TEMP_FRACTION] = data_read[0];

	i2c_read(MAX30102_ADDRESS, MAX30102_TEMP_CONFIG, data_read, 1);
	mMax30102Sensor.ucaPrintRegister[TEMP_CONFIG] = data_read[0];

	i2c_read(MAX30102_ADDRESS, MAX30102_REVISION_ID, data_read, 1);
	mMax30102Sensor.ucaPrintRegister[REVISION_ID] = data_read[0];

	i2c_read(MAX30102_ADDRESS, MAX30102_PART_ID, data_read, 1);
	mMax30102Sensor.ucaPrintRegister[PART_ID] = data_read[0];
}

void static vsGetAvarageBeat(void) {
	if (ucCheckForBeat(mMax30102Sensor.uiHR) == TRUE) {
		delta = uisMillis() - lastBeat;
		lastBeat = uisMillis();
		mMax30102Sensor.uiCumulativePulseInterval =
				mMax30102Sensor.uiCumulativePulseInterval + delta;
		mMax30102Sensor.usPulseCounter++;
		if (mMax30102Sensor.usPulseCounter >= PULSE_COUNTER_UP_LIMIT) {
			mMax30102Sensor.usPulseCounter = 0;
			mMax30102Sensor.uiCumulativePulseInterval = 0;
		}
	}
}

uint32_t static uisMillis() {
	tick = HAL_GetTick();
	return tick;
}

// Global Function Definitions

void vMax30102Init(void) {
	vsInit(pw411, sr100, low, i34, i34);
	vsBegin(pw411, i34, sr100);
	vsPrintRegisters();
	vsStartup();
	mMax30102Sensor.cPartId = csGetPartID();
	mMax30102Sensor.cRevId = csGetRevID();
	mMax30102Sensor.iTemp = isReadTemp();
	mMax30102Sensor.iNumOfSample = isGetNumSamp();
}
void vMax30102ReadData(void) {
	vsReadSensor();
	if (mMax30102Sensor.uiHR > 7000) {
		vsGetAvarageBeat();
		mMax30102Sensor.ucPrintRegisterCounter++;
		if (mMax30102Sensor.ucPrintRegisterCounter >= 0xEE) {
			vsPrintRegisters();
			mMax30102Sensor.iNumOfSample = isGetNumSamp();
		}
	}
}

unsigned int uiGetHR() {
	return mMax30102Sensor.uiHR;
}

unsigned int uiGetSPO2() {
	return mMax30102Sensor.uiSPO2;
}

unsigned int uiGetCumPulse() {
	return mMax30102Sensor.uiCumulativePulseInterval;
}

unsigned int uiGetPulseCounter() {
	return mMax30102Sensor.usPulseCounter;
}

