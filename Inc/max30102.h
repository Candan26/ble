/*
 * max30102.h
 *
 *  Created on: 25 Tem 2020
 *      Author: candan
 */

#ifndef MAX30102_H_
#define MAX30102_H_
#include "main.h"

/******************************************************************************/
/*********** PULSE OXIMETER AND HEART RATE REGISTER MAPPING  **************/
/******************************************************************************/

// status registers
#define MAX30102_INT_STATUS          0x00
#define MAX30102_INT_ENABLE          0x02

// FIFO registers
#define MAX30102_FIFO_W_POINTER      0x04
#define MAX30102_OVR_COUNTER         0x05
#define MAX30102_FIFO_R_POINTER      0x06
#define MAX30102_FIFO_DATA_REG       0x07

// configuration registers
#define MAX30102_FIFO_CONFIG         0x08
#define MAX30102_CONFIG              0x09
#define MAX30102_SPO2_CONFIG         0x0A
#define MAX30102_LED_CONFIG_1        0x0C
#define MAX30102_LED_CONFIG_2        0x0D

// temperature registers
#define MAX30102_TEMP_INTEGER        0x1F
#define MAX30102_TEMP_FRACTION       0x20
#define MAX30102_TEMP_CONFIG         0x21

// PART ID registers
#define MAX30102_REVISION_ID         0xFE
#define MAX30102_PART_ID             0xFF

#define I_AM_MAX30102                0x15

/************************************** REGISTERS VALUE *******************************************/

// I2C address
#define MAX30102_ADDRESS             0xAE

//Enable interrupts
#define MAX30102_INT_ENB_A_FULL      ((uint8_t)0x80)
#define MAX30102_INT_ENB_TEMP_RDY    ((uint8_t)0x40)
#define MAX30102_INT_ENB_HR_RDY      ((uint8_t)0x20)
#define MAX30102_INT_ENB_SO2_RDY     ((uint8_t)0x10)

//Mode configuration
#define MAX30102_MODE_SHDN           ((uint8_t)0x80)      // Bit 7 high
#define MAX30102_MODE_RESET          ((uint8_t)0x40)      // Bit 6 high
#define MAX30102_MODE_TEMP_EN        ((uint8_t)0x01)
#define MAX30102_MODE_HR             ((uint8_t)0x02)
#define MAX30102_MODE_SPO2           ((uint8_t)0x03)

//SPO2 configuration
#define MAX30102_SPO2_HI_RES_EN           ((uint8_t)0x40)

typedef enum { // This is the same for both LEDs
	pw68,     // 68us pulse, ADC 15
	pw118,    // 118us pulse, ADC 16
	pw215,    // 215us pulse, ADC 17
	pw411     // 411us pulse, ADC 18
} pulseWidth;

typedef enum {
	sr50,    // 50 samples per second
	sr100,   // 100 samples per second
	sr200,   // 200 samples per second
	sr400,   // 400 samples per second
	sr800,   // 800 samples per second
	sr1000   // 1000 samples per second
} sampleRate;

typedef enum {
	i0,    // No current
	i4,    // 4.4mA
	i8,    // 7.6mA
	i11,   // 11.0mA
	i14,   // 14.2mA
	i17,   // 17.4mA
	i21,   // 20.8mA
	i27,   // 27.1mA
	i31,   // 30.6mA
	i34,   // 33.8mA
	i37,   // 37.0mA
	i40,   // 40.2mA
	i44,   // 43.6mA
	i47,   // 46.8mA
	i50    // 50.0mA
} ledCurrent;

typedef enum {
	low,    // low resolution SPO2
	high    // high resolution SPO2 (18 bit with 411us LED pulse width)
} high_resolution;

typedef enum {
	OXIMETER_OK = 0,
	OXIMETER_ERROR = 1,
	OXIMETER_TIMEOUT = 2,
	OXIMETER_NOT_IMPLEMENTED = 3
} OXIMETER_StatusTypeDef;

/**
 * @brief  MAX30102 driver extended internal structure definition
 */
typedef struct {
	OXIMETER_StatusTypeDef (*Enable_Free_Fall_Detection)(void);
	OXIMETER_StatusTypeDef (*Disable_Free_Fall_Detection)(void);
	OXIMETER_StatusTypeDef (*Get_Status_Free_Fall_Detection)(uint8_t *);
} MAX30102_DrvExtTypeDef;

typedef enum {
	INT_STATUS,
	INT_ENABLE,
	FIFO_W_POINTER,
	OVR_COUNTER,
	FIFO_R_POINTER,
	FIFO_DATA_REG,
	CONFIG,
	SPO2_CONFIG,
	LED_CONFIG,
	TEMP_INTEGER,
	TEMP_FRACTION,
	TEMP_CONFIG,
	REVISION_ID,
	PART_ID
} PrintRegisterStatus_TypeDef;

typedef struct {
	//definitions
	uint32_t uiHR;
	uint32_t uiSPO2;
	uint32_t uiCumulativePulseInterval;
	int32_t iTemp;
	int32_t iNumOfSample;
	uint16_t usPulseCounter;
	uint8_t ucPrintRegisterCounter;
	uint8_t ucaPrintRegister[15];
	char cRevId;
	char cPartId;
} typedef_max30102;

extern typedef_max30102 mMax30102Sensor;

void vMax30102Init(void);
void vMax30102ReadData(void);
#endif /* MAX30102_H_ */
