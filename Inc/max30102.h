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
#define MAX30102_ADDR_WRITE 0xae
#define MAX30102_ADDR_READ 0xaf

#define RES_INTERRUPT_STATUS_1 0x00
#define RES_INTERRUPT_STATUS_2 0x01
#define RES_INTERRUPT_ENABLE_1 0x02
#define RES_INTERRUPT_ENABLE_2 0x03
#define RES_FIFO_WRITE_POINTER 0x04
#define RES_OVERFLOW_COUNTER 0x05
#define RES_FIFO_READ_POINTER 0x06
#define RES_FIFO_DATA_REGISTER 0x07
#define RES_FIFO_CONFIGURATION 0x08
#define RES_MODE_CONFIGURATION 0x09
#define RES_SPO2_CONFIGURATION 0x0a
#define RES_LED_PLUSE_AMPLITUDE_1 0x0c
#define RES_LED_PLUSE_AMPLITUDE_2 0x0d
#define RES_PROXIMITY_MODE_LED_PLUSE_AMPLITUDE 0x10
#define RES_MULTI_LED_MODE_CONTROL_1 0x11
#define RES_MULTI_LED_MODE_CONTROL_2 0x12
#define RES_DIE_TEMP_INTEGER 0x1f
#define RES_DIE_TEMP_FRACTION 0x20
#define RES_DIE_TEMPERATURE_CONFIG 0x21
#define RES_PROXIMITY_INTERRUPT_THRESHOLD 0x30
#define RES_REVISION_ID 0xfe
#define RES_PART_ID 0xff

typedef struct samplestruct{
    uint32_t red;
    uint32_t iRed;
} SAMPLE;

typedef struct {
	//definitions
	volatile uint8_t ucHR;
	volatile uint8_t ucSPO2;
	volatile int16_t usDiff;
	volatile int32_t iNumOfSample;
	volatile uint32_t uiRed;
	volatile uint32_t uiIRed;
	volatile uint16_t usPulseCounter;
} typedef_max30102;

extern volatile typedef_max30102 mMax30102Sensor;

void vMax30102Init(void);
void vMax30102ReadData(void);
void vMax30102Shutdown(void);   // Instructs device to power-save
void vMax30102StartUp(void);    // Leaves power-save


unsigned char ucGetMax30102HR();
unsigned char ucGetMax30102SPO2();
unsigned short usGetMax30102Diff();
unsigned int uiGetMax30102PulseCounter();
uint32_t uiGetMax30102Red();
uint32_t uiGetMax30102IRed();


#endif /* MAX30102_H_ */
