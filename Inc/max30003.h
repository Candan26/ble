/*
 * max30003.h
 *
 *  Created on: 25 Tem 2020
 *      Author: candan
 */

#ifndef MAX30003_H_
#define MAX30003_H_

#include "main.h"

#define WREG 0x00
#define RREG 0x01

#define   NO_OP           0x00
#define   STATUS          0x01
#define   EN_INT          0x02
#define   EN_INT2         0x03
#define   MNGR_INT        0x04
#define   MNGR_DYN        0x05
#define   SW_RST          0x08
#define   SYNCH           0x09
#define   FIFO_RST        0x0A
#define   INFO            0x0F
#define   CNFG_GEN        0x10
#define   CNFG_CAL        0x12
#define   CNFG_EMUX       0x14
#define   CNFG_ECG        0x15
#define   CNFG_RTOR1      0x1D
#define   CNFG_RTOR2      0x1E
#define   ECG_FIFO_BURST  0x20
#define   ECG_FIFO        0x21
#define   RTOR            0x25
//#define   NO_OP           0x7F

typedef struct {
	//definitions
	volatile float fHR;
	uint64_t ulRtor;
	uint64_t ulData;
	int64_t lEcgData;
	uint32_t uiHR;
	uint32_t uiRR;
	uint8_t ucaDataPacketHeader[20];
} typedef_max3003;

extern typedef_max3003 mMax3003Sensor;

void vMax30003Init(void);
void vMax30003ReadData(void);

unsigned int uiGetMax3003ECG();
unsigned int uiGetMax3003RR();
#endif /* MAX30003_H_ */
