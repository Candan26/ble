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

/*
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
 #define   NO_OP           0x7F
*/

typedef enum Registers_e {
	NO_OP = 0x00,
	STATUS = 0x01,
	EN_INT = 0x02,
	EN_INT2 = 0x03,
	MNGR_INT = 0x04,
	MNGR_DYN = 0x05,
	SW_RST = 0x08,
	SYNCH = 0x09,
	FIFO_RST = 0x0A,
	INFO = 0x0F,
	CNFG_GEN = 0x10,
	CNFG_ALL = 0x12,
	CNFG_EMUX = 0x14,
	CNFG_ECG = 0x15,
	CNFG_RTOR1 = 0x1D,
	CNFG_RTOR2 = 0x1E,
	ECG_FIFO_BURST = 0x20,
	ECG_FIFO = 0x21,
	RTOR = 0x25,
	NO_OP2 = 0x7F
} TypeDefRegister;

typedef union Status_u {
///Access all bits
	uint32_t all;

///Access individual bits
	struct BitField_s {
		uint32_t loff_nl :1;
		uint32_t loff_nh :1;
		uint32_t loff_pl :1;
		uint32_t loff_ph :1;
		uint32_t reserved1 :4;
		uint32_t pllint :1;
		uint32_t samp :1;
		uint32_t rrint :1;
		uint32_t lonint :1;
		uint32_t reserved2 :8;
		uint32_t dcloffint :1;
		uint32_t fstint :1;
		uint32_t eovf :1;
		uint32_t eint :1;
		uint32_t reserved3 :8;
	} bits;
} TypeDefstatus;

typedef union EnableInterrupts_u {
///Access all bits
	uint32_t all;

///Access individual bits
	struct BitField_s1 {
		uint32_t intb_type :2;
		uint32_t reserved1 :6;
		uint32_t en_pllint :1;
		uint32_t en_samp :1;
		uint32_t en_rrint :1;
		uint32_t en_loint :1;
		uint32_t reserved2 :8;
		uint32_t en_dcloffint :1;
		uint32_t en_fstint :1;
		uint32_t en_eovf :1;
		uint32_t en_eint :1;
		uint32_t reserved3 :8;
	} bits;
} TypeDefEnableInterrupts;

///Manage Interrupt register bits
typedef union ManageInterrupts_u {
///Access all bits
	uint32_t all;

///Access individual bits
	struct BitField_s2 {
		uint32_t samp_it :4;
		uint32_t clr_samp :1;
		uint32_t reserved1 :1;
		uint32_t clr_rrint :2;
		uint32_t clr_fast :1;
		uint32_t reserved2 :12;
		uint32_t efit :5;
		uint32_t reserved3 :8;
	} bits;
} TypeDefManageInterrupts;

///Manage Dynamic Modes register bits
typedef union ManageDynamicModes_u {
///Access all bits
	uint32_t all;

///Access individual bits
	struct BitField_s3 {
		uint32_t reserved1 :16;
		uint32_t fast_th :6;
		uint32_t fast :2;
		uint32_t reserved2 :8;
	} bits;
} TypeDefManageDynamicModes;

///General Configuration bits
typedef union GeneralConfiguration_u {
///Access all bits
	uint32_t all;

///Access individual bits
	struct BitField_s4 {
		uint32_t rbiasn :1;
		uint32_t rbiasp :1;
		uint32_t rbiasv :2;
		uint32_t en_rbias :2;
		uint32_t vth :2;
		uint32_t imag :3;
		uint32_t ipol :1;
		uint32_t en_dcloff :2;
		uint32_t reserved1 :5;
		uint32_t en_ecg :1;
		uint32_t fmstr :2;
		uint32_t en_ulp_lon :2;
		uint32_t reserved2 :8;
	} bits;
} TypeDefGeneralConfiguration;

///Cal Configuration bits
typedef union CalConfiguration_u {
///Access all bits
	uint32_t all;

///Access individual bits
	struct BitField_s5 {
		uint32_t thigh :11;
		uint32_t fifty :1;
		uint32_t fcal :3;
		uint32_t reserved1 :5;
		uint32_t vmag :1;
		uint32_t vmode :1;
		uint32_t en_vcal :1;
		uint32_t reserved2 :9;

	} bits;
} TypeDefCalConfiguration;

///Mux Configuration bits
typedef union MuxConfiguration_u {
///Access all bits
	uint32_t all;

///Access individual bits
	struct BitField_s6 {
		uint32_t reserved1 :16;
		uint32_t caln_sel :2;
		uint32_t calp_sel :2;
		uint32_t openn :1;
		uint32_t openp :1;
		uint32_t reserved2 :1;
		uint32_t pol :1;
		uint32_t reserved3 :8;
	} bits;
} TypeDefMuxConfiguration;

///ECG Configuration bits
typedef union ECGConfiguration_u {
///Access all bits
	uint32_t all;

///Access individual bits
	struct BitField_s7 {
		uint32_t reserved1 :12;
		uint32_t dlpf :2;
		uint32_t dhpf :1;
		uint32_t reserved2 :1;
		uint32_t gain :2;
		uint32_t reserved3 :4;
		uint32_t rate :2;
		uint32_t reserved4 :8;
	} bits;
} TypeDefECGConfiguration;

///RtoR1 Configuration bits
typedef union RtoR1Configuration_u {
///Access all bits
	uint32_t all;

///Access individual bits
	struct BitField_s8 {
		uint32_t reserved1 :8;
		uint32_t ptsf :4;
		uint32_t pavg :2;
		uint32_t reserved2 :1;
		uint32_t en_rtor :1;
		uint32_t rgain :4;
		uint32_t wndw :4;
		uint32_t reserved3 :8;
	} bits;
} TypeDefRtoR1Configuration;

///RtoR2 Configuration bits
typedef union RtoR2Configuration_u {
///Access all bits
	uint32_t all;

///Access individual bits
	struct BitField_s9 {
		uint32_t reserved1 :8;
		uint32_t rhsf :3;
		uint32_t reserved2 :1;
		uint32_t ravg :2;
		uint32_t reserved3 :2;
		uint32_t hoff :6;
		uint32_t reserved4 :10;
	} bits;
} TypeDefRtoR2Configuration;

typedef struct {
//definitions
	volatile float fHR;
	volatile uint64_t ulRtor;
	volatile uint64_t ulData;
	volatile int64_t lEcgData;
	volatile uint16_t usaEcgVal[160];
	volatile uint16_t usEcgCounter;
	volatile float faBpm[5];
	volatile uint8_t ucBpmCounter;
	volatile uint32_t uiaRorVal[5];
	volatile uint8_t ucRorCounter;
} typedef_max3003;

extern volatile typedef_max3003 mMax3003Sensor;

void vMax30003Init(void);
void vMax30003ReadData(void);
void vMax30003SoftwareReset();

unsigned int uiGetMax3003ECG();
unsigned int uiGetMax3003RR();
unsigned int uiGetMax3003HR();
#endif /* MAX30003_H_ */
