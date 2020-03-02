#ifndef __si7021__
#define __si7021__
#include "main.h"


#define ADDRESS_OF_SI7021 0x80
#define SEQ_NO_SEND 0
#define SEQ_NO_RECIEVE 1


#define SI7021_MEASRH_HOLD_CMD           0xE5 /**< Measure Relative Humidity, Hold Master Mode */
#define SI7021_MEASRH_NOHOLD_CMD         0xF5 /**< Measure Relative Humidity, No Hold Master Mode */
#define SI7021_MEASTEMP_HOLD_CMD         0xE3 /**< Measure Temperature, Hold Master Mode */
#define SI7021_MEASTEMP_NOHOLD_CMD       0xF3 /**< Measure Temperature, No Hold Master Mode */
#define SI7021_READPREVTEMP_CMD          0xE0 /**< Read Temperature Value from Previous RH Measurement */
#define SI7021_RESET_CMD                 0xFE /**< Reset Command */
#define SI7021_WRITERHT_REG_CMD          0xE6 /**< Write RH/T User Register 1 */
#define SI7021_READRHT_REG_CMD           0xE7 /**< Read RH/T User Register 1 */
#define SI7021_WRITEHEATER_REG_CMD       0x51 /**< Write Heater Control Register */
#define SI7021_READHEATER_REG_CMD        0x11 /**< Read Heater Control Register */
#define SI7021_ID1_CMD_MS                   0xFA
#define SI7021_ID1_CMD_LS 					0x0F /**< Read Electronic ID 1st Byte */
#define SI7021_ID2_CMD_MS                   0xFC
#define SI7021_ID2_CMD_LS				    0xC9 /**< Read Electronic ID 2nd Byte */
#define SI7021_FIRMVERS_CMD_MS              0x84
#define SI7021_FIRMVERS_CMD_LS 				0xB8 /**< Read Firmware Revision */

#define SI7021_REV_1					0xff  /**< Sensor revision 1 */
#define SI7021_REV_2					0x20  /**< Sensor revision 2 */

const static int TRANSACTION_TIMEOUT=100;

typedef struct {
	//definitions
	volatile float fHumidty;
	volatile float fTemperature;

	uint32_t uiSernum_a;
	uint32_t uiSernum_b;

	uint8_t ucFirmwareRevision;
	uint8_t ucI2cAddr;
	uint8_t ucChecksumHum;
	uint8_t ucChecksumTemp;
	uint8_t ucRegisterVal;

	uint8_t ucSeqNumber;
	uint8_t ucTempOrHumidty;

}typedef_si7021;

//function prototypes
//global functions
unsigned char vInitsi7021();
void vSi7021ProcessHumidityAndTemperature();


extern typedef_si7021 mSi7021Sensor;
#endif

