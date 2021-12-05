#ifndef __OLED_H
#define __OLED_H
#include "main.h"

// Macro definitions
#define OLED_ADDR 0x78
#define OLED_CMD 0  //???
#define OLED_DATA 1 //???
#define OLED_MODE 0
//typedef defs
typedef enum {
	font6x8,
	font8x8Hunter

}OledFonts;

//function prototypes
void vOledDisplayOff(void);
void vOledInit(void);
void vOledClear(void);
void vOledShowString(OledFonts font, uint8_t x, uint8_t y, uint8_t *str);
void vOledSetPos(uint8_t x, uint8_t y);
void vOledShowNum(uint8_t which, uint8_t num);
void vOledDrawChart(float value);
void vWriteToScreen(I2C_HandleTypeDef *hi2c);

//Ble related
void vOledBlePrintLux(uint32_t lux);
void vOledBlePrintMax30102(uint8_t hr, uint8_t spo2, uint16_t diff);
void vOledBlePrintTemperature(float temperature);
void vOledBlePrintHumidity(float humidity);
void vOledBlePrintSi7021(float temperature, float humidity);
void vOledBlePrintGSR(float gsr);
void vOledBlePrintData(void);
void vOledBleClearScreen(void);

// Sensor related

void vOledShowHeart(uint8_t showOrNot);
void vSetBPM(void);
void vSetPercentage(void);
void vSetSPO2(void);
void vMax30102String(void);
void vOledBleMaxInit30102(void);
void vOledBlePrintMax30003(uint32_t ecg, uint32_t hr, uint32_t rr);

#endif
