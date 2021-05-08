/* USER CODE BEGIN */
#ifndef __HAL_LCD_H
#define __HAL_LCD_H

#include "ssd1306.h"

typedef enum
{
    LCD_CHAR_SMALL,
    LCD_CHAR_MEDIUM,
    LCD_CHAR_BIG,
} LCD_CharSize_t;

void LCD_Init(void);
void LCD_PrintTest(uint16_t x,uint16_t y, char * msg, SSD1306_COLOR_t color, LCD_CharSize_t charSize);
void LCD_Print1stLine(char * msg);
void LCD_Print2ndLine(char * msg);
void LCD_Print(char* line1, char* line2);
void LCD_THREAD_PrintPanId(uint16_t panId);
void LCD_THREAD_PrintRole(char * role);
void LCD_THREAD_PrintRLOC(uint16_t rloc);
void LCD_PrintLabel(char * rloc);
void LCD_PrintError(uint32_t errId);
void LCD_BLE_PrintLocalName(const char * name);
void LCD_BLE_PrintStatus(char * status);
void LCD_BLE_PrintLogo(void);
void LCD_BLE_HRS_PrintBPM(uint8_t BPM);
void LCD_BLE_HTS_PrintTemperature(uint8_t temperature);
void LCD_BLE_TPS_PrintRSSI(uint8_t RSSI);

#endif /* __HAL_LCD_H */
/* USER CODE END */
