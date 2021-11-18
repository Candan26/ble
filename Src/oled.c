#include "oled.h"
#include "oledfont.h"
#include "main.h"
#include "ssd1306.h"

#include <stdio.h>
#include <string.h>

#define SIZE 16
#define XLevelL 0x02
#define XLevelH 0x10
#define Max_Column 128
#define Max_Row 64
#define Brightness 0xFF
#define X_WIDTH 128
#define Y_WIDTH 64

#define OLED_NUM_A 0x1
#define OLED_NUM_B 0x2
#define OLED_NUM_C 0x4
#define OLED_NUM_D 0x8
#define OLED_NUM_E 0x10
#define OLED_NUM_F 0x20
#define OLED_NUM_G 0x40

#define oledI2c	hi2c3

unsigned short usBpmPoslastX;
unsigned short usBpmPoslastY;
unsigned short usBpmPosX;
unsigned short usBpmPosY;

int postorier = 15;
int percentageOfReduction = 5;
uint32_t maxRangeOfGSR = 512;

uint8_t point_x = 0;
uint8_t point_y = 0;
uint8_t whatToDo = 0;
uint8_t oled_cache[8][128];
uint8_t pos_x_this = 0;

#define LCD_BUFFER_LENGHT 64
static char tempLcdBuffer[LCD_BUFFER_LENGHT];
static uint8_t i;

const uint8_t bpm[] = { 0x3E, 0x28, 0x38, 0x00, 0x3E, 0x0A, 0x0E, 0x00, 0x3E,
		0x02, 0x1C, 0x02, 0x3E };
const uint8_t spo2[] = { 0x38, 0x44, 0x44, 0x44, 0x88, 0x00, 0xFC, 0x44, 0x44,
		0x44, 0x38, 0xE1, 0x12, 0x12, 0x12, 0xE1, 0x00, 0x23, 0x10, 0x10, 0x90,
		0x60, 0x07, 0x08, 0x08, 0x08, 0x07, 0x00, 0x0C, 0x0A, 0x09, 0x08, 0x08 };
const uint8_t percent[] = { 0x63, 0x33, 0x18, 0x0C, 0x66, 0x63 };
const uint8_t heart[] = {
		0xF8, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0xFE, 0xFC, 0xF8,
		0xF0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0xFE, 0xFC, 0xF8,
		0x01, 0x07, 0x1F, 0x3F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x1F, 0x07, 0x01, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x0F, 0x0F, 0x0F, 0x07, 0x03, 0x01,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

const uint8_t HR[] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE,
		0x40, 0x40, 0x40, 0x40, 0x40, 0xFE, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC7,
		0x20, 0x20, 0x20, 0x20, 0x20, 0xC7, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
		0x04, 0x04, 0x04, 0x04, 0x0A, 0xF1, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const uint8_t RR[] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC,
		0x42, 0x42, 0x42, 0x42, 0xA2, 0x1C, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCF,
		0x20, 0x20, 0x20, 0x20, 0x20, 0xCF, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
		0x04, 0x04, 0x04, 0x04, 0x0A, 0xF1, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const uint8_t oled_nums[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07,
		0x7f, 0x6f, 0x00 };
const uint8_t oled_nums_pos[] = { 24, 36, 47, 86, 98, 109 };
char pos_y_old = 0;
// function prototypes
static void HAL_I2C_MemTransfer(I2C_HandleTypeDef *hi2c);
static void OLED_WR_Byte(uint8_t dat, uint8_t cmd);
static char min(char a, char b);
static char max(char a, char b);

//local functions
static void HAL_I2C_MemTransfer(I2C_HandleTypeDef *hi2c) {
	static uint8_t y = 0;
	static uint8_t dat = 0;
	if (whatToDo == 4) {
		OLED_WR_Byte(0X8D, OLED_CMD); //SET DCDC??
		OLED_WR_Byte(0X10, OLED_CMD); //DCDC OFF
		OLED_WR_Byte(0XAE, OLED_CMD); //DISPLAY OFF
	} else if (whatToDo == 3) {
		// OLED_WR_Byte((x & 0x0f), OLED_CMD); //0+?????
		dat = 0x00;
		HAL_I2C_Mem_Write(&oledI2c, OLED_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &dat,
				1, 30);
		whatToDo = 0;
	} else if (whatToDo == 2) {
		// OLED_WR_Byte(((x & 0xf0) >> 4) | 0x10, OLED_CMD); //8+?????
		dat = 0x10;
		HAL_I2C_Mem_Write(&oledI2c, OLED_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &dat,
				1, 30);
		whatToDo = 3;
	} else if (whatToDo == 1) {
		// OLED_WR_Byte(0xb0 + y, OLED_CMD); //b+???
		//HAL_I2C_Mem_Write_IT(&hi2c3, OLED_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &dat, 1);
		dat = 0xb0 + y;
		HAL_I2C_Mem_Write(&oledI2c, OLED_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &dat,
				1, 30);
		whatToDo = 2;
	} else if (whatToDo == 0) {
		HAL_I2C_Mem_Write(&oledI2c, OLED_ADDR, 0x40, I2C_MEMADD_SIZE_8BIT,
				(uint8_t *) (&oled_cache[y]), 128, 128);
		y += 1;
		if (y > 7) {
			y = 0;
		}
		whatToDo = 1;
	}
}

static void OLED_WR_Byte(uint8_t dat, uint8_t cmd) {
	if (cmd == OLED_CMD)
		HAL_I2C_Mem_Write(&hi2c3, OLED_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &dat,
				1, 50);
	else {
		oled_cache[point_y][point_x] = dat;
		point_x += 1;
	}
}

static char max(char a, char b) {
	return a > b ? a : b;
}
static char min(char a, char b) {
	return a < b ? a : b;
}
// Global functions
void vWriteToScreen(I2C_HandleTypeDef *hi2c) {
	int i = 0;
	if( HAL_I2C_GetState(&oledI2c) != HAL_I2C_STATE_BUSY){
		for (i = 0; i < 32; i++)
			HAL_I2C_MemTransfer(hi2c);
	}
}

void vOledDisplayOff(void) {
	whatToDo = 4;
	HAL_I2C_MemTransfer(&oledI2c);
}

void vOledSetPos(uint8_t x, uint8_t y) {
	point_x = x;
	point_y = y;
}

void vOledClear(void) {
	uint8_t i, n;
	for (i = 0; i < 8; i++) {
		vOledSetPos(0, i);
		for (n = 0; n < 128; n++)
			OLED_WR_Byte(0x00, OLED_DATA);
	}
}

void vOledShowString(OledFonts font, uint8_t x, uint8_t y, uint8_t *str) {
	uint8_t c = 0, i = 0;
	while (*str != '\0') {
		c = *str - ' '; //???????
		if (x > Max_Column - 1) {
			x = 0;
			y++;
		}
		vOledSetPos(x, y);
		if (font == font6x8) {
			for (i = 0; i < 6; i++)
				OLED_WR_Byte(F6x8[c][i], OLED_DATA);
			x += 6;
		} else if (font == font8x8Hunter) {
			for (i = 0; i < 8; i++)
				OLED_WR_Byte(F8x8_hunter[c][i], OLED_DATA);
			x += 8;
		}
		str++;
	}
}

void vOledInit(void) {
	HAL_Delay(50);
	SSD1306_Init();
	OLED_WR_Byte(0xAE, OLED_CMD); //--display off
	OLED_WR_Byte(0x00, OLED_CMD); //---????4?
	OLED_WR_Byte(0x10, OLED_CMD); //---????4?
	OLED_WR_Byte(0x40, OLED_CMD); //--set start line address
	OLED_WR_Byte(0xB0, OLED_CMD); //--???
	OLED_WR_Byte(0x81, OLED_CMD); // contract control
	OLED_WR_Byte(0xf0, OLED_CMD); //--128
	OLED_WR_Byte(0xA1, OLED_CMD); //set segment remap
	OLED_WR_Byte(0xA6, OLED_CMD); //--a6??,a7??
	OLED_WR_Byte(0xA8, OLED_CMD); //--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3F, OLED_CMD); //--1/32 duty
	OLED_WR_Byte(0xC8, OLED_CMD); //c0?c8????
	OLED_WR_Byte(0xD3, OLED_CMD); //-set display offset
	OLED_WR_Byte(0x00, OLED_CMD); //
	OLED_WR_Byte(0xD5, OLED_CMD); //set osc division
	OLED_WR_Byte(0x80, OLED_CMD); //
	OLED_WR_Byte(0xD9, OLED_CMD); //Set Pre-Charge Period
	OLED_WR_Byte(0xF1, OLED_CMD); //
	OLED_WR_Byte(0xDA, OLED_CMD); //set com pin configuartion
	OLED_WR_Byte(0x12, OLED_CMD); //
	OLED_WR_Byte(0xDB, OLED_CMD); //set Vcomh
	OLED_WR_Byte(0x30, OLED_CMD); //
	OLED_WR_Byte(0x8D, OLED_CMD); //set charge pump enable
	OLED_WR_Byte(0x14, OLED_CMD); //
	OLED_WR_Byte(0xAF, OLED_CMD); //--????,af?,ae?
	//HAL_Delay(50);
	vOledClear();
	vWriteToScreen(&oledI2c);
}

//Sensor Related
void vSetBPM(void) {
	vOledSetPos(58, 4);
	for (i = 0; i < 13; i++) {
		OLED_WR_Byte(bpm[i], OLED_DATA);
	}

}


void vSetPercentage(void) {
	vOledSetPos(121, 4);
	for (i = 0; i < 6; i++) {
		OLED_WR_Byte(percent[i], OLED_DATA);
	}
}


void vSetSPO2(void) {
	vOledSetPos(73, 2);
	for (i = 0; i < 11; i++) {
		OLED_WR_Byte(spo2[i], OLED_DATA);
	}
	vOledSetPos(73, 3);
	for (i = 0; i < 11; i++) {
		OLED_WR_Byte(spo2[11 + i], OLED_DATA);
	}
	vOledSetPos(73, 4);
	for (i = 0; i < 11; i++) {
		OLED_WR_Byte(spo2[22 + i], OLED_DATA);
	}
}

void vMax30102String(void) {
	OledFonts font;
	font = font6x8;
	vOledShowString(font, 0, 0, (uint8_t*) "Heart");
	vOledShowString(font, 33, 0, (uint8_t*) "Rate");
	vOledShowString(font, 60, 0, (uint8_t*) "&");
	vOledShowString(font, 69, 0, (uint8_t*) "Oximetry");
}
//BLE Related

void vOledShowHeart(uint8_t showOrNot) {
	uint8_t i;
	vOledSetPos(1, 2);
	if (showOrNot) {
		for (i = 0; i < 21; i++) {
			OLED_WR_Byte(heart[i], OLED_DATA);
		}
		vOledSetPos(1, 3);
		for (i = 0; i < 21; i++) {
			OLED_WR_Byte(heart[21 + i], OLED_DATA);
		}
		vOledSetPos(1, 4);
		for (i = 0; i < 21; i++) {
			OLED_WR_Byte(heart[42 + i], OLED_DATA);
		}
	} else {
		for (i = 0; i < 21; i++) {
			OLED_WR_Byte(0, OLED_DATA);
		}
		vOledSetPos(1, 3);
		for (i = 0; i < 21; i++) {
			OLED_WR_Byte(0, OLED_DATA);
		}
		vOledSetPos(1, 4);
		for (i = 0; i < 21; i++) {
			OLED_WR_Byte(0, OLED_DATA);
		}
	}
}

void vOledShowNum(uint8_t which, uint8_t num) {
	uint8_t dat = oled_nums[num];
	uint8_t start = oled_nums_pos[which];
	uint8_t i, temp;
	vOledSetPos(start, 2);
	for (i = 0; i < 10; i++) {
		temp = 0;
		if (i <= 1) {
			if (dat & OLED_NUM_F)
				temp |= 0xf8;
		} else if (i >= 2 && i <= 7) {
			if (dat & OLED_NUM_A)
				temp |= 0x06;
		} else {
			if (dat & OLED_NUM_B)
				temp |= 0xf8;
		}
		OLED_WR_Byte(temp, OLED_DATA);
	}
	vOledSetPos(start, 3);
	for (i = 0; i < 10; i++) {
		temp = 0;
		if (i <= 1) {
			if (dat & OLED_NUM_F)
				temp |= 0x03;
			if (dat & OLED_NUM_E)
				temp |= 0xf0;
		} else if (i >= 2 && i <= 7) {
			if (dat & OLED_NUM_G)
				temp |= 0x0c;
		} else {
			if (dat & OLED_NUM_B)
				temp |= 0x03;
			if (dat & OLED_NUM_C)
				temp |= 0xf0;
		}
		OLED_WR_Byte(temp, OLED_DATA);
	}
	vOledSetPos(start, 4);
	for (i = 0; i < 10; i++) {
		temp = 0;
		if (i <= 1) {
			if (dat & OLED_NUM_E)
				temp |= 0x07;
		} else if (i >= 2 && i <= 7) {
			if (dat & OLED_NUM_D)
				temp |= 0x18;
		} else {
			if (dat & OLED_NUM_C)
				temp |= 0x07;
		}
		OLED_WR_Byte(temp, OLED_DATA);
	}
}

void vOledDrawChart(float value) {
	uint8_t dat[] = { 0xff, 0xff, 0xff };
	char pos_y = (char) (value * 0.12) + 10;
	if (pos_y > 23)
		pos_y = 23;
	if (pos_y <= 0)
		pos_y = 0;
	uint8_t i, ii, blank;
	uint8_t y_max = max(pos_y, pos_y_old);
	uint8_t y_min = min(pos_y, pos_y_old);

	if (y_max == y_min)
		y_max = y_min + 1;
	for (i = 0; i < 3; i++) {
		if ((y_min - i * 8) >= 0)
			dat[i] &= (uint8_t) (dat[i] >> (y_min - i * 8));
		if (((i + 1) * 8 - y_max) >= 0)
			dat[i] &= (uint8_t) (dat[i] << ((i + 1) * 8 - y_max));
	}
	for (i = 0; i < 3; i++) {
		vOledSetPos(pos_x_this, 7 - i);
		OLED_WR_Byte(dat[i], OLED_DATA);
		blank = 127 - pos_x_this;
		if (blank > 3)
			blank = 3;
		for (ii = 0; ii < blank; ii++)
			OLED_WR_Byte(0, OLED_DATA);
	}
	pos_y_old = pos_y;
	pos_x_this++;
	if (pos_x_this > 127)
		pos_x_this = 0;
}

void floatToUcharArray(float dest, char *pArray) {
	char tempBufer[2];
	int iPart;
	int fPart;

	iPart = dest;
	fPart = (dest * 100) - (iPart * 100);
	sprintf(tempBufer, "%d", iPart);
	strcat(pArray, tempBufer);
	tempBufer[0] = '.';
	tempBufer[1] = 0;
	strcat(pArray, tempBufer);
	sprintf(tempBufer, "%d", fPart);
	strcat(pArray, tempBufer);
}
int x = 0, y = 10;

void vOledBlePrintData() {

	static int iCharCounter = 0;

	memset(tempLcdBuffer, 0, LCD_BUFFER_LENGHT);
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_GotoXY(0, 0);
	sprintf(tempLcdBuffer, (char *) "DATA SENDING");
	SSD1306_Puts(tempLcdBuffer, &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_GotoXY(40, 14);
	if (iCharCounter == 0) {
		SSD1306_Puts((const char*) "*", &Font_11x18, SSD1306_COLOR_WHITE);
	} else if (iCharCounter == 1) {
		SSD1306_Puts((const char*) "**", &Font_11x18, SSD1306_COLOR_WHITE);
	} else if (iCharCounter == 2) {
		iCharCounter = -1;
		SSD1306_Puts((const char*) "***", &Font_11x18, SSD1306_COLOR_WHITE);
	}
	SSD1306_UpdateScreen();
	iCharCounter++;
}

void vOledBlePrintLux(uint32_t lux) {
	char tempBufer[10];

	memset(tempLcdBuffer, 0, LCD_BUFFER_LENGHT);
	sprintf(tempLcdBuffer, (char *) "Lux: ");
	sprintf(tempBufer, "%d", (int) lux);
	strcat(tempLcdBuffer, tempBufer);
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_GotoXY(7, 8);
	SSD1306_Puts(tempLcdBuffer, &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
}

//
void vOledBlePrintGSR(float gsr) {
	char tempBufer[10];
	int iGsr = gsr;

	if (usBpmPosX > 127 || usBpmPosX == 0) {
		SSD1306_Fill(SSD1306_COLOR_BLACK);
		SSD1306_UpdateScreen();
		usBpmPosX = 1;
		usBpmPoslastX = usBpmPosX;
	}

	SSD1306_DrawFilledRectangle(0, 0, 10, 32, SSD1306_COLOR_BLACK);
	memset(tempLcdBuffer, 0, LCD_BUFFER_LENGHT);
	memset(tempBufer, 0, 10);
	sprintf(tempLcdBuffer, (char *) "GSR: ");
	sprintf(tempBufer, "%4d", iGsr);
	strcat(tempLcdBuffer, tempBufer);
	SSD1306_GotoXY(0, 0);
	gsr = (gsr / maxRangeOfGSR) * 0xFF;
	SSD1306_Puts(tempLcdBuffer, &Font_7x10, SSD1306_COLOR_WHITE);
	usBpmPosY = postorier + (gsr * (0.2509) / percentageOfReduction); //60-(analogValue/10);
	SSD1306_DrawLine(usBpmPoslastX, usBpmPoslastY, usBpmPosX, usBpmPosY,
			SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
	usBpmPoslastX = usBpmPosX;
	usBpmPoslastY = usBpmPosY;
	usBpmPosX++;
}
// Max30102

void vOledBleMaxInit30102(void){
	vOledShowHeart(1);
	vSetBPM();
	vSetPercentage();
	vSetSPO2();
	vMax30102String();
}

void vOledBleClearScreen(void){
	vOledClear();
	vWriteToScreen(&oledI2c);
}

void vOledBlePrintMax30102(uint8_t hr, uint8_t spo2, uint16_t diff) {

	if (hr % 1000 / 100 > 0)
		vOledShowNum(0, hr % 1000 / 100);
	else
		vOledShowNum(0, 10);
	vOledShowNum(1, hr % 100 / 10);
	vOledShowNum(2, hr % 10);
	// ????
	if (spo2 % 1000 / 100 > 0)
		vOledShowNum(3, spo2 % 1000 / 100);
	else
		vOledShowNum(3, 10);
	vOledShowNum(4, spo2 % 100 / 10);
	vOledShowNum(5, spo2 % 10);
	// ???
	//diff = usGetMax30102Diff();
	vOledDrawChart(diff);
	if (diff > 50)
		vOledShowHeart(0);
	else
		vOledShowHeart(1);
	vWriteToScreen(&oledI2c);
}
// si7021

void vMax30003String(void) {
	OledFonts font;
	font = font6x8;
	vOledShowString(font, 0, 0, (uint8_t*) "HR ");
	vOledShowString(font, 33, 0, (uint8_t*) "ECG ");

	vOledShowString(font, 102, 0, (uint8_t*) "RR ");
}

void vOledShowHR() {
	uint8_t i;
	vOledSetPos(1, 2);
		for (i = 0; i < 21; i++) {
			OLED_WR_Byte(HR[i], OLED_DATA);
		}
		vOledSetPos(1, 3);
		for (i = 0; i < 21; i++) {
			OLED_WR_Byte(HR[21 + i], OLED_DATA);
		}
		vOledSetPos(1, 4);
		for (i = 0; i < 21; i++) {
			OLED_WR_Byte(HR[42 + i], OLED_DATA);
		}
}

void vOledShowRR(void) {
	vOledSetPos(70, 2);
	for (i = 0; i < 21; i++) {
		OLED_WR_Byte(RR[i], OLED_DATA);
	}
	vOledSetPos(70, 3);
	for (i = 0; i < 21; i++) {
		OLED_WR_Byte(RR[21 + i], OLED_DATA);
	}
	vOledSetPos(70, 4);
	for (i = 0; i < 21; i++) {
		OLED_WR_Byte(RR[42 + i], OLED_DATA);
	}
}



void vOledBlePrintMax30003(uint32_t ecg, uint32_t hr, uint32_t rr){
	vMax30003String();
	vOledShowHR();
	vOledShowRR();

	vOledShowNum(0, hr% 1000 / 100);
	vOledShowNum(1, hr% 100 / 10);
	vOledShowNum(2, hr% 10);

	vOledShowNum(3, rr% 1000 / 100);
	vOledShowNum(4, rr% 100 / 10);
	vOledShowNum(5, rr% 10);

	vOledDrawChart((float) ecg);
	vWriteToScreen(&oledI2c);
}

void vOledBlePrintTemperature(float temperature) {
	memset(tempLcdBuffer, 0, LCD_BUFFER_LENGHT);
	sprintf(tempLcdBuffer, (char *) "Temp: ");
	floatToUcharArray(temperature, &tempLcdBuffer[6]);
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_GotoXY(7, 8);
	SSD1306_Puts(tempLcdBuffer, &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
}

void vOledBlePrintSi7021(float temperature, float humidity) {
	memset(tempLcdBuffer, 0, LCD_BUFFER_LENGHT);
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	sprintf(tempLcdBuffer, (char *) "Temp: ");
	floatToUcharArray(temperature, &tempLcdBuffer[6]);
	SSD1306_GotoXY(7, 8);
	SSD1306_Puts(tempLcdBuffer, &Font_7x10, SSD1306_COLOR_WHITE);
	memset(tempLcdBuffer, 0, LCD_BUFFER_LENGHT);
	sprintf(&tempLcdBuffer[0], (char *) "Humidity: ");
	floatToUcharArray(humidity, &tempLcdBuffer[10]);
	SSD1306_GotoXY(7, 22);
	strcat(tempLcdBuffer, "%");
	SSD1306_Puts(tempLcdBuffer, &Font_7x10, SSD1306_COLOR_WHITE);

	SSD1306_UpdateScreen();
}

void vOledBlePrintHumidity(float humidity) {
	memset(tempLcdBuffer, 0, LCD_BUFFER_LENGHT);
	sprintf(&tempLcdBuffer[0], (char *) "Humidity: ");
	floatToUcharArray(humidity, &tempLcdBuffer[10]);
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_GotoXY(7, 8);
	strcat(tempLcdBuffer, "%");
	SSD1306_Puts(tempLcdBuffer, &Font_7x10, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
}

