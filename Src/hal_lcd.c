/* USER CODE BEGIN */
#include "hal_lcd.h"
#include "bluetooth_logo.h"
#include <stdio.h>
#include <string.h>

#define LCD_BUFFER_LENGHT 64
static char tempLcdBuffer[LCD_BUFFER_LENGHT];

//TODO *
unsigned short usBpmPoslastX;
unsigned short usBpmPoslastY;
unsigned short usBpmPosX;
unsigned short usBpmPosY;
int postorier=10;
int percentageOfReduction=5;
void floatToUcharArray(float dest, char *pArray);

void LCD_BLE_CS_PrintBPM(int analogValue){
	if(usBpmPosX> 127 || usBpmPosX==0 ){
		SSD1306_Fill(SSD1306_COLOR_BLACK);
		SSD1306_UpdateScreen();
		usBpmPosX=1;
		usBpmPoslastX=usBpmPosX;
	}
	unsigned char tmpVal=analogValue;
	usBpmPosY= postorier+(tmpVal*(0.2509)/percentageOfReduction); //60-(analogValue/10);
	SSD1306_DrawLine(usBpmPoslastX,usBpmPoslastY,usBpmPosX,usBpmPosY,SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
	usBpmPoslastX=usBpmPosX;
	usBpmPoslastY=usBpmPosY;
	usBpmPosX++;
}

void LCD_Init(void)
{
	SSD1306_Init();
}

void LCD_PrintTest(uint16_t x,uint16_t y, char * msg, SSD1306_COLOR_t color, LCD_CharSize_t charSize)
{
    FontDef_t Font;

    /* Clear screen first */
    SSD1306_Fill(SSD1306_COLOR_BLACK);

    SSD1306_GotoXY(x,y);

    if(charSize == LCD_CHAR_SMALL){
        Font = Font_7x10;
    }else if(charSize == LCD_CHAR_MEDIUM)
    {
        Font = Font_11x18;
    }else if(charSize == LCD_CHAR_BIG)
    {
        Font = Font_16x26;
    }else
    {
        /* put Medium size by default */
        Font = Font_11x18;
    }

    SSD1306_Puts("1", &Font, color);
    SSD1306_UpdateScreen();

    SSD1306_GotoXY(x,y+14);
    SSD1306_Puts("2", &Font, color);
    SSD1306_UpdateScreen();
}

/* Only 11 characters per line with font 11x18 can be displayed */
void LCD_Print1stLine(char * msg)
{
    FontDef_t Font;

    /* Clear screen first */
    SSD1306_Fill(SSD1306_COLOR_BLACK);

    SSD1306_GotoXY(0,0);
    /* put Medium size by default */
    Font = Font_11x18;

    SSD1306_Puts(msg, &Font, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
}

void LCD_Print2ndLine(char * msg)
{
    FontDef_t Font;

    /* Clear screen first */
    //SSD1306_Fill(SSD1306_COLOR_BLACK);

    SSD1306_GotoXY(0,14);
    /* put Medium size by default */
    Font = Font_11x18;

    SSD1306_Puts(msg, &Font, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
}

/* Only 11 characters per line with Font_11x18 can be displayed */
/* Only X characters per line with Font_7x10 can be displayed */
void LCD_Print(char* line1, char* line2){
    FontDef_t Font;

    /* Clear screen first */
    SSD1306_Fill(SSD1306_COLOR_BLACK);

    SSD1306_GotoXY(34,10);
    /* put Medium size by default */
    Font = Font_7x10;
    SSD1306_Puts(line1, &Font, SSD1306_COLOR_WHITE);

    if(line2 != NULL)
    {
        Font = Font_7x10;
        SSD1306_GotoXY(34,21);
        SSD1306_Puts(line2, &Font, SSD1306_COLOR_WHITE);
    }

    SSD1306_UpdateScreen();
}

void LCD_PrintLabel(char * label)
{
  SSD1306_DrawFilledRectangle(0,0,80,19,SSD1306_COLOR_BLACK);
  SSD1306_GotoXY(0,0);
  SSD1306_Puts(label, &Font_11x18, SSD1306_COLOR_WHITE); 
  SSD1306_UpdateScreen();
}

void LCD_THREAD_PrintRLOC(uint16_t rloc)
{
  sprintf(tempLcdBuffer, "0x%04X", rloc);
  SSD1306_DrawFilledRectangle(80,20,52,12,SSD1306_COLOR_BLACK);
  SSD1306_GotoXY(82,22);
  SSD1306_Puts(tempLcdBuffer, &Font_7x10, SSD1306_COLOR_WHITE); 
  SSD1306_UpdateScreen();
}

void LCD_THREAD_PrintPanId(uint16_t panId)
{
  sprintf(tempLcdBuffer, "0x%04X", panId);
  SSD1306_DrawFilledRectangle(80,0,52,20,SSD1306_COLOR_BLACK);
  SSD1306_GotoXY(82,7);
  SSD1306_Puts(tempLcdBuffer, &Font_7x10, SSD1306_COLOR_WHITE); 
  SSD1306_UpdateScreen();
}

void LCD_THREAD_PrintRole(char * role)
{
  SSD1306_DrawFilledRectangle(0,20,80,12,SSD1306_COLOR_BLACK);
  SSD1306_DrawFilledRectangle(0,20,5 + (strlen(role) * 7),12,SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(3,22);
  SSD1306_Puts(role, &Font_7x10, SSD1306_COLOR_BLACK);
  SSD1306_UpdateScreen();
}

void LCD_BLE_PrintLocalName(const char * name)
{
  SSD1306_DrawFilledRectangle(31,0,80,19,SSD1306_COLOR_BLACK);
  SSD1306_GotoXY(31,10);
  SSD1306_Puts(name + 1, &Font_11x18, SSD1306_COLOR_WHITE);
  SSD1306_UpdateScreen();
}

void LCD_BLE_PrintLogo(void)
{
  uint16_t x, y;
  
  for (y = 0; y < BLUETOOTH_LOGO_HEIGHT; y++)
  {
    for (x = 0; x < (BLUETOOTH_LOGO_WIDTH); x++)
    {
      SSD1306_DrawPixel(22 - x, y, (SSD1306_COLOR_t)((bluetooth_logo[y] >> (x + 2)) & 0x00000001));
    }
  }
  SSD1306_UpdateScreen();
}

void LCD_BLE_PrintStatus( char * status)
{
  SSD1306_Fill(SSD1306_COLOR_BLACK);
  SSD1306_GotoXY(34,10);
  SSD1306_Puts(status, &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_UpdateScreen();
}

void LCD_BLE_HRS_PrintBPM(uint8_t BPM)
{
  SSD1306_DrawFilledRectangle(0,20,80,12,SSD1306_COLOR_BLACK);
  //SSD1306_DrawFilledRectangle(0,20,5 + (strlen(role) * 7),12,SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(3,22);
  //SSD1306_Puts(role, &Font_7x10, SSD1306_COLOR_BLACK); 
  SSD1306_UpdateScreen();
}

void floatToUcharArray(float dest, char *pArray){
	char tempBufer[2];
	int iPart;
	int fPart;

	iPart= dest;
	fPart= (dest*100)-(iPart*100);
	sprintf(tempBufer,"%d",iPart);
	strcat(pArray,tempBufer);
	tempBufer[0]='.';
	tempBufer[1]=0;
	strcat(pArray,tempBufer);
	sprintf(tempBufer,"%d", fPart);
	strcat(pArray,tempBufer);
}
void  LCD_BLE_HTS_LUX(uint32_t lux){
	  char tempBufer[10];

	  memset(tempLcdBuffer,0,LCD_BUFFER_LENGHT);
	  sprintf(tempLcdBuffer,(char *)"Lux: ");
	  sprintf(tempBufer,"%d",lux);
	  strcat(tempLcdBuffer,tempBufer);
	  SSD1306_Fill(SSD1306_COLOR_BLACK);
	  SSD1306_GotoXY(7,8);
	  SSD1306_Puts(tempLcdBuffer, &Font_7x10, SSD1306_COLOR_WHITE);
	  SSD1306_UpdateScreen();
}

void LCD_BLE_HTS_PrintTemperature(float temperature){
  memset(tempLcdBuffer,0,LCD_BUFFER_LENGHT);
  sprintf(tempLcdBuffer,(char *)"Temp: ");
  floatToUcharArray(temperature,&tempLcdBuffer[6]);
  SSD1306_Fill(SSD1306_COLOR_BLACK);
  SSD1306_GotoXY(7,8);
  SSD1306_Puts(tempLcdBuffer, &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_UpdateScreen();
}

void LCD_BLE_HTS_PrintHumidity(float humidity)
{
  memset(tempLcdBuffer,0,LCD_BUFFER_LENGHT);
  sprintf(&tempLcdBuffer[0],(char *)"Humidity: ");
  floatToUcharArray(humidity,&tempLcdBuffer[10]);
  SSD1306_Fill(SSD1306_COLOR_BLACK);
  SSD1306_GotoXY(7,8);
  strcat(tempLcdBuffer,"%");
  SSD1306_Puts(tempLcdBuffer, &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_UpdateScreen();
}

void LCD_BLE_TPS_PrintRSSI(uint8_t RSSI)
{
  //sprintf(tempLcdBuffer, "#%02d", errId);
  SSD1306_DrawFilledRectangle(0,0,128,32,SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(7,8);
  SSD1306_Puts("ERROR", &Font_11x18, SSD1306_COLOR_BLACK); 
  SSD1306_GotoXY(66,9);
  SSD1306_Puts(tempLcdBuffer, &Font_7x10, SSD1306_COLOR_BLACK); 
  SSD1306_UpdateScreen();
}
/* USER CODE END */
