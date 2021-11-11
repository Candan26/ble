/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_entry.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "scheduler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "si7021.h"
#include "tsl2561.h"
#include "kalman.h"
#include "max30102.h"
#include "max30003.h"
#include "oled.h"
//TODO Pars analog datas
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
void vReadSensorData(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_RF_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
volatile uint32_t uiGSRRawData=0;
volatile static unsigned int uiAd8232AnalogConvertedValue = 0;
volatile static unsigned char ucAd8232AnalogConvertedValue = 0;
volatile static unsigned short usAd8232AnalogConvertedValue = 0;

volatile uint32_t uiTimer16Counter = 0;
volatile uint8_t ucHRSensorReadFlag =0;
volatile uint8_t ecgFIFOIntFlag = 0;

unsigned int ADC_TIMEOUT = 300;
unsigned char ucIsResponseFinished = 1;
unsigned int uiAd8232MaxValue = 4000;

#define fCons  0xff/uiAd8232MaxValue

typedef union {
	float f;
	unsigned char uc[4];
	unsigned short us[2];
	unsigned int ui;
} DataConverterTypeDef;
DataConverterTypeDef dataConverter;
typedefBleData bleData;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Peripherals handlers declaration */
/* ADC handler declaration */
ADC_HandleTypeDef AdcHandle;
/* Variables for ADC conversion data */
uint16_t uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE; /* ADC group regular conversion data */

/* Variables for ADC conversion data computation to physical values */
volatile uint16_t usADCxConvertedData_Voltage_mVolt = 0; /* Value of voltage calculated from ADC conversion data (unit: mV) */

/* Variable to report status of ADC group regular uniter conversion          */
/*  0: ADC group regular unitary conversion is not completed                  */
/*  1: ADC group regular unitary conversion is completed                      */
/*  2: ADC group regular unitary conversion has not been started yet          */
/*     (initial state)                                                        */

void prsCheckAI() {
	/* Init variable containing ADC conversion data */
	uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE;
	if (!ucIsResponseFinished)
		return;
	ucIsResponseFinished = 0;

#ifdef DEBUG_GSR
	printSensorData(uiGetGSRHumanResistance());
#endif
	HAL_ADC_Start_IT(&hadc1);
	/*
	if (HAL_ADC_Start_IT(&AdcHandle) != HAL_OK) {
	}
	*/
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	uhADCxConvertedData = HAL_ADC_GetValue(hadc);
	/* Computation of ADC conversions raw data to physical values           */
	/* using helper macro.                                                  */
	usADCxConvertedData_Voltage_mVolt = __ADC_CALC_DATA_VOLTAGE(VDDA_APPLI,
			uhADCxConvertedData);
	vSetGSRAnalogValue(usADCxConvertedData_Voltage_mVolt);
	//HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
	ucIsResponseFinished = 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM16) {
		/*
		uiTimer16Counter++; // counter for AD8232
		if (uiTimer16Counter >= 9) {
			uiTimer16Counter = 0;
			//prsCheckAI();
			//vSi7021ProcessHumidityAndTemperature();
			//vTsl2561ProcessLuminity();
			//vMax30003ReadData();
			//vMax30102ReadData();
		}
		*/
		ucHRSensorReadFlag = 1;
	}
}

void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
	if(GPIO_Pin == MAX30003_INTB_Pin){
		ecgFIFOIntFlag = 1;
	}
}

void initTimer() {
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

	//timer 4 for uart packet checking with 10 ms interval for 32 mHz
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 20;//4
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 63999;
	HAL_TIM_Base_Init(&htim16);
}

void initAdc() {
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
	}
}

void initInterrupts() {
	HAL_TIM_Base_Start_IT(&htim16);
	//HAL_ADC_Start_IT(&hadc1);
}

void systemInit(void) {
	HAL_NVIC_SetPriority(SysTick_IRQn, 0 ,0);
	initTimer();
	initInterrupts();
	vMax30102Init();
	vOledInit();
	vInitsi7021();
  	vMax30003Init();
	//vMax30102Shutdown();
	//HAL_Delay(15);
	//tTsl2561Init();
	//vInitKalman(KALMAN_FILTER_ACTIVATION_LEVEL,0,KALMAN_FILTER_DEFAULT_MIN_CRETERIA);

}

void vSetGSRAnalogValue(uint32_t value){
	uiGSRRawData=value;
}

uint32_t uiGetGSRHumanResistance(void){
	return uiGSRRawData;//((1024+2*uiGSRRawData)*10000)/(512-uiGSRRawData);
}

unsigned char ucGetAd8232AnalogValue() {
	return ucAd8232AnalogConvertedValue;
}

unsigned int uiGetAd8232AnalogValue() {
	return uiAd8232AnalogConvertedValue;
}

unsigned short usGetAd8232AnalogValue() {
	return usAd8232AnalogConvertedValue;
}

void vSetAdcChannel(uint32_t adcChannel){
	ADC_ChannelConfTypeDef sConfig = { 0 };

	sConfig.Channel = adcChannel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

void vReadSensorData(void){
	if(ucHRSensorReadFlag == 1){
		vSi7021ProcessHumidityAndTemperature();
		vMax30003ReadData();
		vMax30102ReadData();
		ucHRSensorReadFlag=0;
		HAL_ADC_Start_DMA(&hadc1,&uiGSRRawData,1);
	}

}

void printSensorData(uint32_t data){
	unsigned char text[20];
	sprintf((char*)text,"%6d\r\n",data);
	HAL_UART_Transmit(&huart1,text,8,300);
	memset(text,0,20);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
 int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C3_Init();
	MX_USART1_UART_Init();
	MX_RTC_Init();
	MX_RF_Init();
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_TIM16_Init();
	MX_TIM17_Init();
	/* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
	HAL_Delay(250);
	systemInit();

	/* USER CODE END 2 */
	APPE_Init();
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
		SCH_Run(~0);
		vReadSensorData();
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Macro to configure the PLL multiplication factor
	 */
	__HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV1);
	/** Macro to configure the PLL clock source
	 */
	__HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_MSI);
	/** Configure LSE Drive Capability
	 */
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4 | RCC_CLOCKTYPE_HCLK2
			| RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the peripherals clocks
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS
			| RCC_PERIPHCLK_RFWAKEUP | RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_I2C3 | RCC_PERIPHCLK_ADC;
	PeriphClkInitStruct.PLLSAI1.PLLN = 24;
	PeriphClkInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV2;
	PeriphClkInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
	PeriphClkInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV2;
	PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADCCLK;
	PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
	PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
	PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
	PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x0040040C; //0x0060112F
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

}
/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}
/**
 * @brief RF Initialization Function
 * @param None
 * @retval None
 */
static void MX_RF_Init(void) {

	/* USER CODE BEGIN RF_Init 0 */

	/* USER CODE END RF_Init 0 */

	/* USER CODE BEGIN RF_Init 1 */

	/* USER CODE END RF_Init 1 */
	/* USER CODE BEGIN RF_Init 2 */

	/* USER CODE END RF_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */
	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}
	/** Enable the WakeUp
	 */
	if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 0;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 0;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM16_Init 2 */

	/* USER CODE END TIM16_Init 2 */

}
/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 9;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 97;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_8;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	//__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOC, SPI1_CS_Pin, GPIO_PIN_RESET);
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(DISP_VDD_GPIO_Port, DISP_VDD_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin | LED_BLUE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : DISP_VSS_Pin
	GPIO_InitStruct.Pin = DISP_VSS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(DISP_VSS_GPIO_Port, &GPIO_InitStruct);
 */

	/*Configure GPIO pins : MEM_WP_Pin SPI1_CS_Pin */
	GPIO_InitStruct.Pin = SPI1_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : DISP_VDD_Pin */
	GPIO_InitStruct.Pin = DISP_VDD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(DISP_VDD_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : MAX30003_INTB_Pin LOM_Pin */
	GPIO_InitStruct.Pin = MAX30003_INTB_Pin | LOM_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : BUTTON_SW1_Pin */
	GPIO_InitStruct.Pin = BUTTON_SW1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BUTTON_SW1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_GREEN_Pin LED_BLUE_Pin */
	GPIO_InitStruct.Pin = LED_GREEN_Pin | LED_BLUE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : MAX30102_INT_Pin */
	GPIO_InitStruct.Pin = MAX30102_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MAX30102_INT_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 11, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/*
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
*/
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
