/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* Definitions of environment analog values */
/* Value of analog reference voltage (Vref+), connected to analog voltage   */
/* supply Vdda (unit: mV).                                                  */
#define VDDA_APPLI                       ((uint32_t)3300)
#define DIGITAL_SCALE_12BITS             ((uint32_t) 0xFFF)
/* Init variable out of ADC expected conversion data range */
#define VAR_CONVERTED_DATA_INIT_VALUE    (DIGITAL_SCALE_12BITS + 1)
/* ## Definition of ADC related resources ################################### */
/* Definition of ADCx clock resources */
#define ADCx                            ADC1
#define ADCx_CHANNELa                   ADC_CHANNEL_3
#define ADCx_CHANNELa_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define ADCx_CHANNELa_GPIO_PORT         GPIOC
#define ADCx_CHANNELa_PIN               GPIO_PIN_2
/**
 * @brief  Macro to calculate the voltage (unit: mVolt)
 *         corresponding to a ADC conversion data (unit: digital value).
 * @note   ADC measurement data must correspond to a resolution of 12bits
 *         (full scale digital value 4095). If not the case, the data must be
 *         preliminarily rescaled to an equivalent resolution of 12 bits.
 * @note   Analog reference voltage (Vref+) must be known from
 *         user board environment.
 * @param  __VREFANALOG_VOLTAGE__ Analog reference voltage (unit: mV)
 * @param  __ADC_DATA__ ADC conversion data (resolution 12 bits)
 *                       (unit: digital value).
 * @retval ADC conversion data equivalent voltage value (unit: mVolt)
 */
#define __ADC_CALC_DATA_VOLTAGE(__VREFANALOG_VOLTAGE__, __ADC_DATA__)       \
  ((__ADC_DATA__) * (__VREFANALOG_VOLTAGE__) / DIGITAL_SCALE_12BITS)

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	unsigned char ucDataFlag;
}typedefBleData;

extern typedefBleData bleData;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AI0_Pin GPIO_PIN_2
#define AI0_GPIO_Port GPIOC
#define AI1_Pin GPIO_PIN_3
#define AI1_GPIO_Port GPIOC
#define DISP_VDD_Pin GPIO_PIN_0
#define DISP_VDD_GPIO_Port GPIOA
#define MAX30003_INTB_Pin GPIO_PIN_2
#define MAX30003_INTB_GPIO_Port GPIOA
#define MAX30003_INTB_EXTI_IRQn EXTI2_IRQn
#define LOM_Pin GPIO_PIN_3
#define LOM_GPIO_Port GPIOA
#define BUTTON_SW1_Pin GPIO_PIN_4
#define BUTTON_SW1_GPIO_Port GPIOC
#define BUTTON_SW1_EXTI_IRQn EXTI4_IRQn
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_5
#define LED_BLUE_GPIO_Port GPIOB
#define MAX30102_INT_Pin GPIO_PIN_0
#define MAX30102_INT_GPIO_Port GPIOD
#define SPI1_CS_Pin GPIO_PIN_5
#define SPI1_CS_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */
void vSetAd8232AnalogValue(unsigned int value);
void vSetAdcChannel(uint32_t adcChannel);
void vSetGSRAnalogValue(uint32_t value);
unsigned char ucGetAd8232AnalogValue();
unsigned int uiGetAd8232AnalogValue();
unsigned short usGetAd8232AnalogValue();
uint32_t uiGetGSRHumanResistance(void);
void printSensorData(uint32_t data);
extern volatile uint8_t ecgFIFOIntFlag;
extern I2C_HandleTypeDef hi2c3;
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
