/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : template_server_app.c
 * Description        : P2P Server Application
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
/* USER CODE BEGIN UserCode */
/* Includes ------------------------------------------------------------------*/
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "template_server_app.h"
#include "scheduler.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct {
	uint16_t TimeStamp;
	uint16_t Value;
} TEMPLATE_TemperatureCharValue_t;
typedef struct {
	uint8_t NotificationStatus;
	uint16_t Parameter;
	TEMPLATE_TemperatureCharValue_t Temperature;
	uint16_t ChangeStep;
	uint8_t Update_timer_Id;
} TEMPLATE_Server_App_Context_t;

/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define TEMPERATURE_CHANGE_STEP            10
#define EGR_CHANGE_PERIOD        (0.1*1000*1000/CFG_TS_TICK_VAL) /*100ms*/
#define TEMPERATURE_VALUE_MAX_THRESHOLD   350
#define TEMPERATURE_VALUE_MIN_THRESHOLD   100

/**
 * START of Section BLE_APP_CONTEXT
 */
PLACE_IN_SECTION("BLE_APP_CONTEXT") static TEMPLATE_Server_App_Context_t TEMPLATE_Server_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */
/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void TEMPLATE_APP_context_Init(void);
static void TEMPLATE_Send_Notification_Task(void);
static void TEMPLATE_TemperatureChange_Timer_Callback(void);
static void TEMPLATE_TemperatureChange_Timer_Callback(void)
{
  SCH_SetTask(1<<CFG_MY_TASK_NOTIFY_EGR, CFG_SCH_PRIO_0);
}

/* Public functions ----------------------------------------------------------*/
void TEMPLATE_STM_App_Notification(	TEMPLATE_STM_App_Notification_evt_t *pNotification) {
	switch (pNotification->Template_Evt_Opcode) {
	case TEMPLATE_STM_NOTIFY_ENABLED_EVT:
		TEMPLATE_Server_App_Context.NotificationStatus = 1;
#if(CFG_DEBUG_APP_TRACE != 0)
		APP_DBG_MSG("-- TEMPLATE APPLICATION SERVER : NOTIFICATION ENABLED\n");
		APP_DBG_MSG(" \n\r");
#endif
		/* Start the timer used to update the characteristic */
		HW_TS_Start(TEMPLATE_Server_App_Context.Update_timer_Id, EGR_CHANGE_PERIOD);
		break; /* TEMPLATE_STM_NOTIFY_ENABLED_EVT */

	case TEMPLATE_STM_NOTIFY_DISABLED_EVT:
		TEMPLATE_Server_App_Context.NotificationStatus = 0;
#if(CFG_DEBUG_APP_TRACE != 0)
		APP_DBG_MSG("-- TEMPLATE APPLICATION SERVER : NOTIFICATION DISABLED\n");
		APP_DBG_MSG(" \n\r");
#endif

		/* Start the timer used to update the characteristic */
		HW_TS_Stop(TEMPLATE_Server_App_Context.Update_timer_Id);

		break; /* TEMPLATE_STM_NOTIFY_DISABLED_EVT */

	case TEMPLATE_STM_WRITE_EVT:
#if(CFG_DEBUG_APP_TRACE != 0)
		APP_DBG_MSG("-- TEMPLATE APPLICATION SERVER : WRITE EVENT RECEIVED\n");
		APP_DBG_MSG("-- TEMPLATE APPLICATION SERVER : 0x%x\n",
				pNotification->DataTransfered.pPayload[0]);
		APP_DBG_MSG(" \n\r");
#endif
		if (pNotification->DataTransfered.pPayload[0] == 0x00) {
#if(CFG_DEBUG_APP_TRACE != 0)
			APP_DBG_MSG("-- TEMPLATE APPLICATION SERVER : START TASK 2 \n");
#endif
		}
		break; /* TEMPLATE_STM_WRITE_EVT */

#if(BLE_CFG_OTA_REBOOT_CHAR != 0)       
		case TEMPLATE_STM_BOOT_REQUEST_EVT:
#if(CFG_DEBUG_APP_TRACE != 0)
		APP_DBG_MSG("-- TEMPLATE APPLICATION SERVER : BOOT REQUESTED\n");
		APP_DBG_MSG(" \n\r");
#endif

		*(uint32_t*)SRAM1_BASE = *(uint32_t*)pNotification->DataTransfered.pPayload;
		NVIC_SystemReset();

		break; /* TEMPLATE_STM_BOOT_REQUEST_EVT */
#endif

	default:
		break; /* DEFAULT */
	}

	return;
}

void TEMPLATE_APP_Init(void) {
	/* Register task used to update the characteristic (send the notification) */
	SCH_RegTask(CFG_MY_TASK_NOTIFY_EGR, TEMPLATE_Send_Notification_Task);
	/* Create timer to change the Temperature and update charecteristic */
	HW_TS_Create(CFG_TIM_PROC_ID_ISR,
	      &(TEMPLATE_Server_App_Context.Update_timer_Id),
	      hw_ts_Repeated,
	      TEMPLATE_TemperatureChange_Timer_Callback);
	/**
	 * Initialize Template application context
	 */
	TEMPLATE_Server_App_Context.NotificationStatus = 0;
	TEMPLATE_APP_context_Init();
	return;
}

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
static void TEMPLATE_APP_context_Init(void) {
	TEMPLATE_Server_App_Context.Parameter = 0;
	TEMPLATE_Server_App_Context.Temperature.TimeStamp = 0;
	TEMPLATE_Server_App_Context.Temperature.Value = 0;
	TEMPLATE_Server_App_Context.ChangeStep=TEMPERATURE_CHANGE_STEP;
}

static void TEMPLATE_Send_Notification_Task(void) {
	uint8_t value[4];

	value[0] = (uint8_t) (TEMPLATE_Server_App_Context.Temperature.TimeStamp	& 0x00FF);
	value[1] = (uint8_t) (TEMPLATE_Server_App_Context.Temperature.TimeStamp >> 8);
	value[2] = (uint8_t) (TEMPLATE_Server_App_Context.Temperature.Value & 0x00FF);
	value[3] = (uint8_t) (TEMPLATE_Server_App_Context.Temperature.Value >> 8);
	TEMPLATE_Server_App_Context.Temperature.Value += TEMPLATE_Server_App_Context.ChangeStep;
	TEMPLATE_Server_App_Context.Temperature.TimeStamp += TEMPERATURE_CHANGE_STEP;
	if (TEMPLATE_Server_App_Context.Temperature.Value > TEMPERATURE_VALUE_MAX_THRESHOLD) {
	  TEMPLATE_Server_App_Context.ChangeStep = -TEMPERATURE_CHANGE_STEP;
	}
	else if (TEMPLATE_Server_App_Context.Temperature.Value < TEMPERATURE_VALUE_MIN_THRESHOLD)
	{
	  TEMPLATE_Server_App_Context.ChangeStep = +TEMPERATURE_CHANGE_STEP;
	}
	if (TEMPLATE_Server_App_Context.NotificationStatus) {
		TEMPLATE_STM_App_Update_Char(0x0000, (uint8_t *) &value);
	} else {

	}
	return;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* USER CODE END UserCode */
