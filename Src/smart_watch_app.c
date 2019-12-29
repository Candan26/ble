#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "smart_watch_app.h"
#include "scheduler.h"

//
#include "hal_lcd.h"
#include "smart_watch_stm.h"
/* Private typedef -----------------------------------------------------------*/
typedef union {
	uint8_t uc[4];
	uint16_t us[2];
	uint32_t ui;
	int i;
	float f;
} unionTypeDef;

typedef union {
	unsigned short us;
	unsigned char uc[2];
}ShortUnionTypeDef;

typedef struct {
	uint16_t usTimeStamp;
	float fMx[4];
	float fGx[4];
	float fAx[4];
	float fQueternions[4];
} SMART_WATCH_MotionCharValue_t;

typedef struct {
	uint8_t ucDataLen;
	uint8_t *ucpData;
	uint16_t usTimeStamp;
	uint16_t usAddress;
} SMART_WATCH_FlashCharValue_t;

typedef struct {
	uint8_t ucDataLen;
	uint32_t uiData0;
	uint32_t uiData1;
} SMART_WATCH_AmbientLightCharValue_t;

typedef struct {
	uint16_t usTemperature;
	uint16_t usHumidity;
} SMART_WATCH_HTU21DCharValue_t;

typedef struct {
	uint16_t usPressure;
	uint16_t usBAR;
	uint16_t usTemperature;
} SMART_WATCH_MPL3115A2CharValue_t;

typedef struct {
	uint16_t usProximity;
	uint16_t usAmbientLight;
} SMART_WATCH_VCNL4010CharValue_t;

typedef struct {
	uint8_t ucNotificationStatus;
	uint16_t usParameter;
	SMART_WATCH_MotionCharValue_t tMotion;
	SMART_WATCH_FlashCharValue_t tFlash;
	SMART_WATCH_AmbientLightCharValue_t tAmbientLight;
	SMART_WATCH_HTU21DCharValue_t tHumidity;
	SMART_WATCH_VCNL4010CharValue_t tProximity;
	uint16_t usChangeStep;
	uint8_t ucUpdate_timer_Id;
} SMART_WATCH_App_Context_t;

#define TEMPERATURE_CHANGE_PERIOD        (0.1*1000*1000/CFG_TS_TICK_VAL) /*100ms*/

/**
 * START of Section BLE_APP_CONTEXT
 */
PLACE_IN_SECTION("BLE_APP_CONTEXT") static SMART_WATCH_App_Context_t SMART_WATCH_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */
/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void SMART_WATCH_context_Init(void);
static void SMART_WATCH_Send_Notification_Task(void);
static void SMART_WATCH_TemparetureChange_Timer_Callback(void);
static void SMART_WATCH_HumidityChange_Timer_Callback(void);

/* Public functions ----------------------------------------------------------*/
void SMART_WATCH_STM_App_Notification(SMART_WATCH_STM_App_Notification_evt_t *pNotification) {
	switch (pNotification->SMART_WATCH_Evt_Opcode) {
	case SMART_WATCH_STM_NOTIFY_ENABLED_EVT:
		SMART_WATCH_App_Context.ucNotificationStatus = 1;
		/* Start the timer used to update the characteristic */
		HW_TS_Start(SMART_WATCH_App_Context.ucUpdate_timer_Id,TEMPERATURE_CHANGE_PERIOD );//TEMPERATURE_CHANGE_PERIOD
		break; /* TEMPLATE_STM_NOTIFY_ENABLED_EVT */
	case SMART_WATCH_STM_NOTIFY_DISABLED_EVT:
 		SMART_WATCH_App_Context.ucNotificationStatus = 0;
		/* Start the timer used to update the characteristic */
		HW_TS_Stop(SMART_WATCH_App_Context.ucUpdate_timer_Id);
		break; /* TEMPLATE_STM_NOTIFY_DISABLED_EVT */
	case SMART_WATCH_STM_WRITE_EVT:
		if (pNotification->DataTransfered.pPayload[0] == 0x00) {
		}
		break; /* TEMPLATE_STM_WRITE_EVT */
	default:
		break; /* DEFAULT */
	}
	return;
}


void SMART_WATCH_APP_Init(void) {
	/* Register task used to update the characteristic (send the notification) */
	SCH_RegTask(CFG_MY_TASK_NOTIFY_TEMPERATURE,SMART_WATCH_Send_Notification_Task);
	SCH_RegTask(CFG_MY_TASK_NOTIFY_HUMIDITY,SMART_WATCH_Send_Notification_Task);

	/* Create timer to change the Temperature and update charecteristic */


	HW_TS_Create(CFG_TIM_PROC_ID_ISR,
	      &(SMART_WATCH_App_Context.ucUpdate_timer_Id),
	      hw_ts_Repeated,
		  SMART_WATCH_HumidityChange_Timer_Callback);

	HW_TS_Create(CFG_TIM_PROC_ID_ISR,
	      &(SMART_WATCH_App_Context.ucUpdate_timer_Id),
	      hw_ts_Repeated,
		  SMART_WATCH_TemparetureChange_Timer_Callback);
	/**
	 * Initialize Template application context
	 */
	SMART_WATCH_App_Context.ucNotificationStatus = 0;
	SMART_WATCH_context_Init();
	return;
}
/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/


static void SMART_WATCH_HumidityChange_Timer_Callback(void){
	SCH_SetTask(1 << CFG_MY_TASK_NOTIFY_HUMIDITY, CFG_SCH_PRIO_0);
	static unsigned char value[2];
	value[0]--;
	value[1]--;

	SMART_WATCH_STM_App_Update_Char(0x0001, (uint8_t *) &value); //TODO
}

static void SMART_WATCH_TemparetureChange_Timer_Callback(void) {
	SCH_SetTask(1 << CFG_MY_TASK_NOTIFY_TEMPERATURE, CFG_SCH_PRIO_0);
	static unsigned char value[2];
	ShortUnionTypeDef tmpVal;
	tmpVal.us = usGetAd8232AnalogValue();
	LCD_BLE_CS_PrintBPM(tmpVal.uc[0]);
	value[0]=tmpVal.uc[0];
	value[1]=tmpVal.uc[1];
	SMART_WATCH_STM_App_Update_Char(0x0000, (uint8_t *) &value); //TODO
}
static void SMART_WATCH_context_Init(void) {
	//Proximity light sensor
	SMART_WATCH_App_Context.tProximity.usAmbientLight = 0;
	SMART_WATCH_App_Context.tProximity.usProximity = 0;
	//AmbientLight
	SMART_WATCH_App_Context.tAmbientLight.ucDataLen = 0;
	SMART_WATCH_App_Context.tAmbientLight.uiData0 = 0;
	SMART_WATCH_App_Context.tAmbientLight.uiData1 = 0;
	//FLASH variable
	SMART_WATCH_App_Context.tFlash.usTimeStamp = 0;
	//Humidity
	SMART_WATCH_App_Context.tHumidity.usHumidity = 0;
	SMART_WATCH_App_Context.tHumidity.usTemperature = 0;
	//
	SMART_WATCH_App_Context.ucNotificationStatus = 0;
	//SMART_WATCH_App_Context.ucUpdate_timer_Id = 0;
	SMART_WATCH_App_Context.usParameter = 0;
	SMART_WATCH_App_Context.usParameter = 0;
}
static void SMART_WATCH_Send_Notification_Task(void) {

	SMART_WATCH_App_Context.tHumidity.usTemperature +=
			SMART_WATCH_App_Context.usChangeStep;
	SMART_WATCH_App_Context.tMotion.usTimeStamp += 1;
	static int counter =0;
	static unsigned char value =5;
	counter++;
	if(counter%5==0){
		counter=0;
		value++;
	}
	if (SMART_WATCH_App_Context.ucNotificationStatus) {
	//	 SMART_WATCH_STM_App_Update_Char(0x0000, (uint8_t *) &value); //TODO
	} else {
	}
	return;
}
