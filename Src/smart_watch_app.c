#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "smart_watch_app.h"
#include "scheduler.h"

//
#include "hal_lcd.h"
#include "smart_watch_stm.h"
#include "main.h"
#include "si7021.h"
#include "tsl2561.h"
#include "kalman.h"


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
	uint8_t ucUpdate_EGR_Id;
	uint8_t ucUpdate_Temparature_Id;
	uint8_t ucUpdate_Humidity_Id;
	uint8_t ucUpdate_LUX_Id;
	uint8_t ucUpdate_GSR_Id;
	uint8_t ucUpdate_Data_Id;
} SMART_WATCH_App_Context_t;

#define EGR_CHANGE_PERIOD        (0.1*1000*1000/CFG_TS_TICK_VAL) /*100ms*/
#define TEMPERATURE_CHANGE_PERIOD (0.1*1000*1000/CFG_TS_TICK_VAL) /*100ms*/
#define HUMIDITY_CHANGE_PERIOD (0.1*1000*1000/CFG_TS_TICK_VAL) /*100ms*/
#define LUX_CHANGE_PERIOD (0.1*1000*1000/CFG_TS_TICK_VAL) /*100ms*/
#define GSR_CHANGE_PERIOD (0.1*1000*1000/CFG_TS_TICK_VAL) /*100ms*/
#define DATA_CHANGE_PERIOD (0.1*1000*1000/CFG_TS_TICK_VAL) /*100ms*/
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
static void SMART_WATCH_EGR_Timer_Callback(void);
static void SMART_WATCH_Temperature_Timer_Callback(void);
static void SMART_WATCH_Humidity_Timer_Callback(void);
static void SMART_WATCH_LUX_Timer_Callback(void);
static void SMART_WATCH_GSR_Timer_Callback(void);
static void SMART_WATCH_DATA_Timer_Callback(void);
/* Public functions ----------------------------------------------------------*/
void SMART_WATCH_STM_App_Notification(SMART_WATCH_STM_App_Notification_evt_t *pNotification) {
	switch (pNotification->SMART_WATCH_Evt_Opcode) {
	case SMART_WATCH_STM_NOTIFY_ENABLED_EVT:
		SMART_WATCH_App_Context.ucNotificationStatus = 1;
		/* Start the timer used to update the characteristic */
		HW_TS_Start(SMART_WATCH_App_Context.ucUpdate_EGR_Id,EGR_CHANGE_PERIOD );//TEMPERATURE_CHANGE_PERIOD
		break; /* NOTIFY_ENABLED_EVT */
	case SMART_WATCH_STM_NOTIFY_DISABLED_EVT:
 		SMART_WATCH_App_Context.ucNotificationStatus = 0;
		/* Start the timer used to update the characteristic */
		HW_TS_Stop(SMART_WATCH_App_Context.ucUpdate_EGR_Id);
		break; /* NOTIFY_DISABLED_EVT */
	case SMART_WATCH_STM_WRITE_EVT:
		if (pNotification->DataTransfered.pPayload[0] == 0x00) {
		}
		break; /* WRITE_EVT */
	default:
		break; /* DEFAULT */
	}
	return;
}

void SMART_WATCH_STM_App_Notification_TEMPERATURE(SMART_WATCH_STM_App_Notification_evt_t *pNotification){
	switch (pNotification->SMART_WATCH_Evt_Opcode) {
	case SMART_WATCH_STM_NOTIFY_ENABLED_EVT:
		SMART_WATCH_App_Context.ucNotificationStatus = 1;
		/* Start the timer used to update the characteristic */
		HW_TS_Start(SMART_WATCH_App_Context.ucUpdate_Temparature_Id,TEMPERATURE_CHANGE_PERIOD );//TEMPERATURE_CHANGE_PERIOD
		break; /* NOTIFY_ENABLED_EVT */
	case SMART_WATCH_STM_NOTIFY_DISABLED_EVT:
 		SMART_WATCH_App_Context.ucNotificationStatus = 0;
		/* Start the timer used to update the characteristic */
		HW_TS_Stop(SMART_WATCH_App_Context.ucUpdate_Temparature_Id);
		break; /* NOTIFY_DISABLED_EVT */
	default:
		break; /* DEFAULT */
	}
	return;
}

void SMART_WATCH_STM_App_Notification_HUMIDITY(SMART_WATCH_STM_App_Notification_evt_t *pNotification){
	switch (pNotification->SMART_WATCH_Evt_Opcode) {
	case SMART_WATCH_STM_NOTIFY_ENABLED_EVT:
		SMART_WATCH_App_Context.ucNotificationStatus = 1;
		/* Start the timer used to update the characteristic */
		HW_TS_Start(SMART_WATCH_App_Context.ucUpdate_Humidity_Id,HUMIDITY_CHANGE_PERIOD );//HUMIDITY_CHANGE_PERIOD
		break; /* NOTIFY_ENABLED_EVT */
	case SMART_WATCH_STM_NOTIFY_DISABLED_EVT:
 		SMART_WATCH_App_Context.ucNotificationStatus = 0;
		/* Start the timer used to update the characteristic */
		HW_TS_Stop(SMART_WATCH_App_Context.ucUpdate_Humidity_Id);
		break; /* NOTIFY_DISABLED_EVT */
	default:
		break; /* DEFAULT */
	}
	return;
}

void SMART_WATCH_STM_App_Notification_LUX(SMART_WATCH_STM_App_Notification_evt_t *pNotification){
	switch (pNotification->SMART_WATCH_Evt_Opcode) {
	case SMART_WATCH_STM_NOTIFY_ENABLED_EVT:
		SMART_WATCH_App_Context.ucNotificationStatus = 1;
		/* Start the timer used to update the characteristic */
		HW_TS_Start(SMART_WATCH_App_Context.ucUpdate_LUX_Id,LUX_CHANGE_PERIOD );//LUX_CHANGE_PERIOD
		break; /* NOTIFY_ENABLED_EVT */
	case SMART_WATCH_STM_NOTIFY_DISABLED_EVT:
 		SMART_WATCH_App_Context.ucNotificationStatus = 0;
		/* Start the timer used to update the characteristic */
		HW_TS_Stop(SMART_WATCH_App_Context.ucUpdate_LUX_Id);
		break; /* NOTIFY_DISABLED_EVT */
	default:
		break; /* DEFAULT */
	}
	return;
}

void SMART_WATCH_STM_App_Notification_GSR(SMART_WATCH_STM_App_Notification_evt_t *pNotification){
	switch (pNotification->SMART_WATCH_Evt_Opcode) {
	case SMART_WATCH_STM_NOTIFY_ENABLED_EVT:
		SMART_WATCH_App_Context.ucNotificationStatus = 1;
		/* Start the timer used to update the characteristic */
		HW_TS_Start(SMART_WATCH_App_Context.ucUpdate_GSR_Id, GSR_CHANGE_PERIOD);//LUX_CHANGE_PERIOD
		break; /* NOTIFY_ENABLED_EVT */
	case SMART_WATCH_STM_NOTIFY_DISABLED_EVT:
 		SMART_WATCH_App_Context.ucNotificationStatus = 0;
		/* Start the timer used to update the characteristic */
		HW_TS_Stop(SMART_WATCH_App_Context.ucUpdate_GSR_Id);
		break; /* NOTIFY_DISABLED_EVT */
	default:
		break; /* DEFAULT */
	}
	return;
}
void SMART_WATCH_STM_App_Notification_Data(SMART_WATCH_STM_App_Notification_evt_t *pNotification){
	switch (pNotification->SMART_WATCH_Evt_Opcode) {
	case SMART_WATCH_STM_NOTIFY_ENABLED_EVT:
		SMART_WATCH_App_Context.ucNotificationStatus = 1;
		/* Start the timer used to update the characteristic */
		HW_TS_Start(SMART_WATCH_App_Context.ucUpdate_Data_Id, DATA_CHANGE_PERIOD);//LUX_CHANGE_PERIOD
		break; /* NOTIFY_ENABLED_EVT */
	case SMART_WATCH_STM_NOTIFY_DISABLED_EVT:
 		SMART_WATCH_App_Context.ucNotificationStatus = 0;
		/* Start the timer used to update the characteristic */
		HW_TS_Stop(SMART_WATCH_App_Context.ucUpdate_Data_Id);
		break; /* NOTIFY_DISABLED_EVT */
	default:
		break; /* DEFAULT */
	}
	return;
}
void SMART_WATCH_APP_Init(void) {
	/* Register task used to update the characteristic (send the notification) */
	SCH_RegTask(CFG_MY_TASK_NOTIFY_EGR,SMART_WATCH_Send_Notification_Task);
	SCH_RegTask(CFG_MY_TASK_NOTIFY_TEMPERATURE,SMART_WATCH_Send_Notification_Task);
	SCH_RegTask(CFG_MY_TASK_NOTIFY_HUMIDITY,SMART_WATCH_Send_Notification_Task);
	SCH_RegTask(CFG_MY_TASK_NOTIFY_LUX,SMART_WATCH_Send_Notification_Task);
	SCH_RegTask(CFG_MY_TASK_NOTIFY_GSR,SMART_WATCH_Send_Notification_Task);
	SCH_RegTask(CFG_MY_TASK_NOTIFY_DATA,SMART_WATCH_Send_Notification_Task);
	/* Create timer to change the Temperature and update charecteristic */
	//initilizing ble timers
	APP_DBG_MSG("Initializing BLE timers \n");
	HW_TS_Create(CFG_TIM_PROC_ID_ISR,
	      &(SMART_WATCH_App_Context.ucUpdate_Temparature_Id),
	      hw_ts_Repeated,
		  SMART_WATCH_Temperature_Timer_Callback);
	APP_DBG_MSG("Temperature soft BLE timer created \n");

		HW_TS_Create(CFG_TIM_PROC_ID_ISR,
	      &(SMART_WATCH_App_Context.ucUpdate_EGR_Id),
	      hw_ts_Repeated,
		  SMART_WATCH_EGR_Timer_Callback);
	APP_DBG_MSG("EGR soft BLE timer created \n");

	HW_TS_Create(CFG_TIM_PROC_ID_ISR,
	      &(SMART_WATCH_App_Context.ucUpdate_Humidity_Id),
	      hw_ts_Repeated,
		  SMART_WATCH_Humidity_Timer_Callback);
	APP_DBG_MSG("Humidity soft BLE timer created \n");

	HW_TS_Create(CFG_TIM_PROC_ID_ISR,
	      &(SMART_WATCH_App_Context.ucUpdate_LUX_Id),
	      hw_ts_Repeated,
		  SMART_WATCH_LUX_Timer_Callback);
	APP_DBG_MSG("LUX soft BLE timer created \n");

	HW_TS_Create(CFG_TIM_PROC_ID_ISR,
	      &(SMART_WATCH_App_Context.ucUpdate_GSR_Id),
	      hw_ts_Repeated,
		  SMART_WATCH_GSR_Timer_Callback);
	APP_DBG_MSG("LUX soft BLE timer created \n");

	HW_TS_Create(CFG_TIM_PROC_ID_ISR,
	      &(SMART_WATCH_App_Context.ucUpdate_Data_Id),
	      hw_ts_Repeated,
		  SMART_WATCH_DATA_Timer_Callback);
	APP_DBG_MSG("Data soft BLE timer created \n");
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
static void SMART_WATCH_Humidity_Timer_Callback(void){
	static unsigned char value[4];
	unionTypeDef tmpVal;
	int i =0;
	static unsigned char ucPrintCounter=0;

	SCH_SetTask(1 << CFG_MY_TASK_NOTIFY_HUMIDITY, CFG_SCH_PRIO_0);//CFG_SCH_PRIO_0
	tmpVal.f = mSi7021Sensor.fHumidty;
	ucPrintCounter++;
	if(ucPrintCounter==5){
		LCD_BLE_HTS_PrintHumidity(tmpVal.f);
		ucPrintCounter=0;
	}
	for(i=0;i<4;i++)
		value[3-i] = tmpVal.uc[i];
	SMART_WATCH_STM_App_Update_Char(0x0002, (uint8_t *) &value);
}

static void SMART_WATCH_Temperature_Timer_Callback(void){
	static unsigned char ucPrintCounter=0;
	static unsigned char value[4];
	unionTypeDef tmpVal;
	int i =0;

	SCH_SetTask(1 << CFG_MY_TASK_NOTIFY_TEMPERATURE, CFG_SCH_PRIO_0);
	tmpVal.f = mSi7021Sensor.fTemperature;
	ucPrintCounter++;
	if(ucPrintCounter==5){
		LCD_BLE_HTS_PrintTemperature(mSi7021Sensor.fTemperature);
		ucPrintCounter=0;
	}
	for(i=0;i<4;i++)
		value[3-i] = tmpVal.uc[i];
	SMART_WATCH_STM_App_Update_Char(0x0001, (uint8_t *) &value);
}

static void SMART_WATCH_EGR_Timer_Callback(void) {
	static unsigned char value[2];
	ShortUnionTypeDef tmpVal;

	SCH_SetTask(1 << CFG_MY_TASK_NOTIFY_EGR, CFG_SCH_PRIO_0);
	tmpVal.us = usGetAd8232AnalogValue();
	LCD_BLE_CS_PrintBPM(tmpVal.uc[0]);
	value[1]=tmpVal.uc[0];
	value[0]=tmpVal.uc[1];
	SMART_WATCH_STM_App_Update_Char(0x0000, (uint8_t *) &value);
}

static void SMART_WATCH_LUX_Timer_Callback(void) {
	static unsigned char value[4];
	unionTypeDef tmpVal;
	int i =0;

	SCH_SetTask(1 << CFG_MY_TASK_NOTIFY_LUX, CFG_SCH_PRIO_0);
	tmpVal.ui = mTsl2561Sensor.uiLuxValue;
	LCD_BLE_HTS_LUX(tmpVal.ui);
	for(i=0;i<4;i++)
		value[3-i] = tmpVal.uc[i];

	SMART_WATCH_STM_App_Update_Char(0x0003, (uint8_t *) &value);
}

static void SMART_WATCH_GSR_Timer_Callback(void){
	static unsigned char value[4];
	unionTypeDef tmpVal;
	int i =0;
	SCH_SetTask(1 << CFG_MY_TASK_NOTIFY_GSR, CFG_SCH_PRIO_0);
	//TODO add LCD and
	tmpVal.ui=uiGetGSRHumanResistance();
	tmpVal.ui =(unsigned int)dCalculateKalmanDataSet((double)tmpVal.ui);
	LCD_BLE_HTS_GSR((float)tmpVal.ui);
	for(i=0;i<4;i++)
		value[3-i] = tmpVal.uc[i];

	SMART_WATCH_STM_App_Update_Char(0x0004, (uint8_t *) &value);
}

static void SMART_WATCH_DATA_Timer_Callback(void){
	static unsigned char value[20];
	unionTypeDef tmpVal;
	int i =0;
	static unsigned char ucPrintCounter=0;

#define OFFSET_HUMIDTY 0
#define OFFSET_TEMPERATURE OFFSET_HUMIDTY+4
#define OFFSET_EGR OFFSET_TEMPERATURE+4
#define OFFSET_LUX OFFSET_EGR+4
#define OFFSET_GSR OFFSET_LUX+4

	SCH_SetTask(1 << CFG_MY_TASK_NOTIFY_DATA, CFG_SCH_PRIO_0);

	//get humidity data
	tmpVal.f = mSi7021Sensor.fHumidty;
	for(i=0;i<4;i++)
		value[OFFSET_HUMIDTY+3-i] = tmpVal.uc[i];

	//get temperature data
	tmpVal.f = mSi7021Sensor.fTemperature;
	for(i=0;i<4;i++)
		value[OFFSET_TEMPERATURE+3-i] = tmpVal.uc[i];
	//get egr data
	if(bleData.ucDataFlag=='H'){
		tmpVal.ui = usGetAd8232AnalogValue();
	}else{
		tmpVal.ui = 0;
	}
	for(i=0;i<4;i++)
		value[OFFSET_EGR+3-i] = tmpVal.uc[i];
	//get lux data
	tmpVal.ui = mTsl2561Sensor.uiLuxValue;
	for(i=0;i<4;i++)
		value[OFFSET_LUX+3-i] = tmpVal.uc[i];
	//get gsr data
	if(bleData.ucDataFlag=='S'){
		tmpVal.ui=uiGetGSRHumanResistance();
	}else{
		tmpVal.ui=0;
	}

	tmpVal.ui =(unsigned int)dCalculateKalmanDataSet((double)tmpVal.ui);
	for(i=0;i<4;i++)
		value[OFFSET_GSR+3-i] = tmpVal.uc[i];
	// print lcd
	ucPrintCounter++;
	if(ucPrintCounter==5){
		LCD_BLE_HTS_Data();
		ucPrintCounter=0;
	}

	if(bleData.ucDataFlag=='S'){
		bleData.ucDataFlag='H';
		vSetAdcChannel(ADC_CHANNEL_3);
	}else if (bleData.ucDataFlag=='H'){
		bleData.ucDataFlag='S';
		vSetAdcChannel(ADC_CHANNEL_4);
	}

	SMART_WATCH_STM_App_Update_Char(0x0005, (uint8_t *) &value);
}


static void SMART_WATCH_context_Init(void) {
	APP_DBG_MSG("Initializing ble context properties.\n");
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

	//SMART_WATCH_App_Context.ucUpdate_Data_Id=5;
	SMART_WATCH_App_Context.ucUpdate_Humidity_Id = 2;
	SMART_WATCH_App_Context.ucUpdate_EGR_Id = 1;
	SMART_WATCH_App_Context.ucUpdate_Temparature_Id=0;

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
	} else {
	}
	return;
}
