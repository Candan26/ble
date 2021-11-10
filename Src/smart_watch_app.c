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
#include "max30003.h"
#include "max30102.h"
#include "oled.h"
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
	//uint8_t ucUpdate_LUX_Id;
	uint8_t ucUpdate_GSR_Id;
	uint8_t ucUpdate_HR_Id;
	//uint8_t ucUpdate_SPO2_Id;
	uint8_t ucUpdate_Data_Id;
} SMART_WATCH_App_Context_t;

#define EGR_CHANGE_PERIOD        (0.1*1000*1000/CFG_TS_TICK_VAL) /*100ms*/
#define TEMPERATURE_CHANGE_PERIOD (0.1*1000*1000/CFG_TS_TICK_VAL) /*100ms*/
#define HUMIDITY_CHANGE_PERIOD (0.1*1000*1000/CFG_TS_TICK_VAL) /*100ms*/
#define LUX_CHANGE_PERIOD (0.1*1000*1000/CFG_TS_TICK_VAL) /*100ms*/
#define GSR_CHANGE_PERIOD (0.1*1000*1000/CFG_TS_TICK_VAL) /*100ms*/
#define HR_CHANGE_PERIOD (0.1*1000*1000/CFG_TS_TICK_VAL) /*100ms*/
#define SPO2_CHANGE_PERIOD (0.1*1000*1000/CFG_TS_TICK_VAL) /*100ms*/
#define DATA_CHANGE_PERIOD (0.1*1000*1000/CFG_TS_TICK_VAL)*5 /*100ms*/

#define BLE_SWITCH_THRESHOLD 4
#define OFFSET_DATA_HUMIDTY 0
#define OFFSET_DATA_TEMP OFFSET_DATA_HUMIDTY+4
#define OFFSET_DATA_GSR OFFSET_DATA_TEMP+4
#define OFFSET_DATA_HR OFFSET_DATA_GSR+2
#define OFFSET_DATA_SPO2 OFFSET_DATA_HR+1
#define OFFSET_DATA_IRED OFFSET_DATA_SPO2+1
#define OFFSET_DATA_RED OFFSET_DATA_IRED+4
#define OFFSET_DATA_RR_COUNTER OFFSET_DATA_RED+4
#define OFFSET_DATA_RR OFFSET_DATA_RR_COUNTER+1
#define OFFSET_DATA_BPM_COUNTER OFFSET_DATA_RR+20
#define OFFSET_DATA_BPM OFFSET_DATA_BPM_COUNTER+1
#define OFFSET_DATA_ECG_COUNTER OFFSET_DATA_BPM+20
#define OFFSET_DATA_ECG OFFSET_DATA_ECG_COUNTER+2

//#define OFFSET_DATA_LUX OFFSET_DATA_RR+4

#define OFFSET_HR 0
#define OFFSET_SPO2 OFFSET_HR+4
#define OFFSET_CUM_PULSE OFFSET_SPO2+4
#define OFFSET_PULSE_COUNTER OFFSET_CUM_PULSE+4

#define OFFSET_EGR_ECG 0
#define OFFSET_EGR_RR OFFSET_EGR_ECG+4
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
//static void SMART_WATCH_LUX_Timer_Callback(void);
static void SMART_WATCH_GSR_Timer_Callback(void);
static void SMART_WATCH_HR_Timer_Callback(void);
//static void SMART_WATCH_SPO2_Timer_Callback(void);
static void SMART_WATCH_DATA_Timer_Callback(void);
/* Public functions ----------------------------------------------------------*/
void SMART_WATCH_STM_App_Notification_EGR(SMART_WATCH_STM_App_Notification_evt_t *pNotification) {
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

/*
void SMART_WATCH_STM_App_Notification_LUX(SMART_WATCH_STM_App_Notification_evt_t *pNotification){
	switch (pNotification->SMART_WATCH_Evt_Opcode) {
	case SMART_WATCH_STM_NOTIFY_ENABLED_EVT:
		SMART_WATCH_App_Context.ucNotificationStatus = 1;

		HW_TS_Start(SMART_WATCH_App_Context.ucUpdate_LUX_Id,LUX_CHANGE_PERIOD );//LUX_CHANGE_PERIOD
		break;
	case SMART_WATCH_STM_NOTIFY_DISABLED_EVT:
 		SMART_WATCH_App_Context.ucNotificationStatus = 0;

		HW_TS_Stop(SMART_WATCH_App_Context.ucUpdate_LUX_Id);
		break;
	default:
		break;
	}
	return;
}
*/
void SMART_WATCH_STM_App_Notification_HR(SMART_WATCH_STM_App_Notification_evt_t *pNotification){
	switch (pNotification->SMART_WATCH_Evt_Opcode) {
	case SMART_WATCH_STM_NOTIFY_ENABLED_EVT:
		SMART_WATCH_App_Context.ucNotificationStatus = 1;
		/* Start the timer used to update the characteristic */
		HW_TS_Start(SMART_WATCH_App_Context.ucUpdate_HR_Id, HR_CHANGE_PERIOD);//LUX_CHANGE_PERIOD
		break; /* NOTIFY_ENABLED_EVT */
	case SMART_WATCH_STM_NOTIFY_DISABLED_EVT:
 		SMART_WATCH_App_Context.ucNotificationStatus = 0;
		/* Start the timer used to update the characteristic */
		HW_TS_Stop(SMART_WATCH_App_Context.ucUpdate_HR_Id);
		break; /* NOTIFY_DISABLED_EVT */
	default:
		break; /* DEFAULT */
	}
	return;
}
/*
void SMART_WATCH_STM_App_Notification_SPO2(SMART_WATCH_STM_App_Notification_evt_t *pNotification){
	switch (pNotification->SMART_WATCH_Evt_Opcode) {
	case SMART_WATCH_STM_NOTIFY_ENABLED_EVT:
		SMART_WATCH_App_Context.ucNotificationStatus = 1;

		HW_TS_Start(SMART_WATCH_App_Context.ucUpdate_SPO2_Id, SPO2_CHANGE_PERIOD);//SPO2_CHANGE_PERIOD
		break;
	case SMART_WATCH_STM_NOTIFY_DISABLED_EVT:
 		SMART_WATCH_App_Context.ucNotificationStatus = 0;

		HW_TS_Stop(SMART_WATCH_App_Context.ucUpdate_SPO2_Id);
		break;
	default:
		break;
	}
	return;
}
*/
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
	//SCH_RegTask(CFG_MY_TASK_NOTIFY_LUX,SMART_WATCH_Send_Notification_Task);
	SCH_RegTask(CFG_MY_TASK_NOTIFY_GSR,SMART_WATCH_Send_Notification_Task);
	SCH_RegTask(CFG_MY_TASK_NOTIFY_HR,SMART_WATCH_Send_Notification_Task);
	//SCH_RegTask(CFG_MY_TASK_NOTIFY_SPO2,SMART_WATCH_Send_Notification_Task);
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
/*
	HW_TS_Create(CFG_TIM_PROC_ID_ISR,
	      &(SMART_WATCH_App_Context.ucUpdate_LUX_Id),
	      hw_ts_Repeated,
		  SMART_WATCH_LUX_Timer_Callback);
	APP_DBG_MSG("LUX soft BLE timer created \n");
*/
	HW_TS_Create(CFG_TIM_PROC_ID_ISR,
	      &(SMART_WATCH_App_Context.ucUpdate_GSR_Id),
	      hw_ts_Repeated,
		  SMART_WATCH_GSR_Timer_Callback);
	APP_DBG_MSG("GSR soft BLE timer created \n");


	HW_TS_Create(CFG_TIM_PROC_ID_ISR,
	      &(SMART_WATCH_App_Context.ucUpdate_HR_Id),
	      hw_ts_Repeated,
		  SMART_WATCH_HR_Timer_Callback);
	APP_DBG_MSG("HRt BLE timer created \n");
/*
	HW_TS_Create(CFG_TIM_PROC_ID_ISR,
	      &(SMART_WATCH_App_Context.ucUpdate_SPO2_Id),
	      hw_ts_Repeated,
		  SMART_WATCH_SPO2_Timer_Callback);
	APP_DBG_MSG("HRt BLE timer created \n");
*/
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
	static uint8_t sucPrintCounter=0;

	SCH_SetTask(1 << CFG_MY_TASK_NOTIFY_HUMIDITY, CFG_SCH_PRIO_0);//CFG_SCH_PRIO_0
	tmpVal.f = mSi7021Sensor.fHumidty;
	if(sucPrintCounter>=3){
		vOledBlePrintHumidity(tmpVal.f);
		sucPrintCounter=0;
	}
	sucPrintCounter++;
	for(i=0;i<4;i++)
		value[3-i] = tmpVal.uc[i];
	SMART_WATCH_STM_App_Update_Char(SWITCH_HUM, (uint8_t *) &value);
}

static void SMART_WATCH_Temperature_Timer_Callback(void){
	static uint8_t sucPrintCounter=0;
	static unsigned char value[4];
	unionTypeDef tmpVal;
	int i =0;

	SCH_SetTask(1 << CFG_MY_TASK_NOTIFY_TEMPERATURE, CFG_SCH_PRIO_0);
	tmpVal.f = mSi7021Sensor.fTemperature;
	if(sucPrintCounter>=3){
		vOledBlePrintTemperature(mSi7021Sensor.fTemperature);
		sucPrintCounter=0;
	}
	sucPrintCounter++;
	for(i=0;i<4;i++)
		value[3-i] = tmpVal.uc[i];
	SMART_WATCH_STM_App_Update_Char(SWITCH_TEMP, (uint8_t *) &value);
}

static void SMART_WATCH_EGR_Timer_Callback(void) {
	static unsigned char value[8];
	unionTypeDef tmpVal;
	int i=0;
	static uint8_t sucPrintCounter=0;
	SCH_SetTask(1 << CFG_MY_TASK_NOTIFY_EGR, CFG_SCH_PRIO_0);

	tmpVal.ui = uiGetMax3003ECG();
	sucPrintCounter++;
	for(i=0;i<4;i++)
		value[OFFSET_EGR_ECG + 3-i] = tmpVal.uc[i];
/*
 * TODO correct this char later
	tmpVal.ui = uiGetMax3003RR();
	for(i=0;i<4;i++)
		value[OFFSET_EGR_RR + 3-i] = tmpVal.uc[i];

	if(sucPrintCounter>=2){
		vOledBlePrintMax30003(uiGetMax3003ECG(),uiGetMax3003HR(), uiGetMax3003RR()); // TODO correct this line
		sucPrintCounter=0;
	}
*/
	SMART_WATCH_STM_App_Update_Char(SWITCH_EGR, (uint8_t *) &value);
}
/*
static void SMART_WATCH_LUX_Timer_Callback(void) {
	static unsigned char value[4];
	unionTypeDef tmpVal;
	int i =0;

	SCH_SetTask(1 << CFG_MY_TASK_NOTIFY_LUX, CFG_SCH_PRIO_0);
	tmpVal.ui = mTsl2561Sensor.uiLuxValue;
	vOledBlePrintLux(tmpVal.ui);
	for(i=0;i<4;i++)
		value[3-i] = tmpVal.uc[i];
	SMART_WATCH_STM_App_Update_Char(SWITCH_LUX, (uint8_t *) &value);
}
*/
static void SMART_WATCH_GSR_Timer_Callback(void){
	static unsigned char value[4];
	unionTypeDef tmpVal;
	int i =0;

	SCH_SetTask(1 << CFG_MY_TASK_NOTIFY_GSR, CFG_SCH_PRIO_0);
	tmpVal.ui = uiGetGSRHumanResistance();
	//tmpVal.ui =(unsigned int)dCalculateKalmanDataSet((double)tmpVal.ui);
	vOledBlePrintGSR((float)tmpVal.ui);
	for(i=0;i<4;i++)
		value[3-i] = tmpVal.uc[i];

	SMART_WATCH_STM_App_Update_Char(SWITCH_GSR, (uint8_t *) &value);
}

static void SMART_WATCH_HR_Timer_Callback(void){
	static unsigned char value[16];
	unionTypeDef tmpVal;
	int i =0;
	SCH_SetTask(1 << CFG_MY_TASK_NOTIFY_HR, CFG_SCH_PRIO_0);
	static uint8_t sucPrintCounter=0;

	if(sucPrintCounter>=2){
		vOledBlePrintMax30102(ucGetMax30102HR(), ucGetMax30102SPO2(), usGetMax30102Diff());
		sucPrintCounter=0;
	}
	sucPrintCounter++;

	tmpVal.ui=ucGetMax30102HR();
	for(i=0;i<4;i++)
		value[OFFSET_HR + 3-i] = tmpVal.uc[i];

	tmpVal.ui=ucGetMax30102SPO2();
	for(i=0;i<4;i++)
		value[OFFSET_SPO2 + 3-i] = tmpVal.uc[i];

	tmpVal.ui=usGetMax30102Diff();
	for(i=0;i<4;i++)
		value[OFFSET_CUM_PULSE + 3-i] = tmpVal.uc[i];
	tmpVal.ui=uiGetMax30102PulseCounter();
	for(i=0;i<4;i++)
		value[OFFSET_PULSE_COUNTER + 3-i] = tmpVal.uc[i];
	SMART_WATCH_STM_App_Update_Char(SWITCH_HR, (uint8_t *) &value);
}
/*
static void SMART_WATCH_SPO2_Timer_Callback(void){
	static unsigned char value[16];
	unionTypeDef tmpVal;
	int i =0;
	SCH_SetTask(1 << CFG_MY_TASK_NOTIFY_SPO2, CFG_SCH_PRIO_0);

	//TODO add update oled screen
	//LCD_BLE_HTS_SPO2((float)tmpVal.ui);
	LCD_Print("SPO2", "SELECTED");
	tmpVal.ui=ucGetSPO2();
	for(i=0;i<4;i++)
		value[OFFSET_SPO2 + 3-i] = tmpVal.uc[i];
	tmpVal.ui=usGetDiff();
	for(i=0;i<4;i++)
		value[OFFSET_CUM_PULSE + 3-i] = tmpVal.uc[i];
	tmpVal.ui=uiGetPulseCounter();
	for(i=0;i<4;i++)
		value[OFFSET_PULSE_COUNTER + 3-i] = tmpVal.uc[i];
	SMART_WATCH_STM_App_Update_Char(SWITCH_SPO2, (uint8_t *) &value);
}
*/

static void SMART_WATCH_DATA_Timer_Callback(void){
	static unsigned char value[512];
	unionTypeDef tmpVal;
	int i =0;
	static unsigned char ucPrintCounter=0;

	SCH_SetTask(1 << CFG_MY_TASK_NOTIFY_DATA, CFG_SCH_PRIO_0);

	//get humidity data
	tmpVal.f = mSi7021Sensor.fHumidty;
	for(i=0;i<4;i++)
		value[OFFSET_DATA_HUMIDTY+3-i] = tmpVal.uc[i];
	//get temperature data
	tmpVal.f = mSi7021Sensor.fTemperature;
	for(i=0;i<4;i++)
		value[OFFSET_DATA_TEMP+3-i] = tmpVal.uc[i];
	//get gsr data
	tmpVal.ui = uiGetGSRHumanResistance()%0xFFFF;
	//tmpVal.ui =(unsigned int)dCalculateKalmanDataSet((double)tmpVal.ui);
	for(i=0;i<2;i++)
		value[OFFSET_DATA_GSR+1-i] = tmpVal.uc[i];
	//get max30102 HR data
	tmpVal.ui = ucGetMax30102HR()%0xFF;
	for(i=0;i<1;i++)
		value[OFFSET_DATA_HR-i] = tmpVal.uc[i];
	//get max30102 SPO2 data
	tmpVal.ui = ucGetMax30102SPO2()%0xFF;
	for(i=0;i<1;i++)
		value[OFFSET_DATA_SPO2-i] = tmpVal.uc[i];

	//get max30102 IRed data
	tmpVal.ui = uiGetMax30102IRed();
	for(i=0;i<4;i++)
		value[OFFSET_DATA_IRED+3-i] = tmpVal.uc[i];

	//get max30102 Red data
	tmpVal.ui = uiGetMax30102Red();
	for(i=0;i<4;i++)
		value[OFFSET_DATA_RED+3-i] = tmpVal.uc[i];

	//get max3003 RR data
	tmpVal.ui = mMax3003Sensor.ucRorCounter%0xFF;
	for(i=0;i<1;i++)
		value[OFFSET_DATA_RR_COUNTER-i] = tmpVal.uc[i];
	int j = 0;
	for(j = 0 ; j< 5 ; j++){
		tmpVal.ui = mMax3003Sensor.uiaRorVal[j];
		for(i=0;i<4;i++)
			value[OFFSET_DATA_RR+(j*4)+(3-i)] = tmpVal.uc[i];
		mMax3003Sensor.uiaRorVal[j]=0;
	}
	mMax3003Sensor.ucRorCounter=0;

	//get max3003 BPM data
	tmpVal.ui = mMax3003Sensor.ucBpmCounter%0xFF;
	for(i=0;i<1;i++)
		value[OFFSET_DATA_BPM_COUNTER-i] = tmpVal.uc[i];
	for(j = 0 ; j< 5 ; j++){
		tmpVal.f = mMax3003Sensor.faBpm[j];
		for(i=0;i<4;i++)
			value[OFFSET_DATA_BPM+(j*4)+(3-i)] = tmpVal.uc[i];
		mMax3003Sensor.faBpm[j]=0;
	}
	mMax3003Sensor.ucBpmCounter=0;

	//get max3003 ECG data
	tmpVal.ui = mMax3003Sensor.usEcgCounter%0xFFFF;
	for(i=0;i<2;i++)
		value[OFFSET_DATA_ECG_COUNTER-i] = tmpVal.uc[i];
	for(i=0;i<160;i++){
		tmpVal.ui =  mMax3003Sensor.usaEcgVal[i]%0xFFFF;
		for(j=0;j<2;j++)
			value[(OFFSET_DATA_ECG + i*2)+1-j] = tmpVal.uc[j];
		 mMax3003Sensor.usaEcgVal[i] = 0; // Reset data after sending
	}
	 mMax3003Sensor.usEcgCounter=0;
	//TODO  reset data after send

	//get lux data
	/*
	tmpVal.ui = mTsl2561Sensor.uiLuxValue;
	for(i=0;i<4;i++)
		value[OFFSET_DATA_LUX+3-i] = tmpVal.uc[i];
		*/
	// print lcd
	ucPrintCounter++;
	if(ucPrintCounter>=5){
		vOledBlePrintData();
		ucPrintCounter=0;
	}
	SMART_WATCH_STM_App_Update_Char(SWITCH_DATA, (uint8_t *) &value);
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
	//TODO update this
/*
	SMART_WATCH_App_Context.ucUpdate_Data_Id = 7;
	SMART_WATCH_App_Context.ucUpdate_GSR_Id = 6;
	SMART_WATCH_App_Context.ucUpdate_SPO2_Id = 5;
	SMART_WATCH_App_Context.ucUpdate_HR_Id = 4;
	SMART_WATCH_App_Context.ucUpdate_LUX_Id = 3;
	SMART_WATCH_App_Context.ucUpdate_Humidity_Id = 2;
	SMART_WATCH_App_Context.ucUpdate_EGR_Id = 0;
	SMART_WATCH_App_Context.ucUpdate_Temparature_Id = 1;
*/
	SMART_WATCH_App_Context.usParameter = 0;
	SMART_WATCH_App_Context.usParameter = 0;
}
static void SMART_WATCH_Send_Notification_Task(void) {
/*
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
	}*/
	return;
}
