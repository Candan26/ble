/* Includes ------------------------------------------------------------------*/
#include "common_blesvc.h"
#include "hal_lcd.h"
#include "smart_watch_stm.h"
#include "main.h"
#include "oled.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct {
	uint16_t SmartWatchSvcHdle; /**< Service handle */
	uint16_t SmartWatchWriteClientToServerCharHdle; /**< Characteristic handle */
	uint16_t SmartWatchNotifyEGRCharHdle; /**< EGR Characteristic handle handle */
	uint16_t SmartWatchNotifyTemperatureCharHdle;/**< Temperature Characteristic handle handle */
	uint16_t SmartWatchNotifyHumidityCharHdle;/**< Humidity Characteristic handle handle */
	//uint16_t SmartWatchNotifyLUXCharHdle;/**< LUX Characteristic handle handle */
	uint16_t SmartWatchNotifyGSRCharHdle;/**< GSR Characteristic handle handle */
	uint16_t SmartWatchNotifyHRCharHdle;/**< SPO2 Characteristic handle handle  */
	//uint16_t SmartWatchNotifySPO2CharHdle;/**< SPO2 Characteristic handle handle  */
	uint16_t SmartWatchNotifyDataCharHdle;/**< Data Characteristic handle handle */
	uint16_t RebootReqCharHdle; /**< Characteristic handle */
} SmartWatchContext_t;

/* Private defines -----------------------------------------------------------*/
#define UUID_128_SUPPORTED  1

#if (UUID_128_SUPPORTED == 1)
#define BM_UUID_LENGTH  UUID_TYPE_128
#else
#define BM_UUID_LENGTH  UUID_TYPE_16
#endif

#define BM_REQ_CHAR_SIZE    (3)
#if(BLE_CFG_OTA_REBOOT_CHAR != 0)
#if (UUID_128_SUPPORTED == 1)
static const uint8_t BM_REQ_CHAR_UUID[16] = {0x19, 0xed, 0x82, 0xae,
	0xed, 0x21, 0x4c, 0x9d,
	0x41, 0x45, 0x22, 0x8e,
	0x11, 0xFE, 0x00, 0x00};
#else
static const uint8_t BM_REQ_CHAR_UUID[2] = {0x11, 0xFE};
#endif
#endif

PLACE_IN_SECTION("BLE_DRIVER_CONTEXT") static SmartWatchContext_t aSmartWatchContext;

/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t SmartWatch_Event_Handler(void *pckt);

/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/* Hardware Characteristics Service */
/*
 The following 128bits UUIDs have been generated from the random UUID
 generator:
 D973F2E0-B19E-11E2-9E96-0800200C9A66: Service 128bits UUID
 D973F2E1-B19E-11E2-9E96-0800200C9A66: Characteristic_1 128bits UUID
 D973F2E2-B19E-11E2-9E96-0800200C9A66: Characteristic_2 128bits UUID
 */
#define COPY_SMART_WATCH_SERVICE_UUID(uuid_struct)       	 COPY_UUID_128(uuid_struct,0x94,0x53,0x8a,0xee,0xdd,0x71,0x11,0xe9,0x8a,0x34,0x2a,0x2a,0xe2,0xdb,0xcc,0xe4)
#define COPY_SMART_WATCH_WRITE_CHAR_UUID(uuid_struct)    	 COPY_UUID_128(uuid_struct,0x00,0x00,0xAA,0xCC,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_SMART_WATCH_TEMPERATURE_UUID(uuid_struct)   	 COPY_UUID_128(uuid_struct,0x94,0x53,0x8a,0xee,0xdd,0x71,0x11,0xe9,0x90,0xA4,0x2a,0x2a,0xe2,0xdb,0xcc,0xe4)
#define COPY_SMART_WATCH_EGR_SENOR_CHAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x84,0x53,0x8a,0xee,0xdd,0x71,0x11,0xe9,0xA4,0xB4,0x2a,0x2a,0xe2,0xdb,0xcc,0xe4)
#define COPY_SMART_WATCH_HUMIDITY_UUID(uuid_struct)      	 COPY_UUID_128(uuid_struct,0x74,0x53,0x8a,0xee,0xdd,0x71,0x11,0xe9,0x98,0xA9,0x2a,0x2a,0xe2,0xdb,0xcc,0xe4)
#define COPY_SMART_WATCH_LUX_UUID(uuid_struct)      	 	 COPY_UUID_128(uuid_struct,0x64,0x53,0x8a,0xee,0xdd,0x71,0x11,0xe9,0x98,0xA9,0x2a,0x2a,0xe2,0xdb,0xcc,0xe4)
#define COPY_SMART_WATCH_GSR_UUID(uuid_struct)      	 	 COPY_UUID_128(uuid_struct,0x14,0x53,0x8a,0xee,0xdd,0x71,0x11,0xe9,0x98,0xA9,0x2a,0x2a,0xe2,0xdb,0xcc,0xe4)
#define COPY_SMART_WATCH_HR_UUID(uuid_struct)      	 	 	 COPY_UUID_128(uuid_struct,0x54,0x53,0x8a,0xee,0xdd,0x71,0x11,0xe9,0x98,0xA9,0x2a,0x2a,0xe2,0xdb,0xcc,0xe4)
#define COPY_SMART_WATCH_SPO2_UUID(uuid_struct)      	 	 COPY_UUID_128(uuid_struct,0x34,0x53,0x8a,0xee,0xdd,0x71,0x11,0xe9,0x98,0xA9,0x2a,0x2a,0xe2,0xdb,0xcc,0xe4)
#define COPY_SMART_WATCH_DATA_UUID(uuid_struct)      	 	 COPY_UUID_128(uuid_struct,0x24,0x53,0x8a,0xee,0xdd,0x71,0x11,0xe9,0x98,0xA9,0x2a,0x2a,0xe2,0xdb,0xcc,0xe4)

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t SmartWatch_Event_Handler(void *Event) {
	SVCCTL_EvtAckStatus_t return_value;
	hci_event_pckt *event_pckt;
	evt_blue_aci *blue_evt;
	aci_gatt_attribute_modified_event_rp0 * attribute_modified;
	SMART_WATCH_STM_App_Notification_evt_t Notification;
	return_value = SVCCTL_EvtNotAck;
	event_pckt = (hci_event_pckt *) (((hci_uart_pckt*) Event)->data);

	switch (event_pckt->evt) {
	case EVT_VENDOR: {
		blue_evt = (evt_blue_aci*) event_pckt->data;
		switch (blue_evt->ecode) {
		case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED: {
			attribute_modified = (aci_gatt_attribute_modified_event_rp0*) blue_evt->data;
			if(attribute_modified->Attr_Handle == (aSmartWatchContext.SmartWatchNotifyDataCharHdle + 1)){
				ucOledStatusFlag = attribute_modified->Attr_Data[0] & 0x0F;// first 4 bit contains the number of screen
				setActiveSensor(attribute_modified->Attr_Data[0]); // last 4 bit for the active sensor value
				vOledBleClearScreen();
			}
			if (attribute_modified->Attr_Handle == (aSmartWatchContext.SmartWatchNotifyEGRCharHdle + 2)) {

				/**
				 * Descriptor handle
				 */
				return_value = SVCCTL_EvtAckFlowEnable;
				/**
				 * Notify to application
				 */
				if (attribute_modified->Attr_Data[0] & COMSVC_Notification) {
					APP_DBG_MSG("-- GATT : EGR char notification enabled\n");
					Notification.SMART_WATCH_Evt_Opcode = SMART_WATCH_STM_NOTIFY_ENABLED_EVT;
					vOledBleClearScreen();
					SMART_WATCH_STM_App_Notification_EGR(&Notification);
				} else {
					APP_DBG_MSG("-- GATT : EGR char notification disabled\n");
					vOledBleClearScreen();
					LCD_Print("SELECT", "CHARACTER");
					Notification.SMART_WATCH_Evt_Opcode = 	SMART_WATCH_STM_NOTIFY_DISABLED_EVT;
					SMART_WATCH_STM_App_Notification_EGR(&Notification);
				}
			}
			else if (attribute_modified->Attr_Handle == (aSmartWatchContext.SmartWatchNotifyTemperatureCharHdle + 2)){
				return_value = SVCCTL_EvtAckFlowEnable;
				if (attribute_modified->Attr_Data[0] & COMSVC_Notification) {
					APP_DBG_MSG("-- GATT : Temperature char notification enabled\n");
					Notification.SMART_WATCH_Evt_Opcode = SMART_WATCH_STM_NOTIFY_ENABLED_EVT;
					vOledBleClearScreen();
					SMART_WATCH_STM_App_Notification_TEMPERATURE(&Notification);
				} else {
					APP_DBG_MSG("-- GATT : Temperature char notification disabled\n");
					vOledBleClearScreen();
					LCD_Print("SELECT", "CHARACTER");
					Notification.SMART_WATCH_Evt_Opcode = 	SMART_WATCH_STM_NOTIFY_DISABLED_EVT;
					SMART_WATCH_STM_App_Notification_TEMPERATURE(&Notification);
				}
			}
			else if (attribute_modified->Attr_Handle == (aSmartWatchContext.SmartWatchNotifyHumidityCharHdle + 2)){
				return_value = SVCCTL_EvtAckFlowEnable;
				if (attribute_modified->Attr_Data[0] & COMSVC_Notification) {
					APP_DBG_MSG("-- GATT : Humidity  char notification enabled\n");
					Notification.SMART_WATCH_Evt_Opcode = SMART_WATCH_STM_NOTIFY_ENABLED_EVT;
					vOledBleClearScreen();
					SMART_WATCH_STM_App_Notification_HUMIDITY(&Notification);
				} else {
					APP_DBG_MSG("-- GATT : Humidity  char notification disabled\n");
					vOledBleClearScreen();
					LCD_Print("SELECT", "CHARACTER");
					Notification.SMART_WATCH_Evt_Opcode = 	SMART_WATCH_STM_NOTIFY_DISABLED_EVT;
					SMART_WATCH_STM_App_Notification_HUMIDITY(&Notification);
				}
			}
			/*
			else if (attribute_modified->Attr_Handle == (aSmartWatchContext.SmartWatchNotifyLUXCharHdle + 2)){
				return_value = SVCCTL_EvtAckFlowEnable;
				if (attribute_modified->Attr_Data[0] & COMSVC_Notification) {
					APP_DBG_MSG("-- GATT : LUX  char notification enabled\n");
					Notification.SMART_WATCH_Evt_Opcode = SMART_WATCH_STM_NOTIFY_ENABLED_EVT;
					SMART_WATCH_STM_App_Notification_LUX(&Notification);
				} else {
					APP_DBG_MSG("-- GATT : LUX  char notification disabled\n");
					vOledBleClearScreen();
					LCD_Print("SELECT", "CHARACTER");
					Notification.SMART_WATCH_Evt_Opcode = 	SMART_WATCH_STM_NOTIFY_DISABLED_EVT;
					SMART_WATCH_STM_App_Notification_LUX(&Notification);
				}
			}
			*/
			else if (attribute_modified->Attr_Handle == (aSmartWatchContext.SmartWatchNotifyGSRCharHdle + 2)){
				return_value = SVCCTL_EvtAckFlowEnable;
				if (attribute_modified->Attr_Data[0] & COMSVC_Notification) {
					APP_DBG_MSG("-- GATT : GSR  char notification enabled\n");
					Notification.SMART_WATCH_Evt_Opcode = SMART_WATCH_STM_NOTIFY_ENABLED_EVT;
					vOledBleClearScreen();
					SMART_WATCH_STM_App_Notification_GSR(&Notification);
				} else {
					APP_DBG_MSG("-- GATT : GSR  char notification disabled\n");
					vOledBleClearScreen();
					LCD_Print("SELECT", "CHARACTER");
					Notification.SMART_WATCH_Evt_Opcode = 	SMART_WATCH_STM_NOTIFY_DISABLED_EVT;
					SMART_WATCH_STM_App_Notification_GSR(&Notification);
				}
			}
			else if (attribute_modified->Attr_Handle == (aSmartWatchContext.SmartWatchNotifyHRCharHdle + 2)){
				return_value = SVCCTL_EvtAckFlowEnable;
				if (attribute_modified->Attr_Data[0] & COMSVC_Notification) {
					APP_DBG_MSG("-- GATT : HR  char notification enabled\n");
					Notification.SMART_WATCH_Evt_Opcode = SMART_WATCH_STM_NOTIFY_ENABLED_EVT;
					vOledBleMaxInit30102();
					SMART_WATCH_STM_App_Notification_HR(&Notification);
				} else {
					APP_DBG_MSG("-- GATT : HR  char notification disabled\n");
					vOledBleClearScreen();
					LCD_Print("SELECT", "CHARACTER");
					Notification.SMART_WATCH_Evt_Opcode = 	SMART_WATCH_STM_NOTIFY_DISABLED_EVT;
					SMART_WATCH_STM_App_Notification_HR(&Notification);
				}
			}
			/*
			else if (attribute_modified->Attr_Handle == (aSmartWatchContext.SmartWatchNotifySPO2CharHdle + 2)){
				return_value = SVCCTL_EvtAckFlowEnable;
				if (attribute_modified->Attr_Data[0] & COMSVC_Notification) {
					APP_DBG_MSG("--GATT : SPO2  char notification enabled\n");
					Notification.SMART_WATCH_Evt_Opcode = SMART_WATCH_STM_NOTIFY_ENABLED_EVT;
					SMART_WATCH_STM_App_Notification_SPO2(&Notification);
				} else {
					APP_DBG_MSG("-- GATT : SPO2  char notification disabled\n");
					vOledBleClearScreen();
					LCD_Print("SELECT", "CHARACTER");
					Notification.SMART_WATCH_Evt_Opcode = 	SMART_WATCH_STM_NOTIFY_DISABLED_EVT;
					SMART_WATCH_STM_App_Notification_SPO2(&Notification);
				}
			}
			*/
			else if (attribute_modified->Attr_Handle == (aSmartWatchContext.SmartWatchNotifyDataCharHdle + 2)){
				return_value = SVCCTL_EvtAckFlowEnable;
				if (attribute_modified->Attr_Data[0] & COMSVC_Notification) {
					APP_DBG_MSG("-- Data : Data  char notification enabled\n");
					Notification.SMART_WATCH_Evt_Opcode = SMART_WATCH_STM_NOTIFY_ENABLED_EVT;
					vOledBleClearScreen();
					ucOledStatusFlag = OLED_STATUS_BLE;
					SMART_WATCH_STM_App_Notification_Data(&Notification);

				} else {
					APP_DBG_MSG("-- GATT : Data  char notification disabled\n");
					vOledBleClearScreen();
					ucOledStatusFlag = OLED_STATUS_DEF;
					Notification.SMART_WATCH_Evt_Opcode = 	SMART_WATCH_STM_NOTIFY_DISABLED_EVT;
					vOledBleClearScreen();
					SMART_WATCH_STM_App_Notification_Data(&Notification);
				}
			}
			else if (attribute_modified->Attr_Handle == (aSmartWatchContext.SmartWatchNotifyEGRCharHdle+ 1)) {
				APP_DBG_MSG("-- GATT : WRITE CHAR INFO RECEIVED\n");
				Notification.SMART_WATCH_Evt_Opcode = SMART_WATCH_STM_WRITE_EVT;
				Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
				Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
				SMART_WATCH_STM_App_Notification_EGR(&Notification);
			}
			else if (attribute_modified->Attr_Handle
					== (aSmartWatchContext.RebootReqCharHdle + 1)) {
				APP_DBG_MSG("-- GATT : REBOOT REQUEST RECEIVED\n");
				Notification.SMART_WATCH_Evt_Opcode = SMART_WATCH_STM_BOOT_REQUEST_EVT;
				Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
				Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
				SMART_WATCH_STM_App_Notification_EGR(&Notification);
			}
		}
			break;
		default:
			break;
		}
	}
		break; /* HCI_EVT_VENDOR_SPECIFIC */

	default:
		break;
	}

	return (return_value);
}/* end SVCCTL_EvtAckStatus_t */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void SVCCTL_InitSmartWatchSvc(void) {

	Char_UUID_t uuid16;

	/**
	 *	Register the event handler to the BLE controller
	 */
	SVCCTL_RegisterSvcHandler(SmartWatch_Event_Handler);

	/**
	 *  Peer To Peer Service
	 *
	 * Max_Attribute_Records = 2*no_of_char + 1
	 * service_max_attribute_record = 1 for Template service +
	 *                                2 for Template Write characteristic +
	 *                                2 for Template Notify characteristic +
	 *                                1 for client char configuration descriptor +
	 *
	 */

	COPY_SMART_WATCH_SERVICE_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_service(UUID_TYPE_128, (Service_UUID_t *) &uuid16,
	PRIMARY_SERVICE, 100, /*Max_Attribute_Records*/
	&(aSmartWatchContext.SmartWatchSvcHdle));

	/**
	 *  Add Write Characteristic

	COPY_SMART_WATCH_WRITE_CHAR_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_char(aSmartWatchContext.SmartWatchSvcHdle,
	UUID_TYPE_128, &uuid16, 4,
	CHAR_PROP_WRITE_WITHOUT_ RESP | CHAR_PROP_READ | CHAR_PROP_NOTIFY,
	ATTR_PERMISSION_NONE,
	GATT_NOTIFY_ATTRIBUTE_WRITE,
	10,
	1,
	&(aSmartWatchContext.SmartWatchWriteClientToServerCharHdle));

	 */
	/**
	 *   Add Temperature Characteristic
	 */
	/*
	COPY_SMART_WATCH_TEMPERATURE_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_char(aSmartWatchContext.SmartWatchSvcHdle,
	UUID_TYPE_128, &uuid16, 4,
	CHAR_PROP_NOTIFY ,
	ATTR_PERMISSION_NONE,
	GATT_NOTIFY_ATTRIBUTE_WRITE,
	10,
	1,
	&(aSmartWatchContext.SmartWatchNotifyTemperatureCharHdle));

	/**
	 *   Add Humidity Characteristic
	 */
	/*
	COPY_SMART_WATCH_HUMIDITY_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_char(aSmartWatchContext.SmartWatchSvcHdle,
	UUID_TYPE_128, &uuid16, 4,
	CHAR_PROP_NOTIFY,// | CHAR_PROP_READ | CHAR_PROP_WRITE,
	ATTR_PERMISSION_NONE,
	GATT_NOTIFY_ATTRIBUTE_WRITE,
	10,
	1,
	&(aSmartWatchContext.SmartWatchNotifyHumidityCharHdle));
	/**
	 *   Add EGR Characteristic
	 */
	/*
	COPY_SMART_WATCH_EGR_SENOR_CHAR_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_char(aSmartWatchContext.SmartWatchSvcHdle,
	UUID_TYPE_128, &uuid16, 8,
	CHAR_PROP_NOTIFY ,
	ATTR_PERMISSION_NONE,
	GATT_NOTIFY_ATTRIBUTE_WRITE,
	10,
	1,
	&(aSmartWatchContext.SmartWatchNotifyEGRCharHdle));

	/**
	 *   LUX Temperature Characteristic
	 */
	/*
	COPY_SMART_WATCH_LUX_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_char(aSmartWatchContext.SmartWatchSvcHdle,
	UUID_TYPE_128, &uuid16, 4,
	CHAR_PROP_NOTIFY,
	ATTR_PERMISSION_NONE,
	GATT_NOTIFY_ATTRIBUTE_WRITE,
	10,
	1,
	&(aSmartWatchContext.SmartWatchNotifyLUXCharHdle));
	*/

	/**
	 *   Add GSR Characteristic
	 */
	/*
	COPY_SMART_WATCH_GSR_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_char(aSmartWatchContext.SmartWatchSvcHdle,
	UUID_TYPE_128, &uuid16, 4,
	CHAR_PROP_NOTIFY,
	ATTR_PERMISSION_NONE,
	GATT_NOTIFY_ATTRIBUTE_WRITE,
	10,
	1,
	&(aSmartWatchContext.SmartWatchNotifyGSRCharHdle));
	/**
	 *   Add HR Characteristic
	 */
	/*
	COPY_SMART_WATCH_HR_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_char(aSmartWatchContext.SmartWatchSvcHdle,
	UUID_TYPE_128, &uuid16, 16,
	CHAR_PROP_NOTIFY ,
	ATTR_PERMISSION_NONE,
	GATT_NOTIFY_ATTRIBUTE_WRITE,
	10,
	1,
	&(aSmartWatchContext.SmartWatchNotifyHRCharHdle));

	/**
	 *   Add SPO2 Characteristic
	 */
	/*
	COPY_SMART_WATCH_SPO2_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_char(aSmartWatchContext.SmartWatchSvcHdle,
	UUID_TYPE_128, &uuid16, 16,
	CHAR_PROP_NOTIFY,
	ATTR_PERMISSION_NONE,
	GATT_NOTIFY_ATTRIBUTE_WRITE,
	10,
	1,
	&(aSmartWatchContext.SmartWatchNotifySPO2CharHdle));
	*/
	/**
	 *   Add DATA Characteristic
	 */
	COPY_SMART_WATCH_DATA_UUID(uuid16.Char_UUID_128);
	aci_gatt_add_char(aSmartWatchContext.SmartWatchSvcHdle,
	UUID_TYPE_128, &uuid16, 450,
	CHAR_PROP_NOTIFY | CHAR_PROP_WRITE,
	ATTR_PERMISSION_NONE,
	GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
	10, /* encryKeySize */
	1, /* isVariable: 1 */
	&(aSmartWatchContext.SmartWatchNotifyDataCharHdle));

	return;
}

/**
 * @brief  Characteristic update
 * @param  UUID: UUID of the characteristic
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 *
 */
tBleStatus SMART_WATCH_STM_App_Update_Char(uint16_t UUID, uint8_t *pPayload) {
	tBleStatus result = BLE_STATUS_INVALID_PARAMS;
	switch (UUID) {
	case SWITCH_EGR:
		result = aci_gatt_update_char_value(
				aSmartWatchContext.SmartWatchSvcHdle,
				aSmartWatchContext.SmartWatchNotifyEGRCharHdle, 0, /* charValOffset */
				8, /* charValueLen */
				(uint8_t *) pPayload);

		break;

	case SWITCH_TEMP:
		result = aci_gatt_update_char_value(
				aSmartWatchContext.SmartWatchSvcHdle,
				aSmartWatchContext.SmartWatchNotifyTemperatureCharHdle, 0, /* charValOffset */
				4, /* charValueLen */
				(uint8_t *) pPayload);
		break;

	case SWITCH_HUM:
		result = aci_gatt_update_char_value(
				aSmartWatchContext.SmartWatchSvcHdle,
				aSmartWatchContext.SmartWatchNotifyHumidityCharHdle, 0, /* charValOffset */
				4, /* charValueLen */
				(uint8_t *) pPayload);
		break;

/*
	case SWITCH_LUX:
		result = aci_gatt_update_char_value(
				aSmartWatchContext.SmartWatchSvcHdle,
				aSmartWatchContext.SmartWatchNotifyLUXCharHdle, 0,
				4,
				(uint8_t *) pPayload);
		break;
*/
	case SWITCH_HR:
		result = aci_gatt_update_char_value(
				aSmartWatchContext.SmartWatchSvcHdle,
				aSmartWatchContext.SmartWatchNotifyHRCharHdle, 0, /* charValOffset */
				16, /* charValueLen */
				(uint8_t *) pPayload);
		break;
/*
	case SWITCH_SPO2:
		result = aci_gatt_update_char_value(
				aSmartWatchContext.SmartWatchSvcHdle,
				aSmartWatchContext.SmartWatchNotifySPO2CharHdle, 0,
				16,
				(uint8_t *) pPayload);
		break;
*/
	case SWITCH_GSR:
		result = aci_gatt_update_char_value(
				aSmartWatchContext.SmartWatchSvcHdle,
				aSmartWatchContext.SmartWatchNotifyGSRCharHdle, 0, /* charValOffset */
				4, /* charValueLen */
				(uint8_t *) pPayload);
		break;

	case SWITCH_DATA:
		result = aci_gatt_update_char_value(
				aSmartWatchContext.SmartWatchSvcHdle,
				aSmartWatchContext.SmartWatchNotifyDataCharHdle, 0, /* charValOffset */
				450, /* charValueLen */
				(uint8_t *) pPayload);
	default:
		break;
	}

	return result;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
