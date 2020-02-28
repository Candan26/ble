#ifndef __SMART_WATCH_STM_H
#define __SMART_WATCH_STM_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum {
	SMART_WATCH_STM_NOTIFY_ENABLED_EVT,
	SMART_WATCH_STM_NOTIFY_DISABLED_EVT,
	SMART_WATCH_STM_READ_EVT,
	SMART_WATCH_STM_WRITE_EVT,
	SMART_WATCH_STM_BOOT_REQUEST_EVT,
} SMART_WATCH_STM_Opcode_evt_t;

typedef struct {
	uint8_t * pPayload;
	uint8_t Length;
} SMART_WATCH_STM_Data_t;

typedef struct {
	SMART_WATCH_STM_Opcode_evt_t SMART_WATCH_Evt_Opcode;
	SMART_WATCH_STM_Data_t DataTransfered;
	uint16_t ConnectionHandle;
	uint8_t ServiceInstance;
} SMART_WATCH_STM_App_Notification_evt_t;

/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SMART_WATCH_STM_Init(void);
void SMART_WATCH_STM_App_Notification(SMART_WATCH_STM_App_Notification_evt_t *pNotification);
void SMART_WATCH_STM_App_Notification_TEMPERATURE(SMART_WATCH_STM_App_Notification_evt_t *pNotification);
void SMART_WATCH_STM_App_Notification_HUMIDITY(SMART_WATCH_STM_App_Notification_evt_t *pNotification);
tBleStatus SMART_WATCH_STM_App_Update_Char(uint16_t UUID, uint8_t *pPayload);
#ifdef __cplusplus
}
#endif
#endif

