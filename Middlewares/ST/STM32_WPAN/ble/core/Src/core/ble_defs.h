/******************************************************************************
 * @file    ble_defs.h
 * @author  MCD Application Team
 * @brief   BLE definitions
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

#ifndef BLE_DEFS_H__
#define BLE_DEFS_H__


#define HCI_COMMAND_PKT_TYPE          0x01U
#define HCI_ACLDATA_PKT_TYPE          0x02U
#define HCI_EVENT_PKT_TYPE            0x04U

#define HCI_COMMAND_COMPLETE_EVT_CODE 0x0EU
#define HCI_COMMAND_STATUS_EVT_CODE   0x0FU
#define HCI_HARDWARE_ERROR_EVT_CODE   0x10U

#define HCI_COMMAND_HDR_SIZE          4
#define HCI_ACLDATA_HDR_SIZE          5
#define HCI_EVENT_HDR_SIZE            3

#define HCI_COMMAND_MAX_PARAM_LEN     255
#define HCI_ACLDATA_MAX_DATA_LEN      251  /* HC_LE_Data_Packet_Length */
#define HCI_EVENT_MAX_PARAM_LEN       255

#define HCI_COMMAND_PKT_MAX_SIZE \
            (HCI_COMMAND_HDR_SIZE + HCI_COMMAND_MAX_PARAM_LEN)
#define HCI_ACLDATA_PKT_MAX_SIZE \
            (HCI_ACLDATA_HDR_SIZE + HCI_ACLDATA_MAX_DATA_LEN)
#define HCI_EVENT_PKT_MAX_SIZE \
            (HCI_EVENT_HDR_SIZE   + HCI_EVENT_MAX_PARAM_LEN)


#endif /* BLE_DEFS_H__ */

/*********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
