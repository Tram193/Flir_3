/** 
  ******************************************************************************
  * @file    som_communication.h
  * @author  Gremsy Team
  * @version v1.0.0
  * @date    2021
  * @brief   This file contains all the functions prototypes for the  
  *          firmware library.
  *
  ******************************************************************************
  * @Copyright
  * COPYRIGHT NOTICE: (c) 2011 Gremsy. All rights reserved.
  *
  * The information contained herein is confidential
  * property of Company. The use, copying, transfer or 
  * disclosure of such information is prohibited except
  * by express written agreement with Company.
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __GREMSY_LIB_SOM_COMMUNICATION_H
#define __GREMSY_LIB_SOM_COMMUNICATION_H

/* Includes ------------------------------------------------------------------*/
#include "dji_typedef.h"

#include "som_hardware_reset.h"

/*!< Include mavlink library */
#include "mavlink/gsdk_mavlink.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported define ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
  * @brief 
  * @details 
  * @note 
  */
typedef enum {
    SOM_COMM_IPADDR_REMOTE_IPADDR_LOG_ID = 0,
    SOM_COMM_IPADDR_PAYLOAD_IPADDR_LOG_ID = 1,
    SOM_COMM_IPADDR_PAYLOAD_NETMASK_LOG_ID = 2,
    SOM_COMM_NETMASK_UAV_IPADDR_LOG_ID = 3,
    SOM_COMM_PORT_LOG_ID = 4,
    SOM_COMM_NUM_TRACKED_LOG_ID,
} E_LinuxCommLogId;

/**
  * @brief 
  * @details 
  * @note 
  */
enum {
    SOM_IDLE = 0,
    SOM_INIT = 1,
    SOM_WAIT_HDMI,
    SOM_WAIT_MAV,
    SOM_STREAMING,
    SOM_ERROR_HDMI,
    SOM_FLAHSING,
};
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Get som status.
 * @details This function used for getting the onboard status.
 * @note 
 */
uint8_t SomComm_GetSomState(void);

/**
 * @brief Get som status.
 * @details This function used for getting the onboard status.
 * @note 
 */
T_DjiReturnCode SomComm_SetSomState(void);

/** @brief      This functions use set log data message send to som
    @param[in]  *info : message information
    @param[in]  logID : ID of log
    @return     T_DjiReturnCode
*/
T_DjiReturnCode SomComm_SetLogData(const char *info, uint8_t logID);

/**
 * @brief Get onboard status.
 * @details This function used for getting the onboard status.
 * @note 
 */
T_DjiReturnCode SomComm_GetOnboardStatus(mavlink_onboard_computer_status_t* status);

/**
  * @brief gimbal_interface_init
  * Init hardware to communication with the gimbal device
  */
T_DjiReturnCode SomComm_Init(void);

/** @brief      This functions use init network interface of psdk for video streams
    @param[in]  *ipAddr : payload ip address
    @param[in]  *netMask : payload net mask address
    @param[in]  networkHandle : pointer to dji network handle
    @return     T_DjiReturnCode
*/
T_DjiReturnCode SomComm_netWorkInit(const char *ipAddr, const char *netMask, T_DjiNetworkHandle *networkHandle);

/** @brief      This functions use deinit network interface of psdk for video streams
    @param[in]  networkHandle : pointer to dji network handle
    @return     T_DjiReturnCode
*/
T_DjiReturnCode SomComm_netWorkDeInit(T_DjiNetworkHandle networkHandle);
T_DjiReturnCode SomComm_networkGetDeviceInfo(T_DjiHalNetworkDeviceInfo *deviceInfo);
#ifdef __cplusplus
}
#endif

#endif /* __GREMSY_LIB_SOM_COMMUNICATION_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/
