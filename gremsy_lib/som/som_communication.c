/**
  ******************************************************************************
  * @file    som_communication.c
  * @author  Gremsy Team
  * @version v100
  * @date    2021
  * @brief   This file contains all the functions prototypes for the  
  *          firmware library.
  *
  ************************************************************
  ******************
  * @par
  * COPYRIGHT NOTICE: (c) 2011 Gremsy.
  * All rights reserved.Firmware coding style V1.0.beta
  *
  * The information contained herein is confidential
  * property of Company. The use, copying, transfer or
  * disclosure of such information is prohibited except
  * by express written agreement with Company.
  *
  ******************************************************************************
*/
/* Includes------------------------------------------------------------------------------*/
#include "som/som_communication.h"

#include "dji_platform.h"
#include "utils/util_misc.h"
#include "dji_logger.h"

#include "uart.h"
#include "stm32f4xx_hal.h"

//#include "camera/workswell_wiris/enterprise/wwe_interface.h"

/// Include camera emu interface
#include "camera_emu/test_payload_cam_emu_base.h"
/* Private define------------------------------------------------------------------------------*/
#define SOM_COMM_DEBUG 1

#if SOM_COMM_DEBUG
# define DebugMsg(fmt, args ...) do {USER_LOG_DEBUG("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);
# define DebugInfo(fmt, args ...) do {USER_LOG_INFO("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);
# define DebugWarning(fmt, args ...) do {USER_LOG_WARN("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);
#else
# define DebugMsg(fmt, args ...)
# define DebugInfo(fmt, args ...)
# define DebugWarning(fmt, args ...)
#endif

# define DebugError(fmt, args ...) do {USER_LOG_ERROR("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);

#define SOM_COMMUNICATION_UART_NUM           UART_NUM_6
#define SOM_COMMUNICATION_UART_BAUD          115200
#define PSDK_NETWORK_INFO_CMD_MAX_SIZE       64
#define SOM_COMMUNICATION_MAVLINK_CHANNEL    MAVLINK_COMM_2

#define SOM_COMMUNICATION_TASK_FREQ          (1000)
#define SOM_COMMUNICATION_TASK_STACK_SIZE    (1024 * 2)
/* Private typedef------------------------------------------------------------------------------*/

/**
 * @brief 
 * @details Each camera will be defined a list define
 * @note 
 */
enum lt6911_bootstrap_t{
    LT6911_UNKNOWN = 0,
    LT6911_720P50,
    LT6911_720P60,
    LT6911_1080P30,
    LT6911_1080P50,
    LT6911_1080P60,
    LT6911_4K30,
    LT6911_AUTO,
};

#if defined(QE_CAM)
    #define SOM_INIT_HDMI_RESOLUTION     LT6911_AUTO
#endif

#if defined(WWP_CAM) || defined(WWS_CAM) || defined(WWA_CAM) || defined(WWE_CAM) || defined(CORO_CAM)
    #define SOM_INIT_HDMI_RESOLUTION     LT6911_720P50
#endif

#if defined(FLIR_CAM)
    #define SOM_INIT_HDMI_RESOLUTION     LT6911_1080P30
#endif

#if defined(OFIL_CAM)
    #define SOM_INIT_HDMI_RESOLUTION     LT6911_720P60
#endif

/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
static gremsy_mavlink_handler_t s_somCommHandler;

/*!< buffer used to receive data from UART interface from the gimbal*/
static uint8_t s_somCommRecBuf[MAVLINK_MAX_PAYLOAD_LEN];

static T_DjiTaskHandle s_somRecvThreadId;
static T_DjiTaskHandle s_somProcessThreadId;

static T_DjiMutexHandle s_somMutexSend;

static uint32_t s_lastReportMessageMs;
static bool s_hasDetectedS5L = false;
static bool s_flagSendLogData = false;
static mavlink_onboard_computer_status_t s_onboardStatus;

static char s_localIpAddress[DJI_IP_ADDR_STR_SIZE_MAX] = {0};
static char s_configMask[DJI_IP_ADDR_STR_SIZE_MAX] = {0};
static char s_remoteIpAddress[DJI_IP_ADDR_STR_SIZE_MAX] = {0};
static char s_remotePort[DJI_IP_ADDR_STR_SIZE_MAX] = {0};

static char s_networkLocalIpAddress[DJI_IP_ADDR_STR_SIZE_MAX] = {0};
static char s_networkConfigMask[DJI_IP_ADDR_STR_SIZE_MAX] = {0};

static uint8_t s_logId = 0;
static bool    s_isRequestLogID = false;
static uint8_t s_somStatus = SOM_IDLE;
static uint32_t s_taskSomComm_working = 0;

/* Private function prototypes------------------------------------------------------------------------------*/

/* Private functions------------------------------------------------------------------------------*/

/** @group __SOM_COMMUNICATION_FUNCTIONS_
    @{
*/#ifndef __SOM_COMMUNICATION_FUNCTIONS_
#define __SOM_COMMUNICATION_FUNCTIONS_
/** @brief Function send mavlink data to serial
    @param1[in] chan Buffer number used to transmit data
    @param2[in] buf Pointer to buffer
    @param2[in] len Bytes to send
    @return None
*/
int SomComm_SendDataFrame(const uint8_t *buf, uint16_t len)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    int realLen = 0;
    
    /*!< Acquire and lock the mutex when peripheral access is required. */
    if(osalHandler->MutexLock(s_somMutexSend) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("Mutex Lock Failed");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    
    //Read UART data.
    realLen = UART_Write(SOM_COMMUNICATION_UART_NUM, buf, len);
    
    /*!<  Unlock and release the mutex when done with the peripheral access.*/
    if(osalHandler->MutexUnlock(s_somMutexSend) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("Mutex UnLock Failed");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    return realLen;
}

/** @brief Function sends heartbeat to gimbal 1hz periodically 
    @param1[in] None
    @return None
*/
T_DjiReturnCode SomComm_SendHeatbeat(void)
{
    mavlink_heartbeat_t heartbeat;
    
    /*!< Set default information*/
    uint8_t systemid    = SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = SOM_COMMUNICATION_MAVLINK_CHANNEL;
    
    /*!< Set default*/
    heartbeat.type          = MAV_TYPE_ONBOARD_CONTROLLER; 
    heartbeat.autopilot     = MAV_AUTOPILOT_INVALID;
    heartbeat.base_mode     = 0;
    heartbeat.custom_mode   = SOM_INIT_HDMI_RESOLUTION; 
    heartbeat.system_status = MAV_STATE_ACTIVE;
    
    /*!< Message struct*/
    mavlink_message_t message_tx = {0};
    
    /*!< Encoder data to buffer*/
    mavlink_msg_heartbeat_encode_chan(  systemid,
                                        compid,
                                        chan,
                                        &message_tx,
                                        &heartbeat);
    
    uint8_t msgbuf[MAVLINK_MAX_PAYLOAD_LEN];
    uint16_t len = 0;
    
    /*!< Clear buffer*/
    memset(msgbuf, 0x00, MAVLINK_MAX_PAYLOAD_LEN);
    
    len = mavlink_msg_to_send_buffer(msgbuf, &message_tx);
    
    // Check length is valid
    if(len > 0) {
        // Send via MAVLINK_SEND_UART_BYTES 
        SomComm_SendDataFrame(msgbuf, len);
        
        DebugInfo(" Sending HB :%d", len);
    }
    else {
        DebugError("Invalid length of packet");

        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


/** @brief Function sends heartbeat to gimbal 1hz periodically 
    @param1[in] None
    @return None
*/
T_DjiReturnCode SomComm_SendLogData(uint8_t logID)
{
    /*!< Check M300 is ready and Camera is initialized completely.*/
    if(!s_flagSendLogData) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    /*!< Message struct*/
    mavlink_log_data_t log_data = {0};
    mavlink_message_t message_tx = {0};
    
    /*!< Set default information*/
    uint8_t systemid    = SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = SOM_COMMUNICATION_MAVLINK_CHANNEL;
    
    /*!< Get data */
    log_data.ofs = 0; /*<  Offset into the log*/
    log_data.id  = logID; /*<  Log id (from LOG_ENTRY reply)*/
    
    if(logID == SOM_COMM_IPADDR_REMOTE_IPADDR_LOG_ID) {
        strncpy((char*)log_data.data, s_remoteIpAddress, DJI_IP_ADDR_STR_SIZE_MAX);
        log_data.count = DJI_IP_ADDR_STR_SIZE_MAX; /*< [bytes] Number of bytes (zero for end of log)*/
        
        DebugInfo("SEND: %s", s_remoteIpAddress);
    }
    else if(logID == SOM_COMM_IPADDR_PAYLOAD_IPADDR_LOG_ID) {
        strncpy((char*)log_data.data, s_localIpAddress, DJI_IP_ADDR_STR_SIZE_MAX);
        log_data.count = DJI_IP_ADDR_STR_SIZE_MAX; /*< [bytes] Number of bytes (zero for end of log)*/
        
        DebugInfo("SEND: %s", s_localIpAddress);
    }
    else if(logID == SOM_COMM_IPADDR_PAYLOAD_NETMASK_LOG_ID) {
        strncpy((char*)log_data.data, s_configMask, DJI_IP_ADDR_STR_SIZE_MAX);
        log_data.count = DJI_IP_ADDR_STR_SIZE_MAX; /*< [bytes] Number of bytes (zero for end of log)*/
        
        DebugInfo("SEND: %s", s_configMask);
    }
    else if(logID == SOM_COMM_NETMASK_UAV_IPADDR_LOG_ID) {
        strncpy((char*)log_data.data, s_remoteIpAddress, DJI_IP_ADDR_STR_SIZE_MAX);
        log_data.count = DJI_IP_ADDR_STR_SIZE_MAX; /*< [bytes] Number of bytes (zero for end of log)*/
        
        DebugInfo("SEND: %s", s_remoteIpAddress);
    }
    else if(logID == SOM_COMM_PORT_LOG_ID) {
        strncpy((char*)log_data.data, s_remotePort, DJI_IP_ADDR_STR_SIZE_MAX);
        log_data.count = DJI_IP_ADDR_STR_SIZE_MAX; /*< [bytes] Number of bytes (zero for end of log)*/
        
        DebugInfo("SEND: %s", s_remotePort);
    }

    /*!< Encoder data to buffer*/
    mavlink_msg_log_data_encode_chan(systemid, compid, chan, &message_tx, &log_data);
    
    uint8_t msgbuf[MAVLINK_MAX_PAYLOAD_LEN];
    uint16_t len = 0;
    
    /*!< Clear buffer*/
    memset(msgbuf, 0x00, MAVLINK_MAX_PAYLOAD_LEN);
    
    len = mavlink_msg_to_send_buffer(msgbuf, &message_tx);
    
    // Check length is valid
    if(len > 0) {
        // Send via MAVLINK_SEND_UART_BYTES 
        SomComm_SendDataFrame(msgbuf, len);
        
        DebugInfo("Send Network Info: %s %d log %d", log_data.data, len, logID);
    }
    else {
        DebugError("[%s][%d]: Invalid length of packet", __FUNCTION__, __LINE__);
         
         return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Get onboard status.
 * @details This function used for getting the onboard status.
 * @note 
 */
T_DjiReturnCode SomComm_GetOnboardStatus(mavlink_onboard_computer_status_t* status)
{
    /*!< Som has been connection and send data */
    if(s_hasDetectedS5L) {
        *status = s_onboardStatus;
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
}

/**
 * @brief Get som status.
 * @details This function used for getting the onboard status.
 * @note 
 */
uint8_t SomComm_GetSomState(void)
{
    return s_somStatus;
}


/**
 * @brief Get som status.
 * @details This function used for getting the onboard status.
 * @note 
 */
T_DjiReturnCode SomComm_SetSomState(void)
{
    s_isRequestLogID = false;
    s_somStatus = 0;
    s_hasDetectedS5L = false;
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


/**
 * @brief Handle mavlink message
 * @param msg Pointer to frame to be read.
 * @param out mavlink message after parsing
 * @retval - None
 */
void SomComm_HandleMessage(const mavlink_message_t* msg)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    // Check pointer is valid 
    if(msg == NULL) {
        return;
    }
    
    // Parse message id 
    switch(msg->msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            mavlink_heartbeat_t packet;

            /* Call decode function */
            mavlink_msg_heartbeat_decode(msg, &packet);
            
            s_hasDetectedS5L = true;
            
            s_somStatus = packet.system_status;
            
            DebugInfo("Got HB SOM: %d %d ", packet.type, s_somStatus);
                      
            /* Get time report from the gimbal device*/
            osalHandler->GetTimeMs(&s_lastReportMessageMs);
            
            break;
        }
        case MAVLINK_MSG_ID_LOG_REQUEST_DATA: {
            mavlink_log_request_data_t packet = {0};

            /*!< Decode */
            mavlink_msg_log_request_data_decode(msg, &packet);
            
            s_isRequestLogID = true;
            s_logId = packet.id;
            
            DebugWarning("Som request id [%d]", s_logId);
            
            break;
        }
        
        case MAVLINK_MSG_ID_LOG_REQUEST_END: {
            
            s_isRequestLogID = false;
            DebugInfo("Got REQ IP END ");

            /*!< Stop sending message */
            break;
        }
        
        case MAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS:
        {
            mavlink_msg_onboard_computer_status_decode(msg, &s_onboardStatus);
            
            DebugInfo("Temp :%d", s_onboardStatus.temperature_board);
            break;
        }
    }
}

/** @brief      This functions use set log data message send to som
    @param[in]  *info : message information
    @param[in]  logID : ID of log
    @return     T_DjiReturnCode
*/
T_DjiReturnCode SomComm_SetLogData(const char *info, uint8_t logID)
{
    if(info != NULL && logID < SOM_COMM_NUM_TRACKED_LOG_ID) {
        if(logID == SOM_COMM_IPADDR_REMOTE_IPADDR_LOG_ID) {
            strncpy((char*)s_remoteIpAddress, info, DJI_IP_ADDR_STR_SIZE_MAX);
        }
        else if(logID == SOM_COMM_IPADDR_PAYLOAD_IPADDR_LOG_ID) {
            strncpy((char*)s_localIpAddress, info, DJI_IP_ADDR_STR_SIZE_MAX);
        }
        else if(logID == SOM_COMM_IPADDR_PAYLOAD_NETMASK_LOG_ID) {
            strncpy((char*)s_configMask, info, DJI_IP_ADDR_STR_SIZE_MAX);
        }
        else if(logID == SOM_COMM_NETMASK_UAV_IPADDR_LOG_ID) {
            strncpy((char*)s_remoteIpAddress, info, DJI_IP_ADDR_STR_SIZE_MAX);
        }
        else if(logID == SOM_COMM_PORT_LOG_ID) {
            strncpy((char*)s_remotePort, info, DJI_IP_ADDR_STR_SIZE_MAX);
            /*!< Enable for entire messages*/
            s_flagSendLogData = true;
        }
        
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
}

/** @brief      This functions use init network interface of psdk for video streams
    @param[in]  *ipAddr : payload ip address
    @param[in]  *netMask : payload net mask address
    @param[in]  *networkHandle : pointer to dji network handle
    @return     T_DjiReturnCode
*/
T_DjiReturnCode SomComm_netWorkInit(const char *ipAddr, const char *netMask, T_DjiNetworkHandle *networkHandle)
{
    USER_LOG_INFO("Network of payload need config to ipAddr:%s netMask:%s", ipAddr, netMask);
    strncpy(s_networkLocalIpAddress, ipAddr, sizeof(s_networkLocalIpAddress));
    strncpy(s_networkConfigMask, netMask, sizeof(s_networkLocalIpAddress));

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/** @brief      This functions use deinit network interface of psdk for video streams
    @param[in]  networkHandle : pointer to dji network handle
    @return     T_DjiReturnCode
*/
T_DjiReturnCode SomComm_netWorkDeInit(T_DjiNetworkHandle networkHandle)
{
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode SomComm_networkGetDeviceInfo(T_DjiHalNetworkDeviceInfo *deviceInfo)
{
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/** @brief      This functions use set drone and payload network to som
    @param[in]  none
    @return     none
*/
static void SomComm_setPayLoadNetWorkIP(void)
{
    T_DjiReturnCode returnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    bool isCamEmu_initDone = false;
    char ipAddr[DJI_IP_ADDR_STR_SIZE_MAX] = {0};
    uint16_t port = 0;
    
    isCamEmu_initDone = DjiTest_CameraIsInited();
    
    if(isCamEmu_initDone == true)
    {
        returnCode = DjiPayloadCamera_GetVideoStreamRemoteAddress(ipAddr, &port);
        if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                DebugInfo("get video stream remote address:%s_%d", ipAddr, port);
        } else {
                DebugError("get video stream remote address error.");
        }
        
        SomComm_SetLogData((char*)ipAddr, SOM_COMM_IPADDR_REMOTE_IPADDR_LOG_ID);
        SomComm_SetLogData((char*)s_networkLocalIpAddress, SOM_COMM_IPADDR_PAYLOAD_IPADDR_LOG_ID);
        SomComm_SetLogData((char*)s_networkConfigMask, SOM_COMM_IPADDR_PAYLOAD_NETMASK_LOG_ID);

        char portBuf[2];
        sprintf(portBuf, "%d", port);
        SomComm_SetLogData((char*)portBuf, SOM_COMM_PORT_LOG_ID);
    }
}

#endif
/**
    @}
*/

/** @group __SOM_COMMUNICATION_TASKS_
    @{
*/#ifndef __SOM_COMMUNICATION_TASKS_
#define __SOM_COMMUNICATION_TASKS_
/**
  * @brief SomComm_RecvTask
  * Task gimbal will interface with the gimbal by conveying gimbal data 
  * to app as well as gimbal command to gimbal devixe
  */
static void *SomComm_RecvTask(void *arg)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    //  Counts of bytes read actually.
    int realLen = 0;
    
    // receive new packets
    mavlink_message_t   msg;
    mavlink_status_t    status;
    status.packet_rx_drop_count = 0;
    
    USER_UTIL_UNUSED(arg);
    
    for(;;)
    {
        // Read data from the ring buffer
        realLen = UART_Read(SOM_COMMUNICATION_UART_NUM, s_somCommRecBuf, sizeof(s_somCommRecBuf));

        if(realLen > 0) {
            // Scan the counts of bytes read actually.
            for (int i = 0; i < realLen; i++)
            {
                // Get data from buffer
                uint8_t c = s_somCommRecBuf[i];
                
                //===================== MAVLINK Recievie Function ==============================//            
                //// Try to get a new message
                if(mavlink_parse_char(SOM_COMMUNICATION_MAVLINK_CHANNEL, c, &msg, &status)) {
                    // Handle message here
                    SomComm_HandleMessage(&msg);
                }
            }
        }
        
        osalHandler->TaskSleepMs(1);
    }
}

/**
  * @brief SomComm_RecvTask
  * Task gimbal will interface with the gimbal by conveying gimbal data 
  * to app as well as gimbal command to gimbal devixe
  */
static void *SomComm_ProcessTask(void *arg)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    static uint32_t s_taskSomComm_working = 0;
    static uint32_t currentTime = 0;
    T_DjiReturnCode    returnCode;
    
    USER_UTIL_UNUSED(arg);
    
    /*!< Reset time */
    osalHandler->GetTimeMs(&s_lastReportMessageMs);
    
    for(;;)
    {
        osalHandler->TaskSleepMs(1000 / SOM_COMMUNICATION_TASK_FREQ);
        s_taskSomComm_working++;
        
        /*!< Sending hearbeat periodically 1hz*/
         if (USER_UTIL_IS_WORK_TURN(s_taskSomComm_working, 1, SOM_COMMUNICATION_TASK_FREQ)) {
         
            if(s_hasDetectedS5L) {
                // Quick Send heartbeat for testing
                SomComm_SendHeatbeat();
            }
         }
         if (USER_UTIL_IS_WORK_TURN(s_taskSomComm_working, 5, SOM_COMMUNICATION_TASK_FREQ)) {
            
             /*!< Get current time */
            returnCode = osalHandler->GetTimeMs(&currentTime);
            if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                DebugError("Get start time error: 0x%08llX.", returnCode);
            }
    
            /* PHUB has lost connection with the gimbal device*/
            if(currentTime - s_lastReportMessageMs > 15000) {
                /*!< Lost connection */
                s_hasDetectedS5L = false;
                
                /*!< Reset log request IP*/
                s_isRequestLogID = false;
                
                if(s_somStatus != SOM_FLAHSING) {
                    PsdkSom_WriteHardwareResetPin(SOM_HW_RESET_MANAGEMENT_PIN_STATE_RESET);
                    osalHandler->TaskSleepMs(20);
                    PsdkSom_WriteHardwareResetPin(SOM_HW_RESET_MANAGEMENT_PIN_STATE_SET);
                    osalHandler->TaskSleepMs(20); 
                    
                    SomComm_SetSomState();
                    
                    DebugWarning("SoM Reboot");
                }
                
                /*!< Reset time */
                osalHandler->GetTimeMs(&s_lastReportMessageMs);
            }
            else {
                if(s_isRequestLogID && DjiTest_CameraIsInited()) {
                    SomComm_SendLogData(s_logId);
                }
            }
        }
         
        /// setting network drone and payload to som
        SomComm_setPayLoadNetWorkIP();
    }
}



#endif
/**
    @}
*/

/** @group __SOM_COMMUNICATION_CONFIGURATION_
    @{
*/#ifndef __SOM_COMMUNICATION_CONFIGURATION_
#define __SOM_COMMUNICATION_CONFIGURATION_
/**
  * @brief gimbal_interface_init
  * Init hardware to communication with the gimbal device
  */
T_DjiReturnCode SomComm_Init(void)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    // PIN_C6 PIN_C7
    /*!< Init hardware serial */
    UART_Init(SOM_COMMUNICATION_UART_NUM, SOM_COMMUNICATION_UART_BAUD);
    
    /*!< Init mavlink channel and assign function for sending message*/
    s_somCommHandler.chan    = SOM_COMMUNICATION_MAVLINK_CHANNEL;
    s_somCommHandler.write   = SomComm_SendDataFrame;
    
    /*!< Reset flag */
    s_flagSendLogData = false;
    
    /*!< Init mavlink function handler */
    gremsy_mavlink_init(&s_somCommHandler);
    
    if (PsdkSom_HardwareResetPinInit() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("psdk som hardware reset init error");
    }
    
    /*!< Create mutex for managing the access function list when sending*/
    if(osalHandler->MutexCreate(&s_somMutexSend) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("Create Mutex error\n");
    }

    /*!< Create Gimbal process data task*/
    if(osalHandler->TaskCreate("SomComm_recv", SomComm_RecvTask
        , SOM_COMMUNICATION_TASK_STACK_SIZE, NULL, &s_somRecvThreadId) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("Create task som communication failed ");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
        
    /*!< Create Gimbal process data task*/
    if(osalHandler->TaskCreate("SomComm_process", SomComm_ProcessTask
        , SOM_COMMUNICATION_TASK_STACK_SIZE, NULL, &s_somProcessThreadId) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("Create task som communication failed ");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
        
    PsdkSom_WriteHardwareResetPin(SOM_HW_RESET_MANAGEMENT_PIN_STATE_RESET);
    osalHandler->TaskSleepMs(20);
    PsdkSom_WriteHardwareResetPin(SOM_HW_RESET_MANAGEMENT_PIN_STATE_SET);
    osalHandler->TaskSleepMs(20); 
        
    USER_LOG_INFO("Start Som Communication !!!");
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

#endif
/**
    @}
*/

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/
