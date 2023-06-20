/*******************************************************************************
 * Copyright (c) 2020, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 *
 * @file    gremsy_flir_camera.c
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    Aug-27-2020
 * @brief   This file contains lower function for interfacing with FLIR camera
 *
 * -----------------------------------------------------------------------------
 * MSX (Multi Spectral Dynamic Imaging). It provides extraordinary in real-time thermal images
 * by embossing visible camera image information on thermal video and stills.
 * MSX produes exceptional thermal clarity and ensures easier taget indentification without compromising
 * radiometric data. 
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 ******************************************************************************/
/* Private includes ----------------------------------------------------------*/
/*!< DJI library */

/*!< Flir includes file*/
#include "camera/gsdk_flir_camera.h"
/*!< Include mavlink library */
#include "mavlink/gsdk_mavlink.h"
#include "som/som_communication.h"

#include "dji_platform.h"
#include "utils/util_misc.h"
#include "dji_logger.h"
#include "osal.h"
#include "uart.h"
#include "eeprom.h"

#include <ctype.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/*!< Get data for geotagging */
//#include "test_fc_subscription.h"

#include <flash_if.h>

/* Private constants ---------------------------------------------------------*/
/*!< Supported PixyF*/
#define GIMBAL_PIXYF
#define SLOG_CAMERA     STD_ON

#if SLOG_CAMERA
# define DebugMsg(fmt, args ...) do {USER_LOG_DEBUG("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);
# define DebugInfo(fmt, args ...) do {USER_LOG_INFO("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);
# define DebugWarning(fmt, args ...) do {USER_LOG_WARN("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);
#else
# define DebugMsg(fmt, args ...)
# define DebugInfo(fmt, args ...)
# define DebugWarning(fmt, args ...)
#endif

# define DebugError(fmt, args ...) do {USER_LOG_ERROR("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);

/* Private define ------------------------------------------------------------*/
#ifdef GIMBAL_PIXYU
/*!< Define Hardware Interface PIXY U*/
#define FLIR_CAMERA_TASK_FREQ                   (1000)
#define FLIR_CAMERA_TASK_STACK_SIZE             (1024)


#define PSDK_COMM_WITH_CAMERA_UART_NUM          UART_NUM_4
#define PSDK_COMM_WITH_CAMERA_UART_BAUD         57600
#define PSDK_COMM_WITH_CAMERA_BUFFER_SIZE       (255)
#else
/*!< Define Hardware Interface PIXY F*/
#define FLIR_CAMERA_TASK_FREQ                   (1000)
#define FLIR_CAMERA_TASK_STACK_SIZE             (2048) /*!< BUG: = 1024 FAILED stack size*/

#define PSDK_COMM_WITH_CAMERA_UART_NUM          UART_NUM_5
#define PSDK_COMM_WITH_CAMERA_UART_BAUD         57600
#define PSDK_COMM_WITH_CAMERA_BUFFER_SIZE       (255)
#endif

#define PSDK_COMM_WITH_CAMERA_MAV_CHAN          MAVLINK_COMM_1
/* Define ID interface with the camera */
#define PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID      1

#define PSDK_COMM_WAIT_RESPOND_TIMEOUT          6000 /*!< Unit: ms*/
#define PSDK_COMM_WAIT_ACK_TIMEOUT              1000 /*!< Unit: ms*/
/* Private typedef -----------------------------------------------------------*/

typedef struct {
    int32_t VERSION_X;
    int32_t VERSION_Y;
    int32_t VERSION_Z;
    int32_t CAM_ID;
    int32_t VIDEO_TRANSMISSION;
    int32_t TRANSMISSION_MODE;
    int32_t TYPE_RECORDING;
    int32_t SENCE_MODE;
    int32_t COLOR_PALETTE;
    int32_t APERTURE;
    int32_t ISO_NUM;
    int32_t ROTATION_IMAGE;
    int32_t MSX_ENABLE;
    int32_t MSX_LENGTH;
    int32_t FILE_FORMAT;
    int32_t VIDEO_FILE_TYPE;
    int32_t TEMP_UNIT;
    int32_t TEMP_METER;
    int32_t SUBJECT_EMISSIVITY;
    int32_t SKY_CONDITION;
    int32_t AIR_TEMPERATURE;
    int32_t HUMIDITY;
    int32_t SUBJECT_RANGE;
    int32_t NUM_TRACK;
} T_FlirParamters;

/*!< Default parameters */
static T_FlirParamters s_flirParams = {
                1,
                0,
                0,
                100,
                FLIR_VIDEO_TRANSMISSION_VIS,    
                0,
                FLIR_FILE_FORMART_FFF,
                FLIR_SCENE_OUTDOOR,
                FLIR_PATETTE_LAVA,
                0,
                0,
                FLIR_ROTARY_0,
                0,
                50,
                FLIR_FILE_FORMART_JPEG_TIFF,
                FLIR_VIDEO_FILE_TYPE_H264,
                FLIR_TEMP_UNIT_C,
                FLIR_SPOT_METER_OFF,
                50,
                FLIR_CLEAR_SKIES,
                32,
                HUMIDITY_MEDIUM_45,
                200,
                100,
};


/**
 * @brief Structure contain all information related to FLIR camera 
 * @details This structure type is used to
 * @note 
 */
typedef struct _T_FLIRHandle
{
    uint32_t            lastTimeConnection;
    bool                isCameraConnected;
    bool                isSOMReseted;
    
    mavlink_message_t   msg;
    mavlink_status_t    status;
    /*!< Taget system and commponet ID*/
    uint8_t             targetSystem;
    uint8_t             targetComponent;
    
    /*!< Define the handler for mavlink camera */
    gremsy_mavlink_handler_t mavHandler;
    
    /*!< Ping Sequence */
    uint32_t                pingSeq;
    
    /*!< Type Request Data Stream */
    uint32_t                isReqDataStream;
    uint16_t                msgRate;
    
    /*!< ACK*/
    uint16_t                waitCommandAck;
    T_DjiMutexHandle        mutexSendGetAck;
    T_DjiSemaHandle         semaphoreWaitAck;
    
} T_FLIRHandle;

/*!< Process state for camera */
typedef enum {
    CAMERA_RUNNING_STATE_IDLE = 0,
    
    CAMERA_RUNNING_STATE_CHECK_CONNECTION,
    
    CAMERA_RUNNING_STATE_RESET_SOM,
    
    CAMERA_RUNNING_STATE_DEFAULT_SETTING,
    
    CAMERA_RUNNING_QUERY_CAMERA_SETTING,
    
    CAMERA_RUNNING_STATE_PROCESS_EVENT,
    
    CAMERA_RUNNING_STATE_ERROR,
} E_CameraRunningState;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*!< Structure contain FLIR handler */
static T_FLIRHandle *s_pFLIRHandle = NULL;

/*!< Global FLIR CAMERA */
static T_FLIRCamera s_FLIRCamera;
T_FLIRCamera * const FLIRCamera = &s_FLIRCamera;


/*!< OS Vairables */
/*!<   buffer used to receive data from UART interface */
static uint8_t                      s_uartRecBuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE];

/*!< Task handler ID for processing wiris protocol*/
static T_DjiTaskHandle             s_recvThread;

/*!< Task handler ID for processing user application*/
static T_DjiTaskHandle             s_processCmdThread;

/*!< Event update for callback request*/
static EventGroupHandle_t           s_eventCamera;

/*!< Camera running status */
E_CameraRunningState                s_cameraRunningState = CAMERA_RUNNING_STATE_IDLE;

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief Function initialize FLIR camera protocol 
 * @details This structure type is used to
 * @note 
 */
static T_DjiReturnCode FLIRProto_CameraInit(void);

/**
  * @brief Function is used to initialize the wiris protocol
  * @retval T_DjiReturnCode
  */
static T_DjiReturnCode FlirCameraProto_DeInit(void);

/**
  * @brief Function is used to register the function send 
  * @retval T_DjiReturnCode
  */
static T_DjiReturnCode FlirCameraProto_RegSendDataFunc(SendCallbackFunc callbackFunc);

/**
  * @brief Function is used to register the function send 
  * @retval T_DjiReturnCode
  */
static T_DjiReturnCode FlirCameraProto_RegRecvDataFunc(ReceiveCallbackFunc callbackFunc);

/**
  * @brief Function is used to register the function send 
  * @retval T_DjiReturnCode
  */
static T_DjiReturnCode FlirProt_ProcessReceiveData(const uint8_t *pData, uint16_t realLen);

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief Function set param for FLIR DUO PRO R 
 * @details This function only called when the power off
 * @note 
 */
T_DjiReturnCode GsdkFlir_SetParams(const T_FlirParamters *params)
{
    uint32_t result;

    result = FLASH_If_Erase(CAMERA_PARAM_STORE_ADDRESS, CAMERA_PARAM_STORE_ADDRESS_END);
    if (result != FLASHIF_OK) {
        
        DebugWarning("FLASH ERASE FAILED");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    
    result = FLASH_If_Write(CAMERA_PARAM_STORE_ADDRESS, (uint8_t *) params,
                            sizeof(T_FlirParamters));
    if (result != FLASHIF_OK) {
        DebugWarning("FLASH WRITE FAILED");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Function used to get param in flash only called when the first time initialize 
 * @details 
 * @note 
 */
T_DjiReturnCode GsdkFlir_GetParams(T_FlirParamters *params)
{
    T_DjiReturnCode psdkStat  = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    
    *params = *(T_FlirParamters *) (CAMERA_PARAM_STORE_ADDRESS);
    
    if((uint32_t)params->VERSION_X != 1 && params->NUM_TRACK != 100) {
        
        psdkStat = GsdkFlir_SetParams(&s_flirParams);
        
        DebugWarning("FLIR Set Default Params %d ", psdkStat);
    }
    
    DebugWarning("Version: %d:%d", (uint32_t)params->VERSION_X, (uint32_t)params->NUM_TRACK);

    return psdkStat;
}


/*!<====================== MESSAGE_CONTROL ===================================*/

/**
 * @brief Function check FLIR is connected to the system
 * @details 
 * @note 
 */
static bool GremsyFLIR_isConnected(void )
{
    uint32_t tnow_ms = 0;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
   /*!< Get time */
    osalHandler->GetTimeMs(&tnow_ms);

    /*!< Calculate the delta time */
    if(tnow_ms - s_pFLIRHandle->lastTimeConnection > PSDK_COMM_WAIT_RESPOND_TIMEOUT || !s_pFLIRHandle->isCameraConnected) {
        
        s_pFLIRHandle->isCameraConnected = false;
        
        DebugWarning("Flir is not connected!");
        
        return false;
    } 
    
    return true;
}

/**
 * @brief
 * @details This structure type is used to
 * @note Camera R/S (Receive/Send)  
 */
T_DjiReturnCode GremsyFLIR_SendHeartbeat(void)
{
    T_DjiReturnCode psdkStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    /*!< Lock mutex to send data */
    psdkStat = osalHandler->MutexLock(s_pFLIRHandle->mutexSendGetAck);
    if( psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("lock mutexSendGetAck error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    mavlink_heartbeat_t heartbeat;
    
    /*!< Set default information*/
    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;
    
    /*!< Set default*/
    heartbeat.type          = MAV_TYPE_GENERIC; 
    heartbeat.autopilot     = MAV_AUTOPILOT_INVALID;
    heartbeat.base_mode     = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    heartbeat.custom_mode   = 0; 
    heartbeat.system_status = MAV_STATE_ACTIVE;
    
    /*!< Message struct*/
    mavlink_message_t message_tx = {0};
    
    /*!< Encoder data to buffer*/
    mavlink_msg_heartbeat_encode_chan(  systemid, compid, chan, &message_tx, &heartbeat);
    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE];
    uint16_t len = 0;
    
    /*!< Clear buffer*/
    memset(msgbuf, 0x00, PSDK_COMM_WITH_CAMERA_BUFFER_SIZE);
    
    len = mavlink_msg_to_send_buffer(msgbuf, &message_tx);
    
    // Check length is valid
    if(len > 0) {
        // Send via MAVLINK_SEND_UART_BYTES 
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    }
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    psdkStat = osalHandler->MutexUnlock(s_pFLIRHandle->mutexSendGetAck);
    if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("unlock mutexSendGetAck error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief
 * @details This structure type is used to
 * @note R/S If PING is received, then will reply with a PING
 */
T_DjiReturnCode GremsyFLIR_SendPing(void)
{
    if(!s_pFLIRHandle->isCameraConnected) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    mavlink_message_t           msg = {0};
    uint16_t                    len = 0;
    mavlink_ping_t              ping = {0};

    /*< [us] Timestamp (UNIX Epoch time or time since system boot). 
    The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.*/
    
    /*<  PING sequence*/
    /*<  0: request ping from all receiving systems, 
    if greater than 0: message is a ping response and number is the system id of the requesting system*/
    /*<  0: request ping from all receiving components, 
    if greater than 0: message is a ping response and number is the system id of the requesting system*/
    
    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;
    
    ping.time_usec = xTaskGetTickCount() / 1000;
    ping.seq = s_pFLIRHandle->pingSeq++;
    ping.target_system     = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    ping.target_component  = MAV_COMP_ID_CAMERA;

    mavlink_msg_ping_encode_chan(   systemid,
                                    compid,
                                    chan,
                                    &msg,
                                    &ping);

    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE] = {0};
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief System Master clock time 
 * @details 
 * @note 
 */
T_DjiReturnCode GremsyFLIR_SendSystemTime(void)
{
    if(!s_pFLIRHandle->isCameraConnected) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    mavlink_message_t           msg = {0};
    uint16_t                    len = 0;
    mavlink_system_time_t       systemTime = {0};
    
    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;
    
    /*< [us] Timestamp (UNIX epoch time).*/
    systemTime.time_unix_usec = xTaskGetTickCount()/1000;
    /*< [ms] Timestamp (time since system boot).*/
    systemTime.time_boot_ms = 0;
 
    mavlink_msg_system_time_encode_chan(systemid, compid, chan, &msg, &systemTime);

    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE] = {0};
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief The attitude in the aeronautical frame
 * @details 
 * @note 
 */
T_DjiReturnCode GremsyFLIR_SendAttitude(void)
{
    if(!s_pFLIRHandle->isCameraConnected) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    mavlink_message_t           msg = {0};
    uint16_t                    len = 0;
    mavlink_attitude_t          attitude = {0};
    
    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;

    /*< [ms] Timestamp (time since system boot).*/
    attitude.time_boot_ms = xTaskGetTickCount(); 
    /*< [rad] Roll angle (-pi..+pi)*/
    attitude.roll = 0; 
    /*< [rad] Pitch angle (-pi..+pi)*/
    attitude.pitch = 0; 
    /*< [rad] Yaw angle (-pi..+pi)*/
    attitude.yaw = 0; 
    /*< [rad/s] Roll angular speed*/
    attitude.rollspeed = 0; 
    /*< [rad/s] Pitch angular speed*/
    attitude.pitchspeed = 0;
    /*< [rad/s] Yaw angular speed*/
    attitude.yawspeed = 0; 
    
    mavlink_msg_attitude_encode_chan(systemid, compid, chan, &msg, &attitude);

    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE] = {0};
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Filtered GPS position
 * @details This structure type is used to
 * @note 
 */
T_DjiReturnCode GremsyFLIR_SendGlobalPositionInit(void)
{
    T_DjiReturnCode psdkStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    if(!s_pFLIRHandle->isReqDataStream) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    /*!< Lock mutex to send data */
    psdkStat = osalHandler->MutexLock(s_pFLIRHandle->mutexSendGetAck);
    if( psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("lock mutexSendGetAck error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    mavlink_message_t           msg = {0};
    uint16_t                    len = 0;
    mavlink_global_position_int_t          globalPosInit = {0};
    
    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;

    /*!< Longitude, unit: deg. */
    /*!< Latitude, unit: deg. */
    /*!< Height above mean sea level, unit: m. */
    
    /*< [ms] Timestamp (time since system boot).*/
    osalHandler->GetTimeMs(&globalPosInit.time_boot_ms);
    /*< [degE7] Latitude, expressed*/
    globalPosInit.lat = geoTagInfo.rtkPosition.latitude * 1E7; 
    /*< [degE7] Longitude, expressed*/
    globalPosInit.lon = geoTagInfo.rtkPosition.longitude * 1E7; 
    /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
    globalPosInit.alt = geoTagInfo.rtkPosition.hfsl * 1000;
    /*< [mm] Altitude above ground*/
    globalPosInit.relative_alt = 0; 
    /*< [cm/s] Ground X Speed (Latitude, positive north)*/
    globalPosInit.vx = geoTagInfo.rtkVelocity.x * 10000; /*!<  unit: cm/s.*/
    /*< [cm/s] Ground Y Speed (Longitude, positive east)*/
    globalPosInit.vy = geoTagInfo.rtkVelocity.y * 10000; /*!<  unit: cm/s.*/
    /*< [cm/s] Ground Z Speed (Altitude, positive down)*/
    globalPosInit.vz = geoTagInfo.rtkVelocity.z * 10000; /*!<  unit: cm/s.*/
    /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
    globalPosInit.hdg = geoTagInfo.rtkYaw;
    
    /*!< Encode to buffer */
    mavlink_msg_global_position_int_encode_chan(systemid, compid, chan, &msg, &globalPosInit);

    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE] = {0};
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    psdkStat = osalHandler->MutexUnlock(s_pFLIRHandle->mutexSendGetAck);
    if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("unlock mutexSendGetAck error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Higher resolution filter GPS position
 * @details 
 * @note 
 */
T_DjiReturnCode GremsyFLIR_SendGlobalPositionInitCov(void)
{
    T_DjiReturnCode psdkStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    if(!s_pFLIRHandle->isCameraConnected) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    /*!< Lock mutex to send data */
    psdkStat = osalHandler->MutexLock(s_pFLIRHandle->mutexSendGetAck);
    if( psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("lock mutexSendGetAck error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    mavlink_message_t                   msg = {0};
    uint16_t                            len = 0;
    mavlink_global_position_int_cov_t   globalPosInitCov = {0};
    
    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;

     /*< [us] Timestamp (UNIX Epoch time or time since system boot). 
    The receiving end can infer timestamp format (since 1.1.1970 or since system boot) 
    by checking for the magnitude of the number.*/
    globalPosInitCov.time_usec = 0;
    /*< [degE7] Latitude*/
    globalPosInitCov.lat = 0; 
    /*< [degE7] Longitude*/
    globalPosInitCov.lon = 0; 
    /*< [mm] Altitude in meters above MSL*/
    globalPosInitCov.alt = 0; 
    /*< [mm] Altitude above ground*/
    globalPosInitCov.relative_alt = 0; 
    /*< [m/s] Ground X Speed (Latitude)*/
    globalPosInitCov.vx = 0; 
    /*< [m/s] Ground Y Speed (Longitude)*/
    globalPosInitCov.vy = 0; 
    /*< [m/s] Ground Z Speed (Altitude)*/
    globalPosInitCov.vz = 0; 
    /*<  Row-major representation of a 6x6 position and velocity 6x6 cross-covariance matrix 
    (states: lat, lon, alt, vx, vy, vz; first six entries are the first ROW, next six entries are the second row, etc.). 
    If unknown, assign NaN value to first element in the array.*/
//    globalPosInitCov.covariance[36] = {0};
    /*<  Class id of the estimator this estimate originated from.*/
    globalPosInitCov.estimator_type = 0; 
 
    /*!< Encode to buffer */
    mavlink_msg_global_position_int_cov_encode_chan(   systemid,
                                                        compid,
                                                        chan,
                                                        &msg,
                                                        &globalPosInitCov);

    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE] = {0};
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    psdkStat = osalHandler->MutexUnlock(s_pFLIRHandle->mutexSendGetAck);
    if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("unlock mutexSendGetAck error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Raw GPS from sensor 
 * @details 
 * @note 
 */
T_DjiReturnCode GremsyFLIR_SendHilGPS(void)
{
    if(!s_pFLIRHandle->isCameraConnected) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    mavlink_message_t           msg = {0};
    uint16_t                    len = 0;
    mavlink_hil_gps_t           hil_gps = {0};
    
    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;

    /*< [us] Timestamp (UNIX Epoch time or time since system boot). 
    The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
    hil_gps.time_usec = 0; 
    /*< [degE7] Latitude (WGS84)*/
    hil_gps.lat = 0; 
    /*< [degE7] Longitude (WGS84)*/
    hil_gps.lon = 0;
    /*< [mm] Altitude (MSL). Positive for up.*/
    hil_gps.alt = 0; 
    /*< [cm] GPS HDOP horizontal dilution of position. If unknown, set to: 65535*/
    hil_gps.eph = 0; 
    /*< [cm] GPS VDOP vertical dilution of position. If unknown, set to: 65535*/
    hil_gps.epv = 0;
    /*< [cm/s] GPS ground speed. If unknown, set to: 65535*/    
    hil_gps.vel = 0;
    /*< [cm/s] GPS velocity in north direction in earth-fixed NED frame*/    
    hil_gps.vn = 0;
    /*< [cm/s] GPS velocity in east direction in earth-fixed NED frame*/
    hil_gps.ve = 0; 
    /*< [cm/s] GPS velocity in down direction in earth-fixed NED frame*/
    hil_gps.vd = 0; 
    /*< [cdeg] Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to: 65535*/
    hil_gps.cog = 0; 
    /*<  0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.*/
    hil_gps.fix_type = 0; 
    /*<  Number of satellites visible. If unknown, set to 255*/
    hil_gps.satellites_visible = 0; 
    /*<  GPS ID (zero indexed). Used for multiple GPS inputs*/
    hil_gps.id = 0; 
    /*< [cdeg] Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north*/
    hil_gps.yaw = 0;
    
    mavlink_msg_hil_gps_encode_chan(systemid, compid, chan, &msg, &hil_gps);

    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE] = {0};
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Orientation of gimbal 
 * @details This structure type is used to
 * @note 
 */
T_DjiReturnCode GremsyFLIR_SendMountStatus(const T_DjiAttitude3d *attitude)
{
    if(!s_pFLIRHandle->isCameraConnected) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    mavlink_message_t           msg = {0};
    uint16_t                    len = 0;
    mavlink_mount_status_t      mountStatus = {0};
    
    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;

    /*< [cdeg] Pitch.*/
    mountStatus.pointing_a = attitude->pitch*100; 
    /*< [cdeg] Roll.*/
    mountStatus.pointing_b = attitude->roll*100; 
    /*< [cdeg] Yaw.*/
    mountStatus.pointing_c = attitude->yaw*100; 
    
    DebugError("[%d][%d] ", mountStatus.pointing_a, mountStatus.pointing_c);
    
    /*<  System ID.*/
    mountStatus.target_system = 0; 
    /*<  Component ID.*/
    mountStatus.target_component = 0; 
        
    mavlink_msg_mount_status_encode_chan(   systemid,
                                            compid,
                                            chan,
                                            &msg,
                                            &mountStatus);

    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE] = {0};
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


/**
 * @brief Function set layout IR/VIS/VIS_IR
 * @details This structure type is used to
 * @note 
 */
T_DjiReturnCode GremsyFLIR_SendDoControlVideo(uint8_t trans)
{
    T_DjiReturnCode psdkStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    if(!s_pFLIRHandle->isCameraConnected) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    /*!< Lock mutex to send data */
    psdkStat = osalHandler->MutexLock(s_pFLIRHandle->mutexSendGetAck);
    if( psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("lock mutexSendGetAck error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    mavlink_message_t           msg = {0};
    uint16_t                    len = 0;
    mavlink_command_long_t      command_long = {0};

    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;
    
    /// control video
    command_long.command = MAV_CMD_DO_CONTROL_VIDEO;
    command_long.param1 = MAV_COMP_ID_CAMERA; 
    command_long.param2 = trans;                                    /// Transmission(output hdmi).    0: IR, 1: VIS, 2: VIS with IR(nho)
    command_long.param3 = 0;
    command_long.param4 = FLIR_VIDEO_RECORDING_IR_VIS;              /// Recording.                    0: IR, 1: VIS, 2: IR and VIS
    command_long.param5 = 0;
    command_long.param6 = 0;
    command_long.param7 = 0;
    
    command_long.confirmation      = 0;
    command_long.target_component  = MAV_COMP_ID_CAMERA;
    command_long.target_system     = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    
    mavlink_msg_command_long_encode_chan(   systemid,
                                            compid,
                                            chan,
                                            &msg,
                                            &command_long);

    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE] = {0};
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    /*!< Get command ACK and Wait Feedback*/
    s_pFLIRHandle->waitCommandAck = command_long.command;
    
    /*!< Waiting semaphore from command ACK */
    psdkStat = osalHandler->SemaphoreTimedWait(s_pFLIRHandle->semaphoreWaitAck, PSDK_COMM_WAIT_ACK_TIMEOUT);
    if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        
        DebugError("Wait semaphore flir error: 0x%08llX.", psdkStat);
        
    } else {
        /*!< Get video transmission has been set previously */
//        FLIRCamera->videoTransmission = trans;
    }
    
    psdkStat = osalHandler->MutexUnlock(s_pFLIRHandle->mutexSendGetAck);
    if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("unlock mutexSendGetAck error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/** @brief ham dung de cai dat thong so cho camera
    @param[in] scene chon sence loai moi truong hoat dong
    @param[in] palette chon mau hien thi cho camera IR
    @param[in] rotary chon do xoay cua hinh anh (0, 180)
    @param[in] Set MSX function 
    @param[in] Set MSX length 0 thr 100 inclusive
    @return none
*/
T_DjiReturnCode GremsyFLIR_SendDoDigicamConfigure( uint8_t scene, uint8_t palette, uint8_t rotation, uint8_t MSX_enable, uint8_t MSX_length)
{
    T_DjiReturnCode psdkStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    if(!s_pFLIRHandle->isCameraConnected) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    /*!< Lock mutex to send data */
    psdkStat = osalHandler->MutexLock(s_pFLIRHandle->mutexSendGetAck);
    if( psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("lock mutexSendGetAck error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    mavlink_message_t           msg = {0};
    uint16_t                    len = 0;
    mavlink_command_long_t      command_long = {0};

    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;
    
        /// set param
    command_long.command = MAV_CMD_DO_DIGICAM_CONFIGURE;
    command_long.param1 = scene;        /// set scene       0: L, 1: D, 2: S, 3: O, 4: I, 5: M, 6: C1, 7: C2, 
    command_long.param2 = palette;      /// set IR Palette  0: White Hot, 1: Black Hot, 2: Red Hot, 3: Rainbow, 
                                        ///                 4: INVALID, 5: Lava, 6:  Arctic, 7: Globow, 8: Fusion
                                        ///                 9: InstAlert, 10: Grey Red, 11: Sepia, 12: INVALID, 13: INVALID(YELLOW & BLUE)
                                        ///                 14: Green Hot, 15: White Hot Iso, 16: Black Hot Iso, 17: Fusion Iso
                                        ///                 18: Rainbow Iso, 19: Globow Iso, 20: Ironbow White Hot Iso, 21: Ironbow Black Hot Iso,
                                        ///                 22: Sepia Iso, 23: Midrange White Hot Iso, 24: Midrange Black Hot Iso, 25: Ice Fire Iso,
                                        ///                 26: Rainbow HC Iso, 27: Red Hot Iso, 28: Green Hot Iso, 29: Arctic Black Hot Iso,
    command_long.param3 = 0;
    command_long.param4 = 0;
    command_long.param5 = rotation;     /// rotary image        0: Normal,  1: Inverted
    command_long.param6 = MSX_enable;            /// set MSX Function    0: Disable, 1: Enable
    command_long.param7 = MSX_length;            /// Main engine cut-off time before camretrigger in seconds/100 (0 means no cut-off). 0 to 100, inclusive
    
    command_long.confirmation      = 0;
    command_long.target_component  = MAV_COMP_ID_CAMERA;
    command_long.target_system     = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    
    mavlink_msg_command_long_encode_chan(   systemid,
                                            compid,
                                            chan,
                                            &msg,
                                            &command_long);

    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE] = {0};
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    /*!< Get command ACK and Wait Feedback*/
    s_pFLIRHandle->waitCommandAck = command_long.command;
    
   /*!< Waiting semaphore from command ACK */
    psdkStat = osalHandler->SemaphoreTimedWait(s_pFLIRHandle->semaphoreWaitAck, PSDK_COMM_WAIT_ACK_TIMEOUT);
    if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        
        DebugError("Wait semaphore flir error: 0x%08llX.", psdkStat);
    } else {
        /*!< Get Setting has been set previously */
        FLIRCamera->scene         = scene;
        FLIRCamera->palette       = palette;
        FLIRCamera->imageRotation = rotation;
        FLIRCamera->isMSXEnable   = MSX_enable;
        FLIRCamera->MSXLength     = MSX_length;
    }
    
    /*!< Clear command after processing done */
    s_pFLIRHandle->waitCommandAck = 0;
    
    /*!< Unlock Mutex */
    psdkStat = osalHandler->MutexUnlock(s_pFLIRHandle->mutexSendGetAck);
    if (psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("unlock mutex error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
        
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/** @brief Control on-board camera 
    @param[in] Command FFC 
    @param[in] scene chon sence loai moi truong hoat dong
    @param[in] palette chon mau hien thi cho camera IR
    @param[in] rotary chon do xoay cua hinh anh (0, 180)
    @return none
*/
T_DjiReturnCode GremsyFLIR_SendDoDigicamControl( float session,
                                                  float zoom_pos,
                                                  float zoom_step,
                                                  float focus_lock,
                                                  float shot_cmd,
                                                  float command_id,
                                                  float shot_id)
{
    T_DjiReturnCode psdkStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    if(!s_pFLIRHandle->isCameraConnected) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    /*!< Lock mutex to send data */
    psdkStat = osalHandler->MutexLock(s_pFLIRHandle->mutexSendGetAck);
    if( psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("lock mutexSendGetAck error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    mavlink_message_t           msg = {0};
    uint16_t                    len = 0;
    mavlink_command_long_t      command_long = {0};

    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;
    
    
     /// set param
    command_long.command = MAV_CMD_DO_DIGICAM_CONTROL;
    command_long.param1 = session;     /// Session control e.g. show/hide lens
    command_long.param2 = zoom_pos;    /// Zoom's absolute position
    command_long.param3 = zoom_step;   /// Zooming step value to offset zoom from the current position        
    command_long.param4 = focus_lock;  /// Focus Locking, Unlocking or Re-locking        
    command_long.param5 = shot_cmd;    /// Shooting Command
    command_long.param6 = command_id;  /// Command Identity
    command_long.param7 = shot_id;     /// Empty
    
    command_long.confirmation      = 0;
    command_long.target_component  = MAV_COMP_ID_CAMERA;
    command_long.target_system     = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    
    mavlink_msg_command_long_encode_chan(   systemid,
                                            compid,
                                            chan,
                                            &msg,
                                            &command_long);

    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE] = {0};
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    /*!< Get command ACK and Wait Feedback*/
    s_pFLIRHandle->waitCommandAck = command_long.command;
    
    /*!< Waiting semaphore from command ACK */
    psdkStat = osalHandler->SemaphoreTimedWait(s_pFLIRHandle->semaphoreWaitAck, PSDK_COMM_WAIT_ACK_TIMEOUT);
    if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        
        DebugError("Wait semaphore flir error: 0x%08llX.", psdkStat);
    } else {
    }
    
    /*!< Clear command after processing done */
    s_pFLIRHandle->waitCommandAck = 0;
    
    /*!< Unlock Mutex */
    psdkStat = osalHandler->MutexUnlock(s_pFLIRHandle->mutexSendGetAck);
    if (psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("unlock mutex error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
        
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief
 * @details This structure type is used to
 * @note 
 */
T_DjiReturnCode GremsyFLIR_SendZoom(uint8_t zoomVIS, uint8_t zoomIR)
{
    T_DjiReturnCode psdkStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    if(!s_pFLIRHandle->isCameraConnected) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    /*!< Lock mutex to send data */
    psdkStat = osalHandler->MutexLock(s_pFLIRHandle->mutexSendGetAck);
    if( psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("lock mutexSendGetAck error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    if(zoomVIS > FLIR_ZOOM_VIS_MAX_LEVEL) {
        zoomVIS = FLIR_ZOOM_VIS_LEVEL_8;
    } 
    
    if(zoomIR > FLIR_ZOOM_IR_MAX_LEVEL) {
        zoomVIS = FLIR_ZOOM_IR_LEVEL_4;
    } 
    
    mavlink_message_t           msg = {0};
    uint16_t                    len = 0;
    mavlink_command_long_t      command_long = {0};

    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;
    
     /// set param
    command_long.command = MAV_CMD_DO_DIGICAM_CONTROL;
    command_long.param1 = 0;
    command_long.param2 = zoomIR;                   /// Zoom IR tuyet doi   1, 2, 4
    command_long.param3 = zoomVIS;                 /// zoom VIS tuyet doi  1, 2, 4, 8
    command_long.param4 = 0; 
    command_long.param5 = 0;
    command_long.param6 = 0;
    command_long.param7 = 0;
    
    command_long.confirmation      = 0;
    command_long.target_component  = MAV_COMP_ID_CAMERA;
    command_long.target_system     = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    
    mavlink_msg_command_long_encode_chan(   systemid,
                                            compid,
                                            chan,
                                            &msg,
                                            &command_long);

    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE] = {0};
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);

        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    /*!< Get command ACK and Wait Feedback*/
    s_pFLIRHandle->waitCommandAck = command_long.command;
    
    /*!< Waiting semaphore from command ACK */
    psdkStat = osalHandler->SemaphoreTimedWait(s_pFLIRHandle->semaphoreWaitAck, PSDK_COMM_WAIT_ACK_TIMEOUT);
    if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        
        DebugError("Wait semaphore flir error: 0x%08llX.", psdkStat);
    } else {
    }
    
    /*!< Clear command after processing done */
    s_pFLIRHandle->waitCommandAck = 0;
    
    /*!< Unlock Mutex */
    psdkStat = osalHandler->MutexUnlock(s_pFLIRHandle->mutexSendGetAck);
    if (psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("unlock mutex error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
        
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}



/**
  * @brief Start image capture sequence 
  *  Parameter 1 (Duration between two consecutive pictures in seconds). Inverval between captures 0 thr 60 inclusive
  *  Parameter 2 (for the specific command)
  *  Parameter 3 (for the specific command)
  *  Parameter 4 (for the specific command)
  *  Parameter 5 (for the specific command)
  *  Parameter 6 (for the specific command)
  *  Parameter 7 (for the specific command)
 * @note 
 */
T_DjiReturnCode GremsyFLIR_SendImageStartCapture(void)
{
    T_DjiReturnCode psdkStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    if(!s_pFLIRHandle->isCameraConnected) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    /*!< Lock mutex to send data */
    psdkStat = osalHandler->MutexLock(s_pFLIRHandle->mutexSendGetAck);
    if( psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("lock mutexSendGetAck error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    mavlink_message_t           msg = {0};
    uint16_t                    len = 0;
    mavlink_command_long_t      command_long = {0};

    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;
    
     /// set param
    command_long.command = MAV_CMD_IMAGE_START_CAPTURE;
    command_long.param1 = 0;          /// thoi gian giua hai lan chup lien tiep, khi se thoi gian hinh se duoc chup lien tiep. 0 - 60 second
    command_long.param2 = 0;          /// none
    command_long.param3 = FLIRCamera->fileFormat;          /// do phan giai cua hinh chup. 1: JPEG & TIFF, 2: FFF(FLIR File Format)
    command_long.param4 = 0;          /// none
    command_long.param5 = 0;          /// none
    command_long.param6 = 0;          /// none
    command_long.param7 = 0;          /// none
        
    command_long.confirmation      = 0; /*!< if it == 1 it will not run*/
    command_long.target_component  = MAV_COMP_ID_CAMERA;
    command_long.target_system     = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    
    mavlink_msg_command_long_encode_chan(   systemid,
                                            compid,
                                            chan,
                                            &msg,
                                            &command_long);

    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE] = {0};
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    /*!< Get command ACK and Wait Feedback*/
    s_pFLIRHandle->waitCommandAck = command_long.command;
    
    /*!< Waiting semaphore from command ACK */
    psdkStat = osalHandler->SemaphoreTimedWait(s_pFLIRHandle->semaphoreWaitAck, PSDK_COMM_WAIT_ACK_TIMEOUT);
    if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        
        DebugError("Wait semaphore flir error: 0x%08llX.", psdkStat);
    } else {
    }
    
    /*!< Clear command after processing done */
    s_pFLIRHandle->waitCommandAck = 0;
    
    /*!< Unlock Mutex */
    psdkStat = osalHandler->MutexUnlock(s_pFLIRHandle->mutexSendGetAck);
    if (psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("unlock mutex error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief
 * @details This structure type is used to
 * @note 
 */
T_DjiReturnCode GremsyFLIR_SendImageStopCapture(void)
{
    T_DjiReturnCode psdkStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    if(!s_pFLIRHandle->isCameraConnected) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    /*!< Lock mutex to send data */
    psdkStat = osalHandler->MutexLock(s_pFLIRHandle->mutexSendGetAck);
    if( psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("lock mutexSendGetAck error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    mavlink_message_t           msg = {0};
    uint16_t                    len = 0;
    mavlink_command_long_t      command_long = {0};

    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;
    
    command_long.command = MAV_CMD_IMAGE_STOP_CAPTURE;
    command_long.param1 = 0;
    command_long.param2 = 0;
    command_long.param3 = 0;
    command_long.param4 = 0;
    command_long.param5 = 0;
    command_long.param6 = 0;
    command_long.param7 = 0;
    
    command_long.confirmation      = 0;
    command_long.target_component  = MAV_COMP_ID_CAMERA;
    command_long.target_system     = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    
    mavlink_msg_command_long_encode_chan(   systemid,
                                            compid,
                                            chan,
                                            &msg,
                                            &command_long);

    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE] = {0};
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    /*!< Get command ACK and Wait Feedback*/
    s_pFLIRHandle->waitCommandAck = command_long.command;
    
    /*!< Waiting semaphore from command ACK */
    psdkStat = osalHandler->SemaphoreTimedWait(s_pFLIRHandle->semaphoreWaitAck, PSDK_COMM_WAIT_ACK_TIMEOUT);
    if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        
        DebugError("Wait semaphore flir error: 0x%08llX.", psdkStat);
    } else {
    }
    
    /*!< Clear command after processing done */
    s_pFLIRHandle->waitCommandAck = 0;
    
    /*!< Unlock Mutex */
    psdkStat = osalHandler->MutexUnlock(s_pFLIRHandle->mutexSendGetAck);
    if (psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("unlock mutex error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


/**
 * @brief
 * @details This structure type is used to
 * @note 
 */
T_DjiReturnCode GremsyFLIR_SendVideoStartCapture(void)
{
    T_DjiReturnCode psdkStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    if(!s_pFLIRHandle->isCameraConnected) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    /*!< Lock mutex to send data */
    psdkStat = osalHandler->MutexLock(s_pFLIRHandle->mutexSendGetAck);
    if( psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("lock mutexSendGetAck error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    mavlink_message_t           msg = {0};
    uint16_t                    len = 0;
    mavlink_command_long_t      command_long = {0};

    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;
    
    command_long.command = MAV_CMD_VIDEO_START_CAPTURE;
    command_long.param1 = 0; /*!<  Camera ID. 0: All, 1: First, 2: Second.*/
    command_long.param2 = FLIRCamera->videoFileType; /*!< Frames/second. 1: H264, 2: TLFF. == 2 cannot record even using app*/ 
    command_long.param3 = 0;
    command_long.param4 = 0;
    command_long.param5 = 0;
    command_long.param6 = 0;
    command_long.param7 = 0;
    DebugInfo("Set video file type :%d", FLIRCamera->videoFileType);
    command_long.confirmation      = 0;
    command_long.target_component  = MAV_COMP_ID_CAMERA;
    command_long.target_system     = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    
    mavlink_msg_command_long_encode_chan(   systemid,
                                            compid,
                                            chan,
                                            &msg,
                                            &command_long);

    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE] = {0};
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    /*!< Get command ACK and Wait Feedback*/
    s_pFLIRHandle->waitCommandAck = command_long.command;
    
    /*!< Waiting semaphore from command ACK */
    psdkStat = osalHandler->SemaphoreTimedWait(s_pFLIRHandle->semaphoreWaitAck, PSDK_COMM_WAIT_ACK_TIMEOUT);
    if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        
        DebugError("Wait semaphore flir error: 0x%08llX.", psdkStat);
    } else {
    }
    
    /*!< Clear command after processing done */
    s_pFLIRHandle->waitCommandAck = 0;
    
    /*!< Unlock Mutex */
    psdkStat = osalHandler->MutexUnlock(s_pFLIRHandle->mutexSendGetAck);
    if (psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("unlock mutex error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief
 * @details This structure type is used to
 * @note 
 */
T_DjiReturnCode GremsyFLIR_SendVideoStopCapture(void)
{
    T_DjiReturnCode psdkStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    if(!s_pFLIRHandle->isCameraConnected) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    /*!< Lock mutex to send data */
    psdkStat = osalHandler->MutexLock(s_pFLIRHandle->mutexSendGetAck);
    if( psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("lock mutexSendGetAck error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    mavlink_message_t           msg = {0};
    uint16_t                    len = 0;
    mavlink_command_long_t      command_long = {0};

    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;
    
    command_long.command = MAV_CMD_VIDEO_STOP_CAPTURE;
    command_long.param1 = 0; /*!<  Camera ID. 0: All, 1: First, 2: Second.*/
    command_long.param2 = 0; /*!< Frames/second. 1: H264, 2: TLFF*/ 
    command_long.param3 = 0;
    command_long.param4 = 0;
    command_long.param5 = 0;
    command_long.param6 = 0;
    command_long.param7 = 0;
    
    command_long.confirmation      = 0;
    command_long.target_component  = MAV_COMP_ID_CAMERA;
    command_long.target_system     = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    
    mavlink_msg_command_long_encode_chan(   systemid,
                                            compid,
                                            chan,
                                            &msg,
                                            &command_long);

    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE] = {0};
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    /*!< Get command ACK and Wait Feedback*/
    s_pFLIRHandle->waitCommandAck = command_long.command;
    
    /*!< Waiting semaphore from command ACK */
    psdkStat = osalHandler->SemaphoreTimedWait(s_pFLIRHandle->semaphoreWaitAck, PSDK_COMM_WAIT_ACK_TIMEOUT);
    if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        
        DebugError("Wait semaphore flir error: 0x%08llX.", psdkStat);
    } else {
    }
    
    /*!< Clear command after processing done */
    s_pFLIRHandle->waitCommandAck = 0;
    
    /*!< Unlock Mutex */
    psdkStat = osalHandler->MutexUnlock(s_pFLIRHandle->mutexSendGetAck);
    if (psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("unlock mutex error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief
 * @details This structure type is used to
 * @note Skycondition & humidity cannot use
 */
T_DjiReturnCode GremsyFLIR_SendUser1(E_FlirTempUnit tempUnit, 
                                    E_FlirSpotMeter spotMeter, 
                                    uint8_t emissivity, 
                                    uint8_t skyCondition,
                                    int16_t airTemperature,
                                    uint8_t humidity,
                                    uint16_t subjectRange)
{
    T_DjiReturnCode psdkStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    if(!s_pFLIRHandle->isCameraConnected) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    /*!< Lock mutex to send data */
    psdkStat = osalHandler->MutexLock(s_pFLIRHandle->mutexSendGetAck);
    if( psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("lock mutexSendGetAck error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    mavlink_message_t           msg = {0};
    uint16_t                    len = 0;
    mavlink_command_long_t      command_long = {0};

    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;
    
    command_long.command = MAV_CMD_USER_1;
    command_long.param1 = tempUnit;
    command_long.param2 = spotMeter;
    command_long.param3 = emissivity;
    command_long.param4 = skyCondition;
    command_long.param5 = airTemperature;
    command_long.param6 = humidity;
    command_long.param7 = subjectRange;
        
    command_long.confirmation      = 0;
    command_long.target_component  = MAV_COMP_ID_CAMERA;
    command_long.target_system     = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    
    mavlink_msg_command_long_encode_chan(   systemid,
                                            compid,
                                            chan,
                                            &msg,
                                            &command_long);

    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE] = {0};
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    /*!< Get command ACK and Wait Feedback*/
    s_pFLIRHandle->waitCommandAck = command_long.command;
    
    /*!< Waiting semaphore from command ACK */
    psdkStat = osalHandler->SemaphoreTimedWait(s_pFLIRHandle->semaphoreWaitAck, PSDK_COMM_WAIT_ACK_TIMEOUT);
    if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        
        DebugError("Wait semaphore flir error: 0x%08llX.", psdkStat);
    } else {
    }
    
    /*!< Clear command after processing done */
    s_pFLIRHandle->waitCommandAck = 0;
    
    /*!< Unlock Mutex */
    psdkStat = osalHandler->MutexUnlock(s_pFLIRHandle->mutexSendGetAck);
    if (psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("unlock mutex error: 0x%08llX.", psdkStat);
        return psdkStat;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief
 * @details This structure type is used to
 * @note 
 */
T_DjiReturnCode GremsyFLIR_SendRequestInfo(uint16_t cmd )
{
    if(!s_pFLIRHandle->isCameraConnected) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    mavlink_message_t           msg = {0};
    uint16_t                    len = 0;
    mavlink_command_long_t      command_long = {0};

    uint8_t systemid    = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_SYSTEM_CONTROL;
    uint8_t chan        = PSDK_COMM_WITH_CAMERA_MAV_CHAN;
    
    command_long.command = cmd;
    command_long.param1 = 0;
    command_long.param2 = 0;
    command_long.param3 = 0;
    command_long.param4 = 0;
    command_long.param5 = 0;
    command_long.param6 = 0;
    command_long.param7 = 0;
    
    command_long.confirmation      = 0;
    command_long.target_component  = MAV_COMP_ID_CAMERA;
    command_long.target_system     = PSDK_COMM_SYSTEM_CONTROL_SYSTEM_ID;
    
    mavlink_msg_command_long_encode_chan(   systemid,
                                            compid,
                                            chan,
                                            &msg,
                                            &command_long);

    uint8_t msgbuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE] = {0};
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        DebugError("[%s][%d] - Invalid length", __FUNCTION__, __LINE__);
    
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/*
 * @brief Handle mavlink message
 * @param msg Pointer to frame to be read.
 * @param out mavlink message after parsing
 * @retval - None
 */
void PsdkCamera_HandleMessage(const mavlink_message_t* msg)
{
    // Check pointer is valid 
    if(msg == NULL) {
        return;
    }
    
//    DebugInfo("Got ID :%d", msg->msgid);
    
    // Parse message id 
    switch(msg->msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            mavlink_heartbeat_t packet;

            /* Call decode function */
            mavlink_msg_heartbeat_decode(msg, &packet);
            
            /*!< Set flir is connected */
            s_pFLIRHandle->isCameraConnected = true;
            
//            DebugInfo("Got HB type :%d",packet.type);
            
            /*!< Get time*/
            Osal_GetTimeMs(&s_pFLIRHandle->lastTimeConnection);
            
            break;
        }
        case MAVLINK_MSG_ID_PING:
        {
            DebugInfo("Got MAVLINK_MSG_ID_PING...!");
            break;
        }
        case MAVLINK_MSG_ID_SYSTEM_TIME:
        {
            DebugInfo("Got MAVLINK_MSG_ID_SYSTEM_TIME...!");
            
            break;
        }
        /*!< See: "GCS_Mavlink.cpp"*/
        case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
        {
            mavlink_request_data_stream_t packet;
            
            mavlink_msg_request_data_stream_decode(msg, &packet);
           
            /*!< Check whether the request that for camera */
            /*!< Req rate = 2, stream_id = 6, start_stop = 1*/
            DebugInfo("Req rate: %d , id: %d, start/stop: %d, comp: %d, %d", 
                    packet.req_message_rate, 
                    packet.req_stream_id, 
                    packet.start_stop, packet.target_component, packet.target_system);
                    
            s_pFLIRHandle->isReqDataStream = true;
            
            /*!< Get message Rate */
            s_pFLIRHandle->msgRate = packet.req_message_rate;
            
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_ACK:
        {
            mavlink_command_ack_t packet;
            
            mavlink_msg_command_ack_decode(msg, &packet);
            
            DebugInfo("Command ACK : %d result: %d, progress: %d", packet.command, packet.result, packet.progress);
            
            /*!< Check the ack is valid */
            if(s_pFLIRHandle->waitCommandAck == packet.command) {
                /*!< Release semaphore*/
                if(Osal_SemaphorePost(s_pFLIRHandle->semaphoreWaitAck) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    
                    DebugError("Release Semaphore Failed");
                    
                    /*!< Reset command */
                    s_pFLIRHandle->waitCommandAck = 0;
                }
            }
            break;
        }
    }
}


/**
 * @brief Low level PSDK frame read function.
 * @param pReadData Pointer to frame to be read.
 * @param dataLen Frame length.
 * @retval -1: Read error.
 *          not -1: Counts of bytes read actually.
 */
static int PsdkCameraFlir_Read(uint8_t *pReadData, uint16_t dataLen)
{
    int res;

    res = UART_Read(PSDK_COMM_WITH_CAMERA_UART_NUM, pReadData, dataLen);
    return res;
}

/**
 * @brief Low level PSDK frame send function.
 * @param pSendData Pointer to frame to be sent.
 * @param dataLen Frame length.
 * @return PSDK function process state.
 */
static int PsdkCameraFlir_Write(const uint8_t *pSendData, uint16_t dataLen)
{

    int res = UART_Write(PSDK_COMM_WITH_CAMERA_UART_NUM, pSendData, dataLen);
    
//#if (SLOG_WIRIS == 0)
//    for(int i = 0; i < res;  i++)
//        DebugWarning("[Write] - 0x%02X", pSendData[i]);
//#endif
    
    return res;
}

/**
  * @brief Function is used to register the function send 
  * @retval T_DjiReturnCode
  */
T_DjiReturnCode FlirCameraProto_RegSendDataFunc(SendCallbackFunc callbackFunc)
{
    if(callbackFunc == NULL){
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    s_pFLIRHandle->mavHandler.write   = callbackFunc;
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
  * @brief Function is used to register the function send 
  * @retval T_DjiReturnCode
  */
T_DjiReturnCode FlirCameraProto_RegRecvDataFunc(ReceiveCallbackFunc callbackFunc)
{
    if(callbackFunc == NULL){
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    s_pFLIRHandle->mavHandler.read = callbackFunc;
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


/** @brief Function parse FLIR data
  * @param none
  * @param none
  * @return None
  */
static T_DjiReturnCode FlirProt_ProcessReceiveData(const uint8_t *pData, uint16_t realLen)
{
    // Scan the counts of bytes read actually.
    for (int i = 0; i < realLen; i++) {
        // Get data from buffer
        uint8_t c = pData[i];
        
        //===================== MAVLINK Recievie Function ==============================//            
        //// Try to get a new message
        if(mavlink_parse_char(PSDK_COMM_WITH_CAMERA_MAV_CHAN, c, &s_pFLIRHandle->msg, &s_pFLIRHandle->status)) {
            // Handle message here
            PsdkCamera_HandleMessage(&s_pFLIRHandle->msg);
        }
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
  * @brief GremsyFLIR_CameraInit
  * Initialization for interfacing with FLIR camera 
  */
T_DjiReturnCode FLIRProto_CameraInit(void)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    /*!< Allocation memory */
    s_pFLIRHandle = osalHandler->Malloc(sizeof(T_FLIRHandle));

    /*!< Check pointer is valid */
    if(s_pFLIRHandle == NULL){
        DebugError("Malloc Flir Handle Failed. !");
        return DJI_ERROR_SYSTEM_MODULE_CODE_MEMORY_ALLOC_FAILED;
    }
    
    /*!< Clear buffer */
    memset(s_pFLIRHandle, 0x00, sizeof(T_FLIRHandle));
    
    /*!< Init UART*/
    UART_Init(PSDK_COMM_WITH_CAMERA_UART_NUM, PSDK_COMM_WITH_CAMERA_UART_BAUD);
    
   
    /*!< Init mavlink channel and assign function for sending message*/
    s_pFLIRHandle->mavHandler.version = 1;
    s_pFLIRHandle->mavHandler.chan    = PSDK_COMM_WITH_CAMERA_MAV_CHAN;
    s_pFLIRHandle->mavHandler.write   = PsdkCameraFlir_Write;
    
    /*!< Init mavlink function handler */
    gremsy_mavlink_init(&s_pFLIRHandle->mavHandler);
    
    /*!< Create mutext for send get ACK */
    if(osalHandler->MutexCreate(&s_pFLIRHandle->mutexSendGetAck) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("mutexSendGetAck create error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    
    /*!< Using semaphore for synchronization the event when pasrsing the command ack received from camera*/
    if(osalHandler->SemaphoreCreate(1, &s_pFLIRHandle->semaphoreWaitAck) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("semaphorewaitAck create error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    
    /*!< Reset semaphore. This will be set when received a ack from the camera*/
    T_DjiReturnCode djiReturnCode = osalHandler->SemaphoreTimedWait(s_pFLIRHandle->semaphoreWaitAck, 0);
    if(djiReturnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("Take semaphore failed 0x%08llX.", djiReturnCode);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS; 
}

/**
  * @brief GremsyGimbal_RecvTask
  * Task gimbal will interface with the gimbal by conveying gimbal data 
  * to app as well as gimbal command to gimbal devixe
  */
static void *CameraRecTask(void *parameter)
{
    uint32_t step = 0;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    while (1) {
        /*!< BaurdRare (Bd) = 115200  -- > 1Bd = 1 bit/s  --> 8.6us
            Uart Frame 12 bits. Time to read = 12*8.68 = 104.2 us --> frep = 9,6K
            Queue 1024 bytes. Delay 1ms is compatible.
         */
        
        int realLen = PsdkCameraFlir_Read(s_uartRecBuf, sizeof(s_uartRecBuf));
        
        if (realLen > 0) {
            //===================== WIRIS Recievie Function ==============================//
            //all callbacks process in this thread.
            FlirProt_ProcessReceiveData(s_uartRecBuf, realLen);
        }
        
        osalHandler->TaskSleepMs(1000 / FLIR_CAMERA_TASK_FREQ);
        step++;
        
        uint32_t currentTimeMs = 0;
        if(osalHandler->GetTimeMs(&currentTimeMs) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            DebugError("Get Time Ms ");
        }
        
        /*!< Check the command connection */
        if(currentTimeMs - s_pFLIRHandle->lastTimeConnection > PSDK_COMM_WAIT_RESPOND_TIMEOUT) {
            /*!< FLIR lost connection 
              - Clear flag 
              - Switch to state lost connection 
            */
            if(s_pFLIRHandle->isCameraConnected) {
                s_pFLIRHandle->isCameraConnected = false;
            
                /*!< Check if the camera already is running then lost connection  */
//                s_cameraRunningState = CAMERA_RUNNING_STATE_CHECK_CONNECTION;
            }
        }
        
        /*!< Camera now is ready */
        if(s_cameraRunningState == CAMERA_RUNNING_STATE_PROCESS_EVENT) {
            /*!< Send heartbeat periodically 1Hz */
            if (USER_UTIL_IS_WORK_TURN(step, 1, FLIR_CAMERA_TASK_FREQ)) {
                GsdkFLIR_CameraSetEvent(CAM_EVENT_SEND_HANDSHAKE);
            }
            
            /*!< Send GPS periodically 2Hz */
            if (USER_UTIL_IS_WORK_TURN(step, 2, FLIR_CAMERA_TASK_FREQ)) {
                GsdkFLIR_CameraSetEvent(CAM_EVENT_SEND_INFO);
            }
        }
    }
}

/**
  * @brief GremsyGimbal_RecvTask
  * Task gimbal will interface with the gimbal by conveying gimbal data 
  * to app as well as gimbal command to gimbal devixe
  */
static void *ProcessCommandTask(void *parameter)
{
    T_DjiReturnCode psdkStat = DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    
    static uint32_t     step = 0;    
    uint8_t             countMsg = 0;

    /*!< Attempt to create the event group*/
    s_eventCamera = xEventGroupCreate();
    
    /*!< Event bit to check */
    EventBits_t event;
    
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    while(1) {
        
        osalHandler->TaskSleepMs(1000 / FLIR_CAMERA_TASK_FREQ);
        step++;
        
        /*!< Check camera running state */
        if(s_cameraRunningState == CAMERA_RUNNING_STATE_IDLE) {
            
            s_pFLIRHandle->isCameraConnected = false;
            s_pFLIRHandle->isSOMReseted = false;
            
            /*!< Switch to check connection */
            s_cameraRunningState = CAMERA_RUNNING_STATE_CHECK_CONNECTION;
        }
        else if(s_cameraRunningState == CAMERA_RUNNING_STATE_CHECK_CONNECTION) {

            /*!< Send heartbeat periodically 1Hz */
            if (USER_UTIL_IS_WORK_TURN(step, 1, FLIR_CAMERA_TASK_FREQ)) {
                psdkStat = GremsyFLIR_SendHeartbeat();
                if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError(".......Send HB FAILED ! 0x%08llX.", psdkStat);
                }  
            }
            
            /*!< Send heartbeat periodically 2Hz */
            if (USER_UTIL_IS_WORK_TURN(step, 2, FLIR_CAMERA_TASK_FREQ)) {
                psdkStat = GremsyFLIR_SendGlobalPositionInit();
                if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError(".......Send GPS FAILED ! 0x%08llX.", psdkStat);
                }  
            }
            
            /*!< Camera is connected */
            if(s_pFLIRHandle->isCameraConnected) {
                /*!< Has been got request data stream */
                if(s_pFLIRHandle->isReqDataStream) {
                    s_pFLIRHandle->isReqDataStream = false;
                    s_cameraRunningState = CAMERA_RUNNING_STATE_RESET_SOM;
                }
            }
        }
        else if(s_cameraRunningState == CAMERA_RUNNING_STATE_RESET_SOM) {
            /*!< Check whether som is reset ted */
            if(!s_pFLIRHandle->isSOMReseted) {
                 s_pFLIRHandle->isSOMReseted = true;
                
//                if(PsdkLinux_GetSomState() != SOM_INIT) {
//                    DebugWarning("Som is reseting !");
//                    
//                     /*!< Reset SOM */
//                    PsdkSom_WriteHardwareResetPin(SOM_HW_RESET_MANAGEMENT_PIN_STATE_RESET);
//                    PsdkOsal_TaskSleepMs(20);
//                    PsdkSom_WriteHardwareResetPin(SOM_HW_RESET_MANAGEMENT_PIN_STATE_SET);
//                    PsdkOsal_TaskSleepMs(20);
//                    
//                    PsdkLinux_SetSomState();
//                }
            }
            
            s_cameraRunningState = CAMERA_RUNNING_STATE_DEFAULT_SETTING;
        }
        else if(s_cameraRunningState == CAMERA_RUNNING_STATE_DEFAULT_SETTING) {
            
            /*!< Check param is valid */
            if(GsdkFlir_GetParams(&s_flirParams) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                if(!IS_FLIR_SCENE(s_flirParams.SENCE_MODE)) {
                    s_flirParams.SENCE_MODE = FLIR_SCENE_DEFAULT;
                }
                FLIRCamera->scene = s_flirParams.SENCE_MODE;

                /*!< Check param*/
                if(s_flirParams.COLOR_PALETTE < 0 || s_flirParams.COLOR_PALETTE >= FLIR_PALETTE_MAX_COUNT) {
                    s_flirParams.COLOR_PALETTE = FLIR_PATETTE_WHITEHOT;
                }
                FLIRCamera->palette = s_flirParams.COLOR_PALETTE;

                /*!< Check */
                if(s_flirParams.MSX_ENABLE > 1 || s_flirParams.MSX_ENABLE < 0) {
                    s_flirParams.MSX_ENABLE = 1;
                }
                FLIRCamera->isMSXEnable = s_flirParams.MSX_ENABLE;
                
                /*!< Check param */
                if(s_flirParams.MSX_LENGTH < 0 || s_flirParams.MSX_LENGTH > 100) {
                    s_flirParams.MSX_LENGTH = 20;
                }
                FLIRCamera->MSXLength = s_flirParams.MSX_LENGTH;
                
                /*!< Check param */
                if(!IS_FLIR_VIDEO_TRANSMISION(s_flirParams.VIDEO_TRANSMISSION)) {
                    s_flirParams.VIDEO_TRANSMISSION = FLIR_VIDEO_TRANSMISSION_VIS;
                }
                FLIRCamera->videoTransmission = s_flirParams.VIDEO_TRANSMISSION;
                
                /*!< Check param */
                if((s_flirParams.VIDEO_FILE_TYPE != FLIR_VIDEO_FILE_TYPE_H264) || (s_flirParams.VIDEO_FILE_TYPE != FLIR_VIDEO_FILE_TYPE_TLFF)) {
                    s_flirParams.VIDEO_FILE_TYPE = FLIR_VIDEO_FILE_TYPE_H264;
                }
                FLIRCamera->videoFileType = s_flirParams.VIDEO_FILE_TYPE;
                
                /*!< Check file Format is valid */
                if((s_flirParams.FILE_FORMAT != FLIR_FILE_FORMART_JPEG_TIFF) || (s_flirParams.FILE_FORMAT != FLIR_FILE_FORMART_FFF)) {
                    /*!< Set as default */
                    s_flirParams.FILE_FORMAT = FLIR_FILE_FORMART_JPEG_TIFF;
                }
                FLIRCamera->fileFormat = s_flirParams.FILE_FORMAT;
                
                /*!< Check temperature unit */
                if(!IS_FLIR_TEMP_UNIT(s_flirParams.TEMP_UNIT)) {
                    s_flirParams.TEMP_UNIT = FLIR_TEMP_UNIT_C;
                }
                FLIRCamera->tempUnit = s_flirParams.TEMP_UNIT;
                
                /*!< Check param */
                if(!IS_FLIR_SPOT_METER(s_flirParams.TEMP_METER)) {
                    s_flirParams.TEMP_METER = FLIR_SPOT_METER_OFF;
                }
                FLIRCamera->spotMeter = s_flirParams.TEMP_METER;
                
                /*!< Check param */
                if(s_flirParams.SUBJECT_EMISSIVITY < 50) {
                    s_flirParams.SUBJECT_EMISSIVITY = 50;
                }
                FLIRCamera->emissivity = s_flirParams.SUBJECT_EMISSIVITY;
                
                
                /*!< Check param is valid */
                if(!IS_FLIR_SKYCONDITION(s_flirParams.SKY_CONDITION)) {
                    s_flirParams.SKY_CONDITION = FLIR_CLEAR_SKIES;
                }   
                FLIRCamera->skyCondition = s_flirParams.SKY_CONDITION;
                
                /*!< Check param */
                if(s_flirParams.AIR_TEMPERATURE < -50) {
                    s_flirParams.AIR_TEMPERATURE = -50;
                } else if(s_flirParams.AIR_TEMPERATURE > 40) {
                    s_flirParams.AIR_TEMPERATURE = 40;
                }
                FLIRCamera->airTemperature = s_flirParams.AIR_TEMPERATURE;
                
                /*!< Check param */
                if(!IS_FLIR_HUMIDITY(s_flirParams.HUMIDITY)) {
                    s_flirParams.HUMIDITY = HUMIDITY_LOW_30;
                }
                FLIRCamera->humidity = s_flirParams.HUMIDITY;
                
                /*!< Check param */
                if(s_flirParams.SUBJECT_RANGE > 200 || FLIRCamera->subjectRange < 0) {
                    FLIRCamera->subjectRange = 200;
                } 
                FLIRCamera->subjectRange = s_flirParams.SUBJECT_RANGE;
                
                /*!< Switch to setting camera */
                s_cameraRunningState = CAMERA_RUNNING_QUERY_CAMERA_SETTING;
            }
        }
        else if(s_cameraRunningState == CAMERA_RUNNING_QUERY_CAMERA_SETTING) {
            
            if(countMsg == 0) {
                /*!< Set flir default parameter */
                GremsyFLIR_SendDoDigicamConfigure(FLIRCamera->scene, 
                                                FLIRCamera->palette, 
                                                FLIRCamera->imageRotation, 
                                                FLIRCamera->isMSXEnable, 
                                                FLIRCamera->MSXLength);
                countMsg++;
            }
            else if(countMsg == 1) {
                
                GremsyFLIR_SendDoControlVideo(FLIRCamera->videoTransmission);
                countMsg++;
            }
            else if(countMsg == 2) {
                GremsyFLIR_SendUser1(FLIRCamera->tempUnit, 
                                FLIRCamera->spotMeter, 
                                FLIRCamera->emissivity, 
                                FLIRCamera->skyCondition, 
                                FLIRCamera->airTemperature, 
                                FLIRCamera->humidity, 
                                FLIRCamera->subjectRange);
                
                countMsg++;
            }
            else if(countMsg == 3) {
                /*!< Reset zoom value */
                GremsyFLIR_SendZoom(FLIRCamera->zoomVISLevel, FLIRCamera->zoomIRLevel);
                countMsg++;
            }
            else if(countMsg == 4) {                
                FLIRCamera->zoomIRLevel       = FLIR_ZOOM_IR_LEVEL_1;
                FLIRCamera->zoomVISLevel      = FLIR_ZOOM_VIS_LEVEL_1;
                GremsyFLIR_SendZoom(FLIRCamera->zoomVISLevel, FLIRCamera->zoomIRLevel);
                
                countMsg++;
            }
             else if(countMsg == 5) {
                countMsg = 0;
             
                /*!< Switch to process commands*/
                s_cameraRunningState = CAMERA_RUNNING_STATE_PROCESS_EVENT;
             }
        }
        else if(s_cameraRunningState == CAMERA_RUNNING_STATE_PROCESS_EVENT) {
        /*!<----------------------Process Event group --------------------------------
        Block to for any event to be set within the event group. 
        Clear the bit before existing block to wait for one or more bits to be set within a previously created event group. */
            
            event = xEventGroupWaitBits(s_eventCamera, CAM_EVENT_ALLS, pdTRUE, pdFALSE, portMAX_DELAY);

            if(event & CAM_EVENT_SEND_HANDSHAKE)
            {
                psdkStat = GremsyFLIR_SendHeartbeat();
                if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError(".......Send HB FAILED ! 0x%08llX.", psdkStat);
                }  
            }
                
            if(event & CAM_EVENT_SEND_INFO) {
                 psdkStat = GremsyFLIR_SendGlobalPositionInit();
                if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError(".......Send Event GPS FAILED ! 0x%08llX.", psdkStat);
                }  
            }
            
            /*!< Event capture photo*/
            if(event & CAM_EVENT_SET_START_SHOOT_PHOTO) {
                if(GremsyFLIR_SendImageStartCapture() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError("Shoot photo FAILED!");
                }
                else {
                    s_flirParams.FILE_FORMAT = FLIRCamera->fileFormat;
                    
                    DebugInfo("Shoot photo COMPLETE %d !", s_flirParams.FILE_FORMAT);
                }
            }
            
            /*!< Set record video */
            if(event & CAM_EVENT_SET_START_RECORD) {
//                if(!FLIRCamera->isRecording) {
                    /*!< Check where the camera is recording */
                    if(GremsyFLIR_SendVideoStartCapture() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        
                        /*!< Recording failed */
                        FLIRCamera->isRecording = false;
                    } 
                    else {
                        DebugInfo("Camera is recording !");
                        
                        FLIRCamera->isRecording = true;
                        
                        s_flirParams.VIDEO_FILE_TYPE = FLIRCamera->videoFileType;
                    }
//                }
            }
            
            if(event & CAM_EVENT_SET_STOP_RECORD) {
                
                /*!< Check camera is recording*/
//                if(FLIRCamera->isRecording) {
                    if(GremsyFLIR_SendVideoStopCapture() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        FLIRCamera->isRecording = false;
                    } 
//                }
            }
            
            /*!< Event set camera zoom*/
            if(event & CAM_EVENT_SET_ZOOM_IN) {
                DebugInfo("Zoom VIS = %d IR = %d", FLIRCamera->zoomVISLevel, FLIRCamera->zoomIRLevel);
                GremsyFLIR_SendZoom(FLIRCamera->zoomVISLevel, FLIRCamera->zoomIRLevel);
            }
            
            if(event & CAM_EVENT_SET_ZOOM_OUT) {
                DebugInfo("Zoom VIS = %d IR = %d", FLIRCamera->zoomVISLevel, FLIRCamera->zoomIRLevel);
                GremsyFLIR_SendZoom(FLIRCamera->zoomVISLevel, FLIRCamera->zoomIRLevel);
            }
            
            /*!< Reset Zoom value */
            if(event & CAM_EVENT_RESET_DIGITAL_ZOOM_FACTOR) {
                FLIRCamera->zoomIRLevel       = FLIR_ZOOM_IR_LEVEL_1;
                FLIRCamera->zoomVISLevel      = FLIR_ZOOM_VIS_LEVEL_1;
                GremsyFLIR_SendZoom(FLIRCamera->zoomVISLevel, FLIRCamera->zoomIRLevel);
            }
            
            /*!< Event set camera video transmision to HDMI */
            if(event & CAM_EVENT_SET_LAYOUT) {
                DebugInfo("Set layout %d", FLIRCamera->videoTransmission);
                GremsyFLIR_SendDoControlVideo(FLIRCamera->videoTransmission);
                
                /*!< Set param to list */
                s_flirParams.VIDEO_TRANSMISSION = FLIRCamera->videoTransmission;
            }
             
            if(event & CAM_EVENT_SET_DIGICAM_CONTROL) {
                GremsyFLIR_SendDoDigicamControl(1, FLIRCamera->zoomIRLevel, FLIRCamera->zoomVISLevel,1,0,0,0);
            }
            
            /*!< Set event config flir */
            if(event & CAM_EVENT_SET_DIGICAM_CONFIG) {
                DebugInfo("Set config %d %d %d %d %d ",
                                                    FLIRCamera->scene, 
                                                    FLIRCamera->palette, 
                                                    FLIRCamera->imageRotation,
                                                    FLIRCamera->isMSXEnable,
                                                    FLIRCamera->MSXLength);
                
                GremsyFLIR_SendDoDigicamConfigure(FLIRCamera->scene, 
                                                    FLIRCamera->palette, 
                                                    FLIRCamera->imageRotation,
                                                    FLIRCamera->isMSXEnable,
                                                    FLIRCamera->MSXLength);
                
                s_flirParams.SENCE_MODE = FLIRCamera->scene;
                s_flirParams.COLOR_PALETTE = FLIRCamera->palette;

                s_flirParams.MSX_ENABLE = FLIRCamera->isMSXEnable;
                s_flirParams.MSX_LENGTH = FLIRCamera->MSXLength;
            }
            
            if(event & CAM_EVENT_SET_PARMETERS) {
                DebugInfo("Set params %d %d %d %d %d %d %d ", FLIRCamera->tempUnit, 
                                                                        FLIRCamera->spotMeter, 
                                                                        FLIRCamera->emissivity, 
                                                                        FLIRCamera->skyCondition, 
                                                                        FLIRCamera->airTemperature, 
                                                                        FLIRCamera->humidity, 
                                                                        FLIRCamera->subjectRange);
                
                GremsyFLIR_SendUser1(FLIRCamera->tempUnit, 
                                    FLIRCamera->spotMeter, 
                                    FLIRCamera->emissivity, 
                                    FLIRCamera->skyCondition, 
                                    FLIRCamera->airTemperature, 
                                    FLIRCamera->humidity, 
                                    FLIRCamera->subjectRange);

                s_flirParams.TEMP_UNIT = FLIRCamera->tempUnit;
                s_flirParams.TEMP_METER = FLIRCamera->spotMeter;
                s_flirParams.SUBJECT_EMISSIVITY = FLIRCamera->emissivity;
                s_flirParams.SKY_CONDITION = FLIRCamera->skyCondition;
                s_flirParams.AIR_TEMPERATURE = FLIRCamera->airTemperature;
                s_flirParams.HUMIDITY = FLIRCamera->humidity;
                s_flirParams.SUBJECT_RANGE = FLIRCamera->subjectRange;
            }
            
            if(event & CAM_EVENT_SET_DEFAULT_PARAM) {
                s_flirParams.VERSION_X = 1;
                s_flirParams.NUM_TRACK = 100;
                
                if(GsdkFlir_SetParams(&s_flirParams) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    /*!< Save complete */
                    FLIRCamera->isStoringParams = true;
                    
                    DebugWarning("Save FLIR Params Complete");
                }
            }
        }
        else if(s_cameraRunningState == CAMERA_RUNNING_STATE_ERROR) {
            
        }
    }
}

/*!<====================== PUBLIC_FUNCTIONS =================================== */

/** @addtogroup 
  * @{ PUBLIC_FUNCTIONS
  */

/**
 * @brief 
 * @details This structure type is used to
 * @note 
 */
T_DjiReturnCode GsdkFLIR_CameraInit(void) 
{
    T_DjiReturnCode psdkStat = DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    /*!<------------------ FLIR PROTOCOL INIT --------------------------------*/
    psdkStat = FLIRProto_CameraInit();
    if(psdkStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("FLIR Camera proto init Failed");
        return psdkStat;
    }
    
    /*!< Create the camera wiris receive data thread */
    if(osalHandler->TaskCreate("flir_recv_task", CameraRecTask,
        FLIR_CAMERA_TASK_STACK_SIZE, NULL, &s_recvThread) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
        DebugError("user camera task create error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    
    /*!< Create the camera wiris command process thread */
    if(osalHandler->TaskCreate("flir_cmd_task", ProcessCommandTask,
        FLIR_CAMERA_TASK_STACK_SIZE, NULL, &s_processCmdThread) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
        DebugError("user camera task create error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return psdkStat;
}


T_DjiReturnCode GsdkFLIR_CameraDeInit(void)
{
    
}

T_DjiReturnCode GsdkFLIR_CameraSetEvent(const uint32_t uxBitsToSet)
{
     // Was the event group created successfully?
    if( s_eventCamera != NULL && s_cameraRunningState == CAMERA_RUNNING_STATE_PROCESS_EVENT) {
         /*!< Set event to check camera is recording*/
        xEventGroupSetBits(s_eventCamera, uxBitsToSet);
        
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
}

/**
  * @} // PUBLIC_FUNCTIONS
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
