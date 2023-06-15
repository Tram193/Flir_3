/**
  ******************************************************************************
  * @file    gsdk_gimbal.c
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
#include "gimbal/gsdk_gimbal.h"

#include "dji_platform.h"
#include "utils/util_misc.h"
#include "dji_logger.h"

#include "uart.h"

/* Private define------------------------------------------------------------------------------*/
#define GIMBAL_DEBUG 0

#if GIMBAL_DEBUG
# define DebugMsg(fmt, args ...) do {USER_LOG_DEBUG("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);
# define DebugInfo(fmt, args ...) do {USER_LOG_INFO("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);
# define DebugWarning(fmt, args ...) do {USER_LOG_WARN("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);
#else
# define DebugMsg(fmt, args ...)
# define DebugInfo(fmt, args ...)
# define DebugWarning(fmt, args ...)
#endif

# define DebugError(fmt, args ...) do {USER_LOG_ERROR("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);


#define GIMBAL_TASK_STACK_SIZE              (1024 * 2)
#define GIMBAL_MAX_TRANCEIVER_BUFFER_SIZE   (256)

#define GIMBAL_UART_NUM                     UART_NUM_2
#define GIMBAL_MAVLINK_CHANNEL              MAVLINK_COMM_0

/*!<  Override define gimbal CMD */
#define MAVLINK_CMD_CTRL_ON_OFF_GIMBAL      MAV_CMD_USER_1
#define MAVLINK_CMD_CTRL_GIMBAL_MODE        MAV_CMD_USER_2
#define MAVLINK_CMD_CTRL_CTR_AUTO_TUNER     MAV_CMD_USER_3
#define MAVLINK_CMD_UNIQUE_REQUEST          MAV_CMD_USER_4
#define MAVLINK_CMD_CTRL_SPEED_SMOOTH       MAV_CMD_USER_5

/* Private typedef------------------------------------------------------------------------------*/
/**
 * @brief param_index_t
 * Gimbal opens some parameters for setting. Please refer to user manual to learn more how to set 
 * that parameters
 */
typedef enum _param_index_t
{
    GMB_PARAM_VERSION_X = 0,
    GMB_PARAM_VERSION_Y,
    GMB_PARAM_VERSION_Z,

    GMB_PARAM_STIFFNESS_PITCH,
    GMB_PARAM_STIFFNESS_ROLL,
    GMB_PARAM_STIFFNESS_YAW,

    GMB_PARAM_HOLDSTRENGTH_PITCH,
    GMB_PARAM_HOLDSTRENGTH_ROLL,
    GMB_PARAM_HOLDSTRENGTH_YAW,

    GMB_PARAM_OUTPUT_FILTER,
    GMB_PARAM_GYRO_FILTER,
    GMB_PARAM_GAIN,

    GMB_PARAM_SPEED_FOLLOW_PITCH,
    GMB_PARAM_SPEED_FOLLOW_YAW,

    GMB_PARAM_SMOOTH_FOLLOW_PITCH,
    GMB_PARAM_SMOOTH_FOLLOW_YAW,

    GMB_PARAM_WINDOW_FOLLOW_PITCH,
    GMB_PARAM_WINDOW_FOLLOW_YAW,

    GMB_PARAM_SPEED_CONTROL_PITCH,
    GMB_PARAM_SPEED_CONTROL_ROLL,
    GMB_PARAM_SPEED_CONTROL_YAW,

    GMB_PARAM_SMOOTH_CONTROL_PITCH,
    GMB_PARAM_SMOOTH_CONTROL_ROLL,
    GMB_PARAM_SMOOTH_CONTROL_YAW,

    GMB_MAPPING_ENABLE,
    GMB_MAPPING_ANGLE,

    GMB_PARAM_AXIS_DIR,
    
    GMB_PARAM_HEATBEAT_EMIT,
    GMB_PARAM_STATUS_RATE,
    GMB_PARAM_ENCODER_VALUE_RATE,
    GMB_PARAM_ENCODER_TYPE,
    GMB_PARAM_ORIENTATION_RATE,
    GMB_PARAM_RAW_IMU_RATE,
    
    GMB_PARAM_MIN_LIMIT_ANGLE_PITCH,
    GMB_PARAM_MAX_LIMIT_ANGLE_PITCH,
    GMB_PARAM_MIN_LIMIT_ANGLE_ROLL,
    GMB_PARAM_MAX_LIMIT_ANGLE_ROLL,
    GMB_PARAM_MIN_LIMIT_ANGLE_YAW,
    GMB_PARAM_MAX_LIMIT_ANGLE_YAW,
    
    GMB_PARAM_RC_TYPE,

    GIMBAL_NUM_TRACKED_PARAMS
} param_index_t;

/**
 * @brief param_state_t
 * State of param 
 */
typedef enum _param_state_t
{
    PARAM_STATE_NOT_YET_READ        = 0,    // parameter has yet to be initialized
    PARAM_STATE_FETCH_AGAIN         = 1,    // parameter is being fetched
    PARAM_STATE_ATTEMPTING_TO_SET   = 2,    // parameter is being set
    PARAM_STATE_CONSISTENT          = 3,    // parameter is consistent
    PARAM_STATE_NONEXISTANT         = 4     // parameter does not seem to exist
} param_state_t;


// dinh nghia trang thai
typedef enum _gimbal_status1_t
{
    STATUS1_MODE_FOLLOW_LOCK    = 0x01,
    STATUS1_MISS_STEP           = 0x02,
    STATUS1_SENSOR_ERROR        = 0x04,
    STATUS1_BATT_LOW            = 0x08,
    STATUS1_MOTORS              = 0x10,         /// motors on = 1, motor off = 0 (fimware 1.3.4)*/
    STATUS1_INIT_MOTOR          = 0x20,
    STATUS1_AUTO_TUNER          = 0x40,         /// 0b0100 0000
    STATUS1_CANLINK             = 0x80,         /// 0b1000 0000 ket noi can link.
    STATUS1_SEARCH_HOME         = 0x100,        /// search home
    STATUS1_SET_HOME            = 0x200,        /// set home
    STATUS1_SENSOR_CALIB        = 0x400,        /// calib sensor gom accel va gyro
    STATUS1_STARTUP             = 0x800,
    STATUS1_REMOTE              = 0x1000,
    STATUS1_INVERTED            = 0x2000,
    STATUS1_MOTOR_PHASE_ERROR   = 0x4000,
    STATUS1_MOTOR_ANGLE_ERROR   = 0x8000,
} gimbal_status1_t;

// dinh nghia trang thai
typedef enum _gimbal_status2_t
{
    STATUS2_IMU_ERROR               = 0x01,
    STATUS2_MOTOR_TILT_ERROR        = 0x02,
    STATUS2_MOTOR_ROLL_ERROR        = 0x04,
    STATUS2_MOTOR_PAN_ERROR         = 0x08,
    STATUS2_JOYSTICK_ERROR          = 0x10,
    STATUS2_INVERTED_ERROR          = 0x20,
    STATUS2_PAN_SEARCH_HOME_ERROR   = 0x40,

    STATUS2_ANGLE_TILT_ERROR        = 0x80,
    STATUS2_ANGLE_ROLL_ERROR        = 0x100,
    STATUS2_ANGLE_PAN_ERROR         = 0x200,

    STATUS2_MOVE_TILT_ERROR         = 0x400,
    STATUS2_MOVE_ROLL_ERROR         = 0x800,
    STATUS2_MOVE_PAN_ERROR          = 0x1000,
    
    STATUS2_MOTOR_FRAME_ERROR       = 0x2000,
} gimbal_status2_t;

/**
 * @brief control_mode_t
 * Command control gimbal mode lock/follow
 */
typedef enum _gimbal_mode_t
{
    GREMSY_GIMBAL_MODE_LOCK          = 0x01,
    GREMSY_GIMBAL_MODE_FOLLOW        = 0x02,
    GREMSY_GIMBAL_MODE_MAPPING       = 0x03,
    
    GREMSY_GIMBAL_RESET_MODE         = 0x04,

} gimbal_mode_t;

/**
 * @brief
 * @details This structure type is used to
 */
typedef enum _mapping_value
{
    MAPPING_VALUE_DISABLE       = 0,
    MAPPING_VALUE_DOWN_30       = 1,
    MAPPING_VALUE_DOWN_45,
    MAPPING_VALUE_DOWN_60,
    MAPPING_VALUE_DOWN_90,
} mapping_value_t;

/**
 * @brief _control_gimbal_axis_input_mode
 * Command control gimbal input mode for each axis
 */
typedef enum _gimbal_axis_input_mode
{
    CTRL_ANGLE_BODY_FRAME       = 0,
    CTRL_ANGULAR_RATE           = 1,
    CTRL_ANGLE_ABSOLUTE_FRAME   = 2,
} gimbal_axis_input_mode_t;

/**
 * @brief axis direction of motor in control
 * Command control motor is on/off
 */
typedef enum _axis_direction_t
{
    AXIS_DIR_CW     = 0x00,
    AXIS_DIR_CCW    = 0x01
} axis_direction_t;


/**
 * @brief Define Axis control 
 * @details This structure type is used to specific the axis control
 */
typedef enum _axis_control_t
{
    PITCH_AXIS_CONTROL  = 0x01,
    ROLL_AXIS_CONTROL   = 0x02,
    YAW_AXIS_CONTROL    = 0x04,
} axis_control_t;

/** @brief gimbal state
  * 
*/
typedef enum _gimbal_state_t
{
    GIMBAL_STATE_NOT_PRESENT = 0,
    GIMBAL_STATE_PRESENT_INITIALIZING,
    GIMBAL_STATE_PRESENT_ALIGNING,
    GIMBAL_STATE_PRESENT_RUNNING
} gimbal_state_t;

/**
 * @brief _control_gimbal_axis_mode_t
 * Command control gimbal for each axis
 */
typedef struct _gimbal_axis_mode_t
{
    /* stabilize? (1 = yes, 0 = no)*/
    uint8_t stabilize;   
    
    gimbal_axis_input_mode_t    input_mode;
    
} gimbal_axis_mode_t;

/* @brief control_mode_t
 * Command control gimbal mode lock/follow
 */
typedef enum _control_gimbal_motor_t
{
    TURN_OFF = 0x00,
    TURN_ON = 0x01
} control_gimbal_motor_t;

/** @brief Struct contain mavlink gimbal message
    @return None
*/
typedef struct T_MavlinkProto 
{
    int phub_sysid;
    int phub_compid;
    int gimbal_compid;
   
    /* Time to check the gimbal device*/
    uint32_t                    last_report_msg_ms;
    
    /* State of gimbal device */
    gimbal_state_t             state;
    
#if defined(QE_CAM)
    
    bool isMsg[512];
    
    mavlink_sys_status_t                    sys_status;
    mavlink_mount_status_t                  mount_status;
    mavlink_mount_orientation_t             mount_orienstation;
    mavlink_gimbal_device_attitude_status_t attitude_status;
    mavlink_gimbal_device_information_t     device_info;
    mavlink_raw_imu_t                       raw_imu;
    mavlink_command_ack_t                   ack;
    mavlink_param_value_t                    param_value;
    
#endif

} T_MavlinkProto;

/**
  * @brief Parameter structure
  * params_list contains gimbal information
  */
struct _param_list
{
    const uint8_t   gmb_idx;            /*< Gimbal parameter index */
    const char*     gmb_id;             /*< Gimbal parameter identify */
    int16_t         value;              /*< Gimbal parameter value */

    param_state_t   state;              /*< State's parameters */
    uint8_t         fetch_attempts;     /*< Retry to fetch parameter*/
    bool            seen;
} _params_list[GIMBAL_NUM_TRACKED_PARAMS] = {

    // Gimbal version
    {0, "VERSION_X", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {67, "VERSION_Y", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {68, "VERSION_Z", 0, PARAM_STATE_NOT_YET_READ, 0, false},

    // Gimbal stiffness
    {2, "PITCH_P", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {5, "ROLL_P", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {8, "YAW_P", 0, PARAM_STATE_NOT_YET_READ, 0, false},

    // Gimbal hold strength
    {11, "PITCH_POWER", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {12, "ROLL_POWER", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {13, "YAW_POWER", 0, PARAM_STATE_NOT_YET_READ, 0, false},

    {9, "YAW_I", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {29, "GYRO_LPF", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {3, "PITCH_I", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    
    // Gimbal speed follow
    {14, "PITCH_FOLLOW", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {16, "YAW_FOLLOW", 0, PARAM_STATE_NOT_YET_READ, 0, false},

    // Gimbal follow filter
    {17, "PITCH_FILTER", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {19, "YAW_FILTER", 0, PARAM_STATE_NOT_YET_READ, 0, false},

    // Gimbal follow windown
    {57, "TILT_WINDOW", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {58, "PAN_WINDOW", 0, PARAM_STATE_NOT_YET_READ, 0, false},

    // Gimbal speed control
    {60, "RC_PITCH_SPEED", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {61, "RC_ROLL_SPEED", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {62, "RC_YAW_SPEED", 0, PARAM_STATE_NOT_YET_READ, 0, false},

    // Gimbal smooth control
    {36, "RC_PITCH_LPF", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {37, "RC_ROLL_LPF", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {38, "RC_YAW_LPF", 0, PARAM_STATE_NOT_YET_READ, 0, false},

     // Mapping
    {50, "SKIP_GYRO_CALIB", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {18, "ROLL_FILTER", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    
    // Direction
    {63, "JOY_AXIS", 0, PARAM_STATE_NOT_YET_READ, 0, false},

    // Setting message rate
    {72, "HEARTBEAT_EMIT", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {73, "STATUS_RATE", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {74, "ENC_CNT_RATE", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {75, "ENC_TYPE_SEND", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {76, "ORIEN_RATE", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    {77, "IMU_RATE", 0, PARAM_STATE_NOT_YET_READ, 0, false},
    
    {30, "TRAVEL_MIN_PIT", -90, PARAM_STATE_NOT_YET_READ, 0, false},
    {31, "TRAVEL_MAX_PIT", 45, PARAM_STATE_NOT_YET_READ, 0, false},
    {32, "TRAVEL_MIN_ROLL", -45, PARAM_STATE_NOT_YET_READ, 0, false},
    {33, "TRAVEL_MAX_ROLL", 45, PARAM_STATE_NOT_YET_READ, 0, false},
    {69, "TRAVEL_MIN_PAN", -320, PARAM_STATE_NOT_YET_READ, 0, false},
    {70, "TRAVEL_MAX_PAN", 320, PARAM_STATE_NOT_YET_READ, 0, false},
    
    { 28, "RADIO_TYPE", 0, PARAM_STATE_NOT_YET_READ, 0, false },
};
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
static gremsy_mavlink_handler_t s_gimbalHandler;
/*
 * Gremsy System
 */
static T_GremsyGMB  s_GremsyGMB = {0};
volatile T_GremsyGMB *GremsyGMB;

/* Variable used for parameter */
static uint32_t     _last_request_ms    = 0;
static uint32_t     _last_set_ms        = 0;
const uint32_t      _retry_period       = 500;
const uint32_t      _max_fetch_attempts = 10;

static float        speedConversionFactor = 1.0f;

/*!< buffer used to receive data from UART interface from the gimbal*/
static uint8_t s_gimbalRecBuf[255];
static bool     s_hasDetectedGimbal = false;
/*!< Mavlink gimbal*/
static T_MavlinkProto mav_gimbal;

/*!< Mutex send gimbal message*/
static T_DjiMutexHandle s_mutexSend;
/*!< Mutex send gimbal message*/
static T_DjiMutexHandle s_mutexHandleParam;

/*!< Create two variables for 2 tasks handle*/
static T_DjiTaskHandle s_gimbalInterfaceThread;
static T_DjiTaskHandle s_gimbalControlThread;

/* Private function prototypes------------------------------------------------------------------------------*/
/** @brief Handle param value 
    @param1[in] msg const pointer to mavlink message
    @return None
*/
static void handle_param_value(const mavlink_message_t *msg);

/* Private functions------------------------------------------------------------------------------*/

/** @group __GSDK_GIMBAL_ABSTRACT_LAYER_FUNCTIONS_
    @{
*/#ifndef __GSDK_GIMBAL_ABSTRACT_LAYER_FUNCTIONS_
#define __GSDK_GIMBAL_ABSTRACT_LAYER_FUNCTIONS_
/**
 * @brief Low level mavlink frame read function.
 * @param pReadData Pointer to frame to be read.
 * @param dataLen Frame length.
 * @retval -1: Read error.
 *          not -1: Counts of bytes read actually.
 */
int comm_receive_bufer(uint8_t *pReadData, uint16_t dataLen)
{
    int realLen = 0;

    int i = 0;
    
    //Read UART data.
    realLen = UART_Read(GIMBAL_UART_NUM, pReadData, dataLen);
    
    return realLen;
}

/** @brief Function send mavlink data to serial
    @param1[in] chan Buffer number used to transmit data
    @param2[in] buf Pointer to buffer
    @param2[in] len Bytes to send
    @return None
*/
int gimbal_comm_send_buffer(const uint8_t *buf, uint16_t len)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    /*!< Acquire and lock the mutex when peripheral access is required. */
    if(osalHandler->MutexLock(s_mutexSend) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("[%s][%d]: Mutex Lock Failed");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    // Send data to serial
    int written = UART_Write(GIMBAL_UART_NUM, buf, len);

    /*!<  Unlock and release the mutex when done with the peripheral access.*/
    if(osalHandler->MutexUnlock(s_mutexSend) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("[%s][%d]: Mutex UnLock Failed");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    return written;
}


#endif
/**
    @}
*/

/** @group __GSDK_GIMBAL_CONNECTIONS_FUNCTIONS_
    @{
*/#ifndef __GSDK_GIMBAL_CONNECTIONS_FUNCTIONS_
#define __GSDK_GIMBAL_CONNECTIONS_FUNCTIONS_
/** @brief Function check gimbal has been connection to the PHUB system
    @param1[in] none
    @return true if the gimbal device has been connected 
            false gimbal went away
*/
bool GremsyGMB_IsPresent(void)
{
    uint32_t currentTime = 0;
    T_DjiReturnCode    returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    returnCode = osalHandler->GetTimeMs(&currentTime);
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("Get start time error: 0x%08llX.", returnCode);
        
        return false;
    }
    
    /* PHUB has lost connection with the gimbal device*/
    if(mav_gimbal.state != GIMBAL_STATE_NOT_PRESENT && 
        currentTime - mav_gimbal.last_report_msg_ms > 10000 || !s_hasDetectedGimbal)
    {
        s_hasDetectedGimbal = false;
        /* Gimbal went away*/
        mav_gimbal.state = GIMBAL_STATE_NOT_PRESENT;
        
        return false;
    }
    
    return true;
}

/** @brief Function check gimbal has been aligned with the drone (return home done)
    @param1[in] none
    @return true if the gimbal device has been connected and aligned 
            false gimbal went away
*/
bool GremsyGMB_IsAlgined(void)
{
    return GremsyGMB_IsPresent() && GIMBAL_STATE_PRESENT_RUNNING;
}

/** @brief Function converts gimbal id to string
    @param1[in] id
    @return string contain gimbal name
*/
const char *get_gimbal_model(uint8_t id)
{
    /*!< <o> GREMSY GIMBAL     <0x44=> T3V3 <0x22=> S1V3 <0x08=> T7V1 <0x0A=> PixyF 
                      <0x1A=> PixyFI <0x3A=> PixyU <0x4A=> PixyW <0x5A=> PixyD1 
                      <0x0B=> MIO      <0x0FE=> Test Pixy  <0x0FF=> Test T3V2
    */
    if(id == 0x44)
    {
        return "T3V3";
    }
    else if(id == 0x22)
    {
        return "S1V3";
    }
    else if(id == 0x08)
    {
        return "T7V1";
    }
    else if(id == 0x0A)
    {
        return "PixyF";
    }
    else if(id == 0x1A)
    {
        return "PixyFI";
    }
    else if(id == 0x3A)
    {
        return "PixyU";
    }
    else if(id == 0x4A)
    {
        return "PixyW";
    }
    else if(id == 0x0B)
    {
        return "PixyMIO";
    }
    else 
    {
        return "UNKNOWN";
    }
}

#endif
/**
    @}
*/


/** @group __GSDK_GIMBAL_MAVLINK_FUNCTIONS_
    @{
*/#ifndef __GSDK_GIMBAL_MAVLINK_FUNCTIONS_
#define __GSDK_GIMBAL_MAVLINK_FUNCTIONS_
/** @brief       Function send commad long to gimbal
  * @param1[in]  command command ID
  * @param2[in]  param param1 -> param7
    @return      T_DjiReturnCode
*/
static T_DjiReturnCode mavlinkSendCommandLong(uint16_t command, const float param[7])
{
    mavlink_message_t           msg             = { 0 };
    mavlink_command_long_t      command_long    = {0};
    
    uint8_t         systemid    = SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t         compid      = MAV_COMP_ID_ONBOARD_COMPUTER;
    uint8_t         chan        = GIMBAL_MAVLINK_CHANNEL;
    uint16_t        len         = 0;
    
    
    command_long.target_system      = GremsyGMB->info.sysid;
    command_long.target_component   = GremsyGMB->info.compid;
    command_long.command            = command;
    memcpy(&command_long.param1, param, 7 * sizeof(float));
    
    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    
    mavlink_msg_command_long_encode_chan(   systemid,
                                            compid,
                                            chan,
                                            &msg,
                                            &command_long);
                                            
    uint8_t msgbuf[GIMBAL_MAX_TRANCEIVER_BUFFER_SIZE];
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);
    
    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    }
    else {
        DebugError("Invalid length of packet");
         
         return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


/**
 * @brief  Function set angle to gimbal
 * @param1[in] pitch 
 * @param2[in] roll
 * @param3[in] yaw
 * @param4[in] control_mode
 * @return 
 */
T_DjiReturnCode GremsyGMB_SetReboot(void)
{
    /*!< Check GMB is connected */
    if(!GremsyGMB_IsPresent()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    uint16_t command = MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
    const float param[7] = 
    {
        0, /// param1
        0, /// param2
        0, /// param3
        1, /// param4
        0, /// param5
        0, /// param6
        0 /// param7
    };
    
    return mavlinkSendCommandLong(command, param);
}


/** @brief Function sends heartbeat to gimbal 1hz periodically 
    @param1[in] None
    @return None
*/
T_DjiReturnCode GremsyGMB_SendHeatbeat(void)
{
    mavlink_heartbeat_t heartbeat;
    
    /*!< Set default information*/
    uint8_t systemid    = SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_ONBOARD_COMPUTER;
    uint8_t chan        = GIMBAL_MAVLINK_CHANNEL;
    
    /*!< Set default*/
    heartbeat.type          = MAV_TYPE_ONBOARD_CONTROLLER; 
    heartbeat.autopilot     = MAV_AUTOPILOT_INVALID;
    heartbeat.base_mode     = 0;
    heartbeat.custom_mode   = 0; 
    heartbeat.system_status = MAV_STATE_ACTIVE;
    
    /*!< Message struct*/
    mavlink_message_t message_tx = {0};
    
    /*!< Encoder data to buffer*/
    mavlink_msg_heartbeat_encode_chan(  systemid,
                                        compid,
                                        chan,
                                        &message_tx,
                                        &heartbeat);
    uint8_t msgbuf[GIMBAL_MAX_TRANCEIVER_BUFFER_SIZE];
    uint16_t len = 0;
    
    /*!< Clear buffer*/
    memset(msgbuf, 0x00, GIMBAL_MAX_TRANCEIVER_BUFFER_SIZE);
    
    len = mavlink_msg_to_send_buffer(msgbuf, &message_tx);
    
    // Check length is valid
    if(len > 0) {
        // Send via MAVLINK_SEND_UART_BYTES 
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    }
    else {
        DebugError("Invalid length of packet");
         
         return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief  Function set rotation for gimbal
 * @param1[in] pitch 
 * @param2[in] roll
 * @param3[in] yaw
 * @param4[in] control_mode
 * @return 
 */
T_DjiReturnCode GremsyGMB_SetRotation(int16_t pitch, int16_t roll, int16_t yaw, E_DjiGimbalRotationMode rotationMode)
{
    /*!< Check GMB is connected */
    if(!GremsyGMB_IsPresent()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    uint16_t command = MAV_CMD_DO_MOUNT_CONTROL;
    const float param[7] = 
    {
        (float)pitch/10.0f, /// param1
        (float)roll/10.0f, /// param2
        (float)yaw/10.0f, /// param3
        0, /// param4
        0, /// param5
        rotationMode, /// param6
        MAV_MOUNT_MODE_MAVLINK_TARGETING /// param7
    };
    
    return mavlinkSendCommandLong(command, param);
}


/**
 * @brief  Function set angle to gimbal
 * @param1[in] tilt
 * @param2[in] roll
 * @param3[in] None
 * @param4[in] control_mode
 * @return 
 */
T_DjiReturnCode GremsyGMB_SetSpeed(int16_t pitchSpeed, int16_t rollSpeed, int16_t yawSpeed)
{
    /*!< Check GMB is connected */
    if(!GremsyGMB_IsPresent()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    uint16_t command = MAV_CMD_DO_MOUNT_CONTROL;
    
    T_DjiAttitude3f setPoint = {0};
    setPoint.pitch = (float)((pitchSpeed/10.0f) * speedConversionFactor);
    setPoint.roll = (float)((rollSpeed/10.0f) * speedConversionFactor);
    setPoint.yaw = (float)((yawSpeed/10.0f) * speedConversionFactor);

    const float param[7] = 
    {
        setPoint.pitch, /// param1
        setPoint.roll, /// param2
        setPoint.yaw, /// param3
        0, /// param4
        0, /// param5
        DJI_GIMBAL_ROTATION_MODE_SPEED, /// param6
        MAV_MOUNT_MODE_MAVLINK_TARGETING /// param7
    };
    
    return mavlinkSendCommandLong(command, param);
}

/**
 * @brief This function will be used for correct the drift
 * @details Send attitude of the drone periodically 50Hz
 * @note 
 */
T_DjiReturnCode GremsyGMB_SendAttitude(const T_DjiAttitude3d *aircraftAttitude, uint32_t millisecond)
{
    /*!< Check GMB is connected */
    if(!GremsyGMB_IsPresent()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    mavlink_message_t           msg;
    mavlink_attitude_t          attitude = {0};
    uint16_t                    len = 0;

    uint8_t systemid    = SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_ONBOARD_COMPUTER;
    uint8_t chan        = GIMBAL_MAVLINK_CHANNEL;
    
    /*!< Get attitude from the vehicle*/
    attitude.pitch          = (float)((aircraftAttitude->pitch / 10.0f) * DJI_PI / 180);
    attitude.roll           = (float)((aircraftAttitude->roll / 10.0f) * DJI_PI / 180);
    attitude.yaw            = (float)((aircraftAttitude->yaw / 10.0f) * DJI_PI / 180);
    attitude.time_boot_ms   = millisecond;
    
    mavlink_msg_attitude_encode_chan(   systemid,
                                        compid,
                                        chan,
                                        &msg,
                                        &attitude);
    
    uint8_t msgbuf[GIMBAL_MAX_TRANCEIVER_BUFFER_SIZE];
    
    /*!< Clear buffer */
    memset(msgbuf, 0x00, GIMBAL_MAX_TRANCEIVER_BUFFER_SIZE);
    
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief This function use send param request read to gimbal
 * @details Send msg param request read
 * @note 
 */
T_DjiReturnCode GremsyGMB_SendParamRequestRead(mavlink_param_request_read_t *param_request_read)
{
    /*!< Check GMB is connected */
    if(!GremsyGMB_IsPresent()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    mavlink_message_t           msg;
    mavlink_attitude_t          attitude = {0};
    uint16_t                    len = 0;

    uint8_t systemid    = SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_ONBOARD_COMPUTER;
    uint8_t chan        = GIMBAL_MAVLINK_CHANNEL;
    
    mavlink_msg_param_request_read_encode_chan( systemid,
                                                compid,
                                                chan,
                                                &msg,
                                                param_request_read);
    
    uint8_t msgbuf[GIMBAL_MAX_TRANCEIVER_BUFFER_SIZE];
    
    /*!< Clear buffer */
    memset(msgbuf, 0x00, GIMBAL_MAX_TRANCEIVER_BUFFER_SIZE);
    
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    } 
    else {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/*
 * @brief Set Free Mode
 * @details Pitch, roll, Yaw are controllable, meaning the gimbal can move independently of the aircraft's yaw.
 * In this mode, even if the product yaw changes, the camera will continue poiting in the same world direction.
 * @note 
 */
T_DjiReturnCode GremsyGMB_SetFreeMode(void)
{
    /*!< Check GMB is connected */
    if(!GremsyGMB_IsPresent()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    uint16_t command = MAVLINK_CMD_CTRL_GIMBAL_MODE;
    const float param[7] = 
    {
        0, /// param1
        0, /// param2
        0, /// param3
        0, /// param4
        0, /// param5
        0, /// param6
        GREMSY_GIMBAL_MODE_LOCK /// param7
    };
    
    return mavlinkSendCommandLong(command, param);
}

/**
 * @brief Set Yaw Follow Mode
 * @details Pitch and roll are controllable. Yaw will follow the products heading.
 * @note 
 */
T_DjiReturnCode GremsyGMB_SetYawFollowMode(void)
{
    /*!< Check GMB is connected */
    if(!GremsyGMB_IsPresent()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    uint16_t command = MAVLINK_CMD_CTRL_GIMBAL_MODE;
    const float param[7] = 
    {
        0, /// param1
        0, /// param2
        0, /// param3
        0, /// param4
        0, /// param5
        0, /// param6
        GREMSY_GIMBAL_MODE_FOLLOW /// param7
    };
    
    return mavlinkSendCommandLong(command, param);
}

/**
 * @brief This function supports for setting gimbal mode down
 * @details Function will reset yaw axis to the home position and set pitch axis to 90 degrees
 * @note This function allow control pitch and yaw with 2 dial button
 */
T_DjiReturnCode GremsyGMB_SetResetMode(uint8_t reset_mode)
{
    /*!< Check GMB is connected */
    if(!GremsyGMB_IsPresent()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
        uint16_t command = MAVLINK_CMD_CTRL_GIMBAL_MODE;
        const float param[7] = 
        {
            0, /// param1
            0, /// param2
            0, /// param3
            0, /// param4
            0, /// param5
            reset_mode, /// param6
            GREMSY_GIMBAL_RESET_MODE /// param7
        };
        
        return mavlinkSendCommandLong(command, param);
}

/** @addtogroup SET_GIMBAL_MOTOR
  * @{
  */
/**
  * @brief GremsyGMB_SetGimbalOff
  * Task gimbal will interface with the gimbal by conveying gimbal data 
  * to app as well as gimbal command to gimbal devixe
  */

T_DjiReturnCode GremsyGMB_SetGimbalOff(void)
{
	 /*!< Check GMB is connected */
    if(!GremsyGMB_IsPresent()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    uint16_t command = MAVLINK_CMD_CTRL_ON_OFF_GIMBAL;
    const float param[7] = 
    {
        0, /// param1
        0, /// param2
        0, /// param3
        0, /// param4
        0, /// param5
        0, /// param6
        TURN_OFF /// param7
    };
    
    return mavlinkSendCommandLong(command, param);

}

/**
  * @brief GremsyGMB_SetGimbalOn
  * Task gimbal will interface with the gimbal by conveying gimbal data 
  * to app as well as gimbal command to gimbal devixe
  */

T_DjiReturnCode GremsyGMB_SetGimbalOn(void)
{
	 /*!< Check GMB is connected */
    if(!GremsyGMB_IsPresent()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    uint16_t command = MAVLINK_CMD_CTRL_ON_OFF_GIMBAL;
    const float param[7] = 
    {
        0, /// param1
        0, /// param2
        0, /// param3
        0, /// param4
        0, /// param5
        0, /// param6
        TURN_ON /// param7
    };
    
    return mavlinkSendCommandLong(command, param);
}

T_DjiReturnCode GremsyGMB_IsRunning(uint8_t *isRunning)
{
    *isRunning = GremsyGMB->isRunning;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
/**
  * @} // SET_GIMBAL_MOTOR
  */


/**
 * @brief Handle mavlink message
 * @param msg Pointer to frame to be read.
 * @param out mavlink message after parsing
 * @retval - None
 */
void handle_mavlink_message(const mavlink_message_t* msg)
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
           
            if(!s_hasDetectedGimbal) {
                s_hasDetectedGimbal = true;
                USER_LOG_INFO("-------------------------Got HeartBeat from %s----------------------------\n", get_gimbal_model(packet.base_mode));
            }
            
            /*!< Get gimbal ID */
            GremsyGMB->info.type    = packet.base_mode;
            
            /*!< Get system id*/
            GremsyGMB->info.sysid   = msg->sysid;
            GremsyGMB->info.compid  = msg->compid;
            
            /* Get time report from the gimbal device*/
            osalHandler->GetTimeMs(&mav_gimbal.last_report_msg_ms);
            
            break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS:
        {
            mavlink_sys_status_t    sys_status;
            
            /* Call function decode status */
            mavlink_msg_sys_status_decode(msg, &sys_status);
            
#if defined(QE_CAM)
            memcpy(&mav_gimbal.sys_status, &sys_status, sizeof(mavlink_sys_status_t));
            
            mav_gimbal.isMsg[MAVLINK_MSG_ID_SYS_STATUS] = true;
#endif
    
            /*!< Using errors_count1 to get the gimbals status*/
            if((sys_status.errors_count1 & STATUS1_SENSOR_ERROR) || 
                (sys_status.errors_count1 & STATUS1_MOTOR_PHASE_ERROR) ||
                (sys_status.errors_count1 & STATUS1_MOTOR_ANGLE_ERROR)) {
                
                /*!< Specifies whether gimbal is stuck or not. */
                GremsyGMB->systemState.blockingFlag = true;
            }
            else {
                /*!< Specifies whether gimbal is stuck or not. */
                GremsyGMB->systemState.blockingFlag = false;
                
                if(sys_status.errors_count1 & STATUS1_MOTORS) {
                     GremsyGMB->isRunning = 1;
                }
                else {
                    GremsyGMB->isRunning = 0;
                }
            }
                
            /*!< Check gimbal is wherther running in inverted mode*/
            if(sys_status.errors_count1 & STATUS1_INVERTED) {
                /*!< Specifies whether gimbal is upward or not. */
                GremsyGMB->systemState.mountedUpward = true;
            }
            else {
                /*!< Specifies whether gimbal is upward or not. */
                GremsyGMB->systemState.mountedUpward = true;
            }
          
            /*!< TODO: Get param to check whether the ROTATION allow to control*/
            GremsyGMB->systemState.pitchRangeExtensionEnabledFlag = true;
            
            /*!< Check gimbal mode. Always set pitch axis as running in locking mode */
            if(sys_status.errors_count1 & STATUS1_MODE_FOLLOW_LOCK) {
                /*!< Set gimbal to follow yaw mode*/
                GremsyGMB->systemState.gimbalMode = DJI_GIMBAL_MODE_YAW_FOLLOW;
            }
            else {
                /*!< Set gimbal mode to Free mode*/
                GremsyGMB->systemState.gimbalMode = DJI_GIMBAL_MODE_FREE;
            }
            
            /*!< TODO: */
            /*!< Get gimbal Fine tune angles, unit: 0.1 degree. */
            GremsyGMB->systemState.fineTuneAngle.pitch = 0;
            GremsyGMB->systemState.fineTuneAngle.roll= 0;
            GremsyGMB->systemState.fineTuneAngle.yaw = 0;
            
            break;
        }
        case MAVLINK_MSG_ID_MOUNT_ORIENTATION:
        {
            mavlink_mount_orientation_t packet;
            
            /* Call function decode*/
            mavlink_msg_mount_orientation_decode(msg, &packet);

            DebugInfo("P %.3f - R %.3f Y-  %.3f\n", packet.pitch, packet.roll, packet.yaw_absolute);


#if defined(QE_CAM)
            memcpy(&mav_gimbal.mount_orienstation, &packet, sizeof(mavlink_mount_orientation_t));

            mav_gimbal.isMsg[MAVLINK_MSG_ID_MOUNT_ORIENTATION] = true;
#endif

            GremsyGMB->attitudeInformation.attitude.pitch    = packet.pitch*10;
            GremsyGMB->attitudeInformation.attitude.roll     = packet.roll*10;

            if(packet.yaw_absolute > 180) packet.yaw_absolute -= 360;
            if(packet.yaw_absolute < -180) packet.yaw_absolute += 360;
            
            GremsyGMB->attitudeInformation.attitude.yaw      = packet.yaw_absolute*10;
            
            if(packet.yaw > 180) packet.yaw -= 360;
            if(packet.yaw < -180) packet.yaw += 360;

            GremsyGMB->currentYaw   = packet.yaw*10;

            break;
        }
        case MAVLINK_MSG_ID_RAW_IMU:
        {
            mavlink_raw_imu_t   packet;
            
            /*!< Decode raw imu data*/
            mavlink_msg_raw_imu_decode(msg, &packet);
            
#if defined(QE_CAM)
            memcpy(&mav_gimbal.raw_imu, &packet, sizeof(mavlink_raw_imu_t));
            
            mav_gimbal.isMsg[MAVLINK_MSG_ID_RAW_IMU] = true;
#endif
            
            /*!< Get gimbal rotation speed. Unit 0.1 degree/s Ground coordination*/
            GremsyGMB->rotationSpeed.pitch = packet.xmag * 10;
            GremsyGMB->rotationSpeed.roll = packet.ymag * 10;
            GremsyGMB->rotationSpeed.yaw = packet.zmag* 10;
            
//            DebugWarning("P: %d - R: %d Y: %d\n", GremsyGMB->rotationSpeed.pitch
//                                    , GremsyGMB->rotationSpeed.roll, GremsyGMB->rotationSpeed.yaw);
            
            break;
        }
        case MAVLINK_MSG_ID_PARAM_VALUE:
        {
             /*!< Acquire and lock the mutex when peripheral access is required. */
            if(osalHandler->MutexLock(s_mutexHandleParam) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                DebugError("[%s][%d]: Mutex Lock Failed");
            }
                /* Handle param value*/
                handle_param_value(msg);

            /*!<  Unlock and release the mutex when done with the peripheral access.*/
            if(osalHandler->MutexUnlock(s_mutexHandleParam) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                DebugError("[%s][%d]: Mutex UnLock Failed");
            }
            
#if defined(QE_CAM)
            mavlink_param_value_t param_value = {0};
            
            mavlink_msg_param_value_decode(msg, &param_value);
            
            memcpy(&mav_gimbal.param_value, &param_value, sizeof(mavlink_param_value_t));
            
            mav_gimbal.isMsg[MAVLINK_MSG_ID_PARAM_VALUE] = true;
#endif

            break;
        }
#if defined(QE_CAM)
        case MAVLINK_MSG_ID_MOUNT_STATUS:
        {
            mavlink_mount_status_t mount_status;
            
            /// call function decode
            mavlink_msg_mount_status_decode(msg, &mount_status);
            
            memcpy(&mav_gimbal.mount_status, &mount_status, sizeof(mavlink_mount_status_t));
            
            mav_gimbal.isMsg[MAVLINK_MSG_ID_MOUNT_STATUS] = true;
            
            break;
        }
        case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
        {
            mavlink_gimbal_device_attitude_status_t packet;
            
            /// call function decode
            mavlink_msg_gimbal_device_attitude_status_decode(msg, &packet);
            
            memcpy(&mav_gimbal.attitude_status, &packet, sizeof(mavlink_gimbal_device_attitude_status_t));
            
            mav_gimbal.isMsg[MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS] = true;
            
            break;
        }
        case MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION:
        {
            mavlink_gimbal_device_information_t packet;
            
            /// call function decode
            mavlink_msg_gimbal_device_information_decode(msg, &packet);
            
            memcpy(&mav_gimbal.device_info, &packet, sizeof(mavlink_gimbal_device_information_t));
            
            mav_gimbal.isMsg[MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION] = true;
            
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_ACK:
        {
            mavlink_command_ack_t packet;
            
            /// call function decode
            mavlink_msg_command_ack_decode(msg, &packet);
            
            memcpy(&mav_gimbal.ack, &packet, sizeof(mavlink_command_ack_t));
            
            mav_gimbal.isMsg[MAVLINK_MSG_ID_COMMAND_ACK] = true;
            
            DebugWarning("Got command ack from Gimbal !!!");
            
            break;
        }
#endif
    }
}

#endif
/**
    @}
*/

/** @group __GSDK_GIMBAL_PARAMETERS_FUNCTIONS_
    @{
*/#ifndef __GSDK_GIMBAL_PARAMETERS_FUNCTIONS_
#define __GSDK_GIMBAL_PARAMETERS_FUNCTIONS_
/** @brief Function get param name
    @param1[in] param index of the param
    @return param name 
*/
const char* get_param_name(param_index_t param)
{
    return _params_list[param].gmb_id;
}

/** @brief Get param index in the param list
    @param1[in] param
    @return param index in the list
*/
uint8_t get_gmb_index(param_index_t param)
{
    return _params_list[param].gmb_idx;
}

/** @brief Function reset params at the beginning
    @param1[in] none
    @return None
*/
static void reset_params(void)
{
    /* Reset time */
    _last_request_ms    = 0;
    _last_set_ms        = 0;
    
    /* Scan the array parameters */
    for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++)
    {
        _params_list[i].value           = 0;
        _params_list[i].state           = PARAM_STATE_NOT_YET_READ;
        _params_list[i].fetch_attempts  = 0;
        _params_list[i].seen            = false;
    }
}

/** @brief Function fetch all params
    @param1[in] none
    @param2[in] None
    @return None
*/
void fetch_params(void)
{
    for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++)
    {
        /* Check if the param has been read before then fetching again*/
        if(_params_list[i].state != PARAM_STATE_NOT_YET_READ)
        {
            /* Set the state to fetch again*/
            _params_list[i].state = PARAM_STATE_FETCH_AGAIN;
        }
    }
}

/** @brief Function check parameter has read
    @param1[in] none
    @param2[in] None
    @return true    - Read
            false   - Not Yet
*/
static bool params_initialized(void)
{
    for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++) {
        /* Check if the param hasn't read yet*/
        if(_params_list[i].state == PARAM_STATE_NOT_YET_READ) {
            return false;
        }
    }
    
    return true;
}

/** @brief Function check params already received 
    @param1[in] none
    @param2[in] None
    @return return the index if the param list has been update then return GIMBAL_NUM_TRACKED_PARAMS
*/
static uint8_t params_received_all(void)
{
    for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++)
    {
        /* Check the state of param if not read or fetch */
        if(_params_list[i].state == PARAM_STATE_NOT_YET_READ || 
                _params_list[i].state == PARAM_STATE_FETCH_AGAIN)
        {
            return i;
        }
    }
    return GIMBAL_NUM_TRACKED_PARAMS;
}

/** @brief Function get param value from params_list
    @param1[in] param - index of param 
    @return value of that param
*/
static void get_param(param_index_t param, int16_t* value)
{
    // check param has been seen
    if(_params_list[param].seen == true)
    {
        *value = _params_list[param].value;
    }
    else 
    {
        *value = 0;
    }
}

/** @brief Function set param value
    @param1[in] param index of the param
    @param2[in] value the value wants to set
    @return none
*/

static void set_param(param_index_t param, const int16_t value)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    uint8_t system_id   = SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_ONBOARD_COMPUTER;
    uint8_t chan        = GIMBAL_MAVLINK_CHANNEL;
    
    /* check param if it needs to fetch or the value is consistent */
    if((_params_list[param].state == PARAM_STATE_CONSISTENT ) && (_params_list[param].value == value)) {
        DebugWarning("Param is consistent\n");
        
        return;
    }
    
//    if(_params_list[param].state == PARAM_STATE_NONEXISTANT) {
//        DebugWarning("Param is not exist\n");
//        return;
//    }
    
    DebugInfo("ID %d - val: %d", param, value);
    
    _params_list[param].value = value;
    _params_list[param].state = PARAM_STATE_ATTEMPTING_TO_SET;
    _params_list[param].fetch_attempts = 0;
    
    /* Prepare command for on-board mode*/
    mavlink_param_set_t param_set = { 0 };
    
    param_set.param_value       = value;
    param_set.target_system     = GremsyGMB->info.sysid;            /*<  Gimbal System ID*/
    param_set.target_component  = GremsyGMB->info.compid;            /*<  Gimbal Component ID*/
    
    mav_array_memcpy(param_set.param_id, get_param_name(param), sizeof(char)*16);
    param_set.param_type    = MAVLINK_TYPE_UINT16_T;
    
    mavlink_message_t   msg;
    mavlink_msg_param_set_encode(system_id, compid, &msg, &param_set);
    
    uint8_t msgbuf[GIMBAL_MAX_TRANCEIVER_BUFFER_SIZE] = {0};
    uint8_t len = 0;
    
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    if(len > 0) {
        _mavlink_send_uart(chan, (const char*) msgbuf, len);
    }
    else {
        DebugError("Error mavlink send uart");
    } 
    
    /* Get the time from the sending the param*/
    T_DjiReturnCode returnCode = osalHandler->GetTimeMs(&_last_set_ms);
    
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("Get current time error: 0x%08llX.", returnCode);
    }
}

/** @brief Function process parameter interface with gimbal
    @param1[in] none
    @param2[in] None
    @return None
*/
static void param_update(void)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    uint32_t tnow_ms = 0;
    
    T_DjiReturnCode returnCode = osalHandler->GetTimeMs(&tnow_ms);
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("Get current time error: 0x%08llX.", returnCode);
    }
    
    uint8_t system_id   = SYSTEM_CONTROL_SYSTEM_ID;
    uint8_t compid      = MAV_COMP_ID_ONBOARD_COMPUTER;
    uint8_t chan        = GIMBAL_MAVLINK_CHANNEL;
    
    for(uint8_t param_index = 0; param_index < GIMBAL_NUM_TRACKED_PARAMS; param_index++)
    {
        /* Check the state of param if not read or fetch */
        if(_params_list[param_index].state == PARAM_STATE_NOT_YET_READ || _params_list[param_index].state == PARAM_STATE_FETCH_AGAIN)
        {
            /* Check the param if received all*/
            if((tnow_ms - _last_request_ms) > 100)
            {
                /* Get the time for the next time*/
                _last_request_ms = tnow_ms;
                
                /* Check the param which not seen before*/
                if(!_params_list[param_index].seen)
                {
                    mavlink_param_request_read_t    request = {0};
                    
                    request.target_system       = GremsyGMB->info.sysid;
                    request.target_component    = GremsyGMB->info.compid;
                    request.param_index         = _params_list[param_index].gmb_idx;
                    
                    // Get param name from the params_list
                    mav_array_memcpy(request.param_id, get_param_name((param_index_t)param_index), sizeof(char)*16);
                    
                    DebugInfo("Request param read: %s \n", get_param_name((param_index_t)param_index));
                    
                    // --------------------------------------------------------------------------
                    //   SEND MSG 
                    // --------------------------------------------------------------------------
                    GremsyGMB_SendParamRequestRead(&request);
                    
                    /* Send request read try again*/
                    _params_list[param_index].fetch_attempts++;
                }
            }
        }
    }
    
    /* Retry param set */
    for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++)
    {
        /* Check the state of param*/
        if((_params_list[i].state == PARAM_STATE_ATTEMPTING_TO_SET) && 
            ((tnow_ms - _last_set_ms) > _retry_period))
        {
            /* Get the time*/
            _last_set_ms = tnow_ms;
            
            /* Prepare command for on-board mode*/
            mavlink_param_set_t param_set = { 0 };
            
            param_set.param_value       = _params_list[i].value;
            param_set.target_system     = GremsyGMB->info.sysid;//            /*<  Gimbal System ID*/
            param_set.target_component  = GremsyGMB->info.compid;//            /*<  Gimbal Component ID*/
            
            mav_array_memcpy(param_set.param_id, get_param_name(i), sizeof(char)*16);
            param_set.param_type    = MAVLINK_TYPE_UINT16_T;
            
            mavlink_message_t   msg;
            mavlink_msg_param_set_encode(system_id, compid, &msg, &param_set);
            
            uint8_t msgbuf[GIMBAL_MAX_TRANCEIVER_BUFFER_SIZE] = {0};
            uint8_t len = 0;
            
            len = mavlink_msg_to_send_buffer(msgbuf, &msg);

            if(len > 0) {
                _mavlink_send_uart(chan, (const char*) msgbuf, len);
            }
            else {
                DebugError("Error mavlink send uart");
            } 
            
            /* Check the param */
            if (!_params_list[i].seen) {
                _params_list[i].fetch_attempts++;
            }
            
            DebugInfo("Param state :%d", _params_list[i].state);
            
            /*!< Reset and fetch again */
            _params_list[i].state = PARAM_STATE_FETCH_AGAIN;
            _params_list[i].seen = false;
        }
        
        /* Check for nonexistent parameters*/
        if(!_params_list[i].seen && _params_list[i].fetch_attempts > _max_fetch_attempts)
        {
            /* Set the state to NONEXISTANT*/
//            _params_list[i].state= PARAM_STATE_NONEXISTANT;

            DebugMsg("Gimbal parameter %s timed out\n", get_param_name((param_index_t)i));

        }
    }
}

/** @brief Handle param value 
    @param1[in] msg const pointer to mavlink message
    @return None
*/
static void handle_param_value(const mavlink_message_t *msg)
{
    /* Declare a mavlink param packet*/
    mavlink_param_value_t packet = {0};
    
    /* Decode mavlink param value*/
    mavlink_msg_param_value_decode(msg, &packet);
    
    /* Scan in the param list*/
    for(uint8_t i = 0; i < GIMBAL_NUM_TRACKED_PARAMS; i++)
    {
        /* Compare param id from the param list and param receiced */
        if(!strcmp(packet.param_id, get_param_name((param_index_t)i)))
        {
            /* Set the flag has been seen*/
            _params_list[i].seen = true;
            _params_list[i].fetch_attempts = 0;
            
            /* Check the state of the param*/
            switch(_params_list[i].state)
            {
                case PARAM_STATE_NONEXISTANT:
                case PARAM_STATE_NOT_YET_READ:
                {
                    /* Get value to list*/
                    _params_list[i].value =  (int16_t)packet.param_value;
                    
                    /* Update the state for that param*/
                    _params_list[i].state = PARAM_STATE_CONSISTENT;

                    DebugInfo("GOT [%s] %.2f\n", get_param_name((param_index_t)i), _params_list[i].value);

                    break;
                }
                case PARAM_STATE_FETCH_AGAIN:
                {

                    DebugInfo("GOT_FETCH [%s] %d\n", get_param_name((param_index_t)i), (int16_t)packet.param_value);

                    /*!< In case fetch param again but the value has been set before is different with the value got gimbal*/
                    if(_params_list[i].value !=  (int16_t)packet.param_value) {
                        _params_list[i].state = PARAM_STATE_FETCH_AGAIN;
                    }
                    else {
                        /* Get value to list*/
                        _params_list[i].value =  (int16_t)packet.param_value;
                        
                        /* Update the state for that param*/
                        _params_list[i].state = PARAM_STATE_CONSISTENT;
                    }
                    break;
                }

                case PARAM_STATE_CONSISTENT:
                {
                    _params_list[i].value = (int16_t)packet.param_value;
                    break;
                }
                case PARAM_STATE_ATTEMPTING_TO_SET:
                {
                    if(packet.param_value == _params_list[i].value)
                    {
                        _params_list[i].state = PARAM_STATE_CONSISTENT;
                    }
                    break;
                }
            }
            break;
        }
    }
}

/**
 * @brief  This function set motor controls setting
 * @param: tilt, roll, pan - stiffness and holdstrengtg, see user_manual (https://gremsy.com/gremsy-t3-manual/)
 * @param: gyro_filter - The coefficent for denoising the sensor filter
 * @param: output_filter - The coefficent for denoising the output filter
 *  HOLD STRENGTH   TILT    ROLL    PAN
 *                  40      40      40
 *  GAIN            120     120     120
 * @Note - Gain set as default value 120
 */
void GremsyGMB_SetAxisParameters(   const axis_setting_t *pitch, 
                                    const axis_setting_t *roll,
                                    const axis_setting_t *yaw)
{
    int16_t get_dir;

    /* Set param for pitch axis axis */
    set_param(GMB_PARAM_STIFFNESS_PITCH, (int16_t)pitch->stiffness);
    set_param(GMB_PARAM_HOLDSTRENGTH_PITCH, (int16_t)pitch->holdstrength);
    
    // Set default windown = 0. Do not apply the window
//    set_param(GMB_PARAM_WINDOW_FOLLOW_PITCH,(int16_t)pitch->window_follow);
    set_param(GMB_PARAM_WINDOW_FOLLOW_PITCH,(int16_t)0);
    
    get_dir = 0;
    get_param(GMB_PARAM_AXIS_DIR, &get_dir);

    if( pitch->dir == AXIS_DIR_CCW) {
        get_dir = get_dir | 0x01;
    }
    else {
        get_dir &= (~0x01);
    }
    
    
    // Default Tilt Down = negative value
    get_dir &= (~0x01);
    set_param(GMB_PARAM_AXIS_DIR, get_dir);
    
    /* Set param for roll axis axis */
    set_param(GMB_PARAM_STIFFNESS_ROLL, (int16_t)roll->stiffness);
    set_param(GMB_PARAM_HOLDSTRENGTH_ROLL, (int16_t)roll->holdstrength);
    
    // Default roll always maximum speed
//    set_param(GMB_PARAM_SMOOTH_CONTROL_ROLL, (int16_t)roll->smooth_control);
//    set_param(GMB_PARAM_SPEED_CONTROL_ROLL, (int16_t)roll->speed_control);
    set_param(GMB_PARAM_SMOOTH_CONTROL_ROLL, (int16_t)0);
    set_param(GMB_PARAM_SPEED_CONTROL_ROLL, (int16_t)100);

    get_dir = 0;
    get_param(GMB_PARAM_AXIS_DIR, &get_dir);

    if( roll->dir == AXIS_DIR_CCW)
    {
        get_dir = get_dir | 0x04;
    }
    else
    {
        get_dir &= (~0x04);
    }
    set_param(GMB_PARAM_AXIS_DIR, get_dir);
    
    /* Set param for yaw axis axis */
    set_param(GMB_PARAM_STIFFNESS_YAW, (int16_t)yaw->stiffness);
    set_param(GMB_PARAM_HOLDSTRENGTH_YAW, (int16_t)yaw->holdstrength);
    
    /*!< Default window always set to zero*/
//    set_param(GMB_PARAM_WINDOW_FOLLOW_YAW,(int16_t)yaw->window_follow);
    set_param(GMB_PARAM_WINDOW_FOLLOW_YAW,(int16_t)0);

    get_dir = 0;
    get_param(GMB_PARAM_AXIS_DIR, &get_dir);

    if( yaw->dir == AXIS_DIR_CCW) {
        get_dir = get_dir | 0x02;
    }
    else {
        get_dir &= (~0x02);
    }
    
    /*!< Default pan always follow the right-hand rule*/
    get_dir &= (~0x02);
    set_param(GMB_PARAM_AXIS_DIR, get_dir);
}

/**
 * @brief  This function get motor controls setting
 * @param: tilt, roll, pan - stiffness and holdstrengtg, see user_manual (https://gremsy.com/gremsy-t3-manual/)
 * @param: gyro_filter - The coefficent for denoising the sensor filter
 * @param: output_filter - The coefficent for denoising the output filter
 * @param: gain - Defines how fast each axis will return to commanded position. 
 * @ret: gimbal_motor_control_t contains setting related to tilt axis
 *
 *	HOLD STRENGTH 	TILT 	ROLL 	PAN
 *					40 		40 		40
 * 	GAIN 			120		120		120
 */
void get_axis_parameters(volatile axis_setting_t *pitch, 
                         volatile axis_setting_t *roll,
                         volatile axis_setting_t *yaw)
{
    int16_t value = 0;

    /* Get parameter setting for pitch axis*/
    get_param(GMB_PARAM_STIFFNESS_PITCH, &value);
    pitch->stiffness        =  (uint8_t)value;
    get_param(GMB_PARAM_HOLDSTRENGTH_PITCH, &value);
    pitch->holdstrength     =  (uint8_t)value;

    get_param(GMB_PARAM_SMOOTH_CONTROL_PITCH, &value);
    pitch->smooth_control = (uint8_t)value;

    get_param(GMB_PARAM_SMOOTH_FOLLOW_PITCH, &value);
    pitch->smooth_follow = (uint8_t)value; 

    get_param(GMB_PARAM_WINDOW_FOLLOW_PITCH, &value);
    pitch->window_follow = (uint8_t)value; 

    get_param(GMB_PARAM_SPEED_FOLLOW_PITCH,  &value);
    pitch->speed_follow = (uint8_t)value;

    get_param(GMB_PARAM_SPEED_CONTROL_PITCH, &value);
    pitch->speed_control = (uint8_t)value;

    get_param(GMB_PARAM_AXIS_DIR, &value);

    if(value & 0x01)
    {
        pitch->dir = AXIS_DIR_CCW;
    }
    else if(!(value & 0x01))
    {
        pitch->dir = AXIS_DIR_CW;
    }
    
    /* Get param setting form the roll axiss */
    get_param(GMB_PARAM_STIFFNESS_ROLL, &value);
    roll->stiffness         =  (uint8_t)value;
    get_param(GMB_PARAM_HOLDSTRENGTH_ROLL, &value);
    roll->holdstrength      =  (uint8_t)value;
    
    get_param(GMB_PARAM_SMOOTH_CONTROL_ROLL, &value);
    roll->smooth_control = (uint8_t)value;

    // Roll dosen't support in follow mode
    roll->smooth_follow         = 0; 
    roll->window_follow         = 0; 
    roll->speed_follow          = 0;

    get_param(GMB_PARAM_SPEED_CONTROL_ROLL, &value);
    roll->speed_control = (uint8_t)value;

    get_param(GMB_PARAM_AXIS_DIR, &value);

    if(value & 0x04)
    {
        roll->dir = AXIS_DIR_CCW;
    }
    else if(!(value & 0x04))
    {
        roll->dir = AXIS_DIR_CW;
    }
    
    /* Get parameters from the gimbal*/
    get_param(GMB_PARAM_STIFFNESS_YAW, &value);
    yaw->stiffness          =  (uint8_t)value;
    get_param(GMB_PARAM_HOLDSTRENGTH_YAW, &value);
    yaw->holdstrength        =  (uint8_t)value;
    
    get_param(GMB_PARAM_SMOOTH_CONTROL_YAW, &value);
    yaw->smooth_control = (uint8_t)value; 

    get_param(GMB_PARAM_SMOOTH_FOLLOW_YAW, &value);
    yaw->smooth_follow = (uint8_t)value; 

    get_param(GMB_PARAM_WINDOW_FOLLOW_YAW, &value);
    yaw->window_follow = (uint8_t)value; 

    get_param(GMB_PARAM_SPEED_FOLLOW_YAW, &value);
    yaw->speed_follow = (uint8_t)value;

    get_param(GMB_PARAM_SPEED_CONTROL_YAW,&value);
    yaw->speed_control = (uint8_t)value;

    get_param(GMB_PARAM_AXIS_DIR, &value);

    if(value & 0x02)
    {
        yaw->dir = AXIS_DIR_CCW;
    }
    else if(!(value & 0x02))
    {
        yaw->dir = AXIS_DIR_CW;
    }
}

/**
 * @brief  This function set motor controls setting
 * @param: tilt, roll, pan - stiffness and holdstrengtg, see user_manual (https://gremsy.com/gremsy-t3-manual/)
 * @param: gyro_filter - The coefficent for denoising the sensor filter
 * @param: output_filter - The coefficent for denoising the output filter
 *  HOLD STRENGTH   TILT    ROLL    PAN
 *                  40      40      40
 *  GAIN            120     120     120
 * @Note - Gain set as default value 120
 */
void GremsyGMB_SetGyroFilter(const uint8_t gyroFilter)
{
    set_param(GMB_PARAM_GYRO_FILTER, (int16_t)gyroFilter);
}

/** @brief Function get filter
    @param1[in] none
    @param2[in] None
    @return None
*/

void GremsyGMB_GetGyroFilter(uint8_t *gyroFilter)
{
    int16_t value = 0;
    get_param(GMB_PARAM_GYRO_FILTER, &value);
    *gyroFilter             = (uint8_t)value;
}

/**
 * @brief  This function set motor controls setting
 * @param: tilt, roll, pan - stiffness and holdstrengtg, see user_manual (https://gremsy.com/gremsy-t3-manual/)
 * @param: gyro_filter - The coefficent for denoising the sensor filter
 * @param: output_filter - The coefficent for denoising the output filter
 *  HOLD STRENGTH   TILT    ROLL    PAN
 *                  40      40      40
 *  GAIN            120     120     120
 * @Note - Gain set as default value 120
 */
void GremsyGMB_SetOutputFilter(const uint8_t outputFilter)
{
    set_param(GMB_PARAM_OUTPUT_FILTER, (int16_t)outputFilter);
}

/** @brief Function get filter
    @param1[in] none
    @param2[in] None
    @return None
*/

void GremsyGMB_GetOutputFilter(uint8_t *outputFilter)
{
    int16_t value = 0;
    get_param(GMB_PARAM_OUTPUT_FILTER, &value);
    *outputFilter       = (uint8_t)value;
}

/** @brief Function get mapping
    @param1[in] none
    @param2[in] None
    @return None
*/
T_DjiReturnCode GremsyGMB_GetMappingAngle(int8_t *pitchDownAngle) {
    int16_t value = 0;
    get_param(GMB_MAPPING_ANGLE, &value);
    *pitchDownAngle             = (uint8_t)value;
}

/**
 * @brief This function used to set some parameters as default value
 * @details DIR need follow the DJI Gimbal
 * @note 
 */
static T_DjiReturnCode GimbalGremsy_SetParamDefault(void)
{
    /*!< Set default direction for pitch/roll/yaw axis. Revert tilt gremsy to tilt DJI up + . Enable set correct difting */
    int16_t direction = 0x11;
    set_param(GMB_PARAM_AXIS_DIR, direction);
    
    /*!< Set default no support window*/
    set_param(GMB_PARAM_WINDOW_FOLLOW_PITCH, 0);
    set_param(GMB_PARAM_WINDOW_FOLLOW_YAW, 0);
    
    /*!< Set default control for Roll */
    set_param(GMB_PARAM_SMOOTH_CONTROL_ROLL, (int16_t)0);
    set_param(GMB_PARAM_SPEED_CONTROL_ROLL, (int16_t)100);
    
    /*!< Disable Mapping mode */
    set_param(GMB_MAPPING_ENABLE, 0);
    set_param(GMB_MAPPING_ANGLE, 0);
    
    /*!< Set default message rate*/
    set_param(GMB_PARAM_HEATBEAT_EMIT, 1);
    set_param(GMB_PARAM_STATUS_RATE, 10);
    /*!< Temporary set encoder is disable */
    set_param(GMB_PARAM_ENCODER_VALUE_RATE, 0);
    set_param(GMB_PARAM_ENCODER_TYPE, 0);
    
    set_param(GMB_PARAM_ORIENTATION_RATE, 50);
    set_param(GMB_PARAM_RAW_IMU_RATE, 0);

    /*!< Set default limit angle for gimbal */
    set_param(GMB_PARAM_MIN_LIMIT_ANGLE_PITCH, -90);
    set_param(GMB_PARAM_MAX_LIMIT_ANGLE_PITCH, 45);
    
    set_param(GMB_PARAM_MIN_LIMIT_ANGLE_ROLL, -40);
    set_param(GMB_PARAM_MAX_LIMIT_ANGLE_ROLL, 40);
    
    set_param(GMB_PARAM_MIN_LIMIT_ANGLE_YAW, -320);
    set_param(GMB_PARAM_MAX_LIMIT_ANGLE_YAW, 320);
}

/**
 * @brief Function process param
 * @details This function will be calked in the loop for check the parameters
 * @note 
 */
T_DjiReturnCode GimbalGremsy_ParameterProcess(void)
{
    bool isGimbalConnection = false;
    
    /*!< Check GMB is connected */
    isGimbalConnection = GremsyGMB_IsPresent();
    
    if(!isGimbalConnection) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }
    
    /*!< Check GMB state */
    switch(mav_gimbal.state)
    {
        case GIMBAL_STATE_NOT_PRESENT:
        {
            /* Reset param */
            reset_params();
            
            if(isGimbalConnection == true)
            {
                /* Switch to next state*/
                mav_gimbal.state = GIMBAL_STATE_PRESENT_INITIALIZING;
            }
            
            break;
        }
        case GIMBAL_STATE_PRESENT_INITIALIZING:
        {
            /* Reset and update param to param list*/
            param_update();
            
            /* Check if the PHUB got all param from the gimbal*/
            if(params_initialized()) {
                mav_gimbal.state = GIMBAL_STATE_PRESENT_ALIGNING;
            }
            
            break;
        }
        case GIMBAL_STATE_PRESENT_ALIGNING:
        {
            /* Check whether gimbal has been aligned with the drone*/
            param_update();

            /* Check gimbal status if it is running*/
            if(!GremsyGMB->systemState.blockingFlag) {
                
                 /*!< Set Default Value with specific parameters */
                 GimbalGremsy_SetParamDefault();
                
                 /*!< Set dafault mode */
                 GremsyGMB_SetResetMode(3);
                
                /*!< Set gimbal default Follow mode */
                GremsyGMB_SetYawFollowMode();
                
                /*!< Switch to next state*/
                mav_gimbal.state = GIMBAL_STATE_PRESENT_RUNNING;
            }
            break;
        }
        case GIMBAL_STATE_PRESENT_RUNNING:
        {
            param_update();
            
            /* Allow send gimbal mode and gimbal control*/
            break;
        }
    }
}

#endif
/**
    @}
*/

/** @group __GSDK_GIMBAL_CONTROL_FUNCTIONS_
    @{
*/#ifndef __GSDK_GIMBAL_CONTROL_FUNCTIONS_
#define __GSDK_GIMBAL_CONTROL_FUNCTIONS_
/** @brief  ham gui heartbeat den gimbal
    @return T_DjiReturnCode
*/
T_DjiReturnCode GimbalGremsy_sendHeartbeattoGimbal(void)
{
    T_DjiReturnCode returnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    
    returnCode = GremsyGMB_SendHeatbeat();
    
    if(returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        DebugError("send heartbeat to gimbal\n");
        
        return returnCode;
    }
    
    return returnCode;
}

/*!<====================== PITCH_CONFIG_FUNCTIONS ========================== */

/** @addtogroup PITCH_CONFIG_FUNCTIONS
  * @{
  */

/**
 * @brief Set speed conversion factor for speed control from joystick and APP.
 * @details XPort convert speed control command from joystick and APP to rotation speed based on the speed conversion
 * factor. The formula is "speed = maximum rotation speed * conversion factor", and maximum rotation speed is a product of
 * default maximum speed and maximum speed percentage. The default maximum rotation speed is 90degree/s. The maximum
 * speed percentage is set by APP. The default speed conversion factor is 1.0.
 * @note The value will be effective after a while, and the max value is 100ms.
 * @param factor: Speed conversion factor and it has to be smaller than or equal to 1.0.
 * @return Execution result.
 */
T_DjiReturnCode GremsyGMB_SetSpeedConversionFactor(float factor) 
{
    if(factor > 1.0f) {
        speedConversionFactor = 1.0f;
    } else if (factor < 0.01f){
        speedConversionFactor = 0.01f;
    } else {
        speedConversionFactor = factor;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Function used to set stiffness
 * @details If the gimbal has oscillations that cannot be corrected by adjusting stiffness settings
 * @note 
 */
T_DjiReturnCode GremsyGMB_SetPitchStiffness(const uint8_t stiffness) 
{
    /* Set param for pitch axis axis */
    set_param(GMB_PARAM_STIFFNESS_PITCH, (int16_t)stiffness);
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Function used to get stiffness
 * @details If the gimbal has oscillations that cannot be corrected by adjusting stiffness settings
 * @note 
 */
T_DjiReturnCode GremsyGMB_GetPitchStiffness(uint8_t *stiffness) 
{
    int16_t value;
    /* Set param for pitch axis axis */
    get_param(GMB_PARAM_STIFFNESS_PITCH, &value);
    *stiffness        =  (uint8_t)value;
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Function used to set Hold Strength motor
 * @details Constant power level provided to the corresponding axis and should be aajust manual.
 * @note For heavy cameras, it's suggested to increase hold strength for each axis around 10%
 * than the default value (40).
 */
T_DjiReturnCode GremsyGMB_SetPitchHoldStrength(const uint8_t holdStrength) 
{
    /* Set param for pitch axis axis */
    set_param(GMB_PARAM_HOLDSTRENGTH_PITCH, (int16_t)holdStrength);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Function used to get Hold Strength motor
 * @details Constant power level provided to the corresponding axis and should be aajust manual.
 * @note For heavy cameras, it's suggested to increase hold strength for each axis around 10%
 * than the default value (40).
 */
T_DjiReturnCode GremsyGMB_GetPitchHoldStrength(uint8_t *holdStrength) 
{
    int16_t value;
    /* Get parameter setting for pitch axis*/
    get_param(GMB_PARAM_HOLDSTRENGTH_PITCH, &value);
    *holdStrength     =  (uint8_t)value;
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Function used to set speed pitch axis
 * @details The speed value has been calculated depending on the application
 * @note Pitch does not support follow
 */
T_DjiReturnCode GremsyGMB_SetPitchSpeed(const uint8_t speed)
{
    /*!< Check gimbal mode first for specifying the speed mode that mode */
//    if(GremsyGMB->systemState.gimbalMode == PSDK_GIMBAL_MODE_FREE) {
    
//        int16_t speedControl = speed*speedConversionFactor;
//        set_param(GMB_PARAM_SPEED_CONTROL_PITCH, (int16_t)speedControl);
    
    set_param(GMB_PARAM_SPEED_CONTROL_PITCH, (int16_t)speed);
//        PsdkLogger_UserLogInfo("set speed control : %d ", speed);
//    }
//    else {
//        set_param(GMB_PARAM_SPEED_FOLLOW_PITCH, (int16_t)speed);
//        
//        PsdkLogger_UserLogInfo("set speed follow : %d ", speed);

//    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


/**
 * @brief Function used to get speed pitch axis
 * @details The speed value has been calculated depending on the application
 * @note 
 */
T_DjiReturnCode GremsyGMB_GetPitchSpeed(uint8_t *speed)
{
    int16_t value = 0;
    
    /*!< Check gimbal mode first for specifying the speed mode that mode */
    if(GremsyGMB->systemState.gimbalMode == DJI_GIMBAL_MODE_FREE) {
        get_param(GMB_PARAM_SPEED_CONTROL_PITCH, &value);
        *speed = value;
    }
    else {
        get_param(GMB_PARAM_SPEED_FOLLOW_PITCH, &value);
        *speed = value;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
/**
 * @brief Function used to set speed pitch axis
 * @details The speed value has been calculated depending on the application
 * @note 
 */
T_DjiReturnCode GremsyGMB_SetPitchSmooth(const uint8_t smooth)
{
    /*!< Check gimbal mode first for specifying the speed mode that mode */
    set_param(GMB_PARAM_SMOOTH_CONTROL_PITCH, (int16_t)smooth*3);
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

}

/**
 * @brief Function used to get smooth pitch axis
 * @details The speed value has been calculated depending on the application
 * @note 
 */
T_DjiReturnCode GremsyGMB_GetPitchsmooth(uint8_t *smooth)
{
    int16_t value = 0;
    
    /*!< Check gimbal mode first for specifying the speed mode that mode */
    if(GremsyGMB->systemState.gimbalMode == DJI_GIMBAL_MODE_FREE) {
        get_param(GMB_PARAM_SMOOTH_CONTROL_PITCH, &value);
        *smooth = value;
    }
    else {
        get_param(GMB_PARAM_SMOOTH_FOLLOW_PITCH, &value);
        *smooth = value;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
  * @} // PITCH_CONFIG_FUNCTIONS
  */
/*!<====================== ROLL_CONFIG_FUNCTIONS =========================== */

/** @addtogroup 
  * @{ ROLL_CONFIG_FUNCTIONS
  */
/**
 * @brief Function used to set stiffness
 * @details If the gimbal has oscillations that cannot be corrected by adjusting stiffness settings
 * @note 
 */
T_DjiReturnCode GremsyGMB_SetRollStiffness(const uint8_t stiffness)
{
    /* Set param for pitch axis axis */
    set_param(GMB_PARAM_STIFFNESS_ROLL, (int16_t)stiffness);
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Function used to get stiffness
 * @details If the gimbal has oscillations that cannot be corrected by adjusting stiffness settings
 * @note 
 */
T_DjiReturnCode GremsyGMB_GetRollStiffness(uint8_t *stiffness) 
{
    int16_t value;
    /* Set param for pitch axis axis */
    get_param(GMB_PARAM_STIFFNESS_ROLL, &value);
    *stiffness        =  (uint8_t)value;
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Function used to set Hold Strength motor
 * @details Constant power level provided to the corresponding axis and should be aajust manual.
 * @note For heavy cameras, it's suggested to increase hold strength for each axis around 10%
 * than the default value (40).
 */
T_DjiReturnCode GremsyGMB_SetRollHoldStrength(const uint8_t holdStrength) 
{
    /* Set param for pitch axis axis */
    set_param(GMB_PARAM_HOLDSTRENGTH_ROLL, (int16_t)holdStrength);
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Function used to get Hold Strength motor
 * @details Constant power level provided to the corresponding axis and should be aajust manual.
 * @note For heavy cameras, it's suggested to increase hold strength for each axis around 10%
 * than the default value (40).
 */
T_DjiReturnCode GremsyGMB_GetRollHoldStrength(uint8_t *holdStrength) 
{
    int16_t value;
    /* Get parameter setting for pitch axis*/
    get_param(GMB_PARAM_HOLDSTRENGTH_ROLL, &value);
    *holdStrength     =  (uint8_t)value;
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
  * @} // ROLL_CONFIG_FUNCTIONS
  */

/*!<====================== YAW_CONFIG_FUNCTIONS ============================= */

/** @addtogroup YAW_CONFIG_FUNCTIONS
  * @{
  */
/**
 * @brief Function used to set stiffness
 * @details If the gimbal has oscillations that cannot be corrected by adjusting stiffness settings
 * @note 
 */
T_DjiReturnCode GremsyGMB_SetYawStiffness(const uint8_t stiffness)
{
    /* Set param for pitch axis axis */
    set_param(GMB_PARAM_STIFFNESS_YAW, (int16_t)stiffness);
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Function used to get stiffness
 * @details If the gimbal has oscillations that cannot be corrected by adjusting stiffness settings
 * @note 
 */
T_DjiReturnCode GremsyGMB_GetYawStiffness(uint8_t *stiffness) 
{
    int16_t value;
    /* Set param for pitch axis axis */
    get_param(GMB_PARAM_STIFFNESS_YAW, &value);
    *stiffness        =  (uint8_t)value;
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Function used to set Hold Strength motor
 * @details Constant power level provided to the corresponding axis and should be aajust manual.
 * @note For heavy cameras, it's suggested to increase hold strength for each axis around 10%
 * than the default value (40).
 */
T_DjiReturnCode GremsyGMB_SetYawHoldStrength(const uint8_t holdStrength)
{
    /* Set param for pitch axis axis */
    set_param(GMB_PARAM_HOLDSTRENGTH_YAW, (int16_t)holdStrength);
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Function used to get Hold Strength motor
 * @details Constant power level provided to the corresponding axis and should be aajust manual.
 * @note For heavy cameras, it's suggested to increase hold strength for each axis around 10%
 * than the default value (40).
 */
T_DjiReturnCode GremsyGMB_GetYawHoldStrength(uint8_t *holdStrength)
{
    int16_t value;
    /* Get parameter setting for pitch axis*/
    get_param(GMB_PARAM_HOLDSTRENGTH_YAW, &value);
    *holdStrength     =  (uint8_t)value;
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Function used to set speed yaw axis
 * @details The speed value has been calculated depending on the application
 * @note 
 */
T_DjiReturnCode GremsyGMB_SetYawSpeed(const uint8_t speed)
{
    /*!< Check gimbal mode first for specifying the speed mode that mode */
    
//    int16_t speedControl = speed*speedConversionFactor;
    
    set_param(GMB_PARAM_SPEED_CONTROL_YAW, (int16_t)speed);
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Function used to get speed yaw axis
 * @details The speed value has been calculated depending on the application
 * @note 
 */
T_DjiReturnCode GremsyGMB_GetYawSpeed(uint8_t *speed)
{
    int16_t value = 0;
    
    /*!< Check gimbal mode first for specifying the speed mode that mode */
    get_param(GMB_PARAM_SPEED_CONTROL_YAW, &value);
    *speed = value;
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
/**
 * @brief Function used to set speed pitch axis
 * @details The speed value has been calculated depending on the application
 * @note 
 */
T_DjiReturnCode GremsyGMB_SetYawSmooth(const uint8_t smooth)
{
    /*!< Check gimbal mode first for specifying the speed mode that mode */
    set_param(GMB_PARAM_SMOOTH_CONTROL_YAW, (int16_t)smooth*3);
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Function used to get smooth pitch axis
 * @details The speed value has been calculated depending on the application
 * @note 
 */
T_DjiReturnCode GremsyGMB_GetYawsmooth(uint8_t *smooth)
{
    int16_t value = 0;
    
    /*!< Check gimbal mode first for specifying the speed mode that mode */
    get_param(GMB_PARAM_SMOOTH_CONTROL_YAW, &value);
    *smooth = value;
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
/**
  * @} // YAW_CONFIG_FUNCTIONS
  */


#endif
/**
    @}
*/

/** @group __GSDK_GIMBAL_TASKS_
    @{
*/#ifndef __GSDK_GIMBAL_TASKS_
#define __GSDK_GIMBAL_TASKS_
/**
  * @brief GremsyGimbal_RecvTask
  * Task gimbal will interface with the gimbal by conveying gimbal data 
  * to app as well as gimbal command to gimbal devixe
  */
static void *GremsyGMB_RecvTask(void *arg)
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
        realLen = comm_receive_bufer(s_gimbalRecBuf, sizeof(s_gimbalRecBuf));

        if(realLen > 0)
        {
            // Scan the counts of bytes read actually.
            for (int i = 0; i < realLen; i++)
            {
                // Get data from buffer
                uint8_t c = s_gimbalRecBuf[i];
                
                //===================== MAVLINK Recievie Function ==============================//            
                //// Try to get a new message
                if(mavlink_parse_char(GIMBAL_MAVLINK_CHANNEL, c, &msg, &status))
                {
                    // Handle message here
                    handle_mavlink_message(&msg);
                }
            }
        }
        
        osalHandler->TaskSleepMs(1);
    }
}


#endif
/**
    @}
*/

/** @group __GSDK_GIMBAL_CONFIGURATION_
    @{
*/#ifndef __GSDK_GIMBAL_CONFIGURATION_
#define __GSDK_GIMBAL_CONFIGURATION_
/**
  * @brief gimbal_interface_init
  * Init hardware to communication with the gimbal device
  */
T_DjiReturnCode GremsyGMB_Init(void)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    // PIN_D8 PIN_D9
    /*!< Init hardware serial */
    UART_Init(GIMBAL_UART_NUM, 115200);
    
    /*!< Init mavlink channel and assign function for sending message*/
    s_gimbalHandler.chan    = GIMBAL_MAVLINK_CHANNEL;
    s_gimbalHandler.write   = gimbal_comm_send_buffer;
    
    /*!< Init mavlink function handler */
    gremsy_mavlink_init(&s_gimbalHandler);
    
    /* Gremsy system */
    GremsyGMB        = &s_GremsyGMB;
    
    /*!< Create mutex for managing the access function list when sending*/
    if(osalHandler->MutexCreate(&s_mutexSend) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("Create Mutex error\n");
    }

    /*!< Create mutex for managing the access function list when sending*/
    if(osalHandler->MutexCreate(&s_mutexHandleParam) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("Create Mutex error\n");
    }
    
    /*!< Create Gimbal process data task*/
    if(osalHandler->TaskCreate("GBM_RecvTask", GremsyGMB_RecvTask
        , GIMBAL_TASK_STACK_SIZE, NULL, &s_gimbalInterfaceThread) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("user gimbal recv task create error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
        
    USER_LOG_INFO("gremsy gimbal init successfull !!!");
        
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS; 
}


#endif
/**
    @}
*/

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

