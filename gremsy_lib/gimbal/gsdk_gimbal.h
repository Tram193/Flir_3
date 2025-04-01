/** 
  ******************************************************************************
  * @file    gsdk_gimbal.h
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

#ifndef __GREMSY_LIB_GSDK_GIMBAL_H
#define __GREMSY_LIB_GSDK_GIMBAL_H

/* Includes ------------------------------------------------------------------*/

/*!< Include gimbal module*/
#include "dji_gimbal.h"

/*!< Include mavlink library */
#include "mavlink/gsdk_mavlink.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported define ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
 * @brief
 * @details This structure type is used to
 *
 */
typedef struct  _T_GMBSettingFilter
{
    uint8_t gyro_filter;
    uint8_t output_filter;
    uint8_t gain;
    
} T_GMBSettingFilter;
/**
 * @brief _gimbal_motor_control_t
 * stifness: Stiffness setting has a significant impact on the performance of the Gimbal. 
 *      This setting adjusts the degrees to which the gimbal tries to correct 
 *      for unwanted camera movement and hold the camera stable. 
 *      The higher you can run the setting without vibration or oscillation, the better.
 * Holdstrength: Power level required for the corresponding axis. 
 *      This option is only recommended for advanced users. Set 40 as defaults
 */
typedef struct _axis_setting_t
{
    uint8_t stiffness;
    uint8_t holdstrength;
    
    int8_t  dir;
    uint8_t speed_control;
    uint8_t smooth_control;

    uint8_t speed_follow;
    uint8_t smooth_follow;
    uint8_t window_follow;
} axis_setting_t;

/** @defgroup All things are relative to gimbal
 * @{
 */

/**
 * @brief System information
 * @details This structure type is used to
 *
 * 
 */
typedef struct _system_info_t
{
    uint8_t x;  /* Major Version*/
    uint8_t y;  /* Minor Version*/
    uint8_t z;  /* Sub-Minor Version*/
    uint16_t type;  /*gimbal id*/
    uint16_t sysid; /* gimbal sysid */
    uint16_t compid;  /*gimbal compid*/
    const char* model; /* ID: T3V3, S1V3, PixyU, PixyF*/
} system_info_t;

/**
 * @brief Gremsy system struct contain all information about the gimbal of gremsy
 * @details This structure type is used as global gremsy information
 */

typedef struct _gremsy_system_t
{
    system_info_t       info;                   /*!< System information*/

    uint8_t             mappingAngle;           /*!< Specifies the mapping value in degree list
                                                    "DISABLE", "DOWN_30", "DOWN_45", "DOWN_60", "DOWN_90"*/
    T_GMBSettingFilter    gimbalSetting;  
    
    axis_setting_t      pitchSetting;
    axis_setting_t      rollSetting;
    axis_setting_t      yawSetting;

    /*!<  Skyport V2 psdk_gimbal*/
    T_DjiGimbalSystemState             systemState;        /*!< Used to report system state of gimbal*/
    
    int32_t currentYaw;                 /*!< Specifies int32 value of yaw for attitude */
    T_DjiGimbalAttitudeInformation     attitudeInformation; /*!< Used to report attitude information of gimbal*/
    
    T_DjiGimbalCalibrationState        calibrationState; /*!< Used to report calibration state of gimbal*/
    
    T_DjiAttitude3d                    rotationSpeed;    /*!< unit: 0.1 degree/s, ground coordination 
                                                            @Note: Lock mode speed set to zero*/
    /*!< Limit angle of gimbal
     * To prevent the gimbal from being damaged or interfered with the drone's flight mission
     * due to structure interference druing working, you must set the mechanical and software limit.
    */
    T_DjiAttitude3d                    jointAngleLimitMin;             /*!< Unit 0.1 degree*/
    T_DjiAttitude3d                    jointAngleLimitMax;             /*!< Unit 0.1 degree*/
    T_DjiAttitude3d                    eulerAngleLimitMin;             /*!< Unit 0.1 degree*/
    T_DjiAttitude3d                    eulerAngleLimitMax;             /*!< Unit 0.1 degree*/
    int32_t                             pitchEulerAngleExtensionMin;    /*!< Unit 0.1 degree*/
    int32_t                             pitchEulerAngleExtensionMax;    /*!< Unit 0.1 degree*/
    T_DjiAttitude3d                    speedLimit;                     /*!< Unit 0.1 degree/s*/
    
    /*!< Value will be applied for the gimbal control. It depends on the gimbal mode and axis mode*/
    T_DjiAttitude3d                    targetAttitude;                 /*!< Unit: 0.1 degree, ground coordinate*/
    
    bool isRunning;
    
} T_GremsyGMB;

/* Exported Global variables------------------------------------------------- */

/* Variable of gremsy system*/
extern volatile T_GremsyGMB *GremsyGMB;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  Gimbal mode.
 * @param: type see control_gimbal_motor_t
 * @ret: None
 */
T_DjiReturnCode GremsyGMB_SetMappingAngle(int8_t pitchDownAngle);
T_DjiReturnCode GremsyGMB_GetMappingAngle(int8_t *pitchDownAngle);
T_DjiReturnCode GremsyGMB_SetSpeed(int16_t pitchSpeed, int16_t rollSpeed, int16_t yawSpeed);
T_DjiReturnCode GremsyGMB_SetAngle(int16_t pitch, int16_t roll, int16_t yaw);
T_DjiReturnCode GremsyGMB_SetRotation(int16_t pitch, int16_t roll, int16_t yaw, E_DjiGimbalRotationMode rotationMode);
/*!<====================== BASIC_FUNCTIONS =================================== */

/** @addtogroup 
  * @{
  */

T_DjiReturnCode GremsyGMB_SetReboot(void);
bool GremsyGMB_IsPresent(void);
bool GremsyGMB_IsAlgined(void);
                         
/**
 * @brief Function process param
 * @details This function will be calked in the loop for check the parameters
 * @note 
 */

T_DjiReturnCode GimbalGremsy_ParameterProcess(void);
/**
  * @} // 
  */

/*!<====================== AXIS_SETTING_FUNCTIONS =========================== */

/** @addtogroup 
  * @{
  */
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
                                    const axis_setting_t *yaw);
/**
  * @} // 
  */

/*!< */
/* Private defines -----------------------------------------------------------*/
/*!<====================== FILTER_FUNCTIONS ================================= */

/** @addtogroup 
  * @{ FILTER_FUNCTIONS
  */
void GremsyGMB_SetGyroFilter(const uint8_t gyroFilter);
void GremsyGMB_GetGyroFilter(uint8_t *gyroFilter);
void GremsyGMB_SetOutputFilter(const uint8_t outputFilter);
void GremsyGMB_GetOutputFilter(uint8_t *outputFilter);

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
T_DjiReturnCode GremsyGMB_SetSpeedConversionFactor(float factor);
/**
  * @} // FILTER_FUNCTIONS
  */


/*!<====================== PITCH_CONFIG_FUNCTIONS =================================== */

/** @addtogroup PITCH_CONFIG_FUNCTIONS
  * @{
  */
/**
 * @brief Function used to set speed for the pitch axis
 * @details This structure type is used to
 * @note 
 */
T_DjiReturnCode GremsyGMB_SetPitchStiffness(const uint8_t stiffness);  
T_DjiReturnCode GremsyGMB_GetPitchStiffness(uint8_t *stiffness); 
T_DjiReturnCode GremsyGMB_SetPitchHoldStrength(const uint8_t holdStrength);
T_DjiReturnCode GremsyGMB_GetPitchHoldStrength(uint8_t *holdStrength);


T_DjiReturnCode GremsyGMB_SetPitchSpeed(const uint8_t speed);
T_DjiReturnCode GremsyGMB_SetPitchSmooth(const uint8_t smooth);
T_DjiReturnCode GremsyGMB_GetPitchSpeed(uint8_t *speed);
T_DjiReturnCode GremsyGMB_GetPitchsmooth(uint8_t *smooth);
/**
  * @} // PITCH_CONFIG_FUNCTIONS
  */

/*!<====================== ROLL_CONFIG_FUNCTIONS =========================== */

/** @addtogroup 
  * @{ ROLL_CONFIG_FUNCTIONS
  */
T_DjiReturnCode GremsyGMB_SetRollStiffness(const uint8_t stiffness);
T_DjiReturnCode GremsyGMB_GetRollStiffness(uint8_t *stiffness);
T_DjiReturnCode GremsyGMB_SetRollHoldStrength(const uint8_t holdStrength);
T_DjiReturnCode GremsyGMB_GetRollHoldStrength(uint8_t *holdStrength);
/**
  * @} // ROLL_CONFIG_FUNCTIONS
  */


/*!<====================== YAW_CONFIG_FUNCTIONS ============================= */

/** @addtogroup YAW_CONFIG_FUNCTIONS
  * @{
  */
T_DjiReturnCode GremsyGMB_SetYawStiffness(const uint8_t stiffness);
T_DjiReturnCode GremsyGMB_GetYawStiffness(uint8_t *stiffness);
T_DjiReturnCode GremsyGMB_SetYawHoldStrength(const uint8_t holdStrength);
T_DjiReturnCode GremsyGMB_GetYawHoldStrength(uint8_t *holdStrength);

T_DjiReturnCode GremsyGMB_SetYawSpeed(const uint8_t speed);
T_DjiReturnCode GremsyGMB_SetYawSmooth(const uint8_t smooth);
T_DjiReturnCode GremsyGMB_GetYawSpeed(uint8_t *speed);
T_DjiReturnCode GremsyGMB_GetYawsmooth(uint8_t *smooth);

/**
  * @} // YAW_CONFIG_FUNCTIONS
  */

/** @addtogroup SEND_GIMBAL_FUNCTION
  * @{
  */
/**
 * @brief This function will be used for correct the drift
 * @details Send attitude of the drone periodically 50Hz
 * @note 
 */
T_DjiReturnCode GremsyGMB_SendAttitude(const T_DjiAttitude3d *aircraftAttitude, uint32_t millisecond);
/**
  * @} SEND_GIMBAL_FUNCTION
  */
                         
                         
/** @addtogroup SET_GIMBAL_MODE
  * @{
  */
/**
 * @brief Set Free Mode
 * @details Pitch, roll, Yaw are controllable, meaning the gimbal can move independently of the aircraft's yaw.
 * In this mode, even if the product yaw changes, the camera will continue poiting in the same world direction.
 * @note 
 */
T_DjiReturnCode GremsyGMB_SetFreeMode(void);

/**
 * @brief Set Yaw Follow Mode
 * @details Pitch and roll are controllable. Yaw will follow the products heading.
 * @note 
 */
T_DjiReturnCode GremsyGMB_SetYawFollowMode(void);
  
/**
 * @brief This function supports for setting gimbal mode down
 * @details Function will reset yaw axis to the home position and set pitch axis to 90 degrees
 * @note This function allow control pitch and yaw with 2 dial button
 */
T_DjiReturnCode GremsyGMB_SetResetMode(uint8_t reset_mode);

/**
  * @} // SET_GIMBAL_MODE
  */
                         
                         
                         /** @addtogroup SET_GIMBAL_MOTOR
  * @{
  */
/**
  * @brief GremsyGMB_SetGimbalOff
  * Task gimbal will interface with the gimbal by conveying gimbal data 
  * to app as well as gimbal command to gimbal devixe
  */

T_DjiReturnCode GremsyGMB_SetGimbalOff(void);

/**
  * @brief GremsyGMB_SetGimbalOn
  * Task gimbal will interface with the gimbal by conveying gimbal data 
  * to app as well as gimbal command to gimbal devixe
  */

T_DjiReturnCode GremsyGMB_SetGimbalOn(void);

T_DjiReturnCode GremsyGMB_IsRunning(uint8_t *isRunning);
/**
  * @} // SET_GIMBAL_MOTOR
  */
  
/** @brief  ham gui heartbeat den gimbal
    @return T_DjiReturnCode
*/
T_DjiReturnCode GimbalGremsy_sendHeartbeattoGimbal(void);

#if defined(QE_CAM)

/** @brief      This function use check gimbal is present
    @param[in]  none
    @return     bool
*/
bool GremsyGMB_getConnection(void);

/** @brief      This function use get gremsy gimbal type
    @param[in]  none
    @return     uint8_t
*/
T_DjiReturnCode GremsyGMB_getInfo(system_info_t *info);

/** @brief      This function use get number of param red from gimbal
    @param[in]  none
    @return     uint8_t
*/
uint8_t GremsyGMB_getNumberOfParam(void);

/** @brief      This function use get gimbal param list ready
    @param[in]  none
    @return     T_DjiReturnCode
*/
T_DjiReturnCode GremsyGMB_getParamListReady(void);

/** @brief      This funtion use get gimbal param list
    @param[in]  param[] : array param list of gimbal
    @return     T_DjiReturnCode
*/
T_DjiReturnCode GremsyGMB_getParamListValue(float *param);

/** @brief      This funtion use get gimbal param list index
    @param[in]  param[] : array param list of gimbal
    @return     T_DjiReturnCode
*/
T_DjiReturnCode GremsyGMB_getParamListIndex(uint8_t *index);

/** @brief      This function use get gimbal system status
    @param[in]  *sys_status : pointer to mavlink msg system status of gimbal
    @return     T_DjiReturnCode
*/
T_DjiReturnCode GremsyGMB_getSystemStatus(mavlink_sys_status_t *sys_status);

/** @brief      This function use get gimbal mount status
    @param[in]  *mount_status : pointer to mavlink msg mount status of gimbal
    @return     T_DjiReturnCode
*/
T_DjiReturnCode GremsyGMB_getMountStatus(mavlink_mount_status_t *mount_status);

/** @brief      This function use get gimbal mount orienstation
    @param[in]  *mount_orienstation : pointer to mavlink msg mount orienstation of gimbal
    @return     T_DjiReturnCode
*/
T_DjiReturnCode GremsyGMB_getMountOrienstation(mavlink_mount_orientation_t *mount_orienstation);

/** @brief      This function use get gimbal attitude status
    @param[in]  *attitude_status : pointer to mavlink msg attitude status of gimbal
    @return     T_DjiReturnCode
*/
T_DjiReturnCode GremsyGMB_getAttitudeStatus(mavlink_gimbal_device_attitude_status_t *attitude_status);

/** @brief      This function use get gimbal device infomation
    @param[in]  *device_info : pointer to mavlink msg gimbal device infomation
    @return     T_DjiReturnCode
*/
T_DjiReturnCode GremsyGMB_getDeviceInfomation(mavlink_gimbal_device_information_t *device_info);

/** @brief      This function use get gimbal raw imu
    @param[in]  *device_info : pointer to mavlink msg raw imu
    @return     T_DjiReturnCode
*/
T_DjiReturnCode GremsyGMB_getRawImu(mavlink_raw_imu_t *raw_imu);

/** @brief      This function use get gimbal param value
    @param[in]  *param_value : pointer to mavlink msg param value structure
    @return     T_DjiReturnCode
*/
T_DjiReturnCode GremsyGMB_getParamValue(mavlink_param_value_t *param_value);

/** @brief      This functions use send mavlink msg command long to gimbal
    @param[in]  command : command param in msg command long
    @param[in]  param[7] : param in msg command long about 7 param
    @return     T_DjiReturnCode
*/
T_DjiReturnCode GremsyGMB_sendCommandLong(uint16_t command, const float param[7]);

/** @brief      This function use send msg mavlink param request read to gimbal
    @param[in]  *param : pointer to mavlink msg param request read structure
    @return     T_DjiReturnCode
*/
T_DjiReturnCode GremsyGMB_sendMsgParamRequestRead(mavlink_param_request_read_t *param);

/** @brief      This function use get flag recieve msg with msg id
    @param[in]  msgId : mavlink msg Id
    @return     bool
*/
bool GremsyGMB_getFlagIsRecieveMsg(uint16_t msgId);

/** @brief      This function use get command ack from gimbal
    @param[in]  *ack : pointer to msg command ack structure
    @return     T_DjiReturnCode
*/
T_DjiReturnCode GremsyGMB_getCommandAck(mavlink_command_ack_t *ack);

/** @brief      This function use setting param to gimbal
    @param[in]  *param_set : param set from camera to gimbal
    @return     T_DjiReturnCode
*/
T_DjiReturnCode GremsyGMB_sendMsgParamSet(mavlink_param_set_t *param_set);

#endif

/**
 * @brief GremsyGimbal_Init
 * @details This function will initialise the gimbal and mavlink protocol
 * @note TODO - Saperate the hardware (Serial init to higher layer)
 */
T_DjiReturnCode GremsyGMB_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __GREMSY_LIB_GSDK_GIMBAL_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

