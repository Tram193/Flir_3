/**
 ********************************************************************
 * @file    test_widget.c
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "test_widget.h"
#include <dji_widget.h>
#include <dji_logger.h>
#include "../utils/util_misc.h"
#include <dji_platform.h>
#include <stdio.h>
#include "dji_sdk_config.h"
#include "file_binary_array_list_en.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

/*!< Include camera control */
#include "camera/gsdk_wws_camera.h"
#include "camera_emu/test_payload_cam_emu_base.h"
/*!< Include gremsy Gimbal*/
#include "gimbal/gsdk_gimbal.h"
/*!< Include Som*/
#include "som/som_communication.h"
/*!< Include Led indicator */
#include "led.h"

/* Private constants ---------------------------------------------------------*/
#define WIDGET_DIR_PATH_LEN_MAX         (256)
#define WIDGET_TASK_STACK_SIZE          (2048)

/* Private types -------------------------------------------------------------*/
/// define layout for camera
enum 
{
    TYPE_LIST_MAIN_LAYOUT_INDEX             = 0,
    TYPE_SWITCH_MAIN_CAMERA_INDEX           ,
    TYPE_SCALE_THERMAL_IMAGE_OPACITY        ,
    TYPE_SWITCH_GIMBAL_MOTOR_CONTROL        ,
//    TYPE_INT_PITCH_DOWN_INDEX               ,

    TYPE_LIST_PALETTE_INDEX                 ,
    
    /*!< RANGE SETTING */
    TYPE_INPUT_TIME_STABILIZATION           ,
    TYPE_SCALE_HOT_REJECTION,
    TYPE_SCALE_COOL_REJECTION,
    
    /*!< Alarm */
    TYPE_LIST_ALARM_MODE                    ,
    TYPE_SCALE_ALARM_ABOVE,
    TYPE_SCALE_ALARM_BELOW,
    TYPE_LIST_ALARM_COLOR,
    
    /*!< IMAGE & VIDEO SETTING*/
    
    
    /*!< GIMBAL SETTING: Temporary */
    TYPE_LIST_AXIS_SETTING_INDEX            ,
    TYPE_INPUT_BOX_STIFFNESS_INDEX,
    TYPE_INPUT_BOX_HOLDSTRENGTH_INDEX,
    TYPE_INPUT_GYROFILTER_INDEX,
    TYPE_INPUT_OUTPUTFILTER_INDEX,
    TYPE_BUTTON_GMB_REBOOT_INDEX,
} E_gremsyWidgetIndex;
/* Private functions declaration ---------------------------------------------*/
static void *DjiTest_WidgetTask(void *arg);
static T_DjiReturnCode DjiTestWidget_SetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value,
                                                    void *userData);
static T_DjiReturnCode DjiTestWidget_GetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value,
                                                    void *userData);

/* Private values ------------------------------------------------------------*/
static T_DjiTaskHandle s_widgetTestThread;
static bool s_isWidgetFileDirPathConfigured = false;
static char s_widgetFileDirPath[DJI_FILE_PATH_SIZE_MAX] = {0};

static const T_DjiWidgetHandlerListItem s_widgetHandlerList[] = {
    
    {TYPE_LIST_MAIN_LAYOUT_INDEX,       DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue,      DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_SWITCH_MAIN_CAMERA_INDEX,     DJI_WIDGET_TYPE_SWITCH,        DjiTestWidget_SetWidgetValue,      DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_SCALE_THERMAL_IMAGE_OPACITY,  DJI_WIDGET_TYPE_SCALE,         DjiTestWidget_SetWidgetValue,     DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_SWITCH_GIMBAL_MOTOR_CONTROL,  DJI_WIDGET_TYPE_SWITCH,         DjiTestWidget_SetWidgetValue,     DjiTestWidget_GetWidgetValue, NULL},
//    {TYPE_INT_PITCH_DOWN_INDEX,         DJI_WIDGET_TYPE_INT_INPUT_BOX, DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    
    {TYPE_LIST_PALETTE_INDEX,           DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    
    {TYPE_INPUT_TIME_STABILIZATION,     DJI_WIDGET_TYPE_INT_INPUT_BOX, DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_SCALE_HOT_REJECTION,          DJI_WIDGET_TYPE_SCALE,         DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_SCALE_COOL_REJECTION,         DJI_WIDGET_TYPE_SCALE,         DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    
    
    {TYPE_LIST_ALARM_MODE,              DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_SCALE_ALARM_ABOVE,            DJI_WIDGET_TYPE_SCALE,         DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_SCALE_ALARM_BELOW,            DJI_WIDGET_TYPE_SCALE,         DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_LIST_ALARM_COLOR,             DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    
    /*!< Gremsy define in configure interface*/
    {TYPE_LIST_AXIS_SETTING_INDEX,      DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_INPUT_BOX_STIFFNESS_INDEX,    DJI_WIDGET_TYPE_INT_INPUT_BOX, DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_INPUT_BOX_HOLDSTRENGTH_INDEX, DJI_WIDGET_TYPE_INT_INPUT_BOX, DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_INPUT_GYROFILTER_INDEX,       DJI_WIDGET_TYPE_INT_INPUT_BOX, DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_INPUT_OUTPUTFILTER_INDEX,     DJI_WIDGET_TYPE_INT_INPUT_BOX, DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_BUTTON_GMB_REBOOT_INDEX,      DJI_WIDGET_TYPE_BUTTON,        DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL}
};

static const char *s_widgetTypeNameArray[] = {
    "Unknown",
    "Button",
    "Switch",
    "Scale",
    "List",
    "Int input box"
};

static const uint32_t s_widgetHandlerListCount = sizeof(s_widgetHandlerList) / sizeof(T_DjiWidgetHandlerListItem);
static int32_t s_widgetValueList[sizeof(s_widgetHandlerList) / sizeof(T_DjiWidgetHandlerListItem)] = {0};

static T_LedCommonHandler s_ledHandler;

/* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode DjiTest_WidgetStartService(void)
{
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    //Step 1 : Init DJI Widget
    djiStat = DjiWidget_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Dji test widget init error, stat = 0x%08llX", djiStat);
        return djiStat;
    }

#ifdef SYSTEM_ARCH_LINUX
    //Step 2 : Set UI Config (Linux environment)
    char curFileDirPath[WIDGET_DIR_PATH_LEN_MAX];
    char tempPath[WIDGET_DIR_PATH_LEN_MAX];
    djiStat = DjiUserUtil_GetCurrentFileDirPath(__FILE__, WIDGET_DIR_PATH_LEN_MAX, curFileDirPath);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get file current path error, stat = 0x%08llX", djiStat);
        return djiStat;
    }

    if (s_isWidgetFileDirPathConfigured == true) {
        snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/en_big_screen", s_widgetFileDirPath);
    } else {
        snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/en_big_screen", curFileDirPath);
    }

    //set default ui config path
    djiStat = DjiWidget_RegDefaultUiConfigByDirPath(tempPath);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Add default widget ui config error, stat = 0x%08llX", djiStat);
        return djiStat;
    }

    //set ui config for English language
    djiStat = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_ENGLISH,
                                             DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                             tempPath);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Add widget ui config error, stat = 0x%08llX", djiStat);
        return djiStat;
    }

    //set ui config for Chinese language
    if (s_isWidgetFileDirPathConfigured == true) {
        snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/cn_big_screen", s_widgetFileDirPath);
    } else {
        snprintf(tempPath, WIDGET_DIR_PATH_LEN_MAX, "%swidget_file/cn_big_screen", curFileDirPath);
    }

    djiStat = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_CHINESE,
                                             DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                             tempPath);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Add widget ui config error, stat = 0x%08llX", djiStat);
        return djiStat;
    }
#else
    //Step 2 : Set UI Config (RTOS environment)
    T_DjiWidgetBinaryArrayConfig enWidgetBinaryArrayConfig = {
        .binaryArrayCount = g_EnBinaryArrayCount,
        .fileBinaryArrayList = g_EnFileBinaryArrayList
    };

    //set default ui config
    djiStat = DjiWidget_RegDefaultUiConfigByBinaryArray(&enWidgetBinaryArrayConfig);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Add default widget ui config error, stat = 0x%08llX", djiStat);
        return djiStat;
    }
#endif
    //Step 3 : Set widget handler list
    djiStat = DjiWidget_RegHandlerList(s_widgetHandlerList, s_widgetHandlerListCount);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set widget handler list error, stat = 0x%08llX", djiStat);
        return djiStat;
    }

    //Step 4 : Run widget api sample task
    if (osalHandler->TaskCreate("user_widget_task", DjiTest_WidgetTask, WIDGET_TASK_STACK_SIZE, NULL,
                                &s_widgetTestThread) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Dji widget test task create error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_WidgetSetConfigFilePath(const char *path)
{
    memset(s_widgetFileDirPath, 0, sizeof(s_widgetFileDirPath));
    memcpy(s_widgetFileDirPath, path, USER_UTIL_MIN(strlen(path), sizeof(s_widgetFileDirPath) - 1));
    s_isWidgetFileDirPathConfigured = true;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

__attribute__((weak)) void DjiTest_WidgetLogAppend(const char *fmt, ...)
{

}

#ifndef __CC_ARM
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-noreturn"
#pragma GCC diagnostic ignored "-Wreturn-type"
#pragma GCC diagnostic ignored "-Wformat"
#endif

/* Private functions definition-----------------------------------------------*/
static void *DjiTest_WidgetTask(void *arg)
{
    char message[DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN];
    uint32_t sysTimeMs = 0;
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    mavlink_onboard_computer_status_t onboardStatus = {0};

    USER_UTIL_UNUSED(arg);
    
    /*!< Init LED indicator */
    GsdkLed_Init(&s_ledHandler);
    
    static uint32_t step = 0;
    char systemStatus[20];

    while (1) {
        djiStat = osalHandler->GetTimeMs(&sysTimeMs);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Get system time ms error, stat = 0x%08llX", djiStat);
        }

        djiStat = SomComm_GetOnboardStatus(&onboardStatus);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//            USER_LOG_ERROR("Get onboard status error, stat = 0x%08llX", djiStat);
        }
        
        if(DjiTest_CameraIsInited() && GremsyGMB_IsPresent() && djiStat == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            if(SomComm_GetSomState() == SOM_INIT) {
                const char *State = "SOM INT";
                strncpy(systemStatus, State, 20);
                
                s_ledHandler.ToggleColor(RED, 3);
            }
            else if(SomComm_GetSomState() == SOM_WAIT_HDMI) {
                const char *State = "WAIT HDMI";
                strncpy(systemStatus, State, 20);
                
                s_ledHandler.ToggleColor(RED, 3);
            }
            else if(SomComm_GetSomState() == SOM_WAIT_MAV) {
                const char *State = "WAIT MAV";
                strncpy(systemStatus, State, 20);
                
                 s_ledHandler.ToggleColor(RED, 3);
            }
            else if(SomComm_GetSomState() == SOM_ERROR_HDMI) {
                const char *State = "CHECK RES";
                strncpy(systemStatus, State, 20);
                
                s_ledHandler.ToggleColor(CYAN, 3);
            }
            else if(SomComm_GetSomState() == SOM_STREAMING) {
                const char *State = "READY";
                strncpy(systemStatus, State, 20);
            
                s_ledHandler.ToggleColor(GREEN, 5);
            }
                
        }
        else
        {
            if(!DjiTest_CameraIsInited()) {
                const char *State = "CAM ERROR";
                strncpy(systemStatus, State, 20);
                
                s_ledHandler.ToggleColor(WHITE, 3);
            }
            else if(!GremsyGMB_IsPresent()) {
                const char *State = "GMB ERROR";
                strncpy(systemStatus, State, 20);
                s_ledHandler.ToggleColor(PINK, 3);
            }
            else if(djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                const char *State = "SOM IDLE";
                strncpy(systemStatus, State, 20);
                
                s_ledHandler.ToggleColor(RED, 3);
            }
            else {
                const char *State = "ERROR";
                strncpy(systemStatus, State, 20);
 
                s_ledHandler.SetColor(RED);
            }
        }
        
#ifndef USER_FIRMWARE_MAJOR_VERSION
//        uint32_t debug_heap_use = configTOTAL_HEAP_SIZE - xPortGetFreeHeapSize();
//        snprintf(message, DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN, "System Status: %s\r\n%d\r\n"
//                                                                , systemStatus
//                                                                , debug_heap_use);
        
        snprintf(message, DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN, "System Status: %s\r\n", systemStatus);
#else
        snprintf(message, DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN,
                 "System time : %u ms\r\nVersion: v%02d.%02d.%02d.%02d\r\nBuild time: %s %s", sysTimeMs,
                 USER_FIRMWARE_MAJOR_VERSION, USER_FIRMWARE_MINOR_VERSION,
                 USER_FIRMWARE_MODIFY_VERSION, USER_FIRMWARE_DEBUG_VERSION,
                 __DATE__, __TIME__);
#endif

        djiStat = DjiWidgetFloatingWindow_ShowMessage(message);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Floating window show message error, stat = 0x%08llX", djiStat);
        }

        osalHandler->TaskSleepMs(100);
        
        s_ledHandler.LedProcess(step++);
    }
}

#ifndef __CC_ARM
#pragma GCC diagnostic pop
#endif

static T_DjiReturnCode DjiTestWidget_SetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value,
                                                    void *userData)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    USER_UTIL_UNUSED(userData);

    USER_LOG_INFO("Set widget value, widgetType = %s, widgetIndex = %d ,widgetValue = %d",
                  s_widgetTypeNameArray[widgetType], index, value);
    s_widgetValueList[index] = value;
    
    switch(index)
    {
        /*!< Main Interface */
//        case TYPE_INT_PITCH_DOWN_INDEX:
//        {
////            GremsyGMB_SetMappingAngle(value);
//            break;
//        }
        
        case TYPE_LIST_MAIN_LAYOUT_INDEX:
        {
            /*!< Update for wiris information */
            WIRISCamera->layout = value;
            
            /*!< Set event camera layout */
            Dji_CameraSetEvent(CAM_EVENT_SET_LAYOUT);
            break;
        }
        case TYPE_SWITCH_MAIN_CAMERA_INDEX:
        {
            /*!< Update for wiris information */
            WIRISCamera->mainCam = value;
            
            /*!< Set event camera layout */
            Dji_CameraSetEvent(CAM_EVENT_SET_MAINCAM);

            break;
        }
        case TYPE_LIST_PALETTE_INDEX:
        {
            if(value > WIRIS_PALETTE_COUNT_MAX ) {
                WIRISCamera->paletteIdx = WIRIS_PALETTE_01_GRAY;
            } else if(value < WIRIS_PALETTE_01_GRAY) {
                WIRISCamera->paletteIdx = WIRIS_PALETTE_01_GRAY;
            }
            else {
                WIRISCamera->paletteIdx = value;
            }
            
           /*!< Set event camera  */
            Dji_CameraSetEvent(CAM_EVENT_SET_PALETTE);
            
            break;
        }
        case TYPE_INPUT_TIME_STABILIZATION:
        {
            WIRISCamera->timeStabilization = (float)(value / 10.0f);
            
            if(WIRISCamera->timeStabilization > TIME_STATBILIZATION_MAX) {
                WIRISCamera->timeStabilization = TIME_STATBILIZATION_MAX;
            }  
  
            /*!< Set event camera  */
            Dji_CameraSetEvent(CAM_EVENT_SET_TIME_STABI);
            
            break;
        }
        case TYPE_SCALE_HOT_REJECTION:
        {
            WIRISCamera->hotRejection = (float)(value / 10.0f);
            
            /*!< Check limit max for hot rejection */
            if(WIRISCamera->hotRejection > HOT_COOL_REJECTION_PERCENT_MAX) {
                WIRISCamera->hotRejection = HOT_COOL_REJECTION_PERCENT_MAX;
            }
            
            /*!< Set event camera  */
            Dji_CameraSetEvent(CAM_EVENT_SET_HOT_REJECT);
            
            /*!< Set event to get data */
            break;
        }
        case TYPE_SCALE_COOL_REJECTION:
        {
            WIRISCamera->coolRejection = (float)(value / 10.0f);
            
             /*!< Check limit max for hot rejection */
            if(WIRISCamera->coolRejection > HOT_COOL_REJECTION_PERCENT_MAX) {
                WIRISCamera->coolRejection = HOT_COOL_REJECTION_PERCENT_MAX;
            }
            
            /*!< Set event camera  */
            Dji_CameraSetEvent(CAM_EVENT_SET_COOL_REJECT);
            break;
        }
        case TYPE_SCALE_THERMAL_IMAGE_OPACITY:
        {
            /*!<  Sets thermal camera transprancy in PIP Fusion layout from 10 to 100 in percent*/
            if(value < THERMAL_CAMERA_TRANSPARENCY_MIN) {
                value = THERMAL_CAMERA_TRANSPARENCY_MIN;
            } else if (value > THERMAL_CAMERA_TRANSPARENCY_MAX) {
                value = THERMAL_CAMERA_TRANSPARENCY_MAX;
            } else {
                WIRISCamera->transparency = value;
                
                /*!< Set event camera  */
                Dji_CameraSetEvent(CAM_EVENT_SET_OPACITY);
            }
            
            break;
        }
        case TYPE_SWITCH_GIMBAL_MOTOR_CONTROL:
        {
            if(GremsyGMB_IsPresent()){
                if(s_widgetValueList[index] == 1){
                    GremsyGMB_SetGimbalOn();
                    USER_LOG_INFO("Set Gimbal On");
                }else{
                    GremsyGMB_SetGimbalOff();
                    USER_LOG_INFO("Set Gimbal Off");
                }
            }
            
            break;
        }
        case TYPE_LIST_ALARM_MODE:
        {
            WIRISCamera->alarmMode = value;
            
            /*!< Set event camera  */
            Dji_CameraSetEvent(CAM_EVENT_SET_ALARM_MODE);

            break;
        }
        
        case TYPE_SCALE_ALARM_ABOVE: 
        {
            if(WIRISCamera->alarmMode != WIRIS_ALARM_MODE_OFF) {
                WIRISCamera->thresholdAbove = value;
                
                if(WIRISCamera->thresholdAbove > ALARM_THRESHOLD_MAX_PERCENT) {
                    WIRISCamera->thresholdAbove = ALARM_THRESHOLD_MAX_PERCENT;
                } 
                
                /*!< Set event camera  */
                Dji_CameraSetEvent(CAM_EVENT_SET_ALARM_VALUE);
            }
            break;
        }
        case TYPE_SCALE_ALARM_BELOW:
        {
            if(WIRISCamera->alarmMode != WIRIS_ALARM_MODE_OFF) {
                WIRISCamera->thresholdBelow = value;
                
                if(WIRISCamera->thresholdBelow > ALARM_THRESHOLD_MAX_PERCENT) {
                    WIRISCamera->thresholdBelow = ALARM_THRESHOLD_MAX_PERCENT;
                } 
                
                 /*!< Set event camera  */
                Dji_CameraSetEvent(CAM_EVENT_SET_ALARM_VALUE);
            }
            
            break;
        }
        case TYPE_LIST_ALARM_COLOR:
        {
            WIRISCamera->alarmColor = value;
            
            /*!< Set event camera  */
            Dji_CameraSetEvent(CAM_EVENT_SET_ALARM_COLOR);
            break;
        }
        case TYPE_LIST_AXIS_SETTING_INDEX:
        {
            break;
        }
        case TYPE_INPUT_BOX_STIFFNESS_INDEX:
        {
            /*!< Check if seting for pitch*/
            if(s_widgetValueList[TYPE_LIST_AXIS_SETTING_INDEX] == 0) {
                GremsyGMB_SetPitchStiffness(value);
            } 
            /*!< Check if setting for roll*/
            else if(s_widgetValueList[TYPE_LIST_AXIS_SETTING_INDEX] == 1) {
                GremsyGMB_SetRollStiffness(value);
            }
            /*!< Check if setting for yaw*/
            else if(s_widgetValueList[TYPE_LIST_AXIS_SETTING_INDEX] == 2) {
                GremsyGMB_SetYawStiffness(value);
            }
            else {
                USER_LOG_ERROR("Invalid Stiffness");
            }
            break;
        }
        case TYPE_INPUT_BOX_HOLDSTRENGTH_INDEX:
        {
            /*!< Check if seting for pitch*/
            if(s_widgetValueList[TYPE_LIST_AXIS_SETTING_INDEX] == 0) {
                GremsyGMB_SetPitchHoldStrength(value);
            } 
            /*!< Check if setting for roll*/
            else if(s_widgetValueList[TYPE_LIST_AXIS_SETTING_INDEX] == 1) {
                GremsyGMB_SetRollHoldStrength(value);
            }
            /*!< Check if setting for yaw*/
            else if(s_widgetValueList[TYPE_LIST_AXIS_SETTING_INDEX] == 2) {
                GremsyGMB_SetYawHoldStrength(value);
            }
            else {
                USER_LOG_ERROR("Invalid HoldStrength");
            }
            break;
        }
        case TYPE_INPUT_GYROFILTER_INDEX:
        {
            GremsyGMB_SetGyroFilter(value);
            
            break;
        }
        case TYPE_INPUT_OUTPUTFILTER_INDEX:
        {
            GremsyGMB_SetOutputFilter(value);
            break;
        }
        case TYPE_BUTTON_GMB_REBOOT_INDEX:
        {
//            GremsyGMB_SetReboot();
            
            PsdkSom_WriteHardwareResetPin(SOM_HW_RESET_MANAGEMENT_PIN_STATE_RESET);
            osalHandler->TaskSleepMs(20);
            PsdkSom_WriteHardwareResetPin(SOM_HW_RESET_MANAGEMENT_PIN_STATE_SET);
            osalHandler->TaskSleepMs(20);
            
            SomComm_SetSomState();
            USER_LOG_WARN("Reboot");
            
            break;
        }
        default:
            break;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode DjiTestWidget_GetWidgetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value,
                                                    void *userData)
{
    USER_UTIL_UNUSED(userData);
    USER_UTIL_UNUSED(widgetType);
    
    switch(index)
    {
//        /*!< Main Interface */
//        case TYPE_INT_PITCH_DOWN_INDEX:
//        {
//            GremsyGMB_GetMappingAngle((int8_t*)value);
//            break;
//        }
        case TYPE_LIST_MAIN_LAYOUT_INDEX:
        {
            /*!< Get */
            s_widgetValueList[index] = WIRISCamera->layout;
            
            break;
        }
        case TYPE_SWITCH_MAIN_CAMERA_INDEX:
        {
            s_widgetValueList[index] = WIRISCamera->mainCam;
            break;
        }
        case TYPE_LIST_PALETTE_INDEX:
        {
            s_widgetValueList[index] = WIRISCamera->paletteIdx;
            break;
        }
        case TYPE_INPUT_TIME_STABILIZATION:
        {
            /*!< Get time stabilization */
            s_widgetValueList[index] = WIRISCamera->timeStabilization*10;
            break;
        }
        case TYPE_SCALE_HOT_REJECTION:
        {
            /*!< Get hot rejection */
            s_widgetValueList[index] = WIRISCamera->hotRejection*10;
            break;
        }
        case TYPE_SCALE_COOL_REJECTION:
        {
            /*!< Get cool rejection */
            s_widgetValueList[index] = WIRISCamera->coolRejection*10;
            break;
        }
        case TYPE_SCALE_THERMAL_IMAGE_OPACITY:
        {
            /*!< Get opacity */
            s_widgetValueList[index] = WIRISCamera->transparency;
            break;
        }
        case TYPE_SWITCH_GIMBAL_MOTOR_CONTROL:
        {
            if(GremsyGMB_IsPresent()){
                GremsyGMB_IsRunning((uint8_t*)&s_widgetValueList[index]);
            }
            break;
        }
        case TYPE_LIST_ALARM_MODE:
        {
            /*!< Get alarm mode */
            s_widgetValueList[index] = WIRISCamera->alarmMode;
            
            break;
        }
        
        case TYPE_SCALE_ALARM_ABOVE:
        {
            s_widgetValueList[TYPE_SCALE_ALARM_ABOVE] = WIRISCamera->thresholdAbove;
            break;
        }
        case TYPE_SCALE_ALARM_BELOW:
        {
            s_widgetValueList[TYPE_SCALE_ALARM_BELOW] = WIRISCamera->thresholdBelow;
            
            break;
        }
        case TYPE_LIST_ALARM_COLOR:
        {
            s_widgetValueList[index] = WIRISCamera->alarmColor;
            break;
        }
        
        case TYPE_LIST_AXIS_SETTING_INDEX:
        {
            break;
        }
        case TYPE_INPUT_BOX_STIFFNESS_INDEX:
        {
            /*!< Check if seting for pitch*/
            if(s_widgetValueList[TYPE_LIST_AXIS_SETTING_INDEX] == 0) {
                GremsyGMB_GetPitchStiffness((uint8_t*)value);
            } 
            /*!< Check if setting for roll*/
            else if(s_widgetValueList[TYPE_LIST_AXIS_SETTING_INDEX] == 1) {
                GremsyGMB_GetRollStiffness((uint8_t*)value);
            }
            /*!< Check if setting for yaw*/
            else if(s_widgetValueList[TYPE_LIST_AXIS_SETTING_INDEX] == 2) {
                GremsyGMB_GetYawStiffness((uint8_t*)value);
            }
            else {
                USER_LOG_ERROR("Invalid Stiffness");
            }
            
            s_widgetValueList[index] = *value;
            break;
        }
        
        case TYPE_INPUT_BOX_HOLDSTRENGTH_INDEX:
        {
            /*!< Check if seting for pitch*/
            if(s_widgetValueList[TYPE_LIST_AXIS_SETTING_INDEX] == 0) {
                GremsyGMB_GetPitchHoldStrength((uint8_t*)value);
            } 
            /*!< Check if setting for roll*/
            else if(s_widgetValueList[TYPE_LIST_AXIS_SETTING_INDEX] == 1) {
                GremsyGMB_GetRollHoldStrength((uint8_t*)value);
            }
            /*!< Check if setting for yaw*/
            else if(s_widgetValueList[TYPE_LIST_AXIS_SETTING_INDEX] == 2) {
                GremsyGMB_GetYawHoldStrength((uint8_t*)value);
            }
            else {
                USER_LOG_ERROR("Invalid HoldStrength");
            }
            
            s_widgetValueList[index] = *value;
            
            break;
        }
        case TYPE_INPUT_GYROFILTER_INDEX:
        {
            /*!< Get gimbal Gyro Filter */
            GremsyGMB_GetGyroFilter((uint8_t*)value);
            
            s_widgetValueList[index] = *value;
            
            break;
        }
        case TYPE_INPUT_OUTPUTFILTER_INDEX:
        {
            /*!< Get gimbal Output  Filter */
            GremsyGMB_GetOutputFilter((uint8_t*)value);
            
            s_widgetValueList[index] = *value;
            break;
        }
        default:
            break;
    }
    
    /*!< Set value from the value list*/
    *value = s_widgetValueList[index];

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
