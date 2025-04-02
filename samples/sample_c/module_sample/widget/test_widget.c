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
#include "camera/gsdk_flir_camera.h"
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
    TYPE_BUTTON_FFC_INDEX                   = 1,
    TYPE_SWITCH_MSX_INDEX                   = 2,
    TYPE_MSX_LENGTH_INDEX                   = 3,

    TYPE_LIST_PALETTE_INDEX                 = 4,
    TYPE_LIST_SCENE_INDEX                   = 5,
    TYPE_LIST_FILE_FORMAT_INDEX             = 6,
    TYPE_LIST_VIDEO_TYPE_INDEX              = 7,
    
    TYPE_SWITCH_OSD_INDEX,
    TYPE_LIST_TEMP_UNIT_INDEX,
    TYPE_LIST_SKY_CONDITION_INDEX,
    TYPE_LIST_HUMIDITY_INDEX,
    TYPE_INT_AIR_TEMP_INDEX,
    TYPE_SCALE_SUB_EMISS_INDEX,
    TYPE_INPUT_SUB_RANGE_INDEX,
    
    /*!< GIMBAL SETTING */
    TYPE_LIST_AXIS_SETTING_INDEX            = 15,
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
    {TYPE_LIST_MAIN_LAYOUT_INDEX,       DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_BUTTON_FFC_INDEX,             DJI_WIDGET_TYPE_BUTTON,        DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_SWITCH_MSX_INDEX,             DJI_WIDGET_TYPE_SWITCH,        DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_MSX_LENGTH_INDEX,             DJI_WIDGET_TYPE_SCALE,         DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    
    {TYPE_LIST_PALETTE_INDEX,           DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_LIST_SCENE_INDEX,             DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_LIST_FILE_FORMAT_INDEX,       DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_LIST_VIDEO_TYPE_INDEX,        DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    
    {TYPE_SWITCH_OSD_INDEX,             DJI_WIDGET_TYPE_SWITCH,        DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_LIST_TEMP_UNIT_INDEX,         DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_LIST_SKY_CONDITION_INDEX,     DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_LIST_HUMIDITY_INDEX,          DJI_WIDGET_TYPE_LIST,          DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_INT_AIR_TEMP_INDEX,           DJI_WIDGET_TYPE_INT_INPUT_BOX, DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_SCALE_SUB_EMISS_INDEX,        DJI_WIDGET_TYPE_SCALE,         DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},
    {TYPE_INPUT_SUB_RANGE_INDEX,        DJI_WIDGET_TYPE_INT_INPUT_BOX, DjiTestWidget_SetWidgetValue,    DjiTestWidget_GetWidgetValue, NULL},

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

static uint8_t s_axisSetting = DJI_GIMBAL_AXIS_PITCH;
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
        uint32_t debug_heap_use = configTOTAL_HEAP_SIZE - xPortGetFreeHeapSize();
        static uint32_t max_heap_use=0;
        if(debug_heap_use>max_heap_use) {max_heap_use=debug_heap_use;)
        snprintf(message, DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN, "System Status: %s\r\n%d/%d\r\n"
                                                                , systemStatus
                                                                , debug_heap_use
                                                            , max_heap_use);
        
//        snprintf(message, DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN, "System Status: %s\r\n", systemStatus);
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
        case TYPE_LIST_MAIN_LAYOUT_INDEX:
        {
            /*!< Update for wiris information */
            FLIRCamera->videoTransmission = (E_FlirVideoTransmision)value;
            
            /*!< Set event camera layout */
            Dji_CameraSetEvent(CAM_EVENT_SET_LAYOUT);
            break;
        }
        case TYPE_BUTTON_FFC_INDEX: 
        {
            FLIRCamera->commandFFC = value;
            
            Dji_CameraSetEvent(CAM_EVENT_SET_DIGICAM_CONTROL);
            
            break;
        }
        case TYPE_SWITCH_MSX_INDEX:
        {
            /*!< Set MSX == true but needs to check MSX length */
            if(value == 1 && FLIRCamera->MSXLength > 0) {
                FLIRCamera->isMSXEnable = 1;
            } 
            else {
                FLIRCamera->isMSXEnable = 0;
            }
            
            USER_LOG_INFO("length: %d %d", FLIRCamera->MSXLength, FLIRCamera->isMSXEnable);

            /*!< Set event camera layout */
            Dji_CameraSetEvent(CAM_EVENT_SET_DIGICAM_CONFIG);
            break;
        }
        case TYPE_MSX_LENGTH_INDEX:
        {
            /*!< Get MSX length value */
            FLIRCamera->MSXLength = value;
            
            if(FLIRCamera->MSXLength == 0) {
                FLIRCamera->isMSXEnable = 0;
            }
            
            /*!< Set event to camera */
            Dji_CameraSetEvent(CAM_EVENT_SET_DIGICAM_CONFIG);

            break;
        }
        case TYPE_LIST_PALETTE_INDEX:
        {
            if(value > FLIR_PALETTE_MAX_COUNT ) {
                FLIRCamera->palette = FLIR_PATETTE_RAINBOW;
            } else if(value < FLIR_PATETTE_WHITEHOT) {
                FLIRCamera->palette = FLIR_PATETTE_RAINBOW;
            }
            else {
                FLIRCamera->palette = value;
            }
            
           /*!< Set event camera  */
            Dji_CameraSetEvent(CAM_EVENT_SET_DIGICAM_CONFIG);
            
            break;
        }
        case TYPE_LIST_SCENE_INDEX:
        {
            /*!< Get scene value */
            FLIRCamera->scene = value;
            
            /*!< Set event camera  */
            Dji_CameraSetEvent(CAM_EVENT_SET_DIGICAM_CONFIG);
            break;
        }
        case TYPE_LIST_FILE_FORMAT_INDEX:
        {
            if(value == 0){
                FLIRCamera->fileFormat = FLIR_FILE_FORMART_JPEG_TIFF;
            } else if(value == 1) {
                FLIRCamera->fileFormat = FLIR_FILE_FORMART_FFF;
            }
            
            USER_LOG_WARN("Set widget file format :%d", FLIRCamera->fileFormat);
            break;
        }
        case TYPE_LIST_VIDEO_TYPE_INDEX:
        {
            if(value == 0 ){
                FLIRCamera->videoFileType = FLIR_VIDEO_FILE_TYPE_H264;
            } else if(value == 1) {
                FLIRCamera->videoFileType = FLIR_VIDEO_FILE_TYPE_TLFF;
            }
            break;
        }
        case TYPE_SWITCH_OSD_INDEX:
        {
            FLIRCamera->spotMeter = value;
            
            /*!< Set event camera  */
            Dji_CameraSetEvent(CAM_EVENT_SET_PARMETERS);
            break;
        }
        case TYPE_LIST_TEMP_UNIT_INDEX:
        {
            FLIRCamera->tempUnit = value;
            
            /*!< Set event camera  */
            Dji_CameraSetEvent(CAM_EVENT_SET_PARMETERS);
            break;
        }
        
        case TYPE_LIST_SKY_CONDITION_INDEX:
        {
            if(value == 0) {
                FLIRCamera->skyCondition = FLIR_CLEAR_SKIES;
            } else if( value == 1) {
                FLIRCamera->skyCondition = FLIR_SCATTERED_SKIES;
            } else if( value == 2) {
                FLIRCamera->skyCondition = FLIR_CLOUDY_SKIES;
            }
            
            /*!< Set event camera  */
            Dji_CameraSetEvent(CAM_EVENT_SET_PARMETERS);
            
            break;
        }
        case TYPE_LIST_HUMIDITY_INDEX:
        {
            if(value == 0) {
                FLIRCamera->humidity = HUMIDITY_LOW_30;
            } else if( value == 1) {
                FLIRCamera->humidity = HUMIDITY_MEDIUM_45;
            } else if( value == 2) {
                FLIRCamera->humidity = HUMIDITY_HIGH_60;
            }
            
            /*!< Set event camera  */
            Dji_CameraSetEvent(CAM_EVENT_SET_PARMETERS);
            
            break;
        }
        case TYPE_INT_AIR_TEMP_INDEX:
        {
            if(value < -50) {
                FLIRCamera->airTemperature = -50;
            } else if(value > 40) {
                FLIRCamera->airTemperature = 40;
            } else {
                FLIRCamera->airTemperature = value;
            }
            
            /*!< Set event camera  */
            Dji_CameraSetEvent(CAM_EVENT_SET_PARMETERS);
            
            break;
        }
        case TYPE_SCALE_SUB_EMISS_INDEX:
        {
            if(value < 50) {
                value = 50;
            }
            FLIRCamera->emissivity = value;
            
            /*!< Set event camera  */
            Dji_CameraSetEvent(CAM_EVENT_SET_PARMETERS);
            break;
        }
        case TYPE_INPUT_SUB_RANGE_INDEX:
        {
            if(value > 200) {
                FLIRCamera->subjectRange = 200;
            } else if(value < 0) {
                FLIRCamera->subjectRange = 0;
            } else {
                FLIRCamera->subjectRange = value;
            }
            
            Dji_CameraSetEvent(CAM_EVENT_SET_PARMETERS);
            break;
        }
        case TYPE_LIST_AXIS_SETTING_INDEX:
        {
            /*!< Get index specifies the axis setting*/
            s_axisSetting = (E_DjiGimbalAxis)value;
            break;
        }
        case TYPE_INPUT_BOX_STIFFNESS_INDEX:
        {
            /*!< Check if seting for pitch*/
            if(s_axisSetting == DJI_GIMBAL_AXIS_PITCH) {
                GremsyGMB_SetPitchStiffness(value);
            }
            /*!< Check if setting for roll*/
            else if(s_axisSetting == DJI_GIMBAL_AXIS_ROLL) {
                GremsyGMB_SetRollStiffness(value);
            }
            /*!< Check if setting for yaw*/
            else if(s_axisSetting == DJI_GIMBAL_AXIS_YAW) {
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
            if(s_axisSetting == DJI_GIMBAL_AXIS_PITCH) {
                GremsyGMB_SetPitchHoldStrength(value);
            } 
            /*!< Check if setting for roll*/
            else if(s_axisSetting == DJI_GIMBAL_AXIS_ROLL) {
                GremsyGMB_SetRollHoldStrength(value);
            }
            /*!< Check if setting for yaw*/
            else if(s_axisSetting == DJI_GIMBAL_AXIS_YAW) {
                GremsyGMB_SetYawHoldStrength(value);
            }
            else {
                USER_LOG_ERROR("Invalid HoldStrength");
            }
            break;
        }
        case TYPE_INPUT_GYROFILTER_INDEX:
        {
            /*!< Set gyro filter */
            GremsyGMB_SetGyroFilter(value);
            break;
        }
        case TYPE_INPUT_OUTPUTFILTER_INDEX:
        {
            /*!< Set gimbal output Filter */
            GremsyGMB_SetOutputFilter(value);
            break;
        }
        case TYPE_BUTTON_GMB_REBOOT_INDEX:
        {
            /*!< Reboot gimbal */
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
        case TYPE_LIST_MAIN_LAYOUT_INDEX:
        {
            /*!< Update for wiris information */
            *value = (int32_t)FLIRCamera->videoTransmission;
            
            break;
        }
        case TYPE_BUTTON_FFC_INDEX: 
        {
            *value = (int32_t)FLIRCamera->commandFFC;
            
            break;
        }
        case TYPE_SWITCH_MSX_INDEX:
        {
            if(FLIRCamera->MSXLength == 0) {
                *value = 0;
            }
            else {
                *value = FLIRCamera->isMSXEnable;
            }

            break;
        }
        case TYPE_MSX_LENGTH_INDEX:
        {
            /*!< Get MSX length value */
            *value = FLIRCamera->MSXLength;
         
            if(FLIRCamera->MSXLength ==0 ) {
                FLIRCamera->isMSXEnable = 0;
            }
            break;
        }
        case TYPE_LIST_PALETTE_INDEX:
        {
            *value = FLIRCamera->palette;
            
            break;
        }
        case TYPE_LIST_SCENE_INDEX:
        {
            /*!< Get scene value */
            *value = FLIRCamera->scene;
            
            break;
        }
        case TYPE_LIST_FILE_FORMAT_INDEX:
        {
            static int32_t fileFormat = 0;
            
            if(FLIRCamera->fileFormat == FLIR_FILE_FORMART_JPEG_TIFF) {
                fileFormat = 0;
            } 
            else if(FLIRCamera->fileFormat == FLIR_FILE_FORMART_FFF) {
                fileFormat= 1;
            }
            
            *value = fileFormat;
            
            break;
        }
        case TYPE_LIST_VIDEO_TYPE_INDEX:
        {
            static int32_t videoFileType = 0;
            
            if(FLIRCamera->videoFileType == FLIR_VIDEO_FILE_TYPE_H264) {
                videoFileType = 0;
            } 
            else if(FLIRCamera->videoFileType == FLIR_VIDEO_FILE_TYPE_TLFF) {
                videoFileType = 1;
            }
            
            *value = videoFileType;
            break;
        }
        
        case TYPE_SWITCH_OSD_INDEX:
        {
            *value = FLIRCamera->spotMeter;
            
            break;
        }
        case TYPE_LIST_TEMP_UNIT_INDEX:
        {
            *value = FLIRCamera->tempUnit;
            
            break;
        }
        case TYPE_LIST_SKY_CONDITION_INDEX:
        {
            
            if(FLIRCamera->skyCondition == FLIR_CLEAR_SKIES) {
                *value = 0;
            } 
            else if( FLIRCamera->skyCondition == FLIR_SCATTERED_SKIES) {
                *value = 1;
            } 
            else if( FLIRCamera->skyCondition == FLIR_CLOUDY_SKIES) {
                *value = 2;
            }
            break;
        }
      
        case TYPE_LIST_HUMIDITY_INDEX:
        {
            if(FLIRCamera->humidity == HUMIDITY_LOW_30) {
                *value = 0;
            }
            else if(FLIRCamera->humidity == HUMIDITY_MEDIUM_45) {
                *value = 1;
            }
            else if(FLIRCamera->humidity == HUMIDITY_HIGH_60) {
                *value = 2;
            }

            break;
        }
        case TYPE_INT_AIR_TEMP_INDEX:
        {
            if(FLIRCamera->airTemperature < -50) {
                FLIRCamera->airTemperature = -50;
            } else if(FLIRCamera->airTemperature > 40) {
                FLIRCamera->airTemperature = 40;
            } 
            
            *value = (int32_t)FLIRCamera->airTemperature;

            break;
        }
        case TYPE_SCALE_SUB_EMISS_INDEX:
        {
            if(FLIRCamera->emissivity < 50) {
                FLIRCamera->emissivity = 50;
            }
            
            *value = (int32_t)FLIRCamera->emissivity;
            
            break;
        }
        
        case TYPE_INPUT_SUB_RANGE_INDEX:
        {
            if(FLIRCamera->subjectRange > 200) {
                FLIRCamera->subjectRange = 200;
            } 
            
            *value = (int32_t)FLIRCamera->subjectRange;

            break;
        }
        
        case TYPE_LIST_AXIS_SETTING_INDEX:
        {
            *value = s_axisSetting;
            break;
        }
        case TYPE_INPUT_BOX_STIFFNESS_INDEX:
        {
            /*!< Check if seting for pitch*/
            if(s_axisSetting == DJI_GIMBAL_AXIS_PITCH) {
                GremsyGMB_GetPitchStiffness((uint8_t*)value);
            } 
            /*!< Check if setting for roll*/
            else if(s_axisSetting == DJI_GIMBAL_AXIS_ROLL) {
                GremsyGMB_GetRollStiffness((uint8_t*)value);
            }
            /*!< Check if setting for yaw*/
            else if(s_axisSetting == DJI_GIMBAL_AXIS_YAW) {
                GremsyGMB_GetYawStiffness((uint8_t*)value);
            }
            else {
                USER_LOG_ERROR("Invalid Stiffness");
            }
            
            break;
        }
        
        case TYPE_INPUT_BOX_HOLDSTRENGTH_INDEX:
        {
            /*!< Check if seting for pitch*/
            if(s_axisSetting == DJI_GIMBAL_AXIS_PITCH) {
                GremsyGMB_GetPitchHoldStrength((uint8_t*)value);
            } 
            /*!< Check if setting for roll*/
            else if(s_axisSetting == DJI_GIMBAL_AXIS_ROLL) {
                GremsyGMB_GetRollHoldStrength((uint8_t*)value);
            }
            /*!< Check if setting for yaw*/
            else if(s_axisSetting == DJI_GIMBAL_AXIS_YAW) {
                GremsyGMB_GetYawHoldStrength((uint8_t*)value);
            }
            else {
                USER_LOG_ERROR("Invalid HoldStrength");
            }
            
            break;
        }
        case TYPE_INPUT_GYROFILTER_INDEX:
        {
            /*!< Get gimbal Gyro Filter */
            GremsyGMB_GetGyroFilter((uint8_t*)value);
            
            break;
        }
        case TYPE_INPUT_OUTPUTFILTER_INDEX:
        {
            /*!< Get gimbal Output  Filter */
            GremsyGMB_GetOutputFilter((uint8_t*)value);
            
            break;
        }
        default:
            break;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
