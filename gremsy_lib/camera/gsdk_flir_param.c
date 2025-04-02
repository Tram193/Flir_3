/**
 ******************************************************************************
  * @file    gsdk_template.c
  * @version V0.0.0
  * @date    2020/12/10
 * @brief   This file to indicate GSDK coding style and comment style.
 *
 * @copyright (c) 2017-2020 Gremsy. All rights reserved.
 *
 * All information contained herein is, and remains, the property of GREMSY.
 * The intellectual and technical concepts contained herein are proprietary
 * to GREMSY and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of GREMSY.
 *
 * If you receive this source code without GREMSY�s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify GREMSY of its removal. GREMSY reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "eeprom.h"
//#include "flir_camera_parameters.h"
#include <psdk_logger.h>
#include <psdk_gimbal.h>
#include "psdk_platform.h"
#include "gsdk_flir_camera.h"

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief Param state 
 * @details This enum value used for param state 
 * @note 
 */
typedef enum 
{
    PARAM_STATE_NOT_YET_READ = 0,   /*!< Parameter has yet to be initialized*/
    PARAM_STATE_FETCH_AGAIN = 1,    /*!< Parameter is being fetched */
    PARAM_STATE_ATTEMPTING_TO_SET = 2, /*!< Parameter is being set */
    PARAM_STATE_CONSISTENT = 3,     /*!< Parameter is consistent */
    PARAM_STATE_NONEEXISTANT = 4,   /*!< Param does not seem to exist*/
} E_ParamState;

/*!< Flashing step */
typedef enum {
    PARAM_NOT_FLASHING = 0,
    PARAM_FLASHING_WAITING_FOR_SET,
    PARAM_FLASHING_WAITING_FOR_ACK
}E_ParamFlashingStep;


/*!< Define param list for camera. Default*/
struct {
    const uint8_t       camIdx;            /*< Camera parameter index */
    const char*         camId;             /*< Camera parameter identify */
    int16_t             value;              /*< Camera parameter value */

    E_ParamState        state;              /*< State's parameters */
    uint8_t             fetchAttempts;     /*< Retry to fetch parameter*/
    bool                seen;
} s_camParams[] = {
    {PARAM_ID_VERSION_X,            "VERSION_X",            1,                              PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_VERSION_Y,            "VERSION_Y",            0,                              PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_VERSION_Z,            "VERSION_Z",            0,                              PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_CAM_ID,               "CAM_ID",               100,                            PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_VIDEO_TRANSMISSION,   "VIDEO_TRANSMISSION",   FLIR_VIDEO_TRANSMISSION_VIS,    PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_TRANSMISSION_MODE,    "TRANSMISSION_MODE",    0,                              PARAM_STATE_NONEEXISTANT, 0, false},
    {PARAM_ID_TYPE_RECORDING,       "TYPE_RECORDING",       FLIR_FILE_FORMART_FFF,          PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_SENCE_MODE,           "SENCE_MODE",           FLIR_SCENE_OUTDOOR,             PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_COLOR_PALETTE,        "COLOR_PALETTE",        FLIR_PATETTE_LAVA,              PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_APERTURE,             "APERTURE",             0,                              PARAM_STATE_NONEEXISTANT, 0, false},
    {PARAM_ID_ISO_NUM,              "ISO_NUM",              0,                              PARAM_STATE_NONEEXISTANT, 0, false},
    {PARAM_ID_ROTATION_IMAGE,       "ROTATION_IMAGE",       FLIR_ROTARY_0,                  PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_MSX_ENABLE,           "MSX_ENABLE",           0,                              PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_MSX_LENGTH,           "MSX_LENGTH",           50,                             PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_FILE_FORMAT,          "FILE_FORMAT",          FLIR_FILE_FORMART_JPEG_TIFF,    PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_VIDEO_FILE_TYPE,      "VIDEO_FILE_TYPE",      FLIR_VIDEO_FILE_TYPE_H264,      PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_TEMP_UNIT,            "TEMP_UNIT",            FLIR_TEMP_UNIT_C,               PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_TEMP_METER,           "TEMP_METER",           FLIR_SPOT_METER_OFF,            PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_SUBJECT_EMISSIVITY,   "SUBJECT_EMISSIVITY",   50,                             PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_SKY_CONDITION,        "SKY_CONDITION",        FLIR_CLEAR_SKIES,               PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_AIR_TEMPERATURE,      "AIR_TEMPERATURE",      32,                             PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_HUMIDITY,             "HUMIDITY",             HUMIDITY_MEDIUM_45,             PARAM_STATE_NOT_YET_READ, 0, false},
    {PARAM_ID_SUBJECT_RANGE,        "SUBJECT_RANGE",        200,                            PARAM_STATE_NOT_YET_READ, 0, false},
};


/*!< Param list default*/



/* Private define ------------------------------------------------------------*/
#define FLIR_PARAM_DEBUG 1

#if FLIR_PARAM_DEBUG
# define DebugMsg(fmt, args ...) do {PsdkLogger_UserLogMsg("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);
# define DebugInfo(fmt, args ...) do {PsdkLogger_UserLogInfo("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);
# define DebugWarning(fmt, args ...) do {PsdkLogger_UserLogWarn("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);
# define DebugError(fmt, args ...) do {PsdkLogger_UserLogError("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);
#else
# define DebugMsg(fmt, args ...)
# define DebugInfo(fmt, args ...)
# define DebugWarning(fmt, args ...)
# define DebugError(fmt, args ...)
#endif

#define PARAM_RETRY_PERIOD          100    /*!< Unit: ms*/
#define PARAM_MAX_FETCH_ATTEMPTS    5       /*!< Max times to fetch*/
 
#define PARAM_TASK_STACK_SIZE       (2048)
#define PARAM_PROCESS_TASK_FREQ     100
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint32_t             s_lastRequestMs = 0;
static uint32_t             s_lastSetMs = 0;
static E_ParamFlashingStep  s_flashingStep;

static T_PsdkTaskHandle     s_paramUpdateThread;
 static T_PsdkTaskHandle     s_paramSetThread;
/* Exported variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** 
  * @brief Function used to convert platform for writing function 
  * @param none
  * @param none
  * @return None
  */
T_PsdkReturnCode PsdkParam_Write(const uint16_t param, const int16_t value)
{
    /*!< Depending on mavlink or directly flash memory */
    if(EE_WriteVariable(param, value) != HAL_OK) {
        PsdkLogger_UserLogInfo("Write Failed param %d - %d", param, value);
        
        return PSDK_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/** 
  * @brief Function used to convert platform for writing function 
  * @param none
  * @param none
  * @return None
  */
T_PsdkReturnCode PsdkParam_Read(uint16_t param, int16_t *value)
{
    /*!< Depending on mavlink or directly flash memory */
    if(EE_ReadVariable(param, (uint16_t*)value) != HAL_OK) {
        
        PsdkLogger_UserLogInfo("Read Failed param %d - %d", param, value);
        
        return PSDK_ERROR_SYSTEM_MODULE_RAW_CODE_INVALID_PARAMETER;
    }
    
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


/**
 * @brief Function used for reseting param 
 * @details
 * @note 
 */
T_PsdkReturnCode PsdkParam_Reset(void)
{
    PsdkOsal_GetTimeMs(&s_lastRequestMs);
    PsdkOsal_GetTimeMs(&s_lastSetMs);

    s_flashingStep = PARAM_NOT_FLASHING;
    
    PsdkLogger_UserLogInfo("Completed");
}

/** 
  * @brief Function get param name based on the Index 
  * @param param - param index need to get the name
  * @return Name Id of that parameter
  */
const char *PsdkParam_GetParamName(const uint16_t param)
{
    switch(param) {
        case PARAM_ID_VERSION_X: return "VERSION_X";
        case PARAM_ID_VERSION_Y: return "VERSION_Y";
        case PARAM_ID_VERSION_Z: return "VERSION_Z";
        case PARAM_ID_CAM_ID: return "CAM_ID";
        case PARAM_ID_VIDEO_TRANSMISSION: return "VIDEO_TRANSMISSION";
        case PARAM_ID_TRANSMISSION_MODE: return "TRANSMISSION_MODE";
        case PARAM_ID_TYPE_RECORDING: return "TYPE_RECORDING";
        case PARAM_ID_SENCE_MODE: return "SENCE_MODE";
        case PARAM_ID_COLOR_PALETTE: return "COLOR_PALETTE";
        case PARAM_ID_APERTURE: return "APERTURE";
        case PARAM_ID_ISO_NUM: return "ISO_NUM";
        case PARAM_ID_ROTATION_IMAGE: return "ROTATION_IMAGE";
        case PARAM_ID_MSX_ENABLE: return "MSX_ENABLE";
        case PARAM_ID_MSX_LENGTH: return "MSX_LENGTH";
        case PARAM_ID_FILE_FORMAT: return "FILE_FORMAT";
        case PARAM_ID_VIDEO_FILE_TYPE: return "VIDEO_FILE_TYPE";
        case PARAM_ID_TEMP_UNIT: return "TEMP_UNIT";
        case PARAM_ID_TEMP_METER: return "TEMP_METER";
        case PARAM_ID_SUBJECT_EMISSIVITY: return "SUBJECT_EMISSIVITY";
        case PARAM_ID_SKY_CONDITION: return "SKY_CONDITION";
        case PARAM_ID_AIR_TEMPERATURE: return "AIR_TEMPERATURE";
        case PARAM_ID_HUMIDITY: return "HUMIDITY";
        case PARAM_ID_SUBJECT_RANGE: return "SUBJECT_RANGE";
        default: return "";
    }
}

/** 
  * @brief Function Fetch Paramerters
  * @param none
  * @return Execution Result
  */
T_PsdkReturnCode PsdkParam_FetchParams(void)
{
    for(uint8_t i = 0; i < PARAM_ID_NUM_TRACKED; i++){
        /*!< Check whether param has been read before then set state to fetch again */
        if(s_camParams[i].state != PARAM_STATE_NOT_YET_READ) {
            s_camParams[i].state = PARAM_STATE_FETCH_AGAIN;
        }
    }
    
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/** 
  * @brief Function used for checking whether parameters has been initialized 
  * @param none
  * @return True - Parameters have been read
  *         False - Parameters haven't read all
  */
T_PsdkReturnCode PsdkParam_Initialized(void)
{
    for(uint8_t i = 0; i < PARAM_ID_NUM_TRACKED; i++){
        /*!< Check whether param has been read before then set state to fetch again */
        if(s_camParams[i].state == PARAM_STATE_NOT_YET_READ) {
            return PSDK_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
        }
    }
    
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/** 
  * @brief Function used for checking whether parameters has been recevied 
  * @param none
  * @return Return parameter index or if read all return param num tracked 
  */
T_PsdkReturnCode PsdkParam_ReceivedAll(void)
{
    for(uint8_t i = 0; i < PARAM_ID_NUM_TRACKED; i++){
        /*!< Check whether param has been read before then set state to fetch again */
        if(s_camParams[i].state == PARAM_STATE_NOT_YET_READ || s_camParams[i].state == PARAM_STATE_FETCH_AGAIN) {
            return PSDK_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
        }
    }
    
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/** 
  * @brief Function used for getting param
  * @param param Index of that param needs to get
  * @param value pointer to a space memory wheter used for storage that param 
  * @return Execution result 
  */
T_PsdkReturnCode PsdkParam_GetParam(uint16_t param, int16_t *value)
{
    /*!< Check whether param has been read before then set state to fetch again */
    if(!s_camParams[param].seen) {
        return PSDK_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } 
    else {
        *value = s_camParams[param].value;
    }
    
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/** 
  * @brief Function used for setting parameters 
  * @param param index of that param
  * @param value the value needs to set to memory
  * @return Execution Result 
  */
T_PsdkReturnCode PsdkParam_SetParam(uint16_t param, const int16_t value)
{
    T_PsdkReturnCode gsdkStat = PSDK_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    
    /*!< Check whether param is consistent or exist */
    if((s_camParams[param].state == PARAM_STATE_CONSISTENT &&
        s_camParams[param].value == value) || s_camParams[param].state == PARAM_STATE_NONEEXISTANT) {
        return PSDK_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
        
    PsdkLogger_UserLogInfo("Set %d - %d", param, value);
    
    /*!< Set state to notify that need to set the new param */
    s_camParams[param].state = PARAM_STATE_ATTEMPTING_TO_SET;
    s_camParams[param].value = value;
    
    /*!< Call a callback function specified that platform used */
    gsdkStat = PsdkParam_Write(param, value);
    
    /*!< Get time */
    PsdkOsal_GetTimeMs(&s_lastSetMs);
    
    return gsdkStat;
}

/** 
  * @brief Function handle param value 
  * @param arg Depending on the platform used.
  * @param none
  * @return None
  */
void PsdkParam_HandleParamValue(const char* name, const uint16_t param, const int16_t value)
{
    /*!< Process param list and change the state */
//    for(uint8_t i = 0; i < PARAM_ID_NUM_TRACKED; i++) {
        /*!< Check param name is valid in the param list*/
        if(!strcmp(name, PsdkParam_GetParamName(param))) {
            /*!< Switch to notify that just seen that param */
            s_camParams[param].seen = true;
            
            switch(s_camParams[param].state) {
                case PARAM_STATE_NONEEXISTANT:
                case PARAM_STATE_NOT_YET_READ: {
                    /*!< Get value to list */
                    s_camParams[param].value = value;
                    s_camParams[param].state = PARAM_STATE_CONSISTENT;
                    
                    DebugInfo("GOT [%s][%d]: %d", PsdkParam_GetParamName(param), param, s_camParams[param].value);

                    break;
                }
                case PARAM_STATE_FETCH_AGAIN: {
                    /*!< Param has been set is compared with value just read */
                    if(s_camParams[param].value != value) {
                        s_camParams[param].state = PARAM_STATE_FETCH_AGAIN;
                    }
                    else {
                        s_camParams[param].value = value;
                        s_camParams[param].state = PARAM_STATE_CONSISTENT;
                    }
                    
                    DebugInfo("GOT_FETCH [%s][%d]: %d", PsdkParam_GetParamName(param), param, s_camParams[param].value);

                    break;
                }
                case PARAM_STATE_CONSISTENT: {
                    /*!< Receive param */
                    s_camParams[param].value = value;
                    break;
                }
                case PARAM_STATE_ATTEMPTING_TO_SET: {
                    
                    /*!< The state of that param need to set*/
                    if(s_camParams[param].value == value) {
                        s_camParams[param].state = PARAM_STATE_CONSISTENT;
                    }
                    else {  /*!< Clear that param and waiting for response later*/
                        s_camParams[param].value = 0;
                        s_camParams[param].state = PARAM_STATE_CONSISTENT;
                    }
                    break;
                }
            }
//            break;  
//        }
    }
}


/** 
  * @brief Function used for update paramters. Check/set/get 
  * @param none
  * @return Exection Result 
  */
T_PsdkReturnCode PsdkParam_Update(void)
{
    T_PsdkReturnCode gsdkStat = PSDK_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    
    uint32_t timeNowMs;
    int16_t  paramTemp;
    PsdkOsal_GetTimeMs(&timeNowMs);
    
       /*!< Retry initial param retrieval */
       for(uint8_t i = 0; i < PARAM_ID_NUM_TRACKED; i++){
            /*!< Check whether param has been read before then set state to fetch again */
            if(s_camParams[i].state == PARAM_STATE_NOT_YET_READ || s_camParams[i].state == PARAM_STATE_FETCH_AGAIN) {
                
                /*!< Wating for sending a request read */
                if((timeNowMs - s_lastRequestMs) > PARAM_RETRY_PERIOD) {
                    s_lastRequestMs = timeNowMs;
                   
                    PsdkParam_Read(i, &paramTemp);
                   
                    DebugInfo("Read[%d]: %d", i, paramTemp);

                    PsdkParam_HandleParamValue(PsdkParam_GetParamName(i), i, paramTemp);
                    
                    if(!s_camParams[i].seen) {
                        /*!< Increase time check */
                        s_camParams[i].fetchAttempts++;
                    }
                }
            }
        }
   
    /*!< Retry param set */
    for(uint8_t i = 0; i < PARAM_ID_NUM_TRACKED; i++) {
        /*!< Check if param need to set  */
        if((s_camParams[i].state == PARAM_STATE_ATTEMPTING_TO_SET) && (timeNowMs - s_lastSetMs > PARAM_RETRY_PERIOD)) {
            /*!< Save param to platform */
            gsdkStat = PsdkParam_Write(s_camParams[i].camIdx, s_camParams[i].value);
            
            s_lastSetMs = timeNowMs;
            
            /*!< Check param has been seen or not */ 
            if(!s_camParams[i].seen) {
                /*!< Increase time check */
                s_camParams[i].fetchAttempts++;
            }
            
            /*!< Switch to FETCH again to check */
            s_camParams[i].state = PARAM_STATE_FETCH_AGAIN;
            s_camParams[i].seen = false;
        }
    }
    
    /*!< Check for nonexistent parameter  */
    for(uint8_t i = 0; i < PARAM_ID_NUM_TRACKED; i++) {
        if(!s_camParams[i].seen && s_camParams[i].fetchAttempts > PARAM_MAX_FETCH_ATTEMPTS) {
            s_camParams[i].state = PARAM_STATE_NONEEXISTANT;
            PsdkLogger_UserLogInfo("Param[%d] is not exist", i);
        }
    }
    
    return gsdkStat;
}

/** 
  * @brief Read param at startup
  * @param none
  * @param none
  * @return None
  */

T_PsdkReturnCode PsdkParam_ReadParamStartup(void)
{
    int16_t paramTemp = 0;
    
    for(uint8_t i = 0; i < PARAM_ID_NUM_TRACKED; i++) {
        /*!< Read param from flash and override to param list */
        if(PsdkParam_Read(i, &paramTemp) == PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            s_camParams[i].value = paramTemp;
        }
        else {
            /*!< Write as default */
            PsdkParam_Write(i, s_camParams[i].value);
        }
    }
}

/** 
  * @brief
  * @param none
  * @param none
  * @return None
  */
static void *ProcessSetParam_Task(void *arg)
{
    UNUSED(arg);
    
    static uint8_t index = 0;
    
    while(1)
    {
        /*!< Task process in 100 Hz*/
        PsdkOsal_TaskSleepMs(1000);
        
        if(index < PARAM_ID_NUM_TRACKED) {
            PsdkParam_SetParam(index, s_camParams[index].value + 10);
            index++;
        }
    }
}

/** 
  * @brief
  * @param none
  * @param none
  * @return None
  */
static void *ProcessParam_Task(void *arg)
{
    UNUSED(arg);
    
    /*!< Reset param*/
    PsdkParam_Reset();
    
    /*!< Check param default with flash */
    PsdkParam_ReadParamStartup();
    /*!< Request param again. */ 

    while(1)
    {
        /*!< Task process in 100 Hz*/
        PsdkOsal_TaskSleepMs(100 / PARAM_PROCESS_TASK_FREQ);
        
        PsdkParam_Update();
    }
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief GSDK protocol initialization.
 * @param None.
 * @return None.
 */
T_PsdkReturnCode PsdkStorage_Init(T_PsdkStorageHandler *handler)
{
   /* Unlock the Flash Program Erase controller */
    HAL_FLASH_Unlock();

    /*!< EEPROM Init*/
    if(EE_Init() != EE_OK) {
        PsdkLogger_UserLogError("EEPROM Inititialized failed!");
    }
    
    /*!< Reset param*/
    PsdkParam_Reset();
    
    /*!< Check param default with flash */
    PsdkParam_ReadParamStartup();
    
    /*!< Add handler */
    handler->PsdkStorage_Initialized = PsdkParam_Initialized;
    handler->PsdkStorage_ReceviedAll = PsdkParam_ReceivedAll;
    handler->PsdkStorage_Update         = PsdkParam_Update;
    handler->PsdkStorage_GetID          = PsdkParam_GetParamName;
    handler->PsdkStorage_SetParam       = PsdkParam_SetParam;
    handler->PsdkStorage_GetParam       = PsdkParam_GetParam;
    
    /*!< Create Task Handle Parameters*/
//    if(PsdkOsal_TaskCreate(&s_paramUpdateThread, ProcessParam_Task, "process_param", PARAM_TASK_STACK_SIZE, NULL) != 
//        PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//        PsdkLogger_UserLogError("Create Process Param Task Failed!");

//        return PSDK_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
//    }
     /*!<TEST ONLY */
//    if(PsdkOsal_TaskCreate(&s_paramSetThread, ProcessSetParam_Task, "set_param", PARAM_TASK_STACK_SIZE, NULL) != 
//        PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//        PsdkLogger_UserLogError("Create Set Param Task Failed!");
//            
//        return PSDK_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
//    }
        
    return PSDK_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}



/************************ (C) COPYRIGHT GREMSY  **********END OF FILE******/
