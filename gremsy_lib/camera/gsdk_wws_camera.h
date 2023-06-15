/*******************************************************************************
 * Copyright (c) 2020, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 *
 * @file    camera_interface.h
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    Aug-27-2020
 * @brief   This file contains expand of the debugging
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAMERA_WIRIS_PROTOCOL_H
#define __CAMERA_WIRIS_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "dji_typedef.h"
#include "std_typedef.h"
#include "dji_positioning.h"

/* Private includes ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/*!< WIRIS send data callback function type*/
typedef T_DjiReturnCode (*SendCallbackFunc)(const uint8_t *pSendData, uint16_t dataLen);
/*!< WIRIS receive data callback function type*/
typedef T_DjiReturnCode (*ReceiveCallbackFunc) (uint8_t *buf, uint16_t len, uint16_t *realLen);

#define CAM_EVENT_SET_DEFAULT_PARAM             BIT0
#define CAM_EVENT_GET_SYSTEM_STATE              BIT1
#define CAM_EVENT_GET_SDCARD_STATE              BIT2
#define CAM_EVENT_RESET_DIGITAL_ZOOM_FACTOR     BIT3
#define CAM_EVENT_GET_ZOOM_VALUE                BIT4

#define CAM_EVENT_SET_START_PHOTO               BIT5
#define CAM_EVENT_SET_START_RECORD              BIT6
#define CAM_EVENT_SET_STOP_RECORD               BIT7

#define CAM_EVENT_SET_LAYOUT                    BIT8
#define CAM_EVENT_SET_MAINCAM                   BIT9
#define CAM_EVENT_SET_OPACITY                   BIT10

#define CAM_EVENT_SET_PALETTE                   BIT11
#define CAM_EVENT_SET_ALARM_MODE                BIT12
#define CAM_EVENT_SET_ALARM_VALUE               BIT13
#define CAM_EVENT_SET_ALARM_COLOR               BIT14

#define CAM_EVENT_SET_TIME_STABI                BIT15
#define CAM_EVENT_SET_COOL_REJECT               BIT16
#define CAM_EVENT_SET_HOT_REJECT                BIT17

#define CAM_EVENT_SET_SAY_HI                    BIT18
#define CAM_EVENT_SET_ZOOM_IN                   BIT19
#define CAM_EVENT_SET_ZOOM_OUT                  BIT20
#define CAM_EVENT_SET_REBOOT                    BIT21

#define CAM_EVENT_SET_GPS                       BIT22

#define CAM_EVENT_ALLS                          ((CAM_EVENT_SET_GPS << 1) - 1)

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Global Positioning System Fix Data
 * @details
    eg3. $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
    1    = UTC of Position
    2    = Latitude
    3    = N or S
    4    = Longitude
    5    = E or W
    6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
    7    = Number of satellites in use [not those in view]
    8    = Horizontal dilution of position
    9    = Antenna altitude above/below mean sea level (geoid)
    10   = Meters  (Antenna height unit)
    11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
            mean sea level.  -=geoid is below WGS-84 ellipsoid)
    12   = Meters  (Units of geoidal separation)
    13   = Age in seconds since last update from diff. reference station
    14   = Diff. reference station ID#
    15   = Checksum
 */
typedef struct {
    T_DjiTimeSyncAircraftTime  aircraftTime;
    dji_f64_t                  latitude;  /*!< Latitude, unit: deg. */ 
    dji_f64_t                  longtitude; /*!< Longitude, unit: deg. */
    uint8_t                     gpsQuality;
    uint16_t                    numSat;
    dji_f32_t                  hdop; /*!< Horizontal dilution of precision, <1: ideal, 1-2: excellent, 2-5: good, 5-10: moderate, 10-20: fair, >20: poor. */
    dji_f32_t                  hfsl;      /*!< Height above mean sea level, unit: m. */
    uint8_t                     antenaHeight;
    dji_f32_t                  geoidal;
    uint8_t                     geoidalSep; 
    dji_f32_t                  ageInSec;
    uint32_t                    stationId;
} T_GpsDataNmea;

T_DjiReturnCode WirisCamera_SetGPS(const T_GpsDataNmea *data);

/* Private defines -----------------------------------------------------------*/


/*!<====================== BAISC COMMANDS =================================== */
/** @addtogroup BASIC_COMMANDS
  * @{
  */
T_DjiReturnCode WirisCamera_isWirisConnected(void);
T_DjiReturnCode WirisCamera_ActivateCamera(void);
T_DjiReturnCode WirisCamera_GetSerialNumber(char *serial_number);
T_DjiReturnCode WirisCamera_GetArticleNumber(char *article_number);
T_DjiReturnCode WirisCamera_GetFirmwareVersion(char *fw_version);

/**
  * @} // BASIC_COMMANDS
  */
  
/*!<====================== RANGE_FUNCTIONS =================================== */

/** @addtogroup 
  * @{ RANGE_FUNCTIONS 
  * @Note Range settings is available only for WIRIS Pro
  */
  
/*!< Range mode */

typedef enum _E_WirisRange {
    WIRIS_RANGE_AUTOMATIC,
    WIRIS_RANGE_MANUAL,
    WIRIS_RANGE_SPAN,
    WIRIS_RANGE_COUNT,
} E_WirisRange;

  
T_DjiReturnCode WirisCamera_GetRangMode(E_WirisRange *range); 
T_DjiReturnCode WirisCamera_SetRangMode(E_WirisRange range);
T_DjiReturnCode WirisCamera_GetManualRange(float *min, float max);
T_DjiReturnCode WirisCamera_SetManualRange(float min, float max);
T_DjiReturnCode WirisCamera_GetSpanRange(float *min, float max);
T_DjiReturnCode WirisCamera_SetSpanRange(float min, float max);
T_DjiReturnCode WirisCamera_GetEnvironment(float *min, float max);
T_DjiReturnCode WirisCamera_GetListEnvironment(uint8_t list);
T_DjiReturnCode WirisCamera_SetEnvironment(float temp);

/**
  * @} // RANGE_FUNCTIONS
  */
  
/*!<====================== WIRIS Security thermal parameters================= */

/** @addtogroup WIRIS Security thermal parameters
  * @{ Specific parameters for WWS thermal camera.
  * 
  */
#define TIME_STATBILIZATION_MAX         5 /*!< Max stabilization 5 seconds. Each time increase 0.5 seconds*/
#define HOT_COOL_REJECTION_PERCENT_MAX  30 /*!< Max 30 percent each time increase or decrease 0.5 percents*/
#define WIRIS_MULTIPLE_VALUE            5
/**
 * @brief Function sets thermal camera time stabilization inseconds
 * @details 
 * @note 
 */
T_DjiReturnCode WirisCamera_GetTimeStabilization(float *timeStablization);
T_DjiReturnCode WirisCamera_SetTimeStabilization(const float timeStablization);
T_DjiReturnCode WirisCamera_GetHotRejection(float *rejection);
T_DjiReturnCode WirisCamera_SetHotRejection(const float rejection);
T_DjiReturnCode WirisCamera_GetCoolRejection(float *rejection);
T_DjiReturnCode WirisCamera_SetCoolRejection(const float rejection);

/**
  * @} // WIRIS Security thermal parameters
  */

/*!<====================== APPEARANCE_FUNCTIONS =================================== */
/** @addtogroup //APPEARANCE_FUNCTIONS
  * @{
  */

/**
 * @brief Set layout for the HDMI ouput
 * @details 
 * @note 
 */
typedef enum _E_WirisProLayout
{
    SLAY_WWP_INSPECTION,
    SLAY_WWP_SECURITY,
    SLAY_WWP_FULLSCREEN,
    SLAY_WWP_PIP,
}_E_WirisProLayout;

/**
 * @brief Set layout for the HDMI ouput
 * @details 
 * @note 
 */
typedef enum _E_WirisSecLayout
{
    SLAY_WWS_SECURITY = 0,
    SLAY_WWS_FULLSCREEN,
    SLAY_WWS_PIP,
    SLAY_WWS_COUNT_MAX,
} E_WirisSecLayout;

/**
 * @brief Set main camera
 * @details This structure type is used to
 * @note 
 */
typedef enum _E_WirisMainCamera
{
    SMCA_THERMAL = 0,
    SMCA_VISIBLE,
    SMCA_COUNT_MAX,
} E_WirisMainCamera;  
 
#define THERMAL_CAMERA_TRANSPARENCY_MAX         100 
#define THERMAL_CAMERA_TRANSPARENCY_MIN         10

T_DjiReturnCode WirisCamera_SetLayout(const E_WirisSecLayout layout);
T_DjiReturnCode WirisCamera_SetMainCamera(const E_WirisMainCamera main);
T_DjiReturnCode WirisCamera_SetThermalCameraTransparency(const uint8_t percent);
/**
  * @} // APPEARANCE_FUNCTIONS
  */
  
/*!<====================== ZOOM_COMMANDS =================================== */
/** @addtogroup ZOOM_COMMANDS
  * @{
  */
T_DjiReturnCode WirisCamera_ZoomIn(void);
T_DjiReturnCode WirisCamera_ZoomOut(void);
T_DjiReturnCode WirisCamera_GetThermalZoomVal(uint8_t *zoom_index, float *zoom_ratio);
T_DjiReturnCode WirisCamera_GetThermalZoomList(void);
T_DjiReturnCode WirisCamera_SetThermalZoomIndex(const uint8_t index);
T_DjiReturnCode WirisCamera_GetVisibleZoomVal(uint8_t *zoom_index, float *zoom_ratio);
T_DjiReturnCode WirisCamera_GetVisibleZoomlist(void);
T_DjiReturnCode WirisCamera_SetVisibleZoomIndex(const uint8_t index);
/**
  * @} // ZOOM_COMMANDS
  */

/*!<====================== PALATTES_COMMANDS =================================== */
/** @addtogroup PALATTES_COMMANDS
  * @{
  */

/**
 * @brief Palette list for camera 
 * @details This structure type is used to
 * @note 
 */
typedef enum {
    WIRIS_PALETTE_01_GRAY = 0,
    WIRIS_PALETTE_02_GRAY_I,
    WIRIS_PALETTE_03_BLACKGREEN,
    WIRIS_PALETTE_04_BLACKRED,
    WIRIS_PALETTE_05_BWRGB,
    WIRIS_PALETTE_06_WBRGB,
    WIRIS_PALETTE_07_BWIRON,
    WIRIS_PALETTE_08_BWIRON_I,
    WIRIS_PALETTE_09_IRON,
    WIRIS_PALETTE_10_FIRE,
    WIRIS_PALETTE_11_RAINBOW,
    WIRIS_PALETTE_12_RAINBOW_HC,
    WIRIS_PALETTE_13_NATURAL,
    WIRIS_PALETTE_14_SEPIA,
    WIRIS_PALETTE_15_TEMPERATURE,
    WIRIS_PALETTE_COUNT_MAX,
} E_WirisPalette;

T_DjiReturnCode WirisCamera_GetPalette(E_WirisPalette *index);
T_DjiReturnCode WirisCamera_GetPaletteList(void);
T_DjiReturnCode WirisCamera_SetPaletteName(const E_WirisPalette palette);
T_DjiReturnCode WirisCamera_SetPaletteIndex(const E_WirisPalette palette);
/**
  * @} // PALATTES_COMMANDS
  */
/*!<====================== CAPTURE_RECORD_FUNCTIONS =================================== */

/** @addtogroup CAPTURE_RECORD_FUNCTIONS
  * @{
  */
T_DjiReturnCode WirisCamera_Capture(void);
T_DjiReturnCode WirisCamera_IsCapturing(bool *isCapturing);
T_DjiReturnCode WirisCamera_RecordingStart(void);
T_DjiReturnCode WirisCamera_RecordingFinish(void);
T_DjiReturnCode WirisCamera_IsRecording(bool *isRecording);
  
/**
* @} // CAPTURE_RECORD_FUNCTIONS
*/
/*!<====================== ALARMS_FUNCTION=================================== */

/** @addtogroup 
  * @{ ALARMS_FUNCTION
  * TEST DONE
  */
/*!< Enum Alarm mode */
typedef enum _E_WirisAlarmMode
{
    WIRIS_ALARM_MODE_OFF = 0,
    WIRIS_ALARM_MODE_ABOVE,
    WIRIS_ALARM_MODE_BELOW,
    WIRIS_ALARM_MODE_BETWEEN,
    WIRIS_ALARM_MODE_OUTSIDE,
    WIRIS_ALARM_MODE_COUNT,
} E_WirisAlarmMode;
 
/*!< Define list color */
typedef enum _E_WirisAlarmColorList
{
    WIRIS_ALARM_RED_GREEN_BLUE = 0,
    WIRIS_ALARM_GREEN_RED_BLUE = 1,
    WIRIS_ALARM_BLUE_GREEN_RED = 2,
    WIRIS_ALARM_COLOR_LIST_MAX = 3,
} E_WirisAlarmColorList;

#define ALARM_THRESHOLD_MAX_PERCENT     50

T_DjiReturnCode WirisCamera_GetAlarmMode(E_WirisAlarmMode *alarmMode);
T_DjiReturnCode WirisCamera_SetAlarmMode(const E_WirisAlarmMode alarmMode);
T_DjiReturnCode WirisCamera_GetAlarmValue(float *thresholdBelow, float *thresholdAbove);
T_DjiReturnCode WirisCamera_SetAlarmValue(const float thresholdBelow, const float thresholdAbove);

/**
 * @brief Cannot set alram color 
 * @details This structure type is used to
 * @note 
 */
T_DjiReturnCode WirisCamera_GetAlarmColor(E_WirisAlarmColorList *color);
T_DjiReturnCode WirisCamera_SetAlarmColor(const E_WirisAlarmColorList color);


/**
  * @} // ALARMS_FUNCTION
  */

/*!<====================== MEMMORY_FUNCTIONS =================================== */
/** @addtogroup MEMMORY_FUNCTIONS
  * @{
  */
typedef enum {
    WIRIS_MEMMORY_STATUS_NA = 0,
    WIRIS_MEMMORY_STATUS_READY,
    WIRIS_MEMMORY_STATUS_CONNECTED,
}E_WirisMemoryStatus; 
  
T_DjiReturnCode WirisCamera_GetMemoryStatus(uint8_t *status);
T_DjiReturnCode WirisCamera_GetMemorySize(uint64_t* ssd_size, uint64_t* flash_drive_size, uint64_t* sd_card_size);
T_DjiReturnCode WirisCamera_GetMemoryFree(float* ssd_percent, float* flash_drive_percent, float* sd_card_percent);
T_DjiReturnCode WirisCamera_GetCapturedImages(uint64_t* ssd_captured_images, uint64_t* flash_captured_images, uint64_t* sd_captured_images);
T_DjiReturnCode WirisCamera_GetRecordedThermalVideo(uint64_t* seconds);
T_DjiReturnCode WirisCamera_GetRecordedVisibleVideo(uint64_t* seconds);

/**
  * @} // MEMMORY_FUNCTIONS
  */
  
/*!<====================== ZOOM FUNCTIONS =================================== */

/** @addtogroup 
  * @{
  */
T_DjiReturnCode WirisCamera_SetDataAndTime(const T_DjiTimeSyncAircraftTime *aircraftTime);
T_DjiReturnCode WirisCamera_Reboot(void);

/**
  * @} // 
  */
/*!<====================== WIRIS SECURITY STRUCTURE  ======================== */

/** @addtogroup 
  * @{
  */
/*!< Model of Wiris */
typedef enum _E_WirisModel
{
    MODEL_WIRIS_PRO = 0x01,
    MODEL_WIRIS_SEC
} E_WirisModel;


/**
 * @brief  T_WirisCamera
 * @details This structure type is used to
 * @note 
 */
typedef struct  {
    bool                isCamConnected;
    
    uint8_t             model;                  /*!< WWP or WWS :: E_WirisModel*/

    /*!< @NOTE: WIRIS SECURITY Thermal parameters */
    float               timeStabilization;      /*!< Thermal camera time stablization in seconds*/
    float               hotRejection;           /*!< Hot rejection in percent */
    float               coolRejection;          /*!< Cool rejection in percent */
    
    bool                isRecording;            /*!< Video status*/
    bool                isCapturing;            /*!< Video status*/
    
    uint8_t             layout;                 /*!< Layout for output. It depond the model */
    uint8_t             mainCam;                /*!< Main dispaly THERMAL/VISIBLE */
    uint8_t             transparency;           /*!< Thermal camera transparency in PIP Fusion layout from 10 to 100 percent */
    
    uint8_t             thermalZoomIdx; 
    float               thermalZoomRatio;         /*!< Thermal zoom ratio*/
    
    uint8_t             visibleZoomIdx;
    float               visibleZoomRatio;         /*!< Visible zoom ratio*/
    
    E_WirisPalette      paletteIdx;      /*!< Palette index */
    
    /*!< Alarms */
    E_WirisAlarmMode    alarmMode;
    float               thresholdBelow;
    float               thresholdAbove;
    E_WirisAlarmColorList   alarmColor;
    
    E_WirisMemoryStatus memoryStatus;

    uint64_t            ssdSize;
    uint64_t            flashDriveSize;
    uint64_t            sdCardSize;
    
    float               ssdFree;
    float               flashDriveFree;
    float                sdCardFree;
    
    /*!< GPS*/
    T_GpsDataNmea       GPSData;
    bool                isReadyGPS;
    bool                isSetDateTime;
    
} T_WIRISCamera;

/*!< Global */
extern T_WIRISCamera * const WIRISCamera;

T_DjiReturnCode GsdkWiris_CameraInit(void);
T_DjiReturnCode GsdkWiris_CameraDeInit(void);
T_DjiReturnCode GsdkWiris_CameraSetEvent(const uint32_t uxBitsToSet);

/**
  * @} // 
  */
#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_WIRIS_H */

/*********** Portions COPYRIGHT 2020 Gremsy.Co., Ltd.*****END OF FILE****/
