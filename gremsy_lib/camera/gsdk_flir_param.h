/**
 ********************************************************************
 * @file    gsdk_ofil_cam.h
 * @version V1.0.0
 * @date    2021/03/01
 * @brief   This file to contain ofil camera protocol
 *
 * @copyright (c) 2020-2021 GREMSY. All rights reserved.
 *
 * All information contained herein is, and remains, the property of GREMSY.
 * The intellectual and technical concepts contained herein are proprietary
 * to GREMSY and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of GREMSY.
 *
 * If you receive this source code without GREMSY’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify GREMSY of its removal. GREMSY reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GSDK_OFIL_CAM_H
#define GSDK_OFIL_CAM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "psdk_typedef.h"
#include "std_typedef.h"

/* Exported constants --------------------------------------------------------*/


/* Exported macros -----------------------------------------------------------*/
#define CAM_EVENT_START_SHOOT_PHOTO             BIT0
#define CAM_EVENT_STOP_SHOOT_PHOTO              BIT1
#define CAM_EVENT_START_RECORD_VIDEO            BIT2
#define CAM_EVENT_STOP_RECORD_VIDEO             BIT3
#define CAM_EVENT_VIDEO_CAPTURE_STATUS          BIT4

#define CAM_EVENT_SET_ZOOM                      BIT5
#define CAM_EVENT_SET_FOCUS                     BIT6

#define CAM_EVENT_SET_GAIN                      BIT8
#define CAM_EVENT_SET_DATE_TIME                 BIT9
#define CAM_EVENT_SET_DISLAY_MODE               BIT10
#define CAM_EVENT_SET_UV_COLOR                  BIT11
#define CAM_EVENT_SET_LI                        BIT12
#define CAM_EVENT_SET_LIF                       BIT13
#define CAM_EVENT_SET_GPS                       BIT14
#define CAM_EVENT_SET_THR                       BIT15
#define CAM_EVENT_SET_COUNT_WINDOW_SIZE         BIT16
#define CAM_EVENT_GET_COUNT_VALUE               BIT17


#define CAM_EVENT_GET_VALUES                    BIT20

#define CAM_EVENT_ALLS                          ((CAM_EVENT_GET_VALUES << 1) - 1)

/* Exported types ------------------------------------------------------------*/

/*!< Send data callback function type*/
typedef T_PsdkReturnCode (*SendCallbackFunc)(const uint8_t *buf, uint16_t len);
/*!< Receive data callback function type*/
typedef T_PsdkReturnCode (*ReceiveCallbackFunc) (uint8_t *buf, uint16_t len, uint16_t *realLen);

/** 
 * @brief Exposure
 * @details 
 * @note 
 */
typedef enum {
    AE_AUTO_EXPOSURE = 0,
    AE_SHUTTER_SPEED_1_1S = 1,
    AE_SHUTTER_SPEED_1_2S,
    AE_SHUTTER_SPEED_1_3S,
    AE_SHUTTER_SPEED_1_6S,
    AE_SHUTTER_SPEED_1_12S,
    AE_SHUTTER_SPEED_1_25S,
    AE_SHUTTER_SPEED_1_50S,
    AE_SHUTTER_SPEED_1_75S,
    AE_SHUTTER_SPEED_1_100S,
    AE_SHUTTER_SPEED_1_120S,
    AE_SHUTTER_SPEED_1_150S,
    AE_SHUTTER_SPEED_1_215S,
    AE_SHUTTER_SPEED_1_300S,
    AE_SHUTTER_SPEED_1_425S,
    AE_SHUTTER_SPEED_1_600S,
    AE_SHUTTER_SPEED_1_1000S,
    AE_SHUTTER_SPEED_1_1250S,
    AE_SHUTTER_SPEED_1_1750S,
    AE_SHUTTER_SPEED_1_2500S,
    AE_SHUTTER_SPEED_1_3500S,
    AE_SHUTTER_SPEED_1_6000S,
    AE_SHUTTER_SPEED_1_10000S,
} E_Exposure;

/**
 * @brief Dislay mode command. 
 * @details This structure type is used to set display mode 
 * @note 
 */
typedef enum {
    DISPLAY_MODE_VISIBLE = 1, 
    DISPLAY_MODE_UV = 2,
    DISPLAY_MODE_COMBINED = 3,
    DISPLAY_MODE_NUM_TRACK = 4,
} E_DisplayMode;


/**
 * @brief
 * @details This structure type is used to
 * @note 
 */
typedef enum {
    UV_COLOR_RED = 0,
    UV_COLOR_ORANGE,
    UV_COLOR_YELLOW,
    UV_COLOR_GREEN,
    UV_COLOR_LIGHT_BLUE,
    UV_COLOR_DARK_BLUE,
    UV_COLOR_PURPLE,
    UV_COLOR_PINK,
    UV_COLOR_RED_TRANSPARENT,
    UV_COLOR_ORANGE_TRANSPARENT,
    UV_COLOR_YELLOW_TRANSPARENT,
    UV_COLOR_GREEN_TRANSPARENT,
    UV_COLOR_LIGHT_BLUE_TRANSPARENT,
    UV_COLOR_DARK_BLUE_TRANSPARENT,
    UV_COLOR_PURPLE_TRANSPARENT,
    UV_COLOR_PINK_TRANSPARENT,
    
    UV_COLOR_NUM_TRACKED,
} E_UVColor;


/*!< Ofil_Device_flags*/
typedef enum {
    OFIL_DEVICE_FLAGS_UNKNOWN      = 0,
    
    OFIL_DEVICE_FLAGS_CONNECTION  = 1,
    
    OFIL_DEVICE_FLAGS_READY = 2,
    
    OFIL_DEVICE_FLAGS_ERROR_COMMAND = 4,
    
    OFIL_DEVICE_FLAGS_ERROR_HW = 8,
    
    OFIL_DEVICE_FLAGS_OFF_CAM = 16,
} E_DeviceFlags;

/*!< Count frame */
typedef enum {
    OFIL_NO_COUNT_FRAME = 0,
    OFIL_SMALL_COUNT_FRAME = 1,
    OFIL_MEDIUM_COUNT_FRAME = 2,
    OFIL_LARGE_COUNT_FRAME = 3,
    OFIL_COUNT_FRAME_NUM_TRACKED = 4,
} E_CountFrame; 

/*!<  GPS Format Setting */
typedef enum {
    OFIL_GPS_NMEA_FORMAT = 0,   /*!< NMEA Format */
    OFIL_GPS_DD_FORMAT = 1,     /*!< Decimal Degrees Format */
    OFIL_GPS_DMS_FORMAT = 2,    /*!< Degrees, Minutes, seconds Format */
} E_GPSFormat;

/**
 * @brief UTC of Position
 * @details
 * @note 
 */
typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} T_Clock;

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
    T_Clock         clock;
    psdk_f64_t      latitude;  /*!< Latitude, unit: deg. */ 
    psdk_f64_t      longtitude; /*!< Longitude, unit: deg. */
    uint8_t         gpsQuality;
    uint16_t        numSat;
    psdk_f32_t      hdop; /*!< Horizontal dilution of precision, <1: ideal, 1-2: excellent, 2-5: good, 5-10: moderate, 10-20: fair, >20: poor. */
    psdk_f32_t      hfsl;      /*!< Height above mean sea level, unit: m. */
    uint8_t         antenaHeight;
    psdk_f32_t      geoidal;
    uint8_t         geoidalSep; 
    psdk_f32_t      ageInSec;
    uint32_t        stationId;
} T_GpsDataNmea;

/* Exported variables --------------------------------------------------------*/
typedef struct {
    uint8_t         flags;
    
    int8_t          manualZoomVal;
    int8_t          manualZoomMax;
    
    uint8_t         UVZoomVal;
    uint8_t         UVZoomMax;
    
    uint8_t         gain;
    E_UVColor       UVcolor;
    E_DisplayMode   displayMode;
    uint8_t         countWindowSize;
    int16_t         countValue;
    
    char            dateTime[30];
    bool            enableAF;
    
    bool            enableGPS;
    uint8_t         GPSFormat;
    T_GpsDataNmea   GPSData;
    
    bool            enableLongIntegration;
    uint8_t         longIntegrationVal;

    bool            enableTRH; /*!< Activate / deactivate TRH accessory*/
    uint8_t         playbackVal;
    
    bool            isRecording;
} T_OFILCamera; 

/*!< Global */
extern T_OFILCamera *OFILCamera;


/* Exported functions --------------------------------------------------------*/
/**
  * @brief Function is used to initialize the wiris protocol
  * @retval T_PsdkReturnCode
  */
T_PsdkReturnCode GsdkOfilProto_Init(void);

/**
  * @brief Function is used to deinitialize the wiris protocol
  * @retval T_PsdkReturnCode
  */
T_PsdkReturnCode GsdkOfilProto_DeInit(void);

/**
  * @brief Function is used to register the function send 
  * @retval T_PsdkReturnCode
  */
T_PsdkReturnCode GsdkOfilProto_RegSendDataFunc(SendCallbackFunc callbackFunc);

/**
  * @brief Function is used to register the function send 
  * @retval T_PsdkReturnCode
  */
T_PsdkReturnCode GsdkOfilProto_RegRecvDataFunc(ReceiveCallbackFunc callbackFunc);

/** @brief Ofil_ParseData
  *
  * @param1 proParse Pointer to the protocol handle
  * @param2 byteData data received from the camera
  * @return None
  */
T_PsdkReturnCode GsdkOfilProto_ProcessReceiveData(const uint8_t *pData, uint16_t dataLen);

/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
T_PsdkReturnCode GsdkOfilProto_PowerUp(void);
T_PsdkReturnCode GsdkOfilProto_PowerOff(void);

T_PsdkReturnCode GsdkOfilProto_CameraOnstatusQuery(void);

T_PsdkReturnCode GsdkOfilProto_DisplayMode(E_DisplayMode value);
T_PsdkReturnCode GsdkOfilProto_UVC(E_UVColor value);
T_PsdkReturnCode GsdkOfilProto_UVCQuery(void);

T_PsdkReturnCode GsdkOfilProto_SetGain(uint8_t value);
T_PsdkReturnCode GsdkOfilProto_GainQuery(void);

T_PsdkReturnCode GsdkOfilProto_SetCountWindowSize(uint8_t value);
T_PsdkReturnCode GsdkOfilProto_QueryCount(void);

T_PsdkReturnCode GsdkOfilProto_StartRecoding(const char *name);
T_PsdkReturnCode GsdkOfilProto_StopRecoding(void);
T_PsdkReturnCode GsdkOfilProto_TakePicture(const char *name);
T_PsdkReturnCode GsdkOfilProto_TakePictureCompleted(void);
T_PsdkReturnCode GsdkOfilProto_VideoCaptureStatusQuery(void);

/*!<====================== ZOOM FUNCTIONS =================================== */

/** @addtogroup 
  * @{
  */
T_PsdkReturnCode GsdkOfilProto_ManualZoom(uint8_t value);
T_PsdkReturnCode GsdkOfilProto_ManualZoomMaximum(void);
T_PsdkReturnCode GsdkOfilProto_UVZoomMaximum(void);
T_PsdkReturnCode GsdkOfilProto_UVZoom(uint8_t value);

/**
  * @} // 
  */
T_PsdkReturnCode GsdkOfilProto_LongIntegrationEnable(uint8_t value);
T_PsdkReturnCode GsdkOfilProto_LongIntegrationNumberOfFrame(uint8_t value);
T_PsdkReturnCode GsdkOfilProto_SetGPSE(uint8_t value);
T_PsdkReturnCode GsdkOfilProto_SetGPSFormat(uint8_t value);
T_PsdkReturnCode GsdkOfilProto_SetGPSValue(const T_GpsDataNmea *data);


T_PsdkReturnCode GsdkOfilProto_SetAF(bool value);
T_PsdkReturnCode GsdkOfilProto_SetTRHAccessory(uint8_t value);
T_PsdkReturnCode GsdkOfilProto_Playback(uint8_t value);

T_PsdkReturnCode GsdkOfilProto_SetDateAndTime(uint16_t year,
                                                uint8_t month,
                                                uint8_t day,
                                                uint8_t hour,
                                                uint8_t minute,
                                                uint8_t second);

T_PsdkReturnCode GsdkOfilProto_SetDateAndTimeString(const char *dateTime);



#ifdef __cplusplus
}
#endif

#endif //GSDK_OFIL_CAM_H

/************************ (C) COPYRIGHT GREMSY  *************END OF FILE******/
