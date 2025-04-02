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
#ifndef __FLIR_FLIR_FLIR_H
#define __FLIR_FLIR_FLIR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "dji_typedef.h"
#include "std_typedef.h"


/* Private includes ----------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/** @defgroup FLIR_VIDEO_SETUP
    @{
*/#ifndef __FLIR_VIDEO_SETUP
#define __FLIR_VIDEO_SETUP

typedef enum
{
    FLIR_VIDEO_TRANSMISSION_IR = 0,      /// HDMI output IR
    FLIR_VIDEO_TRANSMISSION_VIS = 1,        /// HDMI output VIS
    FLIR_VIDEO_TRANSMISSION_VIS_IR = 2      /// HDMI output VIS and IR(small)
} E_FlirVideoTransmision;

#define IS_FLIR_VIDEO_TRANSMISION(TRANSMISION) ((TRANSMISION == FLIR_VIDEO_TRANSMISSION_IR) || \
                                                (TRANSMISION == FLIR_VIDEO_TRANSMISSION_VIS) || \
                                                (TRANSMISION == FLIR_VIDEO_TRANSMISSION_VIS_IR))

typedef enum
{
    FLIR_VIDEO_RECORDING_IR = 0,
    FLIR_VIDEO_RECORDING_VIS,
    FLIR_VIDEO_RECORDING_IR_VIS,
} E_FlirVideoRecording;

#endif
/**
    @}
*/


/** @defgroup FLIR_SETUP
    @{
*/#ifndef __FLIR_SETUP
#define __FLIR_SETUP

typedef enum
{
    FLIR_SCENE_LINAER = 0,
    FLIR_SCENE_DEFAULT,
    FLIR_SCENE_SEA_OR_SKY,
    FLIR_SCENE_OUTDOOR,
    FLIR_SCENE_INDOOR,
    FLIR_SCENE_MARITIME,
    FLIR_SCENE_CUSTOM1,
    FLIR_SCENE_CUSTOM2,
} E_FlirScene;

#define IS_FLIR_SCENE(SCENE) ((SCENE == FLIR_SCENE_LINAER) || \
                                (SCENE == FLIR_SCENE_DEFAULT) || \
                                (SCENE == FLIR_SCENE_SEA_OR_SKY) || \
                                (SCENE == FLIR_SCENE_OUTDOOR) || \
                                (SCENE == FLIR_SCENE_INDOOR) || \
                                (SCENE == FLIR_SCENE_MARITIME) || \
                                (SCENE == FLIR_SCENE_CUSTOM1) || \
                                (SCENE == FLIR_SCENE_CUSTOM2))

typedef enum
{
    FLIR_PATETTE_WHITEHOT = 0,
    FLIR_PATETTE_BLACKHOT,
    FLIR_PATETTE_REDHOT,
    FLIR_PATETTE_RAINBOW,
    FLIR_PATETTE_IRONBOW,
    FLIR_PATETTE_LAVA,
    FLIR_PATETTE_ARCTIC,
    FLIR_PATETTE_GLOBOW,
    FLIR_PATETTE_FUSION,
    FLIR_PATETTE_INSTALERT,
    FLIR_PATETTE_GREYRED,
    FLIR_PATETTE_SPEPIA,
    FLIR_PATETTE_IRONBOW2,
    FLIR_PATETTE_RAINBOWHC,
    FLIR_PATETTE_GREENHOT,
    FLIR_PATETTE_WHITEHOTISO,
    FLIR_PATETTE_BLACKHOTISO,
    FLIR_PATETTE_FUSIONISO,
    FLIR_PATETTE_RAINBOWISO,
    FLIR_PATETTE_GLOBOWISO,
    FLIR_PATETTE_IRONBOWWHITEHOTISO,
    FLIR_PATETTE_IRONBOWBLACKHOTISO,
    FLIR_PATETTE_SEPIAISO,
    FLIR_PATETTE_MIDRANGEWHITEHOTISO,
    FLIR_PATETTE_MIDRANGEBLACKHOTISO,
    FLIR_PATETTE_ICEFIREISO,
    FLIR_PATETTE_RAINBOWHCISO,
    FLIR_PATETTE_REDHOTISO,
    FLIR_PATETTE_GREENHOTISO,
    FLIR_PATETTE_ARCTICBLACKHOTISO,
    
    FLIR_PALETTE_MAX_COUNT
} E_FlirPalette;

typedef enum _E_FlirRotation
{
    FLIR_ROTARY_0 = 0,
    FLIR_ROTARY_180,
}  E_FlirRotation;

#endif
/**
    @}
*/

/** @defgroup FLIR_CAPTURE_IMAGE
    @{
*/#ifndef __FLIR_CAPTURE_IMAGE
#define __FLIR_CAPTURE_IMAGE

typedef enum
{
    FLIR_FILE_FORMART_JPEG_TIFF = 1,    ///  1 = JPEG & TIFF
    FLIR_FILE_FORMART_FFF       = 2,              /// 2 FFF
} E_FlirFileFormat;

#endif
/**
    @}
*/


/** @defgroup FLIR_RECORD_PARAM
    @{
*/#ifndef __FLIR_RECORD_PARAM
#define __FLIR_RECORD_PARAM

typedef enum
{
    FLIR_RECORD_ID_ALL,
    FLIR_RECORD_ID_FIRST,
    FLIR_RECORD_ID_SECOND
} E_FlirRecordId;

typedef enum
{
    FLIR_VIDEO_FILE_TYPE_H264 = 1,
    FLIR_VIDEO_FILE_TYPE_TLFF,
} E_FlirVideoFileType;

typedef enum
{
    FLIR_CLEAR_SKIES     = 0,
    FLIR_SCATTERED_SKIES = 25,
    FLIR_CLOUDY_SKIES    = 75,
} E_FlirSkyCondition;

#define IS_FLIR_SKYCONDITION(CONDITION) ((CONDITION == FLIR_CLEAR_SKIES) || \
                                        (CONDITION == FLIR_SCATTERED_SKIES) || \
                                        (CONDITION == FLIR_CLOUDY_SKIES))

typedef enum
{
    HUMIDITY_LOW_30         = 30,
    HUMIDITY_MEDIUM_45      = 45,
    HUMIDITY_HIGH_60        = 60,
} E_FlirHumidity;

#define IS_FLIR_HUMIDITY(HUMIDITY) ((HUMIDITY == HUMIDITY_LOW_30) || \
                                    (HUMIDITY == HUMIDITY_MEDIUM_45) || \
                                    (HUMIDITY == HUMIDITY_HIGH_60))

typedef enum _E_FlirZoomIRLevel {

    FLIR_ZOOM_IR_LEVEL_0     = 0,
    FLIR_ZOOM_IR_LEVEL_1     = 1,
    FLIR_ZOOM_IR_LEVEL_2     = 2,
    FLIR_ZOOM_IR_LEVEL_4     = 4,
    FLIR_ZOOM_IR_MAX_LEVEL,
} E_FlirZoomIRIndex;

typedef enum _E_FlirZoomVISLevel {

    FLIR_ZOOM_VIS_LEVEL_0     = 0,
    FLIR_ZOOM_VIS_LEVEL_1     = 1,
    FLIR_ZOOM_VIS_LEVEL_2     = 2,
    FLIR_ZOOM_VIS_LEVEL_4     = 4,
    FLIR_ZOOM_VIS_LEVEL_8     = 8,
    FLIR_ZOOM_VIS_MAX_LEVEL,
} E_FlirZoomVISIndex;


/*!< Temperature unit */
typedef enum _E_FlirTempUnit
{
    FLIR_TEMP_UNIT_C = 0,
    FLIR_TEMP_UNIT_F = 1,
} E_FlirTempUnit;

#define IS_FLIR_TEMP_UNIT(UNIT) ((UNIT == FLIR_TEMP_UNIT_C) || \
                                (UNIT == FLIR_TEMP_UNIT_F))

/*!< Set OSD Temperature meter */
typedef enum _E_FlirTempMeter
{
    FLIR_SPOT_METER_OFF = 0,
    FLIR_SPOT_METER_ON = 1,
} E_FlirSpotMeter;

#define IS_FLIR_SPOT_METER(SPOT) ((SPOT == FLIR_SPOT_METER_OFF) || \
                                (SPOT == FLIR_SPOT_METER_ON))
#endif
/**
    @}
*/
/* Exported macro ------------------------------------------------------------*/
#define CAM_EVENT_SET_DEFAULT_PARAM            BIT0
#define CAM_EVENT_SEND_HANDSHAKE               BIT1
#define CAM_EVENT_SEND_INFO                    BIT2
#define CAM_EVENT_RESET_DIGITAL_ZOOM_FACTOR    BIT3
#define CAM_EVENT_GET_ZOOM_VALUE               BIT4

#define CAM_EVENT_SET_START_RECORD              BIT5
#define CAM_EVENT_SET_STOP_RECORD               BIT6
#define CAM_EVENT_SET_START_SHOOT_PHOTO         BIT7
#define CAM_EVENT_SET_STOP_SHOOT_PHOTO          BIT8

#define CAM_EVENT_SET_ZOOM_IN                  BIT10
#define CAM_EVENT_SET_ZOOM_OUT                 BIT11


#define CAM_EVENT_SET_LAYOUT                    BIT12
#define CAM_EVENT_SET_DIGICAM_CONTROL           BIT13
#define CAM_EVENT_SET_DIGICAM_CONFIG            BIT14
#define CAM_EVENT_SET_PARMETERS                 BIT15

#define CAM_EVENT_ALLS                          ((CAM_EVENT_SET_PARMETERS << 1) - 1)

/* Exported types ------------------------------------------------------------*/


/*!< This structure contain flir information */
typedef struct _T_FlirCamera {
    
    bool                    isStoringParams;
    
    bool                    isRecording;
    
    T_DjiAttitude3d         gimbalAttitude;
    
    uint16_t                rateReq;
    
    E_FlirZoomIRIndex       zoomIRLevel;
    
    E_FlirZoomVISIndex      zoomVISLevel;
    
    int16_t                 videoTransmission;
    
    int16_t                 scene;
    
    int16_t                 fileFormat;
    
    int16_t                 videoFileType;
    
    int16_t                 palette;
    
    int16_t                  imageRotation;
    
    int16_t                 commandFFC;
    
    /*!< Set MSX function*/
    int16_t                 isMSXEnable;
    int16_t                 MSXLength; /*!< 0 thr 100*/
    
    int16_t                 tempUnit;

    int16_t                 spotMeter;
    
    /*!< Set Subject Emissivity */
    int16_t                 emissivity;
    
    /*!< Set SkyCondition*/
    int16_t                 skyCondition;
    
    int16_t                 airTemperature;
    
    int16_t                 humidity;
    
    int16_t                subjectRange;
    
    int16_t                debugCount;
} T_FLIRCamera;

/*!< Global varaible camera */
extern T_FLIRCamera * const FLIRCamera;

/* Exported functions prototypes ---------------------------------------------*/
/*!< WIRIS send data callback function type*/
typedef int (*SendCallbackFunc)(const uint8_t *pSendData, uint16_t dataLen);

/*!< WIRIS receive data callback function type*/
typedef int (*ReceiveCallbackFunc) (uint8_t *pReceiveData, uint16_t dataLen);

/*!<====================== FLIR_FUNCTIONS =================================== */

/** @addtogroup 
  * @{
  */
bool GremsyFLIR_isConnected(void );
T_DjiReturnCode GremsyFLIR_SendHeartbeat(void);
T_DjiReturnCode GremsyFLIR_SendPing(void);
T_DjiReturnCode GremsyFLIR_SendSystemTime(void);
T_DjiReturnCode GremsyFLIR_SendAttitude(void);
T_DjiReturnCode GremsyFLIR_SendGlobalPositionInit(void);
T_DjiReturnCode GremsyFLIR_SendGlobalPositionInitCov(void);
T_DjiReturnCode GremsyFLIR_SendHilGPS(void);
T_DjiReturnCode GremsyFLIR_SendMountStatus(const T_DjiAttitude3d *attitude);
T_DjiReturnCode GremsyFLIR_SendDoControlVideo(const uint8_t trans);

T_DjiReturnCode GremsyFLIR_SendDoDigicamConfigure( uint8_t scene,
                                                    uint8_t palette,
                                                    uint8_t rotation,
                                                    uint8_t MSX_enable,
                                                    uint8_t MSX_length);
                                                    
T_DjiReturnCode GremsyFLIR_SendDoDigicamControl( float session,
                                                  float zoom_pos,
                                                  float zoom_step,
                                                  float focus_lock,
                                                  float shot_cmd,
                                                  float command_id,
                                                  float shot_id);
                                                  
T_DjiReturnCode GremsyFLIR_SendZoom(uint8_t zoom_VIS, uint8_t zoom_IR);
T_DjiReturnCode GremsyFLIR_SendImageStartCapture(void);
T_DjiReturnCode GremsyFLIR_SendImageStopCapture(void);
T_DjiReturnCode GremsyFLIR_SendVideoStartCapture(void);
T_DjiReturnCode GremsyFLIR_SendVideoStopCapture(void);
T_DjiReturnCode GremsyFLIR_SendUser1(uint8_t tempUnit, 
                                        uint8_t spotMeter, 
                                        uint8_t emissivity, 
                                        uint8_t skyCondition,
                                        int16_t airTemperature,
                                        uint8_t humidity,
                                        uint16_t subjectRange);
/**
  * @} // 
  */


/*!<====================== ZOOM FUNCTIONS =================================== */

/** @addtogroup 
* @{
*/

T_DjiReturnCode GsdkFLIR_CameraInit(void) ;
T_DjiReturnCode GsdkFLIR_CameraDeInit(void);
T_DjiReturnCode GsdkFLIR_CameraSetEvent(const uint32_t uxBitsToSet);

/**
* @} // 
*/
                                                  
/* Private defines -----------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* __FLIR_FLIR_FLIR_H */

/*********** Portions COPYRIGHT 2020 Gremsy.Co., Ltd.*****END OF FILE****/
