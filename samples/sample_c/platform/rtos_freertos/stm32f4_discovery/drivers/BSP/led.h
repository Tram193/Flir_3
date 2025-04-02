/**
 ********************************************************************
 * @file    button.h
 * @version V0.0.0
 * @date    2019/01/01
 * @brief   This is the header file for "button.c", defining the structure and
 * (exported) function prototypes.
 *
 * @copyright (c) 2017-2018 DJI. All rights reserved.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LED_H
#define LED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "dji_typedef.h"
/* Exported constants --------------------------------------------------------*/

/**
 * @brief Define Colors
 * @details This structure type is used to defined color for LED indicator 
 * @note 
 */
typedef enum {
    BLACK = 0,
    RED   = 1,
    GREEN = 2,
    BLUE  = 3,
    YELLOW= 4,
    PINK  = 5,
    CYAN  = 6,
    WHITE = 7, 
} T_LedColor;

/**
 * @brief Led common handler 
 * @details This structure type is used to define prototype handler 
 * @note 
 */
typedef struct  {
    /** @brief      ham set mau led rgb
        @param[in]  color mau cua led sang
        @return     none
    */
    void (*SetColor)(T_LedColor color);
    
    /** @brief      ham toggle mau led rgb
        @param[in]  color mau cua led sang
        @param[in]  toggle_time_ms thoi gian toggle cua led tinh bang ms
        @return     none
    */
    void (*ToggleColor)(T_LedColor color, uint32_t time);
    
    /** @brief      ham swap qua lai gia 2 mau led rgb
        @param[in]  color1 mau 1 cua led
        @param[in]  color2 mau 2 cua led
        @param[in]  swap_time_ms thoi gian de swap cua led tinh bang ms
        @return     none
    */
    void (*SwapColor)(T_LedColor color1, T_LedColor color2, uint32_t time);
    
    void (*LedProcess)(uint32_t timeNow);
    
} T_LedCommonHandler;

/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

T_DjiReturnCode GsdkLed_Init(T_LedCommonHandler *handler);

#ifdef __cplusplus
}
#endif

#endif // BUTTON_H

/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
