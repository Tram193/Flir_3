/** 
  ******************************************************************************
  * @file    uart_level_shifter.h
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

#ifndef __BSP_UART_LEVEL_SHIFTER_H
#define __BSP_UART_LEVEL_SHIFTER_H

/* Includes ------------------------------------------------------------------*/
#include "dji_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported define ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/** @brief      This functions use configuration hardware uart level shifter
    @param[in]  none
    @return     T_DjiReturnCode
*/
T_DjiReturnCode UartLS_Init(void);

/** @brief      This functions use get pin status level shifter
    @param[in]  none
    @return     uint32_t
*/
uint32_t UartLS_GetPinState(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_LEVEL_SHIFTER_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

