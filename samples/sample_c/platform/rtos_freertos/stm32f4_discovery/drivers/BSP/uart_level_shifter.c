/**
  ******************************************************************************
  * @file    uart_level_shifter.c
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
#include "uart_level_shifter.h"
#include "stm32f4xx_hal.h"
/* Private typedef------------------------------------------------------------------------------*/
/* Private define------------------------------------------------------------------------------*/
#define UART_SWITCH_PORT        GPIOC
#define UART_SWITCH_PIN         GPIO_PIN_0

#define UART_CAM_DATA_PORT      GPIOC
#define UART_CAM_DATA_PIN       GPIO_PIN_1

#define UART_CAM_CTRL_PORT      GPIOC
#define UART_CAM_CTRL_PIN       GPIO_PIN_2

#define UART_LINUX_COMM_PORT    GPIOC
#define UART_LINUX_COMM_PIN     GPIO_PIN_5
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/

/** @group __UART_LEVEL_SHIFTER_CONFIGURATION_
    @{
*/#ifndef __UART_LEVEL_SHIFTER_CONFIGURATION_
#define __UART_LEVEL_SHIFTER_CONFIGURATION_
/** @brief      This functions use configuration hardware uart level shifter
    @param[in]  none
    @return     T_DjiReturnCode
*/
T_DjiReturnCode UartLS_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOE clock */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure pin as input floating */
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Pin = UART_SWITCH_PIN | UART_CAM_DATA_PIN | UART_CAM_CTRL_PIN | UART_LINUX_COMM_PIN;
    HAL_GPIO_Init(UART_SWITCH_PORT, &GPIO_InitStructure);

    // switch on UART
    HAL_GPIO_WritePin(UART_SWITCH_PORT, UART_SWITCH_PIN, GPIO_PIN_SET);
    // switch on UART
    HAL_GPIO_WritePin(UART_SWITCH_PORT, UART_CAM_DATA_PIN, GPIO_PIN_SET);
    // switch on UART
    HAL_GPIO_WritePin(UART_SWITCH_PORT, UART_CAM_CTRL_PIN, GPIO_PIN_SET);
    // Enable communication with onboard
    HAL_GPIO_WritePin(UART_LINUX_COMM_PORT, UART_LINUX_COMM_PIN, GPIO_PIN_SET);
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

#endif
/**
    @}
*/

/** @group __UART_LEVEL_SHIFTER_FUNCTIONS_
    @{
*/#ifndef __UART_LEVEL_SHIFTER_FUNCTIONS_
#define __UART_LEVEL_SHIFTER_FUNCTIONS_
/** @brief      This functions use get pin status level shifter
    @param[in]  none
    @return     uint32_t
*/
uint32_t UartLS_GetPinState(void)
{
    uint32_t status = 0x00000000;
    
    if(HAL_GPIO_ReadPin(UART_SWITCH_PORT, UART_SWITCH_PIN) == GPIO_PIN_SET) {
        status |= 0x01;
    }
    else {
        HAL_GPIO_WritePin(UART_SWITCH_PORT, UART_SWITCH_PIN, GPIO_PIN_SET);
    }
    
    if(HAL_GPIO_ReadPin(UART_SWITCH_PORT, UART_CAM_DATA_PIN)) {
        status |= 0x02;
    }
    else {
        HAL_GPIO_WritePin(UART_SWITCH_PORT, UART_CAM_DATA_PIN, GPIO_PIN_SET);
    }
    
    if(HAL_GPIO_ReadPin(UART_SWITCH_PORT, UART_CAM_CTRL_PIN)) {
        status |= 0x04;
    }
    else {
        HAL_GPIO_WritePin(UART_SWITCH_PORT, UART_CAM_CTRL_PIN, GPIO_PIN_SET);
    }
    
    if(HAL_GPIO_ReadPin(UART_LINUX_COMM_PORT, UART_LINUX_COMM_PIN)) {
        status |= 0x08;
    }
    else {
        HAL_GPIO_WritePin(UART_SWITCH_PORT, UART_LINUX_COMM_PIN, GPIO_PIN_SET);
    }
    
    return status;
}

#endif
/**
    @}
*/

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

