/*******************************************************************************
 * Copyright (c) 2020, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 *
 * @file    camera_interface.c
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    Aug-27-2020
 * @brief   This file contains lower function for interfacing with WIRIS camera
 *
 ******************************************************************************/
/* Private includes ----------------------------------------------------------*/
#include "led.h"
#include "stm32f4xx.h"

/* Private constants ---------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define LED_BLUE_PIN    GPIO_PIN_5
#define LED_BLUE_PORT   GPIOB

#define LED_GREEN_PIN   GPIO_PIN_6
#define LED_GREEN_PORT  GPIOB

#define LED_RED_PIN     GPIO_PIN_7
#define LED_RED_PORT    GPIOB
/* Private typedef -----------------------------------------------------------*/

/*!< Struct used in private */
typedef struct {
     /// bien luu thoi gian xu ly led
    uint32_t lastTime;
    
    /// thoi gian set de xu ly
    uint32_t timeSet;
    
    ///
    uint32_t timeMin;
    
    /// mau set
    T_LedColor color;
    
    /// mau set 1
    T_LedColor color1;
    
    /// mau set 2
    T_LedColor color2;
    
} T_GsdkLed;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static T_GsdkLed  s_ledInfo;
/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

static void GsdkLed_SetLedRed(void)
{
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, 0);
}

static void GsdkLed_ResetLedRed(void)
{
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, 1);
}

static void GsdkLed_SetLedGreen(void)
{
    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, 0);
}

static void GsdkLed_ResetLedGreen(void)
{
    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, 1);
}

static void GsdkLed_SetLedBlue(void)
{
    HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, 0);
}

static void GsdkLed_ResetLedBlue(void)
{
    HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, 1);
}

/**
 * @brief GsdkLed_EnableLedColor 
 * @details This structure type is used  enable led device 
 * @note 
 */
static void GsdkLed_EnableLedColor(T_LedColor color) 
{
    if(color == RED)
    {
        GsdkLed_SetLedRed();
        GsdkLed_ResetLedGreen();
        GsdkLed_ResetLedBlue();
    }
    else if(color == GREEN)
    {
        GsdkLed_ResetLedRed();
        GsdkLed_SetLedGreen();
        GsdkLed_ResetLedBlue();
    }
    else if(color == BLUE){
        GsdkLed_ResetLedRed();
        GsdkLed_ResetLedGreen();
        GsdkLed_SetLedBlue();
    }
    else if(color == YELLOW)
    {
        GsdkLed_SetLedRed();
        GsdkLed_SetLedGreen();
        GsdkLed_ResetLedBlue();
    }
    else if(color == PINK){
        GsdkLed_SetLedRed();
        GsdkLed_ResetLedGreen();
        GsdkLed_SetLedBlue();
    }
    else if(color == CYAN){
        GsdkLed_ResetLedRed();
        GsdkLed_SetLedGreen();
        GsdkLed_SetLedBlue();
    }
    else if(color == WHITE){
        GsdkLed_SetLedRed();
        GsdkLed_SetLedGreen();
        GsdkLed_SetLedBlue();
    }
    else if(color == BLACK){
        GsdkLed_ResetLedRed();
        GsdkLed_ResetLedGreen();
        GsdkLed_ResetLedBlue();
    }
}

/**
 * @brief GsdkLed_SetColor
 * @details This structure type is used to set color from the user 
 * @note 
 */
static void GsdkLed_SetColor(T_LedColor color)
{
    s_ledInfo.timeSet = 0;
    s_ledInfo.color = color;
}

/**
 * @brief
 * @details This structure type is used to
 * @note 
 */
static void GsdkLed_SetToggle(T_LedColor color, uint32_t time) 
{
    if(time < 100) {
        s_ledInfo.timeSet = 100;
    }
    
    s_ledInfo.color1 = color;
    s_ledInfo.color2 = BLACK;
}
/**
 * @brief GsdkLed_Process
 * @details This structure type is used to process led in a loop
 * @note 
 */
static void GsdkLed_Process(uint32_t timeNow)
{
    /*!< Check time for set solid led */ 
    if(s_ledInfo.timeSet == 0) {
        GsdkLed_EnableLedColor(s_ledInfo.color);
    }
    else {
        
        /*!< Check time now with the last time */
        if(timeNow % s_ledInfo.timeSet) {
            
            /*!< Enable led */
            GsdkLed_EnableLedColor(s_ledInfo.color);
            
            /*!< Switch color */
            if(s_ledInfo.color == s_ledInfo.color1) {
                s_ledInfo.color = s_ledInfo.color2;
            }
            else {
                s_ledInfo.color = s_ledInfo.color1;
            } 
        }
    }
}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
T_DjiReturnCode GsdkLed_Init(T_LedCommonHandler *handler)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*!< Configure clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*!< Initiate GPIO */
    GPIO_InitStruct.Pin = LED_RED_PIN | LED_BLUE_PIN | LED_GREEN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*!< Reset*/
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, 1);
    HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, 1);
    HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, 1);
  
    s_ledInfo.timeMin = 100; /*!< Unit: ms*/
    
    handler->LedProcess     = GsdkLed_Process;
    handler->SetColor       = GsdkLed_SetColor;
    handler->ToggleColor    = GsdkLed_SetToggle;
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
