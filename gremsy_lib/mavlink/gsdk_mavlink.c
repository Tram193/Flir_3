/*******************************************************************************
 * Copyright (c) 2020, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 *
 * @file    gremsy_camera.c
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    09-Sep-2020
 * @brief   This file contains the api for control camera (SONY, FLIR, WIRIS)
 *
 ******************************************************************************/
/* Private includes ----------------------------------------------------------*/
#include "mavlink/gsdk_mavlink.h"

/* Private typedef -----------------------------------------------------------*/
#ifdef MAVLINK_SEPARATE_HELPERS
/* Shut up warnings about mission declaration*/
#include "mavlink_helpers.h"
#endif

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
mavlink_system_t mavlink_system = {1, MAV_COMP_ID_ONBOARD_COMPUTER};

gremsy_mavlink_handler_t*    mavlink_comm_port[MAVLINK_COMM_NUM_BUFFERS];
/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
/**
  * @brief Send a buffer out a MAVLINK channel
  * @retval None
  */
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len)
{
    /*!< Check the channel send is valid */
    if(!valid_channel(chan)) {
        return;
    }
    
    const int written = mavlink_comm_port[chan]->write(buf, len);
    
    (void)written;
}

/** @brief Function init mavlink callback
    @param1[in] none
    @param2[in] None
    @return None
*/
void gremsy_mavlink_init(gremsy_mavlink_handler_t *handler)
{
    if(handler != NULL)
    {
        /*!< Check the channel send is valid */
        if(!valid_channel(handler->chan)) {
            return;
        }
        
        /*!< Set mavlink protocol version */
        if(handler->version == 1) {
            mavlink_set_proto_version(handler->chan, handler->version);
        }
        
        /*!< Get pointer handler*/
        mavlink_comm_port[handler->chan] = handler;
        
        /*!< Assign function send*/
        mavlink_comm_port[handler->chan]->write = handler->write;
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
