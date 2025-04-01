/*******************************************************************************
 * Copyright (c) 2020, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 *
 * @file    Gremsy_MAVLink.h
 * @author  The GremsyCo
 * @version V1.0.0
 * @date    Sep-09-2020
 * @brief   This file contains expand of the mavlink library
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GREMSY_MAVLINK_H
#define __GREMSY_MAVLINK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
// we have separate helpers disabled to make it possible
// to select MAVLink 1.0 in the arduino GUI build
#define MAVLINK_SEPARATE_HELPERS

#define MAVLINK_SEND_UART_BYTES(chan, buf, len) comm_send_buffer(chan, buf, len)

// Only support for gimbal so 
#define MAVLINK_COMM_NUM_BUFFERS            4

/* Define ID interface*/
#define SYSTEM_CONTROL_SYSTEM_ID            1

#include "ardupilotmega/version.h"

// this allows us to make mavlink_message_t much smaller. It means we
// can't support the largest messages in common.xml, but we don't need
// those for APM1/APM2
#define MAVLINK_MAX_PAYLOAD_LEN     255

#include "mavlink_types.h"

/// MAVLink system definition
extern mavlink_system_t mavlink_system;

void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len);

static inline bool valid_channel(mavlink_channel_t chan)
{
    return chan < MAVLINK_COMM_NUM_BUFFERS;
}

/// Read a byte from the nominated MAVLink channel
///
/// @param chan		Channel to receive on
/// @returns		Byte read
///
uint8_t comm_receive_ch(mavlink_channel_t chan);

/// Check for available data on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_available(mavlink_channel_t chan);


/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_txspace(mavlink_channel_t chan);

/*
  return true if the MAVLink parser is idle, so there is no partly parsed
  MAVLink message being processed
 */
bool comm_is_idle(mavlink_channel_t chan);

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "ardupilotmega/mavlink.h"


// return a MAVLink variable type given a AP_Param type
//uint8_t mav_var_type(enum ap_var_type t);

// return CRC byte for a mavlink message ID
uint8_t mavlink_get_message_crc(uint8_t msgid);


typedef struct _gremsy_mavlink_handler_t
{
    
    uint8_t                 version;
    
    mavlink_channel_t       chan;
    
    /** @brief Function send mavlink data to serial
        @param2[in] buf Pointer to buffer
        @param2[in] len Bytes to send
        @return Size of data wrote actually.
    */
    int (*write)(const uint8_t *buf, uint16_t len);
    
    /** @brief Function send mavlink data to serial
        @param2[in] buf Pointer to buffer
        @param2[in] len Bytes to send
        @return Size of data wrote actually.
    */
    int (*read)(uint8_t *buf, uint16_t len);
        
} gremsy_mavlink_handler_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
void gremsy_mavlink_init(gremsy_mavlink_handler_t *handler);
/**
  * @brief System Clock Configuration
  * @retval None
  */

#ifdef __cplusplus
}
#endif

#endif /* __GREMSY_MAVLINK_H */

/*********** Portions COPYRIGHT 2020 Gremsy.Co., Ltd.*****END OF FILE****/
