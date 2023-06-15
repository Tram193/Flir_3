/*******************************************************************************
 * Copyright (c) 2020, The GremsyCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The GremsyCo.
 *
 * @file    gremsy_wiris_camera.c
 * @author  The GremsyCo
 * @version V2.1.0
 * @date    06-05-2021
 * @brief   This file contains wiris command control protocol 
 * The protocol is currently compatible with two modules of WRIS 
 *          + WIRIS PRO (WWP)
 *          + WIRIS SECURITY 
 *
 ******************************************************************************/
/* Private includes ----------------------------------------------------------*/
#include "camera/gsdk_wws_camera.h"
#include "som/som_communication.h"

#include "dji_platform.h"
#include "utils/util_misc.h"
#include "dji_logger.h"
#include "osal.h"
#include "uart.h"

#include <ctype.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


/* Private define ------------------------------------------------------------*/

#define WWS_DEBUG           1
#define WWS_DEBUG_RESPONSE  0

#if WWS_DEBUG
# define DebugMsg(fmt, args ...) do {USER_LOG_DEBUG("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);
# define DebugInfo(fmt, args ...) do {USER_LOG_INFO("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);
# define DebugWarning(fmt, args ...) do {USER_LOG_WARN("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);
#else
# define DebugMsg(fmt, args ...)
# define DebugInfo(fmt, args ...)
# define DebugWarning(fmt, args ...)
#endif

# define DebugError(fmt, args ...) do {USER_LOG_ERROR("[%s]:[%d]: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0);

/**
 * @brief state of packet
 *
 * Contains state related to packet
 */
typedef enum _E_WirisParseState{
    WIRIS_PARSE_STATE_IDLE = 0,
    WIRIS_PARSE_STATE_HEADER,
    WIRIS_PARSE_STATE_LEN1,
    WIRIS_PARSE_STATE_LEN2,
    WIRIS_PARSE_STATE_CMD,
    WIRIS_PARSE_STATE_CRC,
    WIRIS_PARSE_STATE_FOOTER,
} E_WirisParseState;

/* Private macro -------------------------------------------------------------*/
#define WIRIS_TIMEOUT_SEND_GET_ACK              1000
#define WIRIS_CRITICAL_TIMEOUT_SEND_GET_ACK     500
#define WIRIS_SEND_ACK_ENABLE                   STD_ON

#define WIRIS_SLOG                          STD_OFF

#define WIRIS_PROT_HEADER_BYTES             0x03

#define WIRIS_PROT_HEADER_BYTE              0x02
#define WIRIS_PROT_FOOTER_BYTE              0x03

#define WIRIS_PROT_SYSTEM_LEN               6 // (header + len1 + len2 + SUM + FOOTER)
#define WIRIS_PROT_MAX_DATA_LEN             200
#define WIRIS_PROT_MAX_CMD_LEN              20
#define WIRIS_PROT_MAX_FRAME_LEN            WIRIS_PROT_SYSTEM_LEN + WIRIS_PROT_MAX_DATA_LEN

#define NMEA_GPS_CMD_MAX_LEN                30

/* Private typedef -----------------------------------------------------------*/
/*!< The struct is used for WIRIS protocol data struct parse*/
typedef struct {
    uint8_t parseState;
    uint8_t msgLen;
    uint8_t parseBuffer[WIRIS_PROT_MAX_FRAME_LEN];
    uint8_t parseIndex;
}T_WirisProtParse; 
/**
 * @brief Structure wiris protocol
 * @details This structure type is used to define wiris command control protocol
 */
typedef struct _T_WirisProt {
    
    /*!< Receive*/
    T_WirisProtParse            protParse;
    ReceiveCallbackFunc         receiveCallback;
    
    /*!< Send*/
    uint16_t                    sendSeqNum;
    uint8_t                     sendFrameBuf[WIRIS_PROT_MAX_FRAME_LEN];
    uint8_t                     frameDataLen;
    SendCallbackFunc            sendCallback;
    
    uint32_t lastTimeGPS;
    bool     isImmediated;
    
    /*!< ACK*/
    uint8_t isWaitAck;
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN];
    uint8_t ackDataLen;
    
#if SYSTEM_ARCH_RTOS
    T_DjiMutexHandle           mutexGetData;
    T_DjiMutexHandle           mutexSend;
    T_DjiMutexHandle           mutexSendGetAck;
    T_DjiSemaHandle            semaphorewaitAck;
#endif

    uint8_t    errorCount;
    /*!< Callback error */
    void (*WirisCallbackError)(uint8_t errorCount);
} T_WirisProt; 

/*!< Define Hardware Interface PixyWS Impossible IA UART_NUM_4*/
#define PSDK_COMM_WITH_CAMERA_UART_NUM          UART_NUM_5
#define PSDK_COMM_WITH_CAMERA_UART_BAUD         115200
#define PSDK_COMM_WITH_CAMERA_BUFFER_SIZE       (512)

/*!< Define Wiris Task Recv Info*/
#define WIRIS_CAMERA_TASK_FREQ                  (1000)
#define WIRIS_CAMERA_RECV_TASK_STACK_SIZE       (1024)
#define WIRIS_CAMERA_CMD_TASK_STACK_SIZE        (2048 * 1)
#define WIRIS_CAMERA_TIME_FOR_WAIT              (1000) /*!< Unit: Ms*/


/*!< List of visible camera zooms */
typedef struct {
    uint8_t     index;
    
    float       radio;
} T_ListZooms;

/*!< Get list of all available zooms of visible camera. Each line has index number and zoom raOo value.*/
const T_ListZooms s_listVisible[] = {
    {0, 1.0},
    {1, 1.2},
    {2, 1.5},
    {3, 2.0},
    {4, 3.0},
    {5, 4.0},
    {6, 5.0},
    {7, 6.0},
    {8, 8.0},
    {9, 10.0},
    {10, 12.0},
    {11, 16.0},
    {12, 20.0},
    {13, 25.0},
    {14, 30.0},
};

/*!< Get list of all available zooms of visible camera. Each line has index number and zoom raOo value.*/
const T_ListZooms s_listThermal[] = {
    {0, 1.0},
    {1, 1.2},
    {2, 1.6},
    {3, 2.0},
    {4, 3.0},
    {5, 4.0},
    {6, 5.0},
    {7, 6.0},
    {8, 8.0},
    {9, 10.0},
    {10, 12.0},
    {11, 16.0},
};
/*!< Process state for camera */
typedef enum {
    CAMERA_RUNNING_STATE_IDLE = 0,
    
    CAMERA_RUNNING_STATE_CHECK_CONNECTION,
    
    CAMERA_RUNNING_STATE_RESET_SOM,
    
    CAMERA_RUNNING_STATE_ACTIVE_KEY,
    
    CAMERA_RUNNING_QUERY_CAMERA_SETTING,
    
    CAMERA_RUNNING_STATE_PROCESS_EVENT,
    
    CAMERA_RUNNING_STATE_PROCESS_GPS,
    
    CAMERA_RUNNING_STATE_ERROR,
} E_CameraRunningState;

/*!< Camera running status */
E_CameraRunningState   s_cameraRunningState = CAMERA_RUNNING_STATE_IDLE;

/**
 * @brief
 * @details This structure type is used to
 * @note 
 */
typedef enum  {
    REQUEST_CAM_VIDEO_STATUS                = BIT0,
    REQUEST_CAM_CAPTURE_STATUS              = BIT1,
    REQUEST_CAM_ZOOM_VALUE                  = BIT2,
    REQUEST_CAM_PALETTE_VALUE               = BIT3,
    REQUEST_CAM_TIME_STATBI_VALUE           = BIT4,
    REQUEST_CAM_HOT_REJECTION_VALUE         = BIT5,
    REQUEST_CAM_COOL_REJECTION_VALUE        = BIT6,
    REQUEST_CAM_ALARM_MODE_VALUE            = BIT7,
    REQUEST_CAM_ALARM_COLOR_VALUE           = BIT8,
    REQUEST_CAM_THRESHOLD_VALUE             = BIT9,
    REQUEST_CAM_MEMORY_STATUS               = BIT10,
    REQUEST_CAM_MEMORY_SIZE                 = BIT11,
    REQUEST_CAM_MEMORY_FREE                 = BIT12,
} E_RequestCamStatus;

static uint32_t         s_requestCamStatus;

/*!<   buffer used to receive data from UART interface */
static uint8_t      s_uartRecBuf[PSDK_COMM_WITH_CAMERA_BUFFER_SIZE];

/*!< Task handler ID for processing wiris protocol*/
static T_DjiTaskHandle             s_wirisRecvThread;

/*!< Task handler ID for processing user application*/
static T_DjiTaskHandle             s_wirisCmdThread;

/*!< Event update for callback request*/
static EventGroupHandle_t          s_eventCamera;

/* Private variables ---------------------------------------------------------*/

static T_WirisProt  *s_pWirisProt   = NULL;
/*!< Wiris object */
static T_WIRISCamera                 s_WIRISCamera;
T_WIRISCamera * const WIRISCamera = &s_WIRISCamera;

static uint32_t                     timeResponding = 0;
/*!< Protocol variable for wiris camera*/
/* Private function prototypes -----------------------------------------------*/

/*!<====================== PROTO_FUNCTIONS ================================== */

/** @addtogroup 
  * @{ PROTO_FUNCTIONS
  */

/**
  * @brief Function is used to initialize the wiris protocol
  * @retval T_DjiReturnCode
  */
T_DjiReturnCode WirisCameraProto_Init(void);

/**
  * @brief Function is used to initialize the wiris protocol
  * @retval T_DjiReturnCode
  */
T_DjiReturnCode WirisCameraProto_DeInit(void);

/**
  * @brief Function is used to register the function send 
  * @retval T_DjiReturnCode
  */
T_DjiReturnCode WirisCameraProto_RegSendDataFunc(SendCallbackFunc callbackFunc);

/**
  * @brief Function is used to register the function send 
  * @retval T_DjiReturnCode
  */
T_DjiReturnCode WirisCameraProto_RegRecvDataFunc(ReceiveCallbackFunc callbackFunc);

/**
  * @brief Function is used to register the command control list
  * @retval T_DjiReturnCode
  */
T_DjiReturnCode WirisProt_ProcessReceiveData(const uint8_t *pData, uint16_t len);

/**
  * @} // PROTO_FUNCTIONS
  */
/*!<====================== MATH_FUNCTIONS =================================== */
/** @addtogroup 
  * @{ MATH_FUNCTIONS
  */
/*
 * get_opt
 */
static char *GsdkMath_getOpt(char *cp, char **pNext, char bc)
{
    char *sp, lfn = 0, sep_ch = bc;

    /*!< Check the input pointer is valid.*/
    if (cp == NULL)
    { /* Skip NULL pointers*/
        *pNext = cp;
        return cp;
    }

    /*!< Check the character in the memory is equal to end or " then ignoring it.*/
    for ( ; *cp == bc || *cp == '\"'; cp++)
    {
        /* Skip blanks and starting  " */
        if (*cp == '\"')
        {
            sep_ch = '\"';
            lfn = 1;
        }
        *cp = 0;
    }

    for (sp = cp; *sp != '\r' && *sp != '\n'; sp++)
    {
        if (lfn && *sp == '\"')	break;
        if (!lfn && *sp == bc)	break;
    }

    for ( ; *sp == sep_ch || *sp == '\r' || *sp == '\n'; sp++)
    {
        *sp = 0;
        if (lfn && *sp == sep_ch) { sp++; break; }
    }
    *pNext = (*sp) ? sp : NULL;/* Next arg */

    return cp;
}


/** @brief The checksum is mandatory, and the last field in a sentence, It is the 8-bit XOR
    of all characters in the sentence, excluding the "$", "I", or "*" characters; but including all "," and "^". 
    @return none
*/
static uint8_t GsdkMath_nmeaChecksum(const char *sentence)
{
    // Support senteces with or without the starting dollar sign.
    if (*sentence == '$')
        sentence++;

    uint8_t checksum = 0x00;

    // The optional checksum is an XOR of all bytes between "$" and "*".
    while (*sentence && *sentence != '*')
        checksum ^= *sentence++;

    return checksum;
}

/**
 * @brief Function convert Degrees to Decimal Degrees
 * @details
 * @note 
 */
void GsdkMath_deg2dms(float dd, int *pDeg, int *pMin, int *pSec)
{
    int d = 0, m = 0, s = 0;
    float minwork=0.0, secwork=0.0; // intermediate variables
    
    /*!< Float to int conversion leaves only the pre-decimal numbers, ala the deg */
    d = dd;
    /*!< Remove degrees leaves only minutes, and secs */
    dd = dd - d;
    
    /*!< Convert degrees to Minutes */
    minwork = dd*60;
    
   /*!<  float to int conversion*/
    m = minwork;
    
    dd = dd*100;
    dd = dd - minwork; // leaves only secs
    dd = dd/100;
    secwork = dd*60;
    
    /*!<  float to int conversion, if needed.*/
    s = secwork; 
    // at this point, d, m, and s are assigned in the fuction, by need to be moved into main program.
    // the following statements reassign the variables in the main program
    *pDeg = d;
    *pMin = m;
    *pSec = s;
}

/* return true for positive, false for negative,
   and advance `*s` to next position */
static bool GsdkMath_parseSign(const char **const s)
{
    switch (**s) {
    case '-': ++*s; return false;
    case '+': ++*s; return true;
    default: return true;
    }
}

/* return decimal value of digits,
   advancing `*s` to the next character,
   and storing the number of digits read into *count */
static double GsdkMath_parseDigits(const char **const s, int *const count)
{
    double value = 0.0;
    int c = 0;
    while (isdigit(**s)) {
        value = value * 10.0 + (*(*s)++ - '0');
        ++c;
    }
    if (count)
        *count = c;
    return value;
}

double GsdkMath_extendedAtof(const char *s)
{
    /*skip white space*/
    while (isspace(*s))
        ++s;

    const bool valuesign = GsdkMath_parseSign(&s); /* sign of the number */
    double value = GsdkMath_parseDigits(&s, NULL);

    if (*s == '.') {
        int d;                  /* number of digits in fraction */
        ++s;
        double fraction = GsdkMath_parseDigits(&s, &d);
        while (d--)
            fraction /= 10.0;
        value += fraction;
    }

    if (!valuesign)
        value = -value;

    if (tolower(*s++) != 'e')
        return value;

    /* else, we have an exponent; parse its sign and value */
    const double exponentsign = GsdkMath_parseSign(&s) ? 10. : .1;
    int exponent = GsdkMath_parseDigits(&s, NULL);
    while (exponent--)
        value *= exponentsign;

    return value;
}

/** @brief Convert string to integer. 
  * Library function atoi can be used to converted string to an integer
  * @note The terminate used to dectect \0 && \r. Only used for wiris  
  * @return interger
  */
int64_t GsdkMath_StringToInteger(char str[])
{
    int64_t n = 0;
    uint8_t offset = 0, c = 0;
    int8_t sign = 0;
    
    if(str[0] == '-') { // Handle negative integers 
        sign = -1;
    } 
    
    if(sign == -1) { /*!< Set starting position to convert*/
        offset = 1;
    }
    else {
        offset = 0;
    }
        
    for(c = offset; str[c+1] != '\0'; c++) {
        if(str[c] == 0x0D || str[c] == 0x0A)
            break;
        
        n = n * 10 + str[c] - '0';
    }

    if (sign == -1) {
        n = -n;
    }
    
    return n;
}


// Reverses a string 'str' of length 'len' 
void GsdkMath_Reverse(char* str, uint8_t len) 
{ 
    int i = 0, j = len - 1, temp;
    
    while (i < j) { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; 
        j--; 
    } 
} 


/** @brief Converts a interger number to a string
    @param[in] n the value want to convert
    @param[in] res return the char
    @param[in] d If d is more than the number of digits in x,  
    @return none
*/

static uint8_t GsdkMath_IntToString(int16_t x, char str[], int16_t d)
{
    int16_t xx = abs(x);
    int i = 0; 

    while (xx) { 
        str[i++] = (xx % 10) + '0'; 
        xx = xx / 10; 
    } 
  
    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d) 
        str[i++] = '0'; 
  
    GsdkMath_Reverse(str, i); 
    str[i] = '\0';  
    return i;
}

/** @brief Converts a floating-point/double number to a string
    @param[in] n the value want to convert
    @param[in] res return the char
    @return none
*/
static void GsdkMath_FloatToString(double n, char *res,uint8_t beforepoint, uint8_t afterpoint)
{
    // Extract integer part
    int16_t ipart = (int16_t)n;

    uint8_t reserve_0 = 0;
    
    if(ipart < 0) {
        res[0] = '-';
        
        reserve_0 = 1;
    }
    
    // Extract floating part
    double fpart = n - (double)ipart;

    // Convert integer part to string
    int i = GsdkMath_IntToString(ipart, &res[reserve_0], beforepoint);

      // check for display option after point
    if (afterpoint != 0) {
        res[reserve_0 + i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10.0f, afterpoint);

        // Convert to string
        GsdkMath_IntToString((int16_t)fpart, &res[reserve_0] + i + 1, afterpoint);
    }
}

/**
  * @} // MATH_FUNCTIONS
  */

/** @brief Function packet command
    @param1[out] packet will return the bufer need to send 
    @param2[in] command want to send to wiris camera
    @return 
*/
static T_DjiReturnCode WirisProt_DataToFrame(T_WirisProt *pWirisProt, const char* command, const char *sub_command)
{
    if(pWirisProt == NULL) {
        DebugError("Invalid Argument ");
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    /*!< Add Start of Frame */
    pWirisProt->sendFrameBuf[0] = WIRIS_PROT_HEADER_BYTE;

    uint8_t idx = 0;
    uint8_t sum = 0;

    if(command != NULL)
    {
        while(command[idx] != '\0')
        {
            pWirisProt->sendFrameBuf[WIRIS_PROT_HEADER_BYTES + idx] = (uint8_t)command[idx];
            sum += (uint8_t)command[idx];

            idx = idx + 1;
        }
    }
    else 
    {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    // Add sub-command
    if(sub_command != NULL && strlen(sub_command) > 0)
    {
        pWirisProt->sendFrameBuf[WIRIS_PROT_HEADER_BYTES + idx] = 0x20; // space
        idx = idx + 1;

        uint8_t idx_sub_cmd = 0;

        while(sub_command[idx_sub_cmd] != '\0')
        {
            pWirisProt->sendFrameBuf[WIRIS_PROT_HEADER_BYTES + idx] = (uint8_t)sub_command[idx_sub_cmd];

            sum += (uint8_t)sub_command[idx_sub_cmd];

            idx_sub_cmd = idx_sub_cmd + 1;
            idx = idx + 1; // Continue input to buffer
        }
        
        // Calcualte the check sum
        pWirisProt->sendFrameBuf[4 + idx] = (sum + 0x0A + 0x20) & 0xFF;
    }
    else
    {
         pWirisProt->sendFrameBuf[4 + idx] = (sum + 0x0A) & 0xFF;
    }

    pWirisProt->sendFrameBuf[WIRIS_PROT_HEADER_BYTES + idx] = 0x0A; 

    // Get the length
    pWirisProt->sendFrameBuf[1] = (idx + 1) & 0x00FF ;
    pWirisProt->sendFrameBuf[2] = (idx + 1) & 0xFF00;
    
    // Calcualte the check sum
    // packet->bytes[4 + idx] = (sum + 0x0A) & 0xFF;
    pWirisProt->sendFrameBuf[5 + idx] = WIRIS_PROT_FOOTER_BYTE;

    // Total length = 3 bytes (header + 2bytes of length) + length of data + sum byte + footer
    pWirisProt->frameDataLen =  (WIRIS_PROT_HEADER_BYTES + (idx + 1) + 1 + 1);
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/** @brief WIRIS data send function according to protocol information and data to be sent
  * @param1 pWirisProt Pointer to Wiris Protocol handle
  * @param2 command main command
  * @param2 sub_command sub command
  * @return None
  */
T_DjiReturnCode WirisProt_Send(T_WirisProt* pWirisProt, const char* command, const char *sub_command)
{
    T_DjiReturnCode djiReturnCode;
    
#if SYSTEM_ARCH_RTOS
    if(Osal_MutexLock(pWirisProt->mutexSend) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("lock mutexSend error: 0x%08llX.", djiReturnCode);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
#endif 
    djiReturnCode = WirisProt_DataToFrame(pWirisProt, command, sub_command);
    if(djiReturnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
          DebugError("Packed Wiris prototol Faileed: 0x%08llX.", djiReturnCode);
    } 
    
    /*!< Send data frame*/
    if(pWirisProt->sendCallback != NULL) {
        pWirisProt->sendCallback(pWirisProt->sendFrameBuf, pWirisProt->frameDataLen);
    }
    else
    {
        DebugError("Send Data Frame Failed: 0x%08llX.",djiReturnCode);
        djiReturnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    
#if SYSTEM_ARCH_RTOS
    if(Osal_MutexUnlock(pWirisProt->mutexSend) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("unlock mutexSend error: 0x%08llX.", djiReturnCode);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
#endif
    
    return djiReturnCode;
}

/** @brief Wiris protocol send request data and get ack function
  * @note If system is based on OS, command handle task/thread can not call this function otherwise system may hang
  * @note If system is not based on OS (bare metal system), command calling this function to send will fail when ack of
  * previous command have not come
  * @param1 pWirisProt Pointer to WIRIS protocol handle
  * @param command Pointer to a main command to be sent
  * @param sub_command Pointer to a sub command to be sent
  * @param ackDataLen Pointer to ack data len to be received
  * @param ackData Pointer to ack data to be received.
  * @param ackDataLen Pointer to ack data len to be received
  * @param timeout Max time used to wait PSDK ack, unit: millisecond.
  *
  * @return None
  */
static T_DjiReturnCode WirisProt_SendGetAck(T_WirisProt *pWirisProt, const char* command, const char *sub_command, uint8_t *ackData, uint8_t* ackDataLen, uint32_t timeout)
{
    T_DjiReturnCode djiReturnCode = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    
#if SYSTEM_ARCH_RTOS
    if(Osal_MutexLock(pWirisProt->mutexSendGetAck) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("lock mutexSendGetAck error: 0x%08llX.", djiReturnCode);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
#else
    if (pWirisProt->isWaitAck != 0)
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;

    pWirisProt->isReceivedAck = 0;
#endif 
    
    /*!< Enable flag to check need to get ack data from the camera*/
    pWirisProt->isWaitAck = 1;
        
    /*!< Reset data and buffer*/
    pWirisProt->ackDataLen  = 0;
    memset(pWirisProt->ackData, 0x00, WIRIS_PROT_MAX_DATA_LEN);
    
    #if (WWS_DEBUG_RESPONSE == 1)
    DebugWarning("Send CMD: %s\n", command);
    #endif
    /*!< Call Function send data frame*/
    djiReturnCode = WirisProt_Send(pWirisProt, command, sub_command);
    if(djiReturnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("Wiris Prot Send Get ACK Failed.");
    }
    
#if SYSTEM_ARCH_RTOS
    /*!< Waiting for semaphore release. It will be release from the handle frame everytime recieved a feedback*/
    djiReturnCode = Osal_SemaphoreTimedWait(pWirisProt->semaphorewaitAck, timeout);
    if(djiReturnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        /*!< Copy data to ack buffer*/
        memcpy(ackData, pWirisProt->ackData, pWirisProt->ackDataLen);
       *ackDataLen = pWirisProt->ackDataLen;
        
//         for(int i = 0; i < pWirisProt->ackDataLen;  i++){
//            DebugWarning("[ACK] - 0x%02X - %c ", pWirisProt->ackData[i], pWirisProt->ackData[i]);
//         }
        
        /*!< Reset error */
        pWirisProt->errorCount = 0;
    }
    else {
        pWirisProt->ackDataLen  = 0;
        memset(pWirisProt->ackData, 0x00, WIRIS_PROT_MAX_DATA_LEN);
        
        /*!< Increase error */
        pWirisProt->errorCount++;
        
        if(pWirisProt->WirisCallbackError != NULL) {
            /*!< Callback error to notify error*/
            pWirisProt->WirisCallbackError(pWirisProt->errorCount);
        }
        
        /*!< Check count errro */
        if(pWirisProt->errorCount > 10) {
            pWirisProt->errorCount = 0;
            s_cameraRunningState = CAMERA_RUNNING_STATE_ERROR;
        }
        
        DebugError("[%s][%d]Wait semaphore error: 0x%08llX.", __FUNCTION__, __LINE__, djiReturnCode);
    }
    
    /*!< Clear Flag after got the ackData*/
    pWirisProt->isWaitAck = 0;
    
    if(Osal_MutexUnlock(pWirisProt->mutexSendGetAck) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("unlock mutexSendGetAck error: 0x%08llX.", djiReturnCode);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
#else
    /*!< Process in bare metal*/
#endif
    return djiReturnCode;
}

/** @brief WirisProt_ParseData
  *
  * @param1 proParse Pointer to the protocol handle
  * @param2 byteData data received from the camera
  * @return None
  */
static int WirisProt_ParseData(T_WirisProtParse *proParse, uint8_t byteData)
{
    #if (WWS_DEBUG_RESPONSE == 1)
    DebugInfo("[%d] - 0x%02X - %c", proParse->parseState, byteData, byteData);
    #endif
    /*!< Parse data*/
    if(proParse->parseState == WIRIS_PARSE_STATE_IDLE)
    {
        proParse->parseIndex   = 0;
        memset(proParse->parseBuffer, 0x00, WIRIS_PROT_MAX_DATA_LEN);
        
        proParse->parseState = WIRIS_PARSE_STATE_HEADER;
    }
    else if(proParse->parseState == WIRIS_PARSE_STATE_HEADER)
    {
        if(byteData == WIRIS_PROT_HEADER_BYTE)
        {
            proParse->parseIndex   = 0;
            proParse->msgLen       = 0;
            memset(proParse->parseBuffer, 0x00, WIRIS_PROT_MAX_DATA_LEN);

            s_pWirisProt->ackDataLen  = 0;
            
#if (WIRIS_SLOG == STD_ON)
    DebugInfo("Got SOF: 0x%04X - isWaitAck :%d", byteData, s_pWirisProt->isWaitAck);
#endif
            // Switch to next state
            proParse->parseState = WIRIS_PARSE_STATE_LEN1;
        }
    }
    else if(proParse->parseState == WIRIS_PARSE_STATE_LEN1)
    {
        // Get message length
        proParse->msgLen = byteData;
                
        // Switch to next
        proParse->parseState = WIRIS_PARSE_STATE_LEN2;
    }
    else if(proParse->parseState == WIRIS_PARSE_STATE_LEN2)
    {
        proParse->msgLen += 256*byteData;
        
        if(proParse->msgLen > WIRIS_PROT_MAX_DATA_LEN || proParse->msgLen <= 0)
        {
            // Reset all
            proParse->msgLen     = 0;
            
            // Error process
            proParse->parseState = WIRIS_PARSE_STATE_IDLE;
            
            return -WIRIS_PARSE_STATE_LEN2;
        }
        else
        {
            proParse->parseIndex = 0;
                        
            memset(proParse->parseBuffer, 0x00, proParse->msgLen);
            // Switch to next
            proParse->parseState = WIRIS_PARSE_STATE_CMD;
        }
    }
    else if(proParse->parseState == WIRIS_PARSE_STATE_CMD)
    {
        proParse->parseBuffer[proParse->parseIndex++] = byteData;
        
        if(proParse->parseIndex >= proParse->msgLen)
        {
            proParse->parseState = WIRIS_PARSE_STATE_CRC;
        }
    }
    else if(proParse->parseState == WIRIS_PARSE_STATE_CRC)
    {
        uint8_t sum = 0;
        
        /*!< Calculate the control sum*/
        for (uint8_t i = 0; i < proParse->msgLen; i++) {
            sum += proParse->parseBuffer[i];
        }
        
        // Calculate the control summ
        uint8_t control_sum  = sum & 0xFF;

        // Check the control sum with the received byte
        if(control_sum == byteData) {
            proParse->parseState = WIRIS_PARSE_STATE_FOOTER;
        }
        else {
            // Error process
            proParse->parseState = WIRIS_PARSE_STATE_IDLE;
            
            return -WIRIS_PARSE_STATE_CRC;
        }
    }
    else if(proParse->parseState == WIRIS_PARSE_STATE_FOOTER) {
        /*!< Check Footer byte*/
        if(byteData == WIRIS_PROT_FOOTER_BYTE) {
            
#if (WIRIS_SLOG == STD_ON)
    DebugInfo("Got EOF: 0x%02X - isWaitAck :%d - len: %d", byteData, s_pWirisProt->isWaitAck, s_pWirisProt->protParse.msgLen);
#endif
//            if(s_pWirisProt->isWaitAck) {
                
                /*!< Copy data command to ack buffer*/
                memcpy(s_pWirisProt->ackData, s_pWirisProt->protParse.parseBuffer, s_pWirisProt->protParse.msgLen);
                s_pWirisProt->ackDataLen  = s_pWirisProt->protParse.msgLen;
                
#if (WIRIS_SLOG == STD_ON)
                for(int i = 0; i < s_pWirisProt->protParse.msgLen;  i++){
                    DebugWarning("[Read] - 0x%02X - %c ", s_pWirisProt->ackData[i],s_pWirisProt->ackData[i]);
                }
#endif
                /*!< Release semaphore*/
                if(Osal_SemaphorePost(s_pWirisProt->semaphorewaitAck) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError("ERROR Osal_SemaphorePost");
                    proParse->parseState = WIRIS_PARSE_STATE_IDLE;
                }
//            }
            
            // Parse complete
            proParse->parseState = WIRIS_PARSE_STATE_HEADER;
       
            return 1;
        }
        else  {
             proParse->parseState = WIRIS_PARSE_STATE_IDLE;
            
             return -WIRIS_PARSE_STATE_FOOTER;
        }
    }

    return 1;
}
    
/** @brief Process received data from camera communication interface
  * @param1 T_WirisProt *pWirisProt protocol handler
  * @param2 pData Pointer to receive data
  * @param3 len Recived data length
  * @return T_DjiReturnCode
  */
T_DjiReturnCode WirisProt_ProcessReceiveData(const uint8_t *pData, uint16_t len)
{
    int i = 0;
    int parseRes = 0;
    
    for(i = 0; i < len; i++) {
        parseRes = WirisProt_ParseData(&s_pWirisProt->protParse, pData[i]);
        
        /*!< Parse frame complete*/
        if(parseRes > 0) {
            
            /*!< Check error*/
        }
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* Exported function ---------------------------------------------------------*/

/**
  * @brief Function is used to initialize the wiris protocol
  * @retval T_DjiReturnCode
  */
T_DjiReturnCode WirisCameraProto_Init(void)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    /*!< Allocate memory */
    s_pWirisProt =  osalHandler->Malloc(sizeof(T_WirisProt));
    
    if(s_pWirisProt == NULL) {
        DebugError("Wiris camera protocol init failed");
        return DJI_ERROR_SYSTEM_MODULE_CODE_MEMORY_ALLOC_FAILED;
    }

    memset(s_pWirisProt, 0x00, sizeof(T_WirisProt));

    /*!< Send */
    s_pWirisProt->sendSeqNum = 0;
    memset(s_pWirisProt->sendFrameBuf, 0x00, sizeof(s_pWirisProt->sendFrameBuf));
    s_pWirisProt->sendCallback = NULL;
    
    /*!< Receive */
    s_pWirisProt->protParse.parseIndex = 0;
    memset(s_pWirisProt->protParse.parseBuffer, 0x00, sizeof(s_pWirisProt->protParse.parseBuffer));
    s_pWirisProt->receiveCallback = NULL;

#if SYSTEM_ARCH_RTOS
    /*!< Create mutex for managing the get data */
    if(osalHandler->MutexCreate(&s_pWirisProt->mutexGetData) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("mutexGetData create error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    
    /*!< Create mutex for managing the access function list when sending*/
    if(osalHandler->MutexCreate(&s_pWirisProt->mutexSend) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("mutexSend create error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    
    if(osalHandler->MutexCreate(&s_pWirisProt->mutexSendGetAck) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("mutexSendGetAck create error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    
    /*!< Using semaphore for synchronization the event when pasrsing the command ack received from camera*/
    if(osalHandler->SemaphoreCreate(1, &s_pWirisProt->semaphorewaitAck) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("semaphorewaitAck create error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    
    /*!< Reset semaphore. This will be set when received a ack from the camera*/
    T_DjiReturnCode djiReturnCode = osalHandler->SemaphoreTimedWait(s_pWirisProt->semaphorewaitAck, 0);
    if(djiReturnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("Take semaphore failed 0x%08llX.", djiReturnCode);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
#endif

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
  * @brief Function is used to deinitialize the wiris protocol
  * @retval T_DjiReturnCode
  */
T_DjiReturnCode WirisCameraProto_DeInit(void)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
#if SYSTEM_ARCH_RTOS
    if (osalHandler->MutexDestroy(s_pWirisProt->mutexGetData) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("mutex destroy error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    
    if (osalHandler->MutexDestroy(s_pWirisProt->mutexSend) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("mutex destroy error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    
    
    if (osalHandler->MutexDestroy(s_pWirisProt->mutexSendGetAck) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("mutex destroy error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    
    /*!< Using semaphore for synchronization the event when pasrsing the command ack received from camera*/
    if(osalHandler->SemaphoreDestroy(s_pWirisProt->semaphorewaitAck) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("Semaphore wait ACK destroy error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
#endif
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
  * @brief Function is used to register the function send 
  * @retval T_DjiReturnCode
  */
T_DjiReturnCode WirisCameraProto_RegSendDataFunc(SendCallbackFunc callbackFunc)
{
    if(callbackFunc == NULL || s_pWirisProt == NULL){
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    s_pWirisProt->sendCallback = callbackFunc;
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
  * @brief Function is used to register the function send 
  * @retval T_DjiReturnCode
  */
T_DjiReturnCode WirisCameraProto_RegRecvDataFunc(ReceiveCallbackFunc callbackFunc)
{
    if(callbackFunc == NULL || s_pWirisProt == NULL){
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    s_pWirisProt->receiveCallback = callbackFunc;
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
/*!<====================== BAISC COMMANDS =================================== */
/** @addtogroup BASIC_COMMANDS
  * @{
  */

/**
 * @brief Check Connection
 * @details Command used for checking the connection
 * @note Answer OK
 */
T_DjiReturnCode WirisCamera_isWirisConnected(void)
{
    uint8_t ackData[3] = {0};
    uint8_t ackDataLen = 0;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
//    PsdkOsal_GetTimeMs(&timeResponding);
    osalHandler->GetTimeMs(&timeResponding);
    if(WirisProt_SendGetAck(s_pWirisProt, "HIWS", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    /*!< Check len is valid*/
    if(ackDataLen > 0) {
        if( ackData[0] == 0x4F && 
            ackData[1] == 0x4B && 
            ackData[2] == 0x0A) {
                
            uint32_t timeNow = 0;
                
            osalHandler->GetTimeMs(&timeNow);
                
            DebugInfo("Got camera %d", timeNow - timeResponding);
                
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
    }

    for(int i = 0; i < ackDataLen;  i++){
    DebugWarning("[%d] - 0x%02X - %c ",__LINE__, ackData[i], ackData[i]);
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief Check Connection
 * @details Command used for checking the connection
 * @note Answer OK
 */
T_DjiReturnCode WirisCamera_ActivateCamera(void)
{
    uint8_t ackData[3] = {0};
    uint8_t ackDataLen;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "AGGI TRUE", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    /*!< Check len is valid*/
    if(ackDataLen > 0) {
        if( ackData[0] == 0x4F && 
            ackData[1] == 0x4B && 
            ackData[2] == 0x0A) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}


/**
 * @brief Get Wiris Serial Number
 * @details Returns WIRIS device serail number
 */
T_DjiReturnCode WirisCamera_GetSerialNumber(char *serial_number)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    /*!< Send command and wait the ack from the camera*/
    if(WirisProt_SendGetAck(s_pWirisProt, "GSRN", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check length feedback from the camera is valid*/
    if(ackDataLen > 0)  {
        /*!< Copy data to buffer*/
        memcpy(serial_number, ackData, ackDataLen);
        
#if (WIRIS_SLOG == STD_ON)
    DebugInfo("SN: %s", serial_number);
#endif 
        
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    for(int i = 0; i < ackDataLen;  i++){
        DebugWarning("[%d] - 0x%02X - %c ",__LINE__, ackData[i], ackData[i]);
    }


    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief Get Wiris Article Number
 * @details Returns WIRIS device Article number
 */
T_DjiReturnCode WirisCamera_GetArticleNumber(char *article_number)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    /*!< Send command and wait the ack from the camera*/
    if(WirisProt_SendGetAck(s_pWirisProt, "GATN", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check length feedback from the camera is valid*/
    if(ackDataLen > 0) 
    {
        /*!< Copy data to buffer*/
        memcpy(article_number, ackData, ackDataLen);
        
#if (WIRIS_SLOG == STD_ON)
    DebugInfo("AN: %s", article_number);
#endif 
        
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief Get Wiris Firmware Version
 * @details Returns WIRIS device Article number
 */
T_DjiReturnCode WirisCamera_GetFirmwareVersion(char *fw_version)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    /*!< Send command and wait the ack from the camera*/
    if(WirisProt_SendGetAck(s_pWirisProt, "GFWV", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check length feedback from the camera is valid*/
    if(ackDataLen > 0) 
    {
        /*!< Copy data to buffer*/
        memcpy(fw_version, ackData, ackDataLen);
        
#if (WIRIS_SLOG == STD_ON)
    DebugInfo("FW_Ver: %s", fw_version);
#endif 
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
  * @} //  BASIC_COMMANDS
  */

/*!<====================== GPS FUNCTIONS =================================== */

/** @addtogroup GPS_FUNCTIONS
  * @{
  */
/**
 * @brief Get the current GPS coordinates as long as it is provided to the camera.
 * It relies on external source. Returns either N/A when GPS is not connected, 
 * INVALID when GPS data is not valid or the coordinates in following format
 * LATITUDE 14.4444 S
 * LONGTITUDE 57.5555 W
 * ALTITUDE 156.156
 * @details Returns WIRIS device Article number
 */
T_DjiReturnCode WirisCamera_GetGPS(char *coordinates)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    /*!< Send command and wait the ack from the camera*/
    if(WirisProt_SendGetAck(s_pWirisProt, "GGPS", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check length feedback from the camera is valid*/
    if(ackDataLen > 0) 
    {
        memcpy(coordinates, ackData, ackDataLen);
        
#if (WIRIS_SLOG == STD_ON)
    DebugInfo("FW_Ver: %s", coordinates);
#endif 
        
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief Get the current GPS coordinates as long as it is provided to the camera.
 * It relies on external source. Returns either N/A when GPS is not connected, 
 * INVALID when GPS data is not valid or the coordinates in following format
 * LATITUDE 14.4444 S
 * LONGTITUDE 57.5555 W
 * ALTITUDE 156.156
 * @details Returns WIRIS device Article number
 */
T_DjiReturnCode WirisCamera_SetGPS(const T_GpsDataNmea *data)
{
    T_DjiReturnCode djiReturnCode = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    int Deg = 0, Min = 0, Sec = 0;
    char nmeaBuf[WIRIS_PROT_MAX_DATA_LEN] = {0};
    char command[WIRIS_PROT_MAX_DATA_LEN] = {0};
    char strData[NMEA_GPS_CMD_MAX_LEN] = {0};
    
    /*!<            1          2      3  4       5 6 7   8   9 10 11 12 13  14  15
        eg3. $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
        GPSVS$GPGGA,10160.00,010.84,N,106.71,E,,2,50,050.0,000.0,024.7,000.0,0,0,000056
        $GPGGA,181908.00,3404.7041778,N,07044.3966270,W,4,13,1.00,495.144,M,29.200,M,0.10,0000*40
         GPSVS$GPGGA,050121.00,0010.0014800,N,0106.0029483,E,1,00,0.0,0.0,17.0,0.0,M,0,000001

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
    15   = Checksum*/

    /*!< Get header */
    sprintf(nmeaBuf, "$GPGGA");

    /*!< 1    = UTC of Position */
    sprintf(strData, ",%02d%02d%02d.00,", data->aircraftTime.hour, data->aircraftTime.minute, data->aircraftTime.second);
    strcat(nmeaBuf, strData);
    memset(strData, 0, NMEA_GPS_CMD_MAX_LEN);

    int degree = 0;
    double minutes = 0;
    
    /*!<  2    = Latitude */
    degree = (int)data->latitude;
    minutes = (double) ( (data->latitude - (double)degree) * 60.0f);
    uint16_t second_temp = minutes;
    double second = (minutes - second_temp) * 60.0f;
    
    DebugWarning("Latitude[%.15f], Degree[%d], Minutes[%.7f], Second[%.7f]", data->latitude, degree, minutes, second);

    sprintf(strData,"%d%.20f", degree, minutes);
//    sprintf(strData,"%02d%02d.%07d", degree, minutes, seconds);

    strcat(nmeaBuf, strData);
    memset(strData, 0, NMEA_GPS_CMD_MAX_LEN);

    /*!< 3    = N or S  if (string[0] == 'S')
                            gps_Msg.latitude *= -1;*/
    if(data->latitude < 0.0f) {
        sprintf(strData,",S,");
    }
    else {
        sprintf(strData,",N,");
    }
    strcat(nmeaBuf, strData);
    memset(strData, 0, NMEA_GPS_CMD_MAX_LEN);

        /*!< 4    = Longitude */
    degree = (int)data->longtitude;
    minutes = (double) ( (data->longtitude - (double)degree) * 60.0f);
    second_temp = minutes;
    second = (minutes - second_temp) * 60.0f;
    
    DebugWarning("Longitude[%.15f], Degree[%d], Minutes[%.7f], Second[%.7f]", data->longtitude, degree, minutes, second);

    sprintf(strData,"%d%.20f", degree, minutes);
    
    strcat(nmeaBuf, strData);
    memset(strData, 0, NMEA_GPS_CMD_MAX_LEN);

    /*!< 5 = Est or W if (string[0] == 'W')
        gps_Msg.longitude *= -1;*/
    if(data->longtitude < 0.0f) {
        sprintf(strData,",W,");
    }
    else {
        sprintf(strData,",E,");
    }
    strcat(nmeaBuf, strData);
    memset(strData, 0, NMEA_GPS_CMD_MAX_LEN);

    /*!<  6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix) */
    GsdkMath_IntToString(data->gpsQuality, strData, 1);
    strcat(nmeaBuf, strData);
    memset(strData, 0, NMEA_GPS_CMD_MAX_LEN);

    /*!<  7    = Number of satellites in use [not those in view] */
    sprintf(strData, ",");
    GsdkMath_IntToString(data->numSat, &strData[1], 2);
    strcat(nmeaBuf, strData);
    memset(strData, 0, NMEA_GPS_CMD_MAX_LEN);

    /*!<    8    = Horizontal dilution of position */
    sprintf(strData, ",");
    GsdkMath_FloatToString(data->hdop, &strData[1], 1, 1);
    strcat(nmeaBuf, strData);
    memset(strData, 0, NMEA_GPS_CMD_MAX_LEN);

    /*!<  9    = Antenna altitude above/below mean sea level (geoid) */
    sprintf(strData, ",");
    GsdkMath_FloatToString(data->hfsl, &strData[1], 1, 1);
    strcat(nmeaBuf, strData);
    memset(strData, 0, NMEA_GPS_CMD_MAX_LEN);

    /*!<   10 Mean sea level altitude (-9999.9 ~ 17999.9) in meter*/
    sprintf(strData, ",M");
    strcat(nmeaBuf, strData);
    memset(strData, 0, NMEA_GPS_CMD_MAX_LEN);

    /*!< 11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
            mean sea level.  -=geoid is below WGS-84 ellipsoid) */
    sprintf(strData, ",");
    GsdkMath_FloatToString(data->geoidal, &strData[1], 1, 1);
    strcat(nmeaBuf, strData);
    memset(strData, 0, NMEA_GPS_CMD_MAX_LEN);
    /*!<  12   = Meters  (Units of geoidal separation)*/
    sprintf(strData, ",M");
    strcat(nmeaBuf, strData);
    memset(strData, 0, NMEA_GPS_CMD_MAX_LEN);

    /*!<  13   = Age in seconds since last update from diff. reference station*/
    sprintf(strData, ",");
    strcat(nmeaBuf, strData);
    memset(strData, 0, NMEA_GPS_CMD_MAX_LEN);

    /*!<  14   = Diff. reference station ID#*/
    sprintf(strData, ",");
    strcat(nmeaBuf, strData);
    memset(strData, 0, NMEA_GPS_CMD_MAX_LEN);

    /*!< Add detemine */
    sprintf(strData, "*");
    strcat(nmeaBuf, strData);
    memset(strData, 0, NMEA_GPS_CMD_MAX_LEN);
    
    /*!<  15   = Checksum*/
    uint16_t checkSum = GsdkMath_nmeaChecksum(nmeaBuf);
    
    /*!< Accumulate all data */
    sprintf(command, "SGPD %s%02X\r\n", nmeaBuf, checkSum);
    
//    DebugWarning("Command[%s]", command);
    
    /*!< Send command and wait the ack from the camera*/
    if(WirisProt_SendGetAck(s_pWirisProt, command, NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    return djiReturnCode;
}



/**
  * @} // GPS_FUNCTIONS
  */

/*!<====================== RANGE_FUNCTIONS =================================== */

/** @addtogroup RANGE_FUNCTIONS
  * @{ Range settings is available only for WIRIS Pro.
  */
/**
 * @brief Range mode 
 * @details AUTOMATIC/MANUAL/SPAN
 * @note  Return a range mode in string 
 */
static const char *WirisCamera_GetRangeName(E_WirisRange range) 
{
    const char *rangeList[3] = {
        "AUTOMATIC",
        "MANUAL",
        "SPAN",
    };
    
    return rangeList[range];
}

/**
  * @brief WirisCamera_GetRangMode
  * @details Return the current range mdoe 
  * @note 
  */
T_DjiReturnCode WirisCamera_GetRangMode(E_WirisRange *range)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    /*!< Send command and wait the ack from the camera*/
    if(WirisProt_SendGetAck(s_pWirisProt, "GRMD", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
        
    /*!< Check length feedback from the camera is valid*/
    if(ackDataLen > 0) 
    {
        for(uint8_t idx; idx < WIRIS_RANGE_COUNT; idx++) {
            /*!< Check the range name */
            if(!strncmp(WirisCamera_GetRangeName(idx), (const char*)&ackData, ackDataLen)) {
                
                *range = idx;
                return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
            }
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
  * @brief WirisCamera_SetRangMode
  * @details Set the range mode to AUTOMATIC/MANUAL or SPAN
  * @note 
  */
T_DjiReturnCode WirisCamera_SetRangMode(E_WirisRange range)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    /*!< Check input parameter is valid */
    if(range >= WIRIS_RANGE_COUNT) {
        DebugError("Invalid Range mdoe %d.", range);
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    const char* sub = WirisCamera_GetRangeName(range);
    
    if(WirisProt_SendGetAck(s_pWirisProt, "SRMD", sub, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        if( ackData[0] == 0x4F && ackData[1] == 0x4B && ackData[2] == 0x0A){
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
  * @brief WirisCamera_GetManualRange
  * @details Returns the currently set manual thermal range minimum and maximum.
  * @note 
  */
T_DjiReturnCode WirisCamera_GetManualRange(float *min, float max)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    /*!< Send command and wait the ack from the camera*/
    if(WirisProt_SendGetAck(s_pWirisProt, "GRMM", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
        
    /*!< Check length feedback from the camera is valid*/
    if(ackDataLen > 0) 
    {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
  * @brief WirisCamera_SetManualRange
  * @details Sets the thermal manual range minimum and maximum
  * @note 
  */
T_DjiReturnCode WirisCamera_SetManualRange(float min, float max)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    char sub_cmd[10] = {0};
    
    /*!< Convert int to string*/
    sprintf(sub_cmd, "%.1f %.1f", min, max);
    
    if(WirisProt_SendGetAck(s_pWirisProt, "SRMM", sub_cmd, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        if( ackData[0] == 0x4F && ackData[1] == 0x4B && ackData[2] == 0x0A){
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
  * @brief WirisCamera_GetSpanRange
  * @details Returns the current span range window and center.
  * @note 
  */
T_DjiReturnCode WirisCamera_GetSpanRange(float *min, float max)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    /*!< Send command and wait the ack from the camera*/
    if(WirisProt_SendGetAck(s_pWirisProt, "GRWC", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
        
    /*!< Check length feedback from the camera is valid*/
    if(ackDataLen > 0) 
    {
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
  * @brief WirisCamera_SetManualRange
  * @details Sets the manual range window and center
  * @note 
  */

T_DjiReturnCode WirisCamera_SetSpanRange(float min, float max)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    char sub_cmd[10] = {0};
    
    /*!< Convert int to string*/
    sprintf(sub_cmd, "%.1f %.1f", min, max);
    
    if(WirisProt_SendGetAck(s_pWirisProt, "SRWC", sub_cmd, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        if( ackData[0] == 0x4F && ackData[1] == 0x4B && ackData[2] == 0x0A){
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
  * @brief WirisCamera_GetEnvironment
  * @details Returns the current thermal environment (absolute temperature range) setting
  * @note 
  */
T_DjiReturnCode WirisCamera_GetEnvironment(float *min, float max)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    /*!< Send command and wait the ack from the camera*/
    if(WirisProt_SendGetAck(s_pWirisProt, "GREN", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
        
    /*!< Check length feedback from the camera is valid*/
    if(ackDataLen > 0) 
    {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
  * @brief WirisCamera_GetListEnvironment
  * @details Returns the list of available thermal environments. Each line is one environment.
  * @note 
  */
T_DjiReturnCode WirisCamera_GetListEnvironment(uint8_t list)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    /*!< Send command and wait the ack from the camera*/
    if(WirisProt_SendGetAck(s_pWirisProt, "GREL", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
        
    /*!< Check length feedback from the camera is valid*/
    if(ackDataLen > 0) 
    {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
  * @brief WirisCamera_SetEnvironment
  * @details Sets the environment settings. Use just the maximum value of the range. 
  * This command can take up to 10 seconds to perform.
  * @note 
  */

T_DjiReturnCode WirisCamera_SetEnvironment(float temp)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    char sub_cmd[10] = {0};
    
    /*!< Convert int to string*/
    sprintf(sub_cmd, "%.1f", temp);
    
    if(WirisProt_SendGetAck(s_pWirisProt, "SREN", sub_cmd, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        if( ackData[0] == 0x4F && ackData[1] == 0x4B && ackData[2] == 0x0A){
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}
/**
  * @} // RANGE_FUNCTIONS
  */
/*!<====================== WIRIS_SECURITY_THERMAL_PARAMETERS_FUNCTIONS =======*/

/** @addtogroup WIRIS_SECURITY_THERMAL_PARAMETERS_FUNCTIONS
  * @{ 
  */
/**
  * @brief WirisCamera_GetTimeStabilization
  * @details Returns thermal camera time stabilization in seconds.
  * @note 
  */
T_DjiReturnCode WirisCamera_GetTimeStabilization(float *timeStablization)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    /*!< Send command and wait the ack from the camera*/
    if(WirisProt_SendGetAck(s_pWirisProt, "GTST", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
        
    /*!< Check length feedback from the camera is valid*/
    if(ackDataLen > 0) 
    {
        *timeStablization = GsdkMath_extendedAtof((const char*)&ackData);
        
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}



/**
  * @brief WirisCamera_SetTimeStabilization
  * @details Sets thermal camera time stabilization in seconds.
  * @note 
  */

T_DjiReturnCode WirisCamera_SetTimeStabilization(const float timeStablization)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    char sub_cmd[10] = {0};
    
    /*!< Convert int to string*/
    sprintf(sub_cmd, "%.1f", timeStablization);

#if (WIRIS_SEND_ACK_ENABLE == STD_ON)
    if(WirisProt_SendGetAck(s_pWirisProt, "STST", sub_cmd, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        /*!< Return OK*/
        if( ackData[0] == 0x4F && ackData[1] == 0x4B && ackData[2] == 0x0A){
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
#else

    DebugInfo("Set time stablize: %s", sub_cmd);
    if(WirisProt_Send(s_pWirisProt, "STST", sub_cmd) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
#endif
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
  * @brief WirisCamera_GetHotRejection
  * @details Returns hot rejection in percent
  * @note 
  */
T_DjiReturnCode WirisCamera_GetHotRejection(float *rejection)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    /*!< Send command and wait the ack from the camera*/
    if(WirisProt_SendGetAck(s_pWirisProt, "GHRJ", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
        
    /*!< Check length feedback from the camera is valid*/
    if(ackDataLen > 0) 
    {
        *rejection = GsdkMath_extendedAtof((const char*)&ackData);
        
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
  * @brief WirisCamera_SetHotRejection
  * @details Sets hot rejection in percent.
  * @note 
  */

T_DjiReturnCode WirisCamera_SetHotRejection(const float rejection)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    char sub_cmd[10] = {0};
    
    /*!< Convert int to string*/
    sprintf(sub_cmd, "%.1f", rejection);
    
    if(WirisProt_SendGetAck(s_pWirisProt, "SHRJ", sub_cmd, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        /*!< Return OK*/
        if( ackData[0] == 0x4F && ackData[1] == 0x4B && ackData[2] == 0x0A){
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
  * @brief WirisCamera_GetCoolRejection
  * @details Returns cold rejection in percent.
  * @note 
  */
T_DjiReturnCode WirisCamera_GetCoolRejection(float *rejection)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    /*!< Send command and wait the ack from the camera*/
    if(WirisProt_SendGetAck(s_pWirisProt, "GCRJ", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
        
    /*!< Check length feedback from the camera is valid*/
    if(ackDataLen > 0) 
    {
        *rejection = GsdkMath_extendedAtof((const char*)&ackData);
        
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
  * @brief WirisCamera_SetCoolRejection
  * @details Sets cold rejection in percent.
  * @note 
  */
T_DjiReturnCode WirisCamera_SetCoolRejection(const float rejection)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    char sub_cmd[10] = {0};
    
    /*!< Convert int to string*/
    sprintf(sub_cmd, "%.1f", rejection);
    
    if(WirisProt_SendGetAck(s_pWirisProt, "SCRJ", sub_cmd, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        /*!< Return OK*/
        if( ackData[0] == 0x4F && ackData[1] == 0x4B && ackData[2] == 0x0A){
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}
/**
  * @} // WIRIS_SECURITY_THERMAL_PARAMETERS_FUNCTIONS
  */


/*!<====================== THERMAL_PARAMETERS_FUNCTIONS ===================== */

/** @addtogroup THERMAL_PARAMETERS_FUNCTIONS
  * @{
  */
/**
  * @} // THERMAL_PARAMETERS_FUNCTIONS
  */

/*!<====================== APPEARANCE_FUNCTIONS ===================== */

/** @addtogroup APPEARANCE_FUNCTIONS
  * @{
  */
/**
 * @brief Set Layout
 * @details  Set layout for the HDMI output. The available layouts for WIRIS PRO
 * are INSPECTION, SECURITY, FULLSCREEN and PIP
 * The available layouts for WIRIS SECURITY are SECURITY, FULLSCREEN and PIP
 */
T_DjiReturnCode WirisCamera_SetLayout(const E_WirisSecLayout layout)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(layout >= SLAY_WWS_COUNT_MAX) {
        DebugError("Invalid Arg Set Layout: %d", layout);
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    const char *wiris_security[3] = {"SECURITY", "FULLSCREEN", "PIP"};

#if (WIRIS_SEND_ACK_ENABLE == STD_ON)
    
    /*!< Send command and wait the ack from the camera*/
    if(WirisProt_SendGetAck(s_pWirisProt, "SLAY", wiris_security[layout], ackData, &ackDataLen, WIRIS_CRITICAL_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check length feedback from the camera is valid: OK*/
    if(ackDataLen > 0) {
        if( ackData[0] == 0x4F && ackData[1] == 0x4B && ackData[2] == 0x0A) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            for(int i = 0; i < ackDataLen; i++) {
                DebugError("[%s][%d]: %c", __FUNCTION__, __LINE__, ackData[i]);
            }
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    
#else
    /*!< Send command and wait the ack from the camera*/
    if(WirisProt_Send(s_pWirisProt, "SLAY", wiris_security[layout]) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
        
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
#endif
}

/**
 * @brief Set main camera
 * @details Set camera for main display
 */
T_DjiReturnCode WirisCamera_SetMainCamera(const E_WirisMainCamera main)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(main >= SMCA_COUNT_MAX) {
        DebugError("Invalid Arg main camera: %d", main);
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    const char *main_camera[2] = {"THERMAL", "VISIBLE"};
#if (WIRIS_SEND_ACK_ENABLE == STD_ON) // Debug send only  
    /*!< Send command and wait the ack from the camera*/
    if(WirisProt_SendGetAck(s_pWirisProt, "SMCA", main_camera[main], ackData, &ackDataLen, WIRIS_CRITICAL_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check length feedback from the camera is valid: OK*/
    if(ackDataLen > 0) {
        if( ackData[0] == 0x4F &&  ackData[1] == 0x4B && ackData[2] == 0x0A) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
             for(int i = 0; i < ackDataLen; i++) {
                DebugError("[%s][%d]: %c", __FUNCTION__, __LINE__, ackData[i]);
            }
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

#else
     /*!< Send command and wait the ack from the camera*/
    if(WirisProt_Send(s_pWirisProt, "SMCA", main_camera[main])
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
#endif

}

/**
 * @brief Set thermal camera transparency
 * @details  Sets thermal camera transprancy in PIP Fusion layout from 10 to 100 in percent
 */
T_DjiReturnCode WirisCamera_SetThermalCameraTransparency(const uint8_t percent)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    char sub_cmd[4] = {0};
    
    /*!< Check percent is invalid*/
    if(percent < 10 || percent > 100) {
        DebugError("Invalid Transparency value: %d", percent);
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    /*!< Convert int to string*/
    sprintf(sub_cmd, "%d", percent);
    
    /*!< Send command and wait the ack from the camera*/
    if(WirisProt_SendGetAck(s_pWirisProt, "STTY", sub_cmd, ackData, &ackDataLen, WIRIS_CRITICAL_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check length feedback from the camera is valid: OK*/
    if(ackDataLen > 0) {
        if( ackData[0] == 0x4F &&  ackData[1] == 0x4B && ackData[2] == 0x0A) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
  * @} // APPEARANCE_FUNCTIONS
  */

/*!<====================== ZOOM FUNCTIONS =================================== */
/** @addtogroup ZOOM_FUNCTIONS
  * @{ 
  */

/**
 * @brief Zoom in signal
 * @details Zooms current main camera in
 * @note 
 */
T_DjiReturnCode WirisCamera_ZoomIn(void)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "SZIN", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }

    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) 
    {
        if( ackData[0] == 0x4F && 
            ackData[1] == 0x4B && 
            ackData[2] == 0x0A) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief Zoom out signal
 * @details Zooms current main camera in
 * @note 
 */
T_DjiReturnCode WirisCamera_ZoomOut(void)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "SZOT", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }

    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) 
    {
        if( ackData[0] == 0x4F && 
            ackData[1] == 0x4B && 
            ackData[2] == 0x0A) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/** @defgroup THERMAL_CAMERA_ZOOM
  * @brief Functions supports for thermal camera zoom
  * @{
  */

/**
 * @brief Get thermal camera zoom
 * @details Return thermal camera current zoom index and zoom ratio
 */
T_DjiReturnCode WirisCamera_GetThermalZoomVal(uint8_t *zoom_index, float *zoom_ratio)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "GZTV", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }

//    /*!< Check lengh ack data is valid*/
//    if(ackDataLen > 0) 
//    {
//        char index[3] = {0};
//        
//        int i = 0;
//        
//        for(i = 0; i < ackDataLen; i++) {
//            /*!< Check the space*/
//            if(ackData[i] != 0x20){
//                index[i] = ackData[i]; 
//            }
//            else {
//                index[i] = 0x0A;
//                break;
//            }
//        }
//        
//        *zoom_index = GsdkMath_extendedAtof((const char*)&index[0]);
//        *zoom_ratio = GsdkMath_extendedAtof((const char*)&ackData[i]);
//#if (WIRIS_SLOG == STD_ON)
//        DebugInfo("[%d] %d - %f",i, (uint8_t)*zoom_index, *zoom_ratio);
//#endif
//        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
//    }

    if(ackDataLen > 0) {
        char *str, *next;
        
        str = GsdkMath_getOpt((char*) ackData, &next, ' ');
        *zoom_index = GsdkMath_extendedAtof(str);
        str = GsdkMath_getOpt(next, &next, 0);
        *zoom_ratio = GsdkMath_extendedAtof(str);
        
        DebugInfo("ZoomT %d - %f",(uint8_t)*zoom_index, *zoom_ratio);

        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief Get list of visible camera zooms
 * @details Get list of all available zooms of visible camera. Each line has index number and zoom ratio value
 */
T_DjiReturnCode WirisCamera_GetThermalZoomList(void)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "GZTL", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }

//    if(ackDataLen > 0) 
//    {
//        for(int i = 0; i < ackDataLen; i++) {
//            DebugInfo("%c", ackData[i]);
//        }
//        
//        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
//    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief Set thermal camera zoom index 
 * @details Sets the thermal camera zoom index number
 * Thermal camera has only digital zoom, so the stream is not inflicted by the zoom value
 * The zoom changes the area where maximum and minimum values are loocked for.
 */
T_DjiReturnCode WirisCamera_SetThermalZoomIndex(const uint8_t index)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    char command[10] = {0};
    
    /*!< Limit index for Thermal*/
    if(index > 16) {
         return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    
    /*!< Convert int to string*/
    sprintf(command, "SZTN %d", index);
    
    if(WirisProt_SendGetAck(s_pWirisProt, command, NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }

    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        if( ackData[0] == 0x4F && 
            ackData[1] == 0x4B && 
            ackData[2] == 0x0A) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
  * @} // THERMAL_CAMERA_ZOOM
  */

/** @defgroup VISIBLE_CAMERA_ZOOM
  * @brief Functions supports for visible camera zoom
  * @{
  */
/**
 * @brief Get thermal camera zoom
 * @details Return thermal camera current zoom index and zoom ratio
 */
T_DjiReturnCode WirisCamera_GetVisibleZoomVal(uint8_t *zoom_index, float *zoom_ratio)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "GZVV", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }

//    /*!< Check lengh ack data is valid*/
//    if(ackDataLen > 0) {
//        char index[3] = {0};
//        
//        int i = 0;
//        
//        for(i = 0; i < ackDataLen; i++) {
//            /*!< Check the space*/
//            if(ackData[i] != 0x20){
//                index[i] = ackData[i]; 
//            }
//            else {
//                index[i] = 0x0A;
//                break;
//            }
//        }
//        
//        *zoom_index = GsdkMath_extendedAtof((const char*)&index[0]);
//        *zoom_ratio = GsdkMath_extendedAtof((const char*)&ackData[i]);
//#if (WIRIS_SLOG == STD_ON)
//        DebugInfo("[%d] %d - %f",i, (uint8_t)*zoom_index, (float)*zoom_ratio);
//#endif
//        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
//    }

    if(ackDataLen > 0) {
        char *str, *next;
        
        str = GsdkMath_getOpt((char*) ackData, &next, ' ');
        *zoom_index = GsdkMath_extendedAtof(str);
        str = GsdkMath_getOpt(next, &next, 0);
        *zoom_ratio = GsdkMath_extendedAtof(str);
        
        DebugInfo("ZoomV %d - %f",(uint8_t)*zoom_index, *zoom_ratio);

        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief Get thermal camera zoom
 * @details Return thermal camera current zoom index and zoom ratio
 * @note DO NOT RUN
 */
T_DjiReturnCode WirisCamera_GetVisibleZoomlist(void)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "GZVL", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }

//    /*!< Check lengh ack data is valid*/
//    if(ackDataLen > 0) {
//        char index[3] = {0};
//        
//        int i = 0;
//        
//        for(i = 0; i < ackDataLen; i++) {
//            /*!< Check the space*/
//            if(ackData[i] != 0x20){
//                index[i] = ackData[i]; 
//            }
//            else {
//                index[i] = 0x0A;
//                break;
//            }
//        }
//        
//        *zoom_ratio = GsdkMath_extendedAtof((const char*)&ackData[i]);
//#if (WIRIS_SLOG == STD_ON)
//        DebugInfo("[%d] %d - %f",i, (uint8_t)*zoom_index, (float)*zoom_ratio);
//#endif
//        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
//    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief Set visible camera zoom index 
 * Answer OK
 */
T_DjiReturnCode WirisCamera_SetVisibleZoomIndex(const uint8_t index)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    char command[10] = {0};
    
     /*!< Limit index for Thermal*/
    if(index > 30) {
         return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    /*!< Convert int to string*/
    sprintf(command, "SZVN %d", index);
    
    if(WirisProt_SendGetAck(s_pWirisProt, command, NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }

    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        if( ackData[0] == 0x4F && 
            ackData[1] == 0x4B && 
            ackData[2] == 0x0A) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else 
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
  * @} // VISIBLE_CAMERA_ZOOM
  */

/**
  * @} // ZOOM_FUNCTIONS
  */

/*!<====================== PALETTES_FUNCTIONS =================================== */

/** @addtogroup PALETTES_FUNCTIONS
  * @{
  */

const char *WirisCamera_GetPaletteName(const uint8_t palette)
{
    const char *paletteList[15] = {
        "01_GRAY",
        "02_GRAY-I",
        "03_BLACKGREEN",
        "04_BLACKRED",
        "05_BWRGB",
        "06_WBRGB",
        "07_BWIRON",
        "08_BWIRON-I",
        "09_IRON",
        "10_FIRE",
        "11_RAINBOW",
        "12_RAINBOW-HC",
        "13_NATURAL",
        "14_SEPIA",
        "15_TEMPERATURE"
    };
    
    return paletteList[palette];
}

/**
 * @brief Get palette
 * @details Returns current palette index and name
 * @note 
 */
T_DjiReturnCode WirisCamera_GetPalette(E_WirisPalette *index)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "GPTE", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    char buf[3] = {0};
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) 
    {
        for(uint8_t idx = 0; idx < ackDataLen; idx++) {
            
            if(ackData[idx] != 0x20)
                buf[idx] =  ackData[idx];
        }
        
        *index = atoi(buf);
        
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief Get palette list
 * @details Returns all the available palettes. Each line is one palette index and name 
 * @note ("0 GRAY")
 * DO NOT RUN
 */
T_DjiReturnCode WirisCamera_GetPaletteList(void)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "GPTL", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) 
    {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief Set palette by name
 * @details 
 * @note ("SPTE GRAY")
 */
T_DjiReturnCode WirisCamera_SetPaletteName(const E_WirisPalette palette)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    const char* sub = WirisCamera_GetPaletteName(palette);
    
    if(WirisProt_SendGetAck(s_pWirisProt, "SPTE", sub, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        if( ackData[0] == 0x4F && ackData[1] == 0x4B && ackData[2] == 0x0A){
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief Set palette by index
 * @details SPTI 0
 * @note
 */
T_DjiReturnCode WirisCamera_SetPaletteIndex(const E_WirisPalette palette)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(palette >= WIRIS_PALETTE_COUNT_MAX) {
        DebugError("Invalid palette setting: %d", palette);
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    char sub_cmd[1] = {0};
    
    /*!< Convert int to string*/
    sprintf(sub_cmd, "%d", palette);
    
    if(WirisProt_SendGetAck(s_pWirisProt, "SPTI", sub_cmd, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }

    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0)  {
        if( ackData[0] == 0x4F && ackData[1] == 0x4B && ackData[2] == 0x0A){
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}
/**
  * @} // PALETTES_FUNCTIONS
  */

/*!<====================== CAPTURE_RECORD_FUNCTIONS =================================== */

/** @addtogroup CAPTURE_RECORD_FUNCTIONS
  * @{
  */

/**
 * @brief Capture
 * @details Triggers the image capture. The commands an acknowledgement right away.
 * but the capture itself can take up to sevral seconds depending on the settings.
 * @return NOT_READY in teh case the capture cannot be initiated due to the last one not being finished yet
 * @note 
 */
T_DjiReturnCode WirisCamera_Capture(void)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "CPTR", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }

    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) 
    {
        if( ackData[0] == 0x4F && 
            ackData[1] == 0x4B && 
            ackData[2] == 0x0A) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else 
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}
/**
 * @brief Is Capturing
 * @details Check if capture in progress
 * @note Answer TRUE/FALSE
 */
T_DjiReturnCode WirisCamera_IsCapturing(bool *isCapturing)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "ICPT", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }

    char answer[] = {'T', 'R', 'U', 'E', '\n'};
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) 
    {
        /*!< Check the feedback*/
        if(!strncmp(answer, (const char*)&ackData, ackDataLen)) {
            
            *isCapturing = true;
        }
        else {
            *isCapturing = false;
        }
        
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief Recording Start
 * @details Start recording thermal and visible video according to settings
 * @note OK or NOT_READY
 */
T_DjiReturnCode WirisCamera_RecordingStart(void)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "RCRS", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }

    char answer[] = {'O', 'K', '\n'};
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) 
    {
        /*!< Check the feedback*/
        if(!strncmp(answer, (const char*)&ackData, ackDataLen)) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief Recording Finish
 * @details Finish recording thermal and visible video according to settings
 * @note OK or NOT_READY
 */
T_DjiReturnCode WirisCamera_RecordingFinish(void)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "RCRF", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }

    char answer[] = {'O', 'K', '\n'};
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) 
    {
        /*!< Check the feedback*/
        if(!strncmp(answer, (const char*)&ackData, ackDataLen)) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief Check if
 * @details Finish recording thermal and visible video according to settings
 * @note OK or NOT_READY
 */
T_DjiReturnCode WirisCamera_IsRecording(bool *isRecording)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "IRCR", NULL, ackData, &ackDataLen, 2000) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }

    char answer_true[] = {'T', 'R', 'U', 'E','\n'};
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) 
    {
        /*!< Check the feedback*/
        if(!strncmp(answer_true, (const char*)&ackData, ackDataLen)) {
            *isRecording = true;
            
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            *isRecording = false;
            
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        } 
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}
/**
  * @} // CAPTURE_RECORD_FUNCTIONS
  */

/*!<====================== ALARMS_FUNCTION=================================== */

/** @addtogroup 
  * @{ ALARMS_FUNCTION
  */
static const char *WirisCamera_GetAlarmModeName(const E_WirisAlarmMode alarmMode)
{
    if(alarmMode >= WIRIS_ALARM_MODE_COUNT) {
        DebugMsg("Get Alarm Mode name Error %d", alarmMode);
        
        return "UNKNOWN";
    }
    
    const char *alarmModeString[5] = {"OFF", "ABOVE", "BELOW", "BETWEEN", "OUTSIDE"};
    
    return alarmModeString[alarmMode];
}

static const char *WirisCamera_GetAlarmColorName(const E_WirisAlarmColorList color)
{
    if(color > WIRIS_ALARM_COLOR_LIST_MAX) {
        DebugMsg("Get Alarm Color name Error %d", color);
        
        return "UNKNOWN";
    }
    
    const char *alarmColorString[] = {"RED", "GREEN", "BLUE"};
    
    return alarmColorString[color];
}


/**
 * @brief WirisCamera_GetAlarmMode
 * @details Return current alarm mode 
 * @note
 */
T_DjiReturnCode WirisCamera_GetAlarmMode(E_WirisAlarmMode *alarmMode)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
        
    if(WirisProt_SendGetAck(s_pWirisProt, "GALM", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        /*!< Scan in the list with the data feedback*/
        for(E_WirisAlarmMode i = 0; i < WIRIS_ALARM_MODE_COUNT; i++) {
            /*!< Compare a string. But the length need to minus the \n 1 byte*/
            if(!strncmp(WirisCamera_GetAlarmModeName(i), (const char*)&ackData, ackDataLen - 1)) {
                
                /*!< Get index after compare from the mode list*/
                *alarmMode = i;
                
                return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
            }
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief WirisCamera_SetAlarmMode
 * @details Set alarm mode OFF, ABOVE, BELOW, BETWEEN, or OUTSIDE.
 * @note 
 */
T_DjiReturnCode WirisCamera_SetAlarmMode(const E_WirisAlarmMode alarmMode)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    /*!< Check input parameter is valid */
    if(alarmMode >= WIRIS_ALARM_MODE_COUNT) {
        DebugError("Invalid Alarm mode %d.", alarmMode);
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }
    
    const char* sub = WirisCamera_GetAlarmModeName(alarmMode);
    
    if(WirisProt_SendGetAck(s_pWirisProt, "SALM", sub, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        if( ackData[0] == 0x4F && ackData[1] == 0x4B && ackData[2] == 0x0A){
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief WirisCamera_GetAlarmValue
 * @details Get alarm thresholds; below and above.
 * @note 
 */
T_DjiReturnCode WirisCamera_GetAlarmValue(float *thresholdBelow, float *thresholdAbove)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
        
    if(WirisProt_SendGetAck(s_pWirisProt, "GALV", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        
        char buf[10];
        uint8_t idx = 0;
        for(idx = 0; idx < ackDataLen; idx++) {
            if(ackData[idx] != 0x20)
                buf[idx] =  ackData[idx];
            else {
                buf[idx] = 0x0A;
                break;
            }
        }
        /*!< Calulate the below*/
        *thresholdBelow = GsdkMath_extendedAtof(buf);
        
        /*!< Clear buffer */
        memset(buf, 0x00, sizeof(buf));
        
        /*!< Calculate buffer*/
        *thresholdAbove = GsdkMath_extendedAtof((const char*)&ackData[idx + 1]);
        
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/**
 * @brief WirisCamera_SetAlarmValue
 * @details Set alarm thresholds; below and above.
 * @note 
 */
T_DjiReturnCode WirisCamera_SetAlarmValue(const float thresholdBelow, const float thresholdAbove)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    char sub_cmd[10] = {0};
    
    /*!< Convert int to string*/
    sprintf(sub_cmd, "%.1f %.1f", thresholdBelow, thresholdAbove);
    
    if(WirisProt_SendGetAck(s_pWirisProt, "SALV", sub_cmd, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        if( ackData[0] == 0x4F && ackData[1] == 0x4B && ackData[2] == 0x0A){
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}
/**
 * @brief Fuction Get alarm color 
 * @details This structure type is used to
 * @note 
 */
T_DjiReturnCode WirisCamera_GetAlarmColor(E_WirisAlarmColorList *color)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
        
    if(WirisProt_SendGetAck(s_pWirisProt, "GALC", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {

        char *str, *next;
        str = GsdkMath_getOpt((char*) ackData, &next, ' ');
        
        /*!< Scan in the list with the data feedback*/
        for(uint8_t i = 0; i < WIRIS_ALARM_COLOR_LIST_MAX; i++) {
            if(!strcmp(WirisCamera_GetAlarmColorName(i), str)) {
                *color = i;
            }
        }
        
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}


/**
 * @brief WirisCamera_SetAlarmColor
 * @details Set alarm color above, between and below 
 * @note  Possible colors are red, green or blue.
 */
T_DjiReturnCode WirisCamera_SetAlarmColor(const E_WirisAlarmColorList color)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(color == WIRIS_ALARM_RED_GREEN_BLUE) {
        if(WirisProt_SendGetAck(s_pWirisProt, "SALC RED GREEN BLUE", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
        }
    }
    else if(color == WIRIS_ALARM_GREEN_RED_BLUE) {
        if(WirisProt_SendGetAck(s_pWirisProt, "SALC GREEN RED BLUE", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
        }
    }
    else if(color == WIRIS_ALARM_BLUE_GREEN_RED) {
        if(WirisProt_SendGetAck(s_pWirisProt, "SALC BLUE GREEN RED", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
        }
    }
    else {
        DebugError("Invalid Color !!!!");
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        if( ackData[0] == 0x4F && ackData[1] == 0x4B && ackData[2] == 0x0A){
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        else {
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}
/**
  * @} // ALARMS_FUNCTION
  */

/*!<====================== MEMMORY_FUNCTIONS =================================== */
/** @addtogroup MEMMORY_FUNCTIONS
  * @{
  */
/*!< 
    There are three types of memory SSD, SD_CARD or FLASH_DRIVE

    The following commands returns the status of each memory on one line like
    SSD SOME_STATUS
    SD_CARD SOME_STATUS
    FLASH_DRIVE SOME_STATUS
*/
/**
 * @brief Get memory status 
 * @details Finish recording thermal and visible video according to settings
 * @note OK or NOT_READY
 */
T_DjiReturnCode WirisCamera_GetMemoryStatus(uint8_t *status)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "GMST", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }

    char ready[] = {'R', 'E', 'A', 'D', 'Y'};
    char connected[] = {'C', 'O', 'N', 'N', 'E', 'C', 'T', 'E', 'D'};
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) 
    {
        /*!< Step 1: Separate the memory SSD 2564884565*/
        union{
            uint32_t    ull;
            char text[sizeof(uint32_t)];
        }ssd_name = {
            .text = {'S', 'S', 'D', ' '}
        };
        
        uint32_t *p = (uint32_t*) ackData;
        
        /*!< Compare name for SSD*/
        if(*p == ssd_name.ull)
        {
            /*!< Check the feedback*/
            if(!strncmp(ready, (const char*)&ackData[4], sizeof(ready))) {
                *status = 1;
                return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
            }
            else if(!strncmp(connected, (const char*)&ackData[4], sizeof(connected))) {
                *status = 2;
                return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
            }
            else {
                *status = 0;
                return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
            }
        }
        else {
            DebugError("[%s][%d]: Invalid SSD name", __FUNCTION__, __LINE__);
            
//             for(int i = 0; i < ackDataLen;  i++){
//                DebugWarning("[GetMemoryStatus] - 0x%02X - %c ", ackData[i], ackData[i]);
//             }
             return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/** 
 * @brief Get memory size
 * @details Return the current memmory size in bytes, each line one memory type
 * @note SSD 123456789
 */
T_DjiReturnCode WirisCamera_GetMemorySize(uint64_t* ssd_size, uint64_t* flash_drive_size, uint64_t* sd_card_size)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "GMSI", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) 
    {
        /*!< Step 1: Separate the memory SSD 2564884565*/
        union{
            uint32_t    ull;
            char text[sizeof(uint32_t)];
        }ssd_name = {
            .text = {'S', 'S', 'D', ' '}
        };
        
        uint32_t *p = (uint32_t*) ackData;
        if(*p == ssd_name.ull)
        {
            if(ssd_size != NULL)
                *ssd_size = GsdkMath_StringToInteger((char*)&ackData[4]);
        }
        
#if (WIRIS_SLOG == STD_ON)
        DebugInfo("Get size: %d",  *ssd_size);
#endif

        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/** 
 * @brief Get memory size
 * @details Return the current memory in percent, each line one memory type
 * @note SSD 88.88
 */
T_DjiReturnCode WirisCamera_GetMemoryFree(float* ssd_percent, float* flash_drive_percent, float* sd_card_percent)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "GMFR", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) 
    {
        /*!< Step 1: Separate the memory SSD 2564884565*/
        union{
            uint32_t    ull;
            char text[sizeof(uint32_t)];
        }ssd_name = {
            .text = {'S', 'S', 'D', ' '}
        };
        
        uint32_t *p = (uint32_t*) ackData;
        if(*p == ssd_name.ull)
        {
            uint8_t buf[20] = {0};
            
            for(int i = 0; ackData[i+4] != '\0'; i++) {
                if(ackData[i] == 0x0D || ackData[i] == 0x0A) {
                    break;
                }
                buf[i] =  ackData[i+4];
            }
            
            if(ssd_percent != NULL)
                *ssd_percent = GsdkMath_extendedAtof((char*)&buf[0]);
        }
        
#if (WIRIS_SLOG == STD_ON)

#endif
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/** 
 * @brief Get captured images
 * @details Return the current captured images, each line one memory type
 * @note SSD 88.88
 */
T_DjiReturnCode WirisCamera_GetCapturedImages(uint64_t* ssd_captured_images, uint64_t* flash_captured_images, uint64_t* sd_captured_images)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "GMCP", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) 
    {
        /*!< Step 1: Separate the memory SSD 2564884565*/
        union{
            uint32_t    ull;
            char text[sizeof(uint32_t)];
        }ssd_name = {
            .text = {'S', 'S', 'D', ' '}
        };
        
        uint32_t *p = (uint32_t*) ackData;
        if(*p == ssd_name.ull)
        {
            if(ssd_captured_images != NULL)
                *ssd_captured_images = GsdkMath_StringToInteger((char*)&ackData[4]);
        }
        
#if (WIRIS_SLOG == STD_ON)
//        DebugInfo("Get size: %d",  *ssd_captured_images);
#endif
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/** 
 * @brief Get captured images
 * @details Return the recorded radiometric video in seconds.
 * @note  1234
 */
T_DjiReturnCode WirisCamera_GetRecordedThermalVideo(uint64_t* seconds)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "GTRC", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) 
    {
        if(seconds != NULL) {
            *seconds = GsdkMath_StringToInteger((char*)&ackData[0]);
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}

/** 
 * @brief Get captured images
 * @details Return the recorded radiometric video in seconds.
 * @note  1234
 */
T_DjiReturnCode WirisCamera_GetRecordedVisibleVideo(uint64_t* seconds)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "GVRC", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) 
    {
        if(seconds != NULL) {
            *seconds = GsdkMath_StringToInteger((char*)&ackData[0]);
            
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
#if (WIRIS_SLOG == STD_ON)

#endif
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}
/**
  * @} // MEMMORY_FUNCTIONS
  */

/*!<====================== DATE & TIME FUNCTIONS ============================ */

/** @addtogroup 
  * @{
  */
/**
 * @brief Set data and time 
 * @details Set the date and time in strict format yyyy/MM/dd-hh:mm:ss.
 * @note 2019/06/21-10:44:51
 */
T_DjiReturnCode WirisCamera_GetDataAndTime(T_DjiTimeSyncAircraftTime *dateTime)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    
    if(WirisProt_SendGetAck(s_pWirisProt, "GDTI", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        
        char *str, *next;
        
        /*!< Get Date*/
        str = GsdkMath_getOpt((char*)ackData, &next, '/');
        dateTime->year = atoi(str);
        str = GsdkMath_getOpt(next, &next, '/');
        dateTime->month = atoi(str);
        str = GsdkMath_getOpt(next, &next, '-');
        dateTime->day = atoi(str);
        
        /*!< Get Time*/
        str = GsdkMath_getOpt(next, &next, ':');
        dateTime->hour = atoi(str);
        str = GsdkMath_getOpt(next, &next, ':');
        dateTime->minute = atoi(str);
        str = GsdkMath_getOpt(next, &next, 0);
        dateTime->second = atoi(str);

        DebugWarning("GOT %04d/%02d/%02d-%02d:%02d:%02d", dateTime->year,
                                                                    dateTime->month, 
                                                                    dateTime->day, 
                                                                    dateTime->hour, 
                                                                    dateTime->minute,
                                                                      dateTime->second);

        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}
/**
 * @brief Set data and time 
 * @details Set the date and time in strict format yyyy/MM/dd-hh:mm:ss.
 * @note 
 */
T_DjiReturnCode WirisCamera_SetDataAndTime(const T_DjiTimeSyncAircraftTime *aircraftTime)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
    char sub_cmd[WIRIS_PROT_MAX_DATA_LEN] = {0};
    
    memset(sub_cmd, 0x00, WIRIS_PROT_MAX_DATA_LEN);
    
    /*!< Convert int to string*/
    sprintf(sub_cmd, "SDTI %04d/%02d/%02d-%02d:%02d:%02d", aircraftTime->year,
                                          aircraftTime->month, 
                                          aircraftTime->day, 
                                          aircraftTime->hour, 
                                          aircraftTime->minute,
                                          aircraftTime->second);
    
    DebugInfo("CMD: %s", sub_cmd);
    
    if(WirisProt_SendGetAck(s_pWirisProt, sub_cmd, NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        if( ackData[0] == 0x4F && ackData[1] == 0x4B && ackData[2] == 0x0A){
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}
/**
  * @} // 
  */

/**
  * @} // 
  */

/*!<====================== DATE & TIME FUNCTIONS ============================ */

/** @addtogroup 
  * @{
  */

/**
 * @brief Reboot the WIRIS 
 * @details 
 * @note 
 */
T_DjiReturnCode WirisCamera_Reboot(void)
{
    uint8_t ackData[WIRIS_PROT_MAX_DATA_LEN] = {0};
    uint8_t ackDataLen = 0;
        
    if(WirisProt_SendGetAck(s_pWirisProt, "REBT", NULL, ackData, &ackDataLen, WIRIS_TIMEOUT_SEND_GET_ACK) 
        != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_RAW_CODE_TIMEOUT;
    }
    
    /*!< Check lengh ack data is valid*/
    if(ackDataLen > 0) {
        if( ackData[0] == 0x4F && ackData[1] == 0x4B && ackData[2] == 0x0A){
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
}
/**
  * @} // 
  */


/*!<====================== WIRIS FUNCTIONS =================================== */

/*!<========================= HARDWARE_PLATFORM_TASK =================================== */

/** @addtogroup HARDWARE_PLATFORM_TASK
  * @{
  */
/**
 * @brief Low level PSDK frame send function.
 * @param pSendData Pointer to frame to be sent.
 * @param dataLen Frame length.
 * @return PSDK function process state.
 */
static T_DjiReturnCode PsdkCamera_Write(const uint8_t *pSendData, uint16_t dataLen)
{
    UART_Write(PSDK_COMM_WITH_CAMERA_UART_NUM, pSendData, dataLen);
    
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/**
 * @brief Low level PSDK frame read function.
 * @param pReadData Pointer to frame to be read.
 * @param dataLen Frame length.
 * @retval -1: Read error.
 *          not -1: Counts of bytes read actually.
 */
static int PsdkCamera_Read(uint8_t *pReadData, uint16_t dataLen)
{
    int res;

    res = UART_Read(PSDK_COMM_WITH_CAMERA_UART_NUM, pReadData, dataLen);
    
    return res;
}



/**
 * @brief PSDK data receive task function.
 * @note All callback function process in this task.
 * @param parameter
 * @return None.
 */
static void *WirisCameraRecTask(void *parameter)
{
    uint32_t step = 0;
    
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    while (1) {
        /*!< BaurdRare (Bd) = 115200  -- > 1Bd = 1 bit/s  --> 8.6us
            Uart Frame 12 bits. Time to read = 12*8.68 = 104.2 us --> frep = 9,6K
            Queue 1024 bytes. Delay 1ms is compatible.
         */
        
        int realLen = PsdkCamera_Read(s_uartRecBuf, sizeof(s_uartRecBuf));
        
        if (realLen > 0) {
            //===================== WIRIS Recievie Function ==============================//
            //all callbacks process in this thread.
            WirisProt_ProcessReceiveData(s_uartRecBuf, realLen);
        }
        
        osalHandler->TaskSleepMs(1000 / WIRIS_CAMERA_TASK_FREQ);
        step++;
        
        if (USER_UTIL_IS_WORK_TURN(step, 10, WIRIS_CAMERA_TASK_FREQ)) {
            
            if(s_cameraRunningState == CAMERA_RUNNING_STATE_PROCESS_EVENT) {
//                DebugWarning("%d/%d/%d-%d:%d:%d", WIRISCamera->GPSData.aircraftTime.year, 
//                WIRISCamera->GPSData.aircraftTime.month,
//                WIRISCamera->GPSData.aircraftTime.day,
//                WIRISCamera->GPSData.aircraftTime.hour,
//                WIRISCamera->GPSData.aircraftTime.minute,
//                WIRISCamera->GPSData.aircraftTime.second);
                
                /*!< If time is valid and the camera is waiting for setting the date & time*/
                if(WIRISCamera->GPSData.aircraftTime.year == 2021 && !WIRISCamera->isSetDateTime) {
                    GsdkWiris_CameraSetEvent(CAM_EVENT_SET_DEFAULT_PARAM);
                }
                
                /*!< Check GPS is ready */
                if(WIRISCamera->isReadyGPS && WIRISCamera->GPSData.gpsQuality) {
                    
//                    uint32_t timeNowMs = 0;
//                    osalHandler->GetTimeMs(&timeNowMs);

//                    if(timeNowMs - s_pWirisProt->lastTimeGPS > 2000 || s_pWirisProt->isImmediated)
                    {
                        /*!< When Get State will ignore GPS */
                        xEventGroupSetBits(s_eventCamera, CAM_EVENT_SET_GPS);
                        
                        /*!< Clear */
//                        s_pWirisProt->isImmediated = false;
                        WIRISCamera->isReadyGPS = false;
                    }
                }
            }
        }
    }
}

/**
 * @brief PSDK data receive task function.
 * @note All callback function process in this task.
 * @param parameter
 * @return None.
 */
static void *WirisCommandProcessTask(void *parameter)
{
    static uint32_t     step = 0;    
    uint8_t             countMsg = 0;

    int8_t tempVisibleZoomIndex = 0;
    
    /*!< Attempt to create the event group*/
    s_eventCamera = xEventGroupCreate();
    
    T_DjiReturnCode djiReturnCode;
    
    /*!< Event bit to check */
    EventBits_t event;
    
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    
    while(1) {
        
        osalHandler->TaskSleepMs(1000 / WIRIS_CAMERA_TASK_FREQ);
        step++;
        
        /*!< Check camera running state */
        if(s_cameraRunningState == CAMERA_RUNNING_STATE_IDLE) {
            
            /*!< Reset */
            WIRISCamera->isCamConnected = false;
            
            /*!< Switch to sending commands to check connection*/
            s_cameraRunningState = CAMERA_RUNNING_STATE_CHECK_CONNECTION;
        }
        else if(s_cameraRunningState == CAMERA_RUNNING_STATE_CHECK_CONNECTION) {
            
            /*!< Send command to check */
            if(WirisCamera_isWirisConnected() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                /*!< Camera */
                WIRISCamera->isCamConnected = true;
            }
            else {
                char sn[10] = {0};
                    
                if(WirisCamera_GetSerialNumber(sn) ==DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    
                    DebugInfo("Serial Number: %s", sn);
                    
                    WIRISCamera->isCamConnected = true;
                }
            }
            
            /*!< when camera is connected then reset SOM */ 
            if(WIRISCamera->isCamConnected) {
                /*!< Switch to activate key for communication */
                s_cameraRunningState = CAMERA_RUNNING_STATE_RESET_SOM;
            }
            else {
                osalHandler->TaskSleepMs(1000);
            }
            
            DebugWarning("Waiting for camera...!");
        }
        else if(s_cameraRunningState == CAMERA_RUNNING_STATE_RESET_SOM) {
            
//            if(PsdkLinux_GetSomState() != SOM_INIT) {
//            
//                DebugWarning("Som is reseting !");
//                
//                 /*!< Reset SOM */
//                PsdkSom_WriteHardwareResetPin(SOM_HW_RESET_MANAGEMENT_PIN_STATE_RESET);
//                PsdkOsal_TaskSleepMs(20);
//                PsdkSom_WriteHardwareResetPin(SOM_HW_RESET_MANAGEMENT_PIN_STATE_SET);
//                PsdkOsal_TaskSleepMs(20);
//                
//                PsdkLinux_SetSomState();
//            }
//            
//            /*!< Check whether that pins is high */
//            if(PsdkSom_GetState() == SOM_HW_RESET_MANAGEMENT_PIN_STATE_SET) {
                /*!< Switch to activate key for communication */
                s_cameraRunningState = CAMERA_RUNNING_STATE_ACTIVE_KEY;
//            }
        }
        else if(s_cameraRunningState == CAMERA_RUNNING_STATE_ACTIVE_KEY) {
            
            DebugWarning("CAM is activated !");
            
            /*!< Check camera is activated completely */
            if(WirisCamera_ActivateCamera() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                /*!< Switch to set param */
                s_cameraRunningState = CAMERA_RUNNING_QUERY_CAMERA_SETTING;
            }
        }
        else if(s_cameraRunningState == CAMERA_RUNNING_QUERY_CAMERA_SETTING) {
            
            /*!< Set default main camera for display */
            if(WirisCamera_SetMainCamera(SMCA_VISIBLE) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                DebugError("Set Main Camera Error");
            }
            WIRISCamera->mainCam = SMCA_VISIBLE;

            /*!<TODO: Get Model camera */
            WIRISCamera->model = MODEL_WIRIS_SEC;
            WIRISCamera->layout = SLAY_WWS_FULLSCREEN;
            
            if(WIRISCamera->model == MODEL_WIRIS_SEC) {
                if(WirisCamera_SetLayout(SLAY_WWS_FULLSCREEN) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError("Set SLAY_WWS_FULLSCREEN Error");
                    WIRISCamera->layout = SLAY_WWS_FULLSCREEN;
                }
            } 
            else if(WIRISCamera->model == MODEL_WIRIS_PRO) {
                 if(WirisCamera_SetLayout(SLAY_WWS_FULLSCREEN) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    WIRISCamera->layout = SLAY_WWP_FULLSCREEN;
                }
            }
            
            WIRISCamera->transparency       = 10;
            WIRISCamera->timeStabilization  = 10; /*!< 1 second */
            
            /*!< Reset state to get all data first*/
            s_requestCamStatus = 0xFFFFFFF;
            
            /*!< Switch to process commands*/
            s_cameraRunningState = CAMERA_RUNNING_STATE_PROCESS_EVENT;
        }
        else if(s_cameraRunningState == CAMERA_RUNNING_STATE_PROCESS_EVENT) {
        /*!<----------------------Process Event group --------------------------------
        Block to for any event to be set within the event group. 
        Clear the bit before existing block to wait for one or more bits to be set within a previously created event group. */
            
            event = xEventGroupWaitBits(s_eventCamera, CAM_EVENT_ALLS, pdTRUE, pdFALSE, portMAX_DELAY);
                
            if(event & CAM_EVENT_SET_SAY_HI) {
                /*!< Send event say hi*/
                if(WirisCamera_isWirisConnected() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    WIRISCamera->isCamConnected = true;
                }
                else {
                    WIRISCamera->isCamConnected = false;
                }
            }
            
            if(event & CAM_EVENT_SET_REBOOT) {
                
                char sn[10] = {0};
                
                if(WirisCamera_GetSerialNumber(sn) ==DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    
                    DebugInfo("Serial Number: %s", sn);
                    
                    WIRISCamera->isCamConnected = true;
                }
            }
            
            /*!< Reset camera default at the first time open camera*/
            if(event & CAM_EVENT_SET_DEFAULT_PARAM) {
                if(WirisCamera_SetDataAndTime(&WIRISCamera->GPSData.aircraftTime) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError("Set TIME WIRIS Error");
                    WIRISCamera->isSetDateTime = false;
                }
                else {
                    T_DjiTimeSyncAircraftTime wirisTime = {0};
                    
                    /*!< 1980/01/06-00:09:16 */
                    if(WirisCamera_GetDataAndTime(&wirisTime) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        DebugError("GOT Time FAILED ");
                    }
                    else {
                        /*!< Check whether the time has been set to camera correctly */
                        if(WIRISCamera->GPSData.aircraftTime.year == wirisTime.year) {
                            WIRISCamera->isSetDateTime = true;
                        }
                        else {
                            DebugError("Aircraft & Camera is different date&time ");
                        }
                    }
                }
            }
            
            /*!< This event will be call when start recording or stop recording */
            if(event & CAM_EVENT_GET_SYSTEM_STATE) {
                
                if(s_requestCamStatus & REQUEST_CAM_VIDEO_STATUS) {
                    s_requestCamStatus &= (~REQUEST_CAM_VIDEO_STATUS);
                    
                    /*!< Check camera is recording */
                    if(WirisCamera_IsRecording(&WIRISCamera->isRecording) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        DebugError("Check Record state Failed");
                        WIRISCamera->isRecording = false;
                    }
                }
                
                if(s_requestCamStatus & REQUEST_CAM_CAPTURE_STATUS) {
                    s_requestCamStatus &= (~REQUEST_CAM_CAPTURE_STATUS);
                    
                    /*!< Check camera is recording */
                    if(WirisCamera_IsCapturing(&WIRISCamera->isCapturing) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        DebugError("Check Record state Failed");
                    }
                }
                
                if(s_requestCamStatus & REQUEST_CAM_ZOOM_VALUE) {
                    s_requestCamStatus &= (~REQUEST_CAM_ZOOM_VALUE);
                    
                    WirisCamera_GetThermalZoomVal(&WIRISCamera->thermalZoomIdx, &WIRISCamera->thermalZoomRatio);
                    WirisCamera_GetVisibleZoomVal(&WIRISCamera->visibleZoomIdx, &WIRISCamera->visibleZoomRatio);
                    
                    /*!< Update zoom index */
                    tempVisibleZoomIndex = WIRISCamera->visibleZoomIdx;
                }

                if(s_requestCamStatus & REQUEST_CAM_PALETTE_VALUE) {
                    s_requestCamStatus &= (~REQUEST_CAM_PALETTE_VALUE);
                    
                    /*!< Send command */
                    if(WirisCamera_GetPalette(&WIRISCamera->paletteIdx) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        WIRISCamera->paletteIdx = 0;
                        DebugError("Get Palette Failed");
                    }
                }
                
                if(s_requestCamStatus & REQUEST_CAM_TIME_STATBI_VALUE) {
                    s_requestCamStatus &= (~REQUEST_CAM_TIME_STATBI_VALUE);
                    
                    if(WirisCamera_GetTimeStabilization(&WIRISCamera->timeStabilization) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        DebugError("Check Time state Failed");
                    }
                    
                }
                
                if(s_requestCamStatus & REQUEST_CAM_HOT_REJECTION_VALUE) {
                    s_requestCamStatus &= (~REQUEST_CAM_HOT_REJECTION_VALUE);
                    
                    if(WirisCamera_GetHotRejection(&WIRISCamera->hotRejection) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                         WIRISCamera->hotRejection = 0;
                         DebugError("Get hot Rejection Failed");
                    }
                }
                
                if(s_requestCamStatus & REQUEST_CAM_COOL_REJECTION_VALUE) {
                    s_requestCamStatus &= (~REQUEST_CAM_COOL_REJECTION_VALUE);
                    
                    if(WirisCamera_GetCoolRejection(&WIRISCamera->coolRejection) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        WIRISCamera->coolRejection = 0;
                        DebugError("Get Cool Rejection Failed");
                    }
                }
                
                if(s_requestCamStatus & REQUEST_CAM_ALARM_MODE_VALUE) {
                    s_requestCamStatus &= (~REQUEST_CAM_ALARM_MODE_VALUE);
                    
                    if(WirisCamera_GetAlarmMode(&WIRISCamera->alarmMode) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        DebugError("Get alarm failed");
                    }
                }
                
                if(s_requestCamStatus & REQUEST_CAM_ALARM_COLOR_VALUE) {
                    s_requestCamStatus &= (~REQUEST_CAM_ALARM_COLOR_VALUE);
                    
                    if(WirisCamera_GetAlarmColor(&WIRISCamera->alarmColor) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        DebugError("Get alarm failed");
                    }
                }
                
                if(s_requestCamStatus & REQUEST_CAM_THRESHOLD_VALUE) {
                    s_requestCamStatus &= (~REQUEST_CAM_THRESHOLD_VALUE);
                    
                    if(WirisCamera_GetAlarmValue(&WIRISCamera->thresholdBelow, &WIRISCamera->thresholdAbove) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) { 
                        DebugError("GetAlarmValue failed");
                    }
                }
                
                if(s_requestCamStatus & REQUEST_CAM_MEMORY_STATUS) {
                    s_requestCamStatus &= (~REQUEST_CAM_MEMORY_STATUS);
                    
                    if(WirisCamera_GetMemoryStatus(&WIRISCamera->memoryStatus)!= DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) { 
                        DebugError("Mem status failed");
                    }
                }
                
                if(s_requestCamStatus & REQUEST_CAM_MEMORY_SIZE) {
                    s_requestCamStatus &= (~REQUEST_CAM_MEMORY_SIZE);
                    
                    if(WirisCamera_GetMemorySize(&WIRISCamera->ssdSize, &WIRISCamera->flashDriveSize, &WIRISCamera->sdCardSize)!= DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) { 
                        DebugError("Mem status failed");
                    }
                }
                
                if(s_requestCamStatus & REQUEST_CAM_MEMORY_FREE) {
                    s_requestCamStatus &= (~REQUEST_CAM_MEMORY_FREE);
                    
                    if(WirisCamera_GetMemoryFree(&WIRISCamera->ssdFree, &WIRISCamera->flashDriveFree, &WIRISCamera->sdCardFree)!= DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) { 
                        DebugError("Mem status failed");
                    }
                }
            }
            
            if(event & CAM_EVENT_SET_ZOOM_IN) {
                
                /*!< Calculate value to set to camera */
                tempVisibleZoomIndex += 1;
                
                if(tempVisibleZoomIndex > 16) {
                    tempVisibleZoomIndex = 16;
                }
                
                if(WIRISCamera->layout != SLAY_WWS_PIP) {
                     if(WIRISCamera->mainCam == SMCA_THERMAL) {
                          WirisCamera_ZoomIn();
                    }
                    else if(WIRISCamera->mainCam == SMCA_VISIBLE) {
                        WirisCamera_SetVisibleZoomIndex(tempVisibleZoomIndex);
                    }
                }
                else if(WIRISCamera->layout == SLAY_WWS_PIP){
                    if(tempVisibleZoomIndex < 11) {
                        WirisCamera_ZoomIn();
                    }
                    else {
                        WirisCamera_SetVisibleZoomIndex(tempVisibleZoomIndex);
                    }
                }
                
                /*!< Change status to send request */
                s_requestCamStatus |= REQUEST_CAM_ZOOM_VALUE;
                
                /*!< Set event to start proccess*/
                GsdkWiris_CameraSetEvent(CAM_EVENT_GET_SYSTEM_STATE);
            }
            
            if(event & CAM_EVENT_SET_ZOOM_OUT ) {
                /*!< Calculate value to set to camera */
                tempVisibleZoomIndex -= 1;
                
                if(tempVisibleZoomIndex < 0) {
                    tempVisibleZoomIndex = 0;
                }
                
                if(WIRISCamera->layout != SLAY_WWS_PIP) {
                     if(WIRISCamera->mainCam == SMCA_THERMAL) {
                        WirisCamera_ZoomOut();
                    }
                    else if(WIRISCamera->mainCam == SMCA_VISIBLE) {
                        WirisCamera_SetVisibleZoomIndex(tempVisibleZoomIndex);
                    }
                }
                else if(WIRISCamera->layout == SLAY_WWS_PIP){
                     if(tempVisibleZoomIndex < 11) {
                        WirisCamera_ZoomOut();
                    }
                    else {
                        WirisCamera_SetVisibleZoomIndex(tempVisibleZoomIndex);
                    }
                }
                
                /*!< Change status to send request */
                s_requestCamStatus |= REQUEST_CAM_ZOOM_VALUE;
                
                /*!< Set event to start proccess*/
                GsdkWiris_CameraSetEvent(CAM_EVENT_GET_SYSTEM_STATE);
            }
            
            /*!< Reset Zoom */
            if(event & CAM_EVENT_RESET_DIGITAL_ZOOM_FACTOR) {
                if(WIRISCamera->layout != SLAY_WWS_PIP && WIRISCamera->mainCam == SMCA_THERMAL) {
                    /*!< Reset Zoom thermal*/
                    WirisCamera_SetThermalZoomIndex(0);
                } else if(WIRISCamera->layout != SLAY_WWS_PIP && WIRISCamera->mainCam == SMCA_VISIBLE) {
                    /*!< Reset Visible Zoom*/
                    WirisCamera_SetVisibleZoomIndex(0);
                } else {
                    /*!< Reset Zoom thermal*/
                    WirisCamera_SetThermalZoomIndex(0);
                    
                    /*!< Reset Visible Zoom*/
                    WirisCamera_SetVisibleZoomIndex(0);
                }
                tempVisibleZoomIndex = 0;
                
                /*!< Change status to send request */
                s_requestCamStatus |= REQUEST_CAM_ZOOM_VALUE;
                
                /*!< Set event to start proccess*/
                GsdkWiris_CameraSetEvent(CAM_EVENT_GET_SYSTEM_STATE);
            }
            
            /*!< Event to capture camera */
            if(event & CAM_EVENT_SET_START_PHOTO) {
                /*!< Capture photo */
                WirisCamera_Capture();
                
                /*!< Enable get request */
                s_requestCamStatus |= REQUEST_CAM_CAPTURE_STATUS;
                
                /*!< Need to send GPS immediately */
                s_pWirisProt->isImmediated = true;
                
                /*!< Set event to start proccess*/
                GsdkWiris_CameraSetEvent(CAM_EVENT_GET_SYSTEM_STATE);
            }

            if(event & CAM_EVENT_SET_START_RECORD) {
                if(WirisCamera_RecordingStart() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError("Set Recording Failed");
                }
                
                /*!< Enable get request */
                s_requestCamStatus |= REQUEST_CAM_VIDEO_STATUS;
                
                /*!< Set event to start proccess*/
                GsdkWiris_CameraSetEvent(CAM_EVENT_GET_SYSTEM_STATE);
            }
            
            if(event & CAM_EVENT_SET_STOP_RECORD) {
                if(WirisCamera_RecordingFinish() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError("Set Recording Finish Failed");
                }
                
                /*!< Enable get request */
                s_requestCamStatus |= REQUEST_CAM_VIDEO_STATUS;
                
                /*!< Set event to start proccess*/
                GsdkWiris_CameraSetEvent(CAM_EVENT_GET_SYSTEM_STATE);
            }
           
            if(event & CAM_EVENT_SET_LAYOUT) {
                /*!<TODO: Get Model camera */
                WIRISCamera->model = MODEL_WIRIS_SEC;
                
                if(WIRISCamera->model == MODEL_WIRIS_SEC) {
                    if(WirisCamera_SetLayout(WIRISCamera->layout) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        DebugError("Set layout Failed");
                    }
                } 
                else if(WIRISCamera->model == MODEL_WIRIS_PRO) {
                    if(WirisCamera_SetLayout(SLAY_WWS_FULLSCREEN) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                         DebugError("Set layout Failed");
                    }
                }
            }
            
            /*!< Set camera command based on flag */
            if(event & CAM_EVENT_SET_MAINCAM) {
                /*!< Set default main camera for display */
                if(WirisCamera_SetMainCamera(WIRISCamera->mainCam) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError("Set Main Camera Failed");
                }
            }
            
            /*!< Set thermal transparency */
            if(event & CAM_EVENT_SET_OPACITY) {
                if(WirisCamera_SetThermalCameraTransparency(WIRISCamera->transparency) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError("Set Thermal Opacity Failed");
                }
            }
            
            /*!< Set palette */
            if(event & CAM_EVENT_SET_PALETTE) {
                /*!< Set main camera */
                if(WirisCamera_SetPaletteIndex(WIRISCamera->paletteIdx) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError("Set Set Palette Failed");
                }
                
                /*!< Change status to send request */
                s_requestCamStatus |= REQUEST_CAM_PALETTE_VALUE;
                
                /*!< Set event to start proccess*/
                GsdkWiris_CameraSetEvent(CAM_EVENT_GET_SYSTEM_STATE);
            }
            
             /*!< Set palette */
            if(event & CAM_EVENT_SET_ALARM_MODE) {
                /*!< Set alarm mode value */
                if(WirisCamera_SetAlarmMode(WIRISCamera->alarmMode) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError("Set Alarm mode Failed");
                }
                
                /*!< Change status to send request */
                s_requestCamStatus |= REQUEST_CAM_ALARM_MODE_VALUE;
                
                /*!< Set event to start proccess*/
                GsdkWiris_CameraSetEvent(CAM_EVENT_GET_SYSTEM_STATE);
            }
            
            if(event & CAM_EVENT_SET_ALARM_COLOR) {
                if(WirisCamera_SetAlarmColor(WIRISCamera->alarmColor) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError("Set Alarm Color Failed");
                }
                
                /*!< Change status to send request */
                s_requestCamStatus |= REQUEST_CAM_ALARM_COLOR_VALUE;
                
                /*!< Set event to start proccess*/
                GsdkWiris_CameraSetEvent(CAM_EVENT_GET_SYSTEM_STATE);
            }
            
            if(event & CAM_EVENT_SET_ALARM_VALUE) {
                if(WirisCamera_SetAlarmValue(WIRISCamera->thresholdBelow, 
                            WIRISCamera->thresholdAbove) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError("Set Alarm value Failed");
                }
                            
                /*!< Change status to send request */
                s_requestCamStatus |= REQUEST_CAM_THRESHOLD_VALUE;
                
                /*!< Set event to start proccess*/
                GsdkWiris_CameraSetEvent(CAM_EVENT_GET_SYSTEM_STATE);
            }
            

            if(event & CAM_EVENT_SET_TIME_STABI) {
                if(WirisCamera_SetTimeStabilization(WIRISCamera->timeStabilization) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError("Set Time stabilization Failed");
                }
                
                /*!< Change status to send request */
                s_requestCamStatus |= REQUEST_CAM_TIME_STATBI_VALUE;
                
                /*!< Set event to start proccess*/
                GsdkWiris_CameraSetEvent(CAM_EVENT_GET_SYSTEM_STATE);
            }

            if(event & CAM_EVENT_SET_COOL_REJECT) {
                /*!< Set value for camera */
                if(WirisCamera_SetCoolRejection(WIRISCamera->coolRejection) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError("Set Cool Rejection Failed");
                }
                
                /*!< Change status to send request */
                s_requestCamStatus |= REQUEST_CAM_COOL_REJECTION_VALUE;
                
                /*!< Set event to start proccess*/
                GsdkWiris_CameraSetEvent(CAM_EVENT_GET_SYSTEM_STATE);
            }

            if(event & CAM_EVENT_SET_HOT_REJECT) {
                /*!< Set value for camera */
                if(WirisCamera_SetHotRejection(WIRISCamera->hotRejection) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    DebugError("Set Hot Rejection Failed");
                }
                
                /*!< Change status to send request */
                s_requestCamStatus |= REQUEST_CAM_HOT_REJECTION_VALUE;
                
                /*!< Set event to start proccess*/
                GsdkWiris_CameraSetEvent(CAM_EVENT_GET_SYSTEM_STATE);
            }
            
            if(event & CAM_EVENT_SET_GPS) {
                /*!< Get data from geotaging*/
                WirisCamera_SetGPS(&WIRISCamera->GPSData);
            }
        }
        else if(s_cameraRunningState == CAMERA_RUNNING_STATE_ERROR) {
            
            /*!< Clear */
            WIRISCamera->isCamConnected = false;
            s_cameraRunningState = CAMERA_RUNNING_STATE_IDLE;
        }
    }
}

T_DjiReturnCode GsdkWiris_CameraInit(void)
{
    T_DjiReturnCode djiReturnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    /*!< Init UART*/
    UART_Init(PSDK_COMM_WITH_CAMERA_UART_NUM, PSDK_COMM_WITH_CAMERA_UART_BAUD);
    
   /*!<------------------ WIRIS PROTOCOL INIT --------------------------------*/
    djiReturnCode = WirisCameraProto_Init();
    if(djiReturnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("Wiris Camera proto init Failed");
        return djiReturnCode;
    }
    
    /*!< Register send function */
    djiReturnCode = WirisCameraProto_RegSendDataFunc(PsdkCamera_Write);
    if(djiReturnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        DebugError("Wiris Camera proto set callback Failed");
        return djiReturnCode;
    }
    
    /*!< Create the camera wiris receive data thread */
    if(osalHandler->TaskCreate("wws_recv_task", WirisCameraRecTask, 
        WIRIS_CAMERA_RECV_TASK_STACK_SIZE, NULL, &s_wirisRecvThread) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
        DebugError("user camera task create error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    
    /*!< Create the camera wiris command process thread */
    if(osalHandler->TaskCreate( "wws_cmd_task", WirisCommandProcessTask,
        WIRIS_CAMERA_CMD_TASK_STACK_SIZE, NULL, &s_wirisCmdThread) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
        DebugError("user camera task create error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    
    return djiReturnCode;
}

T_DjiReturnCode GsdkWiris_CameraDeInit(void)
{
    
}

/**
 * @brief Function set camera event 
 * @details This structure type is used to
 * @note 
 */
T_DjiReturnCode GsdkWiris_CameraSetEvent(const uint32_t uxBitsToSet)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    // Was the event group created successfully?
    if( s_eventCamera != NULL && s_cameraRunningState == CAMERA_RUNNING_STATE_PROCESS_EVENT) {
         /*!< Set event to check camera is recording*/
        xEventGroupSetBits(s_eventCamera, uxBitsToSet);

        /*!< Reset time */
//        PsdkOsal_GetTimeMs(&s_pWirisProt->lastTimeGPS);
        osalHandler->GetTimeMs(&s_pWirisProt->lastTimeGPS);
        
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
}

/**
  * @} // 
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
