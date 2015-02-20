/*
 * Log.h
 *
 *  Created on: Feb 19, 2015
 *      Author: CBurlacu
 */

#ifndef LOG_H_
#define LOG_H_

#include <stdint.h>

typedef enum
{
   Log_SourceInvalid    = 0x00,
   Log_SourceCAN,
   Log_SourceLEDTask,
   Log_SourceCanTxInt,
   Log_SourceUartTask,
   Log_SourceInit,
   Log_SourceRTCInterrupt,
   Log_SourceCount,
   Log_SourceReserved = 0x7F
}LogSource;

typedef enum
{
   Log_TypeDebug        = 0x00,
   Log_TypeWarning      = 0x01,
   Log_TypeError        = 0x02,
   Log_TypeFatal        = 0x03,
   Log_TypeCount,
   Log_TypeReserved     = 0x7F
}LogType;


typedef enum
{
   Log_Ok               = 0x00,
   Log_InvalidQueue     = 0x010000,
   Log_UknownError      = 0x020000,
   Log_reserved         = 0x7FFFFFFF  ///< prevent from enum down-size compiler optimization.
}LogStatus;

typedef struct __info
{
   char *      description;
   uint32_t    length;
}LogInfo;


typedef LogStatus (*logWriteCallback)(uint32_t msg);


void logInit(logWriteCallback callback);

/**
 * @brief   Add a new message to log queue
 *          Each message have 4 bytes:
 *           6 bits: source
 *           2 bits: message type
 *          24 bits: user data
 *
 * @param source The source of the event message
 * @param type Type of the message
 * @param evId The message id as defined by the user: each component should have its own ids
 * @return the status of the log
 */
LogStatus logMsg(LogSource source, LogType type, uint32_t evId);

void logGenericTaskHandler(void const *);

#endif /* LOG_H_ */
