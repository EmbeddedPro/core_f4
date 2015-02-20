/*
 * Log.c
 *
 *  Created on: Feb 19, 2015
 *      Author: CBurlacu
 */
#include "Log.h"

#include "cmsis_os.h"
extern osMessageQId logQueueHandle;

static LogStatus dummyWriteCallback(uint32_t msg) { return Log_Ok; }

static logWriteCallback logWriteCallbackPtr = dummyWriteCallback;


LogInfo logSourceNames [] =
{
         {"[N/A          ]", 15},
         {"[CAN          ]", 15},
         {"[Led Task     ]", 15},
         {"[CanTxInt     ]", 15},
         {"[UART task    ]", 15},
         {"[Initializing ]", 15},
};

LogInfo logTypeNames [] =
{
         {"[Debug]", 7},
         {"[Warn ]", 7},
         {"[Error]", 7},
         {"[Fatal]", 7},
         {"[N/A  ]", 7}
};

void logInit(logWriteCallback callback)
{
   logWriteCallbackPtr = callback;
}

LogStatus logMsg(LogSource source, LogType type, uint32_t evId)
{
   LogStatus retVal = Log_Ok;
   if (0 != logQueueHandle)
   {
      uint8_t meta = (source << 2) | (type & 0x3);
      retVal = (LogStatus)osMessagePut(logQueueHandle, (meta << 24) | (evId & 0x00FFFFFF), 1);
   }
   else
      retVal = Log_InvalidQueue;

   return retVal;
}

void logGenericTaskHandler(void const* arg)
{
   while(1)
   {
      osEvent ev = osMessageGet(logQueueHandle, osWaitForever);
      if(ev.status != osEventMessage)
      {
         osDelay(10);
      }
      else
      {
         logWriteCallbackPtr(ev.value.v);
      }
   }
}
