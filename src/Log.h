/*
 * Log.h
 *
 *  Created on: Feb 19, 2015
 *      Author: CBurlacu
 */

#ifndef LOG_H_
#define LOG_H_

#include <stdint.h>

typedef struct __log
{
   uint32_t	length;
   char*		message;
}LogMessage;

void logMsg(const char* msg, uint32_t len);

#endif /* LOG_H_ */
