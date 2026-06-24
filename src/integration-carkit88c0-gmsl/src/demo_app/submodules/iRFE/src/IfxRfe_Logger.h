/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#ifndef IFXRFE_LOGGER_H
#define IFXRFE_LOGGER_H 1

#include "IfxRfe_LogCallbacks_t.h"

#define IFXRFE_FMTLOG_BUFFER_SIZE 256

void loggerInit(IfxRfe_logCallbacks_t logInterface);
void errorLogFmt(const char *format, ...);
void errorLog(const char *msg);
void warningLogFmt(const char *format, ...);
void warningLog(const char *msg);
void infoLogFmt(const char *format, ...);
void infoLog(const char *msg);


#endif  //IFXRFE_LOGGER_H