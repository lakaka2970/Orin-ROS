/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#include "IfxRfe_Logger.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>


static IfxRfe_logCallbacks_t _logInterface;

void loggerInit(IfxRfe_logCallbacks_t logInterface)
{
    _logInterface = logInterface;
}

void errorLogFmt(const char *format, ...)
{
    char fmtLogBuffer[IFXRFE_FMTLOG_BUFFER_SIZE];
    if (_logInterface.errorLog == NULL)
        return;  //save runtime if output not needed

    va_list args;
    va_start(args, format);
    vsnprintf(fmtLogBuffer, IFXRFE_FMTLOG_BUFFER_SIZE, format, args);
    va_end(args);
    errorLog(fmtLogBuffer);
}

void errorLog(const char *msg)
{
    if (_logInterface.errorLog != NULL)
    {
        _logInterface.errorLog(msg);
    }
}

void warningLogFmt(const char *format, ...)
{
    char fmtLogBuffer[IFXRFE_FMTLOG_BUFFER_SIZE];
    if (_logInterface.warningLog == NULL)
        return;  //save runtime if output not needed

    va_list args;
    va_start(args, format);
    vsnprintf(fmtLogBuffer, IFXRFE_FMTLOG_BUFFER_SIZE, format, args);
    va_end(args);
    warningLog(fmtLogBuffer);
}

void warningLog(const char *msg)
{
    if (_logInterface.warningLog != NULL)
    {
        _logInterface.warningLog(msg);
    }
}

void infoLogFmt(const char *format, ...)
{
    char fmtLogBuffer[IFXRFE_FMTLOG_BUFFER_SIZE];
    if (_logInterface.infoLog == NULL)
        return;  //save runtime if output not needed

    va_list args;
    va_start(args, format);
    vsnprintf(fmtLogBuffer, IFXRFE_FMTLOG_BUFFER_SIZE, format, args);
    va_end(args);
    infoLog(fmtLogBuffer);
}

void infoLog(const char *msg)
{
    if (_logInterface.infoLog != NULL)
    {
        _logInterface.infoLog(msg);
    }
}
