/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */

#ifndef IFXRFE_LOGCALLBACKS_T_H
#define IFXRFE_LOGCALLBACKS_T_H 1

/// \brief Provides the log function callbacks for the IfxRfe
/// \note   Functions can not block, need to behave asynchronously from the IfxRfe point of view
///         The passed message is on the stack, it needs to be copied immediately, otherwise values will be overwritten
///         The messages are passed line-by-line, without line-feed or carriage return. The provider needs to take
///         care of appending the end of line characters or error/warning/info information.
///         If any of the logs are not needed, a NULL can be configured for the corresponding pointer.
typedef struct
{
    void (*errorLog)(const char *msg);    ///> asynchronous error message callback
    void (*warningLog)(const char *msg);  ///> asynchronous warning message callback
    void (*infoLog)(const char *msg);     ///> asynchronous info message callback
} IfxRfe_logCallbacks_t;

#endif  //IFXRFE_LOGCALLBACKS_T_H