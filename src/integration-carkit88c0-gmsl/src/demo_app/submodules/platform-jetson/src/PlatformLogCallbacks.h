/*
 * File: PlatformLogCallbacks.h
 * Description: Platform-dependant iRFE logging API implementation for the NVIDIA Jetson
 * Project: MIMOrad
 * Created Date: Tuesday, 23 April 2024, 10:07
 * Author: Daniel Klepatsch (daniel.klepatsch@silicon-austria.com)
 * ---------------
 * Last Modified: Tuesday, 23 April 2024, 16:40
 * Modified By: Daniel Klepatsch
 * ---------------
 * Copyright (c) 2024 Silicon Austria Labs GmbH
 */

#ifndef PLATFORMLOGCALLBACKS_H
#define PLATFORMLOGCALLBACKS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void PlatformLogCallbacks_error(const char *msg);
    void PlatformLogCallbacks_warning(const char *msg);
    void PlatformLogCallbacks_info(const char *msg);

    // Suppress all log messages if silent is true
    void PlatformLogCallbacks_setSilent(bool silent);

    // Get the current mode of log suppression
    bool PlatformLogCallbacks_getSilent();

#ifdef __cplusplus
}
#endif

#endif  // PLATFORMLOGCALLBACKS_H