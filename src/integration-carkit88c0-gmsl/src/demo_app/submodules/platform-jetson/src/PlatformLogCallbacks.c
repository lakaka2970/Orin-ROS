/*
 * File: PlatformLogCallbacks.c
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

#include "PlatformLogCallbacks.h"

#include "Util.h"

static bool m_silent = false;

// TODO: implement async

void PlatformLogCallbacks_error(const char *msg)
{
    if (m_silent)
        return;
    printf("ERROR: %s\n", msg);
}

void PlatformLogCallbacks_warning(const char *msg)
{
    if (m_silent)
        return;
    printf("WARNING: %s\n", msg);
}

void PlatformLogCallbacks_info(const char *msg)
{
    if (m_silent)
        return;
    printf("INFO: %s\n", msg);
}

void PlatformLogCallbacks_setSilent(bool silent)
{
    m_silent = silent;
}

bool PlatformLogCallbacks_getSilent()
{
    return m_silent;
}
