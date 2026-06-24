/*
 * File: PlatformTime.c
 * Description: Platform-dependant iRFE Time API implementation for the NVIDIA Jetson
 * Project: MIMOrad
 * Created Date: Tuesday, 23 April 2024, 10:07
 * Author: Daniel Klepatsch (daniel.klepatsch@silicon-austria.com)
 * ---------------
 * Last Modified: Friday, 03 May 2024, 11:01
 * Modified By: Daniel Klepatsch
 * ---------------
 * Copyright (c) 2024 Silicon Austria Labs GmbH
 */

#include <errno.h>
#include <string.h>
#include <time.h>

#include "Util.h"

#include "PlatformTime.h"

int64_t PlatformTime_getDeadLine(uint32_t timeoutMs)
{
    int64_t now      = PlatformTime_now();
    int64_t deadline = now + (timeoutMs * 1000);

    DBG_PRINTF_L1("DEBUG from %s at %s:%d:\n", __func__, __FILE__, __LINE__);
    DBG_PRINTF_L1("    now       = %ld\n", now);
    DBG_PRINTF_L1("    timeoutMs = %d\n", timeoutMs);
    DBG_PRINTF_L1("    deadline  = %ld\n\n", deadline);

    return deadline;
}

int64_t PlatformTime_now(void)
{
    struct timespec ts;
    int sysret = clock_gettime(CLOCK_MONOTONIC, &ts);
    if (sysret != 0)
    {
        char buf[64];
        sprintf(buf, "clock_gettime() returned %d, errno = %d (%s)", sysret, errno, strerror(errno));
        EXIT_WITH_ERROR_MSG(buf);
    }
    int64_t time_us = (ts.tv_sec * 1000000) + (ts.tv_nsec / 1000);

    DBG_PRINTF_L1("DEBUG from %s at %s:%d:\n", __func__, __FILE__, __LINE__);
    DBG_PRINTF_L1("    clock_gettime() returned %d\n", sysret);
    DBG_PRINTF_L1("    tv_sec  = %ld\n", ts.tv_sec);
    DBG_PRINTF_L1("    tv_nsec = %ld\n", ts.tv_nsec);
    DBG_PRINTF_L1("    time_us = %ld\n\n", time_us);

    return time_us;
}

void PlatformTime_waitTime(uint32_t timeoutMs)
{
    DBG_PRINTF_L1("DEBUG from %s at %s:%d:\n", __func__, __FILE__, __LINE__);
    DBG_PRINTF_L1("    timeoutMs = %d\n", timeoutMs);

    struct timespec ts = {
        .tv_sec  = 0,
        .tv_nsec = timeoutMs * 1000000};

    int sysret = nanosleep(&ts, NULL);
    if (sysret != 0)
    {
        char buf[64];
        sprintf(buf, "nanosleep() returned %d, errno = %d (%s)", sysret, errno, strerror(errno));
        EXIT_WITH_ERROR_MSG(buf);
    }

    DBG_PRINTF_L1("    nanosleep() returned %d\n\n", sysret);
}