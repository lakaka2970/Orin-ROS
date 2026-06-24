/*
 * File: PlatformTime.h
 * Description: Platform-dependant iRFE Time API implementation for the NVIDIA Jetson
 * Project: MIMOrad
 * Created Date: Tuesday, 23 April 2024, 10:07
 * Author: Daniel Klepatsch (daniel.klepatsch@silicon-austria.com)
 * ---------------
 * Last Modified: Tuesday, 23 April 2024, 16:40
 * Modified By: Daniel Klepatsch
 * ---------------
 * Copyright (c) 2024 Silicon Austria Labs GmbH
 */

#ifndef PLATFORMTIME_H
#define PLATFORMTIME_H

#include <stdint.h>

int64_t PlatformTime_getDeadLine(uint32_t timeoutMs);
int64_t PlatformTime_now(void);
void PlatformTime_waitTime(uint32_t timeoutMs);

#endif  // PLATFORMTIME_H