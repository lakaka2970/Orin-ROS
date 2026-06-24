/*
 * File: Util.h
 * Description: Utilities used in the NVIDIA Jetson platform implementation
 * Project: MIMOrad
 * Created Date: Tuesday, 23 April 2024, 10:07
 * Author: Daniel Klepatsch (daniel.klepatsch@silicon-austria.com)
 * ---------------
 * Last Modified: Thursday, 16 January 2025, 15:03
 * Modified By: Daniel Klepatsch
 * ---------------
 * Copyright (c) 2024 Silicon Austria Labs GmbH
 */

#ifndef UTIL_H
#define UTIL_H

#include "PlatformErrors.h"

#include <stdio.h>
#include <stdlib.h>

//#define DEBUG 1

#if DEBUG == 1
    // Debug Level 1
    #define DBG_PRINTF_L1(msg, ...) printf(msg, ##__VA_ARGS__)
    #define DBG_PRINTF_L2(msg, ...)
#elif DEBUG == 2
    // Debug Level 2
    #define DBG_PRINTF_L1(msg, ...) printf(msg, ##__VA_ARGS__)
    #define DBG_PRINTF_L2(msg, ...) printf(msg, ##__VA_ARGS__)
#else
    #define DBG_PRINTF_L1(msg, ...)
    #define DBG_PRINTF_L2(msg, ...)
#endif

#define EXIT_WITH_ERROR_MSG(msg)                                                                   \
    {                                                                                              \
        printf("Error in function %s() in file %s:%d -> %s\n", __func__, __FILE__, __LINE__, msg); \
        exit(EXIT_FAILURE);                                                                        \
    }

#define RETURN_ON_IFX_ERROR(func)                  \
    {                                              \
        int const ret_code = func;                 \
        if (ret_code != PLATFORM_JETSON_E_SUCCESS) \
        {                                          \
            return ret_code;                       \
        }                                          \
    }

#endif  // UTIL_H
