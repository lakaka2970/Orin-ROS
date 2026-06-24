/*
 * (c) (2022-2025), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */

#ifndef PLATFORM_ERRORS_H
#define PLATFORM_ERRORS_H 1

/// \brief Error Code definition used by the Platform implementation
/// \note The gaps in the values are intentional to be backward compatible
#define PLATFORM_JETSON_E_SUCCESS               0x00
#define PLATFORM_JETSON_E_FAILED                0x01
#define PLATFORM_JETSON_E_INVALID_SIZE          0x02
#define PLATFORM_JETSON_E_INVALID_PARAMETER     0x03
#define PLATFORM_JETSON_E_MISSING_PARAMETER     0x04
#define PLATFORM_JETSON_E_UNEXPECTED_VALUE      0x05
#define PLATFORM_JETSON_E_OUT_OF_BOUNDS         0x06
#define PLATFORM_JETSON_E_OVERFLOW              0x07
#define PLATFORM_JETSON_E_TIMEOUT               0x08
#define PLATFORM_JETSON_E_BUSY                  0x09
#define PLATFORM_JETSON_E_ABORTED               0x0A
#define PLATFORM_JETSON_E_NOT_IMPLEMENTED       0x10
#define PLATFORM_JETSON_E_NOT_POSSIBLE          0x11
#define PLATFORM_JETSON_E_NOT_AVAILABLE         0x12
#define PLATFORM_JETSON_E_NOT_ALLOWED           0x13
#define PLATFORM_JETSON_E_NOT_INITIALIZED       0x14
#define PLATFORM_JETSON_E_NOT_CONFIGURED        0x15
#define PLATFORM_JETSON_E_NOT_SUPPORTED         0x17

//Evaluate the return value of a function call and return it, if it is not 0
#define PLATFORM_JETSON_RETURN_ON_ERROR(expression) \
    {                                               \
        const int ret = expression;                 \
        if (ret != PLATFORM_JETSON_E_SUCCESS)       \
        {                                           \
            return ret;                             \
        }                                           \
    }

//Evaluate the return value of a function call and return it after a cleanup, if it is not 0
#define PLATFORM_JETSON_CLEAN_RETURN_ON_ERROR(expression, cleanup) \
    {                                                              \
        const int ret = expression;                                \
        if (ret != PLATFORM_JETSON_E_SUCCESS)                      \
        {                                                          \
            cleanup;                                               \
            return ret;                                            \
        }                                                          \
    }

#endif /* PLATFORM_ERRORS_H */
