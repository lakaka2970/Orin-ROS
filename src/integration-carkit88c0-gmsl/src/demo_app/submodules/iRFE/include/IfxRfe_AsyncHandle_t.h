/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#ifndef IFXRFE_ASYNCHANDLE_T
#define IFXRFE_ASYNCHANDLE_T 1

#include <stdint.h>

typedef struct
{
    uint8_t commandId;
    uint8_t opCode;
    uint8_t CRI;
} IfxRfe_asyncHandle_t;

#endif  //IFXRFE_ASYNCHANDLE_T
