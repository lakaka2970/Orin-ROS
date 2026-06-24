/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#include "IfxRfe_TimeWrapper.h"
#include <stddef.h>

/******************************************************************************/
/*-------------------------Local variables------------------------------------*/
/******************************************************************************/
static IfxRfe_timeFunctions_t _fptrs = {NULL, NULL, NULL};

/******************************************************************************/
/*-------------------------Global Functions-----------------------------------*/
/******************************************************************************/
void InitTimeWrapper(IfxRfe_timeFunctions_t fncs)
{
    _fptrs = fncs;
}

error_t Wrapper_GetDeadLine(uint32_t timeout_ms, int64_t *deadLine)
{
    if (NULL == _fptrs.getDeadLine)
    {
        return IFXRFE_E_INVALID_CONFIGURATION;
    }

    *deadLine = _fptrs.getDeadLine(timeout_ms);
    return IFXRFE_E_SUCCESS;
}

error_t Wrapper_Now(int64_t *now)
{
    if (NULL == _fptrs.now)
    {
        return IFXRFE_E_INVALID_CONFIGURATION;
    }

    *now = _fptrs.now();
    return IFXRFE_E_SUCCESS;
}

error_t Wrapper_WaitTime(uint32_t timeout_ms)
{
    if (NULL == _fptrs.waitTime)
    {
        return IFXRFE_E_INVALID_CONFIGURATION;
    }

    _fptrs.waitTime(timeout_ms);

    return IFXRFE_E_SUCCESS;
}
