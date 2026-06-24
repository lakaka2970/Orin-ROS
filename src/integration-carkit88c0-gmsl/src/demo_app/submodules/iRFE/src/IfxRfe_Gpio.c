/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#include "IfxRfe_Gpio.h"

#include "IfxRfe_GpioWrapper.h"
#include "IfxRfe_Logger.h"
#include "IfxRfe_State.h"
#include "IfxRfe_TimeWrapper.h"
#include <stdio.h>

/******************************************************************************/
/*-------------------------Local function prototypes--------------------------*/
/******************************************************************************/
static error_t waitForPin(uint16_t pinId, uint32_t timeoutMs);

/******************************************************************************/
/*-------------------------Global Functions-----------------------------------*/
/******************************************************************************/


error_t IfxRfe_waitForRftPin(uint32_t timeoutMs)
{
    RETURN_ON_NOT_INITIALIZED();

    if (timeoutMs > IFXRFE_RFT_TIMEOUT_MAX)
    {
        return IFXRFE_E_INVALID_PARAMETER;
    }


    return waitForPin(getCtrxPinDefinition()->spiRftId, timeoutMs);
}

error_t IfxRfe_waitForOkPin(uint32_t timeoutMs)
{
    RETURN_ON_NOT_INITIALIZED();

    if (timeoutMs > IFXRFE_OK_TIMEOUT_MAX)
    {
        return IFXRFE_E_INVALID_PARAMETER;
    }

    return waitForPin(getCtrxPinDefinition()->okId, timeoutMs);
}

error_t IfxRfe_waitForDmux1Pin(uint32_t timeoutMs)
{
    RETURN_ON_NOT_INITIALIZED();

    if (timeoutMs > IFXRFE_DMUX_TIMEOUT_MAX)
    {
        return IFXRFE_E_INVALID_PARAMETER;
    }

    return waitForPin(getCtrxPinDefinition()->dmux1Id, timeoutMs);
}

error_t IfxRfe_waitForDmux2Pin(uint32_t timeoutMs)
{
    RETURN_ON_NOT_INITIALIZED();

    if (timeoutMs > IFXRFE_DMUX_TIMEOUT_MAX)
    {
        return IFXRFE_E_INVALID_PARAMETER;
    }

    return waitForPin(getCtrxPinDefinition()->dmux2Id, timeoutMs);
}

/******************************************************************************/
/*-------------------------Local helper functions-----------------------------*/
/******************************************************************************/

static error_t waitForPin(uint16_t pinId, uint32_t timeoutMs)
{
    error_t waitResult = Wrapper_GpioWait(pinId, true, timeoutMs);
    if (waitResult != IFXRFE_E_NOT_CONFIGURED)
    {
        return waitResult;
    }

    // If the wait function was not configured, we use the internal implementation

    int64_t deadline;
    bool state = false;
    int64_t now;

    IFXRFE_RETURN_ON_ERROR(Wrapper_GetDeadLine(timeoutMs, &deadline));

    IFXRFE_RETURN_ON_ERROR(Wrapper_Now(&now));  //check once here, if no error returned,
    //function is configured correctly and return value does not have to be checked in the loop
    IFXRFE_RETURN_ON_ERROR(Wrapper_GpioGet(pinId, &state));

    while ((false == state) && (now < deadline))
    {
        Wrapper_GpioGet(pinId, &state);
        Wrapper_Now(&now);
    }

    if (false == state)
    {
        return IFXRFE_E_TIMEOUT;
    }
    return IFXRFE_E_SUCCESS;
}
