/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#include "IfxRfe_GpioWrapper.h"
#include <stddef.h>


/******************************************************************************/
/*-------------------------Local variables------------------------------------*/
/******************************************************************************/
static const IfxRfe_gpioDefinitions_t *_ptrGpios = NULL;
static IfxRfe_gpioFunctions_t _fptrs             = {NULL, NULL, NULL, NULL, NULL};

/******************************************************************************/
/*-------------------------Global Functions-----------------------------------*/
/******************************************************************************/
void InitGpioWrapper(IfxRfe_gpioFunctions_t fncs)
{
    _fptrs = fncs;
}

void SetCtrxPins(const IfxRfe_gpioDefinitions_t *gpios)
{
    _ptrGpios = gpios;  //this seems to be the most efficient way to make sure we return NULL in getCtrxPinDefinition() when not initialized
}

const IfxRfe_gpioDefinitions_t *getCtrxPinDefinition()
{
    return _ptrGpios;
}

error_t Wrapper_GpioGet(uint16_t id, bool *state)
{
    if (NULL == _fptrs.gpioGet)
    {
        return IFXRFE_E_INVALID_CONFIGURATION;
    }

    return _fptrs.gpioGet(id, state);
}

error_t Wrapper_GpioSet(uint16_t id, bool state)
{
    if (NULL == _fptrs.gpioSet)
    {
        return IFXRFE_E_INVALID_CONFIGURATION;
    }

    return _fptrs.gpioSet(id, state);
}

error_t Wrapper_GpioConfigure(uint16_t id, uint8_t flags)
{
    if (NULL == _fptrs.gpioConfigure)
    {
        return IFXRFE_E_INVALID_CONFIGURATION;
    }

    return _fptrs.gpioConfigure(id, flags);
}

error_t Wrapper_IsGpioConfigOutput(uint8_t flags, bool *isOutput)
{
    if (NULL == _fptrs.isGpioConfigOutput)
    {
        return IFXRFE_E_INVALID_CONFIGURATION;
    }
    *isOutput = _fptrs.isGpioConfigOutput(flags);
    return IFXRFE_E_SUCCESS;
}

error_t Wrapper_GpioWait(uint16_t id, bool state, uint32_t timeoutMs)
{
    if (NULL == _fptrs.gpioWait)
    {
        return IFXRFE_E_NOT_CONFIGURED;
    }

    return _fptrs.gpioWait(id, state, timeoutMs);
}
