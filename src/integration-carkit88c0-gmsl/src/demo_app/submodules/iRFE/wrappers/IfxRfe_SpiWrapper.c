/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#include "IfxRfe_SpiWrapper.h"

#include "IfxRfe_Logger.h"
#include <stddef.h>
#include <stdio.h>


/******************************************************************************/
/*-------------------------Local variables------------------------------------*/
/******************************************************************************/
static uint8_t _spiId               = 0xFF;
static IfxRfe_spiFunctions_t _fptrs = {NULL, NULL, NULL};

/******************************************************************************/
/*-------------------------Global Functions-----------------------------------*/
/******************************************************************************/

void InitSpiWrapper(IfxRfe_spiFunctions_t fncs)
{
    _fptrs = fncs;
}

uint8_t GetSpiId(void)
{
    return _spiId;
}

void SetSpiId(uint8_t devId)
{
    _spiId = devId;
}

error_t Wrapper_SpiTransfer(uint32_t count, const uint32_t bufWrite[], uint32_t bufRead[], bool keepSel)
{
    if (NULL == _fptrs.spiTransfer)
    {
        return IFXRFE_E_INVALID_CONFIGURATION;
    }

    return _fptrs.spiTransfer(_spiId, count, bufWrite, bufRead, keepSel);
}

error_t Wrapper_SpiWrite(uint32_t count, const uint32_t buffer[], bool keepSel)
{
    if (NULL == _fptrs.spiWrite)
    {
        return IFXRFE_E_INVALID_CONFIGURATION;
    }

    return _fptrs.spiWrite(_spiId, count, buffer, keepSel);
}


error_t Wrapper_SpiConfigure(uint8_t flags, uint32_t speed)
{
    if (NULL == _fptrs.spiConfigure)
    {
        return IFXRFE_E_INVALID_CONFIGURATION;
    }

    return _fptrs.spiConfigure(_spiId, flags, speed);
}
