/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#include "IfxRfe.h"

#include "IfxRfe_BuilderParser.h"
#include "IfxRfe_Gpio.h"
#include "IfxRfe_GpioWrapper.h"
#include "IfxRfe_Logger.h"
#include "IfxRfe_LowLevelCommands.h"
#include "IfxRfe_SpiWrapper.h"
#include "IfxRfe_State.h"
#include "IfxRfe_TimeWrapper.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

/******************************************************************************/
/*-------------------------Local variables------------------------------------*/
/******************************************************************************/

static uint8_t _devCount                      = 0;
static const IfxRfe_gpioDefinitions_t *_gpios = NULL;
/******************************************************************************/
/*----------------------------------Defines-----------------------------------*/
/******************************************************************************/

// Macro to create a mask based on used bit numbers
#define MASK_BITS_RANGE(upper, lower) (((1U << ((upper) - (lower) + 1)) - 1) << (lower))

// for the exact representation of the constant (2^−16 × 50 × 6)
#define FREQ_CALC_DENOMINATOR_CONST 4.57763671875e-03

/******************************************************************************/
/*-------------------------Local Function Prototypes--------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-------------------------Global Function Implementations--------------------*/
/******************************************************************************/

error_t IfxRfe_init(uint8_t devCount, const IfxRfe_gpioDefinitions_t *gpios, IfxRfe_spiFunctions_t spiFncs, IfxRfe_gpioFunctions_t gpioFncs, IfxRfe_timeFunctions_t timeFncs, IfxRfe_logCallbacks_t logInterface)
{
    if (devCount < 1 || devCount > MAX_SUPPORTED_CTRX_DEVICES)
    {
        return IFXRFE_E_OUT_OF_BOUNDS;
    }

    _devCount = devCount;
    _gpios    = gpios;
    loggerInit(logInterface);
    InitSpiWrapper(spiFncs);
    InitGpioWrapper(gpioFncs);
    InitTimeWrapper(timeFncs);

    infoLogFmt("IfxRfe initialized. Version %s", IFXRFE_VERSION);

    SetInitialized(true);

    IfxRfe_selectDevice(0);
    return IFXRFE_E_SUCCESS;
}

error_t IfxRfe_selectDevice(uint8_t devId)
{
    RETURN_ON_NOT_INITIALIZED();

    if (devId >= _devCount)
        return IFXRFE_E_OUT_OF_BOUNDS;

    SetDevId(devId);
    SetSpiId(devId);
    SetCtrxPins(&(_gpios[devId]));

    return IFXRFE_E_SUCCESS;
}

float IfxRfe_qToFloat(int qValue, uint8_t fractionalBits)
{
    return (float)(qValue * (1.0 / (1 << fractionalBits)));
}

int IfxRfe_floatToQ(float value, uint8_t fractionalBits)
{
    return (int)(value * (1 << fractionalBits));
}
