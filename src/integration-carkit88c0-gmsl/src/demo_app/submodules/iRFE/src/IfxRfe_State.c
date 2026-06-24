/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#include "IfxRfe_State.h"

/******************************************************************************/
/*-------------------------Local variables------------------------------------*/
/******************************************************************************/
static bool _initialized = false;


/******************************************************************************/
/*-------------------------Public functions-----------------------------------*/
/******************************************************************************/
void SetInitialized(bool initialized)
{
    _initialized = initialized;
}

bool IsInitialized()
{
    return _initialized;
}