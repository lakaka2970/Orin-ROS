/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */

#include "IfxRfe_SpiConfig.h"

/******************************************************************************/
/*-------------------------MACROS---------------------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-------------------------Local Variables------------------------------------*/
/******************************************************************************/

static uint8_t _gSpiRetransmissionCount = 0;

/******************************************************************************/
/*-------------------------Local Function Prototypes--------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-------------------------Global Function Implementations--------------------*/
/******************************************************************************/

void IfxRfe_setSpiRetransmissionCount(uint8_t count)
{
    _gSpiRetransmissionCount = count;
}

uint8_t IfxRfe_spiRetransmissionCount()
{
    return _gSpiRetransmissionCount;
}

/******************************************************************************/
/*-------------------------Local Function Implementations---------------------*/
/******************************************************************************/
