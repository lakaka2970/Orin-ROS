/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */

#include "IfxRfe_LowLevelCommands.h"
#include "IfxRfe_Buffers.h"
#include "IfxRfe_BuilderParser.h"
#include "IfxRfe_CtrxOpCodes.h"
#include "IfxRfe_Spi.h"
#include "IfxRfe_State.h"
#include <stddef.h>

error_t IfxRfe_doNOP(IfxRfe_ctrxResponse_t *result)
{
    RETURN_ON_NOT_INITIALIZED();
    IfxRfe_ctrxResponse_t temp;
    IfxRfe_ctrxResponse_t *ptr;
    ptr = (NULL == result) ? &temp : result;

    IfxRfe_ctrxRequest_t cmd;
    cmd.payloadLen = 0;
    IFXRFE_RETURN_ON_ERROR(FillHeaderAndCrc(IFXRFE_OPCODE_NOP, 0, &cmd));

    return SpiTransfer(cmd.buffer, 1, ptr->buffer, &ptr->totalLen);
}

error_t IfxRfe_triggerReset(IfxRfe_triggerResetResult_t *result)
{
    RETURN_ON_NOT_INITIALIZED();

    IfxRfe_ctrxRequest_t cmd;
    cmd.payloadLen = 0;
    IFXRFE_RETURN_ON_ERROR(FillHeaderAndCrc(IFXRFE_OPCODE_TRIGGER_RESET, 0x00, &cmd));  //reset type as parameter only used by 8181_EES which is no longer supported

    IfxRfe_ctrxResponse_t rsp;
    IFXRFE_RETURN_ON_ERROR(SpiSendReceive(cmd.buffer, 1, rsp.buffer, &rsp.totalLen));

    //parse result
    if (rsp.totalLen != 4)  //Trigger_Reset() answer is always 4 long
    {
        return IFXRFE_E_SPI_MSG_ERROR;
    }

    //check opcode only, CRI check disabled after trigger reset
    if (GET_OPCODE(cmd.buffer[0]) != GET_OPCODE(rsp.buffer[0]))
    {
        return IFXRFE_E_OPCODE_MISMATCH;
    }

    if (result != NULL)
    {
        //command error code and spi message error at this point 0, otherwise would have returned with error previously
        //crc, reserved fields and header are engineering requirements
        result->iRamClearStatus = ((rsp.buffer[1] & 0x80) != 0);
        result->lastReset       = (uint8_t)(rsp.buffer[1] & 0x7F);
    }

    return IFXRFE_E_SUCCESS;
}
