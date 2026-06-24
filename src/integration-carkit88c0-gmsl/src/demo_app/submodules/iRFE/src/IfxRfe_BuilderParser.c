/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#include "IfxRfe_BuilderParser.h"
#include "IfxRfe_Crc16.h"
#include "IfxRfe_Crc32.h"
#include "IfxRfe_CtrxCommandIds.h"
#include "IfxRfe_CtrxOpCodes.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/******************************************************************************/
/*-------------------------Local variables------------------------------------*/
/******************************************************************************/
static uint8_t _devId                           = 0;    // Store the device id for the CTRX CRI operations
static uint8_t _cri[MAX_SUPPORTED_CTRX_DEVICES] = {0};  // Store CRI for multiple CTRXs

/******************************************************************************/
/*-------------------------Local Function Prototypes--------------------------*/
/******************************************************************************/
static uint8_t generateNewCri();
static uint8_t getPayloadSpiMsgError(const IfxRfe_ctrxResponse_t *rsp);
static uint8_t getPayloadCommandError(const IfxRfe_ctrxResponse_t *rsp);

/******************************************************************************/
/*-------------------------Global Function Implementations--------------------*/
/******************************************************************************/
void SetDevId(uint8_t devId)
{
    _devId = devId;
}

void SetCurrentCri(uint8_t cri)
{
    _cri[_devId] = cri;
}

error_t FillHeaderAndCrc(uint8_t opCode, uint8_t headerParam, IfxRfe_ctrxRequest_t *cmd)
{
    if (NULL == cmd)
    {
        return IFXRFE_E_MISSING_PARAMETER;
    }
    if (cmd->payloadLen > MAX_REQ_PAYLOAD_WORDS)  //leave space for header + CRC32
    {
        return IFXRFE_E_INVALID_PARAMETER;
    }

    bool noPayload = ((IFXRFE_OPCODE_NOP == opCode) || (IFXRFE_OPCODE_TRIGGER_RESET == opCode) || (IFXRFE_OPCODE_EXECUTE_HANDLE == opCode));
    if ((0 != cmd->payloadLen) && (true == noPayload))
    {
        return IFXRFE_E_INVALID_PARAMETER;
    }

    //fill opcode & cri
    cmd->buffer[0] = (uint32_t)(((opCode & 0x0F) << 4) + (generateNewCri() & 0x0F)) << 24;

    //fill header parameter: reset type/handle/payload length
    if ((IFXRFE_OPCODE_TRIGGER_RESET != opCode) && (IFXRFE_OPCODE_EXECUTE_HANDLE != opCode))
    {
        //only above comands can specify a header param, for the others it is the payload length
        headerParam = cmd->payloadLen;
    }
    cmd->buffer[0] |= (uint32_t)headerParam << 16;

    //finalize the header
    uint16_t crc16 = calculateCRC16(cmd->buffer[0]);
    cmd->buffer[0] |= crc16;

    //calculate the CRC32 if command has payload
    if (!noPayload)
    {
        uint8_t securedLen      = (0 == cmd->payloadLen) ? 2 : cmd->payloadLen + 1;  //min the command id + header has to be there
        cmd->buffer[securedLen] = calculateCRC32(cmd->buffer, securedLen);
    }

    return IFXRFE_E_SUCCESS;
}

error_t ExpectedResponseCri(uint32_t *command, uint8_t len, uint8_t *cri)
{
    if (command == NULL || len == 0 || cri == NULL)
    {
        return IFXRFE_E_INVALID_PARAMETER;
    }

    uint8_t opCode = GET_OPCODE(command[0]);
    if (opCode == IFXRFE_OPCODE_TRIGGER_RESET)
    {
        *cri = 0;
        return IFXRFE_E_SUCCESS;
    }

    if (len > 1)
    {
        uint8_t commandId = (0xFF000000 & command[1]) >> 24;
        if (commandId == IFXRFE_FW_COMMAND_TRIGGER_LBIST || commandId == IFXRFE_FW_COMMAND_FINISH_FW_DOWNLOAD)
        {
            *cri = 0;
            return IFXRFE_E_SUCCESS;
        }
    }

    *cri = GET_CRI(command[0]);
    return IFXRFE_E_SUCCESS;
}

error_t CheckResponseHeaderCrc(uint32_t header)
{
    uint16_t calculated = calculateCRC16(header);
    uint16_t received   = header & 0x0000FFFF;
    if (received != calculated)
    {
        return IFXRFE_E_CRC16;
    }
    return IFXRFE_E_SUCCESS;
}

error_t CheckResponsePayloadCrc(uint32_t *buffer, uint8_t len)
{
    if (len < 3)  //at least header, payload and crc32 has to be there
    {
        return IFXRFE_E_INVALID_SIZE;
    }

    uint32_t calculated = calculateCRC32(buffer, len - 1);
    uint32_t received   = buffer[len - 1];
    if (received != calculated)
    {
        return IFXRFE_E_CRC32;
    }
    return IFXRFE_E_SUCCESS;
}

error_t GetPayloadError(const IfxRfe_ctrxResponse_t *response, uint8_t *spiMsgError, uint8_t *commandError)
{
    if (response->totalLen < 2)
    {
        //set to 0 to not indicate an old error
        *spiMsgError  = 0;
        *commandError = 0;
        return IFXRFE_E_SUCCESS;
    }
    *spiMsgError  = getPayloadSpiMsgError(response);
    *commandError = getPayloadCommandError(response);
    if (*spiMsgError != 0)
    {
        return IFXRFE_E_SPI_MSG_ERROR;
    }
    if (*commandError != 0)
    {
        return IFXRFE_E_COMMAND_ERROR;
    }

    return IFXRFE_E_SUCCESS;
}

/******************************************************************************/
/*-------------------------Local Function Implementations---------------------*/
/******************************************************************************/

static uint8_t generateNewCri()
{
    _cri[_devId]++;  //increment first to never start with 0 - reserved for reset response
    if (_cri[_devId] >= 0x10)
    {
        _cri[_devId] = 1;
    }
    return _cri[_devId];
}

static uint8_t getPayloadSpiMsgError(const IfxRfe_ctrxResponse_t *rsp)
{
    uint32_t res = rsp->buffer[1];
    res          = res >> 24;
    return (uint8_t)res;
}

static uint8_t getPayloadCommandError(const IfxRfe_ctrxResponse_t *rsp)
{
    uint32_t res = rsp->buffer[1];
    res          = res >> 16;
    return (uint8_t)res;
}
