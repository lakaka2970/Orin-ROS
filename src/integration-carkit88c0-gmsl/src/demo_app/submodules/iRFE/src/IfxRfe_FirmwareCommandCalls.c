/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#include "IfxRfe_FirmwareCommandCalls.h"

#include "IfxRfe_BuilderParser.h"
#include "IfxRfe_CtrxOpCodes.h"
#include "IfxRfe_Logger.h"
#include "IfxRfe_Spi.h"
#include "IfxRfe_State.h"
#include <stddef.h>
#include <string.h>


#define EXECUTE_DIRECTLY_MIN_LENGTH         3
#define CONFIGURE_HANDLE_MIN_LENGTH         4
#define CONFIGURE_EXECUTE_HANDLE_MIN_LENGTH 3
#define EXECUTE_HANDLE_MIN_LENGTH           3

/******************************************************************************/
/*-------------------------Serializer/deserializer prototypes-----------------*/
/******************************************************************************/
static inline error_t _executeDirectly_serialize(uint8_t command_id, IfxRfe_ctrxRequest_t *cmd, uint8_t *txLen);
static inline error_t _checkResponseForErrors(uint8_t minLength, uint8_t opcode, const IfxRfe_ctrxResponse_t *rsp);
static inline error_t _checkResponseCommandCode(uint8_t commandCode, const IfxRfe_ctrxResponse_t *rsp);

/******************************************************************************/
/*-------------------------Local variables------------------------------------*/
/******************************************************************************/
static uint8_t _errorCheckFlags  = ENABLE_ALL_ERROR_CHECKS;
static uint8_t _lastSpiMsgError  = 0;
static uint8_t _lastCommandError = 0;
/******************************************************************************/
/*------------------------Firmware command call implementations---------------*/
/******************************************************************************/

error_t IfxRfe_fwCall_executeDirectly(uint8_t command_id, IfxRfe_ctrxRequest_t *cmd, IfxRfe_ctrxResponse_t *rsp)
{
    RETURN_ON_NOT_INITIALIZED();

    // some commands will not provide the expected response.
    // they trigger a reset in the firmware and therefore we get back a triggerReset response instead of the expected one.
    // command_id 57: FinishFwDownload
    // command_id 59: TriggerLbist

    if ((NULL == cmd) || (NULL == rsp))
    {
        return IFXRFE_E_MISSING_PARAMETER;
    }

    uint8_t txLen = 0;
    IFXRFE_RETURN_ON_ERROR(_executeDirectly_serialize(command_id, cmd, &txLen));
    IFXRFE_RETURN_ON_ERROR(SpiSendReceive(cmd->buffer, txLen, rsp->buffer, &rsp->totalLen));
    IFXRFE_RETURN_ON_ERROR(_checkResponseForErrors(EXECUTE_DIRECTLY_MIN_LENGTH, GET_OPCODE(cmd->buffer[0]), rsp));
    IFXRFE_RETURN_ON_ERROR(_checkResponseCommandCode(command_id, rsp));
    return IFXRFE_E_SUCCESS;
}

error_t IfxRfe_fwCall_executeDirectly_send(uint8_t command_id, IfxRfe_ctrxRequest_t *cmd, IfxRfe_asyncHandle_t *handle)
{
    RETURN_ON_NOT_INITIALIZED();

    if ((NULL == handle) || (NULL == cmd))
    {
        return IFXRFE_E_MISSING_PARAMETER;
    }

    uint8_t txLen = 0;
    IFXRFE_RETURN_ON_ERROR(_executeDirectly_serialize(command_id, cmd, &txLen));
    handle->commandId = command_id;
    handle->CRI       = GET_CRI(cmd->buffer[0]);
    handle->opCode    = GET_OPCODE(cmd->buffer[0]);
    return SpiSend(cmd->buffer, txLen);
}

error_t IfxRfe_fwCall_executeDirectly_read(IfxRfe_asyncHandle_t handle, IfxRfe_ctrxResponse_t *rsp)
{
    RETURN_ON_NOT_INITIALIZED();

    if (NULL == rsp)
    {
        return IFXRFE_E_MISSING_PARAMETER;
    }

    IFXRFE_RETURN_ON_ERROR(SpiReceive(rsp->buffer, &rsp->totalLen));
    IFXRFE_RETURN_ON_ERROR(_checkResponseForErrors(EXECUTE_DIRECTLY_MIN_LENGTH, handle.opCode, rsp));
    IFXRFE_RETURN_ON_ERROR(_checkResponseCommandCode(handle.commandId, rsp));
    return IFXRFE_E_SUCCESS;
}

error_t IfxRfe_fwCall_configureHandle(uint8_t handle_id, IfxRfe_configHandleAction_t action, IfxRfe_ctrxRequest_t *cmd, uint8_t *handle)
{
    RETURN_ON_NOT_INITIALIZED();
    IfxRfe_ctrxResponse_t rsp;

    if (NULL == cmd)
    {
        return IFXRFE_E_MISSING_PARAMETER;
    }

    if ((NULL == handle) && (IfxRfe_Config_New == action))
    {
        return IFXRFE_E_MISSING_PARAMETER;
    }

    //cmd is already filled with the payload - only mask the first payload word and add the command id
    cmd->buffer[1] = ((handle_id << 24) & 0xFF000000) | (((uint32_t)action) << 16) | (cmd->buffer[1] & 0x0000FFFF);

    IFXRFE_RETURN_ON_ERROR(FillHeaderAndCrc(IFXRFE_OPCODE_CONFIGURE_HANDLE, 0, cmd));

    uint8_t txLen = (0 == cmd->payloadLen) ? 3 : cmd->payloadLen + 2;  //min length is 3, otherwise payload + header + crc32

    IFXRFE_RETURN_ON_ERROR(SpiSendReceive(cmd->buffer, txLen, rsp.buffer, &rsp.totalLen));

    IFXRFE_RETURN_ON_ERROR(_checkResponseForErrors(CONFIGURE_HANDLE_MIN_LENGTH, GET_OPCODE(cmd->buffer[0]), &rsp));

    //additional size check necessary, bc. _checkResponseForErrors only checks for min size
    if ((_errorCheckFlags & ERROR_CHECK_FLAG_SIZE) && (rsp.totalLen != CONFIGURE_HANDLE_MIN_LENGTH))
    {
        return IFXRFE_E_INVALID_SIZE;
    }

    if (IfxRfe_Config_New == action)  //only check the command id on new command, on update we only have the handle
    {
        IFXRFE_RETURN_ON_ERROR(_checkResponseCommandCode(handle_id, &rsp));
    }

    uint8_t receivedHandle = (uint8_t)((rsp.buffer[1] & 0x0000FF00) >> 8);
    if ((IfxRfe_Config_Update == action) && (receivedHandle != handle_id))  //only check the handle on the update command, on new command we only have the command id
    {
        errorLogFmt("received handle %d does not match sent handle/id %d", receivedHandle, handle_id);
        return IFXRFE_E_UNEXPECTED_VALUE;
    }

    if (0xFF == receivedHandle)
    {
        return IFXRFE_E_FAILED;
    }

    if (handle != NULL)
    {
        *handle = receivedHandle;
    }

    return IFXRFE_E_SUCCESS;
}

error_t IfxRfe_fwCall_configureAndExecuteHandle(uint8_t handle_id, IfxRfe_configHandleAction_t action, IfxRfe_ctrxRequest_t *cmd, uint8_t *handle, IfxRfe_ctrxResponse_t *rsp)
{
    RETURN_ON_NOT_INITIALIZED();

    if ((NULL == cmd) || (NULL == rsp))
    {
        return IFXRFE_E_MISSING_PARAMETER;
    }

    if ((NULL == handle) && (IfxRfe_Config_New == action))
    {
        return IFXRFE_E_MISSING_PARAMETER;
    }

    //cmd is already filled with the payload - only mask the first payload word and add the command id
    cmd->buffer[1] = ((handle_id << 24) & 0xFF000000) | (((uint32_t)action) << 16) | (cmd->buffer[1] & 0x0000FFFF);

    IFXRFE_RETURN_ON_ERROR(FillHeaderAndCrc(IFXRFE_OPCODE_CONFIGURE_AND_EXECUTE_HANDLE, 0, cmd));

    uint8_t txLen = (0 == cmd->payloadLen) ? 3 : cmd->payloadLen + 2;  //min command length 3, otherwise payload + header + crc32

    IFXRFE_RETURN_ON_ERROR(SpiSendReceive(cmd->buffer, txLen, rsp->buffer, &rsp->totalLen));

    IFXRFE_RETURN_ON_ERROR(_checkResponseForErrors(CONFIGURE_EXECUTE_HANDLE_MIN_LENGTH, GET_OPCODE(cmd->buffer[0]), rsp));

    if (IfxRfe_Config_New == action)  //only check the command id on new command, on update we only have the handle
    {
        IFXRFE_RETURN_ON_ERROR(_checkResponseCommandCode(handle_id, rsp));
    }

    uint8_t receivedHandle = (uint8_t)((rsp->buffer[1] & 0x0000FF00) >> 8);
    if ((IfxRfe_Config_Update == action) && (receivedHandle != handle_id))  //only check the handle on the update command, on new command we only have the command id
    {
        errorLogFmt("received handle %d does not match sent handle/id %d", receivedHandle, handle_id);
        return IFXRFE_E_UNEXPECTED_VALUE;
    }

    if (0xFF == receivedHandle)
    {
        return IFXRFE_E_FAILED;
    }

    if (handle != NULL)
    {
        *handle = receivedHandle;
    }

    return IFXRFE_E_SUCCESS;
}

error_t IfxRfe_fwCall_executeHandle(uint8_t handle, uint8_t *command_id, IfxRfe_ctrxResponse_t *rsp)
{
    RETURN_ON_NOT_INITIALIZED();

    if (NULL == rsp)
    {
        return IFXRFE_E_MISSING_PARAMETER;
    }

    IfxRfe_ctrxRequest_t cmd;
    cmd.payloadLen = 0;
    IFXRFE_RETURN_ON_ERROR(FillHeaderAndCrc(IFXRFE_OPCODE_EXECUTE_HANDLE, handle, &cmd));

    IFXRFE_RETURN_ON_ERROR(SpiSendReceive(cmd.buffer, 1, rsp->buffer, &rsp->totalLen));

    //check if command code matches (cri and opcode already checked)
    IFXRFE_RETURN_ON_ERROR(_checkResponseForErrors(EXECUTE_HANDLE_MIN_LENGTH, GET_OPCODE(cmd.buffer[0]), rsp));

    //check if handle matches
    uint8_t receivedHandle = (rsp->buffer[1] & 0x0000FF00) >> 8;
    if (receivedHandle != handle)
    {
        errorLogFmt("received handle %d does not match sent handle %d", receivedHandle, handle);
        return IFXRFE_E_UNEXPECTED_VALUE;
    }

    //extract command id
    if (command_id != NULL)
    {
        *command_id = (uint8_t)rsp->buffer[1] & 0x000000FF;
    }

    return IFXRFE_E_SUCCESS;
}

void IfxRfe_fwCall_setErrorCheckFlags(uint8_t flags)
{
    _errorCheckFlags = flags;
}

uint8_t IfxRfe_fwCall_getLastSpiMsgError(void)
{
    return _lastSpiMsgError;
}

uint8_t IfxRfe_fwCall_getLastCommandError(void)
{
    return _lastCommandError;
}

/******************************************************************************/
/*-------------------------Serializers/deserializers--------------------------*/
/******************************************************************************/
static inline error_t _executeDirectly_serialize(uint8_t command_id, IfxRfe_ctrxRequest_t *cmd, uint8_t *txLen)
{
    RETURN_ON_NOT_INITIALIZED();

    if (NULL == cmd)
    {
        return IFXRFE_E_MISSING_PARAMETER;
    }

    //cmd is already filled with the payload - only mask the first payload word and add the command id
    cmd->buffer[1] = ((command_id << 24) & 0xFF000000) | (cmd->buffer[1] & 0x0000FFFF);

    IFXRFE_RETURN_ON_ERROR(FillHeaderAndCrc(IFXRFE_OPCODE_EXECUTE_DIRECTLY_FW_CMD, 0, cmd));

    *txLen = (0 == cmd->payloadLen) ? 3 : cmd->payloadLen + 2;  //min execute directly length is 3, otherwise payload length + header + crc32

    return IFXRFE_E_SUCCESS;
}

static inline error_t _checkResponseForErrors(uint8_t minLength, uint8_t opcode, const IfxRfe_ctrxResponse_t *rsp)
{
    // Retrieve the SPI and command error codes even if error checks are disabled.
    // This allows the user to request them even if he does not want to perform the checks automatically.
    error_t payloadError = GetPayloadError(rsp, &_lastSpiMsgError, &_lastCommandError);

    if (_errorCheckFlags == DISABLE_ALL_ERROR_CHECKS)  // if error check is disabled, skip the checks
    {
        return IFXRFE_E_SUCCESS;
    }

    if (_errorCheckFlags & ERROR_CHECK_FLAG_SIZE)
    {
        //check size
        if (rsp->totalLen < minLength)
        {
            return IFXRFE_E_INVALID_SIZE;
        }
    }

    if (_errorCheckFlags & ERROR_CHECK_FLAG_OPCODE)
    {
        // check opcode
        if (opcode != GET_OPCODE(rsp->buffer[0]))
        {
            return IFXRFE_E_OPCODE_MISMATCH;
        }
    }

    if (_errorCheckFlags & ERROR_CHECK_FLAG_PAYLOAD)
    {
        //check error codes
        IFXRFE_CLEAN_RETURN_ON_ERROR(payloadError, errorLogFmt("payload error code 0x%02X, SPI message error 0x%02X, command error 0x%02X", payloadError, _lastSpiMsgError, _lastCommandError));
    }

    return IFXRFE_E_SUCCESS;
}

static inline error_t _checkResponseCommandCode(uint8_t commandCode, const IfxRfe_ctrxResponse_t *rsp)
{
    if ((_errorCheckFlags & ERROR_CHECK_FLAG_COMMAND_ID))
    {
        uint8_t receivedCmd = rsp->buffer[1] & 0x000000FF;
        if (receivedCmd != commandCode)
        {
            errorLogFmt("received command id %d does not match sent command id %d", receivedCmd, commandCode);
            return IFXRFE_E_UNEXPECTED_VALUE;
        }
    }
    return IFXRFE_E_SUCCESS;
}
