/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */

#ifndef IFXRFE_FIRMWARECOMMANDCALLS_H
#define IFXRFE_FIRMWARECOMMANDCALLS_H 1

#include "IfxRfe_AsyncHandle_t.h"
#include "IfxRfe_Buffers.h"
#include "IfxRfe_ErrorDefinitions.h"
#include <stdbool.h>
#include <stdint.h>

// Error check flags for the response
#define ERROR_CHECK_FLAG_SIZE       (0b00000001)
#define ERROR_CHECK_FLAG_OPCODE     (0b00000010)
#define ERROR_CHECK_FLAG_PAYLOAD    (0b00000100)
#define ERROR_CHECK_FLAG_COMMAND_ID (0b00001000)

#define ENABLE_ALL_ERROR_CHECKS  (ERROR_CHECK_FLAG_SIZE | ERROR_CHECK_FLAG_OPCODE | ERROR_CHECK_FLAG_PAYLOAD | ERROR_CHECK_FLAG_COMMAND_ID)
#define DISABLE_ALL_ERROR_CHECKS (0b00000000)

/// \brief Action type for configure (and execute) handle firmware command calls
typedef enum
{
    IfxRfe_Config_Update = 0,  ///> Update the existing handle
    IfxRfe_Config_New    = 1   ///> Configure a new handle
} IfxRfe_configHandleAction_t;

#ifdef __cplusplus
extern "C"
{
#endif

    /// \brief Execute the Execute_Directly() firmware command
    /// \param command_id Command id to be executed
    /// \param cmd Command structure already containing the serialized payload
    /// \param rsp [out] Response returned from CTRX
    /// \retval IFXRFE_E_SUCCESS on success
    /// \retval IFXRFE_E_NOT_INITIALIZED if IfxRfe not initialized
    /// \retval IFXRFE_E_INVALID_SIZE if received response too short
    /// \retval IFXRFE_E_UNEXPECTED_VALUE if received command id does not match command_id
    /// \retval IFXRFE_E_MISSING_PARAMETER if cmd or rsp is NULL
    /// \retval IFXRFE_E_INVALID_PARAMETER if cmd.payloadLen too long
    /// \retval IFXRFE_E_CRI_MISMATCH if response cri does not match command cri
    /// \retval IFXRFE_E_OPCODE_MISMATCH if response opcode does not match command opcode
    /// \retval IFXRFE_E_SPI_MSG_ERROR or IFXRFE_E_COMMAND_ERROR if the response indicates one of the errors
    /// \note error code from \ref SpiSendReceive otherwise
    error_t IfxRfe_fwCall_executeDirectly(uint8_t command_id, IfxRfe_ctrxRequest_t *cmd, IfxRfe_ctrxResponse_t *rsp);

    /// \brief Execute the Execute_Directly() firmware command, without waiting for it to finish
    /// \param command_id Command id to be executed
    /// \param cmd Command structure already containing the serialized payload
    /// \param handle [out] Handle containing parameters of the call, which need to be checked in \ref IfxRfe_fwCall_executeDirectly_read
    /// \retval IFXRFE_E_SUCCESS on success
    /// \retval IFXRFE_E_NOT_INITIALIZED if IfxRfe not initialized
    /// \retval IFXRFE_E_MISSING_PARAMETER if cmd is NULL or if handle is NULL
    /// \retval IFXRFE_E_INVALID_PARAMETER if cmd.payloadLen too long
    /// \note error code from \ref SpiSend otherwise
    error_t IfxRfe_fwCall_executeDirectly_send(uint8_t command_id, IfxRfe_ctrxRequest_t *cmd, IfxRfe_asyncHandle_t *handle);

    /// \brief Read back the result of the last Execute_Directly() firmware command
    /// \note It is not allowed to execute any commands between the last \ref IfxRfe_fwCall_executeDirectly_send and this call, otherwise the CRI would not match
    /// \note The function waits for the RFT pin blockingly
    /// \param handle Handle of the last \ref IfxRfe_fwCall_executeDirectly_send call, containing the parameters to check
    /// \param rsp [out] Response returned from CTRX
    /// \retval IFXRFE_E_SUCCESS on success
    /// \retval IFXRFE_E_NOT_INITIALIZED if IfxRfe not initialized
    /// \retval IFXRFE_E_MISSING_PARAMETER if rsp is NULL
    /// \retval IFXRFE_E_INVALID_SIZE if received response too short
    /// \retval IFXRFE_E_UNEXPECTED_VALUE if received command id does not match command_id
    /// \retval IFXRFE_E_CRI_MISMATCH if response cri does not match command cri
    /// \retval IFXRFE_E_OPCODE_MISMATCH if response opcode does not match command opcode
    /// \retval IFXRFE_E_SPI_MSG_ERROR or IFXRFE_E_COMMAND_ERROR if the response indicates one of the errors
    /// \note error code from \ref SpiRead otherwise
    error_t IfxRfe_fwCall_executeDirectly_read(IfxRfe_asyncHandle_t handle, IfxRfe_ctrxResponse_t *rsp);

    /// \brief Execute the Configure_Handle() firmware command
    /// \param handle_id Command id for new command or handle of the command to be updated
    /// \param action 0 for updating an existing handle, 1 for configuring a new handle
    /// \param cmd Command structure already containing the serialized payload
    /// \param handle [out] Handle of the stored configuration, 0xFF indicates that the configuration has failed
    /// \retval IFXRFE_E_SUCCESS on success
    /// \retval IFXRFE_E_NOT_INITIALIZED if IfxRfe not initialized
    /// \retval IFXRFE_E_INVALID_SIZE if received response length does not match
    /// \retval IFXRFE_E_UNEXPECTED_VALUE if received command id does not match command_id
    /// \retval IFXRFE_E_MISSING_PARAMETER if params is NULL but params_count != 0, or if handle is NULL
    /// \retval IFXRFE_E_INVALID_PARAMETER if cmd.payloadLen too long
    /// \retval IFXRFE_E_FAILED if an invalid handle was received
    /// \retval IFXRFE_E_CRI_MISMATCH if response cri does not match command cri
    /// \retval IFXRFE_E_OPCODE_MISMATCH if response opcode does not match command opcode
    /// \retval IFXRFE_E_SPI_MSG_ERROR or IFXRFE_E_COMMAND_ERROR if the response indicates one of the errors
    /// \note error code from \ref SpiSendReceive otherwise
    error_t IfxRfe_fwCall_configureHandle(uint8_t handle_id, IfxRfe_configHandleAction_t action, IfxRfe_ctrxRequest_t *cmd, uint8_t *handle);

    /// \brief Execute the Configure_And_Execute_Handle() firmware command
    /// \param handle_id Command id for new command or handle of the command to be updated
    /// \param action 0 for updating an existing handle, 1 for configuring a new handle
    /// \param cmd Command structure already containing the serialized payload
    /// \param handle Handle of the stored configuration, 0xFF indicates that the configuration has failed
    /// \param rsp [out] Response returned from CTRX
    /// \retval IFXRFE_E_SUCCESS on success
    /// \retval IFXRFE_E_NOT_INITIALIZED if IfxRfe not initialized
    /// \retval IFXRFE_E_INVALID_SIZE if received response length does not match
    /// \retval IFXRFE_E_UNEXPECTED_VALUE if received command id does not match command_id
    /// \retval IFXRFE_E_MISSING_PARAMETER if cmd or rsp is NULL or handle is NULL when the action is "new"
    /// \retval IFXRFE_E_INVALID_PARAMETER if cmd.payloadLen too long
    /// \retval IFXRFE_E_FAILED if an invalid handle was received
    /// \note error code from \ref SpiSendReceive otherwise
    error_t IfxRfe_fwCall_configureAndExecuteHandle(uint8_t handle_id, IfxRfe_configHandleAction_t action, IfxRfe_ctrxRequest_t *cmd, uint8_t *handle, IfxRfe_ctrxResponse_t *rsp);

    /// \brief Execute the Execute_Handle firmware command
    /// \param handle Handle to be executed
    /// \param command_id Command id that was configured for the handle, returned by the CTRX
    /// \param rsp [out] Response returned from CTRX
    /// \retval IFXRFE_E_SUCCESS on success
    /// \retval IFXRFE_E_NOT_INITIALIZED if IfxRfe not initialized
    /// \retval IFXRFE_E_MISSING_PARAMETER if rsp is NULL
    /// \retval IFXRFE_E_INVALID_SIZE if received response too short
    /// \retval IFXRFE_E_UNEXPECTED_VALUE if received handle does not match handle
    /// \retval IFXRFE_E_CRI_MISMATCH if response cri does not match command cri
    /// \retval IFXRFE_E_OPCODE_MISMATCH if response opcode does not match command opcode
    /// \retval IFXRFE_E_SPI_MSG_ERROR or IFXRFE_E_COMMAND_ERROR if the response indicates one of the errors
    /// \note error code from \ref SpiSendReceive otherwise
    error_t IfxRfe_fwCall_executeHandle(uint8_t handle, uint8_t *command_id, IfxRfe_ctrxResponse_t *rsp);

    /// \brief Set the error check flags for the response
    /// \param flags Bitmask of the flags to be set
    /// \note default is to check all flags
    void IfxRfe_fwCall_setErrorCheckFlags(uint8_t flags);

    /// \brief Get the SPI message error code from the last received response
    /// \return The SPI message error code, 0 if no error or if unable to extract
    uint8_t IfxRfe_fwCall_getLastSpiMsgError(void);

    /// \brief Get the command error code from the last received response
    /// \return The command error code, 0 if no error or if unable to extract
    uint8_t IfxRfe_fwCall_getLastCommandError(void);

#ifdef __cplusplus
}
#endif

#endif  //IFXRFE_FIRMWARECOMMANDCALLS_H
