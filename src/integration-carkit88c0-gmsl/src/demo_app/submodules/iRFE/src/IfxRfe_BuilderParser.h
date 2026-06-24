/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#ifndef IFXRFE_BUILDERPARSER_H
#define IFXRFE_BUILDERPARSER_H 1

#include "IfxRfe_Buffers.h"
#include "IfxRfe_ErrorDefinitions.h"
#include <stdint.h>

#define MAX_SUPPORTED_CTRX_DEVICES 12

#define GET_CRI(header) ((header & 0x0F000000) >> 24)

//note: in the response frames, this field always corresponds to Payload length
//      in the command frames however, it can be both the payload length or the header parameter
#define GET_PAYLOAD_LEN(header) ((header & 0x00FF0000) >> 16)

#define GET_OPCODE(header) ((header & 0xF0000000) >> 28)

#ifdef __cplusplus
extern "C"
{
#endif

    /// \brief Set device id for CRI value of the specific device
    /// \param _devId device id to change CRI object used
    void SetDevId(uint8_t _devId);

    /// \brief Set current CRI value for the selected device
    /// The next retrieved CRI value will be the one set here incremented by one
    /// \param cri CRI value to set
    void SetCurrentCri(uint8_t cri);

    /// \brief Fill in the header and CRC32 of a ctrx request
    /// \param opCode Opcode for the command. It is not checked for validity.
    /// \param headerParam Header parameter (reset type / handle) for the trigger_reset/execute_handle commands. Ignored for other opcodes.
    /// \param cmd Request structure to be initialized.
    /// \return IFXRFE_E_SUCCESS if finished with no errors
    /// \return IFXRFE_E_MISSING_PARAMETER if cmd is NULL
    /// \return IFXRFE_E_INVALID_PARAMETER if cmd length is too high, or if it is not 0 when opcode indicates request with no payload
    error_t FillHeaderAndCrc(uint8_t opCode, uint8_t headerParam, IfxRfe_ctrxRequest_t *cmd);

    /// \brief Determine the expected CRI value in the response for a given command
    /// \param command Pointer to the command buffer
    /// \param len Length of the command buffer in 32-bit words
    /// \param cri Pointer to store the expected CRI value
    /// \return IFXRFE_E_INVALID_PARAMETER if command, cri is NULL or len is 0
    /// \return IFXRFE_E_SUCCESS if finished with no errors
    error_t ExpectedResponseCri(uint32_t *command, uint8_t len, uint8_t *cri);

    /// \brief Check the response header crc
    /// \param header Header value to check
    /// \return IFXRFE_E_CRC16 if crc does not match IFXRFE_E_SUCCESS otherwise
    error_t CheckResponseHeaderCrc(uint32_t header);

    /// \brief Check the response payload crc
    /// \param buffer Response buffer to check
    /// \param len Nr. of words in the response buffer
    /// \retval IFXRFE_E_INVALID_SIZE if the response does not contain enough payload
    /// \retval IFXRFE_E_INVALID_CRC32 if the payload does not match
    /// \retval IFXRFE_E_SUCCESS otherwise
    error_t CheckResponsePayloadCrc(uint32_t *buffer, uint8_t len);

    /// \brief Performs the error check in the received message payload
    /// \note If the response indicates a header only message, IFXRFE_E_SUCCESS is returned with unchanged errorCode
    /// \param response The message to perform the check on
    /// \param spiMsgError The extracted SPI message error code, 0 if no error or if unable to extract
    /// \param commandError The extracted command error code, 0 if no error or if unable to extract
    /// \retval IFXRFE_E_SPI_MSG_ERROR if payload indicates SPI MSG error code
    /// \retval IFXRFE_E_COMMAND_ERROR if payload indicates command specific error code
    /// \retval IFXRFE_E_SUCCESS otherwise
    error_t GetPayloadError(const IfxRfe_ctrxResponse_t *response, uint8_t *spiMsgError, uint8_t *commandError);

#ifdef __cplusplus
}
#endif

#endif  //IFXRFE_BUILDERPARSER_H
