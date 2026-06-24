/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */

#ifndef IFXRFE_LOWLEVELCOMMANDS_H
#define IFXRFE_LOWLEVELCOMMANDS_H 1

#include "IfxRfe_Buffers.h"
#include "IfxRfe_ErrorDefinitions.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool iRamClearStatus;  ///< indicates if IRAM has been cleared after the reset
    uint8_t lastReset;     ///< lastReset byte of the SPI frame
} IfxRfe_triggerResetResult_t;

#ifdef __cplusplus
extern "C"
{
#endif

    /// \brief Execute Do_NOP command to read answer from last command
    /// \note SPI frame will be extended if response indicates a message with payload
    /// \param result The response from the last command (tha last, which was not a Do_NOP)
    /// \retval IFXRFE_E_SUCCESS on success
    /// \retval IFXRFE_E_NOT_INITIALIZED if IfxRfe not initialized
    /// \note error code from \ref SpiTransfer otherwise
    error_t IfxRfe_doNOP(IfxRfe_ctrxResponse_t *result);

    /// \brief Execute a Trigger_Reset() command on the CTRX
    /// \param result Response from the Ctrx
    /// \retval IFXRFE_E_SUCCESS on success
    /// \retval IFXRFE_E_NOT_INITIALIZED if IfxRfe not initialized
    /// \retval IFXRFE_E_SPI_MSG_ERROR if response length invalid
    /// \retval IFXRFE_E_OPCODE_MISMATCH if response opcode does not match command opcode
    /// \note error code from \ref SpiSendReceive otherwise (note: cri is not checked for reset)
    error_t IfxRfe_triggerReset(IfxRfe_triggerResetResult_t *result);

#ifdef __cplusplus
}
#endif

#endif  //IFXRFE_LOWLEVELCOMMANDS_H
