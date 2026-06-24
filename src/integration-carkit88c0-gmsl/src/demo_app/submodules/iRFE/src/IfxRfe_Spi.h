/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#ifndef IFXRFE_SPI_H
#define IFXRFE_SPI_H 1

#include "IfxRfe_ErrorDefinitions.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /// \brief Transmit command and receive response in full duplex mode (send actual command and receive response from previous one)
    /// \note The spi frame is automatically extended if the response header indicates a longer frame than the command
    /// \note The function is waiting for the RFT pin to be set, blockingly
    /// \param wbuf Data transmit
    /// \param wlen Nr. data words to transmit
    /// \param rbuf Buffer to receive into
    /// \param rlen [out] Nr. of data words received
    /// \retval IFXRFE_E_SUCCESS if ok,
    /// \retval IFXRFE_E_CRC16 on received header crc error
    /// \retval IFXRFE_E_CRC32 on received frame crc32 error
    /// \retval IFXRFE_E_MISSING_PARAMETER if one of the parameters is NULL
    /// \retval IFXRFE_E_TIMEOUT if rft pin does not change in time (IFXRFE_RFT_TIMEOUT_DEF)
    /// \retval IFXRFE_E_UNEXPECTED_VALUE if received crc16 ok, but payload would not fit into buffer
    /// \note error code of the underlying platform spi layer otherwise
    error_t SpiTransfer(uint32_t *wbuf, uint8_t wlen, uint32_t *rbuf, uint8_t *rlen);

    /// \brief Transmit command and receive response in half duplex mode (send actual command and receive response for it)
    /// \note The response payload length is determined automatically from the received header
    /// \note The function is waiting for the RFT pin to be set, blockingly
    /// \param wbuf Pointer to data to be transmitted
    /// \param wlen Nr. of words to be transmitted
    /// \param rbuf Buffer to receive into
    /// \param rlen [out] Nr. of received words
    /// \retval IFXRFE_E_SUCCESS if ok
    /// \note Error code returned from \ref SpiSend or \ref SpiReceive otherwise
    error_t SpiSendReceive(uint32_t *wbuf, uint8_t wlen, uint32_t *rbuf, uint8_t *rlen);

    /// \brief Send command without waiting for the response
    /// \note The function is waiting for the RFT pin to be set, blockingly
    /// \param wbuf Data to be transmitted
    /// \param wlen Nr. of data words to be transmitted
    /// \retval IFXRFE_E_SUCCESS if ok
    /// \retval IFXRFE_E_MISSING_PARAMETER if command is NULL
    /// \retval IFXRFE_E_TIMEOUT if rft pin does not change in time (IFXRFE_RFT_TIMEOUT_DEF)
    /// \note error code returned from \ref Wrapper_SpiWrite if error happens during spi write
    error_t SpiSend(uint32_t *wbuf, uint8_t wlen);

    /// \brief Transmit doNop command and receive response to previous command
    /// \note The response payload length is determined automatically from the received header
    /// \note The function is waiting for the RFT pin to be set, blockingly
    /// \param rbuf The buffer to write the received data words into
    /// \param rlen [out] Nr. of received data words
    /// \retval IFXRFE_E_SUCCESS if ok
    /// \retval IFXRFE_E_MISSING_PARAMETER if any of the pointer parameters is NULL
    /// \retval IFXRFE_E_TIMEOUT if rft pin does not change in time (IFXRFE_RFT_TIMEOUT_DEF)
    /// \note error code returned from \ref SpiTransfer otherwise
    error_t SpiReceive(uint32_t *rbuf, uint8_t *rlen);

#ifdef __cplusplus
}
#endif

#endif  //IFXRFE_SPI_H
