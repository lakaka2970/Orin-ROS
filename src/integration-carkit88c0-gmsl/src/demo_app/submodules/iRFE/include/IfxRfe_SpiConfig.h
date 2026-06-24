/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#ifndef IFXRFE_SPICONFIG_H
#define IFXRFE_SPICONFIG_H 1

#include <stdbool.h>
#include <stdint.h>

#define IFXRFE_SPI_CPOL (1u << 0)                              ///< Clock Polarity CPOL = 1 (otherwise, CPOL = 0)
#define IFXRFE_SPI_CPHA (1u << 1)                              ///< Clock Phase CPHA = 1 (otherwise, CPHA = 0)

#define IFXRFE_SPI_MODE_0 (0 | 0)                              ///< Clock Polarity CPOL = 0, Clock Phase CPHA = 0
#define IFXRFE_SPI_MODE_1 (0 | IFXRFE_SPI_CPHA)                ///< Clock Polarity CPOL = 0, Clock Phase CPHA = 1
#define IFXRFE_SPI_MODE_2 (IFXRFE_SPI_CPOL | 0)                ///< Clock Polarity CPOL = 1, Clock Phase CPHA = 0
#define IFXRFE_SPI_MODE_3 (IFXRFE_SPI_CPOL | IFXRFE_SPI_CPHA)  ///< Clock Polarity CPOL = 1, Clock Phase CPHA = 1

/// \brief Defines the hardware access functions for the SPI wrapper
typedef struct
{
    /// \brief SPI bidirectional transfer function
    /// \param spiId ID of the assigned SPI peripheral
    /// \param count Nr. of 32bit words to be transmitted
    /// \param bufWrite Pointer to the data to be written
    /// \param bufRead Read buffer
    /// \param keepSel True to keep the CS active after transmission, False to set it inactive after transmission
    /// \return 0 if no errors, error code in case of errors
    ///         Error codes should be returned in the callbacks. Error code translation has to be handled by user.
    ///         \see For the error codes definitions for IRFE driver IfxRfe_ErrorDefinitions.h

    int (*spiTransfer)(uint8_t spiId, uint32_t count, const uint32_t bufWrite[], uint32_t bufRead[], bool keepSel);

    /// \brief SPI unidirectional write function
    /// \param spiId ID of the assigned SPI peripheral
    /// \param count Nr. of 32bit words to be written
    /// \param buffer Pointer to the data to be written
    /// \param keepSel True to keep the CS active after transmission, False to set it inactive after transmission
    /// \return 0 if no errors, error code in case of errors
    ///         Error codes should be returned in the callbacks. Error code translation has to be handled by user.
    ///         \see For the error codes definitions for IRFE driver IfxRfe_ErrorDefinitions.h
    int (*spiWrite)(uint8_t spiId, uint32_t count, const uint32_t buffer[], bool keepSel);

    /// \brief SPI peripheral configuration function
    /// \note CS is low active, MSB first, wordsize is 32bits
    /// \param spiId ID of the assigned SPI peripheral
    /// \param flags Settings to be applied, see above
    /// \param speed Baudrate in bits/s
    /// \return 0 if no errors, error code in case of errors
    ///         Error codes should be returned in the callbacks. Error code translation has to be handled by user.
    ///         \see For the error codes definitions for IRFE driver IfxRfe_ErrorDefinitions.h
    int (*spiConfigure)(uint8_t spiId, uint8_t flags, uint32_t speed);
} IfxRfe_spiFunctions_t;

/// \brief Set the number of SPI retransmissions attempts on CRC/CRI errors.
/// \param count Number of automatic retransmissions attempts, 0 to disable automatic retransmission
/// \note By default automatic retransmission is disabled
void IfxRfe_setSpiRetransmissionCount(uint8_t count);

/// \brief Get the current number of SPI retransmissions attempts on CRC/CRI errors.
/// \return Number of SPI automatic retransmission attempts
uint8_t IfxRfe_spiRetransmissionCount();

#endif  //IFXRFE_SPICONFIG_H
