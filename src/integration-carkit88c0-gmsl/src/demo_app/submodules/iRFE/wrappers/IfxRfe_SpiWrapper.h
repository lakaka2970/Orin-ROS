/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#ifndef IFXRFE_SPIWRAPPER_H
#define IFXRFE_SPIWRAPPER_H 1

#include "IfxRfe_ErrorDefinitions.h"
#include "IfxRfe_SpiConfig.h"
#include <stdbool.h>
#include <stdint.h>

/// \brief Inits spi wrapper function callbacks
/// \param fncs Function pointers to the spi functions
void InitSpiWrapper(IfxRfe_spiFunctions_t fncs);

/// \brief Gets the spi device id
/// \return Spi id
uint8_t GetSpiId();

/// \brief Sets the spi device id
/// \param devId Spi id
void SetSpiId(uint8_t devId);

/// \brief SPI bidirectional transfer function
/// \param count Nr. of 32bit words to be transmitted
/// \param bufWrite Pointer to the data to be written
/// \param bufRead Read buffer
/// \param keepSel True to keep the CS active after transmission, False to set it inactive after transmission
/// \retval IFXRFE_E_SUCCESS if ok
/// \retval IFXRFE_E_NOT_CONFIGURED if function pointer configured in \ref InitSpiWrapper is NULL
/// \note error code returned from the function pointer otherwise
error_t Wrapper_SpiTransfer(uint32_t count, const uint32_t bufWrite[], uint32_t bufRead[], bool keepSel);

/// \brief SPI unidirectional write function
/// \param count Nr. of 32bit words to be written
/// \param buffer Pointer to the data to be written
/// \param keepSel True to keep the CS active after transmission, False to set it inactive after transmission
/// \retval IFXRFE_E_SUCCESS if ok
/// \retval IFXRFE_E_NOT_CONFIGURED if function pointer configured in \ref InitSpiWrapper is NULL
/// \note error code returned from the function pointer otherwise
error_t Wrapper_SpiWrite(uint32_t count, const uint32_t buffer[], bool keepSel);

/// \brief SPI peripheral configuration function
/// \note CS is low active, MSB first, wordsize is 32bits
/// \param flags Settings to be applied, see above
/// \param speed Baudrate in bits/s
/// \retval IFXRFE_E_SUCCESS if ok
/// \retval IFXRFE_E_NOT_CONFIGURED if function pointer configured in \ref InitSpiWrapper is NULL
/// \note error code returned from the function pointer otherwise
error_t Wrapper_SpiConfigure(uint8_t flags, uint32_t speed);

#endif  //IFXRFE_SPIWRAPPER_H
