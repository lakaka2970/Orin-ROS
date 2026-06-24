/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#ifndef IFXRFE_GPIOWRAPPER_H
#define IFXRFE_GPIOWRAPPER_H 1

#include "IfxRfe_ErrorDefinitions.h"
#include "IfxRfe_GpioConfig.h"
#include <stdbool.h>
#include <stdint.h>

/// \brief Sets the pin ids
/// \param fncs Hardware access functions
void InitGpioWrapper(IfxRfe_gpioFunctions_t fncs);

/// \brief Sets the pins for the CTRX in use
/// \param gpios Pointer of the pin ids of CTRX
void SetCtrxPins(const IfxRfe_gpioDefinitions_t *gpios);

/// \brief Get a reference to the CTRX related pin definition structure
/// \return Pointer to the pin definition structure, NULL if not initialized
const IfxRfe_gpioDefinitions_t *getCtrxPinDefinition();


///\brief Get the gpio state
///\param id GPIO id to be read
///\param state [out] State of the read GPIO
///\retval IFXRFE_E_SUCCESS if ok
///\retval IFXRFE_E_INVALID_CONFIGURATION if function pointer configured in \ref InitGpioWrapper is NULL
///\note error code returned from the function pointer otherwise
error_t Wrapper_GpioGet(uint16_t id, bool *state);

///\brief Set the gpio to a state
///\param id GPIO id to be set
///\param state State to be set
///\retval IFXRFE_E_SUCCESS if ok
///\retval IFXRFE_E_INVALID_CONFIGURATION if function pointer configured in \ref InitGpioWrapper is NULL
///\note error code returned from the function pointer otherwise
error_t Wrapper_GpioSet(uint16_t id, bool state);

///\brief Configure gpio
///\param id GPIO id to be configured
///\param flags GPIO configuration \see IfxRfe_GpioConfig.h
///\retval IFXRFE_E_SUCCESS if ok
///\retval IFXRFE_E_INVALID_CONFIGURATION if function pointer configured in \ref InitGpioWrapper is NULL
///\note error code returned from the function pointer otherwise
error_t Wrapper_GpioConfigure(uint16_t id, uint8_t flags);

///\brief Determine whether the configuration is output
///\param flags Pin configuration to check
///\param isOutput [out] True if config output, false otherwise
///\retval IFXRFE_E_SUCCESS if ok
///\retval IFXRFE_E_INVALID_CONFIGURATION if function pointer configured in \ref InitGpioWrapper is NULL
error_t Wrapper_IsGpioConfigOutput(uint8_t flags, bool *isOutput);

///\brief Wait for a gpio to reach a certain state
///\param id GPIO id to be read
/// \param state Specifies the state to wait for, True for high, False for low
/// \param timeoutMs The wait timeout in milliseconds
///\retval IFXRFE_E_SUCCESS if ok
///\retval IFXRFE_E_NOT_CONFIGURED if function pointer configured in \ref InitGpioWrapper is NULL
///\note error code returned from the function pointer otherwise
error_t Wrapper_GpioWait(uint16_t id, bool state, uint32_t timeoutMs);

#endif  //IFXRFE_GPIOWRAPPER_H
