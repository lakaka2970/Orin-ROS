/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#ifndef IFXRFE_GPIO_H
#define IFXRFE_GPIO_H 1

#include "IfxRfe_ErrorDefinitions.h"
#include <stdbool.h>
#include <stdint.h>

#define IFXRFE_RFT_TIMEOUT_MAX  10000
#define IFXRFE_OK_TIMEOUT_MAX   10000
#define IFXRFE_DMUX_TIMEOUT_MAX 10000
#define IFXRFE_RFT_TIMEOUT_DEF  100

#ifdef __cplusplus
extern "C"
{
#endif

    /// \brief Wait for the RFT (ready for transmit) pin to be set
    /// \param timeoutMs Timeout in milliseconds, max. value IFXRFE_RFT_TIMEOUT_MAX
    /// \retval     IFXRFE_E_INVALID_PARAMETER if timeout invalid or the configured pin is invalid
    /// \retval     IFXRFE_E_TIMEOUT if pin was not set in the requested time
    /// \retval     IFXRFE_E_SUCCESS if everything ok
    error_t IfxRfe_waitForRftPin(uint32_t timeoutMs);

    /// \brief Wait for the OK pin to be set
    /// \param timeoutMs Timeout in milliseconds, max. value IFXRFE_OK_TIMEOUT_MAX
    /// \retval     IFXRFE_E_INVALID_PARAMETER if timeout invalid or the configured pin is invalid
    /// \retval     IFXRFE_E_TIMEOUT if pin was not set in the requested time
    /// \retval     IFXRFE_E_SUCCESS if everything ok
    error_t IfxRfe_waitForOkPin(uint32_t timeoutMs);

    /// \brief Wait for the DMUX1 pin to be set
    /// \param timeoutMs Timeout in milliseconds, max. value IFXRFE_DMUX_TIMEOUT_MAX
    /// \retval     IFXRFE_E_INVALID_PARAMETER if timeout invalid or the configured pin is invalid
    /// \retval     IFXRFE_E_TIMEOUT if pin was not set in the requested time
    /// \retval     IFXRFE_E_SUCCESS if everything ok
    error_t IfxRfe_waitForDmux1Pin(uint32_t timeoutMs);

    /// \brief Wait for the DMUX2 pin to be set
    /// \param timeoutMs Timeout in milliseconds, max. value IFXRFE_DMUX_TIMEOUT_MAX
    /// \retval     IFXRFE_E_INVALID_PARAMETER if timeout invalid or the configured pin is invalid
    /// \retval     IFXRFE_E_TIMEOUT if pin was not set in the requested time
    /// \retval     IFXRFE_E_SUCCESS if everything ok
    error_t IfxRfe_waitForDmux2Pin(uint32_t timeoutMs);

#ifdef __cplusplus
}
#endif

#endif  //IFXRFE_GPIO_H
