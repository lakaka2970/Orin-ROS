/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#ifndef IFXRFE_TIMEWRAPPER_H
#define IFXRFE_TIMEWRAPPER_H 1

#include "IfxRfe_ErrorDefinitions.h"
#include "IfxRfe_TimeConfig.h"

#include <stdint.h>

/// \brief Initialize the time wrapper
/// \param fncs Collection of function pointers to access the time functions
void InitTimeWrapper(IfxRfe_timeFunctions_t fncs);

/// \brief Get the deadline in timer units
/// \param timeout_ms Timeout defined in ms
/// \param deadLine [out] Deadline in timer units. Deadline = now + timeout_ms
/// \retval IFXRFE_E_SUCCESS if ok
/// \retval IFXRFE_E_INVALID_CONFIGURATION if pointer configured in \ref InitTimeWrapper is missing
error_t Wrapper_GetDeadLine(uint32_t timeout_ms, int64_t *deadLine);

/// \brief Get the current system timer
/// \param now [out] Current time in time units
/// \retval IFXRFE_E_SUCCESS if ok
/// \retval IFXRFE_E_INVALID_CONFIGURATION if pointer configured in \ref InitTimeWrapper is missing
error_t Wrapper_Now(int64_t *now);

/// \brief Wait for specified time blockingly
/// \param timeout_ms Time to wait in ms
/// \retval IFXRFE_E_SUCCESS if ok
/// \retval IFXRFE_E_INVALID_CONFIGURATION if pointer configured in \ref InitTimeWrapper is missing
error_t Wrapper_WaitTime(uint32_t timeout_ms);

#endif
