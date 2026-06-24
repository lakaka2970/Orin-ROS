/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#ifndef IFXRFE_TIMECONFIG_H
#define IFXRFE_TIMECONFIG_H 1

#include <stdint.h>

typedef struct
{
    /// \brief Calculate a deadline in timer units
    /// \note It is the callbacks responsibility to convert the timeout from ms to timer units
    /// \param timeoutMs Specifies the deadline. Deadline = Now + Timeout
    /// \return deadline in timer units
    int64_t (*getDeadLine)(uint32_t timeoutMs);

    /// \brief Get system timer value
    /// \return Return the timer value in timer units
    int64_t (*now)(void);

    /// \brief Wait for specified time blockingly
    /// \param timeoutMs Specifies the time to wait before return in Ms
    void (*waitTime)(uint32_t timeoutMs);
} IfxRfe_timeFunctions_t;

#endif  //IFXRFE_TIMECONFIG_H
