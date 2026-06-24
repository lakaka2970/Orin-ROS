/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#ifndef IFXRFE_GPIOCONFIG_H
#define IFXRFE_GPIOCONFIG_H 1

#include <stdbool.h>
#include <stdint.h>

/// \brief Defines the GPIO id-s needed for the basic iRFE operation
typedef struct
{
    uint16_t spiRftId;
    uint16_t okId;
    uint16_t dmux1Id;
    uint16_t dmux2Id;
    uint16_t dmux3Id;
} IfxRfe_gpioDefinitions_t;

/// \brief Defines the hardware access functions for the GPIO wrapper
typedef struct
{
    /// \brief Function should return the GPIO state
    /// \param id Gpio id
    /// \param state [out] True for high, False for low
    /// \return 0 if ok, error code otherwise
    ///         Error codes should be returned in the callbacks. Error code translation has to be handled by user.
    ///         \see For the error codes definitions for IRFE driver IfxRfe_ErrorDefinitions.h
    int (*gpioGet)(uint16_t id, bool *state);

    /// \brief Function should set the GPIO state
    /// \param id Gpio id
    /// \param state State to be set, True for high, False for low
    /// \return 0 if ok, error code otherwise
    ///         Error codes should be returned in the callbacks. Error code translation has to be handled by user.
    ///         \see For the error codes definitions for IRFE driver IfxRfe_ErrorDefinitions.h
    int (*gpioSet)(uint16_t id, bool state);

    /// \brief Function should configure the GPIO
    /// \param id Gpio id
    /// \param flags Flags to be set according to your platform. The following flags are used in A2G ILLDs
    ///              1 : GPIO_FLAG_OUTPUT_INITIAL_HIGH
    ///              2 : GPIO_FLAG_OUTPUT_DRIVE_LOW
    ///              4 : GPIO_FLAG_OUTPUT_DRIVE_HIGH
    ///              8 : GPIO_FLAG_INPUT_ENABLE
    ///              16: GPIO_FLAG_PULL_UP
    ///              32: GPIO_FLAG_PULL_DOWN
    /// \return 0 if ok, error code otherwise
    ///         Error codes should be returned in the callbacks. Error code translation has to be handled by user.
    ///         \see For the error codes definitions for IRFE driver IfxRfe_ErrorDefinitions.h
    int (*gpioConfigure)(uint16_t id, uint8_t flags);

    /// \brief Determines whether a flag config is output or input
    /// \param flags Flag configuration
    /// \return True if output, False if input
    bool (*isGpioConfigOutput)(uint8_t flags);

    /// \brief Function should wait for a GPIO to reach a certain state
    /// \note This function pointer may be NULL to use the IRFE default implementation.
    /// \note It is intended, if there is specific wait functionality available for the platform.
    /// \param id Gpio id
    /// \param state Specifies the state to wait for, True for high, False for low
    /// \param timeoutMs The wait timeout in milliseconds
    /// \return 0 if ok, error code otherwise
    ///         Error codes should be returned in the callbacks. Error code translation has to be handled by user.
    ///         \see For the error codes definitions for IRFE driver IfxRfe_ErrorDefinitions.h
    int (*gpioWait)(uint16_t id, bool state, uint32_t timeoutMs);
} IfxRfe_gpioFunctions_t;

#endif  //IFXRFE_GPIOCONFIG_H
