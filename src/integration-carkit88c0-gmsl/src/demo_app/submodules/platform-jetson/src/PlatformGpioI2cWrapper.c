/*
 * File: PlatformGpioI2cWrapper.c
 * Description: Wrapper that allows to use I2C GPIOs and regular GPIOs with one API
 * Project: MIMOrad
 * Created Date: Wednesday, 15 January 2025, 10:27
 * Author: Daniel Klepatsch (daniel.klepatsch@silicon-austria.com)
 * ---------------
 * Last Modified: Thursday, 16 January 2025, 15:07
 * Modified By: Daniel Klepatsch
 * ---------------
 * Copyright (c) 2025 Silicon Austria Labs GmbH
 */

#include "PlatformErrors.h"

#include "PlatformGpio.h"
#include "PlatformGpioI2cWrapper.h"
#include "TCA9539.h"
#include "Util.h"

#define GPIO_IDENTIFIER(id) ((id >> 8) & 0xFF)

int PlatformGpioI2cWrapper_get(uint16_t id, bool *state)
{
    if (GPIO_IDENTIFIER(id) == TCA9539_GPIO_IDENTIFIER)
    {
        // I2C GPIO
        uint8_t const portpin = (id & 0xFF);
        return TCA9539_getInputPortpin(portpin, (uint8_t *)state);
    }
    else
    {
        // Regular GPIO
        return PlatformGpio_get(id, state);
    }
}

int PlatformGpioI2cWrapper_set(uint16_t id, bool state)
{
    if (GPIO_IDENTIFIER(id) == TCA9539_GPIO_IDENTIFIER)
    {
        // I2C GPIO
        uint8_t const portpin = (id & 0xFF);
        return TCA9539_setOutputPortpin(portpin, state);
    }
    else
    {
        // Regular GPIO
        return PlatformGpio_set(id, state);
    }
}

int PlatformGpioI2cWrapper_configure(uint16_t id, uint8_t flags)
{
    if (GPIO_IDENTIFIER(id) == TCA9539_GPIO_IDENTIFIER)
    {
        // I2C GPIO

        // Pulls not available for I2C GPIO
        if ((flags & GPIO_FLAG_PULL_UP) || (flags & GPIO_FLAG_PULL_DOWN))
        {
            return PLATFORM_JETSON_E_INVALID_PARAMETER;
        }

        // Check flags parameter
        RETURN_ON_IFX_ERROR(PlatformGpio_checkFlags(flags));

        uint8_t const portpin = (id & 0xFF);

        if ((flags & GPIO_FLAG_OUTPUT_INITIAL_HIGH) || (flags & GPIO_FLAG_OUTPUT_DRIVE_HIGH))
        {
            RETURN_ON_IFX_ERROR(TCA9539_setDirectionPortpin(portpin, TCA9539_DIR_OUTPUT));
            RETURN_ON_IFX_ERROR(TCA9539_setOutputPortpin(portpin, TCA9539_OUTPUT_HIGH));
        }
        else if (flags & GPIO_FLAG_OUTPUT_DRIVE_LOW)
        {
            RETURN_ON_IFX_ERROR(TCA9539_setDirectionPortpin(portpin, TCA9539_DIR_OUTPUT));
            RETURN_ON_IFX_ERROR(TCA9539_setOutputPortpin(portpin, TCA9539_OUTPUT_LOW));
        }
        else if (flags & GPIO_FLAG_INPUT_ENABLE)
        {
            RETURN_ON_IFX_ERROR(TCA9539_setDirectionPortpin(portpin, TCA9539_DIR_INPUT));
        }
        else
        {
            return PLATFORM_JETSON_E_INVALID_PARAMETER;
        }
        return PLATFORM_JETSON_E_SUCCESS;
    }
    else
    {
        // Regular GPIO
        return PlatformGpio_configure(id, flags);
    }
}