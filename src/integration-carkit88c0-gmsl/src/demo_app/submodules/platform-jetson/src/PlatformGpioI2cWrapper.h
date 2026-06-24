/*
 * File: PlatformGpioI2cWrapper.h
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


#ifndef PLATFORMGPIOI2CWRAPPER_H
#define PLATFORMGPIOI2CWRAPPER_H

#include <stdbool.h>
#include <stdint.h>

// Used to differentiate between I2C GPIOs and actual GPIOs
#define TCA9539_GPIO_IDENTIFIER 0xFF

#ifdef __cplusplus
extern "C"
{
#endif

    int PlatformGpioI2cWrapper_get(uint16_t id, bool *state);
    int PlatformGpioI2cWrapper_set(uint16_t id, bool state);
    int PlatformGpioI2cWrapper_configure(uint16_t id, uint8_t flags);

#ifdef __cplusplus
}
#endif

#endif  // PLATFORMGPIOI2CWRAPPER_H