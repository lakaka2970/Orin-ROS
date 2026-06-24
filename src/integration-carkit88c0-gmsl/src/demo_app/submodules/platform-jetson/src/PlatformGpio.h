/*
 * File: PlatformGpio.h
 * Description: Platform-dependant iRFE GPIO API implementation for the NVIDIA Jetson
 * Project: MIMOrad
 * Created Date: Tuesday, 23 April 2024, 10:07
 * Author: Daniel Klepatsch (daniel.klepatsch@silicon-austria.com)
 * ---------------
 * Last Modified: Wednesday, 15 January 2025, 13:53
 * Modified By: Daniel Klepatsch
 * ---------------
 * Copyright (c) 2024 Silicon Austria Labs GmbH
 */

#ifndef PLATFORMGPIO_H
#define PLATFORMGPIO_H

#include <stdbool.h>
#include <stdint.h>

#define GPIO_FLAG_OUTPUT_INITIAL_HIGH (1 << 0)
#define GPIO_FLAG_OUTPUT_DRIVE_LOW    (1 << 1)
#define GPIO_FLAG_OUTPUT_DRIVE_HIGH   (1 << 2)
#define GPIO_FLAG_INPUT_ENABLE        (1 << 3)
#define GPIO_FLAG_PULL_UP             (1 << 4)
#define GPIO_FLAG_PULL_DOWN           (1 << 5)

// Chip IDs
#define GPIO_CHIP_0_ID 0
#define GPIO_CHIP_1_ID 1

#ifdef __cplusplus
extern "C"
{
#endif

    /*
*   Pin id is derived from the /dev/gpiochipX and the internal offset of the pin
*       id = ((X & 0xFF) << 8) | (PIN_OFFSET & 0xFF)
*/

    int PlatformGpio_get(uint16_t id, bool *state);
    int PlatformGpio_set(uint16_t id, bool state);
    int PlatformGpio_configure(uint16_t id, uint8_t flags);
    bool PlatformGpio_isGpioConfigOutput(uint8_t flags);
    int PlatformGpio_gpioWait(uint16_t id, bool state, uint32_t timeoutMs);

    int PlatformGpio_checkFlags(uint8_t const flags);
    void PlatformGpio_deinit();

#ifdef __cplusplus
}
#endif

#endif  // PLATFORMGPIO_H
