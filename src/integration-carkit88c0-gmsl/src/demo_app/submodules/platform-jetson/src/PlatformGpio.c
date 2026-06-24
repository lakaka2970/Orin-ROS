/*
 * File: PlatformGpio.c
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

#include <errno.h>
#include <gpiod.h>
#include <string.h>

#include "PlatformErrors.h"
#include "Util.h"

#include "PlatformGpio.h"
#include "PlatformTime.h"

typedef struct gpiod_chip gpiod_chip;
typedef struct gpiod_line gpiod_line;

#define MAX_NUM_CHIPS 2    // Jetson AGX Orin has 2 gpiochips
#define MAX_NUM_LINES 164  // Jetson AGX Orin has at max 164 lines per gpiochip

static gpiod_chip *gChips[MAX_NUM_CHIPS];
static gpiod_line *gLines[MAX_NUM_CHIPS][MAX_NUM_LINES];

static char const *const gcConsumer = "MIMOrad";

int PlatformGpio_get(uint16_t id, bool *state)
{
    DBG_PRINTF_L1("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);
    DBG_PRINTF_L1("    id        = 0x%X\n", id);

    // Decode pin id to chip id and pin offset
    uint8_t const chipId    = ((id >> 8) & 0xFF);
    uint8_t const pinOffset = (id & 0xFF);

    DBG_PRINTF_L1("    chipId    = %d\n", chipId);
    DBG_PRINTF_L1("    pinOffset = %d\n", pinOffset);

    // Check id parameter
    if (chipId >= MAX_NUM_CHIPS || pinOffset >= MAX_NUM_LINES)
    {
        DBG_PRINTF_L1("    Error: invalid pin id!\n\n");
        return PLATFORM_JETSON_E_INVALID_PARAMETER;
    }

    // Check initailization state
    if (gChips[chipId] == NULL || gLines[chipId][pinOffset] == NULL)
    {
        DBG_PRINTF_L1("    Error: chip or line not configured!\n\n");
        return PLATFORM_JETSON_E_NOT_CONFIGURED;
    }

    // Check wether pin is input
    // NOTE: technically it is possible to get the state from an output pin (i.e.: get
    //       the last set value), but this could lead to endless loops with incorrectly
    //       configured pins.
    if (gpiod_line_direction(gLines[chipId][pinOffset]) != GPIOD_LINE_DIRECTION_INPUT)
    {
        DBG_PRINTF_L1("    Error: not allowed to get value from output pin!\n\n");
        return PLATFORM_JETSON_E_NOT_ALLOWED;
    }

    int value = gpiod_line_get_value(gLines[chipId][pinOffset]);
    if (value < 0)
    {
        DBG_PRINTF_L1("    Error: 0x%X (%s)\n\n", errno, strerror(errno));
        return PLATFORM_JETSON_E_FAILED;
    }

    DBG_PRINTF_L1("    value     = %d\n", value);

    *state = value;

    DBG_PRINTF_L1("\n");

    return PLATFORM_JETSON_E_SUCCESS;
}

int PlatformGpio_set(uint16_t id, bool state)
{
    DBG_PRINTF_L1("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);
    DBG_PRINTF_L1("    id        = 0x%X\n", id);
    DBG_PRINTF_L1("    state     = %d\n", state);

    // Decode pin id to chip id and pin offset
    uint8_t const chipId    = ((id >> 8) & 0xFF);
    uint8_t const pinOffset = (id & 0xFF);

    DBG_PRINTF_L1("    chipId    = %d\n", chipId);
    DBG_PRINTF_L1("    pinOffset = %d\n", pinOffset);

    // Check id parameter
    if (chipId >= MAX_NUM_CHIPS || pinOffset >= MAX_NUM_LINES)
    {
        DBG_PRINTF_L1("    Error: invalid pin id!\n\n");
        return PLATFORM_JETSON_E_INVALID_PARAMETER;
    }

    // Check initailization state
    if (gChips[chipId] == NULL || gLines[chipId][pinOffset] == NULL)
    {
        DBG_PRINTF_L1("    Error: chip or line not configured!\n\n");
        return PLATFORM_JETSON_E_NOT_CONFIGURED;
    }

    // Check wether pin is output
    if (gpiod_line_direction(gLines[chipId][pinOffset]) != GPIOD_LINE_DIRECTION_OUTPUT)
    {
        DBG_PRINTF_L1("    Error: cannot set value to input pin!\n\n");
        return PLATFORM_JETSON_E_NOT_ALLOWED;
    }

    if (gpiod_line_set_value(gLines[chipId][pinOffset], state) < 0)
    {
        DBG_PRINTF_L1("    Error: 0x%X (%s)\n\n", errno, strerror(errno));
        return PLATFORM_JETSON_E_FAILED;
    }

    DBG_PRINTF_L1("\n");

    return PLATFORM_JETSON_E_SUCCESS;
}

int PlatformGpio_configure(uint16_t id, uint8_t flags)
{
    DBG_PRINTF_L1("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);
    DBG_PRINTF_L1("    id        = 0x%X\n", id);
    DBG_PRINTF_L1("    flags     = 0x%X\n", flags);

    // Decode pin id to chip id and pin offset
    uint8_t const chipId    = ((id >> 8) & 0xFF);
    uint8_t const pinOffset = (id & 0xFF);

    DBG_PRINTF_L1("    chipId    = %d\n", chipId);
    DBG_PRINTF_L1("    pinOffset = %d\n", pinOffset);

    // Check id parameter
    if (chipId >= MAX_NUM_CHIPS || pinOffset >= MAX_NUM_LINES)
    {
        return PLATFORM_JETSON_E_INVALID_PARAMETER;
    }

    // Check flags parameter
    int ret = PlatformGpio_checkFlags(flags);
    if (ret != PLATFORM_JETSON_E_SUCCESS)
    {
        return ret;
    }

    // Open chip if necessary
    if (gChips[chipId] == NULL)
    {
        DBG_PRINTF_L1("    opening chip with id: %d\n", chipId);
        gChips[chipId] = gpiod_chip_open_by_number(chipId);
        if (gChips[chipId] == NULL)
        {
            DBG_PRINTF_L1("        Error: 0x%X (%s)\n\n", errno, strerror(errno));
            return PLATFORM_JETSON_E_FAILED;
        }
    }
    else
    {
        DBG_PRINTF_L1("    chip with id %d already open\n", chipId);
    }

    // Open line if necessary
    if (gLines[chipId][pinOffset] == NULL)
    {
        DBG_PRINTF_L1("    opening line with offset: %d\n", pinOffset);
        gLines[chipId][pinOffset] = gpiod_chip_get_line(gChips[chipId], pinOffset);
        if (gLines[chipId][pinOffset] == NULL)
        {
            DBG_PRINTF_L1("        Error: 0x%X (%s)\n\n", errno, strerror(errno));
            return PLATFORM_JETSON_E_FAILED;
        }
    }
    else
    {
        DBG_PRINTF_L1("    line with offset %d already open, releasing before reconfiguring\n", pinOffset);
        gpiod_line_release(gLines[chipId][pinOffset]);
    }

    int config_flags = 0;
    if (flags & GPIO_FLAG_PULL_UP)
    {
        config_flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
    }
    else if (flags & GPIO_FLAG_PULL_DOWN)
    {
        config_flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
    }
    else
    {
        config_flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE;
    }

    struct gpiod_line_request_config config =
        {
            .consumer     = gcConsumer,
            .request_type = (flags & GPIO_FLAG_INPUT_ENABLE) ? GPIOD_LINE_REQUEST_DIRECTION_INPUT : GPIOD_LINE_REQUEST_DIRECTION_OUTPUT,
            .flags        = config_flags,
        };

    DBG_PRINTF_L1("    request config:\n");
    DBG_PRINTF_L1("        consumer     = %s\n", config.consumer);
    DBG_PRINTF_L1("        request_type = %d\n", config.request_type);
    DBG_PRINTF_L1("        flags        = 0x%X\n", config.flags);
    DBG_PRINTF_L1("    sending request to line\n");

    // Reserve the line
    int const default_val = (flags & (GPIO_FLAG_OUTPUT_INITIAL_HIGH | GPIO_FLAG_OUTPUT_DRIVE_HIGH)) ? 1 : 0;
    if (gpiod_line_request(gLines[chipId][pinOffset], &config, default_val) < 0)
    {
        DBG_PRINTF_L1("        Error: 0x%X (%s)\n\n", errno, strerror(errno));
        return PLATFORM_JETSON_E_FAILED;
    }

    DBG_PRINTF_L1("    Success!\n\n");

    return PLATFORM_JETSON_E_SUCCESS;
}

bool PlatformGpio_isGpioConfigOutput(uint8_t flags)
{
    if (flags & (GPIO_FLAG_OUTPUT_DRIVE_HIGH | GPIO_FLAG_OUTPUT_DRIVE_LOW | GPIO_FLAG_OUTPUT_INITIAL_HIGH))
    {
        return true;
    }
    return false;
}

void PlatformGpio_deinit()
{
    DBG_PRINTF_L1("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);
    for (size_t chip = 0; chip < MAX_NUM_CHIPS; chip++)
    {
        if (gChips[chip] != NULL)
        {
            for (size_t line = 0; line < MAX_NUM_LINES; line++)
            {
                if (gLines[chip][line] != NULL)
                {
                    DBG_PRINTF_L1("    Releasing line %ld from chip %ld\n", line, chip);
                    gpiod_line_release(gLines[chip][line]);
                    gLines[chip][line] = NULL;
                }
            }
            DBG_PRINTF_L1("    Closing chip %ld\n", chip);
            gpiod_chip_close(gChips[chip]);
            gChips[chip] = NULL;
        }
    }
}

int PlatformGpio_gpioWait(uint16_t id, bool state, uint32_t timeoutMs)
{
    DBG_PRINTF_L1("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);
    DBG_PRINTF_L1("    id        = 0x%X\n", id);
    DBG_PRINTF_L1("    state     = %d\n", state);
    DBG_PRINTF_L1("    timeoutMs = %d\n", timeoutMs);

    int64_t deadline = PlatformTime_getDeadLine(timeoutMs);
    bool gpioState   = false;
    int64_t now;

    do
    {
        RETURN_ON_IFX_ERROR(PlatformGpio_get(id, &gpioState));
        now = PlatformTime_now();
    } while ((gpioState != state) && (now < deadline));

    if (gpioState == state)
    {
        DBG_PRINTF_L1("DEBUG from %s: GPIO reached desired state (%u)\n\n", __func__, state);
        return PLATFORM_JETSON_E_SUCCESS;
    }
    else
    {
        DBG_PRINTF_L1("DEBUG from %s: desired state not reached within %d ms\n\n", __func__, timeoutMs);
        return PLATFORM_JETSON_E_TIMEOUT;
    }
}

int PlatformGpio_checkFlags(uint8_t const flags)
{
    if ((flags & GPIO_FLAG_INPUT_ENABLE) && (flags & (GPIO_FLAG_OUTPUT_INITIAL_HIGH | GPIO_FLAG_OUTPUT_DRIVE_LOW | GPIO_FLAG_OUTPUT_DRIVE_HIGH)))
    {
        // Configured as input, but one or more output flags were set
        return PLATFORM_JETSON_E_INVALID_PARAMETER;
    }
    if (!(flags & GPIO_FLAG_INPUT_ENABLE) && ((flags & GPIO_FLAG_OUTPUT_DRIVE_LOW) && (flags & (GPIO_FLAG_OUTPUT_DRIVE_HIGH | GPIO_FLAG_OUTPUT_INITIAL_HIGH))))
    {
        // Configured as output, but multiple incompatible output flags were set
        return PLATFORM_JETSON_E_INVALID_PARAMETER;
    }
    if (!(flags & (GPIO_FLAG_INPUT_ENABLE | GPIO_FLAG_OUTPUT_DRIVE_HIGH | GPIO_FLAG_OUTPUT_DRIVE_LOW | GPIO_FLAG_OUTPUT_INITIAL_HIGH)))
    {
        // No direction specified
        return PLATFORM_JETSON_E_INVALID_PARAMETER;
    }
    if ((flags & GPIO_FLAG_PULL_UP) && (flags & GPIO_FLAG_PULL_DOWN))
    {
        // Pull-up and pull-down were both selected
        return PLATFORM_JETSON_E_INVALID_PARAMETER;
    }
    return PLATFORM_JETSON_E_SUCCESS;
}