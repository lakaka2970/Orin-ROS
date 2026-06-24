/*
 * File: PlatformI2c.h
 * Description: Platform-dependant iRFE I2C API implementation for the NVIDIA Jetson
 * Project: MIMOrad
 * Created Date: Friday, 10 January 2025, 10:55
 * Author: Daniel Klepatsch (daniel.klepatsch@silicon-austria.com)
 * ---------------
 * Last Modified: Monday, 27 January 2025, 15:17
 * Modified By: Daniel Klepatsch
 * ---------------
 * Copyright (c) 2025 Silicon Austria Labs GmbH
 */

#ifndef PLATFORMI2C_H
#define PLATFORMI2C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /*
 *   i2cId is derived from the /dev/i2c-A device file:
 *       i2cId = A
 */
    int PlatformI2c_init(uint8_t const i2cId);
    void PlatformI2c_deinit();

    int PlatformI2c_writeWithoutPrefix(uint16_t devAddr, uint16_t length, uint8_t const buffer[]);

    int PlatformI2c_readWith8BitPrefix(uint16_t devAddr, uint8_t prefix, uint16_t length, uint8_t buffer[]);
    int PlatformI2c_readWith16BitPrefix(uint16_t devAddr, uint16_t prefix, uint16_t length, uint8_t buffer[]);
    int PlatformI2c_readWith32BitPrefix(uint16_t devAddr, uint32_t prefix, uint16_t length, uint8_t buffer[]);

    int PlatformI2c_writeWith8BitPrefix(uint8_t devAddr, uint8_t prefix, uint16_t length, uint8_t const buffer[]);
    int PlatformI2c_writeWith16BitPrefix(uint8_t devAddr, uint16_t prefix, uint16_t length, uint8_t const buffer[]);
    int PlatformI2c_writeWith32BitPrefix(uint8_t devAddr, uint32_t prefix, uint16_t length, uint8_t const buffer[]);

#ifdef __cplusplus
}
#endif

#endif  // PLATFORMI2C_H
