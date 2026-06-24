/*
 * File: TCA9539.h
 * Description: Driver for the TCA9539 I2C port expander
 * Project: MIMOrad
 * Created Date: Monday, 13 January 2025, 09:58
 * Author: Daniel Klepatsch (daniel.klepatsch@silicon-austria.com)
 * ---------------
 * Last Modified: Wednesday, 15 January 2025, 14:00
 * Modified By: Daniel Klepatsch
 * ---------------
 * Copyright (c) 2025 Silicon Austria Labs GmbH
 */

#ifndef TCA9539_H
#define TCA9539_H

#include <stdint.h>

#define TCA9539_DIR_OUTPUT  0
#define TCA9539_DIR_INPUT   1
#define TCA9539_POL_INV_OFF 0
#define TCA9539_POL_INV_ON  1
#define TCA9539_OUTPUT_LOW  0
#define TCA9539_OUTPUT_HIGH 1

#ifdef __cplusplus
extern "C"
{
#endif

    // Sets the I2C device address
    void TCA9539_init(uint8_t const i2cAddress);

    // Portpin = ((Port << 4) | Pin)

    int TCA9539_getInputPin(uint8_t const port, uint8_t const pin, uint8_t *value);
    int TCA9539_getInputPort(uint8_t const port, uint8_t *value);
    int TCA9539_getInputPortpin(uint8_t const portpin, uint8_t *value);

    int TCA9539_setOutputPin(uint8_t const port, uint8_t const pin, uint8_t const value);
    int TCA9539_setOutputPort(uint8_t const port, uint8_t const value);
    int TCA9539_setOutputPortpin(uint8_t const portpin, uint8_t const value);
    int TCA9539_getOutputPin(uint8_t const port, uint8_t const pin, uint8_t *value);
    int TCA9539_getOutputPort(uint8_t const port, uint8_t *value);
    int TCA9539_getOutputPortpin(uint8_t const portpin, uint8_t *value);

    int TCA9539_setPolarityInversionPin(uint8_t const port, uint8_t const pin, uint8_t const value);
    int TCA9539_setPolarityInversionPort(uint8_t const port, uint8_t const value);
    int TCA9539_setPolarityInversionPortpin(uint8_t const portpin, uint8_t const value);
    int TCA9539_getPolarityInversionPin(uint8_t const port, uint8_t const pin, uint8_t *value);
    int TCA9539_getPolarityInversionPort(uint8_t const port, uint8_t *value);
    int TCA9539_getPolarityInversionPortpin(uint8_t const portpin, uint8_t *value);

    int TCA9539_setDirectionPin(uint8_t const port, uint8_t const pin, uint8_t const value);
    int TCA9539_setDirectionPort(uint8_t const port, uint8_t const value);
    int TCA9539_setDirectionPortpin(uint8_t const portpin, uint8_t const value);
    int TCA9539_getDirectionPin(uint8_t const port, uint8_t const pin, uint8_t *value);
    int TCA9539_getDirectionPort(uint8_t const port, uint8_t *value);
    int TCA9539_getDirectionPortpin(uint8_t const portpin, uint8_t *value);

#ifdef __cplusplus
}
#endif

#endif  // TCA9539_H
