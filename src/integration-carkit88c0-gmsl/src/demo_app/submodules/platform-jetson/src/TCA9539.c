/*
 * File: TCA9539.c
 * Description:  Driver for the TCA9539 I2C port expander
 * Project: MIMOrad
 * Created Date: Monday, 13 January 2025, 09:58
 * Author: Daniel Klepatsch (daniel.klepatsch@silicon-austria.com)
 * ---------------
 * Last Modified: Wednesday, 15 January 2025, 09:36
 * Modified By: Daniel Klepatsch
 * ---------------
 * Copyright (c) 2025 Silicon Austria Labs GmbH
 */

#include "PlatformErrors.h"

#include "PlatformI2c.h"
#include "TCA9539.h"

#define REG_INPUT_START   0x00
#define REG_OUTPUT_START  0x02
#define REG_POL_INV_START 0x04
#define REG_CONF_START    0x06

static int gI2cAddress = -1;

static inline int TCA9539_getRegister(uint8_t const reg_start, uint8_t const port, uint8_t *value)
{
    return PlatformI2c_readWith8BitPrefix(gI2cAddress, reg_start + port, 1, value);
}

static inline int TCA9539_setRegister(uint8_t const reg_start, uint8_t const port, uint8_t const value)
{
    return PlatformI2c_writeWith8BitPrefix(gI2cAddress, reg_start + port, 1, &value);
}

static inline int TCA9539_getPin(uint8_t const reg_start, uint8_t const port, uint8_t const pin, uint8_t *value)
{
    int retval = TCA9539_getRegister(reg_start, port, value);
    *value     = ((*value >> pin) & 0x01);
    return retval;
}

static inline int TCA9539_setPin(uint8_t const reg_start, uint8_t const port, uint8_t const pin, uint8_t const value)
{
    // Read current port value
    uint8_t port_value;
    int retval = TCA9539_getRegister(reg_start, port, &port_value);
    if (retval != PLATFORM_JETSON_E_SUCCESS)
    {
        return retval;
    }
    // Clear value on pin
    port_value &= ~(0x1 << pin);
    // Set new value on pin
    port_value |= (value << pin);
    // Set new port value
    return TCA9539_setRegister(reg_start, port, port_value);
}

void TCA9539_init(uint8_t const i2cAddress)
{
    gI2cAddress = i2cAddress;
}

int TCA9539_getInputPin(uint8_t const port, uint8_t const pin, uint8_t *value)
{
    return TCA9539_getPin(REG_INPUT_START, port, pin, value);
}

int TCA9539_getInputPort(uint8_t const port, uint8_t *value)
{
    return TCA9539_getRegister(REG_INPUT_START, port, value);
}

int TCA9539_getInputPortpin(uint8_t const portpin, uint8_t *value)
{
    return TCA9539_getInputPin(((portpin >> 4) & 0xF), (portpin & 0xF), value);
}

int TCA9539_setOutputPin(uint8_t const port, uint8_t const pin, uint8_t const value)
{
    return TCA9539_setPin(REG_OUTPUT_START, port, pin, value);
}

int TCA9539_setOutputPort(uint8_t const port, uint8_t const value)
{
    return TCA9539_setRegister(REG_OUTPUT_START, port, value);
}

int TCA9539_setOutputPortpin(uint8_t const portpin, uint8_t const value)
{
    return TCA9539_setOutputPin(((portpin >> 4) & 0xF), (portpin & 0xF), value);
}

int TCA9539_getOutputPin(uint8_t const port, uint8_t const pin, uint8_t *value)
{
    return TCA9539_getPin(REG_OUTPUT_START, port, pin, value);
}

int TCA9539_getOutputPort(uint8_t const port, uint8_t *value)
{
    return TCA9539_getRegister(REG_OUTPUT_START, port, value);
}

int TCA9539_getOutputPortpin(uint8_t const portpin, uint8_t *value)
{
    return TCA9539_getOutputPin(((portpin >> 4) & 0xF), (portpin & 0xF), value);
}

int TCA9539_setPolarityInversionPin(uint8_t const port, uint8_t const pin, uint8_t const value)
{
    return TCA9539_setPin(REG_POL_INV_START, port, pin, value);
}

int TCA9539_setPolarityInversionPort(uint8_t const port, uint8_t const value)
{
    return TCA9539_setRegister(REG_POL_INV_START, port, value);
}

int TCA9539_setPolarityInversionPortpin(uint8_t const portpin, uint8_t const value)
{
    return TCA9539_setPolarityInversionPin(((portpin >> 4) & 0xF), (portpin & 0xF), value);
}

int TCA9539_getPolarityInversionPin(uint8_t const port, uint8_t const pin, uint8_t *value)
{
    return TCA9539_getPin(REG_POL_INV_START, port, pin, value);
}

int TCA9539_getPolarityInversionPort(uint8_t const port, uint8_t *value)
{
    return TCA9539_getRegister(REG_POL_INV_START, port, value);
}

int TCA9539_getPolarityInversionPortpin(uint8_t const portpin, uint8_t *value)
{
    return TCA9539_getPolarityInversionPin(((portpin >> 4) & 0xF), (portpin & 0xF), value);
}

int TCA9539_setDirectionPin(uint8_t const port, uint8_t const pin, uint8_t const value)
{
    return TCA9539_setPin(REG_CONF_START, port, pin, value);
}

int TCA9539_setDirectionPort(uint8_t const port, uint8_t const value)
{
    return TCA9539_setRegister(REG_CONF_START, port, value);
}

int TCA9539_setDirectionPortpin(uint8_t const portpin, uint8_t const value)
{
    return TCA9539_setDirectionPin(((portpin >> 4) & 0xF), (portpin & 0xF), value);
}

int TCA9539_getDirectionPin(uint8_t const port, uint8_t const pin, uint8_t *value)
{
    return TCA9539_getPin(REG_CONF_START, port, pin, value);
}

int TCA9539_getDirectionPort(uint8_t const port, uint8_t *value)
{
    return TCA9539_getRegister(REG_CONF_START, port, value);
}

int TCA9539_getDirectionPortpin(uint8_t const portpin, uint8_t *value)
{
    return TCA9539_getDirectionPin(((portpin >> 4) & 0xF), (portpin & 0xF), value);
}