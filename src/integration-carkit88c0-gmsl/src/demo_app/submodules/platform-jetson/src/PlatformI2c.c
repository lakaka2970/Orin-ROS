/*
 * File: PlatformI2c.c
 * Description: Platform-dependant iRFE I2C API implementation for the NVIDIA Jetson
 * Project: MIMOrad
 * Created Date: Friday, 10 January 2025, 10:55
 * Author: Daniel Klepatsch (daniel.klepatsch@silicon-austria.com)
 * ---------------
 * Last Modified: Wednesday, 05 February 2025, 15:56
 * Modified By: Daniel Klepatsch
 * ---------------
 * Copyright (c) 2025 Silicon Austria Labs GmbH
 */

#include <errno.h>
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "PlatformErrors.h"

#include "PlatformI2c.h"
#include "PlatformLogCallbacks.h"
#include "Util.h"

static int gI2cFd = -1;

int PlatformI2c_init(uint8_t const i2cId)
{
    DBG_PRINTF_L1("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);

    if (gI2cFd >= 0)
    {
        char msg[32];
        snprintf(msg, 32, "I2C: already initialized!");
        PlatformLogCallbacks_error(msg);
        return PLATFORM_JETSON_E_FAILED;
    }

    DBG_PRINTF_L1("    i2cId = %d\n", i2cId);

    // Generate device file name
    char i2cdevname[32];
    sprintf(i2cdevname, "/dev/i2c-%d", i2cId);

    DBG_PRINTF_L1("    i2cdevname = %s\n", i2cdevname);

    gI2cFd = open(i2cdevname, O_RDWR);
    if (gI2cFd < 0)
    {
        char msg[128];
        snprintf(msg, 128, "I2C: open() returned %d, errno = %d (%s)", gI2cFd, errno, strerror(errno));
        PlatformLogCallbacks_error(msg);
        return PLATFORM_JETSON_E_FAILED;
    }

    // check adapter functionality
    unsigned long funcs;
    int sysret = ioctl(gI2cFd, I2C_FUNCS, &funcs);
    if (sysret < 0)
    {
        char msg[128];
        snprintf(msg, 128, "I2C: ioctl() for I2C_FUNCS returned %d, errno = %d (%s)", sysret, errno, strerror(errno));
        PlatformLogCallbacks_error(msg);
        PlatformI2c_deinit();
        return PLATFORM_JETSON_E_FAILED;
    }

    if ((funcs & I2C_FUNC_SMBUS_I2C_BLOCK) != I2C_FUNC_SMBUS_I2C_BLOCK)
    {
        char msg[128];
        snprintf(msg, 128, "I2C adapter is missing functionality: I2C_FUNC_SMBUS_I2C_BLOCK");
        PlatformLogCallbacks_error(msg);
        PlatformI2c_deinit();
        return PLATFORM_JETSON_E_FAILED;
    }

    return PLATFORM_JETSON_E_SUCCESS;
}

void PlatformI2c_deinit()
{
    if (gI2cFd >= 0)
    {
        close(gI2cFd);
        gI2cFd = -1;
    }
}

int PlatformI2c_writeWithoutPrefix(uint16_t devAddr, uint16_t length, uint8_t const buffer[])
{
    DBG_PRINTF_L1("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);
    DBG_PRINTF_L1("    devAddr = 0x%02X\n", devAddr);
    DBG_PRINTF_L1("    length = %d\n", length);
    if (buffer != NULL && length > 0)
    {
        DBG_PRINTF_L1("    buffer:\n");
        DBG_PRINTF_L1("        ");
        for (size_t i = 0; i < length; i++)
        {
            if (i == 0)
            {
                DBG_PRINTF_L1("0x%02X", buffer[i]);
            }
            else
            {
                DBG_PRINTF_L1(", 0x%02X", buffer[i]);
            }
        }
        DBG_PRINTF_L1("\n");
    }

    // Check initialisation state
    if (gI2cFd < 0)
    {
        char msg[32];
        snprintf(msg, 32, "I2C not initialized");
        PlatformLogCallbacks_error(msg);
        return PLATFORM_JETSON_E_NOT_INITIALIZED;
    }

    struct i2c_msg msg              = {.addr = devAddr, .flags = 0, .len = length, .buf = (uint8_t *)buffer};
    struct i2c_rdwr_ioctl_data rdwr = {.msgs = &msg, .nmsgs = 1};

    int sysret = ioctl(gI2cFd, I2C_RDWR, &rdwr);
    if (sysret < 0)
    {
        char msg[128];
        snprintf(msg, 128, "I2C could not write: ioctl() returned %d, errno = %d (%s)", sysret, errno, strerror(errno));
        PlatformLogCallbacks_error(msg);
        return PLATFORM_JETSON_E_FAILED;
    }

    return PLATFORM_JETSON_E_SUCCESS;
}

int PlatformI2c_readWithPrefix(uint16_t devAddr, uint32_t prefix, uint16_t length, uint8_t buffer[], uint8_t prefixBytes)
{
    DBG_PRINTF_L1("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);
    DBG_PRINTF_L1("    devAddr = 0x%02X\n", devAddr);
    DBG_PRINTF_L1("    prefix = 0x%04X\n", prefix);
    DBG_PRINTF_L1("    length = %d\n", length);

    if (prefixBytes > 4)
    {
        char msg[64];
        snprintf(msg, 64, "Invalid prefix bytes count: %d", prefixBytes);
        PlatformLogCallbacks_error(msg);
        return PLATFORM_JETSON_E_INVALID_PARAMETER;
    }

    // Check initialisation state
    if (gI2cFd < 0)
    {
        char msg[32];
        snprintf(msg, 32, "I2C not initialized");
        PlatformLogCallbacks_error(msg);
        return PLATFORM_JETSON_E_NOT_INITIALIZED;
    }

    uint8_t addr_buf[4] = {
        (uint8_t)((prefix >> 24) & 0xFF),
        (uint8_t)((prefix >> 16) & 0xFF),
        (uint8_t)((prefix >> 8) & 0xFF),
        (uint8_t)(prefix & 0xFF),
    };

    struct i2c_msg msgs[2];
    msgs[0].addr  = devAddr;
    msgs[0].flags = 0;
    msgs[0].len   = prefixBytes;
    msgs[0].buf   = (addr_buf + (4 - prefixBytes));

    msgs[1].addr                    = devAddr;
    msgs[1].flags                   = I2C_M_RD;
    msgs[1].len                     = length;
    msgs[1].buf                     = buffer;
    struct i2c_rdwr_ioctl_data rdwr = {.msgs = msgs, .nmsgs = 2};

    int sysret = ioctl(gI2cFd, I2C_RDWR, &rdwr);
    if (sysret < 0)
    {
        char msg[128];
        snprintf(msg, 128, "I2C could not read: ioctl() returned %d, errno = %d (%s)", sysret, errno, strerror(errno));
        PlatformLogCallbacks_error(msg);
        return PLATFORM_JETSON_E_FAILED;
    }

    if (buffer != NULL && length > 0)
    {
        DBG_PRINTF_L1("    read_buffer:\n");
        DBG_PRINTF_L1("        ");
        for (size_t i = 0; i < length; i++)
        {
            if (i == 0)
            {
                DBG_PRINTF_L1("0x%02X", buffer[i]);
            }
            else
            {
                DBG_PRINTF_L1(", 0x%02X", buffer[i]);
            }
        }
        DBG_PRINTF_L1("\n");
    }

    return PLATFORM_JETSON_E_SUCCESS;
}

int PlatformI2c_readWith8BitPrefix(uint16_t devAddr, uint8_t prefix, uint16_t length, uint8_t buffer[])
{
    return PlatformI2c_readWithPrefix(devAddr, prefix, length, buffer, 1);
}

int PlatformI2c_readWith16BitPrefix(uint16_t devAddr, uint16_t prefix, uint16_t length, uint8_t buffer[])
{
    return PlatformI2c_readWithPrefix(devAddr, prefix, length, buffer, 2);
}

int PlatformI2c_readWith32BitPrefix(uint16_t devAddr, uint32_t prefix, uint16_t length, uint8_t buffer[])
{
    return PlatformI2c_readWithPrefix(devAddr, prefix, length, buffer, 4);
}

int PlatformI2c_writeWithPrefix(uint8_t devAddr, uint32_t prefix, uint16_t length, uint8_t const buffer[], uint8_t addressBytes)
{
    DBG_PRINTF_L1("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);
    DBG_PRINTF_L1("    devAddr = 0x%02X\n", devAddr);
    DBG_PRINTF_L1("    regAddr = 0x%02X\n", regAddr);
    DBG_PRINTF_L1("    length = %d\n", length);

    uint8_t write_buffer[255 + addressBytes];
    uint8_t offset = 0;

    for (uint8_t i = 0; i < addressBytes; i++)
    {
        write_buffer[offset] = (prefix >> (8 * (addressBytes - 1 - i))) & 0xFF;
        offset++;
    }
    for (size_t i = 0; i < length; i++)
    {
        write_buffer[offset] = buffer[i];
        offset++;
    }
    return PlatformI2c_writeWithoutPrefix(devAddr, offset, write_buffer);
}

int PlatformI2c_writeWith8BitPrefix(uint8_t devAddr, uint8_t prefix, uint16_t length, uint8_t const buffer[])
{
    return PlatformI2c_writeWithPrefix(devAddr, prefix, length, buffer, 1);
}

int PlatformI2c_writeWith16BitPrefix(uint8_t devAddr, uint16_t prefix, uint16_t length, uint8_t const buffer[])
{
    return PlatformI2c_writeWithPrefix(devAddr, prefix, length, buffer, 2);
}

int PlatformI2c_writeWith32BitPrefix(uint8_t devAddr, uint32_t prefix, uint16_t length, uint8_t const buffer[])
{
    return PlatformI2c_writeWithPrefix(devAddr, prefix, length, buffer, 4);
}
