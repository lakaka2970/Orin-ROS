/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */

#ifndef IFXRFE_ERROR_DEFINITIONS_H
#define IFXRFE_ERROR_DEFINITIONS_H 1

typedef int error_t;

/// \brief Error Code definition used by the iRfe
/// \details The following codes will be returned by the iRfe
///          if a function returns error_t. To be prepared for
///          future the range between 0x00 and 0x30 is reserved.
#define IFXRFE_E_SUCCESS               0x00
#define IFXRFE_E_FAILED                0x01
#define IFXRFE_E_INVALID_SIZE          0x02
#define IFXRFE_E_INVALID_PARAMETER     0x03
#define IFXRFE_E_MISSING_PARAMETER     0x04
#define IFXRFE_E_UNEXPECTED_VALUE      0x05
#define IFXRFE_E_OUT_OF_BOUNDS         0x06
#define IFXRFE_E_OVERFLOW              0x07
#define IFXRFE_E_TIMEOUT               0x08
#define IFXRFE_E_BUSY                  0x09  //Resource busy, process still or already running
#define IFXRFE_E_ABORTED               0x0A
#define IFXRFE_E_CRC16                 0x0B  //header crc does not match
#define IFXRFE_E_CRC32                 0x0C  //payload crc does not match
#define IFXRFE_E_INVALID_CONFIGURATION 0x0D  //a value that was configured previously is invalid
#define IFXRFE_E_CRI_MISMATCH          0x0E  //received cri does not match expected
#define IFXRFE_E_OPCODE_MISMATCH       0x0F  //received opcode does not match expected
#define IFXRFE_E_NOT_IMPLEMENTED       0x10
#define IFXRFE_E_NOT_POSSIBLE          0x11
#define IFXRFE_E_NOT_AVAILABLE         0x12
#define IFXRFE_E_NOT_ALLOWED           0x13
#define IFXRFE_E_NOT_INITIALIZED       0x14
#define IFXRFE_E_NOT_CONFIGURED        0x15
#define IFXRFE_E_NOT_OPEN              0x16
#define IFXRFE_E_NOT_SUPPORTED         0x17
#define IFXRFE_E_SPI_MSG_ERROR         0x18  //received payload indicates SPI MSG error
#define IFXRFE_E_COMMAND_ERROR         0x19  //received payload indicates command error code


//Evaluate the return value of a function call and return it, if it is not 0
#define IFXRFE_RETURN_ON_ERROR(expression) \
    {                                      \
        const error_t ret = expression;    \
        if (ret != IFXRFE_E_SUCCESS)       \
        {                                  \
            return ret;                    \
        }                                  \
    }

//Evaluate the return value of a function call and return it after a cleanup, if it is not 0
#define IFXRFE_CLEAN_RETURN_ON_ERROR(expression, cleanup) \
    {                                                     \
        const error_t ret = expression;                   \
        if (ret != IFXRFE_E_SUCCESS)                      \
        {                                                 \
            cleanup;                                      \
            return ret;                                   \
        }                                                 \
    }

#endif /* IFXRFE_ERROR_DEFINITIONS_H */