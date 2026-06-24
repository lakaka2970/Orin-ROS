/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */

#ifndef IFXRFE_H
#define IFXRFE_H 1

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "IfxRfe_ErrorDefinitions.h"
#include "IfxRfe_GpioConfig.h"
#include "IfxRfe_LogCallbacks_t.h"
#include "IfxRfe_SpiConfig.h"
#include "IfxRfe_TimeConfig.h"

#ifndef __cplusplus
// Backward compatibility for C projects where only one chip type is supported at a time
#include "IfxRfe_CtrxCommandParams.h"
#include "IfxRfe_Specific.h"
#endif

#include <stdint.h>

/******************************************************************************/
/*----------------------------------Defines-----------------------------------*/
/******************************************************************************/
#define I_PINS_CTRX_DMUX_COUNT 3
#define IFXRFE_VERSION         "1.1.0.0"

#define Q_12_3_FRACTIONAL_BITS                3  // Fractional bits of Q12.3 fixed-point format
#define IFXRFE_CONVERT_TEMP_RAW_TO_REAL(temp) IfxRfe_qToFloat((int16_t)temp, Q_12_3_FRACTIONAL_BITS)

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

    /******************************************************************************/
    /*-------------------------Global Function Prototypes-------------------------*/
    /******************************************************************************/

    /// \brief Initialize the IfxRfe middleware
    /// \param devCount Number of CTRX devices
    /// \param gpios CTRX GPIO pin definitions for one or multiple CTRX devices
    /// \param spiFncs spi access function pointers
    /// \param gpioFncs gpio access function pointers
    /// \param timeFncs time function access pointers
    /// \param logInterface callbacks to the logging functions
    /// \retval IFXRFE_E_SUCCESS on success
    /// \retval IFXRFE_E_OUT_OF_BOUNDS if devId is out of bounds
    error_t IfxRfe_init(uint8_t devCount, const IfxRfe_gpioDefinitions_t *gpios, IfxRfe_spiFunctions_t spiFncs, IfxRfe_gpioFunctions_t gpioFncs, IfxRfe_timeFunctions_t timeFncs, IfxRfe_logCallbacks_t logInterface);

    /// \brief Select the device to be used for RFE IfxRfe middleware
    /// \param devId Device id
    /// \retval IFXRFE_E_SUCCESS on success
    /// \retval IFXRFE_E_NOT_INITIALIZED if IfxRfe not initialized
    /// \retval IFXRFE_E_OUT_OF_BOUNDS if devId is out of bounds
    error_t IfxRfe_selectDevice(uint8_t devId);

    /// \brief A helper function to convert Q Format to Floating Point
    /// \param qValue Q Value to be converted
    /// \param fractionalBits Number of fractional bits
    /// \return Floating point number
    float IfxRfe_qToFloat(int qValue, uint8_t fractionalBits);

    /// \brief A helper function to convert Floating Point to Q Format
    /// \param value Float value to be converted
    /// \param fractionalBits Number of fractional bits
    /// \return Integer
    int IfxRfe_floatToQ(float value, uint8_t fractionalBits);

#ifdef __cplusplus
}
#endif

#endif /* IFXRFE_H */
