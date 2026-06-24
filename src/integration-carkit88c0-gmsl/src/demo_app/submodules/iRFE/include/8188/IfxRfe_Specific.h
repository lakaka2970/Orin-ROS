/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */

#ifndef IFXRFE_SPECIFIC_H_8188
#define IFXRFE_SPECIFIC_H_8188 1

// clang-format off
#define STR1(x)                  #x
#define STR(x)                   STR1(x)
#define INCLUDE_JOIN(path, file) STR(path/file)
// clang-format on

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#ifdef __cplusplus
#include INCLUDE_JOIN(FIRMWARE_PATH_8188, ram_fw.h)
#else
#include <ram_fw.h>
#endif

#include "IfxRfe_CtrxCommandParams.h"
#include "IfxRfe_ErrorDefinitions.h"
#include "IfxRfe_GpioConfig.h"
#include "IfxRfe_LogCallbacks_t.h"
#include "IfxRfe_SpiConfig.h"
#include "IfxRfe_TimeConfig.h"

#include <stdint.h>

#ifdef __cplusplus
namespace IfxRfe_Ctrx8188
{
#endif

    /******************************************************************************/
    /*----------------------------------Defines-----------------------------------*/
    /******************************************************************************/

    typedef t_load_fw_spi_frame fw_spi_frame;

    /******************************************************************************/
    /*-----------------------------Data Structures--------------------------------*/
    /******************************************************************************/

    /// \brief Structure to store the Aurix DMUX pin configurations
    typedef struct
    {
        uint8_t dmux1Flags;
        uint8_t dmux2Flags;
        uint8_t dmux3Flags;
    } IfxRfe_dmuxFlags_t;

    typedef enum
    {
        PRIMARY,    // primary device in a cascaded system
        SECONDARY,  // secondary device in a cascaded system
        STANDALONE  // standalone device in a single device system
    } IfxRfe_deviceConfiguration_t;

    typedef struct
    {
        IfxRfe_setClockOutput_t *clockOutputConfig;            // clock output configuration CLKMCU, CLKPHY - set to NULL if clock config not required
        IfxRfe_deviceConfiguration_t deviceConfig;             // Device configuration (PRIMARY, SECONDARY, STANDALONE)
        uint8_t spiConfigureFlags;                             // Specifies the configuration flags.
        uint32_t spiClockSpeed;                                // SPI clock speed in bits/s
        IfxRfe_configureMmicClock_t configureMmicClockConfig;  // CTRX clock configuration
        IfxRfe_initialize_t initializeConfig;                  // CTRX initialization configuration
    } IfxRfe_ctrxInitConfiguration_t;

    /******************************************************************************/
    /*-------------------------Global Function Prototypes-------------------------*/
    /******************************************************************************/

    /// \brief Download a sequence of ram firmware spi frames
    /// \param ramFw pointer to the spi frames
    /// \param nrFrames number of spi frames
    /// \retval IFXRFE_E_SUCCESS on success
    /// \retval IFXRFE_E_NOT_INITIALIZED if IfxRfe not initialized
    /// \note error code from \ref IfxRfe_downloadFwFunctions_exp on error
    /// \note error code from \ref IfxRfe_triggerReset otherwise (also true for 8181_ES because the finish_fw_download command calls a trigger_reset command internally)
    error_t IfxRfe_loadRamFw(const fw_spi_frame *ramFw, uint32_t nrFrames);

    /// \brief Prepares the configuration passed to IfxRfe_ctrxInit according to the provided device configuration
    /// \param config Device configuration (PRIMARY, SECONDARY, STANDALONE)
    /// \retval configuration to be passed to IfxRfe_ctrxInit
    IfxRfe_ctrxInitConfiguration_t prepareCtrxInitConfig(IfxRfe_deviceConfiguration_t devConfig);

    /// \brief Initialize CTRX by downloading firmware, init clock and set CTRX to low power state
    /// \param config Configuration structure containing the CTRX configuration parameters
    /// \retval IFXRFE_E_SUCCESS on success
    /// \retval IFXRFE_E_FAILED if CTRX not initialized
    /// \retval IFXRFE_E_INVALID_PARAMETER if config is invalid
    /// \note error code from \ref IfxRfe_loadRamFw on error
    /// \note error code from \ref IfxRfe_waitForRftPin on error
    /// \note error code from \ref IfxRfe_setClockInput_exp on error
    /// \note error code from \ref IfxRfe_setClockOutput_exp on error
    /// \note error code from \ref IfxRfe_initialize_exp on error
    /// \note error code from \ref IfxRfe_triggerReset on error
    /// \note error code from \ref IfxRfe_getVersion on error
    /// \note error code from \ref IfxRfe_exitPreOperation_exp on error
    error_t IfxRfe_ctrxInit(IfxRfe_ctrxInitConfiguration_t config);

    /// \brief Helper function to calculate the NMOD value for the \ref IfxRfe_configureRfFrequency_exp function
    /// \param fStatic_MHz Static frequency in MHz
    /// \param fLock_MHz Upper boundary of DPLL frequency range in MHz
    /// \param NMOD calculated NMOD value
    /// \retval IFXRFE_E_SUCCESS on success
    /// \retval IFXRFE_E_MISSING_PARAMETER if nmod is NULL
    /// \retval IFXRFE_E_OUT_OF_BOUNDS if fStatic_MHz is out of bounds
    /// \note The function does not perform any range check on the parameter, refer to the CTRX-8191F Manual for the valid range
    error_t IfxRfe_calculateNmod(float fStatic_MHz, float fLock_MHz, uint32_t *nmodResult);

    /// \brief Helper function to calculate the NCW value for the \ref IfxRfe_configureRfFrequency_exp function
    /// \param fLock_MHz Upper boundary of DPLL frequency range in MHz
    /// \return NCW value
    /// \note The function does not perform any range check on the parameter, refer to the CTRX-8191F Manual for the valid range
    uint32_t IfxRfe_calculateNcw(float fLock_MHz);

    /// \brief Helper function to calculate the RampBW value for the \ref IfxRfe_configureRfFrequency_exp function
    /// \param fBw_MHz Relative frequency range in MHz
    /// \return RampBW value
    /// \note The function does not perform any range check on the parameter, refer to the CTRX-8191F Manual for the valid range
    uint32_t IfxRfe_calculateRampBW(float fBw_MHz);

    /// \brief Write sequencer memory through SPI
    /// \param startAddress Offset address of the first word to be written
    /// \param dataWords Data words to be written
    /// \param length Number of data words to be written (1d .. 30d)
    /// \retval IFXRFE_E_SUCCES on success
    /// \retval IFXRFE_E_NOT_INITIALIZED if IfxRfe not initialized
    /// \retval IFXRFE_E_INVALID_SIZE if length too high or 0
    /// \retval IFXRFE_E_MISSING_PARAMETER if dataWords is NULL
    /// \note error code returned from \ref IfxRfe_fwCall_executeDirectly otherwise
    error_t IfxRfe_writeSequencerMemory(uint32_t startAddress, const uint32_t *dataWords, uint8_t length);

    /// \brief Read sequencer memory through SPI
    /// \param startAddress Offset address of the first word to read
    /// \param length Number of data words to be read (value range: 1d-14d)
    /// \param res Result returned from CTRX
    /// \retval IFXRFE_E_SUCCES on success
    /// \retval IFXRFE_E_NOT_INITIALIZED if IfxRfe not initialized
    /// \retval IFXRFE_E_INVALID_SIZE if length too high or 0
    /// \retval IFXRFE_E_MISSING_PARAMETER if res is NULL
    /// \retval IFXRFE_E_UNEXPECTED_VALUE if spi answer length does not match expected length
    /// \retval IFXRFE_E_FAILED if response payload (length field) indicates different size than requested
    /// \note error code returned from \ref IfxRfe_fwCall_executeDirectly otherwise
    error_t IfxRfe_readSequencerMemory(uint32_t startAddress, uint8_t length, IfxRfe_readMemoryResult_t *res);

    /// \brief Load sequencer setup data into CTRX Memory
    /// \param startAddress Offset address of the first word to be loaded
    /// \param sequencerDataWords pointer of sequencer setup data
    /// \param wordCount size of sequencer data in words (Address Range of Sequencer Setup 0000-7FFF)
    /// \retval IFXRFE_E_SUCCESS on success
    /// \retval IFXRFE_E_MISSING_PARAMETER
    /// \note error code from \ref IfxRfe_writeSequencerMemory on error
    error_t IfxRfe_loadSequencerData(uint32_t startAddress, const uint32_t *sequencerDataWords, uint16_t wordCount);

    ///\brief Convenience function to safely configure the DMUX pins by checking the configurations against each other first
    ///\param dmuxCtrx CTRX dmux configuration \see IfxRfe_configureDmux
    ///\param dmuxAurix Aurix dmux pin configuration
    ///\retval  IFXRFE_E_SUCCESS if ok
    ///\retval  IFXRFE_E_INVALID_PARAMETER if the configurations indicate that the Ctrx and Aurix pins would drive against each other
    ///             on one of the dmux lines
    ///\note error code returned from \ref PlatformGpio_configurePin or \ref IfxRfe_configureDmux otherwise
    error_t IfxRfe_safeConfigureDmux(IfxRfe_configureDmux_t dmuxCtrx, IfxRfe_dmuxFlags_t dmuxAurix);

#ifdef __cplusplus
}  // namespace IfxRfe_Ctrx8188
#endif

#endif /* IFXRFE_SPECIFIC_H_8188 */
