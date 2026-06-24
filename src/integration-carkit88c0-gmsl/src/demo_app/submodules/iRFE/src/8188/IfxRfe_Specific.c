/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */

#include "8188/IfxRfe_Specific.h"

#include "8188/IfxRfe_CommandParamEnums.h"
#include "8188/IfxRfe_FirmwareCommands.h"
#include "IfxRfe_BuilderParser.h"
#include "IfxRfe_Gpio.h"
#include "IfxRfe_GpioWrapper.h"
#include "IfxRfe_Logger.h"
#include "IfxRfe_LowLevelCommands.h"
#include "IfxRfe_SpiWrapper.h"
#include "IfxRfe_State.h"
#include "IfxRfe_TimeWrapper.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
namespace IfxRfe_Ctrx8188
{
#endif

    /******************************************************************************/
    /*-------------------------Local variables------------------------------------*/
    /******************************************************************************/

    //0x10000028 - LOIN config
    //0x1180000A - HSRIF CRC32 config
    static uint32_t intitializeIndex[] = {0x10000028, 0x1180000A};
    static uint32_t intitializeValue[] = {1, 2};  // STANDALONE mode, disable CRC32 for now

/******************************************************************************/
/*----------------------------------Defines-----------------------------------*/
/******************************************************************************/

// Macro to create a mask based on used bit numbers
#define MASK_BITS_RANGE(upper, lower) (((1U << ((upper) - (lower) + 1)) - 1) << (lower))

// for the exact representation of the constant (2^−16 × 50 × 6)
#define FREQ_CALC_DENOMINATOR_CONST 4.57763671875e-03

    /******************************************************************************/
    /*-------------------------Local Function Prototypes--------------------------*/
    /******************************************************************************/
    static inline uint32_t freqToValue(double freq);

    /******************************************************************************/
    /*-------------------------Global Function Implementations--------------------*/
    /******************************************************************************/

    error_t IfxRfe_safeConfigureDmux(IfxRfe_configureDmux_t dmuxCtrx, IfxRfe_dmuxFlags_t dmuxAurix)
    {
        RETURN_ON_NOT_INITIALIZED();

        //check if aurix dmux config compatible with ctrx dmux config
        bool isDmux1Out, isDmux2Out, isDmux3Out;
        IFXRFE_RETURN_ON_ERROR(Wrapper_IsGpioConfigOutput(dmuxAurix.dmux1Flags, &isDmux1Out));  //enough to check function return value once, it only returns error if the callback is not defined
        Wrapper_IsGpioConfigOutput(dmuxAurix.dmux2Flags, &isDmux2Out);
        Wrapper_IsGpioConfigOutput(dmuxAurix.dmux3Flags, &isDmux3Out);

        if ((DmuxDir_out == dmuxCtrx.dmux1_dir) && isDmux1Out)
        {
            errorLog("DMUX1 is set to output on both Aurix and Ctrx side");
            return IFXRFE_E_INVALID_PARAMETER;  //it is only a parameter yet, not a config
        }
        if ((DmuxDir_out == dmuxCtrx.dmux2_dir) && isDmux2Out)
        {
            errorLog("DMUX2 is set to output on both Aurix and Ctrx side");
            return IFXRFE_E_INVALID_PARAMETER;  //it is only a parameter yet, not a config
        }
        if ((DmuxDir_out == dmuxCtrx.dmux3_dir) && isDmux3Out)
        {
            errorLog("DMUX3 is set to output on both Aurix and Ctrx side");
            return IFXRFE_E_INVALID_PARAMETER;  //it is only a parameter yet, not a config
        }

        //to avoid short signal contentions, the aurix inputs are configured first, then all the
        // CTRX signals (single firmware command), and the aurix outputs at the end
        const IfxRfe_gpioDefinitions_t *gpio = getCtrxPinDefinition();
        if (false == isDmux1Out)
        {
            IFXRFE_CLEAN_RETURN_ON_ERROR(Wrapper_GpioConfigure(gpio->dmux1Id, dmuxAurix.dmux1Flags), errorLog("DMUX1 id invalid"));
        }
        if (false == isDmux2Out)
        {
            IFXRFE_CLEAN_RETURN_ON_ERROR(Wrapper_GpioConfigure(gpio->dmux2Id, dmuxAurix.dmux2Flags), errorLog("DMUX2 id invalid"));
        }
        if (false == isDmux3Out)
        {
            IFXRFE_CLEAN_RETURN_ON_ERROR(Wrapper_GpioConfigure(gpio->dmux3Id, dmuxAurix.dmux3Flags), errorLog("DMUX3 id invalid"));
        }

        //apply ctrx configuration
        IFXRFE_RETURN_ON_ERROR(IfxRfe_configureDmux(dmuxCtrx));

        //apply aurix output configuration after the CTRX input configuration is applied
        if (isDmux1Out)
        {
            IFXRFE_CLEAN_RETURN_ON_ERROR(Wrapper_GpioConfigure(gpio->dmux1Id, dmuxAurix.dmux1Flags), errorLog("DMUX1 id invalid"));
        }
        if (isDmux2Out)
        {
            IFXRFE_CLEAN_RETURN_ON_ERROR(Wrapper_GpioConfigure(gpio->dmux2Id, dmuxAurix.dmux2Flags), errorLog("DMUX2 id invalid"));
        }
        if (isDmux3Out)
        {
            IFXRFE_CLEAN_RETURN_ON_ERROR(Wrapper_GpioConfigure(gpio->dmux3Id, dmuxAurix.dmux3Flags), errorLog("DMUX3 id invalid"));
        }

        return IFXRFE_E_SUCCESS;
    }

    error_t IfxRfe_loadRamFw(const fw_spi_frame *ramFw, uint32_t nrFrames)
    {
        RETURN_ON_NOT_INITIALIZED();

        // it is required to pass something to download, otherwise the finishFwDownload might be called without any download
        if ((NULL == ramFw) || (nrFrames == 0))
        {
            return IFXRFE_E_MISSING_PARAMETER;
        }

        for (uint32_t i = 0; i < nrFrames; i++)
        {
            IFXRFE_RETURN_ON_ERROR(IfxRfe_downloadFwFunctions_exp(ramFw[i].key1, ramFw[i].length, ramFw[i].key2, ramFw[i].key3, ramFw[i].code_word));
        }

        //8191 B11 handle error has to be called before finish
        uint32_t error_mask_2_arr[]         = {0, 0, 0, 0, 0, 0, 0, 0};  // named variable to avoid pointer on temporary array
        IfxRfe_handleError_t handleErrorReq = {
            .action               = ErrorAction_ClearAndReadBack,
            .error_mask_1         = 0,
            .error_mask_1_present = true,
            .length               = 8,
            .error_mask_2         = error_mask_2_arr};
        IFXRFE_RETURN_ON_ERROR(IfxRfe_handleError(handleErrorReq, NULL));
        IFXRFE_RETURN_ON_ERROR(IfxRfe_finishFwDownload());  // Finish FW Download --> CTRX executes a soft reset


        return IFXRFE_E_SUCCESS;
    }

    IfxRfe_ctrxInitConfiguration_t prepareCtrxInitConfig(IfxRfe_deviceConfiguration_t devConfig)
    {
        // Initialize the necessary variables and structures
        IfxRfe_ctrxInitConfiguration_t config;
        config.spiConfigureFlags = IFXRFE_SPI_MODE_3;
        config.spiClockSpeed     = 25000000;  // 25MHz
        config.deviceConfig      = devConfig;
        config.clockOutputConfig = NULL;      //no clock output per default

        // configureMmicClock config not needed in standalone mode
        config.configureMmicClockConfig.clock_selection          = ClockConfig_Standalone;
        config.configureMmicClockConfig.clkout_source_resistance = SourceResistance_50R;
        config.configureMmicClockConfig.clkout_bias_voltage      = LoadBias_0V;

        config.initializeConfig.length          = sizeof(intitializeIndex) / sizeof(intitializeIndex[0]);
        config.initializeConfig.iram_descriptor = intitializeIndex;
        config.initializeConfig.value           = intitializeValue;

        switch (config.deviceConfig)
        {
            case STANDALONE:
            {
                config.configureMmicClockConfig.clock_selection = ClockConfig_Standalone;  //standalone
                config.initializeConfig.value[0]                = 1;                       //standalone
                break;
            }
            case PRIMARY:
            {
                config.configureMmicClockConfig.clock_selection = ClockConfig_Primary;  //primary
                config.initializeConfig.value[0]                = 15;                   //primary using LOIN2
                break;
            }
            case SECONDARY:
            {
                config.configureMmicClockConfig.clock_selection = ClockConfig_Secondary;  //secondary
                config.initializeConfig.value[0]                = 4;                      //secondary using LOIN1
                break;
            }
        }

        return config;
    }

    error_t IfxRfe_ctrxInit(IfxRfe_ctrxInitConfiguration_t config)
    {
        RETURN_ON_NOT_INITIALIZED();

        Wrapper_SpiConfigure(config.spiConfigureFlags, config.spiClockSpeed);  // Set SPI clock speed to 25MHz

        IFXRFE_RETURN_ON_ERROR(IfxRfe_triggerLbist());                         // LBIST will reset the CTRX
        IFXRFE_RETURN_ON_ERROR(IfxRfe_waitForRftPin(IFXRFE_RFT_TIMEOUT_DEF));

        // ram fw has to be downloaded before clock configuration, since ram is updated during clock configuration
        IFXRFE_RETURN_ON_ERROR(IfxRfe_loadRamFw(scrambled_fw_spi_frame, TOT_NUM_OF_FRAMES));  // download firmware, execute finish_fw_download
        IFXRFE_RETURN_ON_ERROR(IfxRfe_waitForRftPin(IFXRFE_RFT_TIMEOUT_DEF));

        // 8188 Modifications for Exit_pre_operation (required for current ram fw with CV trimmed samples)
        IfxRfe_writeMemory_t writeMemoryParams;
        writeMemoryParams.region     = 18;
        writeMemoryParams.passkey    = 3392857997;
        writeMemoryParams.start_addr = 3644;
        writeMemoryParams.length     = 1;
        uint32_t data[]              = {258};
        writeMemoryParams.data_word  = data;
        IFXRFE_RETURN_ON_ERROR(IfxRfe_writeMemory(writeMemoryParams));

        IFXRFE_RETURN_ON_ERROR(IfxRfe_configureMmicClock(config.configureMmicClockConfig));

        switch (config.deviceConfig)
        {
            case STANDALONE:
                IFXRFE_RETURN_ON_ERROR(IfxRfe_setMmicClock_exp(SetMmicClock_FullyExecute));  // standalone - execute both steps
                break;
            case PRIMARY:
            {
                IFXRFE_RETURN_ON_ERROR(IfxRfe_setMmicClock_exp(SetMmicClock_ExecuteFirst));  // primary - execute separately
                IFXRFE_RETURN_ON_ERROR(IfxRfe_setMmicClock_exp(SetMmicClock_ExecuteSecond));
            }
            break;
            case SECONDARY:
            {
                IFXRFE_RETURN_ON_ERROR(IfxRfe_setMmicClock_exp(SetMmicClock_FullyExecute));  // secondary - execute both steps
            }
            break;
            default:
                return IFXRFE_E_INVALID_PARAMETER;
        }

        // IfxRfe_setClockOutput invalid for a secondary device
        if ((config.clockOutputConfig != NULL) && ((config.deviceConfig == STANDALONE) || (config.deviceConfig == PRIMARY)))
        {
            IFXRFE_RETURN_ON_ERROR(IfxRfe_setClockOutput(*config.clockOutputConfig));
        }

        IFXRFE_RETURN_ON_ERROR(IfxRfe_initialize(config.initializeConfig));

        IfxRfe_getVersionResult_t version;
        IFXRFE_RETURN_ON_ERROR(IfxRfe_getVersion(&version));

        IFXRFE_RETURN_ON_ERROR(IfxRfe_exitPreOperation_exp(0xA69AE4, version.fw_version_code, 0x68373AD2));

        // Check for errors. Equivalent with OK pin check
        IfxRfe_handleError_t errorRequest = {
            .action               = ErrorAction_ReadStatusSummary,  // Summary
            .error_mask_1         = 0,
            .error_mask_1_present = false,
            .length               = 0,
            .error_mask_2         = 0};
        IfxRfe_handleErrorResult_t errorResult;
        IFXRFE_RETURN_ON_ERROR(IfxRfe_handleError(errorRequest, &errorResult));

        if (errorResult.failure_ind_a || errorResult.failure_ind_b || errorResult.failure_ind_c)
        {
            return IFXRFE_E_FAILED;
        }
        return IFXRFE_E_SUCCESS;
    }

    error_t IfxRfe_writeSequencerMemory(uint32_t startAddress, const uint32_t *dataWords, uint8_t length)
    {
        return IfxRfe_writeMemory_exp(IfxRfe_MemoryRegion_Sequencer, length, 0, startAddress, dataWords);
    }

    error_t IfxRfe_readSequencerMemory(uint32_t startAddress, uint8_t length, IfxRfe_readMemoryResult_t *res)
    {
        return IfxRfe_readMemory_exp(IfxRfe_MemoryRegion_Sequencer, length, 0, startAddress, res);
    }

    error_t IfxRfe_loadSequencerData(uint32_t startAddress, const uint32_t *sequencerDataWords, uint16_t wordCount)
    {
        RETURN_ON_NOT_INITIALIZED();

        if (wordCount == 0)
        {
            return IFXRFE_E_MISSING_PARAMETER;
        }

        const uint16_t maxIncrement = MAX_REQ_PAYLOAD_WORDS - 3;  //header, crc32, payload header, passkey, start address
        for (uint16_t i = 0; i < wordCount; i += maxIncrement)
        {
            uint16_t wordCountToWrite = ((wordCount - i) < maxIncrement) ? (wordCount - i) : maxIncrement;
            IFXRFE_RETURN_ON_ERROR(IfxRfe_writeSequencerMemory(startAddress + (i * (sizeof(uint32_t))), &sequencerDataWords[i], (uint8_t)wordCountToWrite));
        }

        return IFXRFE_E_SUCCESS;
    }

    error_t IfxRfe_calculateNmod(float fStatic_MHz, float fLock_MHz, uint32_t *nmodResult)
    {
        if (fStatic_MHz > fLock_MHz)
        {
            return IFXRFE_E_OUT_OF_BOUNDS;
        }

        if (nmodResult == NULL)
        {
            return IFXRFE_E_MISSING_PARAMETER;
        }

        uint32_t nmod = freqToValue(fStatic_MHz - fLock_MHz);
        nmod &= MASK_BITS_RANGE(25, 4);  // Used bits 25:4 @ref CTRX-8191F Manual

        *nmodResult = nmod;

        return IFXRFE_E_SUCCESS;
    }

    uint32_t IfxRfe_calculateNcw(float fLock_MHz)
    {
        uint32_t ncw = freqToValue(fLock_MHz);
        ncw &= MASK_BITS_RANGE(28, 4);  // Used bits 28:4 @ref CTRX-8191F Manual
        return ncw;
    }

    uint32_t IfxRfe_calculateRampBW(float fBw_MHz)
    {
        uint32_t rampBw = freqToValue(fBw_MHz);
        rampBw &= MASK_BITS_RANGE(24, 4);  // Used bits 24:4 @ref CTRX-8191F Manual
        return rampBw;
    }


    /******************************************************************************/
    /*-------------------------Local Function Implementations---------------------*/
    /******************************************************************************/

    static inline uint32_t freqToValue(double freq)
    {
        double temp = round(freq / FREQ_CALC_DENOMINATOR_CONST);
        // Casting directly from a double to uint32_t is undefined behavior
        int32_t signedTemp = (int32_t)(temp * 16);
        return (uint32_t)(signedTemp);
    }

#ifdef __cplusplus
}  // namespace IfxRfe_Ctrx8188
#endif
