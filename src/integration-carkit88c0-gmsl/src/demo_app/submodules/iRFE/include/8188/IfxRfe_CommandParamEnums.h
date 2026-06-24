/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */

#ifndef IFXRFE_CTRXCOMMANDPARAMENUMS_H
#define IFXRFE_CTRXCOMMANDPARAMENUMS_H 1

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include <stdbool.h>
#include <stdint.h>


/******************************************************************************/
/*------------------------------Config Parameters-----------------------------*/
/******************************************************************************/

/// \brief Memory region identifier, to be used in read/write memory, calculate memory crc etc.
typedef enum
{
    IfxRfe_MemoryRegion_Sequencer          = 0xD,  ///> Sequencer program memory
    IfxRfe_MemoryRegion_EmbeddedDataBuffer = 0x17  ///> Embedded data buffer
} IfxRfe_MemoryRegion_t;

/// \brief Mask flag for the clear results firmware command
typedef enum
{
    IfxRfe_ClearMonitoringResult = (1 << 4)  ///> Clear monitoring result area
} IfxRfe_ClearResultsMask_t;

/// \brief Configure DIO-s command config mask field
typedef enum
{
    DioConfig_CLKMCU = (1 << 0),  ///> CLKMCU configuration
    DioConfig_DMUX1  = (1 << 1),  ///> DMUX1 configuration
    DioConfig_DMUX2  = (1 << 2),  ///> DMUX2 configuration
    DioConfig_DMUX3  = (1 << 3),  ///> DMUX3 configuration (only allowed for STAND-ALONE device)
    DioConfig_SO     = (1 << 4),  ///> SO configuration
    DioConfig_RFT    = (1 << 5)   ///> RFT configuration
} IfxRfe_configureDios_configMask_t;

/// \brief Configure DIO-s pull configuration fields
typedef enum
{
    PullConfig_NotSelected = 0,
    PullConfig_TriState    = 1,  ///> Tri-state
    PullConfig_PullDown    = 2,  ///> Pull-down
    PullConfig_PullUp      = 3   ///> Pull-up
} IfxRfe_configureDios_pullConfig_t;

/// \brief Configure DIO-s command CLKMCU pull configuration field
typedef enum
{
    ClkMcuPullConfig_PullDown = 2,  ///> Pull-down
} IfxRfe_configureDios_clkMcuPullConfig_t;

/// \brief Configure DIO-s command RFT pull configuration field
typedef enum
{
    RftPullConfig_TriState = 1,  ///> Tri-state
} IfxRfe_configureDios_rftPullConfig_t;

/// \brief Configure DIO-s command SO pull configuration field
typedef enum
{
    SoPullConfig_TriState = 1,  ///> Tri-state
} IfxRfe_configureDios_soPullConfig_t;

/// \brief Configure DIO-s driver configuration fields
typedef enum
{
    DriverConfig_NotSelected = 0,
    DriverConfig_PushPull    = 1,  ///> Push-pull
    DriverConfig_OpenDrain   = 2   ///> Open-drain
} IfxRfe_configureDios_driverConfig_t;

/// \brief Configure DIO-s CLKMCU driver configuration fields
typedef enum
{
    ClkMcuDriverConfig_PushPull = 1,  ///> Only Push-pull available
} IfxRfe_configureDios_clkMcuDriverConfig_t;

/// \brief Configure DIO-s RFT driver configuration fields
typedef enum
{
    RftDriverConfig_PushPull = 1,  ///> Only Push-pull available
} IfxRfe_configureDios_rftDriverConfig_t;

/// \brief Configure DIO-s SO driver configuration fields
typedef enum
{
    SoDriverConfig_PushPull = 1,  ///> Only Push-pull available
} IfxRfe_configureDios_soDriverConfig_t;

/// \brief Configure DIO-s drive strength fields
typedef enum
{
    DriveStrength_NotSelected = 0,
    DriveStrength_Strongest   = 1,
    DriveStrength_Strong      = 2,
    DriveStrength_Weak        = 3,
    DriveStrength_Weakest     = 4
} IfxRfe_configureDios_driveStrength_t;

/// \brief Configure DMUX config mask field
typedef enum
{
    ConfigMask_DMUX1 = (1 << 0),
    ConfigMask_DMUX2 = (1 << 1),
    ConfigMask_DMUX3 = (1 << 2)
} IfxRfe_configureDmux_configMask_t;

/// \brief Configure DMUX alternate signal field
typedef enum
{
    AltSignal_GeneralPurpose1    = 0x6D,  ///> General purpose signal 1
    AltSignal_GeneralPurpose2    = 0x6E,  ///> General purpose signal 2
    AltSignal_GeneralPurpose3    = 0x6F,  ///> General purpose signal 3
    AltSignal_RxPayloadGateLevel = 0xA0,  ///> Sequencer RX_PAYLOAD_GATE level
    AltSignal_RxPayloadGatePulse = 0xA1,  ///> Sequencer RX_PAYLOAD_GATE pulse
    AltSignal_DmuxALevel         = 0xE9,  ///> Sequencer DMUX A level
    AltSignal_DmuxAPulse         = 0xEA,  ///> Sequencer DMUX A pulse
    AltSignal_DmuxBLevel         = 0xEB,  ///> Sequencer DMUX B level
    AltSignal_DmuxBPulse         = 0xEC,  ///> Sequencer DMUX B pulse
    AltSignal_DmuxCLevel         = 0xED,  ///> Sequencer DMUX C level
    AltSignal_DmuxCPulse         = 0xEE,  ///> Sequencer DMUX C pulse
    AltSignal_DmuxDLevel         = 0xEF,  ///> Sequencer DMUX D level
    AltSignal_DmuxDPulse         = 0xF0   ///> Sequencer DMUX D pulse
} IfxRfe_configureDmux_altSignal_t;

/// \brief Configure DMUX pin direction field
typedef enum
{
    DmuxDir_in  = 0,  ///> Input
    DmuxDir_out = 1   ///> Output
} IfxRfe_configureDmux_dir_t;

/// \brief Configure MMIC clock CLKOUT load bias selection field
typedef enum
{
    LoadBias_0V   = 0,  ///> 0V
    LoadBias_1p2V = 1   ///> 1.2V
} IfxRfe_configureMmicClock_clkoutLoadBiasVoltage_t;

/// \brief Configure MMIC clock CLKOUT source resistance selection field
typedef enum
{
    SourceResistance_50R = 0,  ///> 50 Ohm
    SourceResistance_40R = 1   ///> 40 Ohm
} IfxRfe_configureMmicClock_clkoutSourceResistance_t;

/// \brief Configure MMIC clock configuration selection
typedef enum
{
    ClockConfig_Standalone = 1,
    ClockConfig_Secondary  = 2,
    ClockConfig_Primary    = 7
} IfXRfe_configureMmicClock_clkConfig_t;

/// \brief Configure RX GAIN_SEL field
typedef enum
{
    GainSel_p3dB  = 0,  ///> +3 dB
    GainSel_0dB   = 1,  ///> 0 dB
    GainSel_m3dB  = 2,  ///> -3 dB
    GainSel_m6dB  = 3,  ///> -6 dB
    GainSel_m12dB = 4,  ///> -12 dB
    GainSel_m18dB = 5   ///> -18 dB
} IfxRfe_configureRx_gainSel_t;

/// \brief Configure RX DATA_WIDTH_SEL field
typedef enum
{
    DataWidth_10bits = 1,  ///> 10 bits
    DataWidth_12bits = 2,  ///> 12 bits
    DataWidth_14bits = 3   ///> 14 bits
} IfxRfe_configureRx_dataWidth_t;

/// \brief Configure the DATA_RATE_SEL field
typedef enum
{
    DataRate_1200Mbitsps   = 1,
    DataRate_1000Mbitsps   = 2,
    DataRate_933p33Mbitsps = 3,
    DataRate_800Mbitsps    = 4,
    DataRate_700Mbitsps    = 5,
    DataRate_666p66Mbitsps = 6,
    DataRate_600Mbitsps    = 7,
    DataRate_560Mbitsps    = 8,
    DataRate_500Mbitsps    = 9,
    DataRate_480Mbitsps    = 10,
    DataRate_466p67Mbitsps = 11,
    DataRate_400Mbitsps    = 12,
    DataRate_350Mbitsps    = 13,
    DataRate_333p33Mbitsps = 14,
    DataRate_300Mbitsps    = 15,
    DataRate_280Mbitsps    = 16,
    DataRate_250Mbitsps    = 17,
    DataRate_240Mbitsps    = 18,
    DataRate_233p33Mbitsps = 19,
    DataRate_200Mbitsps    = 20,
    DataRate_175Mbitsps    = 21,
    DataRate_166p67Mbitsps = 22,
    DataRate_150Mbitsps    = 23,
    DataRate_140Mbitsps    = 24,
    DataRate_125Mbitsps    = 25,
    DataRate_120Mbitsps    = 26,
    DataRate_116p67Mbitsps = 27,
    DataRate_100Mbitsps    = 28
} IfxRfe_configureRx_dataRateSel_t;

/// \brief Configure RX DEC_SEL field
typedef enum
{
    DecSel_50MS    = 0,  ///> 50 MS/s
    DecSel_33p33MS = 1,  ///> 33.33 MS/s
    DecSel_25MS    = 2,  ///> 25 MS/s
    DecSel_20MS    = 3,  ///> 20 MS/s
    DecSel_16p67MS = 4,  ///> 16.67 MS/s
    DecSel_12p5MS  = 5,  ///> 12.5 MS/s
    DecSel_10MS    = 6,  ///> 10 MS/s
    DecSel_8p33MS  = 7,  ///> 8.33 MS/s
    DecSel_6p25MS  = 8,  ///> 6.25 MS/s
    DecSel_5MS     = 9   ///> 5 MS/s
} IfxRfe_configureRx_outputSampleRate_t;

/// \brief Configure RX START_MODE field
typedef enum
{
    StartMode_Immediate       = 0,  ///> Mode 1: Immediately after data acquisition
    StartMode_NextPayloadRamp = 1   ///> Mode 2: With start of data acquisition of next payload ramp
} IfxRfe_configureRx_startMode_t;

/// \brief Configure RX/Execute Monitoring/Measure Rx HP_SEL field
typedef enum
{
    RxHp_Cutoff_300kHz  = 0,  ///> 300 kHz
    RxHp_Cutoff_600kHz  = 1,  ///> 600 kHz
    RxHp_Cutoff_1200kHz = 2,  ///> 1200 kHz
    RxHp_Cutoff_2400kHz = 3,  ///> 2400 kHz
    RxHp_Cutoff_4800kHz = 4   ///> 4800 kHz
} IfxRfe_rxHpCutoff_t;

/// \brief Configure TX Power Action mask field
typedef enum
{
    TxPower_ConfigurePowerLevels        = (1 << 0),  ///> Configure power levels
    TxPower_ConfigureSlopeScalingFactor = (1 << 1)   ///> Configure Slope scaling factor
} IfxRfe_configureTxPower_actionMask_t;

/// \brief Configure TX Power/Execute Monitoring/Measure TX/Set Tx Output - Tx Channel mask field
typedef enum
{
    Tx_CH1 = (1 << 0),  ///> Transmitter channel 1
    Tx_CH2 = (1 << 1),  ///> Transmitter channel 2
    Tx_CH3 = (1 << 2),  ///> Transmitter channel 3
    Tx_CH4 = (1 << 3),  ///> Transmitter channel 4
    Tx_CH5 = (1 << 4),  ///> Transmitter channel 5
    Tx_CH6 = (1 << 5),  ///> Transmitter channel 6
    Tx_CH7 = (1 << 6),  ///> Transmitter channel 7
    Tx_CH8 = (1 << 7)   ///> Transmitter channel 8
} IfxRfe_txChannelMask_t;

/// \brief Execute calibration - calibration subfunction id
typedef enum
{
    CalibSubFunc_TxPower              = (1 << 0),   ///> TX power calibration
    CalibSubFunc_TxPhasePowerUp_Part1 = (1 << 2),   ///> TX phase power-up calibration, part 1
    CalibSubFunc_TxPhasePowerUp_Part2 = (1 << 3),   ///> TX phase power-up calibration, part 2
    CalibSubFunc_LOinputPower         = (1 << 4),   ///> LO input power calibration (PRIMARY device only)
    CalibSubFunc_TxPhaseWarmUp        = (1 << 5),   ///> TX phase warm-up calibration
    CalibSubFunc_RxBBADC_CalAStep2    = (1 << 8),   ///> RX BBADC calibration A, step 2
    CalibSubFunc_RxBBADC_CalAStep1    = (1 << 9),   ///> RX BBADC calibration A, step 1
    CalibSubFunc_RxBBADC_CalB         = (1 << 11),  ///> RX BBADC calibration B
    CalibSubFunc_RxGainAndTempComp    = (1 << 12)   ///> RX gain mapping and temperature compensation
} IfxRfe_executeCalibration_subFuncId_t;


/// \brief Execute Monitoring Monitoring Sub-func ID field
typedef enum
{
    MonitoringSubFunc_TxPhaseBalance     = (1 << 0),  ///>  TX phase balance monitoring
    MonitoringSubFunc_TxSignalPower      = (1 << 1),  ///> TX signal power monitoring
    MonitoringSubFunc_TxPowerBalance     = (1 << 2),  ///> TX power balance monitoring
    MonitoringSubFunc_RxRf               = (1 << 4),  ///> RX RF monitoring
    MonitoringSubFunc_RxBaseband         = (1 << 6),  ///> RX baseband monitoring
    MonitoringSubFunc_RegisterContent    = (1 << 8),  ///> Register content integrity check
    MonitoringSubFunc_LoSolderConnection = (1 << 9)   ///>  LO solder connection monitoring (PRIMARY and SECONDARY device only)
} IfxRfe_executeMonitoring_subFuncId_t;

/// \brief Get Results interface field
typedef enum
{
    Interface_Spi  = 0,
    Interface_CSI2 = 1
} IfxRfe_getResults_interface_t;

/// \brief Get Results Result Identifier field
typedef enum
{
    ResultId_Temperature   = 0x80000002,  ///> Temperature (CSI-2 interface only)
    ResultId_ErrorStatus   = 0x8100000A,  ///> Error status (CSI-2 interface only)
    ResultId_ResetResponse = 0x82000002   ///> Reset response
} IfxRfe_getResults_identifier_t;

/// \brief CTRX current state received through get status
typedef enum
{
    State_FW_Download   = 0x02000000,
    State_Sequencing    = 0x08000000,
    State_Operation     = 0x10000000,
    State_Low_Power     = 0x20000000,
    State_Pre_Operation = 0x40000000
} IfxRfe_getStatus_currentState_t;

/// \brief Handle error action field
typedef enum
{
    ErrorAction_ReadStatusSummary  = 0,  ///> Read error status summary
    ErrorAction_ReadDetailedStatus = 1,  ///> Read detailed error status
    ErrorAction_ClearAndReadBack   = 2   ///> Clear and read back detailed error status
} IfxRfe_handleError_action_t;

/// \brief Handle reset reason Act field
typedef enum
{
    ResetReason_ReadPrevious     = 1,  ///> Read previous error status
    ResetReason_ClearAndReadBack = 2,  ///> Clear and read back previous error status
} IfxRfe_handleResetReason_action_t;

/// \brief Measure RX measurement sub func id field
typedef enum
{
    MeasureRx_GainPhase = (1 << 0),  ///> RX gain/phase measurement
    MeasureRx_HpFilter  = (1 << 2),  ///> RX HP filter measurement
    MeasureRx_SpotNoise = (1 << 4),  ///> RX spot noise measurement
    MeasureRx_LpFilter  = (1 << 5)   ///> RX LP filter measurement
} IfxRfe_measureRx_subFunc_t;

/// \brief Measure TX mode field
typedef enum
{
    MeasureTx_OutputPower    = 0,  ///> TX output power using PLD (forward coupled signal)
    MeasureTx_ReflectedPower = 1,  ///> TX reflected power using PLD (reverse coupled signal)
    MeasureTx_Phase          = 2   ///> TX phase using RF monitoring circuitry
} IfxRfe_measureTx_mode_t;

/// \brief Set Clock Output action fields
typedef enum
{
    ClockOutput_Disable    = 0,  ///> Disable CLKPHY/CLKMCU output
    ClockOutput_Enable     = 1,  ///> Enable CLKPHY/CLKMCU output
    ClockOutput_DontChange = 2   ///> Do not reconfigure CLKPHY/CLKMCU
} IfxRfe_setClockOutput_action_t;

/// \brief Set Clock Output CLKMCU frequency setting field
typedef enum
{
    ClkMcu_25MHz = 0,  ///> 25 MHz
    ClkMcu_40MHz = 1,  ///> 40 MHz
    ClkMcu_50MHz = 2   ///> 50 MHz
} IfxRfe_setClockOutput_clkMcuFreq_t;

/// \brief Set MMIC Clock action field
typedef enum
{
    SetMmicClock_FullyExecute  = 0,  ///> Fully execute the command. (00b or 11b)
    SetMmicClock_ExecuteFirst  = 1,  ///> Execute the first command part
    SetMmicClock_ExecuteSecond = 2   ///> Execute the second command part
} IfxRfe_setMmicClock_action_t;

/// \brief Set signals bit value/bit mask fields
typedef enum
{
    SignalBit_GeneralPurpose1 = (1 << 0),  ///> General purpose signal 1
    SignalBit_GeneralPurpose2 = (1 << 1),  ///> General purpose signal 2
    SignalBit_GeneralPurpose3 = (1 << 2),  ///> General purpose signal 3
    SignalBit_DMUXA           = (1 << 3),  ///> Sequencer DMUX A level / pulse
    SignalBit_DMUXB           = (1 << 4),  ///> Sequencer DMUX B level / pulse
    SignalBit_DMUXC           = (1 << 5),  ///> Sequencer DMUX C level / pulse
    SignalBit_DMUXD           = (1 << 6)   ///> Sequencer DMUX D level / pulse
} IfxRfe_setSignal_mask_t;

/// \brief Set Tx Output action mask field
typedef enum
{
    TxOutput_Enable            = (1 << 0),  ///> TX output enable / disable (0 = disable, 1 = enable)
    TxOutput_PALoadDump_Enable = (1 << 2)   ///> TX PA internal load dumping (0 = disable, 1 = enable)
} IfxRfe_setTxOutput_action_t;

///\brief Set Tx Output PL TX fields
typedef enum
{
    TxOutput_Plevel1 = 0,  ///> TXn PLevel_1
    TxOutput_Plevel2 = 1,  ///> TXn PLevel_2
    TxOutput_Plevel3 = 2,  ///> TXn PLevel_3
    TxOutput_Plevel4 = 3   ///> TXn PLevel_4
} IfxRfe_setTxOutput_pLevelIndex_t;

/// \brief Write results region field
typedef enum
{
    WriteResults_HSRIF_Region = 0xE,  ///> HSRIF memory
    WriteResults_EDB_Region   = 0x17  ///> Embedded data buffer
} IfxRfe_writeResults_Region_t;

#endif  //IFXRFE_CTRXCOMMANDPARAMENUMS_H
