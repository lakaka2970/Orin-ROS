#include "config.h"
#include "IfxRfe_CommandParamEnums.h"
#include <stdio.h>


void IrfeDemoConfigInit(IfxRfe_demoConfigParams_t *configParams)
{
    // Configure the tx power
    // Parameters in plvlx_2, plvlx_1, plvlx_4, plvlx_3 order because of code generation
    configParams->txpwr.plvl[0]                = 0 * 128;
    configParams->txpwr.plvl[1]                = 1 * 128;
    configParams->txpwr.plvl[2]                = 2 * 128;
    configParams->txpwr.plvl[3]                = 3 * 128;
    configParams->txpwr.tx_pa_slope_scale_fact = 1 * 256;
    configParams->txpwr.action_mask            = TxPower_ConfigurePowerLevels | TxPower_ConfigureSlopeScalingFactor;
    configParams->txpwr.ch_mask                = 0b11111111;

    // Configure Rx Frontend
    configParams->rxcfg.gain_sel       = GainSel_0dB;
    configParams->rxcfg.data_width_sel = DataWidth_12bits;
    configParams->rxcfg.data_rate_sel  = DataRate_1000Mbitsps;
    configParams->rxcfg.start_mode     = StartMode_Immediate;

    // Configure Ramp Scenario
    configParams->rampScenario.startoffset = 0;  //Sequencer setup structure start address of sequencer program


    // RF Frequency parameters from standalone MATLAB example (CTRX8188-Radar-eval-kit_Strata_3.0.0)
    configParams->rfFreqs.f_static = 79.65;      //static frequency in MHz before ramp sequence starts
    configParams->rfFreqs.f_lock   = 80.988002;  //Upper frequency of the RF modulation bandwidth in MHz
    configParams->rfFreqs.f_bw     = 2.499999;   //RF modulation bandwidth in MHz

    configParams->rfFreqCfg.bc     = 1;
    configParams->rfFreqCfg.nmod   = 62432208U;   //IfxRfe_calculateNmod(configParams->rfFreqs.f_static, configParams->rfFreqs.f_lock);
    configParams->rfFreqCfg.ncw    = 283073584U;  // IfxRfe_calculateNcw(configParams->rfFreqs.f_lock); //;
    configParams->rfFreqCfg.rampbw = 8738128U;    // IfxRfe_calculateRampBW(configParams->rfFreqs.f_bw); //

    printf("\nnmod=%u, ncw=%u, rampbw=%u\n", configParams->rfFreqCfg.nmod, configParams->rfFreqCfg.ncw, configParams->rfFreqCfg.rampbw);

    // Ctrx Dmux config
    configParams->ctrxDmux.config_mask = ConfigMask_DMUX1 | ConfigMask_DMUX2 | ConfigMask_DMUX3;
    configParams->ctrxDmux.dmux1_dir = configParams->ctrxDmux.dmux2_dir = configParams->ctrxDmux.dmux3_dir = DmuxDir_out;

    configParams->ctrxDmux.dmux1_pulse_duration_ext = configParams->ctrxDmux.dmux2_pulse_duration_ext = configParams->ctrxDmux.dmux3_pulse_duration_ext = 63;  // Pulse duration of DMUX1-3 (0: disabled, 3..63: (n+1)*5ns)

    configParams->ctrxDmux.dmux1_alt_signal = AltSignal_RxPayloadGateLevel;
    configParams->ctrxDmux.dmux2_alt_signal = AltSignal_DmuxALevel;
    configParams->ctrxDmux.dmux3_alt_signal = AltSignal_DmuxBLevel;

    // Execute_Calibration
    configParams->calibration.calib_sub_func_id = CalibSubFunc_RxGainAndTempComp | CalibSubFunc_RxBBADC_CalB | CalibSubFunc_RxBBADC_CalAStep1 | CalibSubFunc_RxBBADC_CalAStep2;
    configParams->calibration.tx_ch_pow_idx     = 0xFFFFFFFF;  // enable power calibration for all power levels at all TX channels
    configParams->calibration.ref_temp_idx      = 0;           // no reference temperature. Execute_Calibration() is based only on the LimitTemp. The user has to determine both the reported MMIC temperature and the temperature after the previous calibration.
    configParams->calibration.limit_temp        = 0;           // Calibration shall be called if |latest temperature - reference temperature| > LimitTemp (scaled in Q12.3 format), 0: calibrate regardless of current temperature.
}