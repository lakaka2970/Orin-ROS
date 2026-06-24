#ifndef IRFEDEMOACONFIG_H
#define IRFEDEMOACONFIG_H 1

#include "IfxRfe.h"

typedef struct
{
    float f_static;
    float f_lock;
    float f_bw;
} rfFreqs_t;

typedef struct
{
    IfxRfe_configureDmux_t ctrxDmux;
    IfxRfe_dmuxFlags_t aurixDmux;
    IfxRfe_configureDios_t dios;
    IfxRfe_configureTxPower_t txpwr;
    IfxRfe_configureRx_t rxcfg;
    const uint32_t *seqData;
    uint32_t seqDataNrEntries;
    rfFreqs_t rfFreqs;
    IfxRfe_configureRampScenario_t rampScenario;
    IfxRfe_executeCalibration_t calibration;
    IfxRfe_configureRfFrequency_t rfFreqCfg;
} IfxRfe_demoConfigParams_t;

/// \brief Initialization of demo application configuration parameters
/// \param configParams pointer of IfxRfe_demoConfigParams_t struct
void IrfeDemoConfigInit(IfxRfe_demoConfigParams_t *configParams);

#endif  //IRFEDEMOACONFIG_H