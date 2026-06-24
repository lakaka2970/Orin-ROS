#ifndef MAX20434PEC_H
#define MAX20434PEC_H 1

#include "IfxRfe_ErrorDefinitions.h"
#include <stdbool.h>
#include <stdint.h>
/// Implementation to support on-board PMICs (Power-Management-IC) of type MAX20434
/// which can be configured via I2C using PEC (packet error checking)


/// @brief Structure to hold the I2C address of the PMIC
typedef struct
{
    uint16_t devAddrI2c;  ///< I2C address of the device (7-bit)
} Max20434Pec_t;

/// @brief Register map of the PMIC MAX20434
typedef enum
{
    Max20434_CONFIGE1 = 0x28, //Manual Output Control Register
    Max20434_CTRL1 = 0x35 //General Control Register 1
} Max20434Registers;

typedef enum
{
    SSE_Disabled = 0,
    SSE_3Percent_PseudoRandom = 1,
    SSE_1p5Percent_PseudoRandom = 2,
    SSE_3Percent_Linear = 3
}Max20434_SSE_t;

error_t Max20434_enableSpreadSpectrum(const Max20434Pec_t *pmic, Max20434_SSE_t sseMode);
error_t Max20434_setManualOutputEnable(const Max20434Pec_t *pmic, bool enable);

#endif /* MAX20434PEC_H */
