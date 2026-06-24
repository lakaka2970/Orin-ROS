#include "Max20434Pec.h"

#include "IfxRfe_Crc8.h"
#include "PlatformI2c.h"
#include "IfxRfe_Logger.h"

/******************************************************************************/
/*-------------------------Local variables------------------------------------*/
/******************************************************************************/
static const uint8_t payloadLength = 2;

#define IFXRFE_I2C_MASK_DEV_ADDR_7BIT(devAddr) ((devAddr)&0x007Fu)

/******************************************************************************/
/*-------------------------Local Function Prototypes--------------------------*/
/******************************************************************************/

/// \brief Get the 8-bit I2C address with the read/write bit set for CRC calculation
/// \param devAddr I2C address of the PMIC
/// \param readWriteBit 0 for write, 1 for read
/// \return The 8-bit I2C address with the read/write bit set
static uint8_t devAddressRWBit(uint16_t devAddr, uint8_t readWriteBit);

/// \brief Write a value to a register of the PMIC with PEC
/// \param devAddr I2C address of the PMIC
/// \param regAddr Register address
/// \param value  Value to write
/// \retval IFXRFE_E_SUCCESS on success
/// \note error code from \ref Wrapper_I2cWrite on error
static error_t i2cWrite(uint16_t devAddr, uint8_t regAddr, const uint8_t value);

/// \brief Read a value from a register of the PMIC with PEC
/// \param devAddr I2C address of the PMIC
/// \param regAddr Register address
/// \param value Pointer to the value read
/// \return IFXRFE_E_SUCCESS on success
/// \retval IFXRFE_E_UNEXPECTED_VALUE if Peripheral Error Checking (PEC) failed
/// \note error code from \ref Wrapper_I2cRead on error
static error_t i2cRead(uint16_t devAddr, uint8_t regAddr, uint8_t *value);


/******************************************************************************/
/*-------------------------Global Function Implementations--------------------*/
/******************************************************************************/
error_t Max20434_enableSpreadSpectrum(const Max20434Pec_t *pmic, Max20434_SSE_t sseMode)
{
    uint8_t regValue = (sseMode << 2);

    IFXRFE_RETURN_ON_ERROR(i2cWrite(pmic->devAddrI2c, Max20434_CTRL1, regValue));
    return IFXRFE_E_SUCCESS;
}

error_t Max20434_setManualOutputEnable(const Max20434Pec_t *pmic, bool enable)
{
    uint8_t regValue;

    regValue = enable ? 0x07 : 0x00;
    
    IFXRFE_RETURN_ON_ERROR(i2cWrite(pmic->devAddrI2c, Max20434_CONFIGE1, regValue));
    return IFXRFE_E_SUCCESS;
}


// Local functions definitions
static uint8_t devAddressRWBit(uint16_t devAddr, uint8_t readWriteBit)
{
    uint8_t devAddress7bit = IFXRFE_I2C_MASK_DEV_ADDR_7BIT(devAddr);
    return (devAddress7bit << 1) | readWriteBit;
}

static error_t i2cWrite(uint16_t devAddr, uint8_t regAddr, const uint8_t value)
{
    const uint8_t dataForCrc[] = {devAddressRWBit(devAddr, 0U), regAddr, value};
    const uint8_t crc          = IfxRfe_crc8SmBus(dataForCrc, (uint16_t)(sizeof(dataForCrc)));

    const uint8_t payload[] = {regAddr, value, crc};
    IFXRFE_RETURN_ON_ERROR(PlatformI2c_writeWithoutPrefix(devAddr, sizeof(payload), payload));

    return IFXRFE_E_SUCCESS;
}

static error_t i2cRead(uint16_t devAddr, uint8_t regAddr, uint8_t *value)
{
    uint8_t payload[2];
    IFXRFE_RETURN_ON_ERROR(PlatformI2c_readWith8BitPrefix(devAddr, regAddr, payloadLength, payload));
    *value                      = payload[0];
    const uint8_t pecFromDevice = payload[1];

    // Verify if the PEC (CRC-8 SMBus) value is correct
    const uint8_t crcCheckData[] = {devAddressRWBit(devAddr, 0U), regAddr, devAddressRWBit(devAddr, 1U), *value};
    const uint8_t crcCheck       = IfxRfe_crc8SmBus(crcCheckData, (uint16_t)(sizeof(crcCheckData)));

    if (crcCheck != pecFromDevice)
        return IFXRFE_E_UNEXPECTED_VALUE;

    return IFXRFE_E_SUCCESS;
}