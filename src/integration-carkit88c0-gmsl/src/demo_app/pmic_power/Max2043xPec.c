#include "Max2043xPec.h"

#include "IfxRfe_Crc8.h"
#include "PlatformGpio.h"
#include "PlatformI2c.h"
#include "PlatformErrors.h"
#include "PlatformTime.h"

/******************************************************************************/
/*-------------------------Local variables------------------------------------*/
/******************************************************************************/
static const uint32_t pinTimeOutMs = 100U;
static const uint8_t payloadLength = 2;

#define I2C_MASK_DEV_ADDR_7BIT(devAddr) ((devAddr)&0x007Fu)

/******************************************************************************/
/*-------------------------Local Function Prototypes--------------------------*/
/******************************************************************************/

/// \brief Determine the type of PMIC assembled on the board
/// \param devAddr I2C address of the PMIC
/// \param pmicType Pointer to the variable where the PMIC type will be stored
/// \retval PLATFORM_JETSON_E_SUCCESS on success
/// \retval PLATFORM_JETSON_E_UNEXPECTED_VALUE if Peripheral Error Checking (PEC) failed
/// \note error code from \ref Wrapper_I2cRead on error
/// \note error code from \ref Wrapper_I2cWrite on error
static int determinePmicType(uint16_t devAddr, Max2043xType_t *pmicType);

/// \brief Get the offset for calculating the register value from the voltage for VOUT
/// \param devAddr I2C address of the PMIC
/// \param voltageResult The offset in mV
/// \note The offset is different for different PMIC types. Therefore a derived (specific) class must provide it.
/// \retval PLATFORM_JETSON_E_SUCCESS on success
/// \retval PLATFORM_JETSON_E_UNEXPECTED_VALUE if Peripheral Error Checking (PEC) failed
/// \retval PLATFORM_JETSON_E_UNEXPECTED_VALUE if the PMIC type determined does not match any of the expected types (MAX20430, MAX20431, MAX20433)
/// \note error code from \ref Wrapper_I2cRead on error
/// \note error code from \ref Wrapper_I2cWrite on error
static int getVoutVoltageOffset(uint16_t devAddr, uint16_t *voltageResult);

/// \brief Get the 8-bit I2C address with the read/write bit set for CRC calculation
/// \param devAddr I2C address of the PMIC
/// \param readWriteBit 0 for write, 1 for read
/// \return The 8-bit I2C address with the read/write bit set
static uint8_t devAddressRWBit(uint16_t devAddr, uint8_t readWriteBit);

/// \brief Write a value to a register of the PMIC with PEC
/// \param devAddr I2C address of the PMIC
/// \param regAddr Register address
/// \param value  Value to write
/// \retval PLATFORM_JETSON_E_SUCCESS on success
/// \note error code from \ref Wrapper_I2cWrite on error
static int i2cWrite(uint16_t devAddr, uint8_t regAddr, const uint8_t value);

/// \brief Read a value from a register of the PMIC with PEC
/// \param devAddr I2C address of the PMIC
/// \param regAddr Register address
/// \param value Pointer to the value read
/// \return PLATFORM_JETSON_E_SUCCESS on success
/// \retval PLATFORM_JETSON_E_UNEXPECTED_VALUE if Peripheral Error Checking (PEC) failed
/// \note error code from \ref Wrapper_I2cRead on error
static int i2cRead(uint16_t devAddr, uint8_t regAddr, uint8_t *value);


/******************************************************************************/
/*-------------------------Global Function Implementations--------------------*/
/******************************************************************************/
int Max2043x_switchVoltageOutput(const Max2043xPec_t *pmic, uint8_t output, bool enable)
{
    if ((output & ~MAX2043x_VOLTAGE_OUT_ALL) != 0)
    {
        // Invalid value, unsupported bit is set (only the output voltages are allowed)
        return PLATFORM_JETSON_E_INVALID_PARAMETER;
    }

    uint8_t regValue;
    PLATFORM_JETSON_RETURN_ON_ERROR(i2cRead(pmic->devAddrI2c, Max2043x_CONFIGE, &regValue));

    if (enable)
    {
        regValue |= output;
    }
    else
    {
        regValue &= ~output;
    }

    PLATFORM_JETSON_RETURN_ON_ERROR(i2cWrite(pmic->devAddrI2c, Max2043x_CONFIGE, regValue));
    return PLATFORM_JETSON_E_SUCCESS;
}

int Max2043x_setVoltageOutput(const Max2043xPec_t *pmic, uint8_t output, uint16_t mV)
{
    uint16_t offset;
    PLATFORM_JETSON_RETURN_ON_ERROR(getVoutVoltageOffset(pmic->devAddrI2c, &offset));

    if (mV < offset)
    {
        // would produce a negative value wrapping around and setting a too high value
        return PLATFORM_JETSON_E_UNEXPECTED_VALUE;
    }
    uint32_t vout32 = (uint32_t)((mV - offset) * 8 / 100);  // This is the same as dividing by 12.5mv and then dividing by 1000 (mV => V), but avoids floating point calculation
    if (vout32 > 0xFF)
    {
        // Too high for the 8bit register
        return PLATFORM_JETSON_E_UNEXPECTED_VALUE;
    }
    uint8_t vout = (uint8_t)vout32;
    switch (output)
    {
        case MAX2043x_VOLTAGE_OUT2:
            i2cWrite(pmic->devAddrI2c, Max2043x_VOUT2, vout);
            return PLATFORM_JETSON_E_SUCCESS;
        case MAX2043x_VOLTAGE_OUT4:
            i2cWrite(pmic->devAddrI2c, Max2043x_VOUT4, vout);
            return PLATFORM_JETSON_E_SUCCESS;
        default:
            // Can't set this voltage
            return PLATFORM_JETSON_E_INVALID_PARAMETER;
    }
}

int Max2043x_mapVoltageToReset(const Max2043xPec_t *pmic, uint8_t voltage, bool active)
{
    if ((voltage & ~MAX2043x_VOLTAGE_ALL) != 0)
    {
        // Invalid value, unsupported bit is set (only the input and output voltages are allowed)
        return PLATFORM_JETSON_E_INVALID_PARAMETER;
    }
    uint8_t regValue;
    PLATFORM_JETSON_RETURN_ON_ERROR(i2cRead(pmic->devAddrI2c, Max2043x_PINMAP1, &regValue));
    if (active)
    {
        regValue |= voltage;
    }
    else
    {
        regValue &= ~voltage;
    }
    PLATFORM_JETSON_RETURN_ON_ERROR(i2cWrite(pmic->devAddrI2c, Max2043x_PINMAP1, regValue));

    return PLATFORM_JETSON_E_SUCCESS;
}

int Max2043x_enableAndCheckPmicVoltage(uint8_t voltage, const Max2043xPec_t *pmic, uint16_t pinPmicEn, uint16_t pinPmicRst)
{
    //set up voltage monitor to check voltage
    Max2043x_mapVoltageToReset(pmic, voltage, true);
    //turn on voltage
    Max2043x_switchVoltageOutput(pmic, voltage, true);
    //check if voltage monitor indicates error
    int error = waitForPin(pinPmicRst, pinTimeOutMs);
    if (error)
    {
        PlatformGpio_set(pinPmicEn, false);
        return error;
    }

    return PLATFORM_JETSON_E_SUCCESS;
}

int Max2043x_enableAndCheckLdo(uint16_t pinPmicEn, uint16_t pinLdoEn, uint16_t pinLdoPg)
{
    PlatformGpio_set(pinLdoEn, true);

    int error = waitForPin(pinLdoPg, pinTimeOutMs);
    if (error)
    {
        PlatformGpio_set(pinPmicEn, false);
        return error;
    }

    return PLATFORM_JETSON_E_SUCCESS;
}

int Max2043x_disableWatchdog(const Max2043xPec_t *pmic)
{
    return i2cWrite(pmic->devAddrI2c, Max2043x_WDCFG2, 0);
}


// Local functions definitions
static int determinePmicType(uint16_t devAddr, Max2043xType_t *pmicType)
{
    // The different PMIC types all have different default values in the VOUTx registers.
    // Reading them after power up shows which type is in use.
    // If once written, the registers cannot be used anymore as indicator, so they are only
    // used if we just powered up.
    // For the other cases we use the register VIN6 to store the type, as VIN6 is not used.
    // There's no other way to store the information as this class has a different life cycle than the board.
    //
    // PMIC must be enabled (enable pin) for this function to work

    *pmicType = Max2043xUnknown;

    uint8_t statd;
    PLATFORM_JETSON_RETURN_ON_ERROR(i2cRead(devAddr, Max2043x_STATD, &statd));  // POR bit is automatically reset once this register is read

    const bool por = (statd & 0x08) != 0;
    if (por)
    {
        // We have an unconfigured PMIC
        uint8_t vout2;
        PLATFORM_JETSON_RETURN_ON_ERROR(i2cRead(devAddr, Max2043x_VOUT2, &vout2));
        uint8_t vout4;
        PLATFORM_JETSON_RETURN_ON_ERROR(i2cRead(devAddr, Max2043x_VOUT4, &vout4));

        if ((vout2 == 32) && (vout4 == 16))
        {
            *pmicType = MAX20430;
        }
        else if ((vout2 == 144) && (vout4 == 64))
        {
            *pmicType = MAX20431;
        }
        else if ((vout2 == 104) && (vout4 == 32))
        {
            *pmicType = MAX20433;
        }
        else
        {
            *pmicType = Max2043xUnknown;
        }
        // Write the pmicType to VIN6 as temporary storage in case we need it again
        PLATFORM_JETSON_RETURN_ON_ERROR(i2cWrite(devAddr, Max2043x_VIN6, (uint8_t)*pmicType));
    }
    else
    {
        // The PMIC was already configured, thus VOUTx was already modified and can no longer be used.
        // Read the type from VIN6 register, where it was written after power on reset
        PLATFORM_JETSON_RETURN_ON_ERROR(i2cRead(devAddr, Max2043x_VIN6, (uint8_t *)pmicType));
    }
    return PLATFORM_JETSON_E_SUCCESS;
}

static int getVoutVoltageOffset(uint16_t devAddr, uint16_t *voltageResult)
{
    Max2043xType_t pmic;
    PLATFORM_JETSON_RETURN_ON_ERROR(determinePmicType(devAddr, &pmic));
    switch (pmic)
    {
        case MAX20430:
            *voltageResult = 800;
            break;
        case MAX20431:
            *voltageResult = 500;
            break;
        case MAX20433:
            *voltageResult = 500;
            break;
        default:
            // Currently there are no further types, so this default case is never called.
            // But in case a new type is added, this makes sure that not the wrong offset is used.
            return PLATFORM_JETSON_E_UNEXPECTED_VALUE;
    }

    return PLATFORM_JETSON_E_SUCCESS;
}

static uint8_t devAddressRWBit(uint16_t devAddr, uint8_t readWriteBit)
{
    uint8_t devAddress7bit = I2C_MASK_DEV_ADDR_7BIT(devAddr);
    return (devAddress7bit << 1) | readWriteBit;
}

static int i2cWrite(uint16_t devAddr, uint8_t regAddr, const uint8_t value)
{
    const uint8_t dataForCrc[] = {devAddressRWBit(devAddr, 0U), regAddr, value};
    const uint8_t crc          = IfxRfe_crc8SmBus(dataForCrc, (uint16_t)(sizeof(dataForCrc)));

    const uint8_t payload[] = {regAddr, value, crc};
    PLATFORM_JETSON_RETURN_ON_ERROR(PlatformI2c_writeWithoutPrefix(devAddr, sizeof(payload), payload));

    return PLATFORM_JETSON_E_SUCCESS;
}

static int i2cRead(uint16_t devAddr, uint8_t regAddr, uint8_t *value)
{
    uint8_t payload[2];
    PLATFORM_JETSON_RETURN_ON_ERROR(PlatformI2c_readWith8BitPrefix(devAddr, regAddr, payloadLength, payload));
    *value                      = payload[0];
    const uint8_t pecFromDevice = payload[1];

    // Verify if the PEC (CRC-8 SMBus) value is correct
    const uint8_t crcCheckData[] = {devAddressRWBit(devAddr, 0U), regAddr, devAddressRWBit(devAddr, 1U), *value};
    const uint8_t crcCheck       = IfxRfe_crc8SmBus(crcCheckData, (uint16_t)(sizeof(crcCheckData)));

    if (crcCheck != pecFromDevice)
        return PLATFORM_JETSON_E_UNEXPECTED_VALUE;

    return PLATFORM_JETSON_E_SUCCESS;
}

int waitForPin(uint16_t pinId, uint32_t timeoutMs)
{
    bool state = false;

    int64_t deadline = PlatformTime_getDeadLine(timeoutMs);
    int64_t now = PlatformTime_now();
    PLATFORM_JETSON_RETURN_ON_ERROR(PlatformGpio_get(pinId, &state));

    while ((false == state) && (now < deadline))
    {
        PlatformGpio_get(pinId, &state);
        now = PlatformTime_now();
    }

    if (false == state)
    {
        return PLATFORM_JETSON_E_TIMEOUT;
    }
    return PLATFORM_JETSON_E_SUCCESS;
}
