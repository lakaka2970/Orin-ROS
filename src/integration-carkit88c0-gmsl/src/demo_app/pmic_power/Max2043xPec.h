#ifndef MAX2043XPEC_H
#define MAX2043XPEC_H 1

#include <stdbool.h>
#include <stdint.h>
/// Implementation to support on-board PMICs (Power-Management-IC) of type MAX2043x
/// which can be configured via I2C using PEC (packet error checking)

// Voltage definitions, to be used as function parameters
#define MAX2043x_VOLTAGE_OUT1    0x01
#define MAX2043x_VOLTAGE_OUT2    0x02
#define MAX2043x_VOLTAGE_OUT3    0x04
#define MAX2043x_VOLTAGE_OUT4    0x08
#define MAX2043x_VOLTAGE_IN5     0x10
#define MAX2043x_VOLTAGE_IN6     0x20
#define MAX2043x_VOLTAGE_OUT_ALL (MAX2043x_VOLTAGE_OUT1 | MAX2043x_VOLTAGE_OUT2 | MAX2043x_VOLTAGE_OUT3 | MAX2043x_VOLTAGE_OUT4)
#define MAX2043x_VOLTAGE_IN_ALL  (MAX2043x_VOLTAGE_IN5 | MAX2043x_VOLTAGE_IN6)
#define MAX2043x_VOLTAGE_ALL     (MAX2043x_VOLTAGE_OUT_ALL | MAX2043x_VOLTAGE_IN_ALL)

/// @brief Structure to hold the I2C address of the PMIC
typedef struct
{
    uint16_t devAddrI2c;  ///< I2C address of the device (7-bit)
} Max2043xPec_t;

/// @brief Register map of the PMIC MAX2043x
typedef enum
{
    Max2043x_CID     = 0x00,
    Max2043x_CONFIG1 = 0x01,
    Max2043x_CONFIGE = 0x03,
    Max2043x_FPSCFG  = 0x05,
    Max2043x_PINMAP1 = 0x07,
    Max2043x_STATD   = 0x0B,
    Max2043x_VOUT2   = 0x0E,
    Max2043x_VOUT4   = 0x0F,
    Max2043x_VIN5    = 0x10,
    Max2043x_VIN6    = 0x11,
    Max2043x_WDCFG2  = 0x14,
    Max2043x_WDPROT  = 0x16,
} Max2043xRegisters;


/// @brief Type of PMICs
typedef enum
{
    Max2043xUnknown,
    MAX20430,
    MAX20431,
    MAX20433,
} Max2043xType_t;


/// \brief Switch voltage outputs on or off
/// \param pmic The PMIC object to use
/// \param output Defines which output to switch, the others will be left unchanged. Using the MAX2043x_VOLTAGE_x defines, multiple outputs can be combined (|) for one call.
/// \param enable True to switch the voltage on, false to switch it off
/// \retval PLATFORM_JETSON_E_SUCCESS on success
/// \retval PLATFORM_JETSON_E_INVALID_PARAMETER if the provided voltage parameter has unsupported bits set (only the input and output voltages are allowed)
/// \retval PLATFORM_JETSON_E_UNEXPECTED_VALUE if Peripheral Error Checking (PEC) failed
/// \note error code from \ref PlatformGpio_set on error
/// \note error code from \ref PlatformI2c_readWith8BitPrefix on error
/// \note error code from \ref PlatformI2c_writeWithoutPrefix on error
int Max2043x_switchVoltageOutput(const Max2043xPec_t *pmic, uint8_t output, bool enable);


/// \brief Set the voltage for voltage outputs
/// \param pmic The PMIC object to use
/// \param output Defines the output to change the voltage for. Use the MAX2043x_VOLTAGE_x defines. Only one output can be changed at a time.
/// \param mV The voltage value to set in millivolts
/// \retval PLATFORM_JETSON_E_SUCCESS on success
/// \retval PLATFORM_JETSON_E_UNEXPECTED_VALUE   if the provided millivolt value is less than the offset or
///                                     if the calculated voltage value exceeds the maximum value that can be held by an 8-bit register
///                                     if the PMIC type determined does not match any of the expected types (MAX20430, MAX20431, MAX20433)
/// \retval PLATFORM_JETSON_E_INVALID_PARAMETER if the provided output parameter does not match any of the expected voltage output definitions (MAX2043x_VOLTAGE_OUT2, MAX2043x_VOLTAGE_OUT4)
/// \note error code from \ref PlatformI2c_readWith8BitPrefix on error
/// \note error code from \ref PlatformI2c_writeWithoutPrefix on error
int Max2043x_setVoltageOutput(const Max2043xPec_t *pmic, uint8_t output, uint16_t mV);


/// \brief Map a voltage input or output to the reset output, meaning that the reset output is set if the voltage is not as specified.
/// \param pmic The PMIC object to use
/// \param voltage Defines which voltage to map, the others will be left unchanged. Using the MAX2043x_VOLTAGE_x defines, multiple voltages can be combined (|) for one call.
/// \param active True to map the voltage to reset, false to unmap it
/// \retval PLATFORM_JETSON_E_SUCCESS on success
/// \retval PLATFORM_JETSON_E_INVALID_PARAMETER if the provided voltage parameter has unsupported bits set (only the input and output voltages are allowed)
/// \retval PLATFORM_JETSON_E_UNEXPECTED_VALUE if Peripheral Error Checking (PEC) failed
/// \note error code from \ref PlatformI2c_readWith8BitPrefix on error
/// \note error code from \ref PlatformI2c_writeWithoutPrefix on error
int Max2043x_mapVoltageToReset(const Max2043xPec_t *pmic, uint8_t voltage, bool active);


/// \brief Enables voltage output and checks the reset pin status
/// \param voltage The voltage to enable
/// \param pmic The PMIC object to use
/// \param pinPmicEn The GPIO pin to enable the PMIC
/// \param pinPmicRst The GPIO pin to check the reset signal
/// \retval PLATFORM_JETSON_E_SUCCESS on success
/// \retval PLATFORM_JETSON_E_TIMEOUT if the state of the pin does not change to true within the specified timeout period
/// \note error code from \ref Max2043x_mapVoltageToReset on error
/// \note error code from \ref Max2043x_setVoltageOutput on error
/// \note error code from \ref PlatformGpio_set on error
/// \note error code from \ref PlatformTime_getDeadLine on error
/// \note error code from \ref PlatformTime_now on error
/// \note error code from \ref PlatformGpio_get on error
int Max2043x_enableAndCheckPmicVoltage(uint8_t voltage, const Max2043xPec_t *pmic, uint16_t pinPmicEn, uint16_t pinPmicRst);


/// \brief Enables gpio and checks control gpio if the voltage level is good
/// \param pinPmicEn The GPIO pin to enable the PMIC
/// \param pinLdoEn The GPIO pin to enable the voltage
/// \param pinLdoPg The GPIO pin to check if the voltage is good
/// \retval PLATFORM_JETSON_E_SUCCESS on success
/// \note error code from \ref PlatformGpio_set on error
/// \note error code from \ref PlatformTime_getDeadLine on error
/// \note error code from \ref PlatformTime_now on error
/// \note error code from \ref PlatformGpio_get on error
int Max2043x_enableAndCheckLdo(uint16_t pinPmicEn, uint16_t pinLdoEn, uint16_t pinLdoPg);


/// \brief Disable the watchdog
/// \param pmic The PMIC object to use
/// \retval PLATFORM_JETSON_E_SUCCESS on success
/// \retval PLATFORM_JETSON_E_UNEXPECTED_VALUE if Peripheral Error Checking (PEC) failed
/// \note error code from \ref PlatformI2c_writeWithoutPrefix on error
int Max2043x_disableWatchdog(const Max2043xPec_t *pmic);

/// \brief Wait for a pin to change its state
/// \param pinId The pin to wait for
/// \param timeoutMs The timeout in milliseconds
/// \retval PLATFORM_JETSON_E_SUCCESS on success
/// \retval PLATFORM_JETSON_E_TIMEOUT if the state of the pin does not change to true within the specified timeout period
/// \note error code from \ref PlatformTime_getDeadLine on error
/// \note error code from \ref PlatformTime_now on error
/// \note error code from \ref PlatformGpio_get on error
int waitForPin(uint16_t pinId, uint32_t timeoutMs);

#endif /* MAX2043XPEC_H */
