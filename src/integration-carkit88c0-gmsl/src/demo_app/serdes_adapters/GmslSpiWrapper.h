#ifndef GMSLSPIWRAPPER_H
#define GMSLSPIWRAPPER_H

#include "PlatformSpi.h"
#include <stdbool.h>
#include <stdint.h>

#define GMSL_DEASS_SS_CMD 0xA6  // Deassert both SS outputs

// Enum for GMSL link commands
typedef enum
{
    GMSL_SPI_ID_IGNORE = 0xA0,  // SPI ID ignore (use '00')
    GMSL_SPI_ID_00     = 0xA0,  // Command for SPI ID '00'
    GMSL_SPI_ID_01     = 0xA1,  // Command for SPI ID '01'
    GMSL_SPI_ID_10     = 0xA2,  // Command for SPI ID '10'
    GMSL_SPI_ID_11     = 0xA3   // Command for SPI ID '11'
} gmsl_link_cmd_t;

// Enum for GMSL slave select commands
typedef enum
{
    GMSL_ASS_SS1 = 0xA4,  // Assert SS1 command
    GMSL_ASS_SS2 = 0xA5   // Assert SS2 command
} gmsl_ss_cmd_t;

// Device configuration structure
typedef struct
{
    uint8_t device_id;
    gmsl_link_cmd_t link_cmd;
    gmsl_ss_cmd_t ss_cmd;
} gmsl_device_config_t;

int GmslSpiWrapper_configure(uint8_t spiId, uint8_t flags, uint32_t speed);
int GmslSpiWrapper_write(uint8_t spiId, uint32_t count, const uint32_t buffer[], bool keepSel);
int GmslSpiWrapper_transfer(uint8_t spiId, uint32_t count, const uint32_t bufWrite[], uint32_t bufRead[],
                            bool keepSel);

/**
 * @brief Sets the wrapper configuration with device-specific link commands
 *
 * @param device_configs Array of device configurations including link and SS commands
 * @param devCnt Number of devices
 * @param ro GPIO ID for the Read Only (RO) pin
 * @param bne GPIO ID for the Buffer Not Empty (BNE) pin
 *
 * @return Status code indicating success or failure
 */
int GmslSpiWrapper_setConfig(const gmsl_device_config_t *device_configs, uint8_t devCnt, uint16_t ro, uint16_t bne);

/**
 * @brief Deinitialize the SPI interface
 * 
 * Deasserts the CSs over GMSL (by sending 0xA6 command) and calls PlatformSpi_deinit().
 * 
 * @return void
 */
void GmslSpiWrapper_deinit();

#endif  // GMSLSPIWRAPPER_H