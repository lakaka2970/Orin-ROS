/*
 * File: PlatformSpi.h
 * Description: Platform-dependant iRFE SPI API implementation for the NVIDIA Jetson
 * Project: MIMOrad
 * Created Date: Tuesday, 23 April 2024, 10:07
 * Author: Daniel Klepatsch (daniel.klepatsch@silicon-austria.com)
 * ---------------
 * Copyright (c) 2024 Silicon Austria Labs GmbH
 */

#ifndef PLATFORMSPI_H
#define PLATFORMSPI_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /*
 *   spiId is derived from the /dev/spidevA.B device file:
 *       spiId = ((A & 0xF) << 4) | (B & 0xF)
 */

    int PlatformSpi_transfer(uint8_t spiId, uint32_t count, const uint32_t bufWrite[], uint32_t bufRead[], bool keepSel);
    int PlatformSpi_write(uint8_t spiId, uint32_t count, const uint32_t buffer[], bool keepSel);
    int PlatformSpi_configure(uint8_t spiId, uint8_t flags, uint32_t speed);


    /*
 * Performs a variable bit-width SPI transfer operation.
 * Arguments:
 *   spiId       ... SPI device identifier
 *   byte_count  ... Number of bytes to transfer
 *   bufWrite    ... Pointer to write buffer (input data)
 *   bufRead     ... Pointer to read buffer (output data)
 *   keepSel     ... Whether to keep chip select active after transfer
 *   bitsPerWord ... Number of bits per word (transfer width)
 */
    int PlatformSpi_transfer_variable(uint8_t spiId, uint32_t byte_count, const void *bufWrite, void *bufRead, bool keepSel, uint8_t bitsPerWord);

    /*
 * Performs an 8-bit SPI transfer operation.
 * Arguments:
 *   spiId    ... SPI device identifier
 *   count    ... Number of 8-bit elements to transfer
 *   bufWrite ... Array of 8-bit values to write (input data)
 *   bufRead  ... Array to store 8-bit values read (output data)
 *   keepSel  ... Whether to keep chip select active after transfer
 */
    int PlatformSpi_transfer8(uint8_t spiId, uint32_t count, const uint8_t bufWrite[], uint8_t bufRead[], bool keepSel);

    /*
 *   Sets the Chip-Select GPIO ID for the given SPI IDs.
 *   By default, no GPIO is used for communication and the spidev file according to the SPI ID is used.
 *   If set, the spidevX.0 device file will be used for communication and the corresponding GPIO will be
 *   toggled manually.
 *   Arguments:
 *       spiIds       ... array of SPI IDs
 *       spiCsGpioIds ... array of GPIO IDs for the corresponding SPI ID
 *       len          ... length of the arrays
 */
    void PlatformSpi_setCsGpioIds(uint8_t *spiIds, uint16_t *spiCsGpioIds, uint8_t const len);

    /*
 *  Sets values for the mapping from device ID (used internally when using iRFE middleware and cascaded radars) to SPI ID.
 *  By default, when no mapping is defined, the spiId of the functions will be used to directly address the spidev.
 *  If set, spiId is treated as devId and addressing the spidev is defined by this mapping.
 *  Arguments:
 *      devIds ... array of devices IDs
 *      spiIds ... array of SPI IDs
 *      len    ... lenght of both arrays
 */
    void PlatformSpi_setDevIdSpiIdMapping(uint8_t *devIds, uint8_t *spiIds, uint8_t const len);

    /*
 * Initializes the Platform SPI subsystem.
 * This function should be called once before using any other PlatformSpi functions.
 */
    void PlatformSpi_init(void);

    /*
 * Deinitializes the Platform SPI subsystem.
 * This function should be called to clean up resources when SPI operations are no longer needed.
 */
    void PlatformSpi_deinit(void);

#define SPI_STATS_ENABLED 0

#ifdef SPI_STATS_ENABLED
    #define SPI_STATS(code) code
    void PlatformSpi_printStats();
    void PlatformSpi_resetStats();
#else
    #define SPI_STATS(code)
#endif

#ifdef __cplusplus
}
#endif

#endif  // PLATFORMSPI_H
