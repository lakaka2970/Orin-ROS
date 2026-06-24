/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */

#include "GmslSpiWrapper.h"

#include "IfxRfe_ErrorDefinitions.h"
#include "PlatformGpio.h"
#include "PlatformLogCallbacks.h"
#include "PlatformSpi.h"
#include "Util.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

/******************************************************************************/
/*-------------------------Local variables------------------------------------*/
/******************************************************************************/

#define MAX_DEVICES            4     ///< Maximum number of supported SPI devices
#define BURST_LEN              12    ///< Burst length for SPI transfers. Must be a multiple of 4 and max. 15.
#define BNE_WAIT_TIME          1000
#define CMD_TRANSMISSION_COUNT 5     ///< P2SSW-1184 GMSL commands (e.g. CS) are sometimes not processed correctly - cause yet unknown. 
                                     ///< Sending command multple times as a workaround.

static uint16_t const SPI_CS_NO_GPIO = 0xFFFF;

static uint16_t gRoGmslGpioId                           = SPI_CS_NO_GPIO;  ///< GPIO ID for the Read Only (RO) pin
static uint16_t gBneGmslGpioId                          = SPI_CS_NO_GPIO;  ///< GPIO ID for the Buffer Not Empty (BNE) pin
static gmsl_device_config_t gDeviceConfigs[MAX_DEVICES] = {0};             ///< Array of device configurations
static uint8_t gSpiIds[MAX_DEVICES]                     = {0};             ///< Array of SPI IDs for the SPI devices
static uint8_t gDevCnt                                  = 0;               ///< Number of SPI devices

/******************************************************************************/
/*-------------------------Local Function Prototypes--------------------------*/
/******************************************************************************/

static error_t clearRxBuffer(uint8_t spiId);
static error_t sendGmslCommands(uint8_t spiId, uint8_t* wbuf, uint32_t len);

/******************************************************************************/
/*-------------------------Global Function Implementations--------------------*/
/******************************************************************************/

int GmslSpiWrapper_setConfig(const gmsl_device_config_t *device_configs, uint8_t devCnt, uint16_t ro, uint16_t bne)
{
    if (device_configs == NULL || devCnt == 0)
    {
        return IFXRFE_E_FAILED;
    }

    gRoGmslGpioId  = ro;
    gBneGmslGpioId = bne;
    gDevCnt        = devCnt;

    if (gDevCnt > MAX_DEVICES)
    {
        gDevCnt = MAX_DEVICES;  // Limit the number of devices to the maximum supported
    }

    for (uint8_t i = 0; i < gDevCnt; i++)
    {
        gDeviceConfigs[i] = device_configs[i];
        gSpiIds[i]        = 0x10;  // Always use SPI device 1.0
    }

    PlatformSpi_init();

    // Extract device IDs for mapping
    uint8_t devIds[MAX_DEVICES];
    for (uint8_t i = 0; i < gDevCnt; i++)
    {
        devIds[i] = gDeviceConfigs[i].device_id;
    }

    // Set the device IDs
    PlatformSpi_setDevIdSpiIdMapping(devIds, gSpiIds, gDevCnt);

    return IFXRFE_E_SUCCESS;
}

int GmslSpiWrapper_configure(uint8_t spiId, uint8_t flags, uint32_t speed)
{
    return PlatformSpi_configure(spiId, flags, speed);
}

static gmsl_device_config_t *find_device_config(uint8_t spiId)
{
    for (uint8_t i = 0; i < gDevCnt; i++)
    {
        if (gDeviceConfigs[i].device_id == spiId)
        {
            return &gDeviceConfigs[i];
        }
    }
    return NULL;
}

int GmslSpiWrapper_write(uint8_t spiId, uint32_t count, const uint32_t buffer[], bool keepSel)
{
    DBG_PRINTF_L1("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);
    return GmslSpiWrapper_transfer(spiId, count, buffer, NULL, keepSel);
}

int GmslSpiWrapper_transfer(uint8_t spiId, uint32_t count, const uint32_t bufWrite[], uint32_t bufRead[],
                            bool keepSel)
{
    gmsl_device_config_t *config = find_device_config(spiId);
    if (config == NULL)
    {
        DBG_PRINTF_L1("Device config not found for spiId: 0x%02X\n", spiId);
        return IFXRFE_E_FAILED;
    }

    uint32_t len                        = count * sizeof(uint32_t);  // In bytes
    uint32_t temp_send_buf[16 / 4]      = {0};                       // /4 because we use uint32_t
    uint32_t temp_recv_buf[32 / 4]      = {0};
    uint32_t temp_read_send_buf[32 / 4] = {0xFF};
    uint32_t temp_read_recv_buf[32 / 4] = {0};
    
    // Clear the rx buffer
    RETURN_ON_IFX_ERROR(clearRxBuffer(spiId));

    // Use device-specific link and SS commands
    uint8_t link_ss_cmds[2] = {config->link_cmd, config->ss_cmd};
    RETURN_ON_IFX_ERROR(sendGmslCommands(spiId, link_ss_cmds, sizeof(link_ss_cmds)));
    DBG_PRINTF_L1("Using link_cmd: 0x%02X, ss_cmd: 0x%02X for device: 0x%02X\n", config->link_cmd, config->ss_cmd, spiId);

    // Burst mode - BURST_LEN bytes at a time (if needed)
    // Maintain a maximum BIT less than the maximum buffer size (16 bytes) (GMSL2 general UG 18.3.3 SPI Burst Read/Write)
    for (int i = 0; i < len; i += BURST_LEN)
    {
        // Clear RO
        RETURN_ON_IFX_ERROR(PlatformGpio_set(gRoGmslGpioId, false));

        memset(temp_send_buf, 0, sizeof(temp_send_buf));
        memset(temp_recv_buf, 0, sizeof(temp_recv_buf));

        int tmp_len = ((len - i) > BURST_LEN) ? BURST_LEN : (len - i);

        memcpy(temp_send_buf, ((uint8_t *)bufWrite) + i, tmp_len);
        RETURN_ON_IFX_ERROR(PlatformSpi_transfer(spiId, tmp_len / 4, temp_send_buf, temp_recv_buf, keepSel));
        RETURN_ON_IFX_ERROR(PlatformGpio_gpioWait(gBneGmslGpioId, true, BNE_WAIT_TIME));

        // Enter read mode
        RETURN_ON_IFX_ERROR(PlatformGpio_set(gRoGmslGpioId, true));

        memset(temp_read_recv_buf, 0, sizeof(temp_read_recv_buf));
        memset(temp_read_send_buf, 0, sizeof(temp_read_send_buf));
        RETURN_ON_IFX_ERROR(PlatformSpi_transfer(spiId, tmp_len / 4, temp_read_send_buf, temp_read_recv_buf, keepSel));
        if (bufRead)
        {
            memcpy(((uint8_t *)bufRead) + i, temp_read_recv_buf, tmp_len);
        }
        
        // P2SSW-1184: For unknown reasons, the RX buffer appears to be not empty at this point - clearing buffer manually as workaround
        // RETURN_ON_IFX_ERROR(PlatformGpio_gpioWait(gBneGmslGpioId, false, BNE_WAIT_TIME));
        RETURN_ON_IFX_ERROR(clearRxBuffer(spiId)); 
    }

    // Deassert SS if required
    if (!keepSel)
    {
        uint8_t deassertCmd = GMSL_DEASS_SS_CMD;
        RETURN_ON_IFX_ERROR(sendGmslCommands(spiId, &deassertCmd, 1));
    }

    return IFXRFE_E_SUCCESS;
}

void GmslSpiWrapper_deinit()
{
    DBG_PRINTF_L1("DEBUG from PlatformSpi_deinit!\n");
    uint32_t temp_read_send_buf[32 / 4] = {0xFF};
    uint32_t temp_read_recv_buf[32 / 4] = {0};
    bool temp_state;

    // Clear the rx buffer
    temp_read_send_buf[0] = GMSL_DEASS_SS_CMD;
    (PlatformGpio_set(gRoGmslGpioId, true));
    // Release SS to get CTRX_RFT to high again
    PlatformSpi_transfer8(0, 1, (uint8_t *)temp_read_send_buf, (uint8_t *)temp_read_recv_buf, false);
    (PlatformGpio_get(gBneGmslGpioId, &temp_state));
    if (0 != temp_state)
    {
        PlatformSpi_transfer8(0, sizeof(temp_read_send_buf), (uint8_t *)temp_read_send_buf, (uint8_t *)temp_read_recv_buf, false);
        (PlatformGpio_get(gBneGmslGpioId, &temp_state));
        if (0 != temp_state)
        {
            DBG_PRINTF_L1("SPI BNE should be low, but it's high\n");
        }
    }

    PlatformSpi_deinit();
}

/******************************************************************************/
/*-------------------------Local Function Implementations---------------------*/
/******************************************************************************/

static error_t clearRxBuffer(uint8_t spiId)
{
    bool bne_state;
    uint32_t temp_read_send_buf[32 / 4] = {0};
    uint32_t temp_read_recv_buf[32 / 4] = {0};

    RETURN_ON_IFX_ERROR(PlatformGpio_set(gRoGmslGpioId, true));

    RETURN_ON_IFX_ERROR(PlatformGpio_get(gBneGmslGpioId, &bne_state));
    if (bne_state != 0)
    {
        RETURN_ON_IFX_ERROR(PlatformSpi_transfer8(spiId, sizeof(temp_read_send_buf), (uint8_t *)temp_read_send_buf, (uint8_t *)temp_read_recv_buf, false));
    }

    RETURN_ON_IFX_ERROR(PlatformGpio_get(gBneGmslGpioId, &bne_state));
    if (bne_state != 0)
    {
        DBG_PRINTF_L1("SPI BNE should be low, but it's high\n");
        return IFXRFE_E_FAILED;
    }

    RETURN_ON_IFX_ERROR(PlatformGpio_set(gRoGmslGpioId, false));
    return IFXRFE_E_SUCCESS;
}

static error_t sendGmslCommands(uint8_t spiId, uint8_t* wbuf, uint32_t len)
{
    uint8_t temp_send_buf[len];
    for (int i = 0; i < CMD_TRANSMISSION_COUNT; i++)
    {
        RETURN_ON_IFX_ERROR(PlatformGpio_set(gRoGmslGpioId, true));
        RETURN_ON_IFX_ERROR(PlatformSpi_transfer8(spiId, len, wbuf, temp_send_buf, false));
        RETURN_ON_IFX_ERROR(PlatformGpio_set(gRoGmslGpioId, false));
    }

    return IFXRFE_E_SUCCESS;
}