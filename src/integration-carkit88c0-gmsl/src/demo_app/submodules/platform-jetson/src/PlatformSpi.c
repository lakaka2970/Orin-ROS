/*
 * File: PlatformSpi.c
 * Description: Platform-dependant iRFE SPI API implementation for the NVIDIA Jetson
 * Project: MIMOrad
 * Created Date: Tuesday, 23 April 2024, 10:07
 * Author: Daniel Klepatsch (daniel.klepatsch@silicon-austria.com)
 * ---------------
 * Copyright (c) 2024 Silicon Austria Labs GmbH
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "PlatformErrors.h"

#include "PlatformGpio.h"
#include "PlatformLogCallbacks.h"
#include "PlatformSpi.h"
#include "PlatformTime.h"
#include "Util.h"

static uint16_t const SPI_CS_NO_GPIO = 0xFFFF;
static uint16_t gCsGpioIds[0xFF]     = {SPI_CS_NO_GPIO};

// Maps spiID <=> devId => map[devId] == (spiID | NO_MAPPING)
static uint8_t const NO_MAPPING     = 0xFF;
static uint8_t gDevIdSpiIdMap[0xFF] = {NO_MAPPING};

#define PLATFORM_SPI_CPOL (1u << 0)                                  ///< Clock Polarity CPOL = 1 (otherwise, CPOL = 0)
#define PLATFORM_SPI_CPHA (1u << 1)                                  ///< Clock Phase CPHA = 1 (otherwise, CPHA = 0)

#define PLATFORM_SPI_MODE_0 (0 | 0)                                  ///< Clock Polarity CPOL = 0, Clock Phase CPHA = 0
#define PLATFORM_SPI_MODE_1 (0 | PLATFORM_SPI_CPHA)                  ///< Clock Polarity CPOL = 0, Clock Phase CPHA = 1
#define PLATFORM_SPI_MODE_2 (PLATFORM_SPI_CPOL | 0)                  ///< Clock Polarity CPOL = 1, Clock Phase CPHA = 0
#define PLATFORM_SPI_MODE_3 (PLATFORM_SPI_CPOL | PLATFORM_SPI_CPHA)  ///< Clock Polarity CPOL = 1, Clock Phase CPHA = 1

#if DEBUG > 0
/**
 * @brief Print SPI buffer contents as 32-bit words
 * @param buffer Pointer to 32-bit buffer
 * @param word_count Number of 32-bit words to print
 */
static void debug_print_spi_buffer_32(const uint32_t *buffer, size_t word_count)
{
    if (buffer != NULL && word_count > 0)
    {
        DBG_PRINTF_L1("        ");
        for (size_t i = 0; i < word_count; i++)
        {
            if (i == 0)
            {
                DBG_PRINTF_L1("0x%08X", buffer[i]);
            }
            else
            {
                DBG_PRINTF_L1(", 0x%08X", buffer[i]);
            }
        }
        DBG_PRINTF_L1("\n");
    }
}

/**
 * @brief Print SPI buffer contents as 8-bit bytes
 * @param buffer Pointer to 8-bit buffer
 * @param byte_count Number of bytes to print
 */
static void debug_print_spi_buffer_8(const uint8_t *buffer, size_t byte_count)
{
    if (buffer != NULL && byte_count > 0)
    {
        DBG_PRINTF_L1("        ");
        for (size_t i = 0; i < byte_count; i++)
        {
            if (i == 0)
            {
                DBG_PRINTF_L1("0x%02X", buffer[i]);
            }
            else
            {
                DBG_PRINTF_L1(", 0x%02X", buffer[i]);
            }
        }
        DBG_PRINTF_L1("\n");
    }
}

/**
 * @brief Print SPI buffer contents with variable bit width
 * @param buffer Pointer to buffer
 * @param count Number of elements to print
 * @param bits_per_word 8 for bytes, 32 for words
 * @param label Label for the buffer (e.g., "wrBuffer", "rdBuffer")
 */
static void debug_print_spi_buffer_variable(const void *buffer, size_t count, uint8_t bits_per_word, const char *label)
{
    if (buffer != NULL)
    {
        DBG_PRINTF_L1("    %s:\n", label);
        if (bits_per_word == 8)
        {
            debug_print_spi_buffer_8((const uint8_t *)buffer, count);
        }
        else if (bits_per_word == 32)
        {
            debug_print_spi_buffer_32((const uint32_t *)buffer, count);
        }
    }
    else
    {
        DBG_PRINTF_L1("    No SPI %s\n", label);
    }
}
#endif

struct SpiDevice
{
    uint8_t spiId;
    int fileDescriptor;
    struct SpiDevice *next;
};
typedef struct SpiDevice SpiDevice;

static SpiDevice *gSpiDevicesHead = NULL;

void PlatformSpi_init(void)
{
    for (int i = 0; i < 0xFF; i++)
    {
        gCsGpioIds[i]     = SPI_CS_NO_GPIO;
        gDevIdSpiIdMap[i] = NO_MAPPING;
    }
}

static SpiDevice *PlatfromSpi_SpiDevices_Find(uint8_t const spiId)
{
    DBG_PRINTF_L2("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);
    DBG_PRINTF_L2("    spiId = 0x%02X\n", spiId);

    if (gSpiDevicesHead == NULL)
    {
        DBG_PRINTF_L2("    Not found (no SpiDevices)\n");
        return NULL;
    }
    else
    {
        SpiDevice *spiDevice = gSpiDevicesHead;
        do
        {
            if (spiDevice->spiId == spiId)
            {
                DBG_PRINTF_L2("    Found\n");
                return spiDevice;
            }
            spiDevice = spiDevice->next;
        } while (spiDevice != NULL);
        DBG_PRINTF_L2("    Not found\n");
        return NULL;
    }
}

static SpiDevice *PlatfromSpi_SpiDevices_Create(uint8_t const spiId)
{
    DBG_PRINTF_L2("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);
    DBG_PRINTF_L2("    spiId = 0x%02X\n", spiId);

    if (gSpiDevicesHead == NULL)
    {
        DBG_PRINTF_L2("    Create at HEAD\n");
        gSpiDevicesHead = malloc(sizeof(SpiDevice));
        if (gSpiDevicesHead != NULL)
        {
            gSpiDevicesHead->spiId          = spiId;
            gSpiDevicesHead->fileDescriptor = -1;
            gSpiDevicesHead->next           = NULL;
        }
        return gSpiDevicesHead;
    }
    else
    {
        SpiDevice *spiDevice = gSpiDevicesHead;
        do
        {
            if (spiDevice->spiId == spiId)
            {
                DBG_PRINTF_L2("    Not created, spiId found!\n");
                errno = EEXIST;
                return NULL;
            }
            if (spiDevice->next == NULL)
            {
                DBG_PRINTF_L2("    Created\n");
                SpiDevice *newSpiDevice = malloc(sizeof(SpiDevice));
                if (newSpiDevice == NULL)
                {
                    return NULL;
                }
                newSpiDevice->spiId          = spiId;
                newSpiDevice->fileDescriptor = -1;
                newSpiDevice->next           = NULL;
                spiDevice->next              = newSpiDevice;
                return newSpiDevice;
            }
            spiDevice = spiDevice->next;
        } while (spiDevice != NULL);
    }
    DBG_PRINTF_L2("    Not created?\n");
    return NULL;
}

static void PlatfromSpi_SpiDevices_Delete(uint8_t const spiId)
{
    DBG_PRINTF_L2("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);
    DBG_PRINTF_L2("    spiId = 0x%02X\n", spiId);

    if (gSpiDevicesHead != NULL)
    {
        SpiDevice *spiDevice = gSpiDevicesHead;

        if (spiDevice->spiId == spiId)
        {
            DBG_PRINTF_L2("    Found at HEAD\n");
            gSpiDevicesHead = spiDevice->next;

            // Close fd and free memory
            if (spiDevice->fileDescriptor >= 0)
            {
                close(spiDevice->fileDescriptor);
                DBG_PRINTF_L2("    FD closed\n");
            }
            free(spiDevice);
            DBG_PRINTF_L2("    Device freed\n");
            return;
        }

        while (spiDevice->next != NULL)
        {
            if (spiDevice->next->spiId == spiId)
            {
                DBG_PRINTF_L2("    Found\n");

                SpiDevice *spiDeviceToDelete = spiDevice->next;

                // Unlink from list
                spiDevice->next = spiDevice->next->next;

                // Close fd and free memory
                if (spiDeviceToDelete->fileDescriptor >= 0)
                {
                    close(spiDeviceToDelete->fileDescriptor);
                    DBG_PRINTF_L2("    FD closed\n");
                }
                free(spiDeviceToDelete);
                DBG_PRINTF_L2("    Device freed\n");
                return;
            }
            spiDevice = spiDevice->next;
        }

        DBG_PRINTF_L2("    Not found\n");
    }
}

static void PlatfromSpi_SpiDevices_DeleteAll()
{
    DBG_PRINTF_L2("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);

    SpiDevice *curSpiDevice = gSpiDevicesHead;
    while (curSpiDevice != NULL)
    {
        DBG_PRINTF_L2("    Deleting id 0x%02X\n", curSpiDevice->spiId);
        SpiDevice *nextSpiDevice = curSpiDevice->next;
        if (curSpiDevice->fileDescriptor >= 0)
        {
            close(curSpiDevice->fileDescriptor);
            DBG_PRINTF_L2("        FD closed\n");
        }
        free(curSpiDevice);
        DBG_PRINTF_L2("        Device freed\n");
        curSpiDevice = nextSpiDevice;
    }
}

#ifdef SPI_STATS_ENABLED
static int64_t assert_duration_acc   = 0;
static int64_t send_duration_acc     = 0;
static int64_t deassert_duration_acc = 0;
static uint64_t stat_count           = 0;
static uint64_t transfer_count       = 0;
static int64_t find_duration_acc     = 0;
static uint64_t find_count           = 0;
#endif

int PlatformSpi_transfer_variable(uint8_t spiId, uint32_t byte_count,
                                  const void *bufWrite, void *bufRead,
                                  bool keepSel, uint8_t bitsPerWord)
{
    SPI_STATS(transfer_count++);

    DBG_PRINTF_L1("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);
    DBG_PRINTF_L1("    spiId   = 0x%02X\n", spiId);
    DBG_PRINTF_L1("    keepSel = %d\n", keepSel);
    DBG_PRINTF_L1("    byte_count   = %d\n", byte_count);

#if DEBUG > 0
    debug_print_spi_buffer_variable(bufWrite, (bitsPerWord == 8) ? byte_count : byte_count / 4, bitsPerWord, "wrBuffer");
#endif

    // Check if device ID maps to SPI ID (i.e.: spiId is actually a devId)
    uint8_t const devId       = spiId;
    uint8_t const spiIdMapped = gDevIdSpiIdMap[devId];

    if (spiIdMapped != NO_MAPPING)
    {
        spiId = spiIdMapped;
        DBG_PRINTF_L1("    spiId is actually a devId!\n");
        DBG_PRINTF_L1("    devId %d -> spiId 0x%02X\n", devId, spiIdMapped);
    }

    // Get pin ID for GPIO-based chip select, if used
    uint16_t const cs_gpio_id = gCsGpioIds[spiId];

    if (cs_gpio_id != SPI_CS_NO_GPIO)
    {
        // Always use spidevX.0 if GPIO CS is used
        spiId &= 0xF0;
        DBG_PRINTF_L1("    csGpioId = %d\n", cs_gpio_id);
        DBG_PRINTF_L1("    new spiId = 0x%02X\n", spiId);
    }

    SPI_STATS(int64_t start_find = PlatformTime_now());
    SpiDevice *spiDevice = PlatfromSpi_SpiDevices_Find(spiId);
    SPI_STATS(int64_t end_find = PlatformTime_now());
    SPI_STATS(find_duration_acc += (end_find - start_find));
    SPI_STATS(find_count++);

    if (spiDevice == NULL)
    {
        return PLATFORM_JETSON_E_NOT_CONFIGURED;
    }

    // Create message
    struct spi_ioc_transfer msg;
    memset(&msg, 0, sizeof(msg));

    msg.tx_buf        = (__u64)bufWrite;
    msg.rx_buf        = (__u64)bufRead;
    msg.len           = byte_count;
    msg.cs_change     = false;
    msg.bits_per_word = bitsPerWord;

    DBG_PRINTF_L1("    msg.tx_buf           = %p\n", (void *)msg.tx_buf);
    DBG_PRINTF_L1("    msg.rx_buf           = %p\n", (void *)msg.rx_buf);
    DBG_PRINTF_L1("    msg.len              = %d\n", msg.len);
    DBG_PRINTF_L1("    msg.speed_hz         = %d\n", msg.speed_hz);
    DBG_PRINTF_L1("    msg.bits_per_word    = %d\n", msg.bits_per_word);
    DBG_PRINTF_L1("    msg.delay_usecs      = %d\n", msg.delay_usecs);
    DBG_PRINTF_L1("    msg.cs_change        = %d\n", msg.cs_change);
    DBG_PRINTF_L1("    msg.word_delay_usecs = %d\n", msg.word_delay_usecs);

#ifdef SPI_STATS_ENABLED
    SPI_STATS(int64_t start = PlatformTime_now());
#endif

    // Assert Chip Select
    if (cs_gpio_id != SPI_CS_NO_GPIO)
    {
        PlatformGpio_set(cs_gpio_id, false);
    }

#ifdef SPI_STATS_ENABLED
    SPI_STATS(int64_t after_assert = PlatformTime_now());
#endif

    // Send message
    int sysret = ioctl(spiDevice->fileDescriptor, SPI_IOC_MESSAGE(1), &msg);
    if (sysret < 0)
    {
        char msg[128];
        snprintf(msg, 128,
                 "ioctl() for SPI_IOC_MESSAGE returned %d, errno = %d (%s)", sysret,
                 errno, strerror(errno));
        PlatformLogCallbacks_error(msg);
        return PLATFORM_JETSON_E_FAILED;
    }

#ifdef SPI_STATS_ENABLED
    SPI_STATS(int64_t after_send = PlatformTime_now());
#endif

    // Deassert Chip Select
    if (cs_gpio_id != SPI_CS_NO_GPIO && !keepSel)
    {
        PlatformGpio_set(cs_gpio_id, true);
    }

#ifdef SPI_STATS_ENABLED
    int64_t after_deassert = PlatformTime_now();

    int64_t assert_duration   = after_assert - start;
    int64_t send_duration     = after_send - after_assert;
    int64_t deassert_duration = after_deassert - after_send;

    assert_duration_acc += assert_duration;
    send_duration_acc += send_duration;
    deassert_duration_acc += deassert_duration;
    stat_count++;
#endif

#if DEBUG > 0
    debug_print_spi_buffer_variable(bufRead, (bitsPerWord == 8) ? byte_count : byte_count / 4, bitsPerWord, "rdBuffer");
#endif

    return PLATFORM_JETSON_E_SUCCESS;
}

int PlatformSpi_transfer(uint8_t spiId, uint32_t word_count,
                         const uint32_t bufWrite[], uint32_t bufRead[],
                         bool keepSel)
{
    return PlatformSpi_transfer_variable(spiId, word_count * 4, bufWrite, bufRead, keepSel, 32);
}

int PlatformSpi_transfer8(uint8_t spiId, uint32_t byte_count,
                          const uint8_t bufWrite[], uint8_t bufRead[],
                          bool keepSel)
{
    return PlatformSpi_transfer_variable(spiId, byte_count, bufWrite, bufRead, keepSel, 8);
}

#ifdef SPI_STATS_ENABLED
void PlatformSpi_printStats()
{
    printf("SPI statistics:\n");
    printf("    Transfer count: %ld\n", transfer_count);
    printf("    Avg. Find dur.: %ld us\n", find_duration_acc / find_count);
    printf("    Avg. Assert   : %ld us\n", assert_duration_acc / stat_count);
    printf("    Avg. Send     : %ld us\n", send_duration_acc / stat_count);
    printf("    Avg. Deassert : %ld us\n", deassert_duration_acc / stat_count);
}

void PlatformSpi_resetStats()
{
    assert_duration_acc   = 0;
    send_duration_acc     = 0;
    deassert_duration_acc = 0;
    stat_count            = 0;
    transfer_count        = 0;
    find_duration_acc     = 0;
    find_count            = 0;
}
#endif  // SPI_STATS_ENABLED

int PlatformSpi_write(uint8_t spiId, uint32_t count, const uint32_t buffer[],
                      bool keepSel)
{
    DBG_PRINTF_L1("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);
    return PlatformSpi_transfer(spiId, count, buffer, NULL, keepSel);
}

int PlatformSpi_configure(uint8_t spiId, uint8_t flags, uint32_t speed)
{
    DBG_PRINTF_L1("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);
    DBG_PRINTF_L1("    spiId = 0x%02X\n", spiId);
    DBG_PRINTF_L1("    flags = %d\n", flags);
    DBG_PRINTF_L1("    speed = %d\n", speed);

    // Check if device ID maps to SPI ID (i.e.: spiId is actually a devId)
    uint8_t const devId       = spiId;
    uint8_t const spiIdMapped = gDevIdSpiIdMap[devId];

    if (spiIdMapped != NO_MAPPING)
    {
        spiId = spiIdMapped;
        DBG_PRINTF_L1("    spiId is actually a devId!\n");
        DBG_PRINTF_L1("    devId %d -> spiId 0x%02X\n", devId, spiIdMapped);
    }

    // Get pin ID for GPIO-based chip select, if used
    uint16_t const cs_gpio_id = gCsGpioIds[spiId];

    if (cs_gpio_id != SPI_CS_NO_GPIO)
    {
        // Always use spidevX.0 if GPIO CS is used
        spiId &= 0xF0;
        DBG_PRINTF_L1("    csGpioId = %d\n", cs_gpio_id);
        DBG_PRINTF_L1("    new spiId = 0x%02X\n", spiId);
    }

    SpiDevice *spiDevice         = PlatfromSpi_SpiDevices_Find(spiId);
    bool const alreadyConfigured = spiDevice != NULL;

    if (!alreadyConfigured)
    {
        // Generate device file name
        char spidevname[32];
        sprintf(spidevname, "/dev/spidev%d.%d", ((spiId >> 4) & 0xF),
                (spiId & 0xF));

        DBG_PRINTF_L1("    spidevname = %s\n", spidevname);

        // Open device file
        int fd = open(spidevname, O_RDWR);
        if (fd < 0)
        {
            char msg[128];
            snprintf(msg, 128, "open() returned %d, errno = %d (%s)", fd, errno,
                     strerror(errno));
            PlatformLogCallbacks_error(msg);
            return PLATFORM_JETSON_E_FAILED;
        }

        spiDevice = PlatfromSpi_SpiDevices_Create(spiId);
        if (spiDevice == NULL)
        {
            char msg[64];
            sprintf(msg, "Failed to create a SpiDevice to configure!");
            PlatformLogCallbacks_error(msg);
            return PLATFORM_JETSON_E_FAILED;
        }
        spiDevice->fileDescriptor = fd;
    }

    // Assign SPI Mode
    // Mode definitions differ between IfxRfe and Linux
    // Other flags kept at 0, see:
    // https://elixir.bootlin.com/linux/latest/source/include/uapi/linux/spi/spi.h#L16
    uint32_t spi_mode = 0;
    switch (flags)
    {
        case PLATFORM_SPI_MODE_0:
            spi_mode = SPI_MODE_0;
            break;
        case PLATFORM_SPI_MODE_1:
            spi_mode = SPI_MODE_1;
            break;
        case PLATFORM_SPI_MODE_2:
            spi_mode = SPI_MODE_2;
            break;
        case PLATFORM_SPI_MODE_3:
            spi_mode = SPI_MODE_3;
            break;
        default:
            if (!alreadyConfigured)
            {
                PlatfromSpi_SpiDevices_Delete(spiId);
            }
            return PLATFORM_JETSON_E_INVALID_PARAMETER;
    }

    DBG_PRINTF_L1("    spi_mode = 0x%X\n", spi_mode);

    int sysret = ioctl(spiDevice->fileDescriptor, SPI_IOC_WR_MODE32, &spi_mode);
    if (sysret < 0)
    {
        if (!alreadyConfigured)
        {
            PlatfromSpi_SpiDevices_Delete(spiId);
        }
        char msg[128];
        snprintf(msg, 128,
                 "ioctl() for SPI_IOC_WR_MODE32 returned %d, errno = %d (%s)",
                 sysret, errno, strerror(errno));
        PlatformLogCallbacks_error(msg);
        return PLATFORM_JETSON_E_FAILED;
    }

    uint8_t const bits_per_word = 32;
    sysret                      = ioctl(spiDevice->fileDescriptor, SPI_IOC_WR_BITS_PER_WORD,
                                        &bits_per_word);
    if (sysret < 0)
    {
        if (!alreadyConfigured)
        {
            PlatfromSpi_SpiDevices_Delete(spiId);
        }
        char msg[128];
        snprintf(
            msg, 128,
            "ioctl() for SPI_IOC_WR_BITS_PER_WORD returned %d, errno = %d (%s)",
            sysret, errno, strerror(errno));
        PlatformLogCallbacks_error(msg);
        return PLATFORM_JETSON_E_FAILED;
    }

    sysret = ioctl(spiDevice->fileDescriptor, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (sysret < 0)
    {
        if (!alreadyConfigured)
        {
            PlatfromSpi_SpiDevices_Delete(spiId);
        }
        char msg[128];
        snprintf(msg, 128,
                 "ioctl() for SPI_IOC_WR_MAX_SPEED_HZ returned %d, errno = %d (%s)",
                 sysret, errno, strerror(errno));
        PlatformLogCallbacks_error(msg);
        return PLATFORM_JETSON_E_FAILED;
    }

    if (cs_gpio_id != SPI_CS_NO_GPIO)
    {
        // Configure GPIO pin for manual chip select
        return PlatformGpio_configure(
            cs_gpio_id, GPIO_FLAG_PULL_UP | GPIO_FLAG_OUTPUT_INITIAL_HIGH);
    }
    else
    {
        // GPIO based chip select not implemented for given SPI device
        char msg[128];
        sprintf(msg,
                "GPIO-based Chip Select is not implemented for SPI device with ID "
                "0x%02X!",
                spiId);
        DBG_PRINTF_L1(msg);
        return PLATFORM_JETSON_E_SUCCESS;
    }

    return PLATFORM_JETSON_E_SUCCESS;
}

void PlatformSpi_setCsGpioIds(uint8_t *spiIds, uint16_t *spiCsGpioIds,
                              uint8_t const len)
{
    DBG_PRINTF_L1("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);
    DBG_PRINTF_L1("    Num of CsGpioIds: %d\n", len);
    for (size_t i = 0; i < len; i++)
    {
        gCsGpioIds[spiIds[i]] = spiCsGpioIds[i];
        DBG_PRINTF_L1("        spiId: 0x%02X -> csGpioId: %d\n", spiIds[i],
                      spiCsGpioIds[i]);
    }
}

void PlatformSpi_setDevIdSpiIdMapping(uint8_t *devIds, uint8_t *spiIds,
                                      uint8_t const len)
{
    DBG_PRINTF_L1("DEBUG from %s() at %s:%d:\n", __func__, __FILE__, __LINE__);
    DBG_PRINTF_L1("    Num of mappings: %d\n", len);
    for (size_t i = 0; i < len; i++)
    {
        gDevIdSpiIdMap[devIds[i]] = spiIds[i];
        DBG_PRINTF_L1("        devId: %d -> spiId: 0x%02X\n", devIds[i], spiIds[i]);
    }
}

void PlatformSpi_deinit()
{
    // Deinit GPIO in case they were used for chip select
    PlatformGpio_deinit();

    // Clean up SPI
    PlatfromSpi_SpiDevices_DeleteAll();

    // Reset CS GPIO IDs
    for (size_t i = 0; i < 0xFF; i++)
    {
        gCsGpioIds[i] = SPI_CS_NO_GPIO;
    }

    // Reset devId-spiId mapping
    for (size_t i = 0; i < 0xFF; i++)
    {
        gDevIdSpiIdMap[i] = NO_MAPPING;
    }
}
