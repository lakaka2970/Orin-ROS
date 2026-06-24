/*
 * (c) (2022-2025), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include "../HwDefinitions.h"
#include "8188/IfxRfe_FirmwareCommands.h"
#include "8188/IfxRfe_Specific.h"
#include "GmslSpiWrapper.h"
#include "IfxRfe.h"
#include "IfxRfe_SpiWrapper.h"
#include "Max20434Pec.h"
#include "PlatformGpio.h"
#include "PlatformI2c.h"
#include "PlatformLogCallbacks.h"
#include "PlatformTime.h"
#include "config.h"
#include "sequencer_data.h"
#include "../../error_macros.h"

// Constants
#define CTRX_DEV_COUNT           1
#define DEVICE_STANDALONE        0
#define EXPECTED_OPERATION_STATE 0x10000000
#define EXPECTED_INITIAL_STATE   0x20000000
#define SPI_CLOCK_SPEED          10000000  // 10 MHz
#define RAMP_SCENARIO_TIMEOUT_US 20000000
#define LOOP_DELAY_US            100000
#define RESET_DELAY_US           5000   // 5ms delay after reset toggle
#define FINISH_RAMP_DELAY_US     50000  // 50ms delay to finish ramp scenario
#define NUM_ITERATIONS           50
#define SPI_RETRANSMISSIONS      4

/**
 * @brief Cleanup platform resources
 * 
 * Deinitializes all platform components.
 * This function should be called before program termination or on error
 * to ensure proper resource cleanup.
 * 
 * Resources cleaned up:
 * - I2C interface
 * - SPI interface  
 * - GPIO interface
 */
static void cleanup_platform(void)
{
    PlatformI2c_deinit();     // Deinitialize I2C interface
    GmslSpiWrapper_deinit();  // Deinitialize SPI interface
    PlatformGpio_deinit();    // Deinitialize GPIO interface
}

/**
 * @brief Print device error status information
 * 
 * This function displays detailed error status information for debugging
 * purposes when CTRX initialization or operation encounters issues.
 * 
 * @param deviceName Human-readable device name for output (e.g., "CTRX")
 * @param result Pointer to error result structure
 */
static void print_error_status(const char *deviceName, const IfxRfe_handleErrorResult_t *result)
{
    printf("HandleError %s\n", deviceName);
    printf("-------------------------------------\n");
    printf("\tError Status 1 Word 0: 0x%X\n", result->error_status_1);
    for (int i = 0; i < result->length; i++)
    {
        printf("\tError Status 2 Word %d: 0x%X\n", i, result->error_status_2[i]);
    }
    printf("-------------------------------------\n");
}

/**
 * @brief Main application entry point
 * 
 * This function demonstrates the complete initialization and operation sequence
 * for a radar system using the CARKIT88C0 board. The application performs the following
 * major steps:
 * 
 * 1. Platform initialization (I2C, SPI, GPIO interfaces)
 * 2. IfxRfe library initialization with function callbacks
 * 3. CTRX status checking and conditional initialization
 * 4. RF parameter configuration
 * 5. CTRX transition to operational state
 * 6. TX calibration execution
 * 7. Continuous radar measurement cycles (ramp scenarios)
 * 8. Final cleanup and power down
 * 
 * The application is designed to work with the CARKIT88C0 hardware which
 * contains one CTRX 8188 device.
 * 
 * @return EXIT_SUCCESS (0) on successful completion
 * @return Error code (non-zero) if any step fails
 */
int main(void)
{
    printf("=== IfxRfe App - CARKIT88C0 Example ===\n");

    const Max20434Pec_t pmic = {.devAddrI2c = 0x3f};

    // =============== Initialize I2C Interface ===================
    // Initialize I2C bus 2 for communication
    EXIT_ON_PLATFORM_ERROR(PlatformI2c_init(2), cleanup_platform());

    // =============== Configure GPIO Pins ===================
    // Setup control and status pins for the CTRX devices

    // RFT (Ready for Transfer) pins - indicate when CTRXs are ready for SPI communication
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_configure(GPIO_ID_RFT, GPIO_FLAG_INPUT_ENABLE | GPIO_FLAG_PULL_DOWN), cleanup_platform());

    // Reset pins
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_configure(GPIO_ID_RES_N, GPIO_FLAG_OUTPUT_DRIVE_HIGH | GPIO_FLAG_PULL_DOWN), cleanup_platform());

    // Status monitoring pins
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_configure(GPIO_ID_OK, GPIO_FLAG_INPUT_ENABLE | GPIO_FLAG_PULL_DOWN), cleanup_platform());
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_configure(GPIO_ID_SPI_BNE, GPIO_FLAG_INPUT_ENABLE | GPIO_FLAG_PULL_DOWN), cleanup_platform());

    // Read-only SPI pin
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_configure(GPIO_ID_SPI_RO, GPIO_FLAG_OUTPUT_DRIVE_LOW | GPIO_FLAG_PULL_UP), cleanup_platform());

    // DMUX1 pin for device
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_configure(GPIO_ID_DMUX1, GPIO_FLAG_OUTPUT_DRIVE_HIGH | GPIO_FLAG_PULL_DOWN), cleanup_platform());
    // DMUX2 pin for device
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_configure(GPIO_ID_DMUX2, GPIO_FLAG_INPUT_ENABLE | GPIO_FLAG_PULL_DOWN), cleanup_platform());

    // =============== Setup SPI Communication ===================
    // Configure SPI interface mapping for communication with CTRXs
    gmsl_device_config_t device_configs[] = {
        {
            .device_id = DEVICE_STANDALONE,
            .link_cmd  = GMSL_SPI_ID_IGNORE,  // Serializer accepts all IDs in single-link mode
            .ss_cmd    = GMSL_ASS_SS1         // Device on SS1
        }};

    GmslSpiWrapper_setConfig(device_configs,
                             sizeof(device_configs) / sizeof(device_configs[0]),
                             GPIO_ID_SPI_RO,
                             GPIO_ID_SPI_BNE);

    // =============== Initialize IfxRfe ===================
    // Setup function callbacks for SPI, GPIO, timing, and logging operations

    // SPI function callbacks
    IfxRfe_spiFunctions_t spiFncs = {
        .spiConfigure = GmslSpiWrapper_configure,
        .spiTransfer  = GmslSpiWrapper_transfer,
        .spiWrite     = GmslSpiWrapper_write};

    // Enable automatic SPI retransmission
    IfxRfe_setSpiRetransmissionCount(SPI_RETRANSMISSIONS);

    // GPIO function callbacks - handle pin control and status reading
    IfxRfe_gpioFunctions_t gpioFncs = {
        .gpioGet            = PlatformGpio_get,
        .gpioSet            = PlatformGpio_set,
        .gpioConfigure      = PlatformGpio_configure,
        .isGpioConfigOutput = PlatformGpio_isGpioConfigOutput};

    // Timing function callbacks - handle delays and timeouts
    IfxRfe_timeFunctions_t timeFncs = {
        .getDeadLine = PlatformTime_getDeadLine,
        .now         = PlatformTime_now,
        .waitTime    = PlatformTime_waitTime};

    // Logging function callbacks - handle debug output
    IfxRfe_logCallbacks_t logInterface = {
        .errorLog   = PlatformLogCallbacks_error,
        .warningLog = PlatformLogCallbacks_warning,
        .infoLog    = PlatformLogCallbacks_info};

    // Define GPIO pin mapping for each CTRX device
    uint16_t const GPIO_UNUSED                     = 0;
    IfxRfe_gpioDefinitions_t gpios[CTRX_DEV_COUNT] = {
        {.spiRftId = GPIO_ID_RFT,
         .okId     = GPIO_ID_OK,
         .dmux1Id  = GPIO_ID_DMUX1,
         .dmux2Id  = GPIO_ID_DMUX2,
         .dmux3Id  = GPIO_UNUSED},
    };

    // Initialize IfxRfe with all configuration parameters
    EXIT_ON_IFXRFE_ERROR(
        IfxRfe_init(
            CTRX_DEV_COUNT,
            gpios,
            spiFncs,
            gpioFncs,
            timeFncs,
            logInterface),
        cleanup_platform());

    // =============== PMIC and Configuration Setup ===================

    // enable spread spectrum +-3% pseudo-random
    // EXIT_ON_IFXRFE_ERROR(Max20434_enableSpreadSpectrum(&pmic, SSE_3Percent_PseudoRandom), cleanup_platform());

    // Initialize device configuration parameters
    IfxRfe_demoConfigParams_t configParams;
    // Device config
    IrfeDemoConfigInit(&configParams);

    // Configure SPI clock speed to SPI_CLOCK_SPEED using mode 0
    uint8_t flags = IFXRFE_SPI_MODE_0;
    Wrapper_SpiConfigure(flags, SPI_CLOCK_SPEED);
    while(1)
    {

    }

    // // =============== Check CTRX Status ===================
    // // Check the CTRX status
    // IfxRfe_getStatusResult_t status = {0};

    // // Reset the CTRX by toggling the RESET_N pin
    // // Toggle RESET_N to properly initialize both CTRX devices
    // EXIT_ON_PLATFORM_ERROR(PlatformGpio_set(GPIO_ID_RES_N, false), cleanup_platform());  // Assert reset
    // usleep(RESET_DELAY_US);
    // EXIT_ON_PLATFORM_ERROR(PlatformGpio_set(GPIO_ID_RES_N, true), cleanup_platform());  // Release reset
    // usleep(RESET_DELAY_US);

    // // Get status
    // EXIT_ON_IFXRFE_ERROR(IfxRfe_selectDevice(DEVICE_STANDALONE), cleanup_platform());
    // EXIT_ON_IFXRFE_ERROR(IfxRfe_getStatus(&status), cleanup_platform());

    // printf("Initial status - CTRX: %#010x\n", status.curr_state);

    // // =============== CTRX Initialization  ===================
    // printf("Performing initialization of the CTRX...\n");

    // // Initialize CTRX
    // static uint32_t intitializeIramDesc[] = {0x10000028, 0x1180000A, 0x1000000E};
    // static uint32_t intitializeValue[]    = {1, 2, 1};  // Use continuous CSI-2 clk mode

    // printf("# Initializing CTRX...\n");
    // IfxRfe_ctrxInitConfiguration_t config = prepareCtrxInitConfig(STANDALONE);
    // // Override default initialization config
    // config.initializeConfig = (IfxRfe_initialize_t) {
    //     .iram_descriptor = intitializeIramDesc,
    //     .value           = intitializeValue,
    //     .length          = sizeof(intitializeIramDesc) / sizeof(intitializeIramDesc[0])};
    // config.spiConfigureFlags = IFXRFE_SPI_MODE_0;
    // config.spiClockSpeed     = SPI_CLOCK_SPEED;
    // EXIT_ON_IFXRFE_ERROR(IfxRfe_ctrxInit(config), cleanup_platform());
    // EXIT_ON_IFXRFE_ERROR(IfxRfe_getStatus(&status), cleanup_platform());
    // printf("Status of CTRX after initialization: %#010x\n", status.curr_state);

    // // Check for errors during device initialization
    // IfxRfe_handleError_t errorRequest = {
    //     .action               = 1,
    //     .error_mask_1         = 0,
    //     .error_mask_1_present = 0,
    //     .length               = 0,
    //     .error_mask_2         = 0};
    // IfxRfe_handleErrorResult_t errorResult;
    // EXIT_ON_IFXRFE_ERROR(IfxRfe_handleError(errorRequest, &errorResult), cleanup_platform());
    // print_error_status("CTRX", &errorResult);

    // // =============== RF Configuration ===================
    // // Configure RF parameters and sequencer data

    // // Configure RF parameters
    // printf("Configuring CTRX RF parameters...\n");
    // EXIT_ON_IFXRFE_ERROR(IfxRfe_loadSequencerData(0, seqData_standalone, sizeof(seqData_standalone) / sizeof(uint32_t)), cleanup_platform());
    // printf("Sequencer memory configured for the CTRX\n");

    // // Configure ramp scenario, TX power, RX, and RF frequency
    // EXIT_ON_IFXRFE_ERROR(IfxRfe_configureRampScenario_exp(0), cleanup_platform());
    // EXIT_ON_IFXRFE_ERROR(IfxRfe_configureTxPower(configParams.txpwr), cleanup_platform());
    // EXIT_ON_IFXRFE_ERROR(IfxRfe_configureRx(configParams.rxcfg), cleanup_platform());
    // EXIT_ON_IFXRFE_ERROR(IfxRfe_configureRfFrequency(configParams.rfFreqCfg), cleanup_platform());

    // // =============== Transition to Operation State ===================
    // // Transition to operational state for radar operations
    // EXIT_ON_IFXRFE_ERROR(IfxRfe_gotoOperation(), cleanup_platform());
    // EXIT_ON_IFXRFE_ERROR(IfxRfe_getStatus(&status), cleanup_platform());

    // // Verify device reached operation state
    // if (status.curr_state != EXPECTED_OPERATION_STATE)
    // {
    //     printf("Error: CTRX failed to reach operation state. Current state: 0x%X\n", status.curr_state);
    //     cleanup_platform();
    //     return IFXRFE_E_FAILED;
    // }
    // printf("###########################\n");
    // printf("CTRX OPERATION STATE: 0x%X\n", status.curr_state);

    // // =============== TX Calibration ===================
    // // Execute TX calibration to ensure optimal performance

    // IfxRfe_executeCalibrationResult_t calibResult;

    // // Execute TX calibration
    // EXIT_ON_IFXRFE_ERROR(IfxRfe_executeCalibration(configParams.calibration, &calibResult), cleanup_platform());

    // // =============== Continuous Radar Operation Loop ===================
    // // Run continuous radar measurement cycles with ramp scenarios
    // printf("Starting continuous radar operation with %d iterations...\n", NUM_ITERATIONS);

    // IfxRfe_finishRampScenarioResult_t frsres;

    // for (int i = 0; i < NUM_ITERATIONS; i++)
    // {
    //     printf("Radar iteration: %d\n", i);
    //     EXIT_ON_IFXRFE_ERROR(IfxRfe_startRampScenario(), cleanup_platform());

    //     usleep(FINISH_RAMP_DELAY_US);  // Wait for ramp scenario to complete

    //     EXIT_ON_IFXRFE_ERROR(IfxRfe_finishRampScenario(&frsres), cleanup_platform());

    //     IfxRfe_handleError_t errorRequest2 = {
    //         .action               = 1,
    //         .error_mask_1         = 0,
    //         .error_mask_1_present = 0,
    //         .length               = 0,
    //         .error_mask_2         = 0};
    //     IfxRfe_handleErrorResult_t errorResult2;
    //     EXIT_ON_IFXRFE_ERROR(IfxRfe_handleError(errorRequest2, &errorResult2), cleanup_platform());
    //     print_error_status("CTRX", &errorResult2);

    //     // Transition to low power mode to save energy
    //     EXIT_ON_IFXRFE_ERROR(IfxRfe_gotoLowPower(), cleanup_platform());

    //     // Wait before next iteration
    //     usleep(LOOP_DELAY_US);

    //     // Wake up both devices for next radar cycle
    //     EXIT_ON_IFXRFE_ERROR(IfxRfe_gotoOperation(), cleanup_platform());
    // }

    // printf("Radar operation completed successfully!\n");

    // // =============== Final Cleanup ===================
    // // Put both CTRXs in low power mode and cleanup platform resources
    // printf("Performing final cleanup...\n");

    // EXIT_ON_IFXRFE_ERROR(IfxRfe_gotoLowPower(), cleanup_platform());

    // // Cleanup platform resources
    // cleanup_platform();

    printf("=== SUCCESS ===\n");
    return EXIT_SUCCESS;
}