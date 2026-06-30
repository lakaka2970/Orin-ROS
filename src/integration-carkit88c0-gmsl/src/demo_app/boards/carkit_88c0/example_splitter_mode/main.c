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

#include "../HwDefinitionsSplitterMode.h"
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
#define CTRX_DEV_COUNT           2
#define DEVICE_LINK_A            0
#define DEVICE_LINK_B            1
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
#if 0 
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
#endif
/**
 * @brief Main application entry point
 * 
 * This function demonstrates the complete initialization and operation sequence
 * for a radar system using two CARKIT88C0 boards in reverse splitter mode. The
 * application performs the following major steps:
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
    printf("=== IfxRfe App - CARKIT88C0 Reverse Splitter Mode Example ===\n");

    // const Max20434Pec_t pmic = {.devAddrI2c = 0x3f};

    // =============== Initialize I2C Interface ===================
    // Initialize I2C bus 2 for communication
    // 初始化I2C总线2（用于与PMIC通信），失败则执行cleanup_platform并退出
    EXIT_ON_PLATFORM_ERROR(PlatformI2c_init(2), cleanup_platform());

    // =============== Configure GPIO Pins ===================
    // Setup control and status pins for the CTRX devices
    // RFT（Ready for Transfer）引脚：输入+下拉，指示CTRX是否就绪SPI通信
    // RFT (Ready for Transfer) pins - indicate when CTRXs are ready for SPI communication
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_configure(GPIO_ID_RFT_A, GPIO_FLAG_INPUT_ENABLE | GPIO_FLAG_PULL_DOWN), cleanup_platform());
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_configure(GPIO_ID_RFT_B, GPIO_FLAG_INPUT_ENABLE | GPIO_FLAG_PULL_DOWN), cleanup_platform());

    // Reset pins
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_configure(GPIO_ID_RES_N_A, GPIO_FLAG_OUTPUT_DRIVE_HIGH | GPIO_FLAG_PULL_DOWN), cleanup_platform());
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_configure(GPIO_ID_RES_N_B, GPIO_FLAG_OUTPUT_DRIVE_HIGH | GPIO_FLAG_PULL_DOWN), cleanup_platform());

    // Status monitoring pins
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_configure(GPIO_ID_SPI_BNE, GPIO_FLAG_INPUT_ENABLE | GPIO_FLAG_PULL_DOWN), cleanup_platform());

    // Read-only SPI pin
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_configure(GPIO_ID_SPI_RO, GPIO_FLAG_OUTPUT_DRIVE_LOW | GPIO_FLAG_PULL_UP), cleanup_platform());

    // DMUX1 pin for device
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_configure(GPIO_ID_DMUX1_A, GPIO_FLAG_OUTPUT_DRIVE_HIGH | GPIO_FLAG_PULL_DOWN), cleanup_platform());
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_configure(GPIO_ID_DMUX1_B, GPIO_FLAG_OUTPUT_DRIVE_HIGH | GPIO_FLAG_PULL_DOWN), cleanup_platform());

    // =============== Setup SPI Communication ===================
    // Configure SPI interface mapping for communication with CTRXs
    // Configure reverse splitter mode GMSL setup
    //配置GMSL SPI映射
    gmsl_device_config_t device_configs[] = {
        {
            .device_id = DEVICE_LINK_A,
            .link_cmd  = GMSL_SPI_ID_00,  // First board uses SPI ID '00'
            .ss_cmd    = GMSL_ASS_SS1     // Both on SS1
        },
        {
            .device_id = DEVICE_LINK_B,
            .link_cmd  = GMSL_SPI_ID_01,  // Second board uses SPI ID '01'
            .ss_cmd    = GMSL_ASS_SS1     // Both on SS1
        }};
    // 设置GMSL SPI配置：设备列表、SPI_RO/SPI_BNE引脚映射
    GmslSpiWrapper_setConfig(device_configs,
                             sizeof(device_configs) / sizeof(device_configs[0]),
                             GPIO_ID_SPI_RO,
                             GPIO_ID_SPI_BNE);

    // =============== Initialize IfxRfe ===================
    // Setup function callbacks for SPI, GPIO, timing, and logging operations

    // SPI function callbacks
    // SPI回调函数：绑定GMSL SPI的配置/传输/写操作
    IfxRfe_spiFunctions_t spiFncs = {
        .spiConfigure = GmslSpiWrapper_configure,
        .spiTransfer  = GmslSpiWrapper_transfer,
        .spiWrite     = GmslSpiWrapper_write};

    // Enable automatic SPI retransmission
    // 启用SPI自动重传
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
        {.spiRftId = GPIO_ID_RFT_A,// 链路A RFT引脚
         .dmux1Id  = GPIO_ID_DMUX1_A,// 链路A DMUX1引脚
         .okId     = GPIO_UNUSED,
         .dmux2Id  = GPIO_UNUSED,
         .dmux3Id  = GPIO_UNUSED},
        {.spiRftId = GPIO_ID_RFT_B,// 链路B RFT引脚
         .dmux1Id  = GPIO_ID_DMUX1_B,
         .okId     = GPIO_UNUSED,
         .dmux2Id  = GPIO_UNUSED,
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
    // 启用PMIC的扩频功能（±3%伪随机，降低EMI电磁干扰）
    // enable spread spectrum +-3% pseudo-random
    // EXIT_ON_IFXRFE_ERROR(Max20434_enableSpreadSpectrum(&pmic, SSE_3Percent_PseudoRandom), cleanup_platform());
    // 初始化雷达演示配置参数（默认参数：TX功率、RX配置、RF频率等）
    // Initialize device configuration parameters
    IfxRfe_demoConfigParams_t configParams;
    // Device config
    IrfeDemoConfigInit(&configParams);
    // 配置SPI时钟：模式0（CPOL=0, CPHA=0），10MHz
    // Configure SPI clock speed to SPI_CLOCK_SPEED using mode 0
    uint8_t flags = IFXRFE_SPI_MODE_0;
    Wrapper_SpiConfigure(flags, SPI_CLOCK_SPEED);

#if 0
    // =============== Check CTRX Status ===================
    // Check the CTRX status
    IfxRfe_getStatusResult_t statusLinkA = {0};
    IfxRfe_getStatusResult_t statusLinkB = {0};

    // Reset the CTRX by toggling the RESET_N pin
    // Toggle RESET_N to properly initialize both CTRX devices
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_set(GPIO_ID_RES_N_A, false), cleanup_platform());  // Assert reset
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_set(GPIO_ID_RES_N_B, false), cleanup_platform());  // Assert reset
    usleep(RESET_DELAY_US);
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_set(GPIO_ID_RES_N_A, true), cleanup_platform());  // Release reset
    EXIT_ON_PLATFORM_ERROR(PlatformGpio_set(GPIO_ID_RES_N_B, true), cleanup_platform());  // Release reset
    usleep(RESET_DELAY_US);

    // Get status for device on Link A
    EXIT_ON_IFXRFE_ERROR(IfxRfe_selectDevice(DEVICE_LINK_A), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_getStatus(&statusLinkA), cleanup_platform());
    printf("Initial status - CTRX LINK A: %#010x\n", statusLinkA.curr_state);

    // Get status for device on Link B
    EXIT_ON_IFXRFE_ERROR(IfxRfe_selectDevice(DEVICE_LINK_B), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_getStatus(&statusLinkB), cleanup_platform());
    printf("Initial status - CTRX LINK B: %#010x\n", statusLinkB.curr_state);

    // =============== CTRX Initialization  ===================
    printf("Performing initialization of the CTRX...\n");

    // Initialize CTRX
    // 定义CTRX初始化配置（独立模式、IRAM描述符、SPI参数）
    static uint32_t intitializeIramDesc[] = {0x10000028, 0x1180000A, 0x1000000E};
    static uint32_t intitializeValue[]    = {1, 2, 1};  // Use continuous CSI-2 clk mode

    printf("# Initializing CTRX on LINK A...\n");
    IfxRfe_ctrxInitConfiguration_t config = prepareCtrxInitConfig(STANDALONE);
    // Override default initialization config
    config.initializeConfig = (IfxRfe_initialize_t) {
        .iram_descriptor = intitializeIramDesc,
        .value           = intitializeValue,
        .length          = sizeof(intitializeIramDesc) / sizeof(intitializeIramDesc[0])};
    config.spiConfigureFlags = IFXRFE_SPI_MODE_0;
    config.spiClockSpeed     = SPI_CLOCK_SPEED;

    // Initialize CTRX LINK A
    EXIT_ON_IFXRFE_ERROR(IfxRfe_selectDevice(DEVICE_LINK_A), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_ctrxInit(config), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_getStatus(&statusLinkA), cleanup_platform());
    printf("Status of CTRX LINK A after initialization: %#010x\n", statusLinkA.curr_state);

    // Check for errors during device initialization
    IfxRfe_handleError_t errorRequest = {
        .action               = 1,
        .error_mask_1         = 0,
        .error_mask_1_present = 0,
        .length               = 0,
        .error_mask_2         = 0};
    IfxRfe_handleErrorResult_t errorResult;
    EXIT_ON_IFXRFE_ERROR(IfxRfe_handleError(errorRequest, &errorResult), cleanup_platform());
    print_error_status("CTRX (LINK A)", &errorResult);

    printf("# Initializing CTRX on LINK B...\n");
    // Initialize CTRX LINK B
    EXIT_ON_IFXRFE_ERROR(IfxRfe_selectDevice(DEVICE_LINK_B), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_ctrxInit(config), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_getStatus(&statusLinkB), cleanup_platform());
    printf("Status of CTRX LINK B after initialization: %#010x\n", statusLinkB.curr_state);

    EXIT_ON_IFXRFE_ERROR(IfxRfe_handleError(errorRequest, &errorResult), cleanup_platform());
    print_error_status("CTRX (LINK B)", &errorResult);


    // =============== RF Configuration ===================
    // Configure RF parameters and sequencer data
    // 配置雷达射频参数（序列器、Ramp、TX功率、RX、RF频率）
    // Configure RF parameters for CTRX on LINK A
    printf("Configuring CTRX (LINK A) RF parameters...\n");
    EXIT_ON_IFXRFE_ERROR(IfxRfe_selectDevice(DEVICE_LINK_A), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_loadSequencerData(0, seqData_standalone, sizeof(seqData_standalone) / sizeof(uint32_t)), cleanup_platform());
    printf("Sequencer memory configured for the CTRX (LINK A)\n");

    // Configure ramp scenario, TX power, RX, and RF frequency for CTRX on LINK A
    EXIT_ON_IFXRFE_ERROR(IfxRfe_configureRampScenario_exp(0), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_configureTxPower(configParams.txpwr), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_configureRx(configParams.rxcfg), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_configureRfFrequency(configParams.rfFreqCfg), cleanup_platform());

    // Configure RF parameters for CTRX on LINK B
    printf("Configuring CTRX (LINK B) RF parameters...\n");
    EXIT_ON_IFXRFE_ERROR(IfxRfe_selectDevice(DEVICE_LINK_B), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_loadSequencerData(0, seqData_standalone, sizeof(seqData_standalone) / sizeof(uint32_t)), cleanup_platform());
    printf("Sequencer memory configured for the CTRX (LINK B)\n");

    // Configure ramp scenario, TX power, RX, and RF frequency for CTRX on LINK B
    EXIT_ON_IFXRFE_ERROR(IfxRfe_configureRampScenario_exp(0), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_configureTxPower(configParams.txpwr), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_configureRx(configParams.rxcfg), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_configureRfFrequency(configParams.rfFreqCfg), cleanup_platform());

    // =============== Transition to Operation State ===================
    // Transition both devices to operational state for radar operations
    // 将CTRX从初始化状态切换到运行状态（可执行雷达扫描）
    
    // 链路A切换到运行状态
    EXIT_ON_IFXRFE_ERROR(IfxRfe_selectDevice(DEVICE_LINK_A), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_gotoOperation(), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_getStatus(&statusLinkA), cleanup_platform());

    // Verify device reached operation state
    if (statusLinkA.curr_state != EXPECTED_OPERATION_STATE)
    {
        printf("Error: CTRX (LINK A) failed to reach operation state. Current state: 0x%X\n", statusLinkA.curr_state);
        cleanup_platform();
        return IFXRFE_E_FAILED;
    }
    printf("###########################\n");
    printf("CTRX (LINK A) OPERATION STATE: 0x%X\n", statusLinkA.curr_state);

    EXIT_ON_IFXRFE_ERROR(IfxRfe_selectDevice(DEVICE_LINK_B), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_gotoOperation(), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_getStatus(&statusLinkB), cleanup_platform());

    // Verify device reached operation state
    if (statusLinkB.curr_state != EXPECTED_OPERATION_STATE)
    {
        printf("Error: CTRX (LINK B) failed to reach operation state. Current state: 0x%X\n", statusLinkB.curr_state);
        cleanup_platform();
        return IFXRFE_E_FAILED;
    }
    printf("###########################\n");
    printf("CTRX (LINK B) OPERATION STATE: 0x%X\n", statusLinkB.curr_state);

    // =============== TX Calibration ===================
    // Execute TX calibration to ensure optimal performance

    IfxRfe_executeCalibrationResult_t calibResult;

    // Execute TX calibration for CTRX on LINK A
    EXIT_ON_IFXRFE_ERROR(IfxRfe_selectDevice(DEVICE_LINK_A), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_executeCalibration(configParams.calibration, &calibResult), cleanup_platform());
    // Execute TX calibration for CTRX on LINK B
    EXIT_ON_IFXRFE_ERROR(IfxRfe_selectDevice(DEVICE_LINK_B), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_executeCalibration(configParams.calibration, &calibResult), cleanup_platform());

    // =============== Continuous Radar Operation Loop ===================
    // Run continuous radar measurement cycles with ramp scenarios
    printf("Starting continuous radar operation with %d iterations...\n", NUM_ITERATIONS);

    IfxRfe_finishRampScenarioResult_t frsres;

    for (int i = 0; i < NUM_ITERATIONS; i++)
    {
        printf("Radar iteration: %d\n", i);
        EXIT_ON_IFXRFE_ERROR(IfxRfe_startRampScenario(), cleanup_platform());  // CTRX LINK B (already selected)

        EXIT_ON_IFXRFE_ERROR(IfxRfe_selectDevice(DEVICE_LINK_A), cleanup_platform());
        EXIT_ON_IFXRFE_ERROR(IfxRfe_startRampScenario(), cleanup_platform());

        usleep(FINISH_RAMP_DELAY_US);  // Wait for ramp scenario to complete

        EXIT_ON_IFXRFE_ERROR(IfxRfe_finishRampScenario(&frsres), cleanup_platform());

        EXIT_ON_IFXRFE_ERROR(IfxRfe_handleError(errorRequest, &errorResult), cleanup_platform());
        print_error_status("CTRX (LINK A)", &errorResult);

        // Transition to low power mode to save energy
        EXIT_ON_IFXRFE_ERROR(IfxRfe_gotoLowPower(), cleanup_platform());

        EXIT_ON_IFXRFE_ERROR(IfxRfe_selectDevice(DEVICE_LINK_B), cleanup_platform());
        EXIT_ON_IFXRFE_ERROR(IfxRfe_finishRampScenario(&frsres), cleanup_platform());

        EXIT_ON_IFXRFE_ERROR(IfxRfe_handleError(errorRequest, &errorResult), cleanup_platform());
        print_error_status("CTRX (LINK B)", &errorResult);

        // Transition to low power mode to save energy
        EXIT_ON_IFXRFE_ERROR(IfxRfe_gotoLowPower(), cleanup_platform());

        // Wait before next iteration
        usleep(LOOP_DELAY_US);

        // Wake up both devices for next radar cycle
        EXIT_ON_IFXRFE_ERROR(IfxRfe_selectDevice(DEVICE_LINK_A), cleanup_platform());
        EXIT_ON_IFXRFE_ERROR(IfxRfe_gotoOperation(), cleanup_platform());

        EXIT_ON_IFXRFE_ERROR(IfxRfe_selectDevice(DEVICE_LINK_B), cleanup_platform());
        EXIT_ON_IFXRFE_ERROR(IfxRfe_gotoOperation(), cleanup_platform());
    }

    printf("Radar operation completed successfully!\n");

    // =============== Final Cleanup ===================
    // Put both CTRXs in low power mode and cleanup platform resources
    printf("Performing final cleanup...\n");

    EXIT_ON_IFXRFE_ERROR(IfxRfe_gotoLowPower(), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_selectDevice(DEVICE_LINK_A), cleanup_platform());
    EXIT_ON_IFXRFE_ERROR(IfxRfe_gotoLowPower(), cleanup_platform());

    // Cleanup platform resources
    cleanup_platform();
#endif    

    printf("=== SUCCESS ===\n");
    return EXIT_SUCCESS;
}