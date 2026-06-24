#ifndef HW_DEFINITIONS_CARKIT88C0_GMSL_SPLITTER_H
#define HW_DEFINITIONS_CARKIT88C0_GMSL_SPLITTER_H

#include <stdint.h>

#define GPIO_CHIP_0_ID 0
#define GPIO_CHIP_1_ID 1

// gpiochip 0 registered GPIOs 348 - 511
// gpiochip 1 registered GPIOs 316 - 347
// to get the offset for gpioXXX simply calculate XXX - lower base number
// e.g. gpio444 is on gpiochip0 so calculate 444 - 348 = 96

// Splitter Mode: Two serializers connected (Link A and Link B)
// Each serializer needs its own GPIO mappings

// Link A (First Serializer) GPIO definitions
#define GPIO_OFFSET_RES_N_A 93  // PP.01 (gpio441)
#define GPIO_OFFSET_RFT_A   10  // PBB.02 (gpio236)
#define GPIO_OFFSET_DMUX1_A 4   // PAA.04 (gpio320)

// Link B (Second Serializer) GPIO definitions
#define GPIO_OFFSET_RES_N_B 141  // PAC.03 (gpio489)
#define GPIO_OFFSET_RFT_B   142  // PAC.04 (gpio490)
#define GPIO_OFFSET_DMUX1_B 139  // PAC.01 (gpio487)

// Common SPI GPIOs (shared between both serializers)
#define GPIO_OFFSET_SPI_RO  15   // PCC.03 (gpio331)
#define GPIO_OFFSET_SPI_BNE 140  // PAC.02 (gpio488)

// Link A GPIO IDs
#define GPIO_ID_RES_N_A (((GPIO_CHIP_0_ID & 0xFF) << 8) | (GPIO_OFFSET_RES_N_A & 0xFF))
#define GPIO_ID_RFT_A   (((GPIO_CHIP_1_ID & 0xFF) << 8) | (GPIO_OFFSET_RFT_A & 0xFF))
#define GPIO_ID_DMUX1_A (((GPIO_CHIP_1_ID & 0xFF) << 8) | (GPIO_OFFSET_DMUX1_A & 0xFF))

// Link B GPIO IDs
#define GPIO_ID_RES_N_B (((GPIO_CHIP_0_ID & 0xFF) << 8) | (GPIO_OFFSET_RES_N_B & 0xFF))
#define GPIO_ID_RFT_B   (((GPIO_CHIP_0_ID & 0xFF) << 8) | (GPIO_OFFSET_RFT_B & 0xFF))
#define GPIO_ID_DMUX1_B (((GPIO_CHIP_0_ID & 0xFF) << 8) | (GPIO_OFFSET_DMUX1_B & 0xFF))

// Common SPI GPIO IDs
#define GPIO_ID_SPI_RO  (((GPIO_CHIP_1_ID & 0xFF) << 8) | (GPIO_OFFSET_SPI_RO & 0xFF))
#define GPIO_ID_SPI_BNE (((GPIO_CHIP_0_ID & 0xFF) << 8) | (GPIO_OFFSET_SPI_BNE & 0xFF))

#endif  // HW_DEFINITIONS_CARKIT88C0_GMSL_SPLITTER_H