/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#ifndef IFXRFE_CRC8_H
#define IFXRFE_CRC8_H 1

#include <stdint.h>

/**
 * Calculate 8-bit CRC using Sm Bus polynomial
 *
 * - polynomial: 0x107 (x^8 + x^2 + x + 1)
 * - init: 0x0
 *
 * \param buf       data from which the checksum is calculated
 * \param len       Number of @p data bytes
 * \return 8-bit checksum of @p data
 */
uint8_t IfxRfe_crc8SmBus(const uint8_t *buf, uint16_t len);

#endif  //IFXRFE_CRC16_H