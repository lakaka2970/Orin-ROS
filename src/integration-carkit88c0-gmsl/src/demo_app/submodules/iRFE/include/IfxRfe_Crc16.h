#ifndef IFXRFE_CRC16_H
/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#define IFXRFE_CRC16_H 1

#include <stdint.h>

#define CRC16_CCITT_FALSE_SEED (0xFFFF)

/// \brief Calculate 16-bit CRC using the CRC-CCITT polynomial
///
/// - polynomial: 0x1021 (x^16 + x^12 + x^5 + 1)
/// - seed: 0xFFFF (CRC16_CCITT_FALSE_SEED)
/// - check value: 0x0000
///
/// \param buf       data from which the checksum is calculated
/// \param len       Number of @p data bytes
/// \param crc       initial value for calculation
/// \return 16-bit checksum of @p data
uint16_t crc16_ccitt_false(const uint8_t buf[], uint16_t len, uint16_t crc);

/// \brief Calculate CRC16 for the given header word
/// \param header The header word, the resulting CRC is calculated over the 2 most significant bytes of the header (opcode, CRI and header parameter)
/// \return Calculated CRC16 value
uint16_t calculateCRC16(uint32_t header);

#endif  //IFXRFE_CRC16_H
