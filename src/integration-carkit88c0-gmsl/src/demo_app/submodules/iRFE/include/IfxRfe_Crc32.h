#ifndef IFXRFE_CRC32_H
#define IFXRFE_CRC32_H 1

/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#include <stdint.h>

#define IFXRFE_CRC32_DEFAULT_SEED 0xFFFFFFFF
#define IFXRFE_BZIP2_POLYNOMIAL   0x04C11DB7

/// \brief Simplified CRC32 calculation for big endian systems and output inversion
/// \param buf The input data for the calculation
/// \param len Number of bytes in the input data
/// \param polynomial Generator polynomial to be used
/// \param crc Initial crc value to start with
/// \return Calculated CRC32 value
uint32_t crc32_big_endian_invert_out(const uint8_t buf[], uint16_t len, uint32_t polynomial, uint32_t crc);

/// \brief Convenience crc32 function with polynomials and start crc value fixed to BZIP2 values
/// \param buf The input data for the calculation
/// \param len Number of bytes in the input data
/// \return Calculated CRC32 value
uint32_t crc32_bzip2_big_endian(const uint8_t buf[], uint16_t len);

/// \brief Calculate CRC32 for the given data words
/// \param buf The input data for the calculation
/// \param len Number of words in the input data
/// \return Calculated CRC32 value
uint32_t calculateCRC32(const uint32_t *buffer, uint8_t len);

#endif  //IFXRFE_CRC32_H
