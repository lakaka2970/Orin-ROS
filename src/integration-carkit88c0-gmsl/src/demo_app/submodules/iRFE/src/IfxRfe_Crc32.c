/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#include "IfxRfe_Crc32.h"
#include <stdbool.h>


uint32_t crc32_bzip2_big_endian(const uint8_t buf[], uint16_t len)
{
    return crc32_big_endian_invert_out(buf, len, IFXRFE_BZIP2_POLYNOMIAL, IFXRFE_CRC32_DEFAULT_SEED);
}

uint32_t crc32_big_endian_invert_out(const uint8_t buf[], uint16_t len, uint32_t polynomial, uint32_t crc)
{
    uint8_t byteIndex = 3;
    uint8_t wordIndex = 0;
    while (len--)
    {
        uint8_t val = buf[wordIndex * 4 + byteIndex];

        //in-place reordering the bytes of a 32-bit word to avoid extra buffer allocation and copy
        if (0 == byteIndex)
        {
            wordIndex++;
            byteIndex = 3;
        }
        else
        {
            byteIndex--;
        }

        //normal crc32 algorithm from strata
        for (unsigned int b = 0x80; b >= 0x01; b >>= 1)
        {
            const bool xor_flag = (bool)(crc & 0x80000000);
            crc <<= 1;

            if ((bool)(val & b) != xor_flag)  // bool != is the same as bool xor
            {
                crc ^= polynomial;
            }
        }
    }

    //with output inversion
    return ~crc;  // XOR with all ones
}

uint32_t calculateCRC32(const uint32_t *buffer, uint8_t len)
{
    return crc32_bzip2_big_endian((uint8_t *)buffer, len * sizeof(uint32_t));
}
