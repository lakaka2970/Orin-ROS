/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#include "IfxRfe_Crc8.h"

uint8_t IfxRfe_crc8SmBus(const uint8_t *buf, uint16_t len)
{
    const uint8_t polynomial = (uint8_t)0x107;
    uint8_t crc              = 0;
    uint8_t i, j;
    for (i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (j = 0; j < 8; j++)
        {
            crc = (crc & 0x80) ? (crc << 1) ^ polynomial : (crc << 1);
        }
    }
    return crc;
}