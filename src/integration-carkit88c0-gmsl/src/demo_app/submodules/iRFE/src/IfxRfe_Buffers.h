/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#ifndef IFXRFE_BUFFERS_H
#define IFXRFE_BUFFERS_H 1

#include <stdint.h>

#define MAX_REQ_PAYLOAD_WORDS 33  // Ref: CTRX Manual Section:2.2.1.1 Message from MCU to MMIC
#define MAX_RES_PAYLOAD_WORDS 33  // Ref: CTRX Manual Section:2.2.1.2 Message from MMIC to MCU

/// \brief Generic command structure to hold the data to be sent
typedef struct
{
    uint32_t buffer[MAX_REQ_PAYLOAD_WORDS + 2];  //maximum spi message size(MCU to MMIC) = Header + payload + CRC
    uint8_t payloadLen;                          //used payload length
} IfxRfe_ctrxRequest_t;

/// \brief Generic response structure to hold the received data
typedef struct
{
    uint32_t buffer[MAX_RES_PAYLOAD_WORDS + 2];  //maximum spi message size(MMIC to MCU) = Header + payload + CRC
    uint8_t totalLen;
} IfxRfe_ctrxResponse_t;

#endif  //IFXRFE_BUFFERS_H
