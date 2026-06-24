/*
 * (c) (2022-2023), Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * Use of this file is subject to the terms of use agreed between (i) you or
 * the company in which ordinary course of business you are acting and (ii)
 * Infineon Technologies AG or its licensees.
 */
#include "IfxRfe_Spi.h"

#include "IfxRfe_BuilderParser.h"
#include "IfxRfe_CtrxOpCodes.h"
#include "IfxRfe_Gpio.h"
#include "IfxRfe_Logger.h"
#include "IfxRfe_SpiConfig.h"
#include "IfxRfe_SpiWrapper.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/******************************************************************************/
/*-------------------------MACROS---------------------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-------------------------Local Variables------------------------------------*/
/******************************************************************************/

/******************************************************************************/
/******************************************************************************/
/*-------------------------Local Function Prototypes--------------------------*/
/******************************************************************************/

static error_t SpiTransferOnce(uint32_t *wbuf, uint8_t wlen, uint32_t *rbuf, uint8_t *rlen);
static uint8_t *expectedResponseCri();
static void setExpectedResponseCri(const uint8_t cri);

/******************************************************************************/
/*-------------------------Global Function Implementations--------------------*/
/******************************************************************************/

error_t SpiTransfer(uint32_t *wbuf, uint8_t wlen, uint32_t *rbuf, uint8_t *rlen)
{
    error_t ret         = IFXRFE_E_SUCCESS;
    bool writeSucceeded = false;

    for (uint8_t attempt = 0; attempt <= IfxRfe_spiRetransmissionCount(); attempt++)
    {
        ret = SpiTransferOnce(writeSucceeded ? NULL : wbuf, wlen, rbuf, rlen);
        if (ret == IFXRFE_E_CRC32)
        {
            writeSucceeded = true;
            infoLogFmt("CRC32 error during transmission attempt (%d/%d)\n", attempt + 1, IfxRfe_spiRetransmissionCount() + 1);
        }
        else if (ret == IFXRFE_E_CRI_MISMATCH && wbuf != NULL)
        {
            infoLogFmt("CRI mismatch during transmission attempt (%d/%d)\n", attempt + 1, IfxRfe_spiRetransmissionCount() + 1);
        }
        else
            break;
    }

    return ret;
}

error_t SpiSend(uint32_t *wbuf, uint8_t wlen)
{
    if ((NULL == wbuf) || (0 == wlen))
    {
        return IFXRFE_E_MISSING_PARAMETER;
    }

    return SpiTransfer(wbuf, wlen, NULL, NULL);
}

error_t SpiReceive(uint32_t *rbuf, uint8_t *rlen)
{
    if ((NULL == rbuf) || (NULL == rlen))
    {
        return IFXRFE_E_MISSING_PARAMETER;
    }

    return SpiTransfer(NULL, 0, rbuf, rlen);
}

error_t SpiSendReceive(uint32_t *wbuf, uint8_t wlen, uint32_t *rbuf, uint8_t *rlen)
{
    if ((NULL == wbuf) || (0 == wlen) || (NULL == rbuf) || (NULL == rlen))
    {
        return IFXRFE_E_MISSING_PARAMETER;
    }

    return SpiTransfer(wbuf, wlen, rbuf, rlen);
}

/******************************************************************************/
/*-------------------------Local Function Implementations---------------------*/
/******************************************************************************/

static error_t ResetKeepSel()
{
    return Wrapper_SpiWrite(0, NULL, false);
}

static error_t SpiReadResponseHeader(uint32_t *header)
{
    IfxRfe_ctrxRequest_t nop;
    memset(&nop, 0, sizeof(IfxRfe_ctrxRequest_t));
    IFXRFE_RETURN_ON_ERROR(FillHeaderAndCrc(IFXRFE_OPCODE_NOP, 0, &nop));

    error_t ret;
    for (uint8_t attempt = 0; attempt <= IfxRfe_spiRetransmissionCount(); attempt++)
    {
        IFXRFE_RETURN_ON_ERROR(IfxRfe_waitForRftPin(IFXRFE_RFT_TIMEOUT_DEF));
        IFXRFE_RETURN_ON_ERROR(Wrapper_SpiTransfer(1, nop.buffer, header, true));

        // check response header crc
        ret = CheckResponseHeaderCrc(*header);
        if (ret == IFXRFE_E_CRC16)
        {
            infoLogFmt("CRC16 error during transmission attempt (%d/%d)\n", attempt + 1, IfxRfe_spiRetransmissionCount() + 1);
            ResetKeepSel();
        }
        else
            break;
    }

    return ret;
}

static error_t SpiTransferOnce(uint32_t *wbuf, uint8_t wlen, uint32_t *rbuf, uint8_t *rlen)
{
    if ((wbuf == NULL && rbuf == NULL) || (wbuf && wlen == 0) || (rbuf && rlen == NULL))
    {
        return IFXRFE_E_MISSING_PARAMETER;
    }

    if (wbuf)
    {
        IFXRFE_RETURN_ON_ERROR(IfxRfe_waitForRftPin(IFXRFE_RFT_TIMEOUT_DEF));
        IFXRFE_CLEAN_RETURN_ON_ERROR(Wrapper_SpiTransfer(wlen, wbuf, NULL, false), ResetKeepSel());

        uint8_t opCode = GET_OPCODE(wbuf[0]);
        if (opCode != IFXRFE_OPCODE_NOP)
        {
            uint8_t responseCri;
            IFXRFE_RETURN_ON_ERROR(ExpectedResponseCri(wbuf, wlen, &responseCri));
            setExpectedResponseCri(responseCri);
        }
    }

    if (rbuf)
    {
        uint32_t *responseHeader = &rbuf[0];
        IFXRFE_CLEAN_RETURN_ON_ERROR(SpiReadResponseHeader(responseHeader), ResetKeepSel());

        // Check the CRI of the response against the expected value for this device.
        const uint8_t *expectedCri = expectedResponseCri();
        if (expectedCri != NULL)
        {
            if (GET_CRI(*responseHeader) != *expectedCri)
            {
                ResetKeepSel();
                return IFXRFE_E_CRI_MISMATCH;
            }
        }

        const uint8_t payloadLen = GET_PAYLOAD_LEN(*responseHeader);
        if (payloadLen > MAX_RES_PAYLOAD_WORDS)
        {
            ResetKeepSel();
            return IFXRFE_E_UNEXPECTED_VALUE;
        }

        uint8_t remainingLen = (payloadLen == 0) ? 0 : payloadLen + 1;  // +1 for CRC32
        *rlen                = remainingLen + 1;                        // +1 for header

        // Read the remaining payload (if any)
        if (remainingLen > 0)
        {
            static uint32_t zeroBuf[MAX_REQ_PAYLOAD_WORDS + 1] = {0};
            IFXRFE_CLEAN_RETURN_ON_ERROR(Wrapper_SpiTransfer(remainingLen, zeroBuf, &rbuf[1], false), ResetKeepSel());
            IFXRFE_RETURN_ON_ERROR(CheckResponsePayloadCrc(rbuf, *rlen));
        }
        else
        {
            // header only response
            ResetKeepSel();
        }
    }

    return IFXRFE_E_SUCCESS;
}

/******************************************************************************/
/*-------------------------Local Function Implementations---------------------*/
/******************************************************************************/

static int16_t _expectedResponseCri[MAX_SUPPORTED_CTRX_DEVICES] = {-1};  // Store expected CRI for multiple CTRXs, initialized to -1 (unknown)

static uint8_t *expectedResponseCri()
{
    int16_t *responseCri = &_expectedResponseCri[GetSpiId()];
    return *responseCri == -1 ? NULL : (uint8_t *)responseCri;
}

static void setExpectedResponseCri(const uint8_t cri)
{
    _expectedResponseCri[GetSpiId()] = cri;
}
