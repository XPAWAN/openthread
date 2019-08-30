/*
 *  Copyright (c) 2019, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements the OpenThread platform abstraction for libUARTE communication.
 *
 */

#include <openthread-core-config.h>
#include <openthread/config.h>

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include <utils/code_utils.h>
#include <openthread/platform/toolchain.h>
#include <openthread/platform/uart.h>

#include "openthread-system.h"

#include "nrf_libuarte_async.h"
#include "platform-nrf5.h"
#include <hal/nrf_gpio.h>
#include <hal/nrf_uarte.h>

#if (UARTE_AS_SERIAL_TRANSPORT == 1)

bool sUartEnabled = false;

NRF_LIBUARTE_ASYNC_DEFINE(sLibUarte,
                          NRF_LIBUARTE_UART_INTERFACE_NUM,
                          NRF_LIBUARTE_COUNTER_INTERFACE_NUM,
                          NRF_LIBUARTE_PERIPHERAL_NOT_USED,
                          NRF_LIBUARTE_TIMER_INTERFACE_NUM,
                          NRF_LIBUARTE_RX_BUFFER_SIZE,
                          NRF_LIBUARTE_RX_BUFFER_NUM);

/**
 * @brief UARTE TX variables.
 */
static bool sTransmitStarted = false;
static bool sTransmitDone    = false;

/**
 * @brief UARTE RX buffer variables.
 */
typedef struct
{
    volatile uint8_t *mReadPos;
    volatile uint8_t *mWritePos;
} UarteRxBuffer;

static UarteRxBuffer sRxBuffers[NRF_LIBUARTE_RX_BUFFER_NUM];
static volatile bool sRxEventPending;

/**
 * @brief Find the RX buffer index.
 *
 * This function looks for the RX Buffer with the requested write pointer.
 *
 * @note If @p aBuffer is equal to NULL, the function returns free RX buffer index
 *       i.e. either where write pointer is set to NULL or equal to receive pointer.
 *
 * @param[out] aIndex   Pointer to the index of RX buffer.
 * @param[in]  aBuffer  The RX Buffer address. NULL in case the free buffer is requested.
 *
 * @retval  OT_ERROR_NONE       RX buffer has been found.
 * @retval  OT_ERROR_NOT_FOUND  RX buffer has not been found.
 */
static otError findRxBufferIndex(uint32_t *aIndex, const uint8_t *aBuffer)
{
    otError  error = OT_ERROR_NOT_FOUND;
    uint32_t index;

    for (index = 0; index < NRF_LIBUARTE_RX_BUFFER_NUM; index++)
    {
        if (sRxBuffers[index].mWritePos == aBuffer ||
            (aBuffer == NULL && sRxBuffers[index].mWritePos == sRxBuffers[index].mReadPos))
        {
            error   = OT_ERROR_NONE;
            *aIndex = index;
            break;
        }
    }

    return error;
}

/**
 * @brief Notify application about new bytes received.
 */
static void processReceive(void)
{
    uint32_t index;
    int16_t  length;
    uint8_t *readPos = NULL;

    do
    {
        sRxEventPending = false;

        for (index = 0; index < NRF_LIBUARTE_RX_BUFFER_NUM; index++)
        {
            CRITICAL_REGION_ENTER();
            length = sRxBuffers[index].mWritePos - sRxBuffers[index].mReadPos;
            CRITICAL_REGION_EXIT();

            // The difference should never be negative.
            assert(length >= 0);

            if (length > 0)
            {
                readPos = (uint8_t *)sRxBuffers[index].mReadPos;
                otPlatUartReceived(readPos, length);

                // Increment read pointer first and free the memory for the next reception.
                sRxBuffers[index].mReadPos += length;
                nrf_libuarte_async_rx_free(&sLibUarte, readPos, length);
            }
        }
    } while (sRxEventPending);
}

/**
 * @brief Notify application about transmission being done.
 *
 */
static void processTransmit(void)
{
    otEXPECT(sTransmitDone == true);

    // This should not happen if the transmission was not started.
    assert(sTransmitStarted);

    // Clear Transmition transaction and notify application.
    sTransmitDone    = false;
    sTransmitStarted = false;

    otPlatUartSendDone();

exit:
    return;
}

/**
 * @brief Interrupt event handler.
 *
 * @param[in] aContext Arbitrary data.
 * @param[in] aEvt     Pointer to event structure. Event is allocated on the stack so it is available
 *                     only within the context of the event handler.
 */
static void uarteEventHandler(void *aContext, nrf_libuarte_async_evt_t *aEvt)
{
    OT_UNUSED_VARIABLE(aContext);

    uint32_t index = 0;
    otError  error = OT_ERROR_NONE;

    switch (aEvt->type)
    {
    case NRF_LIBUARTE_ASYNC_EVT_ERROR:
        break;

    case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:
        sRxEventPending = true;

        error = findRxBufferIndex(&index, aEvt->data.rxtx.p_data);
        if (error == OT_ERROR_NOT_FOUND)
        {
            error = findRxBufferIndex(&index, NULL);
            assert(error == OT_ERROR_NONE);

            sRxBuffers[index].mWritePos = aEvt->data.rxtx.p_data;
            sRxBuffers[index].mReadPos  = aEvt->data.rxtx.p_data;
        }

        sRxBuffers[index].mWritePos += aEvt->data.rxtx.length;
        break;

    case NRF_LIBUARTE_ASYNC_EVT_TX_DONE:
        sTransmitDone = true;
        break;

    default:
        break;
    }
}

void nrf5UartProcess(void)
{
    processReceive();
    processTransmit();
}

void nrf5UartInit(void)
{
    // Intentionally empty.
}

void nrf5UartClearPendingData(void)
{
    // Intentionally empty.
}

void nrf5UartDeinit(void)
{
    if (sUartEnabled)
    {
        otPlatUartDisable();
    }
}

otError otPlatUartEnable(void)
{
    otError                     error = OT_ERROR_NONE;
    ret_code_t                  ret;
    nrf_libuarte_async_config_t config = {
        .tx_pin   = UART_PIN_TX,
        .rx_pin   = UART_PIN_RX,
        .cts_pin  = UART_PIN_CTS,
        .rts_pin  = UART_PIN_RTS,
        .baudrate = UART_BAUDRATE,
        .parity   = NRF_UARTE_PARITY_EXCLUDED,
#if (UART_HWFC_ENABLED == 1)
        .hwfc = NRF_UARTE_HWFC_ENABLED,
#else
        .hwfc = NRF_UARTE_HWFC_DISABLED,
#endif
        .timeout_us = UARTE_TIMEOUT,
        .int_prio   = UART_IRQ_PRIORITY
    };

    otEXPECT_ACTION(sUartEnabled == false, error = OT_ERROR_ALREADY);

    ret = nrf_libuarte_async_init(&sLibUarte, &config, uarteEventHandler, NULL);
    assert(ret == NRF_SUCCESS);

    nrf_libuarte_async_enable(&sLibUarte);

    sUartEnabled = true;

exit:
    return error;
}

otError otPlatUartDisable(void)
{
    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION(sUartEnabled == true, error = OT_ERROR_ALREADY);

    nrf_libuarte_async_uninit(&sLibUarte);

    sUartEnabled     = false;
    sTransmitStarted = false;
    sTransmitDone    = false;
    sRxEventPending  = false;
    memset(&sRxBuffers, 0, sizeof(sRxBuffers));

exit:
    return error;
}

otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    otError    error = OT_ERROR_NONE;
    ret_code_t ret;

    otEXPECT_ACTION(sTransmitStarted == false, error = OT_ERROR_BUSY);

    ret = nrf_libuarte_async_tx(&sLibUarte, (uint8_t *)aBuf, aBufLength);
    otEXPECT_ACTION(ret == NRF_SUCCESS, error = OT_ERROR_BUSY);

    sTransmitStarted = true;
    sTransmitDone    = false;

exit:
    assert(error == OT_ERROR_NONE);
    return error;
}

otError otPlatUartFlush(void)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

#endif // UARTE_AS_SERIAL_TRANSPORT == 1

/**
 * The UART driver weak functions definition.
 *
 */
OT_TOOL_WEAK void otPlatUartSendDone(void)
{
}

OT_TOOL_WEAK void otPlatUartReceived(const uint8_t *aBuf, uint16_t aBufLength)
{
    OT_UNUSED_VARIABLE(aBuf);
    OT_UNUSED_VARIABLE(aBufLength);
}
