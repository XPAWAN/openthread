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

typedef enum
{
    BUFFER_STATUS_EMPTY = 0,
    BUFFER_STATUS_RECEIVING,
    BUFFER_STATUS_FILLED
} bufferReceiverState;

/**
 * @brief UARTE RX buffer variables.
 */
typedef struct
{
    bufferReceiverState mBufferReceiverState;
    bool                mBufferProcessing;
    uint8_t *           mBuffPtr;
    volatile uint8_t *  mReadPos;
    volatile uint8_t *  mWritePos;

} UarteRxBuffer;

static UarteRxBuffer sRxBuffers[NRF_LIBUARTE_RX_BUFFER_NUM];

#define TEST_GPIO 25
uint32_t prev_state          = 1;
uint8_t  __CR_SECTION_NESTED = 0;

static void InitRxData(void)
{
    uint32_t index;

    for (index = 0; index < NRF_LIBUARTE_RX_BUFFER_NUM; index++)
    {
        sRxBuffers[index].mWritePos            = 0;
        sRxBuffers[index].mReadPos             = 0;
        sRxBuffers[index].mBuffPtr             = 0;
        sRxBuffers[index].mBufferReceiverState = BUFFER_STATUS_EMPTY;
        sRxBuffers[index].mBufferProcessing    = false;
    }
}

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
static otError findReceiveBufferIndex(uint32_t *aIndex, const uint8_t *aBuffer)
{
    otError  error = OT_ERROR_NOT_FOUND;
    uint32_t index;

    // if the buffer index is equal NRF_LIBUARTE_RX_BUFFER_NUM it is considered as invalid.
    uint32_t emptyBufferIndex = NRF_LIBUARTE_RX_BUFFER_NUM;

    for (index = 0; index < NRF_LIBUARTE_RX_BUFFER_NUM; index++)
    {
        // there should be only one buffer in BUFFER_STATUS_RECEIVING
        if (sRxBuffers[index].mBufferReceiverState == BUFFER_STATUS_RECEIVING)
        {
            if (sRxBuffers[index].mWritePos == aBuffer)
            {
                error   = OT_ERROR_NONE;
                *aIndex = index;
                return error;
            }
            else
                assert(false);
        }

        // we are lloking form empty buffer and save index
        if (sRxBuffers[index].mBufferReceiverState == BUFFER_STATUS_EMPTY &&
            emptyBufferIndex == NRF_LIBUARTE_RX_BUFFER_NUM)
        {
            emptyBufferIndex = index;
        }
    }

    // empty buffer found. It is allocated.
    if (emptyBufferIndex != NRF_LIBUARTE_RX_BUFFER_NUM)
    {
        sRxBuffers[emptyBufferIndex].mWritePos            = (uint8_t *)aBuffer;
        sRxBuffers[emptyBufferIndex].mReadPos             = (uint8_t *)aBuffer;
        sRxBuffers[emptyBufferIndex].mBuffPtr             = (uint8_t *)aBuffer;
        sRxBuffers[emptyBufferIndex].mBufferReceiverState = BUFFER_STATUS_RECEIVING;
        sRxBuffers[emptyBufferIndex].mBufferProcessing    = false;
        error                                             = OT_ERROR_NONE;
        *aIndex                                           = emptyBufferIndex;
    }

    return error;
}

static otError findProcessingBufferIndex(uint32_t *aIndex, uint32_t *rxBytesLength)
{
    otError  error = OT_ERROR_NOT_FOUND;
    uint32_t index;

    // if the buffer index is equal NRF_LIBUARTE_RX_BUFFER_NUM it is considered as invalid.
    uint32_t processingBufferIndex = NRF_LIBUARTE_RX_BUFFER_NUM;
    uint32_t filledBufferIndex     = NRF_LIBUARTE_RX_BUFFER_NUM;
    uint32_t fillingBufferIndex    = NRF_LIBUARTE_RX_BUFFER_NUM;

    for (index = 0; index < NRF_LIBUARTE_RX_BUFFER_NUM; index++)
    {
        if (sRxBuffers[index].mBufferProcessing == true)
        {
            if (processingBufferIndex == NRF_LIBUARTE_RX_BUFFER_NUM)
            {
                processingBufferIndex = index;
            }
            else
            {
                // there can be only one buffer avilable for processing
                assert(false);
            }
        }
        else if (sRxBuffers[index].mBufferReceiverState == BUFFER_STATUS_FILLED)
        {
            if (filledBufferIndex == NRF_LIBUARTE_RX_BUFFER_NUM)
            {
                filledBufferIndex = index;
            }
            else
            {
                // only one buffer can be completely filled.
                assert(false);
            }
        }
        else if (sRxBuffers[index].mBufferReceiverState == BUFFER_STATUS_RECEIVING)
        {
            if (fillingBufferIndex == NRF_LIBUARTE_RX_BUFFER_NUM)
            {
                fillingBufferIndex = index;
            }
            else
            {
                // only one buffer can be filled at a time.
                assert(false);
            }
        }
    }

    if (processingBufferIndex != NRF_LIBUARTE_RX_BUFFER_NUM)
    {
        *rxBytesLength = sRxBuffers[processingBufferIndex].mWritePos - sRxBuffers[processingBufferIndex].mReadPos;
        if (*rxBytesLength != 0)
        {
            error   = OT_ERROR_NONE;
            *aIndex = processingBufferIndex;
        }
    }
    else if (filledBufferIndex != NRF_LIBUARTE_RX_BUFFER_NUM)
    {
        *rxBytesLength = sRxBuffers[filledBufferIndex].mWritePos - sRxBuffers[filledBufferIndex].mReadPos;
        if (*rxBytesLength != 0)
        {
            error                                           = OT_ERROR_NONE;
            *aIndex                                         = filledBufferIndex;
            sRxBuffers[filledBufferIndex].mBufferProcessing = true;
        }
    }
    else if (fillingBufferIndex != NRF_LIBUARTE_RX_BUFFER_NUM)
    {
        *rxBytesLength = sRxBuffers[fillingBufferIndex].mWritePos - sRxBuffers[fillingBufferIndex].mReadPos;
        if (*rxBytesLength != 0)
        {
            error                                            = OT_ERROR_NONE;
            *aIndex                                          = fillingBufferIndex;
            sRxBuffers[fillingBufferIndex].mBufferProcessing = true;
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
    uint32_t length;
    uint8_t *readPos = NULL;
    otError  error   = OT_ERROR_NONE;

    CRITICAL_REGION_ENTER();
    error = findProcessingBufferIndex(&index, &length);
    CRITICAL_REGION_EXIT();

    if (error == OT_ERROR_NOT_FOUND)
    {
        return;
    }

    if (length > 0)
    {
        readPos = (uint8_t *)sRxBuffers[index].mReadPos;
        otPlatUartReceived(readPos, length);

        // Increment read pointer first and free the memory for the next reception.
        nrf_libuarte_async_rx_free(&sLibUarte, readPos, length);
        sRxBuffers[index].mReadPos += length;

        // if the buffer is processed completely it will be released
        if (sRxBuffers[index].mReadPos == sRxBuffers[index].mBuffPtr + NRF_LIBUARTE_RX_BUFFER_SIZE)
        {
            sRxBuffers[index].mWritePos            = 0;
            sRxBuffers[index].mReadPos             = 0;
            sRxBuffers[index].mBuffPtr             = 0;
            sRxBuffers[index].mBufferReceiverState = BUFFER_STATUS_EMPTY;
            sRxBuffers[index].mBufferProcessing    = false;
        }
    }
    else
        assert(false);
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
        assert(false);
        break;

    case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:

        error = findReceiveBufferIndex(&index, aEvt->data.rxtx.p_data);
        if (error == OT_ERROR_NOT_FOUND)
        {
            assert(false);
        }

        sRxBuffers[index].mWritePos += aEvt->data.rxtx.length;
        if (sRxBuffers[index].mWritePos == (sRxBuffers[index].mBuffPtr + NRF_LIBUARTE_RX_BUFFER_SIZE))
        {
            // buffer completely filled by receiver.
            // the remaining data in the buffer must be processed by the aaplication.

            sRxBuffers[index].mBufferReceiverState = BUFFER_STATUS_FILLED;
        }

        break;

    case NRF_LIBUARTE_ASYNC_EVT_TX_DONE:
        sTransmitDone = true;
        break;

    default:
        break;
    }
}

void TestGpio(void)
{
    uint32_t value;

    value = nrf_gpio_pin_read(TEST_GPIO);

    if (prev_state != value)
    {
        if (value == 0)
        {
            app_util_critical_region_enter(&__CR_SECTION_NESTED);
        }
        else
        {
            app_util_critical_region_exit(__CR_SECTION_NESTED);
        }
    }
    prev_state = value;
}

void nrf5UartProcess(void)
{
    TestGpio();
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

void InitTestGpio(void)
{
    nrf_gpio_cfg_input(TEST_GPIO, NRF_GPIO_PIN_PULLUP);
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

    InitRxData();

    ret = nrf_libuarte_async_init(&sLibUarte, &config, uarteEventHandler, NULL);
    assert(ret == NRF_SUCCESS);

    nrf_libuarte_async_enable(&sLibUarte);

    sUartEnabled = true;
    InitTestGpio();
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
    memset(&sRxBuffers, 0, sizeof(sRxBuffers));

exit:
    return error;
}

otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    otError    error = OT_ERROR_NONE;
    ret_code_t ret;

    otEXPECT_ACTION(sTransmitStarted == false, error = OT_ERROR_BUSY);

    sTransmitStarted = true;
    sTransmitDone    = false;

    ret = nrf_libuarte_async_tx(&sLibUarte, (uint8_t *)aBuf, aBufLength);
    otEXPECT_ACTION(ret == NRF_SUCCESS, error = OT_ERROR_BUSY);

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
