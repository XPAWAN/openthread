/**
 * Copyright (c) 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file
 *   This file implements the OpenThread platform abstraction for libUARTE communication.
 *
 */
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <utils/code_utils.h>
#include <openthread/config.h>
#include <openthread/platform/toolchain.h>
#include <openthread/platform/uart.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_uarte.h>
#include <openthread-core-config.h>
#include "nrf_libuarte_async.h"
#include "openthread-system.h"
#include "platform-nrf5.h"


#if (UARTE_AS_SERIAL_TRANSPORT == 1)

bool sUartEnabled = false;
NRF_LIBUARTE_ASYNC_DEFINE(libuarte,
                          0,
                          3,
                          NRF_LIBUARTE_PERIPHERAL_NOT_USED,
                          4,
                          NRF_LIBUARTE_RX_BUFFER_SIZE,
                          NRF_LIBUARTE_RX_BUFFER_NUM);

/**
 *  UART TX buffer variables.
 */
static bool sTransmitInProgress = false;
static bool sTransmitDone       = false;

/**
 *  UART RX ring buffer variables.
 */
static void *         sReceiverBuffer         = NULL;
static void volatile *sReceiverHead           = NULL;
static void *         sReceiverTail           = NULL;
static bool           sReceiverBufferOverflow = false;
static void *         sReceiverOverflowHead   = NULL;
static void *         sReceiverOverflowBuff   = NULL;

/**
 * Function for notifying application about new bytes received.
 */
static void processReceive(void)
{
    int received_bytes = sReceiverHead - sReceiverTail;

    if (received_bytes)
    {
        otPlatUartReceived(sReceiverTail, received_bytes);
        nrf_libuarte_async_rx_free(&libuarte, sReceiverTail, received_bytes);
        sReceiverTail += received_bytes;
    }
}

otError otPlatUartFlush(void)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

/**
 * Function for notifying application about transmission being done.
 */
static void processTransmit(void)
{
    if (sTransmitInProgress && sTransmitDone)
    {
        // Clear Transmition transaction and notify application.
        sTransmitDone       = false;
        sTransmitInProgress = false;
        otPlatUartSendDone();
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

void uart_event_handler(void *context, nrf_libuarte_async_evt_t *p_evt)
{
    switch (p_evt->type)
    {
    case NRF_LIBUARTE_ASYNC_EVT_ERROR:
        break;
    case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:

        if (sReceiverBuffer == NULL)
        {
            sReceiverBuffer = p_evt->data.rxtx.p_data;
            sReceiverHead   = p_evt->data.rxtx.p_data + p_evt->data.rxtx.length;
            sReceiverTail   = p_evt->data.rxtx.p_data;
        }
        else if (sReceiverHead == p_evt->data.rxtx.p_data)
        {
            sReceiverHead = p_evt->data.rxtx.p_data + p_evt->data.rxtx.length;
        }
        else if (sReceiverBufferOverflow == false) // libuarte buffer overflow
        {
            if (sReceiverHead == sReceiverTail)
            {
                sReceiverBuffer = p_evt->data.rxtx.p_data;
                sReceiverTail   = p_evt->data.rxtx.p_data;
                sReceiverHead   = p_evt->data.rxtx.p_data + p_evt->data.rxtx.length;
            }
            else
            {
                sReceiverBufferOverflow = true;
                sReceiverOverflowBuff   = p_evt->data.rxtx.p_data;
                sReceiverOverflowHead   = p_evt->data.rxtx.p_data + p_evt->data.rxtx.length;
            }
        }
        else if (sReceiverOverflowHead == p_evt->data.rxtx.p_data)
        {
            sReceiverOverflowHead = p_evt->data.rxtx.p_data + p_evt->data.rxtx.length;
            if ((sReceiverBufferOverflow == true) && (sReceiverHead == sReceiverTail)) // libuarte buffer overflow
            {
                sReceiverBuffer         = sReceiverOverflowBuff;
                sReceiverBufferOverflow = false;
                sReceiverTail           = sReceiverOverflowBuff;
                sReceiverHead           = sReceiverOverflowHead;
            }
        }
        else
        { // two buffers completely filled with data.
            assert(false);
        }

        break;
    case NRF_LIBUARTE_ASYNC_EVT_TX_DONE:
        sTransmitDone = true;
        break;
    default:
        break;
    }
}

otError otPlatUartEnable(void)
{
    otError                     error = OT_ERROR_NONE;
    ret_code_t                  err_code;
    nrf_libuarte_async_config_t nrf_libuarte_async_config = {
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
        .timeout_us = 100,
        .int_prio   = APP_IRQ_PRIORITY_LOW
    };

    otEXPECT_ACTION(sUartEnabled == false, error = OT_ERROR_ALREADY);
    err_code = nrf_libuarte_async_init(&libuarte, &nrf_libuarte_async_config, uart_event_handler, (void *)&libuarte);
    assert(err_code == NRF_SUCCESS);
    nrf_libuarte_async_enable(&libuarte);
    sUartEnabled = true;

exit:
    return error;
}

otError otPlatUartDisable(void)
{
    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION(sUartEnabled == true, error = OT_ERROR_ALREADY);
    nrf_libuarte_async_uninit(&libuarte);
    sUartEnabled = false;

exit:
    return error;
}

otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    otError    error = OT_ERROR_NONE;
    ret_code_t ret;

    if (sTransmitInProgress == true)
        return OT_ERROR_BUSY;

    sTransmitInProgress = true;
    sTransmitDone       = false;
    ret                 = nrf_libuarte_async_tx(&libuarte, (uint8_t *)aBuf, aBufLength);
    otEXPECT_ACTION(ret == NRF_SUCCESS, error = OT_ERROR_BUSY);

exit:
    return error;
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
