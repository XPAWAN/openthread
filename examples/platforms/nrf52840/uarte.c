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
#include "nrf_libuarte_async.h"
#include "openthread-system.h"
#include "platform-nrf5.h"
#include <assert.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_uarte.h>
#include <openthread-core-config.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <utils/code_utils.h>
#include <openthread/config.h>
#include <openthread/platform/toolchain.h>
#include <openthread/platform/uart.h>

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
 *  UART TX buffer variables.
 */
static bool sTransmitInProgress = false;
static bool sTransmitDone       = false;

/**
 *  UART RX ring buffer variables.
 */
static void *         sReceiveBuffer         = NULL;
static void volatile *sReceiveTail           = NULL;
static void volatile *sReceiveHead           = NULL;
static bool           sReceiveOverflowBuffer = false;
static void *         sReceiveOverflowHead   = NULL;
static void *         sReceiveOverflowBuff   = NULL;

/**
 * Function for notifying application about new bytes received.
 */
static void processReceive(void)
{
    int received_bytes = sReceiveTail - sReceiveHead;

    if (received_bytes)
    {
        otPlatUartReceived((void *)sReceiveHead, received_bytes);
        nrf_libuarte_async_rx_free(&sLibUarte, (void *)sReceiveHead, received_bytes);
        sReceiveHead += received_bytes;
    }
}

/**
 * Function for notifying application about transmission being done.
 */
static void processTransmit(void)
{
    if (sTransmitDone == true)
    {
        assert(sTransmitInProgress);
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

void uart_event_handler(void *aContext, nrf_libuarte_async_evt_t *p_evt)
{
    switch (p_evt->type)
    {
    case NRF_LIBUARTE_ASYNC_EVT_ERROR:
        break;
    case NRF_LIBUARTE_ASYNC_EVT_RX_DATA:

        if (sReceiveBuffer == NULL)
        {
            sReceiveBuffer = p_evt->data.rxtx.p_data;
            sReceiveTail   = p_evt->data.rxtx.p_data + p_evt->data.rxtx.length;
            sReceiveHead   = p_evt->data.rxtx.p_data;
        }
        else if (sReceiveTail == p_evt->data.rxtx.p_data)
        {
            sReceiveTail = p_evt->data.rxtx.p_data + p_evt->data.rxtx.length;
        }
        else if (sReceiveOverflowBuffer == false) // libuarte buffer overflow
        {
            if (sReceiveTail == sReceiveHead)
            {
                sReceiveBuffer = p_evt->data.rxtx.p_data;
                sReceiveHead   = p_evt->data.rxtx.p_data;
                sReceiveTail   = p_evt->data.rxtx.p_data + p_evt->data.rxtx.length;
            }
            else
            {
                sReceiveOverflowBuffer = true;
                sReceiveOverflowBuff   = p_evt->data.rxtx.p_data;
                sReceiveOverflowHead   = p_evt->data.rxtx.p_data + p_evt->data.rxtx.length;
            }
        }
        else if (sReceiveOverflowHead == p_evt->data.rxtx.p_data)
        {
            sReceiveOverflowHead = p_evt->data.rxtx.p_data + p_evt->data.rxtx.length;
            if (sReceiveTail == sReceiveHead) // libuarte buffer overflow
            {
                sReceiveBuffer         = sReceiveOverflowBuff;
                sReceiveOverflowBuffer = false;
                sReceiveHead           = sReceiveOverflowBuff;
                sReceiveTail           = sReceiveOverflowHead;
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

    err_code = nrf_libuarte_async_init(&sLibUarte, &nrf_libuarte_async_config, uart_event_handler, (void *)&sLibUarte);
    assert(err_code == NRF_SUCCESS);

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
    sUartEnabled = false;

exit:
    return error;
}

otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    otError    error = OT_ERROR_NONE;
    ret_code_t ret;

    otEXPECT_ACTION(sTransmitInProgress == false, error = OT_ERROR_BUSY);
    ret = nrf_libuarte_async_tx(&sLibUarte, (uint8_t *)aBuf, aBufLength);
    otEXPECT_ACTION(ret == NRF_SUCCESS, error = OT_ERROR_BUSY);

    sTransmitInProgress = true;
    sTransmitDone       = false;

exit:
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
