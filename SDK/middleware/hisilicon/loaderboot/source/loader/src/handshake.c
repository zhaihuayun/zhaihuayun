/**
  * @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
  * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
  * following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
  * disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
  * following disclaimer in the documentation and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
  * products derived from this software without specific prior written permission.
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * @file    handshake.c
  * @author  MCU Driver Team
  * @brief   The following functions are implemented:
  *              + uart hand shake function
  */

/* Includes ------------------------------------------------------------------*/
#include "loaderboot.h"
#include "transfer.h"
#include "utils.h"
#include "uart_adapt.h"
#include "timer_adapt.h"
#include "handshake.h"

/* Macro definitions ---------------------------------------------------------*/
#define UART_DEFAULT_BAUDRATE   (115200)

/**
  * @brief rsv handshake frame & check.
  * @param timeoutMs.
  * @retval true false.
  */
bool HandShake(unsigned int timeoutMs)
{
    BASE_StatusType ret;
    unsigned int baudrate;
    unsigned char buf[HANDSHAKE_FRAME_SIZE] = {0};
    struct UartFrame frame = {0};

    frame.header = XHDSHK;
    frame.payloadBuf = buf;
    frame.payloadBufLen = sizeof(buf);

    /* Receive handshake frame */
    TimerStart(timeoutMs * US_PER_MS);
    do {
        ret = UartRcvFrame(&frame, sizeof(buf), 1);
        if ((ret != BASE_STATUS_OK) || (frame.header != XHDSHK) || (frame.index != 0x0)) {
            continue;
        }
        /* Check baud rate */
        baudrate = BigEndianToUint(frame.payloadBuf);
        if (baudrate == 0) {
            continue;
        }
        UartAck(ACK_SUCCESS, 0);
        if (baudrate != UART_DEFAULT_BAUDRATE) {
            UART_Init(baudrate);
        }
        return true;
    } while (!IsTimerOut());

    return false;
}
