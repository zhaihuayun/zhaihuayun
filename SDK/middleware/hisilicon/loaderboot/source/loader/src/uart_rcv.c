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
  * @file    uart_rcv.c
  * @author  MCU Driver Team
  * @brief   Boot uart rcv funcs.
  */

/* Includes ------------------------------------------------------------------*/
#include "loaderboot.h"
#include "utils.h"
#include "crc_adapt.h"
#include "uart_adapt.h"
#include "transfer.h"

/* Macro definitions ---------------------------------------------------------*/

/* Typedef definitions -------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
/**
  * @brief Get Data length of Frame
  * @param header     Frame type
  * @param expectLen  Expected frame length(only for data frame)
  * @retval unsigned int Frame length
  */
static unsigned int UartGetFrameDataLen(unsigned char header,
                                        unsigned int expectLen)
{
    if (header == XHDSHK) {
        return HANDSHAKE_FRAME_SIZE - FRAME_COST_SIZE;
    }

    if (header == XHEAD) {
        return HEADER_FRAME_SIZE - FRAME_COST_SIZE;
    }

    if (header == XTAIL) {
        return TAIL_FRAME_SIZE - FRAME_COST_SIZE;
    }

    if (header == XDATA) {
        return expectLen;
    }

    return 0;
}

/**
  * @brief Check if it is a valid frame header.
  * @param header Frame header.
  * @retval true
  * @retval false
  */
static bool UartRcvIsHeaderSupported(unsigned char header)
{
    bool supported = false;

    switch (header) {
        case XHDSHK:
        case XHEAD:
        case XTAIL:
        case XDATA:
        case XACK:
        case XCMD:
        case XKEY:
            supported = true;
            break;

        default:
            break;
    }

    return supported;
}

/**
  * @brief Check if there is a data frame.
  * @param header Frame header.
  * @retval true
  * @retval false
  */
static inline bool UartRcvIsDataFrame(unsigned char header)
{
    return (header == XDATA) ? true : false;
}

/**
  * @brief Check if there is a tail frame.
  * @param header Frame header.
  * @retval true
  * @retval false
  */
static inline bool UartRcvIsTailFrame(unsigned char header)
{
    return (header == XTAIL) ? true : false;
}

/**
  * @brief Check whether the CRC of the received frame is correct.
  * @param frame Frame header.
  * @param len Data payload length
  * @retval true
  * @retval false
  */
static bool UartRcvIsCrcSucc(struct UartFrame *frame, unsigned int len)
{
    unsigned int crc;
    if (frame == NULL) {
        return false;
    }
    crc = MergeToUshort(frame->crcHigh, frame->crcLow);
    CRC_Generate((unsigned char *)(void *)frame, OFFSETOF(struct UartFrame, indexReverse) + 1);
    return CRC_AccCheck(frame->payloadBuf, len, crc);
}

/**
  * @brief  Check whether the CRC of the received ACK frame is correct.
  * @param frame Frame header.
  * @retval true
  * @retval false
  */
static bool UartRcvAckIsCrcSucc(struct UartFrame *frame)
{
    unsigned char buf[sizeof(struct UartFrame)];
    unsigned short crc;
    unsigned int len = 0;

    if (frame == NULL) {
        return false;
    }
    buf[len++] = frame->header;
    buf[len++] = frame->index;
    buf[len++] = frame->indexReverse;
    buf[len++] = frame->ackRslt;
    crc = MergeToUshort(frame->crcHigh, frame->crcLow);
    return CRC_Check(buf, len, crc);
}

/**
  * @brief Receive sync frame
  * @param frame Frame header.
  * @param timeoutUs  Overtime time(us)
  * @retval BASE_STATUS_OK Success
  * @retval BASE_STATUS_ERROR Fail
  */
static BASE_StatusType UartRcvSync(struct UartFrame *const frame, unsigned int timeoutUs)
{
    unsigned int retry = 0;
    BASE_StatusType ret;

    while (retry < timeoutUs) {
        ret = SERIAL_GetCharTimeout(FRAME_RX_DELAY_US, &frame->header);
        if ((ret != BASE_STATUS_OK) ||
            !UartRcvIsHeaderSupported(frame->header)) {
            retry++;
            continue;
        }

        ret = SERIAL_GetCharTimeout(FRAME_RX_DELAY_US, &frame->index);
        ret |= SERIAL_GetCharTimeout(FRAME_RX_DELAY_US, &frame->indexReverse);
        if (ret != BASE_STATUS_OK)  {
            retry++;
            continue;
        }
        if (frame->index & frame->indexReverse) {
            retry++;
            continue;
        }
        if ((frame->index == 0) && (frame->indexReverse != 0xFF)) {
            retry++;
            continue;
        }

        return BASE_STATUS_OK;
    }

    return BASE_STATUS_TIMEOUT;
}

/**
  * @brief  Clear UART FIFO
  * @retval None
  */
void UartFlush(void)
{
    while (1) {
        unsigned char ch = (char)0xff;
        unsigned int ret = SERIAL_GetCharTimeout(HEADER_RX_DELAY_US, &ch);
        if (ret == BASE_STATUS_ERROR) {
            return;
        }
    }
}

/**
  * @brief Receive Frame
  * @param frame      Struct Pointer to save frame
  * @param rcvDataLen The data length
  * @param retryCnt   Maximum number of retries
  * @retval BASE_STATUS_OK Success
  * @retval BASE_STATUS_ERROR Fail
  */
BASE_StatusType UartRcvFrame(struct UartFrame *const frame,
                             unsigned int rcvDataLen,
                             unsigned int retryCnt)
{
    unsigned int ret;
    unsigned int realLen;
    unsigned int i;

    if (!frame ||
        (frame->payloadBufLen < rcvDataLen) ||
        (rcvDataLen > MAX_FRAME_DATA_SIZE)) {
        return BASE_STATUS_ERROR;
    }

    /* Receive frame synchronization */
    ret = UartRcvSync(frame, retryCnt);
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }

    /* According to the frame type, get the frame length and receive the frame
       content cyclically */
    realLen = UartGetFrameDataLen(frame->header, rcvDataLen);

    /*  Because UartRcvSync() above has received the first three bytes of the
        frame, so here we need to start from the third byte */
    for (i = 0; i < realLen; i++) {
        ret = SERIAL_GetCharTimeout(FRAME_RX_DELAY_US, frame->payloadBuf + i);
        if (ret != BASE_STATUS_OK) {
            return ret;
        }
    }
    ret = SERIAL_GetCharTimeout(FRAME_RX_DELAY_US, &frame->crcHigh);
    ret |= SERIAL_GetCharTimeout(FRAME_RX_DELAY_US, &frame->crcLow);
    if (ret != BASE_STATUS_OK) {
        return ret;
    }
    if (!UartRcvIsCrcSucc(frame, realLen)) {
        return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Receive Header frame
  * @param frame   Pointer to frame struct
  * @param rcvLen  Get the total data length
  * @param maxLen  Max data length
  * @retval BASE_STATUS_OK Success
  * @retval BASE_STATUS_ERROR Fail
  */

BASE_StatusType UartRcvHdrFrame(struct UartFrame *const frame, unsigned int *rcvLen, unsigned int maxLen)
{
    signed int frameRcvTh = FRAME_RETRY_MAX_TH;
    unsigned int ret;

    if ((frame == NULL) || (rcvLen == NULL)) {
        return BASE_STATUS_ERROR;
    }
    while (--frameRcvTh >= 0) {
        ret = UartRcvFrame(frame, HEADER_FRAME_SIZE, MAX_SYNC_RETRIES);
        if ((ret == BASE_STATUS_OK) && UartRcvIsHeaderFrame(frame)) {
            break;
        }
        if (frameRcvTh == 0) {
            return BASE_STATUS_ERROR;
        }
    }

    *rcvLen = BigEndianToUint(frame->payloadBuf + 1);

    if (*rcvLen > maxLen) {
        UartAck(ACK_FAILURE, 0);
        return BASE_STATUS_ERROR;
    }

    return BASE_STATUS_OK;
}

/**
  * @brief Receive Data frame
  * @param frame  Pointer to frame struct
  * @param idx    The index of data frame
  * @param length Data length
  * @retval BASE_STATUS_OK Success
  * @retval BASE_STATUS_ERROR Fail
  */
BASE_StatusType UartRcvDataFrame(struct UartFrame *const frame, unsigned char idx, unsigned int length)
{
    signed int frameRcvTh = FRAME_RETRY_MAX_TH;
    BASE_StatusType ret;

    if (frame == NULL) {
        return BASE_STATUS_ERROR;
    }
    while (--frameRcvTh >= 0) {
        ret = UartRcvFrame(frame, length, MAX_SYNC_RETRIES);
        if ((ret == BASE_STATUS_OK) &&
            ((idx == frame->index) || (idx == frame->index + 1))) {
            break;
        }
        if (frameRcvTh > 0) {
            UartAck(ACK_FAILURE, idx);
        } else {
            return BASE_STATUS_ERROR;
        }
    }
    UartAck(ACK_SUCCESS, frame->index);
    return BASE_STATUS_OK;
}

/**
  * @brief Receive tail frame
  * @param frame  Pointer to frame struct
  * @param idx    The index of tail frame
  * @retval BASE_STATUS_OK Success
  * @retval BASE_STATUS_ERROR Fail
  */
BASE_StatusType UartRcvTailFrame(struct UartFrame *const frame,  unsigned char idx)
{
    if (frame == NULL) {
        return BASE_STATUS_ERROR;
    }
    return UartRcvDataFrame(frame, idx, 0);
}

/**
  * @brief Waiting for confirmation frame
  * @param frame  Pointer to frame struct
  * @param idx    The index of waiting frame
  * @retval BASE_STATUS_OK Success
  * @retval BASE_STATUS_ERROR Fail
  */
BASE_StatusType UartWaitAck(struct UartFrame *const frame, unsigned char idx)
{
    BASE_StatusType ret;

    if (frame == NULL) {
        return BASE_STATUS_ERROR;
    }

    /* Wait The Sync bytes of frame */
    ret = UartRcvSync(frame, MAX_SYNC_RETRIES);
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }

    /* Wait CRC and ACK result */
    ret = SERIAL_GetCharTimeout(FRAME_RX_DELAY_US, &frame->ackRslt);
    ret |= SERIAL_GetCharTimeout(FRAME_RX_DELAY_US, &frame->crcHigh);
    ret |= SERIAL_GetCharTimeout(FRAME_RX_DELAY_US, &frame->crcLow);
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }

    /* Do CRC check */
    if (!UartRcvAckIsCrcSucc(frame)) {
        return BASE_STATUS_ERROR;
    }

    if ((frame->ackRslt != ACK_SUCCESS) || (frame->index != idx)) {
        return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}
