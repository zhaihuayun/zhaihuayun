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
  * @file    uart_xmit.c
  * @author  MCU Driver Team
  * @brief   Boot uart send funcs.
  * @details Frame construction function
  */

/* Includes ------------------------------------------------------------------*/
#include "loaderboot.h"
#include "crc_adapt.h"
#include "uart_adapt.h"
#include "transfer.h"

/* Macro definitions ---------------------------------------------------------*/
#define HEAD_FRAME_HEAD 0xFE00FF01  /**< Header Frame head*/

/* Typedef definitions -------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
/**
  * @brief The caller prepares the buffer and reserves the front and rear space
  *        according to the frame format
  *              00   FF
  *        XDATA seq ~seq 0x01 ---len--- ---addr--- CRC_HIGH CRC_LOW
  * @param frame   The buffer to save the frame
  * @param length  The length to be written
  * @param addr    The address to be written
  * @retval BASE_STATUS_OK success
  */
unsigned int UartXmitMakeHeadFrame(unsigned char *const frame, unsigned int len, unsigned int addr)
{
    unsigned short crc;

    if (frame == NULL) {
        return BASE_STATUS_ERROR;
    }
    /* Write in header */
    UintToBigEndian(HEAD_FRAME_HEAD, frame);

    /* Write in length */
    UintToBigEndian(len, frame + HEAD_FRAME_LENGTH_OFFSET);

    /* Write in address */
    UintToBigEndian(addr, frame + HEAD_FRAME_CRC_OFFSET);

    /* Calc and Write CRC */
    crc = CRC_Generate(frame, HEAD_FRAME_CRC_OFFSET);
    UShortToBigEndian(crc, frame + HEAD_FRAME_CRC_OFFSET);

    SERIAL_PutBuf((const char *)frame, HEADER_FRAME_SIZE);
    return BASE_STATUS_OK;
}

/**
  * @brief The caller prepares the buffer and reserves the front and rear space
  *        according to the frame format
  *        -------------------bufLen------------------
  *        XDATA seq ~seq ---dataLen--- CRC_HIGH CRC_LOW
  * @param frame The Pointer to the buffer
  * @param dataLen The data length in frame
  * @param bufLen  The length of buffer
  * @param seq     Sequence of buffer
  * @retval        BOOT_FAILUE  fail
  * @retval        BASE_STATUS_OK success
  */
unsigned int UartXmitMakeDataFrame(unsigned char *const frame,
                                   unsigned int dataLen,
                                   unsigned int bufLen,
                                   unsigned char seq)
{
    unsigned short crc;
    unsigned int sequence = seq;

    /* Parameter Check */
    if (frame == NULL) {
        return BASE_STATUS_ERROR;
    }
    if ((dataLen + FRAME_COST_SIZE > bufLen) ||
        (dataLen > MAX_FRAME_DATA_SIZE)) {
        return BASE_STATUS_ERROR;
    }

    /* Construct the Frame */
    frame[FRAME_TYPE_OFFSET] = XDATA;
    frame[FRAME_SEQ_OFFSET] = seq;
    frame[FRAME_INV_SEQ_OFFSET] = (unsigned char)(~sequence & 0xFF);

    /* Generate CRC and write to frame */
    crc = CRC_Generate(frame, dataLen + FRAME_HEADER_CHK_LEN);
    UShortToBigEndian(crc, &frame[dataLen + DATA_FRAME_CRC_OFFSET]);
    SERIAL_PutBuf((const char *)frame, dataLen + FRAME_COST_SIZE);
    return BASE_STATUS_OK;
}

/**
  * @brief send tail frame.
  *        XTAIL seq ~seq CRC_HIGH CRC_LOW
  * @param seq the sequence number of transmit frame
  * @retval BASE_STATUS_OK success
  */
unsigned int UartXmitMakeTailFrame(unsigned char seq)
{
    unsigned char frame[TAIL_FRAME_SIZE];
    unsigned short crc;
    unsigned int sequence = seq;

    /* Construct the Frame */
    frame[FRAME_TYPE_OFFSET] = XTAIL;
    frame[FRAME_SEQ_OFFSET] = seq;
    frame[FRAME_INV_SEQ_OFFSET] = (unsigned char)(~sequence & 0xFF);

    /* Generate CRC and write to frame */
    crc = CRC_Generate(frame, (DATA_FRAME_CRC_OFFSET - FRAME_TYPE_OFFSET));
    UShortToBigEndian(crc, &frame[DATA_FRAME_CRC_OFFSET]);
    SERIAL_PutBuf((const char *)frame, TAIL_FRAME_SIZE);
    return BASE_STATUS_OK;
}

/**
  * @brief send ack frame.
  *        XACK seq ~seq result CRC_HIGH CRC_LOW
  * @param result Result in ACK Frame
  * @param seq    Sequence in ACK Frame
  * @retval .
  */
void UartAck(unsigned char result, unsigned char seq)
{
    unsigned char frame[ACK_FRAME_SIZE];
    unsigned short crc;
    unsigned int sequence = seq;

    /* Construct the Frame */
    frame[FRAME_TYPE_OFFSET] = XACK;
    frame[FRAME_SEQ_OFFSET] = seq;
    frame[FRAME_INV_SEQ_OFFSET] = (unsigned char)(~sequence & 0xFF);
    frame[ACK_FRAME_RESULT_OFFSET] = result;

    /* Generate CRC and write to frame */
    crc = CRC_Generate(frame, (ACK_FRAME_CRC_OFFSET - FRAME_TYPE_OFFSET));
    UShortToBigEndian(crc, &frame[ACK_FRAME_CRC_OFFSET]);
    SERIAL_PutBuf((const char *)frame, ACK_FRAME_SIZE);
    return;
}
