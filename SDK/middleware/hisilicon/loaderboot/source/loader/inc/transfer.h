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
  * @file    transfer.h
  * @author  MCU Driver Team
  * @brief   uart transfer head file.
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef McuMagicTag_TRANSFER_H
#define McuMagicTag_TRANSFER_H

/* Includes ------------------------------------------------------------------*/
#include "baseinc.h"

/* Macro definitions ---------------------------------------------------------*/
#define MAX_SYNC_RETRIES 0xFFFFFFFF          /**< Maximum number of attempts to receive synchronization frame*/

#define HEADER_RX_DELAY_US 5000000   /**< Frame header receiving timeout time */
#define FRAME_RX_DELAY_US  3000000   /**< Frame receiving timeout time */

#define HANDSHAK_DELAY_US  100000    /**< Handshake timeout time */

#define MAX_FRAME_DATA_SIZE 1024     /**< Max size of Frame Data Payload */

/**
  * @brief Maximum length of various frames
  */
#define HANDSHAKE_FRAME_SIZE 9
#define HEADER_FRAME_SIZE 14
#define TAIL_FRAME_SIZE 5
#define ACK_FRAME_SIZE 6

#define FRAME_RETRY_MAX_TH  1000 /**< Frame overhead size */
#define FRAME_COST_SIZE 5        /**< Number of received frame attempts */

#define FRAME_HEADER_CHK_LEN  3  /**< frame header CRC check length */
#define DATA_FRAME_COST  2       /**< Data frame overhead */

/*
 * @brief Offset of common frame
 */
#define FRAME_TYPE_OFFSET          0
#define FRAME_SEQ_OFFSET           1
#define FRAME_INV_SEQ_OFFSET       2

/**
  * @brief Offset of ack frame
  */
#define ACK_FRAME_RESULT_OFFSET   3
#define ACK_FRAME_CRC_OFFSET      4

/**
  * @brief Offset of header frame
  */
#define HEAD_FRAME_LENGTH_OFFSET   4
#define HEAD_FRAME_ADDRESS_OFFSET  8
#define HEAD_FRAME_CRC_OFFSET      12

/**
  * @brief Offset of data crc frame
  */
#define DATA_FRAME_CRC_OFFSET   3

/**
  * @brief FRAME Type
  */
#define XHDSHK 0xBE
#define XHEAD  0xFE
#define XDATA  0xDA
#define XTAIL  0xED
#define XACK   0xCB
#define XCMD   0xAB
#define XKEY   0xCD

/**
  * @brief ACK Result
  */
#define ACK_SUCCESS 0x5A
#define ACK_FAILURE 0xA5

/* Typedef definitions -------------------------------------------------------*/
/**
  * @brief Uart Frame Structure
  */
struct UartFrame {
    unsigned char  header;          /**< Frame Type */
    unsigned char  index;           /**< Frame index */
    unsigned char  indexReverse;    /**< Frame index Reversed */
    unsigned char *payloadBuf;      /**< Pointer to payload buffer */
    unsigned int   payloadBufLen;   /**< Length of payload buffer */
    unsigned char  crcHigh;         /**< The first byte of CRC16 */
    unsigned char  crcLow;          /**< The Seconde byte of CRC16 */
    unsigned char  ackSeq;          /**< The Sequence of ACK */
    unsigned char  ackRslt;         /**< Result of ACK */
};

/* Functions -----------------------------------------------------------------*/
/**
  * @brief Check whether it is a file header frame
  * @param frame  Pointer to the frame structure
  * @retval true
  * @retval false
  */
static inline bool UartRcvIsHeaderFrame(struct UartFrame *frame)
{
    return ((frame->header == XHEAD) && (frame->index == 0x0)) ? true : false;
}

/**
  * @brief Check whether it is a command header frame
  * @param frame  Pointer to the frame structure
  * @retval true
  * @retval false
  */
static inline bool UartRcvIsCmdHeaderFrame(struct UartFrame *frame)
{
    return (frame->header == XCMD) ? true : false;
}

void UartFlush(void);
BASE_StatusType UartRcvFrame(struct UartFrame *const frame, unsigned int rcvLen, unsigned int timeoutUs);
BASE_StatusType UartRcvHdrFrame(struct UartFrame *const frame, unsigned int *rcvLen, unsigned int maxLen);
BASE_StatusType UartRcvTailFrame(struct UartFrame *const frame, unsigned char index);
BASE_StatusType UartRcvDataFrame(struct UartFrame *const frame, unsigned char idx, unsigned int length);

void UartAck(unsigned char result, unsigned char index);
BASE_StatusType UartWaitAck(struct UartFrame *const frame, unsigned char index);
BASE_StatusType UartXmitMakeHeadFrame(unsigned char *const frame, unsigned int len, unsigned int addr);
BASE_StatusType UartXmitMakeDataFrame(unsigned char *const frame, unsigned int dataLen,
                                      unsigned int bufLen, unsigned char seq);
BASE_StatusType UartXmitMakeTailFrame(unsigned char seq);

#endif /* McuMagicTag_TRANSFER_H */
