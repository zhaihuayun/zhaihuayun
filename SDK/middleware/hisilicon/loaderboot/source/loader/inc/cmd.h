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
  * @file    cmd.h
  * @author  MCU Driver Team
  * @brief   Header file containing functions prototypes of cmd module.
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef McuMagicTag_CMD_H
#define McuMagicTag_CMD_H

/* Includes ------------------------------------------------------------------*/

/* Macro definitions ---------------------------------------------------------*/

#define UART_CMD_PAYLOAD_MAX 60       /**< UART command payload length */
#define CMD_HEADER_SIZE     2         /**< Command header size */

/* Typedef definitions -------------------------------------------------------*/

/**
  * @brief valid cmd
  */
enum {
    CMD_DL_IMAGE = 0xD2,        /**< Download Image */
    CMD_UL_DATA = 0xB4,         /**< Upload Image */
    CMD_RESET = 0x87,           /**< Reset MCU */
};

/**
  * @brief UART CMD frame
  */
typedef struct {
    unsigned char type;                          /**< Command Type */
    unsigned char rcvBuf[UART_CMD_PAYLOAD_MAX];  /**< Command */
    unsigned char crcHigh;                         /**< CRC high byte */
    unsigned char crcLow;                        /**< CRC low byte */
    unsigned char rsv[1];
} UartCmd;

/* Functions -----------------------------------------------------------------*/

void CmdLoop(void);

#endif /* McuMagicTag_CMD_H */
