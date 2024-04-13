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
  * @file    cmd.c
  * @author  MCU Driver Team
  * @brief   cmd module.
  * @details This file provides cmd's functions to parse exec the cmd received.
  */

/* Includes ------------------------------------------------------------------*/
#include "transfer.h"
#include "loaderboot.h"
#include "flash_adapt.h"
#include "crc_adapt.h"
#include "uart_adapt.h"
#include "clock.h"
#include "cmd.h"

/* Macro definitions ---------------------------------------------------------*/
#define IMAGE_ADDR_OFFSET  1    /**< Image address offset field in header frame */
#define IMAGE_SIZE_OFFSET  5    /**< Image size offset field in header frame */
#define FRAME_DATA_OFFSET  3    /**< Data offset field in data frame */
#define BYTE_HIGH_OFFSET   8    /**< Bitoffset of in unsigned short */
#define RESET_DELAY        100  /**< Delay of jump to target(ms) */

/* Typedef definitions -------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
/**
  * @brief calculate cmd's crc.
  * @param cmd, len.
  * @retval true false.
  */
static bool IsCmdCrcSucc(const UartCmd *cmd, unsigned int len)
{
    unsigned char *p = (unsigned char *)(void *)cmd;
    unsigned int crcDataLen = len + sizeof(cmd->type);
    return CRC_Check(p, crcDataLen, MergeToUshort(cmd->crcHigh, cmd->crcLow));
}

#ifndef EFLASH_ENABLE
/**
  * @brief Write Image to SRAM
  * @param daddr  destination address to be written
  * @param saddr  source address
  * @param len    data length
  */
static void WriteToSRAM(unsigned char *daddr, unsigned char *saddr, unsigned int len)
{
    unsigned int i;
    for (i = 0; i < len; ++i) {
        daddr[i] = saddr[i];
    }
}
#endif

/**
  * @brief exec download image cmd.
  * @param cmd.  command type
  * @retval succ fail.
  */
static BASE_StatusType LoaderDownloadImage(const UartCmd *cmd)
{
    unsigned int addr;
    unsigned int size;
    unsigned char idx;
    unsigned int rcvLen;
    unsigned int remainLen;
    unsigned int flashBaseAddr = 0;

    unsigned char buf[MAX_FRAME_DATA_SIZE];
    struct UartFrame frame = {0};

    addr = BigEndianToUint(&cmd->rcvBuf[IMAGE_ADDR_OFFSET]);
    size = BigEndianToUint(&cmd->rcvBuf[IMAGE_SIZE_OFFSET]);

    frame.payloadBuf = buf;
    frame.payloadBufLen = MAX_FRAME_DATA_SIZE;

    /* Get File header Frame, and get the file total length */
    if (UartRcvHdrFrame(&frame, &remainLen, size) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }

    if (addr < FLASH_READ_BASE) {
        return BASE_STATUS_ERROR;
    }

    /* Erase Flash */
    flashBaseAddr = addr - FLASH_READ_BASE;
    if (BASE_STATUS_OK != FLASH_Erase(flashBaseAddr, remainLen)) {
        return BASE_STATUS_ERROR;
    }
    UartAck(ACK_SUCCESS, 0);
    idx = 1;

    /* Get Data from UART and Write to Flash */
    while (remainLen > 0) {
        rcvLen = GetMinValue(remainLen, MAX_FRAME_DATA_SIZE);
        if (UartRcvDataFrame(&frame, idx, rcvLen) != BASE_STATUS_OK) {
            return BASE_STATUS_ERROR;
        }
        if (idx == frame.index) {
#if defined(EFLASH_ENABLE) && (EFLASH_ENABLE == 1)
            if (BASE_STATUS_OK != FLASH_Write(buf, flashBaseAddr, rcvLen)) {
                return BASE_STATUS_ERROR;
            }
            flashBaseAddr += rcvLen;
#else
            WriteToSRAM((unsigned char *)(void*)addr, frame.payloadBuf, rcvLen);
            addr += rcvLen;
#endif
            remainLen -= rcvLen;
            ++idx;
        }
    }
    /* Wait to get the Tail Frame */
    return UartRcvTailFrame(&frame, idx);
}

/**
  * @brief  exec upload image cmd.
  * @param  cmd uart command
  * @retval succ fail.
  */
static BASE_StatusType LoaderUploadImage(const UartCmd *cmd)
{
    unsigned int ret;
    unsigned int sendLen;
    unsigned int remainLen;
    unsigned int addr;
    unsigned int size;
    unsigned char idx;
    unsigned char frame[MAX_FRAME_DATA_SIZE + FRAME_COST_SIZE];

    addr = BigEndianToUint(&cmd->rcvBuf[IMAGE_ADDR_OFFSET]);
    size = BigEndianToUint(&cmd->rcvBuf[IMAGE_SIZE_OFFSET]);

    /* make the header of frame */
    UartXmitMakeHeadFrame(frame, size, addr);

    ret = UartWaitAck((struct UartFrame *)frame, 0);
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }

    /* Send data cyclically */
    idx = 0;
    remainLen = size;
    while (remainLen > 0) {
        idx++;  /* Data frame index starts from 1 */
        sendLen = GetMinValue(remainLen, MAX_FRAME_DATA_SIZE);

        /* Read image from flash */
        ret = FLASH_Read(addr, sendLen, frame + FRAME_DATA_OFFSET, sizeof(frame) - FRAME_DATA_OFFSET);
        if (ret != BASE_STATUS_OK) {
            return BASE_STATUS_ERROR;
        }

        ret = UartXmitMakeDataFrame(frame, sendLen, sizeof(frame), idx);
        if (ret != BASE_STATUS_OK) {
            return BASE_STATUS_ERROR;
        }

        ret = UartWaitAck((struct UartFrame *)frame, idx);
        if (ret != BASE_STATUS_OK) {
            return BASE_STATUS_ERROR;
        }

        remainLen -= sendLen;
        addr += sendLen;
    }

    /* Send the tail frame */
    ret = (unsigned int)UartXmitMakeTailFrame(++idx);
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }

    ret = UartWaitAck((struct UartFrame *)frame, idx);
    if (ret != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }

    return BASE_STATUS_OK;
}

/**
  * @brief Get Command Frame length
  * @param cmd. uart command
  * @param length. Output of command frame length
  * @retval succ fail.
  */
static BASE_StatusType UartGetCmdLength(UartCmd *cmd, unsigned short *length)
{
    unsigned int ret;
    unsigned short cmdLen;

    ret = SERIAL_GetCharTimeout(FRAME_RX_DELAY_US, &cmd->rcvBuf[0]);
    ret |= SERIAL_GetCharTimeout(FRAME_RX_DELAY_US, &cmd->rcvBuf[1]);
    ret |= SERIAL_GetCharTimeout(FRAME_RX_DELAY_US, &cmd->crcHigh);
    ret |= SERIAL_GetCharTimeout(FRAME_RX_DELAY_US, &cmd->crcLow);
    if ((ret != BASE_STATUS_OK) || (!IsCmdCrcSucc(cmd, CMD_HEADER_SIZE))) {
        return BASE_STATUS_ERROR;
    }
    cmdLen = (cmd->rcvBuf[0] << BYTE_HIGH_OFFSET) + cmd->rcvBuf[1];
    if (cmdLen > UART_CMD_PAYLOAD_MAX) {
        return BASE_STATUS_ERROR;
    }
    UartAck(ACK_SUCCESS, 0); /* Received cmd header frame success */
    *length = cmdLen;
    return BASE_STATUS_OK;
}

/**
  * @brief Get Command Context
  * @param cmd. uart command
  * @param length. Output of command frame length
  * @retval succ fail.
  */
static BASE_StatusType UartGetCmdContext(UartCmd *cmd, unsigned short length)
{
    unsigned int ret;
    unsigned short cmdLen = length;

    for (unsigned int i = 0; i < cmdLen; i++) {
        ret = SERIAL_GetCharTimeout(FRAME_RX_DELAY_US, cmd->rcvBuf + i); /* get timeout status */
        if (ret != BASE_STATUS_OK) {
            return BASE_STATUS_ERROR;
        }
    }

    ret = SERIAL_GetCharTimeout(FRAME_RX_DELAY_US, &cmd->crcHigh); /* read crc high */
    ret |= SERIAL_GetCharTimeout(FRAME_RX_DELAY_US, &cmd->crcLow); /* read crc low */
    if ((ret != BASE_STATUS_OK) || (!IsCmdCrcSucc(cmd, cmdLen))) {
        return BASE_STATUS_ERROR;
    }
    UartAck(ACK_SUCCESS, 0); /* received a valid cmd */
    return BASE_STATUS_OK;
}

/**
  * @brief loop receive a valid cmd.
  * @param cmd. uart command
  * @retval succ fail
  */
static BASE_StatusType UartRcvCmd(UartCmd *cmd)
{
    unsigned int i;
    unsigned int ret;
    unsigned short cmdLen = 0;

    /* loop receive cmd header frame & cmd frame */
    while (1) {
        ret = SERIAL_GetCharTimeout(HEADER_RX_DELAY_US, &cmd->type);
        if (ret != BASE_STATUS_OK) {
            continue;
        }
        switch (cmd->type) {
            case XCMD:
                if (UartGetCmdLength(cmd, &cmdLen) != BASE_STATUS_OK) {
                    continue;
                }
                break;

            case XKEY:
                if (UartGetCmdContext(cmd, cmdLen) != BASE_STATUS_OK) {
                    continue;
                }
                return BASE_STATUS_OK;

            default:
                break;
        }
    }
    return BASE_STATUS_ERROR;
}

/**
  * @brief exec reset cmd.
  * @param cmd uart command
  * @retval succ fail.
  */
static BASE_StatusType LoaderReset(void)
{
    BOOT_Msg0("\nExecution Reset====\n");
    Reset();
    return BASE_STATUS_OK;
}

/**
  * @brief parse cmd.
  * @param cmd. uart command
  * @retval succ fail.
  */
static BASE_StatusType LoaderCmdExec(UartCmd *cmd)
{
    unsigned char cmdType = cmd->rcvBuf[0];
    if (cmdType == CMD_DL_IMAGE) {
        LOADER_VER type = GetLoaderType();
        if (type == LOADERBOOT_V2) {
            return LoaderDownloadImage(cmd);
        } else {
            if (LoaderDownloadImage(cmd) == BASE_STATUS_OK) {
                BASE_FUNC_DELAY_MS(RESET_DELAY);
                LoaderReset();
            }
            return BASE_STATUS_ERROR;
        }
    }

    if (cmdType == CMD_UL_DATA) {
        return LoaderUploadImage(cmd);
    }

    if (cmdType == CMD_RESET) {
        UartAck(ACK_SUCCESS, 0);
        BASE_FUNC_DELAY_MS(RESET_DELAY);
        return LoaderReset();
    }
    return BASE_STATUS_ERROR;
}

/**
  * @brief loop receive & parse & exec cmd.
  * @param .
  * @retval .
  */
void CmdLoop(void)
{
    unsigned int ret;
    UartCmd cmd;

    for (;;) {
        /* recevice cmd header frame */
        UartRcvCmd(&cmd);
        /* parse cmd & exec */
        ret = LoaderCmdExec(&cmd);
        if (ret != BASE_STATUS_OK) {
            UartAck(ACK_FAILURE, 0);
            continue;
        }
    }
}
