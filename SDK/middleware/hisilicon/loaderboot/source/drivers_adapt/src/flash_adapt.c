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
  * @file      flash_adapt.c
  * @author    MCU Driver Team
  * @brief     FLASH adapt module.
  * @details   This file provides functions about the following functionalities:
  *                 + Flash Init
  *                 + Flash Write
  *                 + Flash Read
  *                 + Flash Erase
  */
#include "flash.h"
#include "flash_adapt.h"

FLASH_Handle g_flashBlockingHandle;

/**
 * @brief Flash Init Adapter
 * @param None
 * @retval None
 */
void FLASH_Init(void)
{
    g_flashBlockingHandle.baseAddress = (EFC_RegStruct *)EFC_BASE;
    g_flashBlockingHandle.peMode = FLASH_PE_OP_BLOCK;

    HAL_FLASH_Init(&g_flashBlockingHandle);
}

/**
 * @brief Flash Erase Adapter
 * @param addr The earse address
 * @param len  The erase length
 * @retval BASE_StatusType
 */
BASE_StatusType FLASH_Erase(unsigned int addr, unsigned int len)
{
    unsigned int pageNum = (len + FLASH_ONE_PAGE_SIZE - 1) / FLASH_ONE_PAGE_SIZE;
    return HAL_FLASH_EraseBlocking(&g_flashBlockingHandle, FLASH_ERASE_MODE_PAGE, addr, pageNum);
}

/**
 * @brief Flash Write Adapter
 * @param buf  The data to be written
 * @param dest The write address
 * @param len  The write length
 * @retval BASE_StatusType
 */
BASE_StatusType FLASH_Write(unsigned char *buf, unsigned int dest, unsigned int rcvLen)
{
    if (buf == NULL) {
        return BASE_STATUS_ERROR;
    }
    return HAL_FLASH_WriteBlocking(&g_flashBlockingHandle, (unsigned int)(uintptr_t)buf, dest, rcvLen);
}

/**
 * @brief Flash Read Adapter
 * @param srcAddr  Read address
 * @param readLen  Read length
 * @param dataBuff Buffer to keep the read out data
 * @param buffLen  Sizeof buffer
 * @retval BASE_StatusType
 */
BASE_StatusType FLASH_Read(const unsigned int srcAddr, unsigned int readLen,
                           unsigned char *dataBuff, const unsigned int buffLen)
{
    unsigned int *ptemp = NULL;
    unsigned int *dst = NULL;
    unsigned int len = readLen;

    if (dataBuff == NULL) {
        return BASE_STATUS_ERROR;
    }
    dst = (unsigned int *)dataBuff;

    ptemp = (unsigned int *)(uintptr_t)srcAddr;
    if (readLen > buffLen) {
        return BASE_STATUS_ERROR;
    }
    len /= sizeof(unsigned int);
    while (len--) {
        *dst++ = *ptemp++;
    }
    return BASE_STATUS_OK;
}