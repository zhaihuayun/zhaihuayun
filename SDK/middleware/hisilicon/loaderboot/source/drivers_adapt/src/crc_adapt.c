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
  * @file      crc_adapt.c
  * @author    MCU Driver Team
  * @brief     CRC adapt module.
  * @details   This file provides functions about the following functionalities:
  *                 + Generate CRC base CRC16
  *                 + Check CRC
  */

/* Includes ------------------------------------------------------------------*/
#include "crc.h"
#include "crc_adapt.h"

/* Macro definitions ---------------------------------------------------------*/

/* Typedef definitions -------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
CRC_Handle crcHandle;

/**
 * @brief CRC Init Adapter
 * @param None
 * @retval None
 */
void CRC_Init(void)
{
    crcHandle.baseAddress = CRC;
    crcHandle.algoMode = CRC16_XMODEM;
    crcHandle.inputDataFormat = CRC_MODE_BIT8; /* Valid bits provided for CRC */
    crcHandle.timeout = CRC_8_BIT;
    crcHandle.enableIT = false;
    crcHandle.enableErrInject = false;

    HAL_CRC_Init(&crcHandle);
}

/**
  * @brief Generate CRC of buf.
  * @param buf     Data buffer which need generate CRC
  * @param len     Data buffer length
  * @retval The Generated CRC
  * @note   crcStart must be 0 or 0xFFFF
  */
unsigned int CRC_Generate(const unsigned char *buf, unsigned int len)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcHandle.baseAddress));
    CRC_ASSERT_PARAM(buf != NULL);
    return HAL_CRC_Calculate(&crcHandle, buf, len);
}

/**
  * @brief Check the CRC
  * @param buf     Data buffer which need generate CRC
  * @param len     Data buffer length
  * @param crc     the expect crc
  * @retval true CRC OK
  * @retval false CRC ERROR
  */
bool CRC_Check(const unsigned char *buf, unsigned int len, unsigned int crc)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcHandle.baseAddress));
    CRC_ASSERT_PARAM(buf != NULL);
    return HAL_CRC_CheckInputData(&crcHandle, buf, len, crc);
}


/**
  * @brief Check the CRC with seed
  * @param buf     Data buffer which need generate CRC
  * @param len     Data buffer length
  * @param crc     the expect crc
  * @retval true CRC OK
  * @retval false CRC ERROR
  * @note   Must follow the CRC Generate, The Output of CRC_Generate is the input of CRC_AccCheck
  */
bool CRC_AccCheck(const unsigned char *buf, unsigned int len, unsigned int crc)
{
    CRC_ASSERT_PARAM(IsCRCInstance(crcHandle.baseAddress));
    CRC_ASSERT_PARAM(buf != NULL);
    return HAL_CRC_Accumulate(&crcHandle, buf, len) == crc;
}
