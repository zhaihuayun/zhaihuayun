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
 * @file    utils.h
 * @author  MCU Driver Team
 * @brief   Header file containing typedef and function of utils.
 */

#ifndef McuMagicTag_UTILS_H
#define McuMagicTag_UTILS_H

/*
 * byte and bit offset in unsigned int
 */
#define BYTE_0            0
#define BYTE_0_OFFSET     24
#define BYTE_1            1
#define BYTE_1_OFFSET     16
#define BYTE_2            2
#define BYTE_2_OFFSET     8
#define BYTE_3 3

#define OFFSETOF(struct_t, member) ((unsigned int)&((struct_t *)0)->member)

/**
  * @brief Get the Min Value
  * @param a number 1
  * @param b number 2
  * @retval int The min Value
  */
static inline unsigned int GetMinValue(unsigned int a, unsigned int b)
{
    return a < b ? a : b;
}

/**
  * @brief Merge two byte into unsigned int short
  * @param highByte Byte in higher
  * @param lowByte  Byte in lower
  * @retval unsigned short Converted value
  */
static inline unsigned short MergeToUshort(unsigned char highByte, unsigned char lowByte)
{
    return (highByte << 0x8) + lowByte;  /* Make highByte in the high 8 bit */
}

/**
  * @brief Endian conversion
  * @param buf Perform a pointer of the char type of a 32-bit integer
  * @retval unsigned int  Converted value
  */
static inline unsigned int BigEndianToUint(const unsigned char *buf)
{
    unsigned int i = 0;
    unsigned int sum;
    if (buf == NULL) {
        return 0;
    }
    sum = buf[i++] << BYTE_0_OFFSET;
    sum += buf[i++] << BYTE_1_OFFSET;
    sum += buf[i++] << BYTE_2_OFFSET;
    sum += buf[i++];
    return sum;
}

/**
  * @brief Host sequence to network byte sequence conversion
  * @param data Integer to be converted
  * @param buf  The result of the conversion
  * @retval None
  */
static inline void UintToBigEndian(unsigned int data, unsigned char *const buf)
{
    unsigned int i = 0;
    buf[i++] = (unsigned char)((data >> BYTE_0_OFFSET) & 0xFF);
    buf[i++] = (unsigned char)((data >> BYTE_1_OFFSET) & 0xFF);
    buf[i++] = (unsigned char)((data >> BYTE_2_OFFSET) & 0xFF);
    buf[i] = (unsigned char)(data & 0xFF);
}

/**
  * @brief Host sequence to network byte sequence conversion
  * @param data Unsigned short to be converted
  * @param buf  The result of the conversion
  * @retval None
  */
static inline void UShortToBigEndian(unsigned short data, unsigned char *const buf)
{
    unsigned int i = 0;
    buf[i++] = (unsigned char)((data >> 0x8) & 0xFF);   /* get the higher 8 bit of unsigned short */
    buf[i] = (unsigned char)(data & 0xFF);
}

#endif