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
  * @file    uart_adapt.h
  * @author  MCU Driver Team
  * @brief   Header file containing adapter functions of uart
  */

#ifndef McuMagicTag_UART_ADAPT_H
#define McuMagicTag_UART_ADAPT_H

void UART_Init(unsigned int baudRate);

void UART_DeInit(void);

void BOOT_PutHex(unsigned int h, bool print0);

void BOOT_Msg0(const char *s);

void BOOT_Msg1(const char *s, unsigned int h);

void BOOT_Msg2(const char *s, unsigned int h1, unsigned int h2);

void BOOT_Msg4(const char *s, unsigned int h1, unsigned int h2, unsigned int h3, unsigned int h4);

void SERIAL_SetMute(void);

void SERIAL_CancelMute(void);

void SERIAL_PutChar(const char c);

void SERIAL_PutStr(const char *s);

void SERIAL_PutBuf(const char *buffer, signed int length);

unsigned char SERIAL_GetChar(void);

bool SERIAL_TstChar(void);

void SERIAL_PutHex(unsigned int h, bool printAll);

unsigned int SERIAL_GetCharTimeout(unsigned int timeout_us, unsigned char *ch);

#endif
