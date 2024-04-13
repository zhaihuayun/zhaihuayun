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
  * @file      uart_adapt.c
  * @author    MCU Driver Team
  * @brief     UART adapt module.
  * @details   This file provides functions about the following functionalities:
  *                 + UART Init/DeInit
  *                 + Receive Data from UART
  *                 + Send Data to UART
  */
#include "loaderboot.h"
#include "uart.h"
#include "uart_adapt.h"

#define UART_GETC_DELAY_TIME    20
#define BITS_PER_BYTE           8
#define DECIMAL                 10

static UART_Handle g_uartHandle;
#define BOOT_UART UART0

/**
 * @brief UART Init Adapter
 * @param baudRate Uart baud rate
 * @retval None
 */
void UART_Init(unsigned int baudRate)
{
    UART_Handle *handle = &g_uartHandle;

    handle->baseAddress = BOOT_UART;
    handle->baudRate = baudRate;
    handle->dataLength = UART_DATALENGTH_8BIT;
    handle->stopBits = UART_STOPBITS_ONE;
    handle->parity = UART_PARITY_NONE;
    handle->txMode = UART_MODE_BLOCKING;
    handle->rxMode = UART_MODE_BLOCKING;
    handle->fifoMode = true;
    handle->fifoTxThr = UART_FIFOFULL_ONE_EIGHT;
    handle->fifoRxThr = UART_FIFOFULL_ONE_EIGHT;
    handle->hwFlowCtr = UART_HW_FLOWCTR_DISABLE;

    HAL_UART_Init(handle);
}

/**
  * @brief Write char to UART TX FIFO
  * @param c Character to be written
  * @retval None
  */
static inline void DwPutChar(char c)
{
    while (BOOT_UART->UART_FR.BIT.txff != 0) {    /* True when the TX FIFO is full */
        ;
    }
    BOOT_UART->UART_DR.BIT.data = (unsigned char)c;
}

/**
  * @brief  Check if the RX FIFO is not empty
  * @param  None
  * @retval 0(empty) or 1(not empty)
  */
static inline bool DwTstChar(void)
{
    return (BOOT_UART->UART_FR.BIT.rxfe == 1) ? false : true;
}

/**
  * @brief  Read one character from RX FIFO
  * @param  None
  * @retval Character read
  */
static inline unsigned char DwGetChar(void)
{
    while (!DwTstChar()) {
        ;
    }
    return BOOT_UART->UART_DR.BIT.data;
}

/**
  * @brief  Check if the UART is in mute mode, always return false
  * @param  None
  * @retval false
  */
static inline bool UartCheckMute(void)
{
    return false;
}

/**
  * @brief Write a character to the UART
  * @param c Character to be written
  * @retval None
  */
void SERIAL_PutChar(const char c)
{
    bool isMute = UartCheckMute();
    if (isMute == true) {
        return;
    }

    if (c == '\n') {
        DwPutChar('\r');
    }

    DwPutChar(c);
}

/**
  * @brief Write string to UART
  * @param str String to be written
  * @retval None
  */
void SERIAL_PutStr(const char *str)
{
    bool isMute = UartCheckMute();
    const char *p = str;
    if (str == NULL) {
        return;
    }
    if (isMute == true) {
        return;
    }
    while (*p != 0) {
        SERIAL_PutChar(*p++);
    }
}

/**
  * @brief Write data in buffer to UART
  * @param buffer Pointer to data
  * @param length Data length
  * @retval None
  */
void SERIAL_PutBuf(const char *buffer, signed int length)
{
    signed int i;
    bool isMute;
    if (buffer == NULL) {
        return;
    }
    isMute = UartCheckMute();
    if (isMute == true) {
        return;
    }

    for (i = 0; i < length; i++) {
        DwPutChar(buffer[i]);
    }
}

/**
  * @brief  Get character from the serial port
  * @param  None
  * @retval Character
  */
unsigned char SERIAL_GetChar(void)
{
    return DwGetChar();
}

/**
  * @brief  Get character from the serial port
  * @param  None
  * @retval true  Rcv Data
  * @retval false No Data
  */
bool SERIAL_TstChar(void)
{
    return DwTstChar();
}

/**
  * @brief Output to serial port in hexadecimal format
  * @param h  Integer to be output
  * @param printAll  Whether the left side is filled with 0
  * @retval None
  */
void SERIAL_PutHex(unsigned int hex, bool printAll)
{
    signed int i;
    unsigned int h = hex;
    char c;
    char mark = 0;

    bool isMute = UartCheckMute();
    if (isMute == true) {
        return;
    }

    SERIAL_PutStr("0x");

    for (i = 0; i < BITS_PER_BYTE; i++) {
        c = (h >> 28) & 0x0F; /* u32 right shift 28 */

        if (c >= DECIMAL) {
            c = (c - DECIMAL) + 'A';
        } else {
            c = c + '0';
        }

        if (printAll) {
            SERIAL_PutChar(c);
            h = h << 4; /* u32 left shift 4 */
            continue;
        }

        /* If it is not the last number and the previous numbers are all 0 */
        if ((mark == 0) && (i != BITS_PER_BYTE - 1)) {
            if (c != '0') {
                mark = 1;
                SERIAL_PutChar(c);
            }
        } else {
            SERIAL_PutChar(c);
        }

        h = h << 4; /* u32 left shift 4 */
    }
}

/**
  * @brief Read a character from the serial port.
  *        if the reading fails, return with timeout
  * @param timeoutUs overtime time(us)
  * @param ch    Pointer to read character
  * @retval BASE_STATUS_OK (success)
  * @retval BASE_STATUS_ERROR (fail)
  */
unsigned int SERIAL_GetCharTimeout(unsigned int timeoutUs, unsigned char *ch)
{
    unsigned int maxCnt = timeoutUs / (unsigned int)UART_GETC_DELAY_TIME;
    unsigned int cnt = 0;

    if (ch == NULL) {
        return BASE_STATUS_ERROR;
    }
    while ((SERIAL_TstChar() == false) && (cnt < maxCnt)) {
        BASE_FUNC_DELAY_US(UART_GETC_DELAY_TIME);
        cnt++;
    }

    if (SERIAL_TstChar() == true) {
        *ch = SERIAL_GetChar();
        return BASE_STATUS_OK;
    }

    return BASE_STATUS_ERROR;
}

/**
  * @brief Output formatted hexadecimal number
  * @param h  Value
  * @param print0 Output type
  * @retval None
  */
void BOOT_PutHex(unsigned int h, bool print0)
{
    SERIAL_PutStr(" ");
    SERIAL_PutHex(h, print0);
    SERIAL_PutStr("\r\n");
}

/**
  * @brief Output one string with newline
  * @param s String
  * @retval None
  */
void BOOT_Msg0(const char *s)
{
    if (s == NULL) {
        return;
    }
    SERIAL_PutStr(s);
    SERIAL_PutStr("\r\n");
}

/**
  * @brief Output one string and one hexadecimal number with newline
  * @param s String
  * @param h unsigned integer parameter printed out in hexadecimal
  * @retval None
  */
void BOOT_Msg1(const char *s, unsigned int h)
{
    if (s == NULL) {
        return;
    }
    SERIAL_PutStr(s);
    SERIAL_PutHex(h, 0);
    SERIAL_PutStr("\r\n");
}

/**
  * @brief Output one string and two hexadecimal number with newline
  * @param s String
  * @param h1 unsigned integer parameter 1 printed out in hexadecimal
  * @param h2 unsigned integer parameter 2 printed out in hexadecimal
  * @retval None
  */
void BOOT_Msg2(const char *s, unsigned int h1, unsigned int h2)
{
    if (s == NULL) {
        return;
    }
    SERIAL_PutStr(s);
    SERIAL_PutHex(h1, 0);
    SERIAL_PutStr(" ");
    SERIAL_PutHex(h2, 0);
    SERIAL_PutStr("\r\n");
}

/**
  * @brief Output one string and four hexadecimal number with newline
  * @param s String
  * @param h1 unsigned integer parameter 1 printed out in hexadecimal
  * @param h2 unsigned integer parameter 2 printed out in hexadecimal
  * @param h3 unsigned integer parameter 3 printed out in hexadecimal
  * @param h4 unsigned integer parameter 4 printed out in hexadecimal
  * @retval None
  */
void BOOT_Msg4(const char *s, unsigned int h1, unsigned int h2, unsigned int h3, unsigned int h4)
{
    if (s == NULL) {
        return;
    }
    SERIAL_PutStr(s);
    SERIAL_PutHex(h1, 0);
    SERIAL_PutStr(" ");
    SERIAL_PutHex(h2, 0);
    SERIAL_PutStr(" ");
    SERIAL_PutHex(h3, 0);
    SERIAL_PutStr(" ");
    SERIAL_PutHex(h4, 0);
    SERIAL_PutStr("\r\n");
}
