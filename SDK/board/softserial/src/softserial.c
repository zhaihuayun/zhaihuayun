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
  * @file      softserial.h
  * @author    MCU Application Team
  * @brief     Soft Serial module application.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the dimming.
  *             + Initialization and de-initialization functions
  *             + Serial Port Transmitting and Receiving Function
  * @verbatim
  * usage:
  * Sending Function：
  * 1) Use BOARD_SOFTSERIAL_TxInit() to initialize the serial port sending function.
  * 2) Use BOARD_SOFTSERIAL_PrintCh() to send characters.
  * Receiving Function：
  * 1) Use BOARD_SOFTSERIAL_RxInit() to initialize the serial port receiving function.
  * 2) Use BOARD_SOFTSERIAL_GetChar() to receive characters.
  * @endverbatim
  */

/* Includes ------------------------------------------------------------------ */
#include "softserial.h"

/* Private variables --------------------------------------------------------- */
BOARD_SOFTSERIAL_TxHandle g_txHandle;
BOARD_SOFTSERIAL_RxHandle g_rxHandle;
/* Function declare --------------------------------------------------------- */
static void TxDataBitCntProcess(TIMER_Handle *handle);

static unsigned char BOARD_SOFTSERIAL_GetBitOneNum(unsigned int x);

/**
  * @brief Calculates the number of bits 1 in the specified data.
  * @param x Data to be calculated.
  * @return unsigned char Number of bits 1.
  */
static unsigned char BOARD_SOFTSERIAL_GetBitOneNum(unsigned int x)
{
    unsigned char ret = 0;
    while (x > 0) {
        ret++;
        x &= (x - 1);
    }

    return ret;
}

/**
  * @brief Initialize the transmission of the soft serial port.
  * @param gpioHandle GPIO for output.
  * @param timerHandle Timer for timing based on baud rate.
  * @param baudRate Baud rate of communication.
  * @param dataLength @ref BOARD_SOFTSERIAL_DataLength.
  * @param parityMode @ref BOARD_SOFTSERIAL_ParityMode.
  * @param stopBits @ref BOARD_SOFTSERIAL_StopBits.
  */
void BOARD_SOFTSERIAL_TxInit(GPIO_Handle *gpioHandle, TIMER_Handle *timerHandle, unsigned int baudRate,
    BOARD_SOFTSERIAL_DataLength dataLength, BOARD_SOFTSERIAL_ParityMode parityMode, BOARD_SOFTSERIAL_StopBits stopBits)
{
    SOFTSERIAL_ASSERT_PARAM(gpioHandle != NULL);
    SOFTSERIAL_ASSERT_PARAM(timerHandle != NULL);
    SOFTSERIAL_PARAM_CHECK_NO_RET(baudRate > 0);

    g_txHandle.dataLength = dataLength;
    g_txHandle.parityMode = parityMode;
    g_txHandle.stopBits = stopBits;
    g_txHandle.writeIndex = 0;
    g_txHandle.readIndex = 0;
    g_txHandle.gpioHandle = gpioHandle;
    g_txHandle.timerHandle = timerHandle;

    g_txHandle.txDataFrameLength = 1 + dataLength + stopBits;
    if (parityMode != BOARD_SOFTSERIAL_PARITY_NONE) {
        g_txHandle.txDataFrameLength++;
    }

    g_txHandle.txDataBitCnt = g_txHandle.txDataFrameLength;

    HAL_GPIO_SetDirection(gpioHandle, gpioHandle->pins, GPIO_OUTPUT_MODE);
    HAL_GPIO_SetValue(gpioHandle, gpioHandle->pins, GPIO_HIGH_LEVEL);

    timerHandle->bgLoad = HAL_CRG_GetIpFreq((void *)timerHandle->baseAddress) / baudRate;
    timerHandle->load = timerHandle->bgLoad;

    HAL_TIMER_Config(timerHandle, TIMER_CFG_BGLOAD);
    HAL_TIMER_Config(timerHandle, TIMER_CFG_LOAD);

    HAL_TIMER_RegisterCallback(timerHandle, BOARD_SOFTSERIAL_TxTimerCallBack);
}

/**
  * @brief Output character.
  * @param ch Characters to be output.
  * @return BOARD_SOFTSERIAL_Ret Return Status.
  */
BOARD_SOFTSERIAL_Ret BOARD_SOFTSERIAL_PrintCh(unsigned char ch)
{
    if ((g_txHandle.writeIndex + 1) % BOARD_SOFTSERIAL_TX_BUFFER_SIZE == g_txHandle.readIndex) {
        return BOARD_SOFTSERIAL_ERR_TX_BUFFER_FULL;
    }

    g_txHandle.txBuffer[g_txHandle.writeIndex] = ch;
    g_txHandle.writeIndex = (g_txHandle.writeIndex + 1) % BOARD_SOFTSERIAL_TX_BUFFER_SIZE;

    HAL_TIMER_Start(g_txHandle.timerHandle);

    return BOARD_SOFTSERIAL_OK;
}

/**
  * @brief process of txBitCnt > txDataFrameLength.
  * @param None.
  * @return None.
  */
static void TxDataBitCntProcess(TIMER_Handle *handle)
{
    if (g_txHandle.writeIndex == g_txHandle.readIndex) {
        DCL_GPIO_SetValue(g_txHandle.gpioHandle->baseAddress, g_txHandle.gpioHandle->pins, GPIO_HIGH_LEVEL);
        HAL_TIMER_Stop(handle);
        return;
    }

    g_txHandle.txDataBitCnt = 0;
    /* 1 : stop bit check */
    g_txHandle.txDataFrame = 1;
    if (g_txHandle.stopBits == BOARD_SOFTSERIAL_STOPBITS_TWO) {
        g_txHandle.txDataFrame <<= 1;
        g_txHandle.txDataFrame++;
    }

    if (g_txHandle.parityMode != BOARD_SOFTSERIAL_PARITY_NONE) {
        g_txHandle.txDataFrame <<= 1;
        /* 2 : Calculating Data Parity */
        unsigned char tmp = BOARD_SOFTSERIAL_GetBitOneNum(g_txHandle.txBuffer[g_txHandle.readIndex]) % 2;
        if ((tmp == 1 && g_txHandle.parityMode == BOARD_SOFTSERIAL_PARITY_ODD) ||
            (tmp == 0 && g_txHandle.parityMode == BOARD_SOFTSERIAL_PARITY_EVEN)) {
            g_txHandle.txDataFrame++;
        }
    }
    /* 3 : process tx data */
    g_txHandle.txDataFrame <<= g_txHandle.dataLength;
    g_txHandle.txDataFrame += g_txHandle.txBuffer[g_txHandle.readIndex];
    g_txHandle.readIndex = (g_txHandle.readIndex + 1) % BOARD_SOFTSERIAL_TX_BUFFER_SIZE;

    g_txHandle.txDataFrame <<= 1;
}

/**
  * @brief Timer callback function.
  * @param param Pointer to the timer handle.
  */
void BOARD_SOFTSERIAL_TxTimerCallBack(void *param)
{
    TIMER_Handle *handle = (TIMER_Handle *)param;
    HAL_TIMER_IrqClear(handle);

    if (g_txHandle.txDataBitCnt >= g_txHandle.txDataFrameLength) {
        TxDataBitCntProcess(handle);
    }

    if ((g_txHandle.txDataFrame & 0x01) == 1) {
        DCL_GPIO_SetValue(g_txHandle.gpioHandle->baseAddress, g_txHandle.gpioHandle->pins, GPIO_HIGH_LEVEL);
    } else {
        DCL_GPIO_SetValue(g_txHandle.gpioHandle->baseAddress, g_txHandle.gpioHandle->pins, GPIO_LOW_LEVEL);
    }

    g_txHandle.txDataFrame >>= 1;
    g_txHandle.txDataBitCnt++;
}

/**
  * @brief Initialize the reception of the soft serial port.
  * @param gpioHandle GPIO for input.
  * @param timerHandle Timer for timing based on baud rate.
  * @param baudRate Baud rate of communication.
  * @param dataLength @ref BOARD_SOFTSERIAL_DataLength.
  * @param parityMode @ref BOARD_SOFTSERIAL_ParityMode.
  * @param stopBits @ref BOARD_SOFTSERIAL_StopBits.
  */
void BOARD_SOFTSERIAL_RxInit(GPIO_Handle *gpioHandle, TIMER_Handle *timerHandle, unsigned int baudRate,
    BOARD_SOFTSERIAL_DataLength dataLength, BOARD_SOFTSERIAL_ParityMode parityMode, BOARD_SOFTSERIAL_StopBits stopBits)
{
    SOFTSERIAL_ASSERT_PARAM(gpioHandle != NULL);
    SOFTSERIAL_ASSERT_PARAM(timerHandle != NULL);

    g_rxHandle.dataLength = dataLength;
    g_rxHandle.parityMode = parityMode;
    g_rxHandle.stopBits = stopBits;
    g_rxHandle.gpioHandle = gpioHandle;
    g_rxHandle.timerHandle = timerHandle;
    g_rxHandle.samplingValue = 0;
    g_rxHandle.samplingCntThisBit = 0;

    g_rxHandle.rxDataFrameLength = 1 + dataLength + stopBits;
    if (parityMode != BOARD_SOFTSERIAL_PARITY_NONE) {
        g_rxHandle.rxDataFrameLength++;
    }

    g_rxHandle.writeIndex = 0;
    g_rxHandle.readIndex = 0;

    g_rxHandle.rxDataBitCnt = 0;
    g_rxHandle.rxBuffer[g_rxHandle.writeIndex] = 0;

    HAL_GPIO_SetDirection(gpioHandle, gpioHandle->pins, GPIO_INPUT_MODE);
    HAL_GPIO_SetIrqType(gpioHandle, gpioHandle->pins, GPIO_INT_TYPE_FALL_EDGE);
    HAL_GPIO_RegisterCallBack(gpioHandle, gpioHandle->pins, BOARD_SOFTSERIAL_RxGpioCallBack);

    timerHandle->bgLoad =
        HAL_CRG_GetIpFreq((void *)timerHandle->baseAddress) / (baudRate * BOARD_SOFTSERIAL_RX_SAMPLING_CNT_PER_BIT);
    timerHandle->load = timerHandle->bgLoad;
    HAL_TIMER_Config(timerHandle, TIMER_CFG_BGLOAD);
    HAL_TIMER_Config(timerHandle, TIMER_CFG_LOAD);
    HAL_TIMER_RegisterCallback(timerHandle, BOARD_SOFTSERIAL_RxTimerCallBack);
}

/**
  * @brief Gpio callback function.
  * @param param Pointer to the gpio handle.
  */
void BOARD_SOFTSERIAL_RxGpioCallBack(void *param)
{
    GPIO_Handle *handle = (GPIO_Handle *)param;
    DCL_GPIO_DisableIrq(g_rxHandle.gpioHandle->baseAddress, g_rxHandle.gpioHandle->pins);
    HAL_TIMER_Start(g_rxHandle.timerHandle);
}

/**
  * @brief Timer callback function.
  * @param param Pointer to the timer handle.
  */
void BOARD_SOFTSERIAL_RxTimerCallBack(void *param)
{
    TIMER_Handle *handle = (TIMER_Handle *)param;
    HAL_TIMER_IrqClear(handle);

    if (HAL_GPIO_GetPinValue(g_rxHandle.gpioHandle, g_rxHandle.gpioHandle->pins) == GPIO_HIGH_LEVEL) {
        g_rxHandle.samplingValue++;
    } else {
        g_rxHandle.samplingValue--;
    }

    g_rxHandle.samplingCntThisBit++;
    if (g_rxHandle.samplingCntThisBit >= BOARD_SOFTSERIAL_RX_SAMPLING_CNT_PER_BIT) {
        g_rxHandle.rxBuffer[g_rxHandle.writeIndex] <<= 1;
        if (g_rxHandle.samplingValue > 0) {
            g_rxHandle.rxBuffer[g_rxHandle.writeIndex]++;
        }

        g_rxHandle.samplingCntThisBit = 0;
        g_rxHandle.samplingValue = 0;

        g_rxHandle.rxDataBitCnt++;

        if (g_rxHandle.rxDataBitCnt >= g_rxHandle.rxDataFrameLength) {
            g_rxHandle.rxDataBitCnt = 0;
            g_rxHandle.writeIndex = (g_rxHandle.writeIndex + 1) % BOARD_SOFTSERIAL_RX_BUFFER_SIZE;
            g_rxHandle.rxBuffer[g_rxHandle.writeIndex] = 0;

            HAL_TIMER_Stop(g_rxHandle.timerHandle);
            DCL_GPIO_EnableIrq(g_rxHandle.gpioHandle->baseAddress, g_rxHandle.gpioHandle->pins);
        }
    }
}

/**
  * @brief Read characters from buffer.
  * @param pData Output Read Value.
  * @return BOARD_SOFTSERIAL_Ret Return Status.
  */
BOARD_SOFTSERIAL_Ret BOARD_SOFTSERIAL_GetChar(unsigned char *pData)
{
    *pData = 0;

    if (g_rxHandle.writeIndex == g_rxHandle.readIndex) {
        return BOARD_SOFTSERIAL_ERR_RX_BUFFER_NULL;
    }

    unsigned short tmp = g_rxHandle.rxBuffer[g_rxHandle.readIndex];

    g_rxHandle.readIndex = (g_rxHandle.readIndex + 1) % BOARD_SOFTSERIAL_RX_BUFFER_SIZE;

    for (unsigned int i = 0; i < g_rxHandle.stopBits; i++) {
        if ((tmp & 0x01) != 0x01) {
            return BOARD_SOFTSERIAL_ERR_STOP_BIT;
        }
        tmp >>= 1;
    }

    if (g_txHandle.parityMode != BOARD_SOFTSERIAL_PARITY_NONE) {
        unsigned char parityFlag =
            (BOARD_SOFTSERIAL_GetBitOneNum((unsigned int)*pData) + (tmp & 0x01)) % 2; /* 2 : Calculating Data Parity */
        tmp >>= 1;
        if ((g_txHandle.parityMode == BOARD_SOFTSERIAL_PARITY_ODD && parityFlag == 1) ||
            (g_txHandle.parityMode == BOARD_SOFTSERIAL_PARITY_EVEN && parityFlag == 0)) {
            return BOARD_SOFTSERIAL_ERR_PARITY_ERR;
        }
    }

    for (unsigned int i = 0; i < g_rxHandle.dataLength; i++) {
        *pData <<= 1;
        *pData |= (tmp & 0x01);
        tmp >>= 1;
    }

    return BOARD_SOFTSERIAL_OK;
}