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
  * @file    spi.c
  * @author  MCU Driver Team
  * @brief   SPI module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the SPI.
  *          + Initialization and de-initialization functions
  *          + Peripheral Control functions
  */

/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "spi.h"
#include "systick.h"

/* Macro definitions ---------------------------------------------------------*/
#define SPI_WAIT_TIMEOUT   0x400

#define SPI_DATA_WIDTH_SHIFT_8BIT    1
#define SPI_DATA_WIDTH_SHIFT_16BIT   2

#define SPI_INTERRUPT_SET_ALL 0xD
#define SPI_DMA_FIFO_ENABLE   0x3

#define SPI_TICK_MS_DIV       1000

void SPI_RspInit(const SPI_Handle *handle);
void SPI_RspDeInit(const SPI_Handle *handle);

/**
  * @brief Check all initial configuration parameters.
  * @param handle SPI handle.
  * @retval None.
  */
static void CheckAllInitParameters(SPI_Handle *handle)
{
    SPI_ASSERT_PARAM(IsSpiMode(handle->mode));
    SPI_ASSERT_PARAM(IsSpiXferMode(handle->xFerMode));
    SPI_ASSERT_PARAM(IsSpiEndian(handle->endian));
    SPI_ASSERT_PARAM(IsSpiFrameFormat(handle->frameFormat));
    SPI_ASSERT_PARAM(IsSpiDataWidth(handle->dataWidth));

    if (handle->mode == HAL_SPI_MASTER) {
        SPI_ASSERT_PARAM(IsSpiFreqCpsdvsr(handle->freqCpsdvsr));
    }

    if (handle->frameFormat == HAL_SPI_MODE_MOTOROLA) {
        SPI_ASSERT_PARAM(IsSpiClkPolarity(handle->clkPolarity));
        SPI_ASSERT_PARAM(IsSpiClkPhase(handle->clkPhase));
    }

    if (handle->frameFormat == HAL_SPI_MODE_MICROWIRE) {
        SPI_ASSERT_PARAM(IsSpiWaitVal(handle->waitVal));
    }

    if (handle->xFerMode == HAL_XFER_MODE_INTERRUPTS) {
        SPI_ASSERT_PARAM(IsSpiTxIntSize(handle->txIntSize));
        SPI_ASSERT_PARAM(IsSpiRxIntSize(handle->rxIntSize));
    }

    if (handle->xFerMode == HAL_XFER_MODE_DMA) {
        SPI_ASSERT_PARAM(IsSpiTxDmaBurstSize(handle->txDMABurstSize));
        SPI_ASSERT_PARAM(IsSpiRxDmaBurstSize(handle->rxDMABurstSize));
    }
}

/**
  * @brief Internal chip select control.
  * @param handle SPI handle.
  * @param control SPI_CHIP_DESELECT or SPI_CHIP_SELECT
  * @retval None.
  */
static void InternalCsControl(SPI_Handle *handle, unsigned int control)
{
    BASE_FUNC_UNUSED(handle);
    BASE_FUNC_UNUSED(control);
}

/**
  * @brief Chip select control.
  * @param handle SPI handle.
  * @param control SPI_CHIP_DESELECT or SPI_CHIP_SELECT
  * @retval None.
  */
static void SpiCsControl(SPI_Handle *handle, unsigned int control)
{
    if (handle->csMode == SPI_CHIP_SELECT_MODE_INTERNAL) {
        InternalCsControl(handle, control);
    } else {
        if (handle->CsCtrlCallback != NULL) {
            handle->csCtrl = control;
            handle->CsCtrlCallback(handle);
        }
    }
}

/**
  * @brief Writes data from the buffer to the FIFO.
  * @param handle SPI handle.
  * @retval None.
  */
static void WriteData(SPI_Handle *handle)
{
    while (handle->baseAddress->SPISR.BIT.tnf &&
           handle->baseAddress->SPISR.BIT.rff != BASE_CFG_SET &&
           (handle->transferSize > handle->txCount)) {
        if (handle->dataWidth > SPI_DATA_WIDTH_8BIT) {
            /* Only data needs to be read. Due to SPI characteristics,
               data must be transmitted before data can be read. Therefore, 0x0 is transmitted. */
            if (handle->txBuff == NULL) {
                handle->baseAddress->SPIDR.data = 0x0;
                handle->txCount += SPI_DATA_WIDTH_SHIFT_16BIT;
            } else {
                handle->baseAddress->SPIDR.data = *(unsigned short *)handle->txBuff;
                handle->txCount += SPI_DATA_WIDTH_SHIFT_16BIT;
                handle->txBuff += SPI_DATA_WIDTH_SHIFT_16BIT;
            }
        } else {
            if (handle->txBuff == NULL) {
                handle->baseAddress->SPIDR.data = 0x0;
                handle->txCount += SPI_DATA_WIDTH_SHIFT_8BIT;
            } else {
                handle->baseAddress->SPIDR.data = *(unsigned char *)handle->txBuff;
                handle->txCount += SPI_DATA_WIDTH_SHIFT_8BIT;
                handle->txBuff += SPI_DATA_WIDTH_SHIFT_8BIT;
            }
        }
    }
}

/**
  * @brief Reads data from the FIFO to the buffer.
  * @param handle SPI handle.
  * @retval None.
  */
static void ReadData(SPI_Handle *handle)
{
    unsigned short val;

    while (handle->baseAddress->SPISR.BIT.rne && (handle->transferSize > handle->rxCount)) {
        if (handle->dataWidth > SPI_DATA_WIDTH_8BIT) {
            /* When only data is transmitted, the data in the RX FIFO needs to be read. */
            if (handle->rxBuff == NULL) {
                val = handle->baseAddress->SPIDR.data;
                BASE_FUNC_UNUSED(val);
                handle->rxCount += SPI_DATA_WIDTH_SHIFT_16BIT;
            } else {
                *(unsigned short *)handle->rxBuff = handle->baseAddress->SPIDR.data;
                handle->rxCount += SPI_DATA_WIDTH_SHIFT_16BIT;
                handle->rxBuff += SPI_DATA_WIDTH_SHIFT_16BIT;
            }
        } else {
            if (handle->rxBuff == NULL) {
                val = handle->baseAddress->SPIDR.data;
                BASE_FUNC_UNUSED(val);
                handle->rxCount += SPI_DATA_WIDTH_SHIFT_8BIT;
            } else {
                *(unsigned char *)handle->rxBuff = handle->baseAddress->SPIDR.data & 0xff;
                handle->rxCount += SPI_DATA_WIDTH_SHIFT_8BIT;
                handle->rxBuff += SPI_DATA_WIDTH_SHIFT_8BIT;
            }
        }
    }

    while (handle->baseAddress->SPISR.BIT.rne) {
        val = handle->baseAddress->SPIDR.data;
        BASE_FUNC_UNUSED(val);
    }
}

/**
  * @brief Read/write based on input parameters.
  *        The Motorola SPI/TI synchronous serial interface is full-duplex.
  *        Each data is received. Even if only data needs to be transmitted,
  *        the RX FIFO needs to be cleared.
  * @param handle SPI handle.
  * @retval None.
  */
static void ReadWriteData(SPI_Handle *handle)
{
    unsigned int preTick = DCL_SYSTICK_GetTick();
    unsigned int curTick = preTick;
    unsigned long long delta = 0;
    /* Calculate the timeout tick. */
    unsigned long long targetDelta = SYSTICK_GetCRGHZ() / SPI_TICK_MS_DIV * SPI_WAIT_TIMEOUT;
    /* Before writing data to the FIFO, disable the transmission and write more data to the FIFO.
       This enhances communication reliability. */
    handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_UNSET;
    WriteData(handle);
    handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_SET;
    while (true) {
        /* Wait for the write operation to complete */
        if (handle->baseAddress->SPISR.BIT.bsy == BASE_CFG_UNSET &&
            handle->baseAddress->SPISR.BIT.tfe == BASE_CFG_SET &&
            handle->baseAddress->SPISR.BIT.rne == BASE_CFG_SET) {
                break;
            }
        curTick = DCL_SYSTICK_GetTick();
        delta += curTick > preTick ? curTick - preTick : SYSTICK_MAX_VALUE - preTick + curTick;
        /* Exit upon timeout */
        if (delta >= targetDelta) {
            handle->errorCode = BASE_STATUS_TIMEOUT;
            break;
        }
        preTick = curTick;
    }
    ReadData(handle);
}

/**
  * @brief Blocking read/write data processing.
  * @param handle SPI handle.
  * @retval None.
  */
static void ReadWriteBlocking(SPI_Handle *handle)
{
    unsigned int preTick = DCL_SYSTICK_GetTick();
    unsigned int curTick = preTick;
    unsigned long long delta = 0;
    unsigned long long targetDelta = SYSTICK_GetCRGHZ() / SPI_TICK_MS_DIV * handle->timeout;

    SpiCsControl(handle, SPI_CHIP_SELECT);
    if (!handle->baseAddress->SPICR1.BIT.sse) {
        handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_SET;
    }

    while (handle->transferSize > handle->txCount || handle->transferSize > handle->rxCount) {
        curTick = DCL_SYSTICK_GetTick();
        delta += curTick > preTick ? curTick - preTick : SYSTICK_MAX_VALUE - preTick + curTick;
        if (delta >= targetDelta) {
            handle->errorCode = BASE_STATUS_TIMEOUT;
            break;
        }
        ReadWriteData(handle);
        preTick = curTick;
    }

    SpiCsControl(handle, SPI_CHIP_DESELECT);
    handle->state = HAL_SPI_STATE_READY;
}

/**
  * @brief Interrupt read/write enable control.
  * @param handle SPI handle.
  * @retval None.
  */
static void ReadWriteInterruptEnable(SPI_Handle *handle)
{
    SpiCsControl(handle, SPI_CHIP_SELECT);
    if (!handle->baseAddress->SPICR1.BIT.sse) {
        handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_SET;
    }
    handle->baseAddress->SPIIMSC.reg = SPI_INTERRUPT_SET_ALL;
}

/**
 * @brief SPI read/write parameter configuration.
 * @param handle SPI handle.
 * @param rData Address of the data buff to be Receiving.
 * @param wData Address of the data buff to be sent.
 * @param dataSiz Number of the data to be Receivingd and sent.
 * @retval None.
 */
static void ConfigTransmissionParameter(SPI_Handle *handle,
                                        unsigned char *rData,
                                        unsigned char *wData,
                                        unsigned int dataSize)
{
    handle->errorCode = BASE_STATUS_OK;
    handle->rxBuff = rData;
    handle->txBuff = wData;
    if (handle->dataWidth > SPI_DATA_WIDTH_8BIT &&
        handle->xFerMode == HAL_XFER_MODE_DMA) {
        handle->transferSize = dataSize / 2; /* Processes 2 bytes at a time */
    } else {
        handle->transferSize = dataSize;
    }
    handle->txCount = 0;
    handle->rxCount = 0;
}

__weak void SPI_RspInit(const SPI_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
}

__weak void SPI_RspDeInit(const SPI_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
}

/**
  * @brief Initializing the SPI Module.
  * @param handle SPI handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_Init(SPI_Handle *handle)
{
    unsigned int cr0Reg;
    unsigned int temp;
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    CheckAllInitParameters(handle);

    SPI_RspInit(handle);

    handle->state = HAL_SPI_STATE_BUSY;

    handle->baseAddress->SPICR1.BIT.lbm = BASE_CFG_UNSET;
    handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_UNSET;
    handle->baseAddress->SPICR1.BIT.bitend = handle->endian;
    handle->baseAddress->SPICR1.BIT.ms = handle->mode;
    handle->baseAddress->SPICR1.BIT.mode_altasens = BASE_CFG_UNSET;

    temp = ((unsigned int)handle->freqScr) << SPI_CR0_SCR_POS;
    cr0Reg = (handle->baseAddress->SPICR0.reg & (~SPI_CR0_SCR_MASK)) | temp;
    handle->baseAddress->SPICR0.reg = cr0Reg;
    /* Modulo 2 to get an even number */
    if ((handle->freqCpsdvsr % 2) == 0) {
        handle->baseAddress->SPICPSR.BIT.cpsdvsr = handle->freqCpsdvsr;
    } else {
        handle->baseAddress->SPICPSR.BIT.cpsdvsr = handle->freqCpsdvsr - 1;
    }

    handle->baseAddress->SPICR0.BIT.sph = handle->clkPhase;
    handle->baseAddress->SPICR0.BIT.spo = handle->clkPolarity;

    handle->baseAddress->SPICR0.BIT.frf = handle->frameFormat;
    handle->baseAddress->SPICR0.BIT.dss = handle->dataWidth;

    if ((handle->frameFormat == HAL_SPI_MODE_MICROWIRE) && (handle->waitEn == BASE_CFG_ENABLE)) {
        handle->baseAddress->SPICR1.BIT.waitval = handle->waitVal;
        handle->baseAddress->SPICR1.BIT.waiten = BASE_CFG_SET;
    } else {
        handle->baseAddress->SPICR1.BIT.waiten = BASE_CFG_UNSET;
    }

    if (handle->xFerMode == HAL_XFER_MODE_BLOCKING) {
        handle->baseAddress->SPIIMSC.reg = 0x0;
    } else if (handle->xFerMode == HAL_XFER_MODE_INTERRUPTS) {
        handle->baseAddress->SPIIMSC.reg = SPI_INTERRUPT_SET_ALL;
        handle->baseAddress->SPITXFIFOCR.BIT.txintsize = handle->txIntSize;
        handle->baseAddress->SPIRXFIFOCR.BIT.rxintsize = handle->rxIntSize;
    } else if (handle->xFerMode == HAL_XFER_MODE_DMA) {
        handle->baseAddress->SPIIMSC.reg = 0x0;

        handle->baseAddress->SPITXFIFOCR.BIT.dmatxbrsize = handle->txDMABurstSize;
        handle->baseAddress->SPIRXFIFOCR.BIT.dmarxbrsize = handle->rxDMABurstSize;
    } else {
        handle->errorCode = BASE_STATUS_ERROR;
        handle->state = HAL_SPI_STATE_RESET;
        return BASE_STATUS_ERROR;
    }
    handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_SET;
    handle->state = HAL_SPI_STATE_READY;
    return BASE_STATUS_OK;
}

/**
  * @brief Deinitialize the SPI module.
  * @param handle SPI handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_Deinit(SPI_Handle *handle)
{
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));

    handle->state = HAL_SPI_STATE_BUSY;

    SPI_RspDeInit(handle);

    handle->baseAddress->SPIIMSC.reg = 0x0;
    handle->baseAddress->SPIDMACR.BIT.rxdmae = BASE_CFG_UNSET;
    handle->baseAddress->SPIDMACR.BIT.txdmae = BASE_CFG_UNSET;
    handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_UNSET;
    handle->state = HAL_SPI_STATE_RESET;
    /* Clean callback */
    handle->TxCpltCallback = NULL;
    handle->RxCpltCallback = NULL;
    handle->TxRxCpltCallback = NULL;
    handle->ErrorCallback = NULL;
    handle->CsCtrlCallback = NULL;
    return BASE_STATUS_OK;
}

/**
  * @brief SPI Parameter Configuration.
  * @param handle SPI handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_ConfigParameter(SPI_Handle *handle)
{
    unsigned int cr0Reg;
    unsigned int temp;
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    CheckAllInitParameters(handle);

    handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_UNSET;
    handle->baseAddress->SPICR1.BIT.ms = handle->mode;
    handle->baseAddress->SPICR0.BIT.frf = handle->frameFormat;
    handle->baseAddress->SPICR0.BIT.dss = handle->dataWidth;
    handle->baseAddress->SPICR1.BIT.bitend = handle->endian;
    handle->baseAddress->SPICR0.BIT.sph = handle->clkPhase;
    handle->baseAddress->SPICR0.BIT.spo = handle->clkPolarity;
    handle->baseAddress->SPICR1.BIT.waitval = handle->waitVal;

    temp = ((unsigned int)handle->freqScr) << SPI_CR0_SCR_POS;
    cr0Reg = (handle->baseAddress->SPICR0.reg & (~SPI_CR0_SCR_MASK)) | temp;
    handle->baseAddress->SPICR0.reg = cr0Reg;

    /* Modulo 2 to get an even number */
    if ((handle->freqCpsdvsr % 2) == 0) {
        handle->baseAddress->SPICPSR.BIT.cpsdvsr = handle->freqCpsdvsr;
    } else {
        handle->baseAddress->SPICPSR.BIT.cpsdvsr = handle->freqCpsdvsr - 1;
    }

    if (handle->xFerMode == HAL_XFER_MODE_INTERRUPTS) {
        handle->baseAddress->SPITXFIFOCR.BIT.txintsize = handle->txIntSize;
        handle->baseAddress->SPIRXFIFOCR.BIT.rxintsize = handle->rxIntSize;
    } else if (handle->xFerMode == HAL_XFER_MODE_DMA) {
        handle->baseAddress->SPITXFIFOCR.BIT.dmatxbrsize = handle->txDMABurstSize;
        handle->baseAddress->SPIRXFIFOCR.BIT.dmarxbrsize = handle->rxDMABurstSize;
    } else {
        ;
    }

    handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_SET;
    return BASE_STATUS_OK;
}

/**
  * @brief Callback Function Registration.
  * @param handle SPI handle.
  * @param callbackID Callback function ID..
  * @param pcallback Pointer to the address of the registered callback function.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_RegisterCallback(SPI_Handle *handle,
                                         HAL_SPI_CallbackID callbackID,
                                         SPI_CallbackFuncType pcallback)
{
    BASE_StatusType ret = BASE_STATUS_OK;
    SPI_ASSERT_PARAM(handle != NULL && pcallback != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));

    if (handle->state == HAL_SPI_STATE_READY) {
        switch (callbackID) {
            case SPI_TX_COMPLETE_CB_ID :
                handle->TxCpltCallback = pcallback;
                break;
            case SPI_RX_COMPLETE_CB_ID :
                handle->RxCpltCallback = pcallback;
                break;
            case SPI_TX_RX_COMPLETE_CB_ID :
                handle->TxRxCpltCallback = pcallback;
                break;
            case SPI_ERROR_CB_ID :
                handle->ErrorCallback = pcallback;
                break;
            case SPI_CS_CB_ID:
                handle->CsCtrlCallback = pcallback;
                break;
            default :
                handle->errorCode = BASE_STATUS_ERROR;
                ret = BASE_STATUS_ERROR;
                break;
        }
    } else {
        handle->errorCode = BASE_STATUS_ERROR;
        ret = BASE_STATUS_ERROR;
    }
    return ret;
}

/**
  * @brief Receiving data in blocking mode.
  * @param handle SPI handle.
  * @param rData Address of the data buff to be Receiving.
  * @param dataSize Number of the data to be Receiving.
  * @param timeout Timeout period,unit: ms.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_ReadBlocking(SPI_Handle *handle,
                                     unsigned char *rData,
                                     unsigned int dataSize,
                                     unsigned int timeout)
{
    SPI_ASSERT_PARAM(handle != NULL && rData != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_RX;

    ConfigTransmissionParameter(handle, rData, NULL, dataSize);
    handle->timeout = timeout;

    ReadWriteBlocking(handle);
    if (handle->errorCode != BASE_STATUS_OK) {
        if (handle->ErrorCallback != NULL) {
            handle->ErrorCallback(handle);
        }
        return handle->errorCode;
    }
    if (handle->RxCpltCallback != NULL) {
        handle->RxCpltCallback(handle);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Send data in blocking mode.
  * @param handle SPI handle.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be sent.
  * @param timeout Timeout period,unit: ms.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_WriteBlocking(SPI_Handle *handle,
                                      unsigned char *wData,
                                      unsigned int dataSize,
                                      unsigned int timeout)
{
    SPI_ASSERT_PARAM(handle != NULL && wData != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));

    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_TX;

    ConfigTransmissionParameter(handle, NULL, wData, dataSize);
    handle->timeout = timeout;

    ReadWriteBlocking(handle);
    if (handle->errorCode != BASE_STATUS_OK) {
        if (handle->ErrorCallback != NULL) {
            handle->ErrorCallback(handle);
        }
        return handle->errorCode;
    }
    if (handle->TxCpltCallback != NULL) {
        handle->TxCpltCallback(handle);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Receiving and send data in blocking mode.
  * @param handle SPI handle.
  * @param rData Address of the data buff to be Receiving.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be Receivingd and sent.
  * @param timeout Timeout period,unit: ms.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_WriteReadBlocking(SPI_Handle *handle,
                                          unsigned char *rData,
                                          unsigned char *wData,
                                          unsigned int dataSize,
                                          unsigned int timeout)
{
    SPI_ASSERT_PARAM(handle != NULL && rData != NULL && wData != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));

    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_TX_RX;
    handle->timeout = timeout;

    ConfigTransmissionParameter(handle, rData, wData, dataSize);

    ReadWriteBlocking(handle);
    if (handle->errorCode != BASE_STATUS_OK) {
        if (handle->ErrorCallback != NULL) {
            handle->ErrorCallback(handle);
        }
        return handle->errorCode;
    }
    if (handle->TxRxCpltCallback != NULL) {
        handle->TxRxCpltCallback(handle);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Receiving data in interrupts mode.
  * @param handle SPI handle.
  * @param rData Address of the data buff to be Receiving.
  * @param dataSize Number of the data to be Receiving.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_ReadIT(SPI_Handle *handle, unsigned char *rData, unsigned int dataSize)
{
    SPI_ASSERT_PARAM(handle != NULL && rData != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));

    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_RX;
    ConfigTransmissionParameter(handle, rData, NULL, dataSize);

    ReadWriteInterruptEnable(handle);

    return BASE_STATUS_OK;
}

/**
  * @brief Send data in interrupts mode.
  * @param handle SPI handle.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be sent.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_WriteIT(SPI_Handle *handle, unsigned char *wData, unsigned int dataSize)
{
    SPI_ASSERT_PARAM(handle != NULL && wData != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));

    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_TX;
    ConfigTransmissionParameter(handle, NULL, wData, dataSize);

    ReadWriteInterruptEnable(handle);

    return BASE_STATUS_OK;
}

/**
  * @brief Receiving and send data in interrupts mode.
  * @param handle SPI handle.
  * @param rData Address of the data buff to be Receiving.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be Receiving and sent.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_WriteReadIT(SPI_Handle *handle,
                                    unsigned char *rData,
                                    unsigned char *wData,
                                    unsigned int dataSize)
{
    SPI_ASSERT_PARAM(handle != NULL && rData != NULL && wData != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));

    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_TX_RX;
    ConfigTransmissionParameter(handle, rData, wData, dataSize);

    ReadWriteInterruptEnable(handle);

    return BASE_STATUS_OK;
}

/**
  * @brief SPI DMA read completion callback function.
  * @param handle SPI handle.
  * @retval None
  */
static void ReadDmaFinishFun(void *handle)
{
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_Handle *spiHandle = (SPI_Handle *)(handle);
    SPI_ASSERT_PARAM(IsSPIInstance(spiHandle->baseAddress));
    if (spiHandle->state == HAL_SPI_STATE_BUSY_RX) {
        if (spiHandle->RxCpltCallback != NULL) {
            spiHandle->RxCpltCallback(spiHandle);
        }
    }

    if (spiHandle->state == HAL_SPI_STATE_BUSY_TX_RX) {
        if (spiHandle->TxRxCpltCallback != NULL) {
            spiHandle->TxRxCpltCallback(spiHandle);
        }
    }

    if (spiHandle->state == HAL_SPI_STATE_BUSY_TX) {
        if (spiHandle->TxCpltCallback != NULL) {
            spiHandle->TxCpltCallback(spiHandle);
        }
    }

    spiHandle->state = HAL_SPI_STATE_READY;
    spiHandle->baseAddress->SPIDMACR.BIT.rxdmae = BASE_CFG_UNSET;
}

/**
  * @brief SPI DMA write completion callback function.
  * @param handle SPI handle.
  * @retval None
  */
static void WriteDmaFinishFun(void *handle)
{
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_Handle *spiHandle = (SPI_Handle *)(handle);
    SPI_ASSERT_PARAM(IsSPIInstance(spiHandle->baseAddress));

    spiHandle->baseAddress->SPIDMACR.BIT.txdmae = BASE_CFG_UNSET;
    if (spiHandle->TxCpltCallback != NULL && spiHandle->state == HAL_SPI_STATE_READY) {
        spiHandle->TxCpltCallback(spiHandle);
    }
    if (spiHandle->frameFormat == HAL_SPI_MODE_MICROWIRE) {
        spiHandle->state = HAL_SPI_STATE_READY;
    }
}

/**
  * @brief SPI DMA error callback function.
  * @param handle SPI handle.
  * @retval None
  */
static void DmaErrorFun(void *handle)
{
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_Handle *spiHandle = (SPI_Handle *)(handle);
    SPI_ASSERT_PARAM(IsSPIInstance(spiHandle->baseAddress));

    spiHandle->baseAddress->SPIDMACR.reg = 0;

    if (spiHandle->ErrorCallback != NULL) {
        spiHandle->ErrorCallback(spiHandle);
    }
    spiHandle->state = HAL_SPI_STATE_READY;
}

/**
  * @brief DMA enable Configuration.
  * @param handle SPI handle.
  * @retval None
  */
static void EnableDma(SPI_Handle *handle)
{
    handle->baseAddress->SPIIMSC.reg = 0x0;
    SpiCsControl(handle, SPI_CHIP_SELECT);
    if (!handle->baseAddress->SPICR1.BIT.sse) {
        handle->baseAddress->SPICR1.BIT.sse = BASE_CFG_SET;
    }
    handle->baseAddress->SPIDMACR.reg = SPI_DMA_FIFO_ENABLE;
}

/**
  * @brief SPI read and write configures the DMA for channel callback functions.
  * @param handle SPI handle.
  * @retval None
  */
static void SetDmaCallBack(SPI_Handle *handle)
{
    handle->dmaHandle->DMA_CallbackFuns[handle->rxDmaCh].ChannelFinishCallBack = ReadDmaFinishFun;
    handle->dmaHandle->DMA_CallbackFuns[handle->rxDmaCh].ChannelErrorCallBack = DmaErrorFun;
    handle->dmaHandle->DMA_CallbackFuns[handle->txDmaCh].ChannelFinishCallBack = WriteDmaFinishFun;
    handle->dmaHandle->DMA_CallbackFuns[handle->txDmaCh].ChannelErrorCallBack = DmaErrorFun;
}

/**
  * @brief Configuring SPI Read and Write DMA Channel Parameters.
  * @param handle SPI handle.
  * @retval None
  */
static void ConfigDmaChannelParam(SPI_Handle *handle)
{
    DMA_ChannelParam channelParam;

    /* Set tx param */
    channelParam.srcPeriph = DMA_REQUEST_SPI_RX;
    channelParam.destPeriph = DMA_REQUEST_SPI_TX;
    channelParam.direction = DMA_MEMORY_TO_PERIPH_BY_DMAC;
    if (handle->state == HAL_SPI_STATE_BUSY_RX) {
        channelParam.srcAddrInc = DMA_ADDR_UNALTERED;
    } else {
        channelParam.srcAddrInc = DMA_ADDR_INCREASE;
    }
    channelParam.destAddrInc = DMA_ADDR_UNALTERED;
    channelParam.srcBurst = handle->rxDMABurstSize;
    channelParam.destBurst = handle->txDMABurstSize;
    if (handle->dataWidth > SPI_DATA_WIDTH_8BIT) {
        channelParam.srcWidth = DMA_TRANSWIDTH_HALFWORD;
        channelParam.destWidth = DMA_TRANSWIDTH_HALFWORD;
    } else {
        channelParam.srcWidth = DMA_TRANSWIDTH_BYTE;
        channelParam.destWidth = DMA_TRANSWIDTH_BYTE;
    }
    channelParam.pHandle = handle;
    HAL_DMA_InitChannel(handle->dmaHandle, &channelParam, handle->txDmaCh);

    /* Set rx param */
    channelParam.direction = DMA_PERIPH_TO_MEMORY_BY_DMAC;
    channelParam.srcAddrInc = DMA_ADDR_UNALTERED;
    if (handle->state == HAL_SPI_STATE_BUSY_TX) {
        channelParam.destAddrInc = DMA_ADDR_UNALTERED;
    } else {
        channelParam.destAddrInc = DMA_ADDR_INCREASE;
    }
    HAL_DMA_InitChannel(handle->dmaHandle, &channelParam, handle->rxDmaCh);
}

/**
  * @brief Receiving  data in DMA mode.
  * @param handle SPI handle.
  * @param rData Address of the data buff to be Receiving.
  * @param dataSize Number of the data to be Receiving.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_ReadDMA(SPI_Handle *handle, unsigned char *rData, unsigned int dataSize)
{
    static unsigned short writeVal = 0;
    BASE_StatusType ret;

    SPI_ASSERT_PARAM(handle != NULL && rData != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    SPI_ASSERT_PARAM(handle->txDmaCh < CHANNEL_MAX_NUM);
    SPI_ASSERT_PARAM(handle->rxDmaCh < CHANNEL_MAX_NUM);

    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_RX;
    ConfigTransmissionParameter(handle, rData, NULL, dataSize);

    SetDmaCallBack(handle);
    ConfigDmaChannelParam(handle);
    ret = HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)&(handle->baseAddress->SPIDR),
                          (uintptr_t)handle->rxBuff, handle->transferSize, handle->rxDmaCh);
    if (ret != BASE_STATUS_OK) {
        handle->state = HAL_SPI_STATE_READY;
        return ret;
    }
    ret = HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)&writeVal,
                          (uintptr_t)&(handle->baseAddress->SPIDR),
                          handle->transferSize, handle->txDmaCh);
    if (ret != BASE_STATUS_OK) {
        handle->state = HAL_SPI_STATE_READY;
    } else {
        EnableDma(handle);
    }

    return ret;
}

/**
  * @brief Send data in DMA mode.
  * @param handle SPI handle.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be sent.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_WriteDMA(SPI_Handle *handle, unsigned char *wData, unsigned int dataSize)
{
    static unsigned short readVal;
    BASE_StatusType ret;

    SPI_ASSERT_PARAM(handle != NULL && wData != NULL);
    SPI_ASSERT_PARAM(handle->txDmaCh < CHANNEL_MAX_NUM);
    SPI_ASSERT_PARAM(handle->rxDmaCh < CHANNEL_MAX_NUM);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));

    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_TX;
    ConfigTransmissionParameter(handle, NULL, wData, dataSize);

    SetDmaCallBack(handle);
    ConfigDmaChannelParam(handle);

    ret = HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)&(handle->baseAddress->SPIDR),
                          (uintptr_t)&readVal, handle->transferSize, handle->rxDmaCh);
    if (ret != BASE_STATUS_OK) {
        handle->state = HAL_SPI_STATE_READY;
        return ret;
    }
    ret = HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)handle->txBuff,
                          (uintptr_t)&(handle->baseAddress->SPIDR), handle->transferSize, handle->txDmaCh);
    if (ret != BASE_STATUS_OK) {
        handle->state = HAL_SPI_STATE_READY;
    } else {
        EnableDma(handle);
    }

    return ret;
}

/**
  * @brief Receiving and send data in DMA mode.
  * @param handle SPI handle.
  * @param rData Address of the data buff to be Receiving.
  * @param wData Address of the data buff to be sent.
  * @param dataSize Number of the data to be Receiving and sent.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_WriteReadDMA(SPI_Handle *handle,
                                     unsigned char *rData,
                                     unsigned char *wData,
                                     unsigned int dataSize)
{
    BASE_StatusType ret;

    SPI_ASSERT_PARAM(handle != NULL && rData != NULL && wData != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    SPI_ASSERT_PARAM(handle->txDmaCh < CHANNEL_MAX_NUM);
    SPI_ASSERT_PARAM(handle->rxDmaCh < CHANNEL_MAX_NUM);
    SPI_PARAM_CHECK_WITH_RET(handle->state == HAL_SPI_STATE_READY, BASE_STATUS_ERROR);
    SPI_PARAM_CHECK_WITH_RET(dataSize > 0, BASE_STATUS_ERROR);

    handle->state = HAL_SPI_STATE_BUSY_TX_RX;
    ConfigTransmissionParameter(handle, rData, wData, dataSize);
    SetDmaCallBack(handle);
    ConfigDmaChannelParam(handle);
    ret = HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)&(handle->baseAddress->SPIDR),
                          (uintptr_t)handle->rxBuff, handle->transferSize, handle->rxDmaCh);
    if (ret != BASE_STATUS_OK) {
        handle->state = HAL_SPI_STATE_READY;
        return ret;
    }
    ret = HAL_DMA_StartIT(handle->dmaHandle, (uintptr_t)handle->txBuff,
                          (uintptr_t)&(handle->baseAddress->SPIDR), handle->transferSize, handle->txDmaCh);
    if (ret != BASE_STATUS_OK) {
        handle->state = HAL_SPI_STATE_READY;
    } else {
        EnableDma(handle);
    }

    return ret;
}

/**
  * @brief Stop DMA transfer.
  * @param handle SPI handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_DMAStop(SPI_Handle *handle)
{
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    BASE_StatusType ret;

    ret = HAL_DMA_StopChannel(handle->dmaHandle, handle->txDmaCh);
    if (ret != BASE_STATUS_OK) {
        return ret;
    }
    ret = HAL_DMA_StopChannel(handle->dmaHandle, handle->rxDmaCh);
    return ret;
}

/**
  * @brief CS Channel Configuration.
  * @param handle SPI handle.
  * @param channel SPI CS channel.For details, see the enumeration definition of SPI_ChipSelectChannel.
  * @retval BASE status type: OK, ERROR.
  */
BASE_StatusType HAL_SPI_ChipSelectChannelSet(SPI_Handle *handle, SPI_ChipSelectChannel channel)
{
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    /* Check the validity of the CS parameters. */
    SPI_PARAM_CHECK_WITH_RET(channel >= SPI_CHIP_SELECT_CHANNEL_0 && channel < SPI_CHIP_SELECT_CHANNEL_MAX,
                             BASE_STATUS_ERROR);
    handle->baseAddress->SPICSNSEL.BIT.spi_csn_sel = channel;
    return BASE_STATUS_OK;
}

/**
  * @brief Obtains the currently configured CS channel.
  * @param handle SPI handle.
  * @param channel Pointer to the address for storing the obtained CS channel value.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_SPI_ChipSelectChannelGet(SPI_Handle *handle, SPI_ChipSelectChannel *channel)
{
    SPI_ASSERT_PARAM(handle != NULL && channel != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    *channel = handle->baseAddress->SPICSNSEL.BIT.spi_csn_sel;
    return BASE_STATUS_OK;
}

/**
  * @brief Interrupt Handling Function.
  * @param irqNum SPI interrupt number.
  * @param arg Handle pointers.
  * @retval None.
  */
void HAL_SPI_IRQHandler(void *arg)
{
    SPI_Handle *handle = (SPI_Handle *) arg;
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));

    if (handle->baseAddress->SPIMIS.reg == 0) {
        return;
    }

    if (handle->baseAddress->SPIMIS.BIT.rormis) {
        handle->baseAddress->SPIIMSC.reg = 0x0;

        handle->baseAddress->SPIICR.BIT.roric = BASE_CFG_SET;
        handle->baseAddress->SPIICR.BIT.rtic = BASE_CFG_SET;

        handle->errorCode = BASE_STATUS_ERROR;
        handle->state = HAL_SPI_STATE_ERROR;
        if (handle->ErrorCallback != NULL) {
            handle->ErrorCallback(handle);
        }
        SpiCsControl(handle, SPI_CHIP_DESELECT);
        return;
    }

    ReadWriteData(handle);

    if (handle->txCount == handle->transferSize) {
        if (handle->TxCpltCallback != NULL) {
            handle->TxCpltCallback(handle);
        }
    }

    if (handle->rxCount >= handle->transferSize) {
        /* Disable all interrupt */
        handle->baseAddress->SPIIMSC.reg = 0x0;
        /* Clear all interrupt */
        handle->baseAddress->SPIICR.BIT.roric = BASE_CFG_SET;
        handle->baseAddress->SPIICR.BIT.rtic = BASE_CFG_SET;

        SpiCsControl(handle, SPI_CHIP_DESELECT);

        if (handle->RxCpltCallback != NULL) {
            handle->RxCpltCallback(handle);
        }

        if (handle->TxRxCpltCallback != NULL) {
            handle->TxRxCpltCallback(handle);
        }

        handle->state = HAL_SPI_STATE_READY;
    }
    IRQ_ClearN(handle->irqNum);
}

/**
  * @brief SPI Interrupt Service Function.
  * @param irqNum SPI interrupt number.
  * @param handle SPI handle.
  * @retval None
  */
void HAL_SPI_IRQService(SPI_Handle *handle)
{
    SPI_ASSERT_PARAM(handle != NULL);
    SPI_ASSERT_PARAM(IsSPIInstance(handle->baseAddress));
    IRQ_Register(handle->irqNum, HAL_SPI_IRQHandler, handle);
}