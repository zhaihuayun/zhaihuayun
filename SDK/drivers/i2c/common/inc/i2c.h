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
  * @file    i2c.h
  * @author  MCU Driver Team,
  * @brief   I2C module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the I2C.
  *          + Initialization and de-initialization functions.
  *          + Peripheral transmit and receiving functions.
  *          + I2C parameter handle definition.
  *          + Basic Configuration Parameter Enumeration Definition.
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef McuMagicTag_I2C_H
#define McuMagicTag_I2C_H

/* Includes ------------------------------------------------------------------*/
#include "dma.h"
#include "i2c_ip.h"

/**
  * @defgroup I2C I2C
  * @brief I2C module.
  * @{
  */

/**
  * @defgroup I2C_Common I2C Common
  * @brief I2C common external module.
  * @{
  */

/* Typedef definitions -------------------------------------------------------*/

/**
  * @defgroup I2C_Handle_Definition I2C Handle Definition
  * @{
  */

/**
 * @brief Callback Function ID Enumeration Definition
 */
typedef enum {
    I2C_MASTER_TX_COMPLETE_CB_ID = 0x00000000U,
    I2C_MASTER_RX_COMPLETE_CB_ID = 0x00000001U,
    I2C_ERROR_CB_ID              = 0x00000002U,
} I2C_CallbackId;

/**
 * @brief Module Status Enumeration Definition
 */
typedef enum {
    I2C_STATE_RESET   = 0x00000000U,
    I2C_STATE_READY   = 0x00000001U,
    I2C_STATE_BUSY    = 0x00000002U,
    I2C_STATE_BUSY_TX = 0x00000003U,
    I2C_STATE_BUSY_RX = 0x00000004U,
    I2C_STATE_TIMEOUT = 0x00000005U,
    I2C_STATE_ERROR   = 0x00000006U
} I2C_StateType;

/**
 * @brief Module handle structure definition
 */
typedef struct _I2C_Handle {
    I2C_RegStruct   *baseAddress;          /**< Register base address. */
    I2C_AddressMode  addrMode;             /**< 7bit or 10bit. */
    unsigned int     sdaHoldTime;          /**< SDA hold time. */
    unsigned int     freq;                 /**< Operating Frequency. */
    unsigned char    ignoreAckFlag;        /**< Ignore the response flag bit. */
    unsigned int     txWaterMark;          /**< TX threshold configuration. */
    unsigned int     rxWaterMark;          /**< RX threshold configuration. */
    unsigned int     irqNum;               /**< I2C interrupt number. */
    unsigned char    msgStopFlag;          /**< Data transfer stop flag. If this bit is set to 1,
                                                the data to be transferred is the last data. */
    unsigned char   *rxBuff;               /**< RX buffer. */
    unsigned char   *txBuff;               /**< TX buffer. */
    unsigned int     transferSize;         /**< Transmission Data Length. */
    unsigned int     transferCount;        /**< Transferred Data Count. */
    unsigned int     timeout;              /**< Timeout period. */
    I2C_StateType    state;                /**< Running Status. */
    BASE_StatusType  errorCode;            /**< Error Code. */

    unsigned int     dmaCh;                /**< DMA channel */
    unsigned int     srcBurst;             /**< DMA source burst,Reference DMA_BurstLength */
    unsigned int     destBurst;            /**< DMA destination burst,Reference DMA_BurstLength */
    unsigned int     srcWidth;             /**< DMA source width,Reference DMA_TransmisWidth */
    unsigned int     destWidth;            /**< DMA destination width,Reference DMA_TransmisWidth*/
    DMA_Handle      *dmaHandle;            /**< DMA handle */

    void (*MasterTxCplCallback)(struct _I2C_Handle *handle); /**< Sending completion callback function. */
    void (*MasterRxCplCallback)(struct _I2C_Handle *handle); /**< Receive completion callback function. */
    void (*ErrorCallback)(struct _I2C_Handle *handle);       /**< Error callback function. */
} I2C_Handle;
/**
  * @}
  */

/**
  * @defgroup I2C_API_Declaration I2C HAL API
  * @{
  */
/**
 * @brief Callback Function Type Definition.
 */
typedef void (*I2C_CallbackFunType)(I2C_Handle *handle);

/* Function Interface Definition -------------------------------------------------------*/
BASE_StatusType HAL_I2C_Init(I2C_Handle *handle);
BASE_StatusType HAL_I2C_Deinit(I2C_Handle *handle);
BASE_StatusType HAL_I2C_ConfigParameter(I2C_Handle *handle);
BASE_StatusType HAL_I2C_RegisterCallback(I2C_Handle *handle, I2C_CallbackId callbackID, I2C_CallbackFunType pcallback);
BASE_StatusType HAL_I2C_MasterReadBlocking(I2C_Handle *handle,
                                           unsigned short devAddr,
                                           unsigned char *rData,
                                           unsigned int dataSize,
                                           unsigned int timeout);
BASE_StatusType HAL_I2C_MasterWriteBlocking(I2C_Handle *handle,
                                            unsigned short devAddr,
                                            unsigned char *wData,
                                            unsigned int dataSize,
                                            unsigned int timeout);
BASE_StatusType HAL_I2C_MasterReadIT(I2C_Handle *handle,
                                     unsigned short devAddr,
                                     unsigned char *rData,
                                     unsigned int dataSize);
BASE_StatusType HAL_I2C_MasterWriteIT(I2C_Handle *handle,
                                      unsigned short devAddr,
                                      unsigned char *wData,
                                      unsigned int dataSize);
BASE_StatusType HAL_I2C_MasterReadDMA(I2C_Handle *handle,
                                      unsigned short devAddr,
                                      unsigned char *rData,
                                      unsigned int dataSize);
BASE_StatusType HAL_I2C_MasterWriteDMA(I2C_Handle *handle,
                                       unsigned short devAddr,
                                       unsigned char *wData,
                                       unsigned int dataSize);

void HAL_I2C_IRQHandler(void *arg);
void HAL_I2C_IRQService(I2C_Handle *handle);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* #ifndef McuMagicTag_I2C_H */