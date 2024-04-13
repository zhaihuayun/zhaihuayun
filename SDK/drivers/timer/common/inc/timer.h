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
  * @file      timer.h
  * @author    MCU Driver Team
  * @brief     TIMER module driver.
  * @details   This file provides firmware TIMER Handle structure and Functions
  *            prototypes to manage the following functionalities of the TIMER.
  *                + Initialization and de-initialization functions
  *                + config the register of timer
  */

#ifndef McuMagicTag_TIMER_H
#define McuMagicTag_TIMER_H

/* Includes ------------------------------------------------------------------*/
#include "timer_ip.h"

/* Macro definitions ---------------------------------------------------------*/

/**
 * @defgroup TIMER TIMER
 * @brief TIMER module.
 * @{
 */

/**
  * @defgroup TIMER_Common TIMER Common
  * @brief TIMER common external module.
  * @{
  */

/**
 * @brief  Typedef TIMER Parameter Config type
 */
typedef enum {
    TIMER_CFG_LOAD      = 0x0001,
    TIMER_CFG_BGLOAD    = 0x0002,
    TIMER_CFG_MODE      = 0x0004,
    TIMER_CFG_INTERRUPT = 0x0008,
    TIMER_CFG_PRESCALER = 0x0010,
    TIMER_CFG_SIZE      = 0x0020,
    TIMER_CFG_DMAADC_SINGLE_REQ = 0x0040,
    TIMER_CFG_DMA_BURST_REQ = 0x0080,
} TIMER_CFG_TYPE;

/**
 * @defgroup TIMER_Handle_Definition TIMER Handle Definition
 * @{
 */

/**
 * @brief  Typedef callback function of TIMER
 */
typedef void (*TIMER_CallBackFunc)(void *param);

/**
 * @brief  Time base address and Configuration Structure definition
 */
typedef struct {
    TIMER_RegStruct       *baseAddress;    /**< Base address of timer */
    unsigned int          load;            /**< Period, set the TIMERx_LOAD */
    TIMER_Mode            mode;            /**< Timer counting mode */
    unsigned int          interruptEn;     /**< Interrupt enable or disable */
    TIMER_PrescalerFactor prescaler;       /**< Timer prescaler */
    TIMER_Size            size;            /**< Timer size 16 or 32 bits */
    unsigned int          bgLoad;          /**< Background period, set the TIMEx_BGLOAD */
    unsigned int          irqNum;          /**< Timer interrupt number */
    TIMER_CallBackFunc    Callback;        /**< Callback function of timer */
    bool                  dmaAdcSingleReqEnable; /**< Enable bit for DMA Single request and trigger ADC sampling */
    bool                  dmaBurstReqEnable; /**< DMA Burst Request Enable Bit */
} TIMER_Handle;
/**
  * @}
  */

/**
  * @defgroup TIMER_API_Declaration TIMER HAL API
  * @{
  */
BASE_StatusType HAL_TIMER_Init(TIMER_Handle *handle);

void HAL_TIMER_DeInit(TIMER_Handle *handle);

void HAL_TIMER_Start(TIMER_Handle *handle);

void HAL_TIMER_Stop(TIMER_Handle *handle);

void HAL_TIMER_RspInit(TIMER_Handle *handle);

BASE_StatusType HAL_TIMER_Config(TIMER_Handle *handle, TIMER_CFG_TYPE cfgType);

BASE_StatusType HAL_TIMER_GetConfig(TIMER_Handle *handle);

void HAL_TIMER_IrqClear(TIMER_Handle *handle);

BASE_StatusType HAL_TIMER_RegisterCallback(TIMER_Handle *handle, TIMER_CallBackFunc callBackFunc);

void HAL_TIMER_UnRegisterCallback(TIMER_Handle *handle);

/**
  * @}
  */

/**
  * @}
  */

/**
 * @}
 */
#endif /* McuMagicTag_TIMER_H */