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
  * @file      crc.h
  * @author    MCU Driver Team
  * @brief     CRC module driver
  * @details   The header file contains the following declaration:
  *             + CRC handle structure definition.
  *             + Initialization functions.
  *             + CRC Set And Get Functions.
  *             + Interrupt Service Functions.
  */

#ifndef McuMagicTag_CRC_H
#define McuMagicTag_CRC_H
/* Includes ------------------------------------------------------------------*/
#include "crc_ip.h"

/* Macro definition */
/**
  * @defgroup CRC CRC
  * @brief CRC module.
  * @{
  */

/**
  * @defgroup CRC_Common CRC Common
  * @brief CRC common external module.
  * @{
  */

/**
  * @defgroup CRC_Handle_Definition CRC Handle Definition
  * @{
  */


/* Typedef definitions -------------------------------------------------------*/
typedef void (* CRC_CallbackType)(void *param);

typedef struct {
    CRC_RegStruct      *baseAddress;     /**< CRC Registers */
    CRC_AlgorithmMode   algoMode;        /**< CRC calculate algorithm mode */
    CRC_InputDataFormat inputDataFormat; /**< CRC byte mode */
    bool                enableIT;        /**< Enable pready timeout interrupt */
    bool                enableErrInject; /**< Enable error inject */
    unsigned int        timeout;         /**< APB pready max timeout value */
    unsigned int        irqNum;          /**< CRC interrupt number */
    CRC_CallbackType    preadyTimeoutCallback;   /**< CRC pready Timeout Callback */
} CRC_Handle;

/**
  * @}
  */
 
/**
  * @defgroup CRC_API_Declaration CRC HAL API
  * @{
  */
void HAL_CRC_Init(CRC_Handle *handle);
void HAL_CRC_DeInit(CRC_Handle *handle);
void CRC_RspInit(CRC_Handle *handle);
unsigned int HAL_CRC_SetInputDataGetCheck(CRC_Handle *handle, unsigned int data);
unsigned int HAL_CRC_Accumulate(CRC_Handle *handle, const void *pData, unsigned int length);
unsigned int HAL_CRC_Calculate(CRC_Handle *handle, const void *pData, unsigned int length);
bool HAL_CRC_CheckInputData(CRC_Handle *handle, const void *pData, unsigned int length, unsigned int crcValue);
void HAL_CRC_SetCheckInData(CRC_Handle *handle, unsigned int data);
unsigned int HAL_CRC_LoadCheckInData(CRC_Handle *handle);
void HAL_CRC_RegisterCallback(CRC_Handle *handle, CRC_CallbackType callBackFunc);
void HAL_CRC_IRQHandler(void *param);
void HAL_CRC_IRQService(CRC_Handle *handle);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_CRC_H */