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
  * @file      cfd.c
  * @author    MCU Driver Team
  * @brief     CFD module driver.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the CFD.
  *             + Initialization and de-initialization functions.
  *             + Config the register of cfd.
  *             + Interrupt callback function and user registration function.
  */

/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "cfd.h"

/**
  * @brief Default Hardware Resource Initialization Function.
  * @param handle CFD handle.
  * @retval @ref BASE_StatusType.
  */
__weak BASE_StatusType CFD_RspInit(CFD_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    return BASE_STATUS_OK;
}

/**
  * @brief Default Hardware Resource Deinitialization Function.
  * @param handle CFD handle.
  * @retval @ref BASE_StatusType.
  */
__weak BASE_StatusType CFD_RspDeInit(CFD_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    return BASE_STATUS_OK;
}

/**
  * @brief Perform initial configuration based on the handle.
  * @param handle CFD handle.
  * @retval @ref BASE_StatusType.
  */
BASE_StatusType HAL_CFD_Init(CFD_Handle *handle)
{
    CFD_ASSERT_PARAM(handle != NULL);
    CFD_ASSERT_PARAM(IsCFDInstance(handle->baseAddress));

    handle->baseAddress->CFDWDOH.BIT.cfdwdoh = handle->upperBound;
    handle->baseAddress->CFDINTENA.reg = handle->interruptType;

    return CFD_RspInit(handle);
}

/**
  * @brief Deinitialize configurations based on the handle.
  * @param handle CFD handle.
  * @retval @ref BASE_StatusType.
  */
BASE_StatusType HAL_CFD_DeInit(CFD_Handle *handle)
{
    CFD_ASSERT_PARAM(handle != NULL);
    CFD_ASSERT_PARAM(IsCFDInstance(handle->baseAddress));
    /* Clear interrupt callback function. */
    handle->PllClockStopCallback = NULL;
    handle->CheckEndCallback = NULL;
    /* Clear register value. */
    handle->baseAddress->CFDINTENA.reg = BASE_CFG_DISABLE;
    return CFD_RspDeInit(handle);
}

/**
  * @brief Set this parameter based on the configuration item parameters.
  * @param handle CFD handle.
  * @param cfgType Configurable item. @ref CFD_CFG_TYPE.
  * @retval @ref BASE_StatusType.
  */
BASE_StatusType HAL_CFD_Config(CFD_Handle *handle, CFD_CFG_TYPE cfgType)
{
    CFD_ASSERT_PARAM(handle != NULL);
    CFD_ASSERT_PARAM(IsCFDInstance(handle->baseAddress));

    switch (cfgType) {
        case CFD_CFG_UPPER_BOUND:
            handle->baseAddress->CFDWDOH.BIT.cfdwdoh = handle->upperBound;
            break;
        case CFD_CFG_INT_TYPE:
            handle->baseAddress->CFDINTENA.reg = handle->interruptType;
            break;
        default:
            return BASE_STATUS_ERROR;
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Reads the register configuration value to the handle.
  * @param handle CFD handle.
  * @retval None.
  */
void HAL_CFD_GetConfig(CFD_Handle *handle)
{
    CFD_ASSERT_PARAM(handle != NULL);
    CFD_ASSERT_PARAM(IsCFDInstance(handle->baseAddress));

    handle->upperBound = handle->baseAddress->CFDWDOH.BIT.cfdwdoh;
    handle->interruptType = handle->baseAddress->CFDINTENA.reg;
}

/**
  * @brief Start CFD Module.
  * @param handle CFD handle.
  * @retval None.
  */
void HAL_CFD_Start(CFD_Handle *handle)
{
    CFD_ASSERT_PARAM(handle != NULL);
    CFD_ASSERT_PARAM(IsCFDInstance(handle->baseAddress));

    handle->baseAddress->CFDCTRL.BIT.cfden = BASE_CFG_ENABLE;
}

/**
  * @brief Stop CFD Module.
  * @param handle CFD handle.
  * @retval None.
  */
void HAL_CFD_Stop(CFD_Handle *handle)
{
    CFD_ASSERT_PARAM(handle != NULL);
    CFD_ASSERT_PARAM(IsCFDInstance(handle->baseAddress));

    handle->baseAddress->CFDCTRL.BIT.cfden = BASE_CFG_DISABLE;
}

/**
  * @brief Registers the interrupt function to the specified interrupt type.
  * @param handle CFD handle.
  * @param type Specified interrupt type.
  * @param callback Interrupt callback function.
  * @retval @ref BASE_StatusType.
  */
BASE_StatusType HAL_CFD_RegisterCallback(CFD_Handle *handle, CFD_Interrupt_Type type, CFD_CallBackFuncType callback)
{
    CFD_ASSERT_PARAM(handle != NULL);
    CFD_ASSERT_PARAM(callback != NULL);
    CFD_ASSERT_PARAM(IsCFDInstance(handle->baseAddress));

    switch (type) {
        case CFD_INT_PLL_REF_CLOCK_STOP_MASK:
            handle->PllClockStopCallback = callback;
            break;
        case CFD_INT_CHECK_END_MASK:
            handle->CheckEndCallback = callback;
            break;
        default:
            return BASE_STATUS_ERROR;
    }

    return BASE_STATUS_OK;
}

/**
  * @brief Interrupt service processing function.
  * @param param Handle.
  * @retval None.
  */
void HAL_CFD_IRQHandler(void *param)
{
    CFD_Handle *handle = (CFD_Handle *)param;

    CFD_ASSERT_PARAM(handle != NULL);
    CFD_ASSERT_PARAM(IsCFDInstance(handle->baseAddress));

    if (handle->baseAddress->CFDINTSTS.BIT.clk_fail_int == 0x01) {
        handle->baseAddress->CFDINTRAW.BIT.clk_fail_raw = BASE_CFG_SET;
        if (handle->PllClockStopCallback) {
            handle->PllClockStopCallback(handle);
        }
    }

    if (handle->baseAddress->CFDINTSTS.BIT.chk_end_int == 0x01) {
        handle->baseAddress->CFDINTRAW.BIT.chk_end_raw = BASE_CFG_SET;
        if (handle->CheckEndCallback) {
            handle->CheckEndCallback(handle);
        }
    }

    IRQ_ClearN(handle->irqNum);
}

/**
  * @brief Registering the IRQHandler to the CFD interrupt
  * @param handle CFD handle.
  * @retval None.
  */
void HAL_CFD_IRQService(CFD_Handle *handle)
{
    CFD_ASSERT_PARAM(handle != NULL);
    CFD_ASSERT_PARAM(IsCFDInstance(handle->baseAddress));

    IRQ_Register(handle->irqNum, HAL_CFD_IRQHandler, handle);
}