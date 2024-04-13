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
  * @file      wdg.c
  * @author    MCU Driver Team
  * @brief     WDG module driver
  * @details   This file provides firmware functions to manage the following functionalities of the WDG and IWDG.
  *             + Initialization functions.
  *             + WDG or IWDG Set And Get Functions.
  *             + Interrupt Service Functions.
  */

/* Includes ------------------------------------------------------------------*/
#include "interrupt.h"
#include "wdg.h"

static unsigned int WDG_CalculateRegTimeout(WDG_RegStruct *baseAddress, float loadValue, WDG_TimeType timeType);

/**
  * @brief Configure the hardware resources of the wdg.
  * @param handle Value of @ref WDG_handle.
  * @retval None.
  */
__weak void WDG_RspInit(WDG_Handle *handle)
{
    WDG_ASSERT_PARAM(handle != NULL);
    WDG_ASSERT_PARAM(IsWDGInstance(handle->baseAddress));
    BASE_FUNC_UNUSED(handle);
}

/**
  * @brief Initializing WDG or IWDG register values
  * @param handle Value of @ref WDG_handle.
  * @retval None
  */
void HAL_WDG_Init(WDG_Handle *handle)
{
    WDG_ASSERT_PARAM(handle != NULL);
    WDG_ASSERT_PARAM(IsWDGInstance(handle->baseAddress));
    WDG_PARAM_CHECK_NO_RET(IsWdgTimeType(handle->timeType));
    WDG_PARAM_CHECK_NO_RET(IsWdgLoadValue(handle->baseAddress, handle->loadValue, handle->timeType));
    /* baseaddress = IWDG or baseaddress = WDG */
    HAL_WDG_SetLoadValue(handle, handle->loadValue, handle->timeType);
    handle->baseAddress->WDG_CONTROL.BIT.resen = handle->enableRST;
    handle->baseAddress->WDG_CONTROL.BIT.wdgen = handle->enableIT;
    WDG_RspInit(handle);
}

/**
  * @brief Calculate Reg Timeout.
  * @param loadValue Value to be load to wdg.
  * @param timeType Value of @ref WDG_TimeType.
  * @retval unsigned int timeout Value.
  */
static unsigned int WDG_CalculateRegTimeout(WDG_RegStruct *baseAddress, float loadValue, WDG_TimeType timeType)
{
    float clockFreq = (float)HAL_CRG_GetIpFreq((void *)baseAddress);
    /* check clockFreq not equal zero */
    if (clockFreq > -FLT_EPSILON && clockFreq < FLT_EPSILON) {
        return 0;
    }
    unsigned int timeoutValue = 0x00000000U;

    switch (timeType) {
        case WDG_TIME_UNIT_TICK:
            timeoutValue = loadValue;
            break;
        case WDG_TIME_UNIT_S:
            timeoutValue = (unsigned int)(loadValue * clockFreq);
            break;
        case WDG_TIME_UNIT_MS:
            timeoutValue = (unsigned int)(loadValue * clockFreq / FREQ_CONVERT_MS_UNIT);
            break;
        case WDG_TIME_UNIT_US:
            timeoutValue = (unsigned int)(loadValue * clockFreq / FREQ_CONVERT_US_UNIT);
            break;
        default:
            break;
    }
    return timeoutValue;
}


/**
  * @brief Setting the load value of the WDG counter.
  * @param handle Value of @ref WDG_handle.
  * @param loadValue Load value of the WDG counter.
  * @param timeType WDG time type.
  * @retval None.
  */
void HAL_WDG_SetLoadValue(WDG_Handle *handle, unsigned int loadValue, WDG_TimeType timeType)
{
    WDG_ASSERT_PARAM(handle != NULL);
    WDG_ASSERT_PARAM(IsWDGInstance(handle->baseAddress) || IsIWDGInstance(handle->baseAddress));
    WDG_PARAM_CHECK_NO_RET(IsWdgTimeType(handle->timeType));
    WDG_PARAM_CHECK_NO_RET(IsWdgLoadValue(handle->baseAddress, handle->loadValue, handle->timeType));
    /* handle->baseAddress only could be configured WDG or IWDG */
    unsigned int value = WDG_CalculateRegTimeout(handle->baseAddress, loadValue, timeType);
    DCL_WDG_SetLoadValue(handle->baseAddress, value);
}

/**
  * @brief refresh the WDG counter.
  * @param handle Value of @ref WDG_handle.
  * @retval None.
  */
void HAL_WDG_Refresh(WDG_Handle *handle)
{
    WDG_ASSERT_PARAM(handle != NULL);
    WDG_ASSERT_PARAM(IsWDGInstance(handle->baseAddress));
    handle->baseAddress->wdg_intclr = BASE_CFG_SET;
}

/**
  * @brief refresh the WDG load value.
  * @param handle Value of @ref WDG_handle.
  * @retval unsigned int Load value.
  */
unsigned int HAL_WDG_GetLoadValue(WDG_Handle *handle)
{
    WDG_ASSERT_PARAM(handle != NULL);
    WDG_ASSERT_PARAM(IsWDGInstance(handle->baseAddress));
    return DCL_WDG_GetLoadValue(handle->baseAddress);
}

/**
  * @brief Refresh the WDG counter value.
  * @param handle Value of @ref WDG_handle.
  * @retval unsigned int Counter value.
  */
unsigned int HAL_WDG_GetCounterValue(WDG_Handle *handle)
{
    WDG_ASSERT_PARAM(handle != NULL);
    WDG_ASSERT_PARAM(IsWDGInstance(handle->baseAddress));
    float res = (float)handle->baseAddress->wdgvalue;
    unsigned int freq = HAL_CRG_GetIpFreq((void *)handle->baseAddress);
    /* check clockFreq not equal zero */
    if (freq == 0) {
        return 0;
    }
    switch (handle->timeType) {
        case WDG_TIME_UNIT_TICK :
            break;
        case WDG_TIME_UNIT_S :
            res = res / freq;
            break;
        case WDG_TIME_UNIT_MS :
            res = res * FREQ_CONVERT_MS_UNIT / freq;
            break;
        case WDG_TIME_UNIT_US :
            res = res * FREQ_CONVERT_US_UNIT / freq;
            break;
        default:
            break;
    }
    return (unsigned int)res;
}

/**
  * @brief Start the WDG count.
  * @param handle Value of @ref WDG_handle.
  * @retval None.
  */
void HAL_WDG_Start(WDG_Handle *handle)
{
    WDG_ASSERT_PARAM(handle != NULL);
    WDG_ASSERT_PARAM(IsWDGInstance(handle->baseAddress));
    DCL_WDG_EnableInterrupt(handle->baseAddress);
    DCL_WDG_Refresh(handle->baseAddress);
}

/**
  * @brief Stop the WDG count.
  * @param handle Value of @ref WDG_handle.
  * @retval None.
  */
void HAL_WDG_Stop(WDG_Handle *handle)
{
    WDG_ASSERT_PARAM(handle != NULL);
    WDG_ASSERT_PARAM(IsWDGInstance(handle->baseAddress));
    DCL_WDG_DisableReset(handle->baseAddress);
    DCL_WDG_DisableInterrupt(handle->baseAddress);
}

/**
  * @brief   Register WDG interrupt callback.
  * @param   handle Value of @ref WDG_handle.
  * @param   callBackFunc Value of @ref WDG_CallbackType.
  * @retval  None
  */
void HAL_WDG_RegisterCallback(WDG_Handle *handle, WDG_CallbackType callBackFunc)
{
    WDG_ASSERT_PARAM(handle != NULL);
    WDG_ASSERT_PARAM(callBackFunc != NULL);
    WDG_ASSERT_PARAM(IsWDGInstance(handle->baseAddress));
    if (callBackFunc != NULL) {
        handle->callbackFunc = callBackFunc;
    }
}

/**
  * @brief Interrupt service processing function.
  * @param param Handle.
  * @retval None.
  */
void HAL_WDG_IRQHandler(void *param)
{
    WDG_Handle *handle = (WDG_Handle *)param;
    WDG_ASSERT_PARAM(handle != NULL);
    WDG_ASSERT_PARAM(IsWDGInstance(handle->baseAddress));

    if (handle->baseAddress->WDG_MIS.BIT.wdogmis == 0x01) { /* Interrupt flag is set, fed dog in callback */
        if (handle->callbackFunc) {
            handle->callbackFunc(handle);
        }
    }
    IRQ_ClearN(handle->irqNum);
}

/**
  * @brief Registering the IRQHandler to the WDG interrupt
  * @param handle WDG handle.
  * @retval None.
  */
void HAL_WDG_IRQService(WDG_Handle *handle)
{
    WDG_ASSERT_PARAM(handle != NULL);
    WDG_ASSERT_PARAM(IsWDGInstance(handle->baseAddress));
    IRQ_Register(handle->irqNum, HAL_WDG_IRQHandler, handle);
}
