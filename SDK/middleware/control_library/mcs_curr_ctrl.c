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
  * @file      mcs_curr_ctrl.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of motor current control.
  */

#include "typedefs.h"
#include "mcs_curr_ctrl.h"
#include "mcs_math_const.h"
#include "mcs_assert.h"

/**
  * @brief Reset the current control handle, fill with zero, NULL.
  * @param currHandle The current control handle.
  * @retval None.
  */
void CURRCTRL_Reset(CurrCtrlHandle *currHandle)
{
    MCS_ASSERT_PARAM(currHandle != NULL);
    /* Reset the current control handle, fill with zero, NULL. */
    currHandle->currRef    = NULL;
    currHandle->currFdbk   = NULL;
    currHandle->currFf.d   = 0.0f;
    currHandle->currFf.q   = 0.0f;
    currHandle->mtrParam   = NULL;
    currHandle->outLimit   = 0.0f;
    currHandle->ctrlPeriod = 0.0f;
    /* Reset Dq axis PID current control */
    PID_Reset(&currHandle->dAxisPi);
    PID_Reset(&currHandle->qAxisPi);
}

/**
  * @brief Clear historical values of current controller.
  * @param currHandle Current controller struct handle.
  * @retval None.
  */
void CURRCTRL_Clear(CurrCtrlHandle *currHandle)
{
    MCS_ASSERT_PARAM(currHandle != NULL);
    PID_Clear(&currHandle->dAxisPi);
    PID_Clear(&currHandle->qAxisPi);
}

/**
  * @brief Simplified current controller PI calculation.
  * @param currHandle Current controller struct handle.
  * @param voltRef Dq-axis voltage reference which is the output of current controller.
  * @retval None.
  */
void CURRCTRL_Exec(CurrCtrlHandle *currHandle, DqAxis *voltRef)
{
    MCS_ASSERT_PARAM(currHandle != NULL);
    MCS_ASSERT_PARAM(voltRef != NULL);
    /* Calculate the current error of the dq axis. */
    currHandle->dAxisPi.error = currHandle->currRef->d - currHandle->currFdbk->d;
    currHandle->qAxisPi.error = currHandle->currRef->q - currHandle->currFdbk->q;
    /* Calculation of the PI of the Dq axis current. */
    voltRef->d = PI_Exec(&currHandle->dAxisPi);
    voltRef->q = PI_Exec(&currHandle->qAxisPi);
}
