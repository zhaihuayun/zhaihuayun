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
  * @file      pfc_curr_ctrl.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of power factor correction(PFC) current control
  */
#include "pfc_curr_ctrl.h"
#include "mcs_assert.h"

/**
  * @brief Clear historical values of power factor correction(PFC) current controller.
  * @param currCtrl PFC current control structure
  * @retval None.
  */
void PFC_CurrCtrlClear(PFC_CurrCtrlHandle *currCtrl)
{
    MCS_ASSERT_PARAM(currCtrl != NULL);
    currCtrl->currPiCtrl.differ = 0.0f;
    currCtrl->currPiCtrl.integral = 0.0f;
}

/**
  * @brief Simplified power factor correction(PFC) current controller PI calculation.
  * @param currCtrl PFC current control structure
  * @retval None.
  */
void PFC_CurrCtrlExec(PFC_CurrCtrlHandle *currCtrl)
{
    MCS_ASSERT_PARAM(currCtrl != NULL);
    /* Calculate the current error of power factor correction(PFC). */
    currCtrl->currPiCtrl.error = currCtrl->currRef - currCtrl->unitCurrFdbk;
    /* Calculation the output pwm duty of power factor correction(PFC) current. */
    currCtrl->pwmDuty = PI_Exec(&currCtrl->currPiCtrl);
}
