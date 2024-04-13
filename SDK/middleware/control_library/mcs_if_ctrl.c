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
  * @file      mcs_if_ctrl.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of I/F control.
  */

#include "mcs_if_ctrl.h"
#include "mcs_assert.h"

#define ANGLE_360 65536.0f /* 0 - 65536 indicates 0 to 360°. */

/**
  * @brief Initialzer of I/F control struct handle.
  * @param ifHandle I/F handle.
  * @param ifInit IfCtrlInit struct handle.
  * @retval None.
  */
void IF_Init(IfHandle *ifHandle, IfCtrlInit *ifInit)
{
    MCS_ASSERT_PARAM(ifHandle != NULL);
    MCS_ASSERT_PARAM(ifInit != NULL);
    MCS_ASSERT_PARAM(ifInit->targetAmp > 0.0f);
    MCS_ASSERT_PARAM(ifInit->currSlope > 0.0f);
    MCS_ASSERT_PARAM(ifInit->anglePeriod > 0.0f && ifInit->anglePeriod < 1.0f);
    MCS_ASSERT_PARAM(ifInit->stepAmpPeriod > 0.0f);
    ifHandle->targetAmp = ifInit->targetAmp;
    ifHandle->stepAmp = ifInit->currSlope * ifInit->stepAmpPeriod;  /* current step increment */
    ifHandle->curAmp = 0.0f;

    ifHandle->anglePeriod = ifInit->anglePeriod;
    ifHandle->ratio = ANGLE_360 * ifHandle->anglePeriod; /* Typical 0.0001f */
    ifHandle->angle = 0;
}

/**
  * @brief Clear historical values of first-order filter handle.
  * @param ifHandle I/F control handle.
  * @retval None.
  */
void IF_Clear(IfHandle *ifHandle)
{
    MCS_ASSERT_PARAM(ifHandle != NULL);
    ifHandle->curAmp = 0.0f;
    ifHandle->angle = 0;
}

/**
  * @brief I/F current amplitude calculation.
  * @param ifHandle I/F control handle.
  * @retval I/F current amplitude (A).
  */
float IF_CurrAmpCalc(IfHandle *ifHandle)
{
    MCS_ASSERT_PARAM(ifHandle != NULL);
    /* Calculation of IF Current Amplitude */
    if (ifHandle->curAmp < ifHandle->targetAmp) {
        ifHandle->curAmp += ifHandle->stepAmp;
    } else {
        ifHandle->curAmp = ifHandle->targetAmp;
    }

    return ifHandle->curAmp;
}

/**
  * @brief I/F current angle calculation.
  * @param ifHandle I/F control handle.
  * @param spdRefHz  Frequency of current vector.
  * @retval I/F output angle (Q15).
  */
signed short IF_CurrAngleCalc(IfHandle *ifHandle, float spdRefHz)
{
    MCS_ASSERT_PARAM(ifHandle != NULL);
    float delta = spdRefHz * ifHandle->ratio;
    ifHandle->angleTemp += delta;
    
    /* Params force convert to signed short. */
    ifHandle->angle = (signed short)ifHandle->angleTemp;
    return ifHandle->angle;
}
