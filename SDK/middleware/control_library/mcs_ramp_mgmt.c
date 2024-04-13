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
  * @file      mcs_ramp_mgmt.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of ramp function.
  */

#include "mcs_ramp_mgmt.h"
#include "mcs_assert.h"

/**
  * @brief Initializer of RMG handle.
  * @param rmgHandle: Pointer of RMG handle.
  * @param ctrlPeriod: Control period of the RMG module.
  * @param slope: Target value divide time of variation.
  * @retval None.
  */
void RMG_Init(RmgHandle *rmgHandle, float ctrlPeriod, float slope)
{
    MCS_ASSERT_PARAM(rmgHandle != NULL);
    MCS_ASSERT_PARAM(ctrlPeriod > 0.0f);
    MCS_ASSERT_PARAM(slope > 0.0f);
    /* Initializer of RMG handle. */
    rmgHandle->delta = ctrlPeriod * slope;
    rmgHandle->yLast = 0.0f;
    rmgHandle->ctrlPeriod = ctrlPeriod;
    rmgHandle->slope = slope;
}

/**
  * @brief Clear historical values of RMG handle.
  * @param rmgHandle: Pointer of RMG handle.
  * @retval None.
  */
void RMG_Clear(RmgHandle *rmgHandle)
{
    MCS_ASSERT_PARAM(rmgHandle != NULL);
    rmgHandle->yLast = 0.0f;
}

/**
  * @brief Ramp generation and management.
  * @param rmgHandle: Pointer of RMG handle.
  * @param targetVal: The target value.
  * @retval The reference value which is ramped.
  */
float RMG_Exec(RmgHandle *rmgHandle, float targetVal)
{
    MCS_ASSERT_PARAM(rmgHandle != NULL);
    float out;
    /* Calculate the current output value based on the target value and slope. */
    if (rmgHandle->yLast < targetVal) {
        out = rmgHandle->yLast + rmgHandle->delta;
    } else if (rmgHandle->yLast > targetVal) {
        out = rmgHandle->yLast - rmgHandle->delta;
    } else {
        out = rmgHandle->yLast;
    }
    /* Recording and outputting slope calculation results. */
    rmgHandle->yLast = out;
    return out;
}
