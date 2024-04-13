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
  * @file      mcs_filter.c
  * @author    MCU Algorithm Team
  * @brief     This file provides functions of first-order filter.
  */

#include "mcs_filter.h"
#include "mcs_math_const.h"
#include "mcs_assert.h"

/**
  * @brief Initialzer of first-order low-pass filter handle.
  * @param lpfHandle First-order filter handle.
  * @param ts Control period (s).
  * @param fc Cut-off frequency (Hz).
  * @retval None.
  */
void FoLowPassFilterInit(FoFilterHandle *lpfHandle, float ts, float fc)
{
    MCS_ASSERT_PARAM(lpfHandle != NULL);
    MCS_ASSERT_PARAM(ts > 0.0f);
    MCS_ASSERT_PARAM(fc > 0.0f);
    lpfHandle->ctrlPeriod = ts;
    lpfHandle->fc = fc;

    FoFilterClear(lpfHandle);

    /* y(k) = (1/(1+wcTs)) * y(k-1) + (wcTs/(1+wcTs)) * u(k) */
    float wcTs = DOUBLE_PI * fc * ts;
    lpfHandle->a1 = 1.0f / (1.0f + wcTs); /* wcTs > 0 */
    lpfHandle->b1 = 1.0f - lpfHandle->a1;
}

/**
  * @brief Clear historical values of first-order filter handle.
  * @param foFilterHandle First-order filter handle.
  * @retval None.
  */
void FoFilterClear(FoFilterHandle *foFilterHandle)
{
    MCS_ASSERT_PARAM(foFilterHandle != NULL);
    foFilterHandle->uLast = 0.0f;
    foFilterHandle->yLast = 0.0f;
}

/**
  * @brief Calculation method of first-order filter.
  * @param lpfHandle First-order filter handle.
  * @param u The signal that wants to be filtered.
  * @retval The signal that is filered.
  */
float FoLowPassFilterExec(FoFilterHandle *lpfHandle, float u)
{
    MCS_ASSERT_PARAM(lpfHandle != NULL);
    float out;

    /* y(k) = (1/(1+wcTs)) * y(k-1) + (wcTs/(1+wcTs)) * u(k) */
    out = lpfHandle->a1 * lpfHandle->yLast + lpfHandle->b1 * u;
    lpfHandle->yLast = out;
    return out;
}
