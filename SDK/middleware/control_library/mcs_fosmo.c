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
  * @file      mcs_fosmo.c
  * @author    MCU Algorithm Team
  * @brief     This file provides functions of position sliding mode observer (SMO) module.
  */

#include "mcs_fosmo.h"
#include "mcs_math_const.h"
#include "mcs_math.h"
#include "typedefs.h"
#include "mcs_assert.h"

/**
  * @brief Clear historical values of SMO handle.
  * @param smoHandle SMO struct handle.
  * @retval None.
  */
void FOSMO_Clear(FoSmoHandle *smoHandle)
{
    MCS_ASSERT_PARAM(smoHandle != NULL);
    /* Clear historical values of SMO handle */
    smoHandle->currEst.alpha     = 0.0f;
    smoHandle->currEst.beta      = 0.0f;
    smoHandle->currEstLast.alpha = 0.0f;
    smoHandle->currEstLast.beta  = 0.0f;
    smoHandle->emfEstUnFil.alpha = 0.0f;
    smoHandle->emfEstUnFil.beta  = 0.0f;
    smoHandle->emfEstFil.alpha   = 0.0f;
    smoHandle->emfEstFil.beta    = 0.0f;
    /* Clear historical values of PLL controller */
    PLL_Clear(&smoHandle->pll);
    /* Clear historical values of first-order smoHandle speed filter */
    FoFilterClear(&smoHandle->spdFilter);
}

/**
  * @brief Calculation method of first-order SMO.
  * @param smoHandle SMO struct handle.
  * @param currAlbe Feedback currents in the alpha-beta coordinate (A).
  * @param voltAlbe FOC output voltages in alpha-beta coordinate (V).
  * @param refHz The reference frequency (Hz).
  * @retval None.
  */
void FOSMO_Exec(FoSmoHandle *smoHandle, const AlbeAxis *currAlbe, const AlbeAxis *voltAlbe, float refHz)
{
    MCS_ASSERT_PARAM(smoHandle != NULL);
    MCS_ASSERT_PARAM(currAlbe != NULL);
    MCS_ASSERT_PARAM(voltAlbe != NULL);
    float err;
    float wcTs;
    float fcAbs = Abs(refHz);
    signed short filCompAngle; /* Compensation angle */
    float currAlpha = smoHandle->currEstLast.alpha;
    float currBeta  = smoHandle->currEstLast.beta;
    float emfUnAlpha = smoHandle->emfEstUnFil.alpha;
    float emfUnBeta  = smoHandle->emfEstUnFil.beta;

    smoHandle->currEst.alpha =
        (smoHandle->a1 * currAlpha) + (smoHandle->a2 * (voltAlbe->alpha - emfUnAlpha));
    smoHandle->currEst.beta =
        (smoHandle->a1 * currBeta) + (smoHandle->a2 * (voltAlbe->beta - emfUnBeta));

    smoHandle->currEstLast.alpha = smoHandle->currEst.alpha;
    smoHandle->currEstLast.beta   = smoHandle->currEst.beta;

    /* Estmated back EMF by sign function. */
    err = smoHandle->currEst.alpha - currAlbe->alpha;
    smoHandle->emfEstUnFil.alpha = smoHandle->kSmo * ((err > 0.0f) ? 1.0f : -1.0f);
    err = smoHandle->currEst.beta - currAlbe->beta;
    smoHandle->emfEstUnFil.beta  = smoHandle->kSmo * ((err > 0.0f) ? 1.0f : -1.0f);

    /* Estmated back EMF is filtered by first-order LPF. */
    if (fcAbs <= smoHandle->emfLpfMinFreq) {
        wcTs = smoHandle->emfLpfMinFreq * DOUBLE_PI * smoHandle->ctrlPeriod * smoHandle->lambda;
    } else {
        wcTs = fcAbs * DOUBLE_PI * smoHandle->ctrlPeriod * smoHandle->lambda;
    }
    smoHandle->emfEstFil.alpha = (smoHandle->emfEstFil.alpha + wcTs * smoHandle->emfEstUnFil.alpha) / (wcTs + 1.0f);
    smoHandle->emfEstFil.beta  = (smoHandle->emfEstFil.beta + wcTs * smoHandle->emfEstUnFil.beta) / (wcTs + 1.0f);

    /* Get phase angle and frequency from BEMF by PLL. */
    PLL_Exec(&smoHandle->pll, -smoHandle->emfEstFil.alpha, smoHandle->emfEstFil.beta);

    /* Compensation phase lag caused by the LPF. */
    filCompAngle = (refHz > 0.0f) ? (smoHandle->filCompAngle) : (INT16_MAX - smoHandle->filCompAngle);
    smoHandle->elecAngle = smoHandle->pll.angle + filCompAngle;

    /* Estmated speed is filtered by first-order LPF. */
    smoHandle->spdEstHz = FoLowPassFilterExec(&smoHandle->spdFilter, smoHandle->pll.freq);
}
