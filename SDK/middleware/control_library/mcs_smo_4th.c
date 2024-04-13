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
  * @file      mcs_smo_4th.c
  * @author    MCU Algorithm Team
  * @brief     This file provides function of 4th order smo function.
  */

#include "mcs_smo_4th.h"
#include "mcs_math.h"
#include "mcs_math_const.h"


/**
  * @brief internal initial function for PLL
  * @param pll The PLL handle for SMO observer.
  * @param smo The SMO handle.
  */
static void PLL_Init(PllHandle *pll, Smo4thHandle *smo)
{
    /* clear PI */
    PID_Reset(&pll->pi);
    pll->ctrlPeriod = smo->ts;
    /* Init PI parameters with given bandwidth */
    pll->pi.kp = (1.414f) * smo->pllBdw;
    pll->pi.ki = smo->pllBdw * smo->pllBdw * smo->ts;
    pll->pi.upperLimit = 5000.0f;
    pll->pi.lowerLimit = -pll->pi.upperLimit;
    pll->minAmp = 0.1f;
    pll->freq = 0.0f;
    pll->ratio = (65536.0f) * pll->ctrlPeriod;
    pll->angle = 0;
}

/**
  * @brief The 4th SMO observer initial function.
  * @param smo The SMO handle.
  * @param ld The motor's D-axis inductance Ld.
  * @param lq The motor's Q-axis inductance Lq.
  * @param rs The motor's resistor value.
  * @param ts control period
  */
void SMO4TH_Init(Smo4thHandle *smo, float ld, float lq, float rs, float ts)
{
    /* give SMO observer the motor parameter for current & voltage calculation */
    smo->ld = ld;
    smo->lq = lq;
    smo->rs = rs;
    smo->ts = ts;
    /* SMO parameter init from user */
    smo->kd = SPECIAL_SMO4TH_KD;
    smo->kq = SPECIAL_SMO4TH_KQ;
    smo->pllBdw = SPECIAL_SMO4TH_PLL_BDW;
    SMO4TH_Clear(smo);
    PLL_Init(&smo->pll, smo);
    FoLowPassFilterInit(&smo->spdFilter, ts, SPECIAL_SMO4TH_SPD_FILTER_CUTOFF_FREQ);
}

/**
  * @brief Set kd, kq, pll bandwidth and cut-off frequency for 4TH smo, etc.
  * @param smo The SMO handle.
  * @param kd The d-axis gain.
  * @param kq The q-axis gain.
  * @param pllBdw The PLL bandwidth.
  * @param cutOffFreq The first-order low pass filter cut-off frequency.
  */
void Set_SMO4TH_PARAM(Smo4thHandle *smo, float kd, float kq, float pllBdw, float cutOffFreq)
{
    smo->kd = kd;
    smo->kq = kq;
    smo->pllBdw = pllBdw;
    smo->spdFilter.fc = cutOffFreq;
}

/**
  * @brief Clear function, put initial zero to voltage, current, etc.
  * @param smo The SMO handle.
  */
void SMO4TH_Clear(Smo4thHandle *smo)
{
    /* zero all current & voltage first */
    smo->currAlbeEst.alpha = 0.0f;
    smo->currAlbeEst.beta = 0.0f;
    smo->emfAlbeEst.alpha = 0.0f;
    smo->emfAlbeEst.beta = 0.0f;
    PLL_Clear(&smo->pll);
    FoFilterClear(&smo->spdFilter);
}

/**
  * @brief Calculate estimated current in the SMO observer.
  * @param smo The SMO handle.
  * @param currAlbeFbk Feedback current in Alpha-Beta axis.
  * @param voltAlbeRef Voltage reference in Alpha-Beta axis.
  */
static void SMO4TH_CurrEstCalc(Smo4thHandle *smo, AlbeAxis *currAlbeFbk, AlbeAxis *voltAlbeRef)
{
    float ld = smo->ld;
    float lq = smo->lq;
    float rs = smo->rs;
    float ts = smo->ts;
    float kd = smo->kd;
    float kq = smo->kq;
    /* use more local variables */
    float currEstA = smo->currAlbeEst.alpha;
    float currEstB = smo->currAlbeEst.beta;
    AlbeAxis *emfAlbe = &(smo->emfAlbeEst);
    AlbeAxis *currAlbe = &(smo->currAlbeEst);
    float ia;
    float ib;
    float eVa;
    float eVb;
    float iEstSignA;
    float iEstSignB;

    /* use Rad in calculation */
    float weEst = smo->spdEstHz * DOUBLE_PI;
    AlbeAxis eAlbeEst = smo->emfAlbeEst;
    AlbeAxis *iAlbeFbk = currAlbeFbk;
    AlbeAxis *uAlbeRef = voltAlbeRef;
    iEstSignA = ((currEstA - iAlbeFbk->alpha) > 0.0f) ? (1.0f) : (-1.0f);
    iEstSignB = ((currEstB - iAlbeFbk->beta) > 0.0f) ? (1.0f) : (-1.0f);

    /* calculate estimated current */
    ia = (-rs * currEstA - weEst * (ld - lq) * currEstB + uAlbeRef->alpha - eAlbeEst.alpha) / ld - kd * iEstSignA;
    ib = (weEst * (ld - lq) * currEstA - rs * currEstB + uAlbeRef->beta - eAlbeEst.beta) / ld - kd * iEstSignB;
    /* estimated current alpha, beta */
    currAlbe->alpha += ts * ia;
    currAlbe->beta += ts * ib;

    /* calculate estimate back EMF */
    eVa = -weEst * eAlbeEst.beta + (kd * ld * kq * iEstSignA - kd * ld * weEst * iEstSignB);
    eVb = weEst * eAlbeEst.alpha + (kd * ld * weEst * iEstSignA + kd * ld * kq * iEstSignB);
    /* estimated emf alpha, beta */
    emfAlbe->alpha += ts * eVa;
    emfAlbe->beta += ts * eVb;
}

/**
  * @brief Excute the 4th SMO observer.
  * @param smo the SMO handle.
  * @param currAlbeFbk current feedback in Alpha-Beta coordinate.
  * @param voltAlbeRef voltage feedback in Alpha-Beta coordinate.
  */
void SMO4TH_Exec(Smo4thHandle *smo, AlbeAxis *currAlbeFbk, AlbeAxis *voltAlbeRef)
{
    SMO4TH_CurrEstCalc(smo, currAlbeFbk, voltAlbeRef);
    PLL_Exec(&smo->pll, -smo->emfAlbeEst.alpha, smo->emfAlbeEst.beta);
    smo->elecAngle = smo->pll.angle;
    smo->spdEstHz = FoLowPassFilterExec(&smo->spdFilter, smo->pll.freq);
}