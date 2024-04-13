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
  * @file      mcs_smo_4th.h
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of 4th order smo module.
  */
#ifndef McuMagicTag_MCS_SMO_4TH_H
#define McuMagicTag_MCS_SMO_4TH_H

#include "mcs_typedef.h"
#include "mcs_pll.h"
#include "mcs_filter.h"

#define SPECIAL_SMO4TH_PLL_BDW (30.0f * 6.28318f)
#define SPECIAL_SMO4TH_SPD_FILTER_CUTOFF_FREQ (40.0f)
#define SPECIAL_SMO4TH_KD (300.0f)
#define SPECIAL_SMO4TH_KQ (2000.0f)

typedef struct {
    /* Model parameters */
    float ld;
    float lq;
    float rs;
    float ts;
    /* Internal variable */
    AlbeAxis currAlbeEst;
    AlbeAxis emfAlbeEst;
    PllHandle pll;
    signed short elecAngle;
    FoFilterHandle spdFilter;
    float spdEstHz;
    /* observer gain */
    float kd;
    float kq;
    float pllBdw;
} Smo4thHandle;

void SMO4TH_Init(Smo4thHandle *smo, float ld, float lq, float rs, float ts);

void SMO4TH_Clear(Smo4thHandle *smo);

void Set_SMO4TH_PARAM(Smo4thHandle *smo, float kd, float kq, float pllBdw, float cutOffFreq);

void SMO4TH_Exec(Smo4thHandle *smo, AlbeAxis *currAlbeFbk, AlbeAxis *voltAlbeRef);

#endif