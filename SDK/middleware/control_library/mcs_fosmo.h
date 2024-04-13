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
  * @file      mcs_fosmo.h
  * @author    MCU Algorithm Team
  * @brief     Sliding-mode observer (SMO) for motor position acquisition.
  *            This file provides position SMO and Phase-locked loop (PLL) declaration for motor control.
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_FOSMO_H
#define McuMagicTag_MCS_FOSMO_H

/* Includes ------------------------------------------------------------------------------------ */
#include "mcs_mtr_param.h"
#include "mcs_typedef.h"
#include "mcs_pll.h"
#include "mcs_filter.h"

/**
  * @defgroup FOSMO_MODULE  FOSMO MODULE
  * @brief The First Order Sliding Mode Observer module.
  * @{
  */

/**
  * @defgroup FOSMO_STRUCT  FOSMO STRUCT
  * @brief The First Order Sliding Mode Observer's data struct definition.
  * @{
  */
/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief Position SMO struct members and parameters.
  */
typedef struct {
    MtrParamHandle  *mtrParam;      /**< Motor parameters. */
    float            ctrlPeriod;    /**< SMO control period (s). */
    float            a1;            /**< Coefficient of differential equation. */
    float            a2;            /**< Coefficient of differential equation. */
    AlbeAxis         currEst;       /**< SMO estimated currents in the alpha-beta coordinate. */
    AlbeAxis         currEstLast;   /**< SMO history values of estimated currents in the alpha-beta coordinate. */
    float            kSmo;          /**< SMO gain. */
    AlbeAxis         emfEstUnFil;   /**< Estimated back-EMF in the alpha-beta coordinate by differential equation. */
    float            lambda;        /**< SMO coefficient of cut-off frequency. */
    float            emfLpfMinFreq; /**< The minimum cut-off frequency of back-EMF filter. */
    signed short     filCompAngle;  /**< Compensation angle for the back-EMF filter. */
    AlbeAxis         emfEstFil;     /**< SMO estimated back-EMF in the alpha-beta coordinate. */
    PllHandle        pll;           /**< PLL handle. */
    signed short     elecAngle;     /**< SMO estimated electronic angle (Q15). */
    FoFilterHandle   spdFilter;     /**< First-order LPF for speed. */
    float            spdEstHz;      /**< SMO estimated electronic speed (Hz). */
} FoSmoHandle;
/**
  * @}
  */

/**
  * @defgroup FOSMO_API  FOSMO API
  * @brief The First Order Sliding Mode Observer's API declaration.
  * @{
  */
void FOSMO_Clear(FoSmoHandle *smoHandle);
void FOSMO_Exec(FoSmoHandle *smoHandle, const AlbeAxis *currAlbe, const AlbeAxis *voltAlbe, float refHz);
/**
  * @}
  */

/**
  * @}
  */

#endif
