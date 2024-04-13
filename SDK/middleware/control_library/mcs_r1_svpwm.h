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
  * @file      mcs_r1_svpwm.h
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of Space-vector pulse-width-modulation calculations.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_MCS_R1_SVPWM_H
#define McuMagicTag_MCS_R1_SVPWM_H

/* Includes ------------------------------------------------------------------*/
#include "mcs_typedef.h"

/** The ADC sampling twice for one resistor motor control application, SOCA + SOCB */
#define R1_ADC_SAMPLE_NUMS 2

/**
  * @defgroup R1_SVPWM_MODULE  R1 SVPWM MODULE
  * @brief The SVPWM module for R1(One Resistor) application.
  * @{
  */

/**
  * @defgroup R1_SVPWM_STRUCT  R1 SVPWM STRUCT
  * @brief The SVPWM module's struct definition for R1(One Resistor) application.
  * @{
  */
/* Typedef definitions -------------------------------------------------------*/
/**
  * @brief R1SVPWM struct members and parameters.
  */
typedef struct {
    float voltPu;                          /**< Voltage per unit value. */
    float oneDivVoltPu;                    /**< Reciprocal of voltage unit value. */
    float sampleWindow;                    /**< Sampling Window */
    float samplePointShift;                /**< Sampling point phase shift */
    short voltIndex;                       /**< Index of voltage sector. */
    short voltIndexLast;                   /**< Index of last voltage sector. */
    float samplePoint[R1_ADC_SAMPLE_NUMS]; /**< Sample point of twice sample. */
} R1SvpwmHandle;
/**
  * @}
  */

/**
  * @defgroup R1_SVPWM_API  R1 SVPWM API
  * @brief The SVPWM module's API declaration for R1(One Resistor) application.
  * @{
  */
void R1SVPWM_Init(R1SvpwmHandle *r1svHandle, float voltPu, float samplePointShift, float sampleWindow);
void R1SVPWM_Clear(R1SvpwmHandle *r1svHandle);
void R1SVPWM_Exec(R1SvpwmHandle *r1svHandle, const AlbeAxis *uAlbe, UvwAxis *dutyUvwLeft, UvwAxis *dutyUvwRight);
void R1CurrReconstruct(short sectorIndex, float currSocA, float currSocB, UvwAxis *curr);
/**
  * @}
  */

/**
  * @}
  */

#endif  /* McuMagicTag_MCS_SVPWM_H */
