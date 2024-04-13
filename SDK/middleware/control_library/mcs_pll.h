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
  * @file      mcs_pll.h
  * @author    MCU Algorithm Team
  * @brief     This file provides functions declaration of Phase-locked loop (PLL) module.
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_PLL_H
#define McuMagicTag_MCS_PLL_H

/* Includes ------------------------------------------------------------------------------------ */
#include "mcs_pid_ctrl.h"

/**
  * @defgroup PLL_MODULE  PLL MODULE
  * @brief The PLL module.
  * @{
  */

/**
  * @defgroup PLL_STRUCT  PLL STRUCT
  * @brief The PLL module data structure.
  * @{
  */
/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief PLL struct.
  */
typedef struct {
    PidHandle pi;       /**< PI controller for the PLL. */
    float minAmp;       /**< Minimum value of the input value in case of the divergence of the PLL. */
    float ctrlPeriod;   /**< Control period of the PLL. */
    float ratio;        /**< Conversion factor, ctrlPeriod * 65535 / TWO_PI. */
    float freq;         /**< Output estimated frequency (Hz). */
    signed short angle; /**< Output estimated phasse angle (Q15). */
} PllHandle;
/**
  * @}
  */

/**
  * @defgroup PLL_API  PLL API
  * @brief The PLL module API definitions.
  * @{
  */
void PLL_Reset(PllHandle *pllHandle);
void PLL_Clear(PllHandle *pllHandle);
void PLL_Exec(PllHandle *pllHandle, float sinVal, float cosVal);
/**
  * @}
  */

/**
  * @}
  */
#endif
