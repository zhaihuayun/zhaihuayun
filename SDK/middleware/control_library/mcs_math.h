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
  * @file      mcs_math.h
  * @author    MCU Algorithm Team
  * @brief     Math library.
  *            This file provides math functions declaration of motor math module.
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_MATH_H
#define McuMagicTag_MCS_MATH_H

/* Includes ------------------------------------------------------------------------------------ */
#include "mcs_typedef.h"

/**
  * @defgroup MATH MATH
  * @brief The common math const define for motor control.
  * @{
  */

/**
  * @defgroup MATH_Triangle  MATH Triangle
  * @brief The triangle structure definition.
  * @{
  */
/**
  * @brief sin cos define
  */
typedef struct {
    float sin; /**< The sine value of input angle. */
    float cos; /**< The cosine value of input angle. */
} TrigVal;
/**
  * @}
  */

/**
  * @defgroup MATH_API  MATH API
  * @brief The common math API definition.
  * @{
  */
void TrigCalc(TrigVal *val, signed short angle);
void ParkCalc(const AlbeAxis *albe, signed short angle, DqAxis *dq);
void InvParkCalc(const DqAxis *dq, signed short angle, AlbeAxis *albe);
void ClarkeCalc(const UvwAxis *uvw, AlbeAxis *albe);
float Abs(float val);
float Clamp(float val, float upperLimit, float lowerLimit);
float Max(float val1, float val2);
float Min(float val1, float val2);
float ASM_Sqrt(float val);
/**
  * @}
  */

/**
  * @}
  */
#endif
