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
  * @file      mcs_ramp_mgmt.h
  * @author    MCU Algorithm Team
  * @brief     Ramp generation and management for motor control.
  *            This file provides functions declaration of ramp generation and management module.
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_RAMP_MGMT_H
#define McuMagicTag_MCS_RAMP_MGMT_H

/**
  * @defgroup RAMP_MODULE  RAMP MODULE
  * @brief The RAMP management module.
  * @{
  */

/**
  * @defgroup RAMP_STRUCT  RAMP STRUCT
  * @brief The RAMP management data structure.
  * @{
  */
/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief Ramp mgmt Struct.
  */
typedef struct {
    float delta;      /**< Step value per calculate period. */
    float yLast;      /**< History value of output value. */
    float ctrlPeriod; /**< Control period of the RMG module. */
    float slope;      /**< Slope, target value divide time of variation. */
} RmgHandle;
/**
  * @}
  */

/**
  * @defgroup RAMP_API  RAMP API
  * @brief The RAMP API definitions.
  * @{
  */
void RMG_Init(RmgHandle *rmgHandle, float ctrlPeriod, float slope);
void RMG_Clear(RmgHandle *rmgHandle);
float RMG_Exec(RmgHandle *rmgHandle, float targetVal);
/**
  * @}
  */

/**
  * @}
  */

#endif
