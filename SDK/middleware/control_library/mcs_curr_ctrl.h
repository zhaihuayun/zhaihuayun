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
  * @file      mcs_curr_ctrl.h
  * @author    MCU Algorithm Team
  * @brief     Current controller for motor control.
  *            This file provides functions declaration of the current controller module.
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_CURR_CTRL_H
#define McuMagicTag_MCS_CURR_CTRL_H

/* Includes ------------------------------------------------------------------------------------ */
#include "mcs_typedef.h"
#include "mcs_pid_ctrl.h"
#include "mcs_mtr_param.h"

/**
  * @defgroup CURRENT_CONTROLLER CURRENT CONTROLLER MODULE
  * @brief The current controller function.
  * @{
  */

/**
  * @defgroup CURRENT_CONTROLLER_STRUCT CURRENT CONTROLLER STRUCT
  * @brief The current controller's data structure definition.
  * @{
  */

/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief Current controller struct members and parameters.
  */
typedef struct {
    DqAxis *currRef;           /**< Current reference in the d-q coordinate (A). */
    DqAxis *currFdbk;          /**< Current feedback in the d-q coordinate (A). */
    DqAxis currFf;             /**< Current feedforward value (V). */
    PidHandle dAxisPi;         /**< d-axis current PI controller. */
    PidHandle qAxisPi;         /**< q-axis current PI controller. */
    MtrParamHandle *mtrParam;  /**< Motor parameters. */
    float outLimit;            /**< Current controller output voltage limitation (V). */
    float ctrlPeriod;          /**< Current controller control period (s). */
} CurrCtrlHandle;
/**
  * @}
  */

/**
  * @defgroup CURRENT_CONTROLLER_API CURRENT CONTROLLER API
  * @brief The current controller's API declaration.
  * @{
  */
void CURRCTRL_Reset(CurrCtrlHandle *currHandle);
void CURRCTRL_Clear(CurrCtrlHandle *currHandle);
void CURRCTRL_Exec(CurrCtrlHandle *currHandle, DqAxis *voltRef);
/**
  * @}
  */

/**
  * @}
  */

#endif
