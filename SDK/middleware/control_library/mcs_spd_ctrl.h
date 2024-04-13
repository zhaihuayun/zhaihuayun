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
  * @file      mcs_spd_ctrl.h
  * @author    MCU Algorithm Team
  * @brief     Speed controller for motor control.
  *            This file provides functions declaration of the speed controller module.
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_MCS_SPD_CTRL_H
#define McuMagicTag_MCS_SPD_CTRL_H

/* Includes ------------------------------------------------------------------------------------ */
#include "mcs_typedef.h"
#include "mcs_pid_ctrl.h"
#include "mcs_mtr_param.h"

/**
  * @defgroup SPEED_CONTROLLER  SPEED CONTROLLER
  * @brief The speed controller module
  * @{
  */

/**
  * @defgroup SPEED_CONTROLLER_STRUCT  SPEED CONTROLLER STRUCT
  * @brief The speed controller data struct.
  * @{
  */
/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief Speed controller struct members and parameters.
  */
typedef struct {
    PidHandle spdPi;           /**< PI controller struct in the speed controller. */
    float trqLimit;            /**< Maximum of the speed controller output torque (Nm). */
    MtrParamHandle *mtrParam;  /**< Motor parameters. */
    float ctrlPeriod;          /**< Speed controller control period (s). */
} SpdCtrlHandle;
/**
  * @}
  */

/**
  * @defgroup SPEED_CONTROLLER_API  SPEED CONTROLLER API
  * @brief The speed controller API declaration.
  * @{
  */
void SPDCTRL_Reset(SpdCtrlHandle *spdHandle);
void SPDCTRL_Clear(SpdCtrlHandle *spdHandle);
void SPDCTRL_Exec(SpdCtrlHandle *spdHandle, float spdTarget, float spdFdbk, DqAxis *currRef);
/**
  * @}
  */

/**
  * @}
  */
#endif
