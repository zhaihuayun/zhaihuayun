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
  * @file      mcs_pid_ctrl.c
  * @author    MCU Algorithm Team
  * @brief     This file provides functions of general PID controller
  */

#include "mcs_pid_ctrl.h"
#include "mcs_math.h"
#include "mcs_assert.h"

/**
  * @brief Reset all member variables of PID controller to zero.
  * @param pidHandle PID controller struct handle.
  * @retval None.
  */
void PID_Reset(PidHandle *pidHandle)
{
    MCS_ASSERT_PARAM(pidHandle != NULL);
    /* Reset the PID parameter. */
    pidHandle->error       = 0.0f;
    pidHandle->errorLast   = 0.0f;
    pidHandle->feedforward = 0.0f;
    pidHandle->integral    = 0.0f;
    pidHandle->saturation  = 0.0f;
    pidHandle->differ      = 0.0f;
    pidHandle->kp          = 0.0f;
    pidHandle->ki          = 0.0f;
    pidHandle->kd          = 0.0f;
    pidHandle->ns          = 0.0f;
    pidHandle->ka          = 0.0f;
    /* Reset the Limiting Value. */
    pidHandle->upperLimit  = 0.0f;
    pidHandle->lowerLimit  = 0.0f;
}

/**
  * @brief Clear historical values of PID controller.
  * @param pidHandle PID controller struct handle.
  * @retval None.
  */
void PID_Clear(PidHandle *pidHandle)
{
    MCS_ASSERT_PARAM(pidHandle != NULL);
    /* Clear historical values of PID controller. */
    pidHandle->differ      = 0.0f;
    pidHandle->integral    = 0.0f;
    pidHandle->saturation  = 0.0f;
    pidHandle->feedforward = 0.0f;
    pidHandle->error       = 0.0f;
    pidHandle->errorLast   = 0.0f;
}

/**
  * @brief Execute simplified PI controller calculation, static clamping, no feedfoward.
  * @param pidHandle PI controller struct handle.
  * @retval PI control output.
  */
float PI_Exec(PidHandle *pidHandle)
{
    MCS_ASSERT_PARAM(pidHandle != NULL);
    /* Proportional Item */
    float p = pidHandle->kp * pidHandle->error;

    /* Integral Item */
    float i = pidHandle->ki * pidHandle->error + pidHandle->integral;
    i = Clamp(i, pidHandle->upperLimit, pidHandle->lowerLimit);
    pidHandle->integral = i;

    /* static clamping and output calculaiton */
    float val = p + i;
    float out = Clamp(val, pidHandle->upperLimit, pidHandle->lowerLimit);

    return out;
}

/**
  * @brief Execute PID controller calculation. dynamic clamping, feedfoward compenstaion
  * @param pidHandle PID controller struct handle.
  * @retval PID control output.
  */
float PID_Exec(PidHandle *pidHandle)
{
    MCS_ASSERT_PARAM(pidHandle != NULL);
    /* Proportional Item */
    float p = pidHandle->kp * pidHandle->error;

    /* Integral Item */
    float i = pidHandle->ki * (pidHandle->error - pidHandle->ka * pidHandle->saturation) + pidHandle->integral;
    i = Clamp(i, Max(0.0f, pidHandle->upperLimit), Min(0.0f, pidHandle->lowerLimit));
    pidHandle->integral = i;

    /* Differential Item */
    float d = pidHandle->kd * pidHandle->ns * (pidHandle->error - pidHandle->errorLast) - \
              pidHandle->differ * (pidHandle->ns - 1.0f);

    pidHandle->errorLast = pidHandle->error;
    pidHandle->differ = d;

    /* Output value update and saturation value calculation */
    float val = p + i + d + pidHandle->feedforward;
    float out = Clamp(val, pidHandle->upperLimit, pidHandle->lowerLimit);
    pidHandle->saturation = val - out;

    return out;
}

/**
  * @brief Set the proportional parameter kp of PID controller.
  * @param pidHandle PID controller struct handle.
  * @param kp The proportional parameter.
  * @retval None.
  */
void PID_SetKp(PidHandle *pidHandle, float kp)
{
    MCS_ASSERT_PARAM(pidHandle != NULL);
    MCS_ASSERT_PARAM(kp >= 0.0f);
    pidHandle->kp = kp;
}

/**
  * @brief Set the integral parameter ki of PID controller.
  * @param pidHandle PID controller struct handle.
  * @param ki The integral parameter.
  * @retval None.
  */
void PID_SetKi(PidHandle *pidHandle, float ki)
{
    MCS_ASSERT_PARAM(pidHandle != NULL);
    MCS_ASSERT_PARAM(ki >= 0.0f);
    pidHandle->ki = ki;
}

/**
  * @brief Set the derivative parameter kd of PID controller.
  * @param pidHandle PID controller struct handle.
  * @param kd The derivative parameter.
  * @retval None.
  */
void PID_SetKd(PidHandle *pidHandle, float kd)
{
    MCS_ASSERT_PARAM(pidHandle != NULL);
    MCS_ASSERT_PARAM(kd >= 0.0f);
    pidHandle->kd = kd;
}
