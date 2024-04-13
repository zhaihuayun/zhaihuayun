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
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * @file      mcs_fw_ctrl.c
  * @author    MCU Algorithm Team
  * @brief     This file provides Flux-Weakening control for motor control.
  */
#include "mcs_fw_ctrl.h"
#include "mcs_math.h"
#include "mcs_math_const.h"

/**
  * @brief Clear historical values of Flux-Weakening handle.
  * @param FwCtrlHandle Flux-Weakening struct handle.
  * @retval None.
  */
static void FwCtrl_Clear(FwCtrlHandle *fw)
{
    fw->idRef = 0.0f;
}

/**
  * @brief Flux-Weakening control Handle Initialization.
  * @param FwCtrlHandle Flux-Weakening struct handle.
  * @param ts
  * @param enable
  * @retval None.
  */
void FwCtrl_Init(FwCtrlHandle *fw, float ts, int enable)
{
    /* Indicates whether to enable the Flux-Weakening field function. */
    fw->enable = enable;
    fw->ts = ts;
    fw->udcThreshPer = SPECIAL_FW_CTRL_VOLT_BUS_THRESHOLD_PER * ONE_DIV_SQRT3;
    /* id control step */
    fw->idStep = SPECIAL_FW_CTRL_ID_CHANGE_PER_SEC * ts;
    fw->idMaxAmp = \
    (SPECIAL_FW_CTRL_OC_AMP < SPECIAL_FW_CTRL_ID_DEMAG_AMP) ? SPECIAL_FW_CTRL_OC_AMP : SPECIAL_FW_CTRL_ID_DEMAG_AMP;
    FwCtrl_Clear(fw);
}

/**
  * @brief Flux-Weakening calculation execution function.
  * @param FwCtrlHandle Flux-Weakening struct handle.
  * @param udqRef dq axis voltage reference.
  * @param udc bus voltage.
  * @param idRefRaw  Command value of the d axis current.
  * @retval None.
  */
float FwCtrl_Exec(FwCtrlHandle *fw, DqAxis udqRef, float udc, float idRefRaw)
{
    float udcLimit = udc * fw->udcThreshPer;
    float voltRefAmp = ASM_Sqrt(udqRef.d * udqRef.d + udqRef.q * udqRef.q);
    float voltErr = udcLimit - voltRefAmp;
    /* Check whether the Flux-Weakening field function is enabled.  */
    if (!fw->enable) {
        fw->idRef = idRefRaw;
        return fw->idRef;
    }
    /* Adjust the injection ID based on the output voltage error. */
    /* The voltage error is positive. -id of weakening injection. */
    if (voltErr >= 0.0f) {
        fw->idRef += fw->idStep;
        if (fw->idRef > idRefRaw) {
            fw->idRef = idRefRaw;
        }
    }
    /* The voltage error is negative. Add the ingested -id. */
    if (voltErr < 0.0f) {
        fw->idRef -= fw->idStep;
        if (fw->idRef < -fw->idMaxAmp) {
            fw->idRef = -fw->idMaxAmp;
        }
    }
    return fw->idRef;
}
