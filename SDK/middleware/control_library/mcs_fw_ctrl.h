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
  * @file      mcs_fw_ctrl.h
  * @author    MCU Algorithm Team
  * @brief     This file provides functions of Flux-Weakening control.
  */
#ifndef McuMagicTag_MCS_SPECIAL_FW_CTRL_H
#define McuMagicTag_MCS_SPECIAL_FW_CTRL_H

#include "mcs_typedef.h"

#define SPECIAL_FW_CTRL_ID_CHANGE_PER_SEC (5.0f)
#define SPECIAL_FW_CTRL_VOLT_BUS_THRESHOLD_PER (0.95f)
#define SPECIAL_FW_CTRL_ID_DEMAG_AMP (10.0f)
#define SPECIAL_FW_CTRL_OC_AMP (7.0f)

typedef struct {
    int   enable;
    float udcThreshPer;
    float ts;
    float idStep;   /* iD injection control step */
    float idRef;    /* reference instruction value. */
    float idMaxAmp; /* Maximum id ingested */
} FwCtrlHandle;

void FwCtrl_Init(FwCtrlHandle *fw, float ts, int enable);
float FwCtrl_Exec(FwCtrlHandle *fw, DqAxis udqRef, float udc, float idRefRaw);

#endif