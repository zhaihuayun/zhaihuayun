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
 * @file    pga.c
 * @author  MCU Driver Team.
 * @brief   Programmable Gain Amplifier HAL level module driver.
 *          This file provides firmware functions to manage the following
 *          functionalities of the amplifier
 *           + Programmable Gain Amplifier's Initialization and de-initialization functions
 *           + Set amplifier's gain value
 */
#include "pga.h"
#include "assert.h"

#define PGA_CTRL0_ENABLE 0x07

/**
  * @brief PGA HAL Init
  * @param pgaHandle: PGA handle.
  * @retval BASE_StatusType.
  */
BASE_StatusType HAL_PGA_Init(PGA_Handle *pgaHandle)
{
    PGA_ASSERT_PARAM(pgaHandle != NULL);
    PGA_ASSERT_PARAM(IsPGAInstance(pgaHandle->baseAddress));
    /* PGA parameter check: pga gain value, pga mux selection, pga vin sw check, pga switch */
    PGA_PARAM_CHECK_WITH_RET(pgaHandle->gain <= PGA_PGA_MAX_GAIN, BASE_STATUS_ERROR);
    PGA_PARAM_CHECK_WITH_RET(pgaHandle->pgaMux <= PGA_PAG_MAX_SMUX, BASE_STATUS_ERROR);
    PGA_PARAM_CHECK_WITH_RET(IsPGASwSelection(pgaHandle->pgaSwVinN), BASE_STATUS_ERROR);
    PGA_PARAM_CHECK_WITH_RET(IsPGASwSelection(pgaHandle->pgaSwVinP), BASE_STATUS_ERROR);
    if (pgaHandle->enable == true) {
        pgaHandle->baseAddress->PGA_CTRL0.reg |= PGA_CTRL0_ENABLE;            /* Overall enable */
        pgaHandle->baseAddress->PGA_CTRL2.BIT.pga_gain = pgaHandle->gain;     /* PGA gain value */
        pgaHandle->baseAddress->PGA_CTRL2.BIT.pga_smux = pgaHandle->pgaMux;   /* Input channel selection. */
        pgaHandle->baseAddress->PGA_CTRL3.BIT.pga_sw_enlv_p = pgaHandle->pgaSwVinP; /* PGA Vin P input channel. */
        pgaHandle->baseAddress->PGA_CTRL3.BIT.pga_sw_enlv_n = pgaHandle->pgaSwVinN; /* PGA Vin N input channel */
        pgaHandle->baseAddress->PGA_CTRL3.BIT.pga_ext_loopback = pgaHandle->extLoopbackEn;
    } else {
        pgaHandle->baseAddress->PGA_CTRL0.reg &= (~PGA_CTRL0_ENABLE);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief PGA HAL DeInit
  * @param pgaHandle: PGA handle.
  * @retval BASE_StatusType.
  */
BASE_StatusType HAL_PGA_DeInit(PGA_Handle *pgaHandle)
{
    PGA_ASSERT_PARAM(pgaHandle != NULL);
    PGA_ASSERT_PARAM(IsPGAInstance(pgaHandle->baseAddress));
    pgaHandle->baseAddress->PGA_CTRL0.reg = BASE_CFG_DISABLE;  /* Disable PGA. */
    pgaHandle->baseAddress->PGA_CTRL2.reg = BASE_CFG_DISABLE;  /* Gain and channel deinitialization. */
    pgaHandle->baseAddress->PGA_CTRL3.reg = BASE_CFG_DISABLE;  /* Deinitialize the loopback switch and SW switch. */
    return BASE_STATUS_OK;
}

/**
  * @brief Set Gain value
  * @param pgaHandle: PGA handle.
  * @param gain: gain value. @ref PGA_GainValue
  * @retval None.
  */
void HAL_PGA_SetGain(PGA_Handle *pgaHandle, PGA_GainValue gain)
{
    PGA_ASSERT_PARAM(pgaHandle != NULL);
    PGA_ASSERT_PARAM(IsPGAInstance(pgaHandle->baseAddress));
    PGA_PARAM_CHECK_NO_RET(IsPGAGainValue(gain));         /* Verify the incremental value of the PGA. */
    pgaHandle->baseAddress->PGA_CTRL2.BIT.pga_gain = gain;
}