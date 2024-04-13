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
 * @file    acmp.c
 * @author  MCU Driver Team.
 * @brief   ACMP HAL level module driver.
 *          This file provides firmware functions to manage the following
 *          functionalities of the DAC and Comparator.
 *           + Comparator's Initialization and de-initialization functions
 *           + Set Comparator's hysteresis voltage function
 *           + Set software blking valid function
 *           + Set software blking invalid function
 */
#include "acmp.h"
#include "assert.h"

/**
  * @brief Input and output initialization of comparator
  * @param acmpHandle: ACMP handle.
  * @retval None.
  */
static void ACMP_InputOutputInit(ACMP_Handle *acmpHandle)
{
    ACMP_ASSERT_PARAM(acmpHandle != NULL);
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    ACMP_PARAM_CHECK_NO_RET(IsACMPOutputPolarity(acmpHandle->inOutConfig.polarity));
    /* Check input multiplexing selection and input switch selection */
    ACMP_PARAM_CHECK_NO_RET(IsACMPSwitchVinNumber(acmpHandle->inOutConfig.swVinNNum));
    ACMP_PARAM_CHECK_NO_RET(IsACMPSwitchVinNumber(acmpHandle->inOutConfig.swVinPNum));
    ACMP_PARAM_CHECK_NO_RET(IsACMPVinNumber(acmpHandle->inOutConfig.vinNNum));
    ACMP_PARAM_CHECK_NO_RET(IsACMPVinNumber(acmpHandle->inOutConfig.vinPNum));
    /* input mux selection */
    acmpHandle->baseAddress->CMP_CTRL1.BIT.cmp_mux_p =  acmpHandle->inOutConfig.vinPNum;
    acmpHandle->baseAddress->CMP_CTRL1.BIT.cmp_mux_n = acmpHandle->inOutConfig.vinNNum;
    /* input switch selection */
    acmpHandle->baseAddress->CMP_SW.BIT.cmp_sw_enlv_p = acmpHandle->inOutConfig.swVinPNum;
    acmpHandle->baseAddress->CMP_SW.BIT.cmp_sw_enlv_n = acmpHandle->inOutConfig.swVinNNum;
    /* output polarity selection */
    acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_rslt_inv = acmpHandle->inOutConfig.polarity;
}

/**
  * @brief Filter initialization of comparator
  * @param acmpHandle: ACMP handle.
  * @retval None.
  */
static void ACMP_FilterInit(ACMP_Handle *acmpHandle)
{
    ACMP_ASSERT_PARAM(acmpHandle != NULL);
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    /* Check input parameters: Blking APT mask window num, Blking source */
    ACMP_PARAM_CHECK_NO_RET(IsACMPBlkingSrcType(acmpHandle->filterCtrl.blkingSrcSelect));
    ACMP_PARAM_CHECK_NO_RET(IsACMPAptMaskWindow(acmpHandle->filterCtrl.blkingFromAptNum));
    unsigned short blkingSrc;
    switch (acmpHandle->filterCtrl.filterMode) {
        case ACMP_FILTER_NONE:
            acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_blking_en = BASE_CFG_DISABLE; /* Disable Blking */
            acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_qual_en = BASE_CFG_DISABLE; /* Disable Deshark */
            break;
        case ACMP_FILTER_BLKING:
            acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_blking_en = BASE_CFG_ENABLE; /* Enable Blking */
            blkingSrc = acmpHandle->filterCtrl.blkingSrcSelect;
            acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_blking_sel = blkingSrc; /* Setting Blking source */
            if (blkingSrc == ACMP_BLKING_SRC_APT) {
                /* Setting Blking source apt number */
                acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_apt_sel = acmpHandle->filterCtrl.blkingFromAptNum;
            }
            break;
        case ACMP_FILTER_DESHARK:
            /* Enable Deshark */
            acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_qual_en = BASE_CFG_ENABLE;
            /* Setting deshark by step */
            acmpHandle->baseAddress->CMP_QUALI2.BIT.cmp_qual_step = acmpHandle->filterCtrl.desharkByStep;
            /* Setting deshark by times */
            acmpHandle->baseAddress->CMP_QUALI2.BIT.cmp_qual_sel = acmpHandle->filterCtrl.desharkByTimes;
            break;
        case ACMP_FILTER_BOTH:
            acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_blking_en = BASE_CFG_ENABLE; /* Enable Blking */
            blkingSrc = acmpHandle->filterCtrl.blkingSrcSelect;
            acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_blking_sel = blkingSrc; /* Setting Blking source */
            if (blkingSrc == ACMP_BLKING_SRC_APT) {
                /* Setting Blking source apt number */
                acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_apt_sel = acmpHandle->filterCtrl.blkingFromAptNum;
            }
            acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_qual_en = BASE_CFG_ENABLE; /* Enable Deshark */
            /* Setting deshark by step */
            acmpHandle->baseAddress->CMP_QUALI2.BIT.cmp_qual_step = acmpHandle->filterCtrl.desharkByStep;
            /* Setting deshark by times */
            acmpHandle->baseAddress->CMP_QUALI2.BIT.cmp_qual_sel = acmpHandle->filterCtrl.desharkByTimes;
            break;
        default:
            acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_blking_en = BASE_CFG_DISABLE; /* Disable Blking */
            acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_qual_en = BASE_CFG_DISABLE; /* Disable Deshark */
            break;
    }
}

/**
  * @brief Comparator HAL Init
  * @param acmpHandle: ACMP handle.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_ACMP_Init(ACMP_Handle *acmpHandle)
{
    ACMP_ASSERT_PARAM(acmpHandle != NULL);
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    ACMP_PARAM_CHECK_WITH_RET(acmpHandle->hysteresisVol >=  ACMP_HYS_VOL_ZERO, BASE_STATUS_ERROR);
    ACMP_PARAM_CHECK_WITH_RET(acmpHandle->hysteresisVol <= ACMP_HYS_VOL_30MV, BASE_STATUS_ERROR);
    ACMP_PARAM_CHECK_WITH_RET(acmpHandle->filterCtrl.desharkByStep >= 0, BASE_STATUS_ERROR);
    ACMP_PARAM_CHECK_WITH_RET(acmpHandle->filterCtrl.desharkByStep <= ACMP_DESHARK_BY_CLK_MAX, BASE_STATUS_ERROR);
    ACMP_PARAM_CHECK_WITH_RET(acmpHandle->filterCtrl.desharkByTimes >= 0, BASE_STATUS_ERROR);
    ACMP_PARAM_CHECK_WITH_RET(acmpHandle->filterCtrl.desharkByTimes <= ACMP_DESHARK_BY_CMP_MAX, BASE_STATUS_ERROR);
    if (acmpHandle->enable == false) {
        acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_ana_en = BASE_CFG_DISABLE;
        acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_dig_en = BASE_CFG_DISABLE;
        return BASE_STATUS_OK;
    } else {
        acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_ana_en = BASE_CFG_ENABLE;
        acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_dig_en = BASE_CFG_ENABLE;
    }
    ACMP_InputOutputInit(acmpHandle);
    ACMP_FilterInit(acmpHandle);
    /* synchronize */
    if (acmpHandle->syncEn == true) {
        acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_sync_sel = BASE_CFG_ENABLE;
    } else {
        acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_sync_sel = BASE_CFG_DISABLE;
    }
    /* set hysteresis voltage */
    HAL_ACMP_SetHystVol(acmpHandle, acmpHandle->hysteresisVol);

    return BASE_STATUS_OK;
}

/**
  * @brief Comparator HAL DeInit
  * @param acmpHandle: ACMP handle.
  * @retval BASE_StatusType: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_ACMP_DeInit(ACMP_Handle *acmpHandle)
{
    ACMP_ASSERT_PARAM(acmpHandle != NULL);
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    acmpHandle->baseAddress->CMP_CTRL.reg = BASE_CFG_DISABLE;   /* Disable ACMP. */
    acmpHandle->baseAddress->CMP_CTRL1.reg = BASE_CFG_DISABLE;  /* Deinitializes the comparison input setting. */
    acmpHandle->baseAddress->CMP_SW.reg = BASE_CFG_DISABLE;    /* Deinitializes the SW control. */
    return BASE_STATUS_OK;
}

/**
  * @brief Set hysteresis Voltage
  * @param acmpHandle: ACMP handle.
  * @param voltage: hysteresis voltage to be set. @ref ACMP_HystVol
  * @retval None.
  */
void HAL_ACMP_SetHystVol(ACMP_Handle *acmpHandle, ACMP_HystVol voltage)
{
    ACMP_ASSERT_PARAM(acmpHandle != NULL);
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    ACMP_PARAM_CHECK_NO_RET(voltage >=  ACMP_HYS_VOL_ZERO);
    ACMP_PARAM_CHECK_NO_RET(voltage <= ACMP_HYS_VOL_30MV);
    acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_hy_vol_sel = voltage;
}

/**
  * @brief Set Software Blking Valid
  * @param acmpHandle: ACMP handle.
  * @retval None.
  */
void HAL_ACMP_SoftBlkingValid(ACMP_Handle *acmpHandle)
{
    ACMP_ASSERT_PARAM(acmpHandle != NULL);
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_blking_cfg = BASE_CFG_ENABLE;
}

/**
  * @brief Set Software Blking Invalid
  * @param acmpHandle: ACMP handle.
  * @retval None.
  */
void HAL_ACMP_SoftBlkingInvalid(ACMP_Handle *acmpHandle)
{
    ACMP_ASSERT_PARAM(acmpHandle != NULL);
    ACMP_ASSERT_PARAM(IsACMPInstance(acmpHandle->baseAddress));
    acmpHandle->baseAddress->CMP_CTRL.BIT.cmp_blking_cfg = BASE_CFG_DISABLE;
}
