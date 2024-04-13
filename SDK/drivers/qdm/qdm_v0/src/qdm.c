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
 * @file    qdm.c
 * @author  MCU Driver Team.
 * @brief   QDM HAL level module driver.
 * @details This file provides firmware functions to manage the following
 *          functionalities of the QDM.
 *           + Initialization and de-initialization functions.
 *           + Qdm Module Control functions.
 *           + Speed measure use M function.
 *           + Stall condition detection.
 */
#include "qdm.h"
#include "interrupt.h"

/**
  * @brief Set Decoder configurations
  * @param qdmHandle Value of @ref QDM_Handle.
  * @retval None
  */
static void QDM_DecoderConfig(QDM_Handle *qdmHandle)
{
    QDM_ASSERT_PARAM(qdmHandle != NULL);
    QDM_ASSERT_PARAM(IsQDMInstance(qdmHandle->baseAddress));
    QDM_PARAM_CHECK_NO_RET(IsInputMode(qdmHandle->ctrlConfig.inputMode));
    QDM_PARAM_CHECK_NO_RET(IsSwap(qdmHandle->ctrlConfig.swap));
    QDM_PARAM_CHECK_NO_RET(IsResolution(qdmHandle->ctrlConfig.resolution));
    QDM_PARAM_CHECK_NO_RET(IsTrgLockMode(qdmHandle->ctrlConfig.trgLockMode));
    QDM_PARAM_CHECK_NO_RET(IsPtuMode(qdmHandle->ctrlConfig.ptuMode));

    /* input mode setting */
    qdmHandle->baseAddress->QCTRL.BIT.qdu_mode = qdmHandle->ctrlConfig.inputMode;
    /* swap */
    qdmHandle->baseAddress->QCTRL.BIT.qdm_ab_swap = qdmHandle->ctrlConfig.swap;
    /* qdm xclk */
    qdmHandle->baseAddress->QCTRL.BIT.qdu_xclk = qdmHandle->ctrlConfig.resolution;
    /* polarity */
    /* bit0: A input polarity, bit value: 0--direct input, 1--invert input */
    qdmHandle->baseAddress->QCTRL.BIT.qdma_polarity = (qdmHandle->ctrlConfig.polarity & 0x01);
    /* bit1: B input polarity, bit value: 0--direct input, 1--invert input */
    qdmHandle->baseAddress->QCTRL.BIT.qdmb_polarity = ((qdmHandle->ctrlConfig.polarity >> 1) & 0x01);
    /* bit2: index input polarity, bit value: 0--direct input, 1--invert input */
    qdmHandle->baseAddress->QCTRL.BIT.qdmi_polarity = ((qdmHandle->ctrlConfig.polarity >> 2) & 0x01);
    /* lock mode */
    qdmHandle->baseAddress->QCTRL.BIT.qtrg_lock_mode = qdmHandle->ctrlConfig.trgLockMode;
    /* ptu mode */
    qdmHandle->baseAddress->QCTRL.BIT.ptu_mode = qdmHandle->ctrlConfig.ptuMode;
}

/**
  * @brief Set counter configurations
  * @param qdmHandle Value of @ref QDM_Handle.
  * @retval None
  */
static void QDM_CounterConfig(QDM_Handle *qdmHandle)
{
    QDM_ASSERT_PARAM(qdmHandle != NULL);
    QDM_ASSERT_PARAM(IsQDMInstance(qdmHandle->baseAddress));
    QDM_PARAM_CHECK_NO_RET(IsPcntMode(qdmHandle->pcntMode));
    QDM_PARAM_CHECK_NO_RET(IsPcntRstMode(qdmHandle->pcntRstMode));
    QDM_PARAM_CHECK_NO_RET(IsPcntIdxInitMode(qdmHandle->pcntIdxInitMode));
    QDM_PARAM_CHECK_NO_RET(IsTsuPrescaler(qdmHandle->tsuPrescaler));
    QDM_PARAM_CHECK_NO_RET(IsCevtPrescaler(qdmHandle->cevtPrescaler));

    /* set pcnt mode */
    qdmHandle->baseAddress->QPPUCTRL.BIT.pcnt_mode = qdmHandle->pcntMode;
    qdmHandle->baseAddress->QPPUCTRL.BIT.pcnt_rst_mode = qdmHandle->pcntRstMode;
    qdmHandle->baseAddress->QPPUCTRL.BIT.pcnt_idx_init_mode = qdmHandle->pcntIdxInitMode;
    /* set TSU */
    qdmHandle->baseAddress->QTSUCTRL.BIT.tsu_prescaler = qdmHandle->tsuPrescaler;
    qdmHandle->baseAddress->QTSUCTRL.BIT.cevt_prescaler = qdmHandle->cevtPrescaler;
    /* set init value */
    qdmHandle->baseAddress->QPOSINIT = qdmHandle->posInit;
    /* set count max value */
    qdmHandle->baseAddress->QPOSMAX  = qdmHandle->posMax;
    qdmHandle->baseAddress->QUPRD    = qdmHandle->period;
    qdmHandle->baseAddress->QCMAX = qdmHandle->qcMax;
}

/**
  * @brief enable submodules
  * @param qdmHandle Value of @ref QDM_Handle.
  * @retval None
  */
static void QDM_EnableSubmodule(QDM_Handle *qdmHandle)
{
    QDM_ASSERT_PARAM(qdmHandle != NULL);
    QDM_ASSERT_PARAM(IsQDMInstance(qdmHandle->baseAddress));
    if (qdmHandle->subModeEn == true) {
        qdmHandle->baseAddress->QCTRL.BIT.ppu_en = BASE_CFG_ENABLE;
        qdmHandle->baseAddress->QCTRL.BIT.ptu_en = BASE_CFG_ENABLE;
        qdmHandle->baseAddress->QCTRL.BIT.tsu_en = BASE_CFG_ENABLE;
    }
}

/**
  * @brief enable interrupt
  * @param qdmHandle Value of @ref QDM_Handle.
  * @retval None
  */
static void QDM_InterruptEnable(QDM_Handle *qdmHandle)
{
    QDM_ASSERT_PARAM(qdmHandle != NULL);
    QDM_ASSERT_PARAM(IsQDMInstance(qdmHandle->baseAddress));
    qdmHandle->baseAddress->QINTENA.reg = qdmHandle->interruptEn;
    IRQ_EnableN(qdmHandle->irqNum);
}

/**
 * @brief Speed lose interrupt.
 * @param qdmHandle Value of @ref QDM_Handle.
 * @retval None
 */
static void SpeedLose(QDM_Handle *qdmHandle)
{
    if (qdmHandle->QDM_SpeedLoseCallback != NULL) {
        qdmHandle->QDM_SpeedLoseCallback(qdmHandle);
    }
}

/**
 * @brief QDM Z index lock interrupt.
 * @param qdmHandle Value of @ref QDM_Handle.
 * @retval None
 */
static void ZIndexLock(QDM_Handle *qdmHandle)
{
    if (qdmHandle->ZIndexLockedCallBack != NULL) {
        qdmHandle->ZIndexLockedCallBack(qdmHandle);
    }
}

/**
 * @brief Orthogonal direction change interrupt.
 * @param qdmHandle Value of @ref QDM_Handle.
 * @retval None
 */
static void OrthoDirChange(QDM_Handle *qdmHandle)
{
    if (qdmHandle->OrthogonalDirectionChangeCallBack != NULL) {
        qdmHandle->OrthogonalDirectionChangeCallBack(qdmHandle);
    }
}

/**
 * @brief Orthogonal phase error interrupt.
 * @param qdmHandle Value of @ref QDM_Handle.
 * @retval None
 */
static void OrthoPhaseErr(QDM_Handle *qdmHandle)
{
    if (qdmHandle->OrthogonalPhaseErrorCallBack != NULL) {
        qdmHandle->OrthogonalPhaseErrorCallBack(qdmHandle);
    }
}

/**
 * @brief Position compare match interrupt.
 * @param qdmHandle Value of @ref QDM_Handle.
 * @retval None
 */
static void PosCompareMatch(QDM_Handle *qdmHandle)
{
    if (qdmHandle->PositionCompareMatchCallBack != NULL) {
        qdmHandle->PositionCompareMatchCallBack(qdmHandle);
    }
}

/**
 * @brief Position compare ready interrupt.
 * @param qdmHandle Value of @ref QDM_Handle.
 * @retval None
 */
static void PosCompareReady(QDM_Handle *qdmHandle)
{
    if (qdmHandle->PositionCompareReadyCallBack != NULL) {
        qdmHandle->PositionCompareReadyCallBack(qdmHandle);
    }
}

/**
 * @brief Position counter error interrupt.
 * @param qdmHandle Value of @ref QDM_Handle.
 * @retval None
 */
static void PosCounterErr(QDM_Handle *qdmHandle)
{
    if (qdmHandle->PositionCounterErrorCallBack != NULL) {
        qdmHandle->PositionCounterErrorCallBack(qdmHandle);
    }
}

/**
 * @brief Position counter overflow interrupt.
 * @param qdmHandle Value of @ref QDM_Handle.
 * @retval None
 */
static void PosCounterOverflow(QDM_Handle *qdmHandle)
{
    if (qdmHandle->PositionCounterOverFlowCallBack != NULL) {
        qdmHandle->PositionCounterOverFlowCallBack(qdmHandle);
    }
}

/**
 * @brief Position counter underflow interrupt.
 * @param qdmHandle Value of @ref QDM_Handle.
 * @retval None
 */
static void PosCounterUnderflow(QDM_Handle *qdmHandle)
{
    if (qdmHandle->PositionCounterUnderFlowCallBack != NULL) {
        qdmHandle->PositionCounterUnderFlowCallBack(qdmHandle);
    }
}

/**
 * @brief Other interrupt callback function.
 * @param qdmHandle Value of @ref QDM_Handle.
 * @param qinsts: Interrupt status register.
 * @retval None
 */
static void OtherInterruptCallBack(QDM_Handle *qdmHandle, QDM_QINTSTS_REG qinsts)
{
    if (qinsts.BIT.qphs_err_int == BASE_CFG_SET) {
        /* Orthogonal phase error interrupt. */
        OrthoPhaseErr(qdmHandle);
    }
    if (qinsts.BIT.pcnt_cpm_int == BASE_CFG_SET) {
        /* Position compare match interrupt. */
        PosCompareMatch(qdmHandle);
    }
    if (qinsts.BIT.pcnt_cpr_int == BASE_CFG_SET) {
        /* Position compare ready interrupt. */
        PosCompareReady(qdmHandle);
    }
    if (qinsts.BIT.pcnt_err_int == BASE_CFG_SET) {
        /* Position counter error interrupt. */
        PosCounterErr(qdmHandle);
    }
    if (qinsts.BIT.pcnt_ovf_int == BASE_CFG_SET) {
        /* Position counter overflow interrupt. */
        PosCounterOverflow(qdmHandle);
    }
    if (qinsts.BIT.pcnt_udf_int == BASE_CFG_SET) {
        /* Position counter underflow interrupt. */
        PosCounterUnderflow(qdmHandle);
    }
}

/**
  * @brief IRQ Handler
  * @param qdmHandle Value of @ref QDM_Handle.
  * @retval None
  */
void HAL_QDM_IRQHandler(void *handle)
{
    QDM_ASSERT_PARAM(handle != NULL);
    QDM_Handle *qdmHandle = (QDM_Handle *)handle;
    unsigned int posLockValue;
    int speed;
    if (qdmHandle->motorLineNum == 0 || qdmHandle->period == 0) {
        /* clear interrupt */
        qdmHandle->baseAddress->QINTRAW.BIT.sped_lst_raw = BASE_CFG_ENABLE;
        qdmHandle->baseAddress->QINTRAW.BIT.utmr_prd_raw = BASE_CFG_ENABLE;
        IRQ_ClearN(qdmHandle->irqNum); /* clear QDM interrupt */
        return;
    }
    
    QDM_QINTSTS_REG qinsts = qdmHandle->baseAddress->QINTSTS;
    if (qinsts.BIT.utmr_prd_int == BASE_CFG_SET) {
        posLockValue = qdmHandle->baseAddress->QPOSLOCK;
        if (qdmHandle->baseAddress->QDMSTS.BIT.qdir_sts == 1) { /* forward */
            speed = (((posLockValue) * SECONDS_PER_MINUTES) / qdmHandle->motorLineNum) \
         * (BASE_FUNC_GetCpuFreqHz() / qdmHandle->period);
            qdmHandle->speedRpm = speed;
        } else { /* reverse */
            speed = (((qdmHandle->qcMax - posLockValue) * SECONDS_PER_MINUTES) / qdmHandle->motorLineNum) \
         * (BASE_FUNC_GetCpuFreqHz() / qdmHandle->period);
            qdmHandle->speedRpm = -speed;
        }
        /* PTU timer cycle triggle interrupt */
        if (qdmHandle->QDM_PTUCycleTrgCallback != NULL) {
            qdmHandle->QDM_PTUCycleTrgCallback(qdmHandle);
        }
    }
    if (qinsts.BIT.sped_lst_int == BASE_CFG_SET) {
        /* speed lose interrupt */
        SpeedLose(qdmHandle);
    }
    if (qinsts.BIT.indx_lck_int == BASE_CFG_SET) {
        /* QDM Z index lock interrupt. */
        ZIndexLock(qdmHandle);
    }
    if (qinsts.BIT.qdir_chg_int == BASE_CFG_SET) {
        /* Orthogonal direction change interrupt. */
        OrthoDirChange(qdmHandle);
    }
    OtherInterruptCallBack(qdmHandle, qinsts);
    /* clear interrupt */
    qdmHandle->baseAddress->QINTRAW.BIT.sped_lst_raw = BASE_CFG_ENABLE;
    qdmHandle->baseAddress->QINTRAW.BIT.utmr_prd_raw = BASE_CFG_ENABLE;
    IRQ_ClearN(qdmHandle->irqNum); /* clear QDM interrupt */

    return;
}

/**
 * @brief Select the interrupt callback function by the switch-case.
 * @param qdmHandle Value of @ref QDM_Handle.
 * @param typeId: Interrupt type.
 * @param pCallBack: Interrupt callback function.
 * @retval None
 */
static void SelectInterruptCallback(QDM_Handle *qdmHandle, QDM_CallbackFuncType typeID, QDM_CallbackType pCallback)
{
    switch (typeID) {
        case QDM_TSU_CYCLE:
            /* PTU timer cycle triggle interrupt. */
            qdmHandle->QDM_PTUCycleTrgCallback = pCallback;
            break;
        case QDM_SPEED_LOSE:
            /* Speed lose interrupt. */
            qdmHandle->QDM_SpeedLoseCallback = pCallback;
            break;
        case QDM_INDEX_LOCKED:
            /* QDM Z index lock interrupt. */
            qdmHandle->ZIndexLockedCallBack = pCallback;
            break;
        case QDM_DIR_CHANGE:
            /* Orthogonal direction change interrupt. */
            qdmHandle->OrthogonalDirectionChangeCallBack = pCallback;
            break;
        case QDM_PHASE_ERROR:
            /* Orthogonal phase error interrupt. */
            qdmHandle->OrthogonalPhaseErrorCallBack = pCallback;
            break;
        case QDM_POS_MATCH:
            /* Position compare match interrupt. */
            qdmHandle->PositionCompareMatchCallBack = pCallback;
            break;
        case QDM_POS_READY:
            /* Position compare ready interrupt. */
            qdmHandle->PositionCompareReadyCallBack = pCallback;
            break;
        case QDM_POS_CNT_ERROR:
            /* Position counter error interrupt. */
            qdmHandle->PositionCounterErrorCallBack = pCallback;
            break;
        case QDM_POS_CNT_OVERFLOW:
            /* Position counter overflow interrupt. */
            qdmHandle->PositionCounterOverFlowCallBack = pCallback;
            break;
        case QDM_POS_CNT_UNDERFLOW:
            /* Position counter underflow interrupt. */
            qdmHandle->PositionCounterUnderFlowCallBack = pCallback;
            break;
        default:
            return;
    }
}

/**
  * @brief Register IRQ  callback functions
  * @param qdmHandle Value of @ref QDM_Handle.
  * @param typeID: callback function type ID.
  * @param pCallback: pointer of callback function.
  * @retval None
  */
void HAL_QDM_RegisterCallback(QDM_Handle *qdmHandle, QDM_CallbackFuncType typeID, QDM_CallbackType pCallback)
{
    QDM_ASSERT_PARAM(qdmHandle != NULL);
    QDM_ASSERT_PARAM(pCallback != NULL);
    QDM_ASSERT_PARAM(IsQDMInstance(qdmHandle->baseAddress));
    /* Select the interrupt callback function by the switch-case. */
    SelectInterruptCallback(qdmHandle, typeID, pCallback);
}

/**
  * @brief register interrupt
  * @param qdmHandle Value of @ref QDM_Handle.
  * @retval None
  */
void HAL_QDM_IRQService(QDM_Handle *qdmHandle)
{
    QDM_ASSERT_PARAM(qdmHandle != NULL);
    IRQ_Register(qdmHandle->irqNum, HAL_QDM_IRQHandler, qdmHandle);
}

/**
  * @brief  QDM initialization functions
  * @param qdmHandle Value of @ref QDM_Handle.
  * @retval BASE_StatusType:BASE_STATUS_OK, BASE_STATUS_ERROR, BASE_STATUS_BUSY, BASE_STATUS_TIMEOUT
  */
BASE_StatusType HAL_QDM_Init(QDM_Handle *qdmHandle)
{
    QDM_ASSERT_PARAM(qdmHandle != NULL);
    QDM_ASSERT_PARAM(IsQDMInstance(qdmHandle->baseAddress));
    QDM_PARAM_CHECK_WITH_RET(IsEmuMode(qdmHandle->emuMode), BASE_STATUS_ERROR);
    QDM_PARAM_CHECK_WITH_RET(IsLockMode(qdmHandle->lock_mode), BASE_STATUS_ERROR);
    qdmHandle->baseAddress->QEMUMODE.BIT.emu_mode = qdmHandle->emuMode;

    /* Set Z index locked mode. */
    if ((qdmHandle->interruptEn & QDM_INT_INDEX_EVNT_LATCH) == QDM_INT_INDEX_EVNT_LATCH) {
        qdmHandle->baseAddress->QPPUCTRL.BIT.pcnt_idx_lock_mode = qdmHandle->lock_mode;
    }

    /* Set input filter width. */
    DCL_QDM_SetInputFilterWidth(qdmHandle->baseAddress, qdmHandle->inputFilter.qdmAFilterLevel, \
                                qdmHandle->inputFilter.qdmBFilterLevel, qdmHandle->inputFilter.qdmZFilterLevel);
    QDM_DecoderConfig(qdmHandle);
    QDM_CounterConfig(qdmHandle);
    /* Enable interrupt. */
    QDM_InterruptEnable(qdmHandle);
    QDM_EnableSubmodule(qdmHandle);
    return BASE_STATUS_OK;
}

/**
  * @brief  QDM deinitialization functions
  * @param qdmHandle Value of @ref QDM_Handle.
  * @retval BASE_StatusType:BASE_STATUS_OK, BASE_STATUS_ERROR, BASE_STATUS_BUSY, BASE_STATUS_TIMEOUT
  */
BASE_StatusType HAL_QDM_DeInit(QDM_Handle *qdmHandle)
{
    QDM_ASSERT_PARAM(qdmHandle != NULL);
    QDM_ASSERT_PARAM(IsQDMInstance(qdmHandle->baseAddress));
    /* Clear QDM iterrupt callback functions. */
    qdmHandle->QDM_PTUCycleTrgCallback = NULL;
    qdmHandle->QDM_SpeedLoseCallback = NULL;
    /* Disable interrupt. */
    qdmHandle->baseAddress->QINTENA.reg = BASE_CFG_DISABLE;
    /* Disable submodules. */
    qdmHandle->baseAddress->QCTRL.BIT.ppu_en = BASE_CFG_DISABLE;
    qdmHandle->baseAddress->QCTRL.BIT.ptu_en = BASE_CFG_DISABLE;
    qdmHandle->baseAddress->QCTRL.BIT.tsu_en = BASE_CFG_DISABLE;
    return BASE_STATUS_OK;
}

/**
  * @brief read position count register value and direct
  * @param qdmHandle Value of @ref QDM_Handle.
  * @param count: count value pointer.
  * @param dir: dir.
  * @retval none.
  */
void HAL_QDM_ReadPosCountAndDir(const QDM_Handle *qdmHandle, unsigned int *count, unsigned int *dir)
{
    QDM_ASSERT_PARAM(qdmHandle != NULL);
    QDM_ASSERT_PARAM(count != NULL);
    QDM_ASSERT_PARAM(dir != NULL);
    QDM_ASSERT_PARAM(IsQDMInstance(qdmHandle->baseAddress));
    *count = qdmHandle->baseAddress->QPOSCNT;
    *dir = qdmHandle->baseAddress->QDMSTS.BIT.qdir_sts;

    return;
}

/**
  * @brief  get phase error status.
  * @param qdmHandle Value of @ref QDM_Handle.
  * @param errStatus: phase error status.
  * @retval none.
  */
void HAL_QDM_GetPhaseErrorStatus(const QDM_Handle *qdmHandle, unsigned int *errStatus)
{
    QDM_ASSERT_PARAM(qdmHandle != NULL);
    QDM_ASSERT_PARAM(errStatus != NULL);
    QDM_ASSERT_PARAM(IsQDMInstance(qdmHandle->baseAddress));
    *errStatus = qdmHandle->baseAddress->QDMSTS.BIT.qcdr_err_sts;

    return;
}

/**
  * @brief  Get motor speed use M method
  * @param qdmHandle Value of @ref QDM_Handle.
  * @retval int: motor's speed
  */
int HAL_QDM_GetSpeedRpmM(QDM_Handle *qdmHandle)
{
    QDM_ASSERT_PARAM(qdmHandle != NULL);
    return qdmHandle->speedRpm;
}

/**
  * @brief  Get motor speed use MT method
  * @param qdmHandle Value of @ref QDM_Handle.
  * @retval int: motor's speed
  */
int HAL_QDM_GetSpeedRpmMT(QDM_Handle *qdmHandle)
{
    int rpm;
    unsigned int utime;
    unsigned int tmp;

    QDM_ASSERT_PARAM(qdmHandle != NULL);
    QDM_ASSERT_PARAM(IsQDMInstance(qdmHandle->baseAddress));
    QDM_ASSERT_PARAM(qdmHandle->motorLineNum != 0);
    qdmHandle->baseAddress->QDMSTS.BIT.cevt_sts = BASE_CFG_SET; /* clear cevt status bit */
    while (qdmHandle->baseAddress->QDMSTS.BIT.cevt_sts != BASE_CFG_SET) {
        ;
    }
    if (qdmHandle->baseAddress->QDMSTS.BIT.qctmr_ovf_sts == BASE_CFG_SET) {
        qdmHandle->baseAddress->QDMSTS.reg = BASE_CFG_SET; /* clear qctmr overflow status */
        return 0;
    }
    utime = BASE_FUNC_GetCpuFreqHz() / qdmHandle->motorLineNum;
    tmp = utime << qdmHandle->baseAddress->QTSUCTRL.BIT.cevt_prescaler \
            >> qdmHandle->baseAddress->QTSUCTRL.BIT.tsu_prescaler >> qdmHandle->baseAddress->QCTRL.BIT.qdu_xclk;
    rpm = tmp * SECONDS_PER_MINUTES / qdmHandle->baseAddress->QCPRD;

    if (qdmHandle->baseAddress->QDMSTS.BIT.qdir_sts == BASE_CFG_SET) {
        qdmHandle->speedRpm = rpm;
    } else {
        qdmHandle->speedRpm = -rpm;
    }

    return qdmHandle->speedRpm;
}
