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
  * @file    adc.c
  * @author  MCU Driver Team
  * @brief   ADC module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the ADC.
  *           + ADC initialization function.
  *           + Start ADC sample and conversion.
  *           + Start ADC sample and conversion with interrupt.
  *           + Start ADC sample and conversion with DMA.
  *           + Start ADC sample and conversion synchronously.
  *           + Query the ADC conversion result.
  *           + Single and multichannel software trigger functions.
  *           + Interrupt callback function and user registration function.
  */

/* Includes ------------------------------------------------------------------*/

#include "adcinit.h"
#include "adc.h"

static BASE_StatusType ADC_InitSOC(ADC_Handle *adcHandle, ADC_SOCNumber soc);
static void ADC_CapCalibrationMode(ADC_Handle *adcHandle);

/**
  * @brief Configuring ADC vref parameters.
  * @param adcHandle ADC handle.
  * @param vrefBuf type of adc vrefBuf.
  * @retval None.
  */
static void ADC_SetVrefBuf(ADC_Handle *adcHandle, unsigned int vrefBuf)
{
    unsigned int val = BASE_CFG_ENABLE;
    val |= (vrefBuf << 4);  /* Shift left 4 bit to configurate the type of vrefbuf */
    if (adcHandle->baseAddress == ADC0) {
        SYSCTRL1->ADC0_VREF_CTRL.reg |= val;
    } else if (adcHandle->baseAddress == ADC1) {
        SYSCTRL1->ADC1_VREF_CTRL.reg |= val;
    } else if (adcHandle->baseAddress == ADC2) {
        SYSCTRL1->ADC2_VREF_CTRL.reg |= val;
    }
}

/**
 * @brief ADC Capacitor Calibration Mode1.
 * @param None.
 * @retval None.
 */
static void ADC_CapCalibrationMode(ADC_Handle *adcHandle)
{
    adcHandle->baseAddress->ADC_CTRL.BIT.adc_cal_en = 0x01;
    adcHandle->baseAddress->ADC_CTRL.BIT.adc_cal_mode = 0x01;   /* Enter Capacitance Calibration Mode 1 */
    adcHandle->baseAddress->ADC_ANA_CTRL.BIT.ana_logic_mode = 0x01;
    adcHandle->baseAddress->ADC_CAP_CALI0.BIT.adc_weight_ini_sel = 0x01;
    adcHandle->baseAddress->ADC_SAR_CTRL0.BIT.cap_start_index = 0xF;
    adcHandle->baseAddress->ADC_SAR_CTRL3.BIT.cap_start = 0x01;
    BASE_FUNC_DELAY_US(270);     /* wait 270us */
    adcHandle->baseAddress->ADC_CTRL.BIT.adc_cal_en = 0x0;
    adcHandle->baseAddress->ADC_CTRL.BIT.adc_cal_mode = 0x0;    /* Enter the working mode */
    adcHandle->baseAddress->ADC_ANA_CTRL.BIT.ana_logic_mode = 0x0;
}

/**
  * @brief Initialize the ADC hardware controller.
  * @param adcHandle ADC handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_ADC_Init(ADC_Handle *adcHandle)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    if (adcHandle->initEnable == BASE_CFG_ENABLE) {
        return BASE_STATUS_ERROR;
    }
    ADC_PARAM_CHECK_WITH_RET(IsADCPriorityMode(adcHandle->socPriority) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCVrefBufType(adcHandle->vrefBuf) == true, BASE_STATUS_ERROR);
    adcHandle->baseAddress->ADC_ANA_PD.BIT.adc_pwdnz = BASE_CFG_ENABLE;
    BASE_FUNC_DELAY_US(10);     /* wait 10us */
    ADC_SetVrefBuf(adcHandle, (unsigned int)adcHandle->vrefBuf);
    unsigned int val = 0x00;
    if (adcHandle->vrefBuf == ADC_VREF_2P0V) {
        val = 0x02;     /* 0x02: gian = 0.6 */
    } else if (adcHandle->vrefBuf == ADC_VREF_2P5V) {
        val = 0x01;    /* 0x01: gian = 0.75 */
    }
    adcHandle->baseAddress->ADC_ANA_CTRL.BIT.adc_ana_gsh0 = val;
    adcHandle->baseAddress->ADC_ANA_CTRL.BIT.adc_ana_gsh1 = val;
    /* ADC Calibration */
    if (g_versionId != 0xFF && g_versionId != 0) {
        ADC_CapCalibrationMode(adcHandle);
    }
    DCL_ADC_SOCxSetPriority(adcHandle->baseAddress, adcHandle->socPriority);
    adcHandle->initEnable = BASE_CFG_ENABLE;
    return BASE_STATUS_OK;
}

/**
  * @brief DeInitialize the ADC hardware controller.
  * @param adcHandle ADC handle.
  * @retval BASE status type: OK.
  */
BASE_StatusType HAL_ADC_Deinit(ADC_Handle *adcHandle)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    adcHandle->baseAddress->ADC_ANA_PD.BIT.adc_pwdnz = BASE_CFG_DISABLE;
    adcHandle->initEnable = BASE_CFG_DISABLE;
    return BASE_STATUS_OK;
}

/**
  * @brief configurating the specified SOC parameters.
  * @param adcHandle ADC handle.
  * @param soc ID of SOC(start of conversion), managing the specific sample inputs.
  * @param socParam Param struct of SOC.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_ADC_ConfigureSoc(ADC_Handle *adcHandle, ADC_SOCNumber soc, SOC_Param *socParam)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(soc) == true, BASE_STATUS_ERROR);
    ADC_ASSERT_PARAM(socParam != NULL);
    ADC_PARAM_CHECK_WITH_RET(IsADCSampleChannel(socParam->adcInput) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCChargeTime(socParam->sampleTotalTime) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCHodeTime(socParam->sampleHoldTime) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCSoftTrig(socParam->softTrigSource) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCPeriphTrig(socParam->periphTrigSource) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCIntTrig(socParam->intTrigSource) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCFinishMode(socParam->finishMode) == true, BASE_STATUS_ERROR);
    adcHandle->ADC_SOCxParam[soc].adcInput = socParam->adcInput;
    adcHandle->ADC_SOCxParam[soc].sampleTotalTime = socParam->sampleTotalTime;
    adcHandle->ADC_SOCxParam[soc].sampleHoldTime = socParam->sampleHoldTime;
    adcHandle->ADC_SOCxParam[soc].softTrigSource = socParam->softTrigSource;
    adcHandle->ADC_SOCxParam[soc].periphTrigSource = socParam->periphTrigSource;
    adcHandle->ADC_SOCxParam[soc].intTrigSource = socParam->intTrigSource;
    adcHandle->ADC_SOCxParam[soc].finishMode = socParam->finishMode;
    ADC_InitSOC(adcHandle, soc);
    return BASE_STATUS_OK;
}

/**
  * @brief Initialize the ADC specified SOC.
  * @param adcHandle ADC handle.
  * @param soc ID of SOC(start of conversion), managing the specific sample inputs.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
static BASE_StatusType ADC_InitSOC(ADC_Handle *adcHandle, ADC_SOCNumber soc)
{
    unsigned int input = adcHandle->ADC_SOCxParam[soc].adcInput;
    unsigned int acqps = adcHandle->ADC_SOCxParam[soc].sampleTotalTime;
    unsigned int shHold = adcHandle->ADC_SOCxParam[soc].sampleHoldTime;
    unsigned int periphTrig = adcHandle->ADC_SOCxParam[soc].periphTrigSource;
    unsigned int intTrig = adcHandle->ADC_SOCxParam[soc].intTrigSource;
    DCL_ADC_SOCxSelcetChannel(adcHandle->baseAddress, soc, input);
    DCL_ADC_SOCxSetAcqps(adcHandle->baseAddress, soc, acqps);
    DCL_ADC_SOCxSetShHold(adcHandle->baseAddress, soc, shHold);
    DCL_ADC_SOCxSelcetIntxTrig(adcHandle->baseAddress, soc, intTrig);
    DCL_ADC_SOCxSelcetTrigSource(adcHandle->baseAddress, soc, periphTrig);
    return BASE_STATUS_OK;
}

/**
  * @brief Callback function that ADC completes the sample conversion and uses the DMA to complete the transmission.
  * @param handle ADC handle.
  * @retval None.
  */
static void ADC_DMATransFinish(void *handle)
{
    ADC_ASSERT_PARAM(handle != NULL);
    ADC_Handle *adcHandle = (ADC_Handle *)(handle);
    if (adcHandle->DmaFinishCallBack != NULL) {
        adcHandle->DmaFinishCallBack(adcHandle);
    }
    return;
}

/**
  * @brief Callback function that ADC falis to use DMA.
  * @param handle ADC handle.
  * @retval None.
  */
static void ADC_DMATransError(void *handle)
{
    ADC_ASSERT_PARAM(handle != NULL);
    ADC_Handle *adcHandle = (ADC_Handle *)(handle);
    if (adcHandle->DmaErrorCallBack != NULL) {
        adcHandle->DmaErrorCallBack(adcHandle);
    }
    return;
}

/**
  * @brief Start the ADC conversion and enable ADC DMA. After the SOC conversion using the DMA is complete, use the DMA
  * to transfer data The DMA can transfer the sampling results of consecutive SOCs. The start and end of DMA transfer
  * are determined by startSoc and endSoc.
  * @param adcHandle ADC handle.
  * @param startSoc First SOC result for DMA transfer.
  * @param endSoc Last SOC result for DMA transfer.
  * @param saveData Address where the converted result is saved.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_ADC_StartDma(ADC_Handle *adcHandle, unsigned int startSoc,
                                 unsigned int endSoc, unsigned int *saveData)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(startSoc) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(endSoc) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(startSoc <= endSoc, BASE_STATUS_ERROR);
    ADC_ASSERT_PARAM(saveData != NULL);
    ADC_ASSERT_PARAM(adcHandle->dmaHandle != NULL);
    ADC_PARAM_CHECK_WITH_RET(IsDmaChannelNum(adcHandle->adcDmaChn) == true, BASE_STATUS_ERROR);
    unsigned int dmaSOCx = 0;
    unsigned int dataLength = endSoc - startSoc + 1;
    for (int i = 0; i < SOC_MAX_NUM; i++) {
        if (adcHandle->ADC_SOCxParam[i].finishMode == ADC_SOCFINISH_DMA) {
            dmaSOCx = i;
        }
    }

    DCL_ADC_DMARequestSource(adcHandle->baseAddress, dmaSOCx);
    DCL_ADC_EnableDMABurstReq(adcHandle->baseAddress);
    DCL_ADC_EnableDMASingleReq(adcHandle->baseAddress);
    uintptr_t srcAddr = (uintptr_t)(void *)(adcHandle->baseAddress);
    srcAddr = srcAddr + 4 * startSoc;   /* The base address difference of adjacent SOC result registers is 4 */
    adcHandle->dmaHandle->DMA_CallbackFuns[adcHandle->adcDmaChn].ChannelFinishCallBack = ADC_DMATransFinish;
    adcHandle->dmaHandle->DMA_CallbackFuns[adcHandle->adcDmaChn].ChannelErrorCallBack = ADC_DMATransError;
    HAL_DMA_StartIT(adcHandle->dmaHandle, srcAddr, (uintptr_t)(void *)(saveData),
                    dataLength, adcHandle->adcDmaChn);
    return BASE_STATUS_OK;
}

/**
  * @brief Start the ADC conversion and enable ADC interrupt. After the SOC completes sample conversion, the ADC
  * interrupt is reported.
  * @param adcHandle ADC handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_ADC_StartIt(ADC_Handle *adcHandle)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    unsigned int intVal = 0;
    for (int i = 0; i < SOC_MAX_NUM; i++) {
        intVal = adcHandle->ADC_SOCxParam[i].finishMode;
        switch (intVal) {
            case ADC_SOCFINISH_INT1:
                DCL_ADC_SetSOCxBlindIntx(adcHandle->baseAddress, ADC_INT_NUMBER1, i);
                break;
            case ADC_SOCFINISH_INT2:
                DCL_ADC_SetSOCxBlindIntx(adcHandle->baseAddress, ADC_INT_NUMBER2, i);
                break;
            case ADC_SOCFINISH_INT3:
                DCL_ADC_SetSOCxBlindIntx(adcHandle->baseAddress, ADC_INT_NUMBER3, i);
                break;
            case ADC_SOCFINISH_INT4:
                DCL_ADC_SetSOCxBlindIntx(adcHandle->baseAddress, ADC_INT_NUMBER4, i);
                break;
            default:
                break;
        }
    }
    DCL_ADC_EnableIntx(adcHandle->baseAddress, ADC_INT_NUMBER1);
    DCL_ADC_EnableIntx(adcHandle->baseAddress, ADC_INT_NUMBER2);
    DCL_ADC_EnableIntx(adcHandle->baseAddress, ADC_INT_NUMBER3);
    DCL_ADC_EnableIntx(adcHandle->baseAddress, ADC_INT_NUMBER4);
    return BASE_STATUS_OK;
}

/**
  * @brief Enable ADC synchronous sample.Use one ADC with two different SOCs, configure the same trigger source, and
  * sample both inputs simultaneously. When used, the inputs in group A need to be configured with one SOC, and
  * the inputs in group B need to be configured with another SOC.
  * @param adcHandle ADC handle.
  * @param syncParam Param struct of synchronous sample.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_ADC_StartSyncSample(ADC_Handle *adcHandle, SOC_SyncParam *syncParam)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_ASSERT_PARAM(syncParam != NULL);
    ADC_PARAM_CHECK_WITH_RET(IsADCGroupAChannel(syncParam->ChannelA) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCGroupBChannel(syncParam->ChannelB) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCChargeTime(syncParam->sampleTotalTime) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCHodeTime(syncParam->sampleHoldTime) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCSoftTrig(syncParam->softTrigSource) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCPeriphTrig(syncParam->periphTrigSource) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCIntTrig(syncParam->intTrigSource) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCFinishMode(syncParam->finishMode) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(IsADCSyncGroup(syncParam->group) == true, BASE_STATUS_ERROR);
    unsigned int socA = 0;
    unsigned int socB = 0;
    unsigned int tmp;
    SOC_Param param;
    param.adcInput = syncParam->ChannelA;
    param.sampleHoldTime = syncParam->sampleHoldTime;
    param.sampleTotalTime = syncParam->sampleTotalTime;
    param.finishMode = ADC_SOCFINISH_NONE;
    param.softTrigSource = syncParam->softTrigSource;
    param.intTrigSource = syncParam->intTrigSource;
    param.periphTrigSource = syncParam->periphTrigSource;
    for (unsigned int i = 0; i < SYNCGROUP_NUM; i++) {
        tmp = (1 << i);
        if (tmp & syncParam->group) {
            socA = 2 * i;           /* 2 for converting from group number to SOC number */
            socB = 2 * i + 1;       /* 2 for converting from group number to SOC number */
            break;
        }
    }
    DCL_ADC_SetSyncSample(adcHandle->baseAddress, syncParam->group);
    /* Group A does not report DMA and interrupts by default. Group B reports DMA and interrupts after sampling */
    HAL_ADC_ConfigureSoc(adcHandle, socA, &param);
    param.finishMode = syncParam->finishMode;
    param.adcInput = syncParam->ChannelB;
    HAL_ADC_ConfigureSoc(adcHandle, socB, &param);
    if (syncParam->finishMode >= ADC_SOCFINISH_INT1) {
        HAL_ADC_StartIt(adcHandle);
    }
    return BASE_STATUS_OK;
}

/**
  * @brief The software triggers multiple SCOs for sampling at the same time.
  * @param adcHandle ADC handle.
  * @param syncTrig Triggering Parameters. The lower 16 bits correspond to one SOC. If this bit is set to 1, the
  * software triggers the SOC.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_ADC_SoftTrigMultiSample(ADC_Handle *adcHandle, ADC_SoftMultiTrig syncTrig)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    unsigned int val = syncTrig.softTrigVal;
    ADC_PARAM_CHECK_WITH_RET(val <= 0xFFFF, BASE_STATUS_ERROR);
    unsigned int tmp = 0;
    for (int i = 0; i < SOC_MAX_NUM; i++) {
        tmp = (1 << i);
        if ((tmp & val) == tmp && adcHandle->ADC_SOCxParam[i].softTrigSource != ADC_TRIGSOC_SOFT) {
            return BASE_STATUS_ERROR;
        }
    }
    DCL_ADC_SOCxMultiSoftTrigger(adcHandle->baseAddress, val);
    return BASE_STATUS_OK;
}

/**
  * @brief The software triggers only one soc.
  * @param adcHandle ADC handle.
  * @param soc ID of SOC.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_ADC_SoftTrigSample(ADC_Handle *adcHandle, unsigned int soc)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(soc) == true, BASE_STATUS_ERROR);
    if (adcHandle->ADC_SOCxParam[soc].softTrigSource !=  ADC_TRIGSOC_SOFT) {
        return BASE_STATUS_ERROR;
    }
    DCL_ADC_SOCxSoftTrigger(adcHandle->baseAddress, soc);
    return BASE_STATUS_OK;
}

/**
  * @brief Obtains the sample result after SOC conversion.
  * @param adcHandle ADC handle.
  * @param soc ID of SOC.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
unsigned int HAL_ADC_GetConvResult(ADC_Handle *adcHandle, unsigned int soc)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(soc) == true, BASE_STATUS_ERROR);
    return DCL_ADC_ReadSOCxResult(adcHandle->baseAddress, soc);
}


/**
  * @brief Check the SOC completion flag.
  * @param adcHandle ADC handle.
  * @param soc ID of SOC.
  * @retval BASE_STATUS_ERROR: The SOC does not complete the data sampling conversion.
  * @retval BASE_STATUS_OK: The SOC has completed data sampling conversion.
  */
BASE_StatusType HAL_ADC_CheckSocFinish(ADC_Handle *adcHandle, unsigned int soc)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(soc) == true, BASE_STATUS_ERROR);
    if (DCL_ADC_GetConvState(adcHandle->baseAddress, soc) == 0) {
        return BASE_STATUS_ERROR;       /* The SOC does not complete the conversion */
    }
    DCL_ADC_ResetConvState(adcHandle->baseAddress, soc);  /* Clear flag bit */
    return BASE_STATUS_OK;
}

/**
  * @brief After the interrupt is complete, call back the user registration interrupt function.
  * @param adcHandle ADC handle.
  * @param intx ADC interrupt type number @ref ADC_IntNumber.
  * @retval None.
  */
static void ADC_IRQHandlerCallback(ADC_Handle *adcHandle, unsigned int intx)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    switch (intx) {
        case ADC_INT_NUMBER1:
            if (adcHandle->Int1FinishCallBack != NULL) {
                adcHandle->Int1FinishCallBack(adcHandle);
            }
            break;
        case ADC_INT_NUMBER2:
            if (adcHandle->Int2FinishCallBack != NULL) {
                adcHandle->Int2FinishCallBack(adcHandle);
            }
            break;
        case ADC_INT_NUMBER3:
            if (adcHandle->Int3FinishCallBack != NULL) {
                adcHandle->Int3FinishCallBack(adcHandle);
            }
            break;
        case ADC_INT_NUMBER4:
            if (adcHandle->Int4FinishCallBack != NULL) {
                adcHandle->Int4FinishCallBack(adcHandle);
            }
            break;
        default:
            break;
    }
}

/**
  * @brief The ADC completes the interrupt processing.
  * @param adcHandle ADC handle.
  * @param intx ADC interrupt type number @ref ADC_IntNumber.
  * @retval None.
  */
static void ADC_IRQHandlerIntx(ADC_Handle *adcHandle, unsigned int intx)
{
    unsigned int eocFlag = adcHandle->baseAddress->ADC_EOC_FLAG.reg;
    ADC_INT1_CTRL_REG int1Ctrl;
    ADC_INT2_CTRL_REG int2Ctrl;
    ADC_INT3_CTRL_REG int3Ctrl;
    ADC_INT4_CTRL_REG int4Ctrl;
    unsigned int eocMask = 0;
    switch (intx) {
        case ADC_INT_NUMBER1:
            int1Ctrl.reg = adcHandle->baseAddress->ADC_INT1_CTRL.reg;
            eocMask = int1Ctrl.BIT.int1_eoc_en;
            break;
        case ADC_INT_NUMBER2:
            int2Ctrl.reg = adcHandle->baseAddress->ADC_INT2_CTRL.reg;
            eocMask = int2Ctrl.BIT.int2_eoc_en;
            break;
        case ADC_INT_NUMBER3:
            int3Ctrl.reg = adcHandle->baseAddress->ADC_INT3_CTRL.reg;
            eocMask = int3Ctrl.BIT.int3_eoc_en;
            break;
        case ADC_INT_NUMBER4:
            int4Ctrl.reg = adcHandle->baseAddress->ADC_INT4_CTRL.reg;
            eocMask = int4Ctrl.BIT.int4_eoc_en;
            break;
        default:
            break;
    }
    unsigned int eoc = eocFlag & eocMask;
    adcHandle->ADC_IntxParam[intx].socxFinish.stateVal = eoc;
    for (int i = 0; i < SOC_MAX_NUM; i++) {
        unsigned int val = (1 << i);
        if (eoc & val) {
            adcHandle->baseAddress->ADC_EOC_FLAG.reg = val;  /* Clear the EOC flag */
        }
    }
    DCL_ADC_ClearIntx(adcHandle->baseAddress, intx);
    IRQ_ClearN(adcHandle->ADC_IntxParam[intx].irqNum);
    ADC_IRQHandlerCallback(adcHandle, intx);
}

/**
  * @brief The ADC overflow interrupt processing
  * @param adcHandle ADC handle.
  * @param overFlag Overflow interrupt flag bit.
  * @retval None.
  */
static void ADC_IRQHandlerOver(ADC_Handle *adcHandle, unsigned int overFlag)
{
    unsigned int eocOver = adcHandle->baseAddress->ADC_EOC_OVFL.reg;
    adcHandle->baseAddress->ADC_INT_OVFL.reg = overFlag;
    adcHandle->baseAddress->ADC_EOC_OVFL.reg = eocOver;
    IRQ_ClearN(adcHandle->irqNumOver);
    unsigned int tmp, lsh;
    unsigned int overState = 0;
    for (unsigned int i = 0; i < SOC_MAX_NUM; i++) {
        tmp = (1 << i);
        if (tmp & eocOver) {
            lsh = 2 * i + 1;    /* 2 indicates overflow */
            tmp = (1 << lsh);   /* soc over state */
            overState |= tmp;
        }
    }
    adcHandle->socxOverState.stateVal = overState;
    if ((overFlag & INT_OVER_MASK) != 0 && adcHandle->IntxOverCallBack != NULL) {
        adcHandle->IntxOverCallBack(adcHandle);
    }
    if ((overFlag & DMA_OVER_MASK) != 0  && adcHandle->DmaOverCallBack != NULL) {
        adcHandle->DmaOverCallBack(adcHandle);
    }
}

/**
  * @brief ADC Interrupt service processing function. Including completion interrupts and overflow interrupt.
  * @param handle ADC handle.
  * @retval None.
  */
void HAL_ADC_IrqHandler(void *handle)
{
    ADC_ASSERT_PARAM(handle != NULL);
    ADC_Handle *adcHandle = (ADC_Handle *)handle;
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    unsigned int intFlag = adcHandle->baseAddress->ADC_INT_FLAG.reg;
    for (unsigned int i = 0; i < INT_MAX_NUM; i++) {
        if (intFlag & (1 << i)) {
            ADC_IRQHandlerIntx(adcHandle, i);
        }
    }
    unsigned int overFlag = adcHandle->baseAddress->ADC_INT_OVFL.reg;
    if (overFlag != 0) {
        ADC_IRQHandlerOver(adcHandle, overFlag);
    }
    return;
}

/**
  * @brief Registering ADC interrupt service processing function.
  * @param adcHandle ADC handle.
  * @retval None.
  */
void HAL_ADC_IrqService(ADC_Handle *adcHandle)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    ADC_ASSERT_PARAM(IsADCInstance(adcHandle->baseAddress));
    ADC_ASSERT_PARAM(adcHandle->ADC_IntxParam != NULL);
    IRQ_Register(adcHandle->irqNumOver, HAL_ADC_IrqHandler, adcHandle);
    IRQ_Register(adcHandle->ADC_IntxParam[0].irqNum, HAL_ADC_IrqHandler, adcHandle);    /* interrupt 0 */
    IRQ_Register(adcHandle->ADC_IntxParam[1].irqNum, HAL_ADC_IrqHandler, adcHandle);    /* interrupt 1 */
    IRQ_Register(adcHandle->ADC_IntxParam[2].irqNum, HAL_ADC_IrqHandler, adcHandle);    /* interrupt 2 */
    IRQ_Register(adcHandle->ADC_IntxParam[3].irqNum, HAL_ADC_IrqHandler, adcHandle);    /* interrupt 3 */
}

/**
  * @brief User callback function registration interface.
  * @param adcHandle ADC handle.
  * @param typeID Id of callback function type.
  * @param pCallback Pointer of the specified callbcak function.
  * @retval None.
  */
void HAL_ADC_RegisterCallBack(ADC_Handle *adcHandle, ADC_CallbackFunType typeID, ADC_CallbackType pCallback)
{
    ADC_ASSERT_PARAM(adcHandle != NULL);
    switch (typeID) {
        case ADC_CALLBACK_INT1:
            adcHandle->Int1FinishCallBack = pCallback;
            break;
        case ADC_CALLBACK_INT2:
            adcHandle->Int2FinishCallBack = pCallback;
            break;
        case ADC_CALLBACK_INT3:
            adcHandle->Int3FinishCallBack = pCallback;
            break;
        case ADC_CALLBACK_INT4:
            adcHandle->Int4FinishCallBack = pCallback;
            break;
        case ADC_CALLBACK_DMA:
            adcHandle->DmaFinishCallBack = pCallback;
            break;
        case ADC_CALLBACK_INTOVER:
            adcHandle->IntxOverCallBack = pCallback;
            break;
        case ADC_CALLBACK_DMAOVER:
            adcHandle->DmaOverCallBack = pCallback;
            break;
        case ADC_CALLBACK_DMAERROR:
            adcHandle->DmaErrorCallBack = pCallback;
            break;
        default:
            return;
    }
}

/**
  * @brief Obtains the sample result after SOC conversion.
  * @param adcHandle ADC handle.
  * @param soc ID of SOC.
  * @param adcHandle ADC handle.
  * @param originalRet Sampling original data.
  * @retval unsigned int, Calibrated data.
  */
unsigned int HAL_ADC_ActiveCalibrateRet(ADC_RegStruct * const adcx, unsigned int soc, unsigned int originalRet)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(soc) == true, BASE_STATUS_ERROR);
    ADC_PARAM_CHECK_WITH_RET(originalRet <= 0x1000, BASE_STATUS_ERROR);
    uintptr_t adcAddr = (uintptr_t)(void *)adcx;
    unsigned int addrIndex = (adcAddr & 0x3000) >> 12;  /* 0x3000 and 12 are used to convert addresses to index */
    unsigned int socNum = DCL_ADC_GetSOCxInputChannel(adcx, soc);
    unsigned int vrefIndex = adcx->ADC_ANA_CTRL.BIT.adc_ana_gsh0;
    unsigned int shIndex = ((socNum & 0x18) == 0) ? 0 : 1;  /* 0x18 is used to convert sh0 or sh1 to index */
    float k1 = g_adcParmList[addrIndex][vrefIndex][shIndex].k1;
    float k2 = g_adcParmList[addrIndex][vrefIndex][shIndex].k2;
    int result = (float)originalRet * k1 + k2;
    if (result < 0) {
        return 0;        /* Limit the range of results after calibration */
    } else if ((unsigned int)result > 0xFFF) {
        return 0xFFF;    /* Limit the range of results after calibration */
    }
    return (unsigned int)result;
}

#if defined (CHIP_3065HRPIRZ) || defined (CHIP_3065HRPICZ) || defined (CHIP_3061HRPIKZ) || \
    defined (AU302PDF51) || defined (AU302NDF51) || defined (AU301LDF51)
/**
  * @brief Initialize the ADC and DAC for VDDA.
  * Note:
  * (1) Ensure that the ADC clock is turned on and the ADC has been initialized using before use.
  * (2) The mapping between the ADC and DAC must be configured as follows:
  * ADC0 -- DAC0, ADC1 -- DAC1, ADC2 -- DAC0, ADC2 -- DAC2.
  * (3) The soc parameter must be set to an SOC that is not occupied in the ADC.
  * (4) The user-configured DAC output value need >= 128.
  * @param dacx DAC register base address.
  * @param adcx ADC register base address.
  * @param soc ID of SOC(start of conversion), managing the specific sample inputs.
  * @param dacx DAC register base address.
  * @param useDac ture: dacx has been used, false: dacx has not been used.
  * @retval BASE_StatusType.
  */
BASE_StatusType HAL_ADC_InitForVdda(ADC_RegStruct *adcx, ADC_SOCNumber soc, DAC_RegStruct *dacx, bool useDac)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_ASSERT_PARAM(IsDACInstance(dacx));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(soc) == true, BASE_STATUS_ERROR);
    DAC_Handle dac = {0};
    dac.baseAddress = dacx;
    unsigned int valueOfDac = 223;  /* 223 is the recommended value for the DAC */
    if (useDac == false) {          /* Check whether the DAC is used */
        dac.dacEn = BASE_CFG_ENABLE;
        dac.dacValue = valueOfDac;
        HAL_DAC_Init(&dac);
    } else {
        valueOfDac = dac.baseAddress->DAC_VALUE.reg;
    }
    if (valueOfDac < 128) { /* The user-configured DAC output value need >= 128 */
        return BASE_STATUS_ERROR;
    }
    ADC_Handle adc = {0};
    unsigned int input = ADC_CH_ADCINB7;
    if (adcx == ADC2 && dacx == DAC0) {
        input = ADC_CH_ADCINB6;
    }
    adc.baseAddress = adcx;
    SOC_Param socParam = {0};
    socParam.adcInput = input;      /* DAC input */
    socParam.sampleHoldTime =  2;   /* adc sample holed time set as 2  */
    socParam.sampleTotalTime = 127; /* adc sample total time set as 127 */
    socParam.softTrigSource = ADC_TRIGSOC_SOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_NONEPERIPH;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&adc, soc, &socParam);
    return BASE_STATUS_OK;
}

/**
  * @brief The DAC is sampled by using the ADC and converted to the VDDA voltage of the DAC.
  * @param adcx ADC register base address.
  * @param soc ID of SOC(start of conversion), managing the specific sample inputs.
  * @param dacx DAC register base address.
  * @param useDac ture: dacx has been used, false: dacx has not been used.
  * @retval float, The reference voltage.
  */
float HAL_ADC_GetVddaByDac(ADC_RegStruct *adcx, ADC_SOCNumber soc, DAC_RegStruct *dacx, bool useDac)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_ASSERT_PARAM(IsDACInstance(dacx));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(soc) == true, BASE_STATUS_ERROR);
    unsigned int valueOfDac = 223;  /* 223 is the recommended value for the DAC */
    if (useDac == true) {           /* Check whether the DAC is used */
        valueOfDac = dacx->DAC_VALUE.reg;
    }
    if (valueOfDac < 128) { /* The user-configured DAC output value need >= 128 */
        return 0.0f;
    }
    unsigned ret = 0;
    unsigned int count = 0;
    ADC_Handle adc = {0};
    adc.baseAddress = adcx;
    adc.ADC_SOCxParam[soc].softTrigSource = ADC_TRIGSOC_SOFT;
    for (unsigned int i = 0; i < 10; ++i) {  /* Average of 10 times */
        HAL_ADC_SoftTrigSample(&adc, soc);
        BASE_FUNC_DELAY_MS(10);  /* delay 10 ms */
        if (HAL_ADC_CheckSocFinish(&adc, soc) == BASE_STATUS_ERROR) {
            continue;
        }
        count++;
        unsigned int tmp = HAL_ADC_GetConvResult(&adc, soc);
        ret += tmp;
    }
    if (count == 0) {
        return 0.0f;
    }
    float ori = (float)ret / (float)count;
    /* 256.0, 3.33333 and 4096.0 are used to convert the voltage */
    float voltage = 256.0f / (float)valueOfDac  * 3.33333f * ori / 4096.0f;
    return voltage;
}

/**
  * @brief set an external reference source to convert the original sampling results of the ADC.
  * @param adcx ADC register base address.
  * @param soc ID of SOC(start of conversion), managing the specific sample inputs.
  * @param vdda Voltage Drain Drain.
  * @retval unsigned int, Sampled results after using the reference voltage.
  */
unsigned int HAL_ADC_GetTransResultByVdda(ADC_RegStruct *adcx, ADC_SOCNumber soc, float vdda)
{
    ADC_ASSERT_PARAM(IsADCInstance(adcx));
    ADC_PARAM_CHECK_WITH_RET(IsADCSOCx(soc) == true, BASE_STATUS_ERROR);
    if (vdda < 2.6f || vdda > 3.63f) {                          /* 2.6v ~ 3.63v is reasonable value range of VDDA */
        return 0;
    }
    unsigned int oriAdcResult = DCL_ADC_ReadSOCxResult(adcx, soc);
    float tmp =  3.33333f / vdda * (float)oriAdcResult;       /* ADC full scale from 3.33333v to VDDA */
    unsigned int ret = (unsigned int)tmp;
    return ret;
}
#endif