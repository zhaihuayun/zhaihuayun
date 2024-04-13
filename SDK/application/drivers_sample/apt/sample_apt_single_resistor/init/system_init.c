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
  * @file      system_init.c
  * @author    MCU Driver Team
  * @brief     This file contains driver init functions.
  */

#include "main.h"
#include "ioconfig.h"

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect)
{
    CRG_Handle crg;
    crg.baseAddress     = CRG;
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllFbDiv        = 32; /* PLL Multiplier 32 */
    crg.pllPostDiv      = CRG_PLL_POSTDIV_1;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_PLL;
    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

static APT_U_ProtectInit(APT_Handle *aptHandle)
{
    APT_OutCtrlProtectEx protectApt;
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = true;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_POE0;
    protectApt.filterCycleNumEx = 0;
    HAL_APT_ProtectInitEx(&g_hAptU, &protectApt);
}

static void APT0_Init(void)
{
    HAL_CRG_IpEnableSet(APT0_BASE, IP_CLK_ENABLE);

    g_hAptU.baseAddress = APT0;
    g_hAptU.irqNumEvt = IRQ_APT0_EVT;
    g_hAptU.irqNumTmr = IRQ_APT0_TMR;

    /* Clock Settings */
    g_hAptU.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_hAptU.waveform.timerPeriod = 20000; /* 20000: counter max value */
    g_hAptU.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;
    /* Wave Form */
    g_hAptU.waveform.basicType = APT_PWM_BASIC_A_LOW_B_HIGH;
    g_hAptU.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_hAptU.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_hAptU.waveform.divInitVal = 0U; /* divide init value */
    g_hAptU.waveform.cntInitVal = 0U; /* counter init value */
    g_hAptU.waveform.cntCmpLeftEdge = 1;      /* 1: left compare point */
    g_hAptU.waveform.cntCmpRightEdge = 19999; /* 19999: right compare point */
    g_hAptU.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_hAptU.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_hAptU.waveform.deadBandCnt = 10; /* 10: dead baud count */
    /* ADC Triggle SOCA */
    g_hAptU.adcTrg.trgEnSOCA = BASE_CFG_ENABLE;
    g_hAptU.adcTrg.cntCmpSOCA = 1;
    g_hAptU.adcTrg.trgSrcSOCA = APT_CS_SRC_CNTR_CMPA_UP;
    g_hAptU.adcTrg.trgScaleSOCA = 1;
    /* ADC Triggle SOCB */
    g_hAptU.adcTrg.trgEnSOCB = BASE_CFG_ENABLE;
    g_hAptU.adcTrg.cntCmpSOCB =  1;
    g_hAptU.adcTrg.trgSrcSOCB = APT_CS_SRC_CNTR_CMPB_UP;
    g_hAptU.adcTrg.trgScaleSOCB = 1;
    g_hAptU.adcTrg.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_hAptU.adcTrg.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    /* Timer Triggle */
    g_hAptU.tmrInterrupt.tmrInterruptEn = BASE_CFG_ENABLE;
    g_hAptU.tmrInterrupt.tmrInterruptSrc = APT_INT_SRC_CNTR_ZERO_PERIOD;
    g_hAptU.tmrInterrupt.tmrInterruptScale = 1;

    HAL_APT_PWMInit(&g_hAptU);
    APT_U_ProtectInit(&g_hAptU);
}

static APT_V_ProtectInit(APT_Handle *aptHandle)
{
    APT_OutCtrlProtectEx protectApt;
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = false;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_POE0;
    protectApt.filterCycleNumEx = 0;
    HAL_APT_ProtectInitEx(&g_hAptV, &protectApt);
}

static void APT1_Init(void)
{
    HAL_CRG_IpEnableSet(APT1_BASE, IP_CLK_ENABLE);

    g_hAptV.baseAddress = APT1;
    g_hAptV.irqNumEvt = IRQ_APT1_EVT;
    g_hAptV.irqNumTmr = IRQ_APT1_TMR;

    /* Clock Settings */
    g_hAptV.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_hAptV.waveform.timerPeriod = 20000; /* 20000: counter max value */
    g_hAptV.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;
    /* Wave Form */
    g_hAptV.waveform.basicType = APT_PWM_BASIC_A_LOW_B_HIGH;
    g_hAptV.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_hAptV.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_hAptV.waveform.divInitVal = 0U; /* divide init value */
    g_hAptV.waveform.cntInitVal = 0U; /* counter init value */
    g_hAptV.waveform.cntCmpLeftEdge = 1; /* 1: left compare point */
    g_hAptV.waveform.cntCmpRightEdge = 19999; /* 19999: right compare point */
    g_hAptV.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_hAptV.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_hAptV.waveform.deadBandCnt = 10; /* 10: dead baud count */
    HAL_APT_PWMInit(&g_hAptV);
    APT_V_ProtectInit(&g_hAptV);
}

static APT_W_ProtectInit(APT_Handle *aptHandle)
{
    APT_OutCtrlProtectEx protectApt;
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = false;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_POE0;
    protectApt.filterCycleNumEx = 0;
    HAL_APT_ProtectInitEx(&g_hAptW, &protectApt);
}

static void APT2_Init(void)
{
    HAL_CRG_IpEnableSet(APT2_BASE, IP_CLK_ENABLE);

    g_hAptW.baseAddress = APT2;
    g_hAptW.irqNumEvt = IRQ_APT2_EVT;
    g_hAptW.irqNumTmr = IRQ_APT2_TMR;

    /* Clock Settings */
    g_hAptW.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_hAptW.waveform.timerPeriod = 20000; /* 20000: counter max value */
    g_hAptW.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;
    /* Wave Form */
    g_hAptW.waveform.basicType = APT_PWM_BASIC_A_LOW_B_HIGH;
    g_hAptW.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_hAptW.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_hAptW.waveform.divInitVal = 0U; /* divide init value */
    g_hAptW.waveform.cntInitVal = 0U; /* counter init value */
    g_hAptW.waveform.cntCmpLeftEdge = 1; /* 1: left compare point */
    g_hAptW.waveform.cntCmpRightEdge = 19999; /* 19999: right compare point */
    g_hAptW.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_hAptW.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_hAptW.waveform.deadBandCnt = 10; /* 10: dead baud count */

    HAL_APT_PWMInit(&g_hAptW);
    APT_W_ProtectInit(&g_hAptW);
}

static void IOConfig(void)
{
    IOConfig_RegStruct *iconfig = IOCONFIG;

    iconfig->iocmg_32.BIT.func = 0x3; /* 0x3 is APT0_PWMA */
    iconfig->iocmg_32.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_32.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_32.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_32.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_32.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_35.BIT.func = 0x3; /* 0x3 is APT0_PWMB */
    iconfig->iocmg_35.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_35.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_35.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_35.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_35.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_33.BIT.func = 0x3; /* 0x3 is APT1_PWMA */
    iconfig->iocmg_33.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_33.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_33.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_33.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_33.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_36.BIT.func = 0x3; /* 0x3 is APT1_PWMB */
    iconfig->iocmg_36.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_36.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_36.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_36.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_36.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_34.BIT.func = 0x3; /* 0x3 is APT2_PWMA */
    iconfig->iocmg_34.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_34.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_34.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_34.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_34.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_37.BIT.func = 0x3; /* 0x3 is APT2_PWMB */
    iconfig->iocmg_37.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_37.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_37.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_37.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_37.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_43.BIT.func = 0x4; /* 0x4 is POE0 */
    iconfig->iocmg_43.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_43.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_43.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_43.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_43.BIT.se = BASE_CFG_DISABLE;
}

void SystemInit(void)
{
    IOConfig();
    APT0_Init();
    APT1_Init();
    APT2_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}