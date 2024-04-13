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

#define UART0_BAND_RATE 115200
#define UART2_BAND_RATE 115200

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

static void ACMP1_Init(void)
{
    HAL_CRG_IpEnableSet(ACMP1_BASE, BASE_CFG_ENABLE);

    g_acmp1.baseAddress =  ACMP1_BASE;

    g_acmp1.enable = true;
    g_acmp1.syncEn = false;
    g_acmp1.inOutConfig.vinNNum = ACMP_VIN_MUX2;
    g_acmp1.inOutConfig.vinPNum = ACMP_VIN_MUX2;
    g_acmp1.inOutConfig.swVinPNum = ACMP_SW_VIN2;
    g_acmp1.inOutConfig.swVinNNum = ACMP_SW_VIN2;
    g_acmp1.inOutConfig.polarity = ACMP_OUT_NOT_INVERT;
    g_acmp1.filterCtrl.filterMode = ACMP_FILTER_NONE;
    g_acmp1.hysteresisVol = 0; /* 0: without hysteresis */
    HAL_ACMP_Init(&g_acmp1);
}

static void ADC0_Init(void)
{
    HAL_CRG_IpEnableSet(ADC0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(ADC0_BASE, CRG_ADC_CLK_SELECT_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC0_BASE, CRG_ADC_DIV_5);

    g_adc0.baseAddress = ADC0;
    g_adc0.socPriority = ADC_PRIMODE_ALL_ROUND;
    g_adc0.vrefBuf = ADC_VREF_2P5V;
    g_adc0.irqNumOver = IRQ_ADC0_OVINT;
    g_adc0.ADC_IntxParam[0].irqNum = IRQ_ADC0_INT1;     /* interrupt 0 */
    g_adc0.ADC_IntxParam[1].irqNum = IRQ_ADC0_INT2;     /* interrupt 1 */
    g_adc0.ADC_IntxParam[2].irqNum = IRQ_ADC0_INT3;     /* interrupt 2 */
    g_adc0.ADC_IntxParam[3].irqNum = IRQ_ADC0_INT4;     /* interrupt 3 */

    HAL_ADC_Init(&g_adc0);

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINA0; /* PGA0_OUT(ADC INA0) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT3_SOCA;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM0, &socParam);

    socParam.adcInput = ADC_CH_ADCINA0; /* PGA0_OUT(ADC INA0) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT3_SOCB;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM1, &socParam);

    socParam.adcInput = ADC_CH_ADCINA3; /* PIN28(ADC INA3) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT3_SOCB;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM2, &socParam);

    socParam.adcInput = ADC_CH_ADCINA4; /* PIN29(ADC INA4) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT3_SOCB;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM3, &socParam);

}

static void ADC1_Init(void)
{
    HAL_CRG_IpEnableSet(ADC1_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(ADC1_BASE, CRG_ADC_CLK_SELECT_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC1_BASE, CRG_ADC_DIV_5);

    g_adc1.baseAddress = ADC1;
    g_adc1.socPriority = ADC_PRIMODE_ALL_ROUND;
    g_adc1.vrefBuf = ADC_VREF_2P5V;
    g_adc1.irqNumOver = IRQ_ADC1_OVINT;
    g_adc1.ADC_IntxParam[0].irqNum = IRQ_ADC1_INT1;     /* interrupt 0 */
    g_adc1.ADC_IntxParam[1].irqNum = IRQ_ADC1_INT2;     /* interrupt 1 */
    g_adc1.ADC_IntxParam[2].irqNum = IRQ_ADC1_INT3;     /* interrupt 2 */
    g_adc1.ADC_IntxParam[3].irqNum = IRQ_ADC1_INT4;     /* interrupt 3 */

    HAL_ADC_Init(&g_adc1);

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINB2; /* PIN7(ADC INB2) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT0_SOCA;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc1, ADC_SOC_NUM0, &socParam);

    socParam.adcInput = ADC_CH_ADCINA2; /* PIN6(ADC INA2) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT0_SOCA;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc1, ADC_SOC_NUM1, &socParam);

    socParam.adcInput = ADC_CH_ADCINA4; /* PIN10(ADC INA4) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT0_SOCB;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc1, ADC_SOC_NUM2, &socParam);

    socParam.adcInput = ADC_CH_ADCINA5; /* PIN11(ADC INA5) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT0_SOCB;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc1, ADC_SOC_NUM3, &socParam);

    socParam.adcInput = ADC_CH_ADCINB4; /* PIN3(ADC INB4) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT0_SOCB;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc1, ADC_SOC_NUM4, &socParam);

    socParam.adcInput = ADC_CH_ADCINB5; /* PIN4(ADC INB5) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT0_SOCB;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc1, ADC_SOC_NUM5, &socParam);

}

__weak void ADC2Interrupt1Callback(ADC_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN ADC2_CALLBACK_INT1 */
    /* USER CODE END ADC2_CALLBACK_INT1 */
}

static void ADC2_Init(void)
{
    HAL_CRG_IpEnableSet(ADC2_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(ADC2_BASE, CRG_ADC_CLK_SELECT_PLL_DIV);
    HAL_CRG_IpClkDivSet(ADC2_BASE, CRG_ADC_DIV_5);

    g_adc2.baseAddress = ADC2;
    g_adc2.socPriority = ADC_PRIMODE_ALL_ROUND;
    g_adc2.vrefBuf = ADC_VREF_2P5V;
    g_adc2.irqNumOver = IRQ_ADC2_OVINT;
    g_adc2.ADC_IntxParam[0].irqNum = IRQ_ADC2_INT1;     /* interrupt 0 */
    g_adc2.ADC_IntxParam[1].irqNum = IRQ_ADC2_INT2;     /* interrupt 1 */
    g_adc2.ADC_IntxParam[2].irqNum = IRQ_ADC2_INT3;     /* interrupt 2 */
    g_adc2.ADC_IntxParam[3].irqNum = IRQ_ADC2_INT4;     /* interrupt 3 */

    HAL_ADC_Init(&g_adc2);

    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINA4; /* PIN48(ADC INA4) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT8_SOCA;
    socParam.finishMode = ADC_SOCFINISH_INT1;
    HAL_ADC_ConfigureSoc(&g_adc2, ADC_SOC_NUM0, &socParam);

    socParam.adcInput = ADC_CH_ADCINA3; /* PIN47(ADC INA3) */

    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.intTrigSource = ADC_TRIGSOC_NONEINT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT8_SOCB;
    socParam.finishMode = ADC_SOCFINISH_NONE;
    HAL_ADC_ConfigureSoc(&g_adc2, ADC_SOC_NUM1, &socParam);

    HAL_ADC_RegisterCallBack(&g_adc2, ADC_CALLBACK_INT1, ADC2Interrupt1Callback);

    IRQ_SetPriority(IRQ_ADC2_INT1, 6);
    IRQ_EnableN(IRQ_ADC2_INT1);
    HAL_ADC_IrqService(&g_adc2);
}

__weak void APT0EventCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
    /* USER CODE BEGIN APT0_EVENT_INTERRUPT */
    /* USER CODE END APT0_EVENT_INTERRUPT */
}

__weak void APT0TimerCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
    /* USER CODE BEGIN APT0_TIMER_INTERRUPT */
    /* USER CODE END APT0_TIMER_INTERRUPT */
}

static void APT0_ProtectInit(void)
{
    APT_OutCtrlProtectEx protectApt = {0};
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = BASE_CFG_ENABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_ACMP1;
    protectApt.evtPolarityMaskEx = APT_EM_ACMP1_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    HAL_APT_ProtectInitEx(&g_apt0, &protectApt);
}

static void APT0_Init(void)
{
    HAL_CRG_IpEnableSet(APT0_BASE, IP_CLK_ENABLE);

    g_apt0.baseAddress = APT0;
    g_apt0.irqNumEvt = IRQ_APT0_EVT;
    g_apt0.irqNumTmr = IRQ_APT0_TMR;

    /* Clock Settings */
    g_apt0.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt0.waveform.timerPeriod = 6250;
    g_apt0.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_apt0.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt0.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt0.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt0.waveform.divInitVal = 0;
    g_apt0.waveform.cntInitVal = 0;
    g_apt0.waveform.cntCmpLeftEdge = 3125;
    g_apt0.waveform.cntCmpRightEdge = 3125;
    g_apt0.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt0.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt0.waveform.deadBandCnt = 200;

    /* ADC Trigger SOCA */
    g_apt0.adcTrg.trgEnSOCA = BASE_CFG_ENABLE;
    g_apt0.adcTrg.cntCmpSOCA = 6249;
    g_apt0.adcTrg.trgSrcSOCA = APT_CS_SRC_CNTR_CMPA_UP;
    g_apt0.adcTrg.trgScaleSOCA = 1;

    /* ADC Trigger SOCB */
    g_apt0.adcTrg.trgEnSOCB = BASE_CFG_ENABLE;
    g_apt0.adcTrg.cntCmpSOCB =  1;
    g_apt0.adcTrg.trgSrcSOCB = APT_CS_SRC_CNTR_CMPB_DOWN;
    g_apt0.adcTrg.trgScaleSOCB = 1;

    g_apt0.adcTrg.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt0.adcTrg.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;

    /* Timer Trigger */
    g_apt0.tmrInterrupt.tmrInterruptEn = BASE_CFG_ENABLE;
    g_apt0.tmrInterrupt.tmrInterruptSrc = APT_INT_SRC_CNTR_ZERO;
    g_apt0.tmrInterrupt.tmrInterruptScale = 1;

    APT0_ProtectInit();

    HAL_APT_PWMInit(&g_apt0);
    HAL_APT_RegisterCallBack(&g_apt0, APT_EVENT_INTERRUPT, APT0EventCallback);
    IRQ_SetPriority(g_apt0.irqNumEvt, 7);
    HAL_APT_IRQService(&g_apt0);
    IRQ_EnableN(g_apt0.irqNumEvt);
    HAL_APT_RegisterCallBack(&g_apt0, APT_TIMER_INTERRUPT, APT0TimerCallback);
    IRQ_SetPriority(g_apt0.irqNumTmr, 5);
    HAL_APT_IRQService(&g_apt0);
    IRQ_EnableN(g_apt0.irqNumTmr);
}

static void APT1_ProtectInit(void)
{
    APT_OutCtrlProtectEx protectApt = {0};
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = BASE_CFG_DISABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_ACMP1;
    protectApt.evtPolarityMaskEx = APT_EM_ACMP1_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    HAL_APT_ProtectInitEx(&g_apt1, &protectApt);
}

static void APT1_Init(void)
{
    HAL_CRG_IpEnableSet(APT1_BASE, IP_CLK_ENABLE);

    g_apt1.baseAddress = APT1;
    g_apt1.irqNumEvt = IRQ_APT1_EVT;
    g_apt1.irqNumTmr = IRQ_APT1_TMR;

    /* Clock Settings */
    g_apt1.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt1.waveform.timerPeriod = 6250;
    g_apt1.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_apt1.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt1.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt1.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt1.waveform.divInitVal = 0;
    g_apt1.waveform.cntInitVal = 0;
    g_apt1.waveform.cntCmpLeftEdge = 3125;
    g_apt1.waveform.cntCmpRightEdge = 3125;
    g_apt1.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt1.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt1.waveform.deadBandCnt = 200;

    APT1_ProtectInit();

    HAL_APT_PWMInit(&g_apt1);
}

static void APT2_ProtectInit(void)
{
    APT_OutCtrlProtectEx protectApt = {0};
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = BASE_CFG_DISABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_ACMP1;
    protectApt.evtPolarityMaskEx = APT_EM_ACMP1_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    HAL_APT_ProtectInitEx(&g_apt2, &protectApt);
}

static void APT2_Init(void)
{
    HAL_CRG_IpEnableSet(APT2_BASE, IP_CLK_ENABLE);

    g_apt2.baseAddress = APT2;
    g_apt2.irqNumEvt = IRQ_APT2_EVT;
    g_apt2.irqNumTmr = IRQ_APT2_TMR;

    /* Clock Settings */
    g_apt2.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt2.waveform.timerPeriod = 6250;
    g_apt2.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_apt2.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt2.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt2.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt2.waveform.divInitVal = 0;
    g_apt2.waveform.cntInitVal = 0;
    g_apt2.waveform.cntCmpLeftEdge = 3125;
    g_apt2.waveform.cntCmpRightEdge = 3125;
    g_apt2.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt2.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt2.waveform.deadBandCnt = 200;

    APT2_ProtectInit();

    HAL_APT_PWMInit(&g_apt2);
}

__weak void APT3EventCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
    /* USER CODE BEGIN APT3_EVENT_INTERRUPT */
    /* USER CODE END APT3_EVENT_INTERRUPT */
}

__weak void APT3TimerCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
    /* USER CODE BEGIN APT3_TIMER_INTERRUPT */
    /* USER CODE END APT3_TIMER_INTERRUPT */
}

static void APT3_ProtectInit(void)
{
    APT_OutCtrlProtectEx protectApt = {0};
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = BASE_CFG_ENABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_POE2;
    protectApt.evtPolarityMaskEx = APT_EM_POE2_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    HAL_APT_ProtectInitEx(&g_apt3, &protectApt);
}

static void APT3_Init(void)
{
    HAL_CRG_IpEnableSet(APT3_BASE, IP_CLK_ENABLE);

    g_apt3.baseAddress = APT3;
    g_apt3.irqNumEvt = IRQ_APT3_EVT;
    g_apt3.irqNumTmr = IRQ_APT3_TMR;

    /* Clock Settings */
    g_apt3.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt3.waveform.timerPeriod = 16000;
    g_apt3.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_apt3.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt3.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt3.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt3.waveform.divInitVal = 0;
    g_apt3.waveform.cntInitVal = 0;
    g_apt3.waveform.cntCmpLeftEdge = 8000;
    g_apt3.waveform.cntCmpRightEdge = 8000;
    g_apt3.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt3.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO | APT_COMPARE_LOAD_EVENT_PERIOD;
    g_apt3.waveform.deadBandCnt = 300;

    /* ADC Trigger SOCA */
    g_apt3.adcTrg.trgEnSOCA = BASE_CFG_ENABLE;
    g_apt3.adcTrg.cntCmpSOCA = 12000;
    g_apt3.adcTrg.trgSrcSOCA = APT_CS_SRC_CNTR_CMPA_DOWN;
    g_apt3.adcTrg.trgScaleSOCA = 1;

    /* ADC Trigger SOCB */
    g_apt3.adcTrg.trgEnSOCB = BASE_CFG_ENABLE;
    g_apt3.adcTrg.cntCmpSOCB =  4000;
    g_apt3.adcTrg.trgSrcSOCB = APT_CS_SRC_CNTR_CMPB_DOWN;
    g_apt3.adcTrg.trgScaleSOCB = 1;

    g_apt3.adcTrg.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt3.adcTrg.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_PERIOD;

    /* Timer Trigger */
    g_apt3.tmrInterrupt.tmrInterruptEn = BASE_CFG_ENABLE;
    g_apt3.tmrInterrupt.tmrInterruptSrc = APT_INT_SRC_CNTR_ZERO;
    g_apt3.tmrInterrupt.tmrInterruptScale = 1;

    APT3_ProtectInit();

    HAL_APT_PWMInit(&g_apt3);
    HAL_APT_RegisterCallBack(&g_apt3, APT_EVENT_INTERRUPT, APT3EventCallback);
    IRQ_SetPriority(g_apt3.irqNumEvt, 7);
    HAL_APT_IRQService(&g_apt3);
    IRQ_EnableN(g_apt3.irqNumEvt);
    HAL_APT_RegisterCallBack(&g_apt3, APT_TIMER_INTERRUPT, APT3TimerCallback);
    IRQ_SetPriority(g_apt3.irqNumTmr, 4);
    HAL_APT_IRQService(&g_apt3);
    IRQ_EnableN(g_apt3.irqNumTmr);
}

__weak void APT4TimerCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
    /* USER CODE BEGIN APT4_TIMER_INTERRUPT */
    /* USER CODE END APT4_TIMER_INTERRUPT */
}

static void APT4_ProtectInit(void)
{
    APT_OutCtrlProtectEx protectApt = {0};
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = BASE_CFG_DISABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_POE2;
    protectApt.evtPolarityMaskEx = APT_EM_POE2_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    HAL_APT_ProtectInitEx(&g_apt4, &protectApt);
}

static void APT4_Init(void)
{
    HAL_CRG_IpEnableSet(APT4_BASE, IP_CLK_ENABLE);

    g_apt4.baseAddress = APT4;
    g_apt4.irqNumEvt = IRQ_APT4_EVT;
    g_apt4.irqNumTmr = IRQ_APT4_TMR;

    /* Clock Settings */
    g_apt4.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt4.waveform.timerPeriod = 16000;
    g_apt4.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_apt4.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt4.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt4.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt4.waveform.divInitVal = 0;
    g_apt4.waveform.cntInitVal = 0;
    g_apt4.waveform.cntCmpLeftEdge = 8000;
    g_apt4.waveform.cntCmpRightEdge = 8000;
    g_apt4.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt4.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO | APT_COMPARE_LOAD_EVENT_PERIOD;
    g_apt4.waveform.deadBandCnt = 300;

    /* Timer Trigger */
    g_apt4.tmrInterrupt.tmrInterruptEn = BASE_CFG_ENABLE;
    g_apt4.tmrInterrupt.tmrInterruptSrc = APT_INT_SRC_CNTR_PERIOD;
    g_apt4.tmrInterrupt.tmrInterruptScale = 1;

    APT4_ProtectInit();

    HAL_APT_PWMInit(&g_apt4);
    HAL_APT_RegisterCallBack(&g_apt4, APT_TIMER_INTERRUPT, APT4TimerCallback);
    IRQ_SetPriority(g_apt4.irqNumTmr, 4);
    HAL_APT_IRQService(&g_apt4);
    IRQ_EnableN(g_apt4.irqNumTmr);
}

static void APT5_ProtectInit(void)
{
    APT_OutCtrlProtectEx protectApt = {0};
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = BASE_CFG_DISABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_POE2;
    protectApt.evtPolarityMaskEx = APT_EM_POE2_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    HAL_APT_ProtectInitEx(&g_apt5, &protectApt);
}

static void APT5_Init(void)
{
    HAL_CRG_IpEnableSet(APT5_BASE, IP_CLK_ENABLE);

    g_apt5.baseAddress = APT5;
    g_apt5.irqNumEvt = IRQ_APT5_EVT;
    g_apt5.irqNumTmr = IRQ_APT5_TMR;

    /* Clock Settings */
    g_apt5.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt5.waveform.timerPeriod = 16000;
    g_apt5.waveform.cntMode = APT_COUNT_MODE_UP_DOWN;

    /* Wave Form */
    g_apt5.waveform.basicType = APT_PWM_BASIC_A_HIGH_B_LOW;
    g_apt5.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt5.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt5.waveform.divInitVal = 0;
    g_apt5.waveform.cntInitVal = 0;
    g_apt5.waveform.cntCmpLeftEdge = 8000;
    g_apt5.waveform.cntCmpRightEdge = 8000;
    g_apt5.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt5.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO | APT_COMPARE_LOAD_EVENT_PERIOD;
    g_apt5.waveform.deadBandCnt = 300;

    APT5_ProtectInit();

    HAL_APT_PWMInit(&g_apt5);
}

__weak void APT8EventCallback(void *aptHandle)
{
    BASE_FUNC_UNUSED(aptHandle);
    /* USER CODE BEGIN APT8_EVENT_INTERRUPT */
    /* USER CODE END APT8_EVENT_INTERRUPT */
}

static void APT8_ProtectInit(void)
{
    APT_OutCtrlProtectEx protectApt = {0};
    protectApt.ocEventEnEx = BASE_CFG_ENABLE;
    protectApt.ocEventModeEx = APT_OUT_CTRL_ONE_SHOT;
    protectApt.cbcClrModeEx = APT_CLEAR_CBC_ON_CNTR_ZERO;
    protectApt.ocActionEx = APT_OUT_CTRL_ACTION_LOW;
    protectApt.ocEvtInterruptEnEx = BASE_CFG_ENABLE;
    protectApt.ocSysEvent = APT_SYS_EVT_DEBUG | APT_SYS_EVT_CLK | APT_SYS_EVT_MEM;
    protectApt.originalEvtEx = APT_EM_ORIGINAL_SRC_POE0;
    protectApt.evtPolarityMaskEx = APT_EM_POE0_INVERT_BIT;
    protectApt.filterCycleNumEx = 0;
    HAL_APT_ProtectInitEx(&g_apt8, &protectApt);
}

static void APT8_Init(void)
{
    HAL_CRG_IpEnableSet(APT8_BASE, IP_CLK_ENABLE);

    g_apt8.baseAddress = APT8;
    g_apt8.irqNumEvt = IRQ_APT8_EVT;
    g_apt8.irqNumTmr = IRQ_APT8_TMR;

    /* Clock Settings */
    g_apt8.waveform.dividerFactor = 1 - 1;
    /* Timer Settings */
    g_apt8.waveform.timerPeriod = 5000;
    g_apt8.waveform.cntMode = APT_COUNT_MODE_UP;

    /* Wave Form */
    g_apt8.waveform.basicType = APT_PWM_BASIC_A_LOW_B_HIGH;
    g_apt8.waveform.chAOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt8.waveform.chBOutType = APT_PWM_OUT_BASIC_TYPE;
    g_apt8.waveform.divInitVal = 0;
    g_apt8.waveform.cntInitVal = 0;
    g_apt8.waveform.cntCmpLeftEdge = 0;
    g_apt8.waveform.cntCmpRightEdge = 0;
    g_apt8.waveform.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt8.waveform.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;
    g_apt8.waveform.deadBandCnt = 0;

    /* ADC Trigger SOCA */
    g_apt8.adcTrg.trgEnSOCA = BASE_CFG_ENABLE;
    g_apt8.adcTrg.cntCmpSOCA = 1250;
    g_apt8.adcTrg.trgSrcSOCA = APT_CS_SRC_CNTR_CMPA_UP;
    g_apt8.adcTrg.trgScaleSOCA = 1;

    /* ADC Trigger SOCB */
    g_apt8.adcTrg.trgEnSOCB = BASE_CFG_ENABLE;
    g_apt8.adcTrg.cntCmpSOCB =  3750;
    g_apt8.adcTrg.trgSrcSOCB = APT_CS_SRC_CNTR_CMPB_UP;
    g_apt8.adcTrg.trgScaleSOCB = 1;

    g_apt8.adcTrg.cntCmpLoadMode = APT_BUFFER_INDEPENDENT_LOAD;
    g_apt8.adcTrg.cntCmpLoadEvt = APT_COMPARE_LOAD_EVENT_ZERO;

    APT8_ProtectInit();

    HAL_APT_PWMInit(&g_apt8);
    HAL_APT_RegisterCallBack(&g_apt8, APT_EVENT_INTERRUPT, APT8EventCallback);
    IRQ_SetPriority(g_apt8.irqNumEvt, 7);
    HAL_APT_IRQService(&g_apt8);
    IRQ_EnableN(g_apt8.irqNumEvt);
}

static void GPIO_Init(void)
{
    HAL_CRG_IpEnableSet(GPIO1_BASE, IP_CLK_ENABLE);
    g_gpio1.baseAddress = GPIO1;
    g_gpio1.dir = GPIO_OUTPUT_MODE;
    g_gpio1.value = GPIO_LOW_LEVEL;
    g_gpio1.interruptMode = GPIO_INT_TYPE_NONE;
    g_gpio1.pins = GPIO_PIN_6 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4;
    HAL_GPIO_Init(&g_gpio1);

    g_gpio1.dir = GPIO_OUTPUT_MODE;
    g_gpio1.value = GPIO_HIGH_LEVEL;
    g_gpio1.interruptMode = GPIO_INT_TYPE_NONE;
    g_gpio1.pins = GPIO_PIN_0;
    HAL_GPIO_Init(&g_gpio1);

    HAL_CRG_IpEnableSet(GPIO5_BASE, IP_CLK_ENABLE);
    g_gpio5.baseAddress = GPIO5;
    g_gpio5.dir = GPIO_OUTPUT_MODE;
    g_gpio5.value = GPIO_HIGH_LEVEL;
    g_gpio5.interruptMode = GPIO_INT_TYPE_NONE;
    g_gpio5.pins = GPIO_PIN_0 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(&g_gpio5);

    HAL_CRG_IpEnableSet(GPIO6_BASE, IP_CLK_ENABLE);
    g_gpio6.baseAddress = GPIO6;
    g_gpio6.dir = GPIO_OUTPUT_MODE;
    g_gpio6.value = GPIO_LOW_LEVEL;
    g_gpio6.interruptMode = GPIO_INT_TYPE_NONE;
    g_gpio6.pins = GPIO_PIN_2 | GPIO_PIN_4;
    HAL_GPIO_Init(&g_gpio6);

    HAL_CRG_IpEnableSet(GPIO0_BASE, IP_CLK_ENABLE);
    g_gpio0.baseAddress = GPIO0;
    g_gpio0.dir = GPIO_OUTPUT_MODE;
    g_gpio0.value = GPIO_LOW_LEVEL;
    g_gpio0.interruptMode = GPIO_INT_TYPE_NONE;
    g_gpio0.pins = GPIO_PIN_5;
    HAL_GPIO_Init(&g_gpio0);

    g_gpio0.dir = GPIO_OUTPUT_MODE;
    g_gpio0.value = GPIO_HIGH_LEVEL;
    g_gpio0.interruptMode = GPIO_INT_TYPE_NONE;
    g_gpio0.pins = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(&g_gpio0);

    return;
}

static void PGA0_Init(void)
{
    HAL_CRG_IpEnableSet(PGA0_BASE, IP_CLK_ENABLE);

    g_pga0.baseAddress = PGA0_BASE;
    g_pga0.enable = BASE_CFG_ENABLE;
    g_pga0.extLoopbackEn = BASE_CFG_ENABLE;
    g_pga0.pgaMux = PGA_EXT_RES_VI0;
    g_pga0.pgaSwVinN = PGA_SW_VIN0;
    g_pga0.pgaSwVinP = PGA_SW_VIN0;
    g_pga0.gain = PGA_GAIN_1X;

    HAL_PGA_Init(&g_pga0);
}

static void PGA1_Init(void)
{
    HAL_CRG_IpEnableSet(PGA1_BASE, IP_CLK_ENABLE);

    g_pga1.baseAddress = PGA1_BASE;
    g_pga1.enable = BASE_CFG_ENABLE;
    g_pga1.extLoopbackEn = BASE_CFG_DISABLE;
    g_pga1.pgaMux = PGA_INTER_RES_VI0;
    g_pga1.pgaSwVinN = PGA_SW_VIN0;
    g_pga1.pgaSwVinP = PGA_SW_VIN0;
    g_pga1.gain = PGA_GAIN_1X;

    HAL_PGA_Init(&g_pga1);
}

__weak void TIMER0CallbackFunction(void *handle)
{
    HAL_TIMER_IrqClear((TIMER_Handle *)handle);
    /* USER CODE BEGIN TIMER0 ITCallBackFunc */
    /* USER CODE END TIMER0 ITCallBackFunc */
}
static void TIMER0_Init(void)
{
    unsigned int load = HAL_CRG_GetIpFreq((void *)TIMER0) / 1000000u * 160;

    HAL_CRG_IpEnableSet(TIMER0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(TIMER0_BASE, CRG_PLL_NO_PREDV);

    g_timer0.baseAddress = TIMER0;
    g_timer0.irqNum = IRQ_TIMER0;

    g_timer0.load        = load - 1; /* Set timer value immediately */
    g_timer0.bgLoad      = load - 1; /* Set timer value */
    g_timer0.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timer0.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer0.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer0.dmaAdcSingleReqEnable = BASE_CFG_DISABLE;
    g_timer0.dmaBurstReqEnable = BASE_CFG_DISABLE;
    g_timer0.interruptEn = BASE_CFG_ENABLE;
    HAL_TIMER_Init(&g_timer0);

    HAL_TIMER_RegisterCallback(&g_timer0, TIMER0CallbackFunction);
    IRQ_SetPriority(g_timer0.irqNum, 3);
    IRQ_EnableN(g_timer0.irqNum);
}

__weak void TIMER1CallbackFunction(void *handle)
{
    HAL_TIMER_IrqClear((TIMER_Handle *)handle);
    /* USER CODE BEGIN TIMER1 ITCallBackFunc */
    /* USER CODE END TIMER1 ITCallBackFunc */
}
static void TIMER1_Init(void)
{
    unsigned int load = HAL_CRG_GetIpFreq((void *)TIMER1) / 1000000u * 320;

    HAL_CRG_IpEnableSet(TIMER1_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(TIMER1_BASE, CRG_PLL_NO_PREDV);

    g_timer1.baseAddress = TIMER1;
    g_timer1.irqNum = IRQ_TIMER1;

    g_timer1.load        = load - 1; /* Set timer value immediately */
    g_timer1.bgLoad      = load - 1; /* Set timer value */
    g_timer1.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timer1.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer1.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer1.dmaAdcSingleReqEnable = BASE_CFG_DISABLE;
    g_timer1.dmaBurstReqEnable = BASE_CFG_DISABLE;
    g_timer1.interruptEn = BASE_CFG_ENABLE;
    HAL_TIMER_Init(&g_timer1);

    HAL_TIMER_RegisterCallback(&g_timer1, TIMER1CallbackFunction);
    IRQ_SetPriority(g_timer1.irqNum, 2);
    IRQ_EnableN(g_timer1.irqNum);
}

__weak void TIMER2CallbackFunction(void *handle)
{
    HAL_TIMER_IrqClear((TIMER_Handle *)handle);
    /* USER CODE BEGIN TIMER2 ITCallBackFunc */
    /* USER CODE END TIMER2 ITCallBackFunc */
}
static void TIMER2_Init(void)
{
    unsigned int load = HAL_CRG_GetIpFreq((void *)TIMER2) / 1000000u * 1000;

    HAL_CRG_IpEnableSet(TIMER2_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(TIMER2_BASE, CRG_PLL_NO_PREDV);

    g_timer2.baseAddress = TIMER2;
    g_timer2.irqNum = IRQ_TIMER2;

    g_timer2.load        = load - 1; /* Set timer value immediately */
    g_timer2.bgLoad      = load - 1; /* Set timer value */
    g_timer2.mode        = TIMER_MODE_RUN_PERIODIC; /* Run in period mode */
    g_timer2.prescaler   = TIMERPRESCALER_NO_DIV; /* Don't frequency division */
    g_timer2.size        = TIMER_SIZE_32BIT; /* 1 for 32bit, 0 for 16bit */
    g_timer2.dmaAdcSingleReqEnable = BASE_CFG_DISABLE;
    g_timer2.dmaBurstReqEnable = BASE_CFG_DISABLE;
    g_timer2.interruptEn = BASE_CFG_ENABLE;
    HAL_TIMER_Init(&g_timer2);

    HAL_TIMER_RegisterCallback(&g_timer2, TIMER2CallbackFunction);
    IRQ_SetPriority(g_timer2.irqNum, 1);
    IRQ_EnableN(g_timer2.irqNum);
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART0_BASE, CRG_PLL_NO_PREDV);

    g_uart0.baseAddress = UART0;
    g_uart0.irqNum = IRQ_UART0;

    g_uart0.baudRate = UART0_BAND_RATE;
    g_uart0.dataLength = UART_DATALENGTH_8BIT;
    g_uart0.stopBits = UART_STOPBITS_ONE;
    g_uart0.parity = UART_PARITY_NONE;
    g_uart0.txMode = UART_MODE_BLOCKING;
    g_uart0.rxMode = UART_MODE_BLOCKING;
    g_uart0.fifoMode = BASE_CFG_ENABLE;
    g_uart0.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart0.hwFlowCtr = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart0);
}

static void UART2_Init(void)
{
    HAL_CRG_IpEnableSet(UART2_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART2_BASE, CRG_PLL_NO_PREDV);

    g_uart2.baseAddress = UART2;
    g_uart2.irqNum = IRQ_UART2;

    g_uart2.baudRate = UART2_BAND_RATE;
    g_uart2.dataLength = UART_DATALENGTH_8BIT;
    g_uart2.stopBits = UART_STOPBITS_ONE;
    g_uart2.parity = UART_PARITY_NONE;
    g_uart2.txMode = UART_MODE_BLOCKING;
    g_uart2.rxMode = UART_MODE_BLOCKING;
    g_uart2.fifoMode = BASE_CFG_ENABLE;
    g_uart2.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart2.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart2.hwFlowCtr = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart2);
}

static void IOConfig(void)
{
    IOConfig_RegStruct *iconfig = IOCONFIG;
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode = 0;
    SYSCTRL0->SC_SYS_STAT.BIT.update_mode_clear = 1;

    iconfig->iocmg_17.BIT.func = 0x0; /* 0x0 is GPIO1_6 */
    iconfig->iocmg_17.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_17.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_17.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_17.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_17.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_43.BIT.func = 0x0; /* 0x0 is GPIO5_0 */
    iconfig->iocmg_43.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_43.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_43.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_43.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_43.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_48.BIT.func = 0x0; /* 0x0 is GPIO5_5 */
    iconfig->iocmg_48.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_48.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_48.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_48.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_48.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_49.BIT.func = 0x0; /* 0x0 is GPIO5_6 */
    iconfig->iocmg_49.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_49.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_49.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_49.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_49.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_50.BIT.func = 0x0; /* 0x0 is GPIO5_7 */
    iconfig->iocmg_50.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_50.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_50.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_50.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_50.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_53.BIT.func = 0x0; /* 0x0 is GPIO6_2 */
    iconfig->iocmg_53.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_53.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_53.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_53.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_53.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_55.BIT.func = 0x0; /* 0x0 is GPIO6_4 */
    iconfig->iocmg_55.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_55.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_55.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_55.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_55.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_8.BIT.func = 0x0; /* 0x0 is GPIO0_5 */
    iconfig->iocmg_8.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_8.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_8.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_8.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_8.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_9.BIT.func = 0x0; /* 0x0 is GPIO0_6 */
    iconfig->iocmg_9.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_9.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_9.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_9.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_9.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_10.BIT.func = 0x0; /* 0x0 is GPIO0_7 */
    iconfig->iocmg_10.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_10.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_10.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_10.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_10.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_11.BIT.func = 0x0; /* 0x0 is GPIO1_0 */
    iconfig->iocmg_11.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_11.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_11.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_11.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_11.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_12.BIT.func = 0x0; /* 0x0 is GPIO1_1 */
    iconfig->iocmg_12.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_12.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_12.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_12.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_12.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_14.BIT.func = 0x0; /* 0x0 is GPIO1_3 */
    iconfig->iocmg_14.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_14.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_14.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_14.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_14.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_15.BIT.func = 0x0; /* 0x0 is GPIO1_4 */
    iconfig->iocmg_15.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_15.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_15.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_15.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_15.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_18.BIT.func = 0x9; /* 0x9 is ACMP1_ANA_N */
    iconfig->iocmg_18.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_18.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_18.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_18.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_18.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_21.BIT.func = 0x9; /* 0x9 is ACMP1_ANA_P */
    iconfig->iocmg_21.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_21.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_21.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_21.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_21.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_22.BIT.func = 0x8; /* 0x8 is ADC1_ANA_A2 */
    iconfig->iocmg_22.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_22.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_22.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_22.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_22.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_23.BIT.func = 0x8; /* 0x8 is ADC1_ANA_B2 */
    iconfig->iocmg_23.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_23.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_23.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_23.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_23.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_26.BIT.func = 0x8; /* 0x8 is ADC1_ANA_A4 */
    iconfig->iocmg_26.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_26.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_26.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_26.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_26.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_27.BIT.func = 0x8; /* 0x8 is ADC1_ANA_A5 */
    iconfig->iocmg_27.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_27.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_27.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_27.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_27.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_19.BIT.func = 0x8; /* 0x8 is ADC1_ANA_B4 */
    iconfig->iocmg_19.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_19.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_19.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_19.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_19.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_20.BIT.func = 0x8; /* 0x8 is ADC1_ANA_B5 */
    iconfig->iocmg_20.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_20.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_20.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_20.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_20.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_24.BIT.func = 0x8; /* 0x8 is PGA1_ANA_EXT */
    iconfig->iocmg_24.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_24.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_24.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_24.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_24.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_25.BIT.func = 0x4; /* 0x4 is POE0 */
    iconfig->iocmg_25.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_25.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_25.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_25.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_25.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_54.BIT.func = 0x2; /* 0x2 is POE2 */
    iconfig->iocmg_54.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_54.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_54.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_54.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_54.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_28.BIT.func = 0x3; /* 0x3 is APT8_PWMB */
    iconfig->iocmg_28.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_28.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_28.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_28.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_28.BIT.se = BASE_CFG_DISABLE;

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

    iconfig->iocmg_38.BIT.func = 0x9; /* 0x9 is PGA0_ANA_P */
    iconfig->iocmg_38.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_38.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_38.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_38.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_38.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_39.BIT.func = 0x9; /* 0x9 is PGA0_ANA_N */
    iconfig->iocmg_39.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_39.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_39.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_39.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_39.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_40.BIT.func = 0x8; /* 0x8 is PGA0_ANA_EXT */
    iconfig->iocmg_40.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_40.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_40.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_40.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_40.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_41.BIT.func = 0x8; /* 0x8 is ADC0_ANA_A3 */
    iconfig->iocmg_41.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_41.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_41.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_41.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_41.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_42.BIT.func = 0x8; /* 0x8 is ADC0_ANA_A4 */
    iconfig->iocmg_42.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_42.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_42.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_42.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_42.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_58.BIT.func = 0x1; /* 0x1 is APT3_PWMA */
    iconfig->iocmg_58.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_58.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_58.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_58.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_58.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_61.BIT.func = 0x1; /* 0x1 is APT3_PWMB */
    iconfig->iocmg_61.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_61.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_61.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_61.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_61.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_59.BIT.func = 0x1; /* 0x1 is APT4_PWMA */
    iconfig->iocmg_59.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_59.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_59.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_59.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_59.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_62.BIT.func = 0x1; /* 0x1 is APT4_PWMB */
    iconfig->iocmg_62.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_62.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_62.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_62.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_62.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_60.BIT.func = 0x1; /* 0x1 is APT5_PWMA */
    iconfig->iocmg_60.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_60.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_60.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_60.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_60.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_63.BIT.func = 0x1; /* 0x1 is APT5_PWMB */
    iconfig->iocmg_63.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_63.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_63.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_63.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_63.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_67.BIT.func = 0x8; /* 0x8 is ADC2_ANA_A4 */
    iconfig->iocmg_67.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_67.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_67.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_67.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_67.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_66.BIT.func = 0x8; /* 0x8 is ADC2_ANA_A3 */
    iconfig->iocmg_66.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_66.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_66.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_66.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_66.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_6.BIT.func = 0x4; /* 0x4 is UART0_TXD */
    iconfig->iocmg_6.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_6.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_6.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_6.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_6.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_7.BIT.func = 0x4; /* 0x4 is UART0_RXD */
    iconfig->iocmg_7.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_7.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_7.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_7.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_7.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_51.BIT.func = 0x3; /* 0x3 is UART2_TXD */
    iconfig->iocmg_51.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_51.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_51.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_51.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_51.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_52.BIT.func = 0x3; /* 0x3 is UART2_RXD */
    iconfig->iocmg_52.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_52.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_52.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_52.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_52.BIT.se = BASE_CFG_DISABLE;
}

static void APT_SyncMasterInit(void)
{
    HAL_APT_MasterSyncInit(&g_apt3, APT_SYNC_OUT_ON_CNTR_ZERO);
    HAL_APT_MasterSyncInit(&g_apt0, APT_SYNC_OUT_ON_CNTR_ZERO);
}

static void APT_SyncSlaveInit(void)
{
    APT_SlaveSyncIn aptSlave;

    aptSlave.divPhase = 0; /* divide phase value */
    aptSlave.cntPhase = 0; /* counter phase value 0 */
    aptSlave.syncCntMode = APT_COUNT_MODE_AFTER_SYNC_UP;
    aptSlave.syncInSrc = APT_SYNCIN_SRC_APT3_SYNCOUT; /* sync source selection */
    aptSlave.cntrSyncSrc = APT_CNTR_SYNC_SRC_SYNCIN;
    HAL_APT_SlaveSyncInit(&g_apt4, &aptSlave);

    aptSlave.divPhase = 0; /* divide phase value */
    aptSlave.cntPhase = 0; /* counter phase value 0 */
    aptSlave.syncCntMode = APT_COUNT_MODE_AFTER_SYNC_UP;
    aptSlave.syncInSrc = APT_SYNCIN_SRC_APT3_SYNCOUT; /* sync source selection */
    aptSlave.cntrSyncSrc = APT_CNTR_SYNC_SRC_SYNCIN;
    HAL_APT_SlaveSyncInit(&g_apt5, &aptSlave);

    aptSlave.divPhase = 0; /* divide phase value */
    aptSlave.cntPhase = 0; /* counter phase value 0 */
    aptSlave.syncCntMode = APT_COUNT_MODE_AFTER_SYNC_UP;
    aptSlave.syncInSrc = APT_SYNCIN_SRC_APT0_SYNCOUT; /* sync source selection */
    aptSlave.cntrSyncSrc = APT_CNTR_SYNC_SRC_SYNCIN;
    HAL_APT_SlaveSyncInit(&g_apt1, &aptSlave);

    aptSlave.divPhase = 0; /* divide phase value */
    aptSlave.cntPhase = 0; /* counter phase value 0 */
    aptSlave.syncCntMode = APT_COUNT_MODE_AFTER_SYNC_UP;
    aptSlave.syncInSrc = APT_SYNCIN_SRC_APT0_SYNCOUT; /* sync source selection */
    aptSlave.cntrSyncSrc = APT_CNTR_SYNC_SRC_SYNCIN;
    HAL_APT_SlaveSyncInit(&g_apt2, &aptSlave);
}

void SystemInit(void)
{
    IOConfig();
    UART0_Init();
    UART2_Init();
    ACMP1_Init();
    APT0_Init();
    APT1_Init();
    APT2_Init();
    APT3_Init();
    APT4_Init();
    APT5_Init();
    APT8_Init();
    ADC0_Init();
    ADC1_Init();
    ADC2_Init();
    PGA0_Init();
    PGA1_Init();
    TIMER0_Init();
    TIMER1_Init();
    TIMER2_Init();
    GPIO_Init();

    APT_SyncMasterInit();
    APT_SyncSlaveInit();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}