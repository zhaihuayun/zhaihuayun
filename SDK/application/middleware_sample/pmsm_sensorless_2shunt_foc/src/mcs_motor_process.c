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
  * @file      mcs_motor_process.c
  * @author    MCU Algorithm Team
  * @brief     This file provides motor application for ECMCU105H board.
  * @details   Single FOC application based on the ECMCU105H board
  *            1) Motor model is Gimbal GBM2804H-100T.
  *            2) Select the pmsm sensorless 2shunt foc example in the sample column
                  of chipConfig and click Generate Code.
  *            3) It's power supply must be changed to 12V.
  */
#include "main.h"
#include "mcs_motor_process.h"
#include "mcs_config.h"
#include "mcs_math_const.h"
#include "mcs_math.h"
#include "mcs_motor.h"
#include "mcs_carrier.h"

/*------------------------------- Macro Definition -----------------------------------------------*/
#define US_PER_MS               1000
#define ANGLE_RANGE_ABS         65536
#define ANGLE_360_F             65536.0f /* 0 - 65536 indicates 0 to 360. */
#define APT_FULL_DUTY           1.0f
/*------------------------------- Param Definition -----------------------------------------------*/
/* Motor parameters. */
static MotorConfig g_motorParam = {
    .mtrNp = 7,
    .mtrRs = 10.1f,
    .mtrLd = 0.00362f,
    .mtrLq = 0.00362f,
    .mtrPsif = 0.0f,
    .mtrJ = 0.0f,
    .maxElecSpd = 180.25f,
    .maxCurr = 1.0f
};

/* Motor PI param. */
static PiCtrlParam g_motorPiParam = {
    .currPiKp = 0.7414f,
    .currPiKi = 0.1256f,
    .currPiLimitPu = 1.0f,
    .spdPiKp = 0.00505f,
    .spdPiKi = 0.000003f,
    .spdPiLimit = 1.0f
};

static APT_RegStruct* g_aptCp[PHASE_MAX_NUM] = {APT_U_CP, APT_V_CP, APT_W_CP};
/* Motor control handle */
static MtrCtrlHandle g_mc = {0};
/* global variables for variable trace */
volatile float g_mc_u = 0.0;
volatile float g_mc_v = 0.0;
volatile float g_mc_w = 0.0;
volatile int g_tracevalue_ref = 1;
/*------------------------------- Function Definition -----------------------------------------------*/
/**
  * @brief Initialzer of system tick.
  * @param mtrCtrl Motor control struct handle.
  * @retval None.
  */
static void TimerTickInit(MtrCtrlHandle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    mtrCtrl->sysTickCnt = 0;
    mtrCtrl->msTickNum = US_PER_MS / SYSTICK_PERIOD_US;
    mtrCtrl->capChargeTickNum = (INV_CAP_CHARGE_MS * US_PER_MS / SYSTICK_PERIOD_US);
}

/**
  * @brief Initialzer of Current controller.
  * @param currHandle Current control handle.
  * @param mtrCtrl Motor control handle.
  * @param piCtrlTable Dual loop control parameters.
  * @retval None.
  */
static void CurrCtrlInit(CurrCtrlHandle *currHandle, MtrCtrlHandle *mtrCtrl, const PiCtrlParam *piCtrlTable)
{
    MCS_ASSERT_PARAM(currHandle != NULL);
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(piCtrlTable != NULL);
    currHandle->currRef  = &mtrCtrl->currRefDq;
    currHandle->currFdbk = &mtrCtrl->currDq;
    /* Initialization current feedforward value */
    currHandle->currFf.d = 0.0f;
    currHandle->currFf.q = 0.0f;

    PID_Reset(&currHandle->dAxisPi); /* clear d-axis PI param */

    PID_Reset(&currHandle->qAxisPi); /* clear q-axis PI param */

    currHandle->outLimit = 0.0f;
    currHandle->mtrParam = &(mtrCtrl->mtrParam);
    currHandle->ctrlPeriod = mtrCtrl->currCtrlPeriod;

    currHandle->dAxisPi.kp = piCtrlTable->currPiKp;
    currHandle->dAxisPi.ki = piCtrlTable->currPiKi;
    currHandle->qAxisPi.kp = piCtrlTable->currPiKp;
    currHandle->qAxisPi.ki = piCtrlTable->currPiKi;
    currHandle->outLimit = INV_VOLTAGE_BUS * ONE_DIV_SQRT3 * piCtrlTable->currPiLimitPu; /* output voltage limit */
    currHandle->dAxisPi.upperLimit = currHandle->outLimit;
    currHandle->dAxisPi.lowerLimit = -currHandle->outLimit;
    currHandle->qAxisPi.upperLimit = currHandle->outLimit;
    currHandle->qAxisPi.lowerLimit = -currHandle->outLimit;
}

/**
  * @brief Initialzer of speed control struct handle.
  * @param spdHandle Speed control struct handle.
  * @param piCtrlTable PI controller parameter table.
  * @retval None.
  */
static void SpdCtrlInit(SpdCtrlHandle *spdHandle, const PiCtrlParam *piCtrlTable)
{
    MCS_ASSERT_PARAM(spdHandle != NULL);
    MCS_ASSERT_PARAM(piCtrlTable != NULL);
    PID_Reset(&spdHandle->spdPi); /* reset speed loop PI */

    spdHandle->trqLimit = 0.0f;
    spdHandle->mtrParam = NULL;
    spdHandle->ctrlPeriod = CTRL_SYSTICK_PERIOD;
    /* Initialize the PI parameter of the speed loop. */
    spdHandle->spdPi.kp = piCtrlTable->spdPiKp;
    spdHandle->spdPi.ki = piCtrlTable->spdPiKi;
    spdHandle->trqLimit = piCtrlTable->spdPiLimit;
    spdHandle->spdPi.upperLimit = spdHandle->trqLimit;
    spdHandle->spdPi.lowerLimit = -spdHandle->trqLimit;
}

/**
  * @brief Initialzer of Pll struct handle.
  * @param pllHandle pll struct handle.
  * @retval None.
  */
static void PLL_Init(PllHandle *pllHandle)
{
    MCS_ASSERT_PARAM(pllHandle != NULL);
#ifdef SMO4TH
    float we = SPECIAL_SMO4TH_PLL_BDW;  /* PLL bandwidth for smo4th(unit: Hz) */
#else
    float we = 300.0f; /* 300.0 : PLL bandwidth for smo(unit: Hz) */
#endif
    /* Reset PLL PID. */
    PID_Reset(&pllHandle->pi);
    /* Initializing PLL Parameters. */
    pllHandle->ctrlPeriod = CTRL_CURR_PERIOD;
    pllHandle->pi.kp = 1.414f * we; /* 1.414 = sqrt(2) * we */
    pllHandle->pi.ki = we * we * CTRL_CURR_PERIOD;
    pllHandle->pi.upperLimit = 5000.0f; /* 5000.0 : The upper limit value of the pid comp output. */
    pllHandle->pi.lowerLimit = -pllHandle->pi.upperLimit;
    pllHandle->minAmp = 0.1f; /* 0.1 : Minimum value of the input value in case of the divergence of the PLL. */
    pllHandle->freq = 0.0f;
    pllHandle->ratio = ANGLE_360_F * CTRL_CURR_PERIOD;
    pllHandle->angle = 0;
}

/**
  * @brief Initialzer of Smo control struct handle.
  * @param smoHandle Position SMO struct handle.
  * @param mtrParam Motor parameter handle.
  * @retval None.
  */
static void SmoInit(FoSmoHandle *smoHandle, MtrParamHandle *mtrParam)
{
    MCS_ASSERT_PARAM(smoHandle != NULL);
    MCS_ASSERT_PARAM(mtrParam != NULL);
    smoHandle->mtrParam = mtrParam;
    smoHandle->ctrlPeriod = CTRL_CURR_PERIOD;

    smoHandle->a1 = 1.0f - (smoHandle->ctrlPeriod * smoHandle->mtrParam->mtrRs / smoHandle->mtrParam->mtrLd);
    smoHandle->a2 = smoHandle->ctrlPeriod / smoHandle->mtrParam->mtrLd;

    smoHandle->kSmo = FOSMO_GAIN;
    smoHandle->lambda = 2.0f; /* SMO coefficient of cut-off frequency is 2.0. */
    /* smo angle  filcompAngle */
    smoHandle->filCompAngle = FILTER_ANGLE_COMPENSATION;
    FOSMO_Clear(smoHandle);

    smoHandle->emfLpfMinFreq = 2.0f; /* The minimum cutoff frequency of the back EMF filter is 2.0. */
    PLL_Init(&smoHandle->pll);

    /* low pass filter cutoff frequency for speed estimation is 40Hz */
    FoLowPassFilterInit(&smoHandle->spdFilter, smoHandle->ctrlPeriod, SPEED_FILTER_CUTOFF_FREQUENCY);
}

/**
  * @brief Init motor control task.
  * @retval None.
  */
static void TSK_InitCp(void)
{
    IfCtrlInit ifInitParam;

    g_mc.stateMachine = FSM_IDLE;
    g_mc.aptMaxcntCmp = g_apt0.waveform.timerPeriod;
    g_mc.sampleMode = DUAL_RESISTORS;

    ifInitParam.currSlope = USER_CURR_SLOPE;
    ifInitParam.anglePeriod = CTRL_CURR_PERIOD;
    ifInitParam.targetAmp = CTRL_IF_CURR_AMP_A;
    ifInitParam.stepAmpPeriod = CTRL_SYSTICK_PERIOD;

    IF_Init(&g_mc.ifCtrl, &ifInitParam);
    RMG_Init(&g_mc.spdRmg, CTRL_SYSTICK_PERIOD, USER_SPD_SLOPE); /* Init speed slope */
    MtrParamInit(&g_mc.mtrParam, &g_motorParam);
    TimerTickInit(&g_mc);
    SVPWM_Init(&g_mc.sv, INV_VOLTAGE_BUS * ONE_DIV_SQRT3);
    R1SVPWM_Init(&g_mc.r1Sv, INV_VOLTAGE_BUS * ONE_DIV_SQRT3, SAMPLE_POINT_SHIFT, SAMPLE_WINDOW_DUTY);
    g_mc.currCtrlPeriod = CTRL_CURR_PERIOD; /* Init current controller */
    CurrCtrlInit(&g_mc.currCtrl, &g_mc, &g_motorPiParam);
    SpdCtrlInit(&g_mc.spdCtrl, &g_motorPiParam); /* Init speed controller */
    SmoInit(&g_mc.smo, &g_mc.mtrParam); /* Init the SMO observer */
    SMO4TH_Init(&g_mc.smo4th, g_mc.mtrParam.mtrLd, g_mc.mtrParam.mtrLq, g_mc.mtrParam.mtrRs, CTRL_CURR_PERIOD);
    STARTUP_Init(&g_mc.startup, USER_SWITCH_SPDBEGIN_HZ, USER_SWITCH_SPDEND_HZ);
    FwCtrl_Init(&g_mc.fw, CTRL_SYSTICK_PERIOD, 1);
}

/**
  * @brief Clear historical values of all controller before start-up.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void ClearBeforeStartup(MtrCtrlHandle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    /* The initial angle is 0. */
    mtrCtrl->axisAngle = 0;

    mtrCtrl->spdRefHz = 0.0f;
    /* The initial dq-axis reference current is 0. */
    mtrCtrl->currRefDq.d = 0.0f;
    mtrCtrl->currRefDq.q = 0.0f;

    mtrCtrl->vdq.d = 0.0f;
    mtrCtrl->vdq.q = 0.0f;
    /* Clear Duty Cycle Value. The initial duty cycle is 0.5. */
    mtrCtrl->dutyUvwLeft.u = 0.5f;
    mtrCtrl->dutyUvwLeft.v = 0.5f;
    mtrCtrl->dutyUvwLeft.w = 0.5f;
    mtrCtrl->dutyUvwRight.u = 0.5f;
    mtrCtrl->dutyUvwRight.v = 0.5f;
    mtrCtrl->dutyUvwRight.w = 0.5f;

    RMG_Clear(&mtrCtrl->spdRmg); /* Clear the history value of speed slope control */
    CURRCTRL_Clear(&mtrCtrl->currCtrl);
    IF_Clear(&mtrCtrl->ifCtrl);
    SPDCTRL_Clear(&mtrCtrl->spdCtrl);
    FOSMO_Clear(&mtrCtrl->smo);
    STARTUP_Clear(&mtrCtrl->startup);
    R1SVPWM_Clear(&mtrCtrl->r1Sv);
}

/**
  * @brief To set the comparison value of the IGBT single-resistance ADC sampling trigger position.
  * @param aptx The APT register struct handle.
  * @param cntCmpA A Count compare reference of time-base counter.
  * @param cntCmpB B Count compare reference of time-base counter.
  * @param maxCntCmp Maximum Comparison Value
  * @retval None.
  */
static void MCS_SetAdcCompareR1(APT_RegStruct *aptx, unsigned short cntCmpA,
                                unsigned short cntCmpB, unsigned short maxCntCmp)
{
    MCS_ASSERT_PARAM(aptx != NULL);
    MCS_ASSERT_PARAM(maxCntCmp != 0);
    unsigned short tmp;
    /* Sets the A Count compare reference of time-base counter. */
    tmp = (cntCmpA >= maxCntCmp) ? (maxCntCmp - 1) : cntCmpA;
    DCL_APT_SetCounterCompare(aptx, APT_COMPARE_REFERENCE_A, tmp);
    /* Sets the B Count compare reference of time-base counter. */
    tmp = (cntCmpB >= maxCntCmp) ? (maxCntCmp - 1) : cntCmpB;
    DCL_APT_SetCounterCompare(aptx, APT_COMPARE_REFERENCE_B, tmp);
}

/**
  * @brief Open the three-phase lower pipe.
  * @param aptAddr Three-phase APT address pointer.
  * @param maxDutyCnt Max duty count.
  * @retval None.
  */
static void AptTurnOnLowSidePwm(void **aptAddr, unsigned int maxDutyCnt)
{
    MCS_ASSERT_PARAM(aptAddr != NULL);
    MCS_ASSERT_PARAM(maxDutyCnt != 0);
    unsigned int dutyCnt;
    dutyCnt = maxDutyCnt * APT_FULL_DUTY;
    /* Open the three-phase lower pipe */
    for (unsigned int i = 0; i < PHASE_MAX_NUM; i++) {
        APT_RegStruct *aptx = (APT_RegStruct *)(aptAddr[i]);
        aptx->TC_REFC.BIT.rg_cnt_refch = dutyCnt;
        aptx->TC_REFD.BIT.rg_cnt_refdh = dutyCnt;
    }
}

/**
  * @brief Enable three-phase pwm output.
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
static void MotorPwmOutputEnable(void **aptAddr)
{
    MCS_ASSERT_PARAM(aptAddr != NULL);
    /* Enable three-phase pwm output */
    for (unsigned int i = 0; i < PHASE_MAX_NUM; i++) {
        APT_RegStruct *aptx = (APT_RegStruct *)(aptAddr[i]);
        aptx->PG_OUT_FRC.BIT.rg_pga_frc_en = BASE_CFG_UNSET;
        aptx->PG_OUT_FRC.BIT.rg_pgb_frc_en = BASE_CFG_UNSET;
    }
}

/**
  * @brief Disable three-phase pwm output.
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
static void MotorPwmOutputDisable(void **aptAddr)
{
    MCS_ASSERT_PARAM(aptAddr != NULL);
    /* Disable three-phase pwm output. */
    for (unsigned int i = 0; i < PHASE_MAX_NUM; i++) {
        APT_RegStruct *aptx = (APT_RegStruct *)(aptAddr[i]);
        aptx->PG_OUT_FRC.BIT.rg_pga_frc_en = BASE_CFG_SET;
        aptx->PG_OUT_FRC.BIT.rg_pgb_frc_en = BASE_CFG_SET;
        DCL_APT_ForcePWMOutputLow(aptx);
    }
}

/**
  * @brief Smo IF angle difference calculation.
  * @param smoElecAngle Smo electrical angle.
  * @param ifCtrlAngle IF control angle.
  * @retval signed short angle difference.
  */
static signed short SmoIfAngleDiffCalc(signed short smoElecAngle, signed short ifCtrlAngle)
{
    /* Smo IF angle difference calculation */
    signed int tmpS32 = (signed int)(smoElecAngle - ifCtrlAngle);
    if (tmpS32 > INT16_MAX) {
        tmpS32 -= ANGLE_RANGE_ABS;
    }
    if (tmpS32 < INT16_MIN) {
        tmpS32 += ANGLE_RANGE_ABS;
    }
    return (signed short)(tmpS32);
}

/**
  * @brief Construct a new mcs startupswitch object.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void MCS_StartupSwitch(MtrCtrlHandle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    StartupHandle *startup = &mtrCtrl->startup;
    DqAxis *currRefDq = &mtrCtrl->currRefDq;
    float iftargetAmp = mtrCtrl->ifCtrl.targetAmp;
    float spdRefHz = mtrCtrl->spdRefHz;

    switch (startup->stage) {
        case STARTUP_STAGE_CURR:
            if (mtrCtrl->ifCtrl.curAmp >= iftargetAmp) {
                /* Stage change */
                currRefDq->q = iftargetAmp;
                startup->stage = STARTUP_STAGE_SPD;
            } else {
                /* current amplitude increase */
                currRefDq->q = IF_CurrAmpCalc(&mtrCtrl->ifCtrl);
                spdRefHz = 0.0f;
            }
            break;
        case STARTUP_STAGE_SPD:
            /* current frequency increase */
            if (Abs(spdRefHz) >= startup->spdBegin) {
                /* Stage change */
                startup->stage = STARTUP_STAGE_SWITCH;
                TrigVal localTrigVal;
                TrigCalc(&localTrigVal, SmoIfAngleDiffCalc(mtrCtrl->smo.elecAngle, mtrCtrl->ifCtrl.angle));
                currRefDq->d = iftargetAmp * localTrigVal.sin;
                mtrCtrl->startup.initCurr = currRefDq->d;
                currRefDq->q = iftargetAmp;
                mtrCtrl->spdCtrl.spdPi.integral = iftargetAmp * localTrigVal.cos;
            } else {
                /* Speed rmg */
                spdRefHz = RMG_Exec(&mtrCtrl->spdRmg, mtrCtrl->spdCmdHz);
            }
            break;

        case STARTUP_STAGE_SWITCH:
            /* Switch from IF to SMO */
            spdRefHz = RMG_Exec(&mtrCtrl->spdRmg, mtrCtrl->spdCmdHz);
            currRefDq->d = STARTUP_CurrCal(&mtrCtrl->startup, spdRefHz);
            SPDCTRL_Exec(&mtrCtrl->spdCtrl, spdRefHz, mtrCtrl->smo.spdEstHz, &mtrCtrl->currRefDq);
            if (spdRefHz >= startup->spdEnd) {
                /* Stage change */
                currRefDq->d = 0.0f;
                mtrCtrl->stateMachine = FSM_RUN;
            }
            break;

        default:
            break;
    }
    mtrCtrl->spdRefHz = spdRefHz;
}

/**
  * @brief Pre-processing of motor status.
  * @param statusReg System status.
  * @param stateMachine Motor Control Status.
  * @retval None.
  */
static void MotorStatePerProc(SysStatusReg *statusReg, volatile FsmState *stateMachine)
{
    MCS_ASSERT_PARAM(statusReg != NULL);
    MCS_ASSERT_PARAM(stateMachine != NULL);
    /* Get system status */
    if (SysIsError(statusReg)) {
        *stateMachine = FSM_FAULT;
    }
    if (SysGetCmdStop(statusReg)) {
        SysCmdStopClr(statusReg);
        *stateMachine = FSM_STOP;
    }
}

/**
  * @brief Check over current status.
  * @param statusReg System status.
  * @param stateMachine Motor Control Status.
  * @retval None.
  */
static void CheckOverCurrentState(SysStatusReg *statusReg, FsmState *stateMachine)
{
    MCS_ASSERT_PARAM(statusReg != NULL);
    MCS_ASSERT_PARAM(stateMachine != NULL);
    /* check systerm error status */
    if (SysIsError(statusReg) == false) {
        *stateMachine = FSM_IDLE;
    }
}

/**
  * @brief Check bootstrap capacitor charge time.
  * @param mtrCtrl The motor control handle.
  * @param stateMachine Motor Control Status.
  * @retval None.
  */
static void CheckBootstrpCapChargeTime(MtrCtrlHandle *mtrCtrl, FsmState *stateMachine)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(stateMachine != NULL);
    mtrCtrl->sysTickCnt++;
    /* check bootstrap capacitor charge time */
    if (mtrCtrl->sysTickCnt == mtrCtrl->capChargeTickNum) {
        *stateMachine = FSM_CLEAR;
    }
}

/**
  * @brief Check systerm cmd start signal.
  * @param mtrCtrl The motor control handle.
  * @param aptAddr Three-phase APT address pointer.
  * @param statusReg System status.
  * @param stateMachine Motor Control Status.
  * @retval None.
  */
static void CheckSysCmdStart(MtrCtrlHandle *mtrCtrl, void **aptAddr, SysStatusReg *statusReg, FsmState *stateMachine)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(aptAddr != NULL);
    MCS_ASSERT_PARAM(statusReg != NULL);
    MCS_ASSERT_PARAM(stateMachine != NULL);
    /* check start cmd */
    if (SysGetCmdStart(statusReg)) {
        SysRunningSet(statusReg);
        SysCmdStartClr(statusReg);
        mtrCtrl->sysTickCnt = 0;
        *stateMachine = FSM_CAP_CHARGE;
        /* Preparation for charging the bootstrap capacitor. */
        AptTurnOnLowSidePwm(aptAddr, mtrCtrl->aptMaxcntCmp);
        /* Out put pwm */
        MotorPwmOutputEnable(aptAddr);
    }
}

/**
  * @brief System timer tick task.
  * @param mtrCtrl The motor control handle.
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
static void TSK_SystickIsr(MtrCtrlHandle *mtrCtrl, void **aptAddr)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    MCS_ASSERT_PARAM(aptAddr != NULL);
    SysStatusReg *statusReg = &mtrCtrl->statusReg;
    volatile FsmState *stateMachine = &mtrCtrl->stateMachine;
    mtrCtrl->msTickCnt++;
    /* Pre-processing of motor status. */
    MotorStatePerProc(statusReg, stateMachine);
    /* statemachine */
    switch (*stateMachine) {
        case FSM_IDLE:
            CheckSysCmdStart(mtrCtrl, aptAddr, statusReg, stateMachine);
            break;
        case FSM_CAP_CHARGE:
            /* Bootstrap Capacitor Charging Timing */
            CheckBootstrpCapChargeTime(mtrCtrl, stateMachine);
            break;
        case FSM_CLEAR:
            ClearBeforeStartup(mtrCtrl);
            *stateMachine = FSM_STARTUP;
            break;
        case FSM_STARTUP:
            MCS_StartupSwitch(mtrCtrl);
            break;
        case FSM_RUN:
            /* Speed ramp control */
            mtrCtrl->spdRefHz = RMG_Exec(&mtrCtrl->spdRmg, mtrCtrl->spdCmdHz);
            /* speed loop control */
            SPDCTRL_Exec(&mtrCtrl->spdCtrl, mtrCtrl->spdRefHz, mtrCtrl->smo.spdEstHz, &mtrCtrl->currRefDq);
            break;
        case FSM_STOP:
            MotorPwmOutputDisable(aptAddr);
            SysRunningClr(statusReg);
            *stateMachine = FSM_IDLE;
            break;
        case FSM_FAULT: /* Overcurrent state */
            CheckOverCurrentState(statusReg, stateMachine);
            break;
        default:
            break;
    }
}

/**
  * @brief Read the ADC initialize trim value.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void ReadInitCurrUvwCp(MtrCtrlHandle *mtrCtrl)
{
    unsigned int adc0SampleTemp = 0;  /* Current bias value for temp store */
    unsigned int adc1SampleTemp = 0;
    unsigned int adc0TempSum = 0;
    unsigned int adc1TempSum = 0;  /* Current bias sum value for 20 times */
    unsigned int adcSampleTimes = 0;  /* ADC sample times */
     /* ADC Trigger SOCA SOCB */
    g_apt0.adcTrg.trgEnSOCA = BASE_CFG_DISABLE;
    g_apt0.adcTrg.trgEnSOCB = BASE_CFG_DISABLE;
    HAL_APT_PWMInit(&g_apt0);
    SOC_Param socParam = {0};
    socParam.adcInput = ADC_CH_ADCINA0;
    socParam.sampleHoldTime =  2; /* adc sample holed time 2 adc_clk */
    socParam.sampleTotalTime = 3; /* adc sample total time 3 adc_clk */
    socParam.softTrigSource = ADC_TRIGSOC_SOFT;  /* set software trigger as adc trigsource */
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM8, &socParam);
    HAL_ADC_ConfigureSoc(&g_adc1, ADC_SOC_NUM8, &socParam);
    for (int i = 0; i < ADC_READINIT_TIMES; i++) {
        HAL_ADC_SoftTrigSample(&g_adc0, ADC_SOC_NUM8);
        HAL_ADC_SoftTrigSample(&g_adc1, ADC_SOC_NUM8);
        BASE_FUNC_DELAY_US(ADC_INITREAD_DELAY);
        adc0SampleTemp = HAL_ADC_GetConvResult(&g_adc0, ADC_SOC_NUM8);
        adc1SampleTemp = HAL_ADC_GetConvResult(&g_adc1, ADC_SOC_NUM8);
        if (adc0SampleTemp != 0 && adc1SampleTemp != 0) {
            adcSampleTimes++;
            adc0TempSum += adc0SampleTemp;
            adc1TempSum += adc1SampleTemp;
        }
    }
    adc0SampleTemp = (int) (adc0TempSum / adcSampleTimes);
    adc1SampleTemp = (int) (adc1TempSum / adcSampleTimes);
    mtrCtrl->adc0Compensate = (float) adc0SampleTemp;    /* Force convert to float */
    mtrCtrl->adc1Compensate = (float) adc1SampleTemp;
    /* The normal value scope is: 1950.0 < adc0Compensate < 2150.0 */
    if(g_mc.adc0Compensate < ADC_TRIMVALUE_MIN || g_mc.adc0Compensate > ADC_TRIMVALUE_MAX \
       || g_mc.adc1Compensate < ADC_TRIMVALUE_MIN || g_mc.adc1Compensate > ADC_TRIMVALUE_MAX) {
        DBG_PRINTF("ADC trim value error,please reset!");
        HAL_GPIO_SetValue(&g_gpio0, GPIO_PIN_6, GPIO_LOW_LEVEL);
    }
    adcSampleTimes = 0;
    adc0TempSum = 0;
    adc1TempSum = 0;
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT0_SOCA;
    HAL_ADC_ConfigureSoc(&g_adc0, ADC_SOC_NUM8, &socParam);
    socParam.softTrigSource = ADC_TRIGSOC_NONESOFT;
    socParam.periphTrigSource = ADC_TRIGSOC_APT0_SOCA;
    HAL_ADC_ConfigureSoc(&g_adc1, ADC_SOC_NUM8, &socParam);
    /* ADC Trigger SOCA SOCB */
    g_apt0.adcTrg.trgEnSOCA = BASE_CFG_ENABLE;
    g_apt0.adcTrg.trgEnSOCB = BASE_CFG_ENABLE;
    HAL_APT_PWMInit(&g_apt0);
}

/**
  * @brief Read the ADC current sampling value.
  * @param CurrUvw Three-phase current.
  * @retval None.
  */
static void ReadCurrUvwCp(UvwAxis *CurrUvw)
{
    MCS_ASSERT_PARAM(CurrUvw != NULL);
    unsigned int adc0, adc1;
    adc0 = HAL_ADC_GetConvResult(&g_adc0, ADC_SOC_NUM8);
    adc1 = HAL_ADC_GetConvResult(&g_adc1, ADC_SOC_NUM8);
    /* when current is zero, the adc1 value is adc0Compensate/adc1Compensate value */
    CurrUvw->u = -(float)(adc0 - g_mc.adc0Compensate) * ADC_CURR_COFFI_CP;
    CurrUvw->w = -(float)(adc1 - g_mc.adc1Compensate) * ADC_CURR_COFFI_CP;
    CurrUvw->v = -CurrUvw->u - CurrUvw->w;
}

/**
  * @brief Setting the APT Output Duty Cycle.
  * @param aptx APT register base address.
  * @param leftDuty Left duty cycle.
  * @param rightDuty Right duty cycle.
  * @retval None.
  */
static void SetPwmDuty(APT_Handle *aptx, float leftDuty, float rightDuty)
{
    MCS_ASSERT_PARAM(aptx != NULL);
    MCS_ASSERT_PARAM(leftDuty < 0);
    MCS_ASSERT_PARAM(rightDuty < 0);
    unsigned short maxPeriodCnt = aptx->waveform.timerPeriod;
    unsigned short cntCmpLeftEdge = (unsigned short)(leftDuty * maxPeriodCnt);
    unsigned short cntCmpRightEdge = (unsigned short)(rightDuty * maxPeriodCnt);
    /* avoid overflowing */
    cntCmpLeftEdge = (cntCmpLeftEdge > maxPeriodCnt) ? maxPeriodCnt : cntCmpLeftEdge;
    cntCmpRightEdge = (cntCmpRightEdge > maxPeriodCnt) ? maxPeriodCnt : cntCmpRightEdge;
    HAL_APT_SetPWMDuty(aptx, cntCmpLeftEdge, cntCmpRightEdge);
}

/**
  * @brief Duty Cycle Setting.
  * @param dutyUvwLeft Three-phase left duty cycle.
  * @param dutyUvwRight Three-phase right duty cycle.
  * @retval None.
  */
static void SetPwmDutyCp(UvwAxis *dutyUvwLeft, UvwAxis *dutyUvwRight)
{
    MCS_ASSERT_PARAM(dutyUvwLeft != NULL);
    MCS_ASSERT_PARAM(dutyUvwRight != NULL);
    /* Setting the Three-Phase Duty Cycle */
    SetPwmDuty(&g_apt0, dutyUvwLeft->u, dutyUvwRight->u);
    SetPwmDuty(&g_apt1, dutyUvwLeft->v, dutyUvwRight->v);
    SetPwmDuty(&g_apt2, dutyUvwLeft->w, dutyUvwRight->w);
}

/**
  * @brief To set the ADC sampling trigger comparison value.
  * @param cntCmpSOCA Soca Compare Count Value.
  * @param cntCmpSOCB Socb Compare Count Value.
  * @retval None.
  */
static void SetADCTriggerTimeCp(unsigned short cntCmpSOCA, unsigned short cntCmpSOCB)
{
    MCS_SetAdcCompareR1(g_aptCp[PHASE_U], cntCmpSOCA, cntCmpSOCB, g_mc.aptMaxcntCmp);
}

/**
  * @brief System timer ISR for Motor Statemachine CallBack function.
  * @param param The systick timer handle.
  * @retval None.
  */
void MotorStatemachineCallBack(void *param)
{
    MCS_ASSERT_PARAM(param != NULL);
    TIMER_Handle *timer = (TIMER_Handle *)param;
    TIMER_ASSERT_PARAM(timer != NULL);
    TIMER_ASSERT_PARAM(timer->baseAddress != NULL);
    /* Clear timer interrupt. */
    HAL_TIMER_IrqClear(timer);
    TSK_SystickIsr(&g_mc, g_aptCp);
}

/**
  * @brief Check Potentiometer Value callback function.
  * @param param The TIMER_Handle.
  * @retval None.
  */
void CheckPotentiometerValueCallback(void *param)
{
    static unsigned int potentiomitorAdcValue = 0;
    static float spdCmdHz = 0;
    static float spdCmdHzLast = USER_MAX_SPD_HZ;
    MCS_ASSERT_PARAM(param != NULL);
    TIMER_Handle *timer = (TIMER_Handle *)param;
    TIMER_ASSERT_PARAM(timer != NULL);
    TIMER_ASSERT_PARAM(timer->baseAddress != NULL);
    /* Clear timer interrupt. */
    HAL_TIMER_IrqClear(timer);
    HAL_ADC_SoftTrigSample(&g_adc2, ADC_SOC_NUM0);
    potentiomitorAdcValue = HAL_ADC_GetConvResult(&g_adc2, ADC_SOC_NUM0);
    /* 4045.0 is adc sample max value of potentiomitor, make sure max spd 180.25 */
    spdCmdHz = (float)potentiomitorAdcValue / 4045.0 * USER_MAX_SPD_HZ;
    if (Abs(spdCmdHzLast - spdCmdHz) < 1.0) {
        return;
    }
    spdCmdHzLast = spdCmdHz;
    if (spdCmdHz < 35.0) {  /* 35.0 is the cmdHz lower limit */
        spdCmdHz = 35.0;    /* 35.0 is the cmdHz lower limit */
    }
    if (spdCmdHz > 180.25) { /* 180.25 is the cmdHz upper limit */
        spdCmdHz = 180.25;   /* 180.25 is the cmdHz upper limit */
    }
    DBG_PRINTF("speed cmd Hz is %f, potentiomitor value is %d \r\n", spdCmdHz, potentiomitorAdcValue);
    g_mc.spdCmdHz = spdCmdHz;
}

/**
  * @brief The carrier ISR wrapper function.
  * @param aptHandle The APT handle.
  * @retval None.
  */
void MotorCarrierProcessCallback(void *aptHandle)
{
    MCS_ASSERT_PARAM(aptHandle != NULL);
    /* the carrierprocess of motor */
    MCS_CarrierProcess(&g_mc);
    g_mc_u = g_mc.currUvw.u;
    g_mc_v = g_mc.currUvw.v;
    g_mc_w = g_mc.currUvw.w;
    int tmp = g_tracevalue_ref;
    BASE_FUNC_UNUSED(aptHandle);
}

/**
  * @brief Event interrupt callback function of APT module.
  * @param para APT module handle.
  * @retval None.
  */
void MotorSysErrCallback(void *para)
{
    MCS_ASSERT_PARAM(para != NULL);
    APT_Handle *handle = (APT_Handle *)para;
    /* The fan IPM overcurrent triggers and disables the three-phase PWM output. */
    MotorPwmOutputDisable(g_aptCp);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_U], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_V], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_W], APT_OC_COMBINE_EVENT_A1);
    /* Status setting error */
    SysErrorSet(&g_mc.statusReg);
    DBG_PRINTF("APT error! \r\n");
    HAL_GPIO_SetValue(&g_gpio0, GPIO_PIN_6, GPIO_LOW_LEVEL);
    BASE_FUNC_UNUSED(handle);
}


/**
  * @brief Init motor controller's data structure.
  * @retval None.
  */
static void InitSoftware(void)
{
    /* Initializing motor Control Tasks */
    TSK_InitCp();
    /* MCU peripheral configuration function used for initial motor control */
    g_mc.readCurrUvwCb = ReadCurrUvwCp;
    g_mc.setPwmDutyCb = SetPwmDutyCp;
    g_mc.setADCTriggerTimeCb = SetADCTriggerTimeCp;
}

/**
  * @brief Config the master APT.
  * @param aptx The master APT handle.
  * @retval None.
  */
static void AptMasterSet(APT_Handle *aptx)
{
    MCS_ASSERT_PARAM(aptx != NULL);
    /* Config the master APT. */
    HAL_APT_MasterSyncInit(aptx, APT_SYNC_OUT_ON_CNTR_ZERO);
}

/**
  * @brief Config the slave APT.
  * @param aptx The slave APT handle.
  * @retval None.
  */
static void AptSalveSet(APT_Handle *aptx)
{
    MCS_ASSERT_PARAM(aptx != NULL);
    APT_SlaveSyncIn slave;
    /* Config the slave APT. */
    slave.divPhase = 0;
    slave.cntPhase = 0;
    slave.syncCntMode = APT_COUNT_MODE_AFTER_SYNC_UP;
    slave.syncInSrc = APT_SYNC_IN_SRC;
    slave.cntrSyncSrc = APT_CNTR_SYNC_SRC_SYNCIN;
    HAL_APT_SlaveSyncInit(aptx, &slave);
}
/**
  * @brief Configuring Master and Slave APTs.
  * @retval None.
  */
static void AptMasterSalveSet(void)
{
    /* motor fan APT master/slave synchronization */
    AptMasterSet(&g_apt0);
    AptSalveSet(&g_apt1);
    AptSalveSet(&g_apt2);
}

/**
  * @brief Config the KEY func.
  * @param handle The GPIO handle.
  * @retval None.
  */
KEY_State Key_StateRead(GPIO_Handle *handle)
{
    if (HAL_GPIO_GetPinValue(handle, handle->pins) == 0) {
        BASE_FUNC_DELAY_MS(30);  /* delay 30ms for deshake */
        if (HAL_GPIO_GetPinValue(handle, handle->pins) == 0) {
            while (HAL_GPIO_GetPinValue(handle, handle->pins) == 0) {
            }
            return KEY_DOWN;
        }
        return KEY_UP;
    }
}

/**
  * @brief Control motor start and stop state by key func.
  * @param param The GPIO handle.
  * @retval None.
  */
void MotorStartStopKeyCallback(void *param)
{
    static unsigned char motorStateFlag = 0;
    GPIO_Handle *handle = (GPIO_Handle *)param;
    if (Key_StateRead(handle) == KEY_DOWN) {
        motorStateFlag = ~motorStateFlag;
        if (motorStateFlag == 0) { /* stop apt output, motor is off status */
            HAL_APT_StopModule(RUN_APT0 | RUN_APT1 | RUN_APT2);
            g_mc.statusReg.Bit.cmdStop = 1;
            InitSoftware(); /* reset software config */
        } else { /* start apt output, motor is on status */
            HAL_APT_StartModule(RUN_APT0 | RUN_APT1 | RUN_APT2);
            g_mc.statusReg.Bit.cmdStart = 1;
        }
    }
}

/**
  * @brief User application main entry for ECMCU105H board.
  * @retval BSP_OK.
  */
int MotorMainProcess(void)
{
    unsigned int tickNum1Ms = 2; /* 1ms tick */
    static unsigned int tickCnt1Ms = 0;
    unsigned int tickNum500Ms = 1000; /* 500ms tick */
    static unsigned int tickCnt500Ms = 0;

    SystemInit();
    DBG_UartPrintInit(115200); /* debug port baudrate is 115200 */
    HAL_TIMER_Start(&g_timer0);
    HAL_TIMER_Start(&g_timer1);

    AptMasterSalveSet();
    /* Disable PWM output before startup. */
    MotorPwmOutputDisable(g_aptCp);

    /* Software initialization. */
    InitSoftware();

    /* System Timer clock. */
    BASE_FUNC_DELAY_MS(ADC_READINIT_DELAY);
    ReadInitCurrUvwCp(&g_mc);
    BASE_FUNC_DELAY_MS(MOTOR_START_DELAY);

    while (1) {
        if (g_mc.msTickCnt - tickCnt1Ms >= tickNum1Ms) {
            tickCnt1Ms = g_mc.msTickCnt;
            /* User Code 1ms Event */

            /* User Code 1ms Event */
        }

        if (g_mc.msTickCnt - tickCnt500Ms >= tickNum500Ms) {
            if (SysIsError(&g_mc.statusReg) != true) {
               /* The LED blinks when no status is not error. */
                HAL_GPIO_TogglePin(&g_gpio0, GPIO_PIN_7);
            }
            tickCnt500Ms = g_mc.msTickCnt;
        }
    }
    return 0;
}