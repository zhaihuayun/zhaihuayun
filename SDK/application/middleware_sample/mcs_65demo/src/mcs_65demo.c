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
  * @file      mcs_65demo.c
  * @author    MCU Algorithm Team
  * @brief     This file provides motor application for AD105HDMA board.
  * @details   Dual FOC application based on the AD105HDMA VER.A board
  *            1) Both motor model is JMC 42JSF630AS-1000.
  *            2) Select the Motorcontrolsystem example in the sample column of chipConfig and click Generate Code.
  *            3) The AD105HDMA VER.A board is a high-voltage board, its power supply must be changed to 24V.
  */
#include "mcs_65demo.h"
#include "mcs_config_65demo.h"
#include "mcs_math_const.h"
#include "mcs_math.h"
#include "mcs_config_motor.h"
#include "mcs_carrier_process.h"

#define MICROSECOND_NUM_PER_MILLISECOND 1000
#define ANGLE_RANGE_ABS 65536
#define ANGLE_360_F 65536.0f /* 0 - 65536 indicates 0 to 360. */
#define APT_FULL_DUTY 1.0f

/* Motor parameters. */
static MotorConfig g_MotorParam[MOTOR_MAX_NUM] = {
    {4, 0.5f, 0.000295f, 0.000295f, 0.0f, 0.0f, 200.0f, 4.0f, 4000, 410}, /* Wildfire 24V */
    {4, 0.5f, 0.000295f, 0.000295f, 0.0f, 0.0f, 200.0f, 4.0f, 4000, 410}  /* Wildfire 24V */
};

/* Motor PI param. */
static PiCtrlParam g_motorPiParam[MOTOR_MAX_NUM] = {
    /* currKp，ki, limitPu, spdKp, ki, limit */
    {0.7414f, 0.1256f, 1.0f, 0.0105f, 0.000003f, 1.0f}, /* Wildfire 24V */
    {0.7414f, 0.1256f, 1.0f, 0.0105f, 0.000003f, 1.0f}  /* Wildfire 24V */
};

static void *g_aptCp[PHASE_MAX_NUM] = {APT_U_CP, APT_V_CP, APT_W_CP};
static void *g_aptFan[PHASE_MAX_NUM] = {APT_U_FAN, APT_V_FAN, APT_W_FAN};

/* Motor control handle for compressor */
static MtrCtrlHandle g_mc;

/* Motor control handle for fan */
static MtrCtrlHandle g_fan;

/**
  * @brief Initialzer of system tick.
  * @param mtrCtrl Motor control struct handle.
  * @retval None.
  */
static void TimerTickInit(MtrCtrlHandle *mtrCtrl)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    mtrCtrl->sysTickCnt = 0;
    mtrCtrl->msTickNum = MICROSECOND_NUM_PER_MILLISECOND / SYSTICK_PERIOD_US;
    mtrCtrl->capChargeTickNum = (INV_CAP_CHARGE_MS * MICROSECOND_NUM_PER_MILLISECOND / SYSTICK_PERIOD_US);
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
    float we = 300.0f; /* PLL bandwidth (unit: Hz) */
    /* Reset PLL PID. */
    PID_Reset(&pllHandle->pi);
    /* Initializing PLL Parameters. */
    pllHandle->ctrlPeriod = CTRL_CURR_PERIOD;
    pllHandle->pi.kp = 1.414f * we; /* sqrt(2) * we */
    pllHandle->pi.ki = we * we * CTRL_CURR_PERIOD;
    pllHandle->pi.upperLimit = 5000.0f; /* The upper limit value of the pid comp output. */
    pllHandle->pi.lowerLimit = -pllHandle->pi.upperLimit;
    pllHandle->minAmp = 0.1f; /* Minimum value of the input value in case of the divergence of the PLL. */
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
    g_mc.spdCmdHz = USER_TARGET_SPD_HZ;
    g_mc.aptMaxcntCmp = g_apt3.waveform.timerPeriod;
    g_mc.sampleMode = SINGLE_RESISTOR;

    ifInitParam.currSlope = USER_CURR_SLOPE;
    ifInitParam.anglePeriod = CTRL_CURR_PERIOD;
    ifInitParam.targetAmp = CTRL_IF_CURR_AMP_A;
    ifInitParam.stepAmpPeriod = CTRL_SYSTICK_PERIOD;

    IF_Init(&g_mc.ifCtrl, &ifInitParam);

    RMG_Init(&g_mc.spdRmg, CTRL_SYSTICK_PERIOD, USER_SPD_SLOPE); /* Init speed slope */

    MtrParamInit(&g_mc.mtrParam, &g_MotorParam[MOTOR_NO0]);

    TimerTickInit(&g_mc);

    SVPWM_Init(&g_mc.sv, INV_VOLTAGE_BUS * ONE_DIV_SQRT3);

    R1SVPWM_Init(&g_mc.r1Sv, INV_VOLTAGE_BUS * ONE_DIV_SQRT3, SAMPLE_POINT_SHIFT, SAMPLE_WINDOW_DUTY);

    g_mc.currCtrlPeriod = CTRL_CURR_PERIOD; /* Init current controller */
    CurrCtrlInit(&g_mc.currCtrl, &g_mc, &g_motorPiParam[MOTOR_NO0]);

    SpdCtrlInit(&g_mc.spdCtrl, &g_motorPiParam[MOTOR_NO0]); /* Init speed controller */

    SmoInit(&g_mc.smo, &g_mc.mtrParam); /* Init the SMO observer */

    STARTUP_Init(&g_mc.startup, USER_SWITCH_SPDBEGIN_HZ, USER_SWITCH_SPDEND_HZ);
}

/**
  * @brief Init motor control task.
  * @retval None.
  */
static void TSK_InitFan(void)
{
    IfCtrlInit ifInitParam;

    g_fan.stateMachine = FSM_IDLE;
    g_fan.spdCmdHz = USER_TARGET_SPD_HZ_FAN;
    g_fan.aptMaxcntCmp = g_apt0.waveform.timerPeriod;
    g_fan.sampleMode = SINGLE_RESISTOR;

    ifInitParam.anglePeriod = CTRL_CURR_PERIOD;
    ifInitParam.currSlope = USER_CURR_SLOPE_FAN;
    ifInitParam.stepAmpPeriod = CTRL_SYSTICK_PERIOD;
    ifInitParam.targetAmp = CTRL_IF_CURR_AMP_A_FAN;
    IF_Init(&g_fan.ifCtrl, &ifInitParam);

    RMG_Init(&g_fan.spdRmg, CTRL_SYSTICK_PERIOD, USER_SPD_SLOPE_FAN); /* Init speed slope */

    MtrParamInit(&g_fan.mtrParam, &g_MotorParam[MOTOR_NO1]);

    TimerTickInit(&g_fan);

    R1SVPWM_Init(&g_fan.r1Sv, INV_VOLTAGE_BUS * ONE_DIV_SQRT3, SAMPLE_POINT_SHIFT, SAMPLE_WINDOW_DUTY);

    g_fan.currCtrlPeriod = CTRL_CURR_PERIOD; /* Init current controller */

    CurrCtrlInit(&g_fan.currCtrl, &g_fan, &g_motorPiParam[MOTOR_NO1]);

    SpdCtrlInit(&g_fan.spdCtrl, &g_motorPiParam[MOTOR_NO1]); /* Init speed controller */

    SmoInit(&g_fan.smo, &g_fan.mtrParam); /* Init the SMO observer */

    STARTUP_Init(&g_fan.startup, USER_SWITCH_SPDBEGIN_HZ_FAN, USER_SWITCH_SPDEND_HZ_FAN);
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
            break;

        case FSM_CAP_CHARGE:
            /* Bootstrap Capacitor Charging Timing */
            mtrCtrl->sysTickCnt++;
            if (mtrCtrl->sysTickCnt == mtrCtrl->capChargeTickNum) {
                *stateMachine = FSM_CLEAR;
            }
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
            if (SysIsError(statusReg) == false) {
                *stateMachine = FSM_IDLE;
            }
            break;
        default:
            break;
    }
}

/**
  * @brief Read the ADC current sampling value of the compressor.
  * @param CurrUvw Three-phase current.
  * @retval None.
  */
static void ReadCurrUvwCp(UvwAxis *CurrUvw)
{
    MCS_ASSERT_PARAM(CurrUvw != NULL);
    float iBusSocA, iBusSocB;

    /* Zero sampling value of hardware circuit is 2104.0f. */
    iBusSocA = (float)(HAL_ADC_GetConvResult(&g_adc2, ADC_SOC_NUM8) - 2104.0f) * ADC_CURR_COFFI_CP;
    iBusSocB = (float)(HAL_ADC_GetConvResult(&g_adc2, ADC_SOC_NUM9) - 2104.0f) * ADC_CURR_COFFI_CP;
    R1CurrReconstruct(g_mc.r1Sv.voltIndexLast, iBusSocA, iBusSocB, CurrUvw);
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
  * @brief Compressor Duty Cycle Setting.
  * @param dutyUvwLeft Three-phase left duty cycle.
  * @param dutyUvwRight Three-phase right duty cycle.
  * @retval None.
  */
static void SetPwmDutyCp(UvwAxis *dutyUvwLeft, UvwAxis *dutyUvwRight)
{
    MCS_ASSERT_PARAM(dutyUvwLeft != NULL);
    MCS_ASSERT_PARAM(dutyUvwRight != NULL);
    /* Setting the Three-Phase Duty Cycle */
    SetPwmDuty(&g_apt3, dutyUvwLeft->u, dutyUvwRight->u);
    SetPwmDuty(&g_apt4, dutyUvwLeft->v, dutyUvwRight->v);
    SetPwmDuty(&g_apt5, dutyUvwLeft->w, dutyUvwRight->w);
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
  * @brief Obtaining the Three-Phase Current of the Fan.
  * @param CurrUvw Three-phase current data return pointer.
  * @retval None.
  */
static void ReadCurrUvwFan(UvwAxis *CurrUvw)
{
    MCS_ASSERT_PARAM(CurrUvw != NULL);
    float iBusSocA, iBusSocB;

    /* Zero sampling value of hardware circuit is 2076.0f. */
    iBusSocA = (float)(HAL_ADC_GetConvResult(&g_adc0, ADC_SOC_NUM0) - 2076.0f) * ADC_CURR_COFFI_FAN;
    iBusSocB = (float)(HAL_ADC_GetConvResult(&g_adc0, ADC_SOC_NUM1) - 2076.0f) * ADC_CURR_COFFI_FAN;
    R1CurrReconstruct(g_fan.r1Sv.voltIndexLast, iBusSocA, iBusSocB, CurrUvw);
}

/**
  * @brief Fan Duty Cycle Setting.
  * @param dutyUvwLeft Three-phase left duty cycle.
  * @param dutyUvwRight Three-phase right duty cycle.
  * @retval None.
  */
static void SetPwmDutyFan(UvwAxis *dutyUvwLeft, UvwAxis *dutyUvwRight)
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
static void SetADCTriggerTimeFan(unsigned short cntCmpSOCA, unsigned short cntCmpSOCB)
{
    MCS_SetAdcCompareR1(g_aptFan[PHASE_U], cntCmpSOCA, cntCmpSOCB, g_fan.aptMaxcntCmp);
}

/**
  * @brief System timer ISR function.
  * @param param The systick timer handle.
  * @retval None.
  */
void Timer1ITCallBack(void *param)
{
    MCS_ASSERT_PARAM(param != NULL);
    TIMER_Handle *timer = (TIMER_Handle *)param;
    TIMER_ASSERT_PARAM(timer != NULL);
    TIMER_ASSERT_PARAM(timer->baseAddress != NULL);
    /* Clear timer interrupt. */
    HAL_TIMER_IrqClear(timer);
    TSK_SystickIsr(&g_mc, g_aptCp);
    TSK_SystickIsr(&g_fan, g_aptFan);
}

/* global variables for variable trace */
volatile float g_mc_u, g_mc_v, g_mc_w;
volatile float g_fan_u, g_fan_v, g_fan_w;
volatile int g_tracevalue_ref = 1;
/**
  * @brief The carrier ISR wrapper function, entry for both compressor and fan.
  * @param aptHandle The APT handle.
  * @retval None.
  */
void APT3TimerCallback(void *aptHandle)
{
    MCS_ASSERT_PARAM(aptHandle != NULL);
    /* the carrierprocess of comp */
    MCS_CarrierProcess(&g_mc);
    g_mc_u = g_mc.currUvw.u;
    g_mc_v = g_mc.currUvw.v;
    g_mc_w = g_mc.currUvw.w;
    
    /* the carrierprocess of fan */
    MCS_CarrierProcess(&g_fan);
    g_fan_u = g_fan.currUvw.u;
    g_fan_v = g_fan.currUvw.v;
    g_fan_w = g_fan.currUvw.w;

    int tmp = g_tracevalue_ref;
 
    BASE_FUNC_UNUSED(aptHandle);
}

/**
  * @brief Event interrupt callback function of APT module.
  * @param para APT module handle.
  * @retval None.
  */
void APT0EventCallback(void *para)
{
    MCS_ASSERT_PARAM(para != NULL);
    APT_Handle *handle = (APT_Handle *)para;
    /* The fan IPM overcurrent triggers and disables the three-phase PWM output. */
    MotorPwmOutputDisable(g_aptFan);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptFan[PHASE_U], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptFan[PHASE_V], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptFan[PHASE_W], APT_OC_COMBINE_EVENT_A1);
    /* Status setting error */
    SysErrorSet(&g_fan.statusReg);
    DBG_PRINTF("APT_EVT_IRQ_FAN\r\n");
    BASE_FUNC_UNUSED(handle);
}

/**
  * @brief Event interrupt callback function of APT module.
  * @param para APT module handle.
  * @retval None.
  */
void APT3EventCallback(void *para)
{
    MCS_ASSERT_PARAM(para != NULL);
    APT_Handle *handle = (APT_Handle *)para;
    /* The compressor IPM overcurrent triggers and disables the three-phase PWM output. */
    MotorPwmOutputDisable(g_aptCp);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_U], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_V], APT_OC_COMBINE_EVENT_A1);
    DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_W], APT_OC_COMBINE_EVENT_A1);
    /* Status setting error */
    SysErrorSet(&g_mc.statusReg);
    DBG_PRINTF("APT_EVT_IRQ_CP\r\n");
    BASE_FUNC_UNUSED(handle);
}

/**
  * @brief Disable PWM output when IPM temperature too high.
  * @retval None.
  */
static void OverTempProtProcess(void)
{
    float tempV;

    HAL_ADC_SoftTrigSample(&g_adc2, ADC_SOC_NUM7);
    tempV = HAL_ADC_GetConvResult(&g_adc2, ADC_SOC_NUM7) * IPM_VOLT_COEFFI;
    /* IPM voltage at 90°C */
    if (tempV >= IPM_90DEGRESS_V) {
        MotorPwmOutputDisable(g_aptCp);
    }
}

/**
  * @brief Init motor controller's data structure.
  * @retval None.
  */
static void InitSoftware(void)
{
    /* Initializing Compressor and Fan Control Tasks */
    TSK_InitCp();
    TSK_InitFan();

    /* MCU peripheral configuration function used for initial motor control */
    g_mc.readCurrUvwCb = ReadCurrUvwCp;
    g_mc.setPwmDutyCb = SetPwmDutyCp;
    g_mc.setADCTriggerTimeCb = SetADCTriggerTimeCp;

    g_fan.readCurrUvwCb = ReadCurrUvwFan;
    g_fan.setPwmDutyCb = SetPwmDutyFan;
    g_fan.setADCTriggerTimeCb = SetADCTriggerTimeFan;
}

/**
  * @brief Confit the master APT.
  * @param aptx The master APT handle.
  * @retval None.
  */
static void AptMasterSet(APT_Handle *aptx)
{
    MCS_ASSERT_PARAM(aptx != NULL);
    /* Confit the master APT. */
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
    /* Compressor fan APT master/slave synchronization */
    AptMasterSet(&g_apt3);
    AptSalveSet(&g_apt4);
    AptSalveSet(&g_apt5);
    AptSalveSet(&g_apt0);
    AptSalveSet(&g_apt1);
    AptSalveSet(&g_apt2);
}

/**
  * @brief Switch motor running status.
  * @param mtrCtrl The motor control handle.
  * @param motorNum The motor ID number.
  */
static void GetExternalParam(MtrCtrlHandle *mtrCtrl, MOTOR_ENUM motorNum)
{
    MCS_ASSERT_PARAM(mtrCtrl != NULL);
    BASE_FUNC_UNUSED(mtrCtrl->spdCmdHz); /* Externally controlled motor speed, Control motor speed */
    BASE_FUNC_UNUSED(motorNum); /* Current motor number */
}

/**
  * @brief Initial configuration of the underlying driver.
  * @retval None.
  */
__weak void SystemInit(void)
{
}

/**
  * @brief User application main entry for 65demo board.
  * @retval BSP_OK.
  */
int Main65Demo(void)
{
    unsigned int tickNum1Ms = 2; /* 1ms tick */
    static unsigned int tickCnt1Ms = 0;

    unsigned int tickNum500Ms = 1000; /* 500ms tick */
    static unsigned int tickCnt500Ms = 0;

    SystemInit();

    HAL_TIMER_Start(&g_timer1);

    AptMasterSalveSet();
    /* Disable PWM output before startup. */
    MotorPwmOutputDisable(g_aptCp);
    MotorPwmOutputDisable(g_aptFan);

    HAL_CRG_IpEnableSet(UART0_BASE, BASE_CFG_ENABLE);
    DBG_UartPrintInit(BAUDRATE);
    /* Software initialization. */
    InitSoftware();
    /* Start the PWM clock. */
    HAL_APT_StartModule(RUN_APT0 | RUN_APT1 | RUN_APT2 | RUN_APT3 | RUN_APT4 | RUN_APT5);

    BASE_FUNC_DELAY_S(PTC_RELAY_DELAY);
    /* Open PTC relay */
    HAL_GPIO_SetValue(&g_gpio2, GPIO_PIN_0, GPIO_HIGH_LEVEL);

    BASE_FUNC_DELAY_S(MOTOR_START_DELAY);
    /* Starting motor. */
    SysCmdStartSet(&g_mc.statusReg);
    SysCmdStartSet(&g_fan.statusReg);

    while (1) {
        if (g_mc.msTickCnt - tickCnt1Ms >= tickNum1Ms) {
            GetExternalParam(&g_mc, MOTOR_NO0);
            GetExternalParam(&g_fan, MOTOR_NO1);
            tickCnt1Ms = g_mc.msTickCnt;
            if (g_mc.stateMachine == FSM_RUN ||
                g_mc.stateMachine == FSM_STARTUP) {
                OverTempProtProcess();
            }
        }

        if (g_mc.msTickCnt - tickCnt500Ms >= tickNum500Ms) {
            if (SysIsError(&g_mc.statusReg) != true && SysIsError(&g_fan.statusReg) != true) {
               /* The LED blinks when no status is not error. */
                HAL_GPIO_TogglePin(&g_gpio0, GPIO_PIN_5);
            }
            tickCnt500Ms = g_mc.msTickCnt;
        }
    }
    return 0;
}