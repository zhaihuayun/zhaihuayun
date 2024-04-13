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
  * @file      mcs_61h48.c
  * @author    MCU Algorithm Team
  * @brief     This file provides motor application for AD101LDMA board.
  * @details   Single FOC application based on the AD101LDMA VER.A board
  *            1) The motor model is JMC 42JSF630AS-1000.
  *            2) Select the Motorcontrolsystem example in the sample column of chipConfig and click Generate Code.
  *            3) The AD101LDMA VER.A board is a high-voltage board, its power supply must be changed to 24V.
  */
#include "mcs_61h48.h"
#include "mcs_config_61h48.h"
#include "mcs_math_const.h"
#include "mcs_math.h"
#include "mcs_config_motor.h"
#include "mcs_carrier_process.h"

#define MHZ 1000000u

#define SYSTICK_PERIOD_US    500u /* systick period */
/* APT module interrupt priority. */
#define EVT_INTERRUPT_PRIORITY 7
#define TMR_INTERRUPT_PRIORITY 6
#define SYS_TMR_INTERRUPT_PRIORITY IRQ_PRIO_LOWEST
/* Use sync-out pulse from APT_U as the sync-in source for slave APT module. */
#define APT_SYNC_IN_SRC     APT_SYNCIN_SRC_APT0_SYNCOUT

/* Define of OC Event selection */
#define APT_OC_ENABLE BASE_CFG_ENABLE

#define APT_U_CP          APT0_BASE /* Base address of U phase APT module */
#define APT_V_CP          APT1_BASE /* Base address of V phase APT module */
#define APT_W_CP          APT2_BASE /* Base address of W phase APT module */

#define APT_TMR_IRQ       IRQ_APT0_TMR

#define POE_EVENT_PROTECT  0
#define ACMP_EVENT_PROTECT 1

#define POE_ACMP_EVENT_PROTECT  ACMP_EVENT_PROTECT

/**
  *  Output control event polarity for IPM.
  *  APT_EM_EVENT_POLARITY_NOT_INVERT (High active)
  *  APT_EM_EVENT_POLARITY_INVERT (Low active)
  */
#define PROTECT_EVENT_POLARITY_CP   APT_EM_EVENT_POLARITY_INVERT

#define APT_EVT_IRQ_CP              IRQ_APT0_EVT

/* Some configuration values of APT modules. */
#define APT_PWM_FREQ        10000       /* Set PWM frequency to 10KHz. */
#define APT_DUTY_MAX        (HAL_CRG_GetIpFreq(APT_U_CP) / (APT_PWM_FREQ * 2))
#define APT_DIVIDER_FACTOR  1           /* The APT clock is not divided. */
#define DB_US               1.5f        /* Dead-Band time, in units of us */

#define MICROSECOND_NUM_PER_MILLISECOND 1000
#define ANGLE_RANGE_ABS 65536
#define ANGLE_360_F 65536.0f /* 0 - 65536 indicates 0 to 360. */
#define APT_FULL_DUTY 1.0f

void ISR_Systick(void *param);
void ISR_Carrier(void *aptHandle);
void ISR_OverCurrProt(void *aptHandle);

/* Motor parameters. */
static  MotorConfig g_MotorParam[1] = {
    {4, 0.5f, 0.000295f, 0.000295f, 0.0f, 0.0f, 200.0f, 4.0f, 4000, 410}, /* Wildfire 24V */
};

/* Motor PI param. */
static PiCtrlParam g_motorPiParam[1] = {
    /* currKp，ki, limitPu, spdKp, ki, limit */
    {0.7414f, 0.1256f, 1.0f, 0.0105f, 0.000003f, 1.0f}, /* Wildfire 24V */
};

static void *g_aptCp[PHASE_MAX_NUM] = {APT_U_CP, APT_V_CP, APT_W_CP};

/* Motor control handle for compressor */
static MtrCtrlHandle g_mc;

/**
  * @brief Initialzer of system tick.
  * @param mtrCtrl Motor control struct handle.
  * @retval None.
  */
static void TimerTickInit(MtrCtrlHandle *mtrCtrl)
{
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
    currHandle->currRef  = &mtrCtrl->currRefDq;
    currHandle->currFdbk = &mtrCtrl->currDq;
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
    g_mc.aptMaxcntCmp = APT_DUTY_MAX;
    g_mc.sampleMode = SINGLE_RESISTOR;

    ifInitParam.currSlope = USER_CURR_SLOPE;
    ifInitParam.anglePeriod = CTRL_CURR_PERIOD;
    ifInitParam.targetAmp = CTRL_IF_CURR_AMP_A;
    ifInitParam.stepAmpPeriod = CTRL_SYSTICK_PERIOD;

    IF_Init(&g_mc.ifCtrl, &ifInitParam);

    RMG_Init(&g_mc.spdRmg, CTRL_SYSTICK_PERIOD, USER_SPD_SLOPE); /* Init speed slope */

    MtrParamInit(&g_mc.mtrParam, &g_MotorParam[0]);

    TimerTickInit(&g_mc);

    SVPWM_Init(&g_mc.sv, INV_VOLTAGE_BUS * ONE_DIV_SQRT3);

    R1SVPWM_Init(&g_mc.r1Sv, INV_VOLTAGE_BUS * ONE_DIV_SQRT3, SAMPLE_POINT_SHIFT, SAMPLE_WINDOW_DUTY);

    g_mc.currCtrlPeriod = CTRL_CURR_PERIOD; /* Init current controller */

    CurrCtrlInit(&g_mc.currCtrl, &g_mc, &g_motorPiParam[0]);

    SpdCtrlInit(&g_mc.spdCtrl, &g_motorPiParam[0]); /* Init speed controller */

    SmoInit(&g_mc.smo, &g_mc.mtrParam); /* Init the SMO observer */

    STARTUP_Init(&g_mc.startup, USER_SWITCH_SPDBEGIN_HZ, USER_SWITCH_SPDEND_HZ);
}

/**
  * @brief Clear historical values of all controller before start-up.
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void ClearBeforeStartup(MtrCtrlHandle *mtrCtrl)
{
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
  * @brief To set the comparison value of the IGBT single-resistance ADC sampling trigger position
  * @param aptx The APT register struct handle.
  * @param cntCmpA A Count compare reference of time-base counter.
  * @param cntCmpB B Count compare reference of time-base counter.
  * @param maxCntCmp Maximum Comparison Value
  * @retval None.
  */
static void MCS_SetAdcCompareR1(APT_RegStruct *aptx, unsigned short cntCmpA,
                                unsigned short cntCmpB, unsigned short maxCntCmp)
{
    unsigned short tmp;
    /* Sets the A Count compare reference of time-base counter. */
    tmp = (cntCmpA >= maxCntCmp) ? (maxCntCmp - 1) : cntCmpA;
    DCL_APT_SetCounterCompare(aptx, APT_COMPARE_REFERENCE_A, tmp);
    /* Sets the B Count compare reference of time-base counter. */
    tmp = (cntCmpB >= maxCntCmp) ? (maxCntCmp - 1) : cntCmpB;
    DCL_APT_SetCounterCompare(aptx, APT_COMPARE_REFERENCE_B, tmp);
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
    unsigned short maxPeriodCnt = aptx->waveform.timerPeriod;
    unsigned short cntCmpLeftEdge = (unsigned short)(leftDuty * maxPeriodCnt);
    unsigned short cntCmpRightEdge = (unsigned short)(rightDuty * maxPeriodCnt);
    /* avoid overflowing */
    cntCmpLeftEdge = (cntCmpLeftEdge > maxPeriodCnt) ? maxPeriodCnt : cntCmpLeftEdge;
    cntCmpRightEdge = (cntCmpRightEdge > maxPeriodCnt) ? maxPeriodCnt : cntCmpRightEdge;
    HAL_APT_SetPWMDuty(aptx, cntCmpLeftEdge, cntCmpRightEdge);
}

/**
  * @brief Open the three-phase lower pipe.
  * @param aptAddr Three-phase APT address pointer.
  * @param maxDutyCnt Max duty count.
  * @retval None.
  */
static void AptTurnOnLowSidePwm(void **aptAddr, unsigned int maxDutyCnt)
{
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
  * @brief Construct a new mcs startupswitch object
  * @param mtrCtrl The motor control handle.
  * @retval None.
  */
static void MCS_StartupSwitch(MtrCtrlHandle *mtrCtrl)
{
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
                /* current Amplitude increase */
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
  * @brief Actiong for FSM_CAP_CHARGE status.
  * @param mtrCtrl The motor control handle.
  * @param stateMachine Motor Control Status.
  * @retval None.
  */
static void MCS_CAPChage(MtrCtrlHandle *mtrCtrl, FsmState *stateMachine)
{
    mtrCtrl->sysTickCnt++;
    if (mtrCtrl->sysTickCnt == mtrCtrl->capChargeTickNum) {
        *stateMachine = FSM_CLEAR;
    }
}

/**
  * @brief Actiong for FSM_FAlT status.
  * @param statusReg System status.
  * @param stateMachine Motor Control Status.
  * @retval None.
  */
static void MCS_Fault(SysStatusReg *statusReg, FsmState *stateMachine)
{
    if (SysIsError(statusReg) == false) {
        *stateMachine = FSM_IDLE;
    }
}

/**
  * @brief System timer tick task.+
  * @param mtrCtrl The motor control handle.
  * @param aptAddr Three-phase APT address pointer.
  * @retval None.
  */
static void TSK_SystickIsr(MtrCtrlHandle *mtrCtrl, void **aptAddr)
{
    SysStatusReg *statusReg = &mtrCtrl->statusReg;
    volatile FsmState *stateMachine = &mtrCtrl->stateMachine;
    mtrCtrl->msTickCnt++;
    /* Pre-processing of motor status */
    MotorStatePerProc(statusReg, stateMachine);
    /* statemachine */
    switch (*stateMachine) {
        case FSM_IDLE:
            if (SysGetCmdStart(statusReg)) {
                SysRunningSet(statusReg);
                SysCmdStartClr(statusReg);
                mtrCtrl->sysTickCnt = 0;
                *stateMachine = FSM_CAP_CHARGE;
                /* Preparation for charging the bootstrap capacitor */
                AptTurnOnLowSidePwm(aptAddr, mtrCtrl->aptMaxcntCmp);
                /* Out put pwm */
                MotorPwmOutputEnable(aptAddr);
            }
            break;

        case FSM_CAP_CHARGE:
            /* Bootstrap Capacitor Charging Timing */
            MCS_CAPChage(mtrCtrl, stateMachine);
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
            MCS_Fault(statusReg, stateMachine);
            break;
        default:
            break;
    }
}
/**
  * @brief Obtaining the Three-Phase Current of the Compressor.
  * @param CurrUvw Three-phase current data return pointer.
  */
static void ReadCurrUvwCp(UvwAxis *CurrUvw)
{
    float iBusSocA, iBusSocB;
    iBusSocA = (float)(HAL_ADC_GetConvResult(&g_adc, ADC_SOC_NUM8) - 2086.0f) * ADC_CURR_COFFI_CP;
    iBusSocB = (float)(HAL_ADC_GetConvResult(&g_adc, ADC_SOC_NUM9) - 2086.0f) * ADC_CURR_COFFI_CP;
    R1CurrReconstruct(g_mc.r1Sv.voltIndexLast, iBusSocA, iBusSocB, CurrUvw);
}

/**
  * @brief Compressor Duty Cycle Setting.
  * @param dutyUvwLeft Three-phase left duty cycle.
  * @param dutyUvwRight Three-phase right duty cycle.
  * @retval None.
  */
static void SetPwmDutyCp(UvwAxis *dutyUvwLeft, UvwAxis *dutyUvwRight)
{
    SetPwmDuty(&g_aptUcp, dutyUvwLeft->u, dutyUvwRight->u);
    SetPwmDuty(&g_aptVcp, dutyUvwLeft->v, dutyUvwRight->v);
    SetPwmDuty(&g_aptWcp, dutyUvwLeft->w, dutyUvwRight->w);
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
  * @brief System timer ISR function.
  * @param param The systick timer handle.
  * @retval None.
  */
void ISR_Systick(void *param)
{
    TIMER_Handle *timer = (TIMER_Handle *)param;
    TIMER_ASSERT_PARAM(timer != NULL);
    TIMER_ASSERT_PARAM(timer->baseAddress != NULL);
    /* Clear timer interrupt. */
    HAL_TIMER_IrqClear(timer);
    TSK_SystickIsr(&g_mc, g_aptCp);
}


volatile float g_mc_u, g_mc_v, g_mc_w;
volatile int g_tracevalue_ref = 1;
/**
  * @brief Timer interrupt callback function of U phase APT module.
  * @param aptHandle APT module handle.
  * @retval None.
  */
void ISR_Carrier(void *aptHandle)
{
    /* the carrierprocess */
    MCS_CarrierProcess(&g_mc);
    g_mc_u = g_mc.currUvw.u;
    g_mc_v = g_mc.currUvw.v;
    g_mc_w = g_mc.currUvw.w;
    int tmp = g_tracevalue_ref;
    BASE_FUNC_UNUSED(aptHandle);
}

/**
  * @brief Event interrupt callback function of APT module.
  * @param aptHandle APT module handle.
  * @retval None.
  */
void ISR_OverCurrProt(void *aptHandle)
{
    APT_Handle *apt = (APT_Handle *)aptHandle;
    if (apt->irqNumEvt == APT_EVT_IRQ_CP) {
        /* The IPM overcurrent triggers and disables the three-phase PWM output. */
        MotorPwmOutputDisable(g_aptCp);
        DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_U], APT_OC_COMBINE_EVENT_A1);
        DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_V], APT_OC_COMBINE_EVENT_A1);
        DCL_APT_ClearOutCtrlEventFlag((APT_RegStruct *)g_aptCp[PHASE_W], APT_OC_COMBINE_EVENT_A1);
        /* Status setting error */
        SysErrorSet(&g_mc.statusReg);
        DBG_PRINTF("APT_EVT_IRQ\r\n");
    } else {
        DBG_PRINTF("ISR_OverCurrProt\r\n");
    }
    HAL_GPIO_TogglePin(&g_Led, GPIO_PIN_6);
}

/**
  * @brief Init motor controller's data structure.
  * @retval None.
  */
static void InitSoftware(void)
{
    /* Initializing Compressor and Fan Control Tasks */
    TSK_InitCp();
    /* MCU peripheral configuration function used for initial motor control */
    g_mc.readCurrUvwCb = ReadCurrUvwCp;
    g_mc.setPwmDutyCb = SetPwmDutyCp;
    g_mc.setADCTriggerTimeCb = SetADCTriggerTimeCp;
}

/**
  * @brief Switch motor running status.
  * @param mtrCtrl The motor control handle.
  */
static void GetExternalParam(MtrCtrlHandle *mtrCtrl)
{
    BASE_FUNC_UNUSED(mtrCtrl->spdCmdHz); /* Externally controlled motor speed, Control motor speed */
}

/**
  * @brief Initial configuration of the underlying driver.
  * @retval None.
  */
__weak void MCS_SystemInit(void)
{
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
    AptMasterSet(&g_aptUcp);
    AptSalveSet(&g_aptVcp);
    AptSalveSet(&g_aptWcp);
}

/**
  * @brief User application entry.
  * @retval BSP_OK.
  */
int Main61H48(void)
{
    SystemInit();
    AptMasterSalveSet();
    HAL_TIMER_Start(&g_sysTickTimer);
    /* Disable PWM output before startup. */
    MotorPwmOutputDisable(g_aptCp);
    /* Software initialization. */
    InitSoftware();
    /* Start the PWM clock. */
    HAL_APT_StartModule(RUN_APT0 | RUN_APT1 | RUN_APT2);

    unsigned int tickNum_1ms = 2; /* 1ms tick */
    static unsigned int tickCnt_1ms = 0;

    unsigned int tickNum_500ms = 1000; /* 500ms tick */
    static unsigned int tickCnt_500ms = 0;

    BASE_FUNC_DELAY_S(MOTOR_START_DELAY);
    SysCmdStartSet(&g_mc.statusReg); /* start motor */

    while (1) {
        if (g_mc.msTickCnt - tickCnt_1ms >= tickNum_1ms) {
            tickCnt_1ms = g_mc.msTickCnt;
            if (g_mc.stateMachine == FSM_RUN ||
                g_mc.stateMachine == FSM_STARTUP) {
            }
        }

        if (g_mc.msTickCnt - tickCnt_500ms >= tickNum_500ms) {
            if (SysIsError(&g_mc.statusReg) != true) {
                /* The LED blinks when no status is not error. */
                HAL_GPIO_TogglePin(&g_Led, G_LED_PIN);
            }
            tickCnt_500ms = g_mc.msTickCnt;
        }
    }
    return 0;
}