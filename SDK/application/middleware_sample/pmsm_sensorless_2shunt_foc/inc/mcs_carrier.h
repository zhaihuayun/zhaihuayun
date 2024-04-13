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
  * @file     mcs_carrier.h
  * @author   MCU Algorithm Team
  * @brief    This file provides functions declaration for carrier interrupt processing function.
  */
#ifndef McuMagicTag_MCS_CARRIER_H
#define McuMagicTag_MCS_CARRIER_H

#include "mcs_status.h"
#include "mcs_mtr_param.h"
#include "mcs_svpwm.h"
#include "mcs_curr_ctrl.h"
#include "mcs_if_ctrl.h"
#include "mcs_ramp_mgmt.h"
#include "mcs_spd_ctrl.h"
#include "mcs_fosmo.h"
#include "mcs_smo_4th.h"
#include "mcs_pll.h"
#include "mcs_startup.h"
#include "mcs_r1_svpwm.h"
#include "mcs_fw_ctrl.h"

typedef void (*MCS_ReadCurrUvwCb)(UvwAxis *CurrUvw);
typedef void (*MCS_SetPwmDutyCb)(UvwAxis *dutyUvwLeft, UvwAxis *dutyUvwRight);
typedef void (*MCS_SetADCTriggerTimeCb)(unsigned short cntCmpSOCA, unsigned short cntCmpSOCB);

/**
  * @brief motor control FSM state define.
  * @details motor control FSM state type:
  *          + FSM_IDLE         -- IDLE state, system startup default.
  *          + FSM_OFFSET_CALIB -- Self calibrate, for ADC init.
  *          + FSM_CAP_CHARGE   -- IPM cap charge.
  *          + FSM_CLEAR        -- Clear before every run.
  *          + FSM_LOCATE       -- Rotor position pre-locate.
  *          + FSM_STARTUP      -- Start up.
  *          + FSM_SWITCH       -- Transition state, control from open to closed loop.
  *          + FSM_RUN          -- Normal running state.
  *          + FSM_WAIT_STOP    -- Wait stopping.
  *          + FSM_STOP         -- Normal stop.
  *          + FSM_FAULT        -- Fault state, waiting for user process.
  */
typedef enum {
    FSM_IDLE = 0,
    FSM_OFFSET_CALIB,
    FSM_CAP_CHARGE,
    FSM_CLEAR,
    FSM_LOCATE,
    FSM_STARTUP,
    FSM_SWITCH,
    FSM_RUN,
    FSM_WAIT_STOP,
    FSM_STOP,
    FSM_FAULT
} FsmState;

/**
  * @brief Sampling mode.
  */
typedef enum {
    DUAL_RESISTORS = 0,
    SINGLE_RESISTOR = 1
} SampleMode;

/**
  * @brief Motor control data structure
  */
typedef struct {
    float spdCmdHz;     /**< External input speed command value */
    short axisAngle;    /**< Angle of the synchronous coordinate system, used for coordinate transformation */
    float spdRefHz;     /**< Command value after speed ramp management */
    float currCtrlPeriod;   /**< current loop control period */
    float adc0Compensate;  /**< ADC0 softwaretrim compensate value */
    float adc1Compensate;  /**< ADC1 softwaretrim compensate value */
    unsigned short aptMaxcntCmp; /**< Apt Maximum Comparison Count */

    unsigned short sysTickCnt;       /**< System Timer Tick Count */
    unsigned short capChargeTickNum; /**< Bootstrap Capacitor Charge Tick Count */
    volatile unsigned int msTickCnt; /**< Millisecond-level counter, which can be used in 1-ms and 5-ms tasks. */
    unsigned short msTickNum;        /**< Number of ticks corresponding to 1 ms */

    SysStatusReg statusReg; /**< System status */
    volatile FsmState stateMachine; /**< Motor Control State Machine */

    SampleMode sampleMode;  /**< sample mode */
    MtrParamHandle mtrParam;    /**< Motor parameters */
    FoSmoHandle smo;            /**< SMO observer handle */
    Smo4thHandle smo4th;        /**< SMO 4th observer handle */
    IfHandle ifCtrl;            /**< I/F control handle */
    SvpwmHandle sv;             /**< SVPWM Handle */
    R1SvpwmHandle r1Sv;         /**< Single-resistance phase-shifted SVPWM handld */
    RmgHandle spdRmg;           /**< Ramp management struct for the speed controller input reference */
    SpdCtrlHandle spdCtrl;      /**< Speed loop Control Handle */
    CurrCtrlHandle currCtrl;    /**< Current loop control handle */
    StartupHandle startup;      /**< Startup Switch Handle */
    FwCtrlHandle fw;            /**< Flux-Weakening Handle */

    DqAxis currRefDq;       /**< Command value of the dq axis current */
    UvwAxis currUvw;        /**< Three-phase current sampling value */
    AlbeAxis currAlbe;      /**< αβ-axis current feedback value */
    DqAxis currDq;          /**< Current feedback value of the dq axis */
    DqAxis vdq;             /**< Current loop output dq voltage */
    AlbeAxis vab;           /**< Current loop output voltage αβ */
    UvwAxis  dutyUvw;       /**< UVW three-phase duty cycle */
    UvwAxis  dutyUvwLeft;   /**< Single Resistor UVW Three-Phase Left Duty Cycle */
    UvwAxis  dutyUvwRight;  /**< Single Resistor UVW Three-Phase Right Duty Cycle*/

    MCS_ReadCurrUvwCb readCurrUvwCb;             /**< Read current callback function */
    MCS_SetPwmDutyCb setPwmDutyCb;	             /**< Set the duty cycle callback function. */
    MCS_SetADCTriggerTimeCb setADCTriggerTimeCb; /**< Sets the ADC trigger point callback function. */
} MtrCtrlHandle;

void MCS_CarrierProcess(MtrCtrlHandle *mtrCtrl);

#endif