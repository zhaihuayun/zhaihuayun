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
  * @file      mcs_config_61h48.h
  * @author    MCU Algorithm Team
  * @brief     This file provides config macros for 61demo app.
  */
#ifndef McuMagicTag_MCS_61H48_H
#define McuMagicTag_MCS_61H48_H

#include "debug.h"
#include "typedefs.h"

#define INV_CAP_CHARGE_MS    3u
#define INV_VOLTAGE_BUS      24.0f   /* Bus voltage, V */

#define CTRL_CURR_PERIOD     0.0001f /* carrier ISR period */
#define CTRL_SYSTICK_PERIOD  0.0005f /* systick control period */

/** Duty of sample window, the real time is 0.06*50us = 3us. */
#define SAMPLE_WINDOW_DUTY 0.06f

/** Duty of sample point shift as flip point, the real time is 0.008*50us = 0.4us. */
#define SAMPLE_POINT_SHIFT 0.008f

/* SMO */
#define SPEED_FILTER_CUTOFF_FREQUENCY 40.0f
#define FOSMO_GAIN 10.0f
#define FILTER_ANGLE_COMPENSATION 4826

/* User_Commond */
#define CTRL_IF_CURR_AMP_A          1.0f      /* IF control current amplitude */
/* 0.008055 Sampling resistance20 Ohm  Sampling resistance 100mOhm 0.001611f */
#define ADC_CURR_COFFI_CP           0.001611f
#define USER_TARGET_SPD_HZ          (100.0f)   /* Parentheses are used to enter negative instructions */
#define USER_SWITCH_SPDBEGIN_HZ     (70.0f)   /* Start of handover interval */
#define USER_SWITCH_SPDEND_HZ       (73.0f)   /* End of handover period */

#define USER_SPD_SLOPE  15.0f                       /* slope of velocity change */
#define USER_CURR_SLOPE (CTRL_IF_CURR_AMP_A * 0.5f) /* Current change slope  */

/* Service Definition */
#define PTC_RELAY_DELAY 2
#define MOTOR_START_DELAY 10

#endif /* McuMagicTag_MCS_61H48_H */
