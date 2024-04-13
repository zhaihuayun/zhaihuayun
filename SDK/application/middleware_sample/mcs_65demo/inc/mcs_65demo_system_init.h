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
  * @file      mcs_65demo_system_init.h
  * @author    MCU Algorithm Team
  * @brief     This file provides motor sample param declaration for AD105HDMA board.
  */
#ifndef McuMagicTag_MCS_65DEMO_SYSTEM_INIT_H
#define McuMagicTag_MCS_65DEMO_SYSTEM_INIT_H

#include "gpio.h"
#include "apt.h"
#include "apt_ip.h"
#include "adc.h"
#include "pga.h"
#include "timer.h"

extern ADC_Handle g_adc0;
extern ADC_Handle g_adc2;
extern APT_Handle g_apt0;
extern APT_OutCtrlProtect g_protectApt0;
extern APT_Handle g_apt1;
extern APT_OutCtrlProtect g_protectApt1;
extern APT_Handle g_apt2;
extern APT_OutCtrlProtect g_protectApt2;
extern APT_Handle g_apt3;
extern APT_OutCtrlProtect g_protectApt3;
extern APT_Handle g_apt4;
extern APT_OutCtrlProtect g_protectApt4;
extern APT_Handle g_apt5;
extern APT_OutCtrlProtect g_protectApt5;
extern GPIO_Handle g_gpio0;
extern GPIO_Handle g_gpio2;
extern TIMER_Handle g_timer1;

#define APT_SYNC_IN_SRC     APT_SYNCIN_SRC_APT3_SYNCOUT

#define APT_U_CP          APT3_BASE /* Base address of U phase APT module */
#define APT_V_CP          APT4_BASE /* Base address of V phase APT module */
#define APT_W_CP          APT5_BASE /* Base address of W phase APT module */

#define APT_U_FAN         APT0_BASE /* Base address of U phase APT module */
#define APT_V_FAN         APT1_BASE /* Base address of V phase APT module */
#define APT_W_FAN         APT2_BASE /* Base address of W phase APT module */

void SystemInit(void);

#endif /* McuMagicTag_MCS_65DEMO_SYSTEM_INIT_H */
