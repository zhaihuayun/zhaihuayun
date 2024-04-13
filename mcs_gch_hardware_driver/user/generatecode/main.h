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
  * @file      main.h
  * @author    MCU Driver Team
  * @brief     This file contains driver init functions.
  */

/* Define to prevent recursive inclusion ------------------------------------- */
#ifndef McuMagicTag_SYSTEM_INIT_H
#define McuMagicTag_SYSTEM_INIT_H

#include "adc.h"
#include "acmp.h"
#include "apt.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"
#include "pga.h"
#include "crg.h"

#define    IO_SPEED_FAST     0x00U
#define    IO_SPEED_SLOW     0x01U

#define    IO_DRV_LEVEL4     0x00U
#define    IO_DRV_LEVEL3     0x01U
#define    IO_DRV_LEVEL2     0x02U
#define    IO_DRV_LEVEL1     0x03U

#define    XTAL_DRV_LEVEL4   0x03U
#define    XTAL_DRV_LEVEL3   0x02U
#define    XTAL_DRV_LEVEL2   0x01U
#define    XTAL_DRV_LEVEL1   0x00U

extern ACMP_Handle g_acmp1;
extern PGA_Handle g_pga0;
extern PGA_Handle g_pga1;
extern TIMER_Handle g_timer0;
extern TIMER_Handle g_timer1;
extern TIMER_Handle g_timer2;
extern UART_Handle g_uart0;
extern UART_Handle g_uart2;
extern APT_Handle g_apt0;
extern APT_Handle g_apt1;
extern APT_Handle g_apt2;
extern APT_Handle g_apt3;
extern APT_Handle g_apt4;
extern APT_Handle g_apt5;
extern APT_Handle g_apt8;
extern ADC_Handle g_adc0;
extern ADC_Handle g_adc1;
extern ADC_Handle g_adc2;

extern GPIO_Handle g_gpio1;
extern GPIO_Handle g_gpio5;
extern GPIO_Handle g_gpio6;
extern GPIO_Handle g_gpio0;

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect);
void SystemInit(void);

void APT8EventCallback(void *aptHandle);
void APT3TimerCallback(void *aptHandle);
void APT3EventCallback(void *aptHandle);
void APT0TimerCallback(void *aptHandle);
void APT0EventCallback(void *aptHandle);
void ADC2Interrupt1Callback(ADC_Handle *handle);
void TIMER0CallbackFunction(void *handle);
void TIMER1CallbackFunction(void *handle);
void TIMER2CallbackFunction(void *handle);
void APT4TimerCallback(void *aptHandle);

#endif /* McuMagicTag_SYSTEM_INIT_H */