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
  * @file      timer_adapt.c
  * @author    MCU Driver Team
  * @brief     TIMER adapt module.
  * @details   This file provides functions about the following functionalities:
  *                 + Start Timer
  *                 + Check Timer is out or not
  */
#include "chipinc.h"
#include "loaderboot.h"
#include "timer.h"
#include "timer_adapt.h"

TIMER_Handle g_htim1;

#define TIMER_1US  (HAL_CRG_GetIpFreq((void *)TIMER1_BASE) / US_PER_SEC)

/**
  * @brief   Start timer.
  * @param   timeout  Timer Timeout value, unit:us
  * @retval  None
  */
void TimerStart(unsigned int timeout)
{
    g_htim1.baseAddress = TIMER1;
    g_htim1.load        = timeout * TIMER_1US - 1;
    g_htim1.bgLoad      = g_htim1.load;
    g_htim1.mode        = TIMER_MODE_RUN_ONTSHOT;  /* Run in period mode */
    g_htim1.prescaler   = 0;   /* Don't frequency division */
    g_htim1.size        = 1;   /* 1 for 32bit, 0 for 16bit */
    (void)HAL_TIMER_Init(&g_htim1);
    HAL_TIMER_Start(&g_htim1);
}

/**
 * @brief Check Timer out
 * @return true
 * @return false
 */
bool IsTimerOut(void)
{
    return (DCL_TIMER_GetValue(g_htim1.baseAddress) == 0);
}
