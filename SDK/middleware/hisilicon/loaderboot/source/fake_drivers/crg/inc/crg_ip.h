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
 * @file      crg_ip.h
 * @author    MCU Driver Team
 * @brief     TIMER module driver.
 * @details   This file provides firmware functions to manage the following
 *            functionalities of the TIMER.
 *                + CRG register mapping structure
 *                + Direct Configuration Layer functions of CRG
 */
#ifndef McuMagicTag_CRG_IP_H
#define McuMagicTag_CRG_IP_H

/* Includes ------------------------------------------------------------------*/
#include "baseinc.h"

/**
  * @addtogroup CRG
  * @{
  */

/**
  * @defgroup CRG_IP CRG_IP
  * @brief CRG_IP: crg_v0
  * @{
  */
  
/**
 * @defgroup CRG_Param_Def CRG Parameters Definition
 * @brief Definition of CRG configuration parameters.
 * @{
 */
#define CRG_FREQ_1MHz           (1000 * 1000)
#define HCLK_FREQ               (25 * CRG_FREQ_1MHz)
#define PCLK_FREQ               (HCLK_FREQ / 2)
/**
  * @}
  */
  
/**
  * @}
  */
  
/**
 * @}
 */
#endif /* McuMagicTag_CRG_IP_H */
