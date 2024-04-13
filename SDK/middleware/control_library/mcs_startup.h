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
  * @file      mcs_startup.h
  * @author    MCU Algorithm Team
  * @brief     Motor transition process from one speed and angle to another speed and angle.
  */

/* Define to prevent recursive inclusion ------------------------------------------------------- */
#ifndef McuMagicTag_STARTUP_H
#define McuMagicTag_STARTUP_H

/**
  * @defgroup STARTUP_MODULE  STARTUP MODULE
  * @brief The startup management module.
  * @{
  */

/**
  * @defgroup STARTUP_STRUCT  STARTUP STRUCT
  * @brief The startup management data struct definition.
  * @{
  */
/* Typedef definitions ------------------------------------------------------------------------- */
/**
  * @brief Startup process enum.
  * @details Speed transition stages:
  *          + STARTUP_STAGE_CURR -- Stage of current AMP is changing
  *          + STARTUP_STAGE_SPD -- Stage of speed is changing
  *          + STARTUP_STAGE_SWITCH -- Stage of switch
  */
typedef enum {
    STARTUP_STAGE_CURR = 1,
    STARTUP_STAGE_SPD,
    STARTUP_STAGE_SWITCH
} Startup_Stage;

/**
  * @brief Startup handover method struct members and parameters.
  */
typedef struct {
    Startup_Stage stage; /**< Startup switching status. */
    float spdBegin;      /**< Startup switching start speed (Hz). */
    float spdEnd;        /**< Startup switching end speed (Hz). */
    float regionInv;     /**< Inverse of the speed region. */
    float initCurr;      /**< The initial current (A). */
} StartupHandle;
/**
  * @}
  */

/**
  * @defgroup STARTUP_API  STARTUP API
  * @brief The startup management API declaration.
  * @{
  */
void STARTUP_Init(StartupHandle *startHandle, float spdBegin, float spdEnd);
void STARTUP_Clear(StartupHandle *startHandle);
float STARTUP_CurrCal(const StartupHandle *startHandle, float refHz);
/**
  * @}
  */

/**
  * @}
  */

#endif
