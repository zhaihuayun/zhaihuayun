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
  * @file    main.c
  * @author  MCU Driver Team
  * @brief   Defined The following functions
  * @details Loaderboot C function entry
  */

/* Includes ------------------------------------------------------------------*/
#include "baseinc.h"
#include "handshake.h"
#include "loaderboot.h"
#include "flash_adapt.h"
#include "uart_adapt.h"
#include "crc_adapt.h"
#include "reset.h"
#include "systickinit.h"
#include "cmd.h"

/* Macro definitions ---------------------------------------------------------*/
#define LOADER_HANDSHAKE_TIMEOUT_MS 40
#define DEFAULT_BAUDRATE            115200

#pragma GCC push_options
#pragma GCC optimize ("-fno-stack-protector")

/**
  * @brief The C Main Entry of Loader
  * @param None
  * @retval None
  */
void main(void)
{
    bool succ;

    CRC_Init();
    FLASH_Init();
    SYSTICK_Init();

    succ = HandShake(LOADER_HANDSHAKE_TIMEOUT_MS);
    SetLoaderType(succ ? LOADERBOOT_V2 : LOADERBOOT_V1);
    CmdLoop();
    Reset();
}

#pragma GCC pop_options
