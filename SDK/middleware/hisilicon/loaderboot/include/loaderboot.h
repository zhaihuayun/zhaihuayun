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
  * @file    loaderboot.h
  * @author  MCU Driver Team
  * @brief   Header file containing typedef and function as follow.
  * @details Register manipulation macro and Error Number
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef McuMagicTag_LOADERBOOT_H
#define McuMagicTag_LOADERBOOT_H

/* Includes ------------------------------------------------------------------*/
#include "typedefs.h"
#include "utils.h"

/* Macro definitions ---------------------------------------------------------*/
#ifdef FPGA
#define FLASH_SUPPORT
#ifdef FLASH_SUPPORT
#define CONFIG_CPU_CLOCK               (40 * 1000 * 1000)
#else  /* FPGA SRAM Version */
#define CONFIG_CPU_CLOCK               (60 * 1000 * 1000)
#endif
#else  /* ASIC Version */
#define CONFIG_CPU_CLOCK               (25 * 1000 * 1000)
#endif

#define CONFIG_LS_CLOCK                (CONFIG_CPU_CLOCK / 2)
#define CONFIG_UART_CLOCK              CONFIG_LS_CLOCK

#if !defined(FPGA) || defined(FLASH_SUPPORT)
#define EFLASH_ENABLE   1
#else
#define EFLASH_ENABLE   0
#endif

#define SC_LOCKEN_VALUE             (0xEA510000)  /** undo lock need to write to SC_LOCKEN */

#define BOOTROM_FINISH              (1 << 0)   /* bootrom finish flag, 0: not finish, 1: finished */
#define UPDATE_MODE                 (1 << 4)   /* Update mode bit in SC_SYS_STATE, 0:don't update  1: update */

#define MS_PER_SEC   1000
#define US_PER_MS    1000
#define US_PER_SEC   (MS_PER_SEC * US_PER_MS)
#define FREQ_1M      1000000

/* Typedef definitions -------------------------------------------------------*/

typedef enum {
    LOADERBOOT_V1,    /* Loader support download one image */
    LOADERBOOT_V2,    /* Loader support handshake and download images */
} LOADER_VER;


/* Functions -----------------------------------------------------------------*/

void Reset(void);

LOADER_VER GetLoaderType(void);

void SetLoaderType(LOADER_VER type);

#endif /* McuMagicTag_LOADERBOOT_H */
