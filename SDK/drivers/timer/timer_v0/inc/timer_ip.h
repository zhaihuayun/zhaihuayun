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
  * @file      timer_ip.h
  * @author    MCU Driver Team
  * @brief     TIMER module driver.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the TIMER.
  *                + TIMER register mapping structure
  *                + Direct Configuration Layer functions of TIMER
  */


#ifndef McuMagicTag_TIMER_IP_H
#define McuMagicTag_TIMER_IP_H

/* Includes ------------------------------------------------------------------*/
#include "baseinc.h"

/**
  * @addtogroup TIMER
  * @{
  */

/**
  * @defgroup TIMER_IP TIMER_IP
  * @brief TIMER_IP: timer_v0
  * @{
  */

/**
  * @defgroup TIMER_Param_Def TIMER Parameters Definition
  * @brief Definition of TIMER configuration parameters.
  * @{
  */
#ifdef  TIMER_PARAM_CHECK
#define TIMER_ASSERT_PARAM          BASE_FUNC_ASSERT_PARAM
#define TIMER_PARAM_CHECK_NO_RET    BASE_FUNC_PARAMCHECK_NO_RET
#define TIMER_PARAM_CHECK_WITH_RET  BASE_FUNC_PARAMCHECK_WITH_RET
#else
#define TIMER_ASSERT_PARAM(para)               ((void)0U)
#define TIMER_PARAM_CHECK_NO_RET(para)         ((void)0U)
#define TIMER_PARAM_CHECK_WITH_RET(para, ret)  ((void)0U)
#endif

/**
  * @brief Period min value
  */
#define PERIOD_MIN_VALUE     1

/**
 * @brief  TIMER operating mode definition
 */
typedef enum {
    TIMER_MODE_RUN_FREE     = 0x00000000U,
    TIMER_MODE_RUN_PERIODIC = 0x00000001U,
    TIMER_MODE_RUN_ONTSHOT  = 0x00000002U,
} TIMER_Mode;

/**
 * @brief  TIMER division factor definition
 */
typedef enum {
    TIMERPRESCALER_NO_DIV  = 0x00000000U,
    TIMERPRESCALER_DIV_16  = 0x00000001U,
    TIMERPRESCALER_DIV_256 = 0x00000002U,
} TIMER_PrescalerFactor;

/**
 * @brief  TIMER counter size definition
 */
typedef enum {
    TIMER_SIZE_16BIT = 0x00000000U,
    TIMER_SIZE_32BIT = 0x00000001U,
} TIMER_Size;

/**
  * @}
  */

/**
 * @defgroup TIMER_Reg_Def TIMER Register Definition
 * @brief register mapping structure
 * @{
 */

/**
 * @brief TIMER control register structure
 */
typedef union {
    unsigned int   reg;
    struct {
        unsigned int   oneshot    : 1;   /**< Counting mode: single counting mode, periodic counting mode. */
        unsigned int   timersize  : 1;   /**< 16-bit/32-bit counter operation mode. */
        unsigned int   timerpre   : 2;   /**< This field is used to set the prescale factor of the timer. */
        unsigned int   reserved0  : 1;
        unsigned int   intenable  : 1;   /**< TIMERx_RIS interrupt mask. */
        unsigned int   timermode  : 1;   /**< Indicates the count mode of the timer. */
        unsigned int   timeren    : 1;   /**< Timer enable. */
        unsigned int   reserved1  : 24;
    } BIT;
} TIMER_CONTROL_Reg;

/**
 * @brief TIMER original interrupt register
 */
typedef struct {
    unsigned int   timerris   : 1;     /**< Raw interrupt status of timer. */
    unsigned int   reserved0  : 31;
} TIMER_RIS_Reg;

/**
 * @brief TIMER interrupt register of shield
 */
typedef struct {
    unsigned int   timermis   : 1;  /**< Masked interrupt status of timer. */
    unsigned int   reserved0  : 31;
} TIMER_MIS_Reg;

/**
 * @brief TIMER ControlB
 */
typedef union {
    unsigned int   reg;
    struct {
        unsigned int   dmabreqen   : 1;   /**< DMA burst request enable. */
        unsigned int   dmasreqen   : 1;   /**< Enable bit for DMA single request and trigger ADC sampling. */
        unsigned int   reserved0   : 29;
        unsigned int   dmaov       : 1;   /**< DMA request overflow flag. */
    } BIT;
} TIMER_CONTROLB_Reg;

/**
 * @brief TIMER register structure
 */
typedef struct {
    unsigned int       timer_load;      /**< Initial count value register. Offset address: 0x00000000U. */
    unsigned int       timer_value;     /**< Current count value register. Offset address: 0x00000004U. */
    TIMER_CONTROL_Reg  TIMERx_CONTROL;  /**< Timer control register. Offset address: 0x00000008U. */
    unsigned int       timer_intclr;    /**< Interrupt clear register. Offset address: 0x0000000CU. */
    TIMER_RIS_Reg      TIMERx_RIS;      /**< Raw interrupt register. Offset address: 0x00000010U. */
    TIMER_MIS_Reg      TIMERx_MIS;      /**< Masked interrupt register. Offset address: 0x00000014U. */
    unsigned int       timerbgload;     /**< Initial count value register. Offset address: 0x00000018U. */
    TIMER_CONTROLB_Reg TIMERx_CONTROLB; /**< Timerx control register B. Offset address: 0x0000001CU. */
} volatile TIMER_RegStruct;
/**
  * @}
  */

/* Parameter Check -----------------------------------------------------------*/
/**
  * @brief Verify Timer mode configuration
  * @param mode Timer Mode. @ref TIMER_Mode
  * @retval true
  * @retval false
  */
static inline bool IsTimerMode(TIMER_Mode mode)
{
    return (((mode) == TIMER_MODE_RUN_FREE) ||
            ((mode) == TIMER_MODE_RUN_PERIODIC) ||
            ((mode) == TIMER_MODE_RUN_ONTSHOT));
}

/**
  * @brief Verify Timer counter size configuration
  * @param size  Timer Size. @ref TIMER_Size
  * @retval true
  * @retval false
  */
static inline bool IsTimerSize(TIMER_Size size)
{
    return (((size) == TIMER_SIZE_16BIT) ||
            ((size) == TIMER_SIZE_32BIT));
}

/**
  * @brief Verify Timer period configuration
  * @param period
  * @retval true
  * @retval false
  */
static inline bool IsTimerPeriod(unsigned int period)
{
    return ((period) >= PERIOD_MIN_VALUE);
}

/**
  * @brief Verify Timer div configuration
  * @param div @see TIMER_PrescalerFactor
  * @retval true
  * @retval false
  */
static inline bool IsTimerDiv(TIMER_PrescalerFactor div)
{
    return (((div) == TIMERPRESCALER_NO_DIV) ||
            ((div) == TIMERPRESCALER_DIV_16) ||
            ((div) == TIMERPRESCALER_DIV_256));
}

/**
  * @brief Verify Timer interrupt configuration
  * @param interruptEn
  * @retval true
  * @retval false
  */
static inline bool IsTimerInterrupt(unsigned int interruptEn)
{
    return (((interruptEn) == BASE_CFG_SET) || ((interruptEn) == BASE_CFG_UNSET));
}


/* Direct configuration layer ------------------------------------------------*/

/**
 * @brief   Enable the timer, start to run
 * @param   timerx  Timer register baseAddr
 * @retval  None
 */
static inline void DCL_TIMER_Enable(TIMER_RegStruct * const timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    timerx->TIMERx_CONTROL.BIT.timeren = BASE_CFG_SET;
}

/**
 * @brief   Stop the timer
 * @param   timerx  Timer register baseAddr
 * @retval  None
 */
static inline void DCL_TIMER_Disable(TIMER_RegStruct * const timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    timerx->TIMERx_CONTROL.BIT.timeren = BASE_CFG_UNSET;
}

/**
 * @brief   Get the timer enable flag
 * @param   timerx  Timer register baseAddr
 * @retval  None
 */
static inline bool DCL_TIMER_GetTimerEn(const TIMER_RegStruct *timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    return timerx->TIMERx_CONTROL.BIT.timeren;
}

/**
 * @brief   Get current counter in timer
 * @param   timerx  Timer register baseAddr
 * @retval  None
 */
static inline unsigned int DCL_TIMER_GetValue(const TIMER_RegStruct *timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    return timerx->timer_value;
}

/**
 * @brief   Set the counter with load，which change timer value immediately
 * @param   timerx  Timer register baseAddr
 * @param   period  the init value of the counter
 * @retval  None
 */
static inline void DCL_TIMER_SetLoad(TIMER_RegStruct * const timerx, unsigned int period)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    TIMER_PARAM_CHECK_NO_RET(IsTimerPeriod(period));
    timerx->timer_load = period;
}

/**
 * @brief   Get the period of counter
 * @param   timerx  Timer register baseAddr
 * @retval  None
 */
static inline unsigned int DCL_TIMER_GetLoad(const TIMER_RegStruct *timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    return timerx->timer_load;
}

/**
 * @brief   Set the counter with period with bgload
 * @param   timerx  Timer register baseAddr
 * @param   period  the init value of the counter
 * @retval  None
 */
static inline void DCL_TIMER_SetBgLoad(TIMER_RegStruct * const timerx, unsigned int period)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    TIMER_PARAM_CHECK_NO_RET(IsTimerPeriod(period));
    timerx->timerbgload = period;
}

/**
 * @brief   Get the bgLoad of timer
 * @param   timerx  Timer register baseAddr
 * @retval  None
 */
static inline unsigned int DCL_TIMER_GetBgLoad(const TIMER_RegStruct *timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    return timerx->timerbgload;
}

/**
 * @brief   Enable timer interrupt
 * @param   timerx  Timer register baseAddr
 * @retval  None
 */
static inline void DCL_TIMER_InterruptEnable(TIMER_RegStruct * const timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    timerx->TIMERx_CONTROL.BIT.intenable = BASE_CFG_SET;
}

/**
 * @brief   Disable timer interrupt
 * @param   timerx  Timer register baseAddr
 * @retval  None
 */
static inline void DCL_TIMER_InterruptDisable(TIMER_RegStruct * const timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    timerx->TIMERx_CONTROL.BIT.intenable = BASE_CFG_UNSET;
}

/**
 * @brief   Get timer interrupt enable flag
 * @param   timerx  Timer register baseAddr
 * @retval  None
 */
static inline bool DCL_TIMER_GetInterruptEnableFlag(const TIMER_RegStruct *timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    return timerx->TIMERx_CONTROL.BIT.intenable;
}

/**
 * @brief   Set timer size
 * @param   timerx  Timer register baseAddr
 * @param   size  the size of counter, see @ref TIMER_Size
 * @retval  None
 */
static inline void DCL_TIMER_SetTimerSize(TIMER_RegStruct * const timerx, TIMER_Size size)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    TIMER_PARAM_CHECK_NO_RET(IsTimerSize(size));
    timerx->TIMERx_CONTROL.BIT.timersize = (size == TIMER_SIZE_16BIT) ? BASE_CFG_UNSET : BASE_CFG_SET;
}

/**
 * @brief   Set timer size
 * @param   timerx  Timer register baseAddr
 * @retval  None
 */
static inline TIMER_Size DCL_TIMER_GetTimerSize(const TIMER_RegStruct *timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    return timerx->TIMERx_CONTROL.BIT.timersize;
}

/**
 * @brief   Set the counting mode is single counting or periodic counting mode
 * @param   timerx  Timer register baseAddr
 * @param   mode counter mode, see @ref TIMER_Mode
 * @retval  None
 */
static inline void DCL_TIMER_SetTimerMode(TIMER_RegStruct * const timerx, TIMER_Mode mode)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    TIMER_PARAM_CHECK_NO_RET(IsTimerMode(mode));
    if (mode == TIMER_MODE_RUN_ONTSHOT) {
        /* Timer oneshot mode enable. */
        timerx->TIMERx_CONTROL.BIT.oneshot = BASE_CFG_SET;
    } else {
        /* Timer free and period mode setting. */
        timerx->TIMERx_CONTROL.BIT.oneshot = BASE_CFG_UNSET;
        timerx->TIMERx_CONTROL.BIT.timermode = (mode == TIMER_MODE_RUN_FREE) ? BASE_CFG_UNSET : BASE_CFG_SET;
    }
}

/**
 * @brief   Get the counting mode is single counting or periodic counting mode
 * @param   timerx  Timer register baseAddr
 * @retval  TIMER_Mode @ref TIMER_Mode
 */
static inline TIMER_Mode DCL_TIMER_GetTimerMode(const TIMER_RegStruct *timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    if (timerx->TIMERx_CONTROL.BIT.oneshot == BASE_CFG_SET) {
        return TIMER_MODE_RUN_ONTSHOT;
    } else {
        return (timerx->TIMERx_CONTROL.BIT.timermode == BASE_CFG_SET) ? TIMER_MODE_RUN_PERIODIC : TIMER_MODE_RUN_FREE;
    }
}

/**
 * @brief   Set the prescaler factor of the timer
 * @param   timerx  Timer register baseAddr
 * @param   factor  prescaler factor, see @ref TIMER_PrescalerFactor
 * @retval  None
 */
static inline void DCL_TIMER_SetTimerPre(TIMER_RegStruct * const timerx, TIMER_PrescalerFactor factor)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    TIMER_PARAM_CHECK_NO_RET(IsTimerDiv(factor));
    timerx->TIMERx_CONTROL.BIT.timerpre = factor;
}

/**
 * @brief   Get the prescaler factor of the timer
 * @param   timerx  Timer register baseAddr
 * @retval  TIMER_PrescalerFactor @ref TIMER_PrescalerFactor
 */
static inline TIMER_PrescalerFactor DCL_TIMER_GetTimerPre(const TIMER_RegStruct *timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    return timerx->TIMERx_CONTROL.BIT.timerpre;
}

/**
 * @brief   Deinitializes TIMERx_CONTROL.
 * @param   timerx  Timer register baseAddr
 * @retval  None.
 */
static inline void DCL_TIMER_DeinitTimerControl(TIMER_RegStruct * const timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    timerx->TIMERx_CONTROL.reg = BASE_CFG_UNSET;
}


/**
 * @brief   Clear the time irq flag
 * @param   timerx  Timer register baseAddr
 * @retval  None
 */
static inline void DCL_TIMER_IrqClear(TIMER_RegStruct * const timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    timerx->timer_intclr = BASE_CFG_SET;
}

/**
 * @brief   Get Original interrupt state
 * @param   timerx  Timer register baseAddr
 * @retval  bool
 */
static inline bool DCL_TIMER_GetTimerOriginalInterruptState(const TIMER_RegStruct *timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    return timerx->TIMERx_RIS.timerris;
}

/**
 * @brief   Get the interrupt status of Timer after shielding
 * @param   timerx  Timer register baseAddr
 * @retval  bool
 */
static inline bool DCL_TIMER_GetTimerShieldlInterruptState(const TIMER_RegStruct *timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    return timerx->TIMERx_MIS.timermis;
}

/**
 * @brief   Get DMA overflow status
 * @param   timerx  Timer register baseAddr
 * @retval  bool
 */
static inline bool DCL_TIMER_GetTimerDmaOverflowStatus(const TIMER_RegStruct *timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    return timerx->TIMERx_CONTROLB.BIT.dmaov;
}

/**
 * @brief   Clear DMA overflow status
 * @param   timerx  Timer register baseAddr
 * @retval  None
 */
static inline void DCL_TIMER_ClearTimerDmaOverflowStatus(TIMER_RegStruct * const timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    timerx->TIMERx_CONTROLB.BIT.dmaov = BASE_CFG_SET;
}

/**
 * @brief   Get DMA single request enable status
 * @param   timerx  Timer register baseAddr
 * @retval  bool
 */
static inline bool DCL_TIMER_GetTimerDmaSingleRequest(const TIMER_RegStruct *timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    return timerx->TIMERx_CONTROLB.BIT.dmasreqen;
}

/**
 * @brief   Set DMA single request
 * @param   timerx  Timer register baseAddr
 * @param   enable  DMA/ADC single trigger enable
 * @retval  None
 */
static inline void DCL_TIMER_SetTimerDmaSingleRequest(TIMER_RegStruct * const timerx, bool enable)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    timerx->TIMERx_CONTROLB.BIT.dmasreqen = (unsigned int)enable;
}

/**
 * @brief   Get DMA burst request enable status
 * @param   timerx  Timer register baseAddr
 * @retval  bool
 */
static inline bool DCL_TIMER_GetTimerDmaBurstRequest(const TIMER_RegStruct *timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    return timerx->TIMERx_CONTROLB.BIT.dmabreqen;
}

/**
 * @brief   Set DMA burst request
 * @param   timerx  Timer register baseAddr
 * @param   enable  DMA burst trigger enable
 * @retval  None
 */
static inline void DCL_TIMER_SetTimerDmaBurstRequest(TIMER_RegStruct * const timerx, bool enable)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    timerx->TIMERx_CONTROLB.BIT.dmabreqen = (unsigned int)enable;
}

/**
 * @brief   Deinitializes TIMERx_CONTROLB.
 * @param   timerx  Timer register baseAddr
 * @retval  None.
 */
static inline void DCL_TIMER_DeinitTimerControlB(TIMER_RegStruct * const timerx)
{
    TIMER_ASSERT_PARAM(IsTIMERInstance(timerx));
    timerx->TIMERx_CONTROLB.reg = BASE_CFG_UNSET;
}

/**
  * @}
  */

/**
 * @}
 */
#endif /* McuMagicTag_TIMER_IP_H */
