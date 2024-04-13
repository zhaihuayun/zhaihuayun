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
  * @file      cmm_ip.h
  * @author    MCU Driver Team
  * @brief     CMM module driver.
  * @details   This file provides firmware functions to manage the following
  *            functionalities of the CMM.
  *             + Register Struct of CMM
  *             + CMM Register Map struct
  *             + Direct Configuration Layer functions of CMM
  */

#ifndef McuMagicTag_CMM_IP_H
#define McuMagicTag_CMM_IP_H

/* Includes ------------------------------------------------------------------ */
#include "baseinc.h"
/* Macro definitions ------------------------------------------------------- */
#ifdef CMM_PARAM_CHECK
    #define CMM_ASSERT_PARAM         BASE_FUNC_ASSERT_PARAM
    #define CMM_PARAM_CHECK_NO_RET   BASE_FUNC_PARAMCHECK_NO_RET
    #define CMM_PARAM_CHECK_WITH_RET BASE_FUNC_PARAMCHECK_WITH_RET
#else
    #define CMM_ASSERT_PARAM(para)                ((void)0U)
    #define CMM_PARAM_CHECK_NO_RET(para)          ((void)0U)
    #define CMM_PARAM_CHECK_WITH_RET(param, ret)  ((void)0U)
#endif
/**
  * @addtogroup CMM
  * @{
  */

/**
  * @defgroup CMM_IP
  * @{
  */

/**
  * @defgroup CMM_Param_Def CMM Parameters Definition
  * @brief Description of CMM configuration parameters.
  * @{
  */

/* Typedef definitions ------------------------------------------------------- */
typedef enum {
    CMM_TRIGGER_RISE = 0x00000000U,
    CMM_TRIGGER_FALL = 0x00000001U,
    CMM_TRIGGER_BOTH = 0x00000002U,
    CMM_TRIGGER_NONE = 0x00000003U,
    CMM_TRIGGER_MAX
} CMM_Trigger_Mode;

typedef enum {
    CMM_TARGET_FREQ_DIV_0 = 0x00000000U,
    CMM_TARGET_FREQ_DIV_32 = 0x00000001U,
    CMM_TARGET_FREQ_DIV_128 = 0x00000002U,
    CMM_TARGET_FREQ_DIV_1024 = 0x00000003U,
    CMM_TARGET_FREQ_DIV_8192 = 0x00000004U,
    CMM_TARGET_FREQ_DIV_MAX
} CMM_Target_Freq_Div_Value;

typedef enum {
    CMM_REF_FREQ_DIV_0 = 0x00000000U,
    CMM_REF_FREQ_DIV_4 = 0x00000001U,
    CMM_REF_FREQ_DIV_8 = 0x00000002U,
    CMM_REF_FREQ_DIV_32 = 0x00000003U,
    CMM_REF_FREQ_DIV_MAX
} CMM_Ref_Freq_Div_Value;

typedef enum {
    CMM_TARGET_CLK_LOSC = 0x00000000U, /* 32K */
    CMM_TARGET_CLK_HOSC = 0x00000001U, /* 25M */
    CMM_TARGET_CLK_TCXO = 0x00000002U,
    CMM_TARGET_CLK_HS_SYS = 0x00000003U,
    CMM_TARGET_CLK_LS_SYS = 0x00000004U,
    CMM_TARGET_CLK_MAX
} CMM_Target_Clock_Source;

typedef enum {
    CMM_REF_CLK_LOSC = 0x00000000U, /* 32K */
    CMM_REF_CLK_HOSC = 0x00000001U, /* 25M */
    CMM_REF_CLK_TCXO = 0x00000002U,
    CMM_REF_CLK_HS_SYS = 0x00000003U,
    CMM_REF_CLK_MAX
} CMM_Ref_Clock_Source;

typedef enum {
    CMM_INT_COUNTER_OVERFLOW_MASK = 0x00000001U,
    CMM_INT_CHECK_END_MASK = 0x00000002U,
    CMM_INT_FREQ_ERR_MASK = 0x00000004U,
    CMM_INT_MAX
} CMM_Interrupt_Type;

/**
  * @}
  */

/**
  * @defgroup CMM_Reg_Def CMM Register Definition
  * @brief Description CMM register mapping structure.
  * @{
  */

/**
  * @brief CMM version registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int month_day : 16;        /**< Month and day. */
        unsigned int year : 8;              /**< Year. */
        unsigned int release_substep : 1;   /**< Version information. */
        unsigned int release_step : 1;      /**< Version information. */
        unsigned int release_ver : 1;       /**< Version information. */
        unsigned int reserved0 : 5;
    } BIT;
} CMVER_Reg;

/**
  * @brief CMM control registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cmen : 1;              /**< CMM enable or disable. */
        unsigned int reserved0 : 31;
    } BIT;
} CMCTRL_Reg;

/**
  * @brief CMM target clock control registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int tgtsel : 3;            /**< CMM target clock source. */
        unsigned int reserved0 : 1;
        unsigned int tgtscale : 3;          /**< CMM target clock divide factor. */
        unsigned int reserved1 : 1;
        unsigned int tgt_edgesel : 2;       /**< CMM target clock effective edge selection. */
        unsigned int reserved2 : 22;
    } BIT;
} CMTGTCTRL_Reg;

/**
  * @brief CMM reference clock control registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int refsel : 2;            /**< CMM reference clock source. */
        unsigned int reserved0 : 2;
        unsigned int refdiv : 2;            /**< CMM reference clock divide factor. */
        unsigned int reserved1 : 26;
    } BIT;
} CMREFCTRL_Reg;

/**
  * @brief CMM check window upper bound registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cmwdoh : 16;           /**< CMM check window upper bound value. */
        unsigned int reserved0 : 16;
    } BIT;
} CMWDOH_Reg;

/**
  * @brief CMM check window low bound registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cmwdol : 16;           /**< CMM check window low bound value. */
        unsigned int reserved0 : 16;
    } BIT;
} CMWDOL_Reg;

/**
  * @brief CMM count locked value registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cmcnt_lock : 16;       /**< CMM count locked value */
        unsigned int reserved0 : 16;
    } BIT;
} CMCNTLOCK_Reg;

/**
  * @brief CMM interrupt enable registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cnt_ovf_en : 1;        /**< CMM count overflow interrupt enable. */
        unsigned int chk_end_en : 1;        /**< CMM check end interrupt enable. */
        unsigned int freq_err_en : 1;       /**< CMM frequence error interrupt enable. */
        unsigned int reserved0 : 29;
    } BIT;
} CMINTENA_Reg;

/**
  * @brief CMM interrupt status registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cnt_ovf_int : 1;       /**< CMM count overflow interrupt status. */
        unsigned int chk_end_int : 1;       /**< CMM check end interrupt status. */
        unsigned int freq_err_int : 1;      /**< CMM frequence error interrupt status. */
        unsigned int reserved0 : 29;
    } BIT;
} CMINTSTS_Reg;

/**
  * @brief CMM initial interrupt registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cnt_ovf_raw : 1;       /**< CMM count overflow initial interrupt. */
        unsigned int chk_end_raw : 1;       /**< CMM check end initial interrupt. */
        unsigned int freq_err_raw : 1;      /**< CMM frequence error initial interrupt. */
        unsigned int reserved0 : 29;
    } BIT;
} CMINTRAW_Reg;

/**
  * @brief CMM interrupt injection registers union structure definition.
  */
typedef union {
    unsigned int reg;
    struct {
        unsigned int cnt_ovf_inj : 1;       /**< CMM frequence error interrupt injection. */
        unsigned int chk_end_inj : 1;       /**< CMM check end interrupt injection. */
        unsigned int freq_err_inj : 1;      /**< CMM frequence error interrupt injection. */
        unsigned int reserved0 : 29;
    } BIT;
} CMINTINJ_Reg;

/**
  * @brief CMM register mapping structure.
  */
typedef struct {
    CMVER_Reg CMVER;         /**< CMM version register, offset address: 0x0000. */
    CMCTRL_Reg CMCTRL;       /**< CMM control register, offset address: 0x0004. */
    CMTGTCTRL_Reg CMTGTCTRL; /**< CMM target clock control register, offset address: 0x0008. */
    CMREFCTRL_Reg CMREFCTRL; /**< CMM reference clock control register, offset address: 0x000C. */
    CMWDOH_Reg CMWDOH;       /**< CMM check window upper bound register, offset address: 0x0010. */
    CMWDOL_Reg CMWDOL;       /**< CMM check window low bound register, offset address: 0x0014. */
    CMCNTLOCK_Reg CMCNTLOCK; /**< CMM count locked value register, offset address: 0x0018. */
    CMINTENA_Reg CMINTENA;   /**< CMM interrupt enable register, offset address: 0x001C. */
    CMINTSTS_Reg CMINTSTS;   /**< CMM interrupt status register, offset address: 0x0020. */
    CMINTRAW_Reg CMINTRAW;   /**< CMM initial interrupt register, offset address: 0x0024. */
    CMINTINJ_Reg CMINTINJ;   /**< CMM interrupt injection register, offset address: 0x0028. */
} volatile CMM_RegStruct;

/**
  * @}
  */

/**
  * @brief Enable CMM module.
  * @param cmmx CMM register base address.
  * @retval None.
  */
static inline void DCL_CMM_Enable(CMM_RegStruct *cmmx)
{
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    cmmx->CMCTRL.BIT.cmen = BASE_CFG_ENABLE;
}

/**
  * @brief Disable CMM module.
  * @param cmmx CMM register base address.
  * @retval None.
  */
static inline void DCL_CMM_Disable(CMM_RegStruct *cmmx)
{
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    cmmx->CMCTRL.BIT.cmen = BASE_CFG_DISABLE;
}

/**
  * @brief Sets the valid edge of the target clock.
  * @param cmmx CMM register base address.
  * @param mode Type of valid edge.
  * @retval None.
  */
static inline void DCL_CMM_SetTargetClockTriggerMode(CMM_RegStruct *cmmx, CMM_Trigger_Mode mode)
{
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    CMM_PARAM_CHECK_NO_RET(mode < CMM_TRIGGER_MAX);
    cmmx->CMTGTCTRL.BIT.tgt_edgesel = mode;
}

/**
  * @brief Gets the valid edge of the target clock.
  * @param cmmx CMM register base address.
  * @retval unsigned int @ref CMM_Trigger_Mode.
  */
static inline unsigned int DCL_CMM_GetTargetClockTriggerMode(CMM_RegStruct *cmmx)
{
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    return cmmx->CMTGTCTRL.BIT.tgt_edgesel;
}

/**
  * @brief Sets the frequency divider of the target clock.
  * @param cmmx CMM register base address.
  * @param value Specified frequency divider.
  * @retval None.
  */
static inline void DCL_CMM_SetTargetClockFreqDivision(CMM_RegStruct *cmmx, CMM_Target_Freq_Div_Value value)
{
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    CMM_PARAM_CHECK_NO_RET(value < CMM_TARGET_FREQ_DIV_MAX);
    cmmx->CMTGTCTRL.BIT.tgtscale = value;
}

/**
  * @brief Gets the frequency divider of the target clock.
  * @param cmmx CMM register base address.
  * @retval unsigned int @ref CMM_Target_Freq_Div_Value.
  */
static inline unsigned int DCL_CMM_GetTargetClockFreqDivision(CMM_RegStruct *cmmx)
{
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    return cmmx->CMTGTCTRL.BIT.tgtscale;
}

/**
  * @brief Sets the target clock source.
  * @param cmmx CMM register base address.
  * @param clockSource Specifies the type of the clock source.
  * @retval None.
  */
static inline void DCL_CMM_SetTargetClockSource(CMM_RegStruct *cmmx, CMM_Target_Clock_Source clockSource)
{
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    CMM_PARAM_CHECK_NO_RET(clockSource < CMM_TARGET_CLK_MAX);
    cmmx->CMTGTCTRL.BIT.tgtsel = clockSource;
}

/**
  * @brief Gets the target clock source.
  * @param cmmx CMM register base address.
  * @retval unsigned int @ref CMM_Target_Clock_Source.
  */
static inline unsigned int DCL_CMM_GetTargetClockSource(CMM_RegStruct *cmmx)
{
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    return cmmx->CMTGTCTRL.BIT.tgtsel;
}

/**
  * @brief Sets the frequency divider of the reference clock.
  * @param cmmx CMM register base address.
  * @param value Specified frequency divider.
  * @retval None.
  */
static inline void DCL_CMM_SetRefClockFreqDivision(CMM_RegStruct *cmmx, CMM_Ref_Freq_Div_Value value)
{
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    CMM_PARAM_CHECK_NO_RET(value < CMM_REF_FREQ_DIV_MAX);
    cmmx->CMREFCTRL.BIT.refdiv = value;
}

/**
  * @brief Gets the frequency divider of the reference clock.
  * @param cmmx CMM register base address.
  * @retval unsigned int @ref CMM_Ref_Freq_Div_Value.
  */
static inline unsigned int DCL_CMM_GetRefClockFreqDivision(CMM_RegStruct *cmmx)
{
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    return cmmx->CMREFCTRL.BIT.refdiv;
}

/**
  * @brief Sets the reference clock source.
  * @param cmmx CMM register base address.
  * @param clockSource Specified reference clock source.
  * @retval None.
  */
static inline void DCL_CMM_SetRefClockSource(CMM_RegStruct *cmmx, CMM_Ref_Clock_Source clockSource)
{
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    CMM_PARAM_CHECK_NO_RET(clockSource < CMM_REF_CLK_MAX);
    cmmx->CMREFCTRL.BIT.refsel = clockSource;
}

/**
  * @brief Gets the reference clock source.
  * @param cmmx CMM register base address.
  * @retval unsigned int @ref CMM_Ref_Clock_Source.
  */
static inline unsigned int DCL_CMM_GetRefClockSource(CMM_RegStruct *cmmx)
{
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    return cmmx->CMREFCTRL.BIT.refsel;
}

/**
  * @brief Sets the upper boundary of the detection window.
  * @param cmmx CMM register base address.
  * @param value The value of the upper bound.
  * @retval None.
  */
static inline void DCL_CMM_SetWindowUpperBound(CMM_RegStruct *cmmx, unsigned short value)
{
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    cmmx->CMWDOH.BIT.cmwdoh = value;
}

/**
  * @brief Gets the upper boundary of the detection window.
  * @param cmmx CMM register base address.
  * @retval The value of the upper bound.
  */
static inline unsigned short DCL_CMM_GetWindowUpperBound(CMM_RegStruct *cmmx)
{
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    return cmmx->CMWDOH.BIT.cmwdoh;
}

/**
  * @brief Sets the lower boundary of the detection window.
  * @param cmmx CMM register base address.
  * @param value The value of the lower bound.
  * @retval None.
  */
static inline void DCL_CMM_SetWindowLowerBound(CMM_RegStruct *cmmx, unsigned short value)
{
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    cmmx->CMWDOL.BIT.cmwdol = value;
}

/**
  * @brief Gets the lower boundary of the detection window.
  * @param cmmx CMM register base address.
  * @retval The value of the lower bound.
  */
static inline unsigned short DCL_CMM_GetWindowLowerBound(CMM_RegStruct *cmmx)
{
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    return cmmx->CMWDOL.BIT.cmwdol;
}

/**
  * @brief Internal counter count latch value.
  * @param cmmx CMM register base address.
  * @retval unsigned short. latch value.
  */
static inline unsigned short DCL_CMM_GetCntValue(CMM_RegStruct *cmmx)
{
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    return cmmx->CMCNTLOCK.BIT.cmcnt_lock;
}

/**
  * @brief Enables the specified type of interrupt.
  * @param cmmx CMM register base address.
  * @param type Mask of the interrupt type.
  * @retval None.
  */
static inline void DCL_CMM_EnableInterrupt(CMM_RegStruct *cmmx, CMM_Interrupt_Type type)
{
    /* if define macro CMM_PARAM_CHECK, function of param check is valid */
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    CMM_PARAM_CHECK_NO_RET(type == CMM_INT_COUNTER_OVERFLOW_MASK || \
                           type == CMM_INT_CHECK_END_MASK || \
                           type == CMM_INT_FREQ_ERR_MASK);
    cmmx->CMINTENA.reg |= type;
}

/**
  * @brief Disables the specified type of interrupt.
  * @param cmmx CMM register base address.
  * @param type Mask of the interrupt type.
  * @retval None.
  */
static inline void DCL_CMM_DisableInterrupt(CMM_RegStruct *cmmx, CMM_Interrupt_Type type)
{
    /* if define macro CMM_PARAM_CHECK, function of param check is valid */
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    CMM_PARAM_CHECK_NO_RET(type == CMM_INT_COUNTER_OVERFLOW_MASK || \
                           type == CMM_INT_CHECK_END_MASK || \
                           type == CMM_INT_FREQ_ERR_MASK);
    cmmx->CMINTENA.reg &= (~type);
}

/**
  * @brief Check whether the specified interrupt is triggered.
  * @param cmmx CMM register base address.
  * @param type Mask of the interrupt type.
  * @retval bool.
  */
static inline bool DCL_CMM_GetInterruptStatus(CMM_RegStruct *cmmx, CMM_Interrupt_Type type)
{
    /* if define macro CMM_PARAM_CHECK, function of param check is valid */
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    CMM_PARAM_CHECK_WITH_RET(type == CMM_INT_COUNTER_OVERFLOW_MASK || \
                             type == CMM_INT_CHECK_END_MASK || \
                             type == CMM_INT_FREQ_ERR_MASK, false);
    return (cmmx->CMINTSTS.reg & type) == 0 ? false : true;
}

/**
  * @brief Clears interrupts of the specified type.
  * @param cmmx CMM register base address.
  * @param type Mask of the interrupt type.
  * @retval None.
  */
static inline void DCL_CMM_ClearInterrupt(CMM_RegStruct *cmmx, CMM_Interrupt_Type type)
{
    /* if define macro CMM_PARAM_CHECK, function of param check is valid */
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    CMM_PARAM_CHECK_NO_RET(type == CMM_INT_COUNTER_OVERFLOW_MASK || \
                           type == CMM_INT_CHECK_END_MASK || \
                           type == CMM_INT_FREQ_ERR_MASK);
    cmmx->CMINTRAW.reg |= type;
}

/**
  * @brief Injects interrupts of the specified type.
  * @param cmmx CMM register base address.
  * @param type Mask of the interrupt type.
  * @retval None.
  */
static inline void DCL_CMM_EnableInterruptInject(CMM_RegStruct *cmmx, CMM_Interrupt_Type type)
{
    /* if define macro CMM_PARAM_CHECK, function of param check is valid */
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    CMM_PARAM_CHECK_NO_RET(type == CMM_INT_COUNTER_OVERFLOW_MASK || \
                           type == CMM_INT_CHECK_END_MASK || \
                           type == CMM_INT_FREQ_ERR_MASK);
    cmmx->CMINTINJ.reg |= type;
}

/**
  * @brief Stop injecting interrupts of a specified type.
  * @param cmmx CMM register base address.
  * @param type Mask of the interrupt type.
  * @retval None.
  */
static inline void DCL_CMM_DisableInterruptInject(CMM_RegStruct *cmmx, CMM_Interrupt_Type type)
{
    /* if define macro CMM_PARAM_CHECK, function of param check is valid */
    CMM_ASSERT_PARAM(IsCMMInstance(cmmx));
    CMM_PARAM_CHECK_NO_RET(type == CMM_INT_COUNTER_OVERFLOW_MASK || \
                           type == CMM_INT_CHECK_END_MASK || \
                           type == CMM_INT_FREQ_ERR_MASK);
    cmmx->CMINTINJ.reg &= (~type);
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* McuMagicTag_CMM_IP_H */