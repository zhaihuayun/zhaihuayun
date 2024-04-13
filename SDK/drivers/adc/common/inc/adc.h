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
  * @file    adc.h
  * @author  MCU Driver Team
  * @brief   ADC module driver
  * @details This file provides functions declaration of the ADC,
  *           + ADC initialization function.
  *           + Start ADC sample and conversion.
  *           + Start ADC sample and conversion with interrupt.
  *           + Start ADC sample and conversion with DMA.
  *           + Start ADC sample and conversion synchronously.
  *           + Query the ADC conversion result.
  *           + Single channel and multichannel software trigger functions.
  *           + Interrupt callback function and user registration function.
  *          This file also provides the definition of the ADC handle structure.
  */

/* Macro definitions */
#ifndef McuMagicTag_ADC_H
#define McuMagicTag_ADC_H

#include "adc_ip.h"
#include "dma.h"
#include "dac.h"
#include "interrupt.h"

/**
  * @defgroup ADC ADC
  * @brief ADC module.
  * @{
  */

/**
  * @defgroup ADC_Common ADC Common
  * @brief ADC common external module.
  * @{
  */

/**
  * @defgroup ADC_Handle_Definition ADC Handle Definition
  * @{
  */

/**
  * @brief The definition of the ADC handle structure.
  */
typedef struct _ADC_Handle {
    ADC_RegStruct          *baseAddress;        /**< ADC registers base address */
    ADC_PriorityMode        socPriority;        /**< ADC clock divider */
    DMA_Handle             *dmaHandle;          /**< ADC_DMA control */
    unsigned int            adcDmaChn;          /**< ADC_DMA channel */
    unsigned int            irqNumOver;         /**< ADC overflow interrupt number */
    ADC_StateSOC            socxOverState;      /**< ADC overflow state, the corresponding bits are set as 2 */
    ADC_VrefType            vrefBuf;            /**< ADC vrefBuf type, configuration options affected by VDDA*/
    bool                    initEnable;         /**< ADC initialization flag */
    struct {
        unsigned int   adcInput : 4;            /**< defined in ADC_Input */
        unsigned int   sampleTotalTime : 8;     /**< SOC specified input sample total time */
        unsigned int   sampleHoldTime : 8;      /**< SOC specified input sample hold time */
        unsigned int   softTrigSource : 2;      /**< software trigger mode, defined in ADC_SoftTrigSoc */
        unsigned int   periphTrigSource : 5;    /**< periph trigger source, defined in ADC_PeriphTrigSoc */
        unsigned int   intTrigSource : 2;       /**< interrupt trigger source, defined in ADC_IntTrigSoc */
        unsigned int   finishMode : 3;          /**< sample finish mode , defined in ADC_SOCFinishMode */
    } ADC_SOCxParam[SOC_MAX_NUM];
    struct {
        unsigned int        irqNum;             /**< ADC interrupt number */
        ADC_StateSOC        socxFinish;         /**< After each SOC is completed, the corresponding bit is set as 1 */
    } ADC_IntxParam[INT_MAX_NUM];
    void (* Int1FinishCallBack)(struct _ADC_Handle *handle); /**< ADC interrupt one complete callback function
                                                                  for users */
    void (* Int2FinishCallBack)(struct _ADC_Handle *handle); /**< ADC interrupt two complete callback function
                                                                  for users */
    void (* Int3FinishCallBack)(struct _ADC_Handle *handle); /**< ADC interrupt three complete callback function
                                                                  for users */
    void (* Int4FinishCallBack)(struct _ADC_Handle *handle); /**< ADC interrupt four complete callback function
                                                                  for users */
    void (* DmaFinishCallBack)(struct _ADC_Handle *handle);  /**< ADC DMA finish callback function for users */
    void (* IntxOverCallBack)(struct _ADC_Handle *handle);   /**< ADC interrupt overflow callback function for
                                                                  users */
    void (* DmaOverCallBack)(struct _ADC_Handle *handle);    /**< ADC DMA overflow callback function for users */
    void (* DmaErrorCallBack)(struct _ADC_Handle *handle);   /**< ADC DMA transmission error callback function for
                                                                  users */
} ADC_Handle;

/**
  * @brief The definition of the ADC callback function.
  */
typedef void (* ADC_CallbackType)(ADC_Handle *handle);

/**
  * @brief The definition of SOC parameter structure.
  */
typedef struct {
    ADC_Input           adcInput;           /**< SOC specified input */
    unsigned int        sampleTotalTime;    /**< SOC specified input sample total time */
    unsigned int        sampleHoldTime;     /**< SOC specified input hold time */
    ADC_SoftTrigSoc     softTrigSource;     /**< SOC specified input software trigger mode */
    ADC_PeriphTrigSoc   periphTrigSource;   /**< SOC specified input periph trigger source */
    ADC_IntTrigSoc      intTrigSource;      /**< SOC specified input interrupt trigger source */
    ADC_SOCFinishMode   finishMode;         /**< SOC specified input mode of finishing sample and conversion */
} SOC_Param;

/**
  * @brief The definition of synchronous sampling parameter structure.
  */
typedef struct {
    ADC_Input           ChannelA;           /**< The selection range is ADC_CH_ADCINA0 to ADC_CH_ADCINA7 */
    ADC_Input           ChannelB;           /**< The selection range is ADC_CH_ADCINB0 to ADC_CH_ADCINB7 */
    ADC_SyncSampleGroup group;              /**< Syncsample group param */
    unsigned int        sampleTotalTime;    /**< Generally, the default value 3 is selected */
    unsigned int        sampleHoldTime;     /**< Generally, the default value 2 is selected */
    ADC_SoftTrigSoc     softTrigSource;     /**< Software trigger mode */
    ADC_PeriphTrigSoc   periphTrigSource;   /**< Peripheral trigger source selection */
    ADC_IntTrigSoc      intTrigSource;      /**< Internal interrupt trigger source select */
    ADC_SOCFinishMode   finishMode;         /**< Sample finish mode */
    unsigned int       *dmaSaveAddr;        /**< When DMA is used, configure destination address for data transfer */
    DMA_ChannelParam   *dmaParam;           /**< When DMA is used, configure DMA transmission parameters */
} SOC_SyncParam;
/**
  * @}
  */

/**
  * @defgroup ADC_API_Declaration ADC HAL API
  * @{
  */
BASE_StatusType HAL_ADC_Init(ADC_Handle *adcHandle);
BASE_StatusType HAL_ADC_Deinit(ADC_Handle *adcHandle);
BASE_StatusType HAL_ADC_ConfigureSoc(ADC_Handle *adcHandle, ADC_SOCNumber soc, SOC_Param *socParam);
BASE_StatusType HAL_ADC_StartDma(ADC_Handle *adcHandle, unsigned int startSoc,
                                 unsigned int endSoc, unsigned int *saveData);
BASE_StatusType HAL_ADC_StartIt(ADC_Handle *adcHandle);
BASE_StatusType HAL_ADC_SoftTrigMultiSample(ADC_Handle *adcHandle, ADC_SoftMultiTrig syncTrig);
BASE_StatusType HAL_ADC_SoftTrigSample(ADC_Handle *adcHandle, unsigned int soc);
unsigned int HAL_ADC_GetConvResult(ADC_Handle *adcHandle, unsigned int soc);
unsigned int HAL_ADC_ActiveCalibrateRet(ADC_RegStruct * const adcx, unsigned int soc, unsigned int originalRet);
BASE_StatusType HAL_ADC_CheckSocFinish(ADC_Handle *adcHandle, unsigned int soc);
BASE_StatusType HAL_ADC_StartSyncSample(ADC_Handle *adcHandle, SOC_SyncParam *syncParam);
void HAL_ADC_RegisterCallBack(ADC_Handle *adcHandle, ADC_CallbackFunType typeID, ADC_CallbackType pCallback);
void HAL_ADC_IrqService(ADC_Handle *adcHandle);
void HAL_ADC_IrqHandler(void *handle);

#if defined (CHIP_3065HRPIRZ) || defined (CHIP_3065HRPICZ) || defined (CHIP_3061HRPIKZ) || \
    defined (AU302PDF51) || defined (AU302NDF51) || defined (AU301LDF51)
BASE_StatusType HAL_ADC_InitForVdda(ADC_RegStruct *adcx, ADC_SOCNumber soc, DAC_RegStruct *dacx, bool useDac);
float HAL_ADC_GetVddaByDac(ADC_RegStruct *adcx, ADC_SOCNumber soc, DAC_RegStruct *dacx, bool useDac);
unsigned int HAL_ADC_GetTransResultByVdda(ADC_RegStruct *adcx, ADC_SOCNumber soc, float vdda);
#endif
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif