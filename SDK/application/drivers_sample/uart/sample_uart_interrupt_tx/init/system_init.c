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
  * @file      system_init.c
  * @author    MCU Driver Team
  * @brief     This file contains driver init functions.
  */

#include "main.h"
#include "ioconfig.h"

#define UART0_BAND_RATE 115200

BASE_StatusType CRG_Config(CRG_CoreClkSelect *coreClkSelect)
{
    CRG_Handle crg;
    crg.baseAddress     = CRG;
    crg.pllRefClkSelect = CRG_PLL_REF_CLK_SELECT_HOSC;
    crg.pllPreDiv       = CRG_PLL_PREDIV_4;
    crg.pllFbDiv        = 32; /* PLL Multiplier 32 */
    crg.pllPostDiv      = CRG_PLL_POSTDIV_1;
    crg.coreClkSelect   = CRG_CORE_CLK_SELECT_PLL;
    if (HAL_CRG_Init(&crg) != BASE_STATUS_OK) {
        return BASE_STATUS_ERROR;
    }
    *coreClkSelect = crg.coreClkSelect;
    return BASE_STATUS_OK;
}

__weak void WriteFinish(UART_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART0WriteInterruptCallback */
    /* USER CODE END UART0WriteInterruptCallback */
}

__weak void UART0ReadInterruptCallback(UART_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART0ReadInterruptCallback */
    /* USER CODE END UART0ReadInterruptCallback */
}

__weak void UART0InterruptErrorCallback(UART_Handle *handle)
{
    BASE_FUNC_UNUSED(handle);
    /* USER CODE BEGIN UART0InterruptErrorCallback */
    /* USER CODE END UART0InterruptErrorCallback */
}

static void UART0_Init(void)
{
    HAL_CRG_IpEnableSet(UART0_BASE, IP_CLK_ENABLE);
    HAL_CRG_IpClkSelectSet(UART0_BASE, CRG_PLL_NO_PREDV);

    g_uart.baseAddress = UART0;
    g_uart.irqNum = IRQ_UART0;

    g_uart.baudRate = UART0_BAND_RATE;
    g_uart.dataLength = UART_DATALENGTH_8BIT;
    g_uart.stopBits = UART_STOPBITS_ONE;
    g_uart.parity = UART_PARITY_NONE;
    g_uart.txMode = UART_MODE_INTERRUPT;
    g_uart.rxMode = UART_MODE_INTERRUPT;
    g_uart.fifoMode = BASE_CFG_ENABLE;
    g_uart.fifoTxThr = UART_FIFOFULL_ONE_TWO;
    g_uart.fifoRxThr = UART_FIFOFULL_ONE_TWO;
    g_uart.hwFlowCtr = BASE_CFG_DISABLE;
    HAL_UART_Init(&g_uart);
    HAL_UART_RegisterCallBack(&g_uart, UART_WRITE_IT_FINISH, WriteFinish);

    HAL_UART_RegisterCallBack(&g_uart, UART_READ_IT_FINISH, UART0ReadInterruptCallback);

    HAL_UART_RegisterCallBack(&g_uart, UART_TRNS_IT_ERROR, UART0InterruptErrorCallback);

    HAL_UART_IRQService(&g_uart);
    IRQ_SetPriority(g_uart.irqNum, 1);
    IRQ_EnableN(g_uart.irqNum);
}

static void IOConfig(void)
{
    IOConfig_RegStruct *iconfig = IOCONFIG;

    iconfig->iocmg_7.BIT.func = 0x4; /* 0x4 is UART0_RXD */
    iconfig->iocmg_7.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_7.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_7.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_7.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_7.BIT.se = BASE_CFG_DISABLE;

    iconfig->iocmg_6.BIT.func = 0x4; /* 0x4 is UART0_TXD */
    iconfig->iocmg_6.BIT.ds = IO_DRV_LEVEL2;
    iconfig->iocmg_6.BIT.pd = BASE_CFG_DISABLE;
    iconfig->iocmg_6.BIT.pu = BASE_CFG_DISABLE;
    iconfig->iocmg_6.BIT.sr = IO_SPEED_SLOW;
    iconfig->iocmg_6.BIT.se = BASE_CFG_DISABLE;
}

void SystemInit(void)
{
    IOConfig();
    UART0_Init();

    /* USER CODE BEGIN system_init */
    /* USER CODE END system_init */
}