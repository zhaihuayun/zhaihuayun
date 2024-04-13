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
  * @file    stack_protect.c
  * @author  MCU Driver Team
  * @brief   gcc stack protect.
  */

/* Includes ------------------------------------------------------------------*/


/* Macro definitions ---------------------------------------------------------*/

/* Typedef definitions -------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/
unsigned long __stack_chk_guard = 0x000a0dff;

/* Functions -----------------------------------------------------------------*/
void __stack_chk_fail(void);
void set_stack_chk_guard(unsigned long canary);

/**
  * @brief The stack check failure is handled on time
  */
void __stack_chk_fail(void)
{
}

#pragma GCC push_options
#pragma GCC optimize ("-fno-stack-protector")
/**
  * @brief Set the stack chk guard object
  * @param canary
  */
void set_stack_chk_guard(unsigned long canary)
{
    __stack_chk_guard = canary;
}
#pragma GCC pop_options
