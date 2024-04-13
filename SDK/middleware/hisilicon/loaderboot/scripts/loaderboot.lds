
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
  * @file   loaderboot.lds
  * @author  MCU Driver Team
  * @brief   loaderboot link config file
  */

OUTPUT_ARCH( "riscv" )
ENTRY(_start)

MEMORY
{
    /* ram for common bss and data */
    SRAM_DATA(xrw) : ORIGIN = 0x4000000+5K,LENGTH = 1K

    /* ram for stack */
    SRAM_STACK(xrw) : ORIGIN = 0x4000000+6K,LENGTH = 2K

    /* ram for loaderboot */
    SRAM_ITCAM_CODE(rx): ORIGIN = 0x2000000,LENGTH = 5K
}

STACK_SIZE      = 2k-300;
NMI_STACK_SIZE  = 200;

SECTIONS
{
      /* The startup code goes first into FLASH */
    .text.entry : ALIGN(4)
    {
        KEEP(*(.text.entry))
    } > SRAM_ITCAM_CODE

    /* Stack in SRAM_DATA at Highest addresses */
    .stacks (NOLOAD) :
    {
        . = ALIGN(4);
        __SYSTEM_STACK_BEGIN__ = ORIGIN(SRAM_STACK);
        KEEP(*(.stacks))
        __SYSTEM_STACK_END__ = ORIGIN(SRAM_STACK) + STACK_SIZE;
        . = ALIGN(0x20);
        __nmi_stack_bottom = .;
        . += NMI_STACK_SIZE;
        __nmi_stack_top = .;
    } > SRAM_STACK
    __SYSTEM_STACK_SIZE__ = __SYSTEM_STACK_END__ - __SYSTEM_STACK_BEGIN__;
    __stack_top = __SYSTEM_STACK_END__;

    .text : ALIGN(4)
    {
        __start_addr = .;
        *(.text*)
        *(.ram.text*)
        . = ALIGN(4);
        __rodata_start = .;
        *(.rodata*)
        *(.got*)
        . = ALIGN(4);
        __rodata_end = .;
        __text_end = .;
    } > SRAM_ITCAM_CODE

    /* data section */
    .data : ALIGN(4)
    {
        __data_load = LOADADDR(.data);
        __data_start = .;
        *(.data*)
        . = ALIGN(4);
        __data_end = .;
    } > SRAM_DATA AT > SRAM_ITCAM_CODE
    __data_size = __data_end - __data_start;
    __global_pointer$ = __data_start + (__data_size / 2);

    /* bss section */
    .bss (NOLOAD) : ALIGN(4)
    {
        __bss_begin__ = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > SRAM_DATA AT > SRAM_ITCAM_CODE
    __bss_size__ = __bss_end__ - __bss_begin__;
}
