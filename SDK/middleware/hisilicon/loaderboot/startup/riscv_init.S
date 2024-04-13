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
  * @file   riscv_init.S
  * @author  MCU Driver Team
  * @brief   loaderboot RISC-V trap handling and startup code.
  */
#ifndef ENTRY_S
#define ENTRY_S

.extern __stack_top

#define LREG lw
#define SREG sw
#define REGBYTES 4

#define INT_SIZE_ON_STACK  (16 * REGBYTES)

#define MSTATUS_MPP_MACHINE       0x00001800
#define MCAULSE_ECALL_FROM_MMODE  11
#define MCAULSE_ECALL_FROM_UMODE  8
#define EXC_SIZE_ON_STACK         (160)

#define  EFC_MAGIC_NUMBER       0xFEDCBA98   /* magic number */
#define  EFC_MAGIC_LOCK_RW      0x14710200   /* cmd operation magic word protection register */

#define YEAR  0x22
#define MON   0x05
#define DAY   0x05
#define VER   0x02

    .extern trap_entry
    .section      .text.entry
    .global _start
    .option norvc
_start:
    j handle_reset

version:
    .byte         YEAR
    .byte         MON
    .byte         DAY
    .byte         VER

trap_vector:
    j trap_entry_wrapper
    .option rvc

.macro push_reg
    addi  sp, sp, -(INT_SIZE_ON_STACK)
    stmia {ra, t0-t2, a0-a7, t3-t6}, (sp)
    addi  sp, sp, -(INT_SIZE_ON_STACK)
.endm

.macro pop_reg
    addi  sp, sp, INT_SIZE_ON_STACK
    ldmia {ra, t0-t2, a0-a7, t3-t6},(sp)
    addi  sp, sp, INT_SIZE_ON_STACK
.endm

trap_entry_wrapper:
    j trap_entry

.align 2
trap_entry:
    push_reg
    csrr  a0, mcause
    li    t1, MCAULSE_ECALL_FROM_MMODE
    beq   a0, t1, switch_to_umode
    li    t1, MCAULSE_ECALL_FROM_UMODE
    beq   a0, t1, switch_to_mmode

    pop_reg
    j trap_entry

.align 2
switch_to_umode:
    li    t2,  MSTATUS_MPP_MACHINE
    csrc  mstatus, t2
    csrr  t0,  mepc
    addi  t0,  t0,  4
    csrw  mepc, t0
    pop_reg
    mret

.align 2
switch_to_mmode:
    li    t2,  MSTATUS_MPP_MACHINE
    csrs  mstatus, t2
    csrr  t0,  mepc
    addi  t0,  t0,  4
    csrw  mepc, t0
    pop_reg
    mret

handle_reset:
    csrwi mstatus, 0
    csrwi mie, 0
    csrci mstatus, 0x08
    la t0, trap_vector
    addi t0, t0, 1
    csrw mtvec, t0
/* lock mtvec */
    #csrwi 0x7EF, 0x1

/* initialize global pointer */
    .option push
    .option norelax
    la gp, __global_pointer$
    .option pop

/* initialize stack pointer */
    la sp, __stack_top

/* perform the rest of initialization in C */
clear_bss:
    la t0, __bss_begin__
    la t1, __bss_end__
    li t2, 0x00000000
clear_bss_loop:
    bge     t0, t1, clear_bss_done /* are we at the end yet, if not , continue till the end */
    sw      t2, (t0)           /* clear BSS location */
    addi    t0, t0, 4          /* increment clear index pointer */
    j       clear_bss_loop
clear_bss_done:

start_coderom_data_copy:
    la t0, __data_start              /* SRAM addr */
    la t1, __data_load               /* ROM addr  */
    la t2, __data_end

coderom_data_copy_loop:
    bge  t0, t2, coderom_data_copy_done
    lw   t3, (t1)
    sw   t3, (t0)
    addi t0, t0, 4
    addi t1, t1, 4
    j    coderom_data_copy_loop
coderom_data_copy_done:

pmp_init:
    li t0, 0xB00
    csrw pmpaddr0, t0
    li t0,0x400400  /* 2C00?~0x1000FFF, BOOTROM, enable R+X */
    csrw pmpaddr1, t0
    li t0,0x800000  /* 0x1001000~0x1FFFFFF, Reserved: disable R+X+W */
    csrw pmpaddr2, t0
    li t0,0x801000  /* 0x2000000~0x2003FFF, SYSRAM_ITCM */
    csrw pmpaddr3, t0
    li t0, 0xC00000   /* 0x2004000 ~ 0x2FFFFFF, Reserved: disable R+X+W */
    csrw pmpaddr4, t0
    li t0, 0x1000000 /* 0x3000000 ~ 0x03FFFFFF: EFLASH: enable R+W */
    csrw pmpaddr5, t0
    li t0,0x1001000  /* 0x4000000 ~ 0x0x04003FFF: SYSTEM_DTCM enable R+W */
    csrw pmpaddr6, t0
    li t0,0x7080000  /* 0x0d:TOR-R-X; 0x0b:TOR-R-W; 0x08:TOR; 0x0c:TOR-x; 0x09:TOR-R */
    csrw pmpaddr7, t0

    li t0,0xf3333333  /* register TOR-R-W */
    csrw 0x7d8,t0

    li t0,0x0d080d8b  /* 0x0d:TOR-R-X; 0x0b:TOR-R-W; 0x08:TOR; 0x0c:TOR-x; 0x09:TOR-R */
    csrw pmpcfg0,t0
    li t0,0x0b0b0b08
    csrw pmpcfg1,t0

/* enable flash cmd */
    li t0, EFC_MAGIC_NUMBER
    li t1, EFC_MAGIC_LOCK_RW
    sw t0, (t1)

/* disable Icache */
    csrwi  0x7C0, 0x0 /* disable ICACHE */
    fence

/* disable Dcache */
    csrwi  0x7C1, 0x0 /* disable DCACHE */
    fence

    ecall        /* ecall: M-mode -> U-mode */

/* jump to C func. */
    tail main
#endif
