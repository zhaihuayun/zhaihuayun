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
  * @file    serial_dw.h
  * @author  MCU Driver Team
  * @brief   serial driver head file.
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef McuMagicTag_SERIAL_DW_H
#define McuMagicTag_SERIAL_DW_H

/* Includes ------------------------------------------------------------------*/

/* Macro definitions ---------------------------------------------------------*/

/* UART register OFFSET */
#define UART_DR                     0x0       /**< Data register */
#define UART_RSR_ECR                0x04      /**< Receive status register/error clear register */
#define UART_FR                     0x18      /**< Flag register */
#define UART_IBRD                   0x24      /**< Integer baud rate register */
#define UART_FBRD                   0x28      /**< Float baud rate register */
#define UART_LCR_H                  0x2C      /**< Line control register */
#define UART_CR                     0x30      /**< Control register */
#define UART_IFLS                   0x34      /**< Interrupt FIFO threshold selection register */
#define UART_IMSC                   0x38      /**< Interrupt mask register */
#define UART_RIS                    0x3C      /**< Raw interrupt status register */
#define UART_MIS                    0x40      /**< Interrupt status register after mask */
#define UART_ICR                    0x44      /**< Interrupt clear register */
#define UART_DMACR                  0x48      /**< DMA control register */

#define UARTFR_TXFE_MASK            0x80      /**< TX FIFO Empty mask */
#define UARTFR_RXFF_MASK            0x40      /**< RX FIFO Full mask */
#define UARTFR_TXFF_MASK            0x20      /**< TX FIFO Full mask */
#define UARTFR_RXFE_MASK            0x10      /**< RX FIFO Empty mask */
#define UARTFR_BUSY_MASK            0x04      /**< Busy mask */
#define UARTDR_DATA_MASK            0xFF      /**< Data mask */

#define UARTLCR_H_CFG               0x60      /**< 8bit, no parity, FIFO disable */
#define UARTLCR_H_CFG_ODD           0x72      /**< 8bit, odd parity,FIFO enable */
#define UARTLCR_H_CFG_FIFO          0x70      /**< 8bit, no parity, FIFO enable */
#define UARTCR_CFG                  0x301     /**< UART tx enable, rx enable, uart enable */
#define UARTCR_CFG_FLOWENABLE       0xC301    /**< UART tx enable, rx enable, uart enable, flow control enable */
#define UARTCR_RTX_FC_SHIFT         14
#define UARTCR_CTX_FC_SHIFT         15
#define UARTCR_ENABLE               0x0001
#define UARTIMSC_CFG                0x0       /**< Disable all uart interrupt */
#define UARTIMSC_CFG_INT            0x50      /**< enable rx time out interrupt */
#define UARTIFS_CFG                 0x10A     /**< FIFO water mark:Rx 16 Tx 32 RTS 56 */
#define UARTIFS_RXFF_SHIFT          3         /**< RX FIFO full shift */
#define UARTIFS_RTSFF_SHIFT         6

#define UART_DMA_ENABLE             0X03      /**< DMA Enable */
#define UART_DMA_DISABLE            0X04      /**< DMA Disable */
#define UARTICR_CFG                 0x7FF     /* Clear up all uart interrupt */

/* Typedef definitions -------------------------------------------------------*/
/**
  * @brief The structure of the serial port baud rate
  */
typedef struct {
    unsigned int   baudrate; /**< Uart baud rate */
    unsigned short iBrd;     /**< Ibrd that needs to be configured to the register*/
    unsigned short fBrd;     /**< Fbrd that needs to be configured to the register*/
} UartBrd;

/**
  * @brief Uart baud rate index
  */
enum UartBrdIdx {
    UART_BRD_IDX_9600    = 0,
    UART_BRD_IDX_115200  = 1,
    UART_BRD_IDX_230400  = 2,
    UART_BRD_IDX_460800  = 3,
    UART_BRD_IDX_1024000 = 4,
    UART_BRD_IDX_2048000 = 5,
    UART_BRD_IDX_2560000 = 6,
    UART_BRD_IDX_3000000 = 7,
    UART_BRD_IDX_6000000 = 8,
    UART_BRD_IDX_MAX,
};

/**
  * @brief Uart baud rate
  */
enum UartBaudrate {
    UART_BRD_1200    = 1200,
    UART_BRD_2400    = 2400,
    UART_BRD_9600    = 9600,
    UART_BRD_19200   = 19200,
    UART_BRD_115200  = 115200,
    UART_BRD_230400  = 230400,
    UART_BRD_460800  = 460800,
    UART_BRD_921600  = 921600,
    UART_BRD_1000000 = 1000000,
    UART_BRD_1024000 = 1024000,
    UART_BRD_1500000 = 1500000,
    UART_BRD_1536000 = 1536000,
    UART_BRD_2000000 = 2000000,
    UART_BRD_2048000 = 2048000,
    UART_BRD_2252800 = 2252800,
    UART_BRD_2304000 = 2304000,
    UART_BRD_2500000 = 2500000,
    UART_BRD_2560000 = 2560000,
    UART_BRD_3000000 = 3000000,
    UART_BRD_3072000 = 3072000,
    UART_BRD_3500000 = 3500000,
    UART_BRD_3584000 = 3584000,
    UART_BRD_3840000 = 3840000,
    UART_BRD_4000000 = 4000000,
    UART_BRD_4096000 = 4096000,
    UART_BRD_4500000 = 4500000,
    UART_BRD_4608000 = 4608000,
    UART_BRD_5000000 = 5000000,
    UART_BRD_6000000 = 6000000,
};

/* Functions -----------------------------------------------------------------*/

#endif /* McuMagicTag_SERIAL_DW_H */
