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
  * @file    flash.c
  * @author  MCU Driver Team
  * @brief   FLASH module driver.
  * @details This file provides firmware functions to manage the following
  *          functionalities of the FLASH.
  *          + Initialization and de-initialization functions.
  *          + Read, write, and erase functions.
  */

/* Includes ------------------------------------------------------------------ */
#include "flash.h"

#define FLASH_CRC_SAVE_BUFFER_LEN   2
#define FLASH_ALL_INTERRUPT_ENABLE  0x001F0010
#define FLASH_ERR_INTERRUPT_MASK    0x001F0000
#define FLASH_CMD_INTERRUPT_MASK    0x00000010

#define FLASH_KEY_REGISTER_UNLOCK_VALUE   0xFEDCBA98
#define FLASH_KEY_REGISTER_LOCK_VALUE     0x0

#define FLASH_INT_ERR_ECC_CHK_MASK      (1 << 20)
#define FLASH_INT_ERR_ECC_CORR_MASK     (1 << 19)
#define FLASH_INT_ERR_AHB_MASK          (1 << 18)
#define FLASH_INT_ERR_SMWR_MASK         (1 << 17)
#define FLASH_INT_ERR_ILLEGAL_MASK      (1 << 16)
#define FLASH_INT_FINISH_MASK           (1<< 4)

/**
  * @brief Check all initial configuration parameters.
  * @param handle FLASH handle.
  * @retval None.
  */
static void CheckAllInitParameters(FLASH_Handle *handle)
{
#ifndef FLASH_PARAM_CHECK
    BASE_FUNC_UNUSED(handle);
#endif
    FLASH_ASSERT_PARAM(IsFlashOperationMode(handle->peMode));
}

/**
  * @brief Writes to the flash memory in the unit of words.
  * @param handle FLASH handle.
  * @param srcAddr Start address of the data buffer to be written.
  * @param destAddr Flash destination address, which must be word-aligned.
  * @param wordNum Number of words written.
  * @retval None.
  */
static BASE_StatusType FLASH_WriteWords(FLASH_Handle *handle,
                                        const unsigned int srcAddr,
                                        const unsigned int destAddr,
                                        const unsigned int wordNum)
{
    unsigned int *data = NULL;
    unsigned int i;
    unsigned int writeSize;
    if (handle->baseAddress->EFLASH_CMD.BIT.cmd_start) {
        return BASE_STATUS_BUSY;
    }

    handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_UNLOCK_VALUE;
    if ((handle->baseAddress->BUF_CLEAR.reg >> FLASH_PGM_WBUF_CNT_POS) & 0xFF) {
        handle->baseAddress->BUF_CLEAR.BIT.pgm_wdata_clr = BASE_CFG_SET;
    }

    writeSize = ((wordNum % FLASH_MIN_PGM_WORDS_SIZE) != 0) ? (wordNum / FLASH_MIN_PGM_WORDS_SIZE + 1) :
                wordNum / FLASH_MIN_PGM_WORDS_SIZE;

    data = (unsigned int *)(uintptr_t)srcAddr;
    handle->baseAddress->EFLASH_ADDR.BIT.cmd_addr = destAddr;
    for (i = 0; i < wordNum; i++) {
        handle->baseAddress->PGM_WDATA = *data;
        data++;
    }
    handle->baseAddress->EFLASH_CMD.BIT.cmd_pgm_size = writeSize;
    handle->baseAddress->EFLASH_CMD.BIT.cmd_code = FLASH_OPERATION_PROGRAM;
    handle->baseAddress->EFLASH_CMD.BIT.cmd_start = BASE_CFG_SET;
    if (handle->peMode == FLASH_PE_OP_BLOCK) {
        while (handle->baseAddress->EFLASH_CMD.BIT.cmd_start) {
            ;
        }
        if (handle->baseAddress->INT_RAW_STATUS.BIT.int_raw_err_illegal ||
            handle->baseAddress->INT_RAW_STATUS.BIT.int_raw_err_smwr) {
            handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE;
            return BASE_STATUS_ERROR;
        }
        if (handle->baseAddress->FLASH_STATUS.reg) {
            handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE;
            return BASE_STATUS_ERROR;
        }
    }
    handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE;
    return BASE_STATUS_OK;
}

/**
  * @brief Obtains the number of words to be supplemented for write alignment to prevent cross-page write.
  * @param handle FLASH handle.
  * @retval Words.
  */
static unsigned int FLASH_GetWriteAlignmentWords(FLASH_Handle *handle)
{
    unsigned int numWords;

    numWords = handle->destAddr % FLASH_MAX_PGM_WORD_SIZE;
    if (numWords > 0) {
        return FLASH_MAX_PGM_WORD_SIZE - numWords;
    }
    return 0;
}

/**
  * @brief Flash erase operation.
  * @param handle FLASH handle.
  * @param startAddr Erasing start address, which must be aligned with the minimum erasing unit.
  * @param mode Erasing operation mode, supporting chip,and page.
  * @retval None.
  */
static BASE_StatusType FLASH_EraseWithMode(FLASH_Handle *handle, unsigned int startAddr, unsigned int mode)
{
    if (handle->baseAddress->EFLASH_CMD.BIT.cmd_start) {
        return BASE_STATUS_BUSY;
    }
    handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_UNLOCK_VALUE;
    handle->baseAddress->EFLASH_ADDR.BIT.cmd_addr = startAddr;
    handle->baseAddress->EFLASH_CMD.BIT.cmd_code  = mode;
    handle->baseAddress->EFLASH_CMD.BIT.cmd_start = BASE_CFG_SET;
    if (handle->peMode == FLASH_PE_OP_BLOCK) {
        while (handle->baseAddress->EFLASH_CMD.BIT.cmd_start) {
            ;
        }
        if (handle->baseAddress->INT_RAW_STATUS.BIT.int_raw_err_illegal ||
            handle->baseAddress->INT_RAW_STATUS.BIT.int_raw_err_smwr) {
            handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE;
            return BASE_STATUS_ERROR;
        }
        if (handle->baseAddress->FLASH_STATUS.reg) {
            handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE;
            return BASE_STATUS_ERROR;
        }
    }
    handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE;
    return BASE_STATUS_OK;
}

/**
  * @brief Write interrupt processing function,
  *        which completes the internal processing of the write operation in interrupt mode.
  * @param handle FLASH handle.
  * @retval None.
  */
static BASE_StatusType FLASH_WriteHandler(FLASH_Handle *handle)
{
    unsigned int dataLeft;
    unsigned int dataLast;
    BASE_StatusType ret;

    if ((handle->writeLen / (FLASH_MAX_PGM_WORD_SIZE * FLASH_ONE_WORD_BYTES_SIZE)) > 0) {
        ret = FLASH_WriteWords(handle, handle->srcAddr, handle->destAddr, FLASH_MAX_PGM_WORD_SIZE);
        if (ret != BASE_STATUS_OK) {
            return ret;
        }
        handle->srcAddr += FLASH_MAX_PGM_WORD_SIZE * FLASH_ONE_WORD_BYTES_SIZE;
        handle->destAddr += FLASH_MAX_PGM_WORD_SIZE;
        handle->writeLen -= FLASH_MAX_PGM_WORD_SIZE * FLASH_ONE_WORD_BYTES_SIZE;
    } else {
        dataLeft = handle->writeLen;
        dataLast = dataLeft % FLASH_ONE_WORD_BYTES_SIZE;
        if (dataLeft > 0) {
            if (dataLast > 0) {
                ret = FLASH_WriteWords(handle, handle->srcAddr, handle->destAddr,
                                       dataLeft / FLASH_ONE_WORD_BYTES_SIZE + 1);
            } else {
                ret = FLASH_WriteWords(handle, handle->srcAddr, handle->destAddr, dataLeft / FLASH_ONE_WORD_BYTES_SIZE);
            }
            if (ret != BASE_STATUS_OK) {
                return ret;
            }
            handle->writeLen = 0;
            handle->destAddr += dataLeft;
        }
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Erase interrupt processing function,
  *        which completes the internal processing of the erase operation in interrupt mode.
  * @param handle FLASH handle.
  * @retval None.
  */
static BASE_StatusType FLASH_EraseHandler(FLASH_Handle *handle)
{
    BASE_StatusType ret;
    ret = FLASH_EraseWithMode(handle, handle->destAddr, FLASH_ERASE_MODE_PAGE);
    if (ret != BASE_STATUS_OK) {
        return ret;
    }
    handle->destAddr += FLASH_ONE_PAGE_SIZE / sizeof(unsigned int);
    handle->eraseNum--;
    return BASE_STATUS_OK;
}

/**
  * @brief Initializing the FLASH Module.
  * @param handle FLASH handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_FLASH_Init(FLASH_Handle *handle)
{
    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    CheckAllInitParameters(handle);
    handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_UNLOCK_VALUE; /* Unlock key registers */
    if (handle->peMode == FLASH_PE_OP_IT) {
        handle->baseAddress->CMD_CFG_COMMON.BIT.int_mode = BASE_CFG_SET;
        handle->baseAddress->INT_ENABLE.reg = FLASH_ALL_INTERRUPT_ENABLE;
        handle->baseAddress->INT_CLEAR.reg = FLASH_ALL_INTERRUPT_ENABLE;
    } else {
        handle->baseAddress->CMD_CFG_COMMON.BIT.int_mode = BASE_CFG_UNSET;
    }
    handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE; /* Locking Key Registers */
    handle->state = FLASH_STATE_READY;
    return BASE_STATUS_OK;
}

/**
  * @brief Deinitialize the FLASH Module.
  * @param handle FLASH handle.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_FLASH_DeInit(FLASH_Handle *handle)
{
    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    handle->state = FLASH_STATE_RESET;
    handle->FlashCallBack = NULL; /* Clean interrupt callback functions. */
    handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_UNLOCK_VALUE;
    /* Disable interrupt mode and interrupt enable bit. */
    handle->baseAddress->CMD_CFG_COMMON.BIT.int_mode = BASE_CFG_UNSET;
    handle->baseAddress->INT_ENABLE.reg = 0x00000000;
    handle->baseAddress->MAGIC_LOCK = FLASH_KEY_REGISTER_LOCK_VALUE; /* Locking Key Registers */
    return BASE_STATUS_OK;
}

/**
  * @brief Registering the Callback Function of the Flash Module.
  * @param handle FLASH handle.
  * @param pcallback Pointer to the callback function.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_FLASH_RegisterCallback(FLASH_Handle *handle, FLASH_CallbackFunType pcallback)
{
    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    handle->FlashCallBack = pcallback;
    return BASE_STATUS_OK;
}

/**
 * @brief blocking write error handle.
 * @param handle FLASH handle.
 * @retval None.
 */
static void FLASH_WritteBlockingErrorHandle(FLASH_Handle *handle)
{
    handle->baseAddress->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_SET;
    handle->state = FLASH_STATE_READY;
}

/**
  * @brief Write the flash memory in blocking mode.
  * @param handle FLASH handle.
  * @param srcAddr Start address of the data buffer to be written.
  * @param destAddr Start address of the flash to be written.The address must be aligned with the minimum writable unit.
  * @param srcLen Length of data to be written,unit:bytes.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_FLASH_WriteBlocking(FLASH_Handle *handle, unsigned int srcAddr,
                                        unsigned int destAddr, const unsigned int srcLen)
{
    BASE_StatusType ret;
    unsigned int dataLeft;
    unsigned int dataLast;
    unsigned int i;
    unsigned int currentLen;
    unsigned int currentWords;

    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    FLASH_PARAM_CHECK_WITH_RET(handle->state == FLASH_STATE_READY, BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET((destAddr % FLASH_MIN_PGM_BYTES_SIZE) == 0, BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(destAddr < FLASH_MAX_SIZE, BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(srcLen > 0 && srcLen <= (FLASH_MAX_SIZE - destAddr), BASE_STATUS_ERROR);

    handle->state = FLASH_STATE_PGM;
    handle->destAddr = destAddr / FLASH_ONE_WORD_BYTES_SIZE;
    handle->srcAddr = srcAddr;
    currentWords = FLASH_GetWriteAlignmentWords(handle);
    if (srcLen > (currentWords * FLASH_ONE_WORD_BYTES_SIZE) && currentWords > 0) {
        ret = FLASH_WriteWords(handle, handle->srcAddr, handle->destAddr, currentWords);
        if (ret != BASE_STATUS_OK) {
            FLASH_WritteBlockingErrorHandle(handle);
            return ret;
        }
        handle->srcAddr += currentWords * FLASH_ONE_WORD_BYTES_SIZE;
        handle->destAddr += currentWords;
        currentLen = srcLen - currentWords * FLASH_ONE_WORD_BYTES_SIZE;
    } else {
        currentLen = srcLen;
    }

    for (i = 0; i < currentLen / (FLASH_MAX_PGM_WORD_SIZE * FLASH_ONE_WORD_BYTES_SIZE); i++) {
        ret = FLASH_WriteWords(handle, handle->srcAddr, handle->destAddr, FLASH_MAX_PGM_WORD_SIZE);
        if (ret != BASE_STATUS_OK) {
            FLASH_WritteBlockingErrorHandle(handle);
            return ret;
        }
        handle->srcAddr += FLASH_MAX_PGM_WORD_SIZE * FLASH_ONE_WORD_BYTES_SIZE;
        handle->destAddr += FLASH_MAX_PGM_WORD_SIZE;
    }

    dataLeft = (currentLen % (FLASH_MAX_PGM_WORD_SIZE * FLASH_ONE_WORD_BYTES_SIZE));
    dataLast = dataLeft % FLASH_ONE_WORD_BYTES_SIZE;
    if (dataLeft > 0) {
        if (dataLast > 0) {
            ret = FLASH_WriteWords(handle, handle->srcAddr, handle->destAddr, dataLeft / FLASH_ONE_WORD_BYTES_SIZE + 1);
        } else {
            ret = FLASH_WriteWords(handle, handle->srcAddr, handle->destAddr, dataLeft / FLASH_ONE_WORD_BYTES_SIZE);
        }
    }

    FLASH_WritteBlockingErrorHandle(handle);
    return ret;
}

/**
  * @brief WriteErase the flash memory in blocking mode.
  * @param handle FLASH handle.
  * @param eraseMode Erasing mode. The options are chip erasing and page erasing.
  * @param startAddr Start address of the flash to be erase. The address must be aligned with the minimum erasable unit.
  * @param eraseNum Number of pages to be erased.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_FLASH_EraseBlocking(FLASH_Handle *handle, FLASH_EraseMode eraseMode,
                                        FLASH_SectorAddr startAddr, unsigned int eraseNum)
{
    BASE_StatusType ret = BASE_STATUS_OK;
    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    FLASH_PARAM_CHECK_WITH_RET(handle->state == FLASH_STATE_READY, BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(startAddr <= FLASH_PAGE_MAX, BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET((startAddr % FLASH_ONE_PAGE_SIZE == 0) || (eraseMode == FLASH_ERASE_MODE_CHIP),\
                               BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(eraseNum > 0 && eraseNum <= (FLASH_MAX_PAGE_NUM - startAddr / FLASH_ONE_PAGE_SIZE),\
                               BASE_STATUS_ERROR);

    handle->eraseNum = eraseNum;
    handle->destAddr = startAddr / sizeof(unsigned int);
    handle->state = FLASH_STATE_ERASE;

    if (eraseMode == FLASH_ERASE_MODE_CHIP) {
        ret = FLASH_EraseWithMode(handle, handle->destAddr, FLASH_ERASE_MODE_CHIP);
        if (ret != BASE_STATUS_OK) {
            handle->baseAddress->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_SET;
            handle->state = FLASH_STATE_READY;
            return ret;
        }
    } else if (eraseMode == FLASH_ERASE_MODE_PAGE) {
        while (handle->eraseNum) {
            ret = FLASH_EraseHandler(handle);
            if (ret != BASE_STATUS_OK) {
                handle->baseAddress->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_SET;
                handle->state = FLASH_STATE_READY;
                return ret;
            }
        }
    }

    handle->baseAddress->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_SET;
    handle->state = FLASH_STATE_READY;
    return ret;
}

/**
  * @brief Write the flash memory in interrupt mode.
  * @param handle FLASH handle.
  * @param srcAddr Start address of the data buffer to be written.
  * @param destAddr Start address of the flash to be written.The address must be aligned with the minimum writable unit.
  * @param srcLen Length of data to be written,unit:bytes.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_FLASH_WriteIT(FLASH_Handle *handle, unsigned int srcAddr,
                                  unsigned int destAddr, unsigned int srcLen)
{
    unsigned int currentWords;
    BASE_StatusType ret;

    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    FLASH_PARAM_CHECK_WITH_RET(destAddr < FLASH_MAX_SIZE, BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(handle->state == FLASH_STATE_READY, BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET((destAddr % FLASH_MIN_PGM_BYTES_SIZE) == 0, BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(srcLen > 0 && srcLen <= (FLASH_MAX_SIZE - destAddr), BASE_STATUS_ERROR);

    handle->state = FLASH_STATE_PGM;

    handle->destAddr = destAddr / FLASH_ONE_WORD_BYTES_SIZE;
    handle->srcAddr = srcAddr;
    handle->writeLen = srcLen;

    currentWords = FLASH_GetWriteAlignmentWords(handle);
    if (handle->writeLen > (currentWords * FLASH_ONE_WORD_BYTES_SIZE) && currentWords > 0) {
        ret = FLASH_WriteWords(handle, handle->srcAddr, handle->destAddr, currentWords);
        if (ret != BASE_STATUS_OK) {
            handle->state = FLASH_STATE_READY;
            return ret;
        }
        handle->srcAddr += currentWords * FLASH_ONE_WORD_BYTES_SIZE;
        handle->destAddr += currentWords;
        handle->writeLen -= currentWords * FLASH_ONE_WORD_BYTES_SIZE;
    } else {
        ret = FLASH_WriteHandler(handle);
        if (ret != BASE_STATUS_OK) {
            handle->state = FLASH_STATE_READY;
            return ret;
        }
    }
    return BASE_STATUS_OK;
}

/**
  * @brief WriteErase the flash memory in interrupt mode.
  * @param handle FLASH handle.
  * @param eraseMode Erasing mode. The options are chip erasing and page erasing.
  * @param startAddr Start address of the flash to be erase. The address must be aligned with the minimum erasable unit.
  * @param eraseNum Number of pages to be erased.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_FLASH_EraseIT(FLASH_Handle *handle, FLASH_EraseMode eraseMode,
                                  FLASH_SectorAddr startAddr, unsigned int eraseNum)
{
    BASE_StatusType ret = BASE_STATUS_OK;
    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    FLASH_PARAM_CHECK_WITH_RET(startAddr <= FLASH_PAGE_MAX, BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(handle->state == FLASH_STATE_READY, BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET((startAddr % FLASH_ONE_PAGE_SIZE == 0) || (eraseMode == FLASH_ERASE_MODE_CHIP),\
                               BASE_STATUS_ERROR);
    FLASH_PARAM_CHECK_WITH_RET(eraseNum > 0 && eraseNum <= (FLASH_MAX_PAGE_NUM - startAddr / FLASH_ONE_PAGE_SIZE),\
                               BASE_STATUS_ERROR);
    handle->eraseNum = eraseNum;
    handle->state = FLASH_STATE_ERASE;
    handle->destAddr = startAddr / sizeof(unsigned int);

    if (eraseMode == FLASH_ERASE_MODE_CHIP) {
        ret = FLASH_EraseWithMode(handle, handle->destAddr, FLASH_ERASE_MODE_CHIP);
        if (ret != BASE_STATUS_OK) {
            handle->state = FLASH_STATE_READY;
            return ret;
        }
        handle->eraseNum = 0;
    } else if (eraseMode == FLASH_ERASE_MODE_PAGE) {
        ret = FLASH_EraseHandler(handle);
        if (ret != BASE_STATUS_OK) {
            handle->state = FLASH_STATE_READY;
            return ret;
        }
    }
    return BASE_STATUS_OK;
}

/**
  * @brief Interface for reading data from the flash memory.
  * @param handle FLASH handle.
  * @param srcAddr Flash address of the data to be read. The address must be aligned with the minimum readable unit.
  * @param readLen Read Data Length,unit:bytes.
  * @param dataBuff Buffer for storing read data.
  * @param buffLen Buffer size for storing read data,unit:bytes.
  * @retval BASE status type: OK, ERROR, BUSY, TIMEOUT.
  */
BASE_StatusType HAL_FLASH_Read(FLASH_Handle *handle,
                               unsigned int srcAddr,
                               unsigned int readLen,
                               unsigned char *dataBuff,
                               unsigned int buffLen)
{
    unsigned char *ptemp = NULL;
    unsigned int tempLen = readLen;
#ifndef FLASH_PARAM_CHECK
    BASE_FUNC_UNUSED(handle);
#endif
    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    FLASH_ASSERT_PARAM(dataBuff != NULL);
    FLASH_PARAM_CHECK_WITH_RET(srcAddr < FLASH_MAX_SIZE, BASE_STATUS_ERROR);

    ptemp = (unsigned char *)(uintptr_t)srcAddr + FLASH_READ_BASE;
    if (readLen > buffLen) {
        return BASE_STATUS_ERROR;
    }
    while (tempLen > 0) {
        tempLen--;
        *dataBuff++ = *ptemp++;
    }
    return BASE_STATUS_OK;
}

/**
 * @brief Interrupt Processing Write.
 * @param handle FLASH handle.
 * @param status Interrupt status
 * @retval None
 */
static void InterruptWriteHandle(FLASH_Handle *handle, unsigned int status)
{
    /* One operation complete */
    if ((status & FLASH_INT_FINISH_MASK) > 0) {
        if (handle->FlashCallBack != NULL) {
            handle->FlashCallBack(handle, FLASH_WRITE_EVENT_SUCCESS, handle->destAddr);
        }
    }
    /* All operations are complete. */
    if (handle->writeLen == 0) {
        if (handle->FlashCallBack != NULL) {
            handle->FlashCallBack(handle, FLASH_WRITE_EVENT_DONE, handle->destAddr);
        }
        handle->baseAddress->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_SET;
        handle->state = FLASH_STATE_READY;
    } else {
        FLASH_WriteHandler(handle);
    }
}

/**
 * @brief Interrupt processing erase.
 * @param handle FLASH handle.
 * @param status Interrupt status
 * @retval None
 */
static void InterruptEraseHandle(FLASH_Handle *handle, unsigned int status)
{
    /* One operation complete */
    if ((status & FLASH_INT_FINISH_MASK) > 0) {
        if (handle->FlashCallBack != NULL) {
            handle->FlashCallBack(handle, FLASH_ERASE_EVENT_SUCCESS, handle->destAddr);
        }
    }
    /* All operations are complete. */
    if (handle->eraseNum == 0) {
        if (handle->FlashCallBack != NULL) {
            handle->FlashCallBack(handle, FLASH_ERASE_EVENT_DONE, handle->destAddr);
        }
        handle->baseAddress->CACHE_CTRL.BIT.cache_invalid_req = BASE_CFG_SET;
        handle->state = FLASH_STATE_READY;
    } else {
        FLASH_EraseHandler(handle);
    }
}

/**
  * @brief Interrupt Handling Function.
  * @param arg Handle pointers
  * @retval None
  */
void HAL_FLASH_IRQHandler(void *arg)
{
    FLASH_Handle *handle = (FLASH_Handle *)arg;
    unsigned int status;
    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    status = handle->baseAddress->INT_RAW_STATUS.reg;
    handle->baseAddress->INT_CLEAR.reg = status & FLASH_CMD_INTERRUPT_MASK;
    IRQ_ClearN(handle->irqNum);

    if (handle->state == FLASH_STATE_PGM) {
        InterruptWriteHandle(handle, status);
    } else if (handle->state == FLASH_STATE_ERASE) {
        InterruptEraseHandle(handle, status);
    } else {
        ;
    }
}

/**
  * @brief Flash Error interrupt Handling Function.
  * @param arg Handle pointers
  * @retval None
  */
void HAL_FLASH_ErrIRQHandler(void *arg)
{
    FLASH_Handle *handle = (FLASH_Handle *)arg;
    unsigned int status;
    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    status = handle->baseAddress->INT_RAW_STATUS.reg;
    handle->baseAddress->INT_CLEAR.reg = status & FLASH_ERR_INTERRUPT_MASK;
    IRQ_ClearN(handle->errIrqNum);

    if ((status & (FLASH_INT_ERR_ECC_CHK_MASK | FLASH_INT_ERR_ECC_CORR_MASK |
                  FLASH_INT_ERR_AHB_MASK | FLASH_INT_ERR_SMWR_MASK | FLASH_INT_ERR_ILLEGAL_MASK)) > 0) {
        if (handle->FlashCallBack != NULL) {
            switch (handle->state) {
                case FLASH_STATE_PGM :
                    handle->FlashCallBack(handle, FLASH_WRITE_EVENT_FAIL, handle->destAddr);
                    break;
                case FLASH_STATE_ERASE :
                    handle->FlashCallBack(handle, FLASH_ERASE_EVENT_FAIL, handle->destAddr);
                    break;
                default:
                    break;
            }
        }
        handle->state = FLASH_STATE_READY;
    }
}

/**
  * @brief FLASH Interrupt Service Function.
  * @param irqNum FLASH interrupt number.
  * @param handle FLASH handle.
  * @retval None
  */
void HAL_FLASH_IRQService(FLASH_Handle *handle)
{
    FLASH_ASSERT_PARAM(handle != NULL);
    FLASH_ASSERT_PARAM(IsEFCInstance(handle->baseAddress));
    IRQ_Register(handle->irqNum, HAL_FLASH_IRQHandler, handle);
    IRQ_Register(handle->errIrqNum, HAL_FLASH_ErrIRQHandler, handle);
}