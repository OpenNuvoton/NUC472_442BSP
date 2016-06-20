/******************************************************************************
 * @file     DataFlashProg.h
 * @brief    M451 series data flash programming driver header
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __DATA_FLASH_PROG_H__
#define __DATA_FLASH_PROG_H__

#define MASS_STORAGE_OFFSET       0x00020000  /* To avoid the code to write APROM */
#define DATA_FLASH_STORAGE_SIZE   (64*1024)  /* Configure the DATA FLASH storage size. To pass USB-IF MSC Test, it needs > 64KB */
#define FLASH_PAGE_SIZE           2048
#define BUFFER_PAGE_SIZE          512

void DataFlashWrite(uint32_t addr, uint32_t size, uint32_t buffer);
void DataFlashRead(uint32_t addr, uint32_t size, uint32_t buffer);

#endif  /* __DATA_FLASH_PROG_H__ */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
