/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 10 $
 * $Date: 15/06/22 9:35a $
 * @brief    Access SPI flash using SPI dual mode.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"

#define TEST_NUMBER 1   /* page numbers */
#define TEST_LENGTH 256 /* length */

#define SPI_FLASH_PORT  SPI2

uint8_t SrcArray[TEST_LENGTH];
uint8_t DestArray[TEST_LENGTH];

uint16_t SpiFlash_ReadMidDid(void)
{
    uint8_t u8RxData[6], u8IDCnt = 0;

    // /CS: active
    SPI_SET_SS0_LOW(SPI_FLASH_PORT);

    // send Command: 0x90, Read Manufacturer/Device ID
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x90);

    // send 24-bit '0', dummy
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // receive 16-bit
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS0_HIGH(SPI_FLASH_PORT);

    while(!SPI_GET_RX_FIFO_EMPTY_FLAG(SPI_FLASH_PORT))
        u8RxData[u8IDCnt ++] = SPI_READ_RX(SPI_FLASH_PORT);

    return ( (u8RxData[4]<<8) | u8RxData[5] );
}

void SpiFlash_ChipErase(void)
{
    // /CS: active
    SPI_SET_SS0_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS0_HIGH(SPI_FLASH_PORT);

    //////////////////////////////////////////

    // /CS: active
    SPI_SET_SS0_LOW(SPI_FLASH_PORT);

    // send Command: 0xC7, Chip Erase
    SPI_WRITE_TX(SPI_FLASH_PORT, 0xC7);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS0_HIGH(SPI_FLASH_PORT);

    SPI_ClearRxFIFO(SPI0);
}

uint8_t SpiFlash_ReadStatusReg(void)
{
    // /CS: active
    SPI_SET_SS0_LOW(SPI_FLASH_PORT);

    // send Command: 0x05, Read status register
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x05);

    // read status
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS0_HIGH(SPI_FLASH_PORT);

    // skip first rx data
    SPI_READ_RX(SPI_FLASH_PORT);

    return (SPI_READ_RX(SPI_FLASH_PORT) & 0xff);
}

void SpiFlash_WriteStatusReg(uint8_t u8Value)
{
    // /CS: active
    SPI_SET_SS0_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS0_HIGH(SPI_FLASH_PORT);

    ///////////////////////////////////////

    // /CS: active
    SPI_SET_SS0_LOW(SPI_FLASH_PORT);

    // send Command: 0x01, Write status register
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x01);

    // write status
    SPI_WRITE_TX(SPI_FLASH_PORT, u8Value);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS0_HIGH(SPI_FLASH_PORT);
}

void SpiFlash_WaitReady(void)
{
    uint8_t ReturnValue;

    do {
        ReturnValue = SpiFlash_ReadStatusReg();
        ReturnValue = ReturnValue & 1;
    } while(ReturnValue!=0); // check the BUSY bit
}

void SpiFlash_NormalPageProgram(uint32_t StartAddress, uint8_t *u8DataBuffer)
{
    uint32_t i = 0;

    // /CS: active
    SPI_SET_SS0_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS0_HIGH(SPI_FLASH_PORT);


    // /CS: active
    SPI_SET_SS0_LOW(SPI_FLASH_PORT);

    // send Command: 0x02, Page program
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x02);

    // send 24-bit start address
    SPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress>>16) & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress>>8)  & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, StartAddress       & 0xFF);

    // write data
    while(1) {
        if(!SPI_GET_TX_FIFO_FULL_FLAG(SPI_FLASH_PORT)) {
            SPI_WRITE_TX(SPI_FLASH_PORT, u8DataBuffer[i++]);
            if(i >= 255) break;
        }
    }

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS0_HIGH(SPI_FLASH_PORT);

    SPI_ClearRxFIFO(SPI_FLASH_PORT);
}

void SpiFlash_DualFastRead(uint32_t StartAddress, uint8_t *u8DataBuffer)
{
    uint32_t i;

    // /CS: active
    SPI_SET_SS0_LOW(SPI_FLASH_PORT);

    // Command: 0x3B, Fast Read dual data
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x3B);

    // send 24-bit start address
    SPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress>>16) & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, (StartAddress>>8)  & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, StartAddress       & 0xFF);

    // dummy byte
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // clear RX buffer
    SPI_ClearRxFIFO(SPI_FLASH_PORT);

    // enable SPI dual IO mode and set direction to input
    SPI_ENABLE_DUAL_INPUT_MODE(SPI_FLASH_PORT);

    // read data
    for(i=0; i<256; i++) {
        SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
        while(SPI_IS_BUSY(SPI_FLASH_PORT));
        u8DataBuffer[i] = SPI_READ_RX(SPI_FLASH_PORT);
    }

    // wait tx finish
    while(SPI_IS_BUSY(SPI_FLASH_PORT));

    // /CS: de-active
    SPI_SET_SS0_HIGH(SPI_FLASH_PORT);

    SPI_DISABLE_DUAL_MODE(SPI_FLASH_PORT);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT,CLK_CLKDIV0_HCLK(1));

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency */
    CLK->PLLCTL = CLK_PLLCTL_84MHz_HXT;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL,CLK_CLKDIV0_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(SPI2_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

    /* SPI2: SS0=GPH_5, MOSI0=GPH_8, MISO0=GPH_7, CLK=GPH_6, MISO1=GPH_9, MOSI11=GPH10 */
    SYS->GPH_MFPL |= (SYS_GPH_MFPL_PH5MFP_SPI2_SS0 | SYS_GPH_MFPL_PH6MFP_SPI2_CLK | SYS_GPH_MFPL_PH7MFP_SPI2_MISO0);
    SYS->GPH_MFPH |= (SYS_GPH_MFPH_PH8MFP_SPI2_MOSI0 | SYS_GPH_MFPH_PH9MFP_SPI2_MISO1 | SYS_GPH_MFPH_PH10MFP_SPI2_MOSI1);

    /* Lock protected registers */
    SYS_LockReg();
}

/* Main */
int main(void)
{
    uint32_t u32ByteCount, u32FlashAddress, u32PageNumber;
    uint32_t nError = 0;
    uint16_t u16ID;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Configure SPI_FLASH_PORT as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 2MHz */
    SPI_Open(SPI_FLASH_PORT, SPI_MASTER, SPI_MODE_0, 8, 2000000);

    /* Enable the automatic hardware slave select function. Select the SS0 pin and configure as low-active. */
    SPI_EnableAutoSS(SPI_FLASH_PORT, SPI_SS0, SPI_SS_ACTIVE_LOW);
    SPI_TRIGGER(SPI_FLASH_PORT);

    printf("\n+------------------------------------------------------------------------+\n");
    printf("|             NUC472/NUC442 SPI Dual Mode with Flash Sample Code         |\n");
    printf("+------------------------------------------------------------------------+\n");

    /* Wait ready */
    SpiFlash_WaitReady();

    if((u16ID = SpiFlash_ReadMidDid()) != 0x1C14) {
        printf("Wrong ID, 0x%x\n", u16ID);
        return -1;
    } else
        printf("Flash found: EN25QH16 ...\n");

    printf("Erase chip ...");

    /* Erase SPI flash */
    SpiFlash_ChipErase();

    /* Wait ready */
    SpiFlash_WaitReady();

    printf("[OK]\n");

    /* init source data buffer */
    for(u32ByteCount=0; u32ByteCount<TEST_LENGTH; u32ByteCount++) {
        SrcArray[u32ByteCount] = u32ByteCount;
    }

    printf("Start to normal write data to Flash ...");
    /* Program SPI flash */
    u32FlashAddress = 0;
    for(u32PageNumber=0; u32PageNumber<TEST_NUMBER; u32PageNumber++) {
        /* page program */
        SpiFlash_NormalPageProgram(u32FlashAddress, SrcArray);
        SpiFlash_WaitReady();
        u32FlashAddress += 0x100;
    }

    printf("[OK]\n");

    /* clear destination data buffer */
    for(u32ByteCount=0; u32ByteCount<TEST_LENGTH; u32ByteCount++) {
        DestArray[u32ByteCount] = 0;
    }

    printf("Dual Read & Compare ...");

    /* Read SPI flash */
    u32FlashAddress = 0;
    for(u32PageNumber=0; u32PageNumber<TEST_NUMBER; u32PageNumber++) {
        /* page read */
        SpiFlash_DualFastRead(u32FlashAddress, DestArray);
        u32FlashAddress += 0x100;

        for(u32ByteCount=0; u32ByteCount<TEST_LENGTH; u32ByteCount++) {
            if(DestArray[u32ByteCount] != SrcArray[u32ByteCount])
                nError ++;
        }
    }

    if(nError == 0)
        printf("[OK]\n");
    else
        printf("[FAIL]\n");

    while(1);
}


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


