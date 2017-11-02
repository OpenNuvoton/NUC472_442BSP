/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 15/12/08 9:41a $
 * @brief    MP3 player sample plays MP3 files stored on SD memory card
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC472_442.h"
#include "config.h"
#include "diskio.h"
#include "ff.h"

uint32_t volatile u32BuffPos = 0;
FATFS FatFs[_VOLUMES];               /* File system object for logical drive */

#ifdef __ICCARM__
#pragma data_alignment=32
BYTE Buff[16] ;                   /* Working buffer */
DMA_DESC_T DMA_DESC[2];
#endif

#ifdef __ARMCC_VERSION
__align(32) BYTE Buff[16] ;       /* Working buffer */
__align(32) DMA_DESC_T DMA_DESC[2];
#endif

uint8_t bAudioPlaying = 0;
extern signed int aPCMBuffer[2][PCM_BUFFER_SIZE];

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

unsigned long get_fattime (void)
{
    unsigned long tmr;

    tmr=0x00000;

    return tmr;
}

void SD_Inits(void)
{
    /* Configure SD multi function pins */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF2MFP_Msk)) | (0x4 << SYS_GPF_MFPL_PF2MFP_Pos); // SD0_CD: GPE_5
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF3MFP_Msk)) | (0x4 << SYS_GPF_MFPL_PF3MFP_Pos); // SD0_CMD: GPE_6
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF4MFP_Msk)) | (0x4 << SYS_GPF_MFPL_PF4MFP_Pos); // SD0_CLK: GPE_7
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF5MFP_Msk)) | (0x4 << SYS_GPF_MFPL_PF5MFP_Pos); // SD0_DAT3: GPE_8
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF6MFP_Msk)) | (0x4 << SYS_GPF_MFPL_PF6MFP_Pos); // SD0_DAT2: GPE_9
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF7MFP_Msk))| (0x4 << SYS_GPF_MFPL_PF7MFP_Pos); // SD0_DAT1: GPE_10
    SYS->GPF_MFPH = (SYS->GPF_MFPH & ~(SYS_GPF_MFPH_PF8MFP_Msk))| (0x4 << SYS_GPF_MFPH_PF8MFP_Pos); // SD0_DAT0: GPE_11

    // power (H.2)
    SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH2MFP_Msk));
    PH->MODE = (PH->MODE & ~GPIO_MODE_MODE2_Msk) | ( GPIO_MODE_OUTPUT << GPIO_MODE_MODE2_Pos);
    PH2 = 0;

    CLK_EnableModuleClock(SDH_MODULE);
    CLK_SetModuleClock(SDH_MODULE, CLK_CLKSEL0_SDHSEL_PLL, CLK_CLKDIV0_SDH(1));
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
    CLK_EnableModuleClock(I2C3_MODULE);
    CLK_EnableModuleClock(I2S1_MODULE);
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Setup SD interface */
    SD_Inits();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

    /* Set GPH3,GPH4 multi-function pins for I2C3 */
    SYS->GPH_MFPL |= SYS_GPH_MFPL_PH3MFP_I2C3_SCL | SYS_GPH_MFPL_PH4MFP_I2C3_SDA ;

    /* Set multi function pin for I2S1 */
    /* GPG7, GPG8, GPG9, GPI11, GPI12 */
    SYS->GPG_MFPL = (SYS->GPG_MFPL & ~SYS_GPG_MFPL_PG7MFP_Msk) | SYS_GPG_MFPL_PG7MFP_I2S1_MCLK;
    SYS->GPG_MFPH = (SYS->GPG_MFPH & ~(SYS_GPG_MFPH_PG8MFP_Msk | SYS_GPG_MFPH_PG9MFP_Msk)) | (SYS_GPG_MFPH_PG8MFP_I2S1_DO | SYS_GPG_MFPH_PG9MFP_I2S1_DI);
    SYS->GPI_MFPH = (SYS->GPI_MFPH & ~(SYS_GPI_MFPH_PI11MFP_Msk | SYS_GPI_MFPH_PI12MFP_Msk)) | (SYS_GPI_MFPH_PI11MFP_I2S1_BCLK | SYS_GPI_MFPH_PI12MFP_I2S1_LRCK);

    /* Lock protected registers */
    SYS_LockReg();
}

void I2C3_Init(void)
{
    /* Open I2C3 and set clock to 100k */
    I2C_Open(I2C3, 100000);

    /* Set I2C3 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C3, 0, 0x15, I2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C3, 1, 0x35, I2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C3, 2, 0x55, I2C_GCMODE_DISABLE);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C3, 3, 0x75, I2C_GCMODE_DISABLE);   /* Slave Address : 0x75 */
}

// Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    DMA_DESC[0].ctl = ((PCM_BUFFER_SIZE-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_INC|PDMA_DAR_FIX|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
    DMA_DESC[0].endsrc = (uint32_t)&aPCMBuffer[0][0] + PCM_BUFFER_SIZE * 4;
    DMA_DESC[0].enddest = (uint32_t)&I2S1->TX;
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1] - (PDMA->SCATBA);

    DMA_DESC[1].ctl = ((PCM_BUFFER_SIZE-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_INC|PDMA_DAR_FIX|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
    DMA_DESC[1].endsrc = (uint32_t)&aPCMBuffer[1][0] + PCM_BUFFER_SIZE * 4;
    DMA_DESC[1].enddest = (uint32_t)&I2S1->TX;
    DMA_DESC[1].offset = (uint32_t)&DMA_DESC[0] - (PDMA->SCATBA);

    PDMA_Open(1 << 2);
    PDMA_SetTransferMode(2, PDMA_I2S1_TX, 1, (uint32_t)&DMA_DESC[0]);

    PDMA_EnableInt(2, 0);
    NVIC_EnableIRQ(PDMA_IRQn);
}

void PDMA_Reset_SCTable(uint8_t id)
{
    DMA_DESC[id].ctl |= PDMA_OP_SCATTER;
    DMA_DESC[id].ctl |= ((PCM_BUFFER_SIZE-1)<<PDMA_DSCT_CTL_TXCNT_Pos);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+------------------------------------------------------------------------+\n");
    printf("|                   MP3 Player Sample with WAU8822 Codec                 |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf(" Please put MP3 files on SD card \n");

    printf("rc=%d\n", (WORD)disk_initialize(0));
    disk_read(0, Buff, 2, 1);
    f_mount(&FatFs[0], 0, 1);

    /* Init I2C3 to access WAU8822 */
    I2C3_Init();

    // select source from HXT(12MHz)
    CLK_SetModuleClock(I2S1_MODULE, CLK_CLKSEL3_I2S1SEL_HXT, 0);

    // Plug-In DET, enable sound output (I3)
    SYS->GPI_MFPL = (SYS->GPI_MFPL & ~(SYS_GPI_MFPL_PI3MFP_Msk));
    GPIO_SetMode(GPI, BIT3, GPIO_MODE_OUTPUT);
    PI3 = 0;

    MP3Player();

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
