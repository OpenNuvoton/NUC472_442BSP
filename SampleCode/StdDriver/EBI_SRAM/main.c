/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 9 $
 * $Date: 15/04/20 10:42a $
 * @brief    Demo for using EBI interface with SRAM module
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"

extern int32_t EbiDataTypeAccess(uint8_t eBank, uint32_t u32Size);
extern int32_t EbiButterfly(uint8_t eBank, uint32_t u32Size);
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

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
    CLK_EnableModuleClock(EBI_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

    /* Configure EBI multi-function pins */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA10MFP_Msk) ) | SYS_GPA_MFPH_PA10MFP_EBI_A20;    /* A20. =   PA10 */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA9MFP_Msk) ) | SYS_GPA_MFPH_PA9MFP_EBI_A19;      /* A19. =   PA9 */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA8MFP_Msk) ) | SYS_GPA_MFPH_PA8MFP_EBI_A18;      /* A18. =   PA8 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA7MFP_Msk) ) | SYS_GPA_MFPL_PA7MFP_EBI_A17;      /* A17. =   PA7 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA6MFP_Msk) ) | SYS_GPA_MFPL_PA6MFP_EBI_A16;      /* A16. =   PA6 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB13MFP_Msk) ) | SYS_GPB_MFPH_PB13MFP_EBI_AD15;   /* AD15 =   PB13 */

    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB12MFP_Msk) ) | SYS_GPB_MFPH_PB12MFP_EBI_AD14;   /* AD14 =   PB12 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB11MFP_Msk) ) | SYS_GPB_MFPH_PB11MFP_EBI_AD13;   /* AD13 =   PB11 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB10MFP_Msk) ) | SYS_GPB_MFPH_PB10MFP_EBI_AD12;   /* AD12 =   PB10 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB9MFP_Msk) ) | SYS_GPB_MFPH_PB9MFP_EBI_AD11;     /* AD11 =   PB9 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB8MFP_Msk) ) | SYS_GPB_MFPH_PB8MFP_EBI_AD10;     /* AD10 =   PB8 */

    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB7MFP_Msk) ) | SYS_GPB_MFPL_PB7MFP_EBI_AD9;      /* AD9 =    PB7 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB6MFP_Msk) ) | SYS_GPB_MFPL_PB6MFP_EBI_AD8;      /* AD8 =    PB6 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk) ) | SYS_GPB_MFPL_PB5MFP_EBI_AD7;      /* AD7 =    PB5 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk) ) | SYS_GPB_MFPL_PB4MFP_EBI_AD6;      /* AD6 =    PB4 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB3MFP_Msk) ) | SYS_GPB_MFPL_PB3MFP_EBI_AD5;      /* AD5 =    PB3 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB2MFP_Msk) ) | SYS_GPB_MFPL_PB2MFP_EBI_AD4;      /* AD4 =    PB2 */

    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA14MFP_Msk) ) | SYS_GPA_MFPH_PA14MFP_EBI_AD3;    /* AD3. =   PA14 */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA13MFP_Msk) ) | SYS_GPA_MFPH_PA13MFP_EBI_AD2;    /* AD2. =   PA13 */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA12MFP_Msk) ) | SYS_GPA_MFPH_PA12MFP_EBI_AD1;    /* AD1. =   PA12 */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA11MFP_Msk) ) | SYS_GPA_MFPH_PA11MFP_EBI_AD0;    /* AD0. =   PA11 */

    SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE6MFP_Msk) ) | SYS_GPE_MFPL_PE6MFP_EBI_nWR;      /* PE.6 =   nWR */
    SYS->GPE_MFPL = (SYS->GPE_MFPL & (~SYS_GPE_MFPL_PE7MFP_Msk) ) | SYS_GPE_MFPL_PE7MFP_EBI_nRD;      /* PE.7 =   nRD */
    SYS->GPE_MFPH = (SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE8MFP_Msk) ) | SYS_GPE_MFPH_PE8MFP_EBI_ALE;      /* PE.8 =   ALE */
    SYS->GPE_MFPH = (SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE9MFP_Msk) ) | SYS_GPE_MFPH_PE9MFP_EBI_nWRH;     /* PE.9 =   WRH */
    SYS->GPE_MFPH = (SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE10MFP_Msk) ) | SYS_GPE_MFPH_PE10MFP_EBI_nWRL;   /* PE.10 =  WRL */

    SYS->GPE_MFPH = (SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE11MFP_Msk) ) | SYS_GPE_MFPH_PE11MFP_EBI_nCS0;   /* PE.11 = nCS0 */
    SYS->GPE_MFPH = (SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE12MFP_Msk) ) | SYS_GPE_MFPH_PE12MFP_EBI_nCS1;   /* PE.12 = nCS1 */
    SYS->GPE_MFPH = (SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE13MFP_Msk) ) | SYS_GPE_MFPH_PE13MFP_EBI_nCS2;   /* PE.13 = nCS2 */
    SYS->GPE_MFPH = (SYS->GPE_MFPH & (~SYS_GPE_MFPH_PE14MFP_Msk) ) | SYS_GPE_MFPH_PE14MFP_EBI_nCS3;   /* PE.14 = nCS3 */

    /* Lock protected registers */
    SYS_LockReg();
}

void EBI_Init(void)
{
    const uint32_t u32Timing = 0x21C;

    /* Open EBI interface */
    EBI_Open(EBI_BANK0, EBI_BUSWIDTH_16BIT, EBI_TIMING_NORMAL, EBI_SEPARATEMODE_DISABLE, EBI_CS_ACTIVE_LOW);
    EBI_Open(EBI_BANK1, EBI_BUSWIDTH_16BIT, EBI_TIMING_NORMAL, EBI_SEPARATEMODE_DISABLE, EBI_CS_ACTIVE_LOW);
    EBI_Open(EBI_BANK2, EBI_BUSWIDTH_16BIT, EBI_TIMING_NORMAL, EBI_SEPARATEMODE_DISABLE, EBI_CS_ACTIVE_LOW);
    EBI_Open(EBI_BANK3, EBI_BUSWIDTH_16BIT, EBI_TIMING_NORMAL, EBI_SEPARATEMODE_DISABLE, EBI_CS_ACTIVE_LOW);

    /* Configure EBI timing */
    EBI_SetBusTiming(EBI_BANK0, u32Timing, EBI_MCLKDIV_2);
    EBI_SetBusTiming(EBI_BANK1, u32Timing, EBI_MCLKDIV_2);
    EBI_SetBusTiming(EBI_BANK2, u32Timing, EBI_MCLKDIV_2);
    EBI_SetBusTiming(EBI_BANK3, u32Timing, EBI_MCLKDIV_2);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    int32_t i32Result;
    uint8_t u8Pass = 1;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+-------------------------------------------------------+\n");
    printf("|           EBI Driver Sample Code BS616LV40 SRAM       |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init EBI */
    EBI_Init();

    /* EBI buttery fly test */
    i32Result = EbiButterfly(EBI_BANK0, 128*1024);
    if(i32Result < 0) u8Pass = 0;
    EbiButterfly(EBI_BANK1, 128*1024);
    if(i32Result < 0) u8Pass = 0;
    EbiButterfly(EBI_BANK2, 128*1024);
    if(i32Result < 0) u8Pass = 0;
    EbiButterfly(EBI_BANK3, 128*1024);
    if(i32Result < 0) u8Pass = 0;

    /* EBI data type test */
    EbiDataTypeAccess(EBI_BANK0, 128*1024);
    if(i32Result < 0) u8Pass = 0;
    EbiDataTypeAccess(EBI_BANK1, 128*1024);
    if(i32Result < 0) u8Pass = 0;
    EbiDataTypeAccess(EBI_BANK2, 128*1024);
    if(i32Result < 0) u8Pass = 0;
    EbiDataTypeAccess(EBI_BANK3, 128*1024);
    if(i32Result < 0) u8Pass = 0;

    if(u8Pass != 1)
        printf("Test fail ...\n");

    while(1);
}
