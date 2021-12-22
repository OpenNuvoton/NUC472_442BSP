/**************************************************************************//**
 * @file     mp_sub.c
 * @version  V1.10
 * $Revision: 7 $
 * $Date: 15/11/20 10:24a $
 * @brief    Show FMC ISP multi-word program function. The loader.bin will
 *           load fmc_multi_word_prog.bin to SRAM and execute it.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"


#define PLLCON_SETTING      CLK_PLLCTL_84MHz_HXT
#define PLL_CLOCK           84000000

uint32_t PllClock         = PLL_CLOCK;


uint32_t    page_buff[FMC_FLASH_PAGE_SIZE/4];


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

    /* Switch HCLK clock source to XTAL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT,CLK_CLKDIV0_HCLK(1));

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency */
    CLK->PLLCTL = PLLCON_SETTING;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UARTSEL_HXT,CLK_CLKDIV0_UART(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

    /* Lock protected registers */
    SYS_LockReg();
}


void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART0->LINE = 0x07;
    UART0->BAUD = 0x30000066;   /* 12MHz reference clock input, for 115200 */
}


void init_test_pattern()
{
    int         i;

    for (i =0; i < FMC_FLASH_PAGE_SIZE/4; i += 2)
    {
        page_buff[i] = 0x5A5A5A5A;
        page_buff[i+1] = 0xA5A5A5A5;
    }
}


int  do_page_program(uint32_t u32PageAddr)
{
    uint32_t    i, idx, u32MPADR, u32Data;
    int32_t     tout;

    printf("Page program 0x%x\n", u32PageAddr);

    if (FMC_Erase(u32PageAddr) < 0)
    {
        printf("Erase page 0x%x failed!\n", u32PageAddr);
        return -1;
    }

    /*
     *  One multi-program can burst program maximum 512 bytes (128 words).
     *  A 2K page required 4 multi-program burst at least.
     */
    for (u32MPADR = u32PageAddr; u32MPADR < u32PageAddr+FMC_FLASH_PAGE_SIZE; u32MPADR += 512)
    {
        idx = (u32MPADR - u32PageAddr)/4;

        // program the first 4 words
        FMC->ISPADDR = u32MPADR;
        FMC->ISPCMD = ISP_ISPCMD_MULTI_WRITE;
        FMC->MPDAT0 = page_buff[idx];
        FMC->MPDAT1 = page_buff[idx+1];
        FMC->MPDAT2 = page_buff[idx+2];
        FMC->MPDAT3 = page_buff[idx+3];
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

        // program the subsequent 124 words
        for (i = idx+4; i < idx+128; i += 4)
        {
            tout = FMC_TIMEOUT_WRITE;
            while ((tout-- > 0) && (FMC->MPSTS & (FMC_MPSTS_D0_Msk | FMC_MPSTS_D1_Msk))) ;
            if (tout <= 0)
            {
                printf("Wait D0/D1 time-out!\n");
                return -1;
            }

            FMC->MPDAT0 = page_buff[i];
            FMC->MPDAT1 = page_buff[i+1];

            tout = FMC_TIMEOUT_WRITE;
            while ((tout-- > 0) && (FMC->MPSTS & (FMC_MPSTS_D2_Msk | FMC_MPSTS_D3_Msk))) ;
            if (tout <= 0)
            {
                printf("Wait D2/D3 time-out!\n");
                return -1;
            }

            FMC->MPDAT2 = page_buff[i+2];
            FMC->MPDAT3 = page_buff[i+3];
        }

        tout = FMC_TIMEOUT_WRITE;
        while ((tout-- > 0) && (FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk)) ;
        if (tout <= 0)
            return -1;
    }

    printf("Verify...\n");

    for (i = 0; i < FMC_FLASH_PAGE_SIZE; i += 4)
    {
        u32Data = FMC_Read(u32PageAddr+i);

        if (u32Data !=  page_buff[i/4])
        {
            printf("Verify error on 0x%x, read:0x%x, expected:0x%x\n", u32PageAddr+i, u32Data, page_buff[i/4]);
            return -1;
        }
    }
    return 0;
}


int  page_program_test(uint32_t u32startaddr, uint32_t u32EndAddr)
{
    uint32_t    u32PgAdr;

    for (u32PgAdr = u32startaddr; u32PgAdr < u32EndAddr; u32PgAdr += FMC_FLASH_PAGE_SIZE)
    {
        if (do_page_program(u32PgAdr) < 0)
            return -1;
    }
    return 0;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    /* Lock protected registers */
    if (SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\n");
    printf("+-----------------------------------------+\n");
    printf("|                                         |\n");
    printf("|     FMC multi-word program sample       |\n");
    printf("|         (Running on SRAM)               |\n");
    printf("|                                         |\n");
    printf("+-----------------------------------------+\n");

    /* Enable FMC ISP function */
    SYS_UnlockReg();
    FMC_Open();

    init_test_pattern();

    FMC_ENABLE_AP_UPDATE();
    if (page_program_test(FMC_APROM_BASE, FMC_APROM_END) < 0)
        goto test_failed;
    FMC_DISABLE_AP_UPDATE();

    FMC_ENABLE_LD_UPDATE();
    if (page_program_test(FMC_LDROM_BASE, FMC_LDROM_END) < 0)
        goto test_failed;
    FMC_DISABLE_LD_UPDATE();

    printf("\nFMC Page Program Test Completed.\n");

    while (1);

test_failed:
    printf("ISPCTL = 0x%x\n", FMC->ISPCTL);
    printf("ISPSTS = 0x%x\n", FMC->ISPSTS);
    printf("MPSTS = 0x%x\n", FMC->MPSTS);
    while (1);
}







