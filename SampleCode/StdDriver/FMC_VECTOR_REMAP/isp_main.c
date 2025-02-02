/**************************************************************************//**
 * @file     isp_main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 14/10/02 6:49p $
 * @brief    Show how to branch programs between LDROM, APROM start page,
 *           and APROM other page
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"
#include "map.h"


#define PLLCON_SETTING      CLK_PLLCTL_84MHz_HXT
#define PLL_CLOCK           84000000

uint32_t PllClock = PLL_CLOCK;


#ifdef __ARMCC_VERSION
void __set_SP(uint32_t _sp)
{
    __set_MSP(_sp);
}
#endif


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


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    int             u8Item;
    FUNC_PTR        *func;
    volatile int    loop;
    uint32_t        u32Data;

    /* Lock protected registers */
    if (SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART0 for printf */
    UART0_Init();

    /* Enable FMC ISP function */
    SYS_UnlockReg();
    FMC_Open();

    do
    {
        printf("\n\n\n");
        printf("+--------------------------------------------+\n");
        printf("|      ISP program running on APROM %dK      |\n", ISP_CODE_BASE/1024);
        printf("+--------------------------------------------+\n");
        printf("|               Program Select               |\n");
        printf("+--------------------------------------------|\n");
        printf("| [0] Run LD boot program (on LDROM)         |\n");
        printf("| [1] Run user application                   |\n");
        printf("+--------------------------------------------+\n");
        printf("Please select...");
        u8Item = getchar();
        printf("%c\n", u8Item);

        switch (u8Item)
        {
        case '0':
            u32Data = FMC_Read(LD_BOOT_CODE_ENTRY);
            func =  (FUNC_PTR *)FMC_Read(LD_BOOT_CODE_ENTRY+4);
            printf("branch_to address 0x%x\n", (int)func);
            printf("\n\nChange VECMAP and branch to ld boot code...\n");

            FMC_SetVectorPageAddr(LD_BOOT_CODE_ENTRY);
#if defined (__GNUC__) && !defined(__ARMCC_VERSION)  /* for GNU C compiler */
            asm("msr msp, %0" : : "r" (u32Data));
#else
           __set_SP(u32Data);
#endif
            func();
            break;

        case '1':
            u32Data = FMC_Read(USER_AP_ENTRY);
            func =  (FUNC_PTR *)FMC_Read(USER_AP_ENTRY+4);
            printf("branch_to address 0x%x\n", (int)func);
            printf("\n\nChange VECMAP and branch to user application...\n");
            while (!UART_IS_TX_EMPTY(UART0));

            FMC_SetVectorPageAddr(USER_AP_ENTRY);
#if defined (__GNUC__) && !defined(__ARMCC_VERSION)  /* for GNU C compiler */
            asm("msr msp, %0" : : "r" (u32Data));
#else
           __set_SP(u32Data);
#endif
            func();
            break;

        default :
            continue;
        }
    }
    while (1);
}


