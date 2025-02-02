/**************************************************************************//**
 * @file     main.c
 * @version  V1.10
 * $Revision: 8 $
 * $Date: 15/11/19 10:48a $
 * @brief    Show Crypto IP Triple DES CBC mode encrypt/decrypt function
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC472_442.h"


#define PLLCON_SETTING      CLK_PLLCTL_84MHz_HXT
#define PLL_CLOCK           84000000

uint32_t PllClock         = PLL_CLOCK;


/* TDES Key:  1e4678a17f2c8a33 800e15ac47891a4c a011453291c23340 */
uint32_t au8MyTDESKey[3][2] =
{
    { 0x1e4678a1, 0x7f2c8a33 },
    { 0x800e15ac, 0x47891a4c },
    { 0xa0114532, 0x91c23340 }
};

uint32_t au32MyTDESIV[2] = {  0x00000000, 0x00000000 };


#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t au8InputData[] =
{
#else
uint8_t au8InputData[] __attribute__((aligned(4))) =
{
#endif
    0x12, 0x34, 0x56, 0x78, 0xAB, 0xCD, 0xEF
};


#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t au8OutputData[1024];
#else
uint8_t au8OutputData[1024] __attribute__((aligned(4)));
#endif

static volatile int  g_TDES_done;

void CRYPTO_IRQHandler()
{
    if (TDES_GET_INT_FLAG())
    {
        g_TDES_done = 1;
        TDES_CLR_INT_FLAG();
    }
}


void  dump_buff_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02x ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
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
    CLK_EnableModuleClock(CRPT_MODULE);
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
    int  tout;

    /* Lock protected registers */
    if(SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART0 for printf */
    UART0_Init();

    printf("+--------------------------------------------+\n");
    printf("|           Crypto TDES Driver Sample Code   |\n");
    printf("+--------------------------------------------+\n");

    NVIC_EnableIRQ(CRPT_IRQn);
    TDES_ENABLE_INT();

    /*---------------------------------------
     *  TDES CBC mode encrypt
     *---------------------------------------*/
    //TDES_Open(0, 1, TDES_MODE_CBC, TDES_IN_OUT_SWAP);
    TDES_Open(0, 1, 1, 1, TDES_MODE_CBC, TDES_IN_OUT_WHL_SWAP);
    TDES_SetKey(0, au8MyTDESKey);
    TDES_SetInitVect(0, au32MyTDESIV[0], au32MyTDESIV[1]);
    TDES_SetDMATransfer(0, (uint32_t)au8InputData, (uint32_t)au8OutputData, sizeof(au8InputData));

    g_TDES_done = 0;
    TDES_Start(0, CRYPTO_DMA_ONE_SHOT);

    tout = SystemCoreClock;
    while ((tout-- > 0) && (!g_TDES_done)) {}
    if (tout <= 0)
    {
        printf("TDES H/W time-out!\n");
        while (1);
    }

    printf("TDES encrypt done.\n\n");
    dump_buff_hex(au8OutputData, sizeof(au8InputData));

    /*---------------------------------------
     *  TDES CBC mode decrypt
     *---------------------------------------*/
    TDES_Open(0, 0, 1, 1, TDES_MODE_CBC, TDES_IN_OUT_WHL_SWAP);
    TDES_SetKey(0, au8MyTDESKey);
    TDES_SetInitVect(0, au32MyTDESIV[0], au32MyTDESIV[1]);
    TDES_SetDMATransfer(0, (uint32_t)au8OutputData, (uint32_t)au8InputData, sizeof(au8InputData));

    g_TDES_done = 0;
    TDES_Start(0, CRYPTO_DMA_ONE_SHOT);

    tout = SystemCoreClock;
    while ((tout-- > 0) && (!g_TDES_done)) {}
    if (tout <= 0)
    {
        printf("TDES H/W time-out!\n");
        while (1);
    }

    printf("TDES decrypt done.\n\n");
    dump_buff_hex(au8InputData, sizeof(au8InputData));

    while (1);
}
