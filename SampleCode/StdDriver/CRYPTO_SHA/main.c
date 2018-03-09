/**************************************************************************//**
 * @file     main.c
 * @version  V1.10
 * $Revision: 7 $
 * $Date: 14/05/30 5:58p $
 * @brief    Use Crypto IP SHA engine to run through known answer SHA1 test vectors.
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


extern void open_test_vector(void);
extern int  get_next_pattern(void);

extern uint8_t  _au8ShaKey[8192];
extern uint8_t  _au8ShaDigest[64];
extern int      _i32KeyLen;

static int  _i32DigestLength = 0;

static volatile int g_SHA_done;


void CRYPTO_IRQHandler()
{
    if (SHA_GET_INT_FLAG())
    {
        g_SHA_done = 1;
        SHA_CLR_INT_FLAG();
    }
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


int  do_compare(uint8_t *output, uint8_t *expect, int cmp_len)
{
    int   i;

    if (memcmp(expect, output, cmp_len))
    {
        printf("\nMismatch!! - %d\n", cmp_len);
        for (i = 0; i < cmp_len; i++)
            printf("0x%02x    0x%02x\n", expect[i], output[i]);
        return -1;
    }
    return 0;
}

int  run_sha()
{
    uint32_t  au32OutputDigest[8];

    SHA_Open(SHA_MODE_SHA1, SHA_IN_SWAP);

    SHA_SetDMATransfer((uint32_t)&_au8ShaKey[0],  _i32KeyLen/8);

    printf("Key len= %d bits\n", _i32KeyLen);

    g_SHA_done = 0;
    SHA_Start(CRYPTO_DMA_ONE_SHOT);
    while (!g_SHA_done) ;

    SHA_Read(au32OutputDigest);

    /*--------------------------------------------*/
    /*  Compare                                   */
    /*--------------------------------------------*/
    if (do_compare((uint8_t *)&au32OutputDigest[0], &_au8ShaDigest[0], _i32DigestLength) < 0)
    {
        printf("Compare error!\n");
        while (1);
    }
    return 0;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    /* Lock protected registers */
    if(SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART0 for printf */
    UART0_Init();

    printf("+-------------------------------------------+\n");
    printf("|           Crypto AES Driver Sample Code   |\n");
    printf("+-------------------------------------------+\n");

    NVIC_EnableIRQ(CRPT_IRQn);
    SHA_ENABLE_INT();

    open_test_vector();

    while (1)
    {
        if (get_next_pattern() < 0)
            break;

        run_sha();
    }

    printf("SHA test done.\n");

    while (1);
}



