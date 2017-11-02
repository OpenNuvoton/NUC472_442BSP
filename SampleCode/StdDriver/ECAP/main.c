
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/11/09 9:01a $
 * @brief    Demonstrate the Enhanced capture function.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"

#define PLLCON_SETTING      CLK_PLLCTL_84MHz_HXT
#define PLL_CLOCK           84000000

#define ECAP_CLKSEL_DIV64             (0x4 << ECAP_CTL1_CLKSEL_Pos)
#define ECAP_EDGESEL0_RISING_FALLING  (0x2 << ECAP_CTL1_EDGESEL0_Pos)
#define ECAP_NFCLKSEL_CAPCLK_DIV16    (0x3 << ECAP_CTL0_NFDIS_Pos)

void ICAP0_IRQHandler(void);

void ICAP0_IRQHandler(void)
{
    uint32_t iFlag;
    iFlag = ECAP0->STATUS;
    ECAP0->STATUS = iFlag;

    if(iFlag & ECAP_STATUS_OVF_Msk)
        printf("overflow interupt \n");
    if(iFlag & ECAP_STATUS_CMPF_Msk)
        printf("compare match interupt \n");
    if(iFlag & ECAP_STATUS_CAPF0_Msk) {
        printf("channel0 capture interupt \n");
        printf("ECAP_CNT: 0x %x \n", ECAP0->CNT);
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

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT,CLK_CLKDIV0_HCLK(1));

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCTL|= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency */
    CLK->PLLCTL = PLLCON_SETTING;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL,CLK_CLKDIV0_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(ECAP0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UARTSEL_HXT,CLK_CLKDIV0_UART(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

    /* Set GPG multi-function pins for CKO */
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC5MFP_CLK_O ;

    /* Set PC5 as ECAMP0 input0, ECAP0_IC0, MFP8 */
    SYS->GPC_MFPL &= ~SYS_GPC_MFPL_PC5MFP_Msk;
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC5MFP_ECAP0_IC0;

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
    UART0->LINE |=0x07;
    UART0->BAUD = 0x30000066;   /* 12MHz reference clock input, for 115200 */
}

int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("+---------------------------------------------+ \n");
    printf("|  EnhancedCapture timer Driver Sample Code   | \n");
    printf("+---------------------------------------------+ \n");

    /* Set ECAP clock to HCLK/64, and capture rising and falling edge input */
    ECAP0->CTL1 = (ECAP_CLKSEL_DIV64 | ECAP_EDGESEL0_RISING_FALLING);

    /* Enable capture function (IC0) and compare/overflow/capture interrupt and set noise filter clock to HCLK/16 */
    ECAP0->CTL0 = (ECAP_CTL0_CAPEN_Msk | ECAP_CTL0_CMPIEN_Msk | ECAP_CTL0_OVIEN_Msk
                   | ECAP_CTL0_CAPIEN0_Msk | ECAP_CTL0_CAPEN0_Msk | ECAP_NFCLKSEL_CAPCLK_DIV16);

    /* Clear all interrupt flag */
    ECAP0->STATUS = (ECAP_STATUS_OVF_Msk | ECAP_STATUS_CMPF_Msk | ECAP_STATUS_CAPF2_Msk
                     | ECAP_STATUS_CAPF1_Msk | ECAP_STATUS_CAPF0_Msk);

    /* Start counter */
    ECAP0->CTL0 |= ECAP_CTL0_CNTEN_Msk;

    NVIC_EnableIRQ(ICAP0_IRQn);

    while(1);

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


