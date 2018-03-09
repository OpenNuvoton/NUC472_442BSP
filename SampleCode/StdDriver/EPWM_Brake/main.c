
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/11/05 4:34p $
 * @brief    Demonstrate the brake function of EPWM0.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"

extern char GetChar(void);

#define PLLCON_SETTING      CLK_PLLCTL_84MHz_HXT
#define PLL_CLOCK           84000000

void EPWM0BRK_IRQHandler(void);

void EPWM0BRK_IRQHandler(void)
{
    printf("\nFault brake!\n");
    printf("Press any key to unlock brake state. (EPWM0 channel 0 output will toggle again)\n");
    GetChar();

    // Clear brake interrupt flag
    EPWM_ClearFaultBrakeFlag(EPWM0, 0);
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
    CLK_EnableModuleClock(EPWM0_MODULE);

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

    /* Set A12 A11 A10 A9 A8 A7 multi-function pins for EPWM0 Channel 0~5  */
    SYS->GPA_MFPH |=  SYS_GPA_MFPH_PA12MFP_EPWM0_CH0 | SYS_GPA_MFPH_PA11MFP_EPWM0_CH1 | SYS_GPA_MFPH_PA10MFP_EPWM0_CH2
                      | SYS_GPA_MFPH_PA9MFP_EPWM0_CH3 | SYS_GPA_MFPH_PA8MFP_EPWM0_CH4;
    SYS->GPA_MFPL |=  SYS_GPA_MFPL_PA7MFP_EPWM0_CH5;
    /* Set B7 multi-function pin for EPWM0 brake pin 0 */
    SYS->GPB_MFPL |=  SYS_GPB_MFPL_PB7MFP_BRAKE00;

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

    printf("\nConnet PB.6 (EPWM0 brake pin 0) to high(3.3V) then to low(0V).\n");
    printf("It will generate brake interrupt and EPWM0 channel 0 output stop toggling.\n");

    // PWM0 frequency is 100Hz, duty 30%,
    EPWM_ConfigOutputChannel(EPWM0, 0, 100, 30);

    // Enable output of all PWM channels
    EPWM_EnableOutput(EPWM0, 0x3F);

    // Enable brake and interrupt
    EPWM_EnableFaultBrake (EPWM0, EPWM_CH_0_MASK, 1, EPWM_BRK0_BKP0);
    EPWM_EnableFaultBrakeInt (EPWM0, 0);

    NVIC_EnableIRQ(EPWM0BRK_IRQn);

    // Start
    EPWM_Start(EPWM0, 0);

    while(1);

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


