
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/11/05 4:34p $
 * @brief    Demonstrate the dead-zone feature with EPWM0 channel 0.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"

#define PLLCON_SETTING      CLK_PLLCTL_84MHz_HXT
#define PLL_CLOCK           84000000

void EPWM0_IRQHandler(void);

void EPWM0_IRQHandler(void)
{
    static uint32_t cnt;
    static uint32_t out;

    // Channel 0 frequency is 100Hz, every 1 second enter this IRQ handler 10 times.
    if(++cnt == 10)
    {
        if(out)
            EPWM_EnableOutput(EPWM0, 0x3F);
        else
            EPWM_DisableOutput(EPWM0, 0x3F);
        out ^= 1;
        cnt = 0;
    }
    // Clear channel 0 period interrupt flag
    EPWM_ClearPeriodIntFlag(EPWM0, 0);
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

    printf("\nThis sample code set EPWM0 channel 0 frequency 100Hz, duty 30\n");
    printf("And also enable/disable PWM0 output every 1 second.\n");
    // PWM0 frequency is 100Hz, duty 30%,
    EPWM_ConfigOutputChannel(EPWM0, 0, 100, 30);
    EPWM_EnableDeadZone(EPWM0, 0, 400);

    // Enable complementary mode
    EPWM_ENABLE_COMPLEMENTARY_MODE(EPWM0);

    // Enable output of all PWM channels
    EPWM_EnableOutput(EPWM0, 0x3F);

    // Enable PWM channel 0 period interrupt, use channel 0 to measure time.
    EPWM_EnablePeriodInt(EPWM0, 0, 0);
    NVIC_EnableIRQ(EPWM0_IRQn);

    // Start
    EPWM_Start(EPWM0, 0);

    while(1);

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


