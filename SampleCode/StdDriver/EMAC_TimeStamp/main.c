/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 14/05/30 5:59p $
 * @brief    Demonstrate the usage of Ethernet time stamp function.
 *           It sets current time to 1000 second and prints out
 *           current time every second. It also sets an alarm at
 *           1010 second. And rewind current time by 5 seconds after
 *           the alarm.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"


#define PLLCON_SETTING      CLK_PLLCON_84MHz_XTAL
#define PLL_CLOCK           84000000

// Our MAC address
uint8_t g_au8MacAddr[6] = {0x00, 0x00, 0x00, 0x59, 0x16, 0x88};

/**
  * @brief  EMAC Tx interrupt handler.
  * @param  None
  * @return None
  */
void EMAC_TX_IRQHandler(void)
{

    if(EMAC_GET_ALARM_FLAG()) {
        printf("Alarm interrupt!! Rewind current time by 5 second : 0 nano second\n");
        // First parameter set 1 means rewind current time, second parameter is second, and third parameter is nano second
        EMAC_UpdateTime(1, 5, 0);
    }

    // This sample sends no packet out, so this must be an alarm interrupt
    EMAC_CLR_ALARM_FLAG();

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
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

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
    CLK_EnableModuleClock(EMAC_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    // Configure MDC clock rate to HCLK / (127 + 1) = 656 kHz if system is running at 84 MHz
    CLK_SetModuleClock(EMAC_MODULE, 0, CLK_CLKDIV3_EMAC(127));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;
    // Configure RMII pins
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC0MFP_EMAC_REFCLK |
                    SYS_GPC_MFPL_PC1MFP_EMAC_MII_RXERR |
                    SYS_GPC_MFPL_PC2MFP_EMAC_MII_RXDV |
                    SYS_GPC_MFPL_PC3MFP_EMAC_MII_RXD1 |
                    SYS_GPC_MFPL_PC4MFP_EMAC_MII_RXD0 |
                    SYS_GPC_MFPL_PC6MFP_EMAC_MII_TXD0 |
                    SYS_GPC_MFPL_PC7MFP_EMAC_MII_TXD1;


    SYS->GPC_MFPH |= SYS_GPC_MFPH_PC8MFP_EMAC_MII_TXEN;
    // Enable high slew rate on all RMII pins
    PC->SLEWCTL |= 0x1DF;

    // Configure MDC, MDIO at PB14 & PB15
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB14MFP_EMAC_MII_MDC | SYS_GPB_MFPH_PB15MFP_EMAC_MII_MDIO;

    /* Lock protected registers */
    SYS_LockReg();

}



// This sample application can response to ICMP ECHO packets (ping)
// IP address is configure with DHCP, but if a lease cannot be acquired, a static IP will be used.
int main(void)
{
    uint32_t s = 0, ns = 0; // current time
    uint32_t old_s = 0;

    SYS_Init();

    UART_Open(UART0, 115200);

    printf("EMAC Timer Stamp Sample\n");

    // Open EMAC interface. No need to enable Rx and Tx in this sample
    // But still connect Ethernet cable to pass auto-negotiation.
    EMAC_Open(g_au8MacAddr);

    // Init time stamp and set current time to 1000s:0ns
    EMAC_EnableTS(1000, 0);

    // Time stamp alarm triggers Tx interrupt, so no need to enable Rx interrupt in this sample
    NVIC_EnableIRQ(EMAC_TX_IRQn);


    // Set Alarm at 1010s:0ns
    EMAC_EnableAlarm(1010, 0);

    while(1) {
        EMAC_GetTime(&s, &ns);
        if(s != old_s) {
            printf("Current time %ds. %dns\n", s, ns);
            old_s = s;
        }
    }

}


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
