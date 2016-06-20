/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 14/05/30 5:59p $
 * @brief    This Ethernet sample tends to get a DHCP lease from DHCP
 *           server. And use 192.168.10.10 as IP address it failed to
 *           get a lease. After IP address configured, this sample can
 *           reply to PING packets.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"
#include "net.h"

#define PLLCON_SETTING      CLK_PLLCON_84MHz_XTAL
#define PLL_CLOCK           84000000

// Our MAC address
uint8_t g_au8MacAddr[6] = {0x00, 0x00, 0x00, 0x59, 0x16, 0x88};
// Our IP address
uint8_t g_au8IpAddr[4] = {0, 0, 0, 0};
uint8_t auPkt[1514];
uint32_t u32PktLen;

// Descriptor pointers holds current Tx and Rx used by IRQ handler here.
uint32_t u32CurrentTxDesc, u32CurrentRxDesc;

/**
  * @brief  EMAC Tx interrupt handler.
  * @param  None
  * @return None
  */
void EMAC_TX_IRQHandler(void)
{
    // Clean up Tx resource occupied by previous sent packet(s)
    EMAC_SendPktDone();
}

/**
  * @brief  EMAC Rx interrupt handler.
  * @param  None
  * @return None
  */
void EMAC_RX_IRQHandler(void)
{

    while(1) {
        // Check if there's any packets available
        if(EMAC_RecvPkt(auPkt, &u32PktLen) == 0)
            break;
        // Process receive packet
        process_rx_packet(auPkt, u32PktLen);
        // Clean up Rx resource occupied by previous received packet
        EMAC_RecvPktDone();
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
    SYS->GPG_MFPL = SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;
    // Configure RMII pins
    SYS->GPC_MFPL = SYS_GPC_MFPL_PC0MFP_EMAC_REFCLK |
                    SYS_GPC_MFPL_PC1MFP_EMAC_MII_RXERR |
                    SYS_GPC_MFPL_PC2MFP_EMAC_MII_RXDV |
                    SYS_GPC_MFPL_PC3MFP_EMAC_MII_RXD1 |
                    SYS_GPC_MFPL_PC4MFP_EMAC_MII_RXD0 |
                    SYS_GPC_MFPL_PC6MFP_EMAC_MII_TXD0 |
                    SYS_GPC_MFPL_PC7MFP_EMAC_MII_TXD1;


    SYS->GPC_MFPH = SYS_GPC_MFPH_PC8MFP_EMAC_MII_TXEN;
    // Enable high slew rate on all RMII pins
    PC->SLEWCTL |= 0x1DF;

    // Configure MDC, MDIO at PB14 & PB15
    SYS->GPB_MFPH = SYS_GPB_MFPH_PB14MFP_EMAC_MII_MDC | SYS_GPB_MFPH_PB15MFP_EMAC_MII_MDIO;

    /* Lock protected registers */
    SYS_LockReg();

}


// This sample application can response to ICMP ECHO packets (ping)
// IP address is configure with DHCP, but if a lease cannot be acquired, a static IP will be used.
int main(void)
{

    SYS_Init();
    UART_Open(UART0, 115200);


    // Select RMII interface by default
    EMAC_Open(g_au8MacAddr);

    NVIC_EnableIRQ(EMAC_TX_IRQn);
    NVIC_EnableIRQ(EMAC_RX_IRQn);

    EMAC_ENABLE_RX();
    EMAC_ENABLE_TX();

    if (dhcp_start() < 0) {
        // Cannot get a DHCP lease, use static IP.
        printf("DHCP failed, use static IP 192.168.10.10\n");
        g_au8IpAddr[0] = 0xC0;
        g_au8IpAddr[1] = 0xA8;
        g_au8IpAddr[2] = 0x0A;
        g_au8IpAddr[3] = 0x0A;
    }
    while(1);

}


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
