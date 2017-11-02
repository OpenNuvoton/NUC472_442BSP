/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 15/09/16 5:25p $
 * @brief    NUC472/NUC442 SPI Driver Sample Code
 *           This is a SPI master mode demo and need to be tested with a slave device.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"

#define TEST_COUNT 16

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

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
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

    /* SPI0: GPE4=SS0, GPE3=MOSI0, GPE2=MISO0, GPE5=CLK */
    SYS->GPE_MFPL |= (SYS_GPE_MFPL_PE2MFP_SPI0_MISO0 | SYS_GPE_MFPL_PE3MFP_SPI0_MOSI0 | SYS_GPE_MFPL_PE4MFP_SPI0_SS0 | SYS_GPE_MFPL_PE5MFP_SPI0_CLK);

    /* Lock protected registers */
    SYS_LockReg();
}

void SPI0_IRQHandler(void)
{
    uint32_t temp;

    while((SPI_GET_STATUS(SPI0) & SPI_STATUS_RXEMPTY_Msk)==0) {
        temp = SPI_READ_RX(SPI0);
        g_au32DestinationData[g_u32RxDataCount++] = temp;
    }

    while( ((SPI_GET_STATUS(SPI0) & SPI_STATUS_TXFULL_Msk)==0) && (g_u32TxDataCount<TEST_COUNT) ) {
        SPI_WRITE_TX(SPI0, g_au32SourceData[g_u32TxDataCount++]);
    }

    if(g_u32TxDataCount>=TEST_COUNT)
        SPI_DisableInt(SPI0, SPI_FIFO_TXTHIEN_MASK); /* Disable TX FIFO threshold interrupt */

    /* Check the Rx FIFO time-out interrupt flag */
    if( SPI_GET_STATUS(SPI0) & SPI_STATUS_RXTOIF_Msk ) {
        while((SPI_GET_STATUS(SPI0) & SPI_STATUS_RXEMPTY_Msk)==0) {
            temp = SPI_READ_RX(SPI0);
            g_au32DestinationData[g_u32RxDataCount++] = temp;
        }
    }
}

int main(void)
{
    uint32_t u32DataCount;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Configure SPI0 as a master, MSB first, 32-bit transaction, SPI Mode-0 timing, clock is 2MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 32, 1000000);

    /* Enable the automatic hardware slave select function. Select the SPI0_SS0 pin and configure as low-active. */
    SPI_EnableAutoSS(SPI0, SPI_SS0, SPI_SS_ACTIVE_LOW);

    printf("\n\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                       SPI Driver Sample Code                         |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");

    printf("Configure SPI0 as a master.\n");
    printf("SPI clock rate: %d Hz\n", SPI_GetBusClock(SPI0));
    printf("SPI controller will transfer %d data to a off-chip slave device.\n", TEST_COUNT);
    printf("In the meanwhile the SPI controller will receive %d data from the off-chip slave device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);
    printf("The SPI master configuration is ready.\n");

    for(u32DataCount=0; u32DataCount<TEST_COUNT; u32DataCount++) {
        g_au32SourceData[u32DataCount] = 0x00550000 + u32DataCount;
        g_au32DestinationData[u32DataCount] = 0;
    }

    printf("Before starting the data transfer, make sure the slave device is ready. Press any key to start the transfer.\n");
    getchar();
    printf("\n");

    SPI_TRIGGER(SPI0);

    /* Set Tx FIFO threshold, enable Tx FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI_SetFIFOThreshold(SPI0, 4, 4);
    SPI_SET_SUSPEND_CYCLE(SPI0, 4);
    SPI_EnableInt(SPI0, SPI_FIFO_RXTOIEN_MASK | SPI_FIFO_TXTHIEN_MASK);

    g_u32TxDataCount = 0;
    g_u32RxDataCount = 0;
    NVIC_EnableIRQ(SPI0_IRQn);

    /* Wait for transfer done */
    while(g_u32RxDataCount<TEST_COUNT);

    printf("Received data:\n");
    for(u32DataCount=0; u32DataCount<TEST_COUNT; u32DataCount++) {
        printf("%d:\t0x%08X\n", u32DataCount, g_au32DestinationData[u32DataCount]);
    }
    /* Disable Tx FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI_DisableInt(SPI0, SPI_FIFO_RXTOIEN_MASK | SPI_FIFO_TXTHIEN_MASK);

    NVIC_DisableIRQ(SPI0_IRQn);
    printf("The data transfer was done.\n");

    while(1);
}

