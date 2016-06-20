/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 14/09/09 8:58a $
 * @brief    Demonstrate the usage of PDMA transfer. One SPI interface
 *           is enabled in loopback mode. Two PDMA channels are used
 *           in this sample, one for transmit, the other for receive.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"

#define PDMA_TEST_COUNT 512

uint32_t g_au32SrcData[PDMA_TEST_COUNT];
uint32_t g_au32DstData[PDMA_TEST_COUNT];
uint32_t volatile u32IsTestOver = 0;

void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS();

    if (status & 0x1) { /* abort */
        if (PDMA_GET_ABORT_STS() & 0x4)
            u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIF_Msk);
    } else if (status & 0x2) { /* done */
        if (PDMA_GET_TD_STS() & 0x4)
            u32IsTestOver = 1;
        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF_Msk);
    } else if (status & 0x400) { /* channel 2 timeout */
        u32IsTestOver = 3;
        PDMA_CLR_TMOUT_FLAG(2);
    } else
        printf("unknown interrupt, status=0x%x !!\n", status);
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
    CLK_EnableModuleClock(SPI0_MODULE);
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL = SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

    /* SPI0: GPE4=SS0, GPE3=MOSI0, GPE2=MISO0, GPE5=CLK */
    SYS->GPE_MFPL = (SYS_GPE_MFPL_PE2MFP_SPI0_MISO0 | SYS_GPE_MFPL_PE3MFP_SPI0_MOSI0 | SYS_GPE_MFPL_PE4MFP_SPI0_SS0 | SYS_GPE_MFPL_PE5MFP_SPI0_CLK);

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t u32Err=0;
    uint32_t u32EndSrc, u32EndDst;
    uint32_t i;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Configure SPI0 as a master, MSB first, 32-bit transaction, SPI Mode-0 timing, clock is 2MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 32, 2000000);

    /* Enable the automatic hardware slave select function. Select the SPI0_SS0 pin and configure as low-active. */
    SPI_EnableAutoSS(SPI0, SPI_SS0, SPI_SS_ACTIVE_LOW);

    printf("\n\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                     SPI with PDMA Sample Code                        |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");

    printf("The I/O connection for SPI0 loopback:\n");
    printf("    SPI0_MISO(PE.2) <--> SPI0_MOSI(PE.3)\n\n\n");
    printf("Please connect SPI0 MISO and MOSI pin, and press any key to start transmission ...");
    getchar();
    printf("\n");

    for(i=0; i<PDMA_TEST_COUNT; i++)
        g_au32SrcData[i] = 0x55550000 + i;

    /* Open Channel 1 for SPI0 TX, channel 2 for SPI0 RX */
    PDMA_Open(3 << 1);

    /* Configure channel 1 */
    PDMA_SetTransferCnt(1, PDMA_WIDTH_32, PDMA_TEST_COUNT);
    u32EndSrc = (uint32_t)g_au32SrcData + PDMA_TEST_COUNT * 4;
    u32EndDst = (uint32_t)&SPI0->TX;
    PDMA_SetTransferAddr(1, u32EndSrc, PDMA_SAR_INC, u32EndDst, PDMA_DAR_FIX);
    PDMA_SetBurstType(1, PDMA_REQ_SINGLE, PDMA_BURST_128);
    PDMA_SetTimeOut(1, 0, 0x5555);
    PDMA_EnableInt(1, 0);

    /* Configure channel 2 */
    PDMA_SetTransferCnt(2, PDMA_WIDTH_32, PDMA_TEST_COUNT);
    u32EndSrc = (uint32_t)&SPI0->RX;
    u32EndDst = (uint32_t)g_au32DstData + PDMA_TEST_COUNT * 4;
    PDMA_SetTransferAddr(2, u32EndSrc, PDMA_SAR_FIX, u32EndDst, PDMA_DAR_INC);
    PDMA_SetBurstType(2, PDMA_REQ_SINGLE, PDMA_BURST_128);
    PDMA_SetTimeOut(2, 0, 0x5555);
    PDMA_EnableInt(2, 0);

    /* Set Channel 1 for SPI0 TX, channel 2 for SPI0 RX, and then start timeout counting */
    PDMA_SetTransferMode(1, PDMA_SPI0_TX, 0, 0);
    PDMA_SetTransferMode(2, PDMA_SPI0_RX, 0, 0);

    /* SPI configuration is ready */
    SPI_TRIGGER(SPI0);

    /* Enable PDMA IRQ */
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Trigger PDMA */
    SPI_TRIGGER_TXRX_PDMA(SPI0);

    /* Wait for PDMA operation finish */
    while(u32IsTestOver == 0);

    /* Check PDMA status */
    if(u32IsTestOver != 1)
        printf("PDMA error !\n");

    /* Check Rx Data */
    for(i=0; i<PDMA_TEST_COUNT; i++) {
        if(g_au32SrcData[i] != g_au32DstData[i]) {
            u32Err ++;
        }
    }

    if(u32Err)
        printf(" [FAIL]\n\n");
    else
        printf(" [PASS]\n\n");

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
