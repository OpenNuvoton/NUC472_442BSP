/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 14/06/30 10:34a $
 * @brief    Demonstrate the usage of PDMA transfer. One SPI
 *           interface is use as a host, and the other is slave.
 *           Totally 4 PDAM channels are used in this sample
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"

#define PDMA_TEST_COUNT 512

/* Data buffer */
uint32_t g_au32MasterSrcData[PDMA_TEST_COUNT];
uint32_t g_au32MasterDstData[PDMA_TEST_COUNT];
uint32_t g_au32SlaveSrcData[PDMA_TEST_COUNT];
uint32_t g_au32SlaveDstData[PDMA_TEST_COUNT];

uint32_t volatile u32IsTestOver = 0;

void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS();

    if (u32Status & 0x1) { /* abort */
        if (PDMA_GET_ABORT_STS() & 0x4)
            u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIF_Msk);
    } else if (u32Status & 0x2) {
        if (PDMA_GET_TD_STS() & 0x4)    /* channel 2 done */
            u32IsTestOver += 1;
        if (PDMA_GET_TD_STS() & 0x10)   /* channel 4 done */
            u32IsTestOver += 8;
        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF_Msk);
    } else
        printf("unknown interrupt, status=0x%x!!\n", u32Status);
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
    CLK_EnableModuleClock(SPI1_MODULE);
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Select IP clock source */
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

    /* SPI1: GPC12=SS0, GPC15=MOSI0, GPD0=MISO0, GPD1=CLK */
    SYS->GPC_MFPH |= (SYS_GPC_MFPH_PC12MFP_SPI1_SS0 | SYS_GPC_MFPH_PC15MFP_SPI1_MOSI0);
    SYS->GPD_MFPL = (SYS_GPD_MFPL_PD0MFP_SPI1_MISO0 | SYS_GPD_MFPL_PD1MFP_SPI1_CLK);

    /* Lock protected registers */
    SYS_LockReg();
}

void ConfigurePDMAChannel(void)
{
    uint32_t u32EndSrc, u32EndDst;

    /* Open Channel 1 for SPI0 TX, Channel 2 for SPI0 RX, Channel 3 for SPI1 TX, Channel 4 for SPI1 RX */
    PDMA_Open(0xf << 1);

    /* Configure Channel 1 */
    PDMA_SetTransferCnt(1, PDMA_WIDTH_32, PDMA_TEST_COUNT);
    u32EndSrc = (uint32_t)g_au32MasterSrcData + PDMA_TEST_COUNT * 4;
    u32EndDst = (uint32_t)&SPI0->TX;
    PDMA_SetTransferAddr(1, u32EndSrc, PDMA_SAR_INC, u32EndDst, PDMA_DAR_FIX);
    PDMA_SetBurstType(1, PDMA_REQ_SINGLE, PDMA_BURST_128);
    PDMA_EnableInt(1, 0);

    /* Configure Channel 2 */
    PDMA_SetTransferCnt(2, PDMA_WIDTH_32, PDMA_TEST_COUNT);
    u32EndSrc = (uint32_t)&SPI0->RX;
    u32EndDst = (uint32_t)g_au32MasterDstData + PDMA_TEST_COUNT * 4;
    PDMA_SetTransferAddr(2, u32EndSrc, PDMA_SAR_FIX, u32EndDst, PDMA_DAR_INC);
    PDMA_SetBurstType(2, PDMA_REQ_SINGLE, PDMA_BURST_128);
    PDMA_EnableInt(2, 0);

    /* Configure Channel 3 */
    PDMA_SetTransferCnt(3, PDMA_WIDTH_32, PDMA_TEST_COUNT);
    u32EndSrc = (uint32_t)g_au32SlaveSrcData + PDMA_TEST_COUNT * 4;
    u32EndDst = (uint32_t)&SPI1->TX;
    PDMA_SetTransferAddr(3, u32EndSrc, PDMA_SAR_INC, u32EndDst, PDMA_DAR_FIX);
    PDMA_SetBurstType(3, PDMA_REQ_SINGLE, PDMA_BURST_128);
    PDMA_EnableInt(3, 0);

    /* Configure Channel 4 */
    PDMA_SetTransferCnt(4, PDMA_WIDTH_32, PDMA_TEST_COUNT);
    u32EndSrc = (uint32_t)&SPI1->RX;
    u32EndDst = (uint32_t)g_au32SlaveDstData + PDMA_TEST_COUNT * 4;
    PDMA_SetTransferAddr(4, u32EndSrc, PDMA_SAR_FIX, u32EndDst, PDMA_DAR_INC);
    PDMA_SetBurstType(4, PDMA_REQ_SINGLE, PDMA_BURST_128);
    PDMA_EnableInt(4, 0);

    /* Set Channel 1 for SPI0 TX, Channel 2 for SPI0 RX, Channel 3 for SPI1 TX, Channel 4 for SPI1 RX */
    PDMA_SetTransferMode(1, PDMA_SPI0_TX, 0, 0);
    PDMA_SetTransferMode(2, PDMA_SPI0_RX, 0, 0);
    PDMA_SetTransferMode(3, PDMA_SPI0_TX, 0, 0);
    PDMA_SetTransferMode(4, PDMA_SPI0_RX, 0, 0);
}

int main(void)
{
    uint32_t u32Err=0;
    uint32_t i;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Configure SPI0 as a master, MSB first, 32-bit transaction, SPI Mode-0 timing, clock is 2MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 32, 2000000);

    /* Enable the automatic hardware slave select function. Select the SPI0_SS0 pin and configure as low-active. */
    SPI_EnableAutoSS(SPI0, SPI_SS0, SPI_SS_ACTIVE_LOW);

    /* Configure SPI1 as a slave, MSB first, 32-bit transaction, SPI Mode-0 timing, clock is 4Mhz */
    SPI_Open(SPI1, SPI_SLAVE, SPI_MODE_0, 32, 4000000);

    /* Configure SPI1 as a low level active device. */
    SPI_SET_SS0_LOW(SPI1);

    printf("\n\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|             SPI Master/Slave with PDMA Sample Code                   |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");

    printf("Configure SPI0 as a master and SPI1 as a slave.\n");
    printf("Data width of a transaction: 32\n");
    printf("SPI clock rate: %d Hz\n", SPI_GetBusClock(SPI0));
    printf("The I/O connection for SPI0/SPI1 loopback:\n");
    printf("    SPI0_SS  (PE.4) <--> SPI1_SS(PC.12)\n    SPI0_CLK(PE.5)  <--> SPI1_CLK(PD.1)\n");
    printf("    SPI0_MISO(PE.2) <--> SPI1_MISO(PD.0)\n    SPI0_MOSI(PE.3) <--> SPI1_MOSI(PC.15)\n\n");
    printf("Please connect SPI0 with SPI1, and press any key to start transmission ...");
    getchar();
    printf("\n");

    for(i=0; i<PDMA_TEST_COUNT; i++) {
        g_au32MasterSrcData[i] = 0x55550000 + i;
        g_au32SlaveSrcData[i] = 0xAAAA0000 + i;
    }

    ConfigurePDMAChannel();

    /* SPI configuration is ready */
    SPI_TRIGGER(SPI0);
    SPI_TRIGGER(SPI1);

    /* Enable PDMA IRQ */
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Trigger PDMA */
    SPI_TRIGGER_TX_PDMA(SPI1);
    SPI_TRIGGER_RX_PDMA(SPI1);

    SPI_TRIGGER_RX_PDMA(SPI0);
    SPI_TRIGGER_TX_PDMA(SPI0);

    /* Wait for PDMA operation finish */
    while(u32IsTestOver == 0);

    /* Check if channel 2 and channel 4 are done or not */
    if(u32IsTestOver != 9)
        printf("PDMA error (%d)!\n", u32IsTestOver);

    /* Check Rx Data */
    for(i=0; i<PDMA_TEST_COUNT; i++) {
        if(g_au32MasterSrcData[i] != g_au32SlaveDstData[i]) {
            u32Err ++;
        }

        if(g_au32SlaveSrcData[i] != g_au32MasterDstData[i]) {
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
