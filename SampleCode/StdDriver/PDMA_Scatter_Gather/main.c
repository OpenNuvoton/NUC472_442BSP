/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Date: 16/06/07 11:17a $
 * @brief    Use PDMA channel 5 to demonstrate memory to memory transfer
 *           by scatter-gather mode
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"

#define PLLCON_SETTING      CLK_PLLCON_84MHz_HXT
#define PLL_CLOCK           84000000

uint32_t PDMA_TEST_LENGTH = 64;
uint8_t SrcArray[256];
uint8_t DestArray0[256];
uint8_t DestArray1[256];

typedef struct dma_desc_t {
    uint32_t ctl;
    uint32_t endsrc;
    uint32_t enddest;
    uint32_t offset;
} DMA_DESC_T;

DMA_DESC_T DMA_DESC[2];

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_NUC472_442Series.s.
 */
void PDMA_IRQHandler(void)
{
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

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
    uint32_t u32EndSrc, u32EndDst0, u32EndDst1;
    uint32_t volatile i;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    for (i=0; i<PDMA_TEST_LENGTH*4; i++) {
        SrcArray[i] = i;
        DestArray0[i] = 0;
        DestArray1[i] = 0;
    }

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-----------------------------------------------------------------------+ \n");
    printf("|           PDMA Memory to Memory Driver Sample Code (Scatter-gather)   | \n");
    printf("+-----------------------------------------------------------------------+ \n");

    u32EndSrc = (uint32_t)SrcArray + PDMA_TEST_LENGTH * 4;
    u32EndDst0 = (uint32_t)DestArray0 + PDMA_TEST_LENGTH * 4;
    u32EndDst1 = (uint32_t)DestArray1 + PDMA_TEST_LENGTH * 4;

    DMA_DESC[0].ctl = ((PDMA_TEST_LENGTH-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_INC|PDMA_DAR_INC|PDMA_BURST_1|PDMA_OP_SCATTER;
    DMA_DESC[0].endsrc = u32EndSrc;
    DMA_DESC[0].enddest = u32EndDst0;
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1] - (PDMA->SCATBA);
    DMA_DESC[1].ctl = ((PDMA_TEST_LENGTH-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_INC|PDMA_DAR_INC|PDMA_BURST_1|PDMA_OP_BASIC;
    DMA_DESC[1].endsrc = u32EndDst0;
    DMA_DESC[1].enddest = u32EndDst1;
    DMA_DESC[1].offset = 0;

    /* Open Channel 5 */
    PDMA_Open(1 << 5);
    PDMA_SetTransferMode(5, PDMA_MEM, 1, (uint32_t)&DMA_DESC[0]);
    PDMA_Trigger(5);
    while(PDMA_IS_CH_BUSY(5));

    printf("test done...\n");

    PDMA_Close();

    while(1);
}


