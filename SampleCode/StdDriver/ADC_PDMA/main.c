/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 14/10/02 7:10p $
 * @brief    Use PDMA channel 1 to move ADC channel 0, 1, 2 converted data to SRAM.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"

#define PLLCON_SETTING      CLK_PLLCTL_84MHz_HXT
#define PLL_CLOCK           84000000

#define ADC_TEST_COUNT 32

uint32_t g_au32RxPDMADestination[ADC_TEST_COUNT];
uint32_t au32AdcData[ADC_TEST_COUNT];

volatile uint32_t g_u32PdmaTDoneInt;
volatile uint32_t g_u32PdmaTAbortInt;

void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS();

    if (u32Status & 0x1) { /* abort */
        if (PDMA_GET_ABORT_STS() & 0x4)
            g_u32PdmaTAbortInt = 1;
        PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIF_Msk);
    } else if (u32Status & 0x2) {
        if (PDMA_GET_TD_STS() & 0x2)    /* channel 1 done */
            g_u32PdmaTDoneInt = 1;
        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF_Msk);
    } else if (u32Status & 0x200) { /* channel 1 timeout */
        PDMA_CLR_TMOUT_FLAG(1);
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
    CLK->PLLCTL|= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency */
    CLK->PLLCTL = PLLCON_SETTING;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL,CLK_CLKDIV0_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(ADC_MODULE);
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UARTSEL_HXT,CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(ADC_MODULE,CLK_CLKSEL1_ADCSEL_PCLK,CLK_CLKDIV0_ADC(5));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL = SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

    /* Set GPG multi-function pins for CKO */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~SYS_GPC_MFPL_PC5MFP_Msk) | SYS_GPC_MFPL_PC5MFP_CLK_O ;

    /* Configure analog input pins ADC0(GPE0) */
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~SYS_GPE_MFPL_PE0MFP_Msk) | SYS_GPE_MFPL_PE0MFP_ADC0_0;

    /* Disable the digital input path */
    PE->DINOFF = PE->DINOFF | GPIO_DINOFF_DINOFF0_Msk;

    /* Reset ADC */
    SYS_ResetModule(ADC_RST);

    /* Reset PDMA */
    SYS_ResetModule(PDMA_RST);

    /* Vref connect to AVDD */
    SYS->VREFCTL |= SYS_VREFCTL_VREF_AVDD;

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

void PDMA_INIT(void)
{
    uint32_t u32EndSrc, u32EndDst;

    /* Open PDMA Channel 1 for ADC */
    PDMA_Open(1 << 1);

    /* Configure Channel 1 */
    PDMA_SetTransferCnt(1, PDMA_WIDTH_32, ADC_TEST_COUNT);
    u32EndSrc = (uint32_t)&ADC->CURDAT;
    u32EndDst = (uint32_t)g_au32RxPDMADestination + ADC_TEST_COUNT * 4;
    PDMA_SetTransferAddr(1, u32EndSrc, PDMA_SAR_FIX, u32EndDst, PDMA_DAR_INC);
    PDMA_SetBurstType(1, PDMA_REQ_SINGLE, PDMA_BURST_128);
    PDMA->DSCT[1].CTL |= 1;
    PDMA_SetTimeOut(1, 0, 0x5555);
    PDMA_EnableInt(1, 0);

    /* Set PDMA Channel 1 for ADC, and start timeout counting */
    PDMA_SetTransferMode(1, PDMA_ADC, 0, 0);
}

int32_t main (void)
{
    uint32_t u32DataCount;
    uint32_t u32ErrorCount;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nThis sample code demonstrates ADC PDMA function.\n");
    printf("Set ADC operation mode to single cycle scan mode, and enable channel 0,1,2,3\n");
    printf("Enable ADC PDMA function, and trigger ADC conversion.\n");
    printf("Compare the log of ADC conversion data register with the content of PDMA target buffer.\n");
    printf("Finally, print the test result.\n\n");

    /* Enable channel 0,1,2,3 */
    ADC_Open(ADC, ADC_INPUT_MODE_SINGLE_END, ADC_OPERATION_MODE_SINGLE_CYCLE, ADC_CH_0_MASK | ADC_CH_1_MASK | ADC_CH_2_MASK | ADC_CH_3_MASK);

    /* Power on ADC */
    ADC_POWER_ON(ADC);

    /* Enable ADC PDMA */
    ADC_ENABLE_PDMA(ADC);

    /* Configure PDMA channel 1 */
    PDMA_INIT();

    /* Enable PDMA IRQ */
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Clear destination buffer */
    for(u32DataCount = 0; u32DataCount < ADC_TEST_COUNT; u32DataCount++)
        g_au32RxPDMADestination[u32DataCount] = 0;

    u32DataCount = 0;
    u32ErrorCount = 0;

    ADC_START_CONV(ADC);

    while(1) {
        uint32_t u32Ch;
        if(ADC_GET_INT_FLAG(ADC,ADC_ADF_INT) == 1) {
            ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

            for (u32Ch = 0; u32Ch < 4; u32Ch++) {
                au32AdcData[u32DataCount++] = ADC_GET_CONVERSION_DATA(ADC, u32Ch);
                if(u32DataCount >= ADC_TEST_COUNT)
                    break;
            }
            if (u32DataCount < ADC_TEST_COUNT)
                ADC_START_CONV(ADC);
            else
                break;
        }
    }
    /* Wait for PDMA transfer down */
    while(g_u32PdmaTDoneInt == 0);

    /* Compare the log of ADC conversion data register with the content of PDMA target buffer */
    for(u32DataCount = 0; u32DataCount < ADC_TEST_COUNT; u32DataCount++) {
        if( au32AdcData[u32DataCount] != (g_au32RxPDMADestination[u32DataCount] & 0xFFF) ) {
            printf("*** Count %d, conversion result: 0x%X, PDMA result: 0x%X.\n",
                   u32DataCount, au32AdcData[u32DataCount], g_au32RxPDMADestination[u32DataCount]);
            u32ErrorCount++;
        }
    }

    if (u32ErrorCount == 0)
        printf("PASS!\n");
    else
        printf("FAIL!\n");

    while (1);

}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


