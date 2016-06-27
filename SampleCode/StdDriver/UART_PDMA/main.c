/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 14/10/06 9:52a $
 * @brief    Demonstrate UART transmit and receive function with PDMA
 *
 * @note
 * Copyright (C) 2011 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"

#include "uart.h"

#define ENABLE_PDMA_INTERRUPT 1
#define PDMA_TEST_LENGTH 100

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint8_t g_u8Tx_Buffer[PDMA_TEST_LENGTH];
static uint8_t g_u8Rx_Buffer[PDMA_TEST_LENGTH];

volatile uint32_t u32IsTestOver = 0;

extern char GetChar(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void PDMA_IRQHandler(void);
void UART_PDMATest(void);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk; // XTAL12M (HXT) Enabled

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HXT;

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UARTSEL_Pos);// Clock source from external 12 MHz or 32 KHz crystal clock

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PG multi-function pins for UART0 RXD, TXD */
    SYS->GPG_MFPL &= ~(SYS_GPG_MFPL_PG1MFP_Msk | SYS_GPG_MFPL_PG2MFP_Msk);
    SYS->GPG_MFPL |= (SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD);

    /* Set PB multi-function pins for UART1 RXD, TXD, RTS and CTS */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk |
                       SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD |
                      SYS_GPB_MFPL_PB4MFP_UART1_RTS | SYS_GPB_MFPL_PB5MFP_UART1_CTS);

    /* Lock protected registers */
    SYS_LockReg();

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART0, 115200);
}

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART1, 115200);
}

void PDMA_Init(void)
{
    /* Open PDMA Channel */
    PDMA_Open(1 << 0); // Channel 0 for UART1 TX
    PDMA_Open(1 << 1); // Channel 1 for UART1 RX
    // Select basic mode
    PDMA_SetTransferMode(0, PDMA_UART1_TX, 0, 0);
    PDMA_SetTransferMode(1, PDMA_UART1_RX, 0, 0);
    // Set data width and transfer count
    PDMA_SetTransferCnt(0, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    PDMA_SetTransferCnt(1, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    //Set PDMA Transfer Address
    PDMA_SetTransferAddr(0, ((uint32_t) (&g_u8Tx_Buffer[PDMA_TEST_LENGTH])), PDMA_SAR_INC, UART1_BASE, PDMA_DAR_FIX);
    PDMA_SetTransferAddr(1, UART1_BASE, PDMA_SAR_FIX, ((uint32_t) (&g_u8Rx_Buffer[PDMA_TEST_LENGTH])), PDMA_DAR_INC);
    //Select Single Request
    PDMA_SetBurstType(0, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(1, PDMA_REQ_SINGLE, 0);


#ifdef ENABLE_PDMA_INTERRUPT
    PDMA_EnableInt(0, 0);
    PDMA_EnableInt(1, 0);
    NVIC_EnableIRQ(PDMA_IRQn);
    u32IsTestOver = 0;
#endif
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART0 for printf */
    UART0_Init();

    /* Init UART1 */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    UART_PDMATest();

    while(1);
}

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_NUC472_442.s.
 */
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS();

    if (status & 0x1) { /* abort */
        printf("target abort interrupt !!\n");
        if (PDMA_GET_ABORT_STS() & 0x4)
            u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIF_Msk);
    } else if (status & 0x2) { /* done */
        if ( (PDMA_GET_TD_STS() & (1 << 0)) && (PDMA_GET_TD_STS() & (1 << 1)) ) {
            u32IsTestOver = 1;
            PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF_Msk);
        }
    } else
        printf("unknown interrupt !!\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*  UART PDMA Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PDMATest()
{
    uint32_t i;

    printf("+-----------------------------------------------------------+\n");
    printf("|  UART PDMA Test                                           |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will demo UART1 PDMA function.         |\n");
    printf("|    Please connect UART1_TX and UART1_RX pin.              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please press any key to start test. \n\n");

    GetChar();

    /*
        Using UAR1 external loop back.
        This code will send data from UART1_TX and receive data from UART1_RX.
    */

    for (i=0; i<PDMA_TEST_LENGTH; i++) {
        g_u8Tx_Buffer[i] = i;
        g_u8Rx_Buffer[i] = 0xff;
    }

    while(1) {
        PDMA_Init();

        UART1->INTEN |= UART_INTEN_TXPDMAEN_Msk | UART_INTEN_RXPDMAEN_Msk;

#ifdef ENABLE_PDMA_INTERRUPT
        while(u32IsTestOver == 0);

        if (u32IsTestOver == 1)
            printf("test done...\n");
        else if (u32IsTestOver == 2)
            printf("target abort...\n");
#else
        while( (!(PDMA_GET_TD_STS() & (1 << 0))) || (!(PDMA_GET_TD_STS() & (1 << 1))) );

        PDMA_CLR_TD_FLAG(PDMA_TDF_TD_F_Msk);
#endif

        UART1->INTEN &= ~UART_INTEN_TXPDMAEN_Msk;
        UART1->INTEN &= ~UART_INTEN_RXPDMAEN_Msk;

        for (i=0; i<PDMA_TEST_LENGTH; i++) {
            if(g_u8Rx_Buffer[i] != i) {
                printf("\n Receive Data Compare Error !!");
                while(1);
            }

            g_u8Rx_Buffer[i] = 0xff;
        }

        printf("\nUART PDMA test Pass.\n");
    }

}


