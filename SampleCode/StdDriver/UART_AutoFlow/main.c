/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 10 $
 * $Date: 14/10/06 9:54a $
 * @brief    Transmit and receive data using auto flow control.
 *
 * @note
 * Copyright (C) 2011 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"

#include "uart.h"


#define RXBUFSIZE 1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8SendData[12] = {0};
uint8_t g_u8RecData[RXBUFSIZE]  = {0};

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;
volatile int32_t g_i32pointer = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void UART_TEST_HANDLE(void);
void AutoFlow_FunctionTest(void);


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
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk; // UART0 Clock Enable
    CLK->APBCLK0 |= CLK_APBCLK0_UART1CKEN_Msk; // UART1 Clock Enable

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

    printf("+--------------------------+\n");
    printf("|  Auto-Flow function test  |\n");
    printf("+--------------------------+\n");

    AutoFlow_FunctionTest();

    while(1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_TEST_HANDLE()
{
    uint8_t u8InChar=0xFF;
    uint32_t u32IntSts= UART1->INTSTS;

    if(u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
        printf("\nInput:");

        /* Get all the input characters */
        while( (!UART_GET_RX_EMPTY(UART1)) )
        {
            /* Get the character from UART Buffer */
            u8InChar = UART_READ(UART1);    /* Rx trigger level is 1 byte*/

            g_i32pointer++;

            printf("%c ", u8InChar);

            if(u8InChar == '0')
            {
                g_bWait = FALSE;
            }

            /* Check if buffer full */
            if(g_u32comRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                g_u8RecData[g_u32comRtail] = u8InChar;
                g_u32comRtail = (g_u32comRtail == (RXBUFSIZE-1)) ? 0 : (g_u32comRtail+1);
                g_u32comRbytes++;
            }
        }
        printf("\nTransmission Test:");
    }

    if(u32IntSts & UART_INTSTS_THREINT_Msk)
    {
        uint16_t tmp;
        tmp = g_u32comRtail;
        if(g_u32comRhead != tmp)
        {
            u8InChar = g_u8RecData[g_u32comRhead];
            UART_WRITE(UART1,u8InChar);
            g_u32comRhead = (g_u32comRhead == (RXBUFSIZE-1)) ? 0 : (g_u32comRhead+1);
            g_u32comRbytes--;
        }
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionTest()
{
    uint8_t u8Item;
    uint32_t u32i;
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  _______                                      _______     |\n");
    printf("| |       |                                    |       |    |\n");
    printf("| |Master |---TXD1    <============>    RXD1---| Slave |    |\n");
    printf("| |       |---RTS1    <============>    CTS1---|       |    |\n");
    printf("| |_______|---CTS1    <============>    RTS1---|_______|    |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n\n");

    /* Set RTS Trigger Level */
    UART1->MODEM |= UART_RTS_IS_HIGH_LEV_TRG;
    UART1->MODEMSTS |= UART_CTS_IS_HIGH_LEV_TRG;
    UART1->FIFO = (UART1->FIFO &~ UART_FIFO_RTSTRGLV_Msk) | UART_FIFO_RTSTRGLV_14BYTES;

    /* Enable RTS and CTS autoflow control */
    UART1->INTEN |= UART_INTEN_ATORTSEN_Msk | UART_INTEN_ATOCTSEN_Msk;

    printf("+-----------------------------------------------------------+\n");
    printf("|       AutoFlow Function Test                              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave. Master will send 1k bytes data     |\n");
    printf("|    to slave.Slave will check if received data is correct  |\n");
    printf("|    after getting 1k bytes data.                           |\n");
    printf("|  Please select Master or Slave test                       |\n");
    printf("|  [0] Master    [1] Slave                                  |\n");
    printf("+-----------------------------------------------------------+\n\n");
    u8Item = getchar();


    if(u8Item=='0')
    {
        for(u32i=0; u32i<(RXBUFSIZE-1); u32i++)
        {
            UART_WRITE(UART1,((u32i+1)&0xFF));

            while(!(UART1->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk));
        }
        printf("\n Transmit Done\n");
    }
    else
    {
        g_i32pointer = 0;

        /* Enable RDA\RLS\RTO Interrupt  */
        UART_ENABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));

        /* Set RX Trigger Level = 8 */
        UART1->FIFO = (UART1->FIFO &~ UART_FIFO_RFITL_Msk) | UART_FIFO_RFITL_8BYTES;

        /* Set Timeout time 0x3E bit-time */
        UART_SetTimeoutCnt(UART1,0x3E);

        NVIC_EnableIRQ(UART1_IRQn);

        printf("Starting to receive %d bytes data...\n", RXBUFSIZE);

        while(g_i32pointer<(RXBUFSIZE-1))
        {
            //printf("%d\r",g_i32pointer);
        }

        /* Compare Data */
        for(u32i=0; u32i!=(RXBUFSIZE-1); u32i++)
        {
            if(g_u8RecData[u32i] != ((u32i+1)&0xFF) )
            {
                printf("Compare Data Failed\n");
                while(1);
            }
        }
        printf("\n Receive OK & Check OK\n");
    }

    NVIC_DisableIRQ(UART1_IRQn);

    UART_DISABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));

}


