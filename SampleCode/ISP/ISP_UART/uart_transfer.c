/**************************************************************************//**
 * @file     uart_transfer.c
 * @version  V1.00
 * $Date: 14/11/17 5:36p $
 * @brief    General UART ISP slave Sample file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "targetdev.h"
#include "uart_transfer.h"

#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t  uart_rcvbuf[MAX_PKT_SIZE] = {0};
#else
uint8_t  uart_rcvbuf[MAX_PKT_SIZE] __attribute__ ((aligned(4))) = {0};
#endif

uint8_t volatile bUartDataReady = 0;
uint8_t volatile bufhead = 0;


/* please check "targetdev.h" for chip specifc define option */

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART_T_IRQHandler(void)
{
    /*----- Determine interrupt source -----*/
    uint32_t u32IntSrc = UART_T->INTSTS;

    if (u32IntSrc & 0x11)   //RDA FIFO interrupt & RDA timeout interrupt
    {
        while (((UART_T->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) && (bufhead < MAX_PKT_SIZE))   //RX fifo not empty
        {
            uart_rcvbuf[bufhead++] = UART_T->DAT;
        }
    }

    if (bufhead == MAX_PKT_SIZE)
    {
        bUartDataReady = TRUE;
        bufhead = 0;
    }
    else if (u32IntSrc & 0x10)
    {
        bufhead = 0;
    }
}

extern uint8_t response_buff[64] __attribute__ ((aligned(4)));
void PutString(void)
{
    uint32_t i;

    for (i = 0; i < MAX_PKT_SIZE; i++)
    {
        while ((UART_T->FIFOSTS & UART_FIFOSTS_TXFULL_Msk));

        UART_T->DAT = response_buff[i];
    }
}

/*
uint32_t UART_IS_CONNECT(void)
{
    if((bufhead >= 4) || (bUartDataReady == TRUE)) {
        uint32_t lcmd;
        lcmd = inpw(uart_rcvbuf);
        if(lcmd == 0x000000AE) {    // CMD_CONNECT
            return 1;
        } else {
            bUartDataReady = 0;
            bufhead = 0;
        }
    }
    return 0;
}
*/

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
//  UART_T->FUN_SEL = UART_FUNC_SEL_UART;
    UART_T->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    UART_T->FIFO = UART_FIFO_RFITL_14BYTES | UART_FIFO_RTSTRGLV_14BYTES;
    UART_T->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(__HIRC, 115200));
//  UART_T->TOR = (UART_T->TOR & ~UART_TOR_TOIC_Msk)| (0x40);
    UART_T->TOUT = 0x40;
    NVIC_SetPriority(UART_T_IRQn, 2);
    NVIC_EnableIRQ(UART_T_IRQn);
    UART_T->INTEN = (UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_RDAIEN_Msk);
}

