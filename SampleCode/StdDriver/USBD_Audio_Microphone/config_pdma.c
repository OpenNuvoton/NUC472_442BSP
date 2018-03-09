/******************************************************************************
 * @file     config_pdma.c
 * @brief    NuMicro series USBD driver Sample file
 * @version  1.0.0
 * @date     23, Sep, 2014
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NUC472_442.h"
#include "usbd_audio.h"

/* DMA scatter-gather descriptor */
DMA_DESC_T DMA_RXDESC[PDMA_RXBUFFER_CNT];

extern uint8_t PcmRecBuff[PDMA_RXBUFFER_CNT][RX_BUFF_LEN];
extern uint8_t u8PcmRxBufFull[PDMA_RXBUFFER_CNT];
extern volatile uint8_t u8DataCntInBuffer;

extern volatile uint8_t u8PDMARxIdx;

/* PDMA Interrupt handler */
void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS();

    if (u32Status & 0x1)   /* abort */
    {
        if (PDMA_GET_ABORT_STS() & 0x4)
            PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIF_Msk);
        printf("Abort!!\n");
    }
    else if (u32Status & 0x2)
    {
        if (PDMA_GET_TD_STS() & 0x4)            /* channel 2 done, Rx */
        {
            /* Reset PDMA Scatter-Gather table */
            PDMA_ResetRxSGTable(u8PDMARxIdx);

            /* Set PCM buffer full flag */
            u8PcmRxBufFull[u8PDMARxIdx] = 1;

            /* Change to next buffer */
            u8PDMARxIdx ++;
            if(u8PDMARxIdx >= PDMA_RXBUFFER_CNT)
                u8PDMARxIdx = 0;
        }
        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF_Msk);

    }
    else        /* unknown interrupt */
    {
        printf("unknown interrupt, status=0x%x!!\n", u32Status);
    }
}

// Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    PDMA_WriteRxSGTable();

    /* Open PDMA channel 1 for I2S TX and channel 2 for I2S RX */
    PDMA_Open(PDMA_I2S_RX_CH << 1);

    /* We want to enable these channels at run time */
    PDMA->CHCTL = 0;

    /* Configure PDMA transfer mode */
    PDMA_SetTransferMode(PDMA_I2S_RX_CH, PDMA_I2S1_RX, 1, (uint32_t)&DMA_RXDESC[0]);

    /* Enable PDMA channel 1&2 interrupt */
    PDMA_EnableInt(PDMA_I2S_RX_CH, 0);

    /* Enable PDMA interrupt */
    NVIC_EnableIRQ(PDMA_IRQn);
}

/* init RX scatter-gather table */
void PDMA_WriteRxSGTable(void)
{
    uint16_t i;

    /* Use PDMA_RXBUFFER_CNT scatter-gather tables and link with each other */
    for(i=0; i<PDMA_RXBUFFER_CNT; i++)
    {
        DMA_RXDESC[i].ctl = (((RX_BUFF_LEN/4)-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_FIX|PDMA_DAR_INC|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
        DMA_RXDESC[i].endsrc = (uint32_t)&I2S1->RX;
        DMA_RXDESC[i].enddest = (uint32_t)&PcmRecBuff[i] + RX_BUFF_LEN;

        if(i != (PDMA_RXBUFFER_CNT-1))
            DMA_RXDESC[i].offset = (uint32_t)&DMA_RXDESC[i+1] - (PDMA->SCATBA);
        else
            DMA_RXDESC[i].offset = (uint32_t)&DMA_RXDESC[0] - (PDMA->SCATBA);
    }
}

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetRxSGTable(uint8_t id)
{
    DMA_RXDESC[id].ctl |= PDMA_OP_SCATTER;
    DMA_RXDESC[id].ctl |= (((RX_BUFF_LEN/4)-1)<<PDMA_DSCT_CTL_TXCNT_Pos);
}
