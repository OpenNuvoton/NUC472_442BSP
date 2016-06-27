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
DMA_DESC_T DMA_TXDESC[PDMA_TXBUFFER_CNT];

extern uint32_t PcmPlayBuffLen[PDMA_TXBUFFER_CNT];
extern uint32_t PcmPlayBuff[PDMA_TXBUFFER_CNT][BUFF_LEN];
extern volatile uint8_t u8DataCntInBuffer;
extern volatile uint8_t u8PDMATxIdx;

/* PDMA Interrupt handler */
void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS();

    if (u32Status & 0x1) { /* abort */
        if (PDMA_GET_ABORT_STS() & 0x4)
            PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIF_Msk);
    } else if (u32Status & 0x2) {
        if (PDMA_GET_TD_STS() & 0x2) {          /* channel 1 done, Tx */

            /* Reset PDMA scatter-gather table */
            PDMA_ResetTxSGTable(u8PDMATxIdx);

            /* Decrease number of full buffer */
            u8DataCntInBuffer --;

            /* Change to next buffer */
            u8PDMATxIdx ++;
            if(u8PDMATxIdx >= PDMA_TXBUFFER_CNT)
                u8PDMATxIdx = 0;
        }

        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF_Msk);

    } 
}

// Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    PDMA_WriteTxSGTable();

    /* Open PDMA channel 1 for I2S TX and channel 2 for I2S RX */
    PDMA_Open(PDMA_I2S_TX_CH << 1);

    /* We want to enable these channels at run time */
    PDMA->CHCTL = 0;

    /* Configure PDMA transfer mode */
    PDMA_SetTransferMode(PDMA_I2S_TX_CH, PDMA_I2S1_TX, 1, (uint32_t)&DMA_TXDESC[0]);

    /* Enable PDMA channel 1&2 interrupt */
    PDMA_EnableInt(PDMA_I2S_TX_CH, 0);

    /* Enable PDMA interrupt */
    NVIC_EnableIRQ(PDMA_IRQn);
}

/* init TX scatter-gather table */
void PDMA_WriteTxSGTable(void)
{
    uint16_t i;

    /* Use PDMA_TXBUFFER_CNT scatter-gather tables and link with each other */
    for(i=0; i<PDMA_TXBUFFER_CNT; i++) {
        DMA_TXDESC[i].ctl = ((BUFF_LEN-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_INC|PDMA_DAR_FIX|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
        DMA_TXDESC[i].endsrc = (uint32_t)&PcmPlayBuff[i] + BUFF_LEN * 4;
        DMA_TXDESC[i].enddest = (uint32_t)&I2S1->TX;

        if(i!=PDMA_TXBUFFER_CNT-1)
            DMA_TXDESC[i].offset = (uint32_t)&DMA_TXDESC[i+1] - (PDMA->SCATBA);
        else
            DMA_TXDESC[i].offset = (uint32_t)&DMA_TXDESC[0] - (PDMA->SCATBA);
    }
}

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetTxSGTable(uint8_t id)
{
    DMA_TXDESC[id].endsrc = (uint32_t)&PcmPlayBuff[id] + PcmPlayBuffLen[id] * 4;
    DMA_TXDESC[id].ctl |= PDMA_OP_SCATTER;
    DMA_TXDESC[id].ctl |= ((PcmPlayBuffLen[id]-1)<<PDMA_DSCT_CTL_TXCNT_Pos);
}

