/******************************************************************************
 * @file     nuc472_442_isr.c
 * @version  V0.10
 * $Revision: 4 $
 * $Date: 14/05/29 1:14p $
 * @brief    NUC472/NUC442 ISR source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC472_442.h"

#include "config.h"

extern volatile uint32_t u32BuffPos;
extern signed int aPCMBuffer[2][PCM_BUFFER_SIZE];
extern volatile uint8_t aPCMBuffer_Full[2];
extern volatile uint8_t u8PCMBuffer_Playing;

void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS();

    if (u32Status & 0x2) { /* done */
        if (PDMA_GET_TD_STS() & 0x4) {
            aPCMBuffer_Full[u8PCMBuffer_Playing] = 0;       //set empty flag
            PDMA_Reset_SCTable(u8PCMBuffer_Playing);
            u8PCMBuffer_Playing ^= 1;
        }
        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF_Msk);
    }

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
