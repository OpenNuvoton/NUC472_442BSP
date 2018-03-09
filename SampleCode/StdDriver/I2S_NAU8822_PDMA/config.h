/******************************************************************************
 * @file     config.h
 * @brief    Nano1xx I2S Driver Sample header file
 * @version  1.0.1
 * @date     04, September, 2012
 *
 * @note
 * Copyright (C) 2012-2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef CONFIG_H
#define CONFIG_H

// use LIN as source, undefine it if MIC is used
#define INPUT_IS_LIN

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define BUFF_LEN        512
#define BUFF_HALF_LEN   (BUFF_LEN/2)

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t endsrc;
    uint32_t enddest;
    uint32_t offset;
} DMA_DESC_T;

extern void PDMA_ResetTxSGTable(uint8_t id);
extern void PDMA_ResetRxSGTable(uint8_t id);
extern void PDMA_Init(void);
#endif

/*** (C) COPYRIGHT 2012 Nuvoton Technology Corp. ***/
