/**************************************************************************//**
 * @file     pdma.c
 * @version  V1.00
 * $Revision: 10 $
 * $Date: 16/06/07 11:13a $
 * @brief    NUC400 series PDMA driver source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "nuc400series.h"


static uint8_t u32ChSelect[PDMA_CH_MAX];

/** @addtogroup NUC400_Device_Driver NUC400 Device Driver
  @{
*/

/** @addtogroup NUC400_PDMA_Driver PDMA Driver
  @{
*/


/** @addtogroup NUC400_PDMA_EXPORTED_FUNCTIONS PDMA Exported Functions
  @{
*/

/**
 * @brief       PDMA Open
 *
 * @param[in]   u32Mask     Channel enable bits.
 *
 * @return      None
 *
 * @details     This function enable the PDMA channels.
 */
void PDMA_Open(uint32_t u32Mask)
{
    int volatile i;

    for (i=0; i<PDMA_CH_MAX; i++)
        u32ChSelect[i] = 0x1f;

    PDMA->CHCON |= u32Mask;
}

/**
 * @brief       PDMA Close
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function disable all PDMA channels.
 */
void PDMA_Close(void)
{
    PDMA->CHCON = 0;
}

/**
 * @brief       Set PDMA Transfer Count
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Width        Data width. PDMA_WIDTH_8, PDMA_WIDTH_16, or PDMA_WIDTH_32
 * @param[in]   u32TransCount   Transfer count
 *
 * @return      None
 *
 * @details     This function set the selected channel data width and transfer count.
 */
void PDMA_SetTransferCnt(uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount)
{
    PDMA->DSCT[u32Ch].CTRL &= ~(PDMA_DSCT_CTRL_TFR_CNT_Msk | PDMA_DSCT_CTRL_TWS_Msk);
    PDMA->DSCT[u32Ch].CTRL |= (u32Width | (u32TransCount << PDMA_DSCT_CTRL_TFR_CNT_Pos));
}

/**
 * @brief       Set PDMA Transfer Address
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32SrcAddr      Source address
 * @param[in]   u32SrcCtrl      Source control attribute. PDMA_SAR_INC or PDMA_SAR_FIX
 * @param[in]   u32DstAddr      destination address
 * @param[in]   u32DstCtrl      destination control attribute. PDMA_DAR_INC or PDMA_DAR_FIX
 *
 * @return      None
 *
 * @details     This function set the selected channel source/destination address and attribute.
 */
void PDMA_SetTransferAddr(uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl)
{
    PDMA->DSCT[u32Ch].ENDSAR = u32SrcAddr;
    PDMA->DSCT[u32Ch].ENDDAR = u32DstAddr;
    PDMA->DSCT[u32Ch].CTRL &= ~(PDMA_DSCT_CTRL_SAR_INC_Msk | PDMA_DSCT_CTRL_DAR_INC_Msk);
    PDMA->DSCT[u32Ch].CTRL |= (u32SrcCtrl | u32DstCtrl);
}

/**
 * @brief       Set PDMA Transfer Mode
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Peripheral   The selected peripheral. PDMA_SPI0_TX, PDMA_UART0_TX, PDMA_I2S_TX,...PDMA_MEM
 * @param[in]   u32ScatterEn    Scatter-gather mode enable
 * @param[in]   u32DescAddr     Scatter-gather descriptor address
 *
 * @return      None
 *
 * @details     This function set the selected channel transfer mode. Include peripheral setting.
 */
void PDMA_SetTransferMode(uint32_t u32Ch, uint32_t u32Peripheral, uint32_t u32ScatterEn, uint32_t u32DescAddr)
{
    u32ChSelect[u32Ch] = u32Peripheral;
    switch (u32Ch) {
    case 0:
        PDMA->SMSEL0 = (PDMA->SMSEL0 & ~PDMA_SMSEL0_CH0_SEL_Msk) | u32Peripheral;
        break;
    case 1:
        PDMA->SMSEL0 = (PDMA->SMSEL0 & ~PDMA_SMSEL0_CH1_SEL_Msk) | (u32Peripheral << PDMA_SMSEL0_CH1_SEL_Pos);
        break;
    case 2:
        PDMA->SMSEL0 = (PDMA->SMSEL0 & ~PDMA_SMSEL0_CH2_SEL_Msk) | (u32Peripheral << PDMA_SMSEL0_CH2_SEL_Pos);
        break;
    case 3:
        PDMA->SMSEL0 = (PDMA->SMSEL0 & ~PDMA_SMSEL0_CH3_SEL_Msk) | (u32Peripheral << PDMA_SMSEL0_CH3_SEL_Pos);
        break;
    case 4:
        PDMA->SMSEL1 = (PDMA->SMSEL1 & ~PDMA_SMSEL1_CH4_SEL_Msk) | u32Peripheral;
        break;
    case 5:
        PDMA->SMSEL1 = (PDMA->SMSEL1 & ~PDMA_SMSEL1_CH5_SEL_Msk) | (u32Peripheral << PDMA_SMSEL1_CH5_SEL_Pos);
        break;
    case 6:
        PDMA->SMSEL1 = (PDMA->SMSEL1 & ~PDMA_SMSEL1_CH6_SEL_Msk) | (u32Peripheral << PDMA_SMSEL1_CH6_SEL_Pos);
        break;
    case 7:
        PDMA->SMSEL1 = (PDMA->SMSEL1 & ~PDMA_SMSEL1_CH7_SEL_Msk) | (u32Peripheral << PDMA_SMSEL1_CH7_SEL_Pos);
        break;
    case 8:
        PDMA->SMSEL2 = (PDMA->SMSEL2 & ~PDMA_SMSEL2_CH8_SEL_Msk) | u32Peripheral;
        break;
    case 9:
        PDMA->SMSEL2 = (PDMA->SMSEL2 & ~PDMA_SMSEL2_CH9_SEL_Msk) | (u32Peripheral << PDMA_SMSEL2_CH9_SEL_Pos);
        break;
    case 10:
        PDMA->SMSEL2 = (PDMA->SMSEL2 & ~PDMA_SMSEL2_CH10_SEL_Msk) | (u32Peripheral << PDMA_SMSEL2_CH10_SEL_Pos);
        break;
    case 11:
        PDMA->SMSEL2 = (PDMA->SMSEL2 & ~PDMA_SMSEL2_CH11_SEL_Msk) | (u32Peripheral << PDMA_SMSEL2_CH11_SEL_Pos);
        break;
    case 12:
        PDMA->SMSEL3 = (PDMA->SMSEL3 & ~PDMA_SMSEL3_CH12_SEL_Msk) | u32Peripheral;
        break;
    case 13:
        PDMA->SMSEL3 = (PDMA->SMSEL3 & ~PDMA_SMSEL3_CH13_SEL_Msk) | (u32Peripheral << PDMA_SMSEL3_CH13_SEL_Pos);
        break;
    case 14:
        PDMA->SMSEL3 = (PDMA->SMSEL3 & ~PDMA_SMSEL3_CH14_SEL_Msk) | (u32Peripheral << PDMA_SMSEL3_CH14_SEL_Pos);
        break;
    case 15:
        PDMA->SMSEL3 = (PDMA->SMSEL3 & ~PDMA_SMSEL3_CH15_SEL_Msk) | (u32Peripheral << PDMA_SMSEL3_CH15_SEL_Pos);
        break;
    default:
        ;
    }

    if (u32ScatterEn) {
        PDMA->DSCT[u32Ch].CTRL = (PDMA->DSCT[u32Ch].CTRL & ~PDMA_DSCT_CTRL_DMA_MODE_Msk) | PDMA_OP_SCATTER;
        PDMA->DSCT[u32Ch].NEXT = u32DescAddr - (PDMA->SCATBA);
    }
    PDMA->DSCT[u32Ch].CTRL = (PDMA->DSCT[u32Ch].CTRL & ~PDMA_DSCT_CTRL_DMA_MODE_Msk) | PDMA_OP_BASIC;
}

/**
 * @brief       Set PDMA Transfer Address
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32BurstType    Burst mode or single mode
 * @param[in]   u32BurstSize    Set the size of burst mode
 *
 * @return      None
 *
 * @details     This function set the selected channel burst type and size.
 */
void PDMA_SetBurstType(uint32_t u32Ch, uint32_t u32BurstType, uint32_t u32BurstSize)
{
    PDMA->DSCT[u32Ch].CTRL &= ~(PDMA_DSCT_CTRL_REQ_TYPE_Msk | PDMA_DSCT_CTRL_BUR_SIZE_Msk);
    PDMA->DSCT[u32Ch].CTRL |= (u32BurstType | u32BurstSize);
}


/**
 * @brief       Trigger PDMA
 *
 * @param[in]   u32Ch           The selected channel
 *
 * @return      None
 *
 * @details     This function trigger the selected channel.
 */
void PDMA_Trigger(uint32_t u32Ch)
{
    if (u32ChSelect[u32Ch] == PDMA_MEM)
        PDMA->SWREQ = (1 << u32Ch);
}

/**
 * @brief       Enable Interrupt
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Mask         The Interrupt Type
 *
 * @return      None
 *
 * @details     This function enable the selected channel interrupt.
 */
void PDMA_EnableInt(uint32_t u32Ch, uint32_t u32Mask)
{
    PDMA->IER |= (1 << u32Ch);
}

/**
 * @brief       Disable Interrupt
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Mask         The Interrupt Type
 *
 * @return      None
 *
 * @details     This function disable the selected channel interrupt.
 */
void PDMA_DisableInt(uint32_t u32Ch, uint32_t u32Mask)
{
    PDMA->IER &= ~(1 << u32Ch);
}

/*@}*/ /* end of group NUC400_PDMA_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NUC400_PDMA_Driver */

/*@}*/ /* end of group NUC400_Device_Driver */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
