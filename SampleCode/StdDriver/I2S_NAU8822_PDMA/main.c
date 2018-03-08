/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/09/17 1:42p $
 * @brief    NUC400 Series I2S Driver Sample Code
 *           This is a I2S demo with PDMA function connected with NAU8822 codec.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC472_442.h"
#include "config.h"

#define NAU8822_ADDR    0x1A                /* NAU8822 Device ID */

uint32_t PcmRxBuff[2][BUFF_LEN] = {0};
uint32_t PcmTxBuff[2][BUFF_LEN] = {0};
DMA_DESC_T DMA_TXDESC[2], DMA_RXDESC[2];

void Delay(int count)
{
    volatile uint32_t i;
    for (i = 0; i < count ; i++);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of NAU8822 with I2C3                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_WriteNAU8822(uint8_t u8addr, uint16_t u16data)
{

    I2C_START(I2C3);
    I2C_WAIT_READY(I2C3);

    I2C_SET_DATA(I2C3, NAU8822_ADDR<<1);
    I2C_SET_CONTROL_REG(I2C3, I2C_SI);
    I2C_WAIT_READY(I2C3);

    I2C_SET_DATA(I2C3, (uint8_t)((u8addr << 1) | (u16data >> 8)));
    I2C_SET_CONTROL_REG(I2C3, I2C_SI);
    I2C_WAIT_READY(I2C3);

    I2C_SET_DATA(I2C3, (uint8_t)(u16data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C3, I2C_SI);
    I2C_WAIT_READY(I2C3);

    I2C_STOP(I2C3);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  NAU8822 Settings with I2C interface                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void NAU8822_Setup()
{
    printf("\nConfigure NAU8822 ...");

    I2C_WriteNAU8822(0,  0x000);   /* Reset all registers */
    Delay(0x200);

#ifdef INPUT_IS_LIN //input source is LIN
    I2C_WriteNAU8822(1,  0x02F);
    I2C_WriteNAU8822(2,  0x1B3);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteNAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    I2C_WriteNAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1FF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1FF);   /* ADC right digital volume control */

    I2C_WriteNAU8822(44, 0x000);   /* LLIN/RLIN is not connected to PGA */
    I2C_WriteNAU8822(47, 0x060);   /* LLIN connected, and its Gain value */
    I2C_WriteNAU8822(48, 0x060);   /* RLIN connected, and its Gain value */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */
#else   //input source is MIC
    I2C_WriteNAU8822(1,  0x03F);
    I2C_WriteNAU8822(2,  0x1BF);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteNAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    I2C_WriteNAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1EF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1EF);   /* ADC right digital volume control */
    I2C_WriteNAU8822(44, 0x033);   /* LMICN/LMICP is connected to PGA */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */
#endif

    printf("[OK]\n");
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
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(I2C3_MODULE);
    CLK_EnableModuleClock(I2S1_MODULE);
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

    /* Set GPH3,GPH4 multi-function pins for I2C3 */
    SYS->GPH_MFPL |= SYS_GPH_MFPL_PH3MFP_I2C3_SCL | SYS_GPH_MFPL_PH4MFP_I2C3_SDA ;

    /* Set multi function pin for I2S1 */
    /* GPG7, GPG8, GPG9, GPI11, GPI12 */
    SYS->GPG_MFPL = (SYS->GPG_MFPL & ~SYS_GPG_MFPL_PG7MFP_Msk) | SYS_GPG_MFPL_PG7MFP_I2S1_MCLK;
    SYS->GPG_MFPH = (SYS->GPG_MFPH & ~(SYS_GPG_MFPH_PG8MFP_Msk | SYS_GPG_MFPH_PG9MFP_Msk)) | (SYS_GPG_MFPH_PG8MFP_I2S1_DO | SYS_GPG_MFPH_PG9MFP_I2S1_DI);
    SYS->GPI_MFPH = (SYS->GPI_MFPH & ~(SYS_GPI_MFPH_PI11MFP_Msk | SYS_GPI_MFPH_PI12MFP_Msk)) | (SYS_GPI_MFPH_PI11MFP_I2S1_BCLK | SYS_GPI_MFPH_PI12MFP_I2S1_LRCK);

    /* Lock protected registers */
    SYS_LockReg();
}

// Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    /* Tx description */
    DMA_TXDESC[0].ctl = ((BUFF_LEN-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_INC|PDMA_DAR_FIX|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
    DMA_TXDESC[0].endsrc = (uint32_t)&PcmTxBuff[0] + BUFF_LEN * 4;
    DMA_TXDESC[0].enddest = (uint32_t)&I2S1->TX;
    DMA_TXDESC[0].offset = (uint32_t)&DMA_TXDESC[1] - (PDMA->SCATBA);

    DMA_TXDESC[1].ctl = ((BUFF_LEN-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_INC|PDMA_DAR_FIX|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
    DMA_TXDESC[1].endsrc = (uint32_t)&PcmTxBuff[1] + BUFF_LEN * 4;
    DMA_TXDESC[1].enddest = (uint32_t)&I2S1->TX;
    DMA_TXDESC[1].offset = (uint32_t)&DMA_TXDESC[0] - (PDMA->SCATBA);   //link to first description

    /* Rx description */
    DMA_RXDESC[0].ctl = ((BUFF_LEN-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_FIX|PDMA_DAR_INC|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
    DMA_RXDESC[0].endsrc = (uint32_t)&I2S1->RX;
    DMA_RXDESC[0].enddest = (uint32_t)&PcmRxBuff[0] + BUFF_LEN * 4;
    DMA_RXDESC[0].offset = (uint32_t)&DMA_RXDESC[1] - (PDMA->SCATBA);

    DMA_RXDESC[1].ctl = ((BUFF_LEN-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_FIX|PDMA_DAR_INC|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
    DMA_RXDESC[1].endsrc = (uint32_t)&I2S1->RX;
    DMA_RXDESC[1].enddest = (uint32_t)&PcmRxBuff[1] + BUFF_LEN * 4;
    DMA_RXDESC[1].offset = (uint32_t)&DMA_RXDESC[0] - (PDMA->SCATBA);   //link to first description

    /* Open PDMA channel 1 for I2S TX and channel 2 for I2S RX */
    PDMA_Open(0x3 << 1);

    /* Configure PDMA transfer mode */
    PDMA_SetTransferMode(1, PDMA_I2S1_TX, 1, (uint32_t)&DMA_TXDESC[0]);
    PDMA_SetTransferMode(2, PDMA_I2S1_RX, 1, (uint32_t)&DMA_RXDESC[0]);

    /* Enable PDMA channel 1&2 interrupt */
    PDMA_EnableInt(1, 0);
    PDMA_EnableInt(2, 0);

    NVIC_EnableIRQ(PDMA_IRQn);
}

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetTxSGTable(uint8_t id)
{
    DMA_TXDESC[id].ctl |= PDMA_OP_SCATTER;
    DMA_TXDESC[id].ctl |= ((BUFF_LEN-1)<<PDMA_DSCT_CTL_TXCNT_Pos);
}

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetRxSGTable(uint8_t id)
{
    DMA_RXDESC[id].ctl |= PDMA_OP_SCATTER;
    DMA_RXDESC[id].ctl |= ((BUFF_LEN-1)<<PDMA_DSCT_CTL_TXCNT_Pos);
}

/* Init I2C interface */
void I2C3_Init(void)
{
    /* Open I2C3 and set clock to 100k */
    I2C_Open(I2C3, 100000);

    /* Get I2C3 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C3));
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+------------------------------------------------------------------------+\n");
    printf("|                   I2S Driver Sample Code with NAU8822                  |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  NOTE: This sample code needs to work with NAU8822.\n");

    /* Init I2C3 to access NAU8822 */
    I2C3_Init();

    // Plug-In DET
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA4MFP_Msk));
    GPIO_SetMode(PA, BIT4, GPIO_MODE_OUTPUT);
    PA4 = 0;

#ifdef INPUT_IS_LIN
    I2S_Open(I2S1, I2S_MODE_SLAVE, 16000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S, I2S_I2S);
#else
    I2S_Open(I2S1, I2S_MODE_SLAVE, 16000, I2S_DATABIT_16, I2S_MONO, I2S_FORMAT_I2S, I2S_I2S);
#endif

    NVIC_EnableIRQ(I2S1_IRQn);

    // select source from HXT(12MHz)
    CLK_SetModuleClock(I2S1_MODULE, CLK_CLKSEL3_I2S1SEL_HXT, 0);

    /* Initialize NAU8822 codec */
    NAU8822_Setup();

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(I2S1, 12000000);

#ifndef INPUT_IS_LIN
    I2S_SET_MONO_RX_CHANNEL(I2S1, I2S_MONO_LEFT);       //NAU8822 will store data in left channel
#endif

    PDMA_Init();

    /* Enable I2S Rx function */
    I2S_ENABLE_RXDMA(I2S1);
    I2S_ENABLE_RX(I2S1);

    /* Enable I2S Tx function */
    I2S_ENABLE_TXDMA(I2S1);
    I2S_ENABLE_TX(I2S1);

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
