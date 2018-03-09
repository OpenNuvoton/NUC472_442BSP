/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 10 $
 * $Date: 14/08/05 3:34p $
 * @brief    Use packet format (all the luma and chroma data interleaved) to
 *           store captured image from NT99141 sensor to SRAM.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "NUC472_442.h"
#include "sensor.h"
#include <stdio.h>


#define PLLCTL_SETTING      CLK_PLLCTL_84MHz_HXT
#define PLL_CLOCK           84000000

#define SIGNATURE       0x125ab234
#define FLAG_ADDR       0x20000FFC


/*------------------------------------------------------------------------------------------*/
/* To run CAP_InterruptHandler, when CAP frame end interrupt                                */
/*------------------------------------------------------------------------------------------*/
volatile uint32_t u32FramePass = 0;
void CAP_InterruptHandler(void)
{
    u32FramePass++;
}

/*------------------------------------------------------------------------------------------*/
/*  CAP_IRQHandler                                                                          */
/*------------------------------------------------------------------------------------------*/
void CAP_IRQHandler(void)
{
    uint32_t u32CapInt;
    u32CapInt = ICAP->INT;
    if( (u32CapInt & (CAP_INT_VIEN_Msk | CAP_INT_VINTF_Msk )) == (CAP_INT_VIEN_Msk | CAP_INT_VINTF_Msk))
    {
        CAP_InterruptHandler();
        ICAP->INT |= CAP_INT_VINTF_Msk;        /* Clear Frame end interrupt */
        u32EscapeFrame = u32EscapeFrame+1;
    }

    if((u32CapInt & (CAP_INT_ADDRMIEN_Msk|CAP_INT_ADDRMINTF_Msk)) == (CAP_INT_ADDRMIEN_Msk|CAP_INT_ADDRMINTF_Msk))
    {
        ICAP->INT |= CAP_INT_ADDRMINTF_Msk; /* Clear Address match interrupt */
    }

    if ((u32CapInt & (CAP_INT_MEIEN_Msk|CAP_INT_MEINTF_Msk)) == (CAP_INT_MEIEN_Msk|CAP_INT_MEINTF_Msk))
    {
        ICAP->INT |= CAP_INT_MEINTF_Msk;    /* Clear Memory error interrupt */
    }

    if ((u32CapInt & (CAP_INT_MDIEN_Msk|CAP_INT_MDINTF_Msk)) == (CAP_INT_MDIEN_Msk|CAP_INT_MDINTF_Msk))
    {
        ICAP->INT |= CAP_INT_MDINTF_Msk;    /* Clear Motion Detection interrupt */
    }
    ICAP->CTL = ICAP->CTL | CAP_CTL_UPDATE;
}

void CAP_SetFreq(uint32_t u32ModFreqKHz,uint32_t u32SensorFreq)
{
    int32_t i32Div;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable CAP Clock */
    CLK->AHBCLK |= CLK_AHBCLK_CAPCKEN_Msk;

    /* Enable Sensor Clock */
    CLK->AHBCLK |= CLK_AHBCLK_SENCKEN_Msk;

    /* Reset IP */
    SYS_ResetModule(CAP_RST);

    /* Specified sensor clock */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_CAPSEL_Msk) | CLK_CLKSEL0_CAPSEL_HCLK ;

    i32Div = CLK_GetHCLKFreq()/u32SensorFreq-1;
    if(i32Div < 0) i32Div = 0;
    CLK->CLKDIV3 = (CLK->CLKDIV3 & ~CLK_CLKDIV3_VSENSEDIV_Msk) | i32Div<<CLK_CLKDIV3_VSENSEDIV_Pos;

    /* Specified engine clock */
    i32Div = CLK_GetHCLKFreq()/u32ModFreqKHz-1;
    if(i32Div < 0) i32Div = 0;
    CLK->CLKDIV3 = (CLK->CLKDIV3 & ~CLK_CLKDIV3_CAPDIV_Msk) | i32Div<<CLK_CLKDIV3_CAPDIV_Pos;

    /* lock protected registers */
    SYS_LockReg();
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_HXT;

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCTL|= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency */
    CLK->PLLCTL = PLLCTL_SETTING;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable IP clock */
    CLK->APBCLK0 = CLK_APBCLK0_UART0CKEN_Msk;

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UARTSEL_HXT;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

    /* Set multi-function pins for CAP */
    SYS->GPD_MFPL = (SYS->GPD_MFPL & ~SYS_GPD_MFPL_PD0MFP_Msk)  | SYS_GPD_MFPL_PD0MFP_CAP_DATA3;
    SYS->GPD_MFPL = (SYS->GPD_MFPL & ~SYS_GPD_MFPL_PD1MFP_Msk)  | SYS_GPD_MFPL_PD1MFP_CAP_DATA2;
    SYS->GPD_MFPL = (SYS->GPD_MFPL & ~SYS_GPD_MFPL_PD2MFP_Msk)  | SYS_GPD_MFPL_PD2MFP_CAP_DATA1;
    SYS->GPD_MFPL = (SYS->GPD_MFPL & ~SYS_GPD_MFPL_PD3MFP_Msk)  | SYS_GPD_MFPL_PD3MFP_CAP_DATA0;
    SYS->GPD_MFPL = (SYS->GPD_MFPL & ~SYS_GPD_MFPL_PD4MFP_Msk)  | SYS_GPD_MFPL_PD4MFP_CAP_SCLK;
    SYS->GPD_MFPL = (SYS->GPD_MFPL & ~SYS_GPD_MFPL_PD5MFP_Msk)  | SYS_GPD_MFPL_PD5MFP_CAP_VSYNC;
    SYS->GPD_MFPL = (SYS->GPD_MFPL & ~SYS_GPD_MFPL_PD6MFP_Msk)  | SYS_GPD_MFPL_PD6MFP_CAP_HSYNC;
    SYS->GPD_MFPL = (SYS->GPD_MFPL & ~SYS_GPD_MFPL_PD7MFP_Msk)  | SYS_GPD_MFPL_PD7MFP_CAP_PIXCLK;
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~SYS_GPC_MFPH_PC12MFP_Msk) | SYS_GPC_MFPH_PC12MFP_CAP_DATA7;
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~SYS_GPC_MFPH_PC13MFP_Msk) | SYS_GPC_MFPH_PC13MFP_CAP_DATA6;
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~SYS_GPC_MFPH_PC14MFP_Msk) | SYS_GPC_MFPH_PC14MFP_CAP_DATA5;
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~SYS_GPC_MFPH_PC15MFP_Msk) | SYS_GPC_MFPH_PC15MFP_CAP_DATA4;

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
    UART_Open(UART0,115200);
}

#define SENSOR_IN_WIDTH                640
#define SENSOR_IN_HEIGHT            480
#define SYSTEM_WIDTH                160
#define SYSTEM_HEIGHT                120
uint8_t u8FrameBuffer[SYSTEM_WIDTH*SYSTEM_HEIGHT*2];

void PacketFormatDownScale(void)
{
    uint32_t u32Frame;

    /* Initialize NT99141 sensor and set NT99141 output YUV422 format  */
    if(InitNT99141_VGA_YUV422()==FALSE) return;

    /* Enable External CAP Interrupt */
    NVIC_EnableIRQ(CAP_IRQn);

    /* Enable External CAP Interrupt */
    CAP_EnableInt(CAP_INT_VIEN_Msk);

    /* Set Vsync polarity, Hsync polarity, pixel clock polarity, Sensor Format and Order */
    CAP_Open(NT99141SensorPolarity | NT99141DataFormatAndOrder, CAP_CTL_PKTEN );

    /* Set Cropping Window Vertical/Horizontal Starting Address and Cropping Window Size */
    CAP_SetCroppingWindow(0,0,SENSOR_IN_HEIGHT,SENSOR_IN_WIDTH);

    /* Set System Memory Packet Base Address Register */
    CAP_SetPacketBuf((uint32_t)u8FrameBuffer);

    /* Set Packet Scaling Vertical/Horizontal Factor Register */
    CAP_SetPacketScaling(SYSTEM_HEIGHT,SENSOR_IN_HEIGHT,SYSTEM_WIDTH,SENSOR_IN_WIDTH);

    /* Set Packet Frame Output Pixel Stride Width */
    CAP_SetPacketStride(SYSTEM_WIDTH);

    /* Start Image Capture Interface */
    CAP_Start();

    u32Frame=u32FramePass;
    while(1)
    {
        if(u32Frame!=u32FramePass)
        {
            u32Frame=u32FramePass;
            printf("Get frame %3d\n",u32Frame);
        }
    }

}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init Engine clock and Sensor clock */
    CAP_SetFreq(7000000 ,7000000);

    /* Using Picket format to Image down scale */
    PacketFormatDownScale();

    while(1);
}





