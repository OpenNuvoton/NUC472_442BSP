/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 11 $
 * $Date: 14/05/30 6:02p $
 * @brief    Demonstrate the RTC function and displays current time to the
 *           UART console
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

volatile uint32_t  g_u32TICK = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* RTC Tick Handle                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_TickHandle(void)
{
    S_RTC_TIME_DATA_T sCurTime;

    /* Get the current time */
    RTC_GetDateAndTime(&sCurTime);

    printf(" Current Time:%d/%02d/%02d %02d:%02d:%02d\n",sCurTime.u32Year,sCurTime.u32Month,sCurTime.u32Day,sCurTime.u32Hour,sCurTime.u32Minute,sCurTime.u32Second);

    g_u32TICK++;
}

/**
  * @brief  RTC ISR to handle interrupt event
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{

    if ( (RTC->INTEN & RTC_INTEN_TICKIEN_Msk) && (RTC->INTSTS & RTC_INTSTS_TICKIF_Msk) )        /* tick interrupt occurred */
    {
        RTC->INTSTS = 0x2;

        RTC_TickHandle();
    }

}

void Delay(uint32_t ucnt)
{
    volatile uint32_t i = ucnt;

    while (i--);
}

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
    CLK->APBCLK0 |= CLK_APBCLK0_RTCCKEN_Msk; // RTC Clock Enable

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

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int32_t main(void)
{
    S_RTC_TIME_DATA_T sInitTime;

    SYS_Init();
    UART0_Init();

    /* Time Setting */
    sInitTime.u32Year       = 2013;
    sInitTime.u32Month      = 10;
    sInitTime.u32Day        = 15;
    sInitTime.u32Hour       = 12;
    sInitTime.u32Minute     = 30;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_TUESDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;

    RTC_Open(&sInitTime);
    if(g_RTC_i32ErrCode == RTC_TIMEOUT_ERR)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");
        while(1);
    }

    printf("\n RTC Time Display Test (Exit after 5 seconds)\n\n");

    /* Set Tick Period */
    RTC_SetTickPeriod(RTC_TICK_1_SEC);

    /* Enable RTC Tick Interrupt */
    RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);
    NVIC_EnableIRQ(RTC_IRQn);

    g_u32TICK = 0;
    while(g_u32TICK < 5);

    printf("\n RTC Time Display Test End !!\n");

    /* Disable RTC Tick Interrupt */
    RTC_DisableInt(RTC_INTEN_TICKIEN_Msk);
    NVIC_DisableIRQ(RTC_IRQn);

    while(1);

}






/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/



