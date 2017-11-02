/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/11/18 10:51a $
 * @brief    Demonstrate simultaneous mode.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"

#define PLLCON_SETTING      CLK_PLLCTL_84MHz_HXT
#define PLL_CLOCK           84000000

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void EADC_FunctionTest(void);


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
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT,CLK_CLKDIV0_HCLK(1));

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCTL|= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency */
    CLK->PLLCTL = PLLCON_SETTING;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL,CLK_CLKDIV0_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(EADC_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UARTSEL_HXT,CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(EADC_MODULE,CLK_CLKSEL1_ADCSEL_PLL,CLK_CLKDIV0_ADC(5));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

    /* Set GPG multi-function pins for CKO */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~SYS_GPC_MFPL_PC5MFP_Msk) | SYS_GPC_MFPL_PC5MFP_CLK_O ;

    /* Configure analog input pins ADC0(GPE0), ADC2(GPE2) */
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~SYS_GPE_MFPL_PE0MFP_Msk) | SYS_GPE_MFPL_PE0MFP_ADC0_0;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~SYS_GPE_MFPL_PE2MFP_Msk) | SYS_GPE_MFPL_PE2MFP_ADC0_2;

    /* Disable the digital input path */
    PE->DINOFF = PE->DINOFF | GPIO_DINOFF_DINOFF0_Msk | GPIO_DINOFF_DINOFF2_Msk;

    /* Reset ADC */
    SYS_ResetModule(ADC_RST);

    /* Vref connect to AVDD */
    SYS->VREFCTL |= SYS_VREFCTL_VREF_AVDD;

    /* Set to EADC mode */
    SYS->VREFCTL |= SYS_VREFCTL_ADCMODESEL_EADC;

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest()
{
    int32_t  i32ConversionData;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      Simultaneous mode test                          |\n");
    printf("|Select EADC0 sample module 2 and EADC1 sample module 2 in simultaneous|\n");
    printf("|sampling mode. Then trigger EADC0 sampling module 2 only, both EADC0  |\n");
    printf("|and EADC1 sample module 2 are triggered. They will do ADC conversion  |\n");
    printf("|at the same time to reach simultaneous sampling goal.                 |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Enable the A/D converter */
    EADC_Open(EADC, 0);

    /* Configure the EADC0 sample module 2 for analog input channel 3 and software trigger source.*/
    EADC_ConfigSampleModule(EADC, EADC0_SAMPLE_MODULE2, EADC_SOFTWARE_TRIGGER, EADC0_CH3);

    /* Configure the EADC1 sample module 2 for analog input channel 5 and software trigger source.*/
    EADC_ConfigSampleModule(EADC, EADC1_SAMPLE_MODULE2, EADC_SOFTWARE_TRIGGER, EADC0_CH5);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, 0x1);

    /* Select EADC0 and EADC1 sample module 2 to be simultaneous mode */
    EADC_ENABLE_SIMULTANEOUS(EADC, 0x4);

    /* Enable the sample module 0 interrupt.  */
    EADC_ENABLE_INT(EADC, 0x1);//Enable sample module A/D ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, 0x4);//Enable sample module 2 interrupt.
    NVIC_EnableIRQ(EADC0_IRQn);

    /* Reset the ADC interrupt indicator and trigger sample module 2 to start A/D conversion */
    g_u32AdcIntFlag = 0;
    EADC_START_CONV(EADC, 0x4);

    /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
    while(g_u32AdcIntFlag == 0);

    /* Disable the ADINT0 interrupt */
    EADC_DISABLE_INT(EADC, 0x1);

    /* Disable EADC0 and EADC1 sample module 2 simultaneous mode */
    EADC_DISABLE_SIMULTANEOUS(EADC, 0x4);

    /* Get the conversion result of the EADC0 and EADC1 sample module 2 */
    i32ConversionData = EADC_GET_CONV_DATA(EADC, EADC0_SAMPLE_MODULE2);
    printf("Conversion result of EADC0 sample module 2 (EADC0 channel 3): 0x%X (%d)\n\n", i32ConversionData, i32ConversionData);
    i32ConversionData = EADC_GET_CONV_DATA(EADC, EADC1_SAMPLE_MODULE2);
    printf("Conversion result of EADC1 sample module 2 (EADC1 channel 5): 0x%X (%d)\n\n", i32ConversionData, i32ConversionData);

    while(1);

}


/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC0_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    EADC_CLR_INT_FLAG(EADC, 0x1);      /* Clear the A/D ADINT0 interrupt flag */
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Reset EADC module */
    SYS_ResetModule(ADC_RST);

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC0_IRQn);

    printf("Exit EADC sample code\n");

    while(1);

}
