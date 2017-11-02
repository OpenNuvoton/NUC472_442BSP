/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/11/18 10:37a $
 * @brief    Demonstrate how to use ADINT interrupt to trigger EADC.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "NUC472_442.h"

#define PLLCON_SETTING      CLK_PLLCTL_84MHz_HXT
#define PLL_CLOCK           84000000

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag = 0;

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
    CLK_WaitClockReady( CLK_STATUS_HXTSTB_Msk);

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

    /* Configure analog input pins ADC0(GPE0), ADC1(GPE1), ADC2(GPE2), ADC3(GPE3) */
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~SYS_GPE_MFPL_PE0MFP_Msk) | SYS_GPE_MFPL_PE0MFP_ADC0_0;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~SYS_GPE_MFPL_PE1MFP_Msk) | SYS_GPE_MFPL_PE1MFP_ADC0_1;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~SYS_GPE_MFPL_PE2MFP_Msk) | SYS_GPE_MFPL_PE2MFP_ADC0_2;
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~SYS_GPE_MFPL_PE3MFP_Msk) | SYS_GPE_MFPL_PE3MFP_ADC0_3;

    /* Disable the digital input path */
    PE->DINOFF = PE->DINOFF | GPIO_DINOFF_DINOFF0_Msk | GPIO_DINOFF_DINOFF1_Msk| GPIO_DINOFF_DINOFF2_Msk | GPIO_DINOFF_DINOFF3_Msk;

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
    uint8_t  u32SAMPLECount = 0;
    int32_t  i32ConversionData[6] = {0};

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      ADINT trigger mode test                         |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIt demonstrates ADINT trigger function.\n");
    printf("Sample module 4,5,6,7 are set to ADINT trigger.\n");
    printf("Trigger and convert sample module 7 only.\n");
    printf("After finishing conversion of sample module 7,\n");
    printf("The ADINT will trigger sample module 4,5,6,7 to start conversion.\n\n\n");


    /* Set the ADC internal sampling time, input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, 0);

    /* Configure the EADC0 sample module 4 for analog input channel 0 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, EADC0_SAMPLE_MODULE4, EADC_ADINT0_TRIGGER, EADC0_CH0);
    /* Configure the EADC0 sample module 5 for analog input channel 1 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, EADC0_SAMPLE_MODULE5, EADC_ADINT0_TRIGGER, EADC0_CH1);
    /* Configure the EADC0 sample module 6 for analog input channel 2 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, EADC0_SAMPLE_MODULE6, EADC_ADINT0_TRIGGER, EADC0_CH2);
    /* Configure the EADC0 sample module 7 for analog input channel 3 and enable ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, EADC0_SAMPLE_MODULE7, EADC_ADINT0_TRIGGER, EADC0_CH3);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, 0x1);

    /* Enable the EADC0 sample module 7 interrupt */
    EADC_ENABLE_INT(EADC, 0x1);//Enable sample module  A/D ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, (0x1 << 7));//Enable sample module 7 interrupt.
    NVIC_EnableIRQ(EADC0_IRQn);

    /* Trigger EADC0 sample module 7 to start A/D conversion */
    EADC_START_CONV(EADC, (0x1 << 7));

    /* Wait for EADC0 sample module 4,5,6,7 conversion done */
    while(EADC_GET_DATA_VALID_FLAG(EADC, 0xF0) != 0xF0);

    /* Get the conversion result of the sample module */
    for(u32SAMPLECount = 4; u32SAMPLECount < 8; u32SAMPLECount++)
        i32ConversionData[u32SAMPLECount] = EADC_GET_CONV_DATA(EADC, u32SAMPLECount);

    for(u32SAMPLECount = 4; u32SAMPLECount < 8; u32SAMPLECount++)
        printf("Conversion result of channel %d: 0x%X (%d)\n", (u32SAMPLECount - 4), i32ConversionData[u32SAMPLECount], i32ConversionData[u32SAMPLECount]);

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

