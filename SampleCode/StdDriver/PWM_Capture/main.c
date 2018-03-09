
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 17 $
 * $Date: 14/05/30 6:01p $
 * @brief    Demonstrate PWM Capture function by using PWM0 channel 2
 *           to capture the output of PWM0 channel 0. Please connect
 *           PA.5 and PC.10 to execute this code
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"

#define PLLCON_SETTING      CLK_PLLCTL_84MHz_HXT
#define PLL_CLOCK           84000000

#define SAMPLE_CNT 32

static uint8_t volatile cap_index;
static uint32_t cap_val[SAMPLE_CNT >> 1][2];

void PWM0CH2_IRQHandler(void);

void PWM0CH2_IRQHandler(void)
{
    static uint8_t token = 0;
    uint32_t u32CapIntFlag;
    uint8_t u8Count = cap_index;

    if(u8Count >= SAMPLE_CNT)
        return;

    // Get channel 2 capture interrupt flag
    u32CapIntFlag = PWM_GetCaptureIntFlag(PWM0, 2);

    // Rising latch condition happened
    if ((u32CapIntFlag & PWM_RISING_LATCH_INT_FLAG) && token == 0)
    {
        cap_val[u8Count >> 1][0] = PWM_GET_CAPTURE_RISING_DATA(PWM0, 2);
        cap_index++;
        token = 1;
    }
    // Falling latch condition happened
    if ((u32CapIntFlag & PWM_FALLING_LATCH_INT_FLAG) && token == 1)
    {
        cap_val[u8Count >> 1][1] = PWM_GET_CAPTURE_FALLING_DATA(PWM0, 2);
        cap_index++;
        token = 0;
    }

    // Clear channel 2 capture interrupt flag
    PWM_ClearCaptureIntFlag(PWM0, 2, PWM_RISING_FALLING_LATCH_INT_FLAG);
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
    CLK->PLLCTL|= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency */
    CLK->PLLCTL = PLLCON_SETTING;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL,CLK_CLKDIV0_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(PWM0CH01_MODULE);
    CLK_EnableModuleClock(PWM0CH23_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UARTSEL_HXT,CLK_CLKDIV0_UART(1));

    /* Set PCLK as PWM0 channel 0~3 clock source */
    CLK_SetModuleClock(PWM0CH01_MODULE,CLK_CLKSEL2_PWM0CH01SEL_PCLK,1);
    CLK_SetModuleClock(PWM0CH23_MODULE,CLK_CLKSEL2_PWM0CH23SEL_PCLK,1);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

    /* Set GPG multi-function pins for CKO */
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC5MFP_CLK_O ;

    /* Set A5 A6 C10 C11 A12 A11 multi-function pins for PWM Channel 0~5  */
    SYS->GPA_MFPL |=  SYS_GPA_MFPL_PA5MFP_PWM0_CH0 | SYS_GPA_MFPL_PA6MFP_PWM0_CH1;
    SYS->GPC_MFPH |=  SYS_GPC_MFPH_PC10MFP_PWM0_CH2 | SYS_GPC_MFPH_PC11MFP_PWM0_CH3;
    SYS->GPA_MFPH |=  SYS_GPA_MFPH_PA12MFP_PWM0_CH4 | SYS_GPA_MFPH_PA11MFP_PWM0_CH5;

    /* Vref connect to AVDD */
    SYS->VREFCTL |= SYS_VREFCTL_ADCMODESEL_Msk;

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
    UART0->LINE |=0x07;
    UART0->BAUD = 0x30000066;   /* 12MHz reference clock input, for 115200 */
}

int32_t main (void)
{
    uint8_t i;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\nPWM0 channel 2 will capture the output of PWM0 channel 0\n");
    printf("So, please connect GPIO port A5 and C10.\n");
    // PWM0 frequency is 25000Hz, duty 30%,
    PWM_ConfigOutputChannel(PWM0, 0, 25000, 30);
    PWM_EnableDeadZone(PWM0, 0, 400);

    // PWM2
    PWM_ConfigCaptureChannel(PWM0,2,50,0);

    // Enable output of channel 0
    PWM_EnableOutput(PWM0, PWM_CH_0_MASK);

    // Eanble capture of channel 2
    PWM_EnableCapture(PWM0, PWM_CH_2_MASK);

    // Enable PWM channel 2 rising and falling edge capture interrupt
    PWM_EnableCaptureInt(PWM0,2,PWM_RISING_FALLING_LATCH_INT_ENABLE);
    NVIC_EnableIRQ(PWM0CH2_IRQn);

    // Start
    PWM_Start(PWM0, (PWM_CH_0_MASK|PWM_CH_2_MASK));

    cap_index = 0;

    while(cap_index < SAMPLE_CNT);

    // Disable PWM channel 2 rising and falling edge capture interrupt
    PWM_DisableCaptureInt(PWM0,2,PWM_RISING_FALLING_LATCH_INT_ENABLE);

    // Stop
    PWM_Stop (PWM0, (PWM_CH_0_MASK|PWM_CH_2_MASK));

    printf("Captured data is as below.\n");
    printf("(rising : falling)\n");
    for(i = 1; i < (SAMPLE_CNT  >> 1); i++)    // ignore first sampled data. it's wrong
    {
        printf("%d, %d : %d\n", i, cap_val[i][0], cap_val[i][1]);
    }

    while(1);

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


