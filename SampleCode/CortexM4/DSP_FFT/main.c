/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 14/10/07 4:00p $
 * @brief    Demonstrate how to call ARM CMSIS DSP library to calculate FFT.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#define TEST_LENGTH_SAMPLES 2048

/* -------------------------------------------------------------------
* External Input and Output buffer Declarations for FFT Bin Example
* ------------------------------------------------------------------- */
extern float32_t testInput_f32_10khz[TEST_LENGTH_SAMPLES];
static float32_t testOutput[TEST_LENGTH_SAMPLES/2];

/* ------------------------------------------------------------------
* Global variables for FFT Bin Example
* ------------------------------------------------------------------- */
uint32_t fftSize = 1024;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

/* Reference index at which max energy of bin occurs */
uint32_t refIndex = 213, testIndex = 0;


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

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

    /* Lock protected registers */
    SYS_LockReg();

}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART0, 115200);
}


int main()
{
    arm_cfft_radix4_instance_f32 S;
    float32_t maxValue;

    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();
    UART_Init();

    /*
        This sample code is used to show how to use ARM Cortex-M4 DSP library to calculate FFT.
    */

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|             DSP FFT Sample Code        |\n");
    printf("+----------------------------------------+\n");

    /* Initialize the CFFT/CIFFT module */
    arm_cfft_radix4_init_f32(&S, fftSize, ifftFlag, doBitReverse);

    /* Process the data through the CFFT/CIFFT module */
    arm_cfft_radix4_f32(&S, testInput_f32_10khz);

    /* Process the data through the Complex Magnitude Module for
    calculating the magnitude at each bin */
    arm_cmplx_mag_f32(testInput_f32_10khz, testOutput, fftSize);

    /* Calculates maxValue and returns corresponding BIN value */
    arm_max_f32(testOutput, fftSize, &maxValue, &testIndex);

    if(testIndex !=  refIndex)
    {
        printf("ERROR: FFT calculation result fail!\n");
    }
    else
    {
        printf("FFT calculation test ok!\n");
    }

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
