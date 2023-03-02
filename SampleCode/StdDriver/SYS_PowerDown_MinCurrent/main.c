/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to minimize power consumption when entering power down mode.
 *
 * @note
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"

#define GPIO_P0_TO_P15  0xFFFF

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

/**
 * @brief       GPIO PB IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PB default IRQ, declared in startup_M4521.s.
 */
void GPB_IRQHandler(void)
{
    uint32_t volatile reg;

    /* To check if PB3 interrupt occurred */
    if (PB->INTSRC & BIT3)
    {
        PB->INTSRC = BIT3;
        printf("PB.3 INT occurred. \n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PORTB interrupts */
        reg = PB->INTSRC;
        PB->INTSRC = reg;
        printf("Un-expected interrupts. \n");
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC,CLK_CLKDIV0_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));

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


int main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------+\n");
    printf("|  SYS_PowerDown_MinCurrent and Wake-up by PB.3 Sample Code   |\n");
    printf("+-------------------------------------------------------------+\n\n");

    printf("+-------------------------------------------------------------------------+\n");
    printf("+ Operating sequence                                                      |\n");
    printf("|  1. Remove all continuous load, e.g. LED.                               |\n");
    printf("|  2. Configure all GPIO as Quasi-bidirectional Mode                      |\n");
    printf("|  3. Must enable LVR                                                     |\n");
    printf("|  4. Disable analog function, e.g. ADC, ACMP, and POR module.            |\n");
    printf("|  5. Enter to Power-Down                                                 |\n");
    printf("|  6. Wait for PB.3 rising-edge interrupt event to wakeup the MCU         |\n");
    printf("+-------------------------------------------------------------------------+\n\n");

    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Configure all GPIO as Quasi-bidirectional Mode*/
    GPIO_SetMode(PA, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PD, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PE, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PF, GPIO_P0_TO_P15, GPIO_MODE_QUASI);

    /* Configure PB.3 as Quasi-bidirectional mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(PB, BIT3, GPIO_MODE_QUASI);
    GPIO_EnableInt(PB, 3, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPB_IRQn);

    /* Enable interupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_IRC10K, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PB, BIT3);

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* LVR must be enabled and the POR will be enabled automatically */
    SYS_ENABLE_LVR();
    /* Disable Power-on Reset */
    SYS_DISABLE_POR();

    printf("Enter to Power-Down ......\n");
    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Waiting for PB.3 rising-edge interrupt event */
    printf("System waken-up done.\n\n");

    while(1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
