/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Date: 14/07/31 1:44p $
 * @brief    NUC472/NUC442 General Purpose I/O Driver Sample Code
 *           Connect PB.3 and PD.7 to test IO In/Out
 *           Test PB2, PC5, PA0(INT0) and PF0(INT5) interrupts
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"
#include "gpio.h"

#define PLLCON_SETTING      CLK_PLLCON_84MHz_HXT
#define PLL_CLOCK           84000000

/**
 * @brief       GPIO IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The GPIO default IRQ, declared in startup_NUC472_442.s.
 */
void GPA_IRQHandler(void)
{
    uint32_t volatile reg;
    reg = PA->INTSRC;
    PA->INTSRC = reg;
}

void GPB_IRQHandler(void)
{
    uint32_t volatile reg;
    /* To check if PB2 interrupt occurred */
    if (PB->INTSRC & BIT2)
    {
        PB->INTSRC = BIT2;
        PD10 = PD10 ^ 1;
        printf("PB2 INT occurred. \n");

    }
    else
    {
        /* Un-expected interrupt. Just clear all PORTB interrupts */
        reg = PB->INTSRC;
        PB->INTSRC = reg;
        printf("Un-expected interrupts. \n");
    }
}

void GPC_IRQHandler(void)
{
    uint32_t volatile reg;
    /* To check if PC5 interrupt occurred */
    if (PC->INTSRC & BIT5)
    {
        PC->INTSRC = BIT5;
        PD10 = PD10 ^ 1;
        printf("PC5 INT occurred. \n");
    }
    else
    {
        reg = PC->INTSRC;
        PC->INTSRC = reg;
        printf("Un-expected interrupts. \n");
    }
}

void GPD_IRQHandler(void)
{
    uint32_t volatile reg;
    reg = PD->INTSRC;
    PD->INTSRC = reg;
}

void GPE_IRQHandler(void)
{
    uint32_t volatile reg;
    reg = PE->INTSRC;
    PE->INTSRC = reg;
}

void GPF_IRQHandler(void)
{
    uint32_t volatile reg;
    reg = PF->INTSRC;
    PF->INTSRC = reg;
}

void GPG_IRQHandler(void)
{
    uint32_t volatile reg;
    reg = PG->INTSRC;
    PG->INTSRC = reg;
}

void GPH_IRQHandler(void)
{
    uint32_t volatile reg;
    reg = PH->INTSRC;
    PH->INTSRC = reg;
}

void GPI_IRQHandler(void)
{
    uint32_t volatile reg;
    reg = GPI->INTSRC;
    GPI->INTSRC = reg;
}

/**
 * @brief       External INT IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT default IRQ, declared in startup_NUC472_442.s.
 */
void EINT0_IRQHandler(void)
{
    /* For PA0, clear the INT flag */
    PA->INTSRC = BIT0;
    PD10 = PD10 ^ 1;
    printf("PA0 EINT0 occurred. \n");
}

void EINT1_IRQHandler(void)
{
    /* For PB0, clear the INT flag */
    PB->INTSRC = BIT0;
    PD10 = PD10 ^ 1;
    printf("PB0 EINT1 occurred. \n");
}

void EINT2_IRQHandler(void)
{
    /* For PC0, clear the INT flag */
    PC->INTSRC = BIT0;
    PD10 = PD10 ^ 1;
    printf("PC0 EINT2 occurred. \n");
}

void EINT3_IRQHandler(void)
{
    /* For PD0, clear the INT flag */
    PD->INTSRC = BIT0;
    PD10 = PD10 ^ 1;
    printf("PD0 EINT3 occurred. \n");
}

void EINT4_IRQHandler(void)
{
    /* For PE0, clear the INT flag */
    PE->INTSRC = BIT0;
    PD10 = PD10 ^ 1;
    printf("PE0 EINT4 occurred. \n");
}

void EINT5_IRQHandler(void)
{
    /* For PF0, clear the INT flag */
    PF->INTSRC = BIT0;
    PD10 = PD10 ^ 1;
    printf("PF0 EINT5 occurred. \n");
}

void EINT6_IRQHandler(void)
{
    /* For PG0, clear the INT flag */
    PG->INTSRC = BIT0;
    PD10 = PD10 ^ 1;
    printf("PG0 EINT6 occurred. \n");
}

void EINT7_IRQHandler(void)
{
    /* For PH0, clear the INT flag */
    PH->INTSRC = BIT0;
    PD10 = PD10 ^ 1;
    printf("PH0 EINT7 occurred. \n");
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

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
    int32_t i32Err;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-------------------------------------+ \n");
    printf("|           GPIO Driver Sample Code   | \n");
    printf("+-------------------------------------+ \n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Basic Mode Test --- Use Pin Data Input/Output to control GPIO pin                              */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("  >> Please connect PB.3 and PD.7 first << \n");
    printf("     Press any key to start test by using [Pin Data Input/Output Control] \n\n");
    getchar();

    /* Configure PB.3 as Output mode and PD.7 as Input mode then close it */
    GPIO_SetMode(PB, BIT3, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT7, GPIO_MODE_INPUT);

    i32Err = 0;
    printf("  GPIO Output/Input test ...... \n");

    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    PB3 = 0;
    if (PD7 != 0)
    {
        i32Err = 1;
    }

    PB3 = 1;
    if (PD7 != 1)
    {
        i32Err = 1;
    }

    if ( i32Err )
    {
        printf("  [FAIL] --- Please make sure PB.3 and PD.7 are connected. \n");
    }
    else
    {
        printf("  [OK] \n");
    }

    /* Configure PB.3 and PD.7 to default Quasi-bidirectional mode */
    GPIO_SetMode(PB, BIT3, GPIO_MODE_QUASI);
    GPIO_SetMode(PD, BIT7, GPIO_MODE_QUASI);


    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Interrupt Function Test                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("\n  PB2, PC5, PA0(INT0) and PF0(INT5) are used to test interrupt\n  and control LEDs(PD10)\n");

    /*Configure PD10 for LED control */
    GPIO_SetMode(PD, BIT10, GPIO_MODE_OUTPUT);

    /* Configure PB2 as Input mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 2, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPB_IRQn);


    /*  Configure PC5 as Quasi-bi-direction mode and enable interrupt by falling edge trigger */
    GPIO_SetMode(PC, BIT5, GPIO_MODE_QUASI);
    GPIO_EnableInt(PC, 5, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPC_IRQn);

    /* Configure PA0 as EINT0 pin and enable interrupt by falling edge trigger */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA0MFP_Msk) | SYS_GPA_MFPL_PA0MFP_INT0;
    GPIO_SetMode(PA, BIT0, GPIO_MODE_INPUT);
    GPIO_EnableEINT0(PA, 0, GPIO_INT_FALLING);
    NVIC_EnableIRQ(EINT0_IRQn);

    /* Configure PF0 as EINT5 pin and enable interrupt by rising and falling edge trigger */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~SYS_GPF_MFPL_PF0MFP_Msk) | SYS_GPF_MFPL_PF0MFP_INT5;
    GPIO_SetMode(PF, BIT0, GPIO_MODE_INPUT);
    GPIO_EnableEINT(PF, 0, GPIO_INT_BOTH_EDGE);
    NVIC_EnableIRQ(EINT5_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_HCLK, GPIO_DBCTL_DBCLKSEL_1);
    GPIO_ENABLE_DEBOUNCE(PA, BIT0);
    GPIO_ENABLE_DEBOUNCE(PB, BIT2);
    GPIO_ENABLE_DEBOUNCE(PC, BIT5);
    GPIO_ENABLE_DEBOUNCE(PF, BIT0);

    /* Waiting for interrupts */
    while (1);

}
