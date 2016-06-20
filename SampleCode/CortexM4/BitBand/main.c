/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 14/10/06 9:44a $
 * @brief    Demonstrate the usage of Cortex?-M4 BitBand.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* Memory Address */
#define MEM_ADDR(address)       *((volatile unsigned long *) (address))
/* BitBand Address */
#define BITBAND(address,bitnum) ((address & 0xF0000000)+0x02000000+((address & 0xFFFFF)<<5)+(bitnum<<2))

void SYS_Init(void)
{
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

void BitBand_Test(void)
{
    uint32_t u32Temp;
    uint32_t u32TestAddress;

    u32TestAddress = 0x20003C00;

    //Read SRAM Address(0x20003F00)
    u32Temp = MEM_ADDR(u32TestAddress);
    printf("\n The value at test address (0x20003F00) is: 0x%x", u32Temp);

    //Use BitBand function to read SRAM bit value
    u32Temp = MEM_ADDR(BITBAND(u32TestAddress, 3));
    printf("\n Use Bit-Band function to get value at bit 3: %x \n", u32Temp);

    //Use BitBand function set bit
    printf("\n Use Bit-Band function set test address (0x20003F00) bit 3 ");
    MEM_ADDR(BITBAND(u32TestAddress, 3)) = 1;
    //Read Test Address Value
    u32Temp = MEM_ADDR(u32TestAddress);
    printf("\n The value at address 0x20003F00 is: 0x%x \n", u32Temp);

    //Use BitBand function clear bit
    printf("\n Use Bit-Band function clear test address (0x20003F00) bit 3 ");
    MEM_ADDR(BITBAND(u32TestAddress, 3)) = 0;
    //Read Test Address Value
    u32Temp = MEM_ADDR(u32TestAddress);
    printf("\n The value at address 0x20003F00 is: 0x%x \n", u32Temp);
}

int main()
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n Start Bit-Band test: \n");

    BitBand_Test();

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
