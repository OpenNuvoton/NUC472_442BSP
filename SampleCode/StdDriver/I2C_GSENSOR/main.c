/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 14/05/30 6:00p $
 * @brief    Read G-sensor (DMARD08) data via I2C interface
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "NUC472_442.h"

#define DMARD05_I2C_SLAVE_ADDR 0x38  //I2C slave address

//Start register address for T, X, Y, Z
#define DMARD05_T_REG_START_ADDRESS 0x00
#define DMARD05_X_REG_START_ADDRESS 0x02
#define DMARD05_Y_REG_START_ADDRESS 0x04
#define DMARD05_Z_REG_START_ADDRESS 0x06

// Read value from DMARD08 via I2C
void Multi_Read_TXYZ(uint8_t u8RegAddr, uint16_t *u16Data)
{
    uint16_t volatile temp0, temp1;

    I2C_START(I2C3);
    I2C_WAIT_READY(I2C3);

    I2C_SET_DATA(I2C3, DMARD05_I2C_SLAVE_ADDR);
    I2C_SET_CONTROL_REG(I2C3, I2C_SI);
    I2C_WAIT_READY(I2C3);

    I2C_SET_DATA(I2C3, u8RegAddr);
    I2C_SET_CONTROL_REG(I2C3, I2C_SI);
    I2C_WAIT_READY(I2C3);

    I2C_SET_CONTROL_REG(I2C3, I2C_STA | I2C_SI);
    I2C_WAIT_READY(I2C3);

    I2C_SET_DATA(I2C3, (DMARD05_I2C_SLAVE_ADDR) | 1);
    I2C_SET_CONTROL_REG(I2C3, I2C_SI);
    I2C_WAIT_READY(I2C3);

    // T
    I2C_SET_CONTROL_REG(I2C3, I2C_SI | I2C_AA);
    I2C_WAIT_READY(I2C3);
    temp0 = I2C_GET_DATA(I2C3);

    I2C_SET_CONTROL_REG(I2C3, I2C_SI | I2C_AA);
    I2C_WAIT_READY(I2C3);
    temp1 = I2C_GET_DATA(I2C3);
    *(u16Data) = (temp0 << 3) | temp1;

    // X
    I2C_SET_CONTROL_REG(I2C3, I2C_SI | I2C_AA);
    I2C_WAIT_READY(I2C3);
    temp0 = I2C_GET_DATA(I2C3);

    I2C_SET_CONTROL_REG(I2C3, I2C_SI | I2C_AA);
    I2C_WAIT_READY(I2C3);
    temp1 = I2C_GET_DATA(I2C3);
    *(u16Data+1) = (temp0 << 3) | temp1;

    // Y
    I2C_SET_CONTROL_REG(I2C3, I2C_SI | I2C_AA);
    I2C_WAIT_READY(I2C3);
    temp0 = I2C_GET_DATA(I2C3);

    I2C_SET_CONTROL_REG(I2C3, I2C_SI | I2C_AA);
    I2C_WAIT_READY(I2C3);
    temp1 = I2C_GET_DATA(I2C3);
    *(u16Data+2) = (temp0 << 3) | temp1;

    // Z
    I2C_SET_CONTROL_REG(I2C3, I2C_SI | I2C_AA);
    I2C_WAIT_READY(I2C3);
    temp0 = I2C_GET_DATA(I2C3);

    I2C_SET_CONTROL_REG(I2C3, I2C_SI);
    I2C_WAIT_READY(I2C3);
    temp1 = I2C_GET_DATA(I2C3);
    *(u16Data+3) = (temp0 << 3) | temp1;

    I2C_STOP(I2C3);
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

    /* Lock protected registers */
    SYS_LockReg();
}

void I2C3_Init(void)
{
    /* Open I2C3 and set clock to 100k */
    I2C_Open(I2C3, 100000);

    /* Set I2C3 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C3, 0, 0x15, I2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C3, 1, 0x35, I2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C3, 2, 0x55, I2C_GCMODE_DISABLE);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C3, 3, 0x75, I2C_GCMODE_DISABLE);   /* Slave Address : 0x75 */
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    uint16_t buffer[4]= {0x00,0x00,0x00,0x00};
    int16_t temp, x, y, z;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Init I2C3 to access GSensor */
    I2C3_Init();

    while(1)
    {
        Multi_Read_TXYZ(DMARD05_T_REG_START_ADDRESS, buffer);

        if(buffer[0] & 0x400)
            temp = (int16_t)(0xF800 | buffer[0]);
        else
            temp = (int16_t)(buffer[0]);

        printf("[H********G-Sensor Status*********\n");
        printf("[Temperature] \n");
        printf(" %.2f 'C\n", (float)temp/(float)16 + (float)25);

        printf("\n[Acceleration] \n");

        if(buffer[1] & 0x400)
            x = (int16_t)(0xF800 | buffer[1]);
        else
            x = (int16_t)(buffer[1]);

        if(buffer[2] & 0x400)
            y = (int16_t)(0xF800 | buffer[2]);
        else
            y = (int16_t)(buffer[2]);

        if(buffer[3] & 0x400)
            z = (int16_t)(0xF800 | buffer[3]);
        else
            z = (int16_t)(buffer[3]);

        printf(" (%.3fg, %.3fg, %.3fg)\n", (float)x/(float)256, (float)y/(float)256, (float)z/(float)256);
        printf("********************************\n");
    }
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
