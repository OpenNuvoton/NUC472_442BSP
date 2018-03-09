/******************************************************************************
 * @file     main.c
 * @version  1.0.0
 * @date     23, Sep, 2014
 * @brief    This is an UAC1.0 sample and used to plays the sound send from PC
 *           through the USB interface
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"
#include "usbd_audio.h"

#define PLLCON_SETTING      CLK_PLLCON_84MHz_HXT
#define PLL_CLOCK           84000000
/*--------------------------------------------------------------------------*/
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

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);
    CLK_EnableModuleClock(I2C3_MODULE);
    CLK_EnableModuleClock(I2S1_MODULE);
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Enable USB PHY */
    SYS->USBPHY = 0x100;  // USB device

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

    /* Set multi function pin for I2S1 */
    /* GPG7, GPG8, GPG9, GPI11, GPI12 */
    SYS->GPG_MFPL = (SYS->GPG_MFPL & ~SYS_GPG_MFPL_PG7MFP_Msk) | SYS_GPG_MFPL_PG7MFP_I2S1_MCLK;
    SYS->GPG_MFPH = (SYS->GPG_MFPH & ~(SYS_GPG_MFPH_PG8MFP_Msk | SYS_GPG_MFPH_PG9MFP_Msk)) | (SYS_GPG_MFPH_PG8MFP_I2S1_DO | SYS_GPG_MFPH_PG9MFP_I2S1_DI);
    SYS->GPI_MFPH = (SYS->GPI_MFPH & ~(SYS_GPI_MFPH_PI11MFP_Msk | SYS_GPI_MFPH_PI12MFP_Msk)) | (SYS_GPI_MFPH_PI11MFP_I2S1_BCLK | SYS_GPI_MFPH_PI12MFP_I2S1_LRCK);

    /* Lock protected registers */
    SYS_LockReg();
}

void I2C3_Init(void)
{
    /* Open I2C3 and set clock to 100k */
    I2C_Open(I2C3, 400000);

    /* Get I2C3 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C3));

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
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("NUC472/NUC442 USB UAC - Microphone\n");

    /* Init I2C3 to access WAU8822 */
    I2C3_Init();

    /* Plug-In DET */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA4MFP_Msk));
    GPIO_SetMode(PA, BIT4, GPIO_MODE_OUTPUT);
    PA4 = 1;

    /* Open I2S1 as slave mode */
    I2S_Open(I2S1, I2S_MODE_SLAVE, 16000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S, I2S_I2S);

    /* select I2S1 clock source is from HXT(12MHz) */
    CLK_SetModuleClock(I2S1_MODULE, CLK_CLKSEL3_I2S1SEL_HXT, 0);

    /* Initialize WAU8822 codec */
    WAU8822_Setup();

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(I2S1, 12000000);

    /* Configure PDMA */
    PDMA_Init();

    USBD_Open(&gsInfo, UAC_ClassRequest, UAC_SetInterface);

    /* Endpoint configuration */
    UAC_Init();
    NVIC_SetPriority (USBD_IRQn, (1<<__NVIC_PRIO_BITS) - 2);
    NVIC_EnableIRQ(USBD_IRQn);
    USBD_Start();
    while(1)
    {
        if (g_usbd_UsbAudioState == UAC_START_AUDIO_RECORD && g_usbd_txflag)
        {
            UAC_SendRecData();
        }
    }
}



/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

