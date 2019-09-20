/***************************************************************************//**
 * @file     main.c
 * @brief
 *           Demonstrate how to upgrade firmware between USB device and PC through USB DFU( Device Firmware Upgrade) class.
 *           A windows tool is also included in this sample code to connect with USB device.
 *
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"
#include "fmc_user.h"
#include "dfu_transfer.h"

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
    WDT->CTL &= ~(WDT_CTL_WDTEN_Msk);
    WDT->CTL |= (WDT_TIMEOUT_2POW18 | WDT_ALTCTL_RSTDSEL_Pos);
    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_HXTEN_Msk);
    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV0_HCLK(1));
    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;
    /* Set PLL frequency */
    CLK->PLLCTL = CLK_PLLCTL_84MHz_HXT;
    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);
    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));
    /* Enable IP clock */
    CLK_EnableModuleClock(USBD_MODULE);
    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));
    /* Enable USB PHY */
    SYS->USBPHY = 0x100;  // USB device
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
//    SYS->GPG_MFPL = SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;
    /* Lock protected registers */
    //SYS_LockReg();
}


void USBD_IRQHandler(void);
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
//  printf("g_apromSize = %X, g_dataFlashAddr = %X, g_dataFlashSize = %X,\n", g_apromSize, g_dataFlashAddr, g_dataFlashSize);

    if (DetectPin != 0)
    {
        goto _APROM;
    }

    USBD_Open(&gsInfo, DFU_ClassRequest, NULL);
    /* Endpoint configuration */
    DFU_Init();
    /* Enable USBD interrupt */
    NVIC_EnableIRQ(USBD_IRQn);
    /* Start transaction */
    USBD_Start();

    /* polling USBD interrupt flag */
    while (DetectPin == 0)
    {
        USBD_IRQHandler();
    }

_APROM:
    outpw(&SYS->RSTSTS, 3);//clear bit
    outpw(&FMC->ISPCTL, inpw(&FMC->ISPCTL) & 0xFFFFFFFC);
    outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

    /* Trap the CPU */
    while (1);
}
