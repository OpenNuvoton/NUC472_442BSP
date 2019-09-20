/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Date: 16/08/02 5:11p $
 * @brief    Use SD as back end storage media to simulate a
 *           30 KB USB pen drive
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC472_442.h"
#include "massstorage.h"

uint8_t volatile g_u8SdInitFlag = 0;
extern int32_t g_TotalSectors;
/*--------------------------------------------------------------------------*/

void SD_IRQHandler(void)
{
    unsigned int volatile isr;
    unsigned int volatile ier;

    // FMI data abort interrupt
    if (SD->GINTSTS & SDH_GINTSTS_DTAIF_Msk)
    {
        /* ResetAllEngine() */
        SD->GCTL |= SDH_GCTL_GCTLRST_Msk;
    }

    //----- SD interrupt status
    isr = SD->INTSTS;
    if (isr & SDH_INTSTS_BLKDIF_Msk)
    {
        extern uint8_t volatile _sd_SDDataReady;
        _sd_SDDataReady = TRUE;
        // block down
        SD->INTSTS = SDH_INTSTS_BLKDIF_Msk;
    }

    if (isr & SDH_INTSTS_CDIF0_Msk)
    { // card detect
        //----- SD interrupt status
        // it is work to delay 50 times for SD_CLK = 200KHz
        {
            int volatile i;         // delay 30 fail, 50 OK
            for (i=0; i<0x500; i++);  // delay to make sure got updated value from REG_SDISR.
            isr = SD->INTSTS;
        }

        if (isr & SDH_INTSTS_CDSTS0_Msk)
        {
            printf("\n***** card remove !\n");
            SD0.IsCardInsert = FALSE;   // SDISR_CD_Card = 1 means card remove for GPIO mode
            memset(&SD0, 0, sizeof(SD_INFO_T));
        }
        else
        {
            printf("***** card insert !\n");
            SD_Open(SD_PORT0 | CardDetect_From_GPIO);
            SD_Probe(SD_PORT0);
            if (SD_GET_CARD_CAPACITY(SD_PORT0) == 0)
            {
                g_u8SdInitFlag = 0;
                printf("SD initial fail!!\n");
            }
            else
            {
                g_u8SdInitFlag = 1;
                g_TotalSectors = SD_DiskInfo0.diskSize;
            }
        }
        SD->INTSTS = SDH_INTSTS_CDIF0_Msk;
    }

    // CRC error interrupt
    if (isr & SDH_INTSTS_CRCIF_Msk)
    {
        if (!(isr & SDH_INTSTS_CRC16_Msk))
        {
            //printf("***** ISR sdioIntHandler(): CRC_16 error !\n");
            // handle CRC error
        }
        else if (!(isr & SDH_INTSTS_CRC7_Msk))
        {
            extern uint32_t _sd_uR3_CMD;
            if (! _sd_uR3_CMD)
            {
                //printf("***** ISR sdioIntHandler(): CRC_7 error !\n");
                // handle CRC error
            }
        }
        SD->INTSTS = SDH_INTSTS_CRCIF_Msk;      // clear interrupt flag
    }

    if (isr & SDH_INTSTS_DINTOIF_Msk)
    {
        printf("***** ISR: data in timeout !\n");
        SD->INTSTS |= SDH_INTSTS_DINTOIF_Msk;
    }

    // Response in timeout interrupt
    if (isr & SDH_INTSTS_RTOIF_Msk)
    {
        printf("***** ISR: response in timeout !\n");
        SD->INTSTS |= SDH_INTSTS_RTOIF_Msk;
    }
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
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));

    /* Enable USB PHY */
    SYS->USBPHY = 0x100;  // USB device

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~(CLK_CLKSEL0_SDHSEL_Msk)); // SD clock from XTL(12MHz)
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF2MFP_Msk)) | (0x4 << SYS_GPF_MFPL_PF2MFP_Pos); // SD0_DAT3: GPF_2
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF3MFP_Msk)) | (0x4 << SYS_GPF_MFPL_PF3MFP_Pos); // SD0_DAT2: GPF_3
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF4MFP_Msk)) | (0x4 << SYS_GPF_MFPL_PF4MFP_Pos); // SD0_DAT1: GPF_4
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF5MFP_Msk)) | (0x4 << SYS_GPF_MFPL_PF5MFP_Pos); // SD0_DAT0: GPF_5
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF6MFP_Msk)) | (0x4 << SYS_GPF_MFPL_PF6MFP_Pos); // SD0_CDn : GPF_6
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF7MFP_Msk)) | (0x4 << SYS_GPF_MFPL_PF7MFP_Pos); // SD0_CMD : GPF_7
    SYS->GPF_MFPH = (SYS->GPF_MFPH & ~(SYS_GPF_MFPH_PF8MFP_Msk)) | (0x4 << SYS_GPF_MFPH_PF8MFP_Pos); // SD0_CLK : GPF_8
    CLK->AHBCLK |= CLK_AHBCLK_SDHCKEN_Msk; // SD Card driving clock.

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

    /* Lock protected registers */
    SYS_LockReg();
}

extern uint8_t volatile g_u8MscStart;

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

    printf("NUC472/NUC442 USB Mass Storage + SD\n");

    /* initial SD card */
    SD_ENABLE_INT(SDH_INTEN_CDIEN0_Msk);
    NVIC_EnableIRQ(SD_IRQn);
    SD_Open(SD_PORT0 | CardDetect_From_GPIO);
    SD_Probe((SD_PORT0 | CardDetect_From_GPIO) & 0x00ff);

    if (SD_GET_CARD_CAPACITY(SD_PORT0) == 0)
    {
        g_u8SdInitFlag = 0;
        printf("SD initial fail!!\n");
    }
    else
        g_u8SdInitFlag = 1;

    USBD_Open(&gsInfo, MSC_ClassRequest, NULL);

    /* Endpoint configuration */
    MSC_Init();

    /* Enable USBD interrupt */
    NVIC_EnableIRQ(USBD_IRQn);

    /* Start transaction */
    while(1)
    {
        if (USBD_IS_ATTACHED())
        {
            USBD_Start();
            break;
        }
    }

    while(1)
    {
        if (g_u8MscStart)
            MSC_ProcessCmd();
    }
}



/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

