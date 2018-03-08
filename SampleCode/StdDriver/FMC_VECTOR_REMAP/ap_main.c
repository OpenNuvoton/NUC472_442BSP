/**************************************************************************//**
 * @file     ap_main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 15/11/20 9:54a $
 * @brief    Show how to branch programs between LDROM, APROM start page,
 *           and APROM other page
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC472_442.h"
#include "map.h"

#define PLLCON_SETTING      CLK_PLLCTL_84MHz_HXT
#define PLL_CLOCK           84000000

uint32_t PllClock = PLL_CLOCK;


static int  load_image_to_flash(uint32_t image_base, uint32_t image_limit, uint32_t flash_addr, uint32_t max_size);


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

    /* Switch HCLK clock source to XTAL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT,CLK_CLKDIV0_HCLK(1));

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency */
    CLK->PLLCTL = PLLCON_SETTING;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UARTSEL_HXT,CLK_CLKDIV0_UART(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL |= SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

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
    UART0->LINE = 0x07;
    UART0->BAUD = 0x30000066;   /* 12MHz reference clock input, for 115200 */
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    int         cbs, ch;
    uint32_t    au32Config[4];

    /* Lock protected registers */
    if (SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART0 for printf */
    UART0_Init();

    /* Enable FMC ISP function */
    SYS_UnlockReg();
    FMC_Open();

    printf("\n\n");
    printf("+--------------------------------------------------+\n");
    printf("|         User program running on APROM            |\n");
    printf("+--------------------------------------------------+\n");

    /*-------------------------------------------------------------
     *  Modify CBS to 11b (boot from APROM)
     *------------------------------------------------------------*/
    if (FMC_ReadConfig(au32Config, 3) < 0)
    {
        printf("\n\nFailed to read Config!\n\n");
        return -1;
    }
    cbs = (au32Config[0] >> 6) & 0x3;
    printf("Config0 = 0x%x, Config1 = 0x%x, CBS=%d\n\n", au32Config[0], au32Config[1], cbs);

    if (cbs != 0x3)
    {
        printf("\n\nChange boot setting to [Boot from APROM].\n");
        FMC_ENABLE_CFG_UPDATE();
        au32Config[0] |= 0xc0;          /* set CBS to 11b */
        FMC_WriteConfig(au32Config, 4);
    }

    printf("\n\n");
    printf("+--------------------------------------+\n");
    printf("|                                      |\n");
    printf("|      FMC VECMAP sample main          |\n");
    printf("|      (Boot from APROM)               |\n");
    printf("|                                      |\n");
    printf("+--------------------------------------+\n");

    /*-------------------------------------------------------------
     *  Program LDROM image
     *------------------------------------------------------------*/
    printf("Writing fmc_ld_boot.bin image to LDROM...\n");
    FMC_ENABLE_LD_UPDATE();
    if (load_image_to_flash((uint32_t)&loaderImage1Base, (uint32_t)&loaderImage1Limit,
                            FMC_LDROM_BASE, FMC_LDROM_SIZE) != 0)
    {
        printf("Load image to LDROM failed!\n");
        return -1;
    }
    FMC_DISABLE_LD_UPDATE();

    /*-------------------------------------------------------------
     *  Program APROM ISP image
     *------------------------------------------------------------*/
    printf("Writing fmc_isp.bin image to APROM address 0x%x...\n", ISP_CODE_BASE);
    FMC_ENABLE_AP_UPDATE();
    if (load_image_to_flash((uint32_t)&loaderImage2Base, (uint32_t)&loaderImage2Limit,
                            ISP_CODE_BASE, ISP_CODE_MAX_SIZE) != 0)
    {
        printf("Load image to APROM failed!\n");
        return -1;
    }
    FMC_DISABLE_LD_UPDATE();

    printf("\n\nWill change boot setting to [Boot from LDROM with IAP]...(Yes/No?)");
    while (1)
    {
        ch = getchar();
        if ((ch == 'N') || (ch == 'n')) while (1);
        if ((ch == 'Y') || (ch == 'y')) break;
    }
    printf("\n\n");

    /*-------------------------------------------------------------
     *  Modify CBS to 00b (boot from LDROM with IAP)
     *------------------------------------------------------------*/
    if (FMC_ReadConfig(au32Config, 3) < 0)
    {
        printf("\n\nFailed to read Config!\n\n");
        return -1;
    }
    cbs = (au32Config[0] >> 6) & 0x3;
    printf("Config0 = 0x%x, Config1 = 0x%x, CBS=%d\n\n", au32Config[0], au32Config[1], cbs);

    if ((cbs != 0) || !(au32Config[0] & 0x1))
    {
        FMC_ENABLE_CFG_UPDATE();
        au32Config[0] &= ~0xc0;         /* set CBS to 00b */
        au32Config[0] |= 0x1;           /* disable Data Flash */
        FMC_WriteConfig(au32Config, 4);
    }

    printf("\nExecute chip reset, press any key to boot from LDROM with VECMAP...\n\n");
    getchar();

    // do chip reset
    SYS->IPRST0 |= SYS_IPRST0_CHIPRST_Msk;

    return 0;
}


static int  load_image_to_flash(uint32_t image_base, uint32_t image_limit, uint32_t flash_addr, uint32_t max_size)
{
    uint32_t   i, j, u32Data, u32ImageSize, *pu32Loader;

    u32ImageSize = image_limit - image_base;
    if (u32ImageSize == 0)
    {
        printf("  ERROR: Loader Image is 0 bytes!\n");
        return -1;
    }

    if (u32ImageSize > max_size)
    {
        printf("  ERROR: Loader Image is larger than %d KBytes!\n", max_size/1024);
        return -1;
    }

    printf("Program image to flash address 0x%x...", flash_addr);
    pu32Loader = (uint32_t *)image_base;
    for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        if (FMC_Erase(flash_addr + i))
        {
            printf("Erase failed on 0x%x\n", flash_addr + i);
            return -1;
        }

        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            FMC_Write(flash_addr + i + j, pu32Loader[(i + j) / 4]);
        }
    }
    printf("OK.\n");

    printf("Verify ...");

    /* Verify loader */
    for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            u32Data = FMC_Read(flash_addr + i + j);
            if (u32Data != pu32Loader[(i+j)/4])
            {
                printf("data mismatch on 0x%x, [0x%x], [0x%x]\n", flash_addr + i + j, u32Data, pu32Loader[(i+j)/4]);
                return -1;
            }

            if (i + j >= u32ImageSize)
                break;
        }
    }
    printf("OK.\n");
    return 0;
}
