/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  0x32
 * @date     14, June, 2017
 *
 * @note
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "targetdev.h"
#include "i2c_transfer.h"

#define PLLCON_SETTING      CLK_PLLCTL_50MHz_HIRC
#define PLL_CLOCK           50191800
#define HCLK_DIV                        1

uint32_t Pclk0;
uint32_t Pclk1;


void SYS_Init(void)
{
    SYS_UnlockReg();
    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    /* Enable Internal and External XTAL 12MHz clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for external XTAL clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCON_SETTING;

    while (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL,CLK_CLKDIV0_HCLK(1));

    //SystemCoreClockUpdate();
    SystemCoreClock = PLL_CLOCK;
    CyclesPerUs     = SystemCoreClock / 1000000;  // For SYS_SysTickDelay()

    /* Enable I2C1 clock */
    CLK_EnableModuleClock(I2C1_MODULE);
    /* Set I2C1 multi-function pins */
    SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH1MFP_Msk | SYS_GPH_MFPL_PH0MFP_Msk)) |
                    (SYS_GPH_MFPL_PH1MFP_I2C1_SDA | SYS_GPH_MFPL_PH0MFP_I2C1_SCL);
    /* I2C pin enable schmitt trigger */
    PH->SMTEN |= GPIO_SMTEN_SMTEN0_Msk | GPIO_SMTEN_SMTEN1_Msk;
}

int main(void)
{
    uint32_t cmd_buff[16];
    SYS_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    I2C_Init();
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while (1)
    {
        if (bI2cDataReady == 1)
        {
            goto _ISP;
        }

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            goto _APROM;
        }

    }

_ISP:

    while (1)
    {
        if (bI2cDataReady == 1)
        {
            memcpy(cmd_buff, i2c_rcvbuf, 64);
            bI2cDataReady = 0;
            ParseCmd((unsigned char *)cmd_buff, 64);
        }
    }

_APROM:
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    FMC->ISPCTL &= ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while (1);
}
