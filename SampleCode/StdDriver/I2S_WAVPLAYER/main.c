/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 12 $
 * $Date: 15/12/08 9:47a $
 * @brief    This is a WAV file player which plays back WAV file stored in 
 *           USB pen drive.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC472_442.h"
#include "config.h"
#include "diskio.h"
#include "ff.h"
#include "usbh_core.h"
#include "usbh_umas.h"

#define NAU8822_ADDR        0x1A                /* NAU8822 Device ID */
#define PLLCON_SETTING      0x4228
#define PLL_CLOCK           84000000

DMA_DESC_T DMA_DESC[2];

uint32_t PllClock         = PLL_CLOCK;

//uint32_t PcmBuff[BUFF_LEN] = {0};
uint32_t volatile u32BuffPos = 0;
FATFS FatFs[_VOLUMES];      /* File system object for logical drive */
#ifdef __ICCARM__
#pragma data_alignment=32
BYTE Buff[1024] ;       /* Working buffer */
#endif

#ifdef __ARMCC_VERSION
__align(32) BYTE Buff[1024] ;       /* Working buffer */
#endif

uint8_t bAudioPlaying = 0;
extern signed int aPCMBuffer[2][PCM_BUFFER_SIZE];

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

unsigned long get_fattime (void)
{
    unsigned long tmr;

    tmr=0x00000;

    return tmr;
}

void Delay(int count)
{
    volatile uint32_t i;
    for (i = 0; i < count ; i++);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of WAU8822 with I2C0                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_WriteWAU8822(uint8_t u8addr, uint16_t u16data)
{

    I2C_START(I2C3);
    I2C_WAIT_READY(I2C3);

    I2C_SET_DATA(I2C3, NAU8822_ADDR<<1);
    I2C_SET_CONTROL_REG(I2C3, I2C_SI);
    I2C_WAIT_READY(I2C3);

    I2C_SET_DATA(I2C3, (uint8_t)((u8addr << 1) | (u16data >> 8)));
    I2C_SET_CONTROL_REG(I2C3, I2C_SI);
    I2C_WAIT_READY(I2C3);

    I2C_SET_DATA(I2C3, (uint8_t)(u16data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C3, I2C_SI);
    I2C_WAIT_READY(I2C3);

    I2C_STOP(I2C3);
}

void WAU8822_ConfigSampleRate(uint32_t u32SampleRate)
{
    if((u32SampleRate % 8) == 0) {
        I2C_WriteWAU8822(36, 0x008);    //12.288Mhz
        I2C_WriteWAU8822(37, 0x00C);
        I2C_WriteWAU8822(38, 0x093);
        I2C_WriteWAU8822(39, 0x0E9);
    } else {
        I2C_WriteWAU8822(36, 0x007);    //11.2896Mhz
        I2C_WriteWAU8822(37, 0x021);
        I2C_WriteWAU8822(38, 0x131);
        I2C_WriteWAU8822(39, 0x026);
    }

    switch (u32SampleRate) {
    case 16000:
        I2C_WriteWAU8822(6, 0x1AD);   /* Divide by 6, 16K */
        I2C_WriteWAU8822(7, 0x006);   /* 16K for internal filter coefficients */
        break;

    case 32000:
        I2C_WriteWAU8822(6, 0x16D);    /* Divide by 3, 32K */
        I2C_WriteWAU8822(7, 0x002);    /* 32K for internal filter coefficients */
        break;

    case 44100:
    case 48000:
        I2C_WriteWAU8822(6, 0x14D);    /* Divide by 1, 48K */
        I2C_WriteWAU8822(7, 0x000);    /* 48K for internal filter coefficients */
        break;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  WAU8822 Settings with I2C interface                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void WAU8822_Setup()
{
    printf("\nConfigure WAU8822 ...");

    I2C_WriteWAU8822(0,  0x000);   /* Reset all registers */
    Delay(0x200);

    I2C_WriteWAU8822(1,  0x02F);
    I2C_WriteWAU8822(2,  0x1B3);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteWAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */

    // offset: 0x4 => default, 24bit, I2S format, Stereo
    I2C_WriteWAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */

    I2C_WriteWAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteWAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    I2C_WriteWAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    I2C_WriteWAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteWAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteWAU8822(15, 0x1EF);   /* ADC left digital volume control */
    I2C_WriteWAU8822(16, 0x1EF);   /* ADC right digital volume control */

    I2C_WriteWAU8822(44, 0x000);   /* LLIN/RLIN is not connected to PGA */
    I2C_WriteWAU8822(47, 0x050);   /* LLIN connected, and its Gain value */
    I2C_WriteWAU8822(48, 0x050);   /* RLIN connected, and its Gain value */
    I2C_WriteWAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteWAU8822(51, 0x001);   /* Right DAC connected to RMIX */

    printf("[OK]\n");
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
    CLK->PLLCTL = PLLCON_SETTING;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLKSEL_PLL;

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(I2C3_MODULE);
    CLK_EnableModuleClock(I2S1_MODULE);
    CLK_EnableModuleClock(PDMA_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USB Host clock                                                                                     */
    /*---------------------------------------------------------------------------------------------------------*/
    SYS->USBPHY |= SYS_USBPHY_LDO33EN_Msk;

    // Configure OTG function as OTG
    SYS->USBPHY |= 0x3;

    // Enable OTG
    CLK->APBCLK0 |= CLK_APBCLK0_OTGCKEN_Msk;

    OTG->PHYCTL = (OTG->PHYCTL | OTG_PHYCTL_OTGPHYEN_Msk) & ~OTG_PHYCTL_IDDETEN_Msk;

    OTG->CTL |= OTG_CTL_OTGEN_Msk | OTG_CTL_BUSREQ_Msk;

    // Multi-function PB.0 OTG_5V status, PB.1 OTG_5V enable
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_USB0_OTG5V_ST | SYS_GPB_MFPL_PB1MFP_USB0_OTG5V_EN)) | 0x11;

    // Multi-function (PB.2,PB.3) for USB port 2 (D-,D+)
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB2MFP_USB1_D_N | SYS_GPB_MFPL_PB3MFP_USB1_D_P)) | 0x3300;

    // Set PB.4 output high to enable USB power
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB4MFP_Msk;
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE4_Msk) | (0x1 << GPIO_MODE_MODE4_Pos);
    PB->DOUT |= 0x10;

    // Enable PLL2, 480 MHz / 2 / (1+4) => 48 MHz output
    CLK->PLL2CTL = (4 << CLK_PLL2CTL_PLL2DIV_Pos);

    // wait PLL2 stable...
    while (!(CLK->STATUS & CLK_STATUS_PLL2STB_Msk));

    // USB clock divided by 1
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_USBHDIV_Msk) | (0 << CLK_CLKDIV0_USBHDIV_Pos);

    // Select USB Host clock source from PLL2
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_USBHSEL_Msk;

    // Enable USB Host
    CLK->AHBCLK |= CLK_AHBCLK_USBHCKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPG_MFPL = SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

    /* Set GPH3,GPH4 multi-function pins for I2C3 */
    SYS->GPH_MFPL = SYS_GPH_MFPL_PH3MFP_I2C3_SCL | SYS_GPH_MFPL_PH4MFP_I2C3_SDA ;

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
    I2C_Open(I2C3, 100000);

    /* Get I2C3 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C3));

    /* Set I2C3 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C3, 0, 0x15, I2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C3, 1, 0x35, I2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C3, 2, 0x55, I2C_GCMODE_DISABLE);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C3, 3, 0x75, I2C_GCMODE_DISABLE);   /* Slave Address : 0x75 */
}

void PDMA_Init(void)
{
    DMA_DESC[0].ctl = ((PCM_BUFFER_SIZE-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_INC|PDMA_DAR_FIX|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
    DMA_DESC[0].endsrc = (uint32_t)&aPCMBuffer[0][0] + PCM_BUFFER_SIZE * 4;
    DMA_DESC[0].enddest = (uint32_t)&I2S1->TX;
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1] - (PDMA->SCATBA);

    DMA_DESC[1].ctl = ((PCM_BUFFER_SIZE-1)<<PDMA_DSCT_CTL_TXCNT_Pos)|PDMA_WIDTH_32|PDMA_SAR_INC|PDMA_DAR_FIX|PDMA_REQ_SINGLE|PDMA_OP_SCATTER;
    DMA_DESC[1].endsrc = (uint32_t)&aPCMBuffer[1][0] + PCM_BUFFER_SIZE * 4;
    DMA_DESC[1].enddest = (uint32_t)&I2S1->TX;
    DMA_DESC[1].offset = (uint32_t)&DMA_DESC[0] - (PDMA->SCATBA);

    PDMA_Open(1 << 2);
    PDMA_SetTransferMode(2, PDMA_I2S1_TX, 1, (uint32_t)&DMA_DESC[0]);

    PDMA_EnableInt(2, 0);
    NVIC_EnableIRQ(PDMA_IRQn);
}

void PDMA_Reset_SCTable(uint8_t id)
{
    DMA_DESC[id].ctl |= PDMA_OP_SCATTER;
    DMA_DESC[id].ctl |= ((PCM_BUFFER_SIZE-1)<<PDMA_DSCT_CTL_TXCNT_Pos);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+------------------------------------------------------------------------+\n");
    printf("|                   I2S Driver Sample Code with WAU8822                  |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  NOTE: This sample code needs to work with WAU8822.\n");

    /* Configure USB Host */
    USBH_Open();

    // Configure OTG function as Dual Role
    SYS_UnlockReg();
    SYS->USBPHY = 0x102;

    USBH_MassInit();

    Delay(0x500000);
    USBH_ProcessHubEvents();

    /* Configure FATFS */
    printf("rc=%d\n", (WORD)disk_initialize(0));
    disk_read(0, Buff, 2, 1);
    f_mount(&FatFs[0], 0, 1);

    /* Init I2C3 to access WAU8822 */
    I2C3_Init();

    /* Configure as I2S slave */
    I2S_Open(I2S1, I2S_MODE_SLAVE, 16000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S, I2S_I2S);
    NVIC_EnableIRQ(I2S0_IRQn);

    // select source from HXT(12MHz)
    CLK_SetModuleClock(I2S1_MODULE, CLK_CLKSEL3_I2S1SEL_HXT, 0);

    /* Initialize WAU8822 codec */
    WAU8822_Setup();

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(I2S1, 12000000);

    /* Configure PDMA and use Scatter-Gather mode */
    PDMA_Init();

    // Plug-In DET, enable sound output (I3)
    SYS->GPI_MFPL = (SYS->GPI_MFPL & ~(SYS_GPI_MFPL_PI3MFP_Msk));
    GPIO_SetMode(GPI, BIT3, GPIO_MODE_OUTPUT);
    PI3 = 0;

    WAVPlayer();


    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
