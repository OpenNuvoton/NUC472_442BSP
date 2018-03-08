/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/12/07 9:20a $
 * @brief    Sample ISP updater that reads firmware from pen driver and updates to APROM.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC472_442.h"
#include "diskio.h"
#include "ff.h"
#include "lw_usbh.h"
#include "lw_umass.h"

#define PLLCON_SETTING      0xc22e
#define PLL_CLOCK           48000000
#define HCLK_HZ             PLL_CLOCK


#define POWER_UP_DELAY      20            /* 200 ms delay after port power on       */
#define ISP_DETECT_TIME     200           /* 1000 ms to detect any USB pen drive    */

#define FIRMWARE_FILE_NAME  "AP.BIN"
#define DATA_FILE_NAME      "data.bin"

uint32_t PllClock         = PLL_CLOCK;

FILINFO Finfo;
FATFS FatFs[_VOLUMES];      /* File system object for logical drive */

static FIL file1;


#ifdef __ICCARM__
#pragma data_alignment=32
BYTE _Buff[FMC_FLASH_PAGE_SIZE];                /* Working buffer */
#else
BYTE _Buff[FMC_FLASH_PAGE_SIZE] __attribute__((aligned(32)));    /* Working buffer */
#endif


static volatile int  g_tick_cnt = 0;

void SysTick_Handler(void)
{
    g_tick_cnt++;
}

uint32_t get_sys_ticks(void)
{
    return g_tick_cnt;
}

void enable_sys_ticks(int ticks_per_second)
{
    uint32_t  systick_load;
    /*
     *  Configure system tick to be 0.1 second count
     */
    systick_load = ((HCLK_HZ/(2 * ticks_per_second)) & SysTick_LOAD_RELOAD_Msk) - 1;
    /* system tick from HCLK/2 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_STCLKSEL_Msk) | (3 << CLK_CLKSEL0_STCLKSEL_Pos  );
    SysTick->LOAD  = systick_load;
    NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority for Cortex-M0 System Interrupts */
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USB Host clock                                                                                     */
    /*---------------------------------------------------------------------------------------------------------*/

    // Configure OTG function as Host-Only
    SYS->USBPHY = 0x101;

    // Multi-function PB.0 OTG_5V status, PB.1 OTG_5V enable
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_USB0_OTG5V_ST | SYS_GPB_MFPL_PB1MFP_USB0_OTG5V_EN)) | 0x11;

    // Multi-function (PB.2,PB.3) for USB port 2 (D-,D+)
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB2MFP_USB1_D_N | SYS_GPB_MFPL_PB3MFP_USB1_D_P)) | 0x3300;

    // Set PB.4 output high to enable USB power
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB4MFP_Msk;
    PB->MODE = (PB->MODE & ~GPIO_MODE_MODE4_Msk) | (0x1 << GPIO_MODE_MODE4_Pos);
    PB->DOUT |= 0x10;

    // Select USB Host clock source from PLL
    CLK->CLKSEL0 |= CLK_CLKSEL0_USBHSEL_Msk;

    // USB clock is HCLK divided by 1
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_USBHDIV_Msk) | (0 << CLK_CLKDIV0_USBHDIV_Pos);

    // Enable USB Host
    CLK->AHBCLK |= CLK_AHBCLK_USBHCKEN_Msk;

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


/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */
unsigned long get_fattime (void)
{
    return 0x0000;
}


void  do_dir()
{
    long    p1;
    FRESULT res;
    char    dir_path[2] = { 0 };    /* root directory */

    DIR dir;                        /* Directory object */
    UINT s1, s2;

    if (f_opendir(&dir, dir_path))
        return;

    p1 = s1 = s2 = 0;
    for (; ;)
    {
        res = f_readdir(&dir, &Finfo);
        if ((res != FR_OK) || !Finfo.fname[0]) break;
        if (Finfo.fattrib & AM_DIR)
        {
            s2++;
        }
        else
        {
            s1++;
            p1 += Finfo.fsize;
        }
        printf("%c%c%c%c%c %u/%02u/%02u %02u:%02u %9lu  %s",
               (Finfo.fattrib & AM_DIR) ? 'D' : '-',
               (Finfo.fattrib & AM_RDO) ? 'R' : '-',
               (Finfo.fattrib & AM_HID) ? 'H' : '-',
               (Finfo.fattrib & AM_SYS) ? 'S' : '-',
               (Finfo.fattrib & AM_ARC) ? 'A' : '-',
               (Finfo.fdate >> 9) + 1980, (Finfo.fdate >> 5) & 15, Finfo.fdate & 31,
               (Finfo.ftime >> 11), (Finfo.ftime >> 5) & 63, Finfo.fsize, Finfo.fname);
#if _USE_LFN
        for (p2 = strlen(Finfo.fname); p2 < 14; p2++)
            putchar(' ');
        printf("%s\n", Lfname);
#else
        putchar('\n');
#endif
    }
    printf("%4u File(s),%10lu bytes total\n%4u Dir(s)", s1, p1, s2);
}


int  program_flash_page(uint32_t page_addr, uint32_t *buff, int count)
{
    uint32_t  addr;
    uint32_t  *p;

    printf("Program page 0x%x, count=%d\n", page_addr, count);

    p = buff;
    FMC_Erase(page_addr);
    for (addr = page_addr; addr < page_addr+count; addr += 4, p++)
    {
        FMC_Write(addr, *p);
    }

    p = buff;
    for (addr = page_addr; addr < page_addr+count; addr += 4, p++)
    {
        if (FMC_Read(addr) != *p)
        {
            printf("Verify failed at 0x%x, read:0x%x, epect:0x%x\n", addr, FMC_Read(addr), *p);
            return -1;
        }
    }
    return 0;
}


int isp_update_by_usb()
{
    uint32_t   addr, dfba;
    UINT       cnt;
    FRESULT    res;

    printf("rc=%d\n", (WORD)disk_initialize(0));
#if (_FATFS == 82786)
    f_mount(0, &FatFs[0]);
#else
    f_mount(&FatFs[0], "", 0);
#endif
    do_dir();

    if (f_open(&file1, FIRMWARE_FILE_NAME, FA_OPEN_EXISTING | FA_READ))
    {
        printf("Firmware %s file not found.\n", FIRMWARE_FILE_NAME);
    }
    else
    {
        printf("Firmare file found, start update firmware...\n");
        /*
         *  Update APROM
         */
        for (addr = 0; ; addr += FMC_FLASH_PAGE_SIZE)
        {
            cnt = FMC_FLASH_PAGE_SIZE;
            res = f_read(&file1, _Buff, cnt, &cnt);
            if ((res == FR_OK) && cnt)
            {
                if (program_flash_page(addr, (uint32_t *)_Buff, cnt) != 0)
                {
                    f_close(&file1);
                    return 0;
                }
            }
            else
                break;
        }

        if (f_eof(&file1))
            printf("Success.\n");
        else
            printf("Update failed!\n");

        f_close(&file1);
    }

    if (FMC_Read(0x300000) & 0x1)
    {
        printf("Data flash is not enabled.\n");
        return 0;
    }

    dfba = FMC_ReadDataFlashBaseAddr();

    if (f_open(&file1, DATA_FILE_NAME, FA_OPEN_EXISTING | FA_READ))
    {
        printf("Data file %s not found.\n", DATA_FILE_NAME);
    }
    else
    {
        /*
         *  Update Data Flash
         */
        for (addr = dfba; ; addr += FMC_FLASH_PAGE_SIZE)
        {
            cnt = FMC_FLASH_PAGE_SIZE;
            res = f_read(&file1, _Buff, cnt, &cnt);
            if ((res == FR_OK) && cnt)
            {
                if (program_flash_page(addr, (uint32_t *)_Buff, cnt) != 0)
                {
                    f_close(&file1);
                    return 0;
                }
            }
            else
                break;
        }

        if (f_eof(&file1))
            printf("Success.\n");
        else
            printf("Update failed!\n");

        f_close(&file1);
    }
    return 0;
}



/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int32_t main(void)
{
    int   t0;

    /* Lock protected registers */
    if(SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART0 for printf */
    UART0_Init();

    printf("+------------------------+\n");
    printf("|  NUC4xx ISP updater    |\n");
    printf("+------------------------+\n");

    /* Enable FMC ISP function */
    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    enable_sys_ticks(100);

    if ((FMC_Read(FMC_CONFIG_BASE) & 0xc0) != 0x40)
    {
        printf("CONFIG0 = 0x%x\n", FMC_Read(FMC_CONFIG_BASE));
        printf("This program must be running under \"Boot from LDROM without IAP\" mode!\n");
        printf("Please modify User Configuration...\n");
        while (1);
    }

    usbh_init();

    t0 = get_sys_ticks();

    while (get_sys_ticks() - t0 < POWER_UP_DELAY) ;

    printf("Detecting USB disk...\n");

    /*
     *  Detecting USB mass storage device...
     */
    t0 = get_sys_ticks();
    while (get_sys_ticks() - t0 < ISP_DETECT_TIME)
    {
        if ((usbh_probe_port(1) == 0) || (usbh_probe_port(0) == 0))
        {
            if (usbh_probe_umass() == 0)
            {
                isp_update_by_usb();
                break;
            }
        }
    }

    if (get_sys_ticks() - t0 >= ISP_DETECT_TIME)
        printf("\n\nUSB disk not found. ");

    printf("Will branch to user application...\n");

    // reboot from APROM...
    FMC_SET_APROM_BOOT();
    SYS->IPRST0 = SYS_IPRST0_CPURST_Msk;
}


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
