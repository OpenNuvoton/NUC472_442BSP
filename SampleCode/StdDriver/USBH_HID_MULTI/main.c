/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/11/27 11:08a $
 * @brief    Sample shows how to use USB Host driver to manage multiple USB HID devices.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC472_442.h"
#include "usbh_core.h"
#include "usbh_hid.h"


#define USBH_CLK_FROM_PLL
//#define USBH_CLK_FROM_PLL2_DUAL_ROLE

#ifdef USBH_CLK_FROM_PLL
#define PLLCON_SETTING      0xc22e
#define PLL_CLOCK           48000000
#else
#define PLLCON_SETTING      0x4228
#define PLL_CLOCK           84000000
#endif

uint32_t PllClock         = PLL_CLOCK;


uint8_t  desc_buff[1024];


void Delay(uint32_t delayCnt)
{
    while(delayCnt--) {
        __NOP();
        __NOP();
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

#ifdef USBH_CLK_FROM_PLL

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

#endif

#ifdef USBH_CLK_FROM_PLL2_DUAL_ROLE

    SYS->USBPHY |= SYS_USBPHY_LDO33EN_Msk;

    // Enable OTG
    CLK->APBCLK0 |= CLK_APBCLK0_OTGCKEN_Msk;

    OTG->PHYCTL = (OTG->PHYCTL | OTG_PHYCTL_OTGPHYEN_Msk) & ~OTG_PHYCTL_IDDETEN_Msk;

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

#endif

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


void  int_read_callback(HID_DEV_T *hdev, uint8_t *rdata, int data_len)
{
    int  i;
    printf("Device [0x%x,0x%x] %d bytes received =>\n",
           hdev->udev->descriptor.idVendor, hdev->udev->descriptor.idProduct, data_len);
    for (i = 0; i < data_len; i++)
        printf("0x%02x ", rdata[i]);
    printf("\n");
}

static uint8_t  _write_data_buff[4];

void  int_write_callback(HID_DEV_T *hdev, uint8_t **wbuff, int *buff_size)
{
    printf("INT-out pipe request to write data.\n");

    *wbuff = &_write_data_buff[0];
    *buff_size = 4;
}


int  init_hid_device(HID_DEV_T *hdev)
{
    int   i, ret;

    printf("\n\n==================================\n");
    printf("  Init HID device : 0x%x\n", (int)hdev);
    printf("  VID: 0x%x, PID: 0x%x\n\n", hdev->udev->descriptor.idVendor, hdev->udev->descriptor.idProduct);

    ret = HID_HidGetReportDescriptor(hdev, desc_buff, 1024);
    if (ret > 0) {
        printf("\nDump report descriptor =>\n");
        for (i = 0; i < ret; i++) {
            if ((i % 16) == 0)
                printf("\n");
            printf("%02x ", desc_buff[i]);
        }
        printf("\n\n");
    }

    /*
     *  Example: GET_PROTOCOL request.
     */
    ret = HID_HidGetProtocol(hdev, desc_buff);
    printf("[GET_PROTOCOL] ret = %d, protocol = %d\n", ret, desc_buff[0]);

    /*
     *  Example: SET_PROTOCOL request.
     */
    ret = HID_HidSetProtocol(hdev, desc_buff[0]);
    printf("[SET_PROTOCOL] ret = %d, protocol = %d\n", ret, desc_buff[0]);

    /*
     *  Example: GET_REPORT request on report ID 0x1, report type FEATURE.
     */
    ret = HID_HidGetReport(hdev, RT_FEATURE, 0x1, desc_buff, 64);
    if (ret > 0) {
        printf("[GET_REPORT] Data => ");
        for (i = 0; i < ret; i++)
            printf("%02x ", desc_buff[i]);
        printf("\n");
    }

    printf("\nUSBH_HidStartIntReadPipe...\n");
    if (USBH_HidStartIntReadPipe(hdev, int_read_callback) == HID_RET_OK) {
        printf("Interrupt in transfer started...\n");
    }

    //if (USBH_HidStartIntWritePipe(hdev, int_write_callback) == HID_RET_OK)
    //{
    //  printf("Interrupt out transfer started...\n");
    //}

    return 0;
}



/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int32_t main(void)
{
    HID_DEV_T    *hdev;

    /* Lock protected registers */
    if(SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\n");
    printf("+--------------------------------------+\n");
    printf("|                                      |\n");
    printf("|     USB Host HID sample program      |\n");
    printf("|                                      |\n");
    printf("+--------------------------------------+\n");

    USBH_Open();

#ifdef USBH_CLK_FROM_PLL2_DUAL_ROLE
    // Configure OTG function as Dual Role
    SYS_UnlockReg();
    SYS->USBPHY = 0x102;
#endif

    Delay(0x800000);

    USBH_HidInit();

    printf("Wait until any HID devices connected...\n");

    while (1) {
        if (USBH_ProcessHubEvents()) {           /* USB Host port detect polling and management */
            hdev = USBH_HidGetDeviceList();
            if (hdev == NULL)
                continue;

            while (hdev != NULL) {
                init_hid_device(hdev);

                if (hdev != NULL)
                    hdev = hdev->next;
            }
        }
    }
}


/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
