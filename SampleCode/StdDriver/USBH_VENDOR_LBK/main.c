/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/10/06 11:22a $
 * @brief    A USB host vendor class sample program. This sample code needs
 *           to test with USBD_VENDOR_LBK.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC472_442.h"
#include "usbh_core.h"
#include "lbk_driver.h"

#define USBH_CLK_FROM_PLL
//#define USBH_CLK_FROM_PLL2_DUAL_ROLE
//#define USBH_CLK_FROM_PLL2_OTG

#ifdef USBH_CLK_FROM_PLL
#define PLLCON_SETTING      0xc22e
#define PLL_CLOCK           48000000
#else
#define PLLCON_SETTING      0x4228
#define PLL_CLOCK           84000000
#endif

uint32_t PllClock         = PLL_CLOCK;



#define DO_CTRL_XFER
#define DO_INT_XFER
#define DO_BULK_XFER
#define DO_ISO_XFER

#define TEST_BUFF_SIZE      512

static uint8_t  g_OutBuff[TEST_BUFF_SIZE];
static uint8_t  g_InBuff[TEST_BUFF_SIZE];

static volatile int g_BulkInBusy = 0;

int  ctrl_cnt, int_in_cnt, int_out_cnt, iso_in_cnt, iso_out_cnt, bulk_cnt;


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
    SYS->GPG_MFPL = SYS_GPG_MFPL_PG1MFP_UART0_RXD | SYS_GPG_MFPL_PG2MFP_UART0_TXD ;

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

#ifdef USBH_CLK_FROM_PLL2_OTG

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


/*
 *  Interrupt-In transfer data delivery callback function.
 */
void int_xfer_read(uint8_t *data_buff, int *data_len)
{
    //  Receive interrupt in transfer data. The data len is (*data_len).
    //  NOTICE: This callback function is in USB Host interrupt context!

    int_in_cnt++;
}


/*
 *  Interrupt-Out transfer data request callback function.
 */
void int_xfer_write(uint8_t *data_buff, int *data_len)
{
    //  LBK request the interrupt out transfer data.
    //  NOTICE: This callback function is in USB Host interrupt context!
    int_out_cnt++;
    *data_len = 8;
    memset(data_buff, int_out_cnt & 0xff, 8);
}

/*
 *  Isochronous-Out transfer data request callback function.
 */
void iso_xfer_write(uint8_t *data_buff, int data_len)
{
    //  Application feeds Isochronous-Out data here.
    //  NOTICE: This callback function is in USB Host interrupt context!
    iso_out_cnt++;
    memset(data_buff, iso_out_cnt & 0xff, data_len);
}

/*
 *  Isochronous-In transfer data request callback function.
 */
void iso_xfer_read(uint8_t *data_buff, int data_len)
{
    //  Application gets Isochronous-In data here.
    //  NOTICE: This callback function is in USB Host interrupt context!
    iso_in_cnt++;
}

/*
 *  Bulk-In transfer status callback function.
 */
void bulk_in_xfer_status(int status)
{
    if (status == 0) {
        bulk_cnt++;
        g_BulkInBusy = 0;
    } else {
        printf("Bulk-In transfer failed, status=%d\n", status);
    }
}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int32_t main(void)
{
    int  loop, connected;

    /* Lock protected registers */
    if(SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\n");
    printf("+-----------------------------------------------+\n");
    printf("|                                               |\n");
    printf("|     USB Host Vendor Class sample program      |\n");
    printf("|                                               |\n");
    printf("+-----------------------------------------------+\n");

    printf("OTG->PHYCTL = 0x%x\n", OTG->PHYCTL);
    printf("OTG->STS = 0x%x\n", OTG->STATUS);
    printf("OTG->CTL = 0x%x\n", OTG->CTL);

    USBH_Open();

#ifdef USBH_CLK_FROM_PLL2_DUAL_ROLE
    // Configure OTG function as Dual Role
    SYS_UnlockReg();
    SYS->USBPHY = 0x102;
#endif

    LBK_Init();

    connected = 0;
    ctrl_cnt = int_in_cnt = int_out_cnt = iso_in_cnt = iso_out_cnt = bulk_cnt = 0;

    for (loop = 0; ; loop++) {
        USBH_ProcessHubEvents();

        if (LBK_IsConnected()) {
            if (!connected) {
#ifdef DO_INT_XFER
                LBK_InstallIntInFunc(int_xfer_read);
                LBK_InstallIntOutFunc(int_xfer_write);
                LBK_StartIntInPipe();
                LBK_StartIntOutPipe();
#endif

#ifdef DO_ISO_XFER
                LBK_InstallIsoOutFunc(iso_xfer_write);
                LBK_InstallIsoInFunc(iso_xfer_read);
                LBK_StartIsoInPipe();
                LBK_StartIsoOutPipe();
#endif
                connected = 1;
            }
        } else {
            connected = 0;
            ctrl_cnt = int_in_cnt = int_out_cnt = iso_in_cnt = iso_out_cnt = bulk_cnt = 0;
            continue;
        }

#ifdef DO_CTRL_XFER
        /*--------------------------------------------*/
        /*  Control transfer loopback demo            */
        /*--------------------------------------------*/
        memset(g_OutBuff, loop & 0xff, 64);
        if (LBK_CtrlSetData(g_OutBuff))
            printf("Control out error!\n");

        if (LBK_CtrlGetData(g_InBuff))
            printf("Control in error!\n");

        if (memcmp(g_OutBuff, g_InBuff, 64))
            printf("Control loopback data mismatch!\n");

        ctrl_cnt++;

        if (!LBK_IsConnected())
            continue;
#endif

#ifdef DO_BULK_XFER
        /*--------------------------------------------*/
        /*  Bulk transfer loopback demo               */
        /*--------------------------------------------*/
        memset(g_OutBuff, bulk_cnt & 0xff, TEST_BUFF_SIZE);
        if (LBK_BulkWriteData(g_OutBuff, 512))
            printf("Bulk out error!\n");

        if (LBK_BulkReadData(g_InBuff, 512))
            printf("Bulk in error!\n");

        bulk_cnt++;
#endif

        if ((int_in_cnt % 100) == 1) {
            printf("Transfer count: C:%d, Ii:%d, Io: %d, Si:%d, So:%d, B:%d\n",
                   ctrl_cnt, int_in_cnt, int_out_cnt, iso_in_cnt, iso_out_cnt, bulk_cnt);
        }
    }
}


/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
